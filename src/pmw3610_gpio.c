/*
 * Bit-banged polling driver for the Pixart PMW3610 mouse sensor on RP2040.
 *
 * The phenom right half is wired exactly like the proven QMK firmware:
 * a 3-wire bit-banged SPI on SCLK=GP10, SDIO=GP11, CS=GP9 with NO motion
 * interrupt pin.  QMK polls the sensor ~every 10 ms, so this driver does
 * the same with a delayable work item.
 *
 * Timing constants and init sequence are ported from the ergohaven QMK
 * driver (drivers/sensors/pmw3610.c) which itself derives from the
 * inorichi zmk-pmw3610-driver.
 */

#define DT_DRV_COMPAT ergohaven_pmw3610_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_REGISTER(pmw3610_gpio, CONFIG_ZMK_LOG_LEVEL);

#define PMW3610_PRODUCT_ID 0x3E

#define PMW3610_REG_PRODUCT_ID     0x00
#define PMW3610_REG_MOTION         0x02
#define PMW3610_REG_DELTA_X_L      0x03
#define PMW3610_REG_DELTA_Y_L      0x04
#define PMW3610_REG_DELTA_XY_H     0x05
#define PMW3610_REG_MOTION_BURST   0x12
#define PMW3610_REG_RUN_DOWNSHIFT  0x1B
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_PERFORMANCE    0x11
#define PMW3610_REG_OBSERVATION    0x2D
#define PMW3610_REG_POWER_UP_RESET 0x3A
#define PMW3610_REG_SPI_CLK_ON_REQ 0x41
#define PMW3610_REG_RES_STEP       0x85
#define PMW3610_REG_SPI_PAGE0      0x7F
#define PMW3610_REG_SPI_PAGE1      0xFF

#define PMW3610_POWERUP_CMD_RESET  0x5A
#define PMW3610_SPI_CLOCK_ENABLE   0xBA
#define PMW3610_SPI_CLOCK_DISABLE  0xB5
#define PMW3610_SPI_WRITE_BIT      0x80

#define PMW3610_BURST_SIZE 7
#define PMW3610_X_L_POS    1
#define PMW3610_Y_L_POS    2
#define PMW3610_XY_H_POS   3

#define PMW3610_MAX_CPI   3200
#define PMW3610_MIN_CPI   200
#define PMW3610_CPI_STEP  200
#define PMW3610_PERFORMANCE_VALUE 0x0D

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

struct pmw3610_gpio_config {
	struct gpio_dt_spec sclk;
	struct gpio_dt_spec sdio;
	struct gpio_dt_spec cs;
	uint16_t cpi;
	uint32_t run_downshift_ms;
	uint32_t rest1_downshift_ms;
	bool swap_xy;
	bool invert_x;
	bool invert_y;
	uint16_t evt_type;
	uint16_t x_code;
	uint16_t y_code;
};

struct pmw3610_gpio_data {
	const struct device *dev;
	struct k_work_delayable poll_work;
	int32_t acc_x;
	int32_t acc_y;
	int64_t last_motion_ms;
	bool burst_confirmed;
	int64_t drain_until;
	int64_t drain_cap;
	uint8_t quiet_run;
};

static inline void spi_delay(uint32_t us) {
	k_busy_wait(us);
}

static void pmw3610_assert_cs(const struct pmw3610_gpio_config *cfg) {
	gpio_pin_set_dt(&cfg->cs, true);
}

static void pmw3610_release_cs(const struct pmw3610_gpio_config *cfg) {
	gpio_pin_set_dt(&cfg->cs, false);
}

static void pmw3610_byte_out(const struct pmw3610_gpio_config *cfg, uint8_t val) {
	for (int8_t i = 7; i >= 0; i--) {
		gpio_pin_set_dt(&cfg->sclk, false);
		gpio_pin_set_dt(&cfg->sdio, (val >> i) & 0x1);
		spi_delay(1);
		gpio_pin_set_dt(&cfg->sclk, true);
		spi_delay(1);
	}
}

static uint8_t pmw3610_byte_in(const struct pmw3610_gpio_config *cfg) {
	uint8_t data = 0;
	for (int8_t i = 7; i >= 0; i--) {
		gpio_pin_set_dt(&cfg->sclk, false);
		spi_delay(1);
		gpio_pin_set_dt(&cfg->sclk, true);
		spi_delay(1);
		data |= (uint8_t)gpio_pin_get_dt(&cfg->sdio) << i;
	}
	return data;
}

static void pmw3610_raw_write(const struct pmw3610_gpio_config *cfg, uint8_t reg_addr,
			      uint8_t data) {
	pmw3610_assert_cs(cfg);
	gpio_pin_configure_dt(&cfg->sdio, GPIO_OUTPUT_INACTIVE);
	pmw3610_byte_out(cfg, reg_addr | PMW3610_SPI_WRITE_BIT);
	pmw3610_byte_out(cfg, data);
	gpio_pin_set_dt(&cfg->sclk, false);
	spi_delay(10);
	pmw3610_release_cs(cfg);
	spi_delay(30);
}

static void pmw3610_write(const struct pmw3610_gpio_config *cfg, uint8_t reg_addr, uint8_t data) {
	pmw3610_raw_write(cfg, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_ENABLE);
	pmw3610_raw_write(cfg, reg_addr, data);
	pmw3610_raw_write(cfg, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_DISABLE);
}

static uint8_t pmw3610_read(const struct pmw3610_gpio_config *cfg, uint8_t reg_addr) {
	pmw3610_assert_cs(cfg);
	gpio_pin_configure_dt(&cfg->sdio, GPIO_OUTPUT_INACTIVE);
	pmw3610_byte_out(cfg, reg_addr);
	gpio_pin_configure_dt(&cfg->sdio, GPIO_INPUT);
	spi_delay(4);
	uint8_t data = pmw3610_byte_in(cfg);
	spi_delay(1);
	pmw3610_release_cs(cfg);
	spi_delay(1);
	return data;
}

static bool pmw3610_read_burst(const struct pmw3610_gpio_config *cfg, uint8_t *buf, size_t len) {
	pmw3610_assert_cs(cfg);
	gpio_pin_configure_dt(&cfg->sdio, GPIO_OUTPUT_INACTIVE);
	pmw3610_byte_out(cfg, PMW3610_REG_MOTION_BURST);
	gpio_pin_configure_dt(&cfg->sdio, GPIO_INPUT);
	spi_delay(4);
	for (size_t i = 0; i < len; i++) {
		buf[i] = pmw3610_byte_in(cfg);
	}
	spi_delay(1);
	pmw3610_release_cs(cfg);
	return true;
}

static void pmw3610_set_cpi(const struct pmw3610_gpio_config *cfg, uint16_t cpi) {
	cpi = CLAMP(cpi, PMW3610_MIN_CPI, PMW3610_MAX_CPI);
	uint8_t value = (uint8_t)(cpi / PMW3610_CPI_STEP);
	pmw3610_write(cfg, PMW3610_REG_SPI_PAGE0, 0xFF);
	pmw3610_write(cfg, PMW3610_REG_RES_STEP, value);
	pmw3610_write(cfg, PMW3610_REG_SPI_PAGE1, 0x00);
}

static void pmw3610_set_downshift(const struct pmw3610_gpio_config *cfg, uint8_t reg_addr,
				  uint32_t time_ms, uint32_t scale_ms) {
	/*
	 * Downshift time = reg_value * scale_ms.  QMK scaling:
	 *   RUN_DOWNSHIFT          : value * 32 ms   (8 * 4ms pos-rate)
	 *   REST1_DOWNSHIFT        : value * 16 * 40 ms  (Rest1_sample_period)
	 */
	if (time_ms < scale_ms) {
		return;
	}
	uint8_t value = (uint8_t)(time_ms / scale_ms);
	pmw3610_write(cfg, reg_addr, value);
}

static void pmw3610_set_performance(const struct device *dev, bool force_awake) {
	const struct pmw3610_gpio_config *cfg = dev->config;

	uint8_t value = pmw3610_read(cfg, PMW3610_REG_PERFORMANCE);
	value = (value & 0x0F) | (force_awake ? 0xF0 : 0x00);
	pmw3610_write(cfg, PMW3610_REG_PERFORMANCE, value);
}

static void pmw3610_report_delta(const struct device *dev, int16_t x, int16_t y) {
	const struct pmw3610_gpio_config *cfg = dev->config;

	if (x != 0) {
		input_report(dev, cfg->evt_type, cfg->x_code, x, y == 0, K_NO_WAIT);
	}
	if (y != 0) {
		input_report(dev, cfg->evt_type, cfg->y_code, y, true, K_NO_WAIT);
	}
}

static void pmw3610_report_data(const struct device *dev) {
	const struct pmw3610_gpio_config *cfg = dev->config;
	struct pmw3610_gpio_data *data = dev->data;
	uint8_t buf[PMW3610_BURST_SIZE];
	int64_t now = k_uptime_get();

	pmw3610_read_burst(cfg, buf, sizeof(buf));

#if defined(CONFIG_EH_PMW3610_STARTUP_DEBUG)
	/* Bounded boot-window trace: raw burst bytes + decoded deltas, printed
	 * for every poll (including frames the motion gate rejects) so a
	 * boot-time garbage burst can be identified.  Diagnostics only. */
	{
		static uint32_t dbg_poll;

		if (dbg_poll < CONFIG_EH_PMW3610_STARTUP_DEBUG_POLLS) {
			int16_t dx = TOINT16(
				(buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
			int16_t dy = TOINT16(
				(buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
			printk("PMW t=%d m=%02x b=%02x %02x %02x x=%d y=%d\n", (int)now, buf[0],
			       buf[1], buf[2], buf[3], dx, dy);
		}
		dbg_poll++;
	}
#endif

	/* Boot drain: the PMW3610 emits a deterministic power-up transient of
	 * large deltas with the motion bit set (~80 ms after polling starts),
	 * which the motion gate below cannot reject.  Suppress reports for at
	 * least CONFIG_EH_PMW3610_STARTUP_DRAIN_MS, then until
	 * CONFIG_EH_PMW3610_STARTUP_QUIET_POLLS consecutive quiet polls, with
	 * CONFIG_EH_PMW3610_STARTUP_MAX_MS as a hard cap.  Reads continue so the
	 * sensor drains itself. */
	if (now < data->drain_cap &&
	    (now < data->drain_until ||
	     data->quiet_run < CONFIG_EH_PMW3610_STARTUP_QUIET_POLLS)) {
		data->quiet_run = (buf[0] & 0x80) ? 0 : (uint8_t)(data->quiet_run + 1);
		data->acc_x = 0;
		data->acc_y = 0;
		data->burst_confirmed = false;
		return;
	}

	/* Match QMK: only report deltas when the motion flag is set (bit 7 of
	 * the MOTION register, first byte of the burst).  This drops spurious
	 * single-count deltas (e.g. rest->run wake frames, keystroke vibration)
	 * that the sensor does not flag as real motion. */
	if ((buf[0] & 0x80) == 0) {
		return;
	}

	int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
	int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

	if (cfg->swap_xy) {
		int16_t t = x;
		x = y;
		y = t;
	}
	if (cfg->invert_x) {
		x = -x;
	}
	if (cfg->invert_y) {
		y = -y;
	}

	/* Report-window filter (variant B, adapted from badjeff/zmk-pmw3610-driver):
	 * the first frame of a motion burst is held until the next poll confirms
	 * real motion; once confirmed, deltas stream out every poll (100 Hz at
	 * 10 ms polling).  If motion stops before CONFIG_EH_PMW3610_REPORT_WINDOW_MS
	 * elapses, the held accumulation is purged, so isolated single-count
	 * frames (keystroke vibration, rest->run wake frames) never reach the
	 * cursor. */
	if (x == 0 && y == 0) {
		if (now - data->last_motion_ms >= CONFIG_EH_PMW3610_REPORT_WINDOW_MS) {
			data->acc_x = 0;
			data->acc_y = 0;
			data->burst_confirmed = false;
		}
		return;
	}

	if (now - data->last_motion_ms >= CONFIG_EH_PMW3610_REPORT_WINDOW_MS) {
		/* New motion burst after a quiet gap: drop stale accumulation. */
		data->acc_x = 0;
		data->acc_y = 0;
		data->burst_confirmed = false;
	}
	data->last_motion_ms = now;
	data->acc_x += x;
	data->acc_y += y;

	if (!data->burst_confirmed) {
		/* First frame of a burst: hold until the next poll confirms motion. */
		data->burst_confirmed = true;
		return;
	}

	if (data->acc_x != 0 || data->acc_y != 0) {
		pmw3610_report_delta(dev, data->acc_x, data->acc_y);
		data->acc_x = 0;
		data->acc_y = 0;
	}
}

static void pmw3610_poll_work(struct k_work *work) {
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct pmw3610_gpio_data *data = CONTAINER_OF(dwork, struct pmw3610_gpio_data, poll_work);

	pmw3610_report_data(data->dev);
	k_work_schedule(dwork, K_MSEC(CONFIG_EH_PMW3610_POLL_INTERVAL_MS));
}

static int pmw3610_init(const struct device *dev) {
	const struct pmw3610_gpio_config *cfg = dev->config;
	struct pmw3610_gpio_data *data = dev->data;

	if (!gpio_is_ready_dt(&cfg->sclk) || !gpio_is_ready_dt(&cfg->sdio) ||
	    !gpio_is_ready_dt(&cfg->cs)) {
		LOG_ERR("GPIO port not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&cfg->sclk, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&cfg->sdio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&cfg->cs, GPIO_OUTPUT_INACTIVE);

	spi_delay(10000);

	pmw3610_write(cfg, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
	spi_delay(200000);

	/* Kick off the on-chip observation test and wait for it to settle. */
	pmw3610_write(cfg, PMW3610_REG_OBSERVATION, 0x00);
	spi_delay(100000);

	uint8_t obs = pmw3610_read(cfg, PMW3610_REG_OBSERVATION);
	if ((obs & 0x0F) != 0x0F) {
		LOG_ERR("PMW3610 not detected (observation 0x%02X)", obs);
		return -ENODEV;
	}

	uint8_t product_id = pmw3610_read(cfg, PMW3610_REG_PRODUCT_ID);
	if (product_id != PMW3610_PRODUCT_ID) {
		LOG_ERR("Unexpected PMW3610 product id: 0x%02X", product_id);
		return -ENODEV;
	}

	/* Flush accumulated motion. */
	pmw3610_read(cfg, PMW3610_REG_MOTION);
	pmw3610_read(cfg, PMW3610_REG_DELTA_X_L);
	pmw3610_read(cfg, PMW3610_REG_DELTA_Y_L);
	pmw3610_read(cfg, PMW3610_REG_DELTA_XY_H);

	pmw3610_set_cpi(cfg, cfg->cpi);
	pmw3610_set_performance(dev, true);
	/* Match QMK downshift times (scale per register). */
	pmw3610_set_downshift(cfg, PMW3610_REG_RUN_DOWNSHIFT, cfg->run_downshift_ms, 32);
	pmw3610_set_downshift(cfg, PMW3610_REG_REST1_DOWNSHIFT, cfg->rest1_downshift_ms, 640);

	data->dev = dev;
	data->drain_until = k_uptime_get() + CONFIG_EH_PMW3610_STARTUP_DRAIN_MS;
	data->drain_cap = k_uptime_get() + CONFIG_EH_PMW3610_STARTUP_MAX_MS;
	data->quiet_run = 0;
	k_work_init_delayable(&data->poll_work, pmw3610_poll_work);
	k_work_schedule(&data->poll_work, K_MSEC(CONFIG_EH_PMW3610_POLL_INTERVAL_MS));

	LOG_INF("PMW3610 initialized (cpi=%u)", cfg->cpi);
#if defined(CONFIG_EH_PMW3610_STARTUP_DEBUG)
	printk("PMW init done t=%d poll=%dms trace=%d\n", (int)k_uptime_get(),
	       CONFIG_EH_PMW3610_POLL_INTERVAL_MS, CONFIG_EH_PMW3610_STARTUP_DEBUG_POLLS);
#endif
	return 0;
}

#define PMW3610_GPIO_INST(n)                                                                       \
	static struct pmw3610_gpio_config pmw3610_gpio_cfg_##n = {                                   \
		.sclk = GPIO_DT_SPEC_INST_GET(n, sclk_gpios),                                        \
		.sdio = GPIO_DT_SPEC_INST_GET(n, sdio_gpios),                                        \
		.cs = GPIO_DT_SPEC_INST_GET(n, cs_gpios),                                            \
		.cpi = DT_INST_PROP_OR(n, cpi, 600),                                                    \
		.run_downshift_ms = DT_INST_PROP_OR(n, run_downshift_ms, 128),                          \
		.rest1_downshift_ms = DT_INST_PROP_OR(n, rest1_downshift_ms, 9600),                     \
		.swap_xy = DT_INST_PROP_OR(n, swap_xy, false),                                         \
		.invert_x = DT_INST_PROP_OR(n, invert_x, false),                                       \
		.invert_y = DT_INST_PROP_OR(n, invert_y, false),                                       \
		.evt_type = DT_INST_PROP(n, evt_type),                                                 \
		.x_code = DT_INST_PROP(n, x_input_code),                                             \
		.y_code = DT_INST_PROP(n, y_input_code),                                             \
	};                                                                                            \
	static struct pmw3610_gpio_data pmw3610_gpio_data_##n;                                       \
	DEVICE_DT_DEFINE(DT_DRV_INST(n), pmw3610_init, NULL, &pmw3610_gpio_data_##n,                  \
			 &pmw3610_gpio_cfg_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_GPIO_INST)

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
	DT_FOREACH_STATUS_OKAY(ergohaven_pmw3610_gpio, GET_PMW3610_DEV)
};

static int pmw3610_on_activity(const zmk_event_t *eh) {
	struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

	if (state_ev == NULL) {
		return ZMK_EV_EVENT_HANDLED;
	}

	bool force_awake = state_ev->state == ZMK_ACTIVITY_ACTIVE;
	for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
		pmw3610_set_performance(pmw3610_devs[i], force_awake);
	}

	return ZMK_EV_EVENT_HANDLED;
}

ZMK_LISTENER(pmw3610_activity, pmw3610_on_activity);
ZMK_SUBSCRIPTION(pmw3610_activity, zmk_activity_state_changed);