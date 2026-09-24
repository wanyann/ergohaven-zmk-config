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
#define PMW3610_REG_REST1_RATE     0x1C
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_REST2_RATE     0x1E
#define PMW3610_REG_REST2_DOWNSHIFT 0x1F
#define PMW3610_REG_REST3_RATE     0x20
#define PMW3610_REG_SMART          0x32
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
#define PMW3610_SQUAL_POS  4
#define PMW3610_SHUTTER_H_POS 5
#define PMW3610_SHUTTER_L_POS 6

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

#if defined(CONFIG_EH_PMW3610_VECT)
/* Ring buffer for the V-segment motion-vector window.  Frames accepted as
 * in-direction (or zero-V cross-axis frames) are pushed with their poll
 * timestamp; the vector is the sum of the ring entries younger than
 * EH_PMW3610_VECT_WIN_MS.  600 ms at 200 Hz = 120 frames, so 160 slots
 * leave headroom for bursty polling. */
#define EH_PMW3610_VECT_RING 160
#endif

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
	bool fa_active;
	uint32_t trace_count;
	bool prev_motion;
	int8_t f_axis;
	int8_t f_dir;
	uint8_t f_opp;
	int8_t f_pend_axis;
	int8_t f_pend_dir;
	uint8_t f_pend_n;
	int64_t f_last_ms;
	int16_t f_ring[3];
	uint8_t f_rn;
	uint8_t f_syn_run;
	int64_t f_last_fast_ms;
	uint8_t f_ri;
	int8_t f_rv_axis;
	int8_t f_rv_dir;
	uint8_t f_rv_n;
#if defined(CONFIG_EH_PMW3610_FSEG_PASSMIN_CONFIRM)
	/* Raw-motion confirmation window for the PASSMIN gate: every motion
	 * frame that reaches the F-segment is pushed here (before gating) with
	 * its timestamp; the gate sums the dominant-axis components of the
	 * entries younger than PASSMIN_WIN_MS.  If the sum confirms f_dir the
	 * opposing frame is a mid-roll micro-bounce and is masked; otherwise
	 * it is real slow movement and passes raw.  Capacity 64 covers the
	 * 40-400 ms window range at any polling rate. */
	int16_t f_wx[64];
	int16_t f_wy[64];
	uint32_t f_wt[64];
	uint8_t f_whead;
	uint8_t f_wn;
#endif
#if defined(CONFIG_EH_PMW3610_VECT)
	int16_t f_vbuf_x[EH_PMW3610_VECT_RING];
	int16_t f_vbuf_y[EH_PMW3610_VECT_RING];
	uint32_t f_vbuf_t[EH_PMW3610_VECT_RING];
	int32_t f_vsx;
	int32_t f_vsy;
	uint8_t f_vhead;
	uint8_t f_vn;
#if defined(CONFIG_EH_PMW3610_VECT_SYNN)
	/* Fractional carry for synthetic frames emitted along the motion
	 * vector: each SYN frame adds m*V/|V| scaled by 256, the integer part
	 * is emitted and the remainder is kept, so a diagonal roll whose
	 * opposing frames are synthesized gets diagonal steps instead of an
	 * axial "wall" (several frames stuck on one axis). */
	int32_t f_sfr_x;
	int32_t f_sfr_y;
#endif
#endif
#if defined(CONFIG_EH_PMW3610_ALT_MODE)
	/* ALT mode: faithful port of the badjeff/zmk-pmw3610-driver (v0.3)
	 * report pipeline.  Deltas accumulate between report ticks (every
	 * EH_PMW3610_ALT_REPORT_INTERVAL_MS) instead of streaming every poll,
	 * and the smart surface-coverage algorithm toggles register 0x32 from
	 * the shutter reading.  The accumulator lives here (not in static
	 * locals) so the state survives across polls and is per-device. */
	int64_t alt_dx;
	int64_t alt_dy;
	int64_t alt_last_smp_ms;
	int64_t alt_last_rpt_ms;
	bool alt_smart_flag;
#endif
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
#if defined(CONFIG_EH_PMW3610_ALT_MODE)
	/* badjeff sets the axis mapping in hardware (RES_STEP bits 7/6/5), so
	 * the ALT report path applies no software swap/invert. */
	if (cfg->swap_xy) {
		value |= (1 << 7);
	}
	if (cfg->invert_x) {
		value |= (1 << 6);
	}
	if (cfg->invert_y) {
		value |= (1 << 5);
	}
#endif
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

#if defined(CONFIG_EH_PMW3610_ALT_MODE)
static void pmw3610_set_sample_time(const struct pmw3610_gpio_config *cfg, uint8_t reg_addr,
				    uint32_t sample_time) {
	/* Sample time = reg_value * 10 ms (badjeff pmw3610_set_sample_time). */
	if (sample_time < 10 || sample_time > 2550) {
		return;
	}
	pmw3610_write(cfg, reg_addr, (uint8_t)(sample_time / 10));
}
#endif

static void pmw3610_set_performance(const struct device *dev, bool force_awake) {
	const struct pmw3610_gpio_config *cfg = dev->config;

#if defined(CONFIG_EH_PMW3610_ALT_MODE)
	/* badjeff gates the performance write on the force-awake DT property;
	 * in ALT mode the equivalent is EH_PMW3610_ALT_FORCE_AWAKE.  With it
	 * enabled the register is 0xF0 while ZMK is active and the low nibble
	 * (normal run rate) while idle; disabled makes this a no-op. */
	if (!IS_ENABLED(CONFIG_EH_PMW3610_ALT_FORCE_AWAKE)) {
		return;
	}
#endif

	uint8_t value = pmw3610_read(cfg, PMW3610_REG_PERFORMANCE);
#if defined(CONFIG_EH_PMW3610_ALT_MODE) && defined(CONFIG_EH_PMW3610_ALT_FORCE_AWAKE_4MS_MODE)
	/* badjeff force-awake-4ms-mode: force all run rates to 4 ms (0x0D)
	 * regardless of the current low nibble; 0xFD while active. */
	value = force_awake ? 0xFD : 0x0D;
#else
	value = (value & 0x0F) | (force_awake ? 0xF0 : 0x00);
#endif
	pmw3610_write(cfg, PMW3610_REG_PERFORMANCE, value);
}

#if !defined(CONFIG_EH_PMW3610_ALT_MODE)
static void pmw3610_report_delta(const struct device *dev, int16_t x, int16_t y) {
	const struct pmw3610_gpio_config *cfg = dev->config;

	if (x != 0) {
		input_report(dev, cfg->evt_type, cfg->x_code, x, y == 0, K_NO_WAIT);
	}
	if (y != 0) {
		input_report(dev, cfg->evt_type, cfg->y_code, y, true, K_NO_WAIT);
	}
}
#endif

#if defined(CONFIG_EH_PMW3610_FSEGMENT)
/* F-segment: drop short runs of deltas opposing the established direction of
 * the current motion burst on its dominant axis.  Returns false to consume
 * (drop) the frame.  The first opposing frame of a run still passes; the run
 * is dropped from EH_PMW3610_FSEG_DROP onwards, and a run longer than
 * EH_PMW3610_FSEG_REVERSE is treated as a genuine reversal and adopted. */

static void fseg_mag_push(struct pmw3610_gpio_data *data, int16_t mag) {
	data->f_ring[data->f_ri] = mag;
	data->f_ri = (data->f_ri + 1) % 3;
	if (data->f_rn < 3) {
		data->f_rn++;
	}
}

static int16_t fseg_mag_est(const struct pmw3610_gpio_data *data) {
	if (data->f_rn == 0) {
		return 1;
	}
	int32_t s = 0;

	for (uint8_t i = 0; i < data->f_rn; i++) {
		s += data->f_ring[i];
	}
	return (int16_t)(s / data->f_rn);
}

#if defined(CONFIG_EH_PMW3610_VECT)
/* V-segment: project in-direction frames onto the running 2D motion vector
 * and snap small perpendicular components (<= EH_PMW3610_VECT_QGATE counts)
 * onto it.  The F-segment only gates the dominant axis, so the cross-axis
 * component of every in-direction frame (and the whole ZERO-V frame) passes
 * through untouched: that is the lateral jitter / random-side cursor drift.
 * The vector is the sum of the ring entries younger than VECT_WIN_MS, so a
 * diagonal roll stays diagonal while cross-axis noise is straightened. */

static void fseg_vect_reset(struct pmw3610_gpio_data *data) {
	data->f_vhead = 0;
	data->f_vn = 0;
	data->f_vsx = 0;
	data->f_vsy = 0;
#if defined(CONFIG_EH_PMW3610_VECT_SYNN)
	data->f_sfr_x = 0;
	data->f_sfr_y = 0;
#endif
}

static void fseg_vect_push(struct pmw3610_gpio_data *data, int16_t x, int16_t y,
			   int64_t now) {
	uint8_t tail = (data->f_vhead + data->f_vn) % EH_PMW3610_VECT_RING;

	data->f_vbuf_x[tail] = x;
	data->f_vbuf_y[tail] = y;
	data->f_vbuf_t[tail] = (uint32_t)now;
	data->f_vsx += x;
	data->f_vsy += y;
	if (data->f_vn < EH_PMW3610_VECT_RING) {
		data->f_vn++;
	} else {
		data->f_vhead = (data->f_vhead + 1) % EH_PMW3610_VECT_RING;
	}
}

static void fseg_vect_expire(struct pmw3610_gpio_data *data, int64_t now) {
	while (data->f_vn > 0) {
		uint8_t head = data->f_vhead;
		uint32_t age = (uint32_t)now - data->f_vbuf_t[head];

		if (age <= CONFIG_EH_PMW3610_VECT_WIN_MS) {
			break;
		}
		data->f_vsx -= data->f_vbuf_x[head];
		data->f_vsy -= data->f_vbuf_y[head];
		data->f_vhead = (data->f_vhead + 1) % EH_PMW3610_VECT_RING;
		data->f_vn--;
	}
}

/* Returns true and rewrites *x/*y when the frame is snapped onto the motion
 * vector.  Only in-direction frames with a small perpendicular component are
 * snapped; larger perpendiculars (real maneuvers / direction changes) pass
 * through untouched. */
static bool fseg_vect_project(struct pmw3610_gpio_data *data, int16_t *x, int16_t *y,
			      int64_t now) {
	fseg_vect_expire(data, now);

	if (data->f_vn < 3) {
		return false;
	}

	int32_t vx = data->f_vsx;
	int32_t vy = data->f_vsy;
	int64_t mag2 = (int64_t)vx * vx + (int64_t)vy * vy;

	if (mag2 <= 4) {
		return false;
	}

	int64_t dot = (int64_t)(*x) * vx + (int64_t)(*y) * vy;

	if (dot <= 0) {
		return false;
	}

	/* Perpendicular component magnitude = |cross| / |V|.  Snap only when it
	 * is at or below the gate.  Guard against int64 overflow of cross*cross:
	 * any |cross| > 2^30 is far above qgate*|V| for realistic vectors, so
	 * it can only be a real maneuver, never a snap candidate. */
	int64_t cross = (int64_t)(*x) * vy - (int64_t)(*y) * vx;
	int64_t qgate = CONFIG_EH_PMW3610_VECT_QGATE;

	if (cross > (1LL << 30) || cross < -(1LL << 30)) {
		return false;
	}
	if (cross * cross > qgate * qgate * mag2) {
		return false;
	}

	/* Project onto the vector: (dot / mag2) * V, rounded to nearest. */
	int32_t px = (int32_t)((dot * vx + mag2 / 2) / mag2);
	int32_t py = (int32_t)((dot * vy + mag2 / 2) / mag2);

	if (px == 0 && py == 0) {
		return false;
	}
	*x = (int16_t)px;
	*y = (int16_t)py;
	return true;
}

#if defined(CONFIG_EH_PMW3610_VECT_SYNN)
/* Integer floor sqrt (Newton), floor(sqrt(n)). */
static uint32_t fseg_isqrt(uint32_t n) {
	uint32_t x = n;
	uint32_t y = (n + 1) / 2;

	while (y < x) {
		x = y;
		y = (x + n / x) / 2;
	}
	return x;
}

/* Floor division (C '/' truncates toward zero; we need floor so negative
 * fractional carries emit their step instead of stalling at 0). */
static int32_t fseg_floor_div(int32_t a, int32_t b) {
	int32_t q = a / b;

	if ((a % b) != 0 && ((a < 0) != (b < 0))) {
		q--;
	}
	return q;
}

/* Emit a synthetic step of magnitude m along the motion vector, with a
 * fractional carry so the average direction matches the vector exactly.
 * Returns false (caller falls back to the plain dominant-axis step) when
 * the vector is too short to define a direction. */
static bool fseg_vect_synth(struct pmw3610_gpio_data *data, int16_t m, int16_t *x, int16_t *y,
			    int64_t now) {
	fseg_vect_expire(data, now);

	if (data->f_vn < 3) {
		return false;
	}

	int32_t vx = data->f_vsx;
	int32_t vy = data->f_vsy;
	int64_t mag2 = (int64_t)vx * vx + (int64_t)vy * vy;

	if (mag2 <= 4) {
		return false;
	}

	uint32_t len = fseg_isqrt((uint32_t)mag2);

	if (len == 0) {
		return false;
	}

	/* Fixed point: add m * V / |V| scaled by 256, emit the integer part. */
	data->f_sfr_x += (int32_t)((int64_t)m * vx * 256 / len);
	data->f_sfr_y += (int32_t)((int64_t)m * vy * 256 / len);
	int32_t sx = fseg_floor_div(data->f_sfr_x, 256);
	int32_t sy = fseg_floor_div(data->f_sfr_y, 256);

	data->f_sfr_x -= sx * 256;
	data->f_sfr_y -= sy * 256;

	if (sx == 0 && sy == 0) {
		return false;
	}
	*x = (int16_t)sx;
	*y = (int16_t)sy;
	return true;
}
#endif
#endif

#if defined(CONFIG_EH_PMW3610_FSEG_PASSMIN_CONFIRM)
/* Raw-motion confirmation window for the PASSMIN gate.  Every motion frame
 * that reaches the F-segment is pushed here (before gating) with its
 * timestamp; the gate sums the dominant-axis components of the entries
 * younger than PASSMIN_WIN_MS.  If that sum still confirms the established
 * direction, an opposing frame is a mid-roll micro-bounce (ball lurch during
 * continuous rolling) and falls through to the synthesis path; if the window
 * opposes f_dir (or is empty/zero), the frame is real slow movement and
 * passes raw.  Ring capacity 64 covers the 40-400 ms window range at any
 * polling rate. */

static void fseg_win_reset(struct pmw3610_gpio_data *data) {
	data->f_whead = 0;
	data->f_wn = 0;
}

static void fseg_win_push(struct pmw3610_gpio_data *data, int16_t x, int16_t y,
			  int64_t now) {
	uint8_t tail = (data->f_whead + data->f_wn) % 64;

	data->f_wx[tail] = x;
	data->f_wy[tail] = y;
	data->f_wt[tail] = (uint32_t)now;
	if (data->f_wn < 64) {
		data->f_wn++;
	} else {
		data->f_whead = (data->f_whead + 1) % 64;
	}
}

static void fseg_win_expire(struct pmw3610_gpio_data *data, int64_t now) {
	while (data->f_wn > 0) {
		uint8_t head = data->f_whead;
		uint32_t age = (uint32_t)now - data->f_wt[head];

		if (age <= CONFIG_EH_PMW3610_FSEG_PASSMIN_WIN_MS) {
			break;
		}
		data->f_whead = (data->f_whead + 1) % 64;
		data->f_wn--;
	}
}

/* Sum of the dominant-axis components of the raw frames younger than
 * PASSMIN_WIN_MS (the current frame included, since it was pushed at the top
 * of pmw3610_fsegment). */
static int32_t fseg_win_axis_sum(struct pmw3610_gpio_data *data, int64_t now) {
	fseg_win_expire(data, now);

	int32_t s = 0;
	uint8_t head = data->f_whead;

	for (uint8_t i = 0; i < data->f_wn; i++) {
		uint8_t idx = (head + i) % 64;

		s += (data->f_axis == 1) ? data->f_wx[idx] : data->f_wy[idx];
	}
	return s;
}
#endif

static bool pmw3610_fsegment(struct pmw3610_gpio_data *data, int16_t *x, int16_t *y,
			     int64_t now) {
	if (now - data->f_last_ms >= CONFIG_EH_PMW3610_REPORT_WINDOW_MS) {
		data->f_axis = 0;
		data->f_dir = 0;
		data->f_opp = 0;
		data->f_pend_axis = 0;
		data->f_pend_dir = 0;
		data->f_pend_n = 0;
		data->f_rn = 0;
		data->f_ri = 0;
		data->f_rv_axis = 0;
		data->f_rv_dir = 0;
		data->f_rv_n = 0;
		data->f_syn_run = 0;
		data->f_last_fast_ms = 0;
#if defined(CONFIG_EH_PMW3610_VECT)
		fseg_vect_reset(data);
#endif
#if defined(CONFIG_EH_PMW3610_FSEG_PASSMIN_CONFIRM)
		fseg_win_reset(data);
#endif
	}
	data->f_last_ms = now;

#if defined(CONFIG_EH_PMW3610_FSEG_PASSMIN_CONFIRM)
	/* Feed every motion frame into the PASSMIN confirmation window before
	 * any gating: the gate needs the recent raw motion on the dominant axis
	 * to tell a mid-roll micro-bounce from real slow movement. */
	fseg_win_push(data, *x, *y, now);
#endif

	/* Record the last fast-roll frame: any frame with a large component is
	 * evidence of a forceful roll whose deceleration tail produces small
	 * opposing frames (ball recoil).  The PASSMIN gate below then stays
	 * disabled for EH_PMW3610_FSEG_FAST_WIN_MS after such a frame, so the
	 * recoil tail is masked instead of passed raw (the "lag constantly"
	 * regression of v0.0.20). */
	if ((*x < 0 ? -*x : *x) >= CONFIG_EH_PMW3610_FSEG_FAST_MAG ||
	    (*y < 0 ? -*y : *y) >= CONFIG_EH_PMW3610_FSEG_FAST_MAG) {
		data->f_last_fast_ms = now;
	}

	if (data->f_axis == 0) {
		int16_t vx = *x;
		int16_t vy = *y;
		int8_t ax = ((vx < 0 ? -vx : vx) >= (vy < 0 ? -vy : vy)) ? 1 : 2;
		int16_t v = (ax == 1) ? vx : vy;

		if (v != 0) {
			int8_t sgn = (v > 0) ? 1 : -1;

			if (data->f_pend_axis == ax && data->f_pend_dir == sgn) {
				data->f_pend_n++;
			} else {
				data->f_pend_axis = ax;
				data->f_pend_dir = sgn;
				data->f_pend_n = 1;
			}
			if (data->f_pend_n >= 3) {
				data->f_axis = ax;
				data->f_dir = sgn;
				data->f_opp = 0;
				data->f_syn_run = 0;
				fseg_mag_push(data, (v < 0 ? -v : v));
			}
		}
		return true;
	}

#if defined(CONFIG_EH_PMW3610_FSEG_REVOTE)
	/* Strong re-vote: two consecutive frames with a dominant-axis component
	 * of at least EH_PMW3610_FSEG_REVOTE_MAG re-adopt the burst direction.
	 * Without this, a strong opposing frame passes the magnitude gate and
	 * resets the opposing-run counter but never updates f_dir, so the
	 * deceleration tail of a genuine direction change is synthesized in the
	 * stale direction (the wrong-direction echo seen with FSEG_SYNTH). */
	{
		int16_t rvx = *x;
		int16_t rvy = *y;
		int8_t rv_ax = ((rvx < 0 ? -rvx : rvx) >= (rvy < 0 ? -rvy : rvy)) ? 1 : 2;
		int16_t rv_d = (rv_ax == 1) ? rvx : rvy;

		if ((rv_d < 0 ? -rv_d : rv_d) >= CONFIG_EH_PMW3610_FSEG_REVOTE_MAG) {
			int8_t rv_sd = (rv_d > 0) ? 1 : -1;

			if (data->f_rv_axis == rv_ax && data->f_rv_dir == rv_sd) {
				data->f_rv_n++;
				if (data->f_rv_n >= 2 &&
				    (rv_ax != data->f_axis || rv_sd != data->f_dir)) {
					data->f_axis = rv_ax;
					data->f_dir = rv_sd;
					data->f_opp = 0;
					data->f_syn_run = 0;
					data->f_rv_n = 0;
					data->f_rv_axis = 0;
					data->f_rv_dir = 0;
					fseg_mag_push(data, (rv_d < 0 ? -rv_d : rv_d));
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
					printk("PMW-ADOPT t=%d ax=%c dir=%d m=%d\n", (int)now,
					       rv_ax == 1 ? 'X' : 'Y', rv_sd, (rv_d < 0 ? -rv_d : rv_d));
#endif
				}
			} else {
				data->f_rv_axis = rv_ax;
				data->f_rv_dir = rv_sd;
				data->f_rv_n = 1;
			}
		} else {
			data->f_rv_n = 0;
			data->f_rv_axis = 0;
			data->f_rv_dir = 0;
		}
	}
#endif

	int16_t v = (data->f_axis == 1) ? *x : *y;

	if (v == 0) {
		data->f_opp = 0;
		data->f_syn_run = 0;
#if defined(CONFIG_EH_PMW3610_VECT)
		/* A zero-V frame is a pure cross-axis frame: the F-segment would
		 * pass it through untouched.  Feed it to the vector window so the
		 * lateral drift it represents is seen by the projection (and gets
		 * snapped when it is small). */
		fseg_vect_push(data, *x, *y, now);
		fseg_vect_project(data, x, y, now);
#endif
		return true;
	}

	if (((v > 0) ? 1 : -1) == data->f_dir) {
		data->f_opp = 0;
		data->f_syn_run = 0;
		fseg_mag_push(data, (v < 0 ? -v : v));
#if defined(CONFIG_EH_PMW3610_VECT)
		fseg_vect_push(data, *x, *y, now);
		fseg_vect_project(data, x, y, now);
#endif
		return true;
	}

	/* Only small opposing frames are candidates for dropping.  A frame with
	 * a large component is real motion (diagonal roll whose cross-axis
	 * component flips sign, dominant-axis switch mid-gesture): reset the
	 * opposing-run counter and pass it through.  Without this gate the
	 * filter eats large chunks of genuine movement.
	 * Exception: right after a fast roll (EH_PMW3610_FSEG_FAST_WIN_MS) a
	 * large opposing frame is the deceleration recoil of that roll, not a
	 * real reversal — let it fall through to the opposing-run path so it is
	 * masked/synthesized like the PASSMIN recoil tail.  Genuine strong
	 * reversals are still caught by the re-vote gate above. */
	if ((*x < 0 ? -*x : *x) > CONFIG_EH_PMW3610_FSEG_REFMAG ||
	    (*y < 0 ? -*y : *y) > CONFIG_EH_PMW3610_FSEG_REFMAG) {
		if (data->f_last_fast_ms == 0 ||
		    now - data->f_last_fast_ms >= CONFIG_EH_PMW3610_FSEG_FAST_WIN_MS) {
			data->f_opp = 0;
			return true;
		}
	}

	data->f_opp++;
	if (data->f_opp > CONFIG_EH_PMW3610_FSEG_REVERSE) {
		data->f_dir = -data->f_dir;
		data->f_opp = 0;
		data->f_syn_run = 0;
		return true;
	}
	if (data->f_opp >= CONFIG_EH_PMW3610_FSEG_DROP) {
#if defined(CONFIG_EH_PMW3610_FSEG_SYNTH)
		/* Minimum ring magnitude: when the recent in-direction velocity
		 * is tiny the gesture has effectively stopped and the small
		 * opposing frames are real slow movement (aiming at a small
		 * target), not recoil.  Synthesizing them continues the stale
		 * established direction and the cursor drifts the wrong way
		 * (lag log 18-06-45: slow aim, ball moved right/down while the
		 * cursor went left); pass them through as-is instead.
		 * Exception: if a fast roll happened within the last
		 * EH_PMW3610_FSEG_FAST_WIN_MS, the small ring is just the
		 * deceleration tail of that roll and the opposing frames are
		 * recoil — keep them masked (lag dumps 19-01-23/19-00-44/
		 * 19-00-32: 99-100% of PASSMIN frames landed inside this
		 * window after a fast roll, the "lag constantly" regression). */
		if (fseg_mag_est(data) < CONFIG_EH_PMW3610_FSEG_SYNTH_MIN_MAG &&
		    (data->f_last_fast_ms == 0 ||
		     now - data->f_last_fast_ms >= CONFIG_EH_PMW3610_FSEG_FAST_WIN_MS)) {
#if defined(CONFIG_EH_PMW3610_FSEG_PASSMIN_CONFIRM)
			/* Direction-confirmation gate: an opposing frame is real
			 * slow movement (passes raw) only when either the opposing
			 * run is sustained (opp >= 3, a genuine slow reversal or
			 * aim) or the recent raw-motion window does NOT confirm
			 * the established direction (wx * f_dir <= 0, the roll has
			 * effectively stopped or turned).  When the window still
			 * confirms the roll, the frame is a mid-roll micro-bounce
			 * and falls through to the synthesis path below (lag dumps
			 * 22-42-22/22-42-46/22-44-13: every wrong-direction event
			 * is a raw frame opposing the rolling direction during
			 * continuous slow diagonal rolling). */
			int32_t wx = fseg_win_axis_sum(data, now);

			if (data->f_opp >= 3 || wx * data->f_dir <= 0) {
#endif
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
				printk("PMW-PASSMIN t=%d ax=%c opp=%u m=%d\n", (int)now,
				       data->f_axis == 1 ? 'X' : 'Y', data->f_opp,
				       fseg_mag_est(data));
#endif
				return true;
#if defined(CONFIG_EH_PMW3610_FSEG_PASSMIN_CONFIRM)
			}
#endif
		}
		/* Cap on consecutive synthetic reports: after this many rewritten
		 * frames since the last honest in-direction frame, the opposing run
		 * is a real (slow) direction change, not a recoil — adopt the
		 * observed direction and pass the current frame through as-is.
		 * The counter is NOT reset by a >REFMAG opposing frame (that would
		 * let a slow sustained opposite run lose its grip), only by genuine
		 * in-direction evidence, establishment, re-vote, reversal or quiet. */
		if (data->f_syn_run >= CONFIG_EH_PMW3610_FSEG_SYNTH_MAXCONSEC) {
			data->f_dir = -data->f_dir;
			data->f_opp = 0;
			data->f_syn_run = 0;
			fseg_mag_push(data, (v < 0 ? -v : v));
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
			printk("PMW-SYNMAX t=%d ax=%c dir=%d m=%d\n", (int)now,
			       data->f_axis == 1 ? 'X' : 'Y', data->f_dir,
			       (v < 0 ? -v : v));
#endif
			return true;
		}
		if (data->f_opp <= CONFIG_EH_PMW3610_FSEG_SYNTH_MAX) {
			/* Decay the synthesized magnitude over the run: the ring
			 * estimate reflects the tail of the previous roll, so a
			 * long opposing run (deceleration into reversal) would
			 * otherwise emit stale-large steps in the old direction
			 * until SYNMAX flips.  Halve every 2 consecutive synths;
			 * the first 1-2 frames stay full to keep recoil masking. */
			int16_t m = fseg_mag_est(data);
			int16_t m_use = m >> (data->f_syn_run / 2);
			if (m_use < 1) {
				m_use = 1;
			}

#if defined(CONFIG_EH_PMW3610_VECT_SYNN)
			/* Synthesize along the motion vector instead of the
			 * dominant axis: a diagonal roll whose opposing frames
			 * are filled as pure axial steps produces an axial
			 * "wall" (several consecutive frames stuck on one
			 * axis).  Emitting vector-direction steps with a
			 * fractional carry keeps the diagonal. */
			if (fseg_vect_synth(data, m_use, x, y, now)) {
				data->f_syn_run++;
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
				printk("PMW-SYN t=%d ax=%c opp=%u m=%d\n", (int)now,
				       data->f_axis == 1 ? 'X' : 'Y', data->f_opp, m_use);
#endif
				return true;
			}
#endif
			if (data->f_axis == 1) {
				*x = (int16_t)(m_use * data->f_dir);
				*y = 0;
			} else {
				*x = 0;
				*y = (int16_t)(m_use * data->f_dir);
			}
			data->f_syn_run++;
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
			printk("PMW-SYN t=%d ax=%c opp=%u m=%d\n", (int)now,
			       data->f_axis == 1 ? 'X' : 'Y', data->f_opp, m_use);
#endif
			return true;
		}
#endif
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
		printk("PMW-DROP t=%d ax=%c opp=%u\n", (int)now, data->f_axis == 1 ? 'X' : 'Y',
		       data->f_opp);
#endif
		return false;
	}
	return true;
}
#endif

#if defined(CONFIG_EH_PMW3610_ALT_MODE)
/* ALT mode: faithful port of the badjeff/zmk-pmw3610-driver (v0.3)
 * pmw3610_report_data().  Raw burst -> decode -> smart algorithm ->
 * accumulate -> report every EH_PMW3610_ALT_REPORT_INTERVAL_MS.  No motion
 * gate, no software swap/invert (done in hardware by set_cpi), no boot
 * drain, no F-segment/V-segment/report-window filters: the sensor data
 * reaches the cursor exactly as the stock driver would deliver it.  The
 * polling transport (5 ms cadence) only replaces badjeff's data-ready IRQ;
 * the algorithm is untouched. */
static void pmw3610_report_data(const struct device *dev) {
	const struct pmw3610_gpio_config *cfg = dev->config;
	struct pmw3610_gpio_data *data = dev->data;
	uint8_t buf[PMW3610_BURST_SIZE];
#if CONFIG_EH_PMW3610_ALT_REPORT_INTERVAL_MS > 0
	int64_t now = k_uptime_get();
#endif

	pmw3610_read_burst(cfg, buf, sizeof(buf));

	int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
	int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

#if defined(CONFIG_EH_PMW3610_ALT_SMART)
	/* Smart surface-coverage algorithm (datasheet): toggle register 0x32
	 * based on the shutter reading so tracking keeps working on a wider
	 * range of surfaces (granite, tiles). */
	int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) +
			  buf[PMW3610_SHUTTER_L_POS];

	if (data->alt_smart_flag && shutter < 45) {
		pmw3610_write(cfg, PMW3610_REG_SMART, 0x00);
		data->alt_smart_flag = false;
	}
	if (!data->alt_smart_flag && shutter > 45) {
		pmw3610_write(cfg, PMW3610_REG_SMART, 0x80);
		data->alt_smart_flag = true;
	}
#endif

#if CONFIG_EH_PMW3610_ALT_REPORT_INTERVAL_MS > 0
	/* Purge accumulated delta if the last sample was not reported on the
	 * last report tick: stale motion is dropped, not delivered late. */
	if (now - data->alt_last_smp_ms >= CONFIG_EH_PMW3610_ALT_REPORT_INTERVAL_MS) {
		data->alt_dx = 0;
		data->alt_dy = 0;
	}
	data->alt_last_smp_ms = now;
#endif

	data->alt_dx += x;
	data->alt_dy += y;

#if CONFIG_EH_PMW3610_ALT_REPORT_INTERVAL_MS > 0
	if (now - data->alt_last_rpt_ms < CONFIG_EH_PMW3610_ALT_REPORT_INTERVAL_MS) {
		return;
	}
#endif

	int16_t rx = (int16_t)CLAMP(data->alt_dx, INT16_MIN, INT16_MAX);
	int16_t ry = (int16_t)CLAMP(data->alt_dy, INT16_MIN, INT16_MAX);
	bool have_x = rx != 0;
	bool have_y = ry != 0;

	if (have_x || have_y) {
#if CONFIG_EH_PMW3610_ALT_REPORT_INTERVAL_MS > 0
		data->alt_last_rpt_ms = now;
#endif
		data->alt_dx = 0;
		data->alt_dy = 0;
		if (have_x) {
			input_report(dev, cfg->evt_type, cfg->x_code, rx, !have_y, K_NO_WAIT);
		}
		if (have_y) {
			input_report(dev, cfg->evt_type, cfg->y_code, ry, true, K_NO_WAIT);
		}
	}
}
#else
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

#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
	/* Motion-gated diagnostics: only frames with the motion bit set can reach
	 * the cursor, so print exactly those (raw sensor + cursor-space deltas,
	 * SQUAL/shutter, force-awake state) plus burst edge markers and activity
	 * transitions.  Bounded by EH_PMW3610_MOTION_TRACE_MAX lines. */
	{
		bool motion = (buf[0] & 0x80) != 0;

		if (motion != data->prev_motion) {
			printk("PMW-%s t=%d\n", motion ? "START" : "STOP", (int)now);
			data->prev_motion = motion;
			/* Per-burst budget: a motion burst ends at STOP, so the next
			 * burst may print frames again.  Without this the whole-session
			 * cap (MOTION_TRACE_MAX) exhausts after ~3 min of rolling and
			 * the trace goes silent for the rest of the session, which
			 * breaks always-on host-side ring-buffer capture. */
			if (!motion) {
				data->trace_count = 0;
			}
		}

		if (motion && data->trace_count < CONFIG_EH_PMW3610_MOTION_TRACE_MAX) {
			int16_t rdx = TOINT16(
				(buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
			int16_t rdy = TOINT16(
				(buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
			int16_t cx = rdx;
			int16_t cy = rdy;
			uint16_t shutter;

			if (cfg->swap_xy) {
				int16_t t = cx;
				cx = cy;
				cy = t;
			}
			if (cfg->invert_x) {
				cx = -cx;
			}
			if (cfg->invert_y) {
				cy = -cy;
			}

			shutter = ((uint16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) |
				  buf[PMW3610_SHUTTER_L_POS];

			printk("PMW %d %02x %d %d %d %d %u %u %d\n", (int)now, buf[0], rdx, rdy, cx, cy,
			       buf[PMW3610_SQUAL_POS], shutter, data->fa_active ? 1 : 0);
			data->trace_count++;
		}
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

#if defined(CONFIG_EH_PMW3610_FSEGMENT)
	if (!pmw3610_fsegment(data, &x, &y, now)) {
		return;
	}
#endif

	/* Report-window filter (variant B, adapted from badjeff/zmk-pmw3610-driver):
	 * the first frame of a motion burst is held until the next poll confirms
	 * real motion; once confirmed, deltas stream out every poll (200 Hz at
	 * 5 ms polling).  If motion stops before CONFIG_EH_PMW3610_REPORT_WINDOW_MS
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
#endif

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
#if defined(CONFIG_EH_PMW3610_ALT_MODE)
	/* badjeff v0.3 async init: 50 ms between clearing OB1 and checking it. */
	spi_delay(50000);
#else
	spi_delay(100000);
#endif

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
#if defined(CONFIG_EH_PMW3610_ALT_MODE)
	/* badjeff v0.3 configure: downshift times + per-mode sample rates
	 * (defaults mirror CONFIG_PMW3610_*_TIME_MS / *_SAMPLE_TIME_MS). */
	pmw3610_set_downshift(cfg, PMW3610_REG_RUN_DOWNSHIFT,
			      CONFIG_EH_PMW3610_ALT_RUN_DOWNSHIFT_MS, 32);
	pmw3610_set_downshift(cfg, PMW3610_REG_REST1_DOWNSHIFT,
			      CONFIG_EH_PMW3610_ALT_REST1_DOWNSHIFT_MS,
			      16 * CONFIG_EH_PMW3610_ALT_REST1_SAMPLE_MS);
	pmw3610_set_downshift(cfg, PMW3610_REG_REST2_DOWNSHIFT,
			      CONFIG_EH_PMW3610_ALT_REST2_DOWNSHIFT_MS,
			      128 * CONFIG_EH_PMW3610_ALT_REST2_SAMPLE_MS);
	pmw3610_set_sample_time(cfg, PMW3610_REG_REST1_RATE,
				CONFIG_EH_PMW3610_ALT_REST1_SAMPLE_MS);
	pmw3610_set_sample_time(cfg, PMW3610_REG_REST2_RATE,
				CONFIG_EH_PMW3610_ALT_REST2_SAMPLE_MS);
	pmw3610_set_sample_time(cfg, PMW3610_REG_REST3_RATE,
				CONFIG_EH_PMW3610_ALT_REST3_SAMPLE_MS);
#else
	/* Match QMK downshift times (scale per register). */
	pmw3610_set_downshift(cfg, PMW3610_REG_RUN_DOWNSHIFT, cfg->run_downshift_ms, 32);
	pmw3610_set_downshift(cfg, PMW3610_REG_REST1_DOWNSHIFT, cfg->rest1_downshift_ms, 640);
#endif

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
		struct pmw3610_gpio_data *data = pmw3610_devs[i]->data;

		data->fa_active = force_awake;
#if defined(CONFIG_EH_PMW3610_MOTION_TRACE)
		printk("PMW-ACT t=%d state=%d fa=%d\n", (int)k_uptime_get(), state_ev->state,
		       force_awake ? 1 : 0);
#endif
		pmw3610_set_performance(pmw3610_devs[i], force_awake);
	}

	return ZMK_EV_EVENT_HANDLED;
}

ZMK_LISTENER(pmw3610_activity, pmw3610_on_activity);
ZMK_SUBSCRIPTION(pmw3610_activity, zmk_activity_state_changed);