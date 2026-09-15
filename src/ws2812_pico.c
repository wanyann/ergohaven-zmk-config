/*
 * Minimal bit-banged WS2812 driver for RP2040.
 *
 * Zephyr's upstream ws2812 gpio driver only supports nRF51, and the
 * SPI/I2S backends cannot drive a LED wired to a plain GPIO such as
 * GP25 on the phenom_mini, so we bit-bang the 800 kHz protocol using
 * the SIO GPIO registers. The core clock is fixed at 125 MHz.
 *
 * Timing is produced with unrolled NOP groups (1 cycle each on
 * Cortex-M0+); phase lengths are tuned to the WS2812 tolerances:
 *   0 bit high  : 200-500 ns  -> ~40 cycles (320 ns)
 *   1 bit high  : 550+ ns     -> ~87 cycles (696 ns)
 *   bit period  : ~1250 ns    -> 156 cycles
 */

#define DT_DRV_COMPAT ergohaven_ws2812_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <stdbool.h>
#include <stdint.h>

#define SIO_BASE             0xd0000000U
#define SIO_GPIO_OUT_SET_OFF 0x14U
#define SIO_GPIO_OUT_CLR_OFF 0x18U

#define RESET_US 100

struct ws2812_pico_cfg {
	struct gpio_dt_spec in_gpio;
	uint16_t chain_length;
};

/* 12 NOPs == 12 cycles == 96 ns at 125 MHz. */
#define NOP_GROUP12                                                                    \
	__asm__ volatile("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop")

struct ws2812_pico_ctx {
	volatile uint32_t *out_set;
	volatile uint32_t *out_clr;
	uint32_t pin_bit;
};

static int ws2812_pico_update_rgb(const struct device *dev, struct led_rgb *pixels,
				  size_t num_pixels) {
	const struct ws2812_pico_cfg *cfg = dev->config;
	struct ws2812_pico_ctx ctx = {
		.out_set = (volatile uint32_t *)(SIO_BASE + SIO_GPIO_OUT_SET_OFF),
		.out_clr = (volatile uint32_t *)(SIO_BASE + SIO_GPIO_OUT_CLR_OFF),
		.pin_bit = BIT(cfg->in_gpio.pin),
	};
	unsigned int key;

	if (num_pixels > cfg->chain_length) {
		return -EINVAL;
	}

	/* The frame is timing-critical, keep interrupts off for the whole
	 * strip so a mid-frame IRQ cannot corrupt the signal. */
	key = irq_lock();

	for (size_t i = 0; i < num_pixels; i++) {
		uint32_t v = ((uint32_t)pixels[i].g << 16) | ((uint32_t)pixels[i].r << 8) |
			     (uint32_t)pixels[i].b;

		for (int b = 23; b >= 0; b--) {
			if (v & BIT(b)) {
				*ctx.out_set = ctx.pin_bit;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				*ctx.out_clr = ctx.pin_bit;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
			} else {
				*ctx.out_set = ctx.pin_bit;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				*ctx.out_clr = ctx.pin_bit;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
				*ctx.out_clr = ctx.pin_bit;
				NOP_GROUP12;
				NOP_GROUP12;
				NOP_GROUP12;
			}
		}
	}

	irq_unlock(key);

	if (num_pixels > 0 && pixels[0].g > pixels[0].r && pixels[0].g > pixels[0].b) {
		printk("[WS] GREEN r=%d g=%d b=%d\n", pixels[0].r, pixels[0].g, pixels[0].b);
	}

	/* Latch/reset: hold the line low. */
	k_busy_wait(RESET_US);

	return 0;
}

static int ws2812_pico_update_channels(const struct device *dev, uint8_t *channels,
				       size_t num_channels) {
	static struct led_rgb pixels[DT_INST_PROP(0, chain_length)];

	if (num_channels > ARRAY_SIZE(pixels) * 3) {
		return -EINVAL;
	}

	for (size_t i = 0; i < num_channels / 3; i++) {
		pixels[i].g = channels[(i * 3) + 0];
		pixels[i].r = channels[(i * 3) + 1];
		pixels[i].b = channels[(i * 3) + 2];
	}

	return ws2812_pico_update_rgb(dev, pixels, num_channels / 3);
}

static int ws2812_pico_init(const struct device *dev) {
	const struct ws2812_pico_cfg *cfg = dev->config;

	if (!device_is_ready(cfg->in_gpio.port)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&cfg->in_gpio, GPIO_OUTPUT_INACTIVE);
}

static const struct led_strip_driver_api ws2812_pico_api = {
	.update_rgb = ws2812_pico_update_rgb,
	.update_channels = ws2812_pico_update_channels,
};

#define WS2812_PICO_INST(n)                                                          \
	static struct ws2812_pico_cfg ws2812_pico_cfg_##n = {                          \
		.in_gpio = GPIO_DT_SPEC_INST_GET(n, in_gpios),                          \
		.chain_length = DT_INST_PROP(n, chain_length),                          \
	};                                                                             \
	DEVICE_DT_DEFINE(DT_DRV_INST(n), ws2812_pico_init, NULL, NULL,                 \
			 &ws2812_pico_cfg_##n, POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, \
			 &ws2812_pico_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_PICO_INST)