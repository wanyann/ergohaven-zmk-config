/*
 * Copyright (c) 2026 The phenom_mini config contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Listens to zmk_layer_state_changed on the central (left) half and drives
 * both halves' WS2812 strips through the &rgb_ug behavior (locality GLOBAL,
 * so a single invocation is forwarded to all split peripherals):
 *   - AutoMouse layer (4) active  -> green
 *   - AutoMouse layer inactive    -> white
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <dt-bindings/zmk/rgb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define AUTOMOUSE_LAYER 4

static void set_underglow_color(uint8_t hue, uint8_t sat, uint8_t brt) {
    const struct device *rgb_ug_dev = DEVICE_DT_GET(DT_NODELABEL(rgb_ug));

    if (!device_is_ready(rgb_ug_dev)) {
        LOG_WRN("rgb_ug behavior device not ready");
        return;
    }

    struct zmk_behavior_binding binding = {
        .behavior_dev = rgb_ug_dev->name,
        .param1 = RGB_COLOR_HSB_CMD,
        .param2 = RGB_COLOR_HSB_VAL(hue, sat, brt),
    };

    struct zmk_behavior_binding_event event = {
        .position = 0,
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

int err = zmk_behavior_invoke_binding(&binding, event, true);
    if (err) {
        LOG_WRN("Failed to invoke rgb_ug behavior: %d", err);
    }
    printk("[LCI] set h=%d s=%d b=%d\n", hue, sat, brt);
}

static int layer_color_indicator_listener(const zmk_event_t *eh) {
    struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_HANDLED;
    }

    if (ev->layer == AUTOMOUSE_LAYER) {
        printk("[LCI] layer4 state=%d\n", ev->state);
        if (ev->state) {
            set_underglow_color(120, 100, 10); // AutoMouse active: green
        } else {
            set_underglow_color(0, 0, 10); // AutoMouse released: white
        }
    }

    return ZMK_EV_EVENT_HANDLED;
}

static int layer_color_indicator_init(void);

ZMK_LISTENER(layer_color_indicator, layer_color_indicator_listener);
ZMK_SUBSCRIPTION(layer_color_indicator, zmk_layer_state_changed);

SYS_INIT(layer_color_indicator_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int layer_color_indicator_init(void) {
    set_underglow_color(0, 0, 10); // Start white at matching reduced brightness
    return 0;
}