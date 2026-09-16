zephyr_library()

zephyr_library_sources_ifdef(CONFIG_EH_PMW3610_GPIO src/pmw3610_gpio.c)

if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  zephyr_library_sources_ifdef(CONFIG_ZMK_LAYER_COLOR_INDICATOR src/layer_color_indicator.c)
  zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()