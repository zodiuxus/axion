set(COMPONENT_ADD_INCLUDEDIRS include)
# The RMT driver variant (owb_rmt.c) is not built: it depends on driver/rmt.h,
# which ESP-IDF 6.x removed. The GPIO bit-bang driver below is the one Axion
# uses (owb_gpio_initialize in temperature.cpp).
set(COMPONENT_SRCS "owb.c" "owb_gpio.c")
set(COMPONENT_REQUIRES "soc" "esp_driver_gpio" "esp_rom")
register_component()
