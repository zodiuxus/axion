#include "stdio.h"
#include "driver/gpio.h"

void blink_led(uint8_t pin_led) {
  gpio_reset_pin(pin_led);
  gpio_set_direction(pin_led, GPIO_MODE_OUTPUT);

  while (1) {
    gpio_set_level(pin_led, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    gpio_set_level(pin_led, 0);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
