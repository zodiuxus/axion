#include <cstdio>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logic.h"

extern "C" {
  void app_main(void);
  void temp_setup(void);
}

void i2c_setup();
void mpu_setup();
void uart_setup();
void at_init();
void combine_sensors();

void app_main(void)
{
  sysReady = xEventGroupCreate();
  // debugOutput = true; // only used to log other info to console
  xTaskCreate(TaskFunction_t(temp_setup), "ds18b20", 4096, nullptr, 0, nullptr);
  xTaskCreate(TaskFunction_t(i2c_setup), "i2c", 1024*2, nullptr, 1, nullptr);
  xTaskCreate(TaskFunction_t(uart_setup), "uart", 2048, nullptr, 1, nullptr);
  xTaskCreate(TaskFunction_t(mpu_setup), "mpu", 1024*4, nullptr, 0, nullptr);
  xTaskCreate(TaskFunction_t(at_init), "at+gnss", 4096, nullptr, 0, nullptr);
  xTaskCreate(TaskFunction_t(combine_sensors), "sensor_output", 4096, nullptr, 0, nullptr);
}
