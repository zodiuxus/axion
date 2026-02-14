#include <cstdio>
#include <cstring>
#include <logic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
 void app_main(void);
}

void i2c_setup();
void mpu_setup();
void uart_setup();
void at_init();
void app_main(void)
{

  // xTaskCreate(TaskFunction_t(i2c_setup), "i2c", 1024*2, nullptr, 2, nullptr);
  // xTaskCreate(TaskFunction_t(uart_setup), "uart", 2048, nullptr, 2, nullptr);
  // vTaskDelay(500);
  // xTaskCreate(TaskFunction_t(mpu_setup), "mpu", 1024*4, nullptr, 2, nullptr);
  // xTaskCreate(TaskFunction_t(at_init), "at+gnss", 4096, nullptr, 2, nullptr);
    xTaskCreate(TaskFunction_t(get_thermometer_reading), "thermometer", 1024, nullptr, 0, nullptr);
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(TaskFunction_t(health_check), "health_check", 1024*4, nullptr, 0, nullptr);
}
