#include <stdio.h>
#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

#define PIN_MPU_INT 6

#define I2C_MASTER_FREQ_HZ 100000

static const char* TAG = "axion test";

static mpu6050_handle_t mpu6050_dev = NULL;
static mpu6050_acce_value_t accel;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t comp_angle;

void i2c_setup() {
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .scl_io_num = PIN_I2C_SCL,
    .sda_io_num = PIN_I2C_SDA,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
  };

  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

void mpu_setup() {
  mpu6050_dev = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
  ESP_ERROR_CHECK(mpu6050_config(mpu6050_dev, ACCE_FS_2G, GYRO_FS_500DPS));
  ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050_dev));
}


void display_gyro_accel() {
  while (1) {
    mpu6050_get_gyro(mpu6050_dev, &gyro);
    mpu6050_get_acce(mpu6050_dev, &accel);
    mpu6050_complimentory_filter(mpu6050_dev, &accel, &gyro, &comp_angle);
    ESP_LOGI(TAG, "Accel_x:%.2f, Accel_y:%.2f, Accel_z:%.2f", accel.acce_x, accel.acce_y, accel.acce_z);
    ESP_LOGI(TAG, "Gyro_x:%.2f, Gyro_y:%.2f, Gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  printf("Hello, world!\nWe are somehow live!\n");
  i2c_setup();
  mpu_setup();
  xTaskCreate(display_gyro_accel, "display_gyro_accel", 4096, NULL, 2, NULL);
}
