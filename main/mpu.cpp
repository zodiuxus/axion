/**
 * mpu.cpp — MPU6050 setup + DMP YPR task + raw accel helper.
 *
 * The MPU6050's full-scale accel range is set to ±4g (8192 LSB/g) rather
 * than the library default of ±2g. This gives headroom above the 2G
 * collision trigger threshold (COLLISION_THRESHOLD_G) so that single-axis
 * readings at the trigger point don't saturate at int16 max. The DMP
 * output is unaffected by this setting — it uses its own internal
 * scaling for the gravity vector and quaternion computation.
 */
#include "mpu.h"

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"
#include "state.h"

static const char *TAG = "axion.mpu";

/* Accel sensitivity at ±4g full-scale range.
 * ±4g -> 16-bit signed range / 8g span = 65536/8 = 8192 LSB/g. */
#define MPU_ACCEL_LSB_PER_G    8192.0f

static MPU6050  s_mpu;
static uint16_t s_packet_size = 42;
static uint8_t  s_fifo_buffer[64];

void i2c_bus_setup(void)
{
    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = PIN_I2C_SDA;
    conf.scl_io_num       = PIN_I2C_SCL;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C bus ready (SDA=%d SCL=%d %u Hz)",
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_MASTER_FREQ_HZ);

    vTaskDelete(nullptr);
}

void mpu_setup(void)
{
    s_mpu.initialize();
    /* Widen the accel range to ±4g BEFORE calibration so the offset
     * registers are calibrated at the range we'll actually operate in.
     * The default ±2g would clip at exactly COLLISION_THRESHOLD_G on a
     * single axis. ±4g gives 8192 LSB/g and 2g of headroom above the
     * 2G trigger. The DMP is unaffected (it uses internal scaling). */
    s_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    s_mpu.dmpInitialize();

    if (!s_mpu.testConnection()) {
        ESP_LOGE(TAG, "MPU6050 connection failed");
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 connected (accel range ±4g)");

    s_mpu.CalibrateAccel(6);
    s_mpu.CalibrateGyro(6);
    s_mpu.setDMPEnabled(true);
    s_packet_size = s_mpu.dmpGetFIFOPacketSize();
    ESP_LOGI(TAG, "DMP started, packet size = %u", s_packet_size);

    axion_state_set_ready(BIT_MPU_READY);
    vTaskDelete(nullptr);
}

void mpu_rpy_task(void * /*arg*/)
{
    axion_state_wait_all(BIT_MPU_READY);

    Quaternion    q;
    VectorFloat   gravity;
    float         ypr[3];

    while (true) {
        uint8_t  int_status = s_mpu.getIntStatus();
        uint16_t fifo_count = s_mpu.getFIFOCount();

        /* Overflow: reset and skip this iteration. */
        if ((int_status & 0x10) || fifo_count == 1024) {
            s_mpu.resetFIFO();
            ESP_LOGW(TAG, "FIFO overflow, reset");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if ((int_status & 0x02) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        while (fifo_count < s_packet_size) {
            fifo_count = s_mpu.getFIFOCount();
        }
        s_mpu.getFIFOBytes(s_fifo_buffer, s_packet_size);
        s_mpu.dmpGetQuaternion(&q, s_fifo_buffer);
        s_mpu.dmpGetGravity(&gravity, &q);
        s_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        axion_state_set_ypr(ypr);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool mpu_get_raw_accel_g(float *ax, float *ay, float *az)
{
    /* Wait until the MPU has been initialized. Callers (e.g. collision_task)
     * normally wait on BIT_MPU_READY first, but this guard makes the helper
     * safe to call even before init completes — it just returns false. */
    if ((xEventGroupGetBits(g_sensors_ready) & BIT_MPU_READY) == 0) {
        return false;
    }

    int16_t rx = 0, ry = 0, rz = 0;
    s_mpu.getAcceleration(&rx, &ry, &rz);

    if (ax) *ax = (float)rx / MPU_ACCEL_LSB_PER_G;
    if (ay) *ay = (float)ry / MPU_ACCEL_LSB_PER_G;
    if (az) *az = (float)rz / MPU_ACCEL_LSB_PER_G;
    return true;
}
