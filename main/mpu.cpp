/**
 * mpu.cpp - MPU6050 setup + interrupt-driven DMP/collision task.
 *
 * The MPU6050's full-scale accel range is set to ±4g (8192 LSB/g) rather
 * than the library default of ±2g. This gives headroom above the 2G
 * collision trigger threshold (COLLISION_THRESHOLD_G) so that single-axis
 * readings at the trigger point don't saturate at int16 max. The DMP
 * output is unaffected by this setting - it uses its own internal
 * scaling for the gravity vector and quaternion computation.
 *
 * The data-ready interrupt fires at the sensor sample rate (200 Hz -
 * SMPLRT_DIV=4, base 1 kHz). It is ORed onto the INT pin with any other
 * enabled interrupt source; we currently only enable data-ready, so the
 * pin effectively pulses on each new sample. mpu_int_task wakes on each
 * rising edge via a task notification, then performs both:
 *   - raw accel magnitude check -> BIT_COLLISION_DETECTED
 *   - DMP FIFO drain (if a packet is ready) -> axion_state_set_ypr()
 *
 * Why one task and not two: the I2C bus is shared, and a separate
 * collision_task would race with mpu_int_task for the bus mutex. Having
 * a single consumer also halves the ISR→read latency for collision
 * (no scheduling delay between two same-priority tasks).
 */
#include "mpu.h"

#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"
#include "state.h"
#include "collision.h"

static const char *TAG = "axion.mpu";

/* Accel sensitivity at ±4g full-scale range.
 * ±4g -> 16-bit signed range / 8g span = 65536/8 = 8192 LSB/g. */
#define MPU_ACCEL_LSB_PER_G    8192.0f

/* Sample rate divider: 1 kHz / (1 + 4) = 200 Hz. This drives the
 * data-ready interrupt frequency - fast enough for sub-5ms collision
 * latency, slow enough to not saturate the I2C bus at 400 kHz. */
#define MPU_SAMPLE_RATE_DIV    4

static MPU6050  s_mpu;
static uint16_t s_packet_size = 42;
static uint8_t  s_fifo_buffer[64];

/* Task handle for the ISR → task notification path. */
static TaskHandle_t s_mpu_int_task_handle = nullptr;

/* ---- GPIO ISR: just notify the task ---------------------------------- */
/* Keep this minimal - never do I2C or any bus access from an ISR.
 * The data-ready interrupt is level-pulsed (active-high for ~50 µs),
 * so we trigger on the rising edge. */
static void IRAM_ATTR mpu_int_isr(void * /*arg*/)
{
    BaseType_t hp = pdFALSE;
    if (s_mpu_int_task_handle) {
        vTaskNotifyGiveFromISR(s_mpu_int_task_handle, &hp);
    }
    portYIELD_FROM_ISR(hp);
}

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
    /* Set sample rate to 200 Hz (1 kHz / (1+4)). This drives the
     * data-ready interrupt - see MPU_SAMPLE_RATE_DIV above. */
    s_mpu.setRate(MPU_SAMPLE_RATE_DIV);
    s_mpu.dmpInitialize();

    if (!s_mpu.testConnection()) {
        ESP_LOGE(TAG, "MPU6050 connection failed");
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 connected (accel ±4g, sample rate %u Hz)",
             1000 / (1 + MPU_SAMPLE_RATE_DIV));

    s_mpu.CalibrateAccel(6);
    s_mpu.CalibrateGyro(6);
    s_mpu.setDMPEnabled(true);
    s_packet_size = s_mpu.dmpGetFIFOPacketSize();
    ESP_LOGI(TAG, "DMP started, packet size = %u", s_packet_size);

    /* Enable data-ready interrupt. The DMP's own interrupt is already
     * enabled by setDMPEnabled(true); we also need DATA_RDY_EN so the
     * INT pin pulses on every new sample (drives our collision check). */
    s_mpu.setIntDataReadyEnabled(true);
    /* Clear any latched interrupt status before we hook up the GPIO,
     * so the first ISR doesn't fire on a stale event. */
    (void)s_mpu.getIntStatus();

    /* Configure PIN_MPU_INT as input with internal pull-up (the MPU6050
     * INT pin is push-pull, but the pull-up is harmless and protects
     * against the pin floating if the MPU is unpowered). */
    gpio_reset_pin(PIN_MPU_INT);
    gpio_set_direction(PIN_MPU_INT, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_MPU_INT);

    /* Install the GPIO ISR service if not already installed (the alert
     * button module may have done it first - that's fine). */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
    }
    isr_err = gpio_isr_handler_add(PIN_MPU_INT, mpu_int_isr, nullptr);
    if (isr_err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add: %s", esp_err_to_name(isr_err));
    }
    /* The MPU6050 INT pin is active-high pulse (~50 µs wide). */
    gpio_set_intr_type(PIN_MPU_INT, GPIO_INTR_POSEDGE);

    ESP_LOGI(TAG, "MPU INT on GPIO %d (data-ready, ~%u Hz)",
             PIN_MPU_INT, 1000 / (1 + MPU_SAMPLE_RATE_DIV));

    axion_state_set_ready(BIT_MPU_READY);
    vTaskDelete(nullptr);
}

/* ---- ISR-driven consumer task ---------------------------------------- */
/* Wakes on every PIN_MPU_INT rising edge, then:
 *   1. Reads INT_STATUS to see what fired (and to clear latched bits).
 *   2. Reads raw accel registers, computes |a|, calls collision_check().
 *      If collision_check returns true, sets BIT_COLLISION_DETECTED.
 *   3. If the DMP data-ready bit is set, drains one FIFO packet and
 *      parses yaw/pitch/roll into shared state.
 *
 * Both reads happen under the implicit I2C mutex (the ESP-IDF I2C driver
 * serializes transactions on the same port). No explicit lock needed
 * because this is the only task that touches the MPU. */
void mpu_int_task(void * /*arg*/)
{
    axion_state_wait_all(BIT_MPU_READY);

    /* Cache our own handle for the ISR. */
    s_mpu_int_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "MPU INT task started");

    Quaternion    q;
    VectorFloat   gravity;
    float         ypr[3];
    int64_t       last_collision_ms = 0;

    while (true) {
        /* Block until the ISR notifies us. pdTRUE clears the count to 0
         * on wake, so back-to-back interrupts (faster than we can
         * service) collapse into one wake - that's fine, we'll just
         * drain whatever's in the FIFO. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 1) Read INT_STATUS - clears latched bits on the MPU6050. */
        uint8_t int_status = s_mpu.getIntStatus();
        uint16_t fifo_count = s_mpu.getFIFOCount();

        /* 2) Raw accel + collision check. We do this on every wake,
         *    regardless of int_status, because the data-ready bit may
         *    have been cleared by the FIFO read above. Reading the raw
         *    accel registers is independent of the FIFO and always
         *    returns the most recent sample. */
        int16_t rx = 0, ry = 0, rz = 0;
        s_mpu.getAcceleration(&rx, &ry, &rz);
        float ax = (float)rx / MPU_ACCEL_LSB_PER_G;
        float ay = (float)ry / MPU_ACCEL_LSB_PER_G;
        float az = (float)rz / MPU_ACCEL_LSB_PER_G;

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (collision_check(ax, ay, az, now_ms, &last_collision_ms)) {
            ESP_LOGW(TAG, "COLLISION: |a|=%.2f g (x=%.2f y=%.2f z=%.2f) -> ALERT",
                     (double)sqrtf(ax * ax + ay * ay + az * az),
                     (double)ax, (double)ay, (double)az);
            xEventGroupSetBits(g_sensors_ready, BIT_COLLISION_DETECTED);
        }

        /* 3) DMP FIFO drain. */
        if ((int_status & 0x10) || fifo_count == 1024) {
            /* Overflow: reset and skip this iteration's YPR update. */
            s_mpu.resetFIFO();
            ESP_LOGW(TAG, "FIFO overflow, reset");
            continue;
        }
        if ((int_status & 0x02) == 0) {
            /* No DMP packet ready - just a data-ready pulse. */
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
    }
}

bool mpu_get_raw_accel_g(float *ax, float *ay, float *az)
{
    /* Wait until the MPU has been initialized. Callers normally wait
     * on BIT_MPU_READY first, but this guard makes the helper safe to
     * call even before init completes - it just returns false. */
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
