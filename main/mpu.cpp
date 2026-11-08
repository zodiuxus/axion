/**
 * mpu.cpp - MPU6050 setup + interrupt-driven DMP/collision task.
 *
 * Offset persistence: the PID calibration runs only on the first boot
 * (or after factory reset / re-mounting); the six raw offset register
 * values are stored in NVS ("mpu_offs") and rewritten on later boots -
 * no "keep still" requirement, and setup is ~1-2 s faster.
 *
 * ALWAYS calibrate the accel at ±2g: i2cdevlib's CalibrateAccel() PID
 * has the gravity-removal constant (16384) hardcoded for that range.
 * If switching FSR to ±4g, do it AFTER applying offsets. The g-scale
 * constant is derived from the live AFS_SEL register at runtime.
 *
 * Collision escalation is gated on the system armed flag (state.h).
 * The data-ready INT (200 Hz) drives both the raw-accel collision check
 * and the DMP FIFO drain from this single task (one INT pin, and a
 * second bus consumer would only race the FIFO read).
 */
#include "mpu.h"

#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "config.h"
#include "state.h"
#include "collision.h"
#include "calibration_store.h"

static const char *TAG = "axion.mpu";

/* Accel sensitivity in LSB/g, derived from the live AFS_SEL register in
 * mpu_setup(): LSB/g = 65536 / (4 << AFS_SEL). Default is the ±2g
 * power-on value. */
static float s_lsb_per_g = 16384.0f;

/* Sample rate divider: 1 kHz / (1 + 4) = 200 Hz - the data-ready
 * interrupt frequency. */
#define MPU_SAMPLE_RATE_DIV    4

static MPU6050  s_mpu;
static uint16_t s_packet_size = 42;
static uint8_t  s_fifo_buffer[64];

/* Task handle for the ISR → task notification path. */
static TaskHandle_t s_mpu_int_task_handle = nullptr;

/* ---- GPIO ISR: just notify the task ---------------------------------- */
/* Never do I2C or bus access from an ISR; rising edge (~50 us pulse). */
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
    if (!I2Cdev::installBus(PIN_I2C_SDA, PIN_I2C_SCL, I2C_MASTER_FREQ_HZ)) {
        ESP_LOGE(TAG, "I2C bus install failed (SDA=%d SCL=%d)",
                 PIN_I2C_SDA, PIN_I2C_SCL);
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "I2C bus ready (SDA=%d SCL=%d %u Hz)",
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_MASTER_FREQ_HZ);

    /* Probe both slave addresses up front. If a chip doesn't ACK here,
    * every subsequent transaction would fail anyway - one clear wiring
    * diagnostic beats hundreds of per-transaction error lines later
    * during the DMP firmware load. */
    if (!I2Cdev::probe(0x68)) {
        ESP_LOGE(TAG, "MPU6050 not answering at 0x68 - check SDA/SCL "
                      "wiring, common GND, and pull-ups (4.7k external "
                      "recommended if the module has none)");
    }
    if (!I2Cdev::probe(0x57)) {
        ESP_LOGE(TAG, "MAX30102 not answering at 0x57 - check wiring/"
                      "pull-ups; INT pin must NOT be on SDA/SCL");
    }

    vTaskDelete(nullptr);
}

void mpu_setup(void)
{
    s_mpu.initialize();
    s_mpu.dmpInitialize();

    /* dmpInitialize() resets the device: ACCEL_CONFIG is back at ±2g
     * and the offset registers are zero - offsets are (re)applied below.
     * Calibrate EXPLICITLY at ±2g (see file header); switch to ±4g only
     * AFTER applying the offsets. */
    s_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    /* ---- Calibrate once, then persist to NVS --------------------------- */
    /* Run the PID loop once, save the six raw offset register values as
     * an NVS blob ("mpu_offs"), and just rewrite them on every later boot.
     * Accel offsets are orientation-dependent (gravity baked in):
     * factory reset or re-mounting must invalidate them. */
    int16_t offs[6] = {0};
    if (calibration_store_is_flag_set(CAL_FLAG_MPU_OFFSETS_VALID) &&
        calibration_store_get_mpu_offsets(offs) == ESP_OK) {
        s_mpu.setXAccelOffset(offs[0]);
        s_mpu.setYAccelOffset(offs[1]);
        s_mpu.setZAccelOffset(offs[2]);
        s_mpu.setXGyroOffset(offs[3]);
        s_mpu.setYGyroOffset(offs[4]);
        s_mpu.setZGyroOffset(offs[5]);
        ESP_LOGI(TAG, "MPU offsets restored from NVS (accel %d/%d/%d, "
                      "gyro %d/%d/%d) - skipping calibration",
                 offs[0], offs[1], offs[2],
                 offs[3], offs[4], offs[5]);
    } else {
        ESP_LOGI(TAG, "no stored MPU offsets - calibrating accel+gyro, "
                      "keep the device still");
        s_mpu.CalibrateAccel(6);
        s_mpu.CalibrateGyro(6);

        offs[0] = s_mpu.getXAccelOffset();
        offs[1] = s_mpu.getYAccelOffset();
        offs[2] = s_mpu.getZAccelOffset();
        offs[3] = s_mpu.getXGyroOffset();
        offs[4] = s_mpu.getYGyroOffset();
        offs[5] = s_mpu.getZGyroOffset();

        if (calibration_store_set_mpu_offsets(offs) == ESP_OK) {
            calibration_store_set_flag(CAL_FLAG_MPU_OFFSETS_VALID);
            ESP_LOGI(TAG, "MPU offsets saved to NVS (accel %d/%d/%d, "
                          "gyro %d/%d/%d) - future boots skip calibration",
                     offs[0], offs[1], offs[2],
                     offs[3], offs[4], offs[5]);
        } else {
            ESP_LOGW(TAG, "MPU offset NVS save failed - will re-calibrate "
                          "on next boot");
        }
    }

    /* Re-apply to keep MPU_SAMPLE_RATE_DIV the single source of truth. */
    s_mpu.setRate(MPU_SAMPLE_RATE_DIV);

    /* Derive the g-scale from the live register so a future FSR change
     * cannot silently corrupt the conversion. */
    uint8_t afs = s_mpu.getFullScaleAccelRange();
    s_lsb_per_g = 65536.0f / (float)(4u << afs);
    if (afs != (uint8_t)MPU6050_ACCEL_FS_2) {
        ESP_LOGW(TAG, "AFS_SEL=%u (expected 0 = ±2g) - sensitivity %.0f LSB/g",
                 afs, (double)s_lsb_per_g);
    }
    ESP_LOGI(TAG, "accel ±%ug, %.0f LSB/g, sample rate %u Hz",
             2 << afs, (double)s_lsb_per_g, 1000 / (1 + MPU_SAMPLE_RATE_DIV));

    if (!s_mpu.testConnection()) {
        ESP_LOGE(TAG, "MPU6050 connection failed");
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 connected");

    s_mpu.setDMPEnabled(true);
    s_packet_size = s_mpu.dmpGetFIFOPacketSize();
    ESP_LOGI(TAG, "DMP started, packet size = %u", s_packet_size);

    /* DATA_RDY_EN: pulse the INT pin on every new sample (drives the
     * collision check). */
    s_mpu.setIntDataReadyEnabled(true);
    /* Clear latched status before hooking up the GPIO. */
    (void)s_mpu.getIntStatus();

    /* Input + pull-up (harmless for the push-pull INT pin, protects
     * against a floating pin if the MPU is unpowered). */
    gpio_reset_pin(PIN_MPU_INT);
    gpio_set_direction(PIN_MPU_INT, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_MPU_INT);

    /* ISR service is installed centrally in app_main; just attach. */
    esp_err_t isr_err = gpio_isr_handler_add(PIN_MPU_INT, mpu_int_isr, nullptr);
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
/* Wakes on every PIN_MPU_INT rising edge: reads INT_STATUS, checks raw
 * accel for collision, drains one DMP FIFO packet into shared YPR state.
 * The only task that touches the MPU (I2C driver serializes the bus). */
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
        /* Block until notified; pdTRUE collapses back-to-back INTs. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 1) Read INT_STATUS - clears latched bits on the MPU6050. */
        uint8_t int_status = s_mpu.getIntStatus();
        uint16_t fifo_count = s_mpu.getFIFOCount();

        /* 2) Raw accel + collision check (independent of the FIFO; uses
         *    the runtime-derived sensitivity). */
        int16_t rx = 0, ry = 0, rz = 0;
        s_mpu.getAcceleration(&rx, &ry, &rz);
        float ax = (float)rx / s_lsb_per_g;
        float ay = (float)ry / s_lsb_per_g;
        float az = (float)rz / s_lsb_per_g;

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (collision_check(ax, ay, az, now_ms, &last_collision_ms)) {
            if (!axion_state_is_armed()) {
                /* Not armed yet: log but don't escalate - the monitor
                 * isn't running, so the bit would fire a stale ALERT
                 * later. */
                ESP_LOGI(TAG, "COLLISION: |a|=%.2f g (x=%.2f y=%.2f z=%.2f)"
                              " - not armed yet, no escalation",
                         (double)sqrtf(ax * ax + ay * ay + az * az),
                         (double)ax, (double)ay, (double)az);
            } else {
                ESP_LOGW(TAG, "COLLISION: |a|=%.2f g (x=%.2f y=%.2f z=%.2f) -> ALERT",
                         (double)sqrtf(ax * ax + ay * ay + az * az),
                         (double)ax, (double)ay, (double)az);
                xEventGroupSetBits(g_sensors_ready, BIT_COLLISION_DETECTED);
            }
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

        /* Bounded wait (~30 ms) for a full DMP packet: an unbounded
         * spin would pin the highest-priority task and hammer the I2C
         * bus if the DMP wedges. */
        int fifo_spins = 0;
        while (fifo_count < s_packet_size) {
            if (++fifo_spins > 150) {
                s_mpu.resetFIFO();
                ESP_LOGW(TAG, "DMP FIFO stalled (no full packet in ~30 ms) "
                              "- reset, waiting for next INT");
                break;
            }
            fifo_count = s_mpu.getFIFOCount();
        }
        if (fifo_count < s_packet_size) continue;
        s_mpu.getFIFOBytes(s_fifo_buffer, s_packet_size);
        s_mpu.dmpGetQuaternion(&q, s_fifo_buffer);
        s_mpu.dmpGetGravity(&gravity, &q);
        s_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        axion_state_set_ypr(ypr);
    }
}

bool mpu_get_raw_accel_g(float *ax, float *ay, float *az)
{
    /* Returns false if called before BIT_MPU_READY is set. */
    if ((xEventGroupGetBits(g_sensors_ready) & BIT_MPU_READY) == 0) {
        return false;
    }

    int16_t rx = 0, ry = 0, rz = 0;
    s_mpu.getAcceleration(&rx, &ry, &rz);

    if (ax) *ax = (float)rx / s_lsb_per_g;
    if (ay) *ay = (float)ry / s_lsb_per_g;
    if (az) *az = (float)rz / s_lsb_per_g;
    return true;
}
