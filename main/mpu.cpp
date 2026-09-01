/**
 * mpu.cpp - MPU6050 setup + interrupt-driven DMP/collision task.
 *
 * ---- Accel range: why we calibrate at ±2g and stay there --------------
 * The collision path reads the accel registers directly, so the g-scale
 * depends on AFS_SEL (±2g -> 16384 LSB/g, ±4g -> 8192 LSB/g). An earlier
 * revision calibrated the accel at ±4g and divided by 8192 - and the
 * resting baseline still read exactly 2.00 g. Root cause: i2cdevlib's
 * CalibrateAccel() PID helper (MPU6050.cpp, PID()) has the gravity-
 * removal constant hardcoded for the ±2g range:
 *
 *     if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;  //remove Gravity
 *
 * Calibrated at ±4g, the PID loop saw a phantom "-0.5 g" residual on Z
 * (8192 real counts minus the 16384 it expects) and kept increasing the
 * Z accel OFFSET register until raw Z read 16384 - which is exactly
 * 2.00 g at ±4g scale. The 2 g baseline was calibrated INTO the device
 * (X/Y were unaffected - gravity only loads Z), and DMP orientation
 * kept working by coincidence: the DMP consumes accel at its own
 * internal scale, where 16384 counts is exactly 1 g.
 *
 * Therefore: ALWAYS calibrate at ±2g (that's the range the library's
 * constant assumes), and derive the conversion constant from the LIVE
 * AFS_SEL register at runtime so a future FSR change can never silently
 * corrupt the scale again. If you ever need headroom above 2 g (±4g),
 * set the FSR AFTER CalibrateAccel() - never before - and the runtime
 * constant adapts automatically.
 *
 * Collision detection with ±2g: a single axis clips at 2.0 g, i.e.
 * exactly the trigger threshold. That is still safe: |a| >= threshold
 * holds at the clip point, and real impacts load multiple axes. What
 * we lose is magnitude fidelity above 2 g, which the collision path
 * doesn't use - it only compares against the threshold.
 *
 * Collision ESCALATION (not detection) is gated on the system "armed"
 * flag in state.h: mpu_int_task only sets BIT_COLLISION_DETECTED once
 * modem_setup_task has finished. Impacts during boot/mounting are
 * logged but not escalated, because the SMS path isn't up yet.
 *
 * The data-ready interrupt fires at the sensor sample rate (200 Hz -
 * SMPLRT_DIV=4, base 1 kHz). It is ORed onto the INT pin with any other
 * enabled interrupt source; we currently only enable data-ready, so the
 * pin effectively pulses on each new sample. mpu_int_task wakes on each
 * rising edge via a task notification, then performs both:
 *   - raw accel magnitude check → BIT_COLLISION_DETECTED
 *   - DMP FIFO drain (if a packet is ready) → axion_state_set_ypr()
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
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "config.h"
#include "state.h"
#include "collision.h"

static const char *TAG = "axion.mpu";

/* Accel sensitivity in LSB/g, derived from the LIVE AFS_SEL register in
 * mpu_setup(): LSB/g = 65536 / (4 << AFS_SEL).
 *   AFS_SEL 0 (±2g)  -> 16384,  1 (±4g) -> 8192,
 *   AFS_SEL 2 (±8g)  ->  4096,  3 (±16g) -> 2048.
 * The default below is the ±2g power-on value so pre-init callers
 * behave sanely; mpu_setup() overwrites it from the register. */
static float s_lsb_per_g = 16384.0f;

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

    /* dmpInitialize() starts with a full device reset, so ACCEL_CONFIG
     * is at its ±2g power-on default here. We calibrate EXPLICITLY at
     * ±2g because the library's gravity-removal constant (16384) is
     * hardcoded for that range - see the file header for how a ±4g
     * calibration baked a 2 g error into the Z offset register. If you
     * ever switch the operating range to ±4g, do it AFTER these two
     * calls, never before. */
    ESP_LOGI(TAG, "calibrating accel+gyro - keep the device still");
    s_mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    s_mpu.CalibrateAccel(6);
    s_mpu.CalibrateGyro(6);

    /* Sample rate divider: 1 kHz / (1 + 4) = 200 Hz data-ready rate.
     * dmpInitialize already picked the same value for its self-test,
     * but re-applying keeps MPU_SAMPLE_RATE_DIV the single source of
     * truth for the interrupt rate. */
    s_mpu.setRate(MPU_SAMPLE_RATE_DIV);

    /* Derive the g-conversion constant from the ACTUAL register instead
     * of hardcoding a divisor - the class of bug described in the file
     * header can never silently corrupt the scale again. */
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

    /* The GPIO ISR service is installed once, centrally, in app_main
     * (axion.cpp). A second gpio_install_isr_service() call not only
     * returns ESP_ERR_INVALID_STATE - the IDF driver itself logs
     * "GPIO isr service already installed" at ERROR level even when the
     * caller handles the code gracefully. We only attach our handler. */
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
         *    returns the most recent sample. The conversion uses the
         *    runtime-derived sensitivity, NOT a hardcoded divisor. */
        int16_t rx = 0, ry = 0, rz = 0;
        s_mpu.getAcceleration(&rx, &ry, &rz);
        float ax = (float)rx / s_lsb_per_g;
        float ay = (float)ry / s_lsb_per_g;
        float az = (float)rz / s_lsb_per_g;

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (collision_check(ax, ay, az, now_ms, &last_collision_ms)) {
            if (!axion_state_is_armed()) {
                /* Impact before modem bring-up finished (a bump while
                 * mounting the box, bench handling): log it, but don't
                 * escalate - the monitor/SMS path isn't running yet, so
                 * the bit would just sit in the event group and fire a
                 * stale ALERT the moment the monitor starts. */
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

    if (ax) *ax = (float)rx / s_lsb_per_g;
    if (ay) *ay = (float)ry / s_lsb_per_g;
    if (az) *az = (float)rz / s_lsb_per_g;
    return true;
}
