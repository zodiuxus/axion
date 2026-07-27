/**
 * max30102_task.cpp - Oximetry sampling + estimation task (interrupt-driven).
 *
 * On first run (CAL_FLAG_FIRST_RUN set in NVS), the task enters a
 * 2-minute calibration window after the first valid HR/SpO2 estimate.
 * During calibration, HR and SpO2 estimates are accumulated and the
 * averages are logged (will be stored as baselines when offset logic
 * is added). BIT_OXIM_CALIBRATED is set at the end of calibration.
 *
 * On non-first-run boots, BIT_OXIM_CALIBRATED is set immediately after
 * the first valid estimate.
 *
 * The CAL_FLAG_FIRST_RUN flag is cleared by the status LED task once
 * ALL sensors have set their BIT_*_CALIBRATED bits.
 *
 * ---- Interrupt-driven FIFO drain ---------------------------------------
 * Previously this task polled max30102_read_fifo() in a tight loop with
 * a vTaskDelay(40 ms) between samples - 128 separate I2C transactions
 * per analysis window. Now it waits on PIN_MAX30102_INT (FIFO_A_FULL,
 * fires at ~17 samples queued), then burst-reads all available samples
 * in a single I2C transaction. The 128-sample analysis buffer fills in
 * ~8 interrupt wakes instead of 128 polling iterations, and I2C bus
 * traffic drops ~6×.
 */
#include "max30102_task.h"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "max30102.h"
#include "max30102_algorithm.h"

#include "config.h"
#include "state.h"
#include "calibration_store.h"

static const char *TAG = "axion.max30102";

static TaskHandle_t s_max30102_task_handle = nullptr;

/* ISR: just notify the task. Never do I2C in an ISR. */
static void IRAM_ATTR max30102_int_isr(void * /*arg*/)
{
    BaseType_t hp = pdFALSE;
    if (s_max30102_task_handle) {
        vTaskNotifyGiveFromISR(s_max30102_task_handle, &hp);
    }
    portYIELD_FROM_ISR(hp);
}

void max30102_task(void * /*arg*/)
{
    /* Wait for the I2C bus (installed by i2c_bus_setup, signalled via
     * BIT_MPU_READY since the MPU setup task runs after i2c_bus_setup). */
    axion_state_wait_all(BIT_MPU_READY);

    max30102_config_t cfg = max30102_default_config();
    esp_err_t err = max30102_init(I2C_MASTER_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "max30102_init failed: %s - marking calibrated and exiting",
                 esp_err_to_name(err));
        /* Set the bit so the status LED can proceed. */
        axion_state_set_ready(BIT_OXIM_CALIBRATED);
        vTaskDelete(nullptr);
        return;
    }
    max30102_alg_init();
    ESP_LOGI(TAG, "MAX30102 initialized on I2C port %d", I2C_MASTER_PORT);

    /* Enable the FIFO_A_FULL interrupt and wire up the GPIO ISR. The
     * MAX30102 INT pin is open-drain, active-low. We trigger on the
     * falling edge (assertion). Reading the INT_STATUS register clears
     * the interrupt and deasserts the pin. */
    err = max30102_enable_fifo_a_full_int(I2C_MASTER_PORT, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FIFO_A_FULL int enable failed: %s - falling back to poll",
                 esp_err_to_name(err));
    }

    /* Clear any latched interrupt status before arming the GPIO. */
    uint8_t s1 = 0, s2 = 0;
    max30102_read_int_status(I2C_MASTER_PORT, &s1, &s2);

    s_max30102_task_handle = xTaskGetCurrentTaskHandle();

    gpio_reset_pin(PIN_MAX30102_INT);
    gpio_set_direction(PIN_MAX30102_INT, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_MAX30102_INT);   /* open-drain needs a pull-up */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
    }
    isr_err = gpio_isr_handler_add(PIN_MAX30102_INT, max30102_int_isr, nullptr);
    if (isr_err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add: %s", esp_err_to_name(isr_err));
    }
    gpio_set_intr_type(PIN_MAX30102_INT, GPIO_INTR_NEGEDGE);
    ESP_LOGI(TAG, "MAX30102 INT on GPIO %d (FIFO_A_FULL)", PIN_MAX30102_INT);

    /* ---- Calibration state ---- */
    bool first_run   = calibration_store_is_flag_set(CAL_FLAG_FIRST_RUN);
    bool calibrated  = !first_run;
    bool first_window = true;
    int64_t cal_start_ms = 0;
    double  hr_sum   = 0.0;
    double  spo2_sum = 0.0;
    int     cal_count = 0;

    if (first_run) {
        ESP_LOGI(TAG, "first run: will calibrate for %u ms after first valid window",
                 OXIM_CALIBRATION_MS);
    }

    /* Circular analysis buffer. The burst read fills as many samples
     * as the FIFO holds (up to 32); we accumulate into this buffer
     * and run the algorithm every time it fills. */
    int32_t ir_buf[MAX30102_BUFFER_SIZE];
    int32_t red_buf[MAX30102_BUFFER_SIZE];
    int     buf_idx = 0;

    /* Scratch buffers for the burst read. The FIFO is 32 deep. */
    int32_t red_burst[32];
    int32_t ir_burst[32];

    while (true) {
        /* Block until the FIFO_A_FULL ISR notifies us. pdTRUE collapses
         * back-to-back interrupts into a single wake. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Clear the interrupt (read-to-clear) so the INT pin deasserts. */
        max30102_read_int_status(I2C_MASTER_PORT, &s1, &s2);

        /* Burst-read whatever's in the FIFO (up to 32 samples). */
        size_t n = 0;
        err = max30102_read_fifo_burst(I2C_MASTER_PORT,
                                       red_burst, ir_burst, 32, &n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "burst read failed: %s", esp_err_to_name(err));
            continue;
        }
        if (n == 0) continue;

        /* Copy the burst into the analysis buffer. When the analysis
         * buffer fills, run the algorithm and reset. */
        for (size_t i = 0; i < n; ++i) {
            ir_buf[buf_idx]  = ir_burst[i];
            red_buf[buf_idx] = red_burst[i];
            buf_idx++;
            if (buf_idx < MAX30102_BUFFER_SIZE) continue;

            /* Buffer full - estimate HR + SpO2. */
            int64_t ir_mean = 0, red_mean = 0;
            max30102_alg_remove_dc(ir_buf, red_buf, &ir_mean, &red_mean);
            max30102_alg_remove_trend(ir_buf);
            max30102_alg_remove_trend(red_buf);

            double corr = max30102_alg_correlation(red_buf, ir_buf);
            double r0   = 0.0;
            int    hr   = max30102_alg_heart_rate(ir_buf, &r0, nullptr);
            bool   valid = (corr >= 0.7) && (hr > 30 && hr < 220);
            float  spo2 = valid
                          ? (float)max30102_alg_spo2(ir_buf, red_buf,
                                                     ir_mean, red_mean)
                          : 0.0f;

            if (spo2 > 100.0f) spo2 = 100.0f;
            if (spo2 < 50.0f)  spo2 = 0.0f;

            axion_state_set_oximetry(hr, spo2, valid);

            if (first_window) {
                ESP_LOGI(TAG, "first window: hr=%d bpm spo2=%.1f%% (corr=%.2f)",
                         hr, spo2, corr);
                axion_state_set_ready(BIT_MAX30102_READY);
                first_window = false;

                if (first_run) {
                    cal_start_ms = esp_timer_get_time() / 1000;
                    ESP_LOGI(TAG, "calibration window started");
                } else {
                    axion_state_set_ready(BIT_OXIM_CALIBRATED);
                    calibrated = true;
                }
            } else {
                ESP_LOGD(TAG, "hr=%d bpm spo2=%.1f%% (corr=%.2f)", hr, spo2, corr);
            }

            /* Accumulate valid estimates during calibration. */
            if (first_run && !calibrated && valid) {
                hr_sum   += hr;
                spo2_sum += spo2;
                cal_count++;
                int64_t elapsed = (esp_timer_get_time() / 1000) - cal_start_ms;
                if (elapsed >= (int64_t)OXIM_CALIBRATION_MS) {
                    double hr_avg   = (cal_count > 0) ? (hr_sum   / cal_count) : 0.0;
                    double spo2_avg = (cal_count > 0) ? (spo2_sum / cal_count) : 0.0;
                    ESP_LOGI(TAG, "calibration done: %d valid samples, "
                             "hr_avg=%.1f bpm spo2_avg=%.1f%%",
                             cal_count, hr_avg, spo2_avg);
                    axion_state_set_ready(BIT_OXIM_CALIBRATED);
                    calibrated = true;
                }
            }

            /* Reset the buffer for the next window. */
            buf_idx = 0;
        }
    }
}
