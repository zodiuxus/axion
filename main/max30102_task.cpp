/**
 * max30102_task.cpp - Oximetry sampling + estimation task.
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
 */
#include "max30102_task.h"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "max30102.h"
#include "max30102_algorithm.h"

#include "config.h"
#include "state.h"
#include "calibration_store.h"

static const char *TAG = "axion.max30102";

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

    int32_t ir_buf[MAX30102_BUFFER_SIZE];
    int32_t red_buf[MAX30102_BUFFER_SIZE];

    while (true) {
        /* Fill a full buffer of samples. */
        for (int i = 0; i < MAX30102_BUFFER_SIZE; ++i) {
            int32_t red = 0, ir = 0;
            err = max30102_read_fifo(I2C_MASTER_PORT, &red, &ir);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "fifo read failed: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(MAX30102_SAMPLE_MS));
                continue;
            }
            ir_buf[i]  = ir;
            red_buf[i] = red;
            vTaskDelay(pdMS_TO_TICKS(MAX30102_SAMPLE_MS));
        }

        /* Estimate HR + SpO2. */
        int64_t ir_mean = 0, red_mean = 0;
        max30102_alg_remove_dc(ir_buf, red_buf, &ir_mean, &red_mean);
        max30102_alg_remove_trend(ir_buf);
        max30102_alg_remove_trend(red_buf);

        double corr = max30102_alg_correlation(red_buf, ir_buf);
        double r0   = 0.0;
        int    hr   = max30102_alg_heart_rate(ir_buf, &r0, nullptr);
        bool   valid = (corr >= 0.7) && (hr > 30 && hr < 220);
        float  spo2 = valid
                      ? (float)max30102_alg_spo2(ir_buf, red_buf, ir_mean, red_mean)
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
        /* If calibration is still in progress but the current window
         * was invalid (finger off), we simply skip accumulation - the
         * calibration window will time out eventually and the averages
         * will be computed from whatever valid samples we collected. */
    }
}
