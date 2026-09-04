/**
 * max30102_task.cpp - Oximetry sampling + estimation task (interrupt-driven
 * with a poll fallback).
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
 * ---- Interrupt-driven FIFO drain (with poll fallback) ------------------
 * The primary wake source is the FIFO_A_FULL interrupt on
 * PIN_MAX30102_INT (fires when 17 samples are unread - every ~680 ms at
 * the effective 25 sps of 200 sps / 8x averaging), then the task
 * burst-reads all available samples in a single I2C transaction.
 *
 * The wait has a 250 ms timeout. A healthy interrupt path always beats
 * the timeout; if the edge is ever lost - INT wire off or misplaced,
 * missed edge, stuck line - the timeout path still drains the FIFO 4x
 * per second, which the 32-deep FIFO absorbs without overflow (it fills
 * in ~1.28 s at 25 sps). So a dead INT line costs sampling latency, not data.
 * When the timeout path fires with an empty FIFO, a throttled
 * diagnostic line distinguishes "sensor sampling but INT never arrives"
 * (check the INT wire) from "sensor not sampling at all" (check mode
 * config / LED currents).
 *
 * ---- Optical bring-up diagnostics --------------------------------------
 * "Is the sensor even working?" is answerable from the log alone:
 *   - init logs the DIE TEMPERATURE (proves silicon is converting,
 *     not just ACKing on I2C),
 *   - the first analysis window (and every 10th invalid window) logs
 *     the raw IR/RED envelope [min..max, mean]. Interpretation:
 *       flat & near constant          -> no light / no coupling
 *                                        (LEDs off, RD/IRD floating?)
 *       low values, flat              -> LEDs firing, no finger
 *       level jumps when finger lands -> healthy optical path
 *       max at 262143                 -> detector saturated
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
#include "I2Cdev.h"

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
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    /* Wait for the I2C bus (installed by i2c_bus_setup, signalled via
     * BIT_MPU_READY since the MPU setup task runs after i2c_bus_setup). */
    axion_state_wait_all(BIT_MPU_READY);

    /* Handle for our 0x57 device on the shared bus (cached by I2Cdev). */
    i2c_master_dev_handle_t max_dev = I2Cdev::deviceHandle(MAX30102_I2C_ADDR);
    if (max_dev == nullptr) {
        ESP_LOGE(TAG, "no I2C device handle for MAX30102 (0x%02x)",
                 MAX30102_I2C_ADDR);
        axion_state_set_ready(BIT_OXIM_CALIBRATED);
        vTaskDelete(nullptr);
        return;
    }

    max30102_config_t cfg = max30102_default_config();
    esp_err_t err = max30102_init(max_dev, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "max30102_init failed: %s - marking calibrated and exiting",
                 esp_err_to_name(err));
        /* Set the bit so the status LED can proceed. */
        axion_state_set_ready(BIT_OXIM_CALIBRATED);
        vTaskDelete(nullptr);
        return;
    }
    max30102_alg_init();
    ESP_LOGI(TAG, "MAX30102 initialized (0x%02x on shared I2C bus)",
             MAX30102_I2C_ADDR);

    /* Die temperature: proves the silicon is alive beyond a bare I2C
     * ACK - the on-die sensor requires working internal conversion to
     * produce this. Should read close to ambient (a degree or two above
     * once the LEDs have been running). */
    float die_temp = 0.0f;
    if (max30102_read_temp(max_dev, &die_temp) == ESP_OK) {
        ESP_LOGI(TAG, "die temp: %.1f C (expected near ambient)",
                 (double)die_temp);
    } else {
        ESP_LOGW(TAG, "die temp read failed - chip ACKs but conversion "
                      "path suspect");
    }

    /* Enable the FIFO_A_FULL interrupt and wire up the GPIO ISR. The
     * MAX30102 INT pin is open-drain, active-low. We trigger on the
     * falling edge (assertion). Reading the INT_STATUS register clears
     * the interrupt and deasserts the pin. If this write fails (or the
     * INT wire is missing entirely), the 250 ms wait timeout in the
     * main loop below degrades to a poll and sampling continues. */
    err = max30102_enable_fifo_a_full_int(max_dev, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FIFO_A_FULL int enable failed: %s - "
                      "poll fallback will drive sampling",
                 esp_err_to_name(err));
    }

    /* Clear any latched interrupt status before arming the GPIO. */
    uint8_t s1 = 0, s2 = 0;
    max30102_read_int_status(max_dev, &s1, &s2);

    s_max30102_task_handle = xTaskGetCurrentTaskHandle();

    gpio_reset_pin(PIN_MAX30102_INT);
    gpio_set_direction(PIN_MAX30102_INT, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_MAX30102_INT);   /* open-drain needs a pull-up */
    /* The GPIO ISR service is installed once, centrally, in app_main
     * (axion.cpp); we only attach our handler here. */
    esp_err_t isr_err = gpio_isr_handler_add(PIN_MAX30102_INT, max30102_int_isr, nullptr);
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

    /* Consecutive poll-timeout wakeups that produced nothing. 40 in a
     * row (~10 s) triggers the throttled INT diagnostics below. */
    int silent_polls = 0;

    /* Raw-signal envelope stats for the current analysis window - see
     * the bring-up diagnostics inside the window-fill branch. */
    int32_t ir_min = 0, ir_max = 0, ir_sum = 0;
    int32_t red_min = 0, red_max = 0, red_sum = 0;
    int     invalid_windows = 0;

    while (true) {
        /* Block until the FIFO_A_FULL ISR notifies us. pdTRUE collapses
         * back-to-back interrupts into a single wake. The 250 ms timeout
         * is the poll fallback: a healthy INT beats it every time; a
         * dead INT path still gets the FIFO drained 4x per second. */
        BaseType_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

        /* Clear the interrupt (read-to-clear) so the INT pin deasserts. */
        max30102_read_int_status(max_dev, &s1, &s2);

        /* Burst-read whatever's in the FIFO (up to 32 samples). */
        size_t n = 0;
        err = max30102_read_fifo_burst(max_dev,
                                       red_burst, ir_burst, 32, &n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "burst read failed: %s", esp_err_to_name(err));
            continue;
        }
        if (n == 0) {
            /* Nothing to drain. If we were ALSO not notified, the
             * interrupt path is suspect - keep sampling via the poll,
             * but say so (throttled to once per ~10 s) with enough
             * detail to pinpoint the fault:
             *   - FIFO has data  -> sensor samples fine, INT never
             *     arrives: check the INT wire to GPIO 8.
             *   - FIFO empty     -> chip is not sampling at all:
             *     check MODE_CONFIG / LED currents / red LED glow. */
            if (!notified && ++silent_polls >= 40) {
                uint8_t wr = 0, rd = 0, ovf = 0, mode = 0;
                max30102_read_reg(max_dev, REG_FIFO_WR_PTR, &wr, 1);
                max30102_read_reg(max_dev, REG_FIFO_RD_PTR, &rd, 1);
                max30102_read_reg(max_dev, REG_OVF_COUNTER, &ovf, 1);
                max30102_read_reg(max_dev, REG_MODE_CONFIG, &mode, 1);
                if (wr != rd || ovf != 0) {
                    ESP_LOGW(TAG, "no INT for ~10 s but FIFO has data "
                             "(wr=%u rd=%u ovf=%u int_gpio=%d) - "
                             "check the INT wire to GPIO %d",
                             wr, rd, ovf,
                             gpio_get_level(PIN_MAX30102_INT),
                             PIN_MAX30102_INT);
                } else {
                    ESP_LOGW(TAG, "no INT for ~10 s and FIFO empty "
                             "(mode=0x%02x int_gpio=%d) - sensor not "
                             "sampling",
                             mode, gpio_get_level(PIN_MAX30102_INT));
                }
                silent_polls = 0;
            }
            continue;
        }
        silent_polls = 0;

        /* Copy the burst into the analysis buffer. When the analysis
         * buffer fills, run the algorithm and reset. Per-sample raw
         * min/max/sum are tracked for the diagnostics below. */
        for (size_t i = 0; i < n; ++i) {
            int32_t ir_s  = ir_burst[i];
            int32_t red_s = red_burst[i];
            if (buf_idx == 0) {
                ir_min  = ir_max  = ir_s;
                red_min = red_max = red_s;
                ir_sum  = 0;
                red_sum = 0;
            }
            if (ir_s  < ir_min)  ir_min  = ir_s;
            if (ir_s  > ir_max)  ir_max  = ir_s;
            if (red_s < red_min) red_min = red_s;
            if (red_s > red_max) red_max = red_s;
            ir_sum  += ir_s;
            red_sum += red_s;
            ir_buf[buf_idx]  = ir_s;
            red_buf[buf_idx] = red_s;
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

            /* ---- Envelope sanity gates (motion-artifact rejection) ----
             * corr >= 0.7 only proves RED and IR move TOGETHER - but
             * motion artifact moves them together too (one mechanical
             * disturbance modulates both channels), so correlation
             * alone happily passes garbage. The one "115 bpm / 68 %"
             * reading observed during bench rewiring was exactly this:
             * hand/wire motion (~4 Hz periodicity, large correlated
             * deflections) sailed through both gates and produced a
             * confident-looking nonsense window. Two cheap checks on
             * the raw envelope close that hole:
             *   1. IR DC band: a coupled finger reads ~10k-150k counts
             *      at the default LED current; < 8k = no finger or
             *      ambient leakage, ~262143 = detector saturation.
             *   2. Perfusion index: (max-min)/mean for a real pulse is
             *      ~0.5-15 % peak-to-peak; motion artifact runs tens
             *      of percent, a flat line ~0 %. Tunable constants -
             *      loosen if good windows get rejected, tighten if
             *      garbage slips through. */
            double ir_dc   = (double)ir_mean;   /* pre-DC-removal mean */
            double ir_pp   = (double)(ir_max - ir_min);
            double perf_ix = (ir_dc > 0.0) ? (ir_pp / ir_dc) : 0.0;
            bool optically_sane = (ir_dc >= 8000.0)  && (ir_dc < 260000.0) &&
                                  (perf_ix >= 0.005) && (perf_ix <= 0.15);
            valid = valid && optically_sane;

            float  spo2 = valid
                          ? (float)max30102_alg_spo2(ir_buf, red_buf,
                                                     ir_mean, red_mean)
                          : 0.0f;

            /* Physiological plausibility gate. The calibration curve is
             * monotonically decreasing, and this is an uncalibrated
             * hobby-grade estimate: for a live finger anything under
             * 70 % is estimate noise, not real hypoxia - report
             * "invalid" (0) rather than an alarming number. (The old
             * 50 % floor was tuned to the previous INVERTED 49.7*R
             * curve, where it silently zeroed every good window with
             * R < 1.0 - the exact cause of "SpO2 is 0 almost always,
             * 51–53 % when it shows anything".) */
            if (spo2 > 100.0f) spo2 = 100.0f;
            if (spo2 < 70.0f)  spo2 = 0.0f;

            /* ---- Bring-up diagnostics: raw optical envelope ----
             * Flat & constant  -> no light / no coupling (LEDs off?)
             * Low, flat        -> LEDs firing, no finger on sensor
             * Jumps with finger, max-min spread grows -> healthy
             * max == 262143    -> ADC saturated (too much light)
             * Big spread + high corr but PI > 15% -> motion artifact,
             *                             not a pulse (gate rejects) */
            if (!valid) invalid_windows++;
            if (first_window || (!valid && (invalid_windows % 10) == 1)) {
                ESP_LOGI(TAG, "raw ir[%ld..%ld mean %ld] "
                               "red[%ld..%ld mean %ld] pi=%.3f",
                         (long)ir_min, (long)ir_max,
                         (long)(ir_sum / MAX30102_BUFFER_SIZE),
                         (long)red_min, (long)red_max,
                         (long)(red_sum / MAX30102_BUFFER_SIZE),
                         perf_ix);
            }

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
              ESP_LOGI(TAG, "win: hr=%d bpm spo2=%.1f%% corr=%.2f pi=%.3f valid=%d",
                 hr, spo2, corr, perf_ix, valid ? 1 : 0);
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
