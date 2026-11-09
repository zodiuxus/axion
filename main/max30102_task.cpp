/**
 * max30102_task.cpp - Oximetry sampling + estimation task.
 *
 * FIFO_A_FULL drives burst reads (250 ms poll as fallback); each 128-sample
 * window yields one HR/SpO2 estimate. No calibration phase.
 */

#include "max30102_task.h"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "max30102.h"
#include "max30102_algorithm.h"
#include "I2Cdev.h"

#include "config.h"
#include "state.h"

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
    /* Wait for the shared bus, not for the MPU: making oximetry depend on an
     * unrelated sensor meant a dead MPU also killed this task. Bounded, so a dead bus does not leave the task blocked. */
    if (!axion_state_wait_all_timeout(BIT_I2C_READY, I2C_READY_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "I2C bus never came up - oximetry unavailable");
        axion_state_set_ready(BIT_OXIM_CALIBRATED);
        vTaskDelete(nullptr);
        return;
    }

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

    /* FIFO_A_FULL int: open-drain active-low pin, falling edge,
     * read-to-clear. On failure the 250 ms poll fallback keeps
     * sampling. */
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

    /* Set once the first analysis window has completed, whether or not it was
     * a valid estimate. There is no calibration phase to wait for: see the
     * first-window block below. */
    bool first_window = true;

    /* Accumulate burst reads; run the algorithm when the buffer fills. */
    int32_t ir_buf[MAX30102_BUFFER_SIZE];
    int32_t red_buf[MAX30102_BUFFER_SIZE];
    int     buf_idx = 0;

    /* Scratch buffers for the burst read. The FIFO is 32 deep. */
    int32_t red_burst[32];
    int32_t ir_burst[32];
    /* Packed sample bytes; caller-owned so the driver allocates nothing
     *. */
    uint8_t raw_burst[32 * 6];

    /* Overflow bookkeeping: the 128-sample window must be a
     * contiguous 40 ms grid, so any lost sample invalidates it. */
    uint8_t  last_ovf       = 0;
    uint32_t overflow_events = 0;

    /* ~10 s of silent polls triggers the INT diagnostics below. */
    int silent_polls = 0;

    /* IR envelope, for the perf-index sanity gate. */
    int32_t ir_min = 0, ir_max = 0;

    while (true) {
        /* Block for the ISR notification; the 250 ms timeout is the
         * poll fallback. */
        BaseType_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

        /* Clear the interrupt (read-to-clear) so the INT pin deasserts. */
        max30102_read_int_status(max_dev, &s1, &s2);

        /* Lost samples break the assumed 40 ms grid: restart the window. The
         * previous-value comparison handles both counter semantics. */
        uint8_t ovf = 0;
        if (max30102_read_reg(max_dev, REG_OVF_COUNTER, &ovf, 1) == ESP_OK) {
            if (ovf != 0 && ovf != last_ovf) {
                ++overflow_events;
                if (overflow_events == 1 || (overflow_events % 10u) == 0u) {
                    ESP_LOGW(TAG, "FIFO overflow: %u sample(s) lost "
                                  "(event %lu) - restarting analysis window",
                             (unsigned)ovf, (unsigned long)overflow_events);
                }
                buf_idx = 0;
            }
            last_ovf = ovf;
        }

        /* Burst-read whatever's in the FIFO (up to 32 samples). */
        size_t n = 0;
        err = max30102_read_fifo_burst(max_dev, red_burst, ir_burst, 32,
                                       raw_burst, sizeof(raw_burst), &n);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "burst read failed: %s", esp_err_to_name(err));
            continue;
        }
        if (n == 0) {
            /* Nothing to drain + not notified: throttled INT
             * diagnostics (FIFO has data = INT wire; empty = not
             * sampling). */
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

        /* Copy the burst into the analysis buffer. */
        for (size_t i = 0; i < n; ++i) {
            int32_t ir_s  = ir_burst[i];
            int32_t red_s = red_burst[i];
            if (buf_idx == 0) ir_min = ir_max = ir_s;
            if (ir_s < ir_min) ir_min = ir_s;
            if (ir_s > ir_max) ir_max = ir_s;
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
             * corr alone can't reject motion (it moves both channels
             * together). Two extra checks:
             *   1. IR DC band: coupled finger ~10k-150k counts; < 8k =
             *      no finger; near full scale = saturation.
             *   2. Perfusion index (p2p/mean): real pulse ~0.5-15 %,
             *      motion artifact tens of percent, flat line ~0 %. */
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

            /* For a live finger anything under 70 % is estimate noise,
             * not real hypoxia - report invalid instead. */
            if (spo2 > 100.0f) spo2 = 100.0f;
            if (spo2 < 70.0f)  spo2 = 0.0f;

            axion_state_set_oximetry(hr, spo2, valid);

            if (first_window) {
                ESP_LOGI(TAG, "first window: hr=%d bpm spo2=%.1f%% (corr=%.2f)",
                         hr, spo2, corr);
                axion_state_set_ready(BIT_MAX30102_READY);
                /* Nothing to calibrate: the SpO2 curve is a fixed,
                 * uncalibrated approximation (README) and the estimator needs
                 * no per-device offset. The old first-run phase averaged two
                 * minutes of estimates and then discarded them, which only
                 * delayed this bit and consumed half of the status LED's
                 * budget. */
                axion_state_set_ready(BIT_OXIM_CALIBRATED);
                first_window = false;
            } else {
                ESP_LOGI(TAG, "win: hr=%d bpm spo2=%.1f%% corr=%.2f "
                              "pi=%.3f valid=%d",
                         hr, spo2, corr, perf_ix, valid ? 1 : 0);
            }

            /* Reset the buffer for the next window. */
            buf_idx = 0;
        }
    }
}
