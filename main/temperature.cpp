/**
 * temperature.cpp - DS18B20 1-Wire body-temperature task.
 *
 * TEMP_BASELINE_PLAUSIBLE_MIN_C only decides whether there is skin in contact
 * (arming bit + calibration samples); the alert band is baseline +/- delta in
 * alert_policy.cpp.
 *
 * An absent or silent probe fails open after TEMP_FIRST_READ_TIMEOUT_MS so
 * fall/collision alerting survives.
 */
#include "temperature.h"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "owb.h"
#include "owb_gpio.h"
#include "ds18b20.h"

#include "config.h"
#include "state.h"
#include "calibration_store.h"

static const char *TAG = "axion.temp";

#define MAX_DEVICES          8
#define SAMPLE_PERIOD_MS     1000U
#define DS18B20_RESOLUTION   DS18B20_RESOLUTION_12_BIT

/* First-run calibration target: TEMP_CALIBRATION_MS' worth of on-body
 * samples at the sampling rate. Keep in sync with SAMPLE_PERIOD_MS. */
#define TEMP_CAL_SAMPLES \
    ((int)(TEMP_CALIBRATION_MS / SAMPLE_PERIOD_MS))

void temperature_task(void * /*arg*/)
{
    OneWireBus *owb = nullptr;
    owb_gpio_driver_info gpio_driver_info;
    owb = owb_gpio_initialize(&gpio_driver_info, PIN_DS18B20);
    owb_use_crc(owb, true);
    ESP_LOGI(TAG, "1-Wire bus on GPIO %d", PIN_DS18B20);

    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {};
    bool found = false;
    owb_search_first(owb, &search_state, &found);

    while (found) {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "  found device %d: %s", num_devices, rom_code_s);
        if (num_devices < MAX_DEVICES) {
            device_rom_codes[num_devices] = search_state.rom_code;
            ++num_devices;
        }
        owb_search_next(owb, &search_state, &found);
    }
    ESP_LOGI(TAG, "Total DS18B20 devices: %d", num_devices);

    if (num_devices == 0) {
        ESP_LOGE(TAG, "No DS18B20 devices detected; marking calibrated and exiting");
        /* Fail-open all three temp bits: the LED may commit the calibration
         * (there is nothing to calibrate), the monitor may arm (a missing
         * sensor must not kill fall/collision alerting), and the alerting
         * channels stay inert because no reading is ever published. */
        axion_state_set_ready(BIT_TEMP_SENSOR_ABSENT);
        axion_state_set_ready(BIT_TEMP_CALIBRATED);
        axion_state_set_ready(BIT_TEMP_MIN_REACHED);
        owb_uninitialize(owb);
        vTaskDelete(nullptr);
        return;
    }

    DS18B20_Info *devices[MAX_DEVICES] = {};
    for (int i = 0; i < num_devices; ++i) {
        devices[i] = ds18b20_malloc();
        if (num_devices == 1) {
            ds18b20_init_solo(devices[i], owb);
        } else {
            ds18b20_init(devices[i], owb, device_rom_codes[i]);
        }
        ds18b20_use_crc(devices[i], true);
        ds18b20_set_resolution(devices[i], DS18B20_RESOLUTION);
    }

    bool parasitic_power = false;
    ds18b20_check_for_parasite_power(owb, &parasitic_power);
    if (parasitic_power) ESP_LOGW(TAG, "Parasitic-powered devices detected");
    owb_use_parasitic_power(owb, parasitic_power);

#ifdef CONFIG_ENABLE_STRONG_PULLUP_GPIO
    owb_use_strong_pullup_gpio(owb, CONFIG_STRONG_PULLUP_GPIO);
#endif

    /* Calibration is complete when a plausible baseline is stored, not when a
     * timer expires; a stored baseline is never recalculated. */
    bool first_run = calibration_store_is_flag_set(CAL_FLAG_FIRST_RUN);

    float stored_baseline      = TEMP_BASELINE_DEFAULT;
    bool  have_stored_baseline = (calibration_store_get_temp_baseline(&stored_baseline) == ESP_OK);
    bool  baseline_published   = false;

    /* Self-heal the state older firmware could leave behind: FIRST_RUN cleared
     * without a stored baseline, which silently disabled temperature alerting.
     * Re-provision instead of staying degraded forever. */
    if (!first_run && !have_stored_baseline) {
        ESP_LOGW(TAG, "FIRST_RUN clear but no baseline stored - re-provisioning "
                      "temperature calibration");
        calibration_store_set_flag(CAL_FLAG_FIRST_RUN);
        first_run = true;
    }

    bool calibrating = first_run && !have_stored_baseline;

    if (calibrating) {
        ESP_LOGI(TAG, "calibrating: %u ms of on-body samples",
                 TEMP_CALIBRATION_MS);
    } else {
        /* No temperature calibration window this boot. */
        axion_state_set_ready(BIT_TEMP_CALIBRATED);
        ESP_LOGI(TAG, "temp calibration already stored: %.2f C (published on "
                      "the first plausible contact reading)",
                 (double)stored_baseline);
    }

    float   cal_sum   = 0.0f;
    int     cal_count = 0;
    bool    min_reached   = false;
    bool    first_reading = true;
    int64_t last_good_read_ms = esp_timer_get_time() / 1000;

    /* Fail-open watchdog state (a silent probe must not block the
     * monitor's arming gate forever). */
    bool read_fail_open = false;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        ds18b20_convert_all(owb);
        ds18b20_wait_for_conversion(devices[0]);

        float reading = 0.0f;
        DS18B20_ERROR err = ds18b20_read_temp(devices[0], &reading);

        if (err == DS18B20_OK) {
            last_good_read_ms = esp_timer_get_time() / 1000;
            axion_state_set_temp(reading);

            if (first_reading) {
                ESP_LOGI(TAG, "first reading: %.2f C%s",
                         (double)reading,
                         (reading < TEMP_BASELINE_PLAUSIBLE_MIN_C)
                             ? " - below plausible skin temperature, waiting for contact"
                             : "");
                axion_state_set_ready(BIT_TEMP_READY);
                first_reading = false;
            }

            /* Contact gate: arming bit + calibration samples only, not
             * alerting. */
            if (reading >= TEMP_BASELINE_PLAUSIBLE_MIN_C) {
                if (!min_reached) {
                    min_reached = true;
                    axion_state_set_ready(BIT_TEMP_MIN_REACHED);
                    ESP_LOGI(TAG, "reading at/above %.2f C - probe in contact, "
                                  "monitor may arm%s",
                             (double)TEMP_BASELINE_PLAUSIBLE_MIN_C,
                             calibrating ? "; calibration window started" : "");
                }

                if (calibrating) {
                    /* On-body samples only: going off-body pauses the window. */
                    cal_sum += reading;
                    cal_count++;
                    if (cal_count >= TEMP_CAL_SAMPLES) {
                        float avg = cal_sum / (float)cal_count;
                        ESP_LOGI(TAG, "calibration done: %d samples, "
                                      "avg=%.2f C", cal_count, (double)avg);
                        /* Persist + publish only plausible averages; a
                         * sub-minimum avg means the device came off mid-cal.
                         * The status LED commits FIRST_RUN only if a baseline
                         * is actually stored, so an unfinished or implausible
                         * calibration retries on the next boot. */
                        if (avg >= TEMP_BASELINE_PLAUSIBLE_MIN_C) {
                            calibration_store_set_temp_baseline(avg);
                            axion_state_set_temp_baseline(avg);
                        } else {
                            ESP_LOGE(TAG, "calibration avg %.2f C below "
                                          "plausible skin temperature - "
                                          "not stored; alerting stays disabled "
                                          "and the next boot retries",
                                     (double)avg);
                        }
                        axion_state_set_ready(BIT_TEMP_CALIBRATED);
                        calibrating = false;   /* window finished for this boot */
                    }
                } else if (have_stored_baseline && !baseline_published) {
                    /* First on-body reading arms temp alerting with the stored
                     * baseline (a previous boot calibrated it). */
                    baseline_published = true;
                    axion_state_set_temp_baseline(stored_baseline);
                    ESP_LOGI(TAG, "probe on skin (%.2f C) - temp alerting "
                                  "armed (baseline %.2f C)",
                             (double)reading, (double)stored_baseline);
                }
            }
        } else {
            ESP_LOGW(TAG, "DS18B20 read error on device 0: %d", (int)err);
        }

        /* Fail-open: probe absent or gone silent before reaching the
         * minimum - release the monitor gate so fall/collision
         * alerting survive a dead sensor. A probe that keeps reading
         * (ambient included) never triggers this. */
        if (!min_reached && !read_fail_open &&
            (esp_timer_get_time() / 1000) - last_good_read_ms >=
                (int64_t)TEMP_FIRST_READ_TIMEOUT_MS) {
            read_fail_open = true;
            ESP_LOGE(TAG, "no DS18B20 reading for %u ms - releasing monitor "
                          "gate, temp alerting DISABLED",
                     (unsigned)TEMP_FIRST_READ_TIMEOUT_MS);
            axion_state_set_ready(BIT_TEMP_MIN_REACHED);
            axion_state_set_ready(BIT_TEMP_CALIBRATED);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    /* Unreachable - task runs forever. */
}
