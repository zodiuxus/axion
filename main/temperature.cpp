/**
 * temperature.cpp — DS18B20 1-Wire body-temperature task.
 *
 * On first run (CAL_FLAG_FIRST_RUN set in NVS), the task enters a
 * 2-minute calibration window after the first successful reading.
 * During calibration, readings are accumulated and the average is
 * logged (and will be stored as a baseline offset in NVS when the
 * offset logic is added — "we'll add those options in detail later on").
 * BIT_TEMP_CALIBRATED is set at the end of calibration.
 *
 * On non-first-run boots, BIT_TEMP_CALIBRATED is set immediately after
 * the first successful reading — no calibration window is needed.
 *
 * The CAL_FLAG_FIRST_RUN flag is cleared by the status LED task once
 * ALL sensors have set their BIT_*_CALIBRATED bits, so a reboot during
 * temperature calibration will re-run it.
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
        /* Even on failure, set the calibrated bit so the status LED
         * can proceed to COMPLETE instead of blinking forever. */
        axion_state_set_ready(BIT_TEMP_CALIBRATED);
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

    /* ---- Calibration state ---- */
    bool first_run = calibration_store_is_flag_set(CAL_FLAG_FIRST_RUN);
    bool calibrated = !first_run;   /* non-first-run: already calibrated */
    bool first_reading = true;
    int64_t cal_start_ms = 0;
    float    cal_sum   = 0.0f;
    int      cal_count = 0;

    if (first_run) {
        ESP_LOGI(TAG, "first run: will calibrate for %u ms after first reading",
                 TEMP_CALIBRATION_MS);
    } else {
        /* Load the previously-stored baseline from NVS so the monitor
         * can compute hypo/hyper thresholds from it immediately. Falls
         * back to TEMP_BASELINE_DEFAULT if NVS has nothing (shouldn't
         * happen on a non-first-run boot, but guard anyway). */
        float stored = TEMP_BASELINE_DEFAULT;
        if (calibration_store_get_temp_baseline(&stored) == ESP_OK) {
            ESP_LOGI(TAG, "loaded temp baseline from NVS: %.2f C", stored);
        } else {
            ESP_LOGW(TAG, "no temp baseline in NVS; using default %.2f C",
                     (double)TEMP_BASELINE_DEFAULT);
        }
        axion_state_set_temp_baseline(stored);
    }

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        ds18b20_convert_all(owb);
        ds18b20_wait_for_conversion(devices[0]);

        float reading = 0.0f;
        DS18B20_ERROR err = ds18b20_read_temp(devices[0], &reading);

        if (err == DS18B20_OK) {
            axion_state_set_temp(reading);

            if (first_reading) {
                ESP_LOGI(TAG, "first reading: %.2f C", reading);
                axion_state_set_ready(BIT_TEMP_READY);
                first_reading = false;

                if (first_run) {
                    cal_start_ms = esp_timer_get_time() / 1000;
                    ESP_LOGI(TAG, "calibration window started");
                } else {
                    /* Non-first-run: calibration was done previously. */
                    axion_state_set_ready(BIT_TEMP_CALIBRATED);
                    calibrated = true;
                }
            }

            /* Accumulate during the calibration window. */
            if (first_run && !calibrated) {
                cal_sum += reading;
                cal_count++;
                int64_t elapsed = (esp_timer_get_time() / 1000) - cal_start_ms;
                if (elapsed >= (int64_t)TEMP_CALIBRATION_MS) {
                    float avg = (cal_count > 0) ? (cal_sum / cal_count) : 0.0f;
                    ESP_LOGI(TAG, "calibration done: %d samples, avg=%.2f C",
                             cal_count, avg);
                    /* Persist the baseline to NVS so future boots use it
                     * without re-calibrating, and push it to shared state
                     * so the monitor can compute thresholds immediately. */
                    if (cal_count > 0) {
                        calibration_store_set_temp_baseline(avg);
                        axion_state_set_temp_baseline(avg);
                    }
                    axion_state_set_ready(BIT_TEMP_CALIBRATED);
                    calibrated = true;
                }
            }
        } else {
            ESP_LOGW(TAG, "DS18B20 read error on device 0: %d", (int)err);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    /* Unreachable — task runs forever. */
}
