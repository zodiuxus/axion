/**
 * temperature.cpp - DS18B20 1-Wire body-temperature task.
 *
 * The current reading is the only signal: >= TEMP_BASELINE_DEFAULT
 * means the probe is on skin - arm the monitor (BIT_TEMP_MIN_REACHED),
 * run the first-run calibration window, publish the stored baseline on
 * later boots. Below it the probe is off the body and the monitor's own
 * per-poll minimum check keeps the temp channel inert. No settle /
 * flatness gating - the arm minimum makes it redundant.
 *
 * First-run calibration averages TEMP_CALIBRATION_MS' worth of ON-BODY
 * samples only, so off-body time merely pauses it. Only hardware faults
 * fail open: an absent or silent probe releases the monitor gate after
 * TEMP_FIRST_READ_TIMEOUT_MS so fall/collision alerting survive.
 *
 * CAL_FLAG_FIRST_RUN is cleared by the status LED once ALL sensors set
 * their BIT_*_CALIBRATED bits (a mid-calibration reboot re-runs it).
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
        /* Fail-open both temp bits: LED proceeds, monitor may arm (a
         * missing sensor must not kill fall/collision alerting). */
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

    /* ---- Calibration state ---- */
    bool first_run = calibration_store_is_flag_set(CAL_FLAG_FIRST_RUN);

    float stored_baseline      = TEMP_BASELINE_DEFAULT;
    bool  have_stored_baseline = false;
    bool  baseline_published   = false;

    if (first_run) {
        ESP_LOGI(TAG, "first run: will calibrate from %u ms of on-body "
                      "samples", TEMP_CALIBRATION_MS);
    } else {
        /* Warm boot: the calibration phase is already done; the stored
         * baseline is published once a reading says the probe is on
         * skin. */
        axion_state_set_ready(BIT_TEMP_CALIBRATED);
        if (calibration_store_get_temp_baseline(&stored_baseline) == ESP_OK) {
            have_stored_baseline = true;
            ESP_LOGI(TAG, "loaded temp baseline from NVS: %.2f C (published "
                          "once a reading reaches %.2f C)",
                     (double)stored_baseline,
                     (double)TEMP_BASELINE_DEFAULT);
        } else {
            ESP_LOGW(TAG, "no temp baseline in NVS - temperature alerting "
                          "DISABLED until calibration");
        }
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
                         (reading < TEMP_BASELINE_DEFAULT)
                             ? " - below minimum, waiting for skin contact"
                             : "");
                axion_state_set_ready(BIT_TEMP_READY);
                first_reading = false;
            }

            /* ---- On-body check: the current reading is the signal. -- */
            if (reading >= TEMP_BASELINE_DEFAULT) {
                if (!min_reached) {
                    min_reached = true;
                    axion_state_set_ready(BIT_TEMP_MIN_REACHED);
                    ESP_LOGI(TAG, "reading at/above %.2f C - probe on skin, "
                                  "monitor may arm%s",
                             (double)TEMP_BASELINE_DEFAULT,
                             first_run ? "; calibration window started" : "");
                }

                if (first_run) {
                    /* Count on-body samples only: taking the device
                     * off mid-window merely pauses calibration. */
                    cal_sum += reading;
                    cal_count++;
                    if (cal_count >= TEMP_CAL_SAMPLES) {
                        float avg = cal_sum / (float)cal_count;
                        ESP_LOGI(TAG, "calibration done: %d samples, "
                                      "avg=%.2f C", cal_count, (double)avg);
                        /* Persist + publish only plausible averages; a
                         * sub-minimum avg means the device came off mid-cal. */
                        if (avg >= TEMP_BASELINE_PLAUSIBLE_MIN_C) {
                            calibration_store_set_temp_baseline(avg);
                            axion_state_set_temp_baseline(avg);
                        } else {
                            ESP_LOGW(TAG, "calibration avg %.2f C below "
                                          "plausible skin temperature - "
                                          "not stored; temp alerting stays "
                                          "disabled",
                                     (double)avg);
                        }
                        axion_state_set_ready(BIT_TEMP_CALIBRATED);
                        first_run = false;   /* cal phase done */
                    }
                } else if (have_stored_baseline && !baseline_published) {
                    /* Warm boot: first on-body reading arms temp
                     * alerting with the stored baseline. */
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
