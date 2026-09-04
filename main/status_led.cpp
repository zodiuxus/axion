/**
 * status_led.cpp - see status_led.h for design notes.
 *
 * The task reads `CAL_FLAG_FIRST_RUN` from NVS to decide whether to
 * enter the CALIBRATING blink phase. It waits for BIT_TEMP_CALIBRATED
 * and BIT_OXIM_CALIBRATED to be set before transitioning to the solid
 * "complete" glow.
 */
#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "state.h"
#include "calibration_store.h"

static const char *TAG = "axion.led";

static inline void led_on(void)  { gpio_set_level(PIN_GREEN_LED, 1); }
static inline void led_off(void) { gpio_set_level(PIN_GREEN_LED, 0); }

/* Blink for up to `duration_ms`, checking a condition each cycle.
 * Returns true if the condition became true, false on timeout. */
static bool blink_until_condition_or_timeout(bool (*cond)(void),
                                             int on_ms, int off_ms,
                                             int64_t deadline_ms)
{
    while (!cond()) {
        if (deadline_ms > 0 && (esp_timer_get_time() / 1000) >= deadline_ms) {
            return false;
        }
        led_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
    return true;
}

/* Condition callbacks. */
static bool all_calibrated(void) {
    return (xEventGroupGetBits(g_sensors_ready) &
            (BIT_TEMP_CALIBRATED | BIT_OXIM_CALIBRATED))
           == (BIT_TEMP_CALIBRATED | BIT_OXIM_CALIBRATED);
}

static bool any_sensor_ready(void) {
    return (xEventGroupGetBits(g_sensors_ready) &
            (BIT_TEMP_READY | BIT_MAX30102_READY)) != 0;
}

void status_led_task(void * /*arg*/)
{
    gpio_reset_pin(PIN_GREEN_LED);
    gpio_set_direction(PIN_GREEN_LED, GPIO_MODE_OUTPUT);
    led_off();

    bool first_run = calibration_store_is_flag_set(CAL_FLAG_FIRST_RUN);
    int64_t boot_ms    = esp_timer_get_time() / 1000;
    int64_t deadline   = boot_ms + LED_CALIBRATION_TIMEOUT_MS;

    ESP_LOGI(TAG, "status LED starting; first_run=%d", first_run ? 1 : 0);

    if (first_run) {
        /* ---- BOOTING: fast blink until at least one sensor produces data ---- */
        blink_until_condition_or_timeout(any_sensor_ready,
                                         LED_BOOT_ON_MS, LED_BOOT_OFF_MS,
                                         deadline);

        /* ---- CALIBRATING: slow blink until all sensors calibrated ---- */
        bool ok = blink_until_condition_or_timeout(all_calibrated,
                                                   LED_CALIB_ON_MS, LED_CALIB_OFF_MS,
                                                   deadline);
        if (!ok) {
            ESP_LOGW(TAG, "calibration timeout; proceeding to complete anyway");
        }
    } else {
        /* ---- BOOTING: fast blink until all sensors calibrated (no cal phase) ---- */
        bool ok = blink_until_condition_or_timeout(all_calibrated,
                                                   LED_BOOT_ON_MS, LED_BOOT_OFF_MS,
                                                   deadline);
        if (!ok) {
            ESP_LOGW(TAG, "startup timeout; proceeding to complete anyway");
        }
    }

    /* ---- COMPLETE: solid 2 s glow, then off ---- */
    /* Clear the first-run flag now that ALL sensors have set their
     * CALIBRATED bits (or the timeout expired). Doing this here - rather
     * than in individual sensor tasks - ensures that a reboot during
     * any sensor's calibration window will re-run ALL calibrations. */
    if (first_run) {
        calibration_store_clear_flag(CAL_FLAG_FIRST_RUN);
        ESP_LOGI(TAG, "first-run complete; flag cleared in NVS");
    }

    ESP_LOGI(TAG, "calibration complete; solid glow");
    led_on();
    vTaskDelay(pdMS_TO_TICKS(LED_COMPLETE_MS));
    led_off();

    ESP_LOGI(TAG, "status LED done; deleting task");
    vTaskDelete(nullptr);
}
