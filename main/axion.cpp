/**
 * axion.cpp - application entry point.
 *
 * Wires up the Axion firmware:
 *   1. Initializes shared state + event group.
 *   2. Initializes NVS (for the calibration_store bitfield).
 *   3. Installs the alert-abort button ISR.
 *   4. Spawns one setup task per peripheral (I2C bus, MPU, modem, temp).
 *   5. Spawns the long-running workers (rpy, gnss, oximetry, monitor,
 *      status LED, optional console).
 *
 * Secrets: SIM PIN and alert phone numbers come from `secrets.h`, which
 * is gitignored. If the file is missing the build fails here with a
 * clear message.
 */
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "secrets.h"           /* gitignored; copy from secrets.h.example */
#include "state.h"
#include "calibration_store.h"
#include "alert_button.h"

#include "mpu.h"
#include "modem.h"
#include "temperature.h"
#include "monitor.h"
#include "console.h"
#include "max30102_task.h"
#include "status_led.h"
#include "collision.h"

static const char *TAG = "axion";

/* Sanity check: secrets.h must define a real phone number for slot 1.
 * We no longer require a '+' prefix because the primary contact may be
 * a local emergency number like "112" (no E.164 prefix).
 * If you see this error, copy main/secrets.h.example to main/secrets.h
 * and edit it with the real values for this device. */
static_assert(ALERT_PHONE_1[0] != '\0',
              "ALERT_PHONE_1 must be set in secrets.h (e.g. \"112\")");

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Axion booting - built " __DATE__ " " __TIME__);

    /* ---- Core init (must come before any task that uses state/NVS) ---- */
    axion_state_init();
    calibration_store_init();
    alert_button_init();

    /* ---- One-shot setup tasks (delete themselves when done) ---------- */
    xTaskCreate(temperature_task,      "ds18b20",   4096, nullptr, 1, nullptr);
    xTaskCreate([](void *a){ i2c_bus_setup(); },   "i2c",      2048, nullptr, 1, nullptr);
    xTaskCreate([](void *a){ mpu_setup(); },       "mpu_init", 4096, nullptr, 1, nullptr);
    xTaskCreate(modem_setup_task,      "modem",     8192, nullptr, 1, nullptr);

    /* ---- Long-running workers ---------------------------------------- */
    xTaskCreate(mpu_rpy_task,          "mpu_rpy",   8192, nullptr, 2, nullptr);
    xTaskCreate(max30102_task,         "max30102",  8192, nullptr, 2, nullptr);
    /* Collision detector runs at a higher priority than the monitor so
     * that a 2G+ impact is detected and BIT_COLLISION_DETECTED is set
     * with minimal latency, even if the monitor is mid-snapshot or
     * pulsing the buzzer. */
    xTaskCreate(collision_task,        "collision", 4096, nullptr, 3, nullptr);
    xTaskCreate(monitor_task,          "monitor",   4096, nullptr, 1, nullptr);
    xTaskCreate(status_led_task,       "status_led",2048, nullptr, 1, nullptr);

    /* Uncomment to enable the live console output (ANSI-aware terminal). */
    // xTaskCreate(console_task,       "console",   4096, nullptr, 0, nullptr);

    ESP_LOGI(TAG, "All tasks spawned. Setup tasks will report when ready.");
}
