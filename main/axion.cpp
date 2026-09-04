/**
 * axion.cpp - application entry point.
 *
 * Wires up the Axion firmware:
 *   1. Initializes shared state + event group.
 *   2. Initializes NVS (for the calibration_store bitfield).
 *   3. Installs the GPIO ISR service + the alert-abort button ISR.
 *   4. Spawns one setup task per peripheral (I2C bus, MPU, modem, temp).
 *   5. Spawns the long-running workers (mpu_int, gnss, oximetry, monitor,
 *      status LED, optional console).
 *
 * mpu_int_task is the single consumer of the MPU6050 INT pin and handles
 * both DMP yaw/pitch/roll reads AND raw-accel collision detection - the
 * chip has one INT pin, so one task covers both. There is no separate
 * collision_task anymore (the threshold/cooldown helper lives in
 * collision.cpp and is called from mpu_int_task).
 *
 * Secrets: SIM PIN and alert phone numbers come from `secrets.h`, which
 * is gitignored. If the file is missing the build fails here with a
 * clear message.
 */
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
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

    /* One GPIO ISR service for the whole app, installed before any module
     * init. The alert button, MPU INT, and MAX30102 INT attach their
     * handlers with gpio_isr_handler_add() only - calling
     * gpio_install_isr_service() a second time returns
     * ESP_ERR_INVALID_STATE and, worse, the IDF driver itself logs
     * "GPIO isr service already installed" at ERROR level even when the
     * caller handles the code gracefully. */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
    }

    alert_button_init();

    /* ---- One-shot setup tasks (delete themselves when done) ---------- */
    xTaskCreate(temperature_task,      "ds18b20",   4096, nullptr, 1, nullptr);
    xTaskCreate([](void *a){ i2c_bus_setup(); },   "i2c_init",      3072, nullptr, 1, nullptr);
    xTaskCreate([](void *a){ mpu_setup(); },       "mpu_init", 4096, nullptr, 1, nullptr);
    xTaskCreate(modem_setup_task,      "modem",     8192, nullptr, 1, nullptr);

    /* ---- Long-running workers ---------------------------------------- */
    /* mpu_int_task is the single consumer of the MPU6050 INT pin. It
     * services both DMP packet reads (yaw/pitch/roll) AND raw-accel
     * collision detection - the chip has one INT pin, so one task
     * covers both. */
    xTaskCreate(mpu_int_task,          "mpu_int",   8192, nullptr, 3, nullptr);
    xTaskCreate(max30102_task,         "max30102",  8192, nullptr, 2, nullptr);
    xTaskCreate(monitor_task,          "monitor",   4096, nullptr, 1, nullptr);
    xTaskCreate(status_led_task,       "status_led",2048, nullptr, 1, nullptr);

    /* Uncomment to enable the live console output (ANSI-aware terminal). */
    // xTaskCreate(console_task,       "console",   4096, nullptr, 0, nullptr);

    ESP_LOGI(TAG, "All tasks spawned. Setup tasks will report when ready.");
}
