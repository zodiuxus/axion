/**
 * axion.cpp - application entry point.
 *
 * Init order: shared state + NVS -> GPIO ISR service + alert button ->
 * one-shot setup tasks (I2C bus, MPU, modem, temp) -> long-running
 * workers (mpu_int, oximetry, monitor, status LED).
 *
 * mpu_int_task is the single consumer of the MPU6050 INT pin and handles
 * both DMP YPR reads and raw-accel collision detection (one INT pin,
 * one task).
 *
 * SIM PIN and alert numbers come from gitignored secrets.h.
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

/* secrets.h must define a real phone number for slot 1 (may be a local
 * emergency number like "112" - no E.164 prefix required). */
static_assert(ALERT_PHONE_1[0] != '\0',
              "ALERT_PHONE_1 must be set in secrets.h (e.g. \"112\")");

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Axion booting - built " __DATE__ " " __TIME__);

    /* ---- Core init (must come before any task that uses state/NVS) ---- */
    axion_state_init();
    calibration_store_init();

    /* One GPIO ISR service for the whole app; modules only attach
     * handlers (a second install logs an ERROR even if handled). */
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

    /* ---- Long-running workers -------------------------------------- */
    xTaskCreate(mpu_int_task,          "mpu_int",   8192, nullptr, 3, nullptr);
    xTaskCreate(max30102_task,         "max30102",  8192, nullptr, 2, nullptr);
    xTaskCreate(monitor_task,          "monitor",   4096, nullptr, 1, nullptr);
    xTaskCreate(status_led_task,       "status_led",2048, nullptr, 1, nullptr);

    /* Uncomment to enable the live console output (ANSI-aware terminal). */
    // xTaskCreate(console_task,       "console",   4096, nullptr, 0, nullptr);

    ESP_LOGI(TAG, "All tasks spawned. Setup tasks will report when ready.");
}
