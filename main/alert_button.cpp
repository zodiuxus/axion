/**
 * alert_button.cpp — see alert_button.h for design notes.
 */
#include "alert_button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "state.h"

static const char *TAG = "axion.button";

static QueueHandle_t s_button_queue = nullptr;

/* ISR: just enqueue a timestamp. Keep this minimal. */
static void IRAM_ATTR button_isr(void * /*arg*/)
{
    int64_t now = esp_timer_get_time();
    /* Non-blocking send; if the queue is full the event is silently
     * dropped — that's fine, the debounce task will catch the next one. */
    xQueueSendFromISR(s_button_queue, &now, nullptr);
}

/* Debounce task: consume timestamps, drop events < BUTTON_DEBOUNCE_MS
 * apart, set BIT_ALERT_ABORT on a confirmed press. */
static void alert_button_task(void * /*arg*/)
{
    int64_t last_press_us = 0;
    int64_t press_time;
    while (true) {
        if (xQueueReceive(s_button_queue, &press_time, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (press_time - last_press_us < (int64_t)BUTTON_DEBOUNCE_MS * 1000) {
            continue;  /* bounce */
        }
        last_press_us = press_time;

        ESP_LOGI(TAG, "alert-abort button pressed");
        /* Edge-triggered signal: the monitor task consumes this via
         * xEventGroupWaitBits with xClearOnExit=pdTRUE. */
        xEventGroupSetBits(g_sensors_ready, BIT_ALERT_ABORT);
    }
}

void alert_button_init(void)
{
    gpio_reset_pin(PIN_ALERT_BUTTON);
    gpio_set_direction(PIN_ALERT_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_ALERT_BUTTON);   /* active-low */

    s_button_queue = xQueueCreate(8, sizeof(int64_t));

    /* Install the GPIO ISR service if not already installed (other
     * modules might have done it first — that's fine, we ignore the
     * "already installed" return code). */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_err));
        return;
    }

    isr_err = gpio_isr_handler_add(PIN_ALERT_BUTTON, button_isr, nullptr);
    if (isr_err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add: %s", esp_err_to_name(isr_err));
        return;
    }
    gpio_set_intr_type(PIN_ALERT_BUTTON, GPIO_INTR_NEGEDGE);

    xTaskCreate(alert_button_task, "alert_btn", 2048, nullptr, 2, nullptr);
    ESP_LOGI(TAG, "alert-abort button on GPIO %d (active-low, %u ms debounce)",
             PIN_ALERT_BUTTON, BUTTON_DEBOUNCE_MS);
}
