/**
 * alert_button.cpp - see alert_button.h for design notes.
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
    /* Non-blocking; a full queue just drops the event (debounce task
     * catches the next one). */
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
        /* Edge-triggered; monitor consumes it clear-on-exit. */
        xEventGroupSetBits(g_sensors_ready, BIT_ALERT_ABORT);
    }
}

void alert_button_init(void)
{
    gpio_reset_pin(PIN_ALERT_BUTTON);
    gpio_set_direction(PIN_ALERT_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_ALERT_BUTTON);   /* active-low */

    s_button_queue = xQueueCreate(8, sizeof(int64_t));
    if (s_button_queue == nullptr) {
        /* The ISR dereferences this queue; without it, the first press would
         * crash the device. Disable the button instead. */
        ESP_LOGE(TAG, "button queue allocation failed - abort button disabled");
        return;
    }

    /* ISR service is installed centrally in app_main; just attach. */
    esp_err_t isr_err = gpio_isr_handler_add(PIN_ALERT_BUTTON, button_isr, nullptr);
    if (isr_err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add: %s - abort button disabled",
                 esp_err_to_name(isr_err));
        return;
    }
    gpio_set_intr_type(PIN_ALERT_BUTTON, GPIO_INTR_NEGEDGE);

    if (xTaskCreate(alert_button_task, "alert_btn", 2048, nullptr, 2,
                    nullptr) != pdPASS) {
        /* No consumer: stop generating events rather than filling the queue
         * with timestamps nothing will ever read. */
        ESP_LOGE(TAG, "debounce task creation failed - abort button disabled");
        gpio_isr_handler_remove(PIN_ALERT_BUTTON);
        return;
    }

    ESP_LOGI(TAG, "alert-abort button on GPIO %d (active-low, %u ms debounce)",
             PIN_ALERT_BUTTON, BUTTON_DEBOUNCE_MS);
}
