#include <cstdio>
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include "logic.h"



static void send_sms() {
    char msg[128];
    snprintf(msg, sizeof(msg), "coords: lat=%.6f, lon=%.6f, alt=%.1f", lat, lon, alt);

    send_at_command("AT+CMGS=\"" ALERT_PHONE "\"", 3000, ">");

    uart_write_bytes(PORT_UART, msg, strlen(msg));
    const char ctrlz = 0x1A;
    uart_write_bytes(PORT_UART, &ctrlz, 1);

    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for modem to finish sending
}

void monitor_task() {
    xEventGroupWaitBits(mpuReady, MPU_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    xEventGroupWaitBits(atReady, AT_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    gpio_reset_pin(PIN_BUZZER);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);

    typedef enum { NORMAL, WARNING, ALERT } state_t;
    state_t state = NORMAL;
    int64_t state_start_ms = 0;

    while (true) {
        float current_speed = speed;
        float current_temp  = temp_values[0];

        bool stopped       = (current_speed < SPEED_STOPPED);
        bool temp_abnormal = (current_temp > TEMP_MIN_VALID) &&
                             (current_temp < TEMP_LOW || current_temp > TEMP_HIGH);

        float roll = ypr[2] * 180.0f / M_PI;
        bool fallen = (fabsf(roll) >= FALL_ANGLE_THRESHOLD);

        bool alert_condition = fallen || temp_abnormal;
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (!stopped || !alert_condition) {
            if (state != NORMAL) {
                gpio_set_level(PIN_BUZZER, 0);
                state = NORMAL;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        switch (state) {
            case NORMAL:
                state = WARNING;
                state_start_ms = now_ms;
                break;

            case WARNING:
                if (now_ms - state_start_ms >= WARNING_MS) {
                    state = ALERT;
                    state_start_ms = now_ms;
                    printf("Warning: vitals!\n");
                }
                break;

            case ALERT:
                gpio_set_level(PIN_BUZZER, (now_ms / BUZZER_PERIOD_MS) % 2);
                if (now_ms - state_start_ms >= ALERT_MS) {
                    gpio_set_level(PIN_BUZZER, 0);
                    printf("Sending message\n");
                    send_sms();
                    state = NORMAL;
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(nullptr);
}
