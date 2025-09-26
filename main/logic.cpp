#include <driver/uart.h>
#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "cstring"
#include "logic.h"

bool send_at_command(const char* cmd, int max_timeout_ms, const char* expected_response) {
    char cmd_buf[64];
    int len = snprintf(cmd_buf, sizeof(cmd_buf), "%s\r", cmd);

    uart_write_bytes(PORT_UART, cmd_buf, len);
    uart_flush(PORT_UART);
    ESP_LOGI(TAG, "Sent AT command: %s\n", cmd);

    len = 0;
    bool response_found = false;
    int64_t start_time = esp_timer_get_time() / 1000;

    while ((esp_timer_get_time() / 1000) - start_time < max_timeout_ms) {
        int read_len = uart_read_bytes(PORT_UART, reinterpret_cast<uint8_t *>(response_buf) + len, BUF_UART - len - 1, pdMS_TO_TICKS(100));

        if (read_len > 0) {
            len += read_len;
            response_buf[len] = '\0';
        }

        if (strstr(response_buf, expected_response)) response_found = true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Response:\n%s", response_buf);
    if (response_found) return true;
    return false;
}

void parse_gnss_info(const char* response) {

}

void get_coords() {
    while (true) {
        send_at_command("AT+CGNSSINFO", 9000, "OK");
        // parse_gnss_info(response_buf);
    vTaskDelay(5000/portTICK_PERIOD_MS);
    }
    vTaskDelete(nullptr);
}
