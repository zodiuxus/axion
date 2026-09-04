/**
 * at_command.cpp - see at_command.h.
 *
 * Replaces the bare `send_at_command` in the old logic.cpp with a proper
 * mutex-protected driver. Fixes:
 *   - The shared response buffer is now file-scoped (not `static` in a
 *     header, which gave every TU its own copy).
 *   - The buffer is cleared at the start of every call, so stale bytes
 *     from a previous command cannot cause false `strstr()` matches.
 *   - `uart_flush` is called BEFORE writing the command (not after),
 *     so legitimate response bytes that arrive in the gap are not lost.
 */
#include "at_command.h"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"

static const char *TAG = "axion.modem";

static SemaphoreHandle_t s_uart_mutex   = nullptr;
static char              *s_resp_buf    = nullptr;
static size_t             s_resp_buf_sz = 0;

static bool take_mutex(TickType_t ticks)
{
    return s_uart_mutex && xSemaphoreTake(s_uart_mutex, ticks) == pdTRUE;
}

static void give_mutex(void)
{
    if (s_uart_mutex) xSemaphoreGive(s_uart_mutex);
}

esp_err_t at_modem_init(void)
{
    if (s_uart_mutex == nullptr) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    if (s_resp_buf == nullptr) {
        s_resp_buf_sz = UART_BUF_SIZE;
        s_resp_buf    = new char[s_resp_buf_sz];
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate           = UART_MODEM_BAUD;
    uart_config.data_bits           = UART_DATA_8_BITS;
    uart_config.parity              = UART_PARITY_DISABLE;
    uart_config.stop_bits           = UART_STOP_BITS_1;
    uart_config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk          = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(UART_MODEM_PORT, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(UART_MODEM_PORT, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(UART_MODEM_PORT, PIN_UART_TX, PIN_UART_RX,
                       PIN_UART_RTS, PIN_UART_CTS);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "UART%d @ %u baud ready (TX=%d RX=%d RTS=%d CTS=%d)",
             UART_MODEM_PORT, UART_MODEM_BAUD,
             PIN_UART_TX, PIN_UART_RX, PIN_UART_RTS, PIN_UART_CTS);
    return ESP_OK;
}

void at_modem_flush_rx(void)
{
    if (!take_mutex(pdMS_TO_TICKS(500))) return;
    uart_flush_input(UART_MODEM_PORT);
    give_mutex();
}

size_t at_command_last_response(char *out, size_t out_size)
{
    if (out == nullptr || out_size == 0) return 0;
    /* Read-only access to the buffer; no need for the mutex because
     * at_command_send() is the only writer and callers typically call
     * this immediately after their own at_command_send() returns. */
    size_t copy_len = strlen(s_resp_buf ? s_resp_buf : "");
    if (copy_len >= out_size) copy_len = out_size - 1;
    memcpy(out, s_resp_buf ? s_resp_buf : "", copy_len);
    out[copy_len] = '\0';
    return copy_len;
}

bool at_command_send(const char *cmd, int max_timeout_ms, const char *expected_response)
{
    if (cmd == nullptr) return false;
    if (!take_mutex(pdMS_TO_TICKS(2000))) {
        ESP_LOGW(TAG, "UART mutex timeout on cmd: %s", cmd);
        return false;
    }

    /* Drain anything left from a previous transaction, then clear our
     * local buffer so stale substrings cannot produce false positives. */
    uart_flush_input(UART_MODEM_PORT);
    if (s_resp_buf && s_resp_buf_sz > 0) s_resp_buf[0] = '\0';

    char cmd_buf[80];
    int  cmd_len = snprintf(cmd_buf, sizeof(cmd_buf), "%s\r", cmd);
    uart_write_bytes(UART_MODEM_PORT, cmd_buf, cmd_len);

    int     len            = 0;
    bool    empty_expected = (expected_response == nullptr || expected_response[0] == '\0');
    bool    response_found = false;
    int64_t start_ms       = esp_timer_get_time() / 1000;

    while ((esp_timer_get_time() / 1000) - start_ms < max_timeout_ms) {
        int read_len = uart_read_bytes(UART_MODEM_PORT,
                                       reinterpret_cast<uint8_t *>(s_resp_buf) + len,
                                       s_resp_buf_sz - len - 1,
                                       pdMS_TO_TICKS(100));
        if (read_len > 0) {
            len += read_len;
            s_resp_buf[len] = '\0';
            if (empty_expected) { response_found = true; break; }
            if (strstr(s_resp_buf, expected_response) != nullptr) {
                response_found = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    give_mutex();
    return response_found;
}

bool at_command_send_sms_body(const char *phone, const char *body)
{
    if (phone == nullptr || body == nullptr) return false;
    if (!take_mutex(pdMS_TO_TICKS(2000))) return false;

    /* Issue the CMGS header. The modem answers with a `>` prompt. */
    char header[64];
    int  header_len = snprintf(header, sizeof(header), "AT+CMGS=\"%s\"\r", phone);
    uart_flush_input(UART_MODEM_PORT);
    uart_write_bytes(UART_MODEM_PORT, header, header_len);

    bool    got_prompt = false;
    int     len        = 0;
    int64_t start_ms   = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) - start_ms < 3000) {
        int read_len = uart_read_bytes(UART_MODEM_PORT,
                                       reinterpret_cast<uint8_t *>(s_resp_buf) + len,
                                       s_resp_buf_sz - len - 1,
                                       pdMS_TO_TICKS(100));
        if (read_len > 0) {
            len += read_len;
            s_resp_buf[len] = '\0';
            if (strchr(s_resp_buf, '>') != nullptr) { got_prompt = true; break; }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    bool ok = false;
    if (got_prompt) {
        uart_write_bytes(UART_MODEM_PORT, body, strlen(body));
        const char ctrlz = 0x1A;
        uart_write_bytes(UART_MODEM_PORT, &ctrlz, 1);

        /* Wait for `+CMGS:` acknowledgement (up to 10 s). */
        s_resp_buf[0] = '\0';
        len           = 0;
        start_ms      = esp_timer_get_time() / 1000;
        while ((esp_timer_get_time() / 1000) - start_ms < 10000) {
            int read_len = uart_read_bytes(UART_MODEM_PORT,
                                           reinterpret_cast<uint8_t *>(s_resp_buf) + len,
                                           s_resp_buf_sz - len - 1,
                                           pdMS_TO_TICKS(100));
            if (read_len > 0) {
                len += read_len;
                s_resp_buf[len] = '\0';
                if (strstr(s_resp_buf, "+CMGS:") != nullptr) { ok = true; break; }
                if (strstr(s_resp_buf, "ERROR")  != nullptr) { ok = false; break; }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        ESP_LOGW(TAG, "no '>' prompt for SMS to %s", phone);
    }

    give_mutex();
    return ok;
}
