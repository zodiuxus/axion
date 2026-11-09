/**
 * at_command.cpp - see at_command.h.
 */
#include "at_command.h"

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
/* Static staging buffer: the AT layer is the only reader/writer and the
 * mutex serialises access. No allocation, no null checks, no leak on an error
 * path. */
static char               s_resp_buf[UART_BUF_SIZE];
static bool               s_uart_installed = false;

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
    s_uart_installed = false;
    if (s_uart_mutex == nullptr) {
        s_uart_mutex = xSemaphoreCreateMutex();
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
    s_uart_installed = true;
    return ESP_OK;
}

bool at_modem_uart_ready(void)
{
    return s_uart_installed;
}

void at_modem_flush_rx(void)
{
    if (!take_mutex(pdMS_TO_TICKS(500))) return;
    uart_flush_input(UART_MODEM_PORT);
    give_mutex();
}

/* Core of the AT layer; the caller holds the UART mutex. The payload is copied
 * out here so no separate accessor is needed. */
static bool send_locked(const char *cmd, int max_timeout_ms,
                        const char *expected_response,
                        char *resp_out, size_t resp_out_size)
{
    /* Flush RX + clear the buffer so stale bytes can't false-match. */
    uart_flush_input(UART_MODEM_PORT);
    s_resp_buf[0] = '\0';

    /* Command + CR written separately: no fixed buffer, no truncated length
     *. */
    uart_write_bytes(UART_MODEM_PORT, cmd, strlen(cmd));
    uart_write_bytes(UART_MODEM_PORT, "\r", 1);

    const size_t expected_len =
        (expected_response != nullptr) ? strlen(expected_response) : 0;

    int     len            = 0;
    bool    empty_expected = (expected_len == 0);
    bool    response_found = false;
    int64_t start_ms       = esp_timer_get_time() / 1000;

    while ((esp_timer_get_time() / 1000) - start_ms < max_timeout_ms) {
        /* Full buffer: keep the tail (still matches across the fold) and keep
         * reading. */
        if (len >= (int)sizeof(s_resp_buf) - 1) {
            size_t keep = 0;
            if (expected_len > 1 && expected_len < sizeof(s_resp_buf)) {
                keep = expected_len - 1;
            }
            if (keep > (size_t)len) {
                keep = (size_t)len;
            }
            memmove(s_resp_buf, s_resp_buf + len - keep, keep);
            len = (int)keep;
            s_resp_buf[len] = '\0';
        }

        int read_len = uart_read_bytes(UART_MODEM_PORT,
                                       reinterpret_cast<uint8_t *>(s_resp_buf) + len,
                                       sizeof(s_resp_buf) - len - 1,
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

    if (resp_out != nullptr && resp_out_size > 0) {
        size_t copy_len = strlen(s_resp_buf);
        if (copy_len >= resp_out_size) copy_len = resp_out_size - 1;
        memcpy(resp_out, s_resp_buf, copy_len);
        resp_out[copy_len] = '\0';
    }

    return response_found;
}

bool at_command_send_ex(const char *cmd, int max_timeout_ms,
                        const char *expected_response,
                        char *resp_out, size_t resp_out_size)
{
    if (cmd == nullptr) return false;
    if (!take_mutex(pdMS_TO_TICKS(2000))) {
        ESP_LOGW(TAG, "UART mutex timeout on cmd: %s", cmd);
        return false;
    }

    const bool ok = send_locked(cmd, max_timeout_ms, expected_response,
                                resp_out, resp_out_size);

    give_mutex();
    return ok;
}

bool at_command_send(const char *cmd, int max_timeout_ms, const char *expected_response)
{
    return at_command_send_ex(cmd, max_timeout_ms, expected_response, nullptr, 0);
}

bool at_command_send_sms_body(const char *phone, const char *body)
{
    if (phone == nullptr || body == nullptr) return false;
    /* Alerts come from their own task now, so waiting out a GNSS poll is fine;
     * do not give up after 2 s and lose the alert. */
    if (!take_mutex(pdMS_TO_TICKS(15000))) return false;

    /* Issue the CMGS header. The modem answers with a `>` prompt. Written in
     * three parts so a long number cannot truncate the CR. */
    static const char k_cmgs_prefix[] = "AT+CMGS=\"";
    static const char k_cmgs_suffix[] = "\"\r";
    uart_flush_input(UART_MODEM_PORT);
    uart_write_bytes(UART_MODEM_PORT, k_cmgs_prefix, sizeof(k_cmgs_prefix) - 1);
    uart_write_bytes(UART_MODEM_PORT, phone, strlen(phone));
    uart_write_bytes(UART_MODEM_PORT, k_cmgs_suffix, sizeof(k_cmgs_suffix) - 1);

    bool    got_prompt = false;
    int     len        = 0;
    int64_t start_ms   = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) - start_ms < 3000) {
        int read_len = uart_read_bytes(UART_MODEM_PORT,
                                       reinterpret_cast<uint8_t *>(s_resp_buf) + len,
                                       sizeof(s_resp_buf) - len - 1,
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
                                           sizeof(s_resp_buf) - len - 1,
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
        /* No prompt: the modem may still wait for the body, so cancel with
         * ESC. */
        ESP_LOGW(TAG, "no '>' prompt for SMS to %s - cancelling with ESC", phone);
        const char esc = 0x1B;
        uart_write_bytes(UART_MODEM_PORT, &esc, 1);
        vTaskDelay(pdMS_TO_TICKS(200));   /* let the modem absorb the cancel */
    }

    give_mutex();
    return ok;
}
