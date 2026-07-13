#pragma once
/**
 * at_command.h — Thread-safe A7670E modem driver.
 *
 * All UART1 access goes through this module. A FreeRTOS mutex serializes
 * both the UART peripheral and the shared response buffer, so multiple
 * tasks (GNSS polling, SMS alerting, AT init) can safely issue commands.
 */
#ifndef AXION_AT_COMMAND_H
#define AXION_AT_COMMAND_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One-time driver setup: installs UART driver + creates the mutex. */
esp_err_t at_modem_init(void);

/**
 * Send an AT command and wait up to `max_timeout_ms` for `expected_response`
 * to appear in the modem's reply.
 *
 * Thread-safe. Acquires the UART mutex for the whole transaction.
 *
 * @param cmd              NUL-terminated command, e.g. "AT+CGNSSPWR=1".
 *                         A trailing "\r" is added internally.
 * @param max_timeout_ms   Total time to wait for the expected response.
 * @param expected_response  Substring to look for; pass "" to return true
 *                         as soon as any byte arrives (best-effort flush).
 * @return true if `expected_response` was seen, false on timeout / UART error.
 */
bool at_command_send(const char *cmd, int max_timeout_ms, const char *expected_response);

/**
 * Send a single PDU body for an SMS (after `AT+CMGS="..."` returns the `>`
 * prompt). Writes `body` followed by CTRL-Z (0x1A) and waits briefly for
 * the modem to acknowledge. Used by the SMS alert path.
 *
 * Thread-safe (acquires the UART mutex).
 */
bool at_command_send_sms_body(const char *phone, const char *body);

/** Drain anything currently in the UART RX FIFO. Thread-safe. */
void at_modem_flush_rx(void);

/**
 * Copy the most recent response payload into `out` (NUL-terminated).
 * Returns the number of bytes copied (excluding the NUL), or 0 if the
 * response was longer than `out_size - 1` and got truncated.
 *
 * The internal buffer is left intact between calls; this getter may be
 * called from any task at any time without holding the UART mutex.
 *
 * Typical two-step pattern:
 *   if (at_command_send("AT+CGNSSINFO", 3000, "+CGNSSINFO:")) {
 *       char buf[UART_BUF_SIZE];
 *       size_t n = at_command_last_response(buf, sizeof(buf));
 *       parse(buf);
 *   }
 */
size_t at_command_last_response(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* AXION_AT_COMMAND_H */
