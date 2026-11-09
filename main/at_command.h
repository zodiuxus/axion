#pragma once
/**
 * at_command.h - Thread-safe A7670E modem driver.
 *
 * Mutex-serialized UART1 access; safe to issue commands from any task.
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

/** True once the UART driver is installed and configured. Lets callers fail
 *  fast instead of waiting out an AT timeout on a bus that cannot work. */
bool at_modem_uart_ready(void);

/** Send an AT command, wait for `expected_response` ("" = any bytes).
 *  @return true if seen, false on timeout / UART error. */
bool at_command_send(const char *cmd, int max_timeout_ms, const char *expected_response);

/** Like at_command_send(), but copies the matched response into `resp_out`
 * before releasing the UART, so callers never race another task. */
bool at_command_send_ex(const char *cmd, int max_timeout_ms,
                        const char *expected_response,
                        char *resp_out, size_t resp_out_size);

/** Send an SMS body: CMGS header, wait for '>', write body + CTRL-Z,
 *  wait for the +CMGS ack. Thread-safe. */
bool at_command_send_sms_body(const char *phone, const char *body);

/** Drain anything currently in the UART RX FIFO. Thread-safe. */
void at_modem_flush_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* AXION_AT_COMMAND_H */
