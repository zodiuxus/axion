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

/**
 * Send an AT command, wait for `expected_response` ("" = any bytes).
 * @param cmd                NUL-terminated command; "\r" added internally.
 * @param max_timeout_ms     Total wait time.
 * @param expected_response  Substring to wait for; "" = best-effort flush.
 * @return true if seen, false on timeout / UART error.
 */
bool at_command_send(const char *cmd, int max_timeout_ms, const char *expected_response);

/** Send an SMS body: CMGS header, wait for '>', write body + CTRL-Z,
 *  wait for the +CMGS ack. Thread-safe. */
bool at_command_send_sms_body(const char *phone, const char *body);

/** Drain anything currently in the UART RX FIFO. Thread-safe. */
void at_modem_flush_rx(void);

/** Copy the most recent response payload into `out` (NUL-terminated).
 *  Buffer stays intact between calls; safe from any task.
 *  @return bytes copied (excluding NUL), 0 if truncated. */
size_t at_command_last_response(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* AXION_AT_COMMAND_H */
