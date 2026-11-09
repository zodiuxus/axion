#pragma once
/**
 * alert_delivery.h - Owns the SMS path.
 *
 * The monitor enqueues (never touches the UART); the worker formats and sends
 * with retries. Only a delivered alert starts the re-page cooldown.
 */
#ifndef AXION_ALERT_DELIVERY_H
#define AXION_ALERT_DELIVERY_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Create the delivery state. Call once from app_main before creating tasks. */
esp_err_t alert_delivery_init(void);

/** Queue an alert. Merges with an alert already pending; never blocks on the
 *  UART. @return false if not initialised or nothing to send. */
bool alert_delivery_enqueue(uint32_t mask, bool collision);

/** True while an alert is queued, sending or retrying. */
bool alert_delivery_busy(void);

/** Read and clear the mask of conditions delivered since the last poll. */
uint32_t alert_delivery_take_delivered(void);

/** FreeRTOS task: waits for work, formats the body, sends with retries. */
void alert_delivery_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_ALERT_DELIVERY_H */
