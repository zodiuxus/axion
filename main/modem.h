#pragma once
/**
 * modem.h — A7670E modem bring-up + GNSS polling.
 */
#ifndef AXION_MODEM_H
#define AXION_MODEM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bring up the A7670E: SIM PIN, network registration, SMS mode, GNSS.
 *  Sets BIT_AT_READY on success, then spawns the GNSS polling task.
 *  Synchronous; deletes the calling task at the end. */
void modem_setup_task(void *arg);

/** FreeRTOS task: periodically polls AT+CGNSSINFO and pushes parsed
 *  lat/lon/alt/speed into the shared state. */
void modem_gnss_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_MODEM_H */
