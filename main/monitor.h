#pragma once
/**
 * monitor.h - Fall & vitals monitor task.
 */
#ifndef AXION_MONITOR_H
#define AXION_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task: watches the shared state for fall events and abnormal
 *  vitals. Goes through a NORMAL -> WARNING -> ALERT state machine and
 *  sends SMS alerts to each recipient listed in secrets.h. */
void monitor_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_MONITOR_H */
