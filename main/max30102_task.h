#pragma once
/**
 * max30102_task.h - FreeRTOS task that periodically samples the
 * MAX30102, computes HR + SpO2, and pushes them into the shared state.
 */
#ifndef AXION_MAX30102_TASK_H
#define AXION_MAX30102_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task: samples MAX30102 into a 128-sample buffer, runs the
 *  autocorrelation HR + RMS-ratio SpO2 estimator, and updates the
 *  shared state. Sets BIT_MAX30102_READY after the first full buffer. */
void max30102_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_MAX30102_TASK_H */
