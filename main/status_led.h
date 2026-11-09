#pragma once
/**
 * status_led.h - Green status LED task.
 *
 * BOOTING: 25 ms on / 75 ms off. CALIBRATING (first run only): 50/150.
 * COMPLETE: solid 2 s, then off. LED_CALIBRATION_TIMEOUT_MS guarantees the
 * COMPLETE state even if a sensor never calibrates.
 */
#ifndef AXION_STATUS_LED_H
#define AXION_STATUS_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task: drives the green status LED through the boot →
 *  calibrate → ready sequence. Deletes itself after the final 2 s glow. */
void status_led_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_STATUS_LED_H */
