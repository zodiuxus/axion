#pragma once
/**
 * status_led.h — Green status LED task.
 *
 * LED behaviour:
 *   - BOOTING (things starting up):  blink 25 ms on / 75 ms off (100 ms period)
 *   - CALIBRATING (first run only): blink 50 ms on / 150 ms off (200 ms period)
 *   - COMPLETE: solid on for 2 s, then off permanently
 *
 * On a non-first-run boot the CALIBRATING phase is skipped — the LED
 * goes straight from BOOTING to COMPLETE once all sensors have set
 * their BIT_*_CALIBRATED bits.
 *
 * A safety timeout (LED_CALIBRATION_TIMEOUT_MS) ensures the LED reaches
 * COMPLETE even if a sensor fails to initialize and never sets its
 * calibrated bit.
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
