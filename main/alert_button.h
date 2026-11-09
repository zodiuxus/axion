#pragma once
/**
 * alert_button.h - Button that aborts the emergency countdown.
 *
 * A press during WARNING/ALERT returns the monitor to NORMAL and prevents the
 * SMS. ISR -> queue -> debounce task -> BIT_ALERT_ABORT.
 */
#ifndef AXION_ALERT_BUTTON_H
#define AXION_ALERT_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

/** Install the GPIO ISR + spawn the debounce task. Call once from
 *  app_main, after axion_state_init(). */
void alert_button_init(void);

#ifdef __cplusplus
}
#endif

#endif /* AXION_ALERT_BUTTON_H */
