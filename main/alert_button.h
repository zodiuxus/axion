#pragma once
/**
 * alert_button.h — Hardware button that aborts the emergency countdown.
 *
 * A short press at any time during the monitor task's WARNING or ALERT
 * phase resets the state machine to NORMAL and prevents the SMS from
 * being sent. The button is debounced in software.
 *
 * Implementation: GPIO falling-edge ISR enqueues a timestamp; a
 * debounce task consumes the queue, drops events that arrive less than
 * BUTTON_DEBOUNCE_MS apart, and sets BIT_ALERT_ABORT in g_sensors_ready.
 * The monitor task consumes that bit via xEventGroupWaitBits.
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
