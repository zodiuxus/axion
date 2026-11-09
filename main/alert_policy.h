#pragma once
/**
 * alert_policy.h - The alerting decision (pure, host-testable).
 *
 * Owns what counts as an alert: freshness gating, the vitals debounce and
 * per-condition cooldowns. No ESP-IDF dependencies.
 */
#ifndef AXION_ALERT_POLICY_H
#define AXION_ALERT_POLICY_H

#include <stdint.h>
#include <stdbool.h>

#include "config_tuning.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Conditions the policy can raise. Collisions are not listed: they arrive as
 *  discrete edges and are handled by the caller (they bypass all cooldowns). */
typedef enum {
    ALERT_SRC_FALL = 0,   /* |roll| >= FALL_ANGLE_THRESHOLD, subject stopped */
    ALERT_SRC_TEMP_LOW,   /* below baseline - TEMP_HYPO_DELTA */
    ALERT_SRC_TEMP_HIGH,  /* above baseline + TEMP_HYPER_DELTA */
    ALERT_SRC_VITALS,     /* HR/SpO2 outside range for N valid windows */
    ALERT_SRC_COUNT
} alert_src_t;

/** One sample of the world. Field names mirror axion_state_t so the caller
 *  fills this in without translation; the struct has no ESP-IDF types. */
typedef struct {
    bool  imu_fresh;             /* ypr age <= YPR_STALE_MS */
    float roll_deg;

    bool  gnss_fresh;            /* last valid fix age <= GNSS_STALE_MS */
    float speed_mps;             /* only meaningful when gnss_fresh */

    bool  temp_fresh;            /* temp age <= TEMP_STALE_MS */
    bool  temp_baseline_valid;
    float temp_c;
    float temp_baseline;

    bool  oxim_fresh;            /* oxim age <= OXIM_STALE_MS */
    uint32_t oxim_seq;           /* bumped on every completed window */
    bool  spo2_valid;
    int   heart_rate;
    float spo2;
} alert_inputs_t;

/** Policy memory. All fields are owned by the caller's task. */
typedef struct {
    uint32_t last_oxim_seq;
    int      vitals_streak;
    int64_t  cooldown_until[ALERT_SRC_COUNT];
    int64_t  abort_cooldown_until;
} alert_policy_t;

/** What is raising on this tick. */
typedef struct {
    bool raise[ALERT_SRC_COUNT];
    bool any;
} alert_decision_t;

/** Reset to the start-up state. */
void alert_policy_init(alert_policy_t *policy);

/** Evaluate one tick. Advances the vitals debounce as a side effect; the
 *  cooldown state is read but not modified. Call once per monitor tick. */
alert_decision_t alert_policy_eval(alert_policy_t *policy,
                                   const alert_inputs_t *in,
                                   int64_t now_ms);

/** Record that an alert for these sources was paged, starting their re-page
 *  cooldown. Pass the mask from alert_decision_mask(). */
void alert_policy_note_paged(alert_policy_t *policy, uint32_t src_mask,
                             int64_t now_ms);

/** Record a user abort: suppresses every condition for ABORT_COOLDOWN_MS. */
void alert_policy_note_abort(alert_policy_t *policy, int64_t now_ms);

/** Bitmask (1u << alert_src_t) of the conditions raised by a decision. */
uint32_t alert_decision_mask(const alert_decision_t *decision);

#ifdef __cplusplus
}
#endif

#endif /* AXION_ALERT_POLICY_H */
