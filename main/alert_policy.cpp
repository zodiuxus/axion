/**
 * alert_policy.cpp - see alert_policy.h.
 *
 * Pure logic: no logging, no tasks, no hardware. Everything that decides
 * whether the device should alarm lives here so it can be tested on a host.
 */
#include "alert_policy.h"

#include <math.h>

static bool cooldown_active(int64_t now_ms, int64_t until_ms)
{
    return now_ms < until_ms;
}

void alert_policy_init(alert_policy_t *policy)
{
    if (policy == nullptr) {
        return;
    }
    policy->last_oxim_seq       = 0;
    policy->vitals_streak       = 0;
    policy->abort_cooldown_until = 0;
    for (int i = 0; i < ALERT_SRC_COUNT; ++i) {
        policy->cooldown_until[i] = 0;
    }
}

uint32_t alert_decision_mask(const alert_decision_t *decision)
{
    if (decision == nullptr) {
        return 0u;
    }
    uint32_t mask = 0u;
    for (int i = 0; i < ALERT_SRC_COUNT; ++i) {
        if (decision->raise[i]) {
            mask |= (1u << i);
        }
    }
    return mask;
}

alert_decision_t alert_policy_eval(alert_policy_t *policy,
                                   const alert_inputs_t *in,
                                   int64_t now_ms)
{
    alert_decision_t decision = {};

    if (policy == nullptr || in == nullptr) {
        return decision;
    }

    /* Count each readable window once; unreadable ones reset the streak. Runs
     * before the cooldown check so a sustained condition keeps counting. */
    if (in->oxim_seq != policy->last_oxim_seq) {
        policy->last_oxim_seq = in->oxim_seq;

        bool bad_window = false;
        if (in->oxim_fresh && in->spo2_valid) {
            if (in->heart_rate < HR_LOW_THRESHOLD ||
                in->heart_rate > HR_HIGH_THRESHOLD) {
                bad_window = true;
            }
            if (in->spo2 >= 70.0f && in->spo2 < SPO2_LOW_THRESHOLD) {
                bad_window = true;
            }
        }
        policy->vitals_streak = bad_window ? (policy->vitals_streak + 1) : 0;
    }

    /* ---- Conditions, each gated on fresh data --------------------------- */
    bool fall = in->imu_fresh &&
                (fabsf(in->roll_deg) >= FALL_ANGLE_THRESHOLD);

    /* Movement gates falls only; a stale speed is unknown, not moving. */
    if (fall && in->gnss_fresh && (in->speed_mps >= SPEED_STOPPED)) {
        fall = false;
    }

    /* Temperature: judged as baseline +/- delta only, so an off-body probe
     * reads as hypothermia. */
    const bool temp_valid = in->temp_fresh && in->temp_baseline_valid;
    const bool temp_low  = temp_valid &&
                           (in->temp_c < in->temp_baseline - TEMP_HYPO_DELTA);
    const bool temp_high = temp_valid &&
                           (in->temp_c > in->temp_baseline + TEMP_HYPER_DELTA);

    const bool vitals = in->oxim_fresh &&
                        (policy->vitals_streak >= VITALS_SUSTAIN_WINDOWS);

    /* Abort silences everything; otherwise each condition has its own
     * cooldown. */
    if (!cooldown_active(now_ms, policy->abort_cooldown_until)) {
        bool *r = decision.raise;
        r[ALERT_SRC_FALL]      = fall &&
                                 !cooldown_active(now_ms, policy->cooldown_until[ALERT_SRC_FALL]);
        r[ALERT_SRC_TEMP_LOW]  = temp_low &&
                                 !cooldown_active(now_ms, policy->cooldown_until[ALERT_SRC_TEMP_LOW]);
        r[ALERT_SRC_TEMP_HIGH] = temp_high &&
                                 !cooldown_active(now_ms, policy->cooldown_until[ALERT_SRC_TEMP_HIGH]);
        r[ALERT_SRC_VITALS]    = vitals &&
                                 !cooldown_active(now_ms, policy->cooldown_until[ALERT_SRC_VITALS]);

        decision.any = r[ALERT_SRC_FALL] || r[ALERT_SRC_TEMP_LOW] ||
                       r[ALERT_SRC_TEMP_HIGH] || r[ALERT_SRC_VITALS];
    }

    return decision;
}

void alert_policy_note_paged(alert_policy_t *policy, uint32_t src_mask,
                             int64_t now_ms)
{
    if (policy == nullptr) {
        return;
    }
    for (int i = 0; i < ALERT_SRC_COUNT; ++i) {
        if ((src_mask & (1u << i)) != 0u) {
            policy->cooldown_until[i] = now_ms + (int64_t)SMS_REPAGE_COOLDOWN_MS;
        }
    }
}

void alert_policy_note_abort(alert_policy_t *policy, int64_t now_ms)
{
    if (policy == nullptr) {
        return;
    }
    policy->abort_cooldown_until = now_ms + (int64_t)ABORT_COOLDOWN_MS;
}
