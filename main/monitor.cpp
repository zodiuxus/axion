/**
 * monitor.cpp - Fall, vitals, and collision monitor.
 *
 * State machine: NORMAL → WARNING (5 s) → ALERT (5 s, buzzer pulsing) → SMS.
 *
 * Two edge-triggered signals from the shared event group drive the
 * out-of-band transitions:
 *
 *   BIT_ALERT_ABORT       (button press)  - at any point in WARNING or
 *                                            ALERT, immediately reset to
 *                                            NORMAL and arm a cooldown so
 *                                            the state machine doesn't
 *                                            re-enter WARNING while the
 *                                            user is still recovering.
 *
 *   BIT_COLLISION_DETECTED (4G+ impact)   - bypass the WARNING phase
 *                                            entirely and jump straight
 *                                            to ALERT. This is the
 *                                            highest-priority alert
 *                                            source: a real collision
 *                                            starts the contacting
 *                                            sequence within tens of ms
 *                                            instead of waiting 5 s.
 *
 * A collision-triggered ALERT is "sticky": the normal "if alert_condition
 * cleared, return to NORMAL" check is skipped while the ALERT was caused
 * by a collision. A real impact shouldn't be cancelled just because the
 * device came to rest. The button is still the escape hatch - a press at
 * any point during the ALERT window cancels the SMS.
 *
 * After a button abort, a cooldown (ABORT_COOLDOWN_MS) prevents the state
 * machine from immediately re-entering WARNING if the alert condition is
 * still present. The cooldown does NOT apply to collision-triggered
 * alerts - a real 2G+ impact is always honored.
 */
#include "monitor.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "at_command.h"
#include "config.h"
#include "secrets.h"
#include "state.h"

static const char *TAG = "axion.monitor";

/* Recipients table - populated from secrets.h. Empty slots are skipped.
 * Order matters: ALERT_PHONE_1 (112) is contacted first, ALERT_PHONE_2
 * (personal backup) second. */
static const char *const k_alert_phones[AXION_ALERT_PHONE_MAX_COUNT] = {
    ALERT_PHONE_1,
    ALERT_PHONE_2,
};

/* Build the SMS body. The trigger label lets emergency services distinguish
 * a collision (e.g. vehicle impact) from a fall/vitals event. If the most
 * recent GNSS poll had no lock, the position is the last known fix, not the
 * current one - the message says so explicitly rather than implying a live
 * position (or, worse, silently sending 0,0 as it used to before a fix was
 * ever acquired). */
static void send_alert_sms(const axion_state_t *s, bool collision_triggered)
{
    char msg[200];
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t fix_age_s = (s->gnss_fix_ms > 0) ? (now_ms - s->gnss_fix_ms) / 1000 : -1;

    if (s->gnss_fix_valid) {
        snprintf(msg, sizeof(msg),
                 "AXION ALERT: %s. "
                 "lat=%.6f lon=%.6f alt=%.1f spd=%.1fm/s "
                 "temp=%.2fC hr=%d spo2=%.1f%%",
                 collision_triggered ? "collision detected" : "fall/vitals",
                 s->lat, s->lon, s->alt, s->speed,
                 s->temp_c, s->heart_rate, s->spo2);
    } else if (fix_age_s >= 0) {
        snprintf(msg, sizeof(msg),
                 "AXION ALERT: %s. "
                 "lat=%.6f lon=%.6f (LAST KNOWN, %llds old) alt=%.1f "
                 "temp=%.2fC hr=%d spo2=%.1f%%",
                 collision_triggered ? "collision detected" : "fall/vitals",
                 s->lat, s->lon, (long long)fix_age_s, s->alt,
                 s->temp_c, s->heart_rate, s->spo2);
    } else {
        /* Never had a fix this boot at all - don't send a bogus 0,0. */
        snprintf(msg, sizeof(msg),
                 "AXION ALERT: %s. "
                 "NO GPS FIX YET "
                 "temp=%.2fC hr=%d spo2=%.1f%%",
                 collision_triggered ? "collision detected" : "fall/vitals",
                 s->temp_c, s->heart_rate, s->spo2);
    }

    for (int i = 0; i < AXION_ALERT_PHONE_MAX_COUNT; ++i) {
        if (k_alert_phones[i] == nullptr || k_alert_phones[i][0] == '\0') continue;
        ESP_LOGW(TAG, "Sending alert to %s", k_alert_phones[i]);
        bool ok = at_command_send_sms_body(k_alert_phones[i], msg);
        ESP_LOGI(TAG, "  -> %s", ok ? "OK" : "FAILED");
    }
}

void monitor_task(void * /*arg*/)
{
    /* Need MPU (for fall + collision detection) + modem (for SMS). */
    axion_state_wait_all(BIT_MPU_READY | BIT_AT_READY);

    gpio_reset_pin(PIN_BUZZER);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);

    typedef enum { NORMAL, WARNING, ALERT } state_t;
    state_t  state                 = NORMAL;
    int64_t  state_start_ms        = 0;
    int64_t  abort_cooldown_until  = 0;   /* suppresses re-trigger after abort */
    int64_t  sms_cooldown_until    = 0;   /* suppresses re-page cycles after an SMS */
    /* True while the current ALERT was triggered by a collision. While
     * set, the "alert_condition cleared -> return to NORMAL" check is
     * bypassed so the ALERT runs its full ALERT_MS window. Cleared on
     * SMS send or button abort. */
    bool     collision_alert_active = false;
    /* Vitals debounce: count consecutive abnormal VALID windows.
     * oxim_seq lets the 100 ms poll consume each 5.12 s window exactly
     * once - without it, one window would be counted ~51 times and
     * the streak would trip within a single window. */
    uint32_t last_oxim_seq = 0;
    int      vitals_streak = 0;

    axion_state_t snap;

    while (true) {
        /* Wait up to 100 ms for either a tick or an edge signal.
         * xClearOnExit=pdTRUE consumes the edge bits (abort + collision). */
        EventBits_t bits = xEventGroupWaitBits(g_sensors_ready,
                                               BIT_ALERT_ABORT | BIT_COLLISION_DETECTED,
                                               pdTRUE,    /* clear on exit */
                                               pdFALSE,   /* wait for ANY */
                                               pdMS_TO_TICKS(100));
        bool abort_pressed = (bits & BIT_ALERT_ABORT) != 0;
        bool collision     = (bits & BIT_COLLISION_DETECTED) != 0;

        int64_t now_ms = esp_timer_get_time() / 1000;

        /* ---- Button abort: highest-priority cancel. -------------------- */
        if (abort_pressed) {
            if (state != NORMAL) {
                gpio_set_level(PIN_BUZZER, 0);
                ESP_LOGW(TAG, "alert aborted by button");
            }
            state                  = NORMAL;
            collision_alert_active = false;
            abort_cooldown_until   = now_ms + ABORT_COOLDOWN_MS;
            continue;
        }

        /* Always pull a fresh snapshot so the SMS body carries current
         * vitals + location, regardless of which path triggered the alert. */
        axion_state_snapshot(&snap);

        /* ---- Collision: bypass WARNING, jump straight to ALERT. ------- */
        if (collision) {
            state                  = ALERT;
            state_start_ms         = now_ms;
            collision_alert_active = true;
            ESP_LOGW(TAG, "collision detected - escalating directly to ALERT "
                         "(WARNING skipped)");
            /* Fall through to the switch below; the ALERT case will pulse
             * the buzzer and send the SMS when ALERT_MS expires. */
        } else if (!collision_alert_active) {
            /* ---- Normal fall/vitals check (skipped during a collision ALERT). -- */
            float roll_deg      = snap.ypr[2] * 180.0f / (float)M_PI;
            bool  stopped       = (snap.speed < SPEED_STOPPED);

            /* Temperature thresholds are per-user: the calibrated baseline
             * (stored in NVS, loaded into shared state) ± fixed deltas.
             *   hypothermia:  temp < baseline - 0.7 °C
             *   hyperthermia: temp > baseline + 1.0 °C
             * If no baseline is available yet (first boot, calibration
             * still running), temp_baseline defaults to 37.0 °C. */
            bool temp_valid = snap.temp_baseline_valid;
            float temp_low  = snap.temp_baseline - TEMP_HYPO_DELTA;
            float temp_high = snap.temp_baseline + TEMP_HYPER_DELTA;
            bool  temp_abnormal  = temp_valid &&
                                   (snap.temp_c < temp_low || snap.temp_c > temp_high);

            /* ---- Heart rate / SpO2 (debounced per window) ----------------
             * Only VALID windows carry signal - a finger-off window
             * reports hr=0 / spo2=0 / valid=0 and must never look
             * like bradycardia or hypoxemia. spo2==0 on a VALID window
             * means the estimator's own <70 % floor rejected the curve
             * output as noise - this layer agrees, so 0 is ignored
             * here too. An invalid OR in-range window resets the
             * streak: no data (or normal data) is not a continuation
             * of an abnormal trend. */
            if (snap.oxim_seq != last_oxim_seq) {
                last_oxim_seq = snap.oxim_seq;
                bool bad_window = false;
                if (snap.spo2_valid) {
                    if (snap.heart_rate < HR_LOW_THRESHOLD ||
                        snap.heart_rate > HR_HIGH_THRESHOLD) {
                        bad_window = true;
                    }
                    if (snap.spo2 >= 70.0f && snap.spo2 < SPO2_LOW_THRESHOLD) {
                        bad_window = true;
                    }
                }
                vitals_streak = bad_window ? (vitals_streak + 1) : 0;
                if (bad_window) {
                    ESP_LOGW(TAG, "vitals window out of range: hr=%d bpm "
                                 "spo2=%.1f%% (streak %d/%d)",
                             snap.heart_rate, (double)snap.spo2,
                             vitals_streak, VITALS_SUSTAIN_WINDOWS);
                }
            }
            bool vitals_abnormal = (vitals_streak >= VITALS_SUSTAIN_WINDOWS);

            bool  fallen        = (fabsf(roll_deg) >= FALL_ANGLE_THRESHOLD);
            bool  alert_condition = fallen || temp_abnormal || vitals_abnormal;
            /* Post-abort cooldown OR post-SMS re-page cooldown: stay in
             * NORMAL even if the condition persists. Collision edges
             * never reach this check, so a real impact is always
             * honored regardless of either cooldown. */
            bool  in_cooldown     = (now_ms < abort_cooldown_until) ||
                                    (now_ms < sms_cooldown_until);

            /* If we're moving, the condition cleared, or we're in the
             * post-abort cooldown, return to / stay in NORMAL. */
            if (!stopped || !alert_condition || in_cooldown) {
                if (state != NORMAL) {
                    gpio_set_level(PIN_BUZZER, 0);
                    state = NORMAL;
                }
                continue;
            }
        }

        switch (state) {
            case NORMAL:
                state          = WARNING;
                state_start_ms = now_ms;
                break;

            case WARNING:
                if (now_ms - state_start_ms >= WARNING_MS) {
                    state          = ALERT;
                    state_start_ms = now_ms;
                    ESP_LOGW(TAG, "vitals warning escalated to ALERT");
                }
                break;

            case ALERT:
                /* Pulse the buzzer at BUZZER_PERIOD_MS. */
                gpio_set_level(PIN_BUZZER, (now_ms / BUZZER_PERIOD_MS) % 2);
                if (now_ms - state_start_ms >= ALERT_MS) {
                    gpio_set_level(PIN_BUZZER, 0);
                    ESP_LOGW(TAG, "ALERT window expired; sending SMS "
                                 "(collision=%d)", collision_alert_active ? 1 : 0);
                    send_alert_sms(&snap, collision_alert_active);
                                        /* Re-page limiter: while this condition sustains, the
                     * next SMS can only go out after SMS_REPAGE_COOLDOWN_MS.
                     * The WARNING (silent) + ALERT (buzzer) cycle re-runs
                     * before each re-page, so the wearer gets ~10 s of
                     * local buzzer warning it's about to re-page. */
                    sms_cooldown_until = now_ms + SMS_REPAGE_COOLDOWN_MS;
                    state                  = NORMAL;
                    collision_alert_active = false;
                }
                break;
        }
    }
}
