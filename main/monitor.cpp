/**
 * monitor.cpp - Fall, temperature, vitals and collision monitor.
 *
 * NORMAL -> WARNING (WARNING_MS) -> ALERT (ALERT_MS, buzzer) -> SMS.
 * alert_policy decides, alert_delivery sends; this file owns the sequence, the
 * buzzer and the abort button.
 *
 * BIT_ALERT_ABORT cancels to NORMAL and arms the abort cooldown.
 * BIT_COLLISION_DETECTED skips WARNING; a collision ALERT is sticky.
 */
#include "monitor.h"

#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "alert_delivery.h"
#include "alert_message.h"
#include "alert_policy.h"
#include "config.h"
#include "diagnostics.h"
#include "state.h"

static const char *TAG = "axion.monitor";

/* Snapshot -> policy inputs, applying the freshness limits: a field whose
 * producer stopped is marked not fresh. */
static void build_inputs(const axion_state_t *s, int64_t now_ms,
                         alert_inputs_t *in)
{
    in->imu_fresh = (s->ypr_ms > 0) &&
                    ((now_ms - s->ypr_ms) <= (int64_t)YPR_STALE_MS);
    in->roll_deg  = s->ypr[2] * 180.0f / (float)M_PI;

    in->gnss_fresh = s->gnss_fix_valid && (s->gnss_fix_ms > 0) &&
                     ((now_ms - s->gnss_fix_ms) <= (int64_t)GNSS_STALE_MS);
    in->speed_mps  = s->speed;

    in->temp_fresh          = (s->temp_ms > 0) &&
                              ((now_ms - s->temp_ms) <= (int64_t)TEMP_STALE_MS);
    in->temp_baseline_valid = s->temp_baseline_valid;
    in->temp_c              = s->temp_c;
    in->temp_baseline       = s->temp_baseline;

    in->oxim_fresh = (s->oxim_ms > 0) &&
                     ((now_ms - s->oxim_ms) <= (int64_t)OXIM_STALE_MS);
    in->oxim_seq   = s->oxim_seq;
    in->spo2_valid = s->spo2_valid;
    in->heart_rate = s->heart_rate;
    in->spo2       = s->spo2;
}

/* Freshness transitions while running; GNSS is excluded (no fix indoors is
 * normal and the poller reports it). */
enum { LIV_IMU = 0, LIV_TEMP, LIV_OXIM, LIV_COUNT };
static const char *const k_liveness_name[LIV_COUNT] = {
    "IMU", "temperature", "oximetry",
};

static void log_liveness(const alert_inputs_t *in, int64_t now_ms)
{
    static bool    s_prev[LIV_COUNT]        = {true, true, true};
    static int64_t s_stale_since[LIV_COUNT] = {0, 0, 0};
    static bool    s_seen                   = false;

    const bool fresh[LIV_COUNT] = {
        in->imu_fresh, in->temp_fresh, in->oxim_fresh,
    };

    if (!s_seen) {
        for (int i = 0; i < LIV_COUNT; ++i) {
            s_prev[i] = fresh[i];
        }
        s_seen = true;
        return;
    }

    for (int i = 0; i < LIV_COUNT; ++i) {
        if (s_prev[i] && !fresh[i]) {
            s_stale_since[i] = now_ms;
            ESP_LOGW(TAG, "%s data stale - channel suspended until it recovers",
                     k_liveness_name[i]);
        } else if (!s_prev[i] && fresh[i]) {
            const int64_t outage_ms =
                (s_stale_since[i] > 0) ? (now_ms - s_stale_since[i]) : 0;
            ESP_LOGW(TAG, "%s data recovered after %lld ms",
                     k_liveness_name[i], (long long)outage_ms);
        }
        s_prev[i] = fresh[i];
    }
}

void monitor_task(void * /*arg*/)
{
    /* Start even if a sensor never arrives: freshness gates each channel, so a
     * degraded start is safe. */
    const uint32_t required = BIT_MPU_READY | BIT_AT_READY | BIT_TEMP_MIN_REACHED;
    const EventBits_t ready = xEventGroupWaitBits(g_sensors_ready, required,
                                                  pdFALSE /* don't clear */,
                                                  pdTRUE  /* wait for all */,
                                                  pdMS_TO_TICKS(ARM_TIMEOUT_MS));
    const uint32_t missing = required & ~((uint32_t)ready);
    if (missing != 0u) {
        ESP_LOGE(TAG, "arming incomplete after %u ms (missing bits 0x%x) - "
                      "starting degraded", (unsigned)ARM_TIMEOUT_MS,
                 (unsigned)missing);
    }

    /* Collision escalation is allowed from here on, independent of the modem. */
    axion_state_set_armed(true);
    diagnostics_mark_steady();

    /* Drop a pre-arm abort edge so it is not replayed as a fresh press. */
    xEventGroupClearBits(g_sensors_ready, BIT_ALERT_ABORT);
    ESP_LOGI(TAG, "monitor started (mpu=%d modem=%d probe_min=%d)",
             (int)((ready & BIT_MPU_READY) != 0),
             (int)((ready & BIT_AT_READY) != 0),
             (int)((ready & BIT_TEMP_MIN_REACHED) != 0));

    gpio_reset_pin(PIN_BUZZER);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);

    typedef enum { NORMAL, WARNING, ALERT } state_t;
    state_t state                  = NORMAL;
    int64_t state_start_ms         = 0;
    /* True while the current ALERT is collision-triggered (sticky). */
    bool    collision_alert_active = false;

    alert_policy_t policy;
    alert_policy_init(&policy);

    axion_state_t snap = {};

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
            alert_policy_note_abort(&policy, now_ms);
            state                  = NORMAL;
            collision_alert_active = false;
            continue;
        }

        /* Only a delivered alert starts the re-page cooldown; a failed send
         * stays free to re-alert. */
        uint32_t delivered = alert_delivery_take_delivered();
        if (delivered != 0u) {
            ESP_LOGI(TAG, "alert delivered (0x%x) - starting re-page cooldown",
                     (unsigned)delivered);
            alert_policy_note_paged(&policy, delivered, now_ms);
        }

        /* Fresh snapshot for the decision. */
        axion_state_snapshot(&snap);

        alert_inputs_t in = {};
        build_inputs(&snap, now_ms, &in);
        log_liveness(&in, now_ms);

        uint32_t prev_seq    = policy.last_oxim_seq;
        int      prev_streak = policy.vitals_streak;
        alert_decision_t decision = alert_policy_eval(&policy, &in, now_ms);
        uint32_t raised = alert_decision_mask(&decision);

        /* Observability: one line per out-of-range window that extends the
         * streak. */
        if ((policy.last_oxim_seq != prev_seq) &&
            (policy.vitals_streak > prev_streak)) {
            ESP_LOGW(TAG, "vitals window out of range: hr=%d bpm spo2=%.1f%% "
                          "(streak %d/%d)",
                     snap.heart_rate, (double)snap.spo2,
                     policy.vitals_streak, VITALS_SUSTAIN_WINDOWS);
        }

        /* ---- Collision: bypass WARNING, jump straight to ALERT. ------- */
        if (collision) {
            state                  = ALERT;
            state_start_ms         = now_ms;
            collision_alert_active = true;
            ESP_LOGW(TAG, "collision detected - escalating directly to ALERT "
                         "(WARNING skipped)");
            /* Fall through: the ALERT case handles buzzer + SMS. */
        } else if (!collision_alert_active &&
                   (!decision.any || alert_delivery_busy())) {
            /* Nothing raising, or the previous alert is still being delivered:
             * settle back to NORMAL. A collision ALERT is sticky and skips this
             * check. */
            if (state != NORMAL) {
                gpio_set_level(PIN_BUZZER, 0);
                state = NORMAL;
            }
            continue;
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
                    ESP_LOGW(TAG, "warning escalated to ALERT (%s)",
                             alert_reason_text(raised, false));
                }
                break;

            case ALERT:
                /* Pulse the buzzer at BUZZER_PERIOD_MS. */
                gpio_set_level(PIN_BUZZER, (now_ms / BUZZER_PERIOD_MS) % 2);
                if (now_ms - state_start_ms >= ALERT_MS) {
                    const char *reason =
                        alert_reason_text(raised, collision_alert_active);
                    gpio_set_level(PIN_BUZZER, 0);
                    ESP_LOGW(TAG, "ALERT window expired; queueing SMS (%s)", reason);
                    /* The delivery task owns the UART and retries; this never
                     * blocks. */
                    if (!alert_delivery_enqueue(raised, collision_alert_active)) {
                        ESP_LOGE(TAG, "ALERT NOT QUEUED (delivery unavailable)");
                    }
                    state                  = NORMAL;
                    collision_alert_active = false;
                }
                break;
        }
    }
}
