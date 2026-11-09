/**
 * alert_policy_test.cpp - host-side unit tests for the alerting policy.
 *
 * These run on the build machine (no ESP32, no ESP-IDF): the policy is pure
 * logic and depends only on config_tuning.h. See tests/README.md.
 */
#include "test_util.h"
#include "alert_policy.h"

namespace {

/* Healthy, idle subject: everything fresh, nothing abnormal. oxim_seq is 0 so
 * the first evaluation does not consume a window unless a test says so. */
alert_inputs_t healthy()
{
    alert_inputs_t in = {};
    in.imu_fresh           = true;
    in.roll_deg            = 0.0f;
    in.gnss_fresh          = true;
    in.speed_mps           = 0.0f;
    in.temp_fresh          = true;
    in.temp_baseline_valid = true;
    in.temp_c              = 36.6f;
    in.temp_baseline       = 36.6f;
    in.oxim_fresh          = true;
    in.oxim_seq            = 0;
    in.spo2_valid          = true;
    in.heart_rate          = 70;
    in.spo2                = 97.0f;
    return in;
}

alert_decision_t eval(alert_policy_t *p, const alert_inputs_t *in,
                      int64_t now_ms = 1000)
{
    return alert_policy_eval(p, in, now_ms);
}

void test_idle_is_quiet()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    alert_decision_t d = eval(&p, &in);
    CHECK(!d.any);
    CHECK(alert_decision_mask(&d) == 0u);
}

void test_fall_raised_when_stopped()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.roll_deg = 170.0f;
    alert_decision_t d = eval(&p, &in);
    CHECK(d.raise[ALERT_SRC_FALL]);
    CHECK(d.any);
}

void test_fall_suppressed_when_moving_with_fresh_speed()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.roll_deg  = 170.0f;
    in.speed_mps = 1.4f;   /* walking */
    alert_decision_t d = eval(&p, &in);
    CHECK(!d.raise[ALERT_SRC_FALL]);
}

/* A stale fix leaves the last speed value behind. That value must not be
 * treated as "still moving" -- unknown is not moving, so a fall alarms. */
void test_fall_raised_when_gnss_is_stale()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.roll_deg   = 170.0f;
    in.speed_mps  = 1.4f;   /* stale leftover from the last fix */
    in.gnss_fresh = false;
    alert_decision_t d = eval(&p, &in);
    CHECK(d.raise[ALERT_SRC_FALL]);
}

/* A stopped producer must not be able to raise an alert. */
void test_stale_imu_cannot_raise_fall()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.roll_deg  = 179.0f;
    in.imu_fresh = false;
    alert_decision_t d = eval(&p, &in);
    CHECK(!d.raise[ALERT_SRC_FALL]);
    CHECK(!d.any);
}

void test_temperature_high_raised()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.temp_c = 38.2f;   /* baseline 36.6 + 1.0 */
    alert_decision_t d = eval(&p, &in);
    CHECK(d.raise[ALERT_SRC_TEMP_HIGH]);
    CHECK(!d.raise[ALERT_SRC_TEMP_LOW]);
}

/* The calibrated baseline is the only reference. A reading 0.7 C below it is
 * hypothermia regardless of its absolute value. */
void test_temperature_low_raised_relative_to_baseline()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();   /* baseline 36.6 */
    in.temp_c = 35.0f;               /* 1.6 C below baseline */
    alert_decision_t d = eval(&p, &in);
    CHECK(d.raise[ALERT_SRC_TEMP_LOW]);
}

/* The band moves with the baseline: the same reading can be normal for one
 * person and hypothermic for another. */
void test_temperature_band_follows_baseline()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();

    in.temp_baseline = 37.2f;
    in.temp_c        = 36.3f;   /* 0.9 below a 37.2 baseline -> hypo */
    alert_decision_t d = eval(&p, &in);
    CHECK(d.raise[ALERT_SRC_TEMP_LOW]);

    in.temp_baseline = 36.2f;
    in.temp_c        = 36.3f;   /* inside the band of a 36.2 baseline */
    d = eval(&p, &in);
    CHECK(!d.raise[ALERT_SRC_TEMP_LOW]);
    CHECK(!d.raise[ALERT_SRC_TEMP_HIGH]);
}

/* Documented consequence of the decision: an off-body probe reads ambient and
 * is therefore reported as hypothermia. If that proves too noisy in the field,
 * the fix is a contact signal or a longer debounce, not an absolute floor
 * (which is what hid real hypothermia). */
void test_temperature_off_body_reads_as_hypothermia()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.temp_c = 22.0f;
    alert_decision_t d = eval(&p, &in);
    CHECK(d.raise[ALERT_SRC_TEMP_LOW]);
}

/* Same for the temperature channel: a frozen last reading is the most
 * dangerous stale value (it is plausible by construction). */
void test_stale_temperature_ignored()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.temp_c     = 39.0f;
    in.temp_fresh = false;
    alert_decision_t d = eval(&p, &in);
    CHECK(!d.raise[ALERT_SRC_TEMP_HIGH]);
}

void test_vitals_require_sustained_windows()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.heart_rate = 130;   /* above HR_HIGH_THRESHOLD */

    for (uint32_t win = 1; win <= (uint32_t)VITALS_SUSTAIN_WINDOWS; ++win) {
        in.oxim_seq = win;
        alert_decision_t d = eval(&p, &in);
        const bool expected = (win >= (uint32_t)VITALS_SUSTAIN_WINDOWS);
        CHECK(d.raise[ALERT_SRC_VITALS] == expected);
    }
}

void test_vitals_normal_window_resets_streak()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.heart_rate = 130;
    in.oxim_seq = 1;
    alert_policy_eval(&p, &in, 1000);

    in.heart_rate = 70;     /* normal window resets the streak */
    in.oxim_seq = 2;
    alert_policy_eval(&p, &in, 1000);

    in.heart_rate = 130;
    in.oxim_seq = 3;
    alert_decision_t d = alert_policy_eval(&p, &in, 1000);
    CHECK(!d.raise[ALERT_SRC_VITALS]);
}

void test_vitals_counts_each_window_once()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.heart_rate = 130;
    in.oxim_seq   = 1;

    for (int i = 0; i < 10; ++i) {
        (void)alert_policy_eval(&p, &in, 1000);   /* same window, ten ticks */
    }
    CHECK(p.vitals_streak == 1);
}

void test_stale_oxim_cannot_raise_vitals()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.heart_rate = 130;
    for (uint32_t win = 1; win <= (uint32_t)VITALS_SUSTAIN_WINDOWS; ++win) {
        in.oxim_seq = win;
        (void)alert_policy_eval(&p, &in, 1000);
    }
    in.oxim_fresh = false;
    alert_decision_t d = alert_policy_eval(&p, &in, 1000);
    CHECK(!d.raise[ALERT_SRC_VITALS]);
}

/* Paging one condition must not silence a different one. */
void test_cooldown_is_per_condition()
{
    alert_policy_t p;
    alert_policy_init(&p);

    /* Establish a vitals alert and page it. */
    alert_inputs_t in = healthy();
    in.heart_rate = 130;
    for (uint32_t win = 1; win <= (uint32_t)VITALS_SUSTAIN_WINDOWS; ++win) {
        in.oxim_seq = win;
        (void)alert_policy_eval(&p, &in, 1000);
    }
    alert_decision_t d = alert_policy_eval(&p, &in, 1000);
    CHECK(d.raise[ALERT_SRC_VITALS]);
    alert_policy_note_paged(&p, alert_decision_mask(&d), 1000);

    /* The vitals condition is now in cooldown... */
    d = alert_policy_eval(&p, &in, 2000);
    CHECK(!d.raise[ALERT_SRC_VITALS]);

    /* ...but a fall is a different condition and must still be reported. */
    in.roll_deg = 170.0f;
    in.speed_mps = 0.0f;
    d = alert_policy_eval(&p, &in, 3000);
    CHECK(d.raise[ALERT_SRC_FALL]);
    CHECK(d.any);
}

void test_cooldown_expires()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.roll_deg = 170.0f;

    alert_decision_t d = alert_policy_eval(&p, &in, 0);
    CHECK(d.raise[ALERT_SRC_FALL]);
    alert_policy_note_paged(&p, alert_decision_mask(&d), 0);

    d = alert_policy_eval(&p, &in, (int64_t)SMS_REPAGE_COOLDOWN_MS - 1);
    CHECK(!d.raise[ALERT_SRC_FALL]);

    d = alert_policy_eval(&p, &in, (int64_t)SMS_REPAGE_COOLDOWN_MS + 1);
    CHECK(d.raise[ALERT_SRC_FALL]);
}

void test_abort_silences_everything_then_expires()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.roll_deg = 170.0f;
    in.temp_c   = 39.0f;

    alert_policy_note_abort(&p, 1000);

    alert_decision_t d = alert_policy_eval(&p, &in, 1001);
    CHECK(!d.any);

    d = alert_policy_eval(&p, &in, 1000 + (int64_t)ABORT_COOLDOWN_MS - 1);
    CHECK(!d.any);

    d = alert_policy_eval(&p, &in, 1000 + (int64_t)ABORT_COOLDOWN_MS + 1);
    CHECK(d.raise[ALERT_SRC_FALL]);
    CHECK(d.raise[ALERT_SRC_TEMP_HIGH]);
}

/* The movement gate exists for falls; it must not suppress vitals. */
void test_moving_does_not_suppress_vitals()
{
    alert_policy_t p;
    alert_policy_init(&p);
    alert_inputs_t in = healthy();
    in.speed_mps  = 1.5f;   /* walking */
    in.gnss_fresh = true;
    in.heart_rate = 130;
    for (uint32_t win = 1; win <= (uint32_t)VITALS_SUSTAIN_WINDOWS; ++win) {
        in.oxim_seq = win;
        (void)alert_policy_eval(&p, &in, 1000);
    }
    alert_decision_t d = alert_policy_eval(&p, &in, 1000);
    CHECK(d.raise[ALERT_SRC_VITALS]);
}

}  // namespace

void run_alert_policy_tests()
{
    test_idle_is_quiet();
    test_fall_raised_when_stopped();
    test_fall_suppressed_when_moving_with_fresh_speed();
    test_fall_raised_when_gnss_is_stale();
    test_stale_imu_cannot_raise_fall();
    test_temperature_high_raised();
    test_temperature_low_raised_relative_to_baseline();
    test_temperature_band_follows_baseline();
    test_temperature_off_body_reads_as_hypothermia();
    test_stale_temperature_ignored();
    test_vitals_require_sustained_windows();
    test_vitals_normal_window_resets_streak();
    test_vitals_counts_each_window_once();
    test_stale_oxim_cannot_raise_vitals();
    test_cooldown_is_per_condition();
    test_cooldown_expires();
    test_abort_silences_everything_then_expires();
    test_moving_does_not_suppress_vitals();
}
