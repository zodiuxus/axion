#pragma once
/**
 * config_tuning.h - Static tuning constants: thresholds, timings, debounce
 * windows and freshness limits. No ESP-IDF includes (the consumers are
 * host-testable); wiring lives in config_pins.h.
 */
#ifndef AXION_CONFIG_TUNING_H
#define AXION_CONFIG_TUNING_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Calibration -------------------------------------------------------- */
/* First-run temperature window (ms), only when CAL_FLAG_FIRST_RUN is set. The
 * oximeter needs no calibration window. */
#define TEMP_CALIBRATION_MS     120000U     /* 2 minutes */

/* Fail-open: a probe that never reads releases the arming gate after this. */
#define TEMP_FIRST_READ_TIMEOUT_MS  30000U
/* Stored baselines below this are implausible for skin. */
#define TEMP_BASELINE_PLAUSIBLE_MIN_C  30.0f

/* Status-LED timeout: reach COMPLETE even if calibration never finishes. The
 * 120 s window counts on-body samples, so this is not a duration guarantee
 *. */
#define LED_CALIBRATION_TIMEOUT_MS  240000U

/* ---- Status LED blink timing -------------------------------------------- */
#define LED_BOOT_ON_MS          25U
#define LED_BOOT_OFF_MS         75U
#define LED_CALIB_ON_MS         50U
#define LED_CALIB_OFF_MS        150U
#define LED_COMPLETE_MS         2000U

/* ---- Collision detection (raw accelerometer) ----------------------------- */
/* Total magnitude, gravity included. The accel runs at ±4g, so 2 g is
 * measured, not clipped. */
#define COLLISION_THRESHOLD_G   2.0f
#define COLLISION_COOLDOWN_MS   3000U

/* ---- Tuning constants ----------------------------------------------------- */
/* Fallen if |roll| >= this (native ±180 deg range). */
#define FALL_ANGLE_THRESHOLD    150.0f

/* Vitals warning / alert timing (ms). */
#define WARNING_MS              5000U
#define ALERT_MS                5000U
#define BUZZER_PERIOD_MS        500U

/* Deltas from the calibrated baseline, the only alerting reference. The default
 * is just the initial shared-state value. */
#define TEMP_BASELINE_DEFAULT   36.2f
#define TEMP_HYPO_DELTA         0.7f
#define TEMP_HYPER_DELTA        1.0f

/* HR/SpO2: wide band, uncalibrated estimator. */
#define HR_LOW_THRESHOLD        40      /* bpm */
#define HR_HIGH_THRESHOLD       120     /* bpm */
#define SPO2_LOW_THRESHOLD      90.0f   /* % */
/* Consecutive abnormal valid windows (5.12 s each): 3 = ~15 s sustained. */
#define VITALS_SUSTAIN_WINDOWS  3
/* Suppress re-paging the same condition while it sustains. */
#define SMS_REPAGE_COOLDOWN_MS  300000U

/* Speed thresholds (m/s). */
#define SPEED_STOPPED           0.5f

/* GNSS polling interval (ms). */
#define GNSS_POLL_MS            5000U

/* MAX30102 (must match MAX30102_ALG_SAMPLE_MS). */
#define MAX30102_SAMPLE_MS      40U
#define MAX30102_BUFFER_SIZE    128

/* Alert recipients configured (driven by secrets.h). */
#define AXION_ALERT_PHONE_MAX_COUNT 2

/* Cooldown after a button abort before re-entering WARNING. */
#define ABORT_COOLDOWN_MS       10000U

/* ---- Arming -------------------------------------------------------------- */
/* Start the monitor at most this late; a missing sensor only disables its own
 * channel via freshness. Must exceed the modem's 45 s wait. */
#define ARM_TIMEOUT_MS          60000U

/* ---- Alert-abort button -------------------------------------------------- */
#define BUTTON_DEBOUNCE_MS      50U

/* How long an I2C consumer waits for i2c_bus_setup. */
#define I2C_READY_TIMEOUT_MS    10000U

/* ---- Liveness ------------------------------------------------------------ */
/* No MPU interrupt for this long means the INT line or DMP stalled. */
#define MPU_ALIVE_TIMEOUT_MS    250U

/* ---- Data freshness ------------------------------------------------------ */
/* A consumer must not act on a field older than its limit; chosen well above
 * the nominal period so one missed cycle does not trip them. */
#define YPR_STALE_MS            500U      /* 200 Hz source */
#define TEMP_STALE_MS           5000U     /* 1 Hz source */
#define OXIM_STALE_MS           15000U    /* 5.12 s analysis window */
#define GNSS_STALE_MS           30000U    /* 5 s poll */

#ifdef __cplusplus
}
#endif

#endif /* AXION_CONFIG_TUNING_H */
