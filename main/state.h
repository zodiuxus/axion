#pragma once
/**
 * state.h - Centralized, mutex-protected sensor state.
 *
 * All sensor-producing tasks (MPU, GNSS, DS18B20, MAX30102) write into a
 * single `axion_state_t` struct under `state_mutex`. All consumer tasks
 * (monitor, debug print) read through `axion_state_snapshot()` to get a
 * consistent point-in-time copy.
 *
 * This replaces the previous design where bare `inline` globals were
 * sprinkled across logic.h and accessed without synchronization.
 */
#ifndef AXION_STATE_H
#define AXION_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bit flags for the `sensors_ready` event group. Each subsystem sets its
 * bit when it has finished initialization and is producing fresh data.
 *
 * Bits 0–4 are "ready" bits (level-triggered, set once and left set).
 * Bits 5–6 are "calibrated" bits (level-triggered, set once calibration
 *         completes or is skipped on non-first-run boots).
 * Bit  7   is the alert-abort signal (edge-triggered, consumed by the
 *         monitor task via xEventGroupWaitBits with xClearOnExit=pdTRUE).
 * Bit  8   is the collision-detected signal (edge-triggered, consumed by
 *         the monitor task via xEventGroupWaitBits with xClearOnExit=pdTRUE).
 *         Set by collision_task when raw accel magnitude >= COLLISION_THRESHOLD_G. */
#define BIT_AT_READY        (1 << 0)   /* A7670E modem ready (SMS available) */
#define BIT_MPU_READY       (1 << 1)   /* MPU6050 DMP streaming */
#define BIT_GNSS_READY      (1 << 2)   /* GNSS producing fixes (may be stale) */
#define BIT_TEMP_READY      (1 << 3)   /* DS18B20 producing readings */
#define BIT_MAX30102_READY  (1 << 4)   /* MAX30102 initialized */

#define BIT_TEMP_CALIBRATED (1 << 5)   /* DS18B20 calibration phase complete */
#define BIT_OXIM_CALIBRATED (1 << 6)   /* MAX30102 calibration phase complete */

#define BIT_ALERT_ABORT     (1 << 7)   /* Alert-abort button pressed (edge) */

#define BIT_COLLISION_DETECTED (1 << 8) /* Raw-accel collision detected (edge) */

typedef struct {
    /* Orientation (MPU6050 DMP). yaw, pitch, roll in radians. */
    float ypr[3];

    /* Position / motion (GNSS). lat/lon/alt/speed hold the LAST VALID fix -
     * they are only overwritten when a new fix is actually acquired, never
     * zeroed out just because the current poll came back with no lock (see
     * gnss_fix_valid below). */
    double lat;         /* degrees, +N - last known valid fix */
    double lon;         /* degrees, +E - last known valid fix */
    float  alt;         /* meters - last known valid fix */
    float  speed;       /* m/s, already converted from knots - last known valid fix */
    bool   gnss_fix_valid;  /* true if the MOST RECENT poll had an actual lock.
                              * false means lat/lon/alt/speed above are stale -
                              * last known position, not current. */
    int64_t gnss_fix_ms;    /* esp_timer_get_time()/1000 at the last valid fix.
                              * 0 if never had a fix this boot. Callers compute
                              * staleness as (now_ms - gnss_fix_ms). */

    /* Body temperature (DS18B20). */
    float temp_c;
    float temp_baseline;   /* calibrated normal body temp (°C); TEMP_BASELINE_DEFAULT until calibrated */
    bool temp_baseline_valid; /* true only once a calibrated baseline was
                                 loaded from NVS or measured - until then,
                                 the monitor MUST NOT treat the temp as abnormal */

    /* Oximetry (MAX30102). */
    int   heart_rate;   /* bpm */
    float spo2;         /* percent */
    bool  spo2_valid;   /* false if correlation too low or finger off */
    uint32_t oxim_seq;  /* incremented on EVERY completed analysis window
                         * (valid or not). The monitor polls state every
                         * 100 ms but must evaluate each 5.12 s window
                         * exactly once - a change in this counter marks
                         * a fresh window, so per-window debouncing
                         * counts windows, not polls. */

} axion_state_t;

/* Initialize the state mutex + event group. Call once from app_main. */
void axion_state_init(void);

/* Block until all of the given ready-bits are set. */
void axion_state_wait_all(uint32_t bits);

/* Block until any of the given ready-bits are set. */
void axion_state_wait_any(uint32_t bits);

/* Set ready-bits (used by sensor init tasks). */
void axion_state_set_ready(uint32_t bits);

/* ---- System arming gate ---------------------------------------------- */
/* Collision escalation is suppressed until modem bring-up completes.
 * Before that, a physical bump (mounting the box, bench handling) would
 * fire an immediate ALERT that can't even be sent - the monitor task
 * itself doesn't start until BIT_AT_READY. Detection still runs from
 * boot and is logged by mpu_int_task; escalation becomes live when the
 * modem task calls axion_state_set_armed(true). Lock-free (atomic):
 * polled on every MPU interrupt wake. */
void axion_state_set_armed(bool armed);
bool axion_state_is_armed(void);

/* Take a consistent snapshot of the shared state. Safe from any task. */
void axion_state_snapshot(axion_state_t *out);

/* Update individual fields. Each call acquires the mutex briefly.
 * Use these from sensor tasks instead of poking fields directly. */
void axion_state_set_ypr(const float ypr[3]);
/* Record a VALID GNSS fix - updates position and marks gnss_fix_valid=true,
 * gnss_fix_ms=now. Call this only when the modem actually reported a lock. */
void axion_state_set_gnss(double lat, double lon, float alt, float speed);
/* Record that the most recent GNSS poll came back with NO fix. Leaves
 * lat/lon/alt/speed untouched (they keep the last known valid position) but
 * clears gnss_fix_valid so consumers (e.g. the alert SMS) know the position
 * they're about to report is stale. */
void axion_state_note_gnss_no_fix(void);
void axion_state_set_temp(float temp_c);
void axion_state_set_temp_baseline(float baseline);
void axion_state_set_oximetry(int heart_rate, float spo2, bool valid);

/* Handles (defined in state.cpp). */
extern EventGroupHandle_t g_sensors_ready;

#ifdef __cplusplus
}
#endif

#endif /* AXION_STATE_H */
