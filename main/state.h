#pragma once
/**
 * state.h - Centralized, mutex-protected sensor state.
 *
 * Sensor tasks write via the axion_state_set_* accessors; consumers read
 * consistent copies via axion_state_snapshot().
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

/* Event-group bits in g_sensors_ready:
 *   0-4  READY       - level-triggered, set once init completes
 *   5-6  CALIBRATED  - level-triggered, set once calibration completes/skips
 *   7    ALERT_ABORT - edge-triggered, consumed by monitor (clear-on-exit)
 *   8    COLLISION   - edge-triggered, consumed by monitor (clear-on-exit) */
#define BIT_AT_READY        (1 << 0)   /* A7670E modem ready (SMS available) */
#define BIT_MPU_READY       (1 << 1)   /* MPU6050 DMP streaming */
#define BIT_GNSS_READY      (1 << 2)   /* GNSS producing fixes (may be stale) */
#define BIT_TEMP_READY      (1 << 3)   /* DS18B20 producing readings */
#define BIT_MAX30102_READY  (1 << 4)   /* MAX30102 initialized */

#define BIT_TEMP_CALIBRATED (1 << 5)   /* temp phase complete (gates monitor arming) */
#define BIT_OXIM_CALIBRATED (1 << 6)   /* MAX30102 calibration phase complete */

#define BIT_ALERT_ABORT     (1 << 7)   /* Alert-abort button pressed (edge) */

#define BIT_COLLISION_DETECTED (1 << 8) /* Raw-accel collision detected (edge) */
#define BIT_TEMP_MIN_REACHED (1 << 9)

typedef struct {
    /* Orientation (MPU6050 DMP). yaw, pitch, roll in radians. */
    float ypr[3];

    /* Position / motion (GNSS). lat/lon/alt/speed hold the LAST VALID fix
     * and are only overwritten when a new fix is acquired. */
    double lat;         /* degrees, +N */
    double lon;         /* degrees, +E */
    float  alt;         /* meters */
    float  speed;       /* m/s (converted from knots) */
    bool   gnss_fix_valid;  /* true only if the most recent poll had a lock */
    int64_t gnss_fix_ms;    /* ms timestamp of the last valid fix; 0 = never */

    /* Body temperature (DS18B20). */
    float temp_c;
    float temp_baseline;   /* calibrated normal body temp (°C) */
    bool temp_baseline_valid; /* false until a calibrated baseline exists;
                                * while false the monitor ignores the temp */

    /* Oximetry (MAX30102). */
    int   heart_rate;   /* bpm */
    float spo2;         /* percent */
    bool  spo2_valid;   /* false if correlation too low or finger off */
    uint32_t oxim_seq;  /* bumped on EVERY completed analysis window
                         * (valid or not) - lets the 100 ms monitor poll
                         * count 5.12 s windows exactly once each */

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
/* Collision ESCALATION is suppressed until modem bring-up completes
 * (mpu_int_task checks this on every wake; set by modem_setup_task).
 * Lock-free atomic. */
void axion_state_set_armed(bool armed);
bool axion_state_is_armed(void);

/* Take a consistent snapshot of the shared state. Safe from any task. */
void axion_state_snapshot(axion_state_t *out);

/* Update individual fields. Each call acquires the mutex briefly. */
void axion_state_set_ypr(const float ypr[3]);
/* Record a VALID GNSS fix (updates position, marks fresh). */
void axion_state_set_gnss(double lat, double lon, float alt, float speed);
/* Mark the most recent poll as no-fix: position fields are left untouched
 * (stale but known); consumers see gnss_fix_valid == false. */
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
