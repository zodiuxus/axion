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

/* Event-group bits in g_sensors_ready. Level-triggered unless marked edge.
 * The #define block below is the authoritative list - keep it in sync. */
#define BIT_AT_READY        (1 << 0)   /* A7670E registered: SMS path usable */
#define BIT_MPU_READY       (1 << 1)   /* MPU initialised (DMP or collision-only) */
#define BIT_GNSS_READY      (1 << 2)   /* GNSS commands sent (not "has fixes") */
#define BIT_TEMP_READY      (1 << 3)   /* DS18B20 producing readings */
#define BIT_MAX30102_READY  (1 << 4)   /* MAX30102 initialized */
#define BIT_TEMP_CALIBRATED (1 << 5)   /* temp calibration complete; gates the
                                        * first-run commit, not alerting */
#define BIT_OXIM_CALIBRATED (1 << 6)   /* first oximetry window analysed */
#define BIT_ALERT_ABORT     (1 << 7)   /* button pressed (edge, clear-on-exit) */
#define BIT_COLLISION_DETECTED (1 << 8) /* raw-accel collision (edge, clear-on-exit) */
#define BIT_TEMP_MIN_REACHED (1 << 9)  /* probe in contact: a plausible reading
                                        * has been seen (plausibility floor) */
/* No DS18B20 on the bus at boot. Temperature calibration can never succeed in
 * that case, so the calibration commit is allowed to proceed without a stored
 * baseline. */
#define BIT_TEMP_SENSOR_ABSENT (1 << 10)
/* Shared I2C0 bus installed (i2c_bus_setup). Set by the bus owner, not by the
 * MPU initialiser: a sensor on that bus must not depend on an unrelated device
 *. */
#define BIT_I2C_READY       (1 << 11)

typedef struct {
    /* Orientation (MPU6050 DMP). yaw, pitch, roll in radians. */
    float ypr[3];
    /* ms timestamp of the last ypr update (esp_timer, 0 = never). Consumers
     * must treat data older than YPR_STALE_MS as unknown. */
    int64_t ypr_ms;

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
    int64_t temp_ms;      /* ms timestamp of the last temp_c update; 0 = never */
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
    int64_t  oxim_ms;   /* ms timestamp of the last completed window; 0 = never */

} axion_state_t;

/* Initialize the state mutex + event group. Call once from app_main. */
void axion_state_init(void);

/* Block until all of the given ready-bits are set. */
void axion_state_wait_all(uint32_t bits);

/* Block until all of the given bits are set, or the timeout elapses.
 * @return true if every bit was set. */
bool axion_state_wait_all_timeout(uint32_t bits, uint32_t timeout_ms);

/* Set ready-bits (used by sensor init tasks). */
void axion_state_set_ready(uint32_t bits);

/* ---- System arming gate ---------------------------------------------- */
/* Collision ESCALATION is suppressed until the monitor task starts
 * (mpu_int_task checks this on every wake; set by monitor_task).
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
