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

    /* Position / motion (GNSS). */
    double lat;         /* degrees, +N */
    double lon;         /* degrees, +E */
    float  alt;         /* meters */
    float  speed;       /* m/s, already converted from knots */

    /* Body temperature (DS18B20). */
    float temp_c;
    float temp_baseline;   /* calibrated normal body temp (°C); TEMP_BASELINE_DEFAULT until calibrated */

    /* Oximetry (MAX30102). */
    int   heart_rate;   /* bpm */
    float spo2;         /* percent */
    bool  spo2_valid;   /* false if correlation too low or finger off */
} axion_state_t;

/* Initialize the state mutex + event group. Call once from app_main. */
void axion_state_init(void);

/* Block until all of the given ready-bits are set. */
void axion_state_wait_all(uint32_t bits);

/* Block until any of the given ready-bits are set. */
void axion_state_wait_any(uint32_t bits);

/* Set ready-bits (used by sensor init tasks). */
void axion_state_set_ready(uint32_t bits);

/* Take a consistent snapshot of the shared state. Safe from any task. */
void axion_state_snapshot(axion_state_t *out);

/* Update individual fields. Each call acquires the mutex briefly.
 * Use these from sensor tasks instead of poking fields directly. */
void axion_state_set_ypr(const float ypr[3]);
void axion_state_set_gnss(double lat, double lon, float alt, float speed);
void axion_state_set_temp(float temp_c);
void axion_state_set_temp_baseline(float baseline);
void axion_state_set_oximetry(int heart_rate, float spo2, bool valid);

/* Handles (defined in state.cpp). */
extern EventGroupHandle_t g_sensors_ready;

#ifdef __cplusplus
}
#endif

#endif /* AXION_STATE_H */
