#pragma once
/**
 * collision.h - Raw-accelerometer collision detector (helper API).
 *
 * Threshold + cooldown check called from mpu_int_task on every
 * data-ready interrupt (raw accel, not the DMP - the DMP filters out
 * exactly the transients a collision produces). A trigger makes the
 * caller set BIT_COLLISION_DETECTED; the monitor then escalates
 * straight to ALERT.
 */
#ifndef AXION_COLLISION_H
#define AXION_COLLISION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Check a single accel sample for collision.
 * @param ax, ay, az  Acceleration in g.
 * @param now_ms      Current time in ms.
 * @param last_ms     In/out: timestamp of the last accepted trigger.
 * @return true if |a| >= COLLISION_THRESHOLD_G and the cooldown elapsed.
 */
bool collision_check(float ax, float ay, float az,
                     int64_t now_ms, int64_t *last_ms);

#ifdef __cplusplus
}
#endif

#endif /* AXION_COLLISION_H */
