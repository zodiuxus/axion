#pragma once
/**
 * collision.h - Raw-accelerometer collision detector (helper API).
 *
 * Previously a standalone polling task. Now the threshold + cooldown
 * check is a pure helper called from mpu_int_task on every data-ready
 * interrupt. This avoids a second I2C consumer racing the DMP FIFO
 * read on the same bus.
 *
 * The monitor task consumes BIT_COLLISION_DETECTED (set by mpu_int_task
 * when this helper returns true) and bypasses the NORMAL->WARNING grace
 * period, escalating directly to ALERT (immediate SMS path).
 *
 * Why raw accel and not the DMP: the DMP output (yaw/pitch/roll) is a
 * filtered, integrated orientation estimate that deliberately smooths
 * out short transients - exactly the opposite of what we want here. A
 * collision is a sharp acceleration spike that the DMP would attenuate.
 * Reading the raw accel registers gives us the unfiltered instantaneous
 * force, which is what a 2G threshold needs to be meaningful.
 *
 * A cooldown (COLLISION_COOLDOWN_MS) prevents a single physical impact
 * (which may ring and oscillate through the mechanical structure for
 * tens of milliseconds) from firing multiple events.
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
 *
 * @param ax, ay, az    Acceleration in g (raw accel registers / 8192).
 * @param now_ms        Current time in ms (esp_timer_get_time()/1000).
 * @param last_ms       In/out: timestamp of the last accepted trigger.
 *                      Updated in place when this call triggers.
 * @return true if |a| >= COLLISION_THRESHOLD_G and the cooldown window
 *                      has elapsed (caller should set the event bit).
 */
bool collision_check(float ax, float ay, float az,
                     int64_t now_ms, int64_t *last_ms);

#ifdef __cplusplus
}
#endif

#endif /* AXION_COLLISION_H */
