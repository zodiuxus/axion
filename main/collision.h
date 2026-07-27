#pragma once
/**
 * collision.h - Raw-accelerometer collision detector.
 *
 * This is the highest-priority alert source in the system. It reads raw
 * accel X/Y/Z from the MPU6050 at ~100 Hz, computes the magnitude vector
 * |a| = sqrt(ax^2 + ay^2 + az^2) in g, and fires BIT_COLLISION_DETECTED
 * in the shared event group whenever |a| >= COLLISION_THRESHOLD_G.
 *
 * The monitor task consumes that bit and bypasses the NORMAL->WARNING
 * grace period, escalating directly to ALERT (immediate SMS path). This
 * means a real 2G+ impact begins the contacting sequence within tens of
 * milliseconds, vs. the 5 s WARNING delay used for fall/vitals detection.
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

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task: polls raw accel at COLLISION_SAMPLE_MS intervals and
 *  signals BIT_COLLISION_DETECTED on threshold exceedance. Runs forever. */
void collision_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_COLLISION_H */
