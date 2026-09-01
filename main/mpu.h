#pragma once
/**
 * mpu.h - MPU6050 setup + interrupt-driven DMP/collision task.
 *
 * The MPU6050 has a single INT pin that ORs together all enabled
 * interrupt sources (data-ready, FIFO, motion, free-fall, DMP). We
 * enable the data-ready interrupt and let one ISR-driven task handle
 * both DMP packet processing AND raw-accel collision detection - there
 * is no separate collision_task anymore.
 *
 * Flow:
 *   MPU INT (GPIO7) ─► ISR ─► notifies mpu_int_task
 *                                ├─ reads INT_STATUS
 *                                ├─ reads raw accel → collision check
 *                                └─ if DMP packet ready: reads FIFO → YPR
 *
 * The data-ready interrupt fires at the sensor sample rate (200 Hz -
 * see mpu_setup(), where SMPLRT_DIV is set to 4). The DMP writes its
 * packet to the FIFO at the same 200 Hz, so each interrupt typically
 * services one DMP packet + one raw accel sample.
 */
#ifndef AXION_MPU_H
#define AXION_MPU_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configure I2C_NUM_0 master. Synchronous; deletes the calling task. */
void i2c_bus_setup(void);

/** Initialize MPU6050 + DMP, configure the data-ready interrupt on
 *  PIN_MPU_INT, then set BIT_MPU_READY on success.
 *  On the first boot (no stored offsets), runs the PID accel+gyro
 *  calibration and persists the six offset register values to NVS; on
 *  every later boot it just restores them - no "keep still" needed,
 *  and setup is ~1-2 s faster. Synchronous; deletes the calling task. */
void mpu_setup(void);

/** FreeRTOS task: wakes on PIN_MPU_INT rising edge, reads INT_STATUS,
 *  drains the DMP FIFO for YPR, and checks raw-accel magnitude for
 *  collision. Runs forever. */
void mpu_int_task(void *arg);

/** Read raw accelerometer X/Y/Z from the MPU6050 and convert to g units
 *  under the configured full-scale range (±4g -> 8192 LSB/g).
 *
 *  Safe to call from any task - the ESP-IDF I2C driver serializes the
 *  underlying bus transaction. (Currently only called from mpu_int_task,
 *  but kept as a public helper for future console/debug commands.)
 *
 *  @param ax, ay, az  Out: acceleration in g. May be NULL.
 *  @return true on a successful I2C read, false on bus error. */
bool mpu_get_raw_accel_g(float *ax, float *ay, float *az);

#ifdef __cplusplus
}
#endif

#endif /* AXION_MPU_H */
