#pragma once
/**
 * mpu.h - MPU6050 setup + DMP yaw/pitch/roll task + raw accel helper.
 */
#ifndef AXION_MPU_H
#define AXION_MPU_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configure I2C_NUM_0 master. Synchronous; deletes the calling task. */
void i2c_bus_setup(void);

/** Initialize MPU6050 + DMP, then set BIT_MPU_READY on success.
 *  Synchronous; deletes the calling task. */
void mpu_setup(void);

/** FreeRTOS task: continuously reads YPR from the DMP FIFO and pushes
 *  it into the shared state. */
void mpu_rpy_task(void *arg);

/** Read raw accelerometer X/Y/Z from the MPU6050 and convert to g units
 *  under the configured full-scale range (±4g -> 8192 LSB/g).
 *
 *  Safe to call from any task - the ESP-IDF I2C driver serializes the
 *  underlying bus transaction, so concurrent calls from mpu_rpy_task
 *  (DMP FIFO reads) and collision_task (raw accel reads) are fine.
 *
 *  @param ax, ay, az  Out: acceleration in g. May be NULL.
 *  @return true on a successful I2C read, false on bus error. */
bool mpu_get_raw_accel_g(float *ax, float *ay, float *az);

#ifdef __cplusplus
}
#endif

#endif /* AXION_MPU_H */
