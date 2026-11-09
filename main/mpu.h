#pragma once
/**
 * mpu.h - MPU6050 setup + interrupt-driven DMP/collision task.
 *
 * The single INT pin (200 Hz data-ready) drives one ISR-task that does
 * both the DMP YPR reads and raw-accel collision detection.
 */
#ifndef AXION_MPU_H
#define AXION_MPU_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configure I2C_NUM_0 master. Synchronous; deletes the calling task. */
void i2c_bus_setup(void);

/** Initialize MPU6050 + DMP, arm the data-ready interrupt on
 *  PIN_MPU_INT, set BIT_MPU_READY. Calibrates and persists offsets on
 *  the first boot; restores them afterwards. Deletes the calling task. */
void mpu_setup(void);

/** FreeRTOS task: wakes on PIN_MPU_INT edges; drains DMP FIFO for YPR
 *  and checks raw-accel magnitude for collision. Runs forever. */
void mpu_int_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_MPU_H */
