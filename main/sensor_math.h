#pragma once
/**
 * sensor_math.h - Pure conversions shared by the sensor code. Header-only so
 * the arithmetic can be host-tested.
 */
#ifndef AXION_SENSOR_MATH_H
#define AXION_SENSOR_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Rescale an accelerometer offset from the ±2g calibration domain into
 * `afs_sel`'s range: offset registers are applied in the current full scale, so
 * the same physical correction is a smaller raw count at a wider range.
 * @param afs_sel  0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
 */
static inline int16_t accel_offset_for_fs(int16_t offset_2g, uint8_t afs_sel)
{
    const int divisor = 1 << (afs_sel & 0x3u);
    return (int16_t)(offset_2g / divisor);
}

#ifdef __cplusplus
}
#endif

#endif /* AXION_SENSOR_MATH_H */
