#pragma once
/**
 * calibration_store.h - NVS-backed persistent calibration state.
 *
 * Flags: a uint32 bitfield in namespace "axion", key "cal_flags".
 *   bit 0  CAL_FLAG_FIRST_RUN          needs calibration on next boot
 *   bit 1  CAL_FLAG_TEMP_BASELINE_VALID temp baseline stored
 *   bit 2  CAL_FLAG_MPU_OFFSETS_VALID  MPU offsets stored
 * Other values (baseline float, MPU offsets blob, carrier/APN strings)
 * live under their own keys; missing keys return ESP_ERR_NVS_NOT_FOUND.
 */
#ifndef AXION_CALIBRATION_STORE_H
#define AXION_CALIBRATION_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAL_FLAG_FIRST_RUN           = (1u << 0),
    CAL_FLAG_TEMP_BASELINE_VALID = (1u << 1),  /* temp baseline stored in NVS */
    CAL_FLAG_MPU_OFFSETS_VALID   = (1u << 2),  /* MPU offsets stored in NVS */
    /* Reserved:
     * CAL_FLAG_OXIM_OFFSET    = (1u << 3),
     */
} cal_flag_t;

/** Initialize NVS and load the cached flags. Call once from app_main
 *  before any other calibration_store function. */
esp_err_t calibration_store_init(void);

/** Read all flags as a uint32 (the raw hex value). */
uint32_t calibration_store_get_flags(void);

/** Check if a specific flag is set. */
bool calibration_store_is_flag_set(uint32_t flag);

/** Set one or more flags (OR into the stored value). Persists immediately. */
esp_err_t calibration_store_set_flag(uint32_t flag);

/** Clear one or more flags (AND-NOT out of the stored value). Persists immediately. */
esp_err_t calibration_store_clear_flag(uint32_t flag);

/** Mark the device factory-fresh: sets CAL_FLAG_FIRST_RUN and
 *  invalidates the stored MPU offsets (they are orientation-dependent). */
esp_err_t calibration_store_request_factory_reset(void);

/* ---- Temperature baseline (float) ------------------------------------ */
/* Key "temp_base": written after first-run calibration, read by the
 * monitor for baseline ± delta thresholds. */

/** Read the stored baseline. Returns ESP_ERR_NVS_NOT_FOUND if no
 *  baseline has been stored yet (first boot). */
esp_err_t calibration_store_get_temp_baseline(float *out);

/** Store the baseline and set CAL_FLAG_TEMP_BASELINE_VALID. */
esp_err_t calibration_store_set_temp_baseline(float val);

/* ---- MPU6050 accel + gyro offsets (blob of 6 x int16) ----------------- */
/* Key "mpu_offs": [xa, ya, za, xg, yg, zg] raw register values.
 * Written once by mpu_setup(); later boots just rewrite them into the
 * (volatile) offset registers. Accel offsets are orientation-dependent:
 * re-mounting requires re-calibration (factory reset invalidates). */

/** Read the stored offsets. Returns ESP_ERR_NVS_NOT_FOUND if none have
 *  been stored yet (first boot). `out` receives 6 int16 values. */
esp_err_t calibration_store_get_mpu_offsets(int16_t out[6]);

/** Store the offsets and set CAL_FLAG_MPU_OFFSETS_VALID. */
esp_err_t calibration_store_set_mpu_offsets(const int16_t val[6]);

/* ---- Carrier + APN (strings) ----------------------------------------- */
/* Seeded from secrets.h on first boot, then NVS is the source of truth.
 * A missing key (ESP_ERR_NVS_NOT_FOUND) means "use the secrets.h
 * default". */

/** Read a string secret from NVS. `out` is NUL-terminated.
 *  Returns ESP_ERR_NVS_NOT_FOUND if the key hasn't been set yet.
 *  Caller must provide a buffer of at least `out_size` bytes. */
esp_err_t calibration_store_get_str(const char *key, char *out, size_t out_size);

/** Write a string secret to NVS. Persists immediately. */
esp_err_t calibration_store_set_str(const char *key, const char *val);

#ifdef __cplusplus
}
#endif

#endif /* AXION_CALIBRATION_STORE_H */
