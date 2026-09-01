#pragma once
/**
 * calibration_store.h - NVS-backed persistent bitfield for calibration
 * and factory-reset state.
 *
 * Storage format: a single uint32 in NVS namespace "axion", key "cal_flags".
 * Each bit is a boolean flag (see cal_flag_t). This is the "hex value"
 * approach the user described - compact, easy to extend (just OR in more
 * bits), and native to NVS. In logs it shows as e.g. 0x00000001.
 *
 * Why NVS instead of a literal file: NVS is the ESP-IDF-native way to
 * persist small settings. It's robust to power loss, wear-leveled, and
 * doesn't require pulling in a filesystem. If you later want to migrate
 * to LittleFS for human-readable files, the change is localized to this
 * module - callers only see cal_flag_t / get / set / clear.
 *
 * Current bit assignments:
 *   bit 0  CAL_FLAG_FIRST_RUN    1 = device needs calibration on next boot
 *                                 (set at factory or via factory_reset)
 *   bit 1  CAL_FLAG_TEMP_BASELINE_VALID  temp baseline stored in NVS
 *   bit 2  CAL_FLAG_MPU_OFFSETS_VALID    MPU accel+gyro offsets in NVS
 *   bits 3–31  reserved for future use (oxim offset valid, etc. -
 *              "we'll add those options in detail later on")
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

/** Convenience: mark the device as factory-fresh (needs calibration).
 *  Sets CAL_FLAG_FIRST_RUN and also invalidates the stored MPU offsets
 *  (a factory reset should force a re-calibration of the sensor too -
 *  the board may have been re-mounted in a different orientation).
 *  The actual trigger (long button press, serial command, etc.) will be
 *  wired up later. */
esp_err_t calibration_store_request_factory_reset(void);

/* ---- Temperature baseline (float) ------------------------------------ */
/* Stored separately from the flag bitfield under key "temp_base".
 * Written by the temperature task after the first-run 2-minute
 * calibration window; read by the monitor to compute per-user hypo/
 * hyper thresholds (baseline ± TEMP_HYPO/HPYER_DELTA). */

/** Read the stored baseline. Returns ESP_ERR_NVS_NOT_FOUND if no
 *  baseline has been stored yet (first boot). */
esp_err_t calibration_store_get_temp_baseline(float *out);

/** Store the baseline and set CAL_FLAG_TEMP_BASELINE_VALID. */
esp_err_t calibration_store_set_temp_baseline(float val);

/* ---- MPU6050 accel + gyro offsets (blob of 6 x int16) ----------------- */
/* Stored under key "mpu_offs" as [xa, ya, za, xg, yg, zg] - the RAW
 * register values that CalibrateAccel()/CalibrateGyro() left in the
 * XA/YA/ZA_OFFS and XG/YG/ZG_OFFS_USR registers. Written once by
 * mpu_setup() on the first boot; on every later boot mpu_setup() just
 * rewrites them into the registers, so the "keep the device still"
 * PID calibration only ever runs once (the MPU6050 offset registers
 * are volatile and reset to zero on every power cycle - that is why
 * every boot previously re-calibrated).
 *
 * The accel offsets are orientation-dependent (they bake in gravity
 * for the mounting orientation), so they must be invalidated if the
 * board is re-mounted - see calibration_store_request_factory_reset(). */

/** Read the stored offsets. Returns ESP_ERR_NVS_NOT_FOUND if none have
 *  been stored yet (first boot). `out` receives 6 int16 values. */
esp_err_t calibration_store_get_mpu_offsets(int16_t out[6]);

/** Store the offsets and set CAL_FLAG_MPU_OFFSETS_VALID. */
esp_err_t calibration_store_set_mpu_offsets(const int16_t val[6]);

/* ---- Carrier + APN (strings) ----------------------------------------- */
/* Carrier-specific secrets (MCC+MNC, APN name/user/pass) are seeded
 * from secrets.h on first boot, then copied into NVS so they can be
 * updated at runtime without reflashing. NVS is the source of truth
 * after first boot; secrets.h is only the factory default.
 *
 * `cal_flag_t` has no per-field valid bits here - a missing key is
 * signaled by ESP_ERR_NVS_NOT_FOUND from the getter, and the modem
 * bring-up code falls back to the secrets.h value in that case. */

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
