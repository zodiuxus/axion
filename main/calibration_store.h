#pragma once
/**
 * calibration_store.h — NVS-backed persistent bitfield for calibration
 * and factory-reset state.
 *
 * Storage format: a single uint32 in NVS namespace "axion", key "cal_flags".
 * Each bit is a boolean flag (see cal_flag_t). This is the "hex value"
 * approach the user described — compact, easy to extend (just OR in more
 * bits), and native to NVS. In logs it shows as e.g. 0x00000001.
 *
 * Why NVS instead of a literal file: NVS is the ESP-IDF-native way to
 * persist small settings. It's robust to power loss, wear-leveled, and
 * doesn't require pulling in a filesystem. If you later want to migrate
 * to LittleFS for human-readable files, the change is localized to this
 * module — callers only see cal_flag_t / get / set / clear.
 *
 * Current bit assignments:
 *   bit 0  CAL_FLAG_FIRST_RUN    1 = device needs calibration on next boot
 *                                 (set at factory or via factory_reset)
 *   bits 1–31  reserved for future use (temp offset valid, oxim offset
 *              valid, etc. — "we'll add those options in detail later on")
 */
#ifndef AXION_CALIBRATION_STORE_H
#define AXION_CALIBRATION_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAL_FLAG_FIRST_RUN           = (1u << 0),
    CAL_FLAG_TEMP_BASELINE_VALID = (1u << 1),  /* temp baseline stored in NVS */
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
 *  Sets CAL_FLAG_FIRST_RUN. The actual trigger (long button press,
 *  serial command, etc.) will be wired up later. */
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

#ifdef __cplusplus
}
#endif

#endif /* AXION_CALIBRATION_STORE_H */
