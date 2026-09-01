/**
 * calibration_store.cpp - see calibration_store.h for design notes.
 */
#include "calibration_store.h"

#include <cstring>
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "axion.cal";

static const char *NVS_NAMESPACE = "axion";
static const char *NVS_KEY       = "cal_flags";
static const char *NVS_KEY_TEMP_BASELINE = "temp_base";
static const char *NVS_KEY_MPU_OFFSETS   = "mpu_offs";

static uint32_t s_cached_flags = 0;
static bool     s_initialized  = false;

esp_err_t calibration_store_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase + reinit");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_handle_t h;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        err = nvs_get_u32(h, NVS_KEY, &s_cached_flags);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "NVS read failed: %s", esp_err_to_name(err));
        }
        nvs_close(h);
    }
    /* If the namespace or key doesn't exist yet (first-ever boot), treat
     * it as a first run - the flag defaults to set. */
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_cached_flags = CAL_FLAG_FIRST_RUN;
        ESP_LOGI(TAG, "NVS key not found; defaulting to first-run=true (0x%08lx)",
                 (unsigned long)s_cached_flags);
    } else {
        ESP_LOGI(TAG, "calibration flags = 0x%08lx", (unsigned long)s_cached_flags);
    }

    s_initialized = true;
    return ESP_OK;
}

uint32_t calibration_store_get_flags(void)
{
    return s_cached_flags;
}

bool calibration_store_is_flag_set(uint32_t flag)
{
    return (s_cached_flags & flag) != 0;
}

static esp_err_t persist_flags(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open(RW) failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_u32(h, NVS_KEY, s_cached_flags);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t calibration_store_set_flag(uint32_t flag)
{
    s_cached_flags |= flag;
    ESP_LOGI(TAG, "set flag 0x%08lx -> flags now 0x%08lx",
             (unsigned long)flag, (unsigned long)s_cached_flags);
    return persist_flags();
}

esp_err_t calibration_store_clear_flag(uint32_t flag)
{
    s_cached_flags &= ~flag;
    ESP_LOGI(TAG, "clear flag 0x%08lx -> flags now 0x%08lx",
             (unsigned long)flag, (unsigned long)s_cached_flags);
    return persist_flags();
}

esp_err_t calibration_store_request_factory_reset(void)
{
    ESP_LOGW(TAG, "factory reset requested - first_run flag will be set");
    esp_err_t err = calibration_store_set_flag(CAL_FLAG_FIRST_RUN);
    /* Also invalidate the stored MPU offsets: a factory reset should
     * force a sensor re-calibration too, and the accel offsets depend
     * on the mounting orientation (gravity baked in at rest). */
    if (err == ESP_OK) {
        err = calibration_store_clear_flag(CAL_FLAG_MPU_OFFSETS_VALID);
    }
    return err;
}

/* ---- Temperature baseline (float) ------------------------------------ */

esp_err_t calibration_store_get_temp_baseline(float *out)
{
    if (out == nullptr) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t sz = sizeof(float);
    err = nvs_get_blob(h, NVS_KEY_TEMP_BASELINE, out, &sz);

    nvs_close(h);
    return err;
}

esp_err_t calibration_store_set_temp_baseline(float val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_TEMP_BASELINE, &val, sizeof(float));
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    if (err != ESP_OK) return err;

    /* Keep the documented contract: storing a baseline marks it valid.
     * (Nothing reads this flag yet - callers probe with get + NOT_FOUND
     * - but the header promises the bit, so set it.) */
    return calibration_store_set_flag(CAL_FLAG_TEMP_BASELINE_VALID);
}

/* ---- MPU6050 accel + gyro offsets (blob of 6 x int16) ---------------- */

esp_err_t calibration_store_get_mpu_offsets(int16_t out[6])
{
    if (out == nullptr) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t sz = 6 * sizeof(int16_t);
    err = nvs_get_blob(h, NVS_KEY_MPU_OFFSETS, out, &sz);
    nvs_close(h);

    /* NVS per-entry CRCs already catch corruption; this length check
     * guards against a future format change writing a different size. */
    if (err == ESP_OK && sz != 6 * sizeof(int16_t)) {
        return ESP_ERR_NVS_INVALID_LENGTH;
    }
    return err;
}

esp_err_t calibration_store_set_mpu_offsets(const int16_t val[6])
{
    if (val == nullptr) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_MPU_OFFSETS, val, 6 * sizeof(int16_t));
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    return err;
}

/* ---- Carrier + APN (strings) ----------------------------------------- */

esp_err_t calibration_store_get_str(const char *key, char *out, size_t out_size)
{
    if (!s_initialized || key == nullptr || out == nullptr || out_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open(RO) for '%s' failed: %s", key, esp_err_to_name(err));
        return err;
    }

    /* First query the required length (including NUL). */
    size_t required = out_size;
    err = nvs_get_str(h, key, out, &required);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* Quiet: callers fall back to the secrets.h default. */
        return err;
    }
    if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGE(TAG, "NVS str '%s' needs %u bytes, buffer is %u",
                 key, (unsigned)required, (unsigned)out_size);
        return err;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS read str '%s' failed: %s", key, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t calibration_store_set_str(const char *key, const char *val)
{
    if (!s_initialized || key == nullptr || val == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open(RW) for '%s' failed: %s", key, esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(h, key, val);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write str '%s' failed: %s", key, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "str '%s' stored (len=%u)", key, (unsigned)std::strlen(val));
    }
    return err;
}
