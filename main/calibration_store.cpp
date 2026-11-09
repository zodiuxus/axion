/**
 * calibration_store.cpp - see calibration_store.h for design notes.
 */
#include "calibration_store.h"

#include <cstring>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "axion.cal";

static const char *NVS_NAMESPACE = "axion";
static const char *NVS_KEY       = "cal_flags";
static const char *NVS_KEY_TEMP_BASELINE = "temp_base";
static const char *NVS_KEY_MPU_OFFSETS   = "mpu_offs";

static uint32_t s_cached_flags = 0;
static bool     s_initialized  = false;
/* Guards s_cached_flags and every read-modify-write of it. Four tasks touch
 * the flags (temperature, oximetry, MPU setup, status LED); without this the
 * plain |= / &= sequences lose updates and persist the loser. */
static SemaphoreHandle_t s_flags_lock = nullptr;

static bool flags_lock(void)
{
    if (s_flags_lock == nullptr) {
        return false;
    }
    return xSemaphoreTake(s_flags_lock, pdMS_TO_TICKS(1000)) == pdTRUE;
}

static void flags_unlock(void)
{
    if (s_flags_lock != nullptr) {
        xSemaphoreGive(s_flags_lock);
    }
}

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

    if (s_flags_lock == nullptr) {
        s_flags_lock = xSemaphoreCreateMutex();
        if (s_flags_lock == nullptr) {
            ESP_LOGE(TAG, "flags lock allocation failed - flag updates are racy");
        }
    }

    nvs_handle_t h;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        err = nvs_get_u32(h, NVS_KEY, &s_cached_flags);
        nvs_close(h);
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "calibration flags = 0x%08lx", (unsigned long)s_cached_flags);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* Missing key = first-ever boot. */
        s_cached_flags = CAL_FLAG_FIRST_RUN;
        ESP_LOGI(TAG, "NVS key not found; defaulting to first-run=true (0x%08lx)",
                 (unsigned long)s_cached_flags);
    } else {
        /* Corrupt, unreadable or wrong-typed flag word: treat the device as
         * unprovisioned instead of pretending it is calibrated. The old code
         * fell through with flags == 0, which is "calibrated, no baseline"
         * and disabled temperature alerting for good. */
        s_cached_flags = CAL_FLAG_FIRST_RUN;
        ESP_LOGE(TAG, "NVS flag read failed (%s); defaulting to first-run=true",
                 esp_err_to_name(err));
    }

    s_initialized = true;
    return ESP_OK;
}

bool calibration_store_is_flag_set(uint32_t flag)
{
    if (!flags_lock()) {
        /* Never block an alerting path on the store: a torn read here only
         * affects first-run bookkeeping, not sensor data. */
        return (s_cached_flags & flag) != 0;
    }
    const bool set = (s_cached_flags & flag) != 0;
    flags_unlock();
    return set;
}

/* Caller must hold s_flags_lock: the cached word and the persisted word are
 * updated as one step so two writers cannot lose each other's bits. */
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
    if (!flags_lock()) {
        ESP_LOGE(TAG, "set flag 0x%08lx: flags lock timeout", (unsigned long)flag);
        return ESP_ERR_TIMEOUT;
    }
    s_cached_flags |= flag;
    ESP_LOGI(TAG, "set flag 0x%08lx -> flags now 0x%08lx",
             (unsigned long)flag, (unsigned long)s_cached_flags);
    const esp_err_t err = persist_flags();
    flags_unlock();
    return err;
}

esp_err_t calibration_store_clear_flag(uint32_t flag)
{
    if (!flags_lock()) {
        ESP_LOGE(TAG, "clear flag 0x%08lx: flags lock timeout", (unsigned long)flag);
        return ESP_ERR_TIMEOUT;
    }
    s_cached_flags &= ~flag;
    ESP_LOGI(TAG, "clear flag 0x%08lx -> flags now 0x%08lx",
             (unsigned long)flag, (unsigned long)s_cached_flags);
    const esp_err_t err = persist_flags();
    flags_unlock();
    return err;
}

esp_err_t calibration_store_request_factory_reset(void)
{
    ESP_LOGW(TAG, "factory reset requested - first_run flag will be set");
    esp_err_t err = calibration_store_set_flag(CAL_FLAG_FIRST_RUN);
    /* Invalidate MPU offsets too: they are orientation-dependent. */
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

    /* Storing a baseline marks it valid (see header). */
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

    /* Length guard against future format changes (CRC handles decay). */
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
