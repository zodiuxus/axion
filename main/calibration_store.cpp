/**
 * calibration_store.cpp — see calibration_store.h for design notes.
 */
#include "calibration_store.h"

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "axion.cal";

static const char *NVS_NAMESPACE = "axion";
static const char *NVS_KEY       = "cal_flags";
static const char *NVS_KEY_TEMP_BASELINE = "temp_base";

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
     * it as a first run — the flag defaults to set. */
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
    ESP_LOGW(TAG, "factory reset requested — first_run flag will be set");
    return calibration_store_set_flag(CAL_FLAG_FIRST_RUN);
}

/* ---- Temperature baseline (float) ------------------------------------ */

esp_err_t calibration_store_get_temp_baseline(float *out)
{
    if (!s_initialized || out == nullptr) return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open(RO) for temp_base failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_get_f32(h, NVS_KEY_TEMP_BASELINE, out);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "no temp baseline in NVS (first boot)");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS read temp_base failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t calibration_store_set_temp_baseline(float val)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open(RW) for temp_base failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_f32(h, NVS_KEY_TEMP_BASELINE, val);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write temp_base failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Mark the baseline as valid so future boots know to load it. */
    calibration_store_set_flag(CAL_FLAG_TEMP_BASELINE_VALID);
    ESP_LOGI(TAG, "temp baseline stored: %.2f C", (double)val);
    return ESP_OK;
}
