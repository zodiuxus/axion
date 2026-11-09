/**
 * diagnostics.cpp - see diagnostics.h.
 */
#include "diagnostics.h"

#include <cstring>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char *TAG = "axion.diag";

static const char *NVS_NAMESPACE = "diag";
static const char *NVS_KEY_LAST_ALERT = "last_alert";

/* RTC no-init memory: survives resets (including panics and watchdog resets)
 * but not a power cycle, which is exactly the window a reset loop lives in. */
#define BOOT_MAGIC 0xA5C0DE01u
RTC_NOINIT_ATTR static uint32_t s_boot_magic;
RTC_NOINIT_ATTR static uint32_t s_boot_count;
/* This boot's number, kept separately because diagnostics_mark_steady()
 * zeroes the counter: an alert must record the boot it was raised in. */
static uint32_t s_boot_id;

/* Bumped if the layout below ever changes. */
typedef struct {
    uint32_t mask;       /* conditions that were paged (bitmask of alert_src_t) */
    uint32_t boot;       /* boot counter value when the alert was raised */
    uint32_t delivered;  /* 1 if at least one recipient accepted the message */
    uint32_t collision;  /* 1 if the alert was collision-triggered */
} last_alert_record_t;

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_UNKNOWN:   return "unknown";
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "external-pin";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "interrupt-wdt";
        case ESP_RST_TASK_WDT:  return "task-wdt";
        case ESP_RST_WDT:       return "other-wdt";
        case ESP_RST_DEEPSLEEP: return "deep-sleep";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_SDIO:      return "sdio";
        default:                return "other";
    }
}

static void log_last_alert(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }

    last_alert_record_t rec = {};
    size_t len = sizeof(rec);
    const esp_err_t err = nvs_get_blob(handle, NVS_KEY_LAST_ALERT, &rec, &len);
    nvs_close(handle);

    if (err != ESP_OK || len != sizeof(rec)) {
        return;
    }

    ESP_LOGW(TAG, "last alert: mask=0x%lx collision=%lu delivered=%lu "
                  "(raised during boot #%lu)",
             (unsigned long)rec.mask, (unsigned long)rec.collision,
             (unsigned long)rec.delivered, (unsigned long)rec.boot);
}

void diagnostics_boot_note(void)
{
    if (s_boot_magic != BOOT_MAGIC) {
        /* First boot since power was applied. */
        s_boot_magic = BOOT_MAGIC;
        s_boot_count = 1;
    } else if (s_boot_count < 0xFFFFu) {
        ++s_boot_count;
    }
    s_boot_id = s_boot_count;

    const esp_reset_reason_t reason = esp_reset_reason();
    const uint32_t heap = (uint32_t)esp_get_free_heap_size();
    const uint32_t heap_min = (uint32_t)esp_get_minimum_free_heap_size();

    if (s_boot_count > 2u) {
        ESP_LOGE(TAG, "boot #%lu without reaching steady state (reason=%s/%d) - "
                      "possible reset loop; heap=%lu min=%lu",
                 (unsigned long)s_boot_count, reset_reason_name(reason),
                 (int)reason, (unsigned long)heap, (unsigned long)heap_min);
    } else {
        ESP_LOGI(TAG, "boot #%lu reason=%s/%d heap=%lu min=%lu",
                 (unsigned long)s_boot_count, reset_reason_name(reason),
                 (int)reason, (unsigned long)heap, (unsigned long)heap_min);
    }

    log_last_alert();
}

void diagnostics_mark_steady(void)
{
    if (s_boot_count > 1u) {
        ESP_LOGW(TAG, "steady state after %lu boot(s) since power-on",
                 (unsigned long)s_boot_count);
    }
    /* Next reset starts counting from 1 again; a boot loop never gets here. */
    s_boot_count = 0;
}

void diagnostics_note_alert(uint32_t mask, bool collision, bool delivered)
{
    last_alert_record_t rec = {};
    rec.mask      = mask;
    rec.boot      = s_boot_id;
    rec.delivered = delivered ? 1u : 0u;
    rec.collision = collision ? 1u : 0u;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for last-alert failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(handle, NVS_KEY_LAST_ALERT, &rec, sizeof(rec));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "last-alert record failed: %s", esp_err_to_name(err));
    }
}
