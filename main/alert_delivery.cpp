/**
 * alert_delivery.cpp - see alert_delivery.h.
 *
 * One worker owns the whole SMS conversation. The monitor only enqueues.
 */
#include "alert_delivery.h"

#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "alert_message.h"
#include "at_command.h"
#include "config.h"
#include "diagnostics.h"
#include "secrets.h"
#include "state.h"

static const char *TAG = "axion.alert";

/* Delivery policy: three attempts, then give up on this alert. A failed alert
 * is not silenced: the monitor applies no cooldown, so a condition that is
 * still raising will queue another alert on its next cycle. */
#define ALERT_TX_ATTEMPTS 3
static const uint32_t k_backoff_ms[ALERT_TX_ATTEMPTS] = {0, 5000, 15000};

/* Recipients from secrets.h, in priority order; empty slots skipped. */
static const char *const k_phones[AXION_ALERT_PHONE_MAX_COUNT] = {
    ALERT_PHONE_1,
    ALERT_PHONE_2,
};

static SemaphoreHandle_t s_lock    = nullptr;   /* guards everything below */
static SemaphoreHandle_t s_work    = nullptr;   /* signal: work pending */
static uint32_t          s_pending_mask      = 0;
static bool              s_pending_collision = false;
static bool              s_has_pending       = false;
static bool              s_busy              = false;
static uint32_t          s_delivered_mask    = 0;

esp_err_t alert_delivery_init(void)
{
    if (s_lock == nullptr) {
        s_lock = xSemaphoreCreateMutex();
    }
    if (s_work == nullptr) {
        s_work = xSemaphoreCreateBinary();
    }
    return (s_lock != nullptr && s_work != nullptr) ? ESP_OK : ESP_ERR_NO_MEM;
}

bool alert_delivery_enqueue(uint32_t mask, bool collision)
{
    if (s_lock == nullptr || s_work == nullptr) {
        return false;
    }
    if (mask == 0u && !collision) {
        return false;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    s_pending_mask      |= mask;
    s_pending_collision  = s_pending_collision || collision;
    s_has_pending        = true;
    const bool was_busy  = s_busy;
    s_busy               = true;

    xSemaphoreGive(s_lock);

    /* Only wake the worker on the idle -> busy edge; otherwise it is already
     * running and will pick the pending alert up on its next pass. */
    if (!was_busy) {
        xSemaphoreGive(s_work);
    }
    return true;
}

bool alert_delivery_busy(void)
{
    if (s_lock == nullptr) {
        return false;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        /* Treat "can't tell" as busy: better to hold the monitor off than to
         * start a second cycle while a send may be in flight. */
        return true;
    }
    const bool busy = s_busy;
    xSemaphoreGive(s_lock);
    return busy;
}

uint32_t alert_delivery_take_delivered(void)
{
    if (s_lock == nullptr) {
        return 0u;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        return 0u;
    }
    const uint32_t mask = s_delivered_mask;
    s_delivered_mask = 0u;
    xSemaphoreGive(s_lock);
    return mask;
}

/* Build the body from the freshest snapshot and send it to every configured
 * recipient. Returns the number of recipients that accepted the message. */
static int send_to_recipients(const char *reason)
{
    axion_state_t snap = {};
    axion_state_snapshot(&snap);

    const int64_t now_ms    = esp_timer_get_time() / 1000;
    const int64_t fix_age_s = (snap.gnss_fix_ms > 0)
                                  ? (now_ms - snap.gnss_fix_ms) / 1000
                                  : -1;

    char msg[ALERT_MESSAGE_MAX_LEN];
    alert_message_format(msg, sizeof(msg), reason, snap.gnss_fix_valid,
                         snap.lat, snap.lon, snap.alt, snap.speed, fix_age_s,
                         snap.temp_c, snap.heart_rate, snap.spo2);

    int delivered = 0;
    for (int i = 0; i < AXION_ALERT_PHONE_MAX_COUNT; ++i) {
        if (k_phones[i] == nullptr || k_phones[i][0] == '\0') {
            continue;
        }
        if (at_command_send_sms_body(k_phones[i], msg)) {
            ++delivered;
            ESP_LOGI(TAG, "alert delivered to %s", k_phones[i]);
        } else {
            ESP_LOGE(TAG, "alert delivery FAILED to %s", k_phones[i]);
        }
    }
    return delivered;
}

static bool deliver_with_retry(uint32_t mask, bool collision)
{
    if (!at_modem_uart_ready()) {
        /* Nothing to retry: the driver is not installed, so no AT command can
         * succeed this boot. Say so loudly instead of burning 20 s. */
        ESP_LOGE(TAG, "UART not initialised - alert cannot be delivered");
        return false;
    }

    const char *reason = alert_reason_text(mask, collision);

    for (int attempt = 0; attempt < ALERT_TX_ATTEMPTS; ++attempt) {
        if (k_backoff_ms[attempt] != 0u) {
            ESP_LOGW(TAG, "alert attempt %d/%d for '%s' in %u ms",
                     attempt + 1, ALERT_TX_ATTEMPTS, reason,
                     (unsigned)k_backoff_ms[attempt]);
            vTaskDelay(pdMS_TO_TICKS(k_backoff_ms[attempt]));
        }
        if (send_to_recipients(reason) > 0) {
            return true;
        }
        ESP_LOGE(TAG, "alert attempt %d/%d failed ('%s')",
                 attempt + 1, ALERT_TX_ATTEMPTS, reason);
    }
    return false;
}

void alert_delivery_task(void * /*arg*/)
{
    for (;;) {
        if (xSemaphoreTake(s_work, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        for (;;) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            const uint32_t mask        = s_pending_mask;
            const bool     collision   = s_pending_collision;
            const bool     has_pending = s_has_pending;
            s_pending_mask      = 0u;
            s_pending_collision = false;
            s_has_pending       = false;
            xSemaphoreGive(s_lock);

            if (!has_pending) {
                xSemaphoreTake(s_lock, portMAX_DELAY);
                s_busy = false;
                xSemaphoreGive(s_lock);
                break;
            }

            ESP_LOGW(TAG, "delivering alert (%s)", alert_reason_text(mask, collision));
            const bool ok = deliver_with_retry(mask, collision);
            /* Survives a reset: the next boot can say whether we managed to
             * tell anyone. */
            diagnostics_note_alert(mask, collision, ok);

            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (ok) {
                /* The monitor polls this and only then starts the re-page
                 * cooldown, so a failed send never silences a condition. */
                s_delivered_mask |= mask;
            }
            const bool more = s_has_pending;
            if (!more) {
                s_busy = false;
            }
            xSemaphoreGive(s_lock);

            if (!more) {
                break;
            }
        }
    }
}
