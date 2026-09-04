/**
 * state.cpp - see state.h.
 */
#include "state.h"

#include <atomic>
#include <cstring>
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "config.h"   /* for TEMP_BASELINE_DEFAULT */

static const char *TAG = "axion.state";

static SemaphoreHandle_t s_state_mutex = nullptr;
static axion_state_t      s_state       = {};
EventGroupHandle_t        g_sensors_ready = nullptr;

extern "C" void axion_state_init(void)
{
    if (s_state_mutex == nullptr) {
        s_state_mutex = xSemaphoreCreateMutex();
    }
    if (g_sensors_ready == nullptr) {
        g_sensors_ready = xEventGroupCreate();
    }
    memset(&s_state, 0, sizeof(s_state));
    s_state.temp_baseline = TEMP_BASELINE_DEFAULT;  /* until calibration provides a real one */
    ESP_LOGI(TAG, "state initialized");
}

extern "C" void axion_state_wait_all(uint32_t bits)
{
    xEventGroupWaitBits(g_sensors_ready, bits, pdFALSE, pdTRUE, portMAX_DELAY);
}

extern "C" void axion_state_wait_any(uint32_t bits)
{
    xEventGroupWaitBits(g_sensors_ready, bits, pdFALSE, pdFALSE, portMAX_DELAY);
}

extern "C" void axion_state_set_ready(uint32_t bits)
{
    xEventGroupSetBits(g_sensors_ready, bits);
}

/* System arming gate - atomic, no mutex: written once by the modem task
 * and polled on every MPU interrupt wake (200 Hz). See state.h. */
static std::atomic<bool> s_armed{false};

extern "C" void axion_state_set_armed(bool armed)
{
    s_armed.store(armed);
}

extern "C" bool axion_state_is_armed(void)
{
    return s_armed.load();
}

extern "C" void axion_state_snapshot(axion_state_t *out)
{
    if (out == nullptr) return;
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(out, &s_state, sizeof(s_state));
        xSemaphoreGive(s_state_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
}

extern "C" void axion_state_set_ypr(const float ypr[3])
{
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_state.ypr[0] = ypr[0];
        s_state.ypr[1] = ypr[1];
        s_state.ypr[2] = ypr[2];
        xSemaphoreGive(s_state_mutex);
    }
}

extern "C" void axion_state_set_gnss(double lat, double lon, float alt, float speed)
{
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_state.lat            = lat;
        s_state.lon            = lon;
        s_state.alt            = alt;
        s_state.speed          = speed;
        s_state.gnss_fix_valid = true;
        s_state.gnss_fix_ms    = esp_timer_get_time() / 1000;
        xSemaphoreGive(s_state_mutex);
    }
}

extern "C" void axion_state_note_gnss_no_fix(void)
{
    /* Deliberately does NOT touch lat/lon/alt/speed - callers that read the
     * snapshot still get the last known valid position, just flagged stale. */
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_state.gnss_fix_valid = false;
        xSemaphoreGive(s_state_mutex);
    }
}

extern "C" void axion_state_set_temp(float temp_c)
{
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_state.temp_c = temp_c;
        xSemaphoreGive(s_state_mutex);
    }
}

extern "C" void axion_state_set_temp_baseline(float baseline)
{
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_state.temp_baseline = baseline;
        s_state.temp_baseline_valid = true;
        xSemaphoreGive(s_state_mutex);
    }
}

extern "C" void axion_state_set_oximetry(int heart_rate, float spo2, bool valid)
{
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_state.heart_rate = heart_rate;
        s_state.spo2       = spo2;
        s_state.spo2_valid = valid;
        s_state.oxim_seq++;
        xSemaphoreGive(s_state_mutex);
    }
}
