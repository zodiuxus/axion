/**
 * collision.cpp - see collision.h for design notes.
 *
 * Implementation outline:
 *   1. Wait for BIT_MPU_READY (so the MPU is initialized and I2C is up).
 *   2. Every COLLISION_SAMPLE_MS, read raw accel in g via mpu_get_raw_accel_g().
 *   3. Compute magnitude |a| = sqrt(ax^2 + ay^2 + az^2).
 *   4. If |a| >= COLLISION_THRESHOLD_G and we're outside the cooldown
 *      window, set BIT_COLLISION_DETECTED and log the impact details.
 *
 * The magnitude includes gravity (~1 g at rest). The user's "2G or more"
 * threshold is therefore interpreted as total acceleration experienced,
 * which is the standard interpretation for impact detection: at rest the
 * device reads ~1 g, and any real impact pushes the total well past 2 g.
 * If you want to detect "2 g above gravity" instead, raise the threshold
 * to ~3 g (1 g gravity + 2 g impact) in config.h.
 */
#include "collision.h"

#include <cmath>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "state.h"
#include "mpu.h"

static const char *TAG = "axion.collision";

void collision_task(void * /*arg*/)
{
    /* The MPU setup task sets BIT_MPU_READY once the accel range is
     * configured and the DMP is running. Don't poll before that - the
     * I2C bus may not even be installed yet. */
    axion_state_wait_all(BIT_MPU_READY);

    ESP_LOGI(TAG, "collision detector started: threshold=%.2f g, sample=%u ms",
             (double)COLLISION_THRESHOLD_G, COLLISION_SAMPLE_MS);

    int64_t last_trigger_ms = 0;   /* suppresses retrigger within cooldown */

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COLLISION_SAMPLE_MS));

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (!mpu_get_raw_accel_g(&ax, &ay, &az)) {
            /* MPU not ready (shouldn't happen after the wait above, but
             * guard anyway in case the device was reset). */
            continue;
        }

        float mag = sqrtf(ax * ax + ay * ay + az * az);

        if (mag < COLLISION_THRESHOLD_G) {
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_trigger_ms < (int64_t)COLLISION_COOLDOWN_MS) {
            /* Still inside the cooldown from a recent trigger - drop this
             * event to avoid spamming the monitor with one physical impact. */
            continue;
        }

        last_trigger_ms = now_ms;

        ESP_LOGW(TAG, "COLLISION: |a|=%.2f g (x=%.2f y=%.2f z=%.2f) -> ALERT",
                 (double)mag, (double)ax, (double)ay, (double)az);

        /* Edge-triggered signal: monitor task consumes this via
         * xEventGroupWaitBits with xClearOnExit=pdTRUE, same pattern as
         * BIT_ALERT_ABORT. */
        xEventGroupSetBits(g_sensors_ready, BIT_COLLISION_DETECTED);
    }
}
