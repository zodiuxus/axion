/**
 * collision.cpp - see collision.h.
 *
 * The magnitude includes gravity (~1 g at rest): the threshold is total
 * acceleration. To detect "2 g above gravity" instead, raise the
 * threshold to ~3 g in config.h.
 */
#include "collision.h"

#include <cmath>
#include "config.h"

bool collision_check(float ax, float ay, float az,
                     int64_t now_ms, int64_t *last_ms)
{
    if (last_ms == nullptr) return false;

    float mag = sqrtf(ax * ax + ay * ay + az * az);

    if (mag < COLLISION_THRESHOLD_G) {
        return false;
    }

    /* Cooldown: one physical impact fires only one event. */
    if (now_ms - *last_ms < (int64_t)COLLISION_COOLDOWN_MS) {
        return false;
    }

    *last_ms = now_ms;
    return true;
}
