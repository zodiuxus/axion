/**
 * collision.cpp - see collision.h for design notes.
 *
 * Implementation: pure helper called from mpu_int_task on every
 * data-ready interrupt. Computes the magnitude vector and applies
 * threshold + cooldown; returns true if the caller should set
 * BIT_COLLISION_DETECTED.
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
#include "config.h"

bool collision_check(float ax, float ay, float az,
                     int64_t now_ms, int64_t *last_ms)
{
    if (last_ms == nullptr) return false;

    float mag = sqrtf(ax * ax + ay * ay + az * az);

    if (mag < COLLISION_THRESHOLD_G) {
        return false;
    }

    /* Cooldown: suppress retrigger within the cooldown window so a
     * single physical impact (which rings through the structure for
     * tens of ms) doesn't fire multiple events. */
    if (now_ms - *last_ms < (int64_t)COLLISION_COOLDOWN_MS) {
        return false;
    }

    *last_ms = now_ms;
    return true;
}
