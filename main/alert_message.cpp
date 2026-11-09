/**
 * alert_message.cpp - see alert_message.h.
 */
#include "alert_message.h"

#include <cstdio>

const char *alert_reason_text(uint32_t mask, bool collision)
{
    if (collision) {
        return "collision detected";
    }
    if ((mask & (1u << ALERT_SRC_FALL)) != 0u) {
        return "fall detected";
    }
    if ((mask & ((1u << ALERT_SRC_TEMP_LOW) | (1u << ALERT_SRC_TEMP_HIGH))) != 0u) {
        return "body temperature abnormal";
    }
    if ((mask & (1u << ALERT_SRC_VITALS)) != 0u) {
        return "vitals abnormal";
    }
    return "alert";
}

void alert_message_format(char *out, size_t out_size,
                          const char *reason,
                          bool fix_valid,
                          double lat, double lon, float alt, float speed_mps,
                          int64_t fix_age_s,
                          float temp_c, int heart_rate, float spo2)
{
    if (out == nullptr || out_size == 0) {
        return;
    }

    const char *why = (reason != nullptr) ? reason : "alert";

    if (fix_valid) {
        snprintf(out, out_size,
                 "AXION ALERT: %s. "
                 "lat=%.6f lon=%.6f alt=%.1f spd=%.1fm/s "
                 "temp=%.2fC hr=%d spo2=%.1f%%",
                 why, lat, lon, (double)alt, (double)speed_mps,
                 (double)temp_c, heart_rate, (double)spo2);
    } else if (fix_age_s >= 0) {
        snprintf(out, out_size,
                 "AXION ALERT: %s. "
                 "lat=%.6f lon=%.6f (LAST KNOWN, %llds old) alt=%.1f "
                 "temp=%.2fC hr=%d spo2=%.1f%%",
                 why, lat, lon, (long long)fix_age_s, (double)alt,
                 (double)temp_c, heart_rate, (double)spo2);
    } else {
        /* Never had a fix this boot at all - don't send a bogus 0,0. */
        snprintf(out, out_size,
                 "AXION ALERT: %s. "
                 "NO GPS FIX YET "
                 "temp=%.2fC hr=%d spo2=%.1f%%",
                 why, (double)temp_c, heart_rate, (double)spo2);
    }
}
