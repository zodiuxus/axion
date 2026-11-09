#pragma once
/**
 * alert_message.h - Pure alert text formatting (host-tested, no ESP-IDF).
 */
#ifndef AXION_ALERT_MESSAGE_H
#define AXION_ALERT_MESSAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "alert_policy.h"   /* alert_src_t + masks for the reason mapping */

#ifdef __cplusplus
extern "C" {
#endif

/** Longest SMS body this module produces (including the NUL). */
#define ALERT_MESSAGE_MAX_LEN 200

/** Short human-readable trigger, used in the body and in logs. */
const char *alert_reason_text(uint32_t mask, bool collision);

/** Format the alert SMS body. Truncation is safe.
 *  @param reason     from alert_reason_text()
 *  @param fix_valid  true if the position comes from a live fix
 *  @param fix_age_s  seconds since the last valid fix, < 0 if never fixed
 */
void alert_message_format(char *out, size_t out_size,
                          const char *reason,
                          bool fix_valid,
                          double lat, double lon, float alt, float speed_mps,
                          int64_t fix_age_s,
                          float temp_c, int heart_rate, float spo2);

#ifdef __cplusplus
}
#endif

#endif /* AXION_ALERT_MESSAGE_H */
