/**
 * alert_message_test.cpp - host tests for the SMS body formatting.
 *
 * The body is the only part of an alert a human reads, so the exact text and
 * the three position cases (live / last known / never fixed) are pinned here.
 */
#include <cstring>

#include "test_util.h"
#include "alert_message.h"

namespace {

bool contains(const char *haystack, const char *needle)
{
    return std::strstr(haystack, needle) != nullptr;
}

void test_reason_mapping()
{
    CHECK(std::strcmp(alert_reason_text(1u << ALERT_SRC_FALL, false),
                      "fall detected") == 0);
    CHECK(std::strcmp(alert_reason_text(1u << ALERT_SRC_VITALS, false),
                      "vitals abnormal") == 0);
    CHECK(std::strcmp(alert_reason_text(1u << ALERT_SRC_TEMP_LOW, false),
                      "body temperature abnormal") == 0);
    CHECK(std::strcmp(alert_reason_text(1u << ALERT_SRC_TEMP_HIGH, false),
                      "body temperature abnormal") == 0);
    /* Collision wins over any continuous condition. */
    CHECK(std::strcmp(alert_reason_text(1u << ALERT_SRC_FALL, true),
                      "collision detected") == 0);
    /* Nothing raised and no collision: generic, never empty. */
    CHECK(std::strcmp(alert_reason_text(0u, false), "alert") == 0);
}

void test_format_live_fix()
{
    char body[ALERT_MESSAGE_MAX_LEN];
    alert_message_format(body, sizeof(body), "fall detected",
                         true, 45.123456, 7.654321, 250.0f, 1.4f, -1,
                         36.70f, 72, 97.5f);
    CHECK(contains(body, "AXION ALERT: fall detected."));
    CHECK(contains(body, "lat=45.123456"));
    CHECK(contains(body, "lon=7.654321"));
    CHECK(contains(body, "spd=1.4m/s"));
    CHECK(contains(body, "temp=36.70C"));
    CHECK(contains(body, "hr=72"));
    CHECK(contains(body, "spo2=97.5%"));
}

void test_format_last_known_fix()
{
    char body[ALERT_MESSAGE_MAX_LEN];
    alert_message_format(body, sizeof(body), "vitals abnormal",
                         false, 45.123456, 7.654321, 250.0f, 0.0f, 42,
                         36.20f, 130, 88.0f);
    CHECK(contains(body, "last known") == false);   /* text is upper-case */
    CHECK(contains(body, "LAST KNOWN, 42s old"));
    CHECK(contains(body, "AXION ALERT: vitals abnormal."));
    CHECK(contains(body, "hr=130"));
}

void test_format_no_fix()
{
    char body[ALERT_MESSAGE_MAX_LEN];
    alert_message_format(body, sizeof(body), "collision detected",
                         false, 0.0, 0.0, 0.0f, 0.0f, -1,
                         36.50f, 80, 96.0f);
    CHECK(contains(body, "NO GPS FIX YET"));
    /* Must not send a bogus 0,0 position. */
    CHECK(contains(body, "lat=") == false);
}

void test_format_truncation_is_safe()
{
    char small[24];
    alert_message_format(small, sizeof(small), "fall detected",
                         true, 45.123456, 7.654321, 250.0f, 1.4f, -1,
                         36.70f, 72, 97.5f);
    CHECK(std::strlen(small) < sizeof(small));
}

}  // namespace

void run_alert_message_tests()
{
    test_reason_mapping();
    test_format_live_fix();
    test_format_last_known_fix();
    test_format_no_fix();
    test_format_truncation_is_safe();
}
