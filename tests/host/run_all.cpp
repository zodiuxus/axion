/**
 * run_all.cpp - entry point for the Axion host test suite.
 *
 * Every *_test.cpp in this directory exports a run_*_tests() function; this
 * file calls them all and prints the shared summary from test_util.h.
 */
#include "test_util.h"

void run_alert_policy_tests();
void run_alert_message_tests();
void run_sensor_math_tests();

int main()
{
    run_alert_policy_tests();
    run_alert_message_tests();
    run_sensor_math_tests();

    std::printf("%d checks, %d failure(s)\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
