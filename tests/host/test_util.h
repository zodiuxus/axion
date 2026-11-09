#pragma once
/**
 * test_util.h - Minimal assertion harness shared by the host tests.
 *
 * Inline variables (C++17) so the counters are shared across translation
 * units. Each test file exports a run_*_tests() function; run_all.cpp calls
 * them and prints the summary.
 */
#include <cstdio>

inline int g_checks   = 0;
inline int g_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                    \
    } while (0)
