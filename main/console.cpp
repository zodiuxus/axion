/**
 * console.cpp - Optional debug console. Replaces combine_sensors().
 */
#include "console.h"

#include <cstdio>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "state.h"

void console_task(void * /*arg*/)
{
    /* Wait for at least MPU + GNSS so the output is meaningful. */
    axion_state_wait_all(BIT_MPU_READY | BIT_GNSS_READY);

    axion_state_t s;
    while (true) {
        axion_state_snapshot(&s);
        printf("\033[7F");                                  /* move up 6 lines */
        printf("\033[KTemp:    %.2f C\n",                    s.temp_c);
        printf("\033[KPos:     %.6f, %.6f, alt=%.1f m\n",    s.lat, s.lon, s.alt);
        printf("\033[KSpeed:   %.2f m/s\n",                  s.speed);
        printf("\033[KRPY:     %3.2f, %3.2f, %3.2f deg\n",
               s.ypr[2] * 180.0f / (float)M_PI,
               s.ypr[1] * 180.0f / (float)M_PI,
               s.ypr[0] * 180.0f / (float)M_PI);
        printf("\033[KHR/SpO2: %d bpm, %.1f%% (%s)\n",
               s.heart_rate, s.spo2,
               s.spo2_valid ? "valid" : "invalid");
        printf("\033[K\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
