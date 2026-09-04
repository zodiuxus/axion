#pragma once
/**
 * console.h - Optional debug console task that pretty-prints sensor state.
 */
#ifndef AXION_CONSOLE_H
#define AXION_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task: prints a compact one-line-per-sensor summary of the
 *  shared state every 100 ms, using ANSI escapes to keep the cursor
 *  stable. Disabled by default; enable in app_main if needed. */
void console_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* AXION_CONSOLE_H */
