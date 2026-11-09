#pragma once
/**
 * diagnostics.h - Field diagnostics that outlive a single boot.
 *
 * Reset reason and boot count use RTC no-init memory (survives resets, not a
 * power cycle, no flash wear). The last alert outcome is one NVS blob.
 */
#ifndef AXION_DIAGNOSTICS_H
#define AXION_DIAGNOSTICS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Log reset reason, boot counter and heap; read back the last alert.
 *  Call once from app_main after NVS is initialised. */
void diagnostics_boot_note(void);

/** Mark steady state: clears the repeated-reset counter, logs heap. */
void diagnostics_mark_steady(void);

/** Record an alert delivery outcome (rare NVS write). */
void diagnostics_note_alert(uint32_t mask, bool collision, bool delivered);

#ifdef __cplusplus
}
#endif

#endif /* AXION_DIAGNOSTICS_H */
