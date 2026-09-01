#pragma once
/**
 * max30102_algorithm.h - Heart-rate & SpO2 estimation from a buffer of
 * MAX30102 RED/IR samples.
 *
 * Adapted from Gabriel-Gardin/max30102_esp32_oximeter/algorithm.{c,h}.
 * The algorithm is autocorrelation-based: it finds the dominant
 * non-zero-lag peak of the IR autocorrelation and converts its lag to
 * bpm. SpO2 is estimated from the RMS ratio of the AC-coupled RED and
 * IR channels.
 *
 * Cleanup notes:
 *   - Hardcoded magic constant `sum_of_x = 325.12` removed; computed
 *     analytically from BUFFER_SIZE and SAMPLE_PERIOD.
 *   - Unused `time` accumulator removed from `sum_of_squared_elements`.
 *   - `calculate_heart_rate` no longer overwrites `resultado` on every
 *     iteration before a peak is found (was returning 333 bpm on no
 *     signal - replaced with a sentinel return of 0).
 *   - `remove_dc_part` now uses signed accumulation (the reference used
 *     uint64_t, which produced wrong results for negative DC offsets).
 */
#ifndef MAX30102_ALGORITHM_H
#define MAX30102_ALGORITHM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_ALG_BUFFER_SIZE   128
#define MAX30102_ALG_SAMPLE_MS     40     /* must match SAMPLE period used by caller */

/**
 * Initialize internal scratch tables (currently just the time axis used
 * by linear regression). Call once before any other algorithm function.
 */
void max30102_alg_init(void);

/**
 * Remove the DC component from both buffers in place. Returns the
 * pre-removal means via *ir_mean and *red_mean (used by SpO2).
 */
void max30102_alg_remove_dc(int32_t *ir, int32_t *red,
                            int64_t *ir_mean, int64_t *red_mean);

/** Remove a best-fit linear trend line from `buffer` in place. */
void max30102_alg_remove_trend(int32_t *buffer);

/**
 * Pearson correlation coefficient between the two buffers. Used to
 * decide whether the signal is good enough to compute SpO2 (typical
 * threshold: >= 0.7).
 */
double max30102_alg_correlation(const int32_t *red, const int32_t *ir);

/**
 * Estimate heart rate from the IR buffer.
 *
 * @param ir                  IR sample buffer (must be at least
 *                             MAX30102_ALG_BUFFER_SIZE elements).
 * @param r0                  Out: autocorrelation at lag 0 (for SNR check).
 * @param autocorr_out        Out: optional buffer to receive the
 *                             normalized autocorrelation curve (length
 *                             MAX30102_ALG_BUFFER_SIZE). May be NULL.
 * @return Heart rate in bpm, or 0 if no reliable peak was found.
 */
int max30102_alg_heart_rate(const int32_t *ir, double *r0, double *autocorr_out);

/**
 * Estimate SpO2 (%) from the AC-coupled IR/RED buffers and their
 * pre-AC means. The caller should ensure the signal is valid (e.g.
 * `max30102_alg_correlation(...) >= 0.7`) before calling this.
 */
double max30102_alg_spo2(const int32_t *ir, const int32_t *red,
                         int64_t ir_mean, int64_t red_mean);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_ALGORITHM_H */
