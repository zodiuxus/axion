/**
 * max30102_algorithm.c - see max30102_algorithm.h for design notes.
 */
#include "max30102_algorithm.h"
#include <math.h>
#include <stdbool.h>

#define MINIMUM_RATIO 0.3

static double s_time_axis[MAX30102_ALG_BUFFER_SIZE];

void max30102_alg_init(void)
{
    double t = 0.0;
    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) {
        s_time_axis[i] = t;
        t += MAX30102_ALG_SAMPLE_MS / 1000.0;
    }
}

void max30102_alg_remove_dc(int32_t *ir, int32_t *red,
                            int64_t *ir_mean, int64_t *red_mean)
{
    int64_t ir_sum  = 0;
    int64_t red_sum = 0;
    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) {
        ir_sum  += ir[i];
        red_sum += red[i];
    }
    int64_t ir_m  = ir_sum  / MAX30102_ALG_BUFFER_SIZE;
    int64_t red_m = red_sum / MAX30102_ALG_BUFFER_SIZE;

    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) {
        red[i] = red[i] - (int32_t)red_m;
        ir[i]  = ir[i]  - (int32_t)ir_m;
    }

    if (ir_mean)  *ir_mean  = ir_m;
    if (red_mean) *red_mean = red_m;
}

static double sum_of_elements(const int32_t *data)
{
    double sum = 0.0;
    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) sum += data[i];
    return sum;
}

static double sum_of_xy_elements(const int32_t *data)
{
    double sum = 0.0;
    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) {
        sum += (double)data[i] * s_time_axis[i];
    }
    return sum;
}

static double sum_of_x_squared(void)
{
    /* Closed form: sum_{i=0}^{N-1} (i * dt)^2 = dt^2 * (N-1)*N*(2N-1)/6 */
    const double dt = MAX30102_ALG_SAMPLE_MS / 1000.0;
    const int    N  = MAX30102_ALG_BUFFER_SIZE;
    return dt * dt * (double)((N - 1) * N * (2 * N - 1)) / 6.0;
}

static double sum_of_x(void)
{
    /* Closed form: sum_{i=0}^{N-1} (i * dt) = dt * N*(N-1)/2 */
    const double dt = MAX30102_ALG_SAMPLE_MS / 1000.0;
    const int    N  = MAX30102_ALG_BUFFER_SIZE;
    return dt * (double)(N * (N - 1)) / 2.0;
}

static void calculate_linear_regression(double *angular, double *linear, const int32_t *data)
{
    const int N = MAX30102_ALG_BUFFER_SIZE;
    double sum_y  = sum_of_elements(data);
    double sum_x  = sum_of_x();
    double sum_x2 = sum_of_x_squared();
    double sum_xy = sum_of_xy_elements(data);

    double denom = sum_x2 - (sum_x * sum_x) / N;
    if (denom == 0.0) denom = 1e-12;

    *angular = (sum_xy - (sum_x * sum_y) / N) / denom;
    *linear  = (sum_y / N) - (*angular * (sum_x / N));
}

void max30102_alg_remove_trend(int32_t *buffer)
{
    double a, b;
    calculate_linear_regression(&a, &b, buffer);
    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) {
        buffer[i] = (int32_t)((double)buffer[i] - a * s_time_axis[i] - b);
    }
}

double max30102_alg_correlation(const int32_t *red, const int32_t *ir)
{
    const int N = MAX30102_ALG_BUFFER_SIZE;
    double x_mean = 0, y_mean = 0;
    for (int i = 0; i < N; ++i) { x_mean += red[i]; y_mean += ir[i]; }
    x_mean /= N; y_mean /= N;

    double sxx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < N; ++i) {
        double dx = red[i] - x_mean;
        double dy = ir[i]  - y_mean;
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    double denom = sqrt(sxx * syy);
    if (denom == 0.0) return 0.0;
    return sxy / denom;
}

static double rms_value(const int32_t *data)
{
    double sum_sq = 0.0;
    for (int i = 0; i < MAX30102_ALG_BUFFER_SIZE; ++i) {
        double v = data[i];
        sum_sq += v * v;
    }
    return sqrt(sum_sq / MAX30102_ALG_BUFFER_SIZE);
}

double max30102_alg_spo2(const int32_t *ir, const int32_t *red,
                         int64_t ir_mean, int64_t red_mean)
{
    double ir_rms  = rms_value(ir);
    double red_rms = rms_value(red);
    if (ir_mean == 0 || red_mean == 0) return 0.0;

    /* R = (red_AC/red_DC) / (ir_AC/ir_DC) - the standard ratio-of-ratios.
     *
     * Calibration curve: SpO2 = -45.06*R^2 + 30.354*R + 94.845. This is
     * the quadratic that shipped (commented out!) in the reference repo
     * this component was adapted from - its active line was the debug
     * placeholder "SpO2 = 49.7*R", which is monotonically INCREASING in
     * R, i.e. physiologically backwards: more red absorption relative to
     * IR means MORE deoxyhemoglobin, so SpO2 must FALL as R rises.
     *
     * The symptom that uncovered it: ordinary finger values of R ~ 1.04
     * to 1.07 mapped to the impossible "51.6–53.3 %" (49.7 * R), while
     * genuinely good windows with R < 1.0 mapped below 50 % and were
     * zeroed by the caller's plausibility gate - hence "SpO2 reads 51 to
     * 53 % when it reads anything at all, 0 % otherwise".
     *
     * The quadratic maps the useful band sensibly:
     *   R 0.5 -> ~98.8 %   R 0.7 -> ~94.0 %   R 0.9 -> ~85.7 %
     * and falls off a cliff for R > 1.2, where the caller's
     * physiological gate (< 70 % -> invalid) takes over. */
    double R = (red_rms / (double)red_mean) / (ir_rms / (double)ir_mean);
    double spo2 = (-45.06 * R + 30.354) * R + 94.845;
    if (spo2 > 100.0) spo2 = 100.0;
    if (spo2 < 0.0)   spo2 = 0.0;
    return spo2;
}

static double autocorrelation(const int32_t *data, int lag)
{
    if (lag < 0 || lag >= MAX30102_ALG_BUFFER_SIZE) return 0.0;
    double sum = 0.0;
    int n = MAX30102_ALG_BUFFER_SIZE - lag;
    for (int i = 0; i < n; ++i) {
        sum += (double)data[i] * (double)data[i + lag];
    }
    /* Divide by the FULL window (N), not the overlap count (n): this is
     * the biased autocorrelation estimator, and the (N-lag)/N shrinkage
     * is deliberate - it weights down long lags, where only a few
     * overlapping samples make the estimate pure noise. With the lag
     * search spanning 11..124, the unbiased form (divide by n) lets
     * long-lag noise peaks rival the true period peak and the estimator
     * locks onto nonsense (~13 bpm). Keep the N divisor. */
    return sum / (double)MAX30102_ALG_BUFFER_SIZE;
}

int max30102_alg_heart_rate(const int32_t *ir, double *r0, double *autocorr_out)
{
    double r0_val   = autocorrelation(ir, 0);
    if (r0) *r0 = r0_val;
    if (r0_val == 0.0) return 0;

    /* Normalized autocorrelation for the whole window. */
    double ac[MAX30102_ALG_BUFFER_SIZE];
    for (int lag = 1; lag < MAX30102_ALG_BUFFER_SIZE; ++lag) {
        ac[lag] = autocorrelation(ir, lag) / r0_val;
        if (autocorr_out) autocorr_out[lag] = ac[lag];
    }

    /* Strongest LOCAL maximum inside the physiological band (lags 11..124
     * correspond to ~487..29 bpm at 40 ms). The candidate peak must beat
     * its immediate neighbours on both sides. A window with no pulse
     * periodicity produces an autocorrelation curve that only decays
     * from the left edge - it has no local maximum, so it now reports
     * "no reading" (0). The previous plain global-max search picked that
     * curve's leftmost point instead: lag 11 == 60/(11*0.040) == 136.4
     * bpm, which is exactly why weak/noisy windows locked onto "136". */
    double best = 0.0;
    int    best_lag = 0;
    for (int lag = 11; lag < 125 && lag < MAX30102_ALG_BUFFER_SIZE - 1; ++lag) {
        double r = ac[lag];
        if (r <= MINIMUM_RATIO) continue;
        if (r < ac[lag - 1] || r < ac[lag + 1]) continue;
        if (r > best) {
            best     = r;
            best_lag = lag;
        }
    }

    if (best_lag == 0) return 0;
    double period_s = best_lag * (MAX30102_ALG_SAMPLE_MS / 1000.0);
    if (period_s <= 0.0) return 0;
    return (int)(60.0 / period_s + 0.5);
}
