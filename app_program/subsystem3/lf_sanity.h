/*
 *  lf_sanity.h
 *  Subsystem 3 -- line sensor ADC statistics and sanity decision (pure C).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Collects min / max / average of raw 12-bit ADC samples of one line sensor
 *  and decides whether the input is plausible (config/thresholds.h ADC_*):
 *  every sample at the low rail, every sample at the high rail, or a
 *  peak-to-peak spread above the noise limit is a sensor fault (a floating,
 *  shorted or double-driven pin; GP26 also has a second, untaped lead on
 *  Grove 6).  Used by the boot check (hal_adc_sanity_check, drv_ir.c) and by
 *  the LF-1 measurement test.  No line-following decisions live here.
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and config only,
 *  so tests/host/test_lf_sanity.c builds it unchanged.
 */

#ifndef LF_SANITY_H
#define LF_SANITY_H

#include <stdbool.h>
#include <stdint.h>

/** Running statistics of one sensor's raw samples. */
typedef struct
{
    uint16_t min;       /**< smallest sample (valid if n > 0) */
    uint16_t max;       /**< largest sample (valid if n > 0) */
    uint32_t sum;       /**< sum of the samples (n <= 2^20 keeps it < 2^32) */
    uint32_t n;         /**< samples added */
} lf_adc_stats_t;

/** Sanity verdict. */
typedef enum
{
    LF_ADC_OK = 0,          /**< plausible */
    LF_ADC_STUCK_LOW,       /**< every sample <= ADC_STUCK_LOW_RAW */
    LF_ADC_STUCK_HIGH,      /**< every sample >= ADC_STUCK_HIGH_RAW */
    LF_ADC_NOISY,           /**< max - min > ADC_NOISE_MAX_RAW */
    LF_ADC_NO_SAMPLES       /**< nothing to judge */
} lf_adc_status_t;

/** Start empty. */
void lf_adc_stats_reset(lf_adc_stats_t *p_s);

/** Add one raw sample (0..ADC_FULL_SCALE_RAW; larger values are clamped). */
void lf_adc_stats_add(lf_adc_stats_t *p_s, uint16_t raw);

/** Average, rounded to nearest; 0 when empty. */
uint16_t lf_adc_stats_avg(lf_adc_stats_t const *p_s);

/** Peak-to-peak spread (max - min); 0 when empty. */
uint16_t lf_adc_stats_spread(lf_adc_stats_t const *p_s);

/** Sanity verdict.  Stuck is tested before noisy: a stuck pin is the
 *  clearer fault. */
lf_adc_status_t lf_adc_check(lf_adc_stats_t const *p_s);

#endif /* LF_SANITY_H */
