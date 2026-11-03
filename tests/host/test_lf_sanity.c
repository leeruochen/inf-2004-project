/*
 *  test_lf_sanity.c
 *  Host tests for the line sensor ADC statistics and sanity decision
 *  (app_program/subsystem3/lf_sanity.c).  Test ID S3-U08.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Expected values are the limits written in config/thresholds.h, typed in
 *  as numbers here, not recomputed from the macros.
 */

#include <stdint.h>
#include <stdio.h>

#include "config/thresholds.h"
#include "app_program/subsystem3/lf_sanity.h"
#include "test_check.h"

static lf_adc_stats_t stats_of(uint16_t const *p_v, uint32_t n)
{
    lf_adc_stats_t s;

    lf_adc_stats_reset(&s);
    for (uint32_t i = 0U; i < n; i++)
    {
        lf_adc_stats_add(&s, p_v[i]);
    }
    return s;
}

/* Test 1: the limits this test was written against. */
static void test_limits(void)
{
    CHECK(ADC_FULL_SCALE_RAW == 4095U);
    CHECK(ADC_STUCK_LOW_RAW == 16U);
    CHECK(ADC_STUCK_HIGH_RAW == 4079U);
    CHECK(ADC_NOISE_MAX_RAW == 400U);
}

/* Test 2: min / max / average / spread, rounding, empty, clamping. */
static void test_stats(void)
{
    uint16_t const v[4] = { 100U, 300U, 200U, 201U };     /* avg 200.25 -> 200 */
    uint16_t const w[2] = { 1U, 2U };                     /* avg 1.5 -> 2 */
    lf_adc_stats_t s = stats_of(v, 4U);

    CHECK(s.n == 4U);
    CHECK(s.min == 100U);
    CHECK(s.max == 300U);
    CHECK(lf_adc_stats_avg(&s) == 200U);
    CHECK(lf_adc_stats_spread(&s) == 200U);
    s = stats_of(w, 2U);
    CHECK(lf_adc_stats_avg(&s) == 2U);

    lf_adc_stats_reset(&s);
    CHECK(s.n == 0U);
    CHECK(lf_adc_stats_avg(&s) == 0U);
    CHECK(lf_adc_stats_spread(&s) == 0U);
    CHECK(lf_adc_check(&s) == LF_ADC_NO_SAMPLES);

    lf_adc_stats_add(&s, 60000U);                         /* not from a 12-bit ADC */
    CHECK(s.max == 4095U);

    lf_adc_stats_add(NULL, 5U);                           /* no crash */
    CHECK(lf_adc_stats_avg(NULL) == 0U);
    CHECK(lf_adc_check(NULL) == LF_ADC_NO_SAMPLES);
}

/* Tests 3-5: stuck low, stuck high, noisy, and the boundaries of each. */
static void test_verdicts(void)
{
    uint16_t const low[3] = { 0U, 16U, 9U };              /* all <= 16 */
    uint16_t const low_ok[3] = { 0U, 17U, 9U };           /* one sample above */
    uint16_t const high[3] = { 4079U, 4095U, 4090U };     /* all >= 4079 */
    uint16_t const high_ok[3] = { 4078U, 4095U, 4090U };
    uint16_t const noisy[2] = { 1000U, 1401U };           /* spread 401 */
    uint16_t const quiet[2] = { 1000U, 1400U };           /* spread 400 */
    uint16_t const white[4] = { 310U, 322U, 305U, 318U }; /* a plausible sensor */
    lf_adc_stats_t s;

    s = stats_of(low, 3U);
    CHECK(lf_adc_check(&s) == LF_ADC_STUCK_LOW);
    s = stats_of(low_ok, 3U);
    CHECK(lf_adc_check(&s) == LF_ADC_OK);
    s = stats_of(high, 3U);
    CHECK(lf_adc_check(&s) == LF_ADC_STUCK_HIGH);
    s = stats_of(high_ok, 3U);
    CHECK(lf_adc_check(&s) == LF_ADC_OK);
    s = stats_of(noisy, 2U);
    CHECK(lf_adc_check(&s) == LF_ADC_NOISY);
    s = stats_of(quiet, 2U);
    CHECK(lf_adc_check(&s) == LF_ADC_OK);
    s = stats_of(white, 4U);
    CHECK(lf_adc_check(&s) == LF_ADC_OK);

    /* A pin flipping between both rails is noisy, not stuck. */
    {
        uint16_t const rails[2] = { 0U, 4095U };

        s = stats_of(rails, 2U);
        CHECK(lf_adc_check(&s) == LF_ADC_NOISY);
    }
}

int main(void)
{
    test_limits();
    test_stats();
    test_verdicts();
    return TEST_RESULT("lf_sanity");
}
