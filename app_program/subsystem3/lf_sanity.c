/*
 *  lf_sanity.c
 *  Subsystem 3 -- line sensor ADC statistics and sanity decision (see
 *  lf_sanity.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/thresholds.h"      /* ADC_FULL_SCALE_RAW, ADC_STUCK_*, ADC_NOISE_MAX_RAW */
#include "app_program/subsystem3/lf_sanity.h"

void lf_adc_stats_reset(lf_adc_stats_t *p_s)
{
    if (p_s != NULL)
    {
        p_s->min = 0U;
        p_s->max = 0U;
        p_s->sum = 0U;
        p_s->n = 0U;
    }
}

void lf_adc_stats_add(lf_adc_stats_t *p_s, uint16_t raw)
{
    if (p_s != NULL)
    {
        uint16_t const v = (raw > ADC_FULL_SCALE_RAW) ? (uint16_t)ADC_FULL_SCALE_RAW : raw;

        if ((p_s->n == 0U) || (v < p_s->min))
        {
            p_s->min = v;
        }
        if ((p_s->n == 0U) || (v > p_s->max))
        {
            p_s->max = v;
        }
        p_s->sum += v;
        p_s->n++;
    }
}

uint16_t lf_adc_stats_avg(lf_adc_stats_t const *p_s)
{
    uint16_t avg = 0U;

    if ((p_s != NULL) && (p_s->n > 0U))
    {
        avg = (uint16_t)((p_s->sum + (p_s->n / 2U)) / p_s->n);    /* round to nearest */
    }

    return avg;
}

uint16_t lf_adc_stats_spread(lf_adc_stats_t const *p_s)
{
    uint16_t spread = 0U;

    if ((p_s != NULL) && (p_s->n > 0U))
    {
        spread = (uint16_t)(p_s->max - p_s->min);
    }

    return spread;
}

lf_adc_status_t lf_adc_check(lf_adc_stats_t const *p_s)
{
    lf_adc_status_t st = LF_ADC_NO_SAMPLES;

    if ((p_s != NULL) && (p_s->n > 0U))
    {
        if (p_s->max <= ADC_STUCK_LOW_RAW)
        {
            st = LF_ADC_STUCK_LOW;
        }
        else if (p_s->min >= ADC_STUCK_HIGH_RAW)
        {
            st = LF_ADC_STUCK_HIGH;
        }
        else if (lf_adc_stats_spread(p_s) > ADC_NOISE_MAX_RAW)
        {
            st = LF_ADC_NOISY;
        }
        else
        {
            st = LF_ADC_OK;
        }
    }

    return st;
}
