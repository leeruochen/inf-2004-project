/*
 *  lf_hw_test.c
 *  Subsystem 3 -- LF-1 line sensor measurement test (S3-U09), APP_TEST only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel headers only (rule R1); the ADC is reached through
 *  the kernel-free drv_ir.h.
 *
 *  Purpose: measure, on the car, what the CENTRE (GP28/ADC2) and RIGHT
 *  (GP26/ADC0) line sensors read over white, over the black line and at the
 *  line edge, so that LF-2's thresholds and polarity come from evidence.
 *  It decides nothing about steering.  The operator follows the "[LF] >>>"
 *  prompts; car on the desk, motors unpowered.
 *
 *  This test task is the only ADC reader while it runs (drv_ir.h): there is
 *  no Line-Following task yet.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "config/pins.h"            /* IR_C_ADC_CH, IR_R_ADC_CH */
#include "config/thresholds.h"      /* ADC_NOISE_MAX_RAW */
#include "config/timing.h"          /* LF_SAMPLE_PERIOD_MS */
#include "drv_ir.h"
#include "lf_hw_test.h"

/** Length of each "keep still" measurement (1000 samples at 5 ms). */
#define LF1_MEASURE_MS          (5000U)
/** Length of the sideways sweep and its log interval (300 lines). */
#define LF1_SWEEP_MS            (15000U)
#define LF1_SWEEP_LOG_MS        (50U)
/** Provisional pass rule: black must differ from white by this much (raw,
 *  about 10 % of full scale), whichever way.  The real thresholds come from
 *  the printed numbers, in LF-2. */
#define LF1_MIN_CONTRAST        (400U)
#define LF1_SECOND_MS           (1000U)

/** Both sensors' statistics for one position. */
typedef struct
{
    lf_adc_stats_t c;           /**< CENTRE, ADC2 */
    lf_adc_stats_t r;           /**< RIGHT, ADC0 */
} lf1_pair_t;

/* Tell the operator what to do and for how long. */
static void lf1_countdown(char const *p_what, uint32_t seconds)
{
    dbg_printf("[LF] >>> %s (%lu s)", p_what, (unsigned long)seconds);
    for (uint32_t left = seconds; left > 0U; left--)
    {
        if (((left % 5U) == 0U) && (left != seconds))
        {
            dbg_printf("[LF] ... %lu s left", (unsigned long)left);
        }
        (void)tk_dly_tsk((RELTIM)LF1_SECOND_MS);
    }
}

/* Sample both sensors every LF_SAMPLE_PERIOD_MS (the line-following rate)
 * for LF1_MEASURE_MS while the operator keeps still, and print the result. */
static lf1_pair_t lf1_measure(char const *p_name)
{
    lf1_pair_t     p;
    uint32_t const n = LF1_MEASURE_MS / LF_SAMPLE_PERIOD_MS;

    lf_adc_stats_reset(&p.c);
    lf_adc_stats_reset(&p.r);
    dbg_printf("[LF] >>> measuring %s: keep still (%lu s)", p_name,
               (unsigned long)(LF1_MEASURE_MS / LF1_SECOND_MS));
    for (uint32_t i = 0U; i < n; i++)
    {
        lf_adc_stats_add(&p.c, drv_ir_read(IR_C_ADC_CH));
        lf_adc_stats_add(&p.r, drv_ir_read(IR_R_ADC_CH));
        (void)tk_dly_tsk((RELTIM)LF_SAMPLE_PERIOD_MS);
    }
    dbg_printf("[LF] %s: C min/avg/max %u/%u/%u  R %u/%u/%u  n=%lu", p_name,
               p.c.min, lf_adc_stats_avg(&p.c), p.c.max,
               p.r.min, lf_adc_stats_avg(&p.r), p.r.max, (unsigned long)p.c.n);
    return p;
}

/* |a - b| of two averages. */
static uint32_t lf1_diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
}

static bool lf1_quiet(lf1_pair_t const *p_p)
{
    return (lf_adc_stats_spread(&p_p->c) <= ADC_NOISE_MAX_RAW)
           && (lf_adc_stats_spread(&p_p->r) <= ADC_NOISE_MAX_RAW);
}

void lf_hw_test(uint32_t *p_pass, uint32_t *p_fail)
{
    uint32_t       test_pass_ = *p_pass;
    uint32_t       test_fail_ = *p_fail;
    lf_adc_stats_t sc;
    lf_adc_stats_t sr;
    lf1_pair_t     white;
    lf1_pair_t     on_c;
    lf1_pair_t     on_r;
    lf1_pair_t     both;

    dbg_puts("[LF] LF-1 line sensor measurement: follow the [LF] >>> prompts (.agents/tasks/s3-lf1-test.md)");

    /* (a) Both inputs plausible (boot check again, now with numbers). */
    lf1_countdown("car on the desk, BOTH line sensors over WHITE track surface", 15U);
    {
        lf_adc_status_t const st_c = drv_ir_sanity(IR_C_ADC_CH, &sc);
        lf_adc_status_t const st_r = drv_ir_sanity(IR_R_ADC_CH, &sr);

        dbg_printf("[LF] sanity: C %u..%u verdict %d, R %u..%u verdict %d (0 = OK)",
                   sc.min, sc.max, (int)st_c, sr.min, sr.max, (int)st_r);
        TEST_CHECK("S3-U09", (st_c == LF_ADC_OK) && (st_r == LF_ADC_OK),
                   "(a) both line sensors pass the ADC sanity check");
    }

    white = lf1_measure("WHITE under both");

    lf1_countdown("black LINE under the CENTRE sensor only, RIGHT on white", 15U);
    on_c = lf1_measure("line under CENTRE");

    lf1_countdown("black LINE under the RIGHT sensor only, CENTRE on white", 15U);
    on_r = lf1_measure("line under RIGHT");

    lf1_countdown("BOTH sensors over black (block or wide tape)", 15U);
    both = lf1_measure("BLACK under both");

    /* The line edge: a slow sideways sweep, logged as it happens. */
    lf1_countdown("sweep next: put the line just LEFT of the CENTRE sensor", 15U);
    dbg_printf("[LF] >>> sweep: slide the line SLOWLY sideways, past CENTRE then past RIGHT (%lu s)",
               (unsigned long)(LF1_SWEEP_MS / LF1_SECOND_MS));
    for (uint32_t t = 0U; t < LF1_SWEEP_MS; t += LF1_SWEEP_LOG_MS)
    {
        dbg_printf("[LF] sweep %5lu ms C=%4u R=%4u", (unsigned long)t,
                   drv_ir_read(IR_C_ADC_CH), drv_ir_read(IR_R_ADC_CH));
        (void)tk_dly_tsk((RELTIM)LF1_SWEEP_LOG_MS);
    }
    dbg_puts("[LF] <<< sweep done");

    /* Polarity and contrast, from the averages. */
    dbg_printf("[LF] polarity: C black %s white by %lu, R black %s white by %lu",
               (lf_adc_stats_avg(&on_c.c) > lf_adc_stats_avg(&white.c)) ? "ABOVE" : "BELOW",
               (unsigned long)lf1_diff(lf_adc_stats_avg(&on_c.c), lf_adc_stats_avg(&white.c)),
               (lf_adc_stats_avg(&on_r.r) > lf_adc_stats_avg(&white.r)) ? "ABOVE" : "BELOW",
               (unsigned long)lf1_diff(lf_adc_stats_avg(&on_r.r), lf_adc_stats_avg(&white.r)));
    TEST_CHECK("S3-U09", lf1_diff(lf_adc_stats_avg(&on_c.c), lf_adc_stats_avg(&white.c)) >= LF1_MIN_CONTRAST,
               "(b) CENTRE: line differs from white by >= 400 raw");
    TEST_CHECK("S3-U09", lf1_diff(lf_adc_stats_avg(&on_r.r), lf_adc_stats_avg(&white.r)) >= LF1_MIN_CONTRAST,
               "(c) RIGHT: line differs from white by >= 400 raw");
    TEST_CHECK("S3-U09", lf1_quiet(&white) && lf1_quiet(&on_c) && lf1_quiet(&on_r) && lf1_quiet(&both),
               "(d) both sensors steady while still (spread <= 400)");

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

#endif /* APP_TEST */
