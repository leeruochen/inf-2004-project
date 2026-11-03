/*
 *  test_lf_core.c
 *  Host tests for the line-following core (app_program/subsystem3/lf_core.c).
 *  Test ID S3-U10.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Where the expected values come from (never from lf_core.c's arithmetic):
 *    - times: design.md section 3.2 / config/timing.h, typed in as numbers
 *      (N = 3 samples, T_lost 100 ms, T_turn 500 ms, T_uturn 1 s,
 *      T_search 2 s, sample 5 ms, steering post 20 ms);
 *    - raw numbers: the LF-1 desk run (white ~140, black lying ~1100-1150);
 *    - steering sign: steer > 0 = steer right (data_repr.h; task_motion.c
 *      makes the left wheel faster);
 *    - closed-loop behaviour: the simulator in lf_sim.h (geometry, the
 *      Vehicle Controller and Motion rules from their code).
 *  Fixtures use a calibration of white 140, black 1140, so a darkness of
 *  d per mille is simply raw = 140 + d.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config/timing.h"
#include "src/common/data_repr.h"
#include "app_program/subsystem3/lf_core.h"
#include "app_program/subsystem3/lf_sanity.h"
#include "test_check.h"
#include "lf_sim.h"

/* ---------------------------------------------------------------------------
 * Fixtures
 * ------------------------------------------------------------------------- */

static lf_cal_t const k_cal = { { 140U, 1140U }, { 140U, 1140U } };
static lf_in_t        g_in;

static uint16_t raw_of(uint16_t d_pm)
{
    return (uint16_t)(140U + d_pm);
}

static lf_out_t feed(lf_core_t *p_lf, uint16_t dc, uint16_t dr)
{
    lf_out_t out;

    g_in.raw_c = raw_of(dc);
    g_in.raw_r = raw_of(dr);
    memset(&out, 0xA5, sizeof(out));        /* every field must be written */
    lf_core_step(p_lf, &g_in, &out);
    return out;
}

typedef struct
{
    uint32_t n;             /* samples fed */
    uint32_t posts;
    uint32_t junctions;
    uint32_t track_lost;
    int32_t  min_steer;     /* over posted values */
    int32_t  max_steer;
    int32_t  first_post_at; /* sample index of the first post, -1 if none */
    lf_out_t last;
} run_t;

static run_t run(lf_core_t *p_lf, uint32_t n, uint16_t dc, uint16_t dr)
{
    run_t r;

    memset(&r, 0, sizeof(r));
    r.min_steer = 1000;
    r.max_steer = -1000;
    r.first_post_at = -1;
    for (uint32_t i = 0U; i < n; i++)
    {
        lf_out_t const o = feed(p_lf, dc, dr);

        r.n++;
        if (o.post_steer)
        {
            if (r.first_post_at < 0)
            {
                r.first_post_at = (int32_t)i;
            }
            r.posts++;
            r.min_steer = (o.steer_q7 < r.min_steer) ? o.steer_q7 : r.min_steer;
            r.max_steer = (o.steer_q7 > r.max_steer) ? o.steer_q7 : r.max_steer;
        }
        if (o.event == LF_EV_JUNCTION)
        {
            r.junctions++;
        }
        if (o.event == LF_EV_TRACK_LOST)
        {
            r.track_lost++;
        }
        r.last = o;
    }
    return r;
}

/* Feed (dc, dr) until the state is st; returns the samples fed, or
 * limit + 1 if it never got there. */
static uint32_t until_state(lf_core_t *p_lf, uint16_t dc, uint16_t dr,
                            lf_state_t st, uint32_t limit)
{
    for (uint32_t i = 1U; i <= limit; i++)
    {
        if (feed(p_lf, dc, dr).state == st)
        {
            return i;
        }
    }
    return limit + 1U;
}

static void reset_inputs(void)
{
    memset(&g_in, 0, sizeof(g_in));
    g_in.manoeuvre = LF_MAN_NONE;
}

/* Calibrated core, car in the start pose (CENTRE on the line body, RIGHT on
 * white), run started, then tracking the right edge for 200 ms. */
static void start_tracking(lf_core_t *p_lf)
{
    reset_inputs();
    lf_core_init(p_lf, &k_cal);
    (void)run(p_lf, 5U, 1000U, 0U);         /* idle, sensors settle */
    g_in.running = true;
    (void)run(p_lf, 40U, 500U, 0U);
}

static lf_adc_stats_t stats_const(uint16_t v, uint32_t n)
{
    lf_adc_stats_t s;

    lf_adc_stats_reset(&s);
    for (uint32_t i = 0U; i < n; i++)
    {
        lf_adc_stats_add(&s, v);
    }
    return s;
}

/* ---------------------------------------------------------------------------
 * Slice 1: constants, calibration, normalisation, zones, edge steering
 * ------------------------------------------------------------------------- */

/* Test 1: the design numbers this file was written against. */
static void test_constants(void)
{
    CHECK(LF_SAMPLE_PERIOD_MS == 5U);
    CHECK(LF_STEER_POST_PERIOD_MS == 20U);
    CHECK(LF_HYST_SAMPLES == 3U);
    CHECK(LF_T_LOST_MS == 100U);
    CHECK(LF_T_TURN_MS == 500U);
    CHECK(LF_T_UTURN_MS == 1000U);
    CHECK(LF_T_SEARCH_MS == 2000U);
    CHECK(MOTION_T_HOLD_MS == 100U);
    CHECK(DREP_STEER_Q7_MAX == 127);
    CHECK((int)LF_ST_COUNT == (int)DREP_FSM_LF_STATES);     /* telemetry: 6 states */
}

/* Test 2: start-pose calibration, from the LF-1 desk numbers. */
static void test_calibration(void)
{
    uint16_t const on_line[6] = { 1130U, 1150U, 1140U, 1135U, 1145U, 1140U }; /* avg 1140 */
    uint16_t const white[6] = { 135U, 145U, 140U, 138U, 142U, 140U };         /* avg 140 */
    lf_adc_stats_t c;
    lf_adc_stats_t r;
    lf_cal_t cal;
    lf_cal_t const sentinel = { { 1U, 2U }, { 3U, 4U } };

    lf_adc_stats_reset(&c);
    lf_adc_stats_reset(&r);
    for (uint32_t i = 0U; i < 6U; i++)
    {
        lf_adc_stats_add(&c, on_line[i]);
        lf_adc_stats_add(&r, white[i]);
    }
    cal = sentinel;
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_OK);
    CHECK(cal.c.white == 140U);
    CHECK(cal.c.black == 1140U);
    CHECK(cal.r.white == 140U);
    CHECK(cal.r.black == 1140U);

    /* Contrast: exactly 400 raw accepted, 399 refused. */
    c = stats_const(540U, 10U);
    r = stats_const(140U, 10U);
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_OK);
    c = stats_const(539U, 10U);
    cal = sentinel;
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_NO_CONTRAST);
    CHECK(memcmp(&cal, &sentinel, sizeof(cal)) == 0);       /* untouched */

    /* Reversed polarity (sensors swapped or car placed wrongly). */
    c = stats_const(140U, 10U);
    r = stats_const(1140U, 10U);
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_NO_CONTRAST);

    /* A bad sensor: stuck at 0, stuck high, noisy, no samples, NULL. */
    c = stats_const(1140U, 10U);
    r = stats_const(0U, 10U);
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_BAD_SENSOR);
    r = stats_const(140U, 10U);
    c = stats_const(4095U, 10U);
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_BAD_SENSOR);
    lf_adc_stats_reset(&c);
    lf_adc_stats_add(&c, 700U);
    lf_adc_stats_add(&c, 1500U);                            /* spread 800 > 400 */
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_BAD_SENSOR);
    lf_adc_stats_reset(&c);
    CHECK(lf_cal_from_start_pose(&c, &r, &cal) == LF_CAL_BAD_SENSOR);
    c = stats_const(1140U, 10U);
    CHECK(lf_cal_from_start_pose(NULL, &r, &cal) == LF_CAL_BAD_SENSOR);
    CHECK(lf_cal_from_start_pose(&c, NULL, &cal) == LF_CAL_BAD_SENSOR);
    CHECK(lf_cal_from_start_pose(&c, &r, NULL) == LF_CAL_BAD_SENSOR);
}

/* Test 3: raw -> darkness per mille. */
static void test_norm(void)
{
    lf_cal_ch_t const cal = { 140U, 1140U };
    lf_cal_ch_t const lf1 = { 140U, 1120U };    /* LF-1: white 140, black 1120 */
    lf_cal_ch_t const flat = { 500U, 500U };

    CHECK(lf_norm(&cal, 140U) == 0U);
    CHECK(lf_norm(&cal, 100U) == 0U);           /* below white */
    CHECK(lf_norm(&cal, 0U) == 0U);
    CHECK(lf_norm(&cal, 390U) == 250U);
    CHECK(lf_norm(&cal, 640U) == 500U);
    CHECK(lf_norm(&cal, 890U) == 750U);
    CHECK(lf_norm(&cal, 1140U) == 1000U);
    CHECK(lf_norm(&cal, 3780U) == 1000U);       /* LF-1 "held close": clamped */
    CHECK(lf_norm(&cal, 4095U) == 1000U);
    CHECK(lf_norm(&lf1, 630U) == 500U);         /* (630-140)/(1120-140) = 0.5 */
    CHECK(lf_norm(&flat, 400U) == 0U);
    CHECK(lf_norm(&flat, 600U) == 0U);
    CHECK(lf_norm(NULL, 600U) == 0U);
}

/* Test 4: zones and their hysteresis. */
static void test_zones(void)
{
    lf_zone_filt_t f;

    CHECK(lf_zone_of(0U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_of(249U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_of(250U) == LF_ZONE_EDGE);
    CHECK(lf_zone_of(500U) == LF_ZONE_EDGE);
    CHECK(lf_zone_of(750U) == LF_ZONE_EDGE);
    CHECK(lf_zone_of(751U) == LF_ZONE_DARK);
    CHECK(lf_zone_of(1000U) == LF_ZONE_DARK);

    /* The first sample sets the zone at once. */
    lf_zone_reset(&f);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_DARK);

    /* A change needs 3 consecutive samples (N = 3). */
    lf_zone_reset(&f);
    CHECK(lf_zone_update(&f, 0U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_DARK);

    /* Two samples, then back: no change. */
    lf_zone_reset(&f);
    (void)lf_zone_update(&f, 0U);
    (void)lf_zone_update(&f, 900U);
    (void)lf_zone_update(&f, 900U);
    CHECK(lf_zone_update(&f, 0U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_DARK);

    /* Light to dark through one EDGE sample: confirmed after 3 DARK. */
    lf_zone_reset(&f);
    (void)lf_zone_update(&f, 0U);
    CHECK(lf_zone_update(&f, 500U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_LIGHT);
    CHECK(lf_zone_update(&f, 900U) == LF_ZONE_DARK);

    /* Noise straddling a threshold every other sample never flips it. */
    lf_zone_reset(&f);
    (void)lf_zone_update(&f, 0U);
    for (uint32_t i = 0U; i < 100U; i++)
    {
        CHECK(lf_zone_update(&f, ((i % 2U) == 0U) ? 270U : 230U) == LF_ZONE_LIGHT);
    }

    /* +-30 raw ADC noise (LF-1) over white and over the edge: no change. */
    {
        lf_cal_ch_t const cal = { 140U, 1140U };
        lf_zone_filt_t fe;
        bool w_ok = true;
        bool e_ok = true;

        lf_zone_reset(&f);
        lf_zone_reset(&fe);
        for (uint32_t i = 0U; i < 1000U; i++)
        {
            int32_t const n = (int32_t)((i * 7919U) % 61U) - 30;       /* -30..+30 */

            w_ok = w_ok && (lf_zone_update(&f, lf_norm(&cal, (uint16_t)(140 + n))) == LF_ZONE_LIGHT);
            e_ok = e_ok && (lf_zone_update(&fe, lf_norm(&cal, (uint16_t)(640 + n))) == LF_ZONE_EDGE);
        }
        CHECK(w_ok);
        CHECK(e_ok);
    }
}

/* Test 5: edge steering, both directions. */
static void test_edge_steer(void)
{
    CHECK(lf_edge_steer(500U) == 0);            /* on the edge: straight */
    CHECK(lf_edge_steer(1000U) == 64);          /* fully on black: drifted left -> right */
    CHECK(lf_edge_steer(0U) == -64);            /* fully on white: drifted right -> left */
    CHECK(lf_edge_steer(750U) == 32);
    CHECK(lf_edge_steer(250U) == -32);
    CHECK(lf_edge_steer(600U) > 0);
    CHECK(lf_edge_steer(400U) < 0);
    CHECK(lf_edge_steer(4000U) <= 127);         /* never beyond full */
    {
        bool mono = true;

        for (uint16_t d = 1U; d <= 1000U; d++)
        {
            mono = mono && (lf_edge_steer(d) >= lf_edge_steer((uint16_t)(d - 1U)));
        }
        CHECK(mono);
    }
}

/* ---------------------------------------------------------------------------
 * Slice 2: the state machine
 * ------------------------------------------------------------------------- */

/* Test 6: idle, start, posting cadence, refused calibration. */
static void test_idle_and_start(void)
{
    lf_core_t lf;
    run_t r;

    reset_inputs();
    lf_core_init(&lf, &k_cal);
    r = run(&lf, 200U, 1000U, 0U);              /* not running, on the line */
    CHECK(r.posts == 0U);
    CHECK(r.junctions == 0U);
    CHECK(r.track_lost == 0U);
    CHECK(r.last.state == LF_ST_IDLE);
    CHECK(!r.last.line_found);
    CHECK(r.last.d_c_pm == 1000U);
    CHECK(r.last.d_r_pm == 0U);

    /* Run starts in the start pose: ON_LINE at once, posting from the first
     * sample, then every 20 ms (4 samples); steers right towards the edge. */
    g_in.running = true;
    r = run(&lf, 40U, 1000U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.first_post_at == 0);
    CHECK(r.posts == 10U);
    CHECK(r.min_steer == 64);
    CHECK(r.max_steer == 64);
    CHECK(r.last.line_found);
    CHECK(r.last.event == LF_EV_NONE);

    /* Running stops: IDLE, nothing posted. */
    g_in.running = false;
    r = run(&lf, 50U, 500U, 0U);
    CHECK(r.last.state == LF_ST_IDLE);
    CHECK(r.posts == 0U);

    /* Start off the line (CENTRE white): no capture, so search, not track. */
    g_in.running = true;
    r = run(&lf, 1U, 0U, 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);

    /* Start with CENTRE on the edge only (not solid black): not captured. */
    g_in.running = false;
    (void)run(&lf, 10U, 500U, 0U);
    g_in.running = true;
    r = run(&lf, 1U, 500U, 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);

    /* Refused calibration: never posts, TRACK_LOST once per run start. */
    reset_inputs();
    lf_core_init(&lf, NULL);
    (void)run(&lf, 10U, 1000U, 0U);
    g_in.running = true;
    r = run(&lf, 200U, 1000U, 0U);
    CHECK(r.posts == 0U);
    CHECK(r.track_lost == 1U);
    CHECK(r.last.state == LF_ST_IDLE);
    g_in.running = false;
    (void)run(&lf, 5U, 1000U, 0U);
    g_in.running = true;
    r = run(&lf, 50U, 1000U, 0U);
    CHECK(r.track_lost == 1U);
    CHECK(r.posts == 0U);
    {
        lf_cal_t const low = { { 140U, 500U }, { 140U, 1140U } };   /* CENTRE span 360 */

        reset_inputs();
        lf_core_init(&lf, &low);
        g_in.running = true;
        r = run(&lf, 50U, 1000U, 0U);
        CHECK(r.posts == 0U);
        CHECK(r.track_lost == 1U);
    }
}

/* Test 7: edge tracking in both directions, and the RIGHT overshoot rule. */
static void test_tracking(void)
{
    lf_core_t lf;
    run_t r;

    start_tracking(&lf);
    r = run(&lf, 40U, 500U, 0U);
    CHECK(r.min_steer == 0);
    CHECK(r.max_steer == 0);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.last.line_found);

    r = run(&lf, 40U, 700U, 0U);                /* darker: car drifted left */
    CHECK(r.min_steer > 0);
    CHECK(r.min_steer == lf_edge_steer(700U));

    r = run(&lf, 40U, 300U, 0U);                /* lighter: car drifted right */
    CHECK(r.max_steer < 0);
    CHECK(r.max_steer == lf_edge_steer(300U));
    CHECK(r.last.state == LF_ST_ON_LINE);

    /* CENTRE on the line and RIGHT darkening at the same time: 22 mm apart
     * on an 18 mm line, a drift cannot do that, so it is a cross line
     * arriving: the steering is held (not "darker -> steer right"). */
    start_tracking(&lf);
    (void)run(&lf, 8U, 700U, 0U);
    r = run(&lf, 20U, 700U, 600U);
    CHECK(r.min_steer == lf_edge_steer(700U));
    CHECK(r.max_steer == lf_edge_steer(700U));
    CHECK(r.junctions == 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);

    /* Overshoot left across the whole line: CENTRE off the line, RIGHT on
     * it -> full right, and not lost (RIGHT still sees the line). */
    start_tracking(&lf);
    (void)run(&lf, 5U, 1000U, 0U);              /* CENTRE across the body */
    r = run(&lf, 40U, 0U, 900U);
    CHECK(r.min_steer == 127);
    CHECK(r.max_steer == 127);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.junctions == 0U);
    r = run(&lf, 20U, 0U, 0U);                  /* both off: still right */
    CHECK(r.min_steer == 127);
}

/* Test 8: junctions, spikes, holding the steering, re-arming. */
static void test_junction(void)
{
    lf_core_t lf;
    run_t r;

    /* 1- and 2-sample "cross lines" are noise: nothing happens. */
    start_tracking(&lf);
    for (uint32_t k = 1U; k <= 2U; k++)
    {
        r = run(&lf, k, 1000U, 1000U);
        CHECK(r.junctions == 0U);
        CHECK(r.last.state == LF_ST_ON_LINE);
        r = run(&lf, 20U, 500U, 0U);
        CHECK(r.junctions == 0U);
    }

    /* A real cross line: 3 samples both DARK -> exactly one JUNCTION, on
     * the 3rd sample; steering is held (0 from the edge), never full right. */
    start_tracking(&lf);
    {
        lf_out_t o1 = feed(&lf, 1000U, 1000U);
        lf_out_t o2 = feed(&lf, 1000U, 1000U);
        lf_out_t o3 = feed(&lf, 1000U, 1000U);

        CHECK(o1.event == LF_EV_NONE);
        CHECK(o2.event == LF_EV_NONE);
        CHECK(o3.event == LF_EV_JUNCTION);
        CHECK(o3.state == LF_ST_JUNCTION);
        CHECK(!o1.post_steer || (o1.steer_q7 == 0));
        CHECK(!o2.post_steer || (o2.steer_q7 == 0));
    }
    /* Straight across (no manoeuvre): steer 0 while on the cross line. */
    r = run(&lf, 30U, 1000U, 1000U);
    CHECK(r.junctions == 0U);                   /* one cross line, one event */
    CHECK(r.posts > 0U);
    CHECK(r.min_steer == 0);
    CHECK(r.max_steer == 0);
    CHECK(r.last.state == LF_ST_JUNCTION);
    /* RIGHT back on white for 3 samples: ON_LINE, re-armed. */
    r = run(&lf, 2U, 500U, 0U);
    CHECK(r.last.state == LF_ST_JUNCTION);
    r = run(&lf, 1U, 500U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    r = run(&lf, 40U, 500U, 0U);
    CHECK(r.junctions == 0U);
    r = run(&lf, 10U, 1000U, 1000U);            /* the next cross line */
    CHECK(r.junctions == 1U);

    /* RIGHT flickering (1-sample spikes) while tracking: no junction, no
     * full-right steering. */
    start_tracking(&lf);
    {
        uint32_t j = 0U;
        int32_t  maxs = -1000;

        for (uint32_t i = 0U; i < 400U; i++)
        {
            lf_out_t const o = feed(&lf, 500U, ((i % 10U) == 0U) ? 1000U : 0U);

            j += (o.event == LF_EV_JUNCTION) ? 1U : 0U;
            if (o.post_steer && (o.steer_q7 > maxs))
            {
                maxs = o.steer_q7;
            }
        }
        CHECK(j == 0U);
        CHECK(maxs == 0);
    }

    /* No junction outside ON_LINE (here: searching). */
    start_tracking(&lf);
    CHECK(until_state(&lf, 0U, 0U, LF_ST_REACQUIRING, 200U) <= 200U);
    r = run(&lf, 2U, 1000U, 1000U);
    CHECK(r.junctions == 0U);
}

/* Test 9: line lost, side memory, stop, search, TRACK_LOST, recovery. */
static void test_lost_and_search(void)
{
    lf_core_t lf;
    run_t r;
    uint32_t n;

    /* Slid off to the right (CENTRE fades, RIGHT never saw anything):
     * steer full LEFT at once, LINE_LOST after 100..115 ms. */
    start_tracking(&lf);
    r = run(&lf, 19U, 0U, 0U);                  /* 95 ms */
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.min_steer == -127);
    CHECK(r.max_steer < 0);
    start_tracking(&lf);
    n = until_state(&lf, 0U, 0U, LF_ST_LINE_LOST, 100U);
    CHECK((n * 5U) >= 100U);
    CHECK((n * 5U) <= 115U);

    /* LINE_LOST posts nothing for 120 ms (Motion's 100 ms hold + one
     * period), then the search starts on the remembered side (left). */
    r = run(&lf, 23U, 0U, 0U);
    CHECK(r.posts == 0U);
    CHECK(r.last.state == LF_ST_LINE_LOST);
    r = run(&lf, 1U, 0U, 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);
    CHECK(r.posts == 1U);
    CHECK(r.min_steer == -127);
    /* Left for 700 ms, then right, until T_search = 2 s after the start. */
    r = run(&lf, 139U, 0U, 0U);
    CHECK(r.max_steer == -127);
    r = run(&lf, 40U, 0U, 0U);
    CHECK(r.min_steer == 127);
    r = run(&lf, 219U, 0U, 0U);                 /* 1 + 139 + 40 + 219 = 399 */
    CHECK(r.track_lost == 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);
    r = run(&lf, 1U, 0U, 0U);                   /* the 400th sample = 2000 ms */
    CHECK(r.track_lost == 1U);
    CHECK(r.last.state == LF_ST_IDLE);
    r = run(&lf, 400U, 0U, 0U);                 /* still "running": stays quiet */
    CHECK(r.track_lost == 0U);
    CHECK(r.posts == 0U);
    CHECK(r.last.state == LF_ST_IDLE);

    /* Overshoot left: RIGHT saw the line just before CENTRE lost it ->
     * the line is on the right: steer full right, search right first. */
    start_tracking(&lf);
    (void)run(&lf, 5U, 1000U, 0U);              /* CENTRE across the body */
    (void)run(&lf, 2U, 500U, 300U);             /* left edge; RIGHT touches */
    r = run(&lf, 19U, 0U, 0U);
    CHECK(r.min_steer == 127);
    CHECK(until_state(&lf, 0U, 0U, LF_ST_REACQUIRING, 100U) <= 100U);
    r = run(&lf, 4U, 0U, 0U);
    CHECK(r.min_steer == 127);

    /* RIGHT saw something 150 ms before (outside the 100 ms memory): left. */
    start_tracking(&lf);
    (void)run(&lf, 1U, 500U, 300U);
    (void)run(&lf, 30U, 500U, 0U);
    r = run(&lf, 10U, 0U, 0U);
    CHECK(r.max_steer == -127);
    /* ...and 50 ms before (inside): right. */
    start_tracking(&lf);
    (void)run(&lf, 1U, 500U, 300U);
    (void)run(&lf, 10U, 500U, 0U);
    r = run(&lf, 10U, 0U, 0U);
    CHECK(r.min_steer == 127);

    /* The line comes back before T_lost: still ON_LINE, no LINE_LOST. */
    start_tracking(&lf);
    (void)run(&lf, 15U, 0U, 0U);
    r = run(&lf, 10U, 500U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);

    /* Found during the search: CENTRE solid black for 3 samples. The edge
     * alone (500) is not a capture. */
    start_tracking(&lf);
    CHECK(until_state(&lf, 0U, 0U, LF_ST_REACQUIRING, 200U) <= 200U);
    r = run(&lf, 50U, 500U, 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);
    CHECK(!r.last.line_found);
    r = run(&lf, 2U, 1000U, 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);
    r = run(&lf, 1U, 1000U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.last.line_found);
    CHECK(r.posts == 1U);                       /* posts again at once */
    CHECK(r.min_steer == 64);
    r = run(&lf, 40U, 500U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.track_lost == 0U);

    /* A new run after TRACK_LOST starts normally. */
    start_tracking(&lf);
    (void)until_state(&lf, 0U, 0U, LF_ST_IDLE, 1000U);
    g_in.running = false;
    (void)run(&lf, 3U, 1000U, 0U);
    g_in.running = true;
    r = run(&lf, 4U, 1000U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
}

/* Test 10: junction turns: silent during the turn, T_turn / T_uturn after. */
static void test_turns(void)
{
    lf_core_t lf;
    run_t r;
    uint32_t n;

    /* 90 degree turn: no steering post at all while it runs. */
    start_tracking(&lf);
    (void)run(&lf, 3U, 1000U, 1000U);           /* JUNCTION */
    g_in.manoeuvre = LF_MAN_TURN;
    r = run(&lf, 200U, 0U, 0U);                 /* sensors off the line, 1 s */
    CHECK(r.posts == 0U);
    CHECK(r.junctions == 0U);
    CHECK(r.track_lost == 0U);
    CHECK(r.last.state == LF_ST_JUNCTION);
    /* Turn done: steer 0 (straight) and wait up to 500 ms for the line. */
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&lf, 40U, 0U, 0U);
    CHECK(r.posts > 0U);
    CHECK(r.min_steer == 0);
    CHECK(r.max_steer == 0);
    r = run(&lf, 3U, 1000U, 0U);                /* found it */
    CHECK(r.last.state == LF_ST_ON_LINE);

    /* Not found: LINE_LOST at 500 ms after the turn, not before. */
    start_tracking(&lf);
    (void)run(&lf, 3U, 1000U, 1000U);
    g_in.manoeuvre = LF_MAN_TURN;
    (void)run(&lf, 50U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    n = until_state(&lf, 0U, 0U, LF_ST_LINE_LOST, 300U);
    CHECK((n * 5U) >= 500U);
    CHECK((n * 5U) <= 505U);

    /* U-turn: silent during it; after it, gentle right (the line is to the
     * right of CENTRE) for up to 1 s; the left edge alone is not a capture. */
    start_tracking(&lf);
    (void)run(&lf, 3U, 1000U, 1000U);
    g_in.manoeuvre = LF_MAN_UTURN;
    r = run(&lf, 300U, 0U, 0U);
    CHECK(r.posts == 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&lf, 100U, 500U, 0U);               /* landed on the left edge */
    CHECK(r.min_steer == 32);
    CHECK(r.max_steer == 32);
    CHECK(r.last.state == LF_ST_JUNCTION);
    r = run(&lf, 3U, 1000U, 0U);                /* walked into the body */
    CHECK(r.last.state == LF_ST_ON_LINE);
    r = run(&lf, 4U, 1000U, 0U);
    CHECK(r.min_steer == 64);                   /* now edge-tracking: right */

    start_tracking(&lf);
    (void)run(&lf, 3U, 1000U, 1000U);
    g_in.manoeuvre = LF_MAN_UTURN;
    (void)run(&lf, 50U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    n = until_state(&lf, 0U, 0U, LF_ST_LINE_LOST, 400U);
    CHECK((n * 5U) >= 1000U);
    CHECK((n * 5U) <= 1005U);
    CHECK(until_state(&lf, 0U, 0U, LF_ST_REACQUIRING, 100U) <= 100U);
    r = run(&lf, 4U, 0U, 0U);
    CHECK(r.min_steer == 127);                  /* search right first */

    /* After a turn the sensors may land on the cross line itself (both
     * black): back ON_LINE, but no second JUNCTION for the same cross line;
     * once RIGHT has been white, the next cross line is a junction again. */
    start_tracking(&lf);
    (void)run(&lf, 3U, 1000U, 1000U);
    g_in.manoeuvre = LF_MAN_TURN;
    (void)run(&lf, 50U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&lf, 20U, 1000U, 1000U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.junctions == 0U);
    r = run(&lf, 40U, 500U, 0U);
    CHECK(r.junctions == 0U);
    r = run(&lf, 10U, 1000U, 1000U);
    CHECK(r.junctions == 1U);

    /* A manoeuvre while tracking (no junction from LF): still silent. */
    start_tracking(&lf);
    g_in.manoeuvre = LF_MAN_TURN;
    r = run(&lf, 100U, 500U, 0U);
    CHECK(r.posts == 0U);
    g_in.manoeuvre = LF_MAN_NONE;
}

/* Test 11: suspended by avoidance, and the handoff back. */
static void test_suspended(void)
{
    lf_core_t lf;
    run_t r;

    start_tracking(&lf);
    g_in.avoid_active = true;
    r = run(&lf, 300U, 0U, 0U);                 /* off the line 1.5 s */
    CHECK(r.posts == 0U);
    CHECK(r.track_lost == 0U);
    CHECK(r.junctions == 0U);
    CHECK(r.last.state == LF_ST_SUSPENDED);
    CHECK(!r.last.line_found);
    r = run(&lf, 3U, 1000U, 1000U);             /* a cross line: not a junction */
    CHECK(r.junctions == 0U);
    r = run(&lf, 3U, 1000U, 0U);                /* the line under CENTRE */
    CHECK(r.last.line_found);
    CHECK(r.posts == 0U);
    /* Handoff with the line found: ON_LINE. */
    g_in.avoid_active = false;
    g_in.reacquired = true;
    r = run(&lf, 1U, 1000U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.posts == 1U);
    g_in.reacquired = false;

    /* Handoff without the line: search. */
    start_tracking(&lf);
    g_in.avoid_active = true;
    (void)run(&lf, 50U, 0U, 0U);
    g_in.avoid_active = false;
    g_in.reacquired = true;
    r = run(&lf, 1U, 0U, 0U);
    CHECK(r.last.state == LF_ST_REACQUIRING);
    g_in.reacquired = false;

    /* Waiting for the handoff (avoid flag already clear): still silent. */
    start_tracking(&lf);
    g_in.avoid_active = true;
    (void)run(&lf, 5U, 500U, 0U);
    g_in.avoid_active = false;
    r = run(&lf, 100U, 500U, 0U);
    CHECK(r.posts == 0U);
    CHECK(r.last.state == LF_ST_SUSPENDED);
}

/* ---------------------------------------------------------------------------
 * Slice 3: closed loop (lf_sim.h)
 * ------------------------------------------------------------------------- */

typedef struct
{
    uint32_t steps;
    double   max_err_mm;        /* |CENTRE - right edge| after settling */
    bool     always_on_line;    /* ON_LINE (or JUNCTION) after settling */
} loop_t;

/* Drive a straight line along x (from x = -500 to x_end).  CENTRE starts
 * at y_c with heading th.  Returns tracking statistics after settle_s. */
static loop_t drive_straight(sim_t *p_sim, double y_c, double th, double secs,
                             double settle_s, double x_end)
{
    lf_core_t lf;
    lf_out_t  out;
    loop_t    l;
    lf_cal_t  cal;

    l.steps = 0U;
    l.max_err_mm = 0.0;
    l.always_on_line = true;
    sim_add_seg(p_sim, -500.0, 0.0, x_end, 0.0);
    sim_place(p_sim, y_c, th);
    cal.c.white = (uint16_t)SIM_WHITE_RAW;
    cal.c.black = (uint16_t)(SIM_WHITE_RAW + 1000.0);   /* calibrated span 1000 */
    cal.r = cal.c;
    reset_inputs();
    lf_core_init(&lf, &cal);
    g_in.running = true;
    for (double t = 0.0; t < secs; t += SIM_DT_S)
    {
        sim_step(p_sim, &lf, &g_in, &out);
        l.steps++;
        if (t >= settle_s)
        {
            double const err = fabs(sim_centre_y(p_sim) + SIM_LINE_HALF_MM);

            l.max_err_mm = fmax(l.max_err_mm, err);
            if ((out.state != LF_ST_ON_LINE) && (out.state != LF_ST_JUNCTION))
            {
                l.always_on_line = false;
            }
        }
    }
    return l;
}

/* Test 12: straight line from the start pose, at several speeds, spacings
 * and spot sizes: settles on the right edge, no events.  Up to the nominal
 * cruise (300 mm/s, MOTION_SPEED_CRUISE_NOMINAL_CMPS) CENTRE stays within
 * 5 mm of the edge; at the 500 mm/s cap (NFR2) the 20 ms post period and
 * the wheel lag make it wobble, so there the pass is "never off the line":
 * within 9 mm (half the line width) of the edge. */
static void test_sim_straight(void)
{
    double const speeds[5] = { 100.0, 200.0, 300.0, 400.0, 500.0 };
    double const tol_mm[5] = { 5.0, 5.0, 5.0, 9.0, 9.0 };
    double const spacings[3] = { 20.0, 22.0, 24.0 };
    double const spots[3] = { 2.0, 3.0, 4.0 };

    for (int i = 0; i < 5; i++)
    {
        sim_t s;
        loop_t l;

        sim_init(&s);
        s.speed_mmps = speeds[i];
        l = drive_straight(&s, 0.0, 0.0, 6.0, 1.0, 5000.0);
        CHECK(l.always_on_line);
        CHECK(l.max_err_mm <= tol_mm[i]);
        CHECK(s.n_junction == 0U);
        CHECK(s.n_line_lost == 0U);
        CHECK(s.n_track_lost == 0U);
        printf("  straight %3.0f mm/s: max |CENTRE - edge| %.1f mm (limit %.0f)\n",
               speeds[i], l.max_err_mm, tol_mm[i]);
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            sim_t s;
            loop_t l;

            sim_init(&s);
            s.spacing_mm = spacings[i];
            s.spot_mm = spots[j];
            l = drive_straight(&s, 0.0, 0.0, 4.0, 1.0, 5000.0);
            CHECK(l.always_on_line);
            CHECK(l.max_err_mm <= 5.0);
            CHECK(s.n_junction == 0U);
            CHECK(s.n_line_lost == 0U);
        }
    }
}

/* Test 13: start offsets and headings, both drift directions. */
static void test_sim_offsets(void)
{
    double const ys[5] = { -6.0, -3.0, 0.0, 4.0, 7.0 };     /* CENTRE on the line */
    double const ths_deg[5] = { -15.0, -7.0, 0.0, 7.0, 15.0 };

    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            sim_t s;
            loop_t l;

            sim_init(&s);
            l = drive_straight(&s, ys[i], ths_deg[j] * SIM_PI / 180.0, 5.0, 2.0, 5000.0);
            CHECK(l.always_on_line);
            CHECK(l.max_err_mm <= 5.0);
            CHECK(s.n_junction == 0U);
            CHECK(s.n_track_lost == 0U);
            if (!(l.always_on_line && (l.max_err_mm <= 5.0)))
            {
                printf("  offset y %.0f th %.0f: max err %.1f, lost %u\n",
                       ys[i], ths_deg[j], l.max_err_mm, s.n_line_lost);
            }
        }
    }
}

/* Test 14: curves both ways (circle of radius R through the origin). */
static void test_sim_curves(void)
{
    double const radii[3] = { 1000.0, 500.0, 300.0 };

    for (int i = 0; i < 3; i++)
    {
        for (int dir = -1; dir <= 1; dir += 2)      /* -1 right, +1 left */
        {
            sim_t s;
            lf_core_t lf;
            lf_out_t out;
            lf_cal_t cal;
            double const R = radii[i];
            double const cy = (double)dir * R;
            double max_err = 0.0;
            bool on = true;

            sim_init(&s);
            sim_add_arc(&s, 0.0, cy, R);
            sim_place(&s, 0.0, 0.0);
            cal.c.white = 140U;
            cal.c.black = 1140U;
            cal.r = cal.c;
            reset_inputs();
            lf_core_init(&lf, &cal);
            g_in.running = true;
            for (double t = 0.0; t < 8.0; t += SIM_DT_S)    /* > one lap at R 300 */
            {
                sim_step(&s, &lf, &g_in, &out);
                if (t >= 1.5)
                {
                    double cxp;
                    double cyp;
                    double lat;

                    sim_centre_xy(&s, &cxp, &cyp);
                    /* Signed offset from the line centre, + = left of travel. */
                    lat = (dir > 0) ? (R - hypot(cxp, cyp - cy)) : (hypot(cxp, cyp - cy) - R);
                    max_err = fmax(max_err, fabs(lat + SIM_LINE_HALF_MM));
                    on = on && (out.state == LF_ST_ON_LINE);
                }
            }
            CHECK(on);
            CHECK(max_err <= 6.0);
            CHECK(s.n_junction == 0U);
            CHECK(s.n_line_lost == 0U);
            if (!(on && (max_err <= 6.0)))
            {
                printf("  curve R %.0f dir %d: max err %.1f, lost %u\n", R, dir, max_err, s.n_line_lost);
            }
        }
    }
}

/* Test 15: a cross junction, straight on (no barcode command), at 100 to
 * 500 mm/s: exactly one JUNCTION, back on the line, never lost. */
static void test_sim_junction(void)
{
    double const speeds[4] = { 100.0, 200.0, 300.0, 500.0 };

    for (int i = 0; i < 4; i++)
    {
        sim_t s;
        loop_t l;

        sim_init(&s);
        s.speed_mmps = speeds[i];
        sim_add_seg(&s, 800.0, -200.0, 800.0, 200.0);   /* cross line at x = 800 */
        l = drive_straight(&s, 0.0, 0.0, 2.0 + (1000.0 / speeds[i]), 1.0, 8000.0);
        CHECK(s.n_junction == 1U);
        CHECK(s.n_line_lost == 0U);
        CHECK(s.n_track_lost == 0U);
        CHECK(l.always_on_line);
        CHECK(s.last_state == LF_ST_ON_LINE);
        if (s.n_junction != 1U)
        {
            printf("  junction %.0f mm/s: %u junctions\n", speeds[i], s.n_junction);
        }
    }
    /* Two cross lines 300 mm apart: two junctions. */
    {
        sim_t s;

        sim_init(&s);
        sim_add_seg(&s, 600.0, -200.0, 600.0, 200.0);
        sim_add_seg(&s, 900.0, -200.0, 900.0, 200.0);
        (void)drive_straight(&s, 0.0, 0.0, 7.0, 1.0, 8000.0);
        CHECK(s.n_junction == 2U);
        CHECK(s.n_line_lost == 0U);
    }
}

/* Test 16: ride height (signal span differs from the calibration) and
 * noise. */
static void test_sim_height_noise(void)
{
    double const spans[4] = { 800.0, 1000.0, 1500.0, 3000.0 };  /* x0.8 .. x3 */

    for (int i = 0; i < 4; i++)
    {
        sim_t s;
        loop_t l;

        sim_init(&s);
        s.span_raw = spans[i];
        sim_add_seg(&s, 800.0, -200.0, 800.0, 200.0);
        l = drive_straight(&s, 0.0, 0.0, 6.0, 1.0, 5000.0);
        CHECK(l.always_on_line);
        CHECK(s.n_junction == 1U);
        CHECK(s.n_line_lost == 0U);
        if (!l.always_on_line || (s.n_junction != 1U))
        {
            printf("  span %.0f: on %d, junctions %u, max err %.1f\n",
                   spans[i], (int)l.always_on_line, s.n_junction, l.max_err_mm);
        }
    }
    /* Heavy noise (+-80 raw, more than twice LF-1's): no false events. */
    {
        sim_t s;
        loop_t l;

        sim_init(&s);
        s.noise_raw = 80.0;
        l = drive_straight(&s, 0.0, 0.0, 10.0, 1.0, 5000.0);
        CHECK(l.always_on_line);
        CHECK(s.n_junction == 0U);
        CHECK(s.n_line_lost == 0U);
    }
}

/* Test 17: knocked off the line (a bump swings the heading), both ways:
 * the side memory sends the search the right way, the car gets back. */
static void test_sim_knock(void)
{
    for (int dir = -1; dir <= 1; dir += 2)          /* -1 knocked right, +1 left */
    {
        sim_t s;
        lf_core_t lf;
        lf_out_t out;
        lf_cal_t cal;
        bool back = false;

        sim_init(&s);
        sim_add_seg(&s, -500.0, 0.0, 8000.0, 0.0);
        sim_place(&s, 0.0, 0.0);
        cal.c.white = 140U;
        cal.c.black = 1140U;
        cal.r = cal.c;
        reset_inputs();
        lf_core_init(&lf, &cal);
        g_in.running = true;
        for (double t = 0.0; t < 6.0; t += SIM_DT_S)
        {
            if (fabs(t - 1.0) < (SIM_DT_S / 2.0))
            {
                s.th += (double)dir * 35.0 * SIM_PI / 180.0;    /* the knock */
            }
            sim_step(&s, &lf, &g_in, &out);
            if ((t > 1.0) && (out.state == LF_ST_ON_LINE) && (s.n_line_lost > 0U))
            {
                back = true;
            }
        }
        CHECK(s.n_track_lost == 0U);
        CHECK(out.state == LF_ST_ON_LINE);
        CHECK(fabs(sim_centre_y(&s) + SIM_LINE_HALF_MM) <= 5.0);
        if ((s.n_track_lost != 0U) || (out.state != LF_ST_ON_LINE))
        {
            printf("  knock %d: state %d, line_lost %u, track_lost %u, y_c %.1f, back %d\n",
                   dir, (int)out.state, s.n_line_lost, s.n_track_lost, sim_centre_y(&s), (int)back);
        }
    }
}

/* Test 18: the line ends: lost, search both ways, one TRACK_LOST, stop. */
static void test_sim_line_end(void)
{
    sim_t s;
    loop_t l;

    sim_init(&s);
    l = drive_straight(&s, 0.0, 0.0, 10.0, 0.0, 1000.0);   /* line ends at x = 1 m */
    (void)l;
    CHECK(s.n_line_lost == 1U);
    CHECK(s.n_track_lost == 1U);
    CHECK(s.last_state == LF_ST_IDLE);
    CHECK(!s.have_cmd);                         /* the car is stopped */
}

int main(void)
{
    test_constants();
    test_calibration();
    test_norm();
    test_zones();
    test_edge_steer();
    test_idle_and_start();
    test_tracking();
    test_junction();
    test_lost_and_search();
    test_turns();
    test_suspended();
    test_sim_straight();
    test_sim_offsets();
    test_sim_curves();
    test_sim_junction();
    test_sim_height_noise();
    test_sim_knock();
    test_sim_line_end();
    return TEST_RESULT("lf_core");
}
