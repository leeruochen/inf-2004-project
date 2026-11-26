/*
 *  test_lf_straddle.c
 *  Host tests for the straddle line-following core
 *  (app_program/subsystem3/lf_straddle.c) and the two-pose calibration
 *  (lf_core.c lf_cal_from_two_poses).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Expected values are worked by hand from the constants:
 *    20 mm etc. in counts = mm x 2464 / 207 (ENC_QUAD_MULT x ENC_CPR /
 *    WHEEL_CIRC_MM); the fixture moves 12 counts per 5 ms sample (~200 mm/s).
 *    steering = LF_SD_KP_Q7 (127) x (dr - dl) / 1000, truncated towards 0.
 *  The closed-loop checks use tests/host/lf_sim.h with the sensors mounted
 *  one each side of the line (spacing 26 mm, CENTRE = LEFT at +13 mm).
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config/thresholds.h"
#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"
#include "app_program/subsystem3/lf_sanity.h"
#include "app_program/subsystem3/lf_straddle.h"
#include "test_check.h"
#include "lf_sim.h"

static int g_deferred = 0;
static int g_deferred_failing = 0;

/* A check that is reported, not yet a gate. */
#define DEFER(cond)                                 \
    do                                              \
    {                                               \
        g_deferred++;                               \
        g_deferred_failing += (cond) ? 0 : 1;       \
    } while (0)

/* ---------------------------------------------------------------------------
 * Fixtures
 * ------------------------------------------------------------------------- */

static lf_cal_t const k_cal = { { 140U, 1140U }, { 140U, 1140U } };
static lf_in_t        g_in;
static uint32_t       g_fwd_step;
static uint32_t       g_sample_n;

/* Hand-worked counts. */
#define CONF_SAMPLES    (119U)      /* 120 mm = 1428 counts / 12 = 119 samples */
#define PROBE_SAMPLES   (60U)       /* 60 mm = 714 counts / 12 = 59.5 -> 60 */
#define DEEP_SAMPLES    (6U)        /* 6 mm = 71 counts / 12 -> 6 */
#define POST_SAMPLES    (119U)      /* LF_D_JCT_POST 120 mm = 1428 counts / 12 */

static uint16_t raw_of(uint16_t d_pm)
{
    return (uint16_t)(140U + d_pm);
}

static lf_out_t feed(lf_sd_t *p_sd, uint16_t dl, uint16_t dr)
{
    lf_out_t out;

    g_in.raw_c = raw_of(dl);
    g_in.raw_r = raw_of(dr);
    g_in.fwd_cnt += g_fwd_step;
    g_in.post_slot = ((g_sample_n % 4U) == 3U);
    g_sample_n++;
    memset(&out, 0xA5, sizeof(out));        /* every field must be written */
    lf_sd_step(p_sd, &g_in, &out);
    return out;
}

static lf_out_t feed_raw(lf_sd_t *p_sd, uint16_t raw_l, uint16_t raw_r)
{
    lf_out_t out;

    g_in.raw_c = raw_l;
    g_in.raw_r = raw_r;
    g_in.fwd_cnt += g_fwd_step;
    g_in.post_slot = ((g_sample_n % 4U) == 3U);
    g_sample_n++;
    lf_sd_step(p_sd, &g_in, &out);
    return out;
}

typedef struct
{
    uint32_t posts;
    uint32_t junctions;
    uint32_t track_lost;
    int32_t  min_steer;
    int32_t  max_steer;
    lf_out_t last;
} run_t;

static run_t run(lf_sd_t *p_sd, uint32_t n, uint16_t dl, uint16_t dr)
{
    run_t r;

    memset(&r, 0, sizeof(r));
    r.min_steer = 1000;
    r.max_steer = -1000;
    for (uint32_t i = 0U; i < n; i++)
    {
        lf_out_t const o = feed(p_sd, dl, dr);

        if (o.post_steer)
        {
            r.posts++;
            r.min_steer = (o.steer_q7 < r.min_steer) ? o.steer_q7 : r.min_steer;
            r.max_steer = (o.steer_q7 > r.max_steer) ? o.steer_q7 : r.max_steer;
        }
        r.junctions += (o.event == LF_EV_JUNCTION) ? 1U : 0U;
        r.track_lost += (o.event == LF_EV_TRACK_LOST) ? 1U : 0U;
        r.last = o;
    }
    return r;
}

static void reset_inputs(void)
{
    memset(&g_in, 0, sizeof(g_in));
    g_in.manoeuvre = LF_MAN_NONE;
    g_fwd_step = 12U;
    g_sample_n = 0U;
}

/* Calibrated, running, centred on the line (both white). */
static void start_centred(lf_sd_t *p_sd)
{
    reset_inputs();
    lf_sd_init(p_sd, &k_cal);
    (void)run(p_sd, 5U, 0U, 0U);
    g_in.running = true;
    (void)run(p_sd, 1U, 0U, 0U);
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
 * Calibration
 * ------------------------------------------------------------------------- */

/* Test 1: two-pose calibration, each sensor on its own readings; the car's
 * own numbers (19 Nov 2026) as the cases. */
static void test_two_pose_cal(void)
{
    lf_cal2_t r;
    lf_adc_stats_t cb, rw, cw, rb;

    /* Good mounting, 21:39: CENTRE black 1888, RIGHT black 2124, whites ~150-167. */
    cb = stats_const(1888U, 32U); rw = stats_const(157U, 32U);
    cw = stats_const(167U, 32U); rb = stats_const(2124U, 32U);
    CHECK(lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    CHECK((r.c == LF_SENS_OK) && (r.r == LF_SENS_OK));
    CHECK((r.cal.c.white == 167U) && (r.cal.c.black == 1888U));    /* CENTRE's own */
    CHECK((r.cal.r.white == 157U) && (r.cal.r.black == 2124U));    /* RIGHT's own, not CENTRE's */

    /* Lowered sensors, 22:04: CENTRE black 702 (span 535): accepted now, the
     * follower handles it (host model, span 560).  Straddle mount, 20 Nov:
     * black 1274, white 145. */
    cb = stats_const(702U, 32U);
    CHECK(lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    cb = stats_const(1274U, 32U);
    CHECK(lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    /* Below the criteria: black 580 (span 413, above the core's 400 but
     * under 600 / 450) -> WEAK, refused; black 450 (span 283) -> BLIND. */
    cb = stats_const(580U, 32U);
    CHECK(!lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    CHECK((r.c == LF_SENS_WEAK) && (r.r == LF_SENS_OK));
    cb = stats_const(450U, 32U);
    CHECK(!lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    CHECK(r.c == LF_SENS_BLIND);

    /* Blind RIGHT: a steady white-ish level in both poses (passes the rail
     * and noise checks), which the old single-pose rule never saw. */
    cb = stats_const(1888U, 32U); rb = stats_const(160U, 32U);   /* RIGHT: 160 over black, 157 over white */
    CHECK(!lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    CHECK((r.c == LF_SENS_OK) && (r.r == LF_SENS_BLIND));

    /* Disconnected RIGHT (at the high rail) and noisy CENTRE. */
    rb = stats_const(4095U, 32U); rw = stats_const(4095U, 32U);
    CHECK(!lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    CHECK(r.r == LF_SENS_DISCONNECTED);
    rw = stats_const(157U, 32U); rb = stats_const(2124U, 32U);
    lf_adc_stats_reset(&cb);
    lf_adc_stats_add(&cb, 1400U);
    lf_adc_stats_add(&cb, 2000U);       /* spread 600 > 400 */
    CHECK(!lf_cal_from_two_poses(&cb, &rw, &cw, &rb, &r));
    CHECK(r.c == LF_SENS_NOISY);

    /* NULLs: refused, out filled. */
    CHECK(!lf_cal_from_two_poses(NULL, &rw, &cw, &rb, &r));
    CHECK((r.c == LF_SENS_DISCONNECTED) && (r.r == LF_SENS_DISCONNECTED));
    CHECK(!lf_cal_from_two_poses(&cb, &rw, &cw, &rb, NULL));
}

/* ---------------------------------------------------------------------------
 * The core, open loop
 * ------------------------------------------------------------------------- */

/* Test 2: start, sign and size of the steering. */
static void test_steer(void)
{
    lf_sd_t sd;
    run_t r;

    /* No calibration: TRACK_LOST once at run start, never steers. */
    reset_inputs();
    lf_sd_init(&sd, NULL);
    g_in.running = true;
    r = run(&sd, 10U, 0U, 0U);
    CHECK(r.track_lost == 1U);
    CHECK(r.posts == 0U);

    start_centred(&sd);
    r = run(&sd, 8U, 0U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK((r.min_steer == 0) && (r.max_steer == 0));     /* centred: straight */
    CHECK(r.last.line_found);

    r = run(&sd, 8U, 500U, 0U);                         /* line under LEFT */
    CHECK((r.min_steer == -63) && (r.max_steer == -63));  /* 127 x -500 / 1000 = -63.5 */
    r = run(&sd, 8U, 0U, 1000U);                        /* under RIGHT */
    CHECK((r.min_steer == 127) && (r.max_steer == 127));
    r = run(&sd, 8U, 300U, 300U);                       /* both partly: centred */
    CHECK((r.min_steer == 0) && (r.max_steer == 0));
    CHECK(r.last.d_c_pm == 300U);                       /* LEFT darkness in d_c */
}

/* Test 3: both white is not assumed for long.  Centred, the core goes on
 * straight for 120 mm, then probes towards the side last touched for at most
 * 60 mm, then LINE_LOST; a touch during the probe keeps it tracking. */
static void test_both_white(void)
{
    lf_sd_t sd;
    run_t r;
    uint32_t n;

    start_centred(&sd);
    (void)run(&sd, 4U, 0U, 400U);                       /* a shallow touch on the RIGHT */
    r = run(&sd, CONF_SAMPLES - 2U, 0U, 0U);
    CHECK((r.min_steer == 0) && (r.max_steer == 0));
    CHECK(sd.bw == LF_SD_BW_CENTRED);
    r = run(&sd, 8U, 0U, 0U);
    CHECK(sd.bw == LF_SD_BW_PROBE);
    CHECK(r.max_steer == LF_SD_PROBE_Q7);               /* right, the side touched */
    r = run(&sd, 4U, 0U, 300U);                         /* the probe finds the line */
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(sd.bw == LF_SD_BW_NONE);

    /* Gone: nothing touches; LINE_LOST at 120 + 60 mm, then the stop, then the
     * search, and TRACK_LOST after its 150 mm (a loss while driving). */
    n = 0U;
    while ((feed(&sd, 0U, 0U).state == LF_ST_ON_LINE) && (n < 400U))
    {
        n++;
    }
    CHECK((n + 1U) >= (CONF_SAMPLES + PROBE_SAMPLES - 2U));
    CHECK((n + 1U) <= (CONF_SAMPLES + PROBE_SAMPLES + 2U));
    r = run(&sd, LF_SAMPLES(LF_T_STOP_MS) - 1U, 0U, 0U);
    CHECK(r.posts == 0U);                               /* stopping: nothing posted */
    r = run(&sd, 160U, 0U, 0U);                         /* 150 mm = 1785 counts / 12 -> 149 */
    CHECK(r.track_lost == 1U);
    CHECK(r.last.state == LF_ST_IDLE);
}

/* Test 4: a deep one-sided contact (the line's body under RIGHT for more
 * than 6 mm, LEFT white) then both white: hold right at LF_SD_HOLD_Q7; a
 * brief touch is not deep. */
static void test_deep_hold(void)
{
    lf_sd_t sd;
    run_t r;

    start_centred(&sd);
    (void)run(&sd, DEEP_SAMPLES + 2U, 0U, 1000U);
    CHECK(sd.deep);
    r = run(&sd, 10U, 0U, 0U);
    CHECK(sd.bw == LF_SD_BW_HOLD);
    CHECK((r.min_steer == LF_SD_HOLD_Q7) && (r.max_steer == LF_SD_HOLD_Q7));

    start_centred(&sd);
    (void)run(&sd, 2U, 0U, 1000U);                      /* 2 samples: 24 counts < 71 */
    CHECK(!sd.deep);
    r = run(&sd, 10U, 0U, 0U);
    CHECK(sd.bw == LF_SD_BW_CENTRED);
    CHECK((r.min_steer == 0) && (r.max_steer == 0));

    /* The hold ends at LF_SD_HOLD_DEG of turn (yaw_cnt), before its distance. */
    start_centred(&sd);
    (void)run(&sd, DEEP_SAMPLES + 2U, 0U, 1000U);
    {
        uint32_t n = 0U;

        while ((feed(&sd, 0U, 0U).state == LF_ST_ON_LINE) && (n < 400U))
        {
            g_in.yaw_cnt += 40;                         /* turning right */
            n++;
        }
        /* 120 deg = 120 x 1065 / 45 = 2840 counts / 40 = 71 samples */
        CHECK((n >= (LF_SD_HOLD_YAW_CNT / 40U) - 2U) && (n <= (LF_SD_HOLD_YAW_CNT / 40U) + 2U));
    }
}

/* Test 5: junction.  Both DARK (3-sample zone filter, then LF_SD_JCT_N = 6):
 * JUNCTION after 120 mm of travel, no steering in that sample; the VC's turn
 * is never fought; afterwards ON_LINE checks at once that the line is
 * between the sensors. */
static void test_junction(void)
{
    lf_sd_t sd;
    run_t r;
    uint32_t n = 0U;
    bool post_in_jct_sample = false;

    start_centred(&sd);
    r = run(&sd, 8U, 1000U, 1000U);
    CHECK(r.last.state == LF_ST_JUNCTION);
    CHECK(r.junctions == 0U);
    for (uint32_t i = 0U; (i < 300U) && (n == 0U); i++)
    {
        lf_out_t const o = feed(&sd, 0U, 0U);

        if (o.event == LF_EV_JUNCTION)
        {
            n = i + 1U;
            post_in_jct_sample = o.post_steer;
        }
    }
    CHECK((n + 8U >= POST_SAMPLES - 4U) && (n + 8U <= POST_SAMPLES + 8U));
    CHECK(!post_in_jct_sample);
    /* No turn: back to ON_LINE within two samples. */
    r = run(&sd, 3U, 0U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.junctions == 0U);

    /* A VC turn: nothing posted while it runs; then ON_LINE, probing. */
    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_TURN;
    r = run(&sd, 100U, 0U, 0U);
    CHECK(r.posts == 0U);
    CHECK(r.last.state == LF_ST_JUNCTION);
    (void)run(&sd, 3U, 0U, 600U);                       /* RIGHT sees the line during it */
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 0U, 0U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(sd.bw == LF_SD_BW_PROBE);
    CHECK(r.max_steer == LF_SD_PROBE_Q7);               /* towards the RIGHT */
}

/* Test 5b: the probe side after a turn when nothing clear was seen during it (20 Nov, right turn
 * at 5 cm/s: the line was faint on the RIGHT, the car probed LEFT and lost it, 2 of 3 runs). */
static void test_turn_side(void)
{
    lf_sd_t sd;
    run_t r;

    /* faintly darker RIGHT at the end of the turn (76 against 23 per mille): probe RIGHT */
    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_TURN;
    (void)run(&sd, 100U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 23U, 76U);
    CHECK(sd.state == LF_ST_REACQUIRING);              /* only an edge in view: the search, not the probe */
    CHECK(r.max_steer == LF_SEARCH_Q7);

    /* faintly darker LEFT: probe LEFT */
    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_TURN;
    (void)run(&sd, 100U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 76U, 23U);
    CHECK(r.min_steer == -LF_SEARCH_Q7);

    /* the 20 Nov U-turn: a clear LEFT contact mid-pivot, a faint RIGHT reading at the end: RIGHT */
    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_UTURN;
    (void)run(&sd, 50U, 0U, 0U);
    (void)run(&sd, 3U, 600U, 0U);                           /* LEFT crosses the line mid-pivot */
    (void)run(&sd, 50U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 5U, 66U);                              /* the end: RIGHT faint, LEFT quiet */
    CHECK(sd.state == LF_ST_REACQUIRING);
    CHECK(r.max_steer == LF_SEARCH_Q7);

    /* the 20 Nov U-turn, the later runs: both sensors quiet at the end, a clear LEFT contact mid-pivot,
     * the pivot turned right (yaw_cnt grows): keep rotating RIGHT, in the search, not a probe left */
    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_UTURN;
    (void)run(&sd, 3U, 0U, 0U);
    g_in.yaw_cnt += 1500;
    (void)run(&sd, 3U, 600U, 0U);                           /* LEFT crosses the line mid-pivot */
    g_in.yaw_cnt += 1500;
    (void)run(&sd, 3U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 0U, 0U);
    CHECK(sd.state == LF_ST_REACQUIRING);
    CHECK(r.max_steer == LF_SEARCH_Q7);
    CHECK(r.min_steer > -LF_SEARCH_Q7);

    /* nothing at all at the end: the turn's own direction (yaw_cnt grows turning right) */
    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_TURN;
    (void)run(&sd, 3U, 0U, 0U);
    g_in.yaw_cnt += 2000;                                   /* a right turn */
    (void)run(&sd, 3U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 0U, 0U);
    CHECK(r.max_steer == LF_SD_PROBE_Q7);

    start_centred(&sd);
    g_in.manoeuvre = LF_MAN_TURN;
    (void)run(&sd, 3U, 0U, 0U);
    g_in.yaw_cnt -= 2000;                                   /* a left turn */
    (void)run(&sd, 3U, 0U, 0U);
    g_in.manoeuvre = LF_MAN_NONE;
    r = run(&sd, 2U, 0U, 0U);
    CHECK(r.min_steer == -LF_SD_PROBE_Q7);
}

/* Test 5c: the search uses faint edge readings (20 Nov, U-turn at 5 cm/s: 95 and 77 per mille, the
 * line between the sensors, never a clear contact).  Both faint and equal: ON_LINE, straight.
 * One side faint: steer towards it, still searching. */
static void test_search_seek(void)
{
    lf_sd_t sd;
    run_t r;
    uint32_t n = 0U;

    start_centred(&sd);
    while ((sd.state != LF_ST_REACQUIRING) && (n < 4000U))
    {
        (void)feed(&sd, 0U, 0U);
        n++;
    }
    CHECK(sd.state == LF_ST_REACQUIRING);
    r = run(&sd, 2U, 95U, 77U);                           /* both faint, nearly equal */
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.last.steer_q7 == 0);

    start_centred(&sd);
    n = 0U;
    while ((sd.state != LF_ST_REACQUIRING) && (n < 4000U))
    {
        (void)feed(&sd, 0U, 0U);
        n++;
    }
    r = run(&sd, 2U, 0U, 80U);                            /* only RIGHT faint: towards it */
    CHECK(r.last.state == LF_ST_REACQUIRING);
    CHECK(r.last.steer_q7 == LF_SD_SEEK_Q7);
    r = run(&sd, 2U, 85U, 0U);                            /* only LEFT faint */
    CHECK(r.last.steer_q7 == -LF_SD_SEEK_Q7);
    r = run(&sd, 2U, 20U, 20U);                           /* nothing faint enough: the arc continues */
    CHECK(r.last.state == LF_ST_REACQUIRING);
    CHECK((r.last.steer_q7 == LF_SEARCH_Q7) || (r.last.steer_q7 == -LF_SEARCH_Q7));
}

/* Test 6: avoidance hand-back and a sensor fault. */
static void test_suspended_and_fault(void)
{
    lf_sd_t sd;
    run_t r;

    start_centred(&sd);
    g_in.avoid_active = true;
    r = run(&sd, 50U, 0U, 0U);
    CHECK(r.posts == 0U);
    CHECK(r.last.state == LF_ST_SUSPENDED);
    CHECK(!r.last.line_found);
    r = run(&sd, 5U, 0U, 800U);                         /* the bypass reaches the line */
    CHECK(r.last.line_found);                           /* LINE_FOUND for Buddy 5 */
    g_in.avoid_active = false;
    g_in.reacquired = true;
    r = run(&sd, 2U, 0U, 800U);
    CHECK(r.last.state == LF_ST_ON_LINE);
    CHECK(r.max_steer > 0);
    g_in.reacquired = false;

    /* RIGHT stuck at the low rail (a disconnected or shorted sensor) for 40
     * samples: TRACK_LOST, then IDLE.  (The high rail is caught earlier, as
     * "over", in test_floor.) */
    start_centred(&sd);
    {
        uint32_t tl = 0U;

        for (uint32_t i = 0U; i < 39U; i++)
        {
            tl += (feed_raw(&sd, 140U, 0U).event == LF_EV_TRACK_LOST) ? 1U : 0U;
        }
        CHECK(tl == 0U);
        CHECK(feed_raw(&sd, 140U, 0U).event == LF_EV_TRACK_LOST);
        CHECK(sd.state == LF_ST_IDLE);
    }
}

/* Test 7: the floor is not a line (the 20 Nov log: both sensors read 3500-3950
 * raw for 1.7 m after the car left the mat, and the core drove on, found the
 * "line", reported two junctions and kept LINE_FOUND set).  The tape never
 * read above 2706 in four logs; LF_SD_OVER_RAW is 3000, held for 10 samples. */
static void test_floor(void)
{
    lf_sd_t sd;
    run_t r;
    uint32_t tl = 0U;

    /* The tape's peak is still the tape: no stop, the steering follows it. */
    start_centred(&sd);
    for (uint32_t i = 0U; i < 40U; i++)
    {
        r.last = feed_raw(&sd, 140U, 2706U);
        tl += (r.last.event == LF_EV_TRACK_LOST) ? 1U : 0U;
    }
    CHECK(tl == 0U);
    CHECK(sd.state != LF_ST_IDLE);

    /* Both sensors on the floor (3800, 3700): TRACK_LOST on the 10th sample,
     * once; never a junction; the core goes IDLE. */
    start_centred(&sd);
    tl = 0U;
    for (uint32_t i = 0U; i < 9U; i++)
    {
        lf_out_t const o = feed_raw(&sd, 3800U, 3700U);

        tl += (o.event == LF_EV_TRACK_LOST) ? 1U : 0U;
        CHECK(o.event != LF_EV_JUNCTION);
    }
    CHECK(tl == 0U);
    CHECK(feed_raw(&sd, 3800U, 3700U).event == LF_EV_TRACK_LOST);
    CHECK(sd.state == LF_ST_IDLE);
    CHECK(!feed_raw(&sd, 3800U, 3700U).line_found);

    /* A brief floor reading (9 samples), then tape again: no stop. */
    start_centred(&sd);
    for (uint32_t i = 0U; i < 9U; i++)
    {
        (void)feed_raw(&sd, 140U, 3179U);
    }
    r.last = feed_raw(&sd, 140U, 900U);
    CHECK(r.last.event == LF_EV_NONE);
    CHECK(sd.state == LF_ST_ON_LINE);

    /* The search does not accept the floor as the line (20 Nov: RIGHT 3179
     * ended the search): in REACQUIRING, floor readings stop the car. */
    start_centred(&sd);
    (void)run(&sd, 30U, 0U, 0U);
    {
        uint32_t n = 0U;

        while ((sd.state != LF_ST_REACQUIRING) && (n < 400U))
        {
            (void)feed(&sd, 0U, 0U);
            n++;
        }
        CHECK(sd.state == LF_ST_REACQUIRING);
        tl = 0U;
        for (uint32_t i = 0U; i < 10U; i++)
        {
            tl += (feed_raw(&sd, 3300U, 3600U).event == LF_EV_TRACK_LOST) ? 1U : 0U;
        }
        CHECK(tl == 1U);
        CHECK(sd.state == LF_ST_IDLE);
    }
}

/* Test 11: the search ends on DISTANCE, not on lf_core's 2 s.  20 Nov, left junction turn at
 * 5 cm/s: the search ran 2.0 s = 47 mm of travel and gave up.  A car at ~3 counts per 5 ms sample
 * (50 mm/s) must search the whole capped distance before TRACK_LOST. */
static void test_slow_search(void)
{
    lf_sd_t sd;
    uint32_t n = 0U;
    uint32_t t0 = 0U;
    bool lost = false;
    uint32_t const fwd0 = 0U;

    (void)fwd0;
    start_centred(&sd);
    g_fwd_step = 3U;                                    /* ~50 mm/s */
    while ((sd.state != LF_ST_REACQUIRING) && (n < 4000U))
    {
        (void)feed(&sd, 0U, 0U);                        /* both white: loss, then the search */
        n++;
    }
    CHECK(sd.state == LF_ST_REACQUIRING);
    t0 = g_in.fwd_cnt;
    n = 0U;
    while (!lost && (n < 6000U))
    {
        lost = (feed(&sd, 0U, 0U).event == LF_EV_TRACK_LOST);
        n++;
    }
    CHECK(lost);
    /* at least 90% of the capped distance (the sample the car was in when it began is not counted) */
    CHECK((g_in.fwd_cnt - t0) >= ((LF_D_SEARCH_MAX_CNT * 9U) / 10U));
    CHECK((g_in.fwd_cnt - t0) <= (LF_D_SEARCH_MAX_CNT + 24U));
    CHECK(n > 400U);                                    /* longer than the old 2 s */
    g_fwd_step = 12U;
}

/* Test 8: both sensors DARK outside a junction is not a line.  After a
 * cross line the junction travel covers 120 mm; both DARK for another 60 mm
 * (714 counts = 60 samples at 12) in ON_LINE is LINE_LOST, not steering 0
 * for ever. */
static void test_both_dark(void)
{
    lf_sd_t sd;
    uint32_t n = 0U;
    uint32_t junctions = 0U;

    start_centred(&sd);
    for (uint32_t i = 0U; i < 8U; i++)
    {
        (void)feed(&sd, 1000U, 1000U);
    }
    CHECK(sd.state == LF_ST_JUNCTION);
    /* the car goes on over a dark surface: the junction posts, then ON_LINE,
     * then the 60 mm limit */
    while ((sd.state != LF_ST_LINE_LOST) && (n < 600U))
    {
        lf_out_t const o = feed(&sd, 1000U, 1000U);

        junctions += (o.event == LF_EV_JUNCTION) ? 1U : 0U;
        n++;
    }
    CHECK(sd.state == LF_ST_LINE_LOST);
    CHECK(junctions == 1U);                 /* one JUNCTION, then lost */
    CHECK((n >= 119U + 60U - 6U) && (n <= 119U + 60U + 12U));
}

/* Test 9: a graze while the car has been steering one way is a bend: hold
 * that way, not straight.  (20 Nov, drive 1: RIGHT grazed at 606 mm after
 * steering +96, both white, the core went straight 110 mm, the line was lost.)
 * A weave on a straight, steering both ways, still goes straight on. */
static void test_bend_memory(void)
{
    lf_sd_t sd;
    run_t r;

    start_centred(&sd);
    for (uint32_t i = 0U; i < 60U; i++)
    {
        (void)feed(&sd, 0U, 700U);          /* the line under RIGHT: steers right (~+67) */
    }
    (void)run(&sd, 3U, 0U, 80U);            /* a graze: 80 pm, shallow */
    CHECK(!sd.deep);
    r = run(&sd, 8U, 0U, 0U);               /* both white */
    CHECK(sd.bw == LF_SD_BW_HOLD);
    CHECK((r.min_steer == LF_SD_HOLD_Q7) && (r.max_steer == LF_SD_HOLD_Q7));

    /* Weave: right then left in turn, a graze, both white: straight on. */
    start_centred(&sd);
    for (uint32_t k = 0U; k < 6U; k++)
    {
        (void)run(&sd, 10U, 0U, 700U);
        (void)run(&sd, 10U, 700U, 0U);
    }
    (void)run(&sd, 3U, 0U, 80U);
    r = run(&sd, 8U, 0U, 0U);
    CHECK(sd.bw == LF_SD_BW_CENTRED);
    CHECK((r.min_steer == 0) && (r.max_steer == 0));
}

/* ---------------------------------------------------------------------------
 * Closed loop (lf_sim.h), sensors straddling the line 26 mm apart
 * ------------------------------------------------------------------------- */

static void sd_fn(void *p_core, lf_in_t const *p_in, lf_out_t *p_out)
{
    lf_sd_step((lf_sd_t *)p_core, p_in, p_out);
}

/* The car's steady full-lock turn at 7 cm/s (S3-U18, 20 Nov 2026, open floor): left 0.60
 * deg/mm, right 0.9 measured (the model's right is 0.69, conservative). */
static bool g_fitted = false;

static void sim_straddle(sim_t *p_s, double v, uint32_t seed)
{
    sim_init(p_s);
    if (g_fitted)
    {
        p_s->steer_dz = 0.0;
        p_s->steer_gain = 1.0;
        p_s->steer_gain_left = 0.87;
    }
    p_s->rng = seed;
    p_s->speed_mmps = v;
    p_s->spacing_mm = 26.0;
    p_s->c_lat_mm = 13.0;
}

/* Sensors' midpoint at (x_s, y_mid), heading th. */
static void sim_start(sim_t *p_s, lf_sd_t *p_sd, double x_s, double y_mid, double th)
{
    p_s->th = th;
    p_s->x = x_s - (p_s->fwd_mm * cos(th));
    p_s->y = y_mid - (p_s->fwd_mm * sin(th));
    reset_inputs();
    lf_sd_init(p_sd, &k_cal);
    g_in.running = true;
}

/* A bend: true if, 300 mm past turning deg, the car is ON_LINE with no
 * TRACK_LOST and no junction. */
static bool sim_bend(double R, int dir, double deg, double v, uint32_t seed)
{
    sim_t s;
    lf_sd_t sd;
    lf_out_t out;
    double past = -1.0;
    bool ok = false;

    sim_straddle(&s, v, seed);
    sim_add_seg(&s, -900.0, 0.0, 0.0, 0.0);
    sim_add_bend(&s, 0.0, 0.0, R, dir, deg, 900.0);
    sim_start(&s, &sd, -400.0, 0.0, 0.0);
    for (double t = 0.0; (t < 20.0) && (s.n_track_lost == 0U); t += SIM_DT_S)
    {
        double const px = s.x;
        double const py = s.y;

        sim_step_core(&s, sd_fn, &sd, &g_in, &out);
        if ((past < 0.0) && ((s.th * 180.0 / SIM_PI * dir) >= (deg - 5.0)))
        {
            past = 0.0;
        }
        if (past >= 0.0)
        {
            past += hypot(s.x - px, s.y - py);
            if (past >= 300.0)
            {
                ok = (out.state == LF_ST_ON_LINE) && (s.n_junction == 0U);
                break;
            }
        }
    }
    return ok;
}

/* Test 10: the track's 117 mm bend (ruler, 16 Nov) at the owner's 7 cm/s, on the model fitted to
 * the car's steady full lock.  With the hold at 60 deg / 150 mm every one of these 180 degree
 * bends failed (as on the car, 20 Nov); the longer hold passes them, and every gentler bend. */
static void test_fitted_u_bends(void)
{
    double const radii[3] = { 117.0, 140.0, 200.0 };
    double const degs[3] = { 90.0, 120.0, 180.0 };

    g_fitted = true;
    for (int dir = -1; dir <= 1; dir += 2)
    {
        for (int ri = 0; ri < 3; ri++)
        {
            for (int di = 0; di < 3; di++)
            {
                for (uint32_t k = 0U; k < 2U; k++)
                {
                    CHECK(sim_bend(radii[ri], dir, degs[di], 70.0, 12345U + (k * 7919U)));
                }
            }
        }
    }
    g_fitted = false;
}

/* Test 7: 90 degree bends both ways at the bring-up speeds, R 150-600 mm
 * (the track's tile bends are about R 150).  Host model, 19 Nov: the edge
 * core passed left bends only from R 200; straddle every one. */
static void test_sim_bends(void)
{
    double const radii[6] = { 150.0, 180.0, 220.0, 300.0, 400.0, 600.0 };
    double const speeds[2] = { 100.0, 150.0 };
    uint32_t pass90 = 0U;
    uint32_t pass180 = 0U;
    uint32_t n180 = 0U;

    for (int vi = 0; vi < 2; vi++)
    {
        for (int dir = -1; dir <= 1; dir += 2)
        {
            for (int ri = 0; ri < 6; ri++)
            {
                for (uint32_t k = 0U; k < 2U; k++)
                {
                    bool const ok = sim_bend(radii[ri], dir, 90.0, speeds[vi], 12345U + (k * 7919U));

                    CHECK(ok);
                    pass90 += ok ? 1U : 0U;
                    if (ri <= 3)
                    {
                        bool const u = sim_bend(radii[ri], dir, 180.0, speeds[vi], 12345U + (k * 7919U));

                        DEFER(u);       /* U-bends: reported, the tightest are marginal */
                        pass180 += u ? 1U : 0U;
                        n180++;
                    }
                }
            }
        }
    }
    printf("  straddle bends: 90 deg %u/48, 180 deg %u/%u (R 150-300)\n", pass90, pass180, n180);
}

/* Test 8: straights from offset and angled starts, a cross line, a line end. */
static void test_sim_straights(void)
{
    double const ys[3] = { -6.0, 0.0, 6.0 };
    double const ths[3] = { -10.0, 0.0, 10.0 };
    double const speeds[2] = { 100.0, 150.0 };

    for (int vi = 0; vi < 2; vi++)
    {
        uint32_t clean = 0U;
        double worst = 0.0;

        for (int a = 0; a < 3; a++)
        {
            for (int b = 0; b < 3; b++)
            {
                sim_t s;
                lf_sd_t sd;
                lf_out_t out;
                bool on = true;
                double mx = 0.0;

                sim_straddle(&s, speeds[vi], 777U);
                sim_add_seg(&s, -500.0, 0.0, 4000.0, 0.0);
                sim_start(&s, &sd, 0.0, ys[a], ths[b] * SIM_PI / 180.0);
                for (double t = 0.0; t < 2500.0 / speeds[vi]; t += SIM_DT_S)
                {
                    double px;
                    double py;

                    sim_step_core(&s, sd_fn, &sd, &g_in, &out);
                    sim_sensor_xy(&s, 0.0, &px, &py);
                    if (t >= 3.0)
                    {
                        mx = fmax(mx, fabs(py));
                        on = on && (out.state == LF_ST_ON_LINE);
                    }
                }
                clean += (on && (s.n_line_lost == 0U) && (s.n_junction == 0U) && (s.n_track_lost == 0U)) ? 1U : 0U;
                worst = fmax(worst, mx);
            }
        }
        CHECK(clean == 9U);
        /* Full steering (127) weaves up to ~19 mm either side of the midpoint on a
         * straight (96: 14 mm), still ON_LINE with no loss; it buys U-bends. */
        CHECK(worst <= 20.0);
        printf("  straddle straight starts %3.0f mm/s: %u/9 clean, max |line - midpoint| %.1f mm\n",
               speeds[vi], clean, worst);
    }

    /* A cross line at x = 1000: one JUNCTION, then straight on. */
    for (int vi = 0; vi < 2; vi++)
    {
        sim_t s;
        lf_sd_t sd;
        lf_out_t out;

        sim_straddle(&s, speeds[vi], 777U);
        sim_add_seg(&s, -500.0, 0.0, 3000.0, 0.0);
        sim_add_seg(&s, 1000.0, -300.0, 1000.0, 300.0);
        sim_start(&s, &sd, 0.0, 0.0, 0.0);
        for (double t = 0.0; t < 2200.0 / speeds[vi]; t += SIM_DT_S)
        {
            sim_step_core(&s, sd_fn, &sd, &g_in, &out);
        }
        CHECK(s.n_junction == 1U);
        CHECK(s.n_line_lost == 0U);
        CHECK(out.state == LF_ST_ON_LINE);
    }

    /* A line end at x = 1000: TRACK_LOST, the car stops; the sensors end
     * within 500 mm of the end (blind travel is bounded). */
    for (int vi = 0; vi < 2; vi++)
    {
        sim_t s;
        lf_sd_t sd;
        lf_out_t out;
        double px;
        double py;

        sim_straddle(&s, speeds[vi], 777U);
        sim_add_seg(&s, -500.0, 0.0, 1000.0, 0.0);
        sim_start(&s, &sd, 0.0, 0.0, 0.0);
        for (double t = 0.0; (t < 16.0) && (s.n_track_lost == 0U); t += SIM_DT_S)
        {
            sim_step_core(&s, sd_fn, &sd, &g_in, &out);
        }
        for (uint32_t k = 0U; k < 60U; k++)
        {
            sim_step_core(&s, sd_fn, &sd, &g_in, &out);
        }
        sim_sensor_xy(&s, 0.0, &px, &py);
        CHECK(s.n_track_lost == 1U);
        CHECK(hypot(px - 1000.0, py) <= 500.0);
        printf("  straddle line end %3.0f mm/s: stopped %.0f mm from the end\n", speeds[vi], hypot(px - 1000.0, py));
    }
}

int main(void)
{
    test_two_pose_cal();
    test_steer();
    test_both_white();
    test_deep_hold();
    test_junction();
    test_suspended_and_fault();
    test_turn_side();
    test_search_seek();
    test_floor();
    test_slow_search();
    test_both_dark();
    test_bend_memory();
    test_sim_bends();
    test_fitted_u_bends();
    test_sim_straights();
    printf("lf_straddle: %d deferred checks not yet a gate, %d of them failing (U-bends)\n",
           g_deferred, g_deferred_failing);
    return TEST_RESULT("lf_straddle");
}
