/*
 *  lf_core.c
 *  Subsystem 3 -- line-following core (see lf_core.h for the method).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).
 *
 *  One call of lf_core_step() = one 5 ms sample.  Every time limit is a
 *  sample count derived from config/timing.h, so the host tests drive the
 *  core sample by sample with no clock.
 *
 *  Integer widths: raw ADC <= 4095 and darkness <= 1000, so
 *  (raw - white) * 1000 <= 4 095 000 fits uint32_t; steering products
 *  (d - LF_EDGE_TARGET_PM) * LF_KP_Q7 stay within +-21 600 in int32_t.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"
#include "app_program/subsystem3/lf_sanity.h"

/* Sample counts (5 ms each). */
#define LF_N_LOST       LF_SAMPLES(LF_T_LOST_MS)        /* 20  = 100 ms */
#define LF_N_LOST_MAX   LF_SAMPLES(LF_T_LOST_MAX_MS)    /* 80  = 400 ms */
#define LF_N_TURN       LF_SAMPLES(LF_T_TURN_MS)        /* 100 = 500 ms */
#define LF_N_UTURN      LF_SAMPLES(LF_T_UTURN_MS)       /* 200 = 1 s    */
#define LF_N_SEARCH     LF_SAMPLES(LF_T_SEARCH_MS)      /* 400 = 2 s    */
#define LF_N_SWAP       LF_SAMPLES(LF_SEARCH_SWAP_MS)   /* 140 = 700 ms */
#define LF_N_STOP       LF_SAMPLES(LF_T_STOP_MS)        /* 24  = 120 ms */
#define LF_N_JCT_MAX    LF_SAMPLES(LF_T_JCT_MAX_MS)     /* 220 = 1100 ms at 15 cm/s */

_Static_assert((uint32_t)LF_ST_COUNT == DREP_FSM_LF_STATES,
               "lf_state_t is the telemetry LF FSM code");
_Static_assert(LF_LIGHT_MAX_PM < LF_EDGE_TARGET_PM, "edge target above LIGHT");
_Static_assert(LF_EDGE_TARGET_PM < LF_PM_FULL, "the line's body steers right");
_Static_assert(LF_RECOVER_Q7 <= LF_SEARCH_Q7, "the search is never gentler than recovery");
_Static_assert(LF_SEARCH_Q7 <= LF_STEER_FULL_Q7, "the search fits full steering");
_Static_assert(LF_DARK_MIN_PM < LF_PM_FULL, "DARK reachable");
_Static_assert(LF_N_SWAP < LF_N_SEARCH, "the search swaps sides before T_search");
_Static_assert(LF_STEER_POST_PERIOD_MS == MOTION_PERIOD_MS,
               "one steering post per Motion period, in its post slot");
_Static_assert(LF_STEER_FULL_Q7 <= DREP_STEER_Q7_MAX, "full steering fits Q7");
_Static_assert((ENC_CPR > 0U) && (WHEEL_CIRC_MM > 0U), "encoder geometry known");
_Static_assert(LF_D_JCT_LEAD_MM < LF_D_PIVOT_MM, "JUNCTION posted after detection");
_Static_assert(LF_D_JCT_LOOK_MM < LF_D_JCT_POST_MM, "the look past the cross ends before the post");
_Static_assert((LF_RECOVER_Q7 + LF_RECOVER_RAMP_Q7) <= LF_STEER_FULL_Q7, "the ramp starts inside full steering");
#ifndef LF_ALLOW_ANY_SPEED
_Static_assert((MOTION_SPEED_CRUISE_CMPS >= LF_CRUISE_MIN_CMPS) && (MOTION_SPEED_CRUISE_CMPS <= LF_CRUISE_MAX_CMPS),
               "line following is validated only for LF_CRUISE_MIN/MAX_CMPS (lf_core.h): see there before changing the cruise, or pass -DLF_ALLOW_ANY_SPEED=1 for a test build");
#endif
_Static_assert(((LF_D_JCT_POST_MM * 100U) / MOTION_SPEED_CRUISE_CMPS) < LF_T_JCT_MAX_MS,
               "at cruise the travel ends before the time cap");
_Static_assert((LF_RESCUE_N >= 2U) && (LF_RESCUE_N < LF_HYST_SAMPLES + 1U),
               "C: RIGHT's rescue needs 2 samples, and no more than the zone filter it bypasses");
_Static_assert((LF_UTURN_HOLD_Q7 > 0) && (LF_UTURN_HOLD_Q7 <= LF_ACQUIRE_STEER_Q7),
               "the hold steers right, no harder than the look for the line");
_Static_assert((LF_EDGE_TARGET_PM < LF_UTURN_HOLD_FULL_PM) && (LF_UTURN_HOLD_FULL_PM <= LF_PM_FULL),
               "the line's body reads above the edge target");
_Static_assert((LF_N_LOST_MAX > LF_N_LOST) && (LF_D_LOST_CNT > 0U),
               "LINE_LOST: the time cap is longer than T_lost, the distance is set");

/* ---------------------------------------------------------------------------
 * Calibration, normalisation, zones, edge steering
 * ------------------------------------------------------------------------- */

lf_cal_status_t lf_cal_from_start_pose(lf_adc_stats_t const *p_c_on_line,
                                       lf_adc_stats_t const *p_r_on_white,
                                       lf_cal_t *p_cal)
{
    lf_cal_status_t st = LF_CAL_BAD_SENSOR;

    if ((p_c_on_line != NULL) && (p_r_on_white != NULL) && (p_cal != NULL)
        && (lf_adc_check(p_c_on_line) == LF_ADC_OK)
        && (lf_adc_check(p_r_on_white) == LF_ADC_OK))
    {
        uint16_t const black = lf_adc_stats_avg(p_c_on_line);
        uint16_t const white = lf_adc_stats_avg(p_r_on_white);

        if ((uint32_t)black >= ((uint32_t)white + LF_MIN_SPAN_RAW))
        {
            p_cal->c.white = white;
            p_cal->c.black = black;
            p_cal->r = p_cal->c;        /* matched sensors (LF-1) */
            st = LF_CAL_OK;
        }
        else
        {
            st = LF_CAL_NO_CONTRAST;
        }
    }

    return st;
}

/* One sensor from its readings over black and over white. */
static lf_sens_t cal_one(lf_adc_stats_t const *p_black, lf_adc_stats_t const *p_white, lf_cal_ch_t *p_ch)
{
    lf_sens_t v = LF_SENS_DISCONNECTED;
    lf_adc_status_t const sb = lf_adc_check(p_black);
    lf_adc_status_t const sw = lf_adc_check(p_white);

    p_ch->black = lf_adc_stats_avg(p_black);
    p_ch->white = lf_adc_stats_avg(p_white);
    if ((sb == LF_ADC_NOISY) || (sw == LF_ADC_NOISY))
    {
        v = LF_SENS_NOISY;
    }
    else if ((sb != LF_ADC_OK) || (sw != LF_ADC_OK))
    {
        v = LF_SENS_DISCONNECTED;
    }
    else if ((uint32_t)p_ch->black < ((uint32_t)p_ch->white + LF_MIN_SPAN_RAW))
    {
        v = LF_SENS_BLIND;
    }
    else if ((p_ch->black < LF_CAL_BLACK_MIN_RAW) || (p_ch->white > LF_CAL_WHITE_MAX_RAW)
             || (((uint32_t)p_ch->black - p_ch->white) < LF_CAL_SPAN_GOOD_RAW))
    {
        v = LF_SENS_WEAK;
    }
    else
    {
        v = LF_SENS_OK;
    }
    return v;
}

bool lf_cal_from_two_poses(lf_adc_stats_t const *p_c_black, lf_adc_stats_t const *p_r_white,
                           lf_adc_stats_t const *p_c_white, lf_adc_stats_t const *p_r_black,
                           lf_cal2_t *p_out)
{
    bool ok = false;

    if (p_out != NULL)
    {
        p_out->cal.c.white = 0U;
        p_out->cal.c.black = 0U;
        p_out->cal.r = p_out->cal.c;
        p_out->c = LF_SENS_DISCONNECTED;
        p_out->r = LF_SENS_DISCONNECTED;
        if ((p_c_black != NULL) && (p_r_white != NULL) && (p_c_white != NULL) && (p_r_black != NULL))
        {
            p_out->c = cal_one(p_c_black, p_c_white, &p_out->cal.c);
            p_out->r = cal_one(p_r_black, p_r_white, &p_out->cal.r);
            ok = (p_out->c == LF_SENS_OK) && (p_out->r == LF_SENS_OK);
        }
    }
    return ok;
}

uint16_t lf_norm(lf_cal_ch_t const *p_cal, uint16_t raw)
{
    uint16_t d = 0U;

    if ((p_cal != NULL) && (p_cal->black > p_cal->white) && (raw > p_cal->white))
    {
        if (raw >= p_cal->black)
        {
            d = (uint16_t)LF_PM_FULL;
        }
        else
        {
            uint32_t const num = ((uint32_t)raw - p_cal->white) * LF_PM_FULL;

            d = (uint16_t)(num / ((uint32_t)p_cal->black - p_cal->white));
        }
    }

    return d;
}

lf_zone_t lf_zone_of(uint16_t d_pm)
{
    lf_zone_t z = LF_ZONE_EDGE;

    if (d_pm < LF_LIGHT_MAX_PM)
    {
        z = LF_ZONE_LIGHT;
    }
    else if (d_pm > LF_DARK_MIN_PM)
    {
        z = LF_ZONE_DARK;
    }
    else
    {
        /* EDGE */
    }

    return z;
}

void lf_zone_reset(lf_zone_filt_t *p_f)
{
    if (p_f != NULL)
    {
        p_f->zone = LF_ZONE_LIGHT;
        p_f->cand = LF_ZONE_LIGHT;
        p_f->cand_n = 0U;
        p_f->primed = false;
    }
}

lf_zone_t lf_zone_update(lf_zone_filt_t *p_f, uint16_t d_pm)
{
    lf_zone_t const z = lf_zone_of(d_pm);
    lf_zone_t       zone = z;

    if (p_f == NULL)
    {
        /* no filter: the instant zone */
    }
    else if (!p_f->primed)
    {
        p_f->zone = z;
        p_f->cand = z;
        p_f->cand_n = 0U;
        p_f->primed = true;
    }
    else if (z == p_f->zone)
    {
        p_f->cand = z;              /* back in the confirmed zone: start over */
        p_f->cand_n = 0U;
    }
    else
    {
        if (z == p_f->cand)
        {
            p_f->cand_n++;
        }
        else
        {
            p_f->cand = z;
            p_f->cand_n = 1U;
        }
        if (p_f->cand_n >= LF_HYST_SAMPLES)
        {
            p_f->zone = z;
            p_f->cand_n = 0U;
        }
    }
    if (p_f != NULL)
    {
        zone = p_f->zone;
    }

    return zone;
}

int32_t lf_edge_steer(uint16_t d_c_pm)
{
    int32_t const d = (d_c_pm > LF_PM_FULL) ? (int32_t)LF_PM_FULL : (int32_t)d_c_pm;
    int32_t s = ((d - (int32_t)LF_EDGE_TARGET_PM) * LF_KP_Q7) / LF_KP_SPAN_PM;

    if (s > LF_STEER_FULL_Q7)
    {
        s = LF_STEER_FULL_Q7;
    }
    else if (s < -LF_STEER_FULL_Q7)
    {
        s = -LF_STEER_FULL_Q7;
    }
    else
    {
        /* within range */
    }

    return s;
}

/* ---------------------------------------------------------------------------
 * State machine helpers
 * ------------------------------------------------------------------------- */

/* Per-sample view of both sensors. */
typedef struct
{
    uint16_t  d_c;
    uint16_t  d_r;
    lf_zone_t c_now;        /* this sample's zone, no hysteresis */
    lf_zone_t r_now;
    lf_zone_t zc;           /* confirmed zones */
    lf_zone_t zr;
} lf_view_t;

static int32_t side_steer(lf_side_t side, int32_t mag)
{
    return (side == LF_SIDE_RIGHT) ? mag : -mag;
}

static lf_side_t other_side(lf_side_t side)
{
    return (side == LF_SIDE_RIGHT) ? LF_SIDE_LEFT : LF_SIDE_RIGHT;
}

static void enter(lf_core_t *p_lf, lf_state_t st)
{
    p_lf->state = st;
    p_lf->timer_n = 0U;
    p_lf->lost_n = 0U;
    p_lf->lost_cnt = 0U;
    p_lf->post_now = true;      /* post at once in the new state */
    p_lf->hold_on = false;      /* the hold after a u-turn never outlives a state */
}

/*
 * Right-sensor bend floor (S3-U17), CENTRE back on the body during bend memory.
 * The first such sample decides: a floor below LF_BEND_MIN_Q7 means a short
 * rescue (a drift), and the memory ends.
 */
static void bend_floor(lf_core_t *p_lf)
{
    if (!p_lf->bend_regain)
    {
        p_lf->bend_regain = true;
        p_lf->bend_on = (p_lf->bend_q7 >= LF_BEND_MIN_Q7);
    }
    else
    {
        /* floor already fixed */
    }
    if (p_lf->bend_on)
    {
        if (p_lf->steer_q7 < p_lf->bend_q7)
        {
            p_lf->steer_q7 = p_lf->bend_q7;
        }
        else
        {
            /* the edge steering is stronger */
        }
        if (p_lf->bend_q7 > LF_BEND_FLOOR_Q7)
        {
            p_lf->bend_q7--;
        }
        else
        {
            /* at the floor */
        }
    }
    else
    {
        /* not a bend */
    }
}

/*
 * Bend memory entry and exit (S3-U17), each sample before the state
 * handler (r_dark_n already counts this sample).
 */
static void bend_track(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    bool const light = (p_v->c_now == LF_ZONE_LIGHT);
    bool const tracking = (p_lf->state == LF_ST_ON_LINE) || (p_lf->state == LF_ST_LINE_LOST)
                          || (p_lf->state == LF_ST_REACQUIRING);

    if (!tracking || !p_in->running || (p_in->manoeuvre != LF_MAN_NONE))
    {
        p_lf->bend_on = false;
    }
    else if (light && (p_lf->r_dark_n >= LF_RESCUE_N)
             && ((p_lf->state == LF_ST_ON_LINE) || (p_lf->state == LF_ST_LINE_LOST)))
    {
        if (!p_lf->bend_on)
        {
            p_lf->bend_on = true;
            p_lf->bend_cnt0 = p_in->fwd_cnt;
            p_lf->bend_edge_n = 0U;
        }
        else
        {
            /* still rescuing */
        }
        p_lf->bend_regain = false;
        p_lf->bend_q7 = (p_lf->steer_q7 > LF_BEND_FLOOR_Q7) ? p_lf->steer_q7 : LF_BEND_FLOOR_Q7;
    }
    else if (p_lf->bend_on)
    {
        if ((p_in->fwd_cnt - p_lf->bend_cnt0) >= LF_BEND_CNT)
        {
            p_lf->bend_on = false;
        }
        else if (!light && (p_v->d_c < LF_EDGE_TARGET_PM))
        {
            p_lf->bend_edge_n++;
            p_lf->bend_on = (p_lf->bend_edge_n < LF_BEND_SETTLE_N);
        }
        else if (!light)
        {
            p_lf->bend_edge_n = 0U;
        }
        else
        {
            /* CENTRE white without RIGHT: keep the count */
        }
    }
    else
    {
        /* off */
    }
}

/*
 * ON_LINE steering, in priority order:
 *   1. CENTRE on the line (not LIGHT) and RIGHT seeing darkness right now:
 *      with the sensors 26 mm apart on an 18 mm line this never comes from a
 *      drift (CENTRE has left the line before RIGHT can reach it), so it is a
 *      cross line arriving (or an angled line) -> hold the last steering
 *      until the junction is confirmed or rejected.  CENTRE's extra darkness
 *      there is the cross line, not a drift.
 *   2. CENTRE white: off the line -> recovery steer towards the remembered
 *      side, growing the longer CENTRE stays off (LF_RECOVER_RAMP_Q7), and
 *      from the base value again if the side changes.  This includes the
 *      overshoot left across the whole line: RIGHT then sees the line, and
 *      update_side() has made the side RIGHT.
 *   3. Otherwise: proportional edge steering from CENTRE.
 */
static void online_steer(lf_core_t *p_lf, lf_view_t const *p_v)
{
    if ((p_v->c_now != LF_ZONE_LIGHT) && (p_v->r_now != LF_ZONE_LIGHT))
    {
        /* hold p_lf->steer_q7 */
    }
    else if (p_v->c_now == LF_ZONE_LIGHT)
    {
        if (p_lf->side != p_lf->recover_side)
        {
            p_lf->recover_side = p_lf->side;
            p_lf->recover_q7 = LF_RECOVER_Q7;
        }
        if (p_lf->recover_q7 < LF_STEER_FULL_Q7)
        {
            p_lf->recover_q7 += LF_RECOVER_RAMP_Q7;
        }
        p_lf->steer_q7 = side_steer(p_lf->side, p_lf->recover_q7);
    }
    else
    {
        p_lf->recover_q7 = LF_RECOVER_Q7;
        p_lf->steer_q7 = lf_edge_steer(p_v->d_c);
        if (p_lf->bend_on && (p_v->d_c >= LF_EDGE_TARGET_PM))
        {
            bend_floor(p_lf);
        }
        else
        {
            /* edge steering */
        }
    }
}

/*
 * The hold after a u-turn (ON_LINE, entered from the u-turn's look for the
 * line).  The pivot leaves the car angled across the line, and the sensors
 * sit LF_D_PIVOT_MM ahead of the axle, so at the edge steering's +4 CENTRE
 * slides on across the body and out over the LEFT edge before the heading
 * has come round.  Hold LF_UTURN_HOLD_Q7 (right) instead, until one of:
 *   - CENTRE has been on the body (>= LF_UTURN_HOLD_FULL_PM) and falls
 *     below LF_EDGE_TARGET_PM: it is at the right edge, the stable one;
 *   - CENTRE is off the line: recovery steering knows better;
 *   - LF_UTURN_HOLD_CNT of travel (fwd_cnt is free-running: the unsigned
 *     difference is right across a wrap).
 * Then online_steer() as always, from this same sample.
 */
static void hold_steer(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    uint32_t const travelled = p_in->fwd_cnt - p_lf->hold_cnt0;

    if (p_v->d_c >= LF_UTURN_HOLD_FULL_PM)
    {
        p_lf->hold_full = true;
    }
    if ((p_lf->hold_full && (p_v->d_c < LF_EDGE_TARGET_PM))
        || (p_v->c_now == LF_ZONE_LIGHT)
        || (travelled >= LF_UTURN_HOLD_CNT))
    {
        p_lf->hold_on = false;
        online_steer(p_lf, p_v);
    }
    else
    {
        p_lf->steer_q7 = LF_UTURN_HOLD_Q7;
    }
}

/* During a pivot, note what the sensors see late in it: the line the car is
 * turning onto goes by under them, and the order says where it ended up. */
static void pivot_watch(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    if (!p_lf->piv_on)
    {
        p_lf->piv_on = true;
        p_lf->piv_yaw0 = p_in->yaw_cnt;
        p_lf->piv_late = false;
        p_lf->piv_c_seen = false;
        p_lf->piv_r_seen = false;
    }
    {
        int32_t const d = (int32_t)((uint32_t)p_in->yaw_cnt - (uint32_t)p_lf->piv_yaw0);
        uint32_t const turned = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
        uint32_t const late = (p_in->manoeuvre == LF_MAN_UTURN) ? LF_PIVOT_UTURN_CNT : LF_PIVOT_HALF_CNT;

        p_lf->piv_left = (d < 0);
        if (turned >= late)
        {
            p_lf->piv_late = true;
            p_lf->piv_c_seen = p_lf->piv_c_seen || (p_v->c_now != LF_ZONE_LIGHT);
            p_lf->piv_r_seen = p_lf->piv_r_seen || (p_v->r_now != LF_ZONE_LIGHT);
        }
    }
}

/* Where the pivot left the line.  A left pivot sweeps the sensors left, so
 * the new line reaches CENTRE first, then RIGHT: once either has seen it, it
 * is to the right, between them or past RIGHT (S3-U14: CENTRE crossed it at
 * 82-87 degrees and the pivot ended at 92).  Seen by neither: still ahead, to
 * the left.  A right pivot, a u-turn included, reaches RIGHT first: seen by
 * CENTRE, it has gone past and is to the left; otherwise it is to the right. */
static lf_side_t pivot_side(lf_core_t const *p_lf)
{
    lf_side_t side;

    if (p_lf->piv_left)
    {
        side = (p_lf->piv_c_seen || p_lf->piv_r_seen) ? LF_SIDE_RIGHT : LF_SIDE_LEFT;
    }
    else
    {
        side = p_lf->piv_c_seen ? LF_SIDE_LEFT : LF_SIDE_RIGHT;
    }
    return side;
}

/* Side memory: the line is on the side of the sensor that saw it last.
 * RIGHT sits right of CENTRE, so RIGHT on the line means the line is to the
 * right; CENTRE on it with RIGHT off means that if CENTRE now slides off, it
 * leaves to the right and the line is to the left.  With both off, the side
 * is kept.  Which sensor saw the line last is what counts, not how long ago:
 * coming back from the left, RIGHT crosses the line first, then CENTRE, and
 * CENTRE can cross all 18 mm in 25 ms.  A rule based on time still said
 * "right" after that and drove the car away from the line (S3-U12, 4 Nov).
 * Between CENTRE leaving to the left and RIGHT arriving (the 26 mm spacing
 * less the line) the side is briefly wrong: RIGHT corrects it within 8 mm. */
static void update_side(lf_core_t *p_lf, lf_view_t const *p_v)
{
    if (p_v->r_now != LF_ZONE_LIGHT)
    {
        p_lf->side = LF_SIDE_RIGHT;
    }
    else if (p_v->c_now != LF_ZONE_LIGHT)
    {
        p_lf->side = LF_SIDE_LEFT;
    }
    else
    {
        /* keep */
    }
}

/* Both confirmed LIGHT for LF_T_LOST_MS and LF_D_LOST_MM of travel, or for
 * LF_T_LOST_MAX_MS whatever the travel -> true.  The travel is this streak's
 * own: each sample adds its forward counts (fwd_cnt only grows; the unsigned
 * difference is right across a wrap, and a streak is at most LF_N_LOST_MAX
 * samples of a few dozen counts, far from overflow).  C: not while RIGHT has
 * read the line for LF_RESCUE_N samples, before its zone filter confirms it. */
static bool lost_check(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    if ((p_v->zc == LF_ZONE_LIGHT) && (p_v->zr == LF_ZONE_LIGHT) && (p_lf->r_dark_n < LF_RESCUE_N))
    {
        p_lf->lost_n++;
        p_lf->lost_cnt += p_in->fwd_cnt - p_lf->prev_fwd;
    }
    else
    {
        p_lf->lost_n = 0U;
        p_lf->lost_cnt = 0U;
    }

    return (p_lf->lost_n >= LF_N_LOST)
           && ((p_lf->lost_cnt >= LF_D_LOST_CNT) || (p_lf->lost_n >= LF_N_LOST_MAX));
}

static void enter_on_line(lf_core_t *p_lf, lf_view_t const *p_v)
{
    enter(p_lf, LF_ST_ON_LINE);
    online_steer(p_lf, p_v);
}

/* capped: the search follows a loss while driving the line, so its own
 * travel is limited to LF_D_SEARCH_MAX_CNT as well as T_search. */
static void enter_search(lf_core_t *p_lf, bool capped)
{
    enter(p_lf, LF_ST_REACQUIRING);
    p_lf->timer_n = 1U;         /* this sample is the search's first */
    p_lf->steer_q7 = side_steer(p_lf->side, LF_SEARCH_Q7);
    p_lf->srch_capped = capped;
    p_lf->srch_cnt = 0U;
}

/* LINE_LOST; driving: the line was lost while the car drove it (not at the
 * end of a junction turn's look). */
static void enter_lost(lf_core_t *p_lf, bool driving)
{
    enter(p_lf, LF_ST_LINE_LOST);
    p_lf->lost_driving = driving;
}

static void enter_turn(lf_core_t *p_lf, lf_manoeuvre_t man)
{
    enter(p_lf, LF_ST_JUNCTION);
    p_lf->man_seen = man;
    p_lf->man_done = false;
    p_lf->jct_pending = false;
}

/* ---------------------------------------------------------------------------
 * States.  Each returns true if steering may be posted this sample.
 * ------------------------------------------------------------------------- */

static bool st_on_line(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    bool may_post = true;

    if (p_in->manoeuvre != LF_MAN_NONE)
    {
        enter_turn(p_lf, p_in->manoeuvre);      /* never fight a turn */
        may_post = false;
    }
    else
    {
        if (p_v->zr == LF_ZONE_LIGHT)
        {
            p_lf->junction_armed = true;
        }
        if (p_lf->junction_armed && (p_v->zc == LF_ZONE_DARK) && (p_v->zr == LF_ZONE_DARK))
        {
            p_lf->junction_armed = false;
            enter_turn(p_lf, LF_MAN_NONE);      /* straight; JUNCTION after F */
            p_lf->jct_pending = true;
            p_lf->jct_line_ahead = false;
            p_lf->jct_cnt0 = p_in->fwd_cnt;
            p_lf->steer_q7 = 0;
        }
        else if (lost_check(p_lf, p_in, p_v))
        {
            enter_lost(p_lf, true);
            may_post = false;
        }
        else if (p_lf->hold_on)
        {
            hold_steer(p_lf, p_in, p_v);
        }
        else
        {
            online_steer(p_lf, p_v);
        }
    }

    return may_post;
}

/* What RIGHT saw was the cross line, not the line we follow: forget it,
 * or a slide off to the right just after the junction would search the
 * wrong way. */
static void forget_cross_line(lf_core_t *p_lf)
{
    p_lf->side = LF_SIDE_LEFT;
}

/*
 * Junction detected, JUNCTION not posted yet: keep driving (steering is
 * posted, so Motion's hold never stops the car) until the car has
 * travelled LF_D_JCT_POST_CNT counts, or LF_N_JCT_MAX samples if the
 * odometer is stuck.  On the cross line: straight (steer 0).  Once the
 * line is seen going on past it (CENTRE on it, RIGHT white), track it as
 * in ON_LINE, so the car is aligned with it at the pivot point, and a
 * drift off it is steered back (the cross line is first wiped from the
 * side memory).  Both sensors white before the line was seen going on:
 * look left for LF_D_JCT_LOOK_CNT from detection.  No line ahead (a T
 * junction): straight.  The line ending is not LINE_LOST here.
 * Returns true if steering may be posted this sample: not in the sample
 * of the post, so the Vehicle Controller acts on JUNCTION before any
 * steering (it runs at a higher priority, so the next sample already sees
 * its turn state).
 */
static bool jct_travel(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v,
                       lf_out_t *p_out)
{
    bool may_post = true;

    if (!p_lf->jct_line_ahead && (p_v->c_now != LF_ZONE_LIGHT)
        && (p_v->r_now == LF_ZONE_LIGHT))
    {
        p_lf->jct_line_ahead = true;
        forget_cross_line(p_lf);
    }
    if (p_lf->jct_line_ahead)
    {
        online_steer(p_lf, p_v);
    }
    else if ((p_v->c_now == LF_ZONE_LIGHT) && (p_v->r_now == LF_ZONE_LIGHT)
             && ((p_in->fwd_cnt - p_lf->jct_cnt0) < LF_D_JCT_LOOK_CNT))
    {
        p_lf->steer_q7 = -LF_RECOVER_Q7;    /* look left for the line */
    }
    else
    {
        p_lf->steer_q7 = 0;
    }
    p_lf->timer_n++;
    if (((p_in->fwd_cnt - p_lf->jct_cnt0) >= LF_D_JCT_POST_CNT)
        || (p_lf->timer_n >= LF_N_JCT_MAX))
    {
        p_out->event = LF_EV_JUNCTION;
        p_lf->jct_pending = false;
        forget_cross_line(p_lf);
        may_post = false;
    }

    return may_post;
}

/*
 * JUNCTION_DETECTED.  man_seen NONE: straight on (steer 0); first the
 * travel to the JUNCTION post (jct_travel), then, if the Vehicle Controller
 * did not turn, back to ON_LINE once RIGHT is white and CENTRE is on the
 * line.  Otherwise the Vehicle Controller is turning: post nothing while it
 * turns, then wait T_turn (T_uturn) for CENTRE to see solid black.  After a
 * pivot, steer gently towards the side pivot_side() found while it looks.
 * A u-turn's solid black starts the hold (hold_steer).
 */
static bool st_junction(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v,
                        lf_out_t *p_out)
{
    bool may_post = false;

    if (p_in->manoeuvre != LF_MAN_NONE)
    {
        p_lf->man_seen = p_in->manoeuvre;
        p_lf->man_done = false;
        p_lf->jct_pending = false;          /* the VC turned anyway: no post */
        p_lf->timer_n = 0U;
    }
    else if (p_lf->man_seen == LF_MAN_NONE)
    {
        p_lf->steer_q7 = 0;
        may_post = true;
        if (p_lf->jct_pending)
        {
            may_post = jct_travel(p_lf, p_in, p_v, p_out);
        }
        else if ((p_v->zr == LF_ZONE_LIGHT) && (p_v->zc != LF_ZONE_LIGHT))
        {
            p_lf->junction_armed = true;
            enter_on_line(p_lf, p_v);
        }
        else if (lost_check(p_lf, p_in, p_v))
        {
            enter_lost(p_lf, true);
            may_post = false;
        }
        else
        {
            /* still on the cross line */
        }
    }
    else
    {
        bool const uturn = (p_lf->man_seen == LF_MAN_UTURN);
        uint32_t const limit = uturn ? LF_N_UTURN : LF_N_TURN;

        if (!p_lf->man_done)
        {
            p_lf->man_done = true;
            p_lf->post_now = true;          /* post at once after the turn */
            if (p_lf->piv_late)
            {
                p_lf->side = pivot_side(p_lf);
            }
        }
        p_lf->timer_n++;
        if (p_lf->piv_late)
        {
            p_lf->steer_q7 = side_steer(p_lf->side, LF_ACQUIRE_STEER_Q7);
        }
        else
        {
            /* A stalled pivot: nothing known.  After a u-turn the line is
             * most likely to the right of CENTRE. */
            p_lf->steer_q7 = uturn ? LF_ACQUIRE_STEER_Q7 : 0;
        }
        may_post = true;
        if (p_v->zc == LF_ZONE_DARK)
        {
            enter_on_line(p_lf, p_v);
            if (uturn && p_lf->piv_late)
            {
                /* Only after a pivot that got past 135 degrees: the car is
                 * then known to stand angled across the line it came along. */
                p_lf->hold_on = true;
                p_lf->hold_full = false;
                p_lf->hold_cnt0 = p_in->fwd_cnt;
                p_lf->steer_q7 = LF_UTURN_HOLD_Q7;
            }
        }
        else if (p_lf->timer_n >= limit)
        {
            if (uturn && !p_lf->piv_late)
            {
                p_lf->side = LF_SIDE_RIGHT;
            }
            enter_lost(p_lf, false);    /* the turn's look ran out: T_search alone */
            may_post = false;
        }
        else
        {
            /* still looking */
        }
    }

    return may_post;
}

/* LINE_LOST: post nothing so Motion's hold timeout stops the car, then
 * search.  The line coming back meanwhile ends it early: under CENTRE, or (C)
 * under RIGHT for LF_RESCUE_N samples, which returns to the recovery towards
 * the right (update_side has made the side RIGHT). */
static bool st_line_lost(lf_core_t *p_lf, lf_view_t const *p_v)
{
    bool may_post = false;

    p_lf->timer_n++;
    if (p_v->zc == LF_ZONE_DARK)
    {
        enter_on_line(p_lf, p_v);
        may_post = true;
    }
    else if (p_lf->r_dark_n >= LF_RESCUE_N)
    {
        enter_on_line(p_lf, p_v);
        may_post = true;
    }
    else if (p_lf->timer_n >= LF_N_STOP)
    {
        enter_search(p_lf, p_lf->lost_driving);
        may_post = true;
    }
    else
    {
        /* stopping */
    }

    return may_post;
}

/* REACQUIRING_TRACK: search steer towards the remembered side, then the other
 * side, until CENTRE sees solid black or T_search runs out, or (a search
 * after a loss while driving) its own travel reaches LF_D_SEARCH_MAX_CNT. */
static bool st_search(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v, lf_out_t *p_out)
{
    bool may_post = true;

    p_lf->timer_n++;
    if (p_lf->srch_capped)
    {
        p_lf->srch_cnt += p_in->fwd_cnt - p_lf->prev_fwd;  /* at most 400 samples of a few dozen counts */
    }
    if (p_v->zc == LF_ZONE_DARK)
    {
        enter_on_line(p_lf, p_v);
    }
    else if ((p_lf->timer_n >= LF_N_SEARCH)
             || (p_lf->srch_capped && (p_lf->srch_cnt >= LF_D_SEARCH_MAX_CNT)))
    {
        p_out->event = LF_EV_TRACK_LOST;        /* once: then IDLE until a new run */
        enter(p_lf, LF_ST_IDLE);
        p_lf->steer_q7 = 0;
        may_post = false;
    }
    else
    {
        lf_side_t const s = (p_lf->timer_n > LF_N_SWAP) ? other_side(p_lf->side) : p_lf->side;

        p_lf->steer_q7 = side_steer(s, LF_SEARCH_Q7);
    }

    return may_post;
}

/* SUSPENDED_BY_AVOIDANCE: sample only.  Leave on LINE_REACQUIRED (with the
 * avoidance flag clear): track if CENTRE is on black, else search. */
static bool st_suspended(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    bool may_post = false;

    if (p_in->reacquired && !p_in->avoid_active)
    {
        if (p_v->zc == LF_ZONE_DARK)
        {
            enter_on_line(p_lf, p_v);
        }
        else
        {
            enter_search(p_lf, false);  /* hand-back without the line: T_search alone */
        }
        may_post = true;
    }

    return may_post;
}

/* ---------------------------------------------------------------------------
 * Public
 * ------------------------------------------------------------------------- */

static void lf_core_init_body(lf_core_t *p_lf, lf_cal_t const *p_cal)
{
    p_lf->cal.c.white = 0U;
    p_lf->cal.c.black = 0U;
    p_lf->cal.r = p_lf->cal.c;
    p_lf->cal_ok = false;
    if (p_cal != NULL)
    {
        p_lf->cal = *p_cal;
        p_lf->cal_ok = ((uint32_t)p_cal->c.black >= ((uint32_t)p_cal->c.white + LF_MIN_SPAN_RAW))
                       && ((uint32_t)p_cal->r.black >= ((uint32_t)p_cal->r.white + LF_MIN_SPAN_RAW));
    }
    lf_zone_reset(&p_lf->zc);
    lf_zone_reset(&p_lf->zr);
    p_lf->prev_running = false;
    p_lf->junction_armed = true;
    p_lf->side = LF_SIDE_LEFT;
    p_lf->steer_q7 = 0;
    p_lf->recover_q7 = LF_RECOVER_Q7;
    p_lf->recover_side = LF_SIDE_LEFT;
    p_lf->piv_on = false;
    p_lf->piv_yaw0 = 0;
    p_lf->piv_late = false;
    p_lf->piv_left = false;
    p_lf->piv_c_seen = false;
    p_lf->piv_r_seen = false;
    p_lf->hold_on = false;
    p_lf->hold_full = false;
    p_lf->hold_cnt0 = 0U;
    p_lf->r_dark_n = 0U;
    p_lf->bend_on = false;
    p_lf->bend_regain = false;
    p_lf->bend_q7 = LF_BEND_FLOOR_Q7;
    p_lf->bend_cnt0 = 0U;
    p_lf->bend_edge_n = 0U;
    p_lf->man_seen = LF_MAN_NONE;
    p_lf->man_done = false;
    p_lf->jct_pending = false;
    p_lf->jct_line_ahead = false;
    p_lf->jct_cnt0 = 0U;
    p_lf->prev_fwd = 0U;
    p_lf->lost_driving = false;
    p_lf->srch_capped = false;
    p_lf->srch_cnt = 0U;
    enter(p_lf, LF_ST_IDLE);
}

void lf_core_init(lf_core_t *p_lf, lf_cal_t const *p_cal)
{
    if (p_lf != NULL)
    {
        lf_core_init_body(p_lf, p_cal);
    }
}

static void lf_core_step_body(lf_core_t *p_lf, lf_in_t const *p_in, lf_out_t *p_out)
{
    lf_view_t v;
    bool      may_post = false;
    bool      rising;

    /* Sensors: darkness, instant zones, confirmed zones, the side. */
    v.d_c = p_lf->cal_ok ? lf_norm(&p_lf->cal.c, p_in->raw_c) : 0U;
    v.d_r = p_lf->cal_ok ? lf_norm(&p_lf->cal.r, p_in->raw_r) : 0U;
    v.c_now = lf_zone_of(v.d_c);
    v.r_now = lf_zone_of(v.d_r);
    v.zc = lf_zone_update(&p_lf->zc, v.d_c);
    v.zr = lf_zone_update(&p_lf->zr, v.d_r);
    update_side(p_lf, &v);
    if (v.r_now == LF_ZONE_LIGHT)
    {
        p_lf->r_dark_n = 0U;
    }
    else if (p_lf->r_dark_n < LF_RESCUE_N)
    {
        p_lf->r_dark_n++;
    }
    else
    {
        /* saturated: RIGHT has the line */
    }
    bend_track(p_lf, p_in, &v);
    if ((p_in->manoeuvre == LF_MAN_TURN) || (p_in->manoeuvre == LF_MAN_UTURN))
    {
        pivot_watch(p_lf, p_in, &v);
    }
    else
    {
        p_lf->piv_on = false;
    }

    rising = p_in->running && !p_lf->prev_running;
    p_lf->prev_running = p_in->running;
    p_out->event = LF_EV_NONE;

    if (!p_in->running)
    {
        if (p_lf->state != LF_ST_IDLE)
        {
            enter(p_lf, LF_ST_IDLE);
        }
        p_lf->steer_q7 = 0;
    }
    else if (p_lf->state == LF_ST_IDLE)
    {
        if (rising)
        {
            if (!p_lf->cal_ok)
            {
                p_out->event = LF_EV_TRACK_LOST;    /* refuse: the VC stops the car */
            }
            else if (v.zc == LF_ZONE_DARK)
            {
                p_lf->junction_armed = true;
                enter_on_line(p_lf, &v);
                may_post = true;
            }
            else
            {
                p_lf->side = LF_SIDE_LEFT;
                enter_search(p_lf, false);  /* run start off the line: T_search alone */
                may_post = true;
            }
        }
    }
    else if (p_in->avoid_active && (p_lf->state != LF_ST_SUSPENDED))
    {
        enter(p_lf, LF_ST_SUSPENDED);
    }
    else
    {
        switch (p_lf->state)
        {
            case LF_ST_ON_LINE:
                may_post = st_on_line(p_lf, p_in, &v);
                break;
            case LF_ST_JUNCTION:
                may_post = st_junction(p_lf, p_in, &v, p_out);
                break;
            case LF_ST_LINE_LOST:
                if (p_in->manoeuvre != LF_MAN_NONE)
                {
                    enter_turn(p_lf, p_in->manoeuvre);
                }
                else
                {
                    may_post = st_line_lost(p_lf, &v);
                }
                break;
            case LF_ST_REACQUIRING:
                if (p_in->manoeuvre != LF_MAN_NONE)
                {
                    enter_turn(p_lf, p_in->manoeuvre);
                }
                else
                {
                    may_post = st_search(p_lf, p_in, &v, p_out);
                }
                break;
            case LF_ST_SUSPENDED:
                may_post = st_suspended(p_lf, p_in, &v);
                break;
            case LF_ST_IDLE:
            case LF_ST_COUNT:
            default:
                enter(p_lf, LF_ST_IDLE);
                break;
        }
    }

    /* Steering post timing (LF-3.1b): Motion acts on the newest command
     * once per 20 ms, when it reads its queue.  Post in the post slot, the
     * last sample before that read, so the steering Motion acts on is at
     * most one sample old whatever the two tasks' phase.  Also post at
     * once on entering a state: if the state changes again before the slot,
     * Motion still gets this state's command; otherwise the slot's post
     * replaces it before Motion reads. */
    p_out->post_steer = false;
    if (may_post && (p_lf->post_now || p_in->post_slot))
    {
        p_out->post_steer = true;
        p_lf->post_now = false;
    }

    p_out->state = p_lf->state;
    p_out->steer_q7 = p_lf->steer_q7;
    p_out->line_found = (p_lf->state != LF_ST_IDLE)
                        && ((p_lf->state == LF_ST_ON_LINE) || (v.zc == LF_ZONE_DARK));
    p_out->d_c_pm = v.d_c;
    p_out->d_r_pm = v.d_r;
    /* Every sample, every state: the next sample's travel is its own. */
    p_lf->prev_fwd = p_in->fwd_cnt;
}

void lf_core_step(lf_core_t *p_lf, lf_in_t const *p_in, lf_out_t *p_out)
{
    if ((p_lf != NULL) && (p_in != NULL) && (p_out != NULL))
    {
        lf_core_step_body(p_lf, p_in, p_out);
    }
}
