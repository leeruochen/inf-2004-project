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
 *  (d - 500) * LF_KP_Q7 stay within +-64 000 in int32_t.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"
#include "app_program/subsystem3/lf_sanity.h"

/* Sample counts (5 ms each). */
#define LF_N_LOST       LF_SAMPLES(LF_T_LOST_MS)        /* 20  = 100 ms */
#define LF_N_TURN       LF_SAMPLES(LF_T_TURN_MS)        /* 100 = 500 ms */
#define LF_N_UTURN      LF_SAMPLES(LF_T_UTURN_MS)       /* 200 = 1 s    */
#define LF_N_SEARCH     LF_SAMPLES(LF_T_SEARCH_MS)      /* 400 = 2 s    */
#define LF_N_SWAP       LF_SAMPLES(LF_SEARCH_SWAP_MS)   /* 140 = 700 ms */
#define LF_N_STOP       LF_SAMPLES(LF_T_STOP_MS)        /* 24  = 120 ms */
#define LF_N_SIDE_MEM   LF_SAMPLES(LF_SIDE_MEMORY_MS)   /* 20  = 100 ms */
#define LF_N_POST       LF_STEER_POST_EVERY_N_SAMPLES   /* 4   = 20 ms  */

/* r_quiet_n saturates here (anything above the side memory is "long ago"). */
#define LF_QUIET_MAX    (LF_N_SIDE_MEM + 1U)

_Static_assert((uint32_t)LF_ST_COUNT == DREP_FSM_LF_STATES,
               "lf_state_t is the telemetry LF FSM code");
_Static_assert(LF_LIGHT_MAX_PM < LF_EDGE_TARGET_PM, "edge target above LIGHT");
_Static_assert(LF_EDGE_TARGET_PM < LF_DARK_MIN_PM, "edge target below DARK");
_Static_assert(LF_DARK_MIN_PM < LF_PM_FULL, "DARK reachable");
_Static_assert(LF_N_SWAP < LF_N_SEARCH, "the search swaps sides before T_search");
_Static_assert(LF_N_POST > 0U, "steering post period >= sample period");
_Static_assert(LF_STEER_FULL_Q7 <= DREP_STEER_Q7_MAX, "full steering fits Q7");

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
    int32_t s = ((d - (int32_t)LF_EDGE_TARGET_PM) * LF_KP_Q7) / (int32_t)LF_EDGE_TARGET_PM;

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
    p_lf->post_n = 0U;          /* post at once in the new state */
}

/*
 * ON_LINE steering, in priority order:
 *   1. CENTRE on the line (not LIGHT) and RIGHT seeing darkness right now:
 *      with the sensors 22 mm apart on an 18 mm line this never comes from a
 *      drift (CENTRE has left the line before RIGHT can reach it), so it is a
 *      cross line arriving (or an angled line) -> hold the last steering
 *      until the junction is confirmed or rejected.  CENTRE's extra darkness
 *      there is the cross line, not a drift.
 *   2. CENTRE white: off the line -> full steer towards the remembered side.
 *      This includes the overshoot left across the whole line: RIGHT then
 *      sees the line, and update_side() has made the side RIGHT.
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
        p_lf->steer_q7 = side_steer(p_lf->side, LF_STEER_FULL_Q7);
    }
    else
    {
        p_lf->steer_q7 = lf_edge_steer(p_v->d_c);
    }
}

/* Side memory.  While CENTRE is on the line, the side is RIGHT if RIGHT saw
 * darkness within LF_SIDE_MEMORY_MS, else LEFT (CENTRE slid off its edge).
 * Once CENTRE is off the line the side is kept; RIGHT seeing the line then
 * still makes it RIGHT. */
static void update_side(lf_core_t *p_lf, lf_view_t const *p_v)
{
    if (p_v->c_now != LF_ZONE_LIGHT)
    {
        p_lf->side = (p_lf->r_quiet_n <= LF_N_SIDE_MEM) ? LF_SIDE_RIGHT : LF_SIDE_LEFT;
    }
    else if (p_v->r_now != LF_ZONE_LIGHT)
    {
        p_lf->side = LF_SIDE_RIGHT;
    }
    else
    {
        /* keep */
    }
}

/* Both confirmed LIGHT for LF_T_LOST_MS -> true. */
static bool lost_check(lf_core_t *p_lf, lf_view_t const *p_v)
{
    if ((p_v->zc == LF_ZONE_LIGHT) && (p_v->zr == LF_ZONE_LIGHT))
    {
        p_lf->lost_n++;
    }
    else
    {
        p_lf->lost_n = 0U;
    }

    return (p_lf->lost_n >= LF_N_LOST);
}

static void enter_on_line(lf_core_t *p_lf, lf_view_t const *p_v)
{
    enter(p_lf, LF_ST_ON_LINE);
    online_steer(p_lf, p_v);
}

static void enter_search(lf_core_t *p_lf)
{
    enter(p_lf, LF_ST_REACQUIRING);
    p_lf->timer_n = 1U;         /* this sample is the search's first */
    p_lf->steer_q7 = side_steer(p_lf->side, LF_STEER_FULL_Q7);
}

static void enter_turn(lf_core_t *p_lf, lf_manoeuvre_t man)
{
    enter(p_lf, LF_ST_JUNCTION);
    p_lf->man_seen = man;
    p_lf->man_done = false;
}

/* ---------------------------------------------------------------------------
 * States.  Each returns true if steering may be posted this sample.
 * ------------------------------------------------------------------------- */

static bool st_on_line(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v,
                       lf_out_t *p_out)
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
            p_out->event = LF_EV_JUNCTION;
            p_lf->junction_armed = false;
            enter_turn(p_lf, LF_MAN_NONE);      /* straight unless the VC turns */
            p_lf->steer_q7 = 0;
        }
        else if (lost_check(p_lf, p_v))
        {
            enter(p_lf, LF_ST_LINE_LOST);
            may_post = false;
        }
        else
        {
            online_steer(p_lf, p_v);
        }
    }

    return may_post;
}

/*
 * JUNCTION_DETECTED.  man_seen NONE: straight across (steer 0) until RIGHT
 * is off the cross line.  Otherwise the Vehicle Controller is turning: post
 * nothing while it turns, then wait T_turn (T_uturn) for CENTRE to see solid
 * black.  After a u-turn the line is to the right of CENTRE: steer gently
 * right meanwhile.
 */
static bool st_junction(lf_core_t *p_lf, lf_in_t const *p_in, lf_view_t const *p_v)
{
    bool may_post = false;

    if (p_in->manoeuvre != LF_MAN_NONE)
    {
        p_lf->man_seen = p_in->manoeuvre;
        p_lf->man_done = false;
        p_lf->timer_n = 0U;
    }
    else if (p_lf->man_seen == LF_MAN_NONE)
    {
        p_lf->steer_q7 = 0;
        may_post = true;
        if (p_v->zr == LF_ZONE_LIGHT)
        {
            /* What RIGHT saw was the cross line, not the line we follow:
             * forget it, or a slide off to the right just after the
             * junction would search the wrong way. */
            p_lf->r_quiet_n = LF_QUIET_MAX;
            p_lf->side = LF_SIDE_LEFT;
            p_lf->junction_armed = true;
            enter_on_line(p_lf, p_v);
        }
        else if (lost_check(p_lf, p_v))
        {
            enter(p_lf, LF_ST_LINE_LOST);
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
            p_lf->post_n = 0U;              /* post at once after the turn */
        }
        p_lf->timer_n++;
        p_lf->steer_q7 = uturn ? LF_ACQUIRE_STEER_Q7 : 0;
        may_post = true;
        if (p_v->zc == LF_ZONE_DARK)
        {
            enter_on_line(p_lf, p_v);
        }
        else if (p_lf->timer_n >= limit)
        {
            if (uturn)
            {
                p_lf->side = LF_SIDE_RIGHT;
            }
            enter(p_lf, LF_ST_LINE_LOST);
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
 * search.  The line coming back meanwhile ends it early. */
static bool st_line_lost(lf_core_t *p_lf, lf_view_t const *p_v)
{
    bool may_post = false;

    p_lf->timer_n++;
    if (p_v->zc == LF_ZONE_DARK)
    {
        enter_on_line(p_lf, p_v);
        may_post = true;
    }
    else if (p_lf->timer_n >= LF_N_STOP)
    {
        enter_search(p_lf);
        may_post = true;
    }
    else
    {
        /* stopping */
    }

    return may_post;
}

/* REACQUIRING_TRACK: full steer towards the remembered side, then the other
 * side, until CENTRE sees solid black or T_search runs out. */
static bool st_search(lf_core_t *p_lf, lf_view_t const *p_v, lf_out_t *p_out)
{
    bool may_post = true;

    p_lf->timer_n++;
    if (p_v->zc == LF_ZONE_DARK)
    {
        enter_on_line(p_lf, p_v);
    }
    else if (p_lf->timer_n >= LF_N_SEARCH)
    {
        p_out->event = LF_EV_TRACK_LOST;        /* once: then IDLE until a new run */
        enter(p_lf, LF_ST_IDLE);
        p_lf->steer_q7 = 0;
        may_post = false;
    }
    else
    {
        lf_side_t const s = (p_lf->timer_n > LF_N_SWAP) ? other_side(p_lf->side) : p_lf->side;

        p_lf->steer_q7 = side_steer(s, LF_STEER_FULL_Q7);
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
            enter_search(p_lf);
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
    p_lf->r_quiet_n = LF_QUIET_MAX;
    p_lf->steer_q7 = 0;
    p_lf->man_seen = LF_MAN_NONE;
    p_lf->man_done = false;
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

    /* Sensors: darkness, instant zones, confirmed zones, RIGHT's age. */
    v.d_c = p_lf->cal_ok ? lf_norm(&p_lf->cal.c, p_in->raw_c) : 0U;
    v.d_r = p_lf->cal_ok ? lf_norm(&p_lf->cal.r, p_in->raw_r) : 0U;
    v.c_now = lf_zone_of(v.d_c);
    v.r_now = lf_zone_of(v.d_r);
    v.zc = lf_zone_update(&p_lf->zc, v.d_c);
    v.zr = lf_zone_update(&p_lf->zr, v.d_r);
    if (v.r_now != LF_ZONE_LIGHT)
    {
        p_lf->r_quiet_n = 0U;
    }
    else if (p_lf->r_quiet_n < LF_QUIET_MAX)
    {
        p_lf->r_quiet_n++;
    }
    else
    {
        /* saturated */
    }
    update_side(p_lf, &v);

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
                enter_search(p_lf);
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
                may_post = st_on_line(p_lf, p_in, &v, p_out);
                break;
            case LF_ST_JUNCTION:
                may_post = st_junction(p_lf, p_in, &v);
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
                    may_post = st_search(p_lf, &v, p_out);
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

    /* Steering post cadence: every LF_N_POST samples, at once on entry. */
    p_out->post_steer = false;
    if (may_post)
    {
        p_out->post_steer = (p_lf->post_n == 0U);
        p_lf->post_n = (p_lf->post_n + 1U) % LF_N_POST;
    }

    p_out->state = p_lf->state;
    p_out->steer_q7 = p_lf->steer_q7;
    p_out->line_found = (p_lf->state != LF_ST_IDLE)
                        && ((p_lf->state == LF_ST_ON_LINE) || (v.zc == LF_ZONE_DARK));
    p_out->d_c_pm = v.d_c;
    p_out->d_r_pm = v.d_r;
}

void lf_core_step(lf_core_t *p_lf, lf_in_t const *p_in, lf_out_t *p_out)
{
    if ((p_lf != NULL) && (p_in != NULL) && (p_out != NULL))
    {
        lf_core_step_body(p_lf, p_in, p_out);
    }
}
