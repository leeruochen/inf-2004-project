/*
 *  lf_straddle.c
 *  Subsystem 3 -- EXPERIMENT: straddle line following (see lf_straddle.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).
 *
 *  One call of lf_sd_step() = one 5 ms sample.  Times are sample counts,
 *  distances are encoder counts from lf_link (fwd_cnt only grows; every
 *  difference below is unsigned, so a wrap is harmless).
 *
 *  Integer widths: darkness <= 1000, so LF_SD_KP_Q7 x (dr - dl) stays within
 *  +-127 000 in int32_t.  The travel counters add one sample's counts (a few
 *  dozen at most) and are cleared well before 2^32.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/thresholds.h"
#include "config/timing.h"
#include "app_program/subsystem3/lf_straddle.h"

/* Sample counts (5 ms each), from the same times as lf_core. */
#define LF_SD_N_STOP     LF_SAMPLES(LF_T_STOP_MS)        /* 24  = 120 ms */
#define LF_SD_N_SEARCH   LF_SAMPLES(LF_T_SEARCH_MS)      /* 400 = 2 s    */
#define LF_SD_N_SWAP     LF_SAMPLES(LF_SEARCH_SWAP_MS)   /* 140 = 700 ms */
#define LF_SD_N_JCT_MAX  LF_SAMPLES(LF_T_JCT_MAX_MS)

_Static_assert((LF_SD_KP_Q7 > 0) && (LF_SD_KP_Q7 <= LF_STEER_FULL_Q7), "Kp fits full steering");
_Static_assert((LF_SD_HOLD_Q7 > 0) && (LF_SD_HOLD_Q7 <= LF_STEER_FULL_Q7), "the hold fits full steering");
_Static_assert((LF_SD_PROBE_Q7 > 0) && (LF_SD_PROBE_Q7 < LF_SD_KP_Q7), "the probe is gentle");
_Static_assert(LF_SD_JCT_N >= 1U, "a junction needs at least one confirmed sample");
_Static_assert((LF_SD_CONF_CNT > 0U) && (LF_SD_PROBE_CNT > 0U) && (LF_SD_HOLD_CNT > 0U),
               "the both-white limits are set");

/* Per-sample view of both sensors. */
typedef struct
{
    uint16_t  dl;           /* LEFT darkness */
    uint16_t  dr;           /* RIGHT darkness */
    bool      contact;      /* either at or above LF_LIGHT_MAX_PM this sample */
    lf_zone_t zl;           /* confirmed zones */
    lf_zone_t zr;
} sd_view_t;

static int32_t sd_clamp(int32_t v)
{
    int32_t s = v;

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

static int32_t sd_side_steer(lf_side_t side, int32_t mag)
{
    return (side == LF_SIDE_RIGHT) ? mag : -mag;
}

static lf_side_t sd_other_side(lf_side_t side)
{
    return (side == LF_SIDE_RIGHT) ? LF_SIDE_LEFT : LF_SIDE_RIGHT;
}

static void sd_enter(lf_sd_t *p_sd, lf_state_t st)
{
    p_sd->state = st;
    p_sd->timer_n = 0U;
    p_sd->post_now = true;
}

/* ON_LINE, any way in.  probe: both white at entry is checked at once
 * (after a junction turn, the line should be between the sensors). */
static void sd_enter_on_line(lf_sd_t *p_sd, bool probe)
{
    sd_enter(p_sd, LF_ST_ON_LINE);
    p_sd->deep = false;
    p_sd->deep_cnt = 0U;
    p_sd->bw = probe ? LF_SD_BW_PROBE : LF_SD_BW_CENTRED;
    p_sd->bw_cnt = probe ? LF_SD_CONF_CNT : 0U;
    /* The entry post goes out at once: never with an old value.  A contact
     * this sample overrides it (sd_contact_steer). */
    p_sd->steer_q7 = probe ? sd_side_steer(p_sd->side, LF_SD_PROBE_Q7) : 0;
}

/* LINE_LOST; driving: lost while driving the line, so the search is capped. */
static void sd_enter_lost(lf_sd_t *p_sd, bool driving)
{
    sd_enter(p_sd, LF_ST_LINE_LOST);
    p_sd->lost_driving = driving;
}

static void sd_enter_search(lf_sd_t *p_sd, bool capped)
{
    sd_enter(p_sd, LF_ST_REACQUIRING);
    p_sd->timer_n = 1U;         /* this sample is the search's first */
    p_sd->steer_q7 = sd_side_steer(p_sd->side, LF_SEARCH_Q7);
    p_sd->srch_capped = capped;
    p_sd->srch_cnt = 0U;
}

/* Proportional steering while a sensor touches the line, and the side and
 * depth of this contact. */
static void sd_contact_steer(lf_sd_t *p_sd, sd_view_t const *p_v, uint32_t dcnt)
{
    uint16_t const near_pm = (p_v->dl >= p_v->dr) ? p_v->dl : p_v->dr;
    uint16_t const far_pm = (p_v->dl >= p_v->dr) ? p_v->dr : p_v->dl;

    if (!p_sd->in_contact)
    {
        p_sd->deep = false;     /* a new contact */
        p_sd->deep_cnt = 0U;
    }
    p_sd->side = (p_v->dl >= p_v->dr) ? LF_SIDE_LEFT : LF_SIDE_RIGHT;
    if ((near_pm >= LF_SD_DEEP_PM) && (far_pm < LF_LIGHT_MAX_PM))
    {
        p_sd->deep_cnt += dcnt;             /* the line's body under one sensor only */
        if (p_sd->deep_cnt >= LF_SD_DEEP_CNT)
        {
            p_sd->deep = true;
        }
    }
    else
    {
        p_sd->deep_cnt = 0U;
    }
    p_sd->bw = LF_SD_BW_NONE;
    p_sd->bw_cnt = 0U;
    p_sd->steer_q7 = sd_clamp((LF_SD_KP_Q7 * ((int32_t)p_v->dr - (int32_t)p_v->dl)) / (int32_t)LF_PM_FULL);
}

/*
 * Both white (lf_straddle.h): straight while believed centred, then the
 * probe, or the hold after a deep contact.  Returns true when the line is
 * to be declared lost.
 */
static bool sd_both_white(lf_sd_t *p_sd, lf_in_t const *p_in, uint32_t dcnt)
{
    bool lost = false;

    if (p_sd->bw == LF_SD_BW_NONE)
    {
        p_sd->bw = p_sd->deep ? LF_SD_BW_HOLD : LF_SD_BW_CENTRED;
        p_sd->bw_cnt = 0U;
        p_sd->hold_yaw0 = p_in->yaw_cnt;
    }
    p_sd->bw_cnt += dcnt;
    if (p_sd->bw == LF_SD_BW_HOLD)
    {
        /* yaw_cnt wraps: the unsigned difference, read as signed, is right */
        int32_t const d = (int32_t)((uint32_t)p_in->yaw_cnt - (uint32_t)p_sd->hold_yaw0);
        uint32_t const turned = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;

        p_sd->steer_q7 = sd_side_steer(p_sd->side, LF_SD_HOLD_Q7);
        lost = (p_sd->bw_cnt >= LF_SD_HOLD_CNT) || (turned >= LF_SD_HOLD_YAW_CNT);
    }
    else if (p_sd->bw_cnt < LF_SD_CONF_CNT)
    {
        p_sd->bw = LF_SD_BW_CENTRED;
        p_sd->steer_q7 = 0;
    }
    else if (p_sd->bw_cnt < (LF_SD_CONF_CNT + LF_SD_PROBE_CNT))
    {
        p_sd->bw = LF_SD_BW_PROBE;
        p_sd->steer_q7 = sd_side_steer(p_sd->side, LF_SD_PROBE_Q7);
    }
    else
    {
        lost = true;
    }
    return lost;
}

/* Both confirmed DARK for LF_SD_JCT_N samples, once per cross line. */
static bool sd_junction_seen(lf_sd_t *p_sd, sd_view_t const *p_v)
{
    bool seen = false;
    bool const both_dark = (p_v->zl == LF_ZONE_DARK) && (p_v->zr == LF_ZONE_DARK);

    if (!both_dark)
    {
        p_sd->jct_n = 0U;
        p_sd->jct_armed = true;
    }
    else if (p_sd->jct_armed)
    {
        p_sd->jct_n++;
        if (p_sd->jct_n >= LF_SD_JCT_N)
        {
            p_sd->jct_armed = false;
            seen = true;
        }
    }
    else
    {
        /* the same cross line */
    }
    return seen;
}

/* ---------------------------------------------------------------------------
 * States.  Each returns true if steering may be posted this sample.
 * ------------------------------------------------------------------------- */

static bool sd_st_on_line(lf_sd_t *p_sd, lf_in_t const *p_in, sd_view_t const *p_v, uint32_t dcnt)
{
    bool may_post = true;

    if (p_in->manoeuvre != LF_MAN_NONE)
    {
        sd_enter(p_sd, LF_ST_JUNCTION);         /* never fight a turn */
        p_sd->man_seen = p_in->manoeuvre;
        p_sd->jct_pending = false;
        may_post = false;
    }
    else if (sd_junction_seen(p_sd, p_v))
    {
        sd_enter(p_sd, LF_ST_JUNCTION);
        p_sd->man_seen = LF_MAN_NONE;
        p_sd->jct_pending = true;
        p_sd->jct_cnt0 = p_in->fwd_cnt;
        p_sd->steer_q7 = 0;
    }
    else if (p_v->contact)
    {
        sd_contact_steer(p_sd, p_v, dcnt);
    }
    else if (sd_both_white(p_sd, p_in, dcnt))
    {
        sd_enter_lost(p_sd, true);
        may_post = false;
    }
    else
    {
        /* both white, still believed on the line */
    }
    p_sd->in_contact = p_v->contact;

    return may_post;
}

/*
 * JUNCTION_DETECTED.  Not posted yet: follow the line on (proportional
 * steering on contact, straight when both are white or both dark) until
 * LF_D_JCT_POST_CNT of travel, or LF_SD_N_JCT_MAX samples if the odometer
 * is stuck; then JUNCTION, and no steering in that sample, so the Vehicle
 * Controller (higher priority) acts on it before any steering.  Posted: the
 * VC's turn shows in the next sample; if none, back to ON_LINE.  During a
 * turn nothing is posted; the side the line was last seen on is kept, and
 * after it ON_LINE checks at once that the line is between the sensors.
 */
static bool sd_st_junction(lf_sd_t *p_sd, lf_in_t const *p_in, sd_view_t const *p_v, lf_out_t *p_out)
{
    bool may_post = false;

    p_sd->timer_n++;
    if (p_in->manoeuvre != LF_MAN_NONE)
    {
        p_sd->man_seen = p_in->manoeuvre;
        p_sd->jct_pending = false;          /* the VC turned: no post */
        if (p_v->contact)
        {
            p_sd->side = (p_v->dl >= p_v->dr) ? LF_SIDE_LEFT : LF_SIDE_RIGHT;
        }
    }
    else if (p_sd->man_seen != LF_MAN_NONE)
    {
        sd_enter_on_line(p_sd, true);       /* the turn ended */
        may_post = true;
    }
    else if (p_sd->jct_pending)
    {
        if (p_v->contact && !((p_v->zl == LF_ZONE_DARK) && (p_v->zr == LF_ZONE_DARK)))
        {
            sd_contact_steer(p_sd, p_v, 0U);
        }
        else
        {
            p_sd->steer_q7 = 0;
        }
        may_post = true;
        if (((p_in->fwd_cnt - p_sd->jct_cnt0) >= LF_D_JCT_POST_CNT) || (p_sd->timer_n >= LF_SD_N_JCT_MAX))
        {
            p_out->event = LF_EV_JUNCTION;
            p_sd->jct_pending = false;
            p_sd->timer_n = 0U;
            may_post = false;
        }
    }
    else if (p_sd->timer_n >= 2U)
    {
        sd_enter_on_line(p_sd, false);      /* the VC went straight on */
        may_post = true;
    }
    else
    {
        /* the sample after the post: wait for the VC's state */
    }
    p_sd->in_contact = p_v->contact;

    return may_post;
}

/* LINE_LOST: post nothing so Motion's hold timeout stops the car, then
 * search.  A sensor touching the line meanwhile ends it. */
static bool sd_st_line_lost(lf_sd_t *p_sd, sd_view_t const *p_v)
{
    bool may_post = false;

    p_sd->timer_n++;
    if ((p_v->zl != LF_ZONE_LIGHT) || (p_v->zr != LF_ZONE_LIGHT))
    {
        sd_enter_on_line(p_sd, false);
        sd_contact_steer(p_sd, p_v, 0U);
        may_post = true;
    }
    else if (p_sd->timer_n >= LF_SD_N_STOP)
    {
        sd_enter_search(p_sd, p_sd->lost_driving);
        may_post = true;
    }
    else
    {
        /* stopping */
    }
    return may_post;
}

/* REACQUIRING_TRACK: arc towards the side last touched, then the other side,
 * until a sensor touches the line, or T_search (or, after a loss while
 * driving, LF_D_SEARCH_MAX_CNT of its own travel) runs out. */
static bool sd_st_search(lf_sd_t *p_sd, sd_view_t const *p_v, uint32_t dcnt, lf_out_t *p_out)
{
    bool may_post = true;

    p_sd->timer_n++;
    if (p_sd->srch_capped)
    {
        p_sd->srch_cnt += dcnt;
    }
    if ((p_v->zl != LF_ZONE_LIGHT) || (p_v->zr != LF_ZONE_LIGHT))
    {
        sd_enter_on_line(p_sd, false);
        sd_contact_steer(p_sd, p_v, dcnt);
    }
    else if ((p_sd->timer_n >= LF_SD_N_SEARCH)
             || (p_sd->srch_capped && (p_sd->srch_cnt >= LF_D_SEARCH_MAX_CNT)))
    {
        p_out->event = LF_EV_TRACK_LOST;        /* once: then IDLE until a new run */
        sd_enter(p_sd, LF_ST_IDLE);
        p_sd->steer_q7 = 0;
        may_post = false;
    }
    else
    {
        lf_side_t const s = (p_sd->timer_n > LF_SD_N_SWAP) ? sd_other_side(p_sd->side) : p_sd->side;

        p_sd->steer_q7 = sd_side_steer(s, LF_SEARCH_Q7);
    }
    return may_post;
}

/* SUSPENDED_BY_AVOIDANCE: sample only.  Leave on LINE_REACQUIRED with the
 * avoidance flag clear: track if a sensor touches the line, else search. */
static bool sd_st_suspended(lf_sd_t *p_sd, lf_in_t const *p_in, sd_view_t const *p_v)
{
    bool may_post = false;

    if (p_in->reacquired && !p_in->avoid_active)
    {
        if ((p_v->zl != LF_ZONE_LIGHT) || (p_v->zr != LF_ZONE_LIGHT))
        {
            sd_enter_on_line(p_sd, false);
            sd_contact_steer(p_sd, p_v, 0U);
        }
        else
        {
            sd_enter_search(p_sd, false);       /* hand-back without the line: T_search alone */
        }
        may_post = true;
    }
    return may_post;
}

/* A sensor at an ADC rail: disconnected, shorted or saturated. */
static bool sd_rail(uint16_t raw)
{
    return (raw <= ADC_STUCK_LOW_RAW) || (raw >= ADC_STUCK_HIGH_RAW);
}

/* ---------------------------------------------------------------------------
 * Public
 * ------------------------------------------------------------------------- */

void lf_sd_init(lf_sd_t *p_sd, lf_cal_t const *p_cal)
{
    if (p_sd != NULL)
    {
        p_sd->cal.c.white = 0U;
        p_sd->cal.c.black = 0U;
        p_sd->cal.r = p_sd->cal.c;
        p_sd->cal_ok = false;
        if (p_cal != NULL)
        {
            p_sd->cal = *p_cal;
            p_sd->cal_ok = ((uint32_t)p_cal->c.black >= ((uint32_t)p_cal->c.white + LF_MIN_SPAN_RAW))
                           && ((uint32_t)p_cal->r.black >= ((uint32_t)p_cal->r.white + LF_MIN_SPAN_RAW));
        }
        lf_zone_reset(&p_sd->zl);
        lf_zone_reset(&p_sd->zr);
        p_sd->prev_running = false;
        p_sd->prev_fwd = 0U;
        p_sd->steer_q7 = 0;
        p_sd->side = LF_SIDE_LEFT;
        p_sd->in_contact = false;
        p_sd->deep = false;
        p_sd->deep_cnt = 0U;
        p_sd->bw = LF_SD_BW_CENTRED;
        p_sd->bw_cnt = 0U;
        p_sd->hold_yaw0 = 0;
        p_sd->jct_armed = true;
        p_sd->jct_n = 0U;
        p_sd->jct_pending = false;
        p_sd->jct_cnt0 = 0U;
        p_sd->man_seen = LF_MAN_NONE;
        p_sd->lost_driving = false;
        p_sd->srch_capped = false;
        p_sd->srch_cnt = 0U;
        p_sd->fault_n = 0U;
        sd_enter(p_sd, LF_ST_IDLE);
    }
}

static void lf_sd_step_body(lf_sd_t *p_sd, lf_in_t const *p_in, lf_out_t *p_out)
{
    sd_view_t      v;
    bool           may_post = false;
    bool           rising;
    uint32_t const dcnt = p_in->fwd_cnt - p_sd->prev_fwd;

    v.dl = p_sd->cal_ok ? lf_norm(&p_sd->cal.c, p_in->raw_c) : 0U;
    v.dr = p_sd->cal_ok ? lf_norm(&p_sd->cal.r, p_in->raw_r) : 0U;
    v.contact = (v.dl >= LF_LIGHT_MAX_PM) || (v.dr >= LF_LIGHT_MAX_PM);
    v.zl = lf_zone_update(&p_sd->zl, v.dl);
    v.zr = lf_zone_update(&p_sd->zr, v.dr);

    rising = p_in->running && !p_sd->prev_running;
    p_sd->prev_running = p_in->running;
    p_out->event = LF_EV_NONE;

    if (p_in->running && (p_sd->state != LF_ST_IDLE) && (sd_rail(p_in->raw_c) || sd_rail(p_in->raw_r)))
    {
        p_sd->fault_n++;
    }
    else
    {
        p_sd->fault_n = 0U;
    }

    if (!p_in->running)
    {
        if (p_sd->state != LF_ST_IDLE)
        {
            sd_enter(p_sd, LF_ST_IDLE);
        }
        p_sd->steer_q7 = 0;
    }
    else if (p_sd->state == LF_ST_IDLE)
    {
        if (rising)
        {
            if (!p_sd->cal_ok)
            {
                p_out->event = LF_EV_TRACK_LOST;    /* refuse: the VC stops the car */
            }
            else
            {
                p_sd->jct_armed = true;
                sd_enter_on_line(p_sd, false);      /* start pose: the line between the sensors */
                if (v.contact)
                {
                    sd_contact_steer(p_sd, &v, 0U);
                }
                else
                {
                    p_sd->steer_q7 = 0;
                }
                p_sd->in_contact = v.contact;
                may_post = true;
            }
        }
    }
    else if (p_sd->fault_n >= LF_SD_FAULT_N)
    {
        p_out->event = LF_EV_TRACK_LOST;            /* a sensor fault: the VC stops the car */
        sd_enter(p_sd, LF_ST_IDLE);
        p_sd->steer_q7 = 0;
    }
    else if (p_in->avoid_active && (p_sd->state != LF_ST_SUSPENDED))
    {
        sd_enter(p_sd, LF_ST_SUSPENDED);
    }
    else
    {
        switch (p_sd->state)
        {
            case LF_ST_ON_LINE:
                may_post = sd_st_on_line(p_sd, p_in, &v, dcnt);
                break;
            case LF_ST_JUNCTION:
                may_post = sd_st_junction(p_sd, p_in, &v, p_out);
                break;
            case LF_ST_LINE_LOST:
                if (p_in->manoeuvre != LF_MAN_NONE)
                {
                    sd_enter(p_sd, LF_ST_JUNCTION);
                    p_sd->man_seen = p_in->manoeuvre;
                    p_sd->jct_pending = false;
                }
                else
                {
                    may_post = sd_st_line_lost(p_sd, &v);
                }
                break;
            case LF_ST_REACQUIRING:
                if (p_in->manoeuvre != LF_MAN_NONE)
                {
                    sd_enter(p_sd, LF_ST_JUNCTION);
                    p_sd->man_seen = p_in->manoeuvre;
                    p_sd->jct_pending = false;
                }
                else
                {
                    may_post = sd_st_search(p_sd, &v, dcnt, p_out);
                }
                break;
            case LF_ST_SUSPENDED:
                may_post = sd_st_suspended(p_sd, p_in, &v);
                break;
            case LF_ST_IDLE:
            case LF_ST_COUNT:
            default:
                sd_enter(p_sd, LF_ST_IDLE);
                break;
        }
    }

    /* Steering post timing as in lf_core: in the post slot, or at once on
     * entering a state. */
    p_out->post_steer = false;
    if (may_post && (p_sd->post_now || p_in->post_slot))
    {
        p_out->post_steer = true;
        p_sd->post_now = false;
    }

    p_out->state = p_sd->state;
    p_out->steer_q7 = p_sd->steer_q7;
    p_out->line_found = (p_sd->state != LF_ST_IDLE)
                        && ((p_sd->state == LF_ST_ON_LINE) || (v.zl != LF_ZONE_LIGHT) || (v.zr != LF_ZONE_LIGHT));
    p_out->d_c_pm = v.dl;
    p_out->d_r_pm = v.dr;
    p_sd->prev_fwd = p_in->fwd_cnt;
}

void lf_sd_step(lf_sd_t *p_sd, lf_in_t const *p_in, lf_out_t *p_out)
{
    if ((p_sd != NULL) && (p_in != NULL) && (p_out != NULL))
    {
        lf_sd_step_body(p_sd, p_in, p_out);
    }
}
