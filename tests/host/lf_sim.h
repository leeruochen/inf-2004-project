/*
 *  lf_sim.h
 *  Closed-loop simulator for the line-following core (S3-U10).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Host-only test helper (doubles and libm are fine here; never firmware).
 *
 *  What it models, and where each rule comes from:
 *    - The track: black strips 18 mm wide (straight segments and circular
 *      arcs) on white; the module's track spec (line width 18 mm).
 *    - The sensors: CENTRE and RIGHT, LF_SIM spacing apart (measured
 *      ~22 mm), F mm in front of the wheel axle; each sees a 1-D spot of
 *      radius r and reads white + (fraction of spot on black) x span, plus
 *      uniform noise.  White 140 raw and span ~1000 raw are the LF-1 desk
 *      numbers (testing/subsystem3/logs/2026-11-03_s3_lf1_run2.txt).
 *    - The Vehicle Controller (task_vc.c handle_line_msg): steering post
 *      -> NAV_LINE_TRACKING, steer_q7 x 2 -> DREP_MC_STEER at cruise;
 *      JUNCTION with a barcode command pending -> NAV_TURN_LEFT / _RIGHT /
 *      U_TURN and a bounded pivot (90 or 180 degrees) at slow speed,
 *      without one -> FWD at cruise, straight; TRACK_LOST -> STOP.
 *    - Motion (task_motion.c): acts on the newest command at its next
 *      20 ms period (motion_phase sets which sample that period follows:
 *      0 = within 5 ms after a sample whose step_n is a multiple of 4) and
 *      publishes its release time tick_ms, motion_lag_ms (1..5) after that
 *      sample's time; STEER duty_l = base + base*arg/256, duty_r = base -
 *      base*arg/256, so steer > 0 turns right; a STEER/FWD command is held
 *      MOTION_T_HOLD_MS, then the car stops; TURN_L drives the wheels
 *      -v/+v, TURN_R and UTURN +v/-v (a pivot about the axle midpoint),
 *      stopping when the mean wheel travel reaches pi x track x angle/360;
 *      it publishes the encoder counts every period.  Wheel speeds follow
 *      the command with a first-order lag.
 *    - The Line-Following task: lf_link_step() on the published counts and
 *      the VC's turn state, lf_link_post_slot() on the published tick_ms and
 *      the sample time (step_n x 5 ms), then lf_core_step().
 *    - Differential drive: omega = (v_r - v_l) / track, track 114 mm
 *      (task_motion.c MOTION_EFF_TRACK_MM); encoder 4 x 616 counts per
 *      207 mm of wheel travel (config/thresholds.h).
 *  Coordinates: x forward along the start line, y to the LEFT, heading
 *  theta counter-clockwise (so a right turn makes theta decrease).
 */

#ifndef LF_SIM_H
#define LF_SIM_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "config/thresholds.h"
#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"
#include "app_program/subsystem3/lf_link.h"

#define SIM_LINE_HALF_MM    (9.0)       /* 18 mm line */
#define SIM_TRACK_MM        (114.0)     /* MOTION_EFF_TRACK_MM */
#define SIM_WHITE_RAW       (140.0)
#define SIM_MAX_STRIPS      (8)
#define SIM_DT_S            ((double)LF_SAMPLE_PERIOD_MS / 1000.0)
#define SIM_PI              (3.14159265358979)
#define SIM_CNT_PER_MM      (((double)ENC_QUAD_MULT * (double)ENC_CPR) / (double)WHEEL_CIRC_MM)
#define SIM_MOTION_EVERY    (MOTION_PERIOD_MS / LF_SAMPLE_PERIOD_MS)   /* 4 samples */

/* Barcode command pending at the VC when the junction comes. */
typedef enum
{
    SIM_BC_NONE = 0,
    SIM_BC_LEFT,
    SIM_BC_RIGHT,
    SIM_BC_UTURN
} sim_bc_t;

/* Motion's active command. */
typedef enum
{
    SIM_MC_STOP = 0,
    SIM_MC_STEER,           /* also FWD (arg 0) */
    SIM_MC_PIVOT_L,
    SIM_MC_PIVOT_R
} sim_mc_t;

typedef enum
{
    SIM_SEG = 0,            /* straight segment (x0,y0)-(x1,y1) */
    SIM_ARC                 /* full circle, centre (x0,y0), radius r */
} sim_kind_t;

typedef struct
{
    sim_kind_t kind;
    double     x0, y0, x1, y1;
    double     r;
} sim_strip_t;

typedef struct
{
    /* Set up by the test. */
    sim_strip_t strips[SIM_MAX_STRIPS];
    int         n_strips;
    double      spacing_mm;     /* CENTRE -> RIGHT, centre to centre */
    double      fwd_mm;         /* sensors in front of the axle */
    double      spot_mm;        /* spot radius */
    double      span_raw;       /* actual black - white on the track */
    double      noise_raw;      /* uniform +- noise */
    double      speed_mmps;     /* cruise speed */
    double      lag_s;          /* wheel response time constant */
    double      turn_mmps;      /* wheel speed in a pivot (VC: slow speed) */
    sim_bc_t    bc;             /* barcode command for the next junction */
    uint32_t    motion_phase;   /* 0..3: samples from a post to Motion's period */
    uint32_t    motion_lag_ms;  /* 1..5: Motion's release after that sample's time */

    /* Car state. */
    double      x, y, th;       /* axle midpoint, heading */
    double      vl, vr;         /* actual wheel speeds */
    double      wl_mm, wr_mm;   /* wheel travel (encoder) */
    uint32_t    rng;
    uint32_t    step_n;

    /* Vehicle Controller -> Motion. */
    lf_manoeuvre_t vc_man;      /* VC in NAV_TURN_x / U_TURN */
    bool        q_new;          /* a command waits for Motion's next period */
    sim_mc_t    q_type;
    int32_t     q_arg;
    sim_mc_t    mc_type;        /* Motion's active command */
    int32_t     cmd_arg;        /* its argument (STEER: -256..+254) */
    bool        have_cmd;       /* Motion is driving (not stopped) */
    uint32_t    hold_ms;        /* Motion: time since the command */
    int32_t     pub[2];         /* published counts [0] left, [1] right */
    uint32_t    pub_tick_ms;    /* published release time (tick_ms) */
    int32_t     piv_start[2];
    double      piv_target_cnt;
    lf_link_t   link;

    /* Results. */
    uint32_t    posts;
    uint32_t    n_junction;
    uint32_t    n_track_lost;
    uint32_t    n_line_lost;    /* entries into LINE_LOST */
    uint32_t    n_turns;        /* pivots Motion finished */
    double      piv_x, piv_y;   /* axle when the last pivot ended (its centre) */
    lf_state_t  last_state;
} sim_t;

static inline double sim_overlap(double t, double r)
{
    /* Fraction of a 1-D spot [t - r, t + r] that lies on [-half, +half]. */
    double const lo = fmax(t - r, -SIM_LINE_HALF_MM);
    double const hi = fmin(t + r, SIM_LINE_HALF_MM);

    return (hi > lo) ? ((hi - lo) / (2.0 * r)) : 0.0;
}

static inline double sim_strip_dist(sim_strip_t const *p_s, double px, double py)
{
    if (p_s->kind == SIM_ARC)
    {
        return fabs(hypot(px - p_s->x0, py - p_s->y0) - p_s->r);
    }
    {
        double const dx = p_s->x1 - p_s->x0;
        double const dy = p_s->y1 - p_s->y0;
        double const len2 = (dx * dx) + (dy * dy);
        double u = (((px - p_s->x0) * dx) + ((py - p_s->y0) * dy)) / len2;

        u = fmin(1.0, fmax(0.0, u));
        return hypot(px - (p_s->x0 + (u * dx)), py - (p_s->y0 + (u * dy)));
    }
}

/* Fraction of the spot on black at (px, py): the darkest strip wins. */
static inline double sim_cover(sim_t const *p_sim, double px, double py)
{
    double c = 0.0;

    for (int i = 0; i < p_sim->n_strips; i++)
    {
        c = fmax(c, sim_overlap(sim_strip_dist(&p_sim->strips[i], px, py), p_sim->spot_mm));
    }
    return c;
}

static inline void sim_add_seg(sim_t *p_sim, double x0, double y0, double x1, double y1)
{
    sim_strip_t *p_s = &p_sim->strips[p_sim->n_strips++];

    p_s->kind = SIM_SEG;
    p_s->x0 = x0; p_s->y0 = y0; p_s->x1 = x1; p_s->y1 = y1; p_s->r = 0.0;
}

static inline void sim_add_arc(sim_t *p_sim, double cx, double cy, double r)
{
    sim_strip_t *p_s = &p_sim->strips[p_sim->n_strips++];

    p_s->kind = SIM_ARC;
    p_s->x0 = cx; p_s->y0 = cy; p_s->x1 = 0.0; p_s->y1 = 0.0; p_s->r = r;
}

/* Defaults: measured geometry, LF-1 desk numbers, bring-up cruise 200 mm/s. */
static inline void sim_init(sim_t *p_sim)
{
    p_sim->n_strips = 0;
    p_sim->spacing_mm = 22.0;
    p_sim->fwd_mm = (double)LF_D_PIVOT_MM;   /* measured F = 70 mm */
    p_sim->spot_mm = 3.0;
    p_sim->span_raw = 1000.0;
    p_sim->noise_raw = 30.0;
    p_sim->speed_mmps = 200.0;
    p_sim->lag_s = 0.04;
    p_sim->turn_mmps = (double)MOTION_SPEED_SLOW_CMPS * 10.0;
    p_sim->bc = SIM_BC_NONE;
    p_sim->motion_phase = 0U;
    p_sim->motion_lag_ms = LF_SAMPLE_PERIOD_MS;   /* the next sample's ms; Motion runs first */
    p_sim->x = 0.0; p_sim->y = 0.0; p_sim->th = 0.0;
    p_sim->vl = 0.0; p_sim->vr = 0.0;
    p_sim->wl_mm = 0.0; p_sim->wr_mm = 0.0;
    p_sim->rng = 12345U;
    p_sim->step_n = 0U;
    p_sim->vc_man = LF_MAN_NONE;
    p_sim->q_new = false; p_sim->q_type = SIM_MC_STOP; p_sim->q_arg = 0;
    p_sim->mc_type = SIM_MC_STOP; p_sim->cmd_arg = 0; p_sim->have_cmd = false;
    p_sim->hold_ms = 0U;
    p_sim->pub[0] = 0; p_sim->pub[1] = 0;
    p_sim->pub_tick_ms = 0U;
    p_sim->piv_start[0] = 0; p_sim->piv_start[1] = 0;
    p_sim->piv_target_cnt = 0.0;
    lf_link_init(&p_sim->link);
    p_sim->posts = 0U; p_sim->n_junction = 0U; p_sim->n_track_lost = 0U;
    p_sim->n_line_lost = 0U; p_sim->n_turns = 0U;
    p_sim->piv_x = 0.0; p_sim->piv_y = 0.0;
    p_sim->last_state = LF_ST_IDLE;
}

/* Sensor position: lateral offset lat (mm, + = left of the car's centreline). */
static inline void sim_sensor_xy(sim_t const *p_sim, double lat, double *p_x, double *p_y)
{
    *p_x = p_sim->x + (p_sim->fwd_mm * cos(p_sim->th)) - (lat * sin(p_sim->th));
    *p_y = p_sim->y + (p_sim->fwd_mm * sin(p_sim->th)) + (lat * cos(p_sim->th));
}

static inline double sim_noise(sim_t *p_sim)
{
    p_sim->rng = (p_sim->rng * 1103515245U) + 12345U;
    return ((((double)((p_sim->rng >> 8) & 0xFFFFU)) / 65535.0) * 2.0 - 1.0) * p_sim->noise_raw;
}

static inline uint16_t sim_raw(sim_t *p_sim, double lat)
{
    double px;
    double py;
    double v;

    sim_sensor_xy(p_sim, lat, &px, &py);
    v = SIM_WHITE_RAW + (sim_cover(p_sim, px, py) * p_sim->span_raw) + sim_noise(p_sim);
    v = fmin(4095.0, fmax(0.0, v));
    return (uint16_t)lround(v);
}

/* Place the car so CENTRE is at lateral y_c (mm, + = left of the line
 * along x) and the car's heading is th (rad), with sensors at x = 0. */
static inline void sim_place(sim_t *p_sim, double y_c, double th)
{
    p_sim->th = th;
    p_sim->x = -(p_sim->fwd_mm * cos(th));
    p_sim->y = y_c - (p_sim->fwd_mm * sin(th));
}

/* CENTRE's lateral position (y of the CENTRE sensor). */
static inline double sim_centre_y(sim_t const *p_sim)
{
    double px;
    double py;

    sim_sensor_xy(p_sim, 0.0, &px, &py);
    return py;
}

static inline void sim_centre_xy(sim_t const *p_sim, double *p_x, double *p_y)
{
    sim_sensor_xy(p_sim, 0.0, p_x, p_y);
}

/* The VC posts a command to Motion (the newest one wins). */
static inline void sim_vc_send(sim_t *p_sim, sim_mc_t type, int32_t arg)
{
    p_sim->q_new = true;
    p_sim->q_type = type;
    p_sim->q_arg = arg;
}

/* One Motion period: publish the counts, take the newest command, run the
 * hold timeout and the pivot's completion check. */
static inline void sim_motion_period(sim_t *p_sim)
{
    p_sim->pub_tick_ms = (p_sim->step_n * LF_SAMPLE_PERIOD_MS) + p_sim->motion_lag_ms;
    p_sim->pub[0] = (int32_t)lround(p_sim->wl_mm * SIM_CNT_PER_MM);
    p_sim->pub[1] = (int32_t)lround(p_sim->wr_mm * SIM_CNT_PER_MM);

    if (p_sim->q_new)
    {
        p_sim->q_new = false;
        p_sim->mc_type = p_sim->q_type;
        p_sim->cmd_arg = p_sim->q_arg;
        p_sim->have_cmd = (p_sim->q_type != SIM_MC_STOP);
        p_sim->hold_ms = 0U;
        if ((p_sim->q_type == SIM_MC_PIVOT_L) || (p_sim->q_type == SIM_MC_PIVOT_R))
        {
            double const deg = (double)p_sim->q_arg;

            p_sim->piv_start[0] = p_sim->pub[0];
            p_sim->piv_start[1] = p_sim->pub[1];
            p_sim->piv_target_cnt = ((SIM_PI * SIM_TRACK_MM * deg) / 360.0) * SIM_CNT_PER_MM;
        }
    }
    else if (p_sim->have_cmd)
    {
        if ((p_sim->mc_type == SIM_MC_PIVOT_L) || (p_sim->mc_type == SIM_MC_PIVOT_R))
        {
            double const dl = fabs((double)(p_sim->pub[0] - p_sim->piv_start[0]));
            double const dr = fabs((double)(p_sim->pub[1] - p_sim->piv_start[1]));

            if (((dl + dr) / 2.0) >= p_sim->piv_target_cnt)
            {
                p_sim->have_cmd = false;
                p_sim->mc_type = SIM_MC_STOP;
                p_sim->n_turns++;
                p_sim->piv_x = p_sim->x;    /* the axle: the pivot centre */
                p_sim->piv_y = p_sim->y;
            }
        }
        else
        {
            p_sim->hold_ms += MOTION_PERIOD_MS;
            if (p_sim->hold_ms >= MOTION_T_HOLD_MS)
            {
                p_sim->have_cmd = false;    /* hold expired -> stop */
                p_sim->mc_type = SIM_MC_STOP;
            }
        }
    }
    else
    {
        /* stopped */
    }
}

/* The VC on a JUNCTION message (task_vc.c handle_line_msg). */
static inline void sim_vc_junction(sim_t *p_sim)
{
    switch (p_sim->bc)
    {
        case SIM_BC_LEFT:
            p_sim->vc_man = LF_MAN_TURN;
            sim_vc_send(p_sim, SIM_MC_PIVOT_L, 90);
            break;
        case SIM_BC_RIGHT:
            p_sim->vc_man = LF_MAN_TURN;
            sim_vc_send(p_sim, SIM_MC_PIVOT_R, 90);
            break;
        case SIM_BC_UTURN:
            p_sim->vc_man = LF_MAN_UTURN;
            sim_vc_send(p_sim, SIM_MC_PIVOT_R, 180);
            break;
        case SIM_BC_NONE:
        default:
            sim_vc_send(p_sim, SIM_MC_STEER, 0);    /* GO_STRAIGHT: FWD */
            break;
    }
    p_sim->bc = SIM_BC_NONE;
}

/* One 5 ms sample: Motion (every 4th) -> sensors -> link -> core -> VC ->
 * car. */
static inline void sim_step(sim_t *p_sim, lf_core_t *p_lf, lf_in_t *p_in, lf_out_t *p_out)
{
    double        tl = 0.0;
    double        tr = 0.0;
    double        a;
    lf_link_out_t lk;

    lf_link_step(&p_sim->link, p_sim->vc_man, p_sim->pub, &lk);
    p_in->manoeuvre = lk.manoeuvre;
    p_in->fwd_cnt = lk.fwd_cnt;
    p_in->post_slot = lf_link_post_slot(p_sim->pub_tick_ms, p_sim->step_n * LF_SAMPLE_PERIOD_MS);
    p_in->raw_c = sim_raw(p_sim, 0.0);
    p_in->raw_r = sim_raw(p_sim, -p_sim->spacing_mm);
    lf_core_step(p_lf, p_in, p_out);

    if (p_out->event == LF_EV_JUNCTION)
    {
        p_sim->n_junction++;
        sim_vc_junction(p_sim);
    }
    if (p_out->event == LF_EV_TRACK_LOST)
    {
        p_sim->n_track_lost++;
        p_sim->vc_man = LF_MAN_NONE;
        sim_vc_send(p_sim, SIM_MC_STOP, 0);     /* VC: STOP */
    }
    if ((p_out->state == LF_ST_LINE_LOST) && (p_sim->last_state != LF_ST_LINE_LOST))
    {
        p_sim->n_line_lost++;
    }
    p_sim->last_state = p_out->state;

    if (p_out->post_steer)
    {
        p_sim->posts++;
        p_sim->vc_man = LF_MAN_NONE;            /* VC: NAV_LINE_TRACKING */
        sim_vc_send(p_sim, SIM_MC_STEER, p_out->steer_q7 * 2);  /* Q7 -> -256..+254 */
    }

    /* Motion's period comes after this sample's post when motion_phase is
     * 0 (the post waits < 5 ms), up to 3 samples later (15 ms). */
    if (((p_sim->step_n + p_sim->motion_phase) % SIM_MOTION_EVERY) == 0U)
    {
        sim_motion_period(p_sim);
    }
    p_sim->step_n++;

    if (p_sim->have_cmd)
    {
        double const base = p_sim->speed_mmps;

        switch (p_sim->mc_type)
        {
            case SIM_MC_STEER:
                tl = base + ((base * (double)p_sim->cmd_arg) / 256.0);
                tr = base - ((base * (double)p_sim->cmd_arg) / 256.0);
                break;
            case SIM_MC_PIVOT_L:
                tl = -p_sim->turn_mmps;
                tr = p_sim->turn_mmps;
                break;
            case SIM_MC_PIVOT_R:
                tl = p_sim->turn_mmps;
                tr = -p_sim->turn_mmps;
                break;
            case SIM_MC_STOP:
            default:
                break;
        }
    }
    a = SIM_DT_S / (p_sim->lag_s + SIM_DT_S);
    p_sim->vl += a * (tl - p_sim->vl);
    p_sim->vr += a * (tr - p_sim->vr);
    p_sim->wl_mm += p_sim->vl * SIM_DT_S;
    p_sim->wr_mm += p_sim->vr * SIM_DT_S;

    {
        double const v = 0.5 * (p_sim->vl + p_sim->vr);
        double const w = (p_sim->vr - p_sim->vl) / SIM_TRACK_MM;

        p_sim->x += v * cos(p_sim->th) * SIM_DT_S;
        p_sim->y += v * sin(p_sim->th) * SIM_DT_S;
        p_sim->th += w * SIM_DT_S;
    }
}

#endif /* LF_SIM_H */
