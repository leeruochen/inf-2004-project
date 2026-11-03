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
 *    - The Vehicle Controller: steer_q7 x 2 -> DREP_MC_STEER at cruise
 *      (task_vc.c handle_line_msg).
 *    - Motion: duty_l = base + base*arg/256, duty_r = base - base*arg/256
 *      (task_motion.c, DREP_MC_STEER), so steer > 0 turns right; the last
 *      command is held MOTION_T_HOLD_MS, then the car stops; wheel speeds
 *      follow the command with a first-order lag.
 *    - Differential drive: omega = (v_r - v_l) / track, track 114 mm
 *      (task_motion.c MOTION_EFF_TRACK_MM).
 *  Coordinates: x forward along the start line, y to the LEFT, heading
 *  theta counter-clockwise (so a right turn makes theta decrease).
 */

#ifndef LF_SIM_H
#define LF_SIM_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"

#define SIM_LINE_HALF_MM    (9.0)       /* 18 mm line */
#define SIM_TRACK_MM        (114.0)     /* MOTION_EFF_TRACK_MM */
#define SIM_WHITE_RAW       (140.0)
#define SIM_MAX_STRIPS      (8)
#define SIM_DT_S            ((double)LF_SAMPLE_PERIOD_MS / 1000.0)
#define SIM_PI              (3.14159265358979)

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

    /* Car state. */
    double      x, y, th;       /* axle midpoint, heading */
    double      vl, vr;         /* actual wheel speeds */
    int32_t     cmd_arg;        /* last DREP_MC_STEER argument */
    double      hold_s;         /* time since the last command */
    bool        have_cmd;
    uint32_t    rng;

    /* Results. */
    uint32_t    posts;
    uint32_t    n_junction;
    uint32_t    n_track_lost;
    uint32_t    n_line_lost;    /* entries into LINE_LOST */
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
    p_sim->fwd_mm = 60.0;
    p_sim->spot_mm = 3.0;
    p_sim->span_raw = 1000.0;
    p_sim->noise_raw = 30.0;
    p_sim->speed_mmps = 200.0;
    p_sim->lag_s = 0.04;
    p_sim->x = 0.0; p_sim->y = 0.0; p_sim->th = 0.0;
    p_sim->vl = 0.0; p_sim->vr = 0.0;
    p_sim->cmd_arg = 0; p_sim->hold_s = 0.0; p_sim->have_cmd = false;
    p_sim->rng = 12345U;
    p_sim->posts = 0U; p_sim->n_junction = 0U; p_sim->n_track_lost = 0U;
    p_sim->n_line_lost = 0U;
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

/* One 5 ms sample: sensors -> core -> VC -> Motion -> car. */
static inline void sim_step(sim_t *p_sim, lf_core_t *p_lf, lf_in_t *p_in, lf_out_t *p_out)
{
    double tl = 0.0;
    double tr = 0.0;
    double a;

    p_in->raw_c = sim_raw(p_sim, 0.0);
    p_in->raw_r = sim_raw(p_sim, -p_sim->spacing_mm);
    lf_core_step(p_lf, p_in, p_out);

    if (p_out->event == LF_EV_JUNCTION)
    {
        p_sim->n_junction++;
    }
    if (p_out->event == LF_EV_TRACK_LOST)
    {
        p_sim->n_track_lost++;
        p_sim->have_cmd = false;            /* VC: STOP */
    }
    if ((p_out->state == LF_ST_LINE_LOST) && (p_sim->last_state != LF_ST_LINE_LOST))
    {
        p_sim->n_line_lost++;
    }
    p_sim->last_state = p_out->state;

    if (p_out->post_steer)
    {
        p_sim->posts++;
        p_sim->cmd_arg = p_out->steer_q7 * 2;  /* task_vc.c: Q7 -> -256..+254 */
        p_sim->hold_s = 0.0;
        p_sim->have_cmd = true;
    }
    else
    {
        p_sim->hold_s += SIM_DT_S;
        if (p_sim->hold_s >= ((double)MOTION_T_HOLD_MS / 1000.0))
        {
            p_sim->have_cmd = false;        /* Motion: hold expired -> stop */
        }
    }

    if (p_sim->have_cmd)
    {
        double const base = p_sim->speed_mmps;

        tl = base + ((base * (double)p_sim->cmd_arg) / 256.0);
        tr = base - ((base * (double)p_sim->cmd_arg) / 256.0);
    }
    a = SIM_DT_S / (p_sim->lag_s + SIM_DT_S);
    p_sim->vl += a * (tl - p_sim->vl);
    p_sim->vr += a * (tr - p_sim->vr);

    {
        double const v = 0.5 * (p_sim->vl + p_sim->vr);
        double const w = (p_sim->vr - p_sim->vl) / SIM_TRACK_MM;

        p_sim->x += v * cos(p_sim->th) * SIM_DT_S;
        p_sim->y += v * sin(p_sim->th) * SIM_DT_S;
        p_sim->th += w * SIM_DT_S;
    }
}

#endif /* LF_SIM_H */
