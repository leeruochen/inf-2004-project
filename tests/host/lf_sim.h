/*
 *  lf_sim.h
 *  Closed-loop simulator for the line-following core (S3-U10).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Host-only test helper (doubles and libm are fine here; never firmware).
 *
 *  What it models, and where each rule comes from:
 *    - The track: black strips 18 mm wide (straight segments and circular
 *      arcs) on white; the module's track spec (line width 18 mm).
 *    - The sensors: CENTRE and RIGHT, spacing_mm apart (ruler, 26 mm),
 *      fwd_mm in front of the wheel axle (ruler, F = 120 mm, the core's
 *      LF_D_PIVOT_MM); each sees a 1-D spot of
 *      radius r and reads white + (fraction of spot on black) x span, plus
 *      uniform noise.  White 140 raw and span ~1000 raw are the LF-1 desk
 *      numbers (testing/subsystem3/logs/2026-11-03_s3_lf1_run2.txt).  The
 *      spot is 1.2 mm from the same log's sweep: CENTRE went white to black
 *      in 2 samples of 1.4 mm (the 18 mm line took 630 ms to pass).
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
 *      it publishes the encoder counts every period.  sim_motion_control()
 *      is an integer copy of apply_control(): the feed-forward duty plus
 *      the per-wheel PID, whose target is the SAME for both wheels, so the
 *      PID pushes back against every steering command.  FWD's sync trim is
 *      not modelled: line following only sends STEER.
 *    - The motors: wheel speed follows K x duty after a dead time, with a
 *      first-order lag.  All three are fitted to the 4 Nov 2026 hardware
 *      runs (S3-U12) at the measured geometry: at steer +127 the wheels ran
 *      at 378 and 90 mm/s (sim 369 and 79); from a standing start the car
 *      had moved 35, 61, 86 and 114 mm at 220, 330, 440 and 550 ms (sim 35,
 *      60, 85, 110); and RIGHT reached the line after 330-560 ms of the
 *      oscillation (sim ~450).  The dead time stands for backlash and
 *      stiction.  Not fitted: how fast the wheels stop (see test 19).
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
#include <string.h>

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

/* task_motion.c constants, static there, copied here. */
#define SIM_PID_KP_PM       (3)
#define SIM_PID_KI_X100     (10)
#define SIM_PID_KD_PM       (1)
#define SIM_INTEGRAL_MAX    (400)
#define SIM_TURN_MIN_PM     (280)
#define SIM_DUTY_MAX_PM     ((int32_t)MOTOR_DUTY_MAX_PCT * 10)
/* Fitted to the hardware (file header). */
#define SIM_MOTOR_K         (0.62)      /* mm/s per permille of duty */
#define SIM_MOTOR_LAG_S     (0.085)
#define SIM_MOTOR_DEAD_N    (5U)        /* samples: 25 ms */
#define SIM_SPOT_MM         (1.2)
#define SIM_DEAD_MAX        (32U)       /* samples of dead-time history */

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
    SIM_ARC,                /* full circle, centre (x0,y0), radius r */
    SIM_ARC_PART            /* arc, centre (x0,y0), radius r, from angle x1 to
                             * y1 (radians, counter-clockwise, y1 > x1) */
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
    double      c_lat_mm;       /* CENTRE's lateral place, + = left (0: edge layout;
                                 * spacing / 2: straddle, CENTRE = LEFT) */
    double      fwd_mm;         /* sensors in front of the axle */
    double      spot_mm;        /* spot radius */
    double      span_raw;       /* actual black - white on the track */
    double      noise_raw;      /* uniform +- noise */
    double      speed_mmps;     /* cruise speed */
    double      motor_k;        /* mm/s per permille of duty */
    double      left_gain;      /* left motor's K relative to the right's */
    double      lag_s;          /* wheel response time constant */
    uint32_t    dead_n;         /* motor dead time, samples (0..SIM_DEAD_MAX) */
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
    int32_t     prev_cnt[2];    /* counts at the last period */
    int32_t     duty[2];        /* permille on the motors */
    int32_t     duty_hist[SIM_DEAD_MAX][2];
    int32_t     pid_int[2];     /* per-wheel PID integral */
    int32_t     pid_err[2];     /* per-wheel PID previous error */
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
    if (p_s->kind == SIM_ARC_PART)
    {
        double a = atan2(py - p_s->y0, px - p_s->x0);

        while (a < p_s->x1)
        {
            a += 2.0 * SIM_PI;
        }
        if (a <= p_s->y1)
        {
            return fabs(hypot(px - p_s->x0, py - p_s->y0) - p_s->r);
        }
        return fmin(hypot(px - (p_s->x0 + (p_s->r * cos(p_s->x1))), py - (p_s->y0 + (p_s->r * sin(p_s->x1)))),
                    hypot(px - (p_s->x0 + (p_s->r * cos(p_s->y1))), py - (p_s->y0 + (p_s->r * sin(p_s->y1)))));
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

/* A bend of angle deg from the end of a straight along +x at (x, y): dir +1
 * turns left, -1 right; the line goes on straight for exit_mm after it. */
static inline void sim_add_bend(sim_t *p_sim, double x, double y, double r, int dir, double deg, double exit_mm);

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
    p_sim->spacing_mm = 26.0;
    p_sim->c_lat_mm = 0.0;
    p_sim->fwd_mm = (double)LF_D_PIVOT_MM;   /* measured F = 120 mm */
    p_sim->spot_mm = SIM_SPOT_MM;
    p_sim->span_raw = 1000.0;
    p_sim->noise_raw = 30.0;
    p_sim->speed_mmps = 200.0;
    p_sim->motor_k = SIM_MOTOR_K;
    p_sim->left_gain = 1.0;
    p_sim->lag_s = SIM_MOTOR_LAG_S;
    p_sim->dead_n = SIM_MOTOR_DEAD_N;
    memset(p_sim->duty_hist, 0, sizeof(p_sim->duty_hist));
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
    p_sim->prev_cnt[0] = 0; p_sim->prev_cnt[1] = 0;
    p_sim->duty[0] = 0; p_sim->duty[1] = 0;
    p_sim->pid_int[0] = 0; p_sim->pid_int[1] = 0;
    p_sim->pid_err[0] = 0; p_sim->pid_err[1] = 0;
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

static inline int32_t sim_clamp(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* task_motion.c pid_step_wheel(). */
static inline int32_t sim_pid(sim_t *p_sim, int w, int32_t target, int32_t actual)
{
    int32_t const err = target - actual;
    int32_t const d_err = err - p_sim->pid_err[w];

    p_sim->pid_int[w] = sim_clamp(p_sim->pid_int[w] + err, -SIM_INTEGRAL_MAX, SIM_INTEGRAL_MAX);
    p_sim->pid_err[w] = err;
    return (SIM_PID_KP_PM * err) + ((SIM_PID_KI_X100 * p_sim->pid_int[w]) / 100)
           + (SIM_PID_KD_PM * d_err);
}

/* task_motion.c stop_internal(): PID state cleared, motors braked. */
static inline void sim_motion_stop(sim_t *p_sim)
{
    p_sim->have_cmd = false;
    p_sim->mc_type = SIM_MC_STOP;
    p_sim->duty[0] = 0; p_sim->duty[1] = 0;
    p_sim->pid_int[0] = 0; p_sim->pid_int[1] = 0;
    p_sim->pid_err[0] = 0; p_sim->pid_err[1] = 0;
}

/* task_motion.c apply_control() for STEER and the pivots, in its integers. */
static inline void sim_motion_control(sim_t *p_sim, int32_t dl, int32_t dr)
{
    bool const    pivot = (p_sim->mc_type != SIM_MC_STEER);
    int32_t const cmps = (int32_t)((pivot ? p_sim->turn_mmps : p_sim->speed_mmps) / 10.0);
    int32_t const base = (cmps * (int32_t)MOTOR_DUTY_MAX_PCT * 10) / (int32_t)MOTION_SPEED_MAX_CMPS;
    int32_t const turn = (base < SIM_TURN_MIN_PM) ? SIM_TURN_MIN_PM : base;
    int32_t const mag = (cmps * 10 * (int32_t)(ENC_QUAD_MULT * ENC_CPR) * (int32_t)MOTION_PERIOD_MS)
                        / ((int32_t)WHEEL_CIRC_MM * 1000);
    int32_t       duty_l;
    int32_t       duty_r;

    switch (p_sim->mc_type)
    {
        case SIM_MC_STEER:
            duty_l = base + ((base * p_sim->cmd_arg) / 256);
            duty_r = base - ((base * p_sim->cmd_arg) / 256);
            break;
        case SIM_MC_PIVOT_L:
            duty_l = -turn;
            duty_r = turn;
            break;
        case SIM_MC_PIVOT_R:
        default:
            duty_l = turn;
            duty_r = -turn;
            break;
    }
    {
        int32_t const tgt_l = (duty_l >= 0) ? mag : -mag;
        int32_t const tgt_r = (duty_r >= 0) ? mag : -mag;

        duty_l += sim_pid(p_sim, 0, tgt_l, dl);
        duty_r += sim_pid(p_sim, 1, tgt_r, dr);
    }
    p_sim->duty[0] = sim_clamp(duty_l, -SIM_DUTY_MAX_PM, SIM_DUTY_MAX_PM);
    p_sim->duty[1] = sim_clamp(duty_r, -SIM_DUTY_MAX_PM, SIM_DUTY_MAX_PM);
}

/* One Motion period (task_motion.c motion_step_cycle): read the counts,
 * take the newest command, run the hold timeout and the pivot's completion
 * check, set the duties, publish. */
static inline void sim_motion_period(sim_t *p_sim)
{
    int32_t dl;
    int32_t dr;

    p_sim->pub_tick_ms = (p_sim->step_n * LF_SAMPLE_PERIOD_MS) + p_sim->motion_lag_ms;
    p_sim->pub[0] = (int32_t)lround(p_sim->wl_mm * SIM_CNT_PER_MM);
    p_sim->pub[1] = (int32_t)lround(p_sim->wr_mm * SIM_CNT_PER_MM);
    dl = p_sim->pub[0] - p_sim->prev_cnt[0];
    dr = p_sim->pub[1] - p_sim->prev_cnt[1];
    p_sim->prev_cnt[0] = p_sim->pub[0];
    p_sim->prev_cnt[1] = p_sim->pub[1];

    if (p_sim->q_new)
    {
        p_sim->q_new = false;
        p_sim->mc_type = p_sim->q_type;
        p_sim->cmd_arg = p_sim->q_arg;
        p_sim->have_cmd = (p_sim->q_type != SIM_MC_STOP);
        p_sim->hold_ms = 0U;
        if (!p_sim->have_cmd)
        {
            sim_motion_stop(p_sim);
        }
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
            double const dl_p = fabs((double)(p_sim->pub[0] - p_sim->piv_start[0]));
            double const dr_p = fabs((double)(p_sim->pub[1] - p_sim->piv_start[1]));

            if (((dl_p + dr_p) / 2.0) >= p_sim->piv_target_cnt)
            {
                sim_motion_stop(p_sim);
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
                sim_motion_stop(p_sim);     /* hold expired -> stop */
            }
        }
    }
    else
    {
        /* stopped */
    }

    if (p_sim->have_cmd)
    {
        sim_motion_control(p_sim, dl, dr);
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

/* The line-following core under test: one call per 5 ms sample. */
typedef void (*sim_core_fn_t)(void *p_core, lf_in_t const *p_in, lf_out_t *p_out);

/* One 5 ms sample with any core: Motion (every 4th) -> sensors -> link ->
 * core -> VC -> car. */
static inline void sim_step_core(sim_t *p_sim, sim_core_fn_t fn, void *p_core, lf_in_t *p_in, lf_out_t *p_out)
{
    double        tl;
    double        tr;
    double        a;
    lf_link_out_t lk;

    lf_link_step(&p_sim->link, p_sim->vc_man, p_sim->pub, &lk);
    p_in->manoeuvre = lk.manoeuvre;
    p_in->fwd_cnt = lk.fwd_cnt;
    p_in->yaw_cnt = lk.yaw_cnt;
    p_in->post_slot = lf_link_post_slot(p_sim->pub_tick_ms, p_sim->step_n * LF_SAMPLE_PERIOD_MS);
    p_in->raw_c = sim_raw(p_sim, p_sim->c_lat_mm);
    p_in->raw_r = sim_raw(p_sim, p_sim->c_lat_mm - p_sim->spacing_mm);
    fn(p_core, p_in, p_out);

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

    {
        uint32_t const w = p_sim->step_n % SIM_DEAD_MAX;
        uint32_t const r = (p_sim->step_n + SIM_DEAD_MAX - p_sim->dead_n) % SIM_DEAD_MAX;

        p_sim->duty_hist[w][0] = p_sim->duty[0];
        p_sim->duty_hist[w][1] = p_sim->duty[1];
        tl = p_sim->motor_k * p_sim->left_gain * (double)p_sim->duty_hist[r][0];
        tr = p_sim->motor_k * (double)p_sim->duty_hist[r][1];
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

static inline void sim_add_bend(sim_t *p_sim, double x, double y, double r, int dir, double deg, double exit_mm)
{
    double const sweep = deg * SIM_PI / 180.0;
    double const cy = y + ((double)dir * r);
    sim_strip_t *p_s = &p_sim->strips[p_sim->n_strips++];

    p_s->kind = SIM_ARC_PART;
    p_s->x0 = x; p_s->y0 = cy; p_s->r = r;
    if (dir > 0)
    {
        p_s->x1 = -SIM_PI / 2.0; p_s->y1 = (-SIM_PI / 2.0) + sweep;
    }
    else
    {
        p_s->x1 = (SIM_PI / 2.0) - sweep; p_s->y1 = SIM_PI / 2.0;
    }
    {
        /* the end of the arc and its heading */
        double const a_end = (dir > 0) ? p_s->y1 : p_s->x1;
        double const ex = x + (r * cos(a_end));
        double const ey = cy + (r * sin(a_end));
        double const hd = (double)dir * sweep;      /* heading after the bend */

        sim_add_seg(p_sim, ex, ey, ex + (exit_mm * cos(hd)), ey + (exit_mm * sin(hd)));
    }
}

static inline void sim_lf_core_fn(void *p_core, lf_in_t const *p_in, lf_out_t *p_out)
{
    lf_core_step((lf_core_t *)p_core, p_in, p_out);
}

/* One 5 ms sample with the edge-following core (lf_core). */
static inline void sim_step(sim_t *p_sim, lf_core_t *p_lf, lf_in_t *p_in, lf_out_t *p_out)
{
    sim_step_core(p_sim, sim_lf_core_fn, p_lf, p_in, p_out);
}

#endif /* LF_SIM_H */
