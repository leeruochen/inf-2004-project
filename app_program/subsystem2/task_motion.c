/*
 *  task_motion.c
 *  Subsystem 2 -- Motion Control task (priority 3, 20 ms periodic PID loop).
 *  Kernel side: kernel and project headers plus kernel-free drv_*.h only,
 *  never SDK headers (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Responsibilities (design.md 3.1/3.2/3.3, specs.md, ARCHITECTURE.md, TEAM.md):
 *    - Drift-free 20 ms periodic execution via app_period_wait()
 *      (MOTION_PERIOD_MS); logs any missed release through dbg_.
 *    - Polls MBF_MOTION_CMD (IF-02) with TMO_POL; validates each command with
 *      drep_mc_valid(), counting and logging rejects.
 *    - Holds the last valid setpoint for up to MOTION_T_HOLD_MS (100 ms) when
 *      no fresh command arrives, then stops (DREP_MC_STOP).
 *    - Enforces the NFR2 hard speed cap (MOTION_SPEED_MAX_CMPS = 50 cm/s).
 *    - Checks EVF_SAFETY (IF-06) with tk_ref_flg(); immediately brakes if
 *      EVF_SAFETY_FAULT_LATCHED is set.
 *    - Reads both wheel encoders via enc_get_count() (IF-10), computes
 *      wrap-safe two's-complement deltas, runs per-wheel PID + cross-wheel
 *      straight-line correction (FR5), and drives PWM via drv_motor.
 *    - Publishes motion_state_t (speed, cumulative distance, encoder counts,
 *      tick_ms) under MTX_MOTION (IF-08, NFR6).
 */

#include "task_motion.h"

#include "../subsystem4/subsystem4.h"   /* subsystem4_get_pitch_half_deg */
#include "config/pins.h"
#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "src/debug/dbg.h"
#include "task_vc.h"                    /* vc_get_system_state */

/* ---------------------------------------------------------------------------
 * Geometry compile-time guards (ARCHITECTURE.md D7, config/thresholds.h):
 *   - Setting only one of ENC_CPR or WHEEL_CIRC_MM is always a build error.
 *   - Defining MOTION_REQUIRE_GEOMETRY=1 while either is 0 is a build error.
 *   - When both are > 0, closed-loop count<->distance/angle/speed conversion
 *     is compiled in; while both are 0 (pre-measurement bring-up), feed-forward
 *     duty plus raw-count straight-line correction (delta_l - delta_r) runs
 *     without dividing by 0.
 * ------------------------------------------------------------------------- */

#if ((ENC_CPR == 0U) && (WHEEL_CIRC_MM != 0U)) || ((ENC_CPR != 0U) && (WHEEL_CIRC_MM == 0U))
#error "task_motion.c: both ENC_CPR and WHEEL_CIRC_MM must be measured (> 0) in config/thresholds.h"
#endif

#if defined(MOTION_REQUIRE_GEOMETRY) && (MOTION_REQUIRE_GEOMETRY != 0)
#if (ENC_CPR == 0U) || (WHEEL_CIRC_MM == 0U)
#error "task_motion.c: ENC_CPR and WHEEL_CIRC_MM are 0 (unmeasured); measure on the car before enabling MOTION_REQUIRE_GEOMETRY"
#endif
#endif

#if (ENC_CPR != 0U) && (WHEEL_CIRC_MM != 0U)
#define MOTION_EFF_ENC_CPR      ((uint32_t)ENC_CPR)
#define MOTION_EFF_WHEEL_CIRC   ((uint32_t)WHEEL_CIRC_MM)
#else
/* Calibrated on target in Phase 1/2 (S2-U10/S2-U11/S2-U18):
 * 66 mm wheel diameter -> 207 mm circumference; Run 2 gave 11454 counts for
 * 950 mm -> 2464 quadrature counts/rev (616 CPR). */
#define MOTION_EFF_ENC_CPR      (616U)
#define MOTION_EFF_WHEEL_CIRC   (207U)
#endif
#define MOTION_HAS_GEOMETRY     (1)
#define MOTION_COUNTS_PER_REV   ((int32_t)(ENC_QUAD_MULT * MOTION_EFF_ENC_CPR))

/** Effective wheel track width (mm) for in-place turns (calibrated on floor:
 *  138 mm -> ~104 deg, 120 mm -> ~95 deg, 114 mm -> 90.0 deg). */
#define MOTION_EFF_TRACK_MM     (114U)
/** Minimum feed-forward duty (permille) to overcome tire stiction in turns. */
#define MOTION_TURN_MIN_BASE_PM (280)

/** Maximum duration (ms) for a bounded-distance or bounded-angle manoeuvre. */
#define MOTION_MANOEUVRE_MAX_MS (15000U)

/* ---------------------------------------------------------------------------
 * Tuned PID and straight-line correction gains (fixed-point x100, TUNE).
 * Calibrated for 2464 quad counts/rev (~48 counts per 20 ms at 20 cm/s).
 * ------------------------------------------------------------------------- */

/** Velocity error proportional gain (permille duty per count/period error). TUNE */
#define MOTION_PID_KP_PM        (3)
/** Velocity error integral gain x100 (permille duty per accumulated count). TUNE */
#define MOTION_PID_KI_X100      (10)
/** Velocity error derivative gain (permille duty per count/period change). TUNE */
#define MOTION_PID_KD_PM        (1)
/** Cumulative heading error gain (permille duty per count of total_dl - total_dr). TUNE */
#define MOTION_SYNC_KP_PM       (2)
/** Instantaneous wheel delta damping gain (permille duty per dl - dr). TUNE */
#define MOTION_SYNC_KD_PM       (2)
/** Maximum differential steering trim (permille) for straight-line tracking. */
#define MOTION_SYNC_MAX_PM      (120)
/** Anti-windup cap on the integral term (in count units). */
#ifndef MOTION_INTEGRAL_MAX
#define MOTION_INTEGRAL_MAX     (400)
#endif

/** Consecutive 20 ms periods at >= 50% duty with zero counts -> stall/encoder fault. */
#define MOTION_STALL_PERIODS    (25U)   /* 25 x 20 ms = 500 ms */
/** Minimum duty magnitude (permille) before stall detection arms. */
#define MOTION_STALL_DUTY_PM    (500)

/* Differential per-wheel speed targets during DREP_MC_STEER (ARCHITECTURE.md D14).
 * When enabled (-DMOTION_STEER_TARGETS=1), left and right wheels track distinct
 * closed-loop target counts scaled by MOTION_STEER_TGT_GAIN_PCT so PID assists
 * turns rather than winding up against steering. */
#ifndef MOTION_STEER_TARGETS
#define MOTION_STEER_TARGETS        (0)
#endif
#ifndef MOTION_STEER_TGT_GAIN_PCT
#define MOTION_STEER_TGT_GAIN_PCT   (100)
#endif

/* ---------------------------------------------------------------------------
 * Gradual IMU-triggered hump climb speed & torque assist (TUNE).
 * When the IMU detects pitch >= 3.0 deg (6 half-deg) while driving straight
 * or bogged down on a hump, target speed and climb torque ramp up gradually
 * every 20 ms cycle without ever clamping curve steering.
 * ------------------------------------------------------------------------- */
#define MOTION_HUMP_PITCH_ON_HD     (6U)    /* 3.0 deg entry/active threshold */
#define MOTION_HUMP_PITCH_STEEP_HD  (8U)    /* 4.0 deg steeper ramp threshold */
#define MOTION_HUMP_RAMP_UP_MMPS    (3U)    /* +3 mm/s per 20 ms (+1.5 cm/s per 100 ms) */
#define MOTION_HUMP_RAMP_BOG_MMPS   (5U)    /* +5 mm/s per 20 ms when pitch >= 4 deg & bogged */
#define MOTION_HUMP_RAMP_DOWN_MMPS  (6U)    /* -6 mm/s per 20 ms fast decay on level/curve */
#define MOTION_HUMP_BOOST_MAX_MMPS  (45U)   /* max +4.5 cm/s boost (5.0 -> 9.5 cm/s) */
#define MOTION_HUMP_FAST_CAP_MMPS   (70)    /* stop growing boost once actual >= 7.0 cm/s */
#define MOTION_HUMP_BOG_SPEED_MMPS  (35)    /* speed below which curve-turn gate allows boost */
#define MOTION_HUMP_HOLD_TICKS      (6U)    /* 120 ms hold bridging front-to-rear wheels */
#define MOTION_HUMP_CLIMB_MAX_PM    (180)   /* max +18% load-sensitive climb duty assist */

/* Shared motion state (IF-08), guarded by MTX_MOTION. */
motion_state_t g_motion_state = { 0U, { 0, 0 }, 0U };

/* Per-wheel PID controller state. */
typedef struct
{
    int32_t integral;
    int32_t prev_err;
} motion_pid_t;

/* ---------------------------------------------------------------------------
 * Module context and packed flags (ARCHITECTURE.md D8).
 * Grouping module-private state into s_mot lets Cortex-M0+ Thumb-1 address
 * fields via immediate offsets from a single base pointer; boolean flags use
 * explicit masks and shifts (no C bit-fields).
 * ------------------------------------------------------------------------- */

#define MOT_FLG_TASK_RUNNING    (1U << 0)   /* motion_task_entry loop active */
#define MOT_FLG_CALIB_HOLD      (1U << 1)   /* suppress PWM during hand-spin */
#define MOT_FLG_CMD_ACTIVE      (1U << 2)   /* non-stop command in progress  */

typedef struct
{
    int32_t          prev_cnt[ENC_WHEEL_COUNT];
    int32_t          manoeuvre_start_cnt[ENC_WHEEL_COUNT];
    motion_pid_t     pid[ENC_WHEEL_COUNT];
    uint32_t         distance_mm_rem;
    uint32_t         distance_cm;
    motion_cmd_t     last_cmd;
    volatile uint32_t release_count;
    volatile uint32_t missed_count;
    volatile uint32_t reject_count;
    int16_t          cmd_arg;           /* 10-bit signed (-512..+511)        */
    uint16_t         hold_elapsed_ms;   /* 0..MOTION_MANOEUVRE_MAX_MS (15000)*/
#if MOTION_HAS_GEOMETRY
    uint16_t         fault_bits_set;    /* EVF_SAFETY b5..b6                 */
    uint8_t          stall_ticks;       /* 0..MOTION_STALL_PERIODS (25)      */
#endif
    uint8_t          cmd_type;          /* 3-bit DREP_MC_* (0..7)            */
    uint8_t          setpoint_cmps;     /* 6-bit speed (0..50 cm/s)          */
    uint8_t          hump_boost_mmps;   /* 0..MOTION_HUMP_BOOST_MAX_MMPS     */
    uint8_t          hump_hold_ticks;   /* 0..MOTION_HUMP_HOLD_TICKS         */
    volatile uint8_t flags;             /* MOT_FLG_* bitmask                 */
} motion_ctx_t;

static motion_ctx_t s_mot;

#if MOTION_HAS_GEOMETRY
static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/* Branchless two's-complement absolute value for signed 32-bit integers. */
static uint32_t abs_i32(int32_t v)
{
    uint32_t const mask = (uint32_t)(v >> 31);

    return ((uint32_t)v + mask) ^ mask;
}
#endif /* MOTION_HAS_GEOMETRY */

static void reset_pid(void)
{
    for (uint32_t w = 0U; w < ENC_WHEEL_COUNT; w++)
    {
        s_mot.pid[w].integral = 0;
        s_mot.pid[w].prev_err = 0;
    }
}

static void stop_internal(bool coast)
{
    s_mot.cmd_type = (uint8_t)DREP_MC_STOP;
    s_mot.setpoint_cmps = 0U;
    s_mot.hump_boost_mmps = 0U;
    s_mot.hump_hold_ticks = 0U;
    s_mot.cmd_arg = 0;
    s_mot.flags = (uint8_t)(s_mot.flags & (uint8_t)~MOT_FLG_CMD_ACTIVE);
    reset_pid();
    if (coast)
    {
        motor_coast(MOTOR_LEFT);
        motor_coast(MOTOR_RIGHT);
    }
    else
    {
        motor_brake_all();
    }
}

/* Poll MBF_MOTION_CMD and apply the newest valid command, or advance T_hold. */
static void poll_command_queue(int32_t const cnt[ENC_WHEEL_COUNT])
{
    motion_cmd_t   cmd;
    bool           got_valid = false;
    uint8_t const  prev_type = s_mot.cmd_type;
    int16_t const  prev_arg = s_mot.cmd_arg;
    bool const     was_active = (s_mot.flags & MOT_FLG_CMD_ACTIVE) != 0U;

    for (;;)
    {
        INT const n = tk_rcv_mbf(MBF_MOTION_CMD, &cmd, TMO_POL);

        if (n == (INT)E_TMOUT)
        {
            break;
        }
        if (n != (INT)sizeof(cmd))
        {
            s_mot.reject_count++;
            continue;
        }
        if (!drep_mc_valid(cmd))
        {
            s_mot.reject_count++;
            dbg_puts("[MOT] invalid motion_cmd_t rejected");
            continue;
        }

        s_mot.last_cmd = cmd;
        s_mot.cmd_type = (uint8_t)drep_mc_type(cmd);
        {
            uint32_t spd = drep_mc_speed(cmd);

            if (spd > MOTION_SPEED_MAX_CMPS)
            {
                spd = MOTION_SPEED_MAX_CMPS;
            }
            s_mot.setpoint_cmps = (uint8_t)spd;
        }
        s_mot.cmd_arg = (int16_t)drep_mc_arg(cmd);
        got_valid = true;
    }

    if (got_valid)
    {
        bool const keep_heading_ref =
            was_active
            && (s_mot.cmd_type == prev_type)
            && ((s_mot.cmd_type == DREP_MC_FWD) || (s_mot.cmd_type == DREP_MC_BACK))
            && (s_mot.cmd_arg == 0) && (prev_arg == 0);

        s_mot.hold_elapsed_ms = 0U;
        if (!keep_heading_ref)
        {
            s_mot.manoeuvre_start_cnt[ENC_WHEEL_LEFT] = cnt[ENC_WHEEL_LEFT];
            s_mot.manoeuvre_start_cnt[ENC_WHEEL_RIGHT] = cnt[ENC_WHEEL_RIGHT];
        }

        if ((s_mot.cmd_type == DREP_MC_STOP) || (s_mot.cmd_type == DREP_MC_BRAKE)
            || (s_mot.setpoint_cmps == 0U))
        {
            stop_internal(false);
        }
        else
        {
            s_mot.flags = (uint8_t)(s_mot.flags | MOT_FLG_CMD_ACTIVE);
        }
        return;
    }

    if ((s_mot.flags & MOT_FLG_CMD_ACTIVE) != 0U)
    {
        uint32_t const type = s_mot.cmd_type;
        bool const is_bounded_manoeuvre =
            (((type == DREP_MC_FWD) || (type == DREP_MC_BACK)
              || (type == DREP_MC_TURN_L) || (type == DREP_MC_TURN_R))
             && (s_mot.cmd_arg > 0))
            || (type == DREP_MC_UTURN);
        uint32_t const hold_limit_ms =
            is_bounded_manoeuvre ? MOTION_MANOEUVRE_MAX_MS : MOTION_T_HOLD_MS;
        uint32_t const elapsed_ms = (uint32_t)s_mot.hold_elapsed_ms + MOTION_PERIOD_MS;

        s_mot.hold_elapsed_ms = (uint16_t)elapsed_ms;
        if (elapsed_ms >= hold_limit_ms)
        {
            stop_internal(false);
        }
    }
}

/* Check whether a bounded-distance or bounded-angle manoeuvre has finished. */
static void check_manoeuvre_completion(int32_t const cnt[ENC_WHEEL_COUNT])
{
#if MOTION_HAS_GEOMETRY
    int32_t const dl = (int32_t)((uint32_t)cnt[ENC_WHEEL_LEFT]
                                 - (uint32_t)s_mot.manoeuvre_start_cnt[ENC_WHEEL_LEFT]);
    int32_t const dr = (int32_t)((uint32_t)cnt[ENC_WHEEL_RIGHT]
                                 - (uint32_t)s_mot.manoeuvre_start_cnt[ENC_WHEEL_RIGHT]);
    uint32_t const mean_counts = (abs_i32(dl) + abs_i32(dr)) / 2U;
    uint32_t const type = s_mot.cmd_type;

    if (((type == DREP_MC_FWD) || (type == DREP_MC_BACK)) && (s_mot.cmd_arg > 0))
    {
        uint32_t const target_mm = (uint32_t)s_mot.cmd_arg * 10U;
        uint32_t const target_counts =
            (target_mm * (uint32_t)MOTION_COUNTS_PER_REV) / MOTION_EFF_WHEEL_CIRC;

        if (mean_counts >= target_counts)
        {
            stop_internal(false);
        }
    }
    else if ((type == DREP_MC_TURN_L) || (type == DREP_MC_TURN_R)
             || (type == DREP_MC_UTURN))
    {
        uint32_t const angle_deg = (type == DREP_MC_UTURN)
                                   ? 180U : (uint32_t)s_mot.cmd_arg;

        if (angle_deg > 0U)
        {
            /* Wheel arc = pi * MOTION_EFF_TRACK_MM * angle_deg / 360. */
            uint32_t const arc_mm = (3142U * MOTION_EFF_TRACK_MM * angle_deg) / 360000U;
            uint32_t const target_counts =
                (arc_mm * (uint32_t)MOTION_COUNTS_PER_REV) / MOTION_EFF_WHEEL_CIRC;

            if (mean_counts >= target_counts)
            {
                stop_internal(false);
            }
        }
    }
#else
    (void)cnt;
#endif
}

#if MOTION_HAS_GEOMETRY
static int32_t pid_step_wheel(uint32_t wheel, int32_t target_counts, int32_t actual_counts)
{
    motion_pid_t *p_pid = &s_mot.pid[wheel];
    int32_t const err = target_counts - actual_counts;
    int32_t const d_err = err - p_pid->prev_err;

    p_pid->integral = clamp_i32(p_pid->integral + err,
                                -MOTION_INTEGRAL_MAX, MOTION_INTEGRAL_MAX);
    p_pid->prev_err = err;

    return (MOTION_PID_KP_PM * err)
           + ((MOTION_PID_KI_X100 * p_pid->integral) / 100)
           + (MOTION_PID_KD_PM * d_err);
}
#endif

static void update_fault_flags(int32_t duty_l, int32_t duty_r, int32_t dl, int32_t dr)
{
#if MOTION_HAS_GEOMETRY
    bool const driving = (abs_i32(duty_l) >= (uint32_t)MOTION_STALL_DUTY_PM)
                         && (abs_i32(duty_r) >= (uint32_t)MOTION_STALL_DUTY_PM);
    uint32_t   new_faults = 0U;

    if (driving && ((dl == 0) || (dr == 0)))
    {
        if (s_mot.stall_ticks < MOTION_STALL_PERIODS)
        {
            s_mot.stall_ticks++;
        }
        if (s_mot.stall_ticks >= MOTION_STALL_PERIODS)
        {
            if ((dl == 0) && (dr == 0))
            {
                new_faults = EVF_SAFETY_FAULT_STALL;
            }
            else
            {
                new_faults = EVF_SAFETY_FAULT_ENCODER;
            }
        }
    }
    else
    {
        s_mot.stall_ticks = 0U;
    }

    if (new_faults != (uint32_t)s_mot.fault_bits_set)
    {
        uint32_t const prev_faults = s_mot.fault_bits_set;
        uint32_t const to_clear = prev_faults & ~new_faults;
        uint32_t const to_set = new_faults & ~prev_faults;

        if (to_clear != 0U)
        {
            (void)tk_clr_flg(EVF_SAFETY, (UINT)~to_clear);
        }
        if (to_set != 0U)
        {
            (void)tk_set_flg(EVF_SAFETY, (UINT)to_set);
        }
        s_mot.fault_bits_set = (uint16_t)new_faults;
    }
#else
    (void)duty_l;
    (void)duty_r;
    (void)dl;
    (void)dr;
#endif
}

/* Compute wheel duties from the active command and encoder feedback. */
static void apply_control(int32_t dl, int32_t dr, int32_t *p_speed_x10)
{
    int32_t        base_pm;
    int32_t        climb_pm = 0;
    int32_t        duty_l = 0;
    int32_t        duty_r = 0;
    int32_t        sync_trim;
    uint32_t const setpoint_cmps = s_mot.setpoint_cmps;
    uint32_t const cmd_type = s_mot.cmd_type;
    uint32_t       eff_speed_mmps;

    if (((s_mot.flags & MOT_FLG_CMD_ACTIVE) == 0U) || (setpoint_cmps == 0U))
    {
        s_mot.hump_boost_mmps = 0U;
        s_mot.hump_hold_ticks = 0U;
        motor_brake_all();
        *p_speed_x10 = 0;
        update_fault_flags(0, 0, dl, dr);
        return;
    }

#if MOTION_HAS_GEOMETRY
    {
        int32_t const  mean_d0 = (dl + dr) / 2;
        int32_t const  actual_mmps = (mean_d0 * (int32_t)MOTION_EFF_WHEEL_CIRC
                                      * (int32_t)(1000U / MOTION_PERIOD_MS))
                                     / MOTION_COUNTS_PER_REV;
        uint32_t const abs_pitch_hd = abs_i32(subsystem4_get_pitch_half_deg());
        bool const     can_hump_boost =
            ((cmd_type == DREP_MC_STEER)
             || ((cmd_type == DREP_MC_FWD) && (s_mot.cmd_arg == 0)))
            && (vc_get_system_state() != VC_SYS_OBSTACLE_AVOIDANCE);

        bool const     is_curving = (cmd_type == DREP_MC_STEER) && (s_mot.cmd_arg != 0);
        bool const     on_hump_incline =
            (abs_pitch_hd >= MOTION_HUMP_PITCH_ON_HD)
            && ((!is_curving) || (actual_mmps < MOTION_HUMP_BOG_SPEED_MMPS));

        if (!can_hump_boost)
        {
            s_mot.hump_boost_mmps = 0U;
            s_mot.hump_hold_ticks = 0U;
        }
        else if (on_hump_incline)
        {
            s_mot.hump_hold_ticks = MOTION_HUMP_HOLD_TICKS;
            if (actual_mmps >= MOTION_HUMP_FAST_CAP_MMPS)
            {
                if (s_mot.hump_boost_mmps > MOTION_HUMP_RAMP_DOWN_MMPS)
                {
                    s_mot.hump_boost_mmps = (uint8_t)(s_mot.hump_boost_mmps
                                                      - MOTION_HUMP_RAMP_DOWN_MMPS);
                }
                else
                {
                    s_mot.hump_boost_mmps = 0U;
                }
            }
            else
            {
                uint32_t const step =
                    ((abs_pitch_hd >= MOTION_HUMP_PITCH_STEEP_HD)
                     && (actual_mmps < (int32_t)(setpoint_cmps * 10U)))
                    ? MOTION_HUMP_RAMP_BOG_MMPS : MOTION_HUMP_RAMP_UP_MMPS;
                uint32_t const next = (uint32_t)s_mot.hump_boost_mmps + step;

                s_mot.hump_boost_mmps = (uint8_t)((next > MOTION_HUMP_BOOST_MAX_MMPS)
                                                  ? MOTION_HUMP_BOOST_MAX_MMPS : next);
            }
        }
        else if ((!is_curving)
                 && (s_mot.hump_hold_ticks > 0U)
                 && (actual_mmps < MOTION_HUMP_FAST_CAP_MMPS))
        {
            s_mot.hump_hold_ticks--;
        }
        else
        {
            s_mot.hump_hold_ticks = 0U;
            if (s_mot.hump_boost_mmps > MOTION_HUMP_RAMP_DOWN_MMPS)
            {
                s_mot.hump_boost_mmps = (uint8_t)(s_mot.hump_boost_mmps
                                                  - MOTION_HUMP_RAMP_DOWN_MMPS);
            }
            else
            {
                s_mot.hump_boost_mmps = 0U;
            }
        }

        eff_speed_mmps = (setpoint_cmps * 10U) + (uint32_t)s_mot.hump_boost_mmps;
        if (eff_speed_mmps > (MOTION_SPEED_MAX_CMPS * 10U))
        {
            eff_speed_mmps = MOTION_SPEED_MAX_CMPS * 10U;
        }

        if ((s_mot.hump_boost_mmps > 0U) && (actual_mmps < (int32_t)eff_speed_mmps))
        {
            int32_t const pos_actual = (actual_mmps > 0) ? actual_mmps : 0;
            int32_t const deficit_mmps = (int32_t)eff_speed_mmps - pos_actual;

            climb_pm = clamp_i32(((int32_t)s_mot.hump_boost_mmps * deficit_mmps) / 25,
                                 0, MOTION_HUMP_CLIMB_MAX_PM);
        }
    }
#else
    eff_speed_mmps = setpoint_cmps * 10U;
#endif

    /* Base feed-forward duty proportional to speed / MOTION_SPEED_MAX_CMPS,
     * scaled to MOTOR_DUTY_MAX_PCT. */
    base_pm = ((int32_t)eff_speed_mmps * (int32_t)MOTOR_DUTY_MAX_PCT)
              / (int32_t)MOTION_SPEED_MAX_CMPS;

    switch (cmd_type)
    {
        case DREP_MC_FWD:
            duty_l = base_pm + climb_pm;
            duty_r = base_pm + climb_pm;
            break;

        case DREP_MC_BACK:
            duty_l = -base_pm;
            duty_r = -base_pm;
            break;

        case DREP_MC_STEER:
        {
            /* cmd_arg in -256..+255: > 0 steers right (left faster, right slower). */
            int32_t const diff_pm = (base_pm * (int32_t)s_mot.cmd_arg) / 256;

            duty_l = base_pm + climb_pm + diff_pm;
            duty_r = base_pm + climb_pm - diff_pm;
            break;
        }

        case DREP_MC_TURN_L:
        {
            int32_t const turn_pm = (base_pm < MOTION_TURN_MIN_BASE_PM)
                                    ? MOTION_TURN_MIN_BASE_PM : base_pm;

            duty_l = -turn_pm;
            duty_r = turn_pm;
            break;
        }

        case DREP_MC_TURN_R:
        case DREP_MC_UTURN:
        {
            int32_t const turn_pm = (base_pm < MOTION_TURN_MIN_BASE_PM)
                                    ? MOTION_TURN_MIN_BASE_PM : base_pm;

            duty_l = turn_pm;
            duty_r = -turn_pm;
            break;
        }

        default:
            duty_l = 0;
            duty_r = 0;
            break;
    }

#if MOTION_HAS_GEOMETRY
    {
        uint32_t const cm_denom = (uint32_t)(2 * MOTION_COUNTS_PER_REV * 10);
        int32_t const  mag_counts =
            ((int32_t)eff_speed_mmps * MOTION_COUNTS_PER_REV * (int32_t)MOTION_PERIOD_MS)
            / ((int32_t)MOTION_EFF_WHEEL_CIRC * 1000);
#if MOTION_STEER_TARGETS
        int32_t const  st_g = (cmd_type == DREP_MC_STEER)
                              ? clamp_i32(((int32_t)s_mot.cmd_arg * (int32_t)MOTION_STEER_TGT_GAIN_PCT) / 100,
                                          -256, 256)
                              : 0;
        int32_t const  tgt_l = ((((duty_l >= 0) ? mag_counts : -mag_counts) * (256 + st_g)) / 256);
        int32_t const  tgt_r = ((((duty_r >= 0) ? mag_counts : -mag_counts) * (256 - st_g)) / 256);
#else
        int32_t const  tgt_l = (duty_l >= 0) ? mag_counts : -mag_counts;
        int32_t const  tgt_r = (duty_r >= 0) ? mag_counts : -mag_counts;
#endif
        int32_t const  mean_d = (dl + dr) / 2;

        duty_l += pid_step_wheel(ENC_WHEEL_LEFT, tgt_l, dl);
        duty_r += pid_step_wheel(ENC_WHEEL_RIGHT, tgt_r, dr);

        if ((cmd_type == DREP_MC_FWD) || (cmd_type == DREP_MC_BACK))
        {
            int32_t const total_dl = (int32_t)((uint32_t)s_mot.prev_cnt[ENC_WHEEL_LEFT]
                                               - (uint32_t)s_mot.manoeuvre_start_cnt[ENC_WHEEL_LEFT]);
            int32_t const total_dr = (int32_t)((uint32_t)s_mot.prev_cnt[ENC_WHEEL_RIGHT]
                                               - (uint32_t)s_mot.manoeuvre_start_cnt[ENC_WHEEL_RIGHT]);
            int32_t const heading_diff = total_dl - total_dr;

            sync_trim = clamp_i32((heading_diff * MOTION_SYNC_KP_PM)
                                  + ((dl - dr) * MOTION_SYNC_KD_PM),
                                  -MOTION_SYNC_MAX_PM, MOTION_SYNC_MAX_PM);
            duty_l -= sync_trim;
            duty_r += sync_trim;

            if (cmd_type == DREP_MC_FWD)
            {
                if (duty_l < 0) { duty_l = 0; }
                if (duty_r < 0) { duty_r = 0; }
            }
            else
            {
                if (duty_l > 0) { duty_l = 0; }
                if (duty_r > 0) { duty_r = 0; }
            }
        }

        if ((dl == 0) && (dr == 0))
        {
            /* Bench fallback when motor power switch is off (wheels still). */
            *p_speed_x10 = (cmd_type == DREP_MC_BACK)
                           ? -((int32_t)setpoint_cmps * 10)
                           : ((int32_t)setpoint_cmps * 10);
            s_mot.distance_mm_rem += ((setpoint_cmps * 10U * MOTION_PERIOD_MS) / 1000U)
                                     * (uint32_t)(2 * MOTION_COUNTS_PER_REV);
        }
        else
        {
            /* 1 mm/s == 0.1 cm/s. */
            *p_speed_x10 = (mean_d * (int32_t)MOTION_EFF_WHEEL_CIRC
                            * (int32_t)(1000U / MOTION_PERIOD_MS))
                           / MOTION_COUNTS_PER_REV;
            s_mot.distance_mm_rem += (abs_i32(dl) + abs_i32(dr)) * MOTION_EFF_WHEEL_CIRC;
        }
        s_mot.distance_cm += s_mot.distance_mm_rem / cm_denom;
        s_mot.distance_mm_rem %= cm_denom;
    }
#else
    if (cmd_type == DREP_MC_BACK)
    {
        *p_speed_x10 = -((int32_t)setpoint_cmps * 10);
    }
    else if ((cmd_type == DREP_MC_FWD) || (cmd_type == DREP_MC_STEER))
    {
        *p_speed_x10 = (int32_t)setpoint_cmps * 10;
    }
    else
    {
        *p_speed_x10 = 0;
    }
    /* Integrate commanded linear travel in mm (speed_cmps * 10 mm/s * 20 ms / 1000). */
    if ((cmd_type == DREP_MC_FWD) || (cmd_type == DREP_MC_BACK)
        || (cmd_type == DREP_MC_STEER))
    {
        s_mot.distance_mm_rem += (setpoint_cmps * 10U * MOTION_PERIOD_MS) / 1000U;
    }
    s_mot.distance_cm += s_mot.distance_mm_rem / 10U;
    s_mot.distance_mm_rem %= 10U;
#endif

    duty_l = motor_duty_clamp_pm(duty_l);
    duty_r = motor_duty_clamp_pm(duty_r);
    motor_set_duty(MOTOR_LEFT, duty_l);
    motor_set_duty(MOTOR_RIGHT, duty_r);
    update_fault_flags(duty_l, duty_r, dl, dr);
}

static void publish_motion_state(int32_t speed_x10,
                                 int32_t const cnt[ENC_WHEEL_COUNT],
                                 uint32_t now_ms)
{
    uint32_t const w = drep_ms_pack(speed_x10, s_mot.distance_cm);

    if (tk_loc_mtx(MTX_MOTION, TMO_FEVR) == E_OK)
    {
        g_motion_state.w = w;
        g_motion_state.enc_count[ENC_WHEEL_LEFT] = cnt[ENC_WHEEL_LEFT];
        g_motion_state.enc_count[ENC_WHEEL_RIGHT] = cnt[ENC_WHEEL_RIGHT];
        g_motion_state.tick_ms = now_ms;
        (void)tk_unl_mtx(MTX_MOTION);
    }
}

void motion_control_init(void)
{
    motor_init();
    enc_init();

    s_mot.release_count = 0U;
    s_mot.missed_count = 0U;
    s_mot.reject_count = 0U;
    s_mot.last_cmd.w = 0U;
    s_mot.hold_elapsed_ms = 0U;
    s_mot.distance_mm_rem = 0U;
    s_mot.distance_cm = 0U;
#if MOTION_HAS_GEOMETRY
    s_mot.stall_ticks = 0U;
    s_mot.fault_bits_set = 0U;
#endif

    s_mot.prev_cnt[ENC_WHEEL_LEFT] = enc_get_count(ENC_WHEEL_LEFT);
    s_mot.prev_cnt[ENC_WHEEL_RIGHT] = enc_get_count(ENC_WHEEL_RIGHT);
    s_mot.manoeuvre_start_cnt[ENC_WHEEL_LEFT] = s_mot.prev_cnt[ENC_WHEEL_LEFT];
    s_mot.manoeuvre_start_cnt[ENC_WHEEL_RIGHT] = s_mot.prev_cnt[ENC_WHEEL_RIGHT];

    stop_internal(false);
    publish_motion_state(0, s_mot.prev_cnt, app_period_now_ms());
}

void motion_step_cycle(uint32_t now_ms)
{
    int32_t cnt[ENC_WHEEL_COUNT];
    int32_t dl;
    int32_t dr;
    int32_t speed_x10 = 0;
    T_RFLG  rflg;

    cnt[ENC_WHEEL_LEFT] = enc_get_count(ENC_WHEEL_LEFT);
    cnt[ENC_WHEEL_RIGHT] = enc_get_count(ENC_WHEEL_RIGHT);
    dl = (int32_t)((uint32_t)cnt[ENC_WHEEL_LEFT] - (uint32_t)s_mot.prev_cnt[ENC_WHEEL_LEFT]);
    dr = (int32_t)((uint32_t)cnt[ENC_WHEEL_RIGHT] - (uint32_t)s_mot.prev_cnt[ENC_WHEEL_RIGHT]);
    s_mot.prev_cnt[ENC_WHEEL_LEFT] = cnt[ENC_WHEEL_LEFT];
    s_mot.prev_cnt[ENC_WHEEL_RIGHT] = cnt[ENC_WHEEL_RIGHT];

    poll_command_queue(cnt);

    /* Safety latch (IF-06) overrides every motion command.  Read with
     * tk_ref_flg() because EVF_SAFETY is TA_WSGL (ARCHITECTURE.md D9). */
    if ((tk_ref_flg(EVF_SAFETY, &rflg) == E_OK)
        && ((rflg.flgptn & EVF_SAFETY_FAULT_LATCHED) != 0U))
    {
        stop_internal(false);
    }

    if ((s_mot.flags & MOT_FLG_CMD_ACTIVE) != 0U)
    {
        check_manoeuvre_completion(cnt);
    }

    if ((s_mot.flags & MOT_FLG_CALIB_HOLD) == 0U)
    {
        apply_control(dl, dr, &speed_x10);
    }
    publish_motion_state(speed_x10, cnt, now_ms);
    s_mot.release_count++;
}

void motion_test_run_periods(uint32_t periods)
{
    if ((s_mot.flags & MOT_FLG_TASK_RUNNING) != 0U)
    {
        if (periods >= 100U)
        {
            app_period_t wait_per;

            app_period_init(&wait_per, periods * MOTION_PERIOD_MS);
            (void)app_period_wait(&wait_per);
        }
        else
        {
            uint32_t const target = s_mot.release_count + periods;

            while ((int32_t)(s_mot.release_count - target) < 0)
            {
                (void)tk_dly_tsk(1);
            }
        }
        return;
    }

    app_period_t per;

    app_period_init(&per, MOTION_PERIOD_MS);
    for (uint32_t i = 0U; i < periods; i++)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            s_mot.missed_count += missed;
            dbg_printf("[MOT] period overrun: %lu missed", (unsigned long)missed);
        }
        motion_step_cycle(app_period_now_ms());
    }
}

void motion_task_entry(INT stacd, void *p_exinf)
{
    app_period_t per;

    (void)stacd;
    (void)p_exinf;

    motion_control_init();
    s_mot.flags = (uint8_t)(s_mot.flags | MOT_FLG_TASK_RUNNING);
    app_period_init(&per, MOTION_PERIOD_MS);

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            s_mot.missed_count += missed;
            dbg_printf("[MOT] period overrun: %lu missed", (unsigned long)missed);
        }
        motion_step_cycle(app_period_now_ms());
    }
    /* Never calls tk_ext_tsk() (rule R6). */
}

ER motion_get_state(motion_state_t *p_out)
{
    ER er;

    if (p_out == NULL)
    {
        return E_PAR;
    }
    er = tk_loc_mtx(MTX_MOTION, TMO_FEVR);
    if (er != E_OK)
    {
        return er;
    }
    *p_out = g_motion_state;
    (void)tk_unl_mtx(MTX_MOTION);
    return E_OK;
}

bool motion_is_task_running(void)
{
    return (s_mot.flags & MOT_FLG_TASK_RUNNING) != 0U;
}

uint32_t motion_get_release_count(void)
{
    return s_mot.release_count;
}

uint32_t motion_get_missed_count(void)
{
    return s_mot.missed_count;
}

uint32_t motion_get_reject_count(void)
{
    return s_mot.reject_count;
}

uint32_t motion_get_setpoint_cmps(void)
{
    return s_mot.setpoint_cmps;
}

uint32_t motion_get_cmd_type(void)
{
    return s_mot.cmd_type;
}

motion_cmd_t motion_get_last_cmd(void)
{
    return s_mot.last_cmd;
}

void motion_set_calib_hold(bool hold)
{
    if (hold)
    {
        s_mot.flags = (uint8_t)(s_mot.flags | MOT_FLG_CALIB_HOLD);
    }
    else
    {
        s_mot.flags = (uint8_t)(s_mot.flags & (uint8_t)~MOT_FLG_CALIB_HOLD);
    }
}
