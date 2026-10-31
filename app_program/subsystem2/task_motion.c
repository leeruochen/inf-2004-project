/*
 *  task_motion.c
 *  Subsystem 2 -- Motion Control task (priority 3, 20 ms periodic PID loop).
 *  Kernel side: kernel and project headers plus kernel-free drv_*.h only,
 *  never SDK headers (rule R1) and never blocking SDK waits (rule R2).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Responsibilities (design.md 3.1/3.2/3.3, specs.md, HANDOVER.md section 3):
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

#include "config/pins.h"
#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "src/debug/dbg.h"

/* ---------------------------------------------------------------------------
 * Geometry compile-time guards (HANDOVER.md section 3, config/thresholds.h):
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
#define MOTION_HAS_GEOMETRY     (1)
#define MOTION_COUNTS_PER_REV   ((int32_t)(ENC_QUAD_MULT * ENC_CPR))
#else
#define MOTION_HAS_GEOMETRY     (0)
#endif

/* ---------------------------------------------------------------------------
 * Initial PID and straight-line correction gains (fixed-point x100, TUNE).
 * ------------------------------------------------------------------------- */

/** Velocity error proportional gain (permille duty per count/period error). TUNE */
#define MOTION_PID_KP_PM        (12)
/** Velocity error integral gain x100 (permille duty per accumulated count). TUNE */
#define MOTION_PID_KI_X100      (25)
/** Velocity error derivative gain (permille duty per count/period change). TUNE */
#define MOTION_PID_KD_PM        (4)
/** Straight-line cross-coupling gain (permille duty per (delta_l - delta_r)). TUNE */
#define MOTION_SYNC_KP_PM       (15)
/** Anti-windup cap on the integral term (in count units). */
#define MOTION_INTEGRAL_MAX     (400)

/** Consecutive 20 ms periods at >= 50% duty with zero counts -> stall/encoder fault. */
#define MOTION_STALL_PERIODS    (25U)   /* 25 x 20 ms = 500 ms */
/** Minimum duty magnitude (permille) before stall detection arms. */
#define MOTION_STALL_DUTY_PM    (500)

/* Shared motion state (IF-08), guarded by MTX_MOTION. */
motion_state_t g_motion_state = { 0U, { 0, 0 }, 0U };

/* Per-wheel PID controller state. */
typedef struct
{
    int32_t integral;
    int32_t prev_err;
} motion_pid_t;

static volatile bool     s_task_running = false;
static volatile uint32_t s_release_count = 0U;
static volatile uint32_t s_missed_count = 0U;
static volatile uint32_t s_reject_count = 0U;

static uint32_t     s_cmd_type = DREP_MC_STOP;
static uint32_t     s_setpoint_cmps = 0U;
static int32_t      s_cmd_arg = 0;
static uint32_t     s_hold_elapsed_ms = 0U;
static bool         s_cmd_active = false;

static int32_t      s_prev_cnt[ENC_WHEEL_COUNT] = { 0, 0 };
static int32_t      s_manoeuvre_start_cnt[ENC_WHEEL_COUNT] = { 0, 0 };
static uint32_t     s_distance_mm_rem = 0U;
static uint32_t     s_distance_cm = 0U;
static motion_pid_t s_pid[ENC_WHEEL_COUNT];
static uint32_t     s_stall_ticks = 0U;
static uint32_t     s_fault_bits_set = 0U;

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

static uint32_t abs_i32(int32_t v)
{
    return (v < 0) ? (uint32_t)(0U - (uint32_t)v) : (uint32_t)v;
}

static void reset_pid(void)
{
    for (uint32_t w = 0U; w < ENC_WHEEL_COUNT; w++)
    {
        s_pid[w].integral = 0;
        s_pid[w].prev_err = 0;
    }
}

static void stop_internal(bool coast)
{
    s_cmd_type = DREP_MC_STOP;
    s_setpoint_cmps = 0U;
    s_cmd_arg = 0;
    s_cmd_active = false;
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
    motion_cmd_t cmd;
    bool         got_valid = false;

    for (;;)
    {
        INT const n = tk_rcv_mbf(MBF_MOTION_CMD, &cmd, TMO_POL);

        if (n == (INT)E_TMOUT)
        {
            break;
        }
        if (n != (INT)sizeof(cmd))
        {
            s_reject_count++;
            continue;
        }
        if (!drep_mc_valid(cmd))
        {
            s_reject_count++;
            dbg_puts("[MOT] invalid motion_cmd_t rejected");
            continue;
        }

        s_cmd_type = drep_mc_type(cmd);
        s_setpoint_cmps = drep_mc_speed(cmd);
        if (s_setpoint_cmps > MOTION_SPEED_MAX_CMPS)
        {
            s_setpoint_cmps = MOTION_SPEED_MAX_CMPS;
        }
        s_cmd_arg = drep_mc_arg(cmd);
        got_valid = true;
    }

    if (got_valid)
    {
        s_hold_elapsed_ms = 0U;
        s_manoeuvre_start_cnt[ENC_WHEEL_LEFT] = cnt[ENC_WHEEL_LEFT];
        s_manoeuvre_start_cnt[ENC_WHEEL_RIGHT] = cnt[ENC_WHEEL_RIGHT];

        if ((s_cmd_type == DREP_MC_STOP) || (s_cmd_type == DREP_MC_BRAKE)
            || (s_setpoint_cmps == 0U))
        {
            stop_internal(false);
        }
        else
        {
            s_cmd_active = true;
        }
        return;
    }

    if (s_cmd_active)
    {
        s_hold_elapsed_ms += MOTION_PERIOD_MS;
        if (s_hold_elapsed_ms >= MOTION_T_HOLD_MS)
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
                                 - (uint32_t)s_manoeuvre_start_cnt[ENC_WHEEL_LEFT]);
    int32_t const dr = (int32_t)((uint32_t)cnt[ENC_WHEEL_RIGHT]
                                 - (uint32_t)s_manoeuvre_start_cnt[ENC_WHEEL_RIGHT]);
    uint32_t const mean_counts = (abs_i32(dl) + abs_i32(dr)) / 2U;

    if (((s_cmd_type == DREP_MC_FWD) || (s_cmd_type == DREP_MC_BACK)) && (s_cmd_arg > 0))
    {
        uint32_t const target_mm = (uint32_t)s_cmd_arg * 10U;
        uint32_t const target_counts =
            (target_mm * (uint32_t)MOTION_COUNTS_PER_REV) / WHEEL_CIRC_MM;

        if (mean_counts >= target_counts)
        {
            stop_internal(false);
        }
    }
    else if ((s_cmd_type == DREP_MC_TURN_L) || (s_cmd_type == DREP_MC_TURN_R)
             || (s_cmd_type == DREP_MC_UTURN))
    {
        uint32_t const angle_deg = (s_cmd_type == DREP_MC_UTURN)
                                   ? 180U : (uint32_t)s_cmd_arg;

        if (angle_deg > 0U)
        {
            /* Wheel arc = pi * CAR_WIDTH_MM * angle_deg / 360. */
            uint32_t const arc_mm = (3142U * CAR_WIDTH_MM * angle_deg) / 360000U;
            uint32_t const target_counts =
                (arc_mm * (uint32_t)MOTION_COUNTS_PER_REV) / WHEEL_CIRC_MM;

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
    motion_pid_t *p_pid = &s_pid[wheel];
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
        if (s_stall_ticks < MOTION_STALL_PERIODS)
        {
            s_stall_ticks++;
        }
        if (s_stall_ticks >= MOTION_STALL_PERIODS)
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
        s_stall_ticks = 0U;
    }

    if (new_faults != s_fault_bits_set)
    {
        uint32_t const to_clear = s_fault_bits_set & ~new_faults;
        uint32_t const to_set = new_faults & ~s_fault_bits_set;

        if (to_clear != 0U)
        {
            (void)tk_clr_flg(EVF_SAFETY, (UINT)~to_clear);
        }
        if (to_set != 0U)
        {
            (void)tk_set_flg(EVF_SAFETY, (UINT)to_set);
        }
        s_fault_bits_set = new_faults;
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
    int32_t base_pm;
    int32_t duty_l = 0;
    int32_t duty_r = 0;
    int32_t sync_trim;

    if ((!s_cmd_active) || (s_setpoint_cmps == 0U))
    {
        motor_brake_all();
        *p_speed_x10 = 0;
        update_fault_flags(0, 0, dl, dr);
        return;
    }

    /* Base feed-forward duty proportional to speed / MOTION_SPEED_MAX_CMPS,
     * scaled to MOTOR_DUTY_MAX_PCT. */
    base_pm = ((int32_t)s_setpoint_cmps * (int32_t)MOTOR_DUTY_MAX_PCT * 10)
              / (int32_t)MOTION_SPEED_MAX_CMPS;

    switch (s_cmd_type)
    {
        case DREP_MC_FWD:
            sync_trim = (dl - dr) * MOTION_SYNC_KP_PM;
            duty_l = base_pm - sync_trim;
            duty_r = base_pm + sync_trim;
            break;

        case DREP_MC_BACK:
            sync_trim = (dl - dr) * MOTION_SYNC_KP_PM;
            duty_l = -base_pm - sync_trim;
            duty_r = -base_pm + sync_trim;
            break;

        case DREP_MC_STEER:
        {
            /* s_cmd_arg in -256..+255: > 0 steers right (left faster, right slower). */
            int32_t const diff_pm = (base_pm * s_cmd_arg) / 256;

            duty_l = base_pm + diff_pm;
            duty_r = base_pm - diff_pm;
            break;
        }

        case DREP_MC_TURN_L:
            duty_l = -base_pm;
            duty_r = base_pm;
            break;

        case DREP_MC_TURN_R:
        case DREP_MC_UTURN:
            duty_l = base_pm;
            duty_r = -base_pm;
            break;

        default:
            duty_l = 0;
            duty_r = 0;
            break;
    }

#if MOTION_HAS_GEOMETRY
    {
        int32_t const mag_counts =
            ((int32_t)s_setpoint_cmps * 10 * MOTION_COUNTS_PER_REV * (int32_t)MOTION_PERIOD_MS)
            / ((int32_t)WHEEL_CIRC_MM * 1000);
        int32_t const tgt_l = (duty_l >= 0) ? mag_counts : -mag_counts;
        int32_t const tgt_r = (duty_r >= 0) ? mag_counts : -mag_counts;
        int32_t const mean_d = (dl + dr) / 2;

        duty_l += pid_step_wheel(ENC_WHEEL_LEFT, tgt_l, dl);
        duty_r += pid_step_wheel(ENC_WHEEL_RIGHT, tgt_r, dr);

        /* 1 mm/s == 0.1 cm/s. */
        *p_speed_x10 = (mean_d * (int32_t)WHEEL_CIRC_MM * (int32_t)(1000U / MOTION_PERIOD_MS))
                       / MOTION_COUNTS_PER_REV;
        s_distance_mm_rem += ((abs_i32(dl) + abs_i32(dr)) * WHEEL_CIRC_MM)
                             / (uint32_t)(2 * MOTION_COUNTS_PER_REV);
    }
#else
    if (s_cmd_type == DREP_MC_BACK)
    {
        *p_speed_x10 = -((int32_t)s_setpoint_cmps * 10);
    }
    else if ((s_cmd_type == DREP_MC_FWD) || (s_cmd_type == DREP_MC_STEER))
    {
        *p_speed_x10 = (int32_t)s_setpoint_cmps * 10;
    }
    else
    {
        *p_speed_x10 = 0;
    }
    /* Integrate commanded linear travel in mm (speed_cmps * 10 mm/s * 20 ms / 1000). */
    if ((s_cmd_type == DREP_MC_FWD) || (s_cmd_type == DREP_MC_BACK)
        || (s_cmd_type == DREP_MC_STEER))
    {
        s_distance_mm_rem += (s_setpoint_cmps * 10U * MOTION_PERIOD_MS) / 1000U;
    }
#endif

    s_distance_cm += s_distance_mm_rem / 10U;
    s_distance_mm_rem %= 10U;

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
    uint32_t const w = drep_ms_pack(speed_x10, s_distance_cm);

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

    s_release_count = 0U;
    s_missed_count = 0U;
    s_reject_count = 0U;
    s_hold_elapsed_ms = 0U;
    s_distance_mm_rem = 0U;
    s_distance_cm = 0U;
    s_stall_ticks = 0U;
    s_fault_bits_set = 0U;

    s_prev_cnt[ENC_WHEEL_LEFT] = enc_get_count(ENC_WHEEL_LEFT);
    s_prev_cnt[ENC_WHEEL_RIGHT] = enc_get_count(ENC_WHEEL_RIGHT);
    s_manoeuvre_start_cnt[ENC_WHEEL_LEFT] = s_prev_cnt[ENC_WHEEL_LEFT];
    s_manoeuvre_start_cnt[ENC_WHEEL_RIGHT] = s_prev_cnt[ENC_WHEEL_RIGHT];

    stop_internal(false);
    publish_motion_state(0, s_prev_cnt, app_period_now_ms());
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
    dl = (int32_t)((uint32_t)cnt[ENC_WHEEL_LEFT] - (uint32_t)s_prev_cnt[ENC_WHEEL_LEFT]);
    dr = (int32_t)((uint32_t)cnt[ENC_WHEEL_RIGHT] - (uint32_t)s_prev_cnt[ENC_WHEEL_RIGHT]);
    s_prev_cnt[ENC_WHEEL_LEFT] = cnt[ENC_WHEEL_LEFT];
    s_prev_cnt[ENC_WHEEL_RIGHT] = cnt[ENC_WHEEL_RIGHT];

    poll_command_queue(cnt);

    /* Safety latch (IF-06) overrides every motion command.  Read with
     * tk_ref_flg() because EVF_SAFETY is TA_WSGL (ARCHITECTURE.md D9). */
    if ((tk_ref_flg(EVF_SAFETY, &rflg) == E_OK)
        && ((rflg.flgptn & EVF_SAFETY_FAULT_LATCHED) != 0U))
    {
        stop_internal(false);
    }

    if (s_cmd_active)
    {
        check_manoeuvre_completion(cnt);
    }

    apply_control(dl, dr, &speed_x10);
    publish_motion_state(speed_x10, cnt, now_ms);
    s_release_count++;
}

void motion_test_run_periods(uint32_t periods)
{
    if (s_task_running)
    {
        (void)tk_dly_tsk((RELTIM)(periods * MOTION_PERIOD_MS));
        return;
    }

    app_period_t per;

    app_period_init(&per, MOTION_PERIOD_MS);
    for (uint32_t i = 0U; i < periods; i++)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            s_missed_count += missed;
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
    s_task_running = true;
    app_period_init(&per, MOTION_PERIOD_MS);

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            s_missed_count += missed;
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
    return s_task_running;
}

uint32_t motion_get_release_count(void)
{
    return s_release_count;
}

uint32_t motion_get_missed_count(void)
{
    return s_missed_count;
}

uint32_t motion_get_reject_count(void)
{
    return s_reject_count;
}

uint32_t motion_get_setpoint_cmps(void)
{
    return s_setpoint_cmps;
}

uint32_t motion_get_cmd_type(void)
{
    return s_cmd_type;
}
