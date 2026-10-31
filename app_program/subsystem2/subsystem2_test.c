/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2 (motion control and Vehicle Controller).
 *  APP_TEST builds only.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem2/logs/ and record the result in
 *  testing/subsystem2/TEST_LOG.md.  Test IDs: S2-U01, S2-U02, ...
 */

#include "subsystem2.h"
#include "../test_report.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "task_motion.h"
#include "task_vc.h"
#include "config/thresholds.h"
#include "config/timing.h"
#include "src/rtos/port/hal.h"

#if APP_TEST

#define S2_TEST_CONSOLE_POLL_MS     (10U)
#define S2_TEST_CONSOLE_SETTLE_MS   (50U)

static bool test_s2_u01_motor_pwm_freq(void)
{
    uint32_t const target_hz = (uint32_t)MOTOR_PWM_FREQ_HZ;
    uint32_t const tol_hz = target_hz / 100U;   /* 1 % */
    uint32_t       freq_l;
    uint32_t       freq_r;

    motor_init();
    freq_l = motor_get_pwm_freq_hz(MOTOR_LEFT);
    freq_r = motor_get_pwm_freq_hz(MOTOR_RIGHT);

    return motor_is_ready()
           && (freq_l >= (target_hz - tol_hz)) && (freq_l <= (target_hz + tol_hz))
           && (freq_r >= (target_hz - tol_hz)) && (freq_r <= (target_hz + tol_hz));
}

static bool test_s2_u02_motor_duty_cap(void)
{
    uint32_t fwd_l = 0U;
    uint32_t rev_l = 0U;
    uint32_t per_l = 0U;
    uint32_t fwd_r = 0U;
    uint32_t rev_r = 0U;
    uint32_t per_r = 0U;
    uint32_t cap_l;
    uint32_t cap_r;

    if (motion_is_task_running())
    {
        motion_test_run_periods(1U);
    }
    motor_init();
    motor_set_duty(MOTOR_LEFT, MOTOR_DUTY_FULL_PM);
    motor_set_duty(MOTOR_RIGHT, MOTOR_DUTY_FULL_PM);
    motor_get_levels(MOTOR_LEFT, &fwd_l, &rev_l, &per_l);
    motor_get_levels(MOTOR_RIGHT, &fwd_r, &rev_r, &per_r);
    motor_brake_all();

    cap_l = (per_l * (uint32_t)MOTOR_DUTY_MAX_PCT) / 100U;
    cap_r = (per_r * (uint32_t)MOTOR_DUTY_MAX_PCT) / 100U;

    return (per_l > 0U) && (per_r > 0U)
           && (fwd_l > 0U) && (fwd_l <= cap_l) && (rev_l == 0U)
           && (fwd_r > 0U) && (fwd_r <= cap_r) && (rev_r == 0U)
           && (motor_duty_clamp_pm(MOTOR_DUTY_FULL_PM)
               == ((int32_t)MOTOR_DUTY_MAX_PCT * 10))
           && (motor_duty_clamp_pm(-MOTOR_DUTY_FULL_PM)
               == -((int32_t)MOTOR_DUTY_MAX_PCT * 10));
}

static bool test_s2_u06_emergency_brake(bool pre_init_ok)
{
    bool active_while_driving;
    bool braked_after_call;

    if (motion_is_task_running())
    {
        motion_test_run_periods(1U);
    }
    motor_init();
    motor_set_duty(MOTOR_LEFT, 500);
    motor_set_duty(MOTOR_RIGHT, -500);
    active_while_driving = !motor_are_all_braked();

    motion_emergency_brake();
    braked_after_call = motor_are_all_braked();

    return pre_init_ok && active_while_driving && braked_after_call;
}

static bool test_s2_u09_encoder_nonblocking(void)
{
    uint32_t t0_ms;
    uint32_t elapsed_ms;
    int32_t  acc = 0;

    enc_init();
    if (enc_is_ready() == 0U)
    {
        return false;
    }

    t0_ms = app_period_now_ms();
    for (uint32_t i = 0U; i < 500U; i++)
    {
        acc += enc_get_count(ENC_WHEEL_LEFT);
        acc += enc_get_count(ENC_WHEEL_RIGHT);
    }
    elapsed_ms = app_period_now_ms() - t0_ms;
    (void)acc;

    /* 1000 calls total: < 10 ms means < 10 us average per call. */
    return (elapsed_ms < 10U) && (enc_get_count(ENC_WHEEL_COUNT) == 0);
}

static bool test_s2_u12_motion_period(void)
{
    uint32_t r0;
    uint32_t m0;
    uint32_t releases;
    uint32_t missed;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
    else
    {
        motion_test_run_periods(1U);
    }
    r0 = motion_get_release_count();
    m0 = motion_get_missed_count();
    motion_test_run_periods(500U);  /* 500 x 20 ms = 10 s */
    releases = motion_get_release_count() - r0;
    missed = motion_get_missed_count() - m0;

    return (releases >= 499U) && (releases <= 501U) && (missed == 0U);
}

static bool test_s2_u13_motion_reject_invalid(void)
{
    motion_cmd_t bad = { 1UL << 20 };   /* stray bit above bit 18 */
    uint32_t     rej0;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
    rej0 = motion_get_reject_count();
    if (tk_snd_mbf(MBF_MOTION_CMD, &bad, (INT)sizeof(bad), TMO_POL) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);

    return (motion_get_reject_count() == (rej0 + 1U))
           && (motion_get_setpoint_cmps() == 0U);
}

static bool test_s2_u14_motion_hold_then_stop(void)
{
    motion_cmd_t const cmd = drep_mc_pack(DREP_MC_FWD, 20, 0);
    bool               held_ok;
    bool               stopped_ok;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
    if (tk_snd_mbf(MBF_MOTION_CMD, &cmd, (INT)sizeof(cmd), TMO_POL) != E_OK)
    {
        return false;
    }

    /* Cycle 1 receives the command (0 ms elapsed); cycles 2-4 = 60 ms (< 100 ms). */
    motion_test_run_periods(4U);
    held_ok = (motion_get_setpoint_cmps() == 20U)
              && (motion_get_cmd_type() == DREP_MC_FWD);

    /* 2 more cycles reach >= 100 ms (MOTION_T_HOLD_MS): must drop to 0 and STOP. */
    motion_test_run_periods(2U);
    stopped_ok = (motion_get_setpoint_cmps() == 0U)
                 && (motion_get_cmd_type() == DREP_MC_STOP);

    return held_ok && stopped_ok;
}

static bool test_s2_u15_motion_speed_cap(void)
{
    motion_cmd_t const packed = drep_mc_pack(DREP_MC_FWD, 63, 0);
    motion_cmd_t       raw_over = { 0U };
    uint32_t           rej0;
    bool               packed_ok;
    bool               raw_rejected_ok;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
    if (tk_snd_mbf(MBF_MOTION_CMD, &packed, (INT)sizeof(packed), TMO_POL) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    packed_ok = drep_mc_oor(packed)
                && (motion_get_setpoint_cmps() == MOTION_SPEED_MAX_CMPS);

    /* A raw word with speed = 63 (> 50) must be rejected by drep_mc_valid(). */
    raw_over.w = drep_set(0U, DREP_MC_TYPE_SHIFT, DREP_MC_TYPE_BITS, DREP_MC_FWD);
    raw_over.w = drep_set(raw_over.w, DREP_MC_SPEED_SHIFT, DREP_MC_SPEED_BITS, 63U);
    rej0 = motion_get_reject_count();
    if (tk_snd_mbf(MBF_MOTION_CMD, &raw_over, (INT)sizeof(raw_over), TMO_POL) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    raw_rejected_ok = (motion_get_reject_count() == (rej0 + 1U))
                      && (motion_get_setpoint_cmps() <= MOTION_SPEED_MAX_CMPS);

    /* Stop cleanly afterwards. */
    {
        motion_cmd_t const stop_cmd = drep_mc_pack(DREP_MC_STOP, 0, 0);
        (void)tk_snd_mbf(MBF_MOTION_CMD, &stop_cmd, (INT)sizeof(stop_cmd), TMO_POL);
        motion_test_run_periods(1U);
    }

    return packed_ok && raw_rejected_ok;
}

static bool test_s2_u16_motion_state_published(void)
{
    motion_cmd_t const fwd = drep_mc_pack(DREP_MC_FWD, 30, 0);
    motion_cmd_t const stop_cmd = drep_mc_pack(DREP_MC_STOP, 0, 0);
    motion_state_t     st1;
    motion_state_t     st2;
    bool               enc_ok;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
    if (tk_snd_mbf(MBF_MOTION_CMD, &fwd, (INT)sizeof(fwd), TMO_POL) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    if (motion_get_state(&st1) != E_OK)
    {
        return false;
    }

    /* Run 5 periods (100 ms), refreshing the command once so T_hold does not
     * expire before the second sample. */
    motion_test_run_periods(3U);
    (void)tk_snd_mbf(MBF_MOTION_CMD, &fwd, (INT)sizeof(fwd), TMO_POL);
    motion_test_run_periods(2U);
    if (motion_get_state(&st2) != E_OK)
    {
        return false;
    }

    (void)tk_snd_mbf(MBF_MOTION_CMD, &stop_cmd, (INT)sizeof(stop_cmd), TMO_POL);
    motion_test_run_periods(1U);

    dbg_printf("[MOT] S2-U16 dt=%lu ms dist=%u->%u cm encL=%ld->%ld encR=%ld->%ld",
               (unsigned long)(st2.tick_ms - st1.tick_ms),
               (unsigned)drep_ms_distance_cm(st1.w),
               (unsigned)drep_ms_distance_cm(st2.w),
               (long)st1.enc_count[ENC_WHEEL_LEFT],
               (long)st2.enc_count[ENC_WHEEL_LEFT],
               (long)st1.enc_count[ENC_WHEEL_RIGHT],
               (long)st2.enc_count[ENC_WHEEL_RIGHT]);

#if (ENC_CPR != 0U) && (WHEEL_CIRC_MM != 0U)
    enc_ok = (st2.enc_count[ENC_WHEEL_LEFT] >= st1.enc_count[ENC_WHEEL_LEFT])
             && (st2.enc_count[ENC_WHEEL_RIGHT] >= st1.enc_count[ENC_WHEEL_RIGHT]);
#else
    /* Before ENC_x_DIR_INVERT and MOTOR_x_DIR_INVERT are calibrated on the
     * physical harness, one wheel's raw quadrature count decreases while
     * spinning forward; verify speed, cumulative distance, and tick_ms. */
    enc_ok = (drep_ms_speed_x10(st2.w) == 300);
#endif

    return (st2.tick_ms > st1.tick_ms)
           && (drep_ms_distance_cm(st2.w) > drep_ms_distance_cm(st1.w))
           && enc_ok;
}

/* Helper: ensure any pending motion_cmd_t in MBF_MOTION_CMD is consumed. */
static void drain_motion_cmd_mbf(void)
{
    if (motion_is_task_running())
    {
        motion_test_run_periods(1U);
    }
    else
    {
        motion_cmd_t dummy;

        while (tk_rcv_mbf(MBF_MOTION_CMD, &dummy, TMO_POL) == (INT)sizeof(dummy))
        {
            /* drain */
        }
    }
}

static bool test_s2_u22_vc_reject_invalid(void)
{
    /* Bypass value 6 is out of range (DREP_BYPASS_COUNT = 5 -> 5..7 invalid). */
    nav_input_t const bad = drep_nav_make(DREP_NAV_SRC_AVOID, DREP_NAV_AVOID_BYPASS, 0U, 6U, false);
    uint32_t          rej0;
    uint32_t          sent0;

    if (!vc_is_task_running())
    {
        vc_init();
        vc_complete_selftest(true);
    }
    rej0 = vc_get_reject_count();
    sent0 = vc_get_sent_cmd_count();

    if (tk_snd_mbf(MBF_NAV_IN, &bad, (INT)sizeof(bad), TMO_POL) != E_OK)
    {
        return false;
    }
    vc_drain_and_step(app_period_now_ms());

    return (vc_get_reject_count() == (rej0 + 1U))
           && (vc_get_sent_cmd_count() == sent0);
}

static bool test_s2_u23_vc_avoid_precedence(void)
{
    nav_input_t const  start_msg = drep_nav_remote(DREP_NAV_REMOTE_START);
    nav_input_t const  line_msg = drep_nav_steer(64);
    nav_input_t const  avoid_msg = drep_nav_avoid(DREP_NAV_AVOID_BYPASS, DREP_BYPASS_LEFT);
    uint32_t           sent_before_line;
    uint32_t           sent_after_line;
    uint32_t           sent_after_avoid;
    motion_cmd_t       last_cmd;

    if (!vc_is_task_running())
    {
        vc_init();
        vc_complete_selftest(true);
    }
    /* Enter navigation, then switch authority to AVOID. */
    (void)tk_snd_mbf(MBF_NAV_IN, &start_msg, (INT)sizeof(start_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    vc_set_authority(VC_AUTH_AVOID);

    sent_before_line = vc_get_sent_cmd_count();
    (void)tk_snd_mbf(MBF_NAV_IN, &line_msg, (INT)sizeof(line_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    sent_after_line = vc_get_sent_cmd_count();

    (void)tk_snd_mbf(MBF_NAV_IN, &avoid_msg, (INT)sizeof(avoid_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    sent_after_avoid = vc_get_sent_cmd_count();
    last_cmd = vc_get_last_motion_cmd();

    /* Restore READY state. */
    {
        nav_input_t const stop_msg = drep_nav_remote(DREP_NAV_REMOTE_STOP);
        (void)tk_snd_mbf(MBF_NAV_IN, &stop_msg, (INT)sizeof(stop_msg), TMO_POL);
        vc_drain_and_step(app_period_now_ms());
    }
    drain_motion_cmd_mbf();

    return (sent_after_line == sent_before_line)
           && (sent_after_avoid == (sent_before_line + 1U))
           && (drep_mc_type(last_cmd) == DREP_MC_TURN_L);
}

static bool test_s2_u24_vc_stops_on_fault_latched(void)
{
    nav_input_t const line_msg = drep_nav_steer(32);
    motion_cmd_t      last_cmd;
    uint32_t          cmd_type;

    if (!vc_is_task_running())
    {
        vc_init();
        vc_complete_selftest(true);
    }
    (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED);
    (void)tk_snd_mbf(MBF_NAV_IN, &line_msg, (INT)sizeof(line_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());

    last_cmd = vc_get_last_motion_cmd();
    cmd_type = drep_mc_type(last_cmd);
    (void)tk_clr_flg(EVF_SAFETY, (UINT)~EVF_SAFETY_FAULT_LATCHED);
    drain_motion_cmd_mbf();

    return (vc_get_system_state() == VC_SYS_SAFETY_FAULT_LATCHED)
           && ((cmd_type == DREP_MC_BRAKE) || (cmd_type == DREP_MC_STOP))
           && (drep_mc_speed(last_cmd) == 0U);
}

static bool test_s2_u25_vc_reset_stationary_only(void)
{
    nav_input_t const reset_msg = drep_nav_remote(DREP_NAV_REMOTE_RESET);
    uint32_t          t0;
    T_RFLG            rflg1 = { 0 };
    T_RFLG            rflg2 = { 0 };
    bool              moving_ignored;
    bool              stationary_cleared;

    if (!vc_is_task_running())
    {
        vc_init();
        vc_complete_selftest(true);
    }

    /* Align with the 20 ms Motion cycle so g_motion_state.w is not overwritten
     * mid-step while simulating non-zero speed. */
    drain_motion_cmd_mbf();
    t0 = app_period_now_ms();

    /* Set FAULT_LATCHED plus another bit (STATUS_NO_ECHO) to verify only
     * FAULT_LATCHED is cleared on reset. */
    (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED | EVF_SAFETY_STATUS_NO_ECHO);

    /* 1. Simulate moving (speed != 0 under MTX_MOTION), then send RESET. */
    if (tk_loc_mtx(MTX_MOTION, TMO_FEVR) == E_OK)
    {
        g_motion_state.w = drep_ms_pack(200, 0U);   /* 20.0 cm/s */
        (void)tk_unl_mtx(MTX_MOTION);
    }
    (void)tk_snd_mbf(MBF_NAV_IN, &reset_msg, (INT)sizeof(reset_msg), TMO_POL);
    vc_drain_and_step(t0);
    (void)tk_ref_flg(EVF_SAFETY, &rflg1);
    moving_ignored = ((rflg1.flgptn & EVF_SAFETY_FAULT_LATCHED) != 0U)
                     && (vc_get_system_state() == VC_SYS_SAFETY_FAULT_LATCHED);

    /* 2. Set speed = 0, mark stationary start at t0, then step at t0 + 500 ms. */
    if (tk_loc_mtx(MTX_MOTION, TMO_FEVR) == E_OK)
    {
        g_motion_state.w = drep_ms_pack(0, 0U);
        (void)tk_unl_mtx(MTX_MOTION);
    }
    t0 = app_period_now_ms();
    vc_drain_and_step(t0);
    if (vc_is_task_running())
    {
        (void)tk_dly_tsk((RELTIM)(SAFETY_T_STILL_MS + 40U));
    }
    (void)tk_snd_mbf(MBF_NAV_IN, &reset_msg, (INT)sizeof(reset_msg), TMO_POL);
    vc_drain_and_step(t0 + SAFETY_T_STILL_MS);
    (void)tk_ref_flg(EVF_SAFETY, &rflg2);

    stationary_cleared = ((rflg2.flgptn & EVF_SAFETY_FAULT_LATCHED) == 0U)
                         && ((rflg2.flgptn & EVF_SAFETY_STATUS_NO_ECHO) != 0U)
                         && (vc_get_system_state() == VC_SYS_READY_WAITING);

    (void)tk_clr_flg(EVF_SAFETY, (UINT)~(EVF_SAFETY_FAULT_LATCHED | EVF_SAFETY_STATUS_NO_ECHO));
    drain_motion_cmd_mbf();
    return moving_ignored && stationary_cleared;
}

static bool test_s2_u26_vc_fsm_powerup_to_nav(void)
{
    nav_input_t const start_msg = drep_nav_remote(DREP_NAV_REMOTE_START);
    uint32_t          st_init;
    uint32_t          st_ready;
    uint32_t          st_nav;

    vc_init();
    st_init = vc_get_system_state();

    vc_complete_selftest(true);
    st_ready = vc_get_system_state();

    (void)tk_snd_mbf(MBF_NAV_IN, &start_msg, (INT)sizeof(start_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    st_nav = vc_get_system_state();

    drain_motion_cmd_mbf();
    return (st_init == VC_SYS_INITIALISING)
           && (st_ready == VC_SYS_READY_WAITING)
           && (st_nav == VC_SYS_NAV_LINE_TRACKING);
}

static bool test_s2_u27_vc_fsm_track_lost_and_stop(void)
{
    nav_input_t const start_msg = drep_nav_remote(DREP_NAV_REMOTE_START);
    nav_input_t const lost_msg = drep_nav_line_event(DREP_NAV_LINE_TRACK_LOST);
    nav_input_t const stop_msg = drep_nav_remote(DREP_NAV_REMOTE_STOP);
    uint32_t          st_after_lost;
    uint32_t          cmd_after_lost;
    uint32_t          st_after_stop;
    uint32_t          cmd_after_stop;

    if (!vc_is_task_running())
    {
        vc_init();
        vc_complete_selftest(true);
    }

    /* Enter NAV, then post LINE TRACK_LOST -> READY + STOP. */
    (void)tk_snd_mbf(MBF_NAV_IN, &start_msg, (INT)sizeof(start_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    (void)tk_snd_mbf(MBF_NAV_IN, &lost_msg, (INT)sizeof(lost_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    st_after_lost = vc_get_system_state();
    cmd_after_lost = drep_mc_type(vc_get_last_motion_cmd());

    /* Restart NAV, then post remote STOP -> READY + STOP. */
    (void)tk_snd_mbf(MBF_NAV_IN, &start_msg, (INT)sizeof(start_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    (void)tk_snd_mbf(MBF_NAV_IN, &stop_msg, (INT)sizeof(stop_msg), TMO_POL);
    vc_drain_and_step(app_period_now_ms());
    st_after_stop = vc_get_system_state();
    cmd_after_stop = drep_mc_type(vc_get_last_motion_cmd());

    drain_motion_cmd_mbf();
    return (st_after_lost == VC_SYS_READY_WAITING)
           && (cmd_after_lost == DREP_MC_STOP)
           && (st_after_stop == VC_SYS_READY_WAITING)
           && (cmd_after_stop == DREP_MC_STOP);
}

static bool test_s2_u28_public_motion_api(void)
{
    motion_cmd_t cmd_fwd;
    motion_cmd_t cmd_back;
    motion_cmd_t cmd_left;
    motion_cmd_t cmd_right;
    motion_cmd_t cmd_stop;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
    drain_motion_cmd_mbf();

    if (moveForward(100) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    cmd_fwd = motion_get_last_cmd();

    if (moveBackward(25) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    cmd_back = motion_get_last_cmd();

    if (turnLeft(90) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    cmd_left = motion_get_last_cmd();

    if (turnRight(45) != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    cmd_right = motion_get_last_cmd();

    if (stop() != E_OK)
    {
        return false;
    }
    motion_test_run_periods(1U);
    cmd_stop = motion_get_last_cmd();

    return drep_mc_valid(cmd_fwd)
           && (drep_mc_type(cmd_fwd) == DREP_MC_FWD)
           && (drep_mc_arg(cmd_fwd) == 100)
           && (drep_mc_speed(cmd_fwd) <= MOTION_SPEED_MAX_CMPS)
           && drep_mc_valid(cmd_back)
           && (drep_mc_type(cmd_back) == DREP_MC_BACK)
           && (drep_mc_arg(cmd_back) == 25)
           && drep_mc_valid(cmd_left)
           && (drep_mc_type(cmd_left) == DREP_MC_TURN_L)
           && (drep_mc_arg(cmd_left) == 90)
           && drep_mc_valid(cmd_right)
           && (drep_mc_type(cmd_right) == DREP_MC_TURN_R)
           && (drep_mc_arg(cmd_right) == 45)
           && drep_mc_valid(cmd_stop)
           && (drep_mc_type(cmd_stop) == DREP_MC_STOP)
           && (drep_mc_speed(cmd_stop) == 0U)
           && (motion_get_setpoint_cmps() == 0U);
}

#ifndef S2_HW_CALIB_MODE
#define S2_HW_CALIB_MODE    (0)
#endif

#if S2_HW_CALIB_MODE
/*
 * Phase 1 manual bench bring-up and calibration sequence:
 *   Step 1 (S2-U03): Left wheel +40% (2 s) then -40% (2 s)
 *   Step 2 (S2-U03): Right wheel +40% (2 s) then -40% (2 s)
 *   Step 3 (S2-U04): Coast (3 s, GP8..11 HIGH) then Brake (2 s, GP8..11 LOW)
 *   Step 4 (S2-U05): Both wheels +50% (1.5 s) then motion_emergency_brake()
 *   Step 5 (S2-U07 / S2-U10): 35 s hand-spin window for sign & 10-turn ENC_CPR
 */
static void run_hw_calibration_stream(void)
{
    int32_t l0;
    int32_t r0;
    int32_t dl;
    int32_t dr;

    motion_set_calib_hold(true);
    motor_init();
    enc_init();
    /* Put motors in COAST so wheels turn freely by hand without dynamic braking. */
    motor_coast(MOTOR_LEFT);
    motor_coast(MOTOR_RIGHT);
    (void)tk_dly_tsk(200);

    l0 = enc_get_count(ENC_WHEEL_LEFT);
    r0 = enc_get_count(ENC_WHEEL_RIGHT);
    dbg_puts("[S2-PHASE1] >>> HAND-SPIN ONE WHEEL FORWARD 10 EXACT REVOLUTIONS (35 s window) <<<");
    for (uint32_t s = 1U; s <= 35U; s++)
    {
        (void)tk_dly_tsk(1000);
        dl = enc_get_count(ENC_WHEEL_LEFT) - l0;
        dr = enc_get_count(ENC_WHEEL_RIGHT) - r0;
        dbg_printf("[S2-CALIB] t=%2lus/35s  dL=%6ld  dR=%6ld  | 1-rev CPR: L=%ld R=%ld | 10-rev CPR: L=%ld R=%ld",
                   (unsigned long)s,
                   (long)dl,
                   (long)dr,
                   (long)(dl / (int32_t)ENC_QUAD_MULT),
                   (long)(dr / (int32_t)ENC_QUAD_MULT),
                   (long)(dl / (10 * (int32_t)ENC_QUAD_MULT)),
                   (long)(dr / (10 * (int32_t)ENC_QUAD_MULT)));
    }
    motor_brake_all();
    dbg_puts("[S2-PHASE1] Hand-spin calibration complete.");
    motion_set_calib_hold(false);
}
#endif /* S2_HW_CALIB_MODE */

void subsystem2_test(void)
{
    bool pre_init_brake_ok = subsystem2_pre_start_brake_ok();

    /* Wait for the USB CDC terminal to open and let dbg_drain_task (priority
     * 10) flush the boot lines first, so no [TEST] lines overflow the 2 KB
     * ring if the terminal connects after the 3 s pre-kernel wait. */
    while (!hal_console_connected())
    {
        (void)tk_dly_tsk(S2_TEST_CONSOLE_POLL_MS);
    }
    (void)tk_dly_tsk(S2_TEST_CONSOLE_SETTLE_MS);

    TEST_INIT();

    if (!pre_init_brake_ok)
    {
        motion_emergency_brake();
        pre_init_brake_ok = motor_are_all_braked();
    }

    TEST_CHECK("S2-U01", test_s2_u01_motor_pwm_freq(),
               "Motor PWM slices 4 and 5 configured at 20 kHz +-1%");
    TEST_CHECK("S2-U02", test_s2_u02_motor_duty_cap(),
               "Motor duty capped at MOTOR_DUTY_MAX_PCT");
    TEST_CHECK("S2-U06", test_s2_u06_emergency_brake(pre_init_brake_ok),
               "motion_emergency_brake drives all four channels LOW without Motion task");
    TEST_CHECK("S2-U09", test_s2_u09_encoder_nonblocking(),
               "enc_get_count is non-blocking (< 10 us avg over 1000 calls)");
#if S2_HW_CALIB_MODE
    run_hw_calibration_stream();
#endif
    TEST_CHECK("S2-U12", test_s2_u12_motion_period(),
               "Motion period: 500 +- 1 releases in 10 s, 0 missed");
    TEST_CHECK("S2-U13", test_s2_u13_motion_reject_invalid(),
               "Motion rejects invalid motion_cmd_t and keeps setpoint");
    TEST_CHECK("S2-U14", test_s2_u14_motion_hold_then_stop(),
               "Motion holds setpoint for MOTION_T_HOLD_MS then stops");
    TEST_CHECK("S2-U15", test_s2_u15_motion_speed_cap(),
               "Motion caps speed at MOTION_SPEED_MAX_CMPS (50 cm/s)");
    TEST_CHECK("S2-U16", test_s2_u16_motion_state_published(),
               "motion_state_t published under MTX_MOTION advances over 100 ms");
    TEST_CHECK("S2-U22", test_s2_u22_vc_reject_invalid(),
               "VC rejects invalid nav_input_t and sends no motion_cmd_t");
    TEST_CHECK("S2-U23", test_s2_u23_vc_avoid_precedence(),
               "VC precedence: AVOID overrides LINE while authority is AVOID");
    TEST_CHECK("S2-U24", test_s2_u24_vc_stops_on_fault_latched(),
               "VC commands stop/brake when EVF_SAFETY_FAULT_LATCHED is set");
    TEST_CHECK("S2-U25", test_s2_u25_vc_reset_stationary_only(),
               "VC clears FAULT_LATCHED only after SAFETY_T_STILL_MS stationary");
    TEST_CHECK("S2-U26", test_s2_u26_vc_fsm_powerup_to_nav(),
               "System FSM transitions INITIALISING -> READY -> NAV_LINE_TRACKING");
    TEST_CHECK("S2-U27", test_s2_u27_vc_fsm_track_lost_and_stop(),
               "System FSM returns to READY and stops on TRACK_LOST and STOP");
    TEST_CHECK("S2-U28", test_s2_u28_public_motion_api(),
               "Public API (moveForward/Backward, turnLeft/Right, stop) posts valid motion_cmd_t");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
