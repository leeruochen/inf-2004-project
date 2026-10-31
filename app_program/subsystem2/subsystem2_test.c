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
#include "config/thresholds.h"
#include "config/timing.h"

#if APP_TEST

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
    uint32_t const r0 = motion_get_release_count();
    uint32_t const m0 = motion_get_missed_count();
    uint32_t       releases;
    uint32_t       missed;

    if (!motion_is_task_running())
    {
        motion_control_init();
    }
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

    return (st2.tick_ms > st1.tick_ms)
           && (drep_ms_distance_cm(st2.w) >= drep_ms_distance_cm(st1.w))
           && (st2.enc_count[ENC_WHEEL_LEFT] >= st1.enc_count[ENC_WHEEL_LEFT])
           && (st2.enc_count[ENC_WHEEL_RIGHT] >= st1.enc_count[ENC_WHEEL_RIGHT]);
}

void subsystem2_test(void)
{
    bool pre_init_brake_ok;

    TEST_INIT();

    /* Exercise motion_emergency_brake() before motor_init() first (S2-U06). */
    motion_emergency_brake();
    pre_init_brake_ok = motor_are_all_braked();

    TEST_CHECK("S2-U01", test_s2_u01_motor_pwm_freq(),
               "Motor PWM slices 4 and 5 configured at 20 kHz +-1%");
    TEST_CHECK("S2-U02", test_s2_u02_motor_duty_cap(),
               "Motor duty capped at MOTOR_DUTY_MAX_PCT");
    TEST_CHECK("S2-U06", test_s2_u06_emergency_brake(pre_init_brake_ok),
               "motion_emergency_brake drives all four channels LOW without Motion task");
    TEST_CHECK("S2-U09", test_s2_u09_encoder_nonblocking(),
               "enc_get_count is non-blocking (< 10 us avg over 1000 calls)");
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

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
