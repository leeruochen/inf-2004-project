/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4 (IMU processing and safety).
 *  APP_TEST builds only.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem4/logs/ and record the result in
 *  testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U01, S4-U02, ...
 *
 *  The IMU task owns I2C1 while these tests run, so they never touch the bus:
 *  they read the init-time scan results and the task's cached samples.
 *  Keep the car still and flat on the bench.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "task_imu.h"
#include "task_safety.h"
#include "../test_report.h"
#include "config/timing.h"

#if APP_TEST

/* Wait for this many 10 ms IMU cycles before checking the samples. */
#define S4_TEST_SETTLE_MS   (100U)

/* Operator-guided tests S4-U13..U16: the console asks for one action at a
 * time and the test detects it.  Off by default so other members' APP_TEST
 * runs do not wait for someone to knock their car; turn on with
 *   -DS4_MANUAL_TESTS=ON */
#ifndef S4_MANUAL_TESTS
#define S4_MANUAL_TESTS     (0)
#endif

#if S4_MANUAL_TESTS
#define S4_WINDOW_MS        (20000U)    /* time given for each action */
#define S4_POLL_MS          (50U)
#define S4_NOTE_EVERY_MS    (5000U)

/* Wait until p_get() changes from before, or window_ms; reminder every 5 s. */
static bool s4_wait_change(uint32_t (*p_get)(void), uint32_t before, uint32_t window_ms)
{
    uint32_t waited = 0U;

    while (waited < window_ms)
    {
        if (p_get() != before)
        {
            return true;
        }

        (void)tk_dly_tsk(S4_POLL_MS);
        waited += S4_POLL_MS;

        if (((waited % S4_NOTE_EVERY_MS) == 0U) && (waited < window_ms))
        {
            dbg_printf("[S4]   waiting... %lu s left", (unsigned long)((window_ms - waited) / 1000U));
        }
    }

    return p_get() != before;
}

static bool s4_fault_latched(void)
{
    T_RFLG rflg = { 0 };

    (void)tk_ref_flg(EVF_SAFETY, &rflg);
    return (rflg.flgptn & EVF_SAFETY_FAULT_LATCHED) != 0U;
}

/* Ask the Vehicle Controller to clear FAULT_LATCHED, as a remote RESET would:
 * it clears only after the car has been still for SAFETY_T_STILL_MS. */
static void s4_reset_fault(void)
{
    nav_input_t const reset_msg = drep_nav_remote(DREP_NAV_REMOTE_RESET);

    (void)tk_dly_tsk((RELTIM)(SAFETY_T_STILL_MS + 200U));
    (void)tk_snd_mbf(MBF_NAV_IN, &reset_msg, (INT)sizeof(reset_msg), TMO_POL);
    (void)tk_dly_tsk(200U);
    dbg_printf("[S4]   fault reset requested: FAULT_LATCHED %s",
               s4_fault_latched() ? "still set" : "cleared");
}

static void s4_manual_tests(uint32_t *p_pass, uint32_t *p_fail)
{
    uint32_t test_pass_ = *p_pass;
    uint32_t test_fail_ = *p_fail;

    dbg_puts("[S4] ===== MANUAL TESTS: do each step when asked (20 s each) =====");

    /* S4-U15: hump, and the pitch sign (nose up must read positive). */
    dbg_puts("[S4] MANUAL S4-U15: put a ~2 cm book under the FRONT wheels for 2 s, then remove it");
    {
        bool const    seen  = s4_wait_change(imu_get_hump_count, imu_get_hump_count(), S4_WINDOW_MS);
        int32_t const pitch = imu_get_last_hump_pitch_half_deg();

        if (seen)
        {
            dbg_printf("[S4]   hump seen, pitch=%ld half-deg", (long)pitch);
        }
        TEST_CHECK("S4-U15", seen && (pitch > 0), "hump reported, nose up = positive pitch");
        (void)tk_dly_tsk(3000U);    /* time to take the book out */
    }

    /* S4-U13: collision -> emergency brake + FAULT_LATCHED. */
    dbg_puts("[S4] MANUAL S4-U13: knock the car firmly once");
    {
        uint32_t const brakes = safety_get_brake_count();
        bool const     seen   = s4_wait_change(imu_get_collision_count, imu_get_collision_count(),
                                               S4_WINDOW_MS);

        (void)tk_dly_tsk(100U);     /* Safety (P1) has run by now */
        TEST_CHECK("S4-U13", seen && (safety_get_brake_count() > brakes) && s4_fault_latched(),
                   "collision -> emergency brake, FAULT_LATCHED set");
        s4_reset_fault();
    }

    /* S4-U14: IMU lost -> watchdog brake.  S4-U16: it recovers when replugged. */
    dbg_puts("[S4] MANUAL S4-U14: unplug the IMU cable, wait 2 s, plug it back in");
    {
        bool const tripped = s4_wait_change(safety_get_wdog_timeout_count,
                                            safety_get_wdog_timeout_count(), S4_WINDOW_MS);
        bool       back    = false;

        TEST_CHECK("S4-U14", tripped && s4_fault_latched(),
                   "IMU lost -> watchdog brake, FAULT_LATCHED set");

        if (tripped)
        {
            dbg_puts("[S4]   watchdog tripped: plug the IMU back in now");

            for (uint32_t waited = 0U; (waited < S4_WINDOW_MS) && !back; waited += 500U)
            {
                uint32_t const good = imu_get_good_sample_count();

                (void)tk_dly_tsk(500U);
                back = (imu_get_good_sample_count() - good) >= 40U;     /* ~80 % of 50 */
            }
        }
        TEST_CHECK("S4-U16", back, "IMU readings resume after it is plugged back in");
        s4_reset_fault();
    }

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}
#endif /* S4_MANUAL_TESTS */

void subsystem4_test(void)
{
    TEST_INIT();

    TEST_CHECK("S4-U01", imu_accel_found(), "accelerometer found and identified");
    TEST_CHECK("S4-U02", imu_mag_found(), "magnetometer present");
    TEST_CHECK("S4-U03", imu_is_ready(), "accel configured (imu_is_ready)");

    uint32_t const good_before = imu_get_good_sample_count();

    (void)tk_dly_tsk(S4_TEST_SETTLE_MS);

    uint32_t const good_after = imu_get_good_sample_count();

    TEST_CHECK("S4-U04", good_after > good_before, "IMU task reads new samples");

    int32_t const  z_mg  = imu_get_z_mg();
    int32_t const  abs_z = (z_mg >= 0) ? z_mg : -z_mg;
    uint32_t const mag   = imu_get_accel_mag_mg();
    int32_t const  pitch = imu_get_pitch_half_deg();
    int32_t const  abs_p = (pitch >= 0) ? pitch : -pitch;

    dbg_printf("[S4] z=%ld mg mag=%lu mg pitch=%ld half-deg (board %ld when flat)",
               (long)z_mg, (unsigned long)mag, (long)pitch,
               (long)imu_get_pitch_offset_half_deg());

    TEST_CHECK("S4-U05", (abs_z > 800) && (abs_z < 1200), "z-axis ~1 g at rest");
    TEST_CHECK("S4-U06", (mag > 800U) && (mag < 1200U), "magnitude ~1 g at rest");
    TEST_CHECK("S4-U07", imu_is_level_done() && (abs_p < 20),
               "pitch within 10 deg of the boot level reference");

    TEST_CHECK("S4-U08", imu_is_task_running(), "IMU task running");
    TEST_CHECK("S4-U09", safety_is_task_running(), "Safety task running");

    /* 100 ms at 10 ms: expect about 10 releases; allow scheduling slack. */
    TEST_CHECK("S4-U10", (good_after - good_before) >= 8U, "IMU period ~10 ms");
    TEST_CHECK("S4-U11", imu_get_i2c_error_count() == 0U, "no I2C errors");
    TEST_CHECK("S4-U12", safety_get_wdog_timeout_count() == 0U,
               "no IMU watchdog timeouts");

#if S4_MANUAL_TESTS
    s4_manual_tests(&test_pass_, &test_fail_);
#else
    dbg_puts("[S4] S4-U13..U16 skipped: operator-guided, build with -DS4_MANUAL_TESTS=ON");
#endif

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
