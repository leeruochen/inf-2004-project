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
#include "imu_hump.h"
#include "imu_lift.h"
#include "task_imu.h"
#include "task_safety.h"
#include "../subsystem2/subsystem2.h"
#include "../test_report.h"
#include "config/timing.h"

#if APP_TEST

/* Wait for this many 10 ms IMU cycles before checking the samples. */
#define S4_TEST_SETTLE_MS   (100U)

/* Operator-guided tests S4-U14..U16, S4-U18..U20, S4-U23, S4-U25 and S4-U26: the console asks for
 * one action at a time and the test detects it.  Off by default so other
 * members' APP_TEST runs do not wait for someone to lift their car; turn on
 * with
 *   -DS4_MANUAL_TESTS=ON
 * (S4-U13, collision, was dropped with the requirement change of 11 Nov 2026.) */
#ifndef S4_MANUAL_TESTS
#define S4_MANUAL_TESTS     (0)
#endif

/* -DS4_HUMP_ONLY=ON: of the guided tests, run only the hump steps
 * (S4-U15/U25 on the desk, S4-U26 on the track). */
#ifndef S4_HUMP_ONLY
#define S4_HUMP_ONLY        (0)
#endif

#if S4_MANUAL_TESTS
#define S4_WINDOW_MS        (20000U)    /* time given for each action */
#define S4_POLL_MS          (50U)
#define S4_NOTE_EVERY_MS    (5000U)

/* S4-U23, lift while driving: drive straight this far at cruise speed (20 cm/s
 * in bring-up builds, about 3.5 s), and require the wheels to stop within
 * NFR10's 200 ms of the lift event.  Wheels count as stopped when neither
 * encoder moves for S4_STILL_POLLS polls of S4_DRIVE_POLL_MS. */
#define S4_DRIVE_CM         (70)
#define S4_DRIVE_POLL_MS    (10U)
#define S4_DRIVE_WINDOW_MS  (6000U)
#define S4_STOP_LIMIT_MS    (200U)
#define S4_STILL_POLLS      (3U)

/* S4-U25: the object put under the front wheels, mm (measure it: a stack of
 * coins, a block, a book), and NFR11's tolerance, +-20 % or +-5 mm,
 * whichever is larger (here 5 mm). */
#define S4_HUMP_OBJ_MM      (20)
#define S4_HUMP_TOL_MM      (5)
/* S4-U26: drive this far over the track's hump (start ~20 cm before it). */
#define S4_HUMP_DRIVE_CM    (40)

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

/* Wait until p_ok() is true, or window_ms. */
static bool s4_wait_true(bool (*p_ok)(void), uint32_t window_ms)
{
    for (uint32_t waited = 0U; waited < window_ms; waited += S4_POLL_MS)
    {
        if (p_ok())
        {
            return true;
        }
        (void)tk_dly_tsk(S4_POLL_MS);
    }

    return p_ok();
}

#if !S4_HUMP_ONLY
/* Sum of both encoder counts (free-running), or the previous value on error. */
static int32_t s4_enc_sum(int32_t prev)
{
    motion_state_t st;

    if (motion_get_state(&st) != E_OK)
    {
        return prev;
    }
    return st.enc_count[0] + st.enc_count[1];
}
#endif /* !S4_HUMP_ONLY */

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

    /* S4-U15: hump, and the pitch sign (nose up must read positive).
     * S4-U25: its height, from the same step (FR11, NFR11). */
    dbg_printf("[S4] MANUAL S4-U15/U25: put a %ld mm object under the FRONT wheels 2 s, then remove it",
               (long)S4_HUMP_OBJ_MM);
    {
        bool const    seen   = s4_wait_change(imu_get_hump_count, imu_get_hump_count(), S4_WINDOW_MS);
        int32_t const pitch  = imu_get_last_hump_pitch_half_deg();
        int32_t const height = imu_get_last_hump_height_mm();
        int32_t const sin_p  = imu_hump_sin_x10000(pitch);

        if (seen)
        {
            /* The wheelbase this pitch implies for the object's height, to
             * check IMU_HUMP_WHEELBASE_MM: wheelbase = height / sin(pitch). */
            dbg_printf("[S4]   hump seen, pitch=%ld half-deg, height=%ld mm (object %ld mm)",
                       (long)pitch, (long)height, (long)S4_HUMP_OBJ_MM);
            dbg_printf("[S4]   implied wheelbase %ld mm (configured %lu mm)",
                       (sin_p > 0) ? (long)((S4_HUMP_OBJ_MM * 10000) / sin_p) : -1L,
                       (unsigned long)IMU_HUMP_WHEELBASE_MM);
        }
        TEST_CHECK("S4-U15", seen && (pitch > 0), "hump reported, nose up = positive pitch");
        TEST_CHECK("S4-U25", seen && (height >= (S4_HUMP_OBJ_MM - S4_HUMP_TOL_MM))
                             && (height <= (S4_HUMP_OBJ_MM + S4_HUMP_TOL_MM)),
                   "hump height within NFR11 of the object under the front wheels");
        (void)tk_dly_tsk(1000U);
    }

#if !S4_HUMP_ONLY
    /* S4-U18: lifted -> emergency brake + FAULT_LATCHED. */
    (void)s4_wait_true(imu_is_lift_armed, S4_WINDOW_MS);
    dbg_puts("[S4] MANUAL S4-U18: LIFT the car ~10 cm off the desk, hold it 2 s, put it back down FLAT");
    {
        uint32_t const brakes = safety_get_brake_count();
        bool const     seen   = s4_wait_change(imu_get_lift_count, imu_get_lift_count(), S4_WINDOW_MS);

        (void)tk_dly_tsk(100U);     /* Safety (P1) has run by now */
        TEST_CHECK("S4-U18", seen && (safety_get_brake_count() > brakes) && s4_fault_latched(),
                   "lifted -> emergency brake, FAULT_LATCHED set");

        /* S4-U19: one pick-up is one event; re-arms once level and still. */
        uint32_t const count = imu_get_lift_count();
        bool const     armed = s4_wait_true(imu_is_lift_armed, S4_WINDOW_MS);

        TEST_CHECK("S4-U19", seen && armed && (imu_get_lift_count() == count),
                   "lift detector re-arms after the car is put back down, no second event");
        s4_reset_fault();
    }

    /* S4-U20: overturned (tilted past theta_lift) -> brake + FAULT_LATCHED. */
    /* Roll it over on the desk: lifting it first would trigger RAISED before
     * the tilt (guided run of 13 Nov 2026). */
    dbg_puts("[S4] MANUAL S4-U20: ROLL the car onto its SIDE on the desk (do NOT lift it), 2 s, then back FLAT");
    {
        uint32_t const brakes = safety_get_brake_count();
        bool const     seen   = s4_wait_change(imu_get_lift_count, imu_get_lift_count(), S4_WINDOW_MS);

        (void)tk_dly_tsk(100U);
        TEST_CHECK("S4-U20", seen && (imu_get_last_lift_event() == (uint32_t)IMU_LIFT_TILTED)
                             && (safety_get_brake_count() > brakes) && s4_fault_latched(),
                   "overturned -> tilt detected, emergency brake, FAULT_LATCHED set");
        (void)s4_wait_true(imu_is_lift_armed, S4_WINDOW_MS);
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
#endif /* !S4_HUMP_ONLY */

    /* S4-U26: drive straight over the track's hump and report its height
     * (FR11 on the real hump; compare with a ruler for NFR11).  Placing the
     * car is a lift, so wait until it stands still, then reset the fault. */
    dbg_puts("[S4] MANUAL S4-U26: PUT the car on the TRACK ~20 cm before the HUMP, facing it; hands off");
    {
        (void)s4_wait_change(imu_get_lift_count, imu_get_lift_count(), S4_WINDOW_MS);
        (void)tk_dly_tsk(1000U);
        bool const placed = s4_wait_true(imu_is_lift_armed, S4_WINDOW_MS);

        s4_reset_fault();
        dbg_printf("[S4]   car drives %ld cm straight over the hump in 3 s: hands off",
                   (long)S4_HUMP_DRIVE_CM);
        (void)tk_dly_tsk(3000U);

        uint32_t const humps   = imu_get_hump_count();
        uint32_t const dropped = imu_get_hump_dropped_count();
        uint32_t const lifts   = imu_get_lift_count();

        (void)motion_move_forward(S4_HUMP_DRIVE_CM);
        (void)tk_dly_tsk(S4_DRIVE_WINDOW_MS);
        (void)motion_stop();
        (void)tk_dly_tsk(4000U);    /* a hump ends 0.5 s after the car stops; then its trace */

        uint32_t const n_humps = imu_get_hump_count() - humps;
        bool const     no_lift = (imu_get_lift_count() == lifts);

        dbg_printf("[S4]   U26 placed=%d humps=%lu dropped=%lu height=%ld mm pitch=%ld hd lift=%d",
                   (int)placed, (unsigned long)n_humps,
                   (unsigned long)(imu_get_hump_dropped_count() - dropped),
                   (long)imu_get_last_hump_height_mm(), (long)imu_get_last_hump_pitch_half_deg(),
                   (int)!no_lift);
        dbg_puts("[S4]   measure the hump with a ruler and write both heights in the TEST_LOG");
        TEST_CHECK("S4-U26", placed && (n_humps == 1U) && (imu_get_last_hump_height_mm() > 0) && no_lift,
                   "one hump with a height reported over the track hump, no false lift");
    }

#if !S4_HUMP_ONLY
    /* S4-U23: lifted WHILE DRIVING -> wheels stop within 200 ms (FR12, NFR10,
     * the demo's end-of-run signal).  Moving the car to the track is itself a
     * lift, so wait until it stands level and still, then reset the fault. */
    dbg_puts("[S4] MANUAL S4-U23: PUT the car on the TRACK on a straight, facing along it; hands off");
    {
        (void)s4_wait_change(imu_get_lift_count, imu_get_lift_count(), S4_WINDOW_MS);
        (void)tk_dly_tsk(1000U);
        bool const placed = s4_wait_true(imu_is_lift_armed, S4_WINDOW_MS);

        s4_reset_fault();
        dbg_puts("[S4]   car drives 70 cm straight in 3 s: get ready to LIFT it while it moves");
        (void)tk_dly_tsk(3000U);

        uint32_t const lifts   = imu_get_lift_count();
        uint32_t const brakes  = safety_get_brake_count();
        int32_t        enc     = s4_enc_sum(0);
        bool           seen    = false;
        bool           moving  = false;
        bool           stopped = false;
        uint32_t       t_lift  = 0U;
        uint32_t       t_stop  = 0U;
        uint32_t       still   = 0U;

        (void)motion_move_forward(S4_DRIVE_CM);
        dbg_puts("[S4] MANUAL S4-U23: LIFT the car NOW, while it is driving");

        for (uint32_t waited = 0U; (waited < S4_DRIVE_WINDOW_MS) && !stopped; waited += S4_DRIVE_POLL_MS)
        {
            (void)tk_dly_tsk(S4_DRIVE_POLL_MS);

            uint32_t const now   = app_period_now_ms();
            int32_t const  e     = s4_enc_sum(enc);
            bool const     turns = (e != enc);

            enc = e;

            if (!seen)
            {
                moving = turns;     /* wheels turning in the poll before the lift */

                if (imu_get_lift_count() != lifts)
                {
                    seen   = true;
                    t_lift = now;
                }
                continue;
            }

            still = turns ? 0U : (still + 1U);

            if (still >= S4_STILL_POLLS)
            {
                stopped = true;
                t_stop  = now - ((S4_STILL_POLLS - 1U) * S4_DRIVE_POLL_MS);
            }
        }

        (void)motion_stop();

        uint32_t const stop_ms = stopped ? (t_stop - t_lift) : 0U;

        dbg_printf("[S4]   U23 placed=%d lifted=%d driving=%d stopped=%d in %lu ms after the lift (limit %lu)",
                   (int)placed, (int)seen, (int)moving, (int)stopped,
                   (unsigned long)stop_ms, (unsigned long)S4_STOP_LIMIT_MS);
        TEST_CHECK("S4-U23", placed && seen && moving && stopped && (stop_ms <= S4_STOP_LIMIT_MS)
                             && (safety_get_brake_count() > brakes) && s4_fault_latched(),
                   "lifted while driving -> wheels stop within 200 ms, FAULT_LATCHED set");
        (void)tk_dly_tsk(1000U);
        dbg_puts("[S4]   put the car back down FLAT and keep it still");
        (void)s4_wait_true(imu_is_lift_armed, S4_WINDOW_MS);
        s4_reset_fault();
    }
#endif /* !S4_HUMP_ONLY */

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}
#endif /* S4_MANUAL_TESTS */

void subsystem4_test(void)
{
    TEST_INIT();

    /* When only subsystem 4's tests run (APP_TEST_ONLY=4) they start right
     * after boot: give the level reference (car flat and still for 0.5 s,
     * 10 s fallback) time to finish first. */
    for (uint32_t waited = 0U; (waited < 11000U) && !imu_is_level_done(); waited += 100U)
    {
        (void)tk_dly_tsk(100U);
    }

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
    TEST_CHECK("S4-U17", imu_is_lift_armed() && (imu_get_lift_count() == 0U),
               "lift detector armed, no false lift event with the car at rest");

#if S4_MANUAL_TESTS
    s4_manual_tests(&test_pass_, &test_fail_);
#else
    dbg_puts("[S4] S4-U14..U16, U18..U20, U23 skipped: operator-guided, -DS4_MANUAL_TESTS=ON");
#endif

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
