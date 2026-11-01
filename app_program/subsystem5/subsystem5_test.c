/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5 (obstacle scanning and avoidance).
 *  APP_TEST builds only.
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem5/logs/ and record the result in
 *  testing/subsystem5/TEST_LOG.md.  Test IDs: S5-U01 .. S5-U25.
 *
 *  S5-U01..S5-U20 are self-contained: they drive the pure geometry and
 *  decision functions with synthetic sweeps, so they pass or fail on logic
 *  alone and need no obstacle in front of the car.  S5-U21..S5-U25 touch the
 *  real hardware and the running tasks, so they need the car powered with
 *  the servo and the ultrasonic sensor connected; S5-U25 needs an object
 *  placed roughly 15 cm in front of the sensor (see TEST_LOG.md).
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

#include "config/thresholds.h"
#include "config/timing.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "task_obs_avoid.h"
#include "task_obs_scan.h"

/** Put one synthetic return into a sweep, at a whole-ten-degree bearing. */
static void put_return(obs_scan_t *p_scan, uint32_t deg, uint16_t mm)
{
    p_scan->dist_mm[OBS_DEG_TO_SLOT(deg)] = mm;
}

/** A sweep covering the coarse span with nothing in it. */
static void empty_sweep(obs_scan_t *p_scan)
{
    for (uint32_t i = 0U; i < OBS_BEARING_SLOTS; i++)
    {
        p_scan->dist_mm[i] = 0U;
    }
    p_scan->lo_slot = (uint8_t)OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MIN);
    p_scan->hi_slot = (uint8_t)OBS_DEG_TO_SLOT(OBS_COARSE_DEG_MAX);
}

void subsystem5_test(void)
{
    TEST_INIT();

    /* -----------------------------------------------------------------
     * Servo driver (S5-U01..S5-U03)
     * ----------------------------------------------------------------- */

    uint32_t const pulse_min = servo_deg_to_pulse_us(SERVO_DEG_MIN);
    uint32_t const pulse_mid = servo_deg_to_pulse_us(SERVO_DEG_CENTRE);
    uint32_t const pulse_max = servo_deg_to_pulse_us(SERVO_DEG_MAX);
    uint32_t const pulse_span = (uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US;

    TEST_CHECK("S5-U01",
               (pulse_min == (uint32_t)SERVO_PULSE_MIN_US)
               && (pulse_max == (uint32_t)SERVO_PULSE_MAX_US)
               && (pulse_mid == ((uint32_t)SERVO_PULSE_MIN_US + (pulse_span / 2U)))
               && (servo_deg_to_pulse_us(500U) == pulse_max),
               "servo angle to pulse: endpoints, centre, clamp");

    uint32_t const pwm_hz = servo_get_pwm_freq_hz();

    TEST_CHECK("S5-U02",
               servo_is_ready()
               && (pwm_hz >= ((uint32_t)SERVO_PWM_FREQ_HZ - 2U))
               && (pwm_hz <= ((uint32_t)SERVO_PWM_FREQ_HZ + 2U)),
               "servo PWM slice 7 runs at SERVO_PWM_FREQ_HZ");

    TEST_CHECK("S5-U03", servo_chan_a_is_low(),
               "slice 7 channel A (GP14, another owner) stays LOW");

    /* -----------------------------------------------------------------
     * Ultrasonic driver (S5-U04..S5-U05)
     * ----------------------------------------------------------------- */

    /* 1160 us round trip is about 199 mm one way at 343 m/s. */
    uint32_t const mm_1160 = us_echo_us_to_mm(1160U);

    TEST_CHECK("S5-U04",
               (us_echo_us_to_mm(0U) == 0U)
               && (us_echo_us_to_mm((uint32_t)US_ECHO_MAX_US + 1U) == 0U)
               && (mm_1160 >= 195U) && (mm_1160 <= 205U),
               "echo width to mm: zero, over-range, 1160 us = ~199 mm");

    TEST_CHECK("S5-U05", us_is_ready() && !us_echo_pin_is_high(),
               "echo handler registered and echo line idle LOW");

    /* -----------------------------------------------------------------
     * Pure geometry (S5-U06..S5-U08)
     * ----------------------------------------------------------------- */

    TEST_CHECK("S5-U06",
               (obs_cos_m1000(0U) == 1000) && (obs_cos_m1000(90U) == 0)
               && (obs_cos_m1000(180U) == -1000)
               && (obs_sin_m1000(90U) == 1000) && (obs_sin_m1000(0U) == 0)
               && (obs_sin_m1000(180U) == 0),
               "cos/sin tables: 0, 90 and 180 degrees");

    TEST_CHECK("S5-U07",
               (obs_lateral_mm(1000U, 30U) > 0)        /* 30 deg is to the right */
               && (obs_lateral_mm(1000U, 150U) < 0)    /* 150 deg is to the left */
               && (obs_lateral_mm(1000U, 90U) == 0)    /* dead ahead */
               && (obs_forward_mm(1000U, 90U) == 1000),
               "lateral sign convention: positive is to the car's right");

    TEST_CHECK("S5-U08",
               (obs_median3_mm(0U, 0U, 0U) == 0U)
               && (obs_median3_mm(150U, 0U, 0U) == 150U)
               && (obs_median3_mm(150U, 900U, 0U) == 150U)
               && (obs_median3_mm(900U, 150U, 160U) == 160U),
               "median of three samples ignores no-echo and rejects an outlier");

    /* -----------------------------------------------------------------
     * Sweep analysis (S5-U09..S5-U13)
     * ----------------------------------------------------------------- */

    obs_scan_t   scan;
    obs_result_t res;

    empty_sweep(&scan);
    TEST_CHECK("S5-U09", !obs_analyse(&scan, &res) && !res.valid,
               "an empty sweep is not an obstacle");

    empty_sweep(&scan);
    put_return(&scan, 90U, 150U);
    TEST_CHECK("S5-U10",
               obs_analyse(&scan, &res) && res.valid
               && (res.near_mm == 150U) && (res.bearing_deg == 90U)
               && (res.width_mm > 0U),
               "one near bearing dead ahead: 150 mm at 90 degrees");

    empty_sweep(&scan);
    put_return(&scan, 60U, 200U);
    put_return(&scan, 70U, 200U);
    put_return(&scan, 80U, 200U);
    put_return(&scan, 90U, 200U);
    put_return(&scan, 100U, 200U);
    put_return(&scan, 110U, 200U);
    put_return(&scan, 120U, 200U);
    scan.lo_slot = (uint8_t)OBS_DEG_TO_SLOT(60U);
    scan.hi_slot = (uint8_t)OBS_DEG_TO_SLOT(120U);
    {
        /* Lateral extent of the run is 200 x (cos 60 - cos 120) = 200 mm. */
        bool const ok = obs_analyse(&scan, &res) && res.valid
                        && (res.left_deg == 120U) && (res.right_deg == 60U)
                        && (res.bearing_deg == 90U)
                        && (res.width_mm >= 200U);

        TEST_CHECK("S5-U11", ok,
                   "a wide obstacle spans 60..120 degrees, centres on 90, reports >= 200 mm");
    }

    empty_sweep(&scan);
    put_return(&scan, 90U, 150U);
    TEST_CHECK("S5-U12",
               obs_analyse(&scan, &res)
               && (res.clear_l_mm == (uint16_t)DREP_OBS_CLR_MAX_MM)
               && (res.clear_r_mm == (uint16_t)DREP_OBS_CLR_MAX_MM),
               "with nothing beside it, both clearances saturate at 635 mm");

    empty_sweep(&scan);
    put_return(&scan, 90U, 150U);       /* the obstacle, dead ahead */
    put_return(&scan, 60U, 220U);       /* a second object to the right */
    {
        /* 220 mm at 60 degrees sits 110 mm to the right of the centre-line. */
        bool const ok = obs_analyse(&scan, &res) && res.valid
                        && (res.bearing_deg == 90U)
                        && (res.clear_r_mm >= 100U) && (res.clear_r_mm <= 120U)
                        && (res.clear_l_mm == (uint16_t)DREP_OBS_CLR_MAX_MM);

        TEST_CHECK("S5-U13", ok,
                   "an object 110 mm to the right cuts clear_R but not clear_L");
    }

    /* -----------------------------------------------------------------
     * IF-03 packing (S5-U14..S5-U15)
     * ----------------------------------------------------------------- */

    empty_sweep(&scan);
    put_return(&scan, 90U, 200U);
    (void)obs_analyse(&scan, &res);
    {
        obstacle_profile_t const prof = obs_result_to_profile(&res, 2U);

        TEST_CHECK("S5-U14",
                   (drep_obs_distance_cm(prof) == 20U)
                   && (drep_obs_bearing_deg(prof) == 90U)
                   && (drep_obs_seq(prof) == 2U)
                   && !drep_obs_no_echo(prof)
                   && (drep_obs_clear_l_mm(prof) == (uint32_t)DREP_OBS_CLR_MAX_MM),
                   "profile round-trip: 20 cm at 90 degrees, sequence kept");
    }

    {
        /* A fine sweep can reach 0 degrees, which IF-03 cannot carry:
         * drep_obs_pack() must clamp the index and raise OOR. */
        obs_result_t edge = res;

        edge.bearing_deg = 0U;
        obstacle_profile_t const prof = obs_result_to_profile(&edge, 0U);

        TEST_CHECK("S5-U15",
                   (drep_obs_bearing_idx(prof) == 0U) && drep_obs_oor(prof),
                   "a 0 degree bearing clamps to index 0 and flags OOR");
    }

    {
        obstacle_profile_t const none = obs_result_to_profile(NULL, 1U);

        TEST_CHECK("S5-U16",
                   drep_obs_no_echo(none) && (drep_obs_distance_cm(none) == 0U),
                   "an invalid result packs as distance 0 (no echo)");
    }

    /* -----------------------------------------------------------------
     * Bypass decisions (S5-U17..S5-U21)
     * ----------------------------------------------------------------- */

    {
        uint32_t const clear = (uint32_t)DREP_OBS_CLR_MAX_MM;
        uint32_t const tight = (uint32_t)OBS_BYPASS_CLEAR_MIN_MM - 5U;
        uint32_t       side = 0U;

        /* Obstacle to the car's LEFT (bearing 120 > 90), both sides clear. */
        obstacle_profile_t const both_left =
            drep_obs_pack(20, ((int32_t)120 - 30) / 10, 100, (int32_t)clear, (int32_t)clear, 0U);

        TEST_CHECK("S5-U17",
                   obs_choose_side(both_left, &side) && (side == OBS_SIDE_RIGHT),
                   "both sides clear, obstacle on the left: pass on the right");

        /* Obstacle to the car's RIGHT (bearing 60 < 90), both sides clear. */
        obstacle_profile_t const both_right =
            drep_obs_pack(20, ((int32_t)60 - 30) / 10, 100, (int32_t)clear, (int32_t)clear, 0U);

        TEST_CHECK("S5-U18",
                   obs_choose_side(both_right, &side) && (side == OBS_SIDE_LEFT),
                   "both sides clear, obstacle on the right: pass on the left");

        /* Only the left has room. */
        obstacle_profile_t const left_only =
            drep_obs_pack(20, 6, 100, (int32_t)clear, (int32_t)tight, 0U);

        TEST_CHECK("S5-U19",
                   obs_choose_side(left_only, &side) && (side == OBS_SIDE_LEFT),
                   "only the left has OBS_BYPASS_CLEAR_MIN_MM: pass on the left");

        /* Neither side has room: blocked. */
        obstacle_profile_t const boxed =
            drep_obs_pack(20, 6, 100, (int32_t)tight, (int32_t)tight, 0U);

        TEST_CHECK("S5-U20", !obs_choose_side(boxed, &side),
                   "neither side has room: the car reports blocked");
    }

    TEST_CHECK("S5-U21",
               (obs_leg_out_mm(0U) > 0U)
               && (obs_leg_out_mm(300U) > obs_leg_out_mm(100U))
               && (obs_mm_to_ms(0U) == 0U)
               && (obs_mm_to_ms(1000U) > obs_mm_to_ms(500U)),
               "out-leg distance and its duration grow with obstacle width");

    /* -----------------------------------------------------------------
     * Running system (S5-U22..S5-U25)
     * ----------------------------------------------------------------- */

    TEST_CHECK("S5-U22", subsystem5_sensors_ok(),
               "power-up sensor check passed (servo PWM, echo handler, echo idle)");

    TEST_CHECK("S5-U23", obs_scan_is_running() && obs_avoid_is_running(),
               "both subsystem 5 tasks are running");

    TEST_CHECK("S5-U24",
               (obs_avoid_get_state() < OBS_AV_STATE_COUNT)
               && (obs_avoid_get_abort_count() == 0U),
               "avoidance starts in a valid state with no aborts");

    /*
     * Give the scanner time for at least one full coarse sweep: about
     * 2 periods per bearing (settle, then ping) over
     * OBS_COARSE_BEARING_COUNT bearings, plus margin.
     */
    {
        uint32_t const sweep_ms = 3U * (uint32_t)OBS_COARSE_BEARING_COUNT
                                  * (uint32_t)OBS_SERVO_STEP_PERIOD_MS;

        (void)tk_dly_tsk((RELTIM)(2U * sweep_ms));

        uint32_t const sweeps = obs_scan_get_sweep_count();
        uint32_t const echoes = us_echo_count();

        dbg_printf("[S5] after %lu ms: %lu sweeps, %lu echoes, %lu misses",
                   (unsigned long)(2U * sweep_ms), (unsigned long)sweeps,
                   (unsigned long)echoes, (unsigned long)us_miss_count());

        TEST_CHECK("S5-U25", sweeps >= 1U,
                   "the scanner completes a coarse sweep on the 60 ms grid");

        /* Manual set-up: an object about 15 cm in front of the sensor.  See
         * testing/subsystem5/TEST_LOG.md for the placement. */
        TEST_CHECK("S5-U26", echoes >= 1U,
                   "the ultrasonic sensor returned at least one echo");
    }

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
