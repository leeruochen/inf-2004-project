/*
 *  test_imu_lift.c
 *  Host tests for the lift / overturn detector (app_program/subsystem4/imu_lift.c).
 *  Test ID S4-U21.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Where the expected values come from (never from imu_lift.c's arithmetic):
 *    - thresholds: config/thresholds.h, typed in as numbers (tilt 30 deg,
 *      upward speed 0.10 m/s) and imu_lift.h (5 samples to confirm,
 *      50 level-and-still samples to re-arm, speed leak 1/32 per sample);
 *    - motion profiles: a smooth lift of height h in time T peaks at about
 *      5.8 h / T^2 (minimum-jerk), so 10 cm in 0.8 s is ~0.09 g up then
 *      ~0.09 g down; climbing a 2 cm hump at 25 cm/s is ~0.05 m/s upward;
 *    - S4-U18 on the car (13 Nov 2026): a gentle level lift was MISSED by the
 *      old 0.3 g / 50 ms acceleration rule;
 *    - vectors: gravity of 1000 mg rotated by the stated angle,
 *      x = 1000 sin(a), z = 1000 cos(a), rounded;
 *    - hump pitch: up to ~15 deg on the car (S4 test run, 13 Nov 2026).
 */

#include <stdint.h>
#include <stdio.h>

#include "app_program/subsystem4/imu_lift.h"
#include "test_check.h"

static imu_lift_t g_lift;

/* Feed the same sample n times; return the first event, or NONE. */
static imu_lift_event_t feed(int32_t x, int32_t y, int32_t z, uint32_t n)
{
    imu_lift_event_t first = IMU_LIFT_NONE;

    for (uint32_t i = 0U; i < n; i++)
    {
        imu_lift_event_t const e = imu_lift_step(&g_lift, x, y, z, NULL);

        if ((first == IMU_LIFT_NONE) && (e != IMU_LIFT_NONE))
        {
            first = e;
        }
    }

    return first;
}

/* Count every event over n samples. */
static uint32_t count_events(int32_t x, int32_t y, int32_t z, uint32_t n)
{
    uint32_t c = 0U;

    for (uint32_t i = 0U; i < n; i++)
    {
        c += (imu_lift_step(&g_lift, x, y, z, NULL) != IMU_LIFT_NONE) ? 1U : 0U;
    }

    return c;
}

static void level_car(void)
{
    imu_lift_init(&g_lift);
    imu_lift_set_up(&g_lift, 0, 0, 1000);
}

static void test_off_until_reference(void)
{
    imu_lift_init(&g_lift);
    CHECK(!g_lift.armed);
    CHECK(feed(0, 0, -1000, 20U) == IMU_LIFT_NONE);     /* upside down, but no reference */

    imu_lift_set_up(&g_lift, 0, 0, 0);                   /* zero vector: stays off */
    CHECK(!g_lift.armed);
    CHECK(feed(0, 0, -1000, 20U) == IMU_LIFT_NONE);
}

static void test_no_false_events(void)
{
    level_car();
    CHECK(g_lift.armed);
    CHECK(count_events(0, 0, 1000, 1000U) == 0U);       /* 10 s at rest */
    CHECK(count_events(30, -40, 1000, 500U) == 0U);     /* sideways offsets */
    CHECK(count_events(259, 0, 966, 100U) == 0U);       /* 15 deg hump, 1 s */
    CHECK(count_events(-259, 0, 966, 100U) == 0U);      /* hump, nose down */
    CHECK(count_events(0, 423, 906, 100U) == 0U);       /* 25 deg roll */
    CHECK(count_events(300, 0, 1000, 100U) == 0U);      /* 0.3 g forward acceleration */
    CHECK(count_events(0, 0, 1020, 1000U) == 0U);       /* 20 mg offset for 10 s: settles ~63 mm/s */
    level_car();
    CHECK(count_events(0, 0, 1030, 1000U) == 0U);       /* 30 mg: settles ~94 mm/s, the limit of
                                                           what the leak tolerates (40 mg would
                                                           read as a slow lift) */
    level_car();
    for (uint32_t i = 0U; i < 20U; i++)                 /* 20 humps: +-0.05 g, 100 ms each way */
    {
        CHECK(count_events(0, 0, 1050, 10U) == 0U);
        CHECK(count_events(0, 0, 950, 10U) == 0U);
        CHECK(count_events(0, 0, 1000, 30U) == 0U);
    }
    for (uint32_t i = 0U; i < 20U; i++)                 /* 20 jolts: +-0.5 g, 20 ms each way */
    {
        CHECK(count_events(0, 0, 1500, 2U) == 0U);
        CHECK(count_events(0, 0, 500, 2U) == 0U);
        CHECK(count_events(0, 0, 1000, 30U) == 0U);
    }
    CHECK(g_lift.armed);
}

static void test_tilt_confirms_after_five(void)
{
    imu_lift_meas_t m;

    level_car();
    CHECK(feed(574, 0, 819, 4U) == IMU_LIFT_NONE);      /* 35 deg, 40 ms */
    CHECK(imu_lift_step(&g_lift, 574, 0, 819, &m) == IMU_LIFT_TILTED);
    CHECK((m.cos_tilt_x1000 >= 815) && (m.cos_tilt_x1000 <= 823));   /* cos 35 = 0.819 */
    CHECK(!g_lift.armed);

    /* A short wobble resets the count. */
    level_car();
    CHECK(feed(0, 574, 819, 4U) == IMU_LIFT_NONE);
    CHECK(feed(0, 0, 1000, 1U) == IMU_LIFT_NONE);
    CHECK(feed(0, 574, 819, 4U) == IMU_LIFT_NONE);
    CHECK(feed(0, 574, 819, 1U) == IMU_LIFT_TILTED);
}

static void test_raised(void)
{
    imu_lift_meas_t m;

    /* The S4-U18 case: smooth level lift, 10 cm in 0.8 s (+0.09 g 0.4 s,
     * then -0.09 g 0.4 s).  Never above 0.1 g, so the old rule missed it. */
    level_car();
    CHECK(feed(0, 0, 1090, 40U) == IMU_LIFT_RAISED);
    CHECK(!g_lift.armed);
    CHECK(feed(0, 0, 910, 40U) == IMU_LIFT_NONE);       /* one event per lift */

    /* Speed is reported in mm/s: 100 mg for 10 samples, leak 1/32. */
    level_car();
    for (uint32_t i = 0U; i < 9U; i++)
    {
        (void)imu_lift_step(&g_lift, 0, 0, 1100, NULL);
    }
    (void)imu_lift_step(&g_lift, 0, 0, 1100, &m);
    CHECK((m.v_mmps >= 85) && (m.v_mmps <= 90));        /* 98 mm/s ideal, ~87 after the leak */
    CHECK((m.up_accel_mg >= 95) && (m.up_accel_mg <= 105));
    CHECK(m.cos_tilt_x1000 >= 995);

    /* A quick lift: 10 cm in 0.4 s (+0.36 g 0.2 s). */
    level_car();
    CHECK(feed(0, 0, 1360, 20U) == IMU_LIFT_RAISED);

    /* Too slow to see: +0.03 g for 1 s settles below 0.1 m/s (only a tilt
     * past 30 deg would catch this lift). */
    level_car();
    CHECK(feed(0, 0, 1030, 100U) == IMU_LIFT_NONE);

    /* Peaks for the tuning log, reset after reading. */
    level_car();
    (void)feed(0, 0, 1090, 3U);
    (void)feed(0, 500, 866, 1U);                        /* 30 deg roll, one sample */
    imu_lift_take_peaks(&g_lift, &m);
    CHECK(m.v_mmps > 0);
    CHECK(m.up_accel_mg >= 85);
    CHECK((m.cos_tilt_x1000 >= 862) && (m.cos_tilt_x1000 <= 870));
    imu_lift_take_peaks(&g_lift, &m);
    CHECK((m.v_mmps == 0) && (m.up_accel_mg == 0) && (m.cos_tilt_x1000 == 1000));
}

static void test_overturned(void)
{
    imu_lift_meas_t m;

    level_car();
    CHECK(feed(1000, 0, 0, 4U) == IMU_LIFT_NONE);       /* on its side */
    CHECK(feed(1000, 0, 0, 1U) == IMU_LIFT_TILTED);

    level_car();
    CHECK(feed(0, 0, -1000, 4U) == IMU_LIFT_NONE);      /* upside down */
    CHECK(imu_lift_step(&g_lift, 0, 0, -1000, &m) == IMU_LIFT_TILTED);
    CHECK(m.cos_tilt_x1000 < 0);

    level_car();
    CHECK(feed(0, 0, 0, 5U) == IMU_LIFT_TILTED);        /* free fall */
}

static void test_one_event_then_rearm(void)
{
    level_car();
    CHECK(count_events(0, 707, 707, 300U) == 1U);       /* 45 deg for 3 s: one event */
    CHECK(count_events(0, 0, 1000, 49U) == 0U);         /* put down: 0.49 s */
    CHECK(!g_lift.armed);
    CHECK(count_events(0, 0, 1000, 1U) == 0U);          /* 0.5 s level and still */
    CHECK(g_lift.armed);
    CHECK(count_events(0, 707, 707, 5U) == 1U);         /* next pick-up reports again */

    /* Held level in the air but still moving up and down: stays disarmed. */
    level_car();
    CHECK(count_events(0, 0, 1400, 10U) == 1U);
    for (uint32_t i = 0U; i < 10U; i++)
    {
        CHECK(count_events(0, 0, 1000, 40U) == 0U);
        CHECK(count_events(0, 0, 1200, 1U) == 0U);      /* 0.2 g bump breaks "still" */
    }
    CHECK(!g_lift.armed);
}

static void test_board_mounting(void)
{
    /* Board upside down: "up" reads -Z. */
    imu_lift_init(&g_lift);
    imu_lift_set_up(&g_lift, 0, 0, -1000);
    CHECK(count_events(0, 0, -1000, 200U) == 0U);
    CHECK(feed(0, 0, 1000, 5U) == IMU_LIFT_TILTED);

    /* Board mounted 5 deg nose up (the car's measured offset): flat is fine,
     * 35 deg further is not. */
    imu_lift_init(&g_lift);
    imu_lift_set_up(&g_lift, 87, 0, 996);
    CHECK(count_events(87, 0, 996, 200U) == 0U);
    CHECK(feed(643, 0, 766, 5U) == IMU_LIFT_TILTED);    /* 40 deg from board level */
}

int main(void)
{
    test_off_until_reference();
    test_no_false_events();
    test_tilt_confirms_after_five();
    test_raised();
    test_overturned();
    test_one_event_then_rearm();
    test_board_mounting();

    return TEST_RESULT("imu_lift (S4-U21)");
}
