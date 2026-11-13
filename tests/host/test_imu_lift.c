/*
 *  test_imu_lift.c
 *  Host tests for the lift / overturn detector (app_program/subsystem4/imu_lift.c).
 *  Test ID S4-U21.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Where the expected values come from (never from imu_lift.c's arithmetic):
 *    - thresholds: config/thresholds.h, typed in as numbers (tilt 30 deg,
 *      upward acceleration 0.3 g) and imu_lift.h (5 samples to confirm,
 *      50 level-and-still samples to re-arm);
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
    CHECK(count_events(30, -40, 1040, 500U) == 0U);     /* noise, small offsets */
    CHECK(count_events(259, 0, 966, 100U) == 0U);       /* 15 deg hump, 1 s */
    CHECK(count_events(-259, 0, 966, 100U) == 0U);      /* hump, nose down */
    CHECK(count_events(0, 423, 906, 100U) == 0U);       /* 25 deg roll */
    CHECK(count_events(300, 0, 1000, 100U) == 0U);      /* 0.3 g forward acceleration */
    CHECK(count_events(0, 0, 1250, 100U) == 0U);        /* 0.25 g up: below a_lift */
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

    level_car();
    CHECK(feed(0, 0, 1350, 4U) == IMU_LIFT_NONE);       /* 0.35 g up */
    CHECK(imu_lift_step(&g_lift, 0, 0, 1350, &m) == IMU_LIFT_RAISED);
    CHECK((m.up_accel_mg >= 345) && (m.up_accel_mg <= 355));
    CHECK((m.cos_tilt_x1000 >= 995));
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
    CHECK(count_events(0, 0, 1400, 5U) == 1U);
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
