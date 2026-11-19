/*
 *  test_imu_hump.c
 *  Host tests for the hump detector and height estimator
 *  (app_program/subsystem4/imu_hump.c).  Test ID S4-U24.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Where the expected values come from (never from imu_hump.c's arithmetic):
 *    - the car is simulated as two ground contacts a wheelbase apart (the
 *      driven wheels behind, the front contact ahead) rolling over a hump
 *      profile z(x).  The pitch is the angle of the line between them, as
 *      the accelerometer would see it, rounded to half-degrees with a
 *      +-0.5 deg wobble; the encoder distance is the path length of the
 *      driven wheels (libm, double precision);
 *    - the expected height is the hump's real height, and the tolerance is
 *      NFR11 (design.md): +-20 % or +-5 mm, whichever is larger;
 *    - thresholds: config/thresholds.h (5 deg entry, wheelbase), typed in;
 *    - encoder scale: 2464 counts per 207 mm wheel turn (S2-U10/S2-U11);
 *    - sin values: tables of sin(x);
 *    - real traces: hump_traces.h, the car over the 10 mm track hump
 *      (S4-U26, 19 Nov 2026); the expected height is the hump's ruler height.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "app_program/subsystem4/imu_hump.h"
#include "hump_traces.h"
#include "test_check.h"

#define PI              (3.14159265358979)
#define WHEELBASE_MM    (120.0)     /* config/thresholds.h, typed in */
#define DT_S            (0.010)     /* IMU period */

typedef double (*profile_fn)(double x_mm);

static double g_h;          /* hump height, mm */
static double g_w;          /* hump width (or ramp length), mm */
static double g_top;        /* flat top length, mm */

/* Flat floor. */
static double flat(double x)
{
    (void)x;
    return 0.0;
}

/* Smooth bump (raised cosine), height g_h, width g_w, starting at x = 0. */
static double bump(double x)
{
    if ((x <= 0.0) || (x >= g_w))
    {
        return 0.0;
    }
    return g_h * 0.5 * (1.0 - cos(2.0 * PI * x / g_w));
}

/* Trapezoid: ramp g_w up, flat top g_top, ramp g_w down. */
static double trapezoid(double x)
{
    if ((x <= 0.0) || (x >= (2.0 * g_w + g_top)))
    {
        return 0.0;
    }
    if (x < g_w)
    {
        return g_h * x / g_w;
    }
    if (x < (g_w + g_top))
    {
        return g_h;
    }
    return g_h * (2.0 * g_w + g_top - x) / g_w;
}

/* Front contact x for the rear contact at xr: the point on the profile one
 * wheelbase away (a few fixed-point steps). */
static double front_x(profile_fn z, double xr)
{
    double xf = xr + WHEELBASE_MM;

    for (int i = 0; i < 20; i++)
    {
        double const dz = z(xf) - z(xr);
        xf = xr + sqrt((WHEELBASE_MM * WHEELBASE_MM) - (dz * dz));
    }
    return xf;
}

static int32_t pitch_half_deg(profile_fn z, double xr, uint32_t n)
{
    double const xf    = front_x(z, xr);
    double const angle = atan2(z(xf) - z(xr), xf - xr) * 180.0 / PI;
    double const wob   = ((n % 4U) == 0U) ? 0.5 : (((n % 4U) == 2U) ? -0.5 : 0.0);
    return (int32_t)lround((angle + wob) * 2.0);
}

typedef struct
{
    uint32_t starts;
    uint32_t ends;
    uint32_t dropped;
    imu_hump_report_t last;
} drive_result_t;

/* Drive the rear contact from x0 to x1 at v mm/s (negative v = reverse),
 * then stand still for 1 s.  Encoder distance arrives every 20 ms like
 * Motion's published state (every other IMU sample sees 0). */
static drive_result_t drive(imu_hump_t *p, profile_fn z, double x0, double x1, double v)
{
    drive_result_t r = { 0 };
    double         x = x0;
    double         pending = 0.0;
    uint32_t       n = 0U;
    double const   step = v * DT_S;
    uint32_t const samples = (uint32_t)(fabs((x1 - x0) / step)) + 100U;

    for (uint32_t i = 0U; i < samples; i++)
    {
        double ds = 0.0;

        if (i < (samples - 100U))
        {
            double const xn = x + step;
            double const dz = z(xn) - z(x);
            pending += copysign(sqrt((step * step) + (dz * dz)), step);
            x = xn;
        }
        if ((n % 2U) == 1U)
        {
            ds = pending;
            pending = 0.0;
        }

        imu_hump_report_t rep;
        imu_hump_event_t const e = imu_hump_step(p, pitch_half_deg(z, x, n),
                                                 (int32_t)lround(ds * 1000.0), false, false, &rep);
        n++;
        r.starts  += (e == IMU_HUMP_START) ? 1U : 0U;
        r.ends    += (e == IMU_HUMP_END) ? 1U : 0U;
        r.dropped += (e == IMU_HUMP_DROPPED) ? 1U : 0U;
        if (e == IMU_HUMP_END)
        {
            r.last = rep;
        }
    }
    return r;
}

/* NFR11: within +-20 % or +-5 mm, whichever is larger. */
static int within_nfr11(int32_t got, double expect)
{
    double const tol = (0.2 * expect > 5.0) ? (0.2 * expect) : 5.0;

    return fabs((double)got - expect) <= tol;
}

static void test_sin_table(void)
{
    CHECK(imu_hump_sin_x10000(0) == 0);
    CHECK(imu_hump_sin_x10000(60) == 5000);         /* sin 30 = 0.5 */
    CHECK(imu_hump_sin_x10000(-60) == -5000);
    CHECK(imu_hump_sin_x10000(180) == 10000);       /* sin 90 */
    CHECK(imu_hump_sin_x10000(19) == 1650);         /* sin 9.5 = 0.16505 (interp. of 1564/1736) */
    CHECK(imu_hump_sin_x10000(300) == 5000);        /* sin 150 */
    CHECK(imu_hump_sin_x10000(360) == 0);           /* sin 180 */
    CHECK(imu_hump_sin_x10000(420) == -5000);       /* sin 210 */
    CHECK(imu_hump_sin_x10000(-300) == -5000);      /* sin -150 */
    CHECK(imu_hump_sin_x10000(10) == 872);          /* sin 5 = 0.0872 */
}

static void test_counts_to_um(void)
{
    CHECK(imu_hump_counts_to_um(0) == 0);
    CHECK(imu_hump_counts_to_um(308) == 25875);     /* 1/8 wheel turn: 207/8 mm */
    CHECK(imu_hump_counts_to_um(-308) == -25875);
    CHECK(imu_hump_counts_to_um(12) == 1008);       /* 12 x 207/2464 mm = 1.008 mm */
    CHECK((imu_hump_counts_to_um(100000) > 49900)   /* glitch: clamped to 50 mm */
          && (imu_hump_counts_to_um(100000) <= 50000));
    CHECK((imu_hump_counts_to_um(-100000) < -49900)
          && (imu_hump_counts_to_um(-100000) >= -50000));
}

static void test_flat_no_hump(void)
{
    imu_hump_t h;

    imu_hump_init(&h);
    drive_result_t const r = drive(&h, flat, 0.0, 2000.0, 250.0);
    CHECK((r.starts == 0U) && (r.ends == 0U));
}

static void test_static_book(void)
{
    /* S4-U15/U25 on the desk: a 20 mm object under the front wheels, car not
     * moving.  pitch = asin(20 / 120) = 9.59 deg. */
    imu_hump_t        h;
    imu_hump_report_t rep = { 0 };
    uint32_t          ends = 0U;

    imu_hump_init(&h);
    for (uint32_t i = 0U; i < 200U; i++)            /* 2 s on the book */
    {
        ends += (imu_hump_step(&h, 19, 0, false, false, &rep) == IMU_HUMP_END) ? 1U : 0U;
    }
    CHECK(ends == 0U);
    for (uint32_t i = 0U; i < 49U; i++)             /* book removed, 0.49 s */
    {
        ends += (imu_hump_step(&h, 0, 0, false, false, &rep) == IMU_HUMP_END) ? 1U : 0U;
    }
    CHECK(ends == 0U);
    CHECK(imu_hump_step(&h, 0, 0, false, false, &rep) == IMU_HUMP_END);    /* 0.5 s */
    CHECK(within_nfr11(rep.height_mm, 20.0));
    CHECK((rep.height_mm >= 19) && (rep.height_mm <= 21));
    CHECK(rep.pitch_half_deg == 19);
    CHECK(rep.climb_mm == 0);
}

static void test_short_bump(double h_mm, double w_mm, double v)
{
    imu_hump_t h;

    g_h = h_mm;
    g_w = w_mm;
    imu_hump_init(&h);
    drive_result_t const r = drive(&h, bump, -200.0, w_mm + 300.0, v);
    CHECK(r.ends == 1U);                            /* front and rear: ONE hump */
    CHECK(r.dropped == 0U);
    CHECK(within_nfr11(r.last.height_mm, h_mm));
    CHECK(r.last.height_mm <= (int32_t)(h_mm + 2.0));   /* never reads high */
    if (!within_nfr11(r.last.height_mm, h_mm) || (r.ends != 1U))
    {
        printf("  bump h=%.0f w=%.0f v=%.0f: ends=%u height=%ld tilt=%ld climb=%ld\n", h_mm, w_mm, v,
               (unsigned)r.ends, (long)r.last.height_mm, (long)r.last.tilt_mm, (long)r.last.climb_mm);
    }
}

static void test_long_hump(double h_mm, double ramp_mm, double top_mm, double v)
{
    imu_hump_t h;

    g_h   = h_mm;
    g_w   = ramp_mm;
    g_top = top_mm;
    imu_hump_init(&h);
    drive_result_t const r = drive(&h, trapezoid, -200.0, 2.0 * ramp_mm + top_mm + 300.0, v);
    CHECK(r.ends == 1U);                            /* flat top does not split it */
    CHECK(within_nfr11(r.last.height_mm, h_mm));
    CHECK(r.last.height_mm <= (int32_t)(h_mm + 2.0));   /* never reads high */
    if (!within_nfr11(r.last.height_mm, h_mm) || (r.ends != 1U))
    {
        printf("  trapezoid h=%.0f ramp=%.0f top=%.0f v=%.0f: ends=%u height=%ld tilt=%ld climb=%ld\n",
               h_mm, ramp_mm, top_mm, v, (unsigned)r.ends, (long)r.last.height_mm,
               (long)r.last.tilt_mm, (long)r.last.climb_mm);
    }
}

static void test_shapes(void)
{
    /* Short humps, shorter than the car: the TILT estimate. */
    test_short_bump(20.0, 60.0, 250.0);
    test_short_bump(20.0, 100.0, 250.0);
    test_short_bump(15.0, 80.0, 200.0);
    test_short_bump(30.0, 120.0, 250.0);
    test_short_bump(20.0, 80.0, 100.0);             /* slow: still one hump */

    /* Long humps with a flat top: the CLIMB estimate. */
    test_long_hump(20.0, 150.0, 200.0, 250.0);
    test_long_hump(30.0, 200.0, 300.0, 250.0);
    test_long_hump(20.0, 100.0, 150.0, 200.0);
}

/* Replay a real trace through the estimator, gating speed changes the way
 * the IMU task does (wheel speed step > 20 mm/s between 20 ms periods, held
 * for 20 samples).  Returns the number of reported humps; *p_h = last height. */
static uint32_t replay(int8_t const *p_pitch, int8_t const *p_ds, uint32_t n, int32_t *p_h)
{
    imu_hump_t h;
    int32_t    v_prev = 0;
    uint32_t   hold   = 0U;
    uint32_t   ends   = 0U;

    imu_hump_init(&h);
    *p_h = -1;
    for (uint32_t i = 0U; i < (n + 100U); i++)
    {
        int32_t const pitch = (i < n) ? p_pitch[i] : p_pitch[n - 1U];
        int32_t const ds_um = (i < n) ? ((int32_t)p_ds[i] * 250) : 0;

        if ((i % 2U) == 0U)             /* a Motion period: travel in 20 ms */
        {
            int32_t const v = ds_um / 20;
            if (((v - v_prev) > 20) || ((v - v_prev) < -20))
            {
                hold = 20U;
            }
            v_prev = v;
        }

        imu_hump_report_t      rep;
        imu_hump_event_t const e = imu_hump_step(&h, pitch, ds_um, false, hold > 0U, &rep);

        hold = (hold > 0U) ? (hold - 1U) : 0U;
        if (e == IMU_HUMP_END)
        {
            ends++;
            *p_h = rep.height_mm;
        }
    }
    return ends;
}

static void test_real_traces(void)
{
    /* The 10 mm track hump, two drives.  Without the speed gate the car
     * reported 13-14 mm from braking and 33-36 mm from pulling away. */
    int32_t        h1;
    int32_t        h2;
    uint32_t const n1 = replay(t1607_p, t1607_d, (uint32_t)sizeof(t1607_p), &h1);
    uint32_t const n2 = replay(t1611_p, t1611_d, (uint32_t)sizeof(t1611_p), &h2);

    CHECK(n1 == 1U);
    CHECK(n2 == 1U);
    CHECK(within_nfr11(h1, 10.0));
    CHECK(within_nfr11(h2, 10.0));
    printf("  real traces of the 10 mm hump: %ld mm and %ld mm\n", (long)h1, (long)h2);
}

static void test_speed_change_ignored(void)
{
    /* Pulling away (+13 deg for 0.15 s) and braking (-13 deg) on the flat:
     * no hump while the speed changes. */
    imu_hump_t        h;
    imu_hump_report_t rep;
    uint32_t          ev = 0U;

    imu_hump_init(&h);
    for (uint32_t i = 0U; i < 15U; i++)
    {
        ev += (imu_hump_step(&h, 26, 2000, false, true, &rep) != IMU_HUMP_NONE) ? 1U : 0U;
    }
    for (uint32_t i = 0U; i < 100U; i++)
    {
        ev += (imu_hump_step(&h, 0, 2000, false, false, &rep) != IMU_HUMP_NONE) ? 1U : 0U;
    }
    for (uint32_t i = 0U; i < 15U; i++)
    {
        ev += (imu_hump_step(&h, -26, 500, false, true, &rep) != IMU_HUMP_NONE) ? 1U : 0U;
    }
    for (uint32_t i = 0U; i < 100U; i++)
    {
        ev += (imu_hump_step(&h, 0, 0, false, false, &rep) != IMU_HUMP_NONE) ? 1U : 0U;
    }
    CHECK(ev == 0U);
}

static void test_top_speed_reads_low(void)
{
    /* At top speed (500 mm/s) a short bump passes in less than the 100 ms
     * hold: the height reads low, never high (known limit). */
    imu_hump_t h;

    g_h = 20.0;
    g_w = 80.0;
    imu_hump_init(&h);
    drive_result_t const r = drive(&h, bump, -200.0, 380.0, 500.0);
    CHECK(r.ends == 1U);
    CHECK((r.last.height_mm > 0) && (r.last.height_mm <= 22));
}

static void test_low_humps_ignored(void)
{
    /* 3 mm bump over 100 mm: peak pitch ~1.4 deg (+-0.5 wobble), below the
     * 3 deg entry: no hump. */
    imu_hump_t h;

    g_h = 3.0;
    g_w = 100.0;
    imu_hump_init(&h);
    drive_result_t const r = drive(&h, bump, -200.0, 400.0, 250.0);
    CHECK(r.ends == 0U);
}

static void test_turn_drops(void)
{
    /* In-place turn: pitch reads -11 deg for 0.5 s (S4 log, 13 Nov 2026). */
    imu_hump_t        h;
    imu_hump_report_t rep;
    uint32_t          ev = 0U;

    imu_hump_init(&h);
    for (uint32_t i = 0U; i < 50U; i++)
    {
        ev += (imu_hump_step(&h, -22, 0, true, false, &rep) != IMU_HUMP_NONE) ? 1U : 0U;
    }
    for (uint32_t i = 0U; i < 100U; i++)
    {
        ev += (imu_hump_step(&h, 0, 0, false, false, &rep) != IMU_HUMP_NONE) ? 1U : 0U;
    }
    CHECK(ev == 0U);                                /* a turn never starts a hump */

    /* A hump that a turn interrupts is dropped, not reported. */
    imu_hump_init(&h);
    CHECK(imu_hump_step(&h, 20, 0, false, false, &rep) == IMU_HUMP_START);  /* 1 sample confirms */
    CHECK(imu_hump_step(&h, 20, 0, false, false, &rep) == IMU_HUMP_NONE);
    (void)imu_hump_step(&h, 20, 0, true, false, &rep);
    uint32_t dropped = 0U;
    uint32_t ends    = 0U;
    for (uint32_t i = 0U; i < 100U; i++)
    {
        imu_hump_event_t const e = imu_hump_step(&h, 0, 0, false, false, &rep);
        dropped += (e == IMU_HUMP_DROPPED) ? 1U : 0U;
        ends    += (e == IMU_HUMP_END) ? 1U : 0U;
    }
    CHECK((dropped == 1U) && (ends == 0U));
}

static void test_reverse(void)
{
    /* Reversing over a short bump: the rear climbs first (nose down). */
    imu_hump_t h;

    g_h = 20.0;
    g_w = 80.0;
    imu_hump_init(&h);
    drive_result_t const r = drive(&h, bump, 300.0, -250.0, -250.0);
    CHECK(r.ends == 1U);
    CHECK(within_nfr11(r.last.height_mm, 20.0));
}

static void test_two_humps(void)
{
    /* Two humps 500 mm apart: two reports. */
    imu_hump_t h;

    g_h = 20.0;
    g_w = 80.0;
    imu_hump_init(&h);
    drive_result_t r1 = drive(&h, bump, -200.0, 300.0, 250.0);
    drive_result_t r2 = drive(&h, bump, -200.0, 300.0, 250.0);
    CHECK((r1.ends == 1U) && (r2.ends == 1U));
}

int main(void)
{
    test_sin_table();
    test_counts_to_um();
    test_flat_no_hump();
    test_static_book();
    test_shapes();
    test_real_traces();
    test_speed_change_ignored();
    test_top_speed_reads_low();
    test_low_humps_ignored();
    test_turn_drops();
    test_reverse();
    test_two_humps();

    return TEST_RESULT("imu_hump (S4-U24)");
}
