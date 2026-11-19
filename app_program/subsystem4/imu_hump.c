/*
 *  imu_hump.c
 *  Subsystem 4 -- hump detector and height estimator (pure C, imu_hump.h).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 */

#include "imu_hump.h"

#include <stddef.h>

#if (IMU_HUMP_WHEELBASE_MM == 0U)
#error "IMU_HUMP_WHEELBASE_MM (config/thresholds.h) must be measured on the car"
#endif

/* sin(0..90 deg) x 10000, 1 deg steps; half-degrees are interpolated. */
static int16_t const s_sin_x10000[91] =
{
    0, 175, 349, 523, 698, 872, 1045, 1219, 1392, 1564,
    1736, 1908, 2079, 2250, 2419, 2588, 2756, 2924, 3090, 3256,
    3420, 3584, 3746, 3907, 4067, 4226, 4384, 4540, 4695, 4848,
    5000, 5150, 5299, 5446, 5592, 5736, 5878, 6018, 6157, 6293,
    6428, 6561, 6691, 6820, 6947, 7071, 7193, 7314, 7431, 7547,
    7660, 7771, 7880, 7986, 8090, 8192, 8290, 8387, 8480, 8572,
    8660, 8746, 8829, 8910, 8988, 9063, 9135, 9205, 9272, 9336,
    9397, 9455, 9511, 9563, 9613, 9659, 9703, 9744, 9781, 9816,
    9848, 9877, 9903, 9925, 9945, 9962, 9976, 9986, 9994, 9998,
    10000
};

/* Largest wheel travel accepted in one sample (a glitch guard): 50 mm,
 * 25x the top speed of 0.5 m/s x 10 ms. */
#define IMU_HUMP_MAX_DS_UM          (50000)
/* "Back down": the climb integral is within this of the floor (or a
 * quarter of its peak, whichever is larger). */
#define IMU_HUMP_DOWN_UM            (3000)
/* Level travel that ends a hump even if the car never came back down
 * (a step up, not a hump), mm. */
#define IMU_HUMP_END_MAX_MM         (1000)

int32_t imu_hump_sin_x10000(int32_t half_deg)
{
    bool    neg = (half_deg < 0);
    int32_t a   = neg ? -half_deg : half_deg;

    a %= 720;                   /* 360 deg */
    if (a > 360)
    {
        a   = 720 - a;          /* sin(360 - x) = -sin(x) */
        neg = !neg;
    }
    if (a > 180)
    {
        a = 360 - a;            /* sin(180 - x) = sin(x) */
    }

    int32_t const deg = a / 2;
    int32_t       s   = s_sin_x10000[deg];

    if (((a & 1) != 0) && (deg < 90))
    {
        s = (s + s_sin_x10000[deg + 1]) / 2;
    }

    return neg ? -s : s;
}

int32_t imu_hump_counts_to_um(int32_t counts)
{
    /* um = counts x WHEEL_CIRC_MM x 1000 / (ENC_QUAD_MULT x ENC_CPR); counts
     * are clamped first so the product stays inside int32. */
    int32_t const per_rev = (int32_t)(ENC_QUAD_MULT * ENC_CPR);
    int32_t const max_cnt = (int32_t)((IMU_HUMP_MAX_DS_UM * (int64_t)per_rev)
                                      / ((int32_t)WHEEL_CIRC_MM * 1000));
    int32_t       c       = counts;

    c = (c > max_cnt) ? max_cnt : c;
    c = (c < -max_cnt) ? -max_cnt : c;

    return (c * (int32_t)WHEEL_CIRC_MM * 1000) / per_rev;
}

static void clear_episode(imu_hump_t *p_hump)
{
    p_hump->active        = false;
    p_hump->turned        = false;
    p_hump->confirm       = 0U;
    p_hump->level_n       = 0U;
    p_hump->level_um      = 0;
    p_hump->climb_um      = 0;
    p_hump->climb_peak_um = 0;
    p_hump->peak_pitch    = 0;
    p_hump->spike_pitch   = 0;
}

void imu_hump_init(imu_hump_t *p_hump)
{
    clear_episode(p_hump);
    p_hump->hold_n = 0U;
    p_hump->hold_i = 0U;
}

/* Push one pitch into the hold window and return the pitch held over the
 * whole window: its smallest |value| if every sample has the same sign,
 * else 0 (also 0 until the window is full). */
static int32_t held_pitch(imu_hump_t *p_hump, int32_t pitch_half_deg)
{
    int32_t p = pitch_half_deg;

    p = (p > 32767) ? 32767 : p;
    p = (p < -32767) ? -32767 : p;
    p_hump->hold[p_hump->hold_i] = (int16_t)p;
    p_hump->hold_i = (p_hump->hold_i + 1U) % IMU_HUMP_HOLD_SAMPLES;
    if (p_hump->hold_n < IMU_HUMP_HOLD_SAMPLES)
    {
        p_hump->hold_n++;
        return 0;
    }

    int32_t lo = p_hump->hold[0];
    int32_t hi = p_hump->hold[0];

    for (uint32_t i = 1U; i < IMU_HUMP_HOLD_SAMPLES; i++)
    {
        lo = (p_hump->hold[i] < lo) ? p_hump->hold[i] : lo;
        hi = (p_hump->hold[i] > hi) ? p_hump->hold[i] : hi;
    }

    if (lo > 0)
    {
        return lo;
    }
    if (hi < 0)
    {
        return hi;
    }
    return 0;
}

static void make_report(imu_hump_t const *p_hump, imu_hump_report_t *p_report)
{
    int32_t const abs_peak = (p_hump->peak_pitch >= 0) ? p_hump->peak_pitch : -p_hump->peak_pitch;
    int32_t const tilt_mm  = ((int32_t)IMU_HUMP_WHEELBASE_MM * imu_hump_sin_x10000(abs_peak) + 5000)
                             / 10000;
    int32_t const climb_mm = (p_hump->climb_peak_um + 500) / 1000;

    p_report->tilt_mm        = tilt_mm;
    p_report->climb_mm       = climb_mm;
    p_report->height_mm      = (tilt_mm > climb_mm) ? tilt_mm : climb_mm;
    p_report->pitch_half_deg = p_hump->peak_pitch;
    p_report->spike_half_deg = p_hump->spike_pitch;
}

imu_hump_event_t imu_hump_step(imu_hump_t *p_hump, int32_t pitch_half_deg, int32_t ds_um,
                               bool turning, bool speed_change, imu_hump_report_t *p_report)
{
    /* While the speed changes the pitch reads the acceleration, not the
     * ground: the sample counts as level (it can end a hump, never start or
     * measure one) and the hold window restarts. */
    int32_t const pitch     = speed_change ? 0 : pitch_half_deg;
    int32_t const abs_pitch = (pitch >= 0) ? pitch : -pitch;
    int32_t       held;
    int32_t       abs_held;
    int32_t       ds        = ds_um;

    if (speed_change)
    {
        p_hump->hold_n = 0U;
    }
    held     = held_pitch(p_hump, pitch);
    abs_held = (held >= 0) ? held : -held;

    ds = (ds > IMU_HUMP_MAX_DS_UM) ? IMU_HUMP_MAX_DS_UM : ds;
    ds = (ds < -IMU_HUMP_MAX_DS_UM) ? -IMU_HUMP_MAX_DS_UM : ds;

    if (!p_hump->active)
    {
        if ((abs_pitch < IMU_HUMP_EXIT_HALF_DEG) || turning)
        {
            clear_episode(p_hump);      /* level, or a turn: no candidate */
            return IMU_HUMP_NONE;
        }
    }
    else if (turning)
    {
        p_hump->turned = true;          /* the pitch is not trustworthy now */
    }

    /* Inside a hump, or a candidate above the exit pitch: integrate the
     * climb and keep the peak pitch.  Not while turning, and not while level
     * (a dead band, so pitch noise on a flat top does not add up). */
    if (!turning && (abs_pitch >= IMU_HUMP_EXIT_HALF_DEG))
    {
        p_hump->climb_um += (imu_hump_sin_x10000(pitch) * (ds / 10)) / 1000;
        if (p_hump->climb_um > p_hump->climb_peak_um)
        {
            p_hump->climb_peak_um = p_hump->climb_um;
        }

        int32_t const abs_peak  = (p_hump->peak_pitch >= 0) ? p_hump->peak_pitch : -p_hump->peak_pitch;
        int32_t const abs_spike = (p_hump->spike_pitch >= 0) ? p_hump->spike_pitch : -p_hump->spike_pitch;

        if (abs_held > abs_peak)
        {
            p_hump->peak_pitch = held;
        }
        if (abs_pitch > abs_spike)
        {
            p_hump->spike_pitch = pitch;
        }
    }

    if (!p_hump->active)
    {
        if (abs_pitch < IMU_HUMP_ENTRY_HALF_DEG)
        {
            p_hump->confirm = 0U;       /* between exit and entry: wait */
            return IMU_HUMP_NONE;
        }

        p_hump->confirm++;
        if (p_hump->confirm < IMU_HUMP_CONFIRM_SAMPLES)
        {
            return IMU_HUMP_NONE;
        }

        p_hump->active   = true;
        p_hump->level_n  = 0U;
        p_hump->level_um = 0;
        return IMU_HUMP_START;
    }

    if (abs_pitch >= IMU_HUMP_EXIT_HALF_DEG)
    {
        p_hump->level_n  = 0U;          /* still on the hump */
        p_hump->level_um = 0;
        return IMU_HUMP_NONE;
    }

    /* Level again.  The hump ends:
     *   - after a wheelbase of level travel (so the rear wheels of a short
     *     hump still belong to it) once the car is back down (the climb has
     *     dropped to a quarter of its peak, or 3 mm): level travel on the
     *     flat top of a long hump does not end it;
     *   - after IMU_HUMP_END_MAX_MM of level travel in any case;
     *   - after a pause standing still (level_n counts only samples without
     *     wheel travel, so a slow drive over a short hump is still one). */
    p_hump->level_n   = (ds == 0) ? (p_hump->level_n + 1U) : 0U;
    p_hump->level_um += (ds >= 0) ? ds : -ds;

    int32_t const down_um = (p_hump->climb_peak_um / 4 > IMU_HUMP_DOWN_UM)
                            ? (p_hump->climb_peak_um / 4) : IMU_HUMP_DOWN_UM;
    bool const    back_down = (p_hump->climb_um <= down_um);
    bool const    ended     = ((p_hump->level_um >= ((int32_t)IMU_HUMP_WHEELBASE_MM * 1000)) && back_down)
                              || (p_hump->level_um >= (IMU_HUMP_END_MAX_MM * 1000))
                              || (p_hump->level_n >= IMU_HUMP_END_SAMPLES);

    if (!ended)
    {
        return IMU_HUMP_NONE;
    }

    imu_hump_event_t const ev = p_hump->turned ? IMU_HUMP_DROPPED : IMU_HUMP_END;

    if ((ev == IMU_HUMP_END) && (p_report != NULL))
    {
        make_report(p_hump, p_report);
    }

    clear_episode(p_hump);
    return ev;
}
