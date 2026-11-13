/*
 *  imu_lift.c
 *  Subsystem 4 -- lift / overturn detector (pure C).  See imu_lift.h.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 */

#include "imu_lift.h"

#include <stddef.h>

/* Integer square root (floor), for vector lengths in mg. */
static uint32_t lift_isqrt(uint64_t v)
{
    uint64_t r   = 0U;
    uint64_t bit = (uint64_t)1U << 62U;

    while (bit > v)
    {
        bit >>= 2U;
    }

    while (bit != 0U)
    {
        if (v >= (r + bit))
        {
            v -= r + bit;
            r  = (r >> 1U) + bit;
        }
        else
        {
            r >>= 1U;
        }
        bit >>= 2U;
    }

    return (uint32_t)r;
}

static void reset_peaks(imu_lift_t *p_lift)
{
    p_lift->peak_v_mmps   = 0;
    p_lift->peak_accel_mg = 0;
    p_lift->min_cos_x1000 = 1000;
}

/* mg x 10 ms -> mm/s: 1 mg = 9.80665 mm/s^2, x 0.01 s. */
static int32_t v_to_mmps(int32_t v_sum)
{
    return (int32_t)(((int64_t)v_sum * 981) / 10000);
}

static uint32_t lift_mag(int32_t x, int32_t y, int32_t z)
{
    return lift_isqrt((uint64_t)(((int64_t)x * x) + ((int64_t)y * y) + ((int64_t)z * z)));
}

void imu_lift_init(imu_lift_t *p_lift)
{
    p_lift->up_x    = 0;
    p_lift->up_y    = 0;
    p_lift->up_z    = 0;
    p_lift->up_mag  = 0U;
    p_lift->armed   = false;
    p_lift->confirm = 0U;
    p_lift->settle  = 0U;
    p_lift->v_sum   = 0;
    reset_peaks(p_lift);
}

void imu_lift_set_up(imu_lift_t *p_lift, int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    p_lift->up_x    = x_mg;
    p_lift->up_y    = y_mg;
    p_lift->up_z    = z_mg;
    p_lift->up_mag  = lift_mag(x_mg, y_mg, z_mg);
    p_lift->armed   = (p_lift->up_mag != 0U);
    p_lift->confirm = 0U;
    p_lift->settle  = 0U;
    p_lift->v_sum   = 0;
}

void imu_lift_take_peaks(imu_lift_t *p_lift, imu_lift_meas_t *p_peak)
{
    p_peak->v_mmps         = p_lift->peak_v_mmps;
    p_peak->up_accel_mg    = p_lift->peak_accel_mg;
    p_peak->cos_tilt_x1000 = p_lift->min_cos_x1000;
    reset_peaks(p_lift);
}

imu_lift_event_t imu_lift_step(imu_lift_t *p_lift, int32_t x_mg, int32_t y_mg, int32_t z_mg,
                               imu_lift_meas_t *p_meas)
{
    if (p_lift->up_mag == 0U)
    {
        return IMU_LIFT_NONE;
    }

    int64_t const  dot   = ((int64_t)x_mg * p_lift->up_x) + ((int64_t)y_mg * p_lift->up_y)
                         + ((int64_t)z_mg * p_lift->up_z);
    uint32_t const f_mag = lift_mag(x_mg, y_mg, z_mg);
    int64_t const  den   = (int64_t)f_mag * (int64_t)p_lift->up_mag;

    /* No acceleration at all (free fall) counts as fully tilted. */
    int32_t const cos_x1000 = (den > 0) ? (int32_t)((dot * 1000) / den) : 0;
    int32_t const up_accel  = (int32_t)(dot / (int64_t)p_lift->up_mag) - (int32_t)p_lift->up_mag;

    /* Vertical speed: leaky integral of the acceleration along up. */
    p_lift->v_sum += up_accel - (p_lift->v_sum / IMU_LIFT_V_LEAK_DIV);

    int32_t const v_mmps = v_to_mmps(p_lift->v_sum);

    if (p_meas != NULL)
    {
        p_meas->cos_tilt_x1000 = cos_x1000;
        p_meas->up_accel_mg    = up_accel;
        p_meas->v_mmps         = v_mmps;
    }

    p_lift->peak_v_mmps   = (v_mmps > p_lift->peak_v_mmps) ? v_mmps : p_lift->peak_v_mmps;
    p_lift->peak_accel_mg = (up_accel > p_lift->peak_accel_mg) ? up_accel : p_lift->peak_accel_mg;
    p_lift->min_cos_x1000 = (cos_x1000 < p_lift->min_cos_x1000) ? cos_x1000 : p_lift->min_cos_x1000;

    bool const tilted = cos_x1000 < (int32_t)IMU_LIFT_COS_TILT_X1000;
    bool const raised = v_mmps >= (int32_t)IMU_LIFT_V_MMPS;

    if (p_lift->armed)
    {
        if (!tilted && !raised)
        {
            p_lift->confirm = 0U;
            return IMU_LIFT_NONE;
        }

        p_lift->confirm++;

        if (p_lift->confirm < IMU_LIFT_CONFIRM_SAMPLES)
        {
            return IMU_LIFT_NONE;
        }

        p_lift->armed   = false;
        p_lift->confirm = 0U;
        p_lift->settle  = 0U;
        return tilted ? IMU_LIFT_TILTED : IMU_LIFT_RAISED;
    }

    /* Disarmed: wait until the car stands level and still again. */
    bool const level = cos_x1000 >= IMU_LIFT_REARM_COS_X1000;
    bool const still = (up_accel <= IMU_LIFT_REARM_ACCEL_MG) && (up_accel >= -IMU_LIFT_REARM_ACCEL_MG);

    if (level && still)
    {
        p_lift->settle++;

        if (p_lift->settle >= IMU_LIFT_REARM_SAMPLES)
        {
            p_lift->armed  = true;
            p_lift->settle = 0U;
            p_lift->v_sum  = 0;     /* standing still: speed is 0 */
        }
    }
    else
    {
        p_lift->settle = 0U;
    }

    return IMU_LIFT_NONE;
}
