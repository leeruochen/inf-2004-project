/*
 *  subsystem4.c
 *  Subsystem 4 -- IMU processing and safety: creates its objects and starts
 *  its tasks.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  task_safety.c calls motion_emergency_brake() ONLY through
 *  ../subsystem2/subsystem2.h; never drv_motor.* or PWM directly.
 *  Safety never takes MTX_MOTION.
 */

#include "subsystem4.h"
#include "task_imu.h"
#include "task_safety.h"

static bool s_imu_found = false;

/* ---------------------------------------------------------------------------
 * Public accessors (thin wrappers around task_imu accessors)
 * ------------------------------------------------------------------------- */

int32_t subsystem4_get_pitch_half_deg(void)
{
    return imu_get_pitch_half_deg();
}

uint32_t subsystem4_get_accel_mag_mg(void)
{
    return imu_get_accel_mag_mg();
}

bool subsystem4_imu_found(void)
{
    return s_imu_found;
}

/* ---------------------------------------------------------------------------
 * Two-phase start (app.h)
 * ------------------------------------------------------------------------- */

int32_t subsystem4_init(void)
{
    s_imu_found = imu_task_init();

    if (!s_imu_found)
    {
        dbg_puts("[S4] IMU not found on I2C1");
    }
    else
    {
        dbg_puts("[S4] IMU init OK");
    }

    return (int32_t)E_OK;
}

int32_t subsystem4_start(void)
{
    int32_t const safety_id = app_start_task(safety_task_entry,
                                             TASK_PRI_SAFETY,
                                             TASK_STACK_SAFETY_BYTES,
                                             "safety");

    if (safety_id < 0)
    {
        dbg_put_err("S4 safety start", safety_id);
        return safety_id;
    }

    int32_t const imu_id = app_start_task(imu_task_entry,
                                          TASK_PRI_IMU,
                                          TASK_STACK_IMU_BYTES,
                                          "imu");

    if (imu_id < 0)
    {
        dbg_put_err("S4 imu start", imu_id);
        return imu_id;
    }

    dbg_puts("[S4] tasks started");
    return (int32_t)E_OK;
}
