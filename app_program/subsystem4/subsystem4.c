/*
 *  subsystem4.c
 *  Subsystem 4 -- IMU processing and safety: creates its objects and starts
 *  its tasks.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Stub (Phase 1 skeleton).  See subsystem4.h for the tasks and interfaces.
 *  task_safety.c (Phase 2) lives in this folder and calls
 *  motion_emergency_brake() ONLY through ../subsystem2/subsystem2.h; never
 *  drv_motor.* or PWM directly.  Safety never takes MTX_MOTION.
 */

#include "subsystem4.h"

int32_t subsystem4_init(void)
{
    /* TODO(phase 2, buddy 4): create the IMU cyclic handler
     * (IMU_SAMPLE_PERIOD_MS, 10 ms), created stopped and started in
     * subsystem4_start(). */
    return (int32_t)E_OK;
}

int32_t subsystem4_start(void)
{
    /* TODO(phase 2, buddy 4): app_start_task(Safety task, TASK_PRI_SAFETY,
     * TASK_STACK_SAFETY_BYTES, "safety") and app_start_task(IMU task,
     * TASK_PRI_IMU, TASK_STACK_IMU_BYTES, "imu"), then start the cyclic
     * handler. */
    return (int32_t)E_OK;
}
