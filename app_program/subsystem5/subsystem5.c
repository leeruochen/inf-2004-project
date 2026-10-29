/*
 *  subsystem5.c
 *  Subsystem 5 -- obstacle scanning and avoidance: creates its objects and
 *  starts its tasks.
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Stub (Phase 1 skeleton).  See subsystem5.h for the tasks and interfaces.
 */

#include "subsystem5.h"

int32_t subsystem5_init(void)
{
    /* TODO(phase 2, buddy 5): create the Obstacle Scanning cyclic handler
     * (OBS_SERVO_STEP_PERIOD_MS, 60 ms), created stopped and started in
     * subsystem5_start(). */
    return (int32_t)E_OK;
}

int32_t subsystem5_start(void)
{
    /* TODO(phase 2, buddy 5): app_start_task(ObsScan task, TASK_PRI_OBS_SCAN,
     * TASK_STACK_OBS_SCAN_BYTES, "obs_scan") and app_start_task(ObsAvoid task,
     * TASK_PRI_OBS_AVOID, TASK_STACK_OBS_AVOID_BYTES, "obs_avoid"), then start
     * the cyclic handler. */
    return (int32_t)E_OK;
}
