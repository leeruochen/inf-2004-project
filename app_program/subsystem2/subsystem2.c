/*
 *  subsystem2.c
 *  Subsystem 2 -- motion control and Vehicle Controller: creates its
 *  objects and starts its tasks.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Stub (Phase 1 skeleton).  See subsystem2.h for the tasks and interfaces.
 */

#include "subsystem2.h"
#include "drv_encoder.h"    /* kernel-free driver interface: enc_get_count() */

int32_t subsystem2_init(void)
{
    /* TODO(phase 2, buddy 2): create the Motion Control cyclic handler
     * (MOTION_PERIOD_MS, 20 ms), created stopped and started in
     * subsystem2_start(). */
    return (int32_t)E_OK;
}

int32_t subsystem2_start(void)
{
    /* TODO(phase 2, buddy 2): app_start_task(VC task, TASK_PRI_VC,
     * TASK_STACK_VC_BYTES, "vc") and app_start_task(Motion task,
     * TASK_PRI_MOTION, TASK_STACK_MOTION_BYTES, "motion"), then start the
     * cyclic handler. */
    return (int32_t)E_OK;
}
