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
#include "drv_motor.h"      /* kernel-free driver interface: motor_brake_all() */

void motion_emergency_brake(void)
{
    motor_brake_all();
}

int32_t subsystem2_init(void)
{
    /* Motion Control uses app_period_wait() (MOTION_PERIOD_MS, 20 ms,
     * HANDOVER section 3 step 7), so no cyclic handler is created. */
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
