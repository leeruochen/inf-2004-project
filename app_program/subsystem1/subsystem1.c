/*
 *  subsystem1.c
 *  Subsystem 1 -- telemetry: creates its objects and starts its task.
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Stub (Phase 1 skeleton).  See subsystem1.h for the task and interfaces.
 */

#include "subsystem1.h"

int32_t subsystem1_init(void)
{
    /* TODO(phase 2, buddy 1): create the telemetry cyclic handler (period
     * TODO(owner-to-confirm, buddy 1), delta D2 not approved), created
     * stopped and started in subsystem1_start(). */
    return (int32_t)E_OK;
}

int32_t subsystem1_start(void)
{
    /* TODO(phase 2, buddy 1): app_start_task(telemetry task, TASK_PRI_TEL,
     * TASK_STACK_TEL_BYTES, "tel"), then start the cyclic handler. */
    return (int32_t)E_OK;
}
