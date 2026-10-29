/*
 *  subsystem3.c
 *  Subsystem 3 -- line following and barcode decode: creates its objects
 *  and starts its tasks.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Stub (Phase 1 skeleton).  See subsystem3.h for the tasks and interfaces.
 */

#include "subsystem3.h"

int32_t subsystem3_init(void)
{
    /* TODO(phase 2, buddy 3): create the Line-Following cyclic handler
     * (LF_SAMPLE_PERIOD_MS, 5 ms), created stopped and started in
     * subsystem3_start(). */
    return (int32_t)E_OK;
}

int32_t subsystem3_start(void)
{
    /* TODO(phase 2, buddy 3): app_start_task(LF task, TASK_PRI_LF,
     * TASK_STACK_LF_BYTES, "lf") and app_start_task(BC task, TASK_PRI_BC,
     * TASK_STACK_BC_BYTES, "bc"), then start the cyclic handler. */
    return (int32_t)E_OK;
}
