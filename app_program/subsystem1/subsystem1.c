/*
 *  subsystem1.c
 *  Subsystem 1 -- telemetry: creates its objects and starts its task.
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Subsystem 1 owns no private kernel objects: its queue (MBF_TELEMETRY,
 *  IF-04) is created by ipc_init(), and the task paces itself with
 *  app_period_wait() instead of a cyclic handler (TEAM.md section 6 budgets
 *  one cyclic handler for S1; it is not needed and can be released).
 */

#include "subsystem1.h"
#include "task_tel.h"

int32_t subsystem1_init(void)
{
    /* Nothing to create before the tasks start (see above).  The CYW43 radio
     * is powered up later, by the Telemetry task itself: it takes up to a
     * second and must not run on the 1 KB init stack. */
    return (int32_t)E_OK;
}

int32_t subsystem1_start(void)
{
    int32_t const tskid = tel_task_start();

    APP_CHECK(tskid, "subsystem1: start tel");
    return (tskid < 0) ? tskid : (int32_t)E_OK;
}
