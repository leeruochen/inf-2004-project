/*
 *  app_main.c
 *  Kernel-side start of the team code: usermain().  Owner: team (shared).
 *
 *  Two entry points, two roles
 *  ---------------------------
 *    src/app/main.c          SDK main() (platform code, created in Batch B,
 *                            not present yet).  SDK init, USB CDC connect
 *                            wait (DBG_USB_CONNECT_WAIT_MS, before the kernel
 *                            only), then starts micro T-Kernel.
 *    app_program/app_main.c  this file.  The kernel creates its init task
 *                            (priority 1, 1 KB stack, INITTASK_STKSZ) and the
 *                            init task calls usermain() below.
 *
 *  usermain() order (two-phase start, from the course template):
 *    1. create every IPC object (ipc_init(), Batch B)
 *    2. every subsystemN_init()
 *    3. every subsystemN_start()
 *    4. APP_TEST builds only: start the test task (TASK_PRI_TEST, 11)
 *    5. tk_slp_tsk(TMO_FEVR); returning from usermain() shuts the kernel down
 *
 *  usermain() runs on the 1 KB init stack with interrupts live: no large
 *  locals, no stdio, no tm_printf (ARCHITECTURE.md, init task stack).
 */

#include "app.h"

#if APP_TEST
/*
 * On-target test task (APP_TEST builds only).  Runs every subsystem's tests
 * in turn at TASK_PRI_TEST, below the debug drain, while the system runs.
 * Results are printed through the dbg_ ring (test_report.h).
 */
static void app_test_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();

    tk_ext_tsk();
}
#endif /* APP_TEST */

int32_t app_start_task(void (*p_entry)(INT stacd, void *p_exinf),
                       uint32_t priority,
                       uint32_t stack_bytes,
                       char const *p_name)
{
    /* TODO(phase 1 batch B, buddy 2): T_CTSK (TA_HLNG, itskpri = priority,
     * stksz = stack_bytes from config/priorities.h) + tk_cre_tsk +
     * tk_sta_tsk; report failures with APP_CHECK(er, p_name). */
    (void)p_entry;
    (void)priority;
    (void)stack_bytes;
    (void)p_name;

    return (int32_t)E_NOSPT;
}

INT usermain(void)
{
    /* 1. TODO(phase 1 batch B, buddy 2): ipc_init() - create every IPC
     *    object (MBF_NAV_IN, MBF_MOTION_CMD, MBF_OBSTACLE, MBF_TELEMETRY,
     *    SEM_ECHO, EVF_SAFETY, EVF_NAV, MTX_MOTION); module not created yet. */

    /* 2. Every subsystem creates its private kernel objects. */
    APP_CHECK(subsystem1_init(), "subsystem1_init");
    APP_CHECK(subsystem2_init(), "subsystem2_init");
    APP_CHECK(subsystem3_init(), "subsystem3_init");
    APP_CHECK(subsystem4_init(), "subsystem4_init");
    APP_CHECK(subsystem5_init(), "subsystem5_init");

    /* 3. Every subsystem starts its tasks. */
    APP_CHECK(subsystem1_start(), "subsystem1_start");
    APP_CHECK(subsystem2_start(), "subsystem2_start");
    APP_CHECK(subsystem3_start(), "subsystem3_start");
    APP_CHECK(subsystem4_start(), "subsystem4_start");
    APP_CHECK(subsystem5_start(), "subsystem5_start");

#if APP_TEST
    /* 4. Test build only: the tests run in their own lowest-priority task. */
    APP_CHECK(app_start_task(app_test_task, TASK_PRI_TEST, TASK_STACK_TEST_BYTES, "app_test"),
              "app_test");
#endif /* APP_TEST */

    /* 5. Sleep forever.  Returning from usermain() would shut down. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}
