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
#include "src/common/stack_mon.h"

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
    /* The kernel allocates the stack (stksz = stack_bytes, plus its own
     * 128 B).  Registered tasks start through the stack monitor, which paints
     * the stack for the high-water report (batchB-plan O8). */
    void         *p_mon = stack_mon_register(p_entry, stack_bytes, p_name);
    T_CTSK const  ctsk =
    {
        .exinf   = p_mon,
        .tskatr  = TA_HLNG | TA_RNG0,
        .task    = (p_mon != NULL) ? (FP)stack_mon_trampoline : (FP)p_entry,
        .itskpri = (PRI)priority,
        .stksz   = (SZ)stack_bytes,
        .bufptr  = NULL,
    };
    ID tskid;
    ER er;

    tskid = tk_cre_tsk(&ctsk);
    if (tskid < E_OK)
    {
        APP_CHECK(tskid, p_name);
        return (int32_t)tskid;
    }

    er = tk_sta_tsk(tskid, 0);
    if (er < E_OK)
    {
        APP_CHECK(er, p_name);
        (void)tk_del_tsk(tskid);
        return (int32_t)er;
    }

    return (int32_t)tskid;
}

INT usermain(void)
{
    /* Measure how deep usermain goes on the 1 KB init stack (O8). */
    stack_mon_paint_init_window();

    /* 1. Every IPC object (design.md 3.3), before any subsystem code runs.
     *    Then the debug drain (DBG_ENABLE builds; a no-op otherwise), which
     *    prints the log of everything after it, including step 1. */
    APP_CHECK(ipc_init(), "ipc_init");
    APP_CHECK(dbg_drain_start(), "dbg_drain_start");

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

    /* 3b. PLATFORM_SELFTEST builds only (stage B4b); a no-op otherwise. */
    APP_CHECK(dbg_selftest_start(), "dbg_selftest_start");

#if APP_TEST
    /* 4. Test build only: the tests run in their own lowest-priority task. */
    APP_CHECK(app_start_task(app_test_task, TASK_PRI_TEST, TASK_STACK_TEST_BYTES, "app_test"),
              "app_test");
#endif /* APP_TEST */

    /* 5. Sleep forever.  Returning from usermain() would shut down. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}
