/*
 *  dbg_selftest.c
 *  Platform self-test task (Batch B stage B4b, batchB-plan B4 checks 2, 4,
 *  5).  PLATFORM_SELFTEST=ON builds only, never in a normal build.
 *  Owner: platform.  Kernel side: kernel headers only.
 *
 *  Runs once per power-up, in three stages kept across resets
 *  (hal_selftest_stage(), watchdog scratch register):
 *    START        no fault record at boot (power-on RAM rejected); CRC-32
 *                 check value; stack painting bounds (O8) and unused bytes
 *                 per task; late kernel text through tm_printf (C3); then a
 *                 forced HardFault -> watchdog reset.
 *    AFTER_FAULT  the fault record was there at boot (the drain prints it
 *                 once, with the PC inside hal_selftest_force_fault); then a
 *                 plain watchdog reset.
 *    AFTER_WARM   no fault record at boot (it was cleared after printing).
 *    DONE         nothing more until the next power cycle.
 *  Every result is one "[SELFTEST] ... PASS|FAIL" line through the dbg_ ring.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_program/app.h"
#include "src/common/crc32.h"
#include "src/common/stack_mon.h"
#include "src/debug/dbg.h"
#include "src/rtos/port/hal.h"

/* Wait for a terminal before anything is forced, so the log is seen. */
#define SELFTEST_POLL_MS            (100)
/* Let the drain print the boot lines (and a fault record) first. */
#define SELFTEST_SETTLE_MS          (1500)
/* Time for the drain to deliver the results before a reset. */
#define SELFTEST_FLUSH_MS           (3000)

static uint32_t s_pass;
static uint32_t s_fail;

static void check(bool ok, char const *p_what)
{
    if (ok)
    {
        s_pass++;
    }
    else
    {
        s_fail++;
    }
    dbg_printf("[SELFTEST] %s %s", ok ? "PASS" : "FAIL", p_what);
}

/* O8: this task's own entry must contain its stack pointer, have the size
 * the kernel allocated, and every painted stack must still have unused
 * pattern at the bottom.  Also prints each stack's unused bytes. */
static void check_stacks(void)
{
    uint32_t                 marker = 0U;
    uint32_t const           here = (uint32_t)(uintptr_t)&marker;
    bool                     own_found = false;
    bool                     own_ok = false;
    bool                     all_painted = true;
    uint32_t                 i;

    for (i = 0U; i < stack_mon_count(); i++)
    {
        stack_mon_entry_t const *p_ent = stack_mon_entry(i);
        uint32_t                 bottom;
        uint32_t                 top;
        uint32_t                 unused;

        if ((p_ent == NULL) || (p_ent->p_bottom == NULL))
        {
            continue;   /* registered task that has not started yet */
        }
        bottom = (uint32_t)(uintptr_t)p_ent->p_bottom;
        top = (uint32_t)(uintptr_t)p_ent->p_top;
        unused = stack_mon_unused_bytes(p_ent);
        dbg_printf("[SELFTEST] stack %s: %lu of %lu B never used (0x%08lx..0x%08lx)",
                   (p_ent->p_name != NULL) ? p_ent->p_name : "?", (unsigned long)unused,
                   (unsigned long)(top - bottom), (unsigned long)bottom, (unsigned long)top);
        if (unused == 0U)
        {
            all_painted = false;
        }
        if ((here >= bottom) && (here < top))
        {
            uint32_t const expect = (p_ent->stack_bytes + 128U + 7U) & ~7UL;

            own_found = true;
            own_ok = ((top & 7U) == 0U) && ((top - bottom) == expect);
        }
    }
    check(own_found && own_ok, "own stack pointer inside its painted bounds, size = round8(stksz + 128)");
    check(all_painted, "every started stack still has unused pattern at the bottom");
}

static void stage_start(void)
{
    uint32_t fault_fn;

    dbg_puts("[SELFTEST] stage 1 of 3: checks, then a forced HardFault");
    check(!hal_fault_seen_at_boot(), "no fault record at power-up (random RAM rejected)");
    check(crc32_ieee("123456789", 9U) == CRC32_CHECK_VALUE, "CRC-32 check value 0xCBF43926");
    check_stacks();
    hal_selftest_late_sysmsg();
    dbg_printf("[SELFTEST] summary: %lu passed, %lu failed",
               (unsigned long)s_pass, (unsigned long)s_fail);

    hal_selftest_set_stage(HAL_SELFTEST_STAGE_AFTER_FAULT);
    fault_fn = (uint32_t)(uintptr_t)hal_selftest_force_fault & ~1UL;
    dbg_puts("[SELFTEST] forcing a HardFault now: expect a reset, then a [FAULT] line");
    dbg_printf("[SELFTEST] expected [FAULT] pc: 0x%08lx..0x%08lx (hal_selftest_force_fault)",
               (unsigned long)fault_fn, (unsigned long)(fault_fn + 8U));
    (void)tk_dly_tsk(SELFTEST_FLUSH_MS);
    hal_selftest_force_fault();
}

static void stage_after_fault(void)
{
    dbg_puts("[SELFTEST] stage 2 of 3: after the forced fault");
    check(hal_fault_seen_at_boot(), "fault record survived the reset (see the [FAULT] line above)");
    dbg_puts("[SELFTEST] plain reset in 3 s: the fault record must NOT print again");
    hal_selftest_set_stage(HAL_SELFTEST_STAGE_AFTER_WARM);
    (void)tk_dly_tsk(SELFTEST_FLUSH_MS);
    hal_selftest_warm_reboot();
}

static void stage_after_warm(void)
{
    dbg_puts("[SELFTEST] stage 3 of 3: after the plain reset");
    check(!hal_fault_seen_at_boot(), "printed fault record was cleared (not shown again)");
    hal_selftest_set_stage(HAL_SELFTEST_STAGE_DONE);
    dbg_printf("[SELFTEST] all stages done: %lu passed, %lu failed in stage 3",
               (unsigned long)s_pass, (unsigned long)s_fail);
    dbg_puts("[SELFTEST] unplug and replug to run the self-test again");
}

static void dbg_selftest_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    while (!hal_console_connected())
    {
        (void)tk_dly_tsk(SELFTEST_POLL_MS);
    }
    (void)tk_dly_tsk(SELFTEST_SETTLE_MS);

    switch (hal_selftest_stage())
    {
        case HAL_SELFTEST_STAGE_START:
            stage_start();
            break;
        case HAL_SELFTEST_STAGE_AFTER_FAULT:
            stage_after_fault();
            break;
        case HAL_SELFTEST_STAGE_AFTER_WARM:
            stage_after_warm();
            break;
        default:
            dbg_puts("[SELFTEST] done earlier in this power-up; unplug and replug to run again");
            break;
    }
    tk_ext_tsk();
}

int32_t dbg_selftest_start(void)
{
    int32_t const tskid = app_start_task(dbg_selftest_task, TASK_PRI_TEST,
                                         TASK_STACK_TEST_BYTES, "selftest");

    return (tskid < 0) ? tskid : 0;
}
