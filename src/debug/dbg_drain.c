/*
 *  dbg_drain.c
 *  The debug drain task: the ONLY caller of stdio (ARCHITECTURE.md D1, rule
 *  R3a).  Owner: platform (Batch B, stage B4).  Kernel side + <stdio.h>.
 *  Built only with DBG_ENABLE=ON (src/debug/sources.cmake).
 *
 *  Every DBG_DRAIN_PERIOD_MS, while a terminal is connected:
 *    1. a valid RAM fault record from before the last reset, printed once
 *       and then cleared (C3, docs/DEBUG.md);
 *    2. the system-message ring (kernel text, tm_snd_dat);
 *    3. the dbg_ ring, one line at a time;
 *    4. the drop counters of both rings, when they change;
 *    5. every DRAIN_ALIVE_PERIOD_S, an alive line with the SDK timer and the
 *       kernel's operating time side by side.  The kernel clock starts later
 *       (after the USB wait), so the DIFFERENCE must stay constant: that
 *       shows the kernel tick is 1 ms (SysTick set for clk_sys = 125 MHz).
 *  With no terminal connected nothing is read: USB CDC would drop the text,
 *  so it waits in the rings (they keep the oldest text and count what does
 *  not fit) and appears when the terminal is opened.  The fault record is
 *  cleared only after it was printed to a connected terminal.
 *
 *  Runs at TASK_PRI_DBG_DRAIN (10), below every control task, so a slow USB
 *  host never delays one.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "app_program/app.h"
#include "config/timing.h"
#include "src/debug/dbg.h"
#include "src/rtos/port/hal.h"

/* Bytes of kernel text moved per read; the ring is 512 B. */
#define DRAIN_SYSMSG_CHUNK  (64U)
/* Alive line period. */
#define DRAIN_ALIVE_PERIOD_S (10U)

static char const *fault_type_name(uint32_t type)
{
    switch (type)
    {
        case HAL_FAULT_TYPE_HARDFAULT:
            return "HardFault";
        case HAL_FAULT_TYPE_SHUTDOWN:
            return "kernel shutdown";
        default:
            return "unknown";
    }
}

/* "[DBG] alive: SDK timer <ms> ms, kernel time <ms> ms, difference <ms> ms". */
static void drain_alive(void)
{
    SYSTIM   otm = { 0, 0U };
    uint32_t sdk_ms = hal_time_us() / 1000U;

    (void)tk_get_otm(&otm);
    printf("[DBG] alive: SDK timer %lu ms, kernel time %lu ms, difference %ld ms\n",
           (unsigned long)sdk_ms, (unsigned long)otm.lo, (long)(sdk_ms - (uint32_t)otm.lo));
}

/* Print and clear a valid fault record.  Returns true when nothing is left
 * to print (no record, or printed and cleared). */
static bool drain_fault_record(void)
{
    hal_fault_payload_t rec;

    if (!hal_fault_get(&rec))
    {
        return true;
    }
    printf("[FAULT] %s before the last reset: pc=0x%08lx lr=0x%08lx xpsr=0x%08lx err=%ld resets=%lu\n",
           fault_type_name(rec.type), (unsigned long)rec.pc, (unsigned long)rec.lr,
           (unsigned long)rec.xpsr, (long)(int32_t)rec.kernel_err, (unsigned long)rec.boot_count);
    hal_fault_clear();
    return true;
}

static void dbg_drain_task(INT stacd, void *p_exinf)
{
    static char s_sysmsg[DRAIN_SYSMSG_CHUNK + 1U];
    static char s_line[DBG_LINE_MAX + 1U];
    bool        fault_done = false;
    uint32_t    shown_dbg_drops = 0U;
    uint32_t    shown_sys_drops = 0U;
    uint32_t    last_alive_us = hal_time_us();

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        if (hal_console_connected())
        {
            uint32_t dbg_drops;
            uint32_t sys_drops;

            if (!fault_done)
            {
                fault_done = drain_fault_record();
            }
            while (hal_sysmsg_read(s_sysmsg, (uint32_t)sizeof(s_sysmsg)) > 0U)
            {
                printf("%s", s_sysmsg);
            }
            while (dbg_ring_read(s_line, (uint32_t)sizeof(s_line)) > 0U)
            {
                printf("%s\n", s_line);
            }

            dbg_drops = dbg_overflow_count();
            sys_drops = hal_sysmsg_dropped();
            if ((dbg_drops != shown_dbg_drops) || (sys_drops != shown_sys_drops))
            {
                printf("[DBG] dropped so far: %lu dbg_ lines, %lu kernel text bytes\n",
                       (unsigned long)dbg_drops, (unsigned long)sys_drops);
                shown_dbg_drops = dbg_drops;
                shown_sys_drops = sys_drops;
            }

            if ((hal_time_us() - last_alive_us) >= (DRAIN_ALIVE_PERIOD_S * 1000000U))
            {
                last_alive_us = hal_time_us();
                drain_alive();
            }
        }
        (void)tk_dly_tsk((TMO)DBG_DRAIN_PERIOD_MS);
    }
}

int32_t dbg_drain_start(void)
{
    int32_t const tskid = app_start_task(dbg_drain_task, TASK_PRI_DBG_DRAIN,
                                         TASK_STACK_DBG_DRAIN_BYTES, "dbg_drain");

    return (tskid < 0) ? tskid : 0;
}
