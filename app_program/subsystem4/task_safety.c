/*
 *  task_safety.c
 *  Subsystem 4 -- Safety / Collision Response task (priority 1).
 *  Kernel side: kernel and project headers only, never SDK headers (rule R1)
 *  and never blocking SDK waits (rule R2).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Responsibilities (design.md 3.1, HANDOVER.md section 3):
 *    - Waits on EVF_SAFETY for COLLISION | IMU_ALIVE with TWF_ORW | TWF_BITCLR
 *      (never TWF_CLR): the wait clears only the bits that woke it, so other
 *      setters are not lost.
 *    - On COLLISION: calls motion_emergency_brake(), sets FAULT_LATCHED, sends
 *      a fault telemetry event.
 *    - IMU watchdog: if the wait times out (SAFETY_IMU_WDOG_MS, 50 ms) without
 *      IMU_ALIVE being set, the IMU missed its heartbeat; brake and set
 *      FAULT_IMU_WDOG.
 *    - Never takes MTX_MOTION (reads motion_state_t only through
 *      motion_get_state() in subsystem2.h, or tk_ref_flg for EVF_SAFETY).
 */

#include "task_safety.h"

#include "../subsystem2/subsystem2.h"
#include "config/timing.h"

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */

static volatile bool     s_running;
static volatile uint32_t s_brake_count;
static volatile uint32_t s_wdog_timeout_count;

/* ---------------------------------------------------------------------------
 * Public accessors
 * ------------------------------------------------------------------------- */

bool safety_is_task_running(void)
{
    return s_running;
}

uint32_t safety_get_brake_count(void)
{
    return s_brake_count;
}

uint32_t safety_get_wdog_timeout_count(void)
{
    return s_wdog_timeout_count;
}

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static void do_emergency_brake(uint32_t reason_bits, char const *p_reason)
{
    motion_emergency_brake();
    s_brake_count++;

    (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED | reason_bits);

    dbg_printf("[SAFETY] BRAKE: %s", p_reason);

    /* Send a fault telemetry event with the current EVF_SAFETY bits. */
    T_RFLG rflg = { 0 };

    (void)tk_ref_flg(EVF_SAFETY, &rflg);

    uint32_t const now = app_period_now_ms();
    tel_evt_t const evt = drep_tel_make(DREP_TEL_FAULT, now,
                                        rflg.flgptn & EVF_SAFETY_ALL_MASK,
                                        false);

    (void)tk_snd_mbf(MBF_TELEMETRY, &evt, (INT)sizeof(evt), TMO_POL);
}

/* ---------------------------------------------------------------------------
 * Task entry
 * ------------------------------------------------------------------------- */

void safety_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    s_running             = true;
    s_brake_count         = 0U;
    s_wdog_timeout_count  = 0U;

    dbg_puts("[SAFETY] task started");

    for (;;)
    {
        UINT flg_pattern = 0U;

        ER const er = tk_wai_flg(EVF_SAFETY,
                                 EVF_SAFETY_WAIT_MASK,
                                 (TWF_ORW | TWF_BITCLR),
                                 &flg_pattern,
                                 (TMO)SAFETY_IMU_WDOG_MS);

        if (er == E_TMOUT)
        {
            /* IMU watchdog: no IMU_ALIVE heartbeat within the timeout. */
            s_wdog_timeout_count++;
            do_emergency_brake(EVF_SAFETY_FAULT_IMU_WDOG, "IMU watchdog");
            continue;
        }

        if (er < E_OK)
        {
            dbg_put_err("safety wai_flg", (int32_t)er);
            (void)tk_dly_tsk(SAFETY_IMU_WDOG_MS);
            continue;
        }

        /* At least one of the waited bits fired. */

        if ((flg_pattern & EVF_SAFETY_COLLISION) != 0U)
        {
            do_emergency_brake(0U, "collision");
        }

        /* IMU_ALIVE is consumed by TWF_BITCLR; its arrival resets the
         * watchdog timer for the next iteration (the TMO restarts). */
    }
}
