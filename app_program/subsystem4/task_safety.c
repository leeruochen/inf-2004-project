/*
 *  task_safety.c
 *  Subsystem 4 -- Safety / Collision Response task (priority 1).
 *  Kernel side: kernel and project headers only, never SDK headers (rule R1)
 *  and never blocking SDK waits (rule R2).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Responsibilities (design.md 3.1, HANDOVER.md section 3):
 *    - Waits on EVF_SAFETY for COLLISION | IMU_ALIVE with TWF_ORW | TWF_BITCLR
 *      (never TWF_CLR), so only the bits that woke it are cleared.
 *    - COLLISION: motion_emergency_brake(), set FAULT_LATCHED, one fault event.
 *    - IMU watchdog: no IMU_ALIVE within SAFETY_IMU_WDOG_MS (NFR13).  Brakes and
 *      sets FAULT_LATCHED | FAULT_IMU_WDOG on every timeout (so a fault reset
 *      by the Vehicle Controller cannot let the car drive without an IMU), but
 *      logs and reports only once per outage.
 *    - Brakes only through motion_emergency_brake() (IF-11).  Never takes
 *      MTX_MOTION.  Only the Vehicle Controller clears FAULT_LATCHED.
 */

#include "task_safety.h"

#include "../subsystem2/subsystem2.h"
#include "config/timing.h"

static volatile bool     s_running;
static volatile uint32_t s_brake_count;
static volatile uint32_t s_wdog_timeout_count;

static bool s_wdog_tripped;

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

static void brake_and_latch(uint32_t reason_bits)
{
    motion_emergency_brake();
    s_brake_count++;
    (void)tk_set_flg(EVF_SAFETY, EVF_SAFETY_FAULT_LATCHED | reason_bits);
}

static void report_fault(char const *p_reason)
{
    T_RFLG rflg = { 0 };

    dbg_printf("[SAFETY] BRAKE: %s", p_reason);

    (void)tk_ref_flg(EVF_SAFETY, &rflg);

    tel_evt_t const evt = drep_tel_make(DREP_TEL_FAULT, app_period_now_ms(),
                                        rflg.flgptn & EVF_SAFETY_ALL_MASK, false);

    (void)tk_snd_mbf(MBF_TELEMETRY, &evt, (INT)sizeof(evt), TMO_POL);
}

void safety_task_entry(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    s_brake_count        = 0U;
    s_wdog_timeout_count = 0U;
    s_wdog_tripped       = false;
    s_running            = true;

    dbg_puts("[SAFETY] task started");

    for (;;)
    {
        UINT     flg_pattern = 0U;
        ER const er = tk_wai_flg(EVF_SAFETY, EVF_SAFETY_WAIT_MASK,
                                 (TWF_ORW | TWF_BITCLR), &flg_pattern,
                                 (TMO)SAFETY_IMU_WDOG_MS);

        if (er == E_TMOUT)
        {
            s_wdog_timeout_count++;
            brake_and_latch(EVF_SAFETY_FAULT_IMU_WDOG);

            if (!s_wdog_tripped)
            {
                s_wdog_tripped = true;
                report_fault("IMU watchdog");
            }
            continue;
        }

        if (er < E_OK)
        {
            /* Cannot watch the IMU: fail safe. */
            dbg_put_err("safety wai_flg", (int32_t)er);
            motion_emergency_brake();
            (void)tk_dly_tsk(SAFETY_IMU_WDOG_MS);
            continue;
        }

        if ((flg_pattern & EVF_SAFETY_COLLISION) != 0U)
        {
            brake_and_latch(0U);
            report_fault("collision");
        }

        if (((flg_pattern & EVF_SAFETY_IMU_ALIVE) != 0U) && s_wdog_tripped)
        {
            s_wdog_tripped = false;
            dbg_puts("[SAFETY] IMU heartbeat back (fault stays latched)");
        }
    }
}
