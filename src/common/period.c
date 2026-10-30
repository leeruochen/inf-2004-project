/*
 *  period.c
 *  Drift-free periodic wait (see period.h).
 *  Owner: platform (Batch B, stage B5).  Kernel side: kernel headers only.
 *  Includes only <tk/tkernel.h> (not app.h), so the host test can build it
 *  with a stand-in kernel (tests/host/test_period.c).
 */

#include <stddef.h>                 /* size_t from the C library (see app.h) */
#include <stdint.h>
#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif
#include <tk/tkernel.h>

#include "src/common/period.h"

uint32_t app_period_now_ms(void)
{
    SYSTIM now;

    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

void app_period_init(app_period_t *p_per, uint32_t period_ms)
{
    app_period_init_at(p_per, period_ms, app_period_now_ms() + period_ms);
}

void app_period_init_at(app_period_t *p_per, uint32_t period_ms, uint32_t first_ms)
{
    p_per->period_ms = (period_ms > 0U) ? period_ms : 1U;
    p_per->next_ms = first_ms;
}

uint32_t app_period_wait(app_period_t *p_per)
{
    int32_t const ahead = (int32_t)(p_per->next_ms - app_period_now_ms());
    uint32_t      missed = 0U;

    if (ahead > 0)
    {
        /* tk_dly_tsk(d) wakes at tick now + d + 1: d = ahead - 1 hits the
         * release tick exactly.  ahead == 1 cannot be hit (d = 0 does not
         * wait); wait 1 (one tick late) rather than release early. */
        (void)tk_dly_tsk((RELTIM)((ahead > 1) ? (ahead - 1) : 1));
    }
    else
    {
        /* Release due now (ahead == 0) or overrun: skip whole missed periods. */
        missed = (uint32_t)(-ahead) / p_per->period_ms;
        p_per->next_ms += missed * p_per->period_ms;
    }
    p_per->next_ms += p_per->period_ms;
    return missed;
}
