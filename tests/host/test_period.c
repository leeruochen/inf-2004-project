/*
 *  Host test for src/common/period.c (app_period_wait).
 *  A simulated kernel clock with the real tk_dly_tsk rule: a delay of d
 *  ticks wakes the task at tick now + d + 1 (knl_timer_insert_reltim()).
 *  The task's own run time is added by the test between waits.
 */

#include <stdint.h>
#include <string.h>

#include "tk/tkernel.h"
#include "src/common/period.h"
#include "test_check.h"

static uint32_t s_now;      /* simulated kernel time, ms */
static uint32_t s_dly_calls;

ER tk_get_otm(SYSTIM *pk_tim)
{
    pk_tim->hi = 0;
    pk_tim->lo = (UW)s_now;
    return E_OK;
}

ER tk_dly_tsk(RELTIM dlytim)
{
    s_dly_calls++;
    if (dlytim > 0U)
    {
        s_now += (uint32_t)dlytim + 1U;
    }
    return E_OK;
}

/* A naive "work; tk_dly_tsk(P)" loop drifts by one tick per cycle. */
static void test_naive_loop_drifts(void)
{
    uint32_t start;
    uint32_t i;

    s_now = 1000U;
    start = s_now;
    for (i = 0U; i < 100U; i++)
    {
        (void)tk_dly_tsk(5U);
    }
    CHECK((s_now - start) == 600U);     /* 6 ms per cycle, not 5 */
}

/* On-time task: every release exactly on the grid, nothing missed. */
static void test_exact_grid(uint32_t period, uint32_t start)
{
    app_period_t per;
    uint32_t     i;
    uint32_t     bad = 0U;
    uint32_t     missed = 0U;

    s_now = start;
    app_period_init(&per, period);
    for (i = 1U; i <= 1000U; i++)
    {
        missed += app_period_wait(&per);
        if (s_now != (start + (i * period)))
        {
            bad++;
        }
    }
    CHECK(bad == 0U);
    CHECK(missed == 0U);
}

/* One cycle overruns 25 ms of a 10 ms period: one release is skipped, the
 * next is served late, then the task is back on the grid. */
static void test_overrun_skips(void)
{
    app_period_t per;
    uint32_t     missed;

    s_now = 0U;
    app_period_init(&per, 10U);
    CHECK(app_period_wait(&per) == 0U);
    CHECK(s_now == 10U);
    s_now += 25U;                               /* long cycle: now 35 */
    missed = app_period_wait(&per);             /* release 20 missed, 30 late */
    CHECK(missed == 1U);
    CHECK(s_now == 35U);
    CHECK(app_period_wait(&per) == 0U);
    CHECK(s_now == 40U);                        /* back on the grid */
    CHECK(app_period_wait(&per) == 0U);
    CHECK(s_now == 50U);
}

/* A cycle that ends one tick before its release can only be released one
 * tick late; the grid itself does not drift. */
static void test_one_tick_ahead(void)
{
    app_period_t per;
    uint32_t     i;
    uint32_t     bad = 0U;

    s_now = 0U;
    app_period_init(&per, 2U);
    for (i = 1U; i <= 1000U; i++)
    {
        (void)app_period_wait(&per);
        if ((s_now < (2U * i)) || (s_now > ((2U * i) + 1U)))
        {
            bad++;
        }
        s_now += 1U;                            /* 1 ms of work per cycle */
    }
    CHECK(bad == 0U);
}

/* Several tasks share one grid; a first release in the past is served at once. */
static void test_init_at(void)
{
    app_period_t per;

    s_now = 500U;
    app_period_init_at(&per, 20U, 600U);
    CHECK(app_period_wait(&per) == 0U);
    CHECK(s_now == 600U);
    CHECK(app_period_wait(&per) == 0U);
    CHECK(s_now == 620U);

    s_now = 700U;
    app_period_init_at(&per, 20U, 650U);        /* 50 ms in the past */
    s_dly_calls = 0U;
    CHECK(app_period_wait(&per) == 2U);         /* 650, 670 missed; 690 late */
    CHECK(s_dly_calls == 0U);
    CHECK(app_period_wait(&per) == 0U);
    CHECK(s_now == 710U);
}

static void test_zero_period(void)
{
    app_period_t per;

    s_now = 0U;
    app_period_init(&per, 0U);
    CHECK(per.period_ms == 1U);
}

int main(void)
{
    test_naive_loop_drifts();
    test_exact_grid(5U, 0U);
    test_exact_grid(2U, 12345U);
    test_exact_grid(60U, 7U);
    test_exact_grid(5U, 0xFFFFF000UL);          /* wraps through 0 */
    test_overrun_skips();
    test_one_tick_ahead();
    test_init_at();
    test_zero_period();
    return TEST_RESULT("period");
}
