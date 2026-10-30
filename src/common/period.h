/*
 *  period.h
 *  Drift-free periodic wait for cyclic tasks (Batch B stage B5).
 *  Owner: platform.  Pure header (<stdint.h> only); period.c is kernel side.
 *
 *  Why not a plain tk_dly_tsk(P) loop: tk_dly_tsk(d) wakes at kernel tick
 *  now + d + 1 (kernel/tkernel/timer.c knl_timer_insert_reltim() adds one
 *  TIMER_PERIOD so the wait is never shorter than d).  A loop of
 *  "work; tk_dly_tsk(5);" therefore runs every 6 ms, not 5, plus its own
 *  run time.  app_period_wait() releases the task on a fixed grid of kernel
 *  ticks instead: start, start + P, start + 2P, ...
 *
 *  Use:
 *      app_period_t per;
 *      app_period_init(&per, LF_SAMPLE_PERIOD_MS);
 *      for (;;)
 *      {
 *          (void)app_period_wait(&per);    // returns missed releases
 *          ... one cycle of work ...
 *      }
 *
 *  Limits: the grid is in whole kernel ticks (1 ms).  A release is late by
 *  up to one tick when the task needs the CPU from just before its release
 *  tick until after it (then only a 2-tick wait is possible).  If a cycle
 *  overruns whole periods, the missed releases are skipped (no burst of
 *  catch-up cycles) and counted in the return value.
 */

#ifndef PERIOD_H
#define PERIOD_H

#include <stdint.h>

typedef struct
{
    uint32_t period_ms;     /* > 0 */
    uint32_t next_ms;       /* kernel time (ms) of the next release */
} app_period_t;

/** Kernel time in ms (low 32 bits of tk_get_otm(); wraps after 49.7 days). */
uint32_t app_period_now_ms(void);

/** First release one period from now. */
void app_period_init(app_period_t *p_per, uint32_t period_ms);

/** First release at kernel time first_ms (lets several tasks share a grid). */
void app_period_init_at(app_period_t *p_per, uint32_t period_ms, uint32_t first_ms);

/**
 * Wait for the next release.  Returns the number of releases skipped
 * because the previous cycle overran them (0 when on time).
 */
uint32_t app_period_wait(app_period_t *p_per);

#endif /* PERIOD_H */
