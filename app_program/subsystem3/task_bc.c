/*
 *  task_bc.c
 *  Subsystem 3 -- Barcode Decode task (see task_bc.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel headers only, never SDK headers (rule R1).
 *
 *  States (design.md 6.2, BC-2A part):
 *    IDLE       tk_slp_tsk() until the GP27 handler wakes the task
 *    CAPTURING  every BC_POLL_PERIOD_MS (ARCHITECTURE.md D3):
 *                 1. read Motion's speed ONCE (motion_get_state, IF-08);
 *                 2. pop EVERY waiting gap, pair each with that speed
 *                    (bc_frame_add stores the magnitude);
 *                 3. ring empty and bc_frame_ended() -> decide the capture.
 *    decide     bc_frame_finish(): letter, or the reason it is invalid
 *               (INVALID_READ_RESET); log it with the raw gaps; back to IDLE.
 *
 *  Speed rule and its limit: all gaps popped in one poll share one speed
 *  reading, taken 0-2 ms (+ any delay of this task) after the newest of
 *  them ended.  At a constant speed that is exact; while the speed changes,
 *  older gaps in the batch get a slightly late speed.  S3-U02 test 12: reads
 *  stay correct up to 340 ms of delay on 200 <-> 400 mm/s ramps; the log
 *  line reports the worst batch and poll interval so hardware runs show the
 *  real delay.
 */

#include "task_bc.h"

#include <stdio.h>                      /* snprintf (formatting only; output via dbg_) */

#include "config/timing.h"              /* BC_POLL_PERIOD_MS */
#include "src/debug/dbg.h"
#include "src/rtos/port/hal.h"          /* hal_time_us() */
#include "../subsystem2/subsystem2.h"   /* motion_get_state() (IF-08) */
#include "bc_decode.h"
#include "bc_frame.h"
#include "drv_bc.h"

/** Raw dump: gap/speed pairs per log line (fits DBG_LINE_MAX = 96). */
#define BC_RAW_PER_LINE         (6U)

static char const *const k_frame_name[BC_STATS_RESULTS] =
{
    "OK", "OVERFLOW", "TOO_LONG", "INCOMPLETE", "NO_SYMBOL", "AMBIGUOUS"
};
static char const *const k_bc1_name[6] =
{
    "OK", "COUNT", "STALL", "PATTERN", "START_STOP", "NOT_LETTER"
};

static bc_frame_t        s_frame;               /* one capture; this task only */
static bc_stats_t        s_stats;               /* this task writes; others copy */
static volatile bool     s_running = false;
#if APP_TEST
static volatile int32_t  s_speed_override = 0;  /* test task writes, one word */
#endif

/* One speed reading for this poll (signed mm/s; bc_frame keeps |v|).  If the
 * state cannot be read the speed is 0, which BC-1 rejects as a stall. */
static int32_t bc_speed_now(void)
{
    int32_t        v = 0;
    motion_state_t ms;

#if APP_TEST
    v = s_speed_override;
#endif
    if ((v == 0) && (motion_get_state(&ms) == E_OK))
    {
        v = drep_ms_speed_x10(ms.w);    /* x10 cm/s = mm/s */
    }
    return v;
}

/* Result line and raw gaps of one capture (tasks only: dbg_printf). */
static void bc_log(bc_frame_result_t res, bc_frame_out_t const *p_out, uint32_t start_gap_us,
                   uint32_t quiet_us, uint32_t max_batch, uint32_t max_poll_us,
                   uint32_t ovf, uint32_t dbl)
{
    uint32_t const n = s_stats.captures;
    int32_t        v_min = 0;
    int32_t        v_max = 0;
    uint32_t       k;

    for (k = 0U; k < s_frame.count; k++)
    {
        int32_t const v = s_frame.elem[k].speed_mmps;

        v_min = ((k == 0U) || (v < v_min)) ? v : v_min;
        v_max = ((k == 0U) || (v > v_max)) ? v : v_max;
    }

    if (res == BC_FRAME_OK)
    {
        dbg_printf("[BC] #%lu OK %c win=%lu n=%lu v=%ld..%ld q=%lums lvl=%u",
                   (unsigned long)n, p_out->letter, (unsigned long)p_out->window,
                   (unsigned long)s_frame.seen, (long)v_min, (long)v_max,
                   (unsigned long)(quiet_us / 1000U), drv_bc_level() ? 1U : 0U);
    }
    else
    {
        /* The BC-1 reason means something only when windows were tried. */
        char const *p_why = (res == BC_FRAME_NO_SYMBOL) ? k_bc1_name[p_out->detail] : "-";

        dbg_printf("[BC] #%lu FAIL %s/%s ok=%lu n=%lu v=%ld..%ld q=%lums lvl=%u",
                   (unsigned long)n, k_frame_name[res], p_why,
                   (unsigned long)p_out->windows_ok, (unsigned long)s_frame.seen,
                   (long)v_min, (long)v_max, (unsigned long)(quiet_us / 1000U),
                   drv_bc_level() ? 1U : 0U);
    }
    dbg_printf("[BC] #%lu ovf=%lu dbl=%lu batch=%lu poll=%luus start=%lu",
               (unsigned long)n, (unsigned long)ovf, (unsigned long)dbl,
               (unsigned long)max_batch, (unsigned long)max_poll_us,
               (unsigned long)start_gap_us);

#if DBG_ENABLE
    /* Raw elements: gap_us/speed_mmps, in crossing order (S3-U02 replay).
     * Debug builds only: Release has no console to print them to. */
    for (k = 0U; k < s_frame.count; k += BC_RAW_PER_LINE)
    {
        char     line[DBG_LINE_MAX + 1U];
        uint32_t pos = 0U;
        uint32_t j;

        pos += (uint32_t)snprintf(&line[pos], sizeof(line) - pos, "[BC] #%lu r%02lu",
                                  (unsigned long)n, (unsigned long)k);
        for (j = k; (j < s_frame.count) && (j < (k + BC_RAW_PER_LINE)) && (pos < sizeof(line)); j++)
        {
            pos += (uint32_t)snprintf(&line[pos], sizeof(line) - pos, " %lu/%ld",
                                      (unsigned long)s_frame.elem[j].gap_us,
                                      (long)s_frame.elem[j].speed_mmps);
        }
        dbg_puts(line);
    }
#endif /* DBG_ENABLE */
}

/* One capture, from the first waiting gap to the decision. */
static void bc_capture(bc_cfg_t const *p_cfg)
{
    uint32_t const    ovf0 = drv_bc_overflow_count();
    uint32_t const    dbl0 = drv_bc_double_count();
    uint32_t          prev_poll_us = hal_time_us();
    uint32_t          start_gap_us = 0U;
    uint32_t          quiet_us = 0U;
    uint32_t          max_batch = 0U;
    uint32_t          max_poll_us = 0U;
    bool              have_start = false;
    bool              done = false;
    bc_frame_out_t    out;
    bc_frame_result_t res;

    bc_frame_reset(&s_frame);
    while (!done)
    {
        int32_t const v = bc_speed_now();       /* one reading per poll */
        uint32_t      batch = 0U;
        uint32_t      gap_us;

        while (drv_bc_pop(&gap_us))
        {
            if (!have_start)
            {
                start_gap_us = gap_us;          /* bc_frame drops it; logged only */
                have_start = true;
            }
            bc_frame_add(&s_frame, gap_us, v);
            batch++;
        }
        max_batch = (batch > max_batch) ? batch : max_batch;

        if (!have_start)
        {
            done = true;                        /* woken with nothing: not a capture */
        }
        else if (drv_bc_pending() == 0U)
        {
            /* Order: ring empty -> last edge -> now -> ring still empty.  An
             * edge after the first check shows up in the second, so the
             * capture never ends on a stale last-edge time. */
            uint32_t const last_us = drv_bc_last_edge_us();
            uint32_t const now_us = hal_time_us();

            quiet_us = now_us - last_us;
            done = (drv_bc_pending() == 0U) && bc_frame_ended(&s_frame, last_us, now_us);
        }
        else
        {
            /* A gap arrived after the pops: take it next poll. */
        }

        if (!done)
        {
            uint32_t now_us;

            (void)tk_dly_tsk((RELTIM)BC_POLL_PERIOD_MS);
            now_us = hal_time_us();
            max_poll_us = ((now_us - prev_poll_us) > max_poll_us) ? (now_us - prev_poll_us) : max_poll_us;
            prev_poll_us = now_us;
        }
    }

    if (have_start)
    {
        uint32_t const ovf = drv_bc_overflow_count() - ovf0;    /* 8-bit count: see drv_bc.h */
        uint32_t const dbl = drv_bc_double_count() - dbl0;

        if (ovf != 0U)
        {
            bc_frame_mark_overflow(&s_frame);
        }
        out.letter = '?';
        res = bc_frame_finish(&s_frame, p_cfg, &out);

        s_stats.captures++;
        s_stats.results[res]++;
        if (res == BC_FRAME_OK)
        {
            s_stats.letters[out.letter - 'A']++;
            s_stats.last_letter = out.letter;
        }
        s_stats.max_gaps_per_poll = (max_batch > s_stats.max_gaps_per_poll) ? max_batch : s_stats.max_gaps_per_poll;
        s_stats.max_poll_gap_us = (max_poll_us > s_stats.max_poll_gap_us) ? max_poll_us : s_stats.max_poll_gap_us;

        bc_log(res, &out, start_gap_us, quiet_us, max_batch, max_poll_us, ovf, dbl);
    }
}

void bc_task_entry(INT stacd, void *p_exinf)
{
    bc_cfg_t cfg;

    (void)stacd;
    (void)p_exinf;
    bc_cfg_default(&cfg);
    s_stats.last_letter = '?';
    s_running = true;

    for (;;)
    {
        /* IDLE.  Wake-ups queued during the last capture (one per edge) are
         * stale: cancel them, then look at the ring.  An edge after this
         * check queues a new wake-up, so tk_slp_tsk() returns at once and
         * no edge is slept through. */
        (void)tk_can_wup(TSK_SELF);
        if (drv_bc_pending() == 0U)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
        bc_capture(&cfg);
    }
    /* Never calls tk_ext_tsk() (rule R6). */
}

bool bc_task_is_running(void)
{
    return s_running;
}

void bc_task_get_stats(bc_stats_t *p_out)
{
    if (p_out != NULL)
    {
        (void)tk_dis_dsp();
        *p_out = s_stats;
        (void)tk_ena_dsp();
    }
}

#if APP_TEST
void bc_task_set_speed_override(int32_t speed_mmps)
{
    s_speed_override = speed_mmps;
}
#endif /* APP_TEST */
