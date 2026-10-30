/*
 *  dbg_bringup.c
 *  Platform bring-up heartbeats for the USB CDC bring-up test (Batch B
 *  stage B5, batchB-plan O1; docs/DEBUG.md "USB CDC bring-up test").
 *  PLATFORM_BRINGUP=ON builds only; usermain() then starts these nine tasks
 *  INSTEAD of the subsystems' tasks.  Delete once the real tasks exist.
 *  Owner: platform.  Kernel side: kernel headers only.
 *
 *  Nine tasks at the design priorities, periods and stack sizes (design.md
 *  3.1; event-driven tasks get a stand-in period, see s_hb).  All share one
 *  release grid (app_period_init_at), so every task is released together
 *  on each 1 s boundary: the worst case for the low-priority tasks.
 *
 *  Each task measures the time between its releases with hal_time_us()
 *  and prints, every BRINGUP_REPORT_MS:
 *    [HB] id=<1-9> <name> seq=<n> cycles=<n> period=<us> dt_min=<us> dt_max=<us> missed=<n>
 *  seq counts the lines of that task (a gap = a lost line).
 *
 *  Stress (DEBUG.md step 3): from BRINGUP_STRESS_START_S to
 *  BRINGUP_STRESS_END_S of kernel time, each task also logs
 *  BRINGUP_STRESS_EXTRA_LINES filler lines per report (10x the line rate).
 *  They go out one cycle AFTER the heartbeat line, as one burst per task,
 *  so the 2 KB ring overflows (drop counter rises) while the heartbeat
 *  lines still get through and show whether the periods held.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>                  /* snprintf only (formatting, no output) */

#include "app_program/app.h"
#include "config/timing.h"
#include "src/rtos/port/hal.h"

/* One heartbeat line per task per this many ms. */
#define BRINGUP_REPORT_MS           (1000U)
/* Stand-in period for the event-driven tasks (VC, ObsAvoid): design.md has
 * no period for them; VC follows the 20 ms steering posts. */
#define BRINGUP_VC_PERIOD_MS        (MOTION_PERIOD_MS)
#define BRINGUP_OBS_AVOID_PERIOD_MS (100U)
/* First release of every task, after the last one has started. */
#define BRINGUP_START_DELAY_MS      (100U)
/* Stress window, kernel time: after the 10-minute run. */
#define BRINGUP_STRESS_START_S      (660U)
#define BRINGUP_STRESS_END_S        (720U)
/* 9 extra lines + the heartbeat line = 10x the normal line rate. */
#define BRINGUP_STRESS_EXTRA_LINES  (9U)
#define BRINGUP_TASK_COUNT          (9U)

typedef struct
{
    char const *p_name;
    uint32_t    priority;
    uint32_t    period_ms;
    uint32_t    stack_bytes;
} hb_spec_t;

/* id = index + 1, in design priority order. */
static hb_spec_t const s_hb[BRINGUP_TASK_COUNT] =
{
    { "safety",    TASK_PRI_SAFETY,    SAFETY_IMU_WDOG_MS,          TASK_STACK_SAFETY_BYTES    },
    { "vc",        TASK_PRI_VC,        BRINGUP_VC_PERIOD_MS,        TASK_STACK_VC_BYTES        },
    { "motion",    TASK_PRI_MOTION,    MOTION_PERIOD_MS,            TASK_STACK_MOTION_BYTES    },
    { "lf",        TASK_PRI_LF,        LF_SAMPLE_PERIOD_MS,         TASK_STACK_LF_BYTES        },
    { "bc",        TASK_PRI_BC,        BC_POLL_PERIOD_MS,           TASK_STACK_BC_BYTES        },
    { "obs_scan",  TASK_PRI_OBS_SCAN,  OBS_SERVO_STEP_PERIOD_MS,    TASK_STACK_OBS_SCAN_BYTES  },
    { "obs_avoid", TASK_PRI_OBS_AVOID, BRINGUP_OBS_AVOID_PERIOD_MS, TASK_STACK_OBS_AVOID_BYTES },
    { "imu",       TASK_PRI_IMU,       IMU_SAMPLE_PERIOD_MS,        TASK_STACK_IMU_BYTES       },
    { "tel",       TASK_PRI_TEL,       TEL_WAKE_PERIOD_MS,          TASK_STACK_TEL_BYTES       },
};

/* Pads each filler line to about 90 B: three tasks released on the same
 * 20 ms tick (vc, motion, tel) then write about 2.4 KB before the drain (the
 * lowest priority) can run, more than the 2 KB ring holds. */
static char const s_dots[] = "................................................................";

/* Kernel time (ms) of everyone's first release; set before any task starts. */
static uint32_t s_first_ms;

static bool stress_active(void)
{
    uint32_t const s = app_period_now_ms() / 1000U;

    return (s >= BRINGUP_STRESS_START_S) && (s < BRINGUP_STRESS_END_S);
}

static void hb_run(uint32_t index)
{
    hb_spec_t const *p_hb = &s_hb[index];
    uint32_t const   id = index + 1U;
    uint32_t const   per_report = (p_hb->period_ms >= BRINGUP_REPORT_MS)
                                  ? 1U : (BRINGUP_REPORT_MS / p_hb->period_ms);
    char             filler[DBG_LINE_MAX];
    app_period_t     per;
    uint32_t         seq = 0U;
    uint32_t         cycles = 0U;
    uint32_t         missed = 0U;
    uint32_t         dt_min = UINT32_MAX;
    uint32_t         dt_max = 0U;
    uint32_t         last_us;
    bool             burst_due = false;
    bool             was_stress = false;

    (void)snprintf(filler, sizeof filler, "[HB] id=%lu stress filler %s", (unsigned long)id, s_dots);

    if (id == 1U)
    {
        dbg_printf("[BRINGUP] %lu heartbeat tasks, one line each per %lu ms; stress %lu-%lu s",
                   (unsigned long)BRINGUP_TASK_COUNT, (unsigned long)BRINGUP_REPORT_MS,
                   (unsigned long)BRINGUP_STRESS_START_S, (unsigned long)BRINGUP_STRESS_END_S);
    }
    app_period_init_at(&per, p_hb->period_ms, s_first_ms);
    (void)app_period_wait(&per);
    last_us = hal_time_us();

    for (;;)
    {
        uint32_t now_us;
        uint32_t dt;

        missed += app_period_wait(&per);
        now_us = hal_time_us();
        dt = now_us - last_us;
        last_us = now_us;
        dt_min = (dt < dt_min) ? dt : dt_min;
        dt_max = (dt > dt_max) ? dt : dt_max;
        cycles++;

        if (burst_due)
        {
            uint32_t k;

            burst_due = false;
            for (k = 0U; k < BRINGUP_STRESS_EXTRA_LINES; k++)
            {
                dbg_puts(filler);
            }
        }

        if (cycles >= per_report)
        {
            bool const stress = stress_active();

            seq++;
            dbg_printf("[HB] id=%lu %s seq=%lu cycles=%lu period=%lu dt_min=%lu dt_max=%lu missed=%lu",
                       (unsigned long)id, p_hb->p_name, (unsigned long)seq,
                       (unsigned long)cycles, (unsigned long)(p_hb->period_ms * 1000U),
                       (unsigned long)dt_min, (unsigned long)dt_max, (unsigned long)missed);
            if ((id == 1U) && (stress != was_stress))
            {
                dbg_printf("[BRINGUP] stress %s", stress ? "ON (10x log rate)" : "OFF");
            }
            was_stress = stress;
            burst_due = stress;
            cycles = 0U;
            missed = 0U;
            dt_min = UINT32_MAX;
            dt_max = 0U;
        }
    }
}

/* The kernel passes no argument through app_start_task(): one entry per id. */
#define HB_ENTRY(n)                                                         \
    static void hb_task_##n(INT stacd, void *p_exinf)                       \
    {                                                                       \
        (void)stacd;                                                        \
        (void)p_exinf;                                                      \
        hb_run(n);                                                          \
    }

HB_ENTRY(0)
HB_ENTRY(1)
HB_ENTRY(2)
HB_ENTRY(3)
HB_ENTRY(4)
HB_ENTRY(5)
HB_ENTRY(6)
HB_ENTRY(7)
HB_ENTRY(8)

static void (*const s_entry[BRINGUP_TASK_COUNT])(INT stacd, void *p_exinf) =
{
    hb_task_0, hb_task_1, hb_task_2, hb_task_3, hb_task_4,
    hb_task_5, hb_task_6, hb_task_7, hb_task_8,
};

int32_t dbg_bringup_start(void)
{
    uint32_t i;

    /* Runs in usermain() on the 1 KB init stack: no formatting here (the
     * safety task prints the banner). */
    s_first_ms = app_period_now_ms() + BRINGUP_START_DELAY_MS;
    for (i = 0U; i < BRINGUP_TASK_COUNT; i++)
    {
        int32_t const tskid = app_start_task(s_entry[i], s_hb[i].priority,
                                             s_hb[i].stack_bytes, s_hb[i].p_name);

        if (tskid < 0)
        {
            return tskid;
        }
    }
    return 0;
}
