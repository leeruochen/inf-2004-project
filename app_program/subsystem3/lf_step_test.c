/*
 *  lf_step_test.c
 *  Subsystem 3 -- S3-U16: what the wheels do after a strong steering command
 *  ends (diagnostic, APP_TEST only, -DS3_STEP_DIAG=1).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel and pure project headers only (rule R1).
 *
 *  Question (16 Nov 2026): on the track the car leaves the line again soon
 *  after a recovery.  In the host model Motion's wheel-speed PID keeps
 *  pushing against a steering command for about a second after it ends,
 *  because in STEER both wheels have the same target and the integral is
 *  cleared only by a stop.  This test measures that on the car, with no
 *  line and no line-following decision in the loop.
 *
 *  Method: the Line-Following task stays DISARMED (it posts nothing).  The
 *  test posts the steering itself on IF-01, exactly as the task would
 *  (drep_nav_steer every 20 ms, TMO_POL), so the Vehicle Controller and
 *  Motion run their normal code:
 *      0 for LF3_RUNUP_MS, then the step for LF3_STEP_MS, then +4 for
 *      LF3_AFTER_MS, then REMOTE_STOP.
 *  Three sequences, twice each: step 0 (the control: what +4 alone does on
 *  this car), -40 (a left recovery) and +32 (the look after a u-turn).
 *  Every 20 ms it records both encoder counts from Motion's published state
 *  (IF-08).  It reports the wheel difference (L - R) / mean in two windows
 *  after the step, 200-800 ms and 800-1400 ms, in tenths of a percent;
 *  > 0 = the car turns right.
 *
 *  Safety: each drive is about 0.55 m and 2.5 s and always ends in
 *  REMOTE_STOP.  It ends early if the Vehicle Controller leaves
 *  NAV_LINE_TRACKING (a safety fault, or obstacle avoidance taking the
 *  authority: such a drive is not counted), or after LF3_MAX_MM.  Nothing
 *  here touches another subsystem's objects except MBF_NAV_IN, as a sender.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/subsystem2.h"   /* motion_get_state, vc_get_system_state, VC_SYS_* */
#include "config/thresholds.h"          /* ENC_*, WHEEL_CIRC_MM */
#include "config/timing.h"              /* MOTION_PERIOD_MS, NAV_MBF_SEND_TMO_MS */
#include "lf_step_test.h"
#include "task_lf.h"

#define LF3_ID              "S3-U16"
#define LF3_POLL_MS         (10U)
#define LF3_SECOND_MS       (1000U)
#define LF3_PLACE_S         (8U)
/** Steering 0 first, so every drive meets the step at the same speed. */
#define LF3_RUNUP_MS        (500U)
/** The strong command: as long as a recovery or a u-turn's look lasts. */
#define LF3_STEP_MS         (600U)
/** Then +4, the edge steering on the line's body (LF_KP_Q7 x 100 / 500). */
#define LF3_AFTER_MS        (1400U)
#define LF3_AFTER_Q7        (4)
#define LF3_TOTAL_MS        (LF3_RUNUP_MS + LF3_STEP_MS + LF3_AFTER_MS)
/** The two windows after the step, ms from its end. */
#define LF3_WIN1_FROM_MS    (200U)
#define LF3_WIN1_TO_MS      (800U)
#define LF3_WIN2_TO_MS      (1400U)
/** A drive is over at this wheel travel whatever the time (expected 550). */
#define LF3_MAX_MM          (900U)
/** A counted drive moved both wheels at least this far. */
#define LF3_MIN_WHEEL_MM    (300U)
#define LF3_ROWS            (140U)      /* 2.5 s at 20 ms, with room */
#define LF3_BURST           (8U)
#define LF3_PAUSE_MS        (100U)
#define LF3_DRIVES          (6U)
#define LF3_SEQS            (3U)

typedef struct
{
    uint16_t t_ms;
    int16_t  cnt_l;         /* since START; 900 mm = 10 700 counts */
    int16_t  cnt_r;
    int8_t   steer_q7;
    uint8_t  vc;
} lf3_row_t;

typedef struct
{
    bool    valid;
    int32_t win1_x10;       /* (L - R) / mean, tenths of a percent */
    int32_t win2_x10;
} lf3_result_t;

static lf3_row_t s_rows[LF3_ROWS];
static uint32_t  s_rows_n;

static void lf3_remote(uint32_t type)
{
    nav_input_t const m = drep_nav_remote(type);

    if (tk_snd_mbf(MBF_NAV_IN, &m, (INT)sizeof(m), (TMO)NAV_MBF_SEND_TMO_MS) != E_OK)
    {
        dbg_puts("[LF] step: remote command not sent (MBF_NAV_IN full)");
    }
}

static uint32_t lf3_mm(int32_t counts)
{
    uint32_t const c = (counts < 0) ? (uint32_t)(-counts) : (uint32_t)counts;

    return (c * WHEEL_CIRC_MM) / (ENC_QUAD_MULT * ENC_CPR);
}

/* (L - R) / mean of the counts gained between two rows, in tenths of a
 * percent: 2000 x (dl - dr) / (dl + dr).  0 if the car did not move. */
static int32_t lf3_diff_x10(lf3_row_t const *p_a, lf3_row_t const *p_b)
{
    int32_t const dl = (int32_t)p_b->cnt_l - (int32_t)p_a->cnt_l;
    int32_t const dr = (int32_t)p_b->cnt_r - (int32_t)p_a->cnt_r;
    int32_t       x10 = 0;

    if ((dl + dr) > 0)
    {
        x10 = (2000 * (dl - dr)) / (dl + dr);
    }
    return x10;
}

/* The row at or just after t_ms; the last row if the drive ended earlier. */
static lf3_row_t const *lf3_row_at(uint32_t t_ms)
{
    uint32_t i = 0U;

    while (((i + 1U) < s_rows_n) && ((uint32_t)s_rows[i].t_ms < t_ms))
    {
        i++;
    }
    return &s_rows[i];
}

/* Tell the operator, count down, and clear a latched fault near the end
 * (catching the car reads as a lift; the VC takes RESET only when still). */
static void lf3_place(uint32_t drive, int32_t step_q7)
{
    dbg_printf("[LF] >>> step drive %lu of %lu (step %ld): car on the open floor, flat",
               (unsigned long)(drive + 1U), (unsigned long)LF3_DRIVES, (long)step_q7);
    dbg_puts("[LF] >>> 1.5 m clear ahead and to both sides; hands off at 0");
    dbg_puts("[LF] >>> the car DRIVES about 0.6 m and stops by itself");
    for (uint32_t left = LF3_PLACE_S; left > 0U; left--)
    {
        if ((left <= 3U) || (left == (LF3_PLACE_S / 2U)))
        {
            dbg_printf("[LF] ... %lu", (unsigned long)left);
        }
        if ((left <= 2U) && (vc_get_system_state() == VC_SYS_SAFETY_FAULT_LATCHED))
        {
            dbg_puts("[LF] fault latched: test sends RESET (car must be still)");
            lf3_remote(DREP_NAV_REMOTE_RESET);
        }
        (void)tk_dly_tsk((RELTIM)LF3_SECOND_MS);
    }
}

static void lf3_dump(uint32_t drive, int32_t step_q7)
{
    uint32_t const dropped0 = dbg_overflow_count();

    dbg_printf("[LF] sr %lu step %ld: t steer L R vc (%lu rows, 20 ms)", (unsigned long)(drive + 1U),
               (long)step_q7, (unsigned long)s_rows_n);
    for (uint32_t i = 0U; i < s_rows_n; i++)
    {
        lf3_row_t const *const p_r = &s_rows[i];

        dbg_printf("[LF] sr %u %d %d %d %u", p_r->t_ms, (int)p_r->steer_q7, (int)p_r->cnt_l,
                   (int)p_r->cnt_r, (unsigned)p_r->vc);
        if (((i + 1U) % LF3_BURST) == 0U)
        {
            (void)tk_dly_tsk((RELTIM)LF3_PAUSE_MS);
        }
    }
    (void)tk_dly_tsk((RELTIM)LF3_PAUSE_MS);
    dbg_printf("[LF] sr %lu end: ring dropped %lu lines", (unsigned long)(drive + 1U),
               (unsigned long)(dbg_overflow_count() - dropped0));
}

/* One drive.  Fills *p_res; valid only if it ran its whole time under the
 * Vehicle Controller's line authority with both wheels counting. */
static void lf3_drive(uint32_t drive, int32_t step_q7, lf3_result_t *p_res)
{
    motion_state_t m0;
    motion_state_t m;
    uint32_t       t0;
    uint32_t       t = 0U;
    uint32_t       tick = 0U;
    uint32_t       vc;
    bool           aborted = false;
    bool           sent_all = true;

    p_res->valid = false;
    p_res->win1_x10 = 0;
    p_res->win2_x10 = 0;
    s_rows_n = 0U;

    lf3_place(drive, step_q7);
    vc = vc_get_system_state();
    if ((vc != VC_SYS_READY_WAITING) || (motion_get_state(&m0) != E_OK))
    {
        dbg_printf("[LF] step drive %lu NOT started: VC state %lu (need 1)", (unsigned long)(drive + 1U),
                   (unsigned long)vc);
        return;
    }

    lf3_remote(DREP_NAV_REMOTE_START);
    t0 = app_period_now_ms();
    while ((t < LF3_TOTAL_MS) && !aborted)
    {
        /* Every second poll = one Motion period: post, then record. */
        if ((tick % 2U) == 0U)
        {
            int32_t const q = (t < LF3_RUNUP_MS) ? 0
                              : ((t < (LF3_RUNUP_MS + LF3_STEP_MS)) ? step_q7 : LF3_AFTER_Q7);
            nav_input_t const msg = drep_nav_steer(q);

            if (tk_snd_mbf(MBF_NAV_IN, &msg, (INT)sizeof(msg), TMO_POL) != E_OK)
            {
                sent_all = false;       /* the next post replaces it */
            }
            vc = vc_get_system_state();
            if ((motion_get_state(&m) == E_OK) && (s_rows_n < LF3_ROWS))
            {
                lf3_row_t *const p_r = &s_rows[s_rows_n];
                int32_t const    cl = (int32_t)((uint32_t)m.enc_count[0] - (uint32_t)m0.enc_count[0]);
                int32_t const    cr = (int32_t)((uint32_t)m.enc_count[1] - (uint32_t)m0.enc_count[1]);

                p_r->t_ms = (uint16_t)t;
                p_r->cnt_l = (int16_t)cl;
                p_r->cnt_r = (int16_t)cr;
                p_r->steer_q7 = (int8_t)q;
                p_r->vc = (uint8_t)vc;
                s_rows_n++;
                if (((lf3_mm(cl) + lf3_mm(cr)) / 2U) >= LF3_MAX_MM)
                {
                    aborted = true;
                }
            }
            if (vc != VC_SYS_NAV_LINE_TRACKING)
            {
                aborted = true;         /* fault, avoidance, or a stop */
            }
        }
        (void)tk_dly_tsk((RELTIM)LF3_POLL_MS);
        tick++;
        t = app_period_now_ms() - t0;
    }
    lf3_remote(DREP_NAV_REMOTE_STOP);
    (void)tk_dly_tsk((RELTIM)(LF3_SECOND_MS / 2U));

    if (s_rows_n >= 2U)
    {
        lf3_row_t const *const p_last = &s_rows[s_rows_n - 1U];
        uint32_t const         end_ms = LF3_RUNUP_MS + LF3_STEP_MS;

        p_res->win1_x10 = lf3_diff_x10(lf3_row_at(end_ms + LF3_WIN1_FROM_MS), lf3_row_at(end_ms + LF3_WIN1_TO_MS));
        p_res->win2_x10 = lf3_diff_x10(lf3_row_at(end_ms + LF3_WIN1_TO_MS), lf3_row_at(end_ms + LF3_WIN2_TO_MS));
        p_res->valid = !aborted && (lf3_mm((int32_t)p_last->cnt_l) >= LF3_MIN_WHEEL_MM)
                       && (lf3_mm((int32_t)p_last->cnt_r) >= LF3_MIN_WHEEL_MM);
        dbg_printf("[LF] step drive %lu (step %ld): %s, L %lu mm R %lu mm, vc at end %lu, posts %s",
                   (unsigned long)(drive + 1U), (long)step_q7, p_res->valid ? "valid" : "NOT VALID",
                   (unsigned long)lf3_mm((int32_t)p_last->cnt_l), (unsigned long)lf3_mm((int32_t)p_last->cnt_r),
                   (unsigned long)vc, sent_all ? "all sent" : "some dropped");
        dbg_printf("[LF] step drive %lu (step %ld): then +4: x10 = %ld (200-800 ms), %ld (800-1400 ms)",
                   (unsigned long)(drive + 1U), (long)step_q7, (long)p_res->win1_x10, (long)p_res->win2_x10);
    }
    lf3_dump(drive, step_q7);
}

void lf_step_test(uint32_t *p_pass, uint32_t *p_fail)
{
    /* The control first in each round, so a drift over the run shows. */
    static int32_t const k_step[LF3_DRIVES] = { 0, -40, 32, 0, -40, 32 };
    uint32_t     test_pass_ = *p_pass;
    uint32_t     test_fail_ = *p_fail;
    lf3_result_t res[LF3_DRIVES];
    uint32_t     n_valid = 0U;

    dbg_puts("[LF] " LF3_ID " steering step diagnostic: OPEN FLOOR, no line, motors LIVE");
    dbg_puts("[LF] the Line-Following task stays disarmed; the test posts the steering");
    lf_task_test_arm(false);

    for (uint32_t d = 0U; d < LF3_DRIVES; d++)
    {
        lf3_drive(d, k_step[d], &res[d]);
        n_valid += res[d].valid ? 1U : 0U;
    }

    /* Per sequence: the mean of its valid drives, and against the control. */
    {
        int32_t  mean1[LF3_SEQS] = { 0, 0, 0 };
        int32_t  mean2[LF3_SEQS] = { 0, 0, 0 };
        uint32_t n[LF3_SEQS] = { 0U, 0U, 0U };

        for (uint32_t d = 0U; d < LF3_DRIVES; d++)
        {
            uint32_t const s = d % LF3_SEQS;

            if (res[d].valid)
            {
                mean1[s] += res[d].win1_x10;
                mean2[s] += res[d].win2_x10;
                n[s]++;
            }
        }
        for (uint32_t s = 0U; s < LF3_SEQS; s++)
        {
            if (n[s] > 0U)
            {
                mean1[s] /= (int32_t)n[s];
                mean2[s] /= (int32_t)n[s];
            }
            dbg_printf("[LF] step result: step %ld, %lu valid: x10 = %ld (200-800 ms), %ld (800-1400 ms)",
                       (long)k_step[s], (unsigned long)n[s], (long)mean1[s], (long)mean2[s]);
        }
        if ((n[0] > 0U) && (n[1] > 0U) && (n[2] > 0U))
        {
            dbg_printf("[LF] step result: minus control, 200-800 ms: after -40 %ld, after +32 %ld (x10)",
                       (long)(mean1[1] - mean1[0]), (long)(mean1[2] - mean1[0]));
        }
    }

    TEST_CHECK(LF3_ID, n_valid == LF3_DRIVES,
               "(d1) six step drives ran their whole time, both wheels counting");

    if (vc_get_system_state() != VC_SYS_READY_WAITING)
    {
        lf3_remote(DREP_NAV_REMOTE_STOP);
    }
    dbg_puts("[LF] " LF3_ID " done: the car stays stopped");

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

#endif /* APP_TEST */
