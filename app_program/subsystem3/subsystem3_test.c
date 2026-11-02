/*
 *  subsystem3_test.c
 *  On-target tests for subsystem 3 (line following and barcode decode).
 *  APP_TEST builds only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem3/logs/ and record the result in
 *  testing/subsystem3/TEST_LOG.md.  Test IDs: S3-U03 .. S3-U06 here (S3-U01
 *  and S3-U02 are host tests).
 *
 *  BC-2A hardware tests, guided by the console: the operator follows the
 *  "[S3] >>>" prompts, the test counts what the Barcode task decoded in each
 *  timed window and decides PASS/FAIL itself.  Guide for the operator:
 *  .agents/tasks/s3-bc2a-test.md.  Order (the car starts on the stand,
 *  because subsystem2_test() spins the wheels before this runs):
 *    S3-U03  GP27 edges, no barcode                 (stand)
 *    S3-U04  hand-slid cards, fixed bench speed     (stand)
 *    S3-U06  invalid reads a, b                     (stand)
 *    S3-U05  driving over cards, real Motion speed  (floor)
 *    S3-U06  invalid reads c, d, e                  (floor)
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include <stdio.h>                      /* snprintf (formatting only) */

#include "../subsystem2/subsystem2.h"   /* motion_get_state(), motion_move_*() */
#include "drv_bc.h"
#include "task_bc.h"

/** Bench speed for hand-slid cards (any constant works: ratios decide). */
#define S3_BENCH_SPEED_MMPS     (300)
/** Time for the Barcode task to finish a capture after the last edge
 *  (worst case: BC_CAPTURE_TIMEOUT_MS) plus margin. */
#define S3_SETTLE_MS            (2000U)
/** Floor pass: card 15 cm ahead of the sensor, 14 cm long: 40 cm clears it. */
#define S3_PASS_CM              (40)
/** Floor stop with the sensor over the middle of the card. */
#define S3_HALF_PASS_CM         (22)
/** Longest wait for one drive (40 cm at 15 cm/s backwards is 2.7 s). */
#define S3_DRIVE_TIMEOUT_MS     (8000U)
/** Distance reached when within this of the target (Motion counts whole cm). */
#define S3_DRIVE_TOL_CM         (2U)
/** Passes per card: forward + backward each. */
#define S3_FLOOR_CYCLES         (5U)
/** Minimum correct reads per 10 passes, and no wrong letter at all. */
#define S3_MIN_CORRECT          (9U)
/** Countdown step, and how often a drive checks Motion's distance. */
#define S3_SECOND_MS            (1000U)
#define S3_DRIVE_POLL_MS        (100U)

/* -------------------------------------------------------------- helpers */

static void s3_snap(bc_stats_t *p_s)
{
    bc_task_get_stats(p_s);
}

static uint32_t s3_ok_total(bc_stats_t const *p_s)
{
    return p_s->results[BC_FRAME_OK];
}

static uint32_t s3_fail_total(bc_stats_t const *p_s)
{
    return p_s->captures - p_s->results[BC_FRAME_OK];
}

/* A countdown: tells the operator what to do and for how long. */
static void s3_countdown(char const *p_tag, char const *p_what, uint32_t seconds)
{
    dbg_printf("[S3] >>> %s: %s (%lu s)", p_tag, p_what, (unsigned long)seconds);
    for (uint32_t left = seconds; left > 0U; left--)
    {
        if (((left % 10U) == 0U) && (left != seconds))
        {
            dbg_printf("[S3] ... %lu s left", (unsigned long)left);
        }
        (void)tk_dly_tsk((RELTIM)S3_SECOND_MS);
    }
    dbg_printf("[S3] <<< %s: stop", p_tag);
}

/* Reads in a window: correct letter, any other letter, refused, overflow. */
typedef struct
{
    uint32_t correct;
    uint32_t wrong;
    uint32_t refused;
    uint32_t ovf;
} s3_tally_t;

static s3_tally_t s3_tally(bc_stats_t const *p_a, bc_stats_t const *p_b, char letter,
                           uint32_t ovf_a, uint32_t ovf_b)
{
    s3_tally_t   t;
    uint32_t const idx = (uint32_t)(letter - 'A');

    t.correct = p_b->letters[idx] - p_a->letters[idx];
    t.wrong = (s3_ok_total(p_b) - s3_ok_total(p_a)) - t.correct;
    t.refused = s3_fail_total(p_b) - s3_fail_total(p_a);
    t.ovf = ovf_b - ovf_a;
    dbg_printf("[S3] card %c: correct=%lu wrong=%lu refused=%lu ovf=%lu", letter,
               (unsigned long)t.correct, (unsigned long)t.wrong,
               (unsigned long)t.refused, (unsigned long)t.ovf);
    return t;
}

/* Drive a distance with the public Motion API and wait until Motion's
 * distance counter has moved that far (or the timeout), then let the
 * capture finish.  Returns true if the distance was reached. */
static bool s3_drive(bool forward, int32_t cm)
{
    motion_state_t a;
    motion_state_t b;
    uint32_t       waited = 0U;
    bool           reached = false;
    ER const       er0 = motion_get_state(&a);
    ER const       er1 = forward ? motion_move_forward(cm) : motion_move_backward(cm);

    while ((er0 == E_OK) && (er1 == E_OK) && !reached && (waited < S3_DRIVE_TIMEOUT_MS))
    {
        (void)tk_dly_tsk((RELTIM)S3_DRIVE_POLL_MS);
        waited += S3_DRIVE_POLL_MS;
        if (motion_get_state(&b) == E_OK)
        {
            /* 16-bit cumulative cm: subtract modulo 2^16. */
            uint32_t const moved = (drep_ms_distance_cm(b.w) - drep_ms_distance_cm(a.w)) & 0xFFFFU;

            reached = (moved + S3_DRIVE_TOL_CM) >= (uint32_t)cm;
        }
    }
    if (!reached)
    {
        dbg_printf("[S3] drive %s %ld cm: not reached in %lu ms (er %d/%d)",
                   forward ? "fwd" : "back", (long)cm, (unsigned long)waited, (int)er0, (int)er1);
    }
    (void)tk_dly_tsk(S3_SETTLE_MS);
    return reached;
}

/* ---------------------------------------------------------------- tests */

/* S3-U03: the GP27 edge path with no barcode. */
static void s3_u03(uint32_t *p_pass, uint32_t *p_fail)
{
    uint32_t test_pass_ = *p_pass;
    uint32_t test_fail_ = *p_fail;
    uint32_t e0;
    uint32_t d0;
    bool     lvl_white;
    bool     lvl_black;

    TEST_CHECK("S3-U03", drv_bc_is_ready() && bc_task_is_running(),
               "(a) GP27 handler registered, Barcode task running");

    s3_countdown("U03 prep", "WHITE paper under the barcode sensor, keep still", 15U);
    e0 = drv_bc_edge_count();
    s3_countdown("U03 white", "keep still over WHITE", 10U);
    lvl_white = drv_bc_level();
    dbg_printf("[S3] white: edges=%lu level=%u", (unsigned long)(drv_bc_edge_count() - e0),
               lvl_white ? 1U : 0U);
    TEST_CHECK("S3-U03", drv_bc_edge_count() == e0, "(b) 0 edges in 10 s over white");

    s3_countdown("U03 prep", "BLACK tape under the barcode sensor, keep still", 15U);
    e0 = drv_bc_edge_count();
    s3_countdown("U03 black", "keep still over BLACK", 10U);
    lvl_black = drv_bc_level();
    dbg_printf("[S3] black: edges=%lu level=%u", (unsigned long)(drv_bc_edge_count() - e0),
               lvl_black ? 1U : 0U);
    TEST_CHECK("S3-U03", (drv_bc_edge_count() == e0) && (lvl_black != lvl_white),
               "(c) 0 edges over black, level differs from white");

    s3_countdown("U03 prep", "back to WHITE, hold the 20 mm black strip ready", 10U);
    e0 = drv_bc_edge_count();
    d0 = drv_bc_glitch_count();
    s3_countdown("U03 strip", "slide the black strip under the sensor 5 times", 20U);
    (void)tk_dly_tsk(S3_SETTLE_MS);
    dbg_printf("[S3] strip: edges=%lu glitches=%lu", (unsigned long)(drv_bc_edge_count() - e0),
               (unsigned long)(drv_bc_glitch_count() - d0));
    TEST_CHECK("S3-U03", (drv_bc_edge_count() - e0) == 10U, "(d) 5 strip passes = exactly 10 edges");

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

/* S3-U04: hand-slid cards at a fixed bench speed, both directions. */
static void s3_u04(uint32_t *p_pass, uint32_t *p_fail)
{
    static char const k_cards[3] = { 'A', 'Z', 'F' };
    uint32_t          test_pass_ = *p_pass;
    uint32_t          test_fail_ = *p_fail;

    bc_task_set_speed_override(S3_BENCH_SPEED_MMPS);
    for (uint32_t c = 0U; c < 3U; c++)
    {
        for (uint32_t dir = 0U; dir < 2U; dir++)
        {
            bc_stats_t a;
            bc_stats_t b;
            uint32_t   ovf_a;
            char       what[DBG_LINE_MAX];

            (void)snprintf(what, sizeof(what), "card *%c*, slide it %s 10 times", k_cards[c],
                           (dir == 0U) ? "LEFT-TO-RIGHT" : "RIGHT-TO-LEFT");
            s3_countdown("U04 prep", "get the card ready, bars across the slide", 15U);
            s3_snap(&a);
            ovf_a = drv_bc_overflow_count();
            s3_countdown("U04 slide", what, 60U);
            (void)tk_dly_tsk(S3_SETTLE_MS);
            s3_snap(&b);

            s3_tally_t const t = s3_tally(&a, &b, k_cards[c], ovf_a, drv_bc_overflow_count());
            TEST_CHECK("S3-U04", (t.correct >= S3_MIN_CORRECT) && (t.wrong == 0U) && (t.ovf == 0U),
                       "hand slide: >=9/10 correct, 0 wrong, 0 ovf");
        }
    }

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

/* S3-U06 a, b (bench): partial and covered passes give no letter. */
static void s3_u06_bench(uint32_t *p_pass, uint32_t *p_fail)
{
    uint32_t   test_pass_ = *p_pass;
    uint32_t   test_fail_ = *p_fail;
    bc_stats_t a;
    bc_stats_t b;

    bc_task_set_speed_override(S3_BENCH_SPEED_MMPS);

    s3_countdown("U06 prep", "card *A*: next, slide it only HALFWAY and pull it back", 15U);
    s3_snap(&a);
    s3_countdown("U06 half", "halfway in, then back out: 3 times", 30U);
    (void)tk_dly_tsk(S3_SETTLE_MS);
    s3_snap(&b);
    dbg_printf("[S3] half: ok=%lu refused=%lu", (unsigned long)(s3_ok_total(&b) - s3_ok_total(&a)),
               (unsigned long)(s3_fail_total(&b) - s3_fail_total(&a)));
    TEST_CHECK("S3-U06", (s3_ok_total(&b) == s3_ok_total(&a)) && (s3_fail_total(&b) > s3_fail_total(&a)),
               "(a) half pass: no letter, invalid read logged");

    s3_countdown("U06 prep", "cover the LAST 10 mm (stop char) of card *A* with white paper", 20U);
    s3_snap(&a);
    s3_countdown("U06 cover", "slide the covered card fully across: 3 times", 30U);
    (void)tk_dly_tsk(S3_SETTLE_MS);
    s3_snap(&b);
    dbg_printf("[S3] covered: ok=%lu refused=%lu", (unsigned long)(s3_ok_total(&b) - s3_ok_total(&a)),
               (unsigned long)(s3_fail_total(&b) - s3_fail_total(&a)));
    TEST_CHECK("S3-U06", (s3_ok_total(&b) == s3_ok_total(&a)) && (s3_fail_total(&b) > s3_fail_total(&a)),
               "(b) stop char covered: no letter, read refused");

    bc_task_set_speed_override(0);
    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

/* S3-U05: driving over cards with Motion's real speed, both directions. */
static void s3_u05(uint32_t *p_pass, uint32_t *p_fail)
{
    static char const k_cards[6] = { 'A', 'Z', 'F', 'W', 'X', 'Y' };
    uint32_t          test_pass_ = *p_pass;
    uint32_t          test_fail_ = *p_fail;

    bc_task_set_speed_override(0);
    for (uint32_t c = 0U; c < 6U; c++)
    {
        bc_stats_t a;
        bc_stats_t b;
        uint32_t   ovf_a;
        bool       drove = true;
        char       what[DBG_LINE_MAX];

        (void)snprintf(what, sizeof(what), "card *%c* flat, 15 cm ahead of the sensor, bars across",
                       k_cards[c]);
        s3_countdown("U05 prep", what, (c == 0U) ? 60U : 30U);
        s3_snap(&a);
        ovf_a = drv_bc_overflow_count();
        for (uint32_t k = 0U; k < S3_FLOOR_CYCLES; k++)
        {
            drove = s3_drive(true, S3_PASS_CM) && drove;
            drove = s3_drive(false, S3_PASS_CM) && drove;
        }
        s3_snap(&b);

        s3_tally_t const t = s3_tally(&a, &b, k_cards[c], ovf_a, drv_bc_overflow_count());
        TEST_CHECK("S3-U05", drove && (t.correct >= S3_MIN_CORRECT) && (t.wrong == 0U) && (t.ovf == 0U),
                   "drive 5 fwd+5 back: >=9/10 correct, 0 wrong");
    }

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

/* S3-U06 c, d, e (floor): stop on the card, park on it, then recover. */
static void s3_u06_floor(uint32_t *p_pass, uint32_t *p_fail)
{
    uint32_t   test_pass_ = *p_pass;
    uint32_t   test_fail_ = *p_fail;
    bc_stats_t a;
    bc_stats_t b;
    uint32_t   e0;

    bc_task_set_speed_override(0);
    s3_countdown("U06 prep", "card *A* flat, 15 cm ahead of the sensor again", 30U);

    s3_snap(&a);
    (void)s3_drive(true, S3_HALF_PASS_CM);
    s3_snap(&b);
    dbg_printf("[S3] stop mid-card: ok=%lu refused=%lu",
               (unsigned long)(s3_ok_total(&b) - s3_ok_total(&a)),
               (unsigned long)(s3_fail_total(&b) - s3_fail_total(&a)));
    TEST_CHECK("S3-U06", (s3_ok_total(&b) == s3_ok_total(&a)) && (s3_fail_total(&b) > s3_fail_total(&a)),
               "(c) car stops on card: no letter, read refused");

    s3_snap(&a);
    e0 = drv_bc_edge_count();
    s3_countdown("U06 park", "car parked on the card, do not touch", 10U);
    s3_snap(&b);
    TEST_CHECK("S3-U06", (drv_bc_edge_count() == e0) && (b.captures == a.captures),
               "(d) parked 10 s on card: no edges, no read");

    (void)s3_drive(false, S3_HALF_PASS_CM);
    s3_snap(&a);
    (void)s3_drive(true, S3_PASS_CM);
    s3_snap(&b);
    TEST_CHECK("S3-U06", ((b.letters[0] - a.letters[0]) >= 1U)
                         && ((s3_ok_total(&b) - s3_ok_total(&a)) == (b.letters[0] - a.letters[0])),
               "(e) next full pass reads A: recovered");
    (void)motion_stop();

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

void subsystem3_test(void)
{
    bc_stats_t s;
    TEST_INIT();

    dbg_puts("[S3] BC-2A hardware tests: follow the [S3] >>> prompts (.agents/tasks/s3-bc2a-test.md)");
    s3_u03(&test_pass_, &test_fail_);
    s3_u04(&test_pass_, &test_fail_);
    s3_u06_bench(&test_pass_, &test_fail_);
    s3_countdown("FLOOR prep", "move the car OFF the stand onto the floor now", 60U);
    s3_u05(&test_pass_, &test_fail_);
    s3_u06_floor(&test_pass_, &test_fail_);

    s3_snap(&s);
    dbg_printf("[S3] totals: captures=%lu ok=%lu overflow=%lu too_long=%lu incomplete=%lu",
               (unsigned long)s.captures, (unsigned long)s.results[BC_FRAME_OK],
               (unsigned long)s.results[BC_FRAME_OVERFLOW], (unsigned long)s.results[BC_FRAME_TOO_LONG],
               (unsigned long)s.results[BC_FRAME_INCOMPLETE]);
    dbg_printf("[S3] totals: no_symbol=%lu ambiguous=%lu edges=%lu glitches=%lu irqs=%lu",
               (unsigned long)s.results[BC_FRAME_NO_SYMBOL], (unsigned long)s.results[BC_FRAME_AMBIGUOUS],
               (unsigned long)drv_bc_edge_count(), (unsigned long)drv_bc_glitch_count(),
               (unsigned long)drv_bc_irq_count());
    dbg_printf("[S3] totals: ring_ovf=%lu ignored=%lu", (unsigned long)drv_bc_overflow_count(),
               (unsigned long)drv_bc_ignored_count());
    dbg_printf("[S3] totals: max_batch=%lu max_poll=%luus dbg_dropped=%lu",
               (unsigned long)s.max_gaps_per_poll, (unsigned long)s.max_poll_gap_us,
               (unsigned long)dbg_overflow_count());

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
