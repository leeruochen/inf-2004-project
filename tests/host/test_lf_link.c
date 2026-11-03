/*
 *  test_lf_link.c
 *  Host tests for the line-following link to Motion Control's published
 *  state (app_program/subsystem3/lf_link.c): forward odometer and the
 *  junction turn-done detector.  Test ID S3-U11.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Expected values are worked by hand from config/timing.h (5 ms samples,
 *  Motion publishes every 20 ms) and typed in as numbers.  Motion's
 *  publication is modelled as in task_motion.c: the counts change at most
 *  once every 4 samples.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config/timing.h"
#include "app_program/subsystem3/lf_link.h"
#include "test_check.h"

static lf_link_t     g_link;
static int32_t       g_enc[2];
static lf_link_out_t g_out;

static lf_link_out_t step(lf_manoeuvre_t vc_man)
{
    memset(&g_out, 0xA5, sizeof(g_out));    /* every field must be written */
    lf_link_step(&g_link, vc_man, g_enc, &g_out);
    return g_out;
}

static lf_link_out_t move(lf_manoeuvre_t vc_man, int32_t dl, int32_t dr)
{
    g_enc[0] = (int32_t)((uint32_t)g_enc[0] + (uint32_t)dl);
    g_enc[1] = (int32_t)((uint32_t)g_enc[1] + (uint32_t)dr);
    return step(vc_man);
}

/* n samples of a pivot as Motion publishes it: the counts change on every
 * 4th sample only.  Returns the samples whose output was not vc_man. */
static uint32_t pivot(lf_manoeuvre_t vc_man, uint32_t n)
{
    uint32_t other = 0U;

    for (uint32_t i = 0U; i < n; i++)
    {
        lf_link_out_t const o = ((i % 4U) == 0U) ? move(vc_man, -8, 8) : step(vc_man);

        other += (o.manoeuvre != vc_man) ? 1U : 0U;
    }
    return other;
}

static void fresh(void)
{
    lf_link_init(&g_link);
    g_enc[0] = 1000;
    g_enc[1] = -5000;
    (void)step(LF_MAN_NONE);
}

/* Test 1: the numbers this file was written against. */
static void test_constants(void)
{
    CHECK(LF_SAMPLE_PERIOD_MS == 5U);
    CHECK(MOTION_PERIOD_MS == 20U);
    CHECK(LF_T_TURN_STILL_MS == 60U);       /* 12 samples */
    CHECK(LF_T_TURN_MAX_MS == 5000U);       /* 1000 samples */
}

/* Test 2: forward odometer. */
static void test_odometer(void)
{
    lf_link_out_t o;

    /* First sample: nothing travelled, whatever the counts are. */
    lf_link_init(&g_link);
    g_enc[0] = 1000;
    g_enc[1] = -5000;
    o = step(LF_MAN_NONE);
    CHECK(o.fwd_cnt == 0U);
    CHECK(o.manoeuvre == LF_MAN_NONE);

    o = move(LF_MAN_NONE, 12, 12);          /* straight */
    CHECK(o.fwd_cnt == 12U);
    o = move(LF_MAN_NONE, 10, 14);          /* gentle right steer: mean 12 */
    CHECK(o.fwd_cnt == 24U);
    o = move(LF_MAN_NONE, 3, 4);            /* mean 3.5: 3 now, half kept */
    CHECK(o.fwd_cnt == 27U);
    o = move(LF_MAN_NONE, 3, 4);            /* 3.5 + 0.5 carried = 4 */
    CHECK(o.fwd_cnt == 31U);
    o = step(LF_MAN_NONE);                  /* no publication: unchanged */
    CHECK(o.fwd_cnt == 31U);

    /* A pivot (wheels opposite) and reversing add nothing. */
    o = move(LF_MAN_NONE, -20, 20);
    CHECK(o.fwd_cnt == 31U);
    o = move(LF_MAN_NONE, -10, -10);
    CHECK(o.fwd_cnt == 31U);
    o = move(LF_MAN_NONE, 5, 5);
    CHECK(o.fwd_cnt == 36U);

    /* Encoder count wrap (int32 free-running): still +10. */
    lf_link_init(&g_link);
    g_enc[0] = INT32_MAX - 4;
    g_enc[1] = INT32_MAX - 4;
    o = step(LF_MAN_NONE);
    CHECK(o.fwd_cnt == 0U);
    o = move(LF_MAN_NONE, 10, 10);
    CHECK(o.fwd_cnt == 10U);
    CHECK(g_enc[0] < 0);                    /* the counts did wrap */

    /* The odometer itself wraps (uint32), and the core subtracts. */
    g_link.fwd_cnt = UINT32_MAX - 2U;
    o = move(LF_MAN_NONE, 12, 12);
    CHECK(o.fwd_cnt == 9U);
}

/* Test 3: a 90 degree turn is done 60 ms after the wheels stop. */
static void test_turn_done(void)
{
    lf_link_out_t o;

    fresh();
    /* While it pivots (counts change every 20 ms): TURN throughout. */
    CHECK(pivot(LF_MAN_TURN, 200U) == 0U);
    /* Last count change, then still: TURN for 11 samples, NONE on the
     * 12th (60 ms). */
    o = move(LF_MAN_TURN, -8, 8);
    CHECK(o.manoeuvre == LF_MAN_TURN);
    for (uint32_t i = 1U; i <= 11U; i++)
    {
        o = step(LF_MAN_TURN);
        CHECK(o.manoeuvre == LF_MAN_TURN);
    }
    o = step(LF_MAN_TURN);
    CHECK(o.manoeuvre == LF_MAN_NONE);
    /* Stays done while the VC is still in its turn state, even if the
     * wheels move again (the core's own steering post does that). */
    CHECK(pivot(LF_MAN_TURN, 100U) == 100U);
    o = move(LF_MAN_TURN, 12, 12);
    CHECK(o.manoeuvre == LF_MAN_NONE);

    /* The VC leaves the turn state; the next turn is watched afresh. */
    o = step(LF_MAN_NONE);
    CHECK(o.manoeuvre == LF_MAN_NONE);
    CHECK(pivot(LF_MAN_TURN, 40U) == 0U);
}

/* Test 4: the wheels have not started yet: not done. */
static void test_turn_not_started(void)
{
    lf_link_out_t o;
    uint32_t      turn = 0U;

    fresh();
    /* The car was standing; the turn starts but the counts do not move
     * for 500 samples (2.5 s): still a turn. */
    for (uint32_t i = 0U; i < 500U; i++)
    {
        turn += (step(LF_MAN_TURN).manoeuvre == LF_MAN_TURN) ? 1U : 0U;
    }
    CHECK(turn == 500U);
    /* Then it pivots and stops: done 12 samples after the last change. */
    CHECK(pivot(LF_MAN_TURN, 20U) == 0U);
    (void)move(LF_MAN_TURN, -8, 8);
    for (uint32_t i = 1U; i <= 11U; i++)
    {
        (void)step(LF_MAN_TURN);
    }
    o = step(LF_MAN_TURN);
    CHECK(o.manoeuvre == LF_MAN_NONE);
}

/* Test 5: the cap, u-turns, and a change of manoeuvre. */
static void test_turn_cap_and_change(void)
{
    lf_link_out_t o;

    /* Wheels never stop: TURN for 999 samples, NONE at 1000 (5 s). */
    fresh();
    CHECK(pivot(LF_MAN_TURN, 999U) == 0U);
    o = move(LF_MAN_TURN, -8, 8);
    CHECK(o.manoeuvre == LF_MAN_NONE);

    /* U-turn: the same rule, reported as UTURN while it runs. */
    fresh();
    CHECK(pivot(LF_MAN_UTURN, 400U) == 0U);
    o = move(LF_MAN_UTURN, -8, 8);          /* last count change */
    CHECK(o.manoeuvre == LF_MAN_UTURN);
    for (uint32_t i = 0U; i < 11U; i++)
    {
        o = step(LF_MAN_UTURN);
        CHECK(o.manoeuvre == LF_MAN_UTURN);
    }
    o = step(LF_MAN_UTURN);
    CHECK(o.manoeuvre == LF_MAN_NONE);
    /* A new manoeuvre straight after (no NONE between): watched afresh. */
    o = move(LF_MAN_TURN, -8, 8);
    CHECK(o.manoeuvre == LF_MAN_TURN);
    CHECK(pivot(LF_MAN_TURN, 40U) == 0U);
}

/* Test 6: odometer during a turn, and NULL arguments. */
static void test_misc(void)
{
    lf_link_out_t o;

    fresh();
    o = move(LF_MAN_NONE, 100, 100);
    CHECK(o.fwd_cnt == 100U);
    (void)pivot(LF_MAN_TURN, 100U);         /* pivot: no forward travel */
    o = step(LF_MAN_TURN);
    CHECK(o.fwd_cnt == 100U);

    lf_link_init(NULL);
    lf_link_step(NULL, LF_MAN_NONE, g_enc, &g_out);
    lf_link_step(&g_link, LF_MAN_NONE, NULL, &g_out);
    lf_link_step(&g_link, LF_MAN_NONE, g_enc, NULL);
    CHECK(1);                               /* no crash */
}

int main(void)
{
    test_constants();
    test_odometer();
    test_turn_done();
    test_turn_not_started();
    test_turn_cap_and_change();
    test_misc();
    return TEST_RESULT("lf_link");
}
