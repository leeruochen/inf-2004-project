/*
 *  test_bc_edge.c
 *  Host tests for the barcode edge glitch filter (app_program/subsystem3/bc_edge.c).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).  Test ID S3-U07.
 *
 *  The flicker model comes from the first hardware runs (1 Nov 2026,
 *  build/s3-bc2a-run.txt): at a black/white edge the sensor output flips
 *  back and forth with pulses of 20-800 us before it settles.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config/timing.h"              /* BC_GLITCH_US */
#include "app_program/subsystem3/bc_decode.h"
#include "app_program/subsystem3/bc_edge.h"
#include "app_program/subsystem3/bc_frame.h"
#include "bc_fixture.h"
#include "test_check.h"

#define G_US        (2000U)             /* glitch limit used by the hand-worked tests */

/* --------------------------------------------------- hand-worked cases */

/* Tests 1-2: clean edges, settle, the newest edge, same-level events. */
static void test_clean(void)
{
    bc_edge_t f;
    uint32_t  gap = 0xDEADU;

    bc_edge_init(&f, true, 0U, G_US);
    CHECK(bc_edge_last_us(&f) == 0U);
    CHECK(bc_edge_on_change(&f, true, 500U, &gap) == BC_EDGE_IGNORED);     /* no change */
    CHECK(bc_edge_on_change(&f, false, 1000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(gap == 0xDEADU);
    CHECK(bc_edge_last_us(&f) == 1000U);
    CHECK(bc_edge_on_change(&f, false, 1500U, &gap) == BC_EDGE_IGNORED);   /* still low */
    CHECK(bc_edge_on_change(&f, true, 11000U, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == 1000U);                                                   /* 0 .. 1000 */
    CHECK(bc_edge_on_change(&f, false, 41000U, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == 10000U);                                                  /* 1000 .. 11000 */

    /* The last element (11000 .. 41000) needs the pin quiet for G_US. */
    gap = 0xDEADU;
    CHECK(!bc_edge_settle(&f, 41000U + G_US - 1U, &gap));
    CHECK(gap == 0xDEADU);
    CHECK(bc_edge_settle(&f, 41000U + G_US, &gap));
    CHECK(gap == 30000U);
    CHECK(bc_edge_last_us(&f) == 41000U);
    CHECK(!bc_edge_settle(&f, 99000U, &gap));                              /* nothing left */

    /* After a settle the next change is a fresh tentative edge. */
    CHECK(bc_edge_on_change(&f, true, 100000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, false, 120000U, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == (100000U - 41000U));
}

/* Tests 3-5: a glitch inside an element; a flicker burst at an edge; a
 * pulse just at and just over the limit. */
static void test_glitch(void)
{
    bc_edge_t f;
    uint32_t  gap = 0U;

    /* 3: high, a 100 us low pulse, high on: one element 0 .. 20000. */
    bc_edge_init(&f, true, 0U, G_US);
    CHECK(bc_edge_on_change(&f, false, 5000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, 5100U, &gap) == BC_EDGE_GLITCH);
    CHECK(bc_edge_last_us(&f) == 0U);                                      /* back to the start */
    CHECK(bc_edge_on_change(&f, false, 20000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, 30000U, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == 20000U);

    /* 4: a burst at the high -> low edge (crossings at 40000, 40050, 40120,
     * 40400, 40600): the boundary is the last crossing, 40600. */
    CHECK(bc_edge_on_change(&f, false, 40000U, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == 10000U);                                                  /* 20000 .. 30000 */
    CHECK(bc_edge_on_change(&f, true, 40050U, &gap) == BC_EDGE_GLITCH);
    CHECK(bc_edge_on_change(&f, false, 40120U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, 40400U, &gap) == BC_EDGE_GLITCH);
    CHECK(bc_edge_on_change(&f, false, 40600U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, 70000U, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == (40600U - 30000U));                                       /* high incl. the burst */
    CHECK(bc_edge_settle(&f, 70000U + G_US, &gap));
    CHECK(gap == (70000U - 40600U));

    /* 5: limits.  A pulse of G_US - 1 is a glitch; G_US is an element. */
    bc_edge_init(&f, true, 0U, G_US);
    CHECK(bc_edge_on_change(&f, false, 10000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, 10000U + G_US - 1U, &gap) == BC_EDGE_GLITCH);
    CHECK(bc_edge_on_change(&f, false, 20000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, 20000U + G_US, &gap) == BC_EDGE_CONFIRMED);
    CHECK(gap == 20000U);
    CHECK(bc_edge_settle(&f, 20000U + (2U * G_US), &gap));
    CHECK(gap == G_US);                                                    /* kept: >= limit */
}

/* Test 6: the microsecond timer wraps between edges and during settle. */
static void test_wrap(void)
{
    bc_edge_t      f;
    uint32_t       gap = 0U;
    uint32_t const t0 = 0xFFFFF000U;           /* 4096 us before the wrap */

    bc_edge_init(&f, true, t0, G_US);
    CHECK(bc_edge_on_change(&f, false, t0 + 1000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, t0 + 1100U, &gap) == BC_EDGE_GLITCH);
    CHECK(bc_edge_on_change(&f, false, t0 + 3000U, &gap) == BC_EDGE_TENTATIVE);
    CHECK(bc_edge_on_change(&f, true, t0 + 13000U, &gap) == BC_EDGE_CONFIRMED);    /* after the wrap */
    CHECK(gap == 3000U);
    CHECK(!bc_edge_settle(&f, t0 + 13000U + G_US - 1U, &gap));
    CHECK(bc_edge_settle(&f, t0 + 13000U + G_US, &gap));
    CHECK(gap == 10000U);
    CHECK(bc_edge_last_us(&f) == (uint32_t)(t0 + 13000U));
}

/* ------------------------------------------------ whole barcodes + flicker */

/* Deterministic pseudo-random numbers (no libc rand: same on every host). */
static uint32_t g_seed = 12345U;

static uint32_t rnd(uint32_t lo, uint32_t hi)
{
    g_seed = (g_seed * 1103515245U) + 12345U;
    return lo + ((g_seed >> 8) % ((hi - lo) + 1U));
}

/* One pin change, as the handler would see it.  raw[] keeps the plain flip
 * intervals for the control run without the filter. */
typedef struct
{
    bc_edge_t  f;
    bc_frame_t fr_filtered;
    bc_frame_t fr_raw;
    uint32_t   last_raw_us;
    int32_t    speed_mmps;
    uint32_t   max_shift_us;
} rig_t;

static void rig_change(rig_t *p_r, bool level, uint32_t t_us)
{
    uint32_t gap = 0U;

    if (bc_edge_on_change(&p_r->f, level, t_us, &gap) == BC_EDGE_CONFIRMED)
    {
        bc_frame_add(&p_r->fr_filtered, gap, p_r->speed_mmps);
    }
    bc_frame_add(&p_r->fr_raw, t_us - p_r->last_raw_us, p_r->speed_mmps);
    p_r->last_raw_us = t_us;
}

/*
 * One pass over "*c*" at a constant speed.  Every true bar/space boundary
 * gets a flicker burst: 'pairs' extra back-and-forth flips (min_pairs ..
 * max_pairs), each pulse pulse_lo .. pulse_hi us, plus a stale same-level
 * event now and then.  Returns the filtered result; *p_raw_ok tells whether
 * the same flips decoded WITHOUT the filter.
 */
static bc_frame_result_t pass_with_flicker(char c, int reversed, double v_mmps,
                                           uint32_t min_pairs, uint32_t max_pairs,
                                           uint32_t pulse_lo, uint32_t pulse_hi,
                                           char *p_letter, bool *p_raw_ok, uint32_t *p_shift)
{
    double         w[SYM_ELEMS];
    char           sym[4];
    rig_t          r;
    bc_cfg_t       cfg;
    bc_frame_out_t out;
    bc_frame_out_t out_raw;
    fixture_t      fx;
    bool           level = true;                     /* white before the barcode */
    double         x_mm = 0.0;
    uint32_t const t_first = 200000U;                /* 0.2 s of white first */
    uint32_t       t_last = 0U;
    uint32_t       gap = 0U;

    sym_of(c, sym);
    fx = fx_symbol(sym, 3.0, 3.0, v_mmps, v_mmps, 0.0);
    if (reversed)
    {
        fx = fx_reversed(&fx);
    }
    memcpy(w, fx.w_mm, sizeof(w));

    memset(&r, 0, sizeof(r));
    r.speed_mmps = (int32_t)v_mmps;
    bc_edge_init(&r.f, level, 0U, BC_GLITCH_US);
    bc_frame_reset(&r.fr_filtered);
    bc_frame_reset(&r.fr_raw);
    bc_cfg_default(&cfg);

    /* 30 boundaries: before element 0, and after each of the 29. */
    for (int k = 0; k <= SYM_ELEMS; k++)
    {
        uint32_t const t_true = t_first + (uint32_t)lround((x_mm / v_mmps) * 1e6);
        uint32_t const pairs = rnd(min_pairs, max_pairs);
        uint32_t       t = t_true;

        level = !level;
        rig_change(&r, level, t);                    /* first crossing */
        for (uint32_t p = 0U; p < pairs; p++)
        {
            t += rnd(pulse_lo, pulse_hi);
            rig_change(&r, !level, t);               /* flick back ... */
            t += rnd(pulse_lo, pulse_hi);
            rig_change(&r, level, t);                /* ... and forward */
        }
        if (rnd(0U, 3U) == 0U)
        {
            rig_change(&r, level, t + 3U);           /* stale event: same level */
        }
        r.max_shift_us = ((t - t_true) > r.max_shift_us) ? (t - t_true) : r.max_shift_us;
        t_last = t;
        if (k < SYM_ELEMS)
        {
            x_mm += w[k];
        }
    }

    /* Quiet white after the barcode: the last element (last bar) settles. */
    if (bc_edge_settle(&r.f, t_last + BC_GLITCH_US, &gap))
    {
        bc_frame_add(&r.fr_filtered, gap, r.speed_mmps);
    }

    out.letter = NO_CHAR;
    out_raw.letter = NO_CHAR;
    *p_raw_ok = (bc_frame_finish(&r.fr_raw, &cfg, &out_raw) == BC_FRAME_OK) && (out_raw.letter == c);
    *p_shift = r.max_shift_us;
    bc_frame_result_t const res = bc_frame_finish(&r.fr_filtered, &cfg, &out);
    *p_letter = out.letter;
    return res;
}

/* Tests 7-8: every letter, both directions, 100/300/500 mm/s, a flicker
 * burst (1-3 pulse pairs of 20-400 us) at EVERY boundary.  Filtered: the
 * right letter every time.  Control: the same flips without the filter
 * never decode. */
static void test_barcodes_with_flicker(void)
{
    double const speeds[3] = { 100.0, 300.0, 500.0 };
    uint32_t     raw_ok = 0U;
    uint32_t     runs = 0U;
    uint32_t     max_shift = 0U;

    for (int s = 0; s < 3; s++)
    {
        for (char c = 'A'; c <= 'Z'; c++)
        {
            for (int rev = 0; rev < 2; rev++)
            {
                char     letter = NO_CHAR;
                bool     raw = false;
                uint32_t shift = 0U;

                CHECK(pass_with_flicker(c, rev, speeds[s], 1U, 3U, 20U, 400U,
                                        &letter, &raw, &shift) == BC_FRAME_OK);
                CHECK(letter == c);
                raw_ok += raw ? 1U : 0U;
                max_shift = (shift > max_shift) ? shift : max_shift;
                runs++;
            }
        }
    }
    CHECK(raw_ok == 0U);                                 /* the filter is what makes it work */
    printf("  flicker 1-3 pairs, 20-400 us: %u reads OK; without the filter %u of %u; "
           "worst boundary shift %u us\n", (unsigned)runs, (unsigned)raw_ok, (unsigned)runs,
           (unsigned)max_shift);
}

/* Test 9: the worst flicker seen on the car, pulses up to 800 us and up to 6
 * pairs (a burst up to ~9.6 ms long), at 100 mm/s (narrow bar 30 ms): still
 * the right letter.  At 500 mm/s a burst that long eats most of a 6 ms
 * narrow bar; then the read must be refused, never a wrong letter. */
static void test_heavy_flicker(void)
{
    uint32_t refused_fast = 0U;

    for (char c = 'A'; c <= 'Z'; c++)
    {
        for (int rev = 0; rev < 2; rev++)
        {
            char              letter = NO_CHAR;
            bool              raw = false;
            uint32_t          shift = 0U;
            bc_frame_result_t res;

            res = pass_with_flicker(c, rev, 100.0, 1U, 6U, 20U, 800U, &letter, &raw, &shift);
            CHECK(res == BC_FRAME_OK);
            CHECK(letter == c);

            res = pass_with_flicker(c, rev, 500.0, 1U, 6U, 20U, 800U, &letter, &raw, &shift);
            CHECK((res != BC_FRAME_OK) || (letter == c));
            refused_fast += (res != BC_FRAME_OK) ? 1U : 0U;
        }
    }
    printf("  heavy flicker at 500 mm/s: %u of 52 refused, 0 misread\n", (unsigned)refused_fast);
}

int main(void)
{
    test_clean();
    test_glitch();
    test_wrap();
    test_barcodes_with_flicker();
    test_heavy_flicker();
    return TEST_RESULT("bc_edge");
}
