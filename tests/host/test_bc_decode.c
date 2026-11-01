/*
 *  test_bc_decode.c
 *  Host tests for the Code 39 decoder core (app_program/subsystem3/bc_decode.c).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).  Test ID S3-U01.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_program/subsystem3/bc_decode.h"
#include "bc_fixture.h"             /* barcode + physics + Motion generator */
#include "test_check.h"

/* Decode a fixture with the default configuration. */
static bc_result_t fx_decode(fixture_t const *p_fx, char *p_char)
{
    bc_element_t el[SYM_ELEMS];
    bc_cfg_t     cfg;

    fx_elements(p_fx, el);
    bc_cfg_default(&cfg);
    return bc_decode(el, SYM_ELEMS, &cfg, p_char);
}

/* Test 1-2: gap time x speed -> width, by hand arithmetic. */
static void test_gap_to_width(void)
{
    /* 10 000 us x 300 mm/s = 3.000 mm = 3000 um. */
    CHECK(bc_gap_to_width_um(10000U, 300) == 3000U);
    /* Largest IF-09 gap x largest IF-08 speed: 1 048 575 x 511 = 535 821 825,
     * / 1000 = 535 821 um (truncated), and no 32-bit overflow. */
    CHECK(bc_gap_to_width_um(0xFFFFFU, 511) == 535821U);
    /* No usable speed gives no width. */
    CHECK(bc_gap_to_width_um(10000U, 0) == 0U);
    CHECK(bc_gap_to_width_um(10000U, -300) == 0U);
}

/* Tests 3-4: every letter A-Z at a constant 300 mm/s, wide:narrow 2:1 and
 * 3:1.  The decoder returns the letter; it does not decide any movement. */
static void test_constant_speed(void)
{
    double const ratios[2] = { 2.0, 3.0 };

    for (int r = 0; r < 2; r++)
    {
        for (char c = 'A'; c <= 'Z'; c++)
        {
            char sym[4];
            char ch = NO_CHAR;

            sym_of(c, sym);
            fixture_t const fx = fx_symbol(sym, 5.0, ratios[r], 300.0, 300.0, 7.0);
            CHECK(fx_decode(&fx, &ch) == BC_OK);
            CHECK(ch == c);
        }
    }
}

/* Test 5: the same barcode at 100, 300 and 500 mm/s reads the same. */
static void test_speed_invariance(void)
{
    double const speeds[3] = { 100.0, 300.0, 500.0 };

    for (int s = 0; s < 3; s++)
    {
        char            ch = NO_CHAR;
        fixture_t const fx = fx_symbol("*B*", 5.0, 2.0, speeds[s], speeds[s], 7.0);

        CHECK(fx_decode(&fx, &ch) == BC_OK);
        CHECK(ch == 'B');
    }
}


/* Tests 6-7: speeding up 200 -> 400 mm/s and slowing down 400 -> 200 mm/s
 * (about 316 mm/s^2 over a 190 mm symbol), A-Z, 2:1 bars, every phase,
 * crossed forwards and backwards. */
static void test_varying_speed(void)
{
    double const v_from[2] = { 200.0, 400.0 };
    double const v_to[2] = { 400.0, 200.0 };

    for (int d = 0; d < 2; d++)
    {
        for (char c = 'A'; c <= 'Z'; c++)
        {
            for (int p = 0; p < 4; p++)
            {
                char sym[4];
                char ch = NO_CHAR;

                sym_of(c, sym);
                fixture_t const fx = fx_symbol(sym, 5.0, 2.0, v_from[d], v_to[d], g_phase_ms[p]);
                fixture_t const rev = fx_reversed(&fx);
                CHECK(fx_decode(&fx, &ch) == BC_OK);
                CHECK(ch == c);
                ch = NO_CHAR;
                CHECK(fx_decode(&rev, &ch) == BC_OK);
                CHECK(ch == c);
            }
        }
    }
}

/* Test 8 (control): the exact gaps of test 6, but every element paired with
 * one constant speed, i.e. decoding from time alone.  It must fail, which
 * shows tests 6-7 pass because of the per-element speed, not despite it. */
static void test_time_only_fails(void)
{
    bc_cfg_t cfg;

    bc_cfg_default(&cfg);
    for (char c = 'A'; c <= 'D'; c++)
    {
        char         sym[4];
        bc_element_t el[SYM_ELEMS];
        char         ch = NO_CHAR;

        sym_of(c, sym);
        fixture_t const fx = fx_symbol(sym, 5.0, 2.0, 200.0, 400.0, 0.0);
        fx_elements(&fx, el);
        for (int k = 0; k < SYM_ELEMS; k++)
        {
            el[k].speed_mmps = 300;
        }
        CHECK(bc_decode(el, SYM_ELEMS, &cfg, &ch) != BC_OK);
        CHECK(ch == NO_CHAR);
    }
}

/* Test 9: anything but exactly 29 elements, or a NULL pointer. */
static void test_reject_count(void)
{
    bc_element_t    el[SYM_ELEMS + 1];
    bc_cfg_t        cfg;
    char            ch = NO_CHAR;
    fixture_t const fx = fx_symbol("*A*", 5.0, 2.0, 300.0, 300.0, 7.0);

    fx_elements(&fx, el);
    el[SYM_ELEMS] = el[SYM_ELEMS - 2];      /* a 30th element: a narrow space */
    bc_cfg_default(&cfg);
    CHECK(bc_decode(el, SYM_ELEMS - 1, &cfg, &ch) == BC_ERR_COUNT);
    CHECK(bc_decode(el, SYM_ELEMS + 1, &cfg, &ch) == BC_ERR_COUNT);
    CHECK(bc_decode(NULL, SYM_ELEMS, &cfg, &ch) == BC_ERR_COUNT);
    CHECK(bc_decode(el, SYM_ELEMS, NULL, &ch) == BC_ERR_COUNT);
    CHECK(bc_decode(el, SYM_ELEMS, &cfg, NULL) == BC_ERR_COUNT);
    CHECK(ch == NO_CHAR);
}

/* Test 10: one element below BC_MIN_SPEED_MMPS (50) is a stall, wherever it
 * is (first bar, a gap between characters, last bar); exactly 50 is not. */
static void test_reject_stall(void)
{
    int const       where[3] = { 0, 9, SYM_ELEMS - 1 };
    int32_t const   slow[3] = { 0, -100, 49 };
    bc_cfg_t        cfg;
    fixture_t const fx = fx_symbol("*A*", 5.0, 2.0, 300.0, 300.0, 7.0);

    bc_cfg_default(&cfg);
    for (int w = 0; w < 3; w++)
    {
        for (int v = 0; v < 3; v++)
        {
            bc_element_t el[SYM_ELEMS];
            char         ch = NO_CHAR;

            fx_elements(&fx, el);
            el[where[w]].speed_mmps = slow[v];
            CHECK(bc_decode(el, SYM_ELEMS, &cfg, &ch) == BC_ERR_STALL);
            CHECK(ch == NO_CHAR);
        }
    }

    /* At exactly the minimum the read stands: every element at 50 mm/s. */
    {
        char            ch = NO_CHAR;
        fixture_t const fx50 = fx_symbol("*A*", 5.0, 2.0, 50.0, 50.0, 7.0);
        bc_element_t    el[SYM_ELEMS];

        fx_elements(&fx50, el);
        for (int k = 0; k < SYM_ELEMS; k++)
        {
            el[k].speed_mmps = 50;
        }
        CHECK(bc_decode(el, SYM_ELEMS, &cfg, &ch) == BC_OK);
        CHECK(ch == 'A');
    }
}

/* Tests 11-12: the ends must both be '*' in the same direction; the middle
 * character must be a letter. */
static void test_reject_chars(void)
{
    /* 11: a letter at an end, or '*' at one end and '*' mirrored (which
     * reads as 'P') at the other: neither forwards nor backwards. */
    char const *const bad_ends[4] = { "AA*", "*AA", "*AP", "PA*" };
    /* 12: valid Code 39, valid ends, but not A-Z: a digit, a symbol, a
     * symbol whose 3 wide elements are all spaces, and '*' itself. */
    char const *const not_letter[4] = { "*0*", "*-*", "*$*", "***" };

    for (int i = 0; i < 4; i++)
    {
        char            ch = NO_CHAR;
        fixture_t const fx_end = fx_symbol(bad_ends[i], 5.0, 2.0, 300.0, 300.0, 7.0);
        fixture_t const fx_nl = fx_symbol(not_letter[i], 5.0, 2.0, 300.0, 300.0, 7.0);
        fixture_t const fx_nl_rev = fx_reversed(&fx_nl);

        CHECK(fx_decode(&fx_end, &ch) == BC_ERR_START_STOP);
        CHECK(fx_decode(&fx_nl, &ch) == BC_ERR_NOT_LETTER);
        CHECK(fx_decode(&fx_nl_rev, &ch) == BC_ERR_NOT_LETTER);
        CHECK(ch == NO_CHAR);
    }
}

/* Test 13: a middle character with 2 or with 4 wide elements of 9. */
static void test_reject_pattern(void)
{
    char const *two[3] = { code39_pattern('*'), "WnnnnWnnn", code39_pattern('*') };
    char const *four[3] = { code39_pattern('*'), "WnnnnWnWW", code39_pattern('*') };
    char        ch = NO_CHAR;

    fixture_t const fx2 = fx_patterns(two, 5.0, 2.0, 300.0, 300.0, 7.0);
    CHECK(fx_decode(&fx2, &ch) == BC_ERR_PATTERN);
    fixture_t const fx4 = fx_patterns(four, 5.0, 2.0, 300.0, 300.0, 7.0);
    CHECK(fx_decode(&fx4, &ch) == BC_ERR_PATTERN);
    CHECK(ch == NO_CHAR);
}

/* Test 14: every letter crossed backwards reads as the same letter (module
 * clarification: both scan directions give the same character), 2:1 and 3:1,
 * at 300 mm/s.  Backwards with a changing speed is in tests 6-7. */
static void test_reversed(void)
{
    double const ratios[2] = { 2.0, 3.0 };

    for (int r = 0; r < 2; r++)
    {
        for (char c = 'A'; c <= 'Z'; c++)
        {
            char sym[4];
            char ch = NO_CHAR;

            sym_of(c, sym);
            fixture_t const fx = fx_symbol(sym, 5.0, ratios[r], 300.0, 300.0, 7.0);
            fixture_t const rev = fx_reversed(&fx);
            CHECK(fx_decode(&rev, &ch) == BC_OK);
            CHECK(ch == c);
        }
    }
}

/* Test 15: the threshold comes from bc_cfg_t.  One narrow element of the 'A'
 * (element 11) stretched to 1.6 x narrow: wide at ratio 150 (so 4 wide, a
 * pattern error), still narrow at ratio 170 (a valid read). */
static void test_cfg_ratio(void)
{
    bc_element_t    el[SYM_ELEMS];
    bc_cfg_t        cfg;
    char            ch = NO_CHAR;
    fixture_t const fx = fx_symbol("*A*", 5.0, 3.0, 300.0, 300.0, 7.0);

    fx_elements(&fx, el);
    el[11].gap_us = (el[11].gap_us * 16U) / 10U;
    bc_cfg_default(&cfg);
    CHECK(cfg.wide_ratio_x100 == 150U);
    CHECK(bc_decode(el, SYM_ELEMS, &cfg, &ch) == BC_ERR_PATTERN);
    CHECK(ch == NO_CHAR);
    cfg.wide_ratio_x100 = 170U;
    CHECK(bc_decode(el, SYM_ELEMS, &cfg, &ch) == BC_OK);
    CHECK(ch == 'A');
}

static void test_official_samples(void)
{
    for (int s = 0; s < OFFICIAL_COUNT; s++)
    {
        official_t const *p_off = &g_official[s];
        int const         fails = g_failures;

        /* The sample data itself: two widths only, 3.0 mm and 9.0 mm. */
        {
            fixture_t const fx = fx_official(p_off, 0, 300.0, 300.0, 0.0);

            for (int k = 0; k < SYM_ELEMS; k++)
            {
                CHECK((fabs(fx.w_mm[k] - 3.0) < 0.01) || (fabs(fx.w_mm[k] - 9.0) < 0.01));
            }
        }

        /* Forwards and backwards, every speed profile and phase: the letter. */
        for (int p = 0; p < 5; p++)
        {
            for (int ph = 0; ph < 4; ph++)
            {
                char            ch = NO_CHAR;
                fixture_t const fwd = fx_official(p_off, 0, g_prof_from[p], g_prof_to[p], g_phase_ms[ph]);
                fixture_t const rev = fx_official(p_off, 1, g_prof_from[p], g_prof_to[p], g_phase_ms[ph]);

                CHECK(fx_decode(&fwd, &ch) == BC_OK);
                CHECK(ch == p_off->letter);
                ch = NO_CHAR;
                CHECK(fx_decode(&rev, &ch) == BC_OK);
                CHECK(ch == p_off->letter);
            }
        }

        if (g_failures != fails)
        {
            printf("  official sample \"%s\" failed\n", p_off->p_name);
        }
    }
}

int main(void)
{
    test_gap_to_width();
    test_constant_speed();
    test_speed_invariance();
    test_varying_speed();
    test_time_only_fails();
    test_reject_count();
    test_reject_stall();
    test_reject_chars();
    test_reject_pattern();
    test_reversed();
    test_cfg_ratio();
    test_official_samples();
    return TEST_RESULT("bc_decode");
}
