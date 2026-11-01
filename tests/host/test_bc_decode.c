/*
 *  test_bc_decode.c
 *  Host tests for the Code 39 decoder core (app_program/subsystem3/bc_decode.c).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).  Test ID S3-U01.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config/thresholds.h"      /* ENC_QUAD_MULT */
#include "config/timing.h"          /* MOTION_PERIOD_MS */
#include "app_program/subsystem3/bc_decode.h"
#include "test_check.h"

/* ===========================================================================
 * Test generator.  The decoder only ever sees what the car will give it: one
 * {gap_us, speed_mmps} pair per bar or space.  Those pairs are produced here
 * from three independent layers, never by the decoder's own arithmetic:
 *   1. Barcode: physical element widths (mm) from Code 39 patterns.
 *   2. Physics: the sensor crosses the barcode at constant acceleration,
 *      x(t) = v0 t + a t^2 / 2; each edge time is rounded to 1 us, as the
 *      GP27 edge interrupt will record it.
 *   3. Motion Control: every MOTION_PERIOD_MS it publishes the mean wheel
 *      speed over the window just ended, from whole encoder counts
 *      (task_motion.c).  Each gap is paired with the speed published most
 *      recently when the Barcode task reads that gap: SHELL_LAG_S after the
 *      gap ended (one BC_POLL_PERIOD_MS poll).
 * ========================================================================= */

#define SYM_ELEMS       (29)        /* '*' + character + '*', 2 gaps between */
#define CHAR_ELEMS      (9)

/* Code 39 element patterns, 9 elements, bar first, alternating bar/space;
 * W = wide, n = narrow.  Measured from the symbols *A* .. *Z*, *0*, *-* and
 * *$* made by the generator the module refers to (barcode.tec-it.com/en/Code39,
 * 1 Nov 2026), so they are a source independent of the hex table in
 * bc_decode.c.  The official samples (test 16) agree for A, F, W, X, Y, Z. */
static char const *code39_pattern(char c)
{
    static char const *const k_letter[26] =
    {
        "WnnnnWnnW", "nnWnnWnnW", "WnWnnWnnn", "nnnnWWnnW", "WnnnWWnnn",     /* A-E */
        "nnWnWWnnn", "nnnnnWWnW", "WnnnnWWnn", "nnWnnWWnn", "nnnnWWWnn",     /* F-J */
        "WnnnnnnWW", "nnWnnnnWW", "WnWnnnnWn", "nnnnWnnWW", "WnnnWnnWn",     /* K-O */
        "nnWnWnnWn", "nnnnnnWWW", "WnnnnnWWn", "nnWnnnWWn", "nnnnWnWWn",     /* P-T */
        "WWnnnnnnW", "nWWnnnnnW", "WWWnnnnnn", "nWnnWnnnW", "WWnnWnnnn",     /* U-Y */
        "nWWnWnnnn"                                                          /* Z */
    };
    char const *p_pat = NULL;

    if ((c >= 'A') && (c <= 'Z'))
    {
        p_pat = k_letter[c - 'A'];
    }
    else
    {
        switch (c)
        {
            case '*': p_pat = "nWnnWnWnn"; break;
            case '0': p_pat = "nnnWWnWnn"; break;
            case '-': p_pat = "nWnnnnWnW"; break;
            case '$': p_pat = "nWnWnWnnn"; break;   /* the 3 wide are all spaces */
            default:  break;
        }
    }
    return p_pat;
}

/* Motion Control's speed estimate (task_motion.c:59-62 and :488): 616 CPR x
 * ENC_QUAD_MULT counts per 207 mm wheel turn, counted over MOTION_PERIOD_MS. */
#define MOTION_COUNTS_PER_REV   (616 * (int32_t)ENC_QUAD_MULT)
#define MOTION_WHEEL_CIRC_MM    (207)
#define MOTION_T_S              ((double)MOTION_PERIOD_MS / 1000.0)
#define MM_PER_COUNT            ((double)MOTION_WHEEL_CIRC_MM / (double)MOTION_COUNTS_PER_REV)
#define SHELL_LAG_S             ((double)BC_POLL_PERIOD_MS / 1000.0)

typedef struct
{
    double w_mm[SYM_ELEMS];     /* layer 1: element widths in crossing order */
    double v0_mmps;             /* layer 2: speed at the first edge */
    double a_mmps2;             /*          constant acceleration */
    double phase_s;             /* layer 3: Motion window boundary vs first edge */
} fixture_t;

/* Layer 1 from three 9-element patterns; the inter-character gaps are narrow. */
static void fx_layout(fixture_t *p_fx, char const *p_pat[3], double narrow_mm, double ratio)
{
    int k = 0;

    for (int c = 0; c < 3; c++)
    {
        for (int e = 0; e < CHAR_ELEMS; e++)
        {
            p_fx->w_mm[k++] = (p_pat[c][e] == 'W') ? (narrow_mm * ratio) : narrow_mm;
        }
        if (c < 2)
        {
            p_fx->w_mm[k++] = narrow_mm;
        }
    }
}

static fixture_t fx_patterns(char const *p_pat[3], double narrow_mm, double ratio,
                             double v0_mmps, double v1_mmps, double phase_ms);

/* A fixture for a 3-character symbol such as "*A*", crossed at a speed that
 * changes uniformly from v0 at the first edge to v1 at the last. */
static fixture_t fx_symbol(char const *p_sym, double narrow_mm, double ratio,
                           double v0_mmps, double v1_mmps, double phase_ms)
{
    char const *pat[3];

    for (int c = 0; c < 3; c++)
    {
        pat[c] = code39_pattern(p_sym[c]);
    }
    return fx_patterns(pat, narrow_mm, ratio, v0_mmps, v1_mmps, phase_ms);
}

/* Layers 2-3 set-up for widths already in p_fx->w_mm: speed v0 at the first
 * edge, changing uniformly to v1 at the last. */
static void fx_motion(fixture_t *p_fx, double v0_mmps, double v1_mmps, double phase_ms)
{
    double len_mm = 0.0;

    for (int k = 0; k < SYM_ELEMS; k++)
    {
        len_mm += p_fx->w_mm[k];
    }
    p_fx->v0_mmps = v0_mmps;
    p_fx->a_mmps2 = ((v1_mmps * v1_mmps) - (v0_mmps * v0_mmps)) / (2.0 * len_mm);
    p_fx->phase_s = phase_ms / 1000.0;
}

/* The same from three explicit patterns (for patterns that are not Code 39). */
static fixture_t fx_patterns(char const *p_pat[3], double narrow_mm, double ratio,
                             double v0_mmps, double v1_mmps, double phase_ms)
{
    fixture_t fx;

    fx_layout(&fx, p_pat, narrow_mm, ratio);
    fx_motion(&fx, v0_mmps, v1_mmps, phase_ms);
    return fx;
}

/* Layer 2: sensor position at time t (the car cruised at v0 before t = 0). */
static double fx_pos_mm(fixture_t const *p_fx, double t_s)
{
    return (t_s < 0.0) ? (p_fx->v0_mmps * t_s)
                       : ((p_fx->v0_mmps * t_s) + (0.5 * p_fx->a_mmps2 * t_s * t_s));
}

/* Layer 2: time at which the sensor reaches position x. */
static double fx_time_s(fixture_t const *p_fx, double x_mm)
{
    double const v0 = p_fx->v0_mmps;
    double const a = p_fx->a_mmps2;

    return (fabs(a) < 1e-9) ? (x_mm / v0) : ((-v0 + sqrt((v0 * v0) + (2.0 * a * x_mm))) / a);
}

/* Layer 3: the speed Motion Control has most recently published at time t. */
static int32_t fx_motion_speed(fixture_t const *p_fx, double t_s)
{
    double const  k = floor((t_s - p_fx->phase_s) / MOTION_T_S);
    double const  t_end = p_fx->phase_s + (k * MOTION_T_S);
    double const  c_end = floor(fx_pos_mm(p_fx, t_end) / MM_PER_COUNT);
    double const  c_beg = floor(fx_pos_mm(p_fx, t_end - MOTION_T_S) / MM_PER_COUNT);
    int32_t const delta = (int32_t)(c_end - c_beg);

    return (delta * MOTION_WHEEL_CIRC_MM * (int32_t)(1000U / MOTION_PERIOD_MS)) / MOTION_COUNTS_PER_REV;
}

/* All three layers: the {gap_us, speed_mmps} pairs the Barcode task will see. */
static void fx_elements(fixture_t const *p_fx, bc_element_t p_out[SYM_ELEMS])
{
    double x_mm = 0.0;
    long   us_prev = 0L;

    for (int k = 0; k < SYM_ELEMS; k++)
    {
        x_mm += p_fx->w_mm[k];
        double const t_s = fx_time_s(p_fx, x_mm);
        long const   us = lround(t_s * 1e6);

        p_out[k].gap_us = (uint32_t)(us - us_prev);
        p_out[k].speed_mmps = fx_motion_speed(p_fx, t_s + SHELL_LAG_S);
        us_prev = us;
    }
}

/* Decode a fixture with the default configuration. */
static bc_result_t fx_decode(fixture_t const *p_fx, char *p_char)
{
    bc_element_t el[SYM_ELEMS];
    bc_cfg_t     cfg;

    fx_elements(p_fx, el);
    bc_cfg_default(&cfg);
    return bc_decode(el, SYM_ELEMS, &cfg, p_char);
}

/* The same barcode crossed the other way: the widths in mirror order.  The
 * speed profile (v0 at the first edge met, a) is kept. */
static fixture_t fx_reversed(fixture_t const *p_fx)
{
    fixture_t r = *p_fx;

    for (int k = 0; k < SYM_ELEMS; k++)
    {
        r.w_mm[k] = p_fx->w_mm[SYM_ELEMS - 1 - k];
    }
    return r;
}

/* "Untouched" marker for the output character. */
#define NO_CHAR         ('#')

static void sym_of(char c, char p_sym[4])
{
    p_sym[0] = '*';
    p_sym[1] = c;
    p_sym[2] = '*';
    p_sym[3] = '\0';
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

/* Motion's window phase relative to the first edge: the car can meet a
 * barcode at any point of Motion's 20 ms cycle. */
static double const g_phase_ms[4] = { 0.0, 5.0, 10.0, 15.0 };

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

/* ===========================================================================
 * Test 16: the official barcode samples from the module material.  These
 * widths were measured from the files themselves, so they are a second
 * source independent of code39_pattern() above.  Each row is the 29 elements
 * in printed order, bar first, in the file's own units:
 *   - Barcode Sample.pdf: bar rectangles of the drawing on pages 1-2 (*A*)
 *     and on pages 3-4 (*Z*), PDF units, 3 decimals;
 *   - BARCODE.pptx: the vector pictures ppt/media/image1.emf (*A*),
 *     image2.emf (*F*), image3.emf (*Z*), image4.emf (*W*) and image5.emf
 *     (*X*), EMF logical units; and the bitmap image6.png (*Y*), pixels along
 *     a row clear of its watermark text.
 * Barcode & Line Specification.pdf shows the printed symbol 141 mm long, and
 * every symbol is 47 narrow widths long (20 narrow + 9 wide x 3), so each row
 * is scaled to 141 mm: narrow 3.0 mm, wide 9.0 mm.
 * Every one is a valid letter and must read as that letter both ways.
 * ========================================================================= */
#define OFFICIAL_SYMBOL_MM      (141.0)

typedef struct
{
    char const *p_name;
    char        letter;         /* what the sample is labelled and must read as */
    double      w[SYM_ELEMS];
} official_t;

static official_t const g_official[] =
{
    { "PDF p1-2 *A*", 'A',
      { 90.581, 271.743, 90.581, 90.579, 271.740, 90.580, 271.740, 90.590, 90.581,
        90.579,
        271.740, 90.580, 90.581, 90.579, 90.581, 271.739, 90.581, 90.579, 271.740,
        90.590,
        90.581, 271.739, 90.581, 90.579, 271.740, 90.580, 271.740, 90.590, 90.581 } },
    { "PDF p3-4 *Z*", 'Z',
      { 90.581, 271.743, 90.581, 90.579, 271.740, 90.580, 271.740, 90.590, 90.581,
        90.579,
        90.581, 271.739, 271.740, 90.580, 271.740, 90.580, 90.581, 90.589, 90.581,
        90.579,
        90.581, 271.739, 90.581, 90.579, 271.740, 90.580, 271.740, 90.590, 90.581 } },
    { "PPTX image1 *A*", 'A',
      { 117, 351, 117, 117, 351, 117, 351, 117, 117,
        117,
        351, 117, 117, 117, 117, 351, 117, 117, 351,
        117,
        117, 351, 117, 117, 351, 117, 351, 117, 117 } },
    { "PPTX image2 *F*", 'F',
      { 117, 351, 117, 117, 351, 117, 351, 117, 117,
        117,
        117, 117, 351, 117, 351, 351, 117, 117, 117,
        117,
        117, 351, 117, 117, 351, 117, 351, 117, 117 } },
    { "PPTX image3 *Z*", 'Z',
      { 117, 351, 117, 117, 351, 117, 351, 117, 117,
        117,
        117, 351, 351, 117, 351, 117, 117, 117, 117,
        117,
        117, 351, 117, 117, 351, 117, 351, 117, 117 } },
    { "PPTX image4 *W*", 'W',
      { 26, 78, 26, 26, 78, 26, 78, 26, 26,
        26,
        78, 78, 78, 26, 26, 26, 26, 26, 26,
        26,
        26, 78, 26, 26, 78, 26, 78, 26, 26 } },
    { "PPTX image5 *X*", 'X',
      { 22, 66, 22, 22, 66, 22, 66, 22, 22,
        22,
        22, 66, 22, 22, 66, 22, 22, 22, 66,
        22,
        22, 66, 22, 22, 66, 22, 66, 22, 22 } },
    { "PPTX image6 *Y*", 'Y',
      { 9, 27, 9, 9, 27, 9, 27, 9, 9,
        9,
        27, 27, 9, 9, 27, 9, 9, 9, 9,
        9,
        9, 27, 9, 9, 27, 9, 27, 9, 9 } },
};
#define OFFICIAL_COUNT  ((int)(sizeof(g_official) / sizeof(g_official[0])))

/* Speed profiles: constant 100, 300, 500 mm/s and both ramps of tests 6-7. */
static double const g_prof_from[5] = { 100.0, 300.0, 500.0, 200.0, 400.0 };
static double const g_prof_to[5] = { 100.0, 300.0, 500.0, 400.0, 200.0 };

/* One official sample, scaled to 141 mm, crossed forwards or backwards. */
static fixture_t fx_official(official_t const *p_off, int reversed,
                             double v0_mmps, double v1_mmps, double phase_ms)
{
    fixture_t fx;
    double    sum = 0.0;

    for (int k = 0; k < SYM_ELEMS; k++)
    {
        sum += p_off->w[k];
    }
    for (int k = 0; k < SYM_ELEMS; k++)
    {
        int const src = reversed ? (SYM_ELEMS - 1 - k) : k;

        fx.w_mm[k] = p_off->w[src] * (OFFICIAL_SYMBOL_MM / sum);
    }
    fx_motion(&fx, v0_mmps, v1_mmps, phase_ms);
    return fx;
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
