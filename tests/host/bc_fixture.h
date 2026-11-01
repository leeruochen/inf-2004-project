/*
 *  bc_fixture.h
 *  Host-test generator for the barcode tests (S3-U01 test_bc_decode.c,
 *  S3-U02 test_bc_frame.c): Code 39 layout, crossing physics and Motion
 *  Control's published speed, plus the official module samples.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Header-only for the host tests; functions are static inline so a test
 *  that uses only some of them builds under -Wall -Werror.
 */

#ifndef BC_FIXTURE_H
#define BC_FIXTURE_H

#include <math.h>
#include <stdint.h>

#include "config/thresholds.h"      /* ENC_QUAD_MULT */
#include "config/timing.h"          /* MOTION_PERIOD_MS, BC_POLL_PERIOD_MS */
#include "app_program/subsystem3/bc_decode.h"

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
static inline void fx_layout(fixture_t *p_fx, char const *p_pat[3], double narrow_mm, double ratio)
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

static inline fixture_t fx_patterns(char const *p_pat[3], double narrow_mm, double ratio,
                             double v0_mmps, double v1_mmps, double phase_ms);

/* A fixture for a 3-character symbol such as "*A*", crossed at a speed that
 * changes uniformly from v0 at the first edge to v1 at the last. */
static inline fixture_t fx_symbol(char const *p_sym, double narrow_mm, double ratio,
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
static inline void fx_motion(fixture_t *p_fx, double v0_mmps, double v1_mmps, double phase_ms)
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
static inline fixture_t fx_patterns(char const *p_pat[3], double narrow_mm, double ratio,
                             double v0_mmps, double v1_mmps, double phase_ms)
{
    fixture_t fx;

    fx_layout(&fx, p_pat, narrow_mm, ratio);
    fx_motion(&fx, v0_mmps, v1_mmps, phase_ms);
    return fx;
}

/* Layer 2: sensor position at time t (the car cruised at v0 before t = 0). */
static inline double fx_pos_mm(fixture_t const *p_fx, double t_s)
{
    return (t_s < 0.0) ? (p_fx->v0_mmps * t_s)
                       : ((p_fx->v0_mmps * t_s) + (0.5 * p_fx->a_mmps2 * t_s * t_s));
}

/* Layer 2: time at which the sensor reaches position x. */
static inline double fx_time_s(fixture_t const *p_fx, double x_mm)
{
    double const v0 = p_fx->v0_mmps;
    double const a = p_fx->a_mmps2;

    return (fabs(a) < 1e-9) ? (x_mm / v0) : ((-v0 + sqrt((v0 * v0) + (2.0 * a * x_mm))) / a);
}

/* Layer 3: the speed Motion Control has most recently published at time t. */
static inline int32_t fx_motion_speed(fixture_t const *p_fx, double t_s)
{
    double const  k = floor((t_s - p_fx->phase_s) / MOTION_T_S);
    double const  t_end = p_fx->phase_s + (k * MOTION_T_S);
    double const  c_end = floor(fx_pos_mm(p_fx, t_end) / MM_PER_COUNT);
    double const  c_beg = floor(fx_pos_mm(p_fx, t_end - MOTION_T_S) / MM_PER_COUNT);
    int32_t const delta = (int32_t)(c_end - c_beg);

    return (delta * MOTION_WHEEL_CIRC_MM * (int32_t)(1000U / MOTION_PERIOD_MS)) / MOTION_COUNTS_PER_REV;
}

/* All three layers: the {gap_us, speed_mmps} pairs the Barcode task will see. */
static inline void fx_elements(fixture_t const *p_fx, bc_element_t p_out[SYM_ELEMS])
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

/* The same barcode crossed the other way: the widths in mirror order.  The
 * speed profile (v0 at the first edge met, a) is kept. */
static inline fixture_t fx_reversed(fixture_t const *p_fx)
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

static inline void sym_of(char c, char p_sym[4])
{
    p_sym[0] = '*';
    p_sym[1] = c;
    p_sym[2] = '*';
    p_sym[3] = '\0';
}


/* Motion's window phase relative to the first edge: the car can meet a
 * barcode at any point of Motion's 20 ms cycle. */
static double const g_phase_ms[4] = { 0.0, 5.0, 10.0, 15.0 };

/* ===========================================================================
 * The official barcode samples from the module material (S3-U01 test 16,
 * S3-U02).  These
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
static inline fixture_t fx_official(official_t const *p_off, int reversed,
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

#endif /* BC_FIXTURE_H */
