/*
 *  test_bc_frame.c
 *  Host tests for barcode capture framing (app_program/subsystem3/bc_frame.c).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).  Test ID S3-U02.
 *
 *  A capture is simulated end to end with the S3-U01 generator (bc_fixture.h:
 *  barcode layout, crossing physics, Motion's published speed) and the
 *  Barcode task's poll rule, which BC-2A.2 must implement exactly:
 *    every BC_POLL_PERIOD_MS: read Motion's speed ONCE; pop EVERY gap
 *    waiting in the edge ring and pair each with that one speed; then, with
 *    the ring empty, ask bc_frame_ended(last edge time, now).
 *  A scheduling delay is a run of skipped polls followed by one late poll,
 *  so several gaps are popped together and share one speed reading.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_program/subsystem3/bc_decode.h"
#include "app_program/subsystem3/bc_frame.h"
#include "bc_fixture.h"
#include "test_check.h"

/* ===========================================================================
 * Capture simulator
 * ========================================================================= */

#define CAP_MAX             (40)            /* elements in a simulated capture */
#define QUIET_MM            (4.0)           /* sheet edge to first/last bar (spec photo) */
#define START_GAP_US        (0xFFFFFU)      /* first edge after a long quiet: saturated */
#define POLL_MS             ((double)BC_POLL_PERIOD_MS)
#define TIMEOUT_MS          ((double)BC_CAPTURE_TIMEOUT_MS)

/* Physical elements of one pass in crossing order: optional extras, the
 * 29-element symbol, optional extras. */
typedef struct
{
    double w_mm[CAP_MAX];
    int    n;
} layout_t;

typedef struct
{
    double   v0_mmps;           /* speed at the first edge */
    double   v1_mmps;           /* speed at the last element's end */
    double   phase_ms;          /* Motion window boundary vs the first edge */
    double   poll_phase_ms;     /* first poll after the first edge */
    double   block_from_ms;     /* task delayed: no poll in [from, from + len) ... */
    double   block_len_ms;      /* ... then one late poll at from + len */
    uint32_t t0_us;             /* hal_time_us() at the first edge (wrap tests) */
    int      edges;             /* edges that happen (partial pass), -1 = all */
} sim_cfg_t;

typedef struct
{
    bc_frame_result_t res;
    bc_frame_out_t    out;
    bool              ended;        /* bc_frame_ended() became true */
    double            end_ms;       /* poll time when it did */
    double            last_edge_ms; /* time of the last edge popped */
    uint32_t          max_per_poll; /* most gaps popped by one poll */
    uint32_t          widest_us;    /* frame's widest element at the end */
    uint32_t          seen;         /* frame's elements seen at the end */
} sim_out_t;

static sim_cfg_t sim_default(double v0_mmps, double v1_mmps, double phase_ms)
{
    sim_cfg_t c;

    memset(&c, 0, sizeof(c));
    c.v0_mmps = v0_mmps;
    c.v1_mmps = v1_mmps;
    c.phase_ms = phase_ms;
    c.poll_phase_ms = 0.7;
    c.block_from_ms = -1.0;
    c.block_len_ms = 0.0;
    c.t0_us = 0x10000000U;
    c.edges = -1;
    return c;
}

/* Layout: n_lead extras, the symbol's 29 widths, n_trail extras. */
static layout_t layout_make(double const *p_lead, int n_lead, double const p_sym[SYM_ELEMS],
                            double const *p_trail, int n_trail)
{
    layout_t l;

    l.n = 0;
    for (int k = 0; k < n_lead; k++)
    {
        l.w_mm[l.n++] = p_lead[k];
    }
    for (int k = 0; k < SYM_ELEMS; k++)
    {
        l.w_mm[l.n++] = p_sym[k];
    }
    for (int k = 0; k < n_trail; k++)
    {
        l.w_mm[l.n++] = p_trail[k];
    }
    return l;
}

/* Widths of "*c*" at 3 mm narrow, 3:1 (the module's barcode), either way. */
static void sym_widths(char c, int reversed, double p_w[SYM_ELEMS])
{
    char      sym[4];
    fixture_t fx;

    sym_of(c, sym);
    fx = fx_symbol(sym, 3.0, 3.0, 300.0, 300.0, 0.0);
    if (reversed)
    {
        fx = fx_reversed(&fx);
    }
    memcpy(p_w, fx.w_mm, sizeof(fx.w_mm));
}

/* Paper-edge extras: what the sensor sees if the track around the sheet
 * reads dark.  Lead: [white quiet zone]; trail: [white quiet zone], then the
 * dark track that follows has no further edge. */
static double const k_quiet[1] = { QUIET_MM };

static sim_out_t sim_run(layout_t const *p_l, sim_cfg_t const *p_c)
{
    sim_out_t  o;
    fixture_t  fx;
    bc_frame_t fr;
    bc_cfg_t   cfg;
    double     t_ms[CAP_MAX + 1];
    uint32_t   e_us[CAP_MAX + 1];
    double     len_mm = 0.0;
    double     x_mm = 0.0;
    int        n_edges = p_l->n + 1;
    int        next = 0;
    bool       late_done = false;

    memset(&o, 0, sizeof(o));
    memset(&fx, 0, sizeof(fx));
    for (int k = 0; k < p_l->n; k++)
    {
        len_mm += p_l->w_mm[k];
    }
    fx.v0_mmps = p_c->v0_mmps;
    fx.a_mmps2 = ((p_c->v1_mmps * p_c->v1_mmps) - (p_c->v0_mmps * p_c->v0_mmps)) / (2.0 * len_mm);
    fx.phase_s = p_c->phase_ms / 1000.0;

    /* Edge k is at the end of element k-1; edge 0 starts the capture. */
    t_ms[0] = 0.0;
    for (int k = 1; k <= p_l->n; k++)
    {
        x_mm += p_l->w_mm[k - 1];
        t_ms[k] = fx_time_s(&fx, x_mm) * 1000.0;
    }
    for (int k = 0; k <= p_l->n; k++)
    {
        e_us[k] = p_c->t0_us + (uint32_t)lround(t_ms[k] * 1000.0);    /* wraps like the timer */
    }
    if ((p_c->edges >= 0) && (p_c->edges < n_edges))
    {
        n_edges = p_c->edges;
    }

    bc_frame_reset(&fr);
    for (long p = 0; p < 2000000L; p++)
    {
        double   tp = p_c->poll_phase_ms + ((double)p * POLL_MS);
        uint32_t popped = 0U;

        if ((tp >= p_c->block_from_ms) && (tp < (p_c->block_from_ms + p_c->block_len_ms)))
        {
            if (late_done)
            {
                continue;
            }
            tp = p_c->block_from_ms + p_c->block_len_ms;   /* the one late poll */
            late_done = true;
        }

        /* One speed reading per poll, shared by every gap popped in it. */
        int32_t const v = fx_motion_speed(&fx, tp / 1000.0);

        while ((next < n_edges) && (t_ms[next] <= tp))
        {
            uint32_t const gap = (next == 0) ? START_GAP_US : (e_us[next] - e_us[next - 1]);

            bc_frame_add(&fr, gap, v);
            o.last_edge_ms = t_ms[next];
            next++;
            popped++;
        }
        if (popped > o.max_per_poll)
        {
            o.max_per_poll = popped;
        }
        if (next > 0)
        {
            uint32_t const now_us = p_c->t0_us + (uint32_t)lround(tp * 1000.0);

            if (bc_frame_ended(&fr, e_us[next - 1], now_us))
            {
                o.ended = true;
                o.end_ms = tp;
                break;
            }
        }
    }

    bc_cfg_default(&cfg);
    o.out.letter = NO_CHAR;
    o.res = bc_frame_finish(&fr, &cfg, &o.out);
    o.widest_us = fr.widest_us;
    o.seen = fr.seen;
    /* Edges still in the future when the capture ended = it ended too early. */
    if (next < n_edges)
    {
        o.ended = false;
    }
    return o;
}

/* The end rule, recomputed from the generator's own edge times (not from
 * the frame): the limit in ms the capture should have waited for. */
static double expected_limit_ms(sim_out_t const *p_o)
{
    double limit = TIMEOUT_MS;

    if (p_o->seen >= BC_SYMBOL_ELEMENTS)
    {
        double const t_end = ((double)BC_T_END_WIDEST_MULT * (double)p_o->widest_us) / 1000.0;

        limit = (t_end < limit) ? t_end : limit;
    }
    return limit;
}

/* A read that ended normally, at the first poll after the limit. */
static void check_ended_on_time(sim_out_t const *p_o)
{
    double const quiet = p_o->end_ms - p_o->last_edge_ms;
    double const limit = expected_limit_ms(p_o);

    CHECK(p_o->ended);
    CHECK(quiet >= (limit - 0.001));
    CHECK(quiet < (limit + POLL_MS + 0.001));
}

/* ===========================================================================
 * Test 1: bc_frame_ended() by hand-worked numbers, including the wrap.
 * ========================================================================= */
static void test_end_rule(void)
{
    bc_frame_t fr;

    CHECK(BC_CAPTURE_TIMEOUT_MS == 1000U);      /* numbers below assume these */
    CHECK(BC_T_END_WIDEST_MULT == 3U);

    /* Start gap only: no element yet, so the timeout (1 000 000 us) applies. */
    bc_frame_reset(&fr);
    bc_frame_add(&fr, START_GAP_US, 300);
    CHECK(fr.count == 0U);
    CHECK(fr.seen == 0U);
    CHECK(fr.widest_us == 0U);                  /* the start gap is not an element */
    CHECK(!bc_frame_ended(&fr, 5000U, 5000U + 999999U));
    CHECK(bc_frame_ended(&fr, 5000U, 5000U + 1000000U));

    /* 28 elements of 10 ms and one of 30 ms: still below 29, timeout only. */
    for (int k = 0; k < 27; k++)
    {
        bc_frame_add(&fr, 10000U, 300);
    }
    bc_frame_add(&fr, 30000U, 300);
    CHECK(fr.seen == 28U);
    CHECK(!bc_frame_ended(&fr, 0U, 90000U));    /* T_end would be 90 ms: not yet used */
    CHECK(!bc_frame_ended(&fr, 0U, 999999U));
    CHECK(bc_frame_ended(&fr, 0U, 1000000U));

    /* The 29th element arms T_end = 3 x 30 000 = 90 000 us. */
    bc_frame_add(&fr, 10000U, 300);
    CHECK(fr.seen == 29U);
    CHECK(fr.widest_us == 30000U);
    CHECK(!bc_frame_ended(&fr, 0U, 89999U));
    CHECK(bc_frame_ended(&fr, 0U, 90000U));

    /* A widest element of 400 ms: T_end 1.2 s, capped by the 1 s timeout. */
    bc_frame_add(&fr, 400000U, 300);
    CHECK(!bc_frame_ended(&fr, 0U, 999999U));
    CHECK(bc_frame_ended(&fr, 0U, 1000000U));

    /* Wrap: last edge 0x100 us before 2^32, now past it. */
    bc_frame_reset(&fr);
    bc_frame_add(&fr, START_GAP_US, 300);
    for (int k = 0; k < 29; k++)
    {
        bc_frame_add(&fr, 10000U, 300);             /* T_end = 30 000 us */
    }
    CHECK(!bc_frame_ended(&fr, 0xFFFFFF00U, 0xFFFFFF00U + 29999U));    /* 0x74A7 after wrap */
    CHECK(bc_frame_ended(&fr, 0xFFFFFF00U, 0xFFFFFF00U + 30000U));
    CHECK(!bc_frame_ended(&fr, 0xFFFFFF00U, 0xFFFFFFFFU));             /* 255 us */
    CHECK(!bc_frame_ended(&fr, 0xFFFFFF00U, 0x00000100U));             /* 512 us */

    /* No frame: timeout only. */
    CHECK(!bc_frame_ended(NULL, 0U, 999999U));
    CHECK(bc_frame_ended(NULL, 0U, 1000000U));
}

/* ===========================================================================
 * Test 2: storage, magnitude, the element cap and the result order.
 * ========================================================================= */
static void test_storage(void)
{
    bc_frame_t     fr;
    bc_cfg_t       cfg;
    bc_frame_out_t out;

    bc_cfg_default(&cfg);
    bc_frame_reset(&fr);
    bc_frame_add(&fr, 777U, 300);               /* start gap: dropped */
    bc_frame_add(&fr, 10000U, -250);            /* reverse drive: magnitude */
    bc_frame_add(&fr, 10000U, INT32_MIN);       /* never from IF-08: no usable speed */
    CHECK(fr.count == 2U);
    CHECK(fr.elem[0].gap_us == 10000U);
    CHECK(fr.elem[0].speed_mmps == 250);
    CHECK(fr.elem[1].speed_mmps == 0);

    /* 35 elements offered: 33 stored, 35 seen, the widest still tracked. */
    bc_frame_reset(&fr);
    bc_frame_add(&fr, START_GAP_US, 300);
    for (int k = 0; k < 34; k++)
    {
        bc_frame_add(&fr, 10000U, 300);
    }
    bc_frame_add(&fr, 50000U, 300);
    CHECK(fr.count == BC_FRAME_MAX_ELEMENTS);
    CHECK(fr.seen == 35U);
    CHECK(fr.widest_us == 50000U);
    CHECK(bc_frame_finish(&fr, &cfg, &out) == BC_FRAME_TOO_LONG);

    /* Order: overflow before too-long before incomplete. */
    bc_frame_mark_overflow(&fr);
    CHECK(bc_frame_finish(&fr, &cfg, &out) == BC_FRAME_OVERFLOW);
    bc_frame_reset(&fr);
    CHECK(!fr.overflow);
    CHECK(bc_frame_finish(&fr, &cfg, &out) == BC_FRAME_INCOMPLETE);
    CHECK(out.letter == '?');

    /* NULL pointers. */
    bc_frame_reset(NULL);
    bc_frame_add(NULL, 1U, 1);
    bc_frame_mark_overflow(NULL);
    CHECK(bc_frame_finish(NULL, &cfg, &out) == BC_FRAME_INCOMPLETE);
    CHECK(bc_frame_finish(&fr, NULL, &out) == BC_FRAME_INCOMPLETE);
    CHECK(bc_frame_finish(&fr, &cfg, NULL) == BC_FRAME_INCOMPLETE);
}

/* ===========================================================================
 * Tests 3-4: every letter, both directions, 29/30/31-element captures
 * (sheet edge before, after, both), constant 300 mm/s and the 200 <-> 400
 * mm/s ramps at every Motion window phase.  Each must read the letter from
 * the right window and end at the first poll after T_end.
 * ========================================================================= */
static void test_letters_and_paper_edges(void)
{
    int const          n_lead[4] = { 0, 1, 0, 1 };
    int const          n_trail[4] = { 0, 0, 1, 1 };
    double const       v_from[3] = { 300.0, 200.0, 400.0 };
    double const       v_to[3] = { 300.0, 400.0, 200.0 };

    for (char c = 'A'; c <= 'Z'; c++)
    {
        for (int rev = 0; rev < 2; rev++)
        {
            double w[SYM_ELEMS];

            sym_widths(c, rev, w);
            for (int e = 0; e < 4; e++)
            {
                layout_t const l = layout_make(k_quiet, n_lead[e], w, k_quiet, n_trail[e]);

                for (int s = 0; s < 3; s++)
                {
                    for (int ph = 0; ph < 4; ph++)
                    {
                        sim_cfg_t const c0 = sim_default(v_from[s], v_to[s], g_phase_ms[ph]);
                        sim_out_t const o = sim_run(&l, &c0);

                        CHECK(o.res == BC_FRAME_OK);
                        CHECK(o.out.letter == c);
                        CHECK(o.out.window == (uint32_t)n_lead[e]);
                        CHECK(o.out.windows_ok == 1U);
                        check_ended_on_time(&o);
                    }
                }
            }
        }
    }
}

/* Test 5: the official module samples, 3 mm / 9 mm, both directions, with
 * the sheet edge both sides (31 elements), every speed profile and phase. */
static void test_official(void)
{
    for (int s = 0; s < OFFICIAL_COUNT; s++)
    {
        int const fails = g_failures;

        for (int rev = 0; rev < 2; rev++)
        {
            for (int p = 0; p < 5; p++)
            {
                for (int ph = 0; ph < 4; ph++)
                {
                    fixture_t const fx = fx_official(&g_official[s], rev, 300.0, 300.0, 0.0);
                    layout_t const  l = layout_make(k_quiet, 1, fx.w_mm, k_quiet, 1);
                    sim_cfg_t const c0 = sim_default(g_prof_from[p], g_prof_to[p], g_phase_ms[ph]);
                    sim_out_t const o = sim_run(&l, &c0);

                    CHECK(o.res == BC_FRAME_OK);
                    CHECK(o.out.letter == g_official[s].letter);
                    CHECK(o.out.window == 1U);
                    check_ended_on_time(&o);
                }
            }
        }
        if (g_failures != fails)
        {
            printf("  official sample \"%s\" failed\n", g_official[s].p_name);
        }
    }
}

/* Test 6: slow crossings.  A symbol starts narrow bar, wide space: if T_end
 * were armed before 29 elements, "3 x widest so far" would end the capture
 * during that space, and the slower the car the closer the call.  At 60 mm/s
 * every read must succeed.  At exactly BC_MIN_SPEED_MMPS (50) Motion's
 * whole-count speed reads 46 or 50 mm/s (11.9 counts per 20 ms), so a read
 * may be refused as a stall, but never misread. */
static void test_slowest(void)
{
    for (char c = 'A'; c <= 'Z'; c++)
    {
        for (int rev = 0; rev < 2; rev++)
        {
            double w[SYM_ELEMS];

            sym_widths(c, rev, w);
            layout_t const l = layout_make(k_quiet, 1, w, k_quiet, 1);
            {
                sim_cfg_t const c0 = sim_default(60.0, 60.0, 7.0);
                sim_out_t const o = sim_run(&l, &c0);

                CHECK(o.res == BC_FRAME_OK);
                CHECK(o.out.letter == c);
                check_ended_on_time(&o);
            }
            {
                sim_cfg_t const c0 = sim_default(50.0, 50.0, 7.0);
                sim_out_t const o = sim_run(&l, &c0);

                CHECK(o.ended);
                CHECK(((o.res == BC_FRAME_OK) && (o.out.letter == c))
                      || ((o.res == BC_FRAME_NO_SYMBOL) && (o.out.detail == BC_ERR_STALL)));
            }
        }
    }
}

/* ===========================================================================
 * Test 7: ambiguity.  31 elements in which window 0 is *D* forwards and
 * window 2 is *I* backwards (found by an exhaustive search over A-Z, both
 * directions, offsets 1-4).  Both windows really decode; the capture must be
 * refused, not read as either letter.
 * ========================================================================= */
static void merge_sym(char p_m[], int offset, char c, int reversed)
{
    char s[SYM_ELEMS];
    char const *p_star = code39_pattern('*');
    char const *p_c = code39_pattern(c);

    for (int e = 0; e < 9; e++)
    {
        s[e] = p_star[e];
        s[10 + e] = p_c[e];
        s[20 + e] = p_star[e];
    }
    s[9] = '?';
    s[19] = '?';
    for (int k = 0; k < SYM_ELEMS; k++)
    {
        char const v = reversed ? s[SYM_ELEMS - 1 - k] : s[k];

        if (v != '?')
        {
            CHECK((p_m[offset + k] == '?') || (p_m[offset + k] == v));
            p_m[offset + k] = v;
        }
    }
}

static void test_ambiguous(void)
{
    char     m[31];
    double   w[31];
    bc_cfg_t cfg;

    memset(m, '?', sizeof(m));
    merge_sym(m, 0, 'D', 0);
    merge_sym(m, 2, 'I', 1);
    for (int k = 0; k < 31; k++)
    {
        w[k] = (m[k] == 'W') ? 9.0 : 3.0;       /* unconstrained gaps: narrow */
    }

    /* Each window on its own is a valid symbol. */
    bc_cfg_default(&cfg);
    for (int win = 0; win <= 2; win += 2)
    {
        bc_element_t el[SYM_ELEMS];
        char         ch = NO_CHAR;

        for (int k = 0; k < SYM_ELEMS; k++)
        {
            el[k].gap_us = (uint32_t)lround(w[win + k] / 300.0 * 1e6);
            el[k].speed_mmps = 300;
        }
        CHECK(bc_decode(el, SYM_ELEMS, &cfg, &ch) == BC_OK);
        CHECK(ch == ((win == 0) ? 'D' : 'I'));
    }

    /* As one capture: refused. */
    {
        layout_t        l;
        sim_cfg_t const c0 = sim_default(300.0, 300.0, 7.0);

        l.n = 31;
        memcpy(l.w_mm, w, sizeof(w));
        sim_out_t const o = sim_run(&l, &c0);
        CHECK(o.res == BC_FRAME_AMBIGUOUS);
        CHECK(o.out.windows_ok == 2U);
        CHECK(o.out.letter == '?');
    }
}

/* ===========================================================================
 * Test 8: partial passes.  The car stops (no more edges) after k elements:
 * no T_end yet, so the capture ends on the 1 s timeout as INCOMPLETE.
 * A pass with extras that reaches 29 elements but not the whole symbol is
 * NO_SYMBOL.  Never a letter.
 * ========================================================================= */
static void test_partial(void)
{
    int const k_elems[5] = { 0, 1, 10, 27, 28 };
    double    w[SYM_ELEMS];

    sym_widths('A', 0, w);
    for (int i = 0; i < 5; i++)
    {
        layout_t const l = layout_make(k_quiet, 0, w, k_quiet, 0);
        sim_cfg_t      c0 = sim_default(300.0, 300.0, 7.0);

        c0.edges = k_elems[i] + 1;              /* the start edge + k element ends */
        sim_out_t const o = sim_run(&l, &c0);
        CHECK(o.res == BC_FRAME_INCOMPLETE);
        CHECK(o.out.letter == '?');
        CHECK(o.ended);
        CHECK((o.end_ms - o.last_edge_ms) >= (TIMEOUT_MS - 0.001));
        CHECK((o.end_ms - o.last_edge_ms) < (TIMEOUT_MS + POLL_MS + 0.001));
    }

    /* Two leading extras + 27 of the symbol = 29 elements, no full symbol. */
    {
        double const   lead[2] = { 6.0, QUIET_MM };
        layout_t const l = layout_make(lead, 2, w, k_quiet, 0);
        sim_cfg_t      c0 = sim_default(300.0, 300.0, 7.0);

        c0.edges = 2 + 27 + 1;
        sim_out_t const o = sim_run(&l, &c0);
        CHECK(o.res == BC_FRAME_NO_SYMBOL);
        CHECK(o.out.letter == '?');
    }
}

/* Test 9: element count limits: 2 extras each side (33) still reads; a
 * fifth extra (34) is TOO_LONG even though the symbol is inside. */
static void test_too_long(void)
{
    double const lead[3] = { 6.0, 5.0, QUIET_MM };
    double const trail[2] = { QUIET_MM, 6.0 };
    double       w[SYM_ELEMS];

    sym_widths('Q', 1, w);
    {
        layout_t const  l = layout_make(&lead[1], 2, w, trail, 2);
        sim_cfg_t const c0 = sim_default(300.0, 300.0, 7.0);
        sim_out_t const o = sim_run(&l, &c0);

        CHECK(l.n == 33);
        CHECK(o.res == BC_FRAME_OK);
        CHECK(o.out.letter == 'Q');
        CHECK(o.out.window == 2U);
    }
    {
        layout_t const  l = layout_make(lead, 3, w, trail, 2);
        sim_cfg_t const c0 = sim_default(300.0, 300.0, 7.0);
        sim_out_t const o = sim_run(&l, &c0);

        CHECK(l.n == 34);
        CHECK(o.res == BC_FRAME_TOO_LONG);
    }
}

/* ===========================================================================
 * Test 10: overflow indication and invalid speeds.
 * ========================================================================= */
static void test_overflow_and_speed(void)
{
    bc_cfg_t       cfg;
    bc_frame_t     fr;
    bc_frame_out_t out;
    double         w[SYM_ELEMS];
    double         x_mm = 0.0;
    double         t_prev = 0.0;

    bc_cfg_default(&cfg);
    sym_widths('K', 0, w);

    /* Build one clean 31-element capture by hand at 300 mm/s, 1 mm = 3333 us. */
    {
        layout_t const l = layout_make(k_quiet, 1, w, k_quiet, 1);

        bc_frame_reset(&fr);
        bc_frame_add(&fr, START_GAP_US, 300);
        for (int k = 0; k < l.n; k++)
        {
            double const t = (x_mm + l.w_mm[k]) / 300.0 * 1e6;

            bc_frame_add(&fr, (uint32_t)lround(t - t_prev), 300);
            x_mm += l.w_mm[k];
            t_prev = (double)lround(t);
        }
    }
    CHECK(bc_frame_finish(&fr, &cfg, &out) == BC_FRAME_OK);
    CHECK(out.letter == 'K');

    /* The ring dropped edges: refused although the elements would decode. */
    {
        bc_frame_t f2 = fr;

        bc_frame_mark_overflow(&f2);
        out.letter = NO_CHAR;
        CHECK(bc_frame_finish(&f2, &cfg, &out) == BC_FRAME_OVERFLOW);
        CHECK(out.letter == '?');
    }

    /* A stalled element inside the symbol (window 1 = elements 1..29):
     * 0, below the 50 mm/s minimum, and INT32_MIN.  Reported as STALL. */
    {
        int32_t const bad[3] = { 0, 49, INT32_MIN };

        for (int b = 0; b < 3; b++)
        {
            bc_frame_t f2;

            bc_frame_reset(&f2);
            bc_frame_add(&f2, START_GAP_US, 300);
            for (uint32_t k = 0U; k < fr.count; k++)
            {
                bc_frame_add(&f2, fr.elem[k].gap_us, (k == 15U) ? bad[b] : 300);
            }
            CHECK(bc_frame_finish(&f2, &cfg, &out) == BC_FRAME_NO_SYMBOL);
            CHECK(out.detail == BC_ERR_STALL);
            CHECK(out.letter == '?');
        }
    }

    /* A stalled speed on the sheet-edge element outside the symbol window
     * does not matter. */
    {
        bc_frame_t f2;

        bc_frame_reset(&f2);
        bc_frame_add(&f2, START_GAP_US, 300);
        for (uint32_t k = 0U; k < fr.count; k++)
        {
            bc_frame_add(&f2, fr.elem[k].gap_us, (k == 0U) ? 0 : 300);
        }
        CHECK(bc_frame_finish(&f2, &cfg, &out) == BC_FRAME_OK);
        CHECK(out.letter == 'K');
    }

    /* Reverse drive: Motion publishes negative speeds; magnitudes decode. */
    {
        bc_frame_t f2;

        bc_frame_reset(&f2);
        bc_frame_add(&f2, START_GAP_US, -300);
        for (uint32_t k = 0U; k < fr.count; k++)
        {
            bc_frame_add(&f2, fr.elem[k].gap_us, -300);
        }
        CHECK(bc_frame_finish(&f2, &cfg, &out) == BC_FRAME_OK);
        CHECK(out.letter == 'K');
    }
}

/* ===========================================================================
 * Test 11: noise.  A 0.2 mm glitch splits one element of the symbol in
 * three (2 extra edges, 31 elements).  Every letter, both directions, every
 * element position: the capture may be refused, but must never read as a
 * different letter.
 * ========================================================================= */
static void test_glitch(void)
{
    uint32_t refused = 0U;
    uint32_t runs = 0U;

    for (char c = 'A'; c <= 'Z'; c++)
    {
        for (int rev = 0; rev < 2; rev++)
        {
            double w[SYM_ELEMS];

            sym_widths(c, rev, w);
            for (int g = 0; g < SYM_ELEMS; g++)
            {
                layout_t  l;
                sim_cfg_t const c0 = sim_default(300.0, 300.0, 7.0);

                l.n = 0;
                for (int k = 0; k < SYM_ELEMS; k++)
                {
                    if (k == g)
                    {
                        double const half = (w[k] - 0.2) / 2.0;

                        l.w_mm[l.n++] = half;
                        l.w_mm[l.n++] = 0.2;
                        l.w_mm[l.n++] = half;
                    }
                    else
                    {
                        l.w_mm[l.n++] = w[k];
                    }
                }
                sim_out_t const o = sim_run(&l, &c0);
                CHECK((o.res != BC_FRAME_OK) || (o.out.letter == c));
                refused += (o.res != BC_FRAME_OK) ? 1U : 0U;
                runs++;
            }
        }
    }
    printf("  glitch: %u of %u captures refused, 0 misread\n", (unsigned)refused, (unsigned)runs);
}

/* ===========================================================================
 * Test 12: several gaps waiting at one poll (the task delayed).  All share
 * the one speed read at that late poll.  At a constant speed that speed is
 * right for all of them; on a ramp it is late by up to the delay.
 * Asserted: constant 300 mm/s with a 200 ms delay anywhere (about half the
 * symbol popped at once), and both ramps with a 50 ms delay at several
 * points, every letter, both directions.  Reported, not asserted: the
 * longest delay at which every ramp read still passes.
 * ========================================================================= */
static void test_poll_delay(void)
{
    double const at_ms[4] = { 5.0, 120.0, 240.0, 400.0 };

    /* Constant speed, 200 ms delay. */
    for (char c = 'A'; c <= 'Z'; c++)
    {
        for (int rev = 0; rev < 2; rev++)
        {
            double w[SYM_ELEMS];

            sym_widths(c, rev, w);
            layout_t const l = layout_make(k_quiet, 1, w, k_quiet, 1);
            for (int a = 0; a < 4; a++)
            {
                sim_cfg_t c0 = sim_default(300.0, 300.0, 7.0);

                c0.block_from_ms = at_ms[a];
                c0.block_len_ms = 200.0;
                sim_out_t const o = sim_run(&l, &c0);
                CHECK(o.res == BC_FRAME_OK);
                CHECK(o.out.letter == c);
                CHECK(o.max_per_poll >= ((a < 3) ? 10U : 2U));   /* it really happened */
            }
        }
    }

    /* Ramps, 50 ms delay. */
    for (int d = 0; d < 2; d++)
    {
        double const v0 = (d == 0) ? 200.0 : 400.0;
        double const v1 = (d == 0) ? 400.0 : 200.0;

        for (char c = 'A'; c <= 'Z'; c++)
        {
            for (int rev = 0; rev < 2; rev++)
            {
                double w[SYM_ELEMS];

                sym_widths(c, rev, w);
                layout_t const l = layout_make(k_quiet, 1, w, k_quiet, 1);
                for (int a = 0; a < 4; a++)
                {
                    sim_cfg_t c0 = sim_default(v0, v1, 3.0);

                    c0.block_from_ms = at_ms[a] * 0.8;
                    c0.block_len_ms = 50.0;
                    sim_out_t const o = sim_run(&l, &c0);
                    CHECK(o.res == BC_FRAME_OK);
                    CHECK(o.out.letter == c);
                    CHECK(o.max_per_poll >= 2U);
                }
            }
        }
    }

    /* Characterisation: longest delay (10 ms steps) with every ramp read OK. */
    {
        double ok_upto = 0.0;

        for (double blk = 10.0; blk <= 400.0; blk += 10.0)
        {
            bool all_ok = true;

            for (int d = 0; (d < 2) && all_ok; d++)
            {
                for (char c = 'A'; (c <= 'Z') && all_ok; c++)
                {
                    for (int rev = 0; (rev < 2) && all_ok; rev++)
                    {
                        for (int a = 0; (a < 4) && all_ok; a++)
                        {
                            double    w[SYM_ELEMS];
                            sim_cfg_t c0 = sim_default((d == 0) ? 200.0 : 400.0,
                                                       (d == 0) ? 400.0 : 200.0, 3.0);

                            sym_widths(c, rev, w);
                            layout_t const l = layout_make(k_quiet, 1, w, k_quiet, 1);
                            c0.block_from_ms = at_ms[a] * 0.8;
                            c0.block_len_ms = blk;
                            sim_out_t const o = sim_run(&l, &c0);
                            all_ok = (o.res == BC_FRAME_OK) && (o.out.letter == c);
                        }
                    }
                }
            }
            if (!all_ok)
            {
                break;
            }
            ok_upto = blk;
        }
        printf("  poll delay on 200<->400 mm/s ramps: every read OK up to %.0f ms\n", ok_upto);
    }
}

/* Test 13: the microsecond timer wraps (a) during the symbol, (b) in the
 * quiet time after the last edge, while the task waits for T_end, and
 * (c) at the first edge. */
static void test_timer_wrap(void)
{
    double         w[SYM_ELEMS];

    sym_widths('M', 1, w);
    layout_t const l = layout_make(k_quiet, 1, w, k_quiet, 1);
    sim_cfg_t      c0 = sim_default(300.0, 300.0, 7.0);
    sim_out_t      o = sim_run(&l, &c0);       /* dry run: when is the last edge? */
    uint32_t const last_us = (uint32_t)lround(o.last_edge_ms * 1000.0);
    uint32_t const t0[3] =
    {
        0U - (last_us / 2U),                    /* (a) */
        0U - (last_us + 10000U),                /* (b) wrap 10 ms after the last edge */
        0xFFFFFFFFU                             /* (c) */
    };

    for (int i = 0; i < 3; i++)
    {
        c0.t0_us = t0[i];
        o = sim_run(&l, &c0);
        CHECK(o.res == BC_FRAME_OK);
        CHECK(o.out.letter == 'M');
        check_ended_on_time(&o);
    }
}

/* Test 14: what the sensor really sees.  Ink spread and the sensor's spot
 * size and threshold make every bar look wider (and every space narrower)
 * by the same amount, or the reverse.  Bars +-0.5 mm (narrow bar 2.5-3.5 mm,
 * wide space 8.5-9.5 mm: up to 3.8 x a narrow bar), every letter, both
 * directions, constant and both ramps, with no sheet edge (29 elements: the
 * first element is the thin bar, so an early T_end would cut the wide space
 * after it) and with the sheet edge both sides (31). */
static void test_bar_growth(void)
{
    double const grow[2] = { 0.5, -0.5 };
    double const v_from[3] = { 300.0, 200.0, 400.0 };
    double const v_to[3] = { 300.0, 400.0, 200.0 };

    for (int gi = 0; gi < 2; gi++)
    {
        for (char c = 'A'; c <= 'Z'; c++)
        {
            for (int rev = 0; rev < 2; rev++)
            {
                double w[SYM_ELEMS];

                sym_widths(c, rev, w);
                for (int k = 0; k < SYM_ELEMS; k++)
                {
                    /* Element 0 is a bar either way (29 elements, bar first). */
                    w[k] += ((k % 2) == 0) ? grow[gi] : -grow[gi];
                }
                for (int edge = 0; edge < 2; edge++)
                {
                    layout_t const l = layout_make(k_quiet, edge, w, k_quiet, edge);

                    for (int sp = 0; sp < 3; sp++)
                    {
                        sim_cfg_t const c0 = sim_default(v_from[sp], v_to[sp], 3.0);
                        sim_out_t const o = sim_run(&l, &c0);

                        CHECK(o.res == BC_FRAME_OK);
                        CHECK(o.out.letter == c);
                        check_ended_on_time(&o);
                    }
                }
            }
        }
    }
}

int main(void)
{
    test_end_rule();
    test_storage();
    test_letters_and_paper_edges();
    test_official();
    test_slowest();
    test_ambiguous();
    test_partial();
    test_too_long();
    test_overflow_and_speed();
    test_glitch();
    test_poll_delay();
    test_timer_wrap();
    test_bar_growth();
    return TEST_RESULT("bc_frame");
}
