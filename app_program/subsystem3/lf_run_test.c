/*
 *  lf_run_test.c
 *  Subsystem 3 -- Line-Following task test on the track (S3-U12, S3-U13),
 *  APP_TEST only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel and pure project headers only (rule R1).
 *
 *  What it shows: the real Line-Following task driving the car on the
 *  track, with the real Vehicle Controller and Motion, at the build's
 *  cruise speed (MOTION_SPEED_CRUISE_CMPS; the default bring-up speed set
 *  gives 200 mm/s).  The test task (priority 11) never reads the ADC: it
 *  reads the task's status (lf_task_get_status), Motion's state (IF-08)
 *  and the VC's state, and plays the operator's remote START / STOP /
 *  RESET through MBF_NAV_IN as Telemetry would.
 *
 *  The motors are live (on the Robo Pico, USB alone powers them).  Every
 *  drive is bounded: the test sends STOP when the wheel travel or the time
 *  reaches the drive's limit, or when the VC hands the car to obstacle
 *  avoidance; Safety's collision brake stays active.  Between drives the
 *  VC is READY (motors stopped) and the operator places the car by hand.
 *
 *  LF-3.3 (S3-U13): LF2_STRAIGHT_ONLY runs the static poses and then the
 *  straight drive LF2_STRAIGHT_RUNS times, nothing else: straight tracking
 *  is proven on the car before recovery, junctions or curves.  Every drive
 *  is recorded every 20 ms into RAM and printed after the car has stopped:
 *  the console ring holds 2 KB and drops lines when full, so 20 ms lines
 *  printed live would lose data and load the system during the drive.
 *
 *  Guide: .agents/tasks/s3-lf33-test.md.  Expected values come from
 *  config/timing.h, lf_core.h and design.md 3.2, typed in: 5 ms samples
 *  (200 per second), one steering post per 20 ms Motion period,
 *  LINE_LOST -> 120 ms stop -> 2 s search -> TRACK_LOST (2120 ms),
 *  steer > 0 = steer right (data_repr.h).
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/subsystem2.h"   /* motion_get_state, vc_get_system_state, VC_SYS_* */
#include "config/thresholds.h"          /* MOTION_SPEED_CRUISE_CMPS, ENC_*, WHEEL_CIRC_MM */
#include "config/timing.h"              /* NAV_MBF_SEND_TMO_MS */
#include "lf_core.h"                    /* lf_norm, LF_DARK_MIN_PM, LF_LIGHT_MAX_PM */
#include "lf_run_test.h"
#include "task_lf.h"

#define LF2_SECOND_MS           (1000U)
#define LF2_POLL_MS             (10U)
/** A "[LF] d" line every this many polls (100 ms) while driving. */
#define LF2_LOG_EVERY_POLLS     (10U)
/** Seconds to place the car before a drive or a static measurement. */
#define LF2_PLACE_S             (10U)
/** Static measurement time per pose. */
#define LF2_STATIC_MS           (3000U)
/** Samples in 1 s at 5 ms, +-5 for the start and end of the window. */
#define LF2_RELEASES_1S_MIN     (195U)
#define LF2_RELEASES_1S_MAX     (205U)
/** Steering post count window on the straight (ms after START). */
#define LF2_WIN_FROM_MS         (500U)
#define LF2_WIN_TO_MS           (2500U)
/** LINE_LOST entry to TRACK_LOST: 120 ms stop + 2000 ms search, +-1 sample
 *  and the kernel tick. */
#define LF2_LOST_TO_TRACK_MIN   (2110U)
#define LF2_LOST_TO_TRACK_MAX   (2135U)
/** First steering from the start pose (CENTRE on the line body): Kp 24 x
 *  (1000 - 900) / 500 = +4 (lf_core.h): any right steering passes. */
#define LF2_STEER_BODY_MIN      (1)
/** "Settled": CENTRE first feels the right edge (below 950 per mille, not
 *  on white) within this wheel travel of START.  The host model takes 168
 *  mm from the line's middle and 374 mm from a poor placement. */
#define LF2_SETTLE_PM_LO        (250U)
#define LF2_SETTLE_PM_HI        (950U)
#define LF2_SETTLE_MAX_MM       (500U)
/** Stable tracking: at least this share of polls ON_LINE with CENTRE on
 *  the line (LF_LIGHT_MAX_PM or darker).  The edge is only ~2.4 mm wide, so
 *  a band inside it is not a fair measure: CENTRE sits at ~880 pm when
 *  settled, on the black side by design. */
#define LF2_BAND_MIN_PCT        (90U)
/** Pose A: CENTRE on the line's middle must read at least this, or the boot
 *  calibration is too high for the 900 pm target (see the guide). */
#define LF2_POSE_A_MIN_PM       (950U)
/** LF-3.3: only the straight drive, this many times. */
#define LF2_STRAIGHT_ONLY       (1U)
#define LF2_STRAIGHT_RUNS       (3U)
/** Test ID of this build's checks. */
#define LF2_ID                  "S3-U13"
/** Trace: one row per this many polls (20 ms), and how it is printed: a
 *  burst of rows, then a pause so the console drains the 2 KB ring. */
#define LF2_TRACE_EVERY_POLLS   (2U)
#define LF2_TRACE_ROWS          (256U)
#define LF2_TRACE_BURST         (8U)
#define LF2_TRACE_PAUSE_MS      (100U)
/** Recovery drives: ON_LINE within this time of START. */
#define LF2_RECOVER_MAX_MS      (1000U)
/** After a drive ends: the wheels must not move between these times. */
#define LF2_STILL_FROM_MS       (300U)
#define LF2_STILL_TO_MS         (500U)
/** Cruise speed of this build, mm/s. */
#define LF2_CRUISE_MMPS         ((uint32_t)MOTION_SPEED_CRUISE_CMPS * 10U)
/** Drive limits (wheel travel).  The time limit is the travel at cruise
 *  plus 1.5 s. */
/* INTERIM S3-U13i: 200 stops the car before the track's cross line, 250 mm
 * past the start.  The full S3-U13 needs 900 on >= 1.2 m of clear line. */
#define LF2_STRAIGHT_MM         (200U)
#define LF2_RECOVER_MM          (500U)
#define LF2_JUNCTION_MM         (800U)
#define LF2_END_MM              (1200U)
#define LF2_END_MS              (7000U)
#define LF2_MS_FOR(mm)          ((((mm) * 1000U) / LF2_CRUISE_MMPS) + 1500U)
#define LF2_NEVER               (0xFFFFFFFFU)

#if (MOTION_SPEED_CRUISE_CMPS == 0U)
#error "lf_run_test.c: MOTION_SPEED_CRUISE_CMPS must not be 0"
#endif

/** Why a drive ended. */
typedef enum
{
    LF2_END_DIST = 0,       /**< wheel travel limit: the test sent STOP */
    LF2_END_TIME,           /**< time limit: the test sent STOP */
    LF2_END_TRACK_LOST,     /**< TRACK_LOST: the VC stopped the car */
    LF2_END_FAULT,          /**< Safety latched a fault: braked */
    LF2_END_AVOID,          /**< obstacle avoidance took over: the test sent STOP */
    LF2_END_VC_STOP,        /**< the VC went READY by itself */
    LF2_END_NOT_READY,      /**< the VC was not READY: no START sent */
    LF2_END_COUNT
} lf2_end_t;

static char const * const k_end_name[LF2_END_COUNT] =
{
    "distance limit", "time limit", "TRACK_LOST", "FAULT", "avoidance",
    "VC stopped", "VC not READY"
};

/** One bounded drive. */
typedef struct
{
    char const *p_tag;      /**< short name in the log */
    char const *p_place;    /**< how to place the car */
    uint32_t    max_mm;     /**< wheel travel limit */
    uint32_t    max_ms;     /**< time limit */
} lf2_drive_t;

/** One row of the 20 ms trace. */
typedef struct
{
    uint16_t t_ms;      /**< since START */
    uint16_t mm;        /**< mean wheel travel since START */
    uint16_t raw_c;
    uint16_t raw_r;
    int16_t  cnt_l;     /**< left wheel counts since START */
    int16_t  cnt_r;     /**< right wheel counts since START */
    int8_t   steer_q7;  /**< last steering posted */
    uint8_t  state;     /**< lf_state_t */
} lf2_trace_row_t;

/** What one drive did. */
typedef struct
{
    lf2_end_t end;
    uint32_t  t_end_ms;         /**< START to the end */
    uint32_t  mm_end;           /**< wheel travel START to the end */
    uint32_t  mm_stop;          /**< wheel travel after the end (0.5 s) */
    bool      still;            /**< no wheel counts 300..500 ms after the end */
    lf_state_t st_end;          /**< LF state at the end */
    bool      first_seen;       /**< a steering post was seen */
    int32_t   first_steer;      /**< the first steering seen after START */
    uint32_t  t_on_ms;          /**< first poll ON_LINE (LF2_NEVER: never) */
    uint32_t  settle_mm;        /**< travel at the first settle (LF2_NEVER: never) */
    uint32_t  settled_polls;    /**< polls after the settle */
    uint32_t  settled_band;     /**< ... ON_LINE with CENTRE in the band */
    uint32_t  half_polls;       /**< polls in the second half of the travel limit */
    uint32_t  half_band;        /**< ... ON_LINE with CENTRE in the band */
    int32_t   steer_abs_max;    /**< largest |steer| after the settle */
    uint32_t  steer_flips;      /**< steering sign changes after the settle */
    bool      lost;             /**< LINE_LOST entered */
    bool      found_flag;       /**< LINE_FOUND seen set while ON_LINE */
    uint32_t  r_dark_polls;     /**< ON_LINE polls with RIGHT not LIGHT */
    uint32_t  r_white_n;        /**< ON_LINE polls with RIGHT LIGHT ... */
    uint32_t  r_white_sum;
    uint16_t  r_white_min;
    uint16_t  r_white_max;
    uint16_t  r_peak_raw;       /**< highest RIGHT raw in the drive */
    bool      win_done;         /**< the post window completed */
    bool      win_on_line;      /**< LF ON_LINE through the window */
    uint32_t  win_posts;        /**< slot posts in the window */
    uint32_t  win_ms;           /**< the window's real length */
    uint32_t  speed_mmps;       /**< wheel travel per second after 0.5 s */
    uint32_t  n_jct;            /**< JUNCTION events in the drive */
    uint32_t  n_lost;           /**< TRACK_LOST events in the drive */
    uint32_t  ev_fail;          /**< failed events in the drive */
    uint32_t  dropped;          /**< dropped steering posts in the drive */
    uint32_t  lost_to_track_ms; /**< LINE_LOST entry to TRACK_LOST */
} lf2_run_t;

/* The test task's stack is small: the snapshots live here. */
static lf2_run_t const  s_zero_run;
static lf2_run_t        s_run;
static lf_task_status_t s_base;
static lf_task_status_t s_now;
static motion_state_t   s_m0;
static motion_state_t   s_m;
static motion_state_t   s_m_half;
static motion_state_t   s_m_end;
/* The 20 ms trace of the current drive (3.6 KB; APP_TEST builds only). */
static lf2_trace_row_t  s_trace[LF2_TRACE_ROWS];
static uint32_t         s_trace_n;
/* Static pose A (start pose): RIGHT over white, for the noise comparison. */
static uint16_t         s_static_r_min;
static uint16_t         s_static_r_max;

static bool lf2_line_found_flag(void)
{
    T_RFLG ref;

    return (tk_ref_flg(EVF_NAV, &ref) >= (ER)E_OK)
           && ((((uint32_t)ref.flgptn) & EVF_NAV_LINE_FOUND) != 0U);
}

static void lf2_remote(uint32_t type)
{
    nav_input_t const m = drep_nav_remote(type);

    if (tk_snd_mbf(MBF_NAV_IN, &m, (INT)sizeof(m), (TMO)NAV_MBF_SEND_TMO_MS) != E_OK)
    {
        dbg_puts("[LF] test: remote command not sent (MBF_NAV_IN full)");
    }
}

static uint32_t lf2_posts(lf_task_status_t const *p_s)
{
    return p_s->posts_slot + p_s->posts_entry;
}

static uint32_t lf2_abs_delta(int32_t a, int32_t b)
{
    int32_t const d = (int32_t)((uint32_t)b - (uint32_t)a);     /* counts wrap */

    return (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
}

/* Wheel travel between two Motion states: the mean of both wheels' counts. */
static uint32_t lf2_travel_mm(motion_state_t const *p_a, motion_state_t const *p_b)
{
    uint32_t const dl = lf2_abs_delta(p_a->enc_count[0], p_b->enc_count[0]);
    uint32_t const dr = lf2_abs_delta(p_a->enc_count[1], p_b->enc_count[1]);

    return (((dl + dr) / 2U) * WHEEL_CIRC_MM) / (ENC_QUAD_MULT * ENC_CPR);
}

static bool lf2_in_band(uint16_t d)
{
    return (d >= LF_LIGHT_MAX_PM);
}

static bool lf2_pct_ok(uint32_t part, uint32_t whole)
{
    return (whole > 0U) && ((part * 100U) >= (whole * LF2_BAND_MIN_PCT));
}

/* Tell the operator where to put the car, then count down.  The VC is
 * READY meanwhile (motors stopped).  If an earlier drive latched a fault
 * (a hard catch reads as a collision), RESET it near the end of the count:
 * the VC accepts it only after SAFETY_T_STILL_MS with the wheels still. */
static void lf2_place(char const *p_tag, char const *p_place, bool drives)
{
    dbg_printf("[LF] >>> %s: %s", p_tag, p_place);
    if (drives)
    {
        dbg_puts("[LF] >>> hands off at 0: the car DRIVES (catch and lift it to stop)");
    }
    for (uint32_t left = LF2_PLACE_S; left > 0U; left--)
    {
        if ((left <= 3U) || (left == (LF2_PLACE_S / 2U)))
        {
            dbg_printf("[LF] ... %lu", (unsigned long)left);
        }
        if ((left <= 2U) && (vc_get_system_state() == VC_SYS_SAFETY_FAULT_LATCHED))
        {
            dbg_puts("[LF] fault latched: test sends RESET (car must be still)");
            lf2_remote(DREP_NAV_REMOTE_RESET);
        }
        (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    }
}

/* Static measurement in one pose, motors stopped.  Returns the average
 * darkness of CENTRE and RIGHT; keeps RIGHT's raw range. */
static void lf2_static(char const *p_tag, char const *p_place,
                       uint16_t *p_dc, uint16_t *p_dr, uint16_t *p_rmin, uint16_t *p_rmax)
{
    uint32_t sum_c = 0U;
    uint32_t sum_r = 0U;
    uint32_t n = 0U;
    uint16_t c_min = 0xFFFFU;
    uint16_t c_max = 0U;
    uint16_t r_min = 0xFFFFU;
    uint16_t r_max = 0U;
    uint16_t avg_c;
    uint16_t avg_r;

    lf2_place(p_tag, p_place, false);
    for (uint32_t t = 0U; t < LF2_STATIC_MS; t += LF2_POLL_MS)
    {
        lf_task_get_status(&s_now);
        sum_c += s_now.raw_c;
        sum_r += s_now.raw_r;
        c_min = (s_now.raw_c < c_min) ? s_now.raw_c : c_min;
        c_max = (s_now.raw_c > c_max) ? s_now.raw_c : c_max;
        r_min = (s_now.raw_r < r_min) ? s_now.raw_r : r_min;
        r_max = (s_now.raw_r > r_max) ? s_now.raw_r : r_max;
        n++;
        (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
    }
    avg_c = (uint16_t)(sum_c / n);
    avg_r = (uint16_t)(sum_r / n);
    *p_dc = lf_norm(&s_now.cal.c, avg_c);
    *p_dr = lf_norm(&s_now.cal.r, avg_r);
    *p_rmin = r_min;
    *p_rmax = r_max;
    dbg_printf("[LF] static %s: C raw %u..%u avg %u = %u pm", p_tag, c_min, c_max, avg_c, *p_dc);
    dbg_printf("[LF] static %s: R raw %u..%u avg %u = %u pm", p_tag, r_min, r_max, avg_r, *p_dr);
}

/* One trace row.  Counts since START fit int16_t: the longest drive is
 * 1200 mm = 14 300 counts. */
static void lf2_trace_add(uint32_t t, uint32_t mm)
{
    if (s_trace_n < LF2_TRACE_ROWS)
    {
        lf2_trace_row_t *const p_r = &s_trace[s_trace_n];

        p_r->t_ms = (uint16_t)t;
        p_r->mm = (uint16_t)mm;
        p_r->raw_c = s_now.raw_c;
        p_r->raw_r = s_now.raw_r;
        p_r->cnt_l = (int16_t)((uint32_t)s_m.enc_count[0] - (uint32_t)s_m0.enc_count[0]);
        p_r->cnt_r = (int16_t)((uint32_t)s_m.enc_count[1] - (uint32_t)s_m0.enc_count[1]);
        p_r->steer_q7 = (int8_t)s_now.last_steer_q7;
        p_r->state = (uint8_t)s_now.state;
        s_trace_n++;
    }
}

/* Print the trace, the car stopped.  The last line gives the ring's dropped
 * line count before and after, so a lost row is visible. */
static void lf2_trace_dump(char const *p_tag)
{
    uint32_t const dropped0 = dbg_overflow_count();

    dbg_printf("[LF] tr %s: t mm st c r s L R (%lu rows, 20 ms)", p_tag, (unsigned long)s_trace_n);
    for (uint32_t i = 0U; i < s_trace_n; i++)
    {
        lf2_trace_row_t const *const p_r = &s_trace[i];

        dbg_printf("[LF] tr %u %u %u %u %u %d %d %d", p_r->t_ms, p_r->mm, p_r->state, p_r->raw_c,
                   p_r->raw_r, (int)p_r->steer_q7, (int)p_r->cnt_l, (int)p_r->cnt_r);
        if (((i + 1U) % LF2_TRACE_BURST) == 0U)
        {
            (void)tk_dly_tsk((RELTIM)LF2_TRACE_PAUSE_MS);
        }
    }
    (void)tk_dly_tsk((RELTIM)LF2_TRACE_PAUSE_MS);
    dbg_printf("[LF] tr %s end: ring dropped %lu lines", p_tag,
               (unsigned long)(dbg_overflow_count() - dropped0));
}

/* Per-poll statistics of a drive. */
static void lf2_account(lf2_run_t *p, uint32_t t, uint32_t mm, uint32_t half_mm,
                        int32_t *p_last_sign)
{
    uint16_t const d_c = lf_norm(&s_now.cal.c, s_now.raw_c);
    uint16_t const d_r = lf_norm(&s_now.cal.r, s_now.raw_r);
    bool const     on = (s_now.state == LF_ST_ON_LINE);
    bool const     band = on && lf2_in_band(d_c);

    if (!p->first_seen && (lf2_posts(&s_now) != lf2_posts(&s_base)))
    {
        p->first_seen = true;
        p->first_steer = s_now.last_steer_q7;
    }
    if (on && (p->t_on_ms == LF2_NEVER))
    {
        p->t_on_ms = t;
    }
    if (on && (p->settle_mm == LF2_NEVER) && (d_c >= LF2_SETTLE_PM_LO) && (d_c <= LF2_SETTLE_PM_HI))
    {
        p->settle_mm = mm;
    }
    if (p->settle_mm != LF2_NEVER)
    {
        int32_t const s = s_now.last_steer_q7;
        int32_t const a = (s < 0) ? -s : s;
        int32_t const sign = (s > 0) ? 1 : ((s < 0) ? -1 : 0);

        p->settled_polls++;
        p->settled_band += band ? 1U : 0U;
        p->steer_abs_max = (a > p->steer_abs_max) ? a : p->steer_abs_max;
        if ((sign != 0) && (*p_last_sign != 0) && (sign != *p_last_sign))
        {
            p->steer_flips++;
        }
        *p_last_sign = (sign != 0) ? sign : *p_last_sign;
    }
    if (mm >= half_mm)
    {
        p->half_polls++;
        p->half_band += band ? 1U : 0U;
    }
    if (s_now.t_entry_ms[LF_ST_LINE_LOST] != s_base.t_entry_ms[LF_ST_LINE_LOST])
    {
        p->lost = true;
    }
    if (on && !p->found_flag && lf2_line_found_flag())
    {
        p->found_flag = true;
    }
    if (on && (d_r < LF_LIGHT_MAX_PM))
    {
        p->r_white_n++;
        p->r_white_sum += s_now.raw_r;
        p->r_white_min = (s_now.raw_r < p->r_white_min) ? s_now.raw_r : p->r_white_min;
        p->r_white_max = (s_now.raw_r > p->r_white_max) ? s_now.raw_r : p->r_white_max;
    }
    else if (on)
    {
        p->r_dark_polls++;
    }
    else
    {
        /* not tracking: RIGHT is counted only while ON_LINE */
    }
    p->r_peak_raw = (s_now.raw_r > p->r_peak_raw) ? s_now.raw_r : p->r_peak_raw;
}

/* One bounded drive: START, watch every 10 ms, STOP at a limit. */
static void lf2_drive(lf2_drive_t const *p_d)
{
    lf2_run_t *const p = &s_run;
    uint32_t const   half_mm = p_d->max_mm / 2U;
    uint32_t         t0;
    uint32_t         t = 0U;
    uint32_t         t_half = 0U;
    uint32_t         t_win = 0U;
    uint32_t         win_base = 0U;
    bool             win_open = false;
    bool             half_seen = false;
    bool             done = false;
    int32_t          last_sign = 0;
    uint32_t         polls = 0U;
    uint32_t         mm = 0U;

    *p = s_zero_run;
    s_trace_n = 0U;
    p->t_on_ms = LF2_NEVER;
    p->settle_mm = LF2_NEVER;
    p->r_white_min = 0xFFFFU;
    p->win_on_line = true;

    lf2_place(p_d->p_tag, p_d->p_place, true);
    if (vc_get_system_state() != VC_SYS_READY_WAITING)
    {
        p->end = LF2_END_NOT_READY;
        dbg_printf("[LF] %s: VC state %lu, not READY: no START", p_d->p_tag,
                   (unsigned long)vc_get_system_state());
        return;
    }

    lf_task_get_status(&s_base);
    (void)motion_get_state(&s_m0);
    t0 = app_period_now_ms();
    lf2_remote(DREP_NAV_REMOTE_START);

    while (!done)
    {
        uint32_t vc;

        (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
        lf_task_get_status(&s_now);
        (void)motion_get_state(&s_m);
        t = app_period_now_ms() - t0;
        mm = lf2_travel_mm(&s_m0, &s_m);
        vc = vc_get_system_state();
        polls++;

        lf2_account(p, t, mm, half_mm, &last_sign);
        if ((polls % LF2_TRACE_EVERY_POLLS) == 0U)
        {
            lf2_trace_add(t, mm);
        }

        if (!half_seen && (t >= LF2_WIN_FROM_MS))
        {
            half_seen = true;
            t_half = t;
            s_m_half = s_m;
        }
        if (!win_open && !p->win_done && (t >= LF2_WIN_FROM_MS))
        {
            win_open = true;
            win_base = s_now.posts_slot;
            t_win = t;
        }
        if (win_open)
        {
            p->win_on_line = p->win_on_line && (s_now.state == LF_ST_ON_LINE);
            if (t >= LF2_WIN_TO_MS)
            {
                win_open = false;
                p->win_done = true;
                p->win_posts = s_now.posts_slot - win_base;
                p->win_ms = t - t_win;
            }
        }

        if ((polls % LF2_LOG_EVERY_POLLS) == 0U)
        {
            dbg_printf("[LF] d t=%lu mm=%lu st=%u vc=%lu c=%u r=%u dc=%u dr=%u s=%ld",
                       (unsigned long)t, (unsigned long)mm, (unsigned)s_now.state,
                       (unsigned long)vc, s_now.raw_c, s_now.raw_r,
                       lf_norm(&s_now.cal.c, s_now.raw_c), lf_norm(&s_now.cal.r, s_now.raw_r),
                       (long)s_now.last_steer_q7);
        }

        done = true;
        if (s_now.n_track_lost != s_base.n_track_lost)
        {
            p->end = LF2_END_TRACK_LOST;
        }
        else if (vc == VC_SYS_SAFETY_FAULT_LATCHED)
        {
            p->end = LF2_END_FAULT;
        }
        else if (vc == VC_SYS_OBSTACLE_AVOIDANCE)
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_AVOID;
        }
        else if (vc == VC_SYS_READY_WAITING)
        {
            p->end = LF2_END_VC_STOP;
        }
        else if (mm >= p_d->max_mm)
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_DIST;
        }
        else if (t >= p_d->max_ms)
        {
            lf2_remote(DREP_NAV_REMOTE_STOP);
            p->end = LF2_END_TIME;
        }
        else
        {
            done = false;
        }
    }

    s_m_end = s_m;
    p->t_end_ms = t;
    p->mm_end = mm;
    p->st_end = s_now.state;
    p->n_jct = s_now.n_junction - s_base.n_junction;
    p->n_lost = s_now.n_track_lost - s_base.n_track_lost;
    p->ev_fail = s_now.ev_send_fail - s_base.ev_send_fail;
    p->dropped = s_now.posts_dropped - s_base.posts_dropped;
    p->lost_to_track_ms = s_now.t_track_lost_ms - s_now.t_entry_ms[LF_ST_LINE_LOST];
    if (half_seen && (t > t_half))
    {
        p->speed_mmps = (lf2_travel_mm(&s_m_half, &s_m_end) * 1000U) / (t - t_half);
    }

    /* Stopping: travel after the end, and the wheels still at 300..500 ms. */
    (void)tk_dly_tsk((RELTIM)LF2_STILL_FROM_MS);
    (void)motion_get_state(&s_m_half);
    (void)tk_dly_tsk((RELTIM)(LF2_STILL_TO_MS - LF2_STILL_FROM_MS));
    (void)motion_get_state(&s_m);
    p->still = (s_m.enc_count[0] == s_m_half.enc_count[0])
               && (s_m.enc_count[1] == s_m_half.enc_count[1]);
    p->mm_stop = lf2_travel_mm(&s_m_end, &s_m);

    dbg_printf("[LF] %s: end %s at %lu ms, %lu mm; then %lu mm, still %u, vc=%lu",
               p_d->p_tag, k_end_name[p->end], (unsigned long)p->t_end_ms,
               (unsigned long)p->mm_end, (unsigned long)p->mm_stop, (unsigned)p->still,
               (unsigned long)vc_get_system_state());
    dbg_printf("[LF] %s: speed %lu mm/s (cruise %lu), first steer %ld, ON_LINE at %ld ms",
               p_d->p_tag, (unsigned long)p->speed_mmps, (unsigned long)LF2_CRUISE_MMPS,
               (long)p->first_steer, (long)((p->t_on_ms == LF2_NEVER) ? -1 : (int32_t)p->t_on_ms));
    dbg_printf("[LF] %s: settle %ld mm, band %lu/%lu after, %lu/%lu 2nd half",
               p_d->p_tag, (long)((p->settle_mm == LF2_NEVER) ? -1 : (int32_t)p->settle_mm),
               (unsigned long)p->settled_band, (unsigned long)p->settled_polls,
               (unsigned long)p->half_band, (unsigned long)p->half_polls);
    dbg_printf("[LF] %s: |steer| max %ld, flips %lu, LINE_LOST %u, LINE_FOUND %u",
               p_d->p_tag, (long)p->steer_abs_max, (unsigned long)p->steer_flips,
               (unsigned)p->lost, (unsigned)p->found_flag);
    dbg_printf("[LF] %s: jct %lu, track_lost %lu, ev fail %lu, dropped %lu",
               p_d->p_tag, (unsigned long)p->n_jct, (unsigned long)p->n_lost,
               (unsigned long)p->ev_fail, (unsigned long)p->dropped);
}

/* RIGHT's statistics of the last drive, against the static pose A. */
static void lf2_log_right(char const *p_tag)
{
    lf2_run_t const *const p = &s_run;
    uint16_t const avg = (p->r_white_n > 0U) ? (uint16_t)(p->r_white_sum / p->r_white_n) : 0U;

    dbg_printf("[LF] %s: R white moving raw %u..%u avg %u (%lu polls), static %u..%u",
               p_tag, (p->r_white_n > 0U) ? p->r_white_min : 0U, p->r_white_max, avg,
               (unsigned long)p->r_white_n, s_static_r_min, s_static_r_max);
    dbg_printf("[LF] %s: R peak raw %u = %u pm, R not white %lu polls", p_tag, p->r_peak_raw,
               lf_norm(&s_now.cal.r, p->r_peak_raw), (unsigned long)p->r_dark_polls);
}

/* Recovery drive checks (r1..r3): ON_LINE soon, settled in the second
 * half, never lost, ended at the distance limit. */
static bool lf2_recovered(void)
{
    lf2_run_t const *const p = &s_run;

    return (p->end == LF2_END_DIST) && (p->t_on_ms <= LF2_RECOVER_MAX_MS) && !p->lost
           && (p->n_lost == 0U) && lf2_pct_ok(p->half_band, p->half_polls);
}

void lf_run_test(uint32_t *p_pass, uint32_t *p_fail)
{
    static lf2_drive_t const k_straight =
    {
        "straight", "START POSE at the start of >= 1 m of straight line",
        LF2_STRAIGHT_MM, LF2_MS_FOR(LF2_STRAIGHT_MM)
    };
    static lf2_drive_t const k_r1 =
    {
        "r1", "car parallel to the line, CENTRE ~1 cm RIGHT of it (on white)",
        LF2_RECOVER_MM, LF2_MS_FOR(LF2_RECOVER_MM)
    };
    static lf2_drive_t const k_r2 =
    {
        "r2", "nose ~15 deg LEFT of the line, CENTRE on the line middle",
        LF2_RECOVER_MM, LF2_MS_FOR(LF2_RECOVER_MM)
    };
    static lf2_drive_t const k_r3 =
    {
        "r3", "nose ~15 deg RIGHT of the line, CENTRE on the line middle",
        LF2_RECOVER_MM, LF2_MS_FOR(LF2_RECOVER_MM)
    };
    static lf2_drive_t const k_jct =
    {
        "junction", "START POSE, a cross junction 40 cm ahead, square to it",
        LF2_JUNCTION_MM, LF2_MS_FOR(LF2_JUNCTION_MM)
    };
    static lf2_drive_t const k_end =
    {
        "line end", "START POSE, the line's END 30 cm ahead (temporary line OK)",
        LF2_END_MM, LF2_END_MS
    };
    uint32_t test_pass_ = *p_pass;
    uint32_t test_fail_ = *p_fail;
    uint32_t releases0;
    uint32_t vc;
    uint16_t dc;
    uint16_t dr;
    uint16_t rmin;
    uint16_t rmax;

    dbg_printf("[LF] %s on the track, motors LIVE, cruise %lu mm/s", LF2_ID,
               (unsigned long)LF2_CRUISE_MMPS);
    dbg_puts("[LF] guide: .agents/tasks/s3-lf33-test.md; follow the [LF] >>> prompts");

    /* (a) The task runs every 5 ms. */
    lf_task_get_status(&s_now);
    releases0 = s_now.releases;
    (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    lf_task_get_status(&s_now);
    dbg_printf("[LF] 1 s: %lu samples, %lu missed so far",
               (unsigned long)(s_now.releases - releases0), (unsigned long)s_now.missed);
    TEST_CHECK(LF2_ID, ((s_now.releases - releases0) >= LF2_RELEASES_1S_MIN)
                         && ((s_now.releases - releases0) <= LF2_RELEASES_1S_MAX)
                         && (s_now.missed == 0U),
               "(a) samples every 5 ms (195-205/s), none missed");

    /* (b) Boot calibration in the start pose. */
    dbg_printf("[LF] boot cal: status %u, C avg %u, R avg %u -> white %u black %u",
               (unsigned)s_now.cal_status, s_now.cal_c_avg, s_now.cal_r_avg,
               s_now.cal.c.white, s_now.cal.c.black);
    TEST_CHECK(LF2_ID, s_now.cal_status == LF_CAL_OK,
               "(b) boot calibration accepted");

    vc = vc_get_system_state();
    if (((vc != VC_SYS_READY_WAITING) && (vc != VC_SYS_INITIALISING)) || (s_now.cal_status != LF_CAL_OK))
    {
        dbg_printf("[LF] VC state %lu (need 0 or 1; 3 = fault, see [SAFETY]), cal %u",
                   (unsigned long)vc, (unsigned)s_now.cal_status);
        TEST_CHECK(LF2_ID, false, "(c-h) not run: VC not READY or cal refused");
        *p_pass = test_pass_;
        *p_fail = test_fail_;
        return;
    }

    /* (c) Armed, VC READY: no output. */
    lf_task_test_arm(true);
    lf_task_get_status(&s_base);
    (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    lf_task_get_status(&s_now);
    TEST_CHECK(LF2_ID, (lf2_posts(&s_now) == lf2_posts(&s_base))
                         && (s_now.n_junction == s_base.n_junction)
                         && (s_now.n_track_lost == s_base.n_track_lost)
                         && (s_now.state == LF_ST_IDLE) && !lf2_line_found_flag(),
               "(c) VC READY: no posts, no events, IDLE, flag clear");

    /* (g) Static levels at ride height, motors stopped.  Pose A gives
     * RIGHT's resting noise over white; pose B RIGHT's black level (the
     * residual risk of the S3-U09 waiver). */
    lf2_static("pose A", "put the car down in the START POSE, hands off", &dc, &dr, &rmin, &rmax);
    s_static_r_min = rmin;
    s_static_r_max = rmax;
    TEST_CHECK(LF2_ID, dc >= LF2_POSE_A_MIN_PM,
               "(g2) pose A: CENTRE >= 950 pm, the boot calibration holds");
    lf2_static("pose B", "RIGHT over the line middle, CENTRE on white, hands off", &dc, &dr, &rmin, &rmax);
    TEST_CHECK(LF2_ID, dr > LF_DARK_MIN_PM,
               "(g) static: RIGHT on black is DARK (> 750 pm)");

    /* (d) + (n) Straight line from the start pose. */
    for (uint32_t run = 1U; run <= LF2_STRAIGHT_RUNS; run++)
    {
        dbg_printf("[LF] straight run %lu of %lu", (unsigned long)run,
                   (unsigned long)LF2_STRAIGHT_RUNS);
        lf2_drive(&k_straight);
        lf2_log_right("straight");
        dbg_printf("[LF] straight: window %lu slot posts in %lu ms, ON_LINE %u",
                   (unsigned long)s_run.win_posts, (unsigned long)s_run.win_ms, (unsigned)s_run.win_on_line);
        lf2_trace_dump("straight");
        TEST_CHECK(LF2_ID, s_run.first_seen && (s_run.first_steer >= LF2_STEER_BODY_MIN)
                           && (s_run.settle_mm <= LF2_SETTLE_MAX_MM),
                   "(d1) straight: steers right, feels the edge in 500 mm");
        TEST_CHECK(LF2_ID, s_run.win_done && s_run.win_on_line
                           && ((s_run.win_posts * LF_STEER_POST_PERIOD_MS) + LF_STEER_POST_PERIOD_MS >= s_run.win_ms)
                           && ((s_run.win_posts * LF_STEER_POST_PERIOD_MS) <= s_run.win_ms + LF_STEER_POST_PERIOD_MS)
                           && (s_run.dropped == 0U),
                   "(d2) one post per 20 ms in the slot, none dropped");
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && !s_run.lost && s_run.found_flag
                           && lf2_pct_ok(s_run.settled_band, s_run.settled_polls),
                   "(d3) stable: CENTRE on the line (>= 90%), never lost, flag");
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.r_white_n > 0U)
                           && (s_run.r_dark_polls == 0U) && (s_run.n_jct == 0U),
                   "(n) motors on: RIGHT stays LIGHT, no false junction");
    }

    if (LF2_STRAIGHT_ONLY == 0U)
    {
        /* (r) Small offsets: recovery. */
        lf2_drive(&k_r1);
        TEST_CHECK(LF2_ID, lf2_recovered() && s_run.first_seen && (s_run.first_steer < 0),
                   "(r1) right of line: steers left, ON_LINE <1 s, settles");
        lf2_drive(&k_r2);
        TEST_CHECK(LF2_ID, lf2_recovered(),
                   "(r2) nose 15 deg left: ON_LINE <1 s, settles");
        lf2_drive(&k_r3);
        TEST_CHECK(LF2_ID, lf2_recovered(),
                   "(r3) nose 15 deg right: ON_LINE <1 s, settles");

        /* (j) Cross junction. */
        lf2_drive(&k_jct);
        lf2_log_right("junction");
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_DIST) && (s_run.n_jct == 1U) && (s_run.ev_fail == 0U)
                             && (lf_norm(&s_now.cal.r, s_run.r_peak_raw) > LF_DARK_MIN_PM)
                             && (s_run.st_end == LF_ST_ON_LINE) && (s_run.n_lost == 0U),
                   "(j) cross: one JUNCTION, RIGHT DARK, tracking after");

        /* (f) Line end: LINE_LOST, the search, TRACK_LOST, the VC stops. */
        lf2_drive(&k_end);
        dbg_printf("[LF] line end: LINE_LOST to TRACK_LOST %lu ms", (unsigned long)s_run.lost_to_track_ms);
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_TRACK_LOST) && (s_run.n_lost == 1U)
                             && (s_run.lost_to_track_ms >= LF2_LOST_TO_TRACK_MIN)
                             && (s_run.lost_to_track_ms <= LF2_LOST_TO_TRACK_MAX),
                   "(f1) line end: one TRACK_LOST 2110-2135 ms after LOST");
        lf_task_get_status(&s_now);
        TEST_CHECK(LF2_ID, (s_run.end == LF2_END_TRACK_LOST)
                             && (vc_get_system_state() == VC_SYS_READY_WAITING)
                             && (s_now.state == LF_ST_IDLE) && !lf2_line_found_flag() && s_run.still,
                   "(f2) VC READY, wheels still, LF IDLE, flag clear");
    }

    /* (h) The whole run. */
    lf_task_get_status(&s_now);
    dbg_printf("[LF] totals: %lu samples, %lu missed, %lu dropped posts, %lu failed events",
               (unsigned long)s_now.releases, (unsigned long)s_now.missed,
               (unsigned long)s_now.posts_dropped, (unsigned long)s_now.ev_send_fail);
    TEST_CHECK(LF2_ID, (s_now.missed == 0U) && (s_now.posts_dropped == 0U) && (s_now.ev_send_fail == 0U),
               "(h) run: no missed period, dropped post, failed event");

    /* Leave the car stopped and the task disarmed for the later tests. */
    if (vc_get_system_state() != VC_SYS_READY_WAITING)
    {
        lf2_remote(DREP_NAV_REMOTE_STOP);
    }
    lf_task_test_arm(false);
    dbg_printf("[LF] %s done: the car stays stopped", LF2_ID);

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

#endif /* APP_TEST */
