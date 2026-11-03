/*
 *  lf_run_test.c
 *  Subsystem 3 -- LF-3.2 Line-Following task test on the car (S3-U12),
 *  APP_TEST only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel and pure project headers only (rule R1).
 *
 *  What it shows: the real Line-Following task, with the real Vehicle
 *  Controller and Motion tasks, on the car.  The test task (priority 11)
 *  never reads the ADC itself: it reads the task's status
 *  (lf_task_get_status), the VC's state and EVF_NAV, and plays the operator's
 *  remote START / STOP through MBF_NAV_IN as Telemetry would.
 *
 *  Setup (guide .agents/tasks/s3-lf32-test.md): car on its wheels with the
 *  battery OFF, so Motion runs but the wheels stay still; a white paper
 *  strip with an 18 mm black line under the line sensors (their real ride
 *  height), free to slide sideways; at power-up CENTRE over the middle of
 *  the line and RIGHT over white (the boot calibration pose).
 *
 *  Expected values come from config/timing.h and design.md 3.2, typed in:
 *  5 ms samples (200 per second), one steering post per 20 ms Motion period
 *  (100 in 2 s), LINE_LOST -> 120 ms stop -> 2 s search -> TRACK_LOST
 *  (2120 ms), steer > 0 = steer right (data_repr.h).
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/subsystem2.h"   /* vc_get_system_state, VC_SYS_* */
#include "config/timing.h"              /* NAV_MBF_SEND_TMO_MS */
#include "lf_core.h"                    /* lf_norm, LF_DARK_MIN_PM, LF_LIGHT_MAX_PM */
#include "lf_run_test.h"
#include "task_lf.h"

#define LF2_SECOND_MS           (1000U)
/** Samples in 1 s at 5 ms, +-5 for the start and end of the window. */
#define LF2_RELEASES_1S_MIN     (195U)
#define LF2_RELEASES_1S_MAX     (205U)
/** Slot posts in 2 s: one per 20 ms Motion period, +-1 at the ends. */
#define LF2_POSTS_2S_MIN        (99U)
#define LF2_POSTS_2S_MAX        (101U)
/** LINE_LOST entry to TRACK_LOST: 120 ms stop + 2000 ms search, +-1 sample
 *  and the kernel tick. */
#define LF2_LOST_TO_TRACK_MIN   (2110U)
#define LF2_LOST_TO_TRACK_MAX   (2135U)
/** Steering at CENTRE on the line body: Kp 64 x (1000 - 500) / 500 = +64
 *  (lf_core.h LF_KP_Q7); "clearly right" is at least half of it. */
#define LF2_STEER_BODY_MIN      (32)
/** CENTRE at the line's right edge (darkness 400..600 per mille): the
 *  posted steering is 64 x (d - 500) / 500 = -12..+12; allow +-24 for the
 *  up-to-20 ms age of the last post and noise.  At least 3 such polls. */
#define LF2_EDGE_PM_LO          (400U)
#define LF2_EDGE_PM_HI          (600U)
#define LF2_STEER_EDGE_ABS_MAX  (24)
#define LF2_EDGE_MIN_POLLS      (3U)
#define LF2_POLL_MS             (10U)

/* Tell the operator what to do and for how long. */
static void lf2_countdown(char const *p_what, uint32_t seconds)
{
    dbg_printf("[LF] >>> %s (%lu s)", p_what, (unsigned long)seconds);
    for (uint32_t left = seconds; left > 0U; left--)
    {
        if (((left % 5U) == 0U) && (left != seconds))
        {
            dbg_printf("[LF] ... %lu s left", (unsigned long)left);
        }
        (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    }
}

static lf_task_status_t lf2_status(void)
{
    lf_task_status_t s;

    lf_task_get_status(&s);
    return s;
}

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

static void lf2_log(char const *p_tag, lf_task_status_t const *p_s)
{
    dbg_printf("[LF] %s: st=%u vc=%lu c=%u r=%u dc=%u steer=%ld slot=%lu entry=%lu",
               p_tag, (unsigned)p_s->state, (unsigned long)p_s->vc_state,
               p_s->raw_c, p_s->raw_r, lf_norm(&p_s->cal.c, p_s->raw_c),
               (long)p_s->last_steer_q7, (unsigned long)p_s->posts_slot,
               (unsigned long)p_s->posts_entry);
}

void lf_run_test(uint32_t *p_pass, uint32_t *p_fail)
{
    uint32_t         test_pass_ = *p_pass;
    uint32_t         test_fail_ = *p_fail;
    lf_task_status_t s0;
    lf_task_status_t s1;
    uint32_t         vc;
    bool             ready;

    dbg_puts("[LF] S3-U12 LF task, battery OFF: follow the [LF] >>> prompts");
    dbg_puts("[LF] guide: .agents/tasks/s3-lf32-test.md");

    /* (a) The task runs every 5 ms. */
    s0 = lf2_status();
    (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    s1 = lf2_status();
    dbg_printf("[LF] 1 s: %lu samples, %lu missed so far",
               (unsigned long)(s1.releases - s0.releases), (unsigned long)s1.missed);
    TEST_CHECK("S3-U12", ((s1.releases - s0.releases) >= LF2_RELEASES_1S_MIN)
                         && ((s1.releases - s0.releases) <= LF2_RELEASES_1S_MAX)
                         && (s1.missed == 0U),
               "(a) task samples every 5 ms (195-205 per s), none missed");

    /* (b) Boot calibration in the start pose. */
    dbg_printf("[LF] boot cal: status %u, C avg %u, R avg %u -> white %u black %u",
               (unsigned)s1.cal_status, s1.cal_c_avg, s1.cal_r_avg,
               s1.cal.c.white, s1.cal.c.black);
    TEST_CHECK("S3-U12", s1.cal_status == LF_CAL_OK,
               "(b) boot calibration accepted (start pose, contrast >= 400)");

    vc = vc_get_system_state();
    ready = (vc == VC_SYS_READY_WAITING) || (vc == VC_SYS_INITIALISING);
    if (!ready || (s1.cal_status != LF_CAL_OK))
    {
        dbg_printf("[LF] VC state %lu (need 0 or 1; 3 = fault, see [SAFETY]), cal %u",
                   (unsigned long)vc, (unsigned)s1.cal_status);
        TEST_CHECK("S3-U12", false, "(c-h) not run: VC not READY or calibration refused");
        *p_pass = test_pass_;
        *p_fail = test_fail_;
        return;
    }

    /* (c) Armed, VC READY: no output. */
    lf_task_test_arm(true);
    s0 = lf2_status();
    (void)tk_dly_tsk((RELTIM)LF2_SECOND_MS);
    s1 = lf2_status();
    TEST_CHECK("S3-U12", (lf2_posts(&s1) == lf2_posts(&s0))
                         && (s1.n_junction == s0.n_junction) && (s1.n_track_lost == s0.n_track_lost)
                         && (s1.state == LF_ST_IDLE) && !lf2_line_found_flag(),
               "(c) VC READY: no steering, no events, IDLE, LINE_FOUND clear");

    /* (d) START in the start pose: tracking, one post per Motion period. */
    dbg_puts("[LF] >>> START POSE: CENTRE over the strip, RIGHT over white");
    lf2_countdown("keep still: START comes next (battery OFF: wheels still)", 10U);
    s0 = lf2_status();
    lf2_remote(DREP_NAV_REMOTE_START);
    (void)tk_dly_tsk((RELTIM)100U);
    s1 = lf2_status();
    lf2_log("started", &s1);
    {
        lf_task_status_t const sa = s1;
        lf_task_status_t sb;

        (void)tk_dly_tsk((RELTIM)(2U * LF2_SECOND_MS));
        sb = lf2_status();
        lf2_log("after 2 s", &sb);
        dbg_printf("[LF] 2 s: %lu slot posts, %lu entry posts since START, %lu dropped",
                   (unsigned long)(sb.posts_slot - sa.posts_slot),
                   (unsigned long)(sb.posts_entry - s0.posts_entry),
                   (unsigned long)sb.posts_dropped);
        TEST_CHECK("S3-U12", (sa.vc_state == VC_SYS_NAV_LINE_TRACKING)
                             && (sb.vc_state == VC_SYS_NAV_LINE_TRACKING)
                             && (sb.state == LF_ST_ON_LINE),
                   "(d1) START: VC NAV_LINE_TRACKING, LF ON_LINE, for 2 s");
        TEST_CHECK("S3-U12", ((sb.posts_slot - sa.posts_slot) >= LF2_POSTS_2S_MIN)
                             && ((sb.posts_slot - sa.posts_slot) <= LF2_POSTS_2S_MAX)
                             && ((sb.posts_entry - s0.posts_entry) <= 1U)
                             && (sb.posts_dropped == 0U),
                   "(d2) one steering post per 20 ms, in Motion's slot (99-101 in 2 s), none dropped");
        TEST_CHECK("S3-U12", (sb.last_steer_q7 >= LF2_STEER_BODY_MIN) && lf2_line_found_flag(),
                   "(d3) CENTRE on the line body: steer right (>= +32), LINE_FOUND set");
    }

    /* (e) + (f) Slide the line off to the left: steering falls at the edge,
     * then LINE_LOST, the search, TRACK_LOST, the VC stops. */
    dbg_puts("[LF] >>> NOW slide the strip SLOWLY to the car's LEFT (~1 cm/s)");
    dbg_puts("[LF] >>> until BOTH sensors are over white, then keep still");
    {
        uint32_t edge_polls = 0U;
        int32_t  edge_worst = 0;            /* largest |steer| at the edge */
        uint32_t t = 0U;
        bool     got = false;

        s0 = lf2_status();
        while ((t < 20000U) && !got)
        {
            lf_task_status_t const s = lf2_status();
            uint16_t const         d_c = lf_norm(&s.cal.c, s.raw_c);

            if ((s.state == LF_ST_ON_LINE) && (d_c >= LF2_EDGE_PM_LO) && (d_c <= LF2_EDGE_PM_HI))
            {
                int32_t const a = (s.last_steer_q7 < 0) ? -s.last_steer_q7 : s.last_steer_q7;

                edge_polls++;
                edge_worst = (a > edge_worst) ? a : edge_worst;
            }
            if ((t % 200U) == 0U)
            {
                lf2_log("slide", &s);
            }
            got = (s.n_track_lost != s0.n_track_lost);
            (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
            t += LF2_POLL_MS;
        }
        s1 = lf2_status();
        dbg_printf("[LF] at the edge: %lu polls, largest |steer| %ld",
                   (unsigned long)edge_polls, (long)edge_worst);
        dbg_printf("[LF] LINE_LOST at %lu, TRACK_LOST at %lu: %lu ms",
                   (unsigned long)s1.t_entry_ms[LF_ST_LINE_LOST], (unsigned long)s1.t_track_lost_ms,
                   (unsigned long)(s1.t_track_lost_ms - s1.t_entry_ms[LF_ST_LINE_LOST]));
        TEST_CHECK("S3-U12", (edge_polls >= LF2_EDGE_MIN_POLLS) && (edge_worst <= LF2_STEER_EDGE_ABS_MAX),
                   "(e) CENTRE on the right edge: steering near 0 (|steer| <= 24)");
        TEST_CHECK("S3-U12", got && ((s1.n_track_lost - s0.n_track_lost) == 1U)
                             && ((s1.t_track_lost_ms - s1.t_entry_ms[LF_ST_LINE_LOST]) >= LF2_LOST_TO_TRACK_MIN)
                             && ((s1.t_track_lost_ms - s1.t_entry_ms[LF_ST_LINE_LOST]) <= LF2_LOST_TO_TRACK_MAX),
                   "(f1) line off: LINE_LOST, search, one TRACK_LOST 2120 ms later");
    }
    (void)tk_dly_tsk((RELTIM)500U);
    s1 = lf2_status();
    TEST_CHECK("S3-U12", (vc_get_system_state() == VC_SYS_READY_WAITING)
                         && (s1.state == LF_ST_IDLE) && !lf2_line_found_flag(),
               "(f2) TRACK_LOST: VC stops the car (READY), LF IDLE, LINE_FOUND clear");

    /* (g) RIGHT's black level at ride height (residual risk of S3-U09). */
    lf2_countdown("strip under RIGHT ONLY (CENTRE on white), keep still", 10U);
    {
        uint32_t sum_r = 0U;
        uint32_t sum_c = 0U;
        uint32_t n = 0U;
        uint16_t avg_r;
        uint16_t avg_c;
        uint16_t d_r;
        uint16_t d_c;

        for (uint32_t t = 0U; t < (3U * LF2_SECOND_MS); t += LF2_POLL_MS)
        {
            lf_task_status_t const s = lf2_status();

            sum_r += s.raw_r;
            sum_c += s.raw_c;
            n++;
            (void)tk_dly_tsk((RELTIM)LF2_POLL_MS);
        }
        s1 = lf2_status();
        avg_r = (uint16_t)(sum_r / n);
        avg_c = (uint16_t)(sum_c / n);
        d_r = lf_norm(&s1.cal.r, avg_r);
        d_c = lf_norm(&s1.cal.c, avg_c);
        dbg_printf("[LF] RIGHT on black: raw %u = %u pm (DARK > %u)", avg_r, d_r, (unsigned)LF_DARK_MIN_PM);
        dbg_printf("[LF] CENTRE on white: raw %u = %u pm", avg_c, d_c);
        TEST_CHECK("S3-U12", d_r > LF_DARK_MIN_PM,
                   "(g) RIGHT reads solid black as DARK with the boot calibration (junctions can fire)");
    }

    /* (h) The whole run. */
    s1 = lf2_status();
    dbg_printf("[LF] totals: %lu samples, %lu missed, %lu dropped posts, %lu failed events",
               (unsigned long)s1.releases, (unsigned long)s1.missed,
               (unsigned long)s1.posts_dropped, (unsigned long)s1.ev_send_fail);
    TEST_CHECK("S3-U12", (s1.missed == 0U) && (s1.posts_dropped == 0U) && (s1.ev_send_fail == 0U),
               "(h) whole run: no missed period, no dropped post, no failed event");

    /* Leave the car stopped and the task disarmed for the later tests. */
    if (vc_get_system_state() != VC_SYS_READY_WAITING)
    {
        lf2_remote(DREP_NAV_REMOTE_STOP);
    }
    lf_task_test_arm(false);

    *p_pass = test_pass_;
    *p_fail = test_fail_;
}

#endif /* APP_TEST */
