/*
 *  task_lf.c
 *  Subsystem 3 -- Line-Following Control task (see task_lf.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel and pure project headers only, never SDK headers
 *  (rule R1); the ADC is reached through the kernel-free drv_ir.h.
 *
 *  Scheduling: periodic, LF_SAMPLE_PERIOD_MS (5 ms, NFR7), with
 *  app_period_wait() (drift-free; ARCHITECTURE.md T8), at TASK_PRI_LF (4),
 *  directly below Motion (3) and the Vehicle Controller (2).
 *
 *  Order inside one sample, and why:
 *    1. motion_get_state() first, then the time.  Motion's tick_ms is then
 *       never later than now_ms, which lf_link_post_slot() relies on (a
 *       Motion release between the two reads is seen as "just read").
 *    2. Sensors, VC state and EVF_NAV, then the pure modules.
 *    3. Sends.  The VC (priority 2) blocks on MBF_NAV_IN, so it takes each
 *       message inside our tk_snd_mbf() and forwards it to Motion at once.
 *    4. The status copy, last, with no blocking call inside it: a reader
 *       of lower priority (lf_task_get_status) can only run while this task
 *       waits, so it never sees a half-written status except by being
 *       preempted itself, which it prevents with tk_dis_dsp().
 */

#include "task_lf.h"

#include "../subsystem2/subsystem2.h"   /* motion_get_state (IF-08), vc_get_system_state */
#include "config/pins.h"                /* IR_C_ADC_CH, IR_R_ADC_CH */
#include "config/timing.h"              /* LF_SAMPLE_PERIOD_MS, NAV_MBF_SEND_TMO_MS */
#include "drv_ir.h"                     /* kernel-free ADC driver */
#include "lf_core.h"
#include "lf_link.h"
#include "lf_sanity.h"
#ifndef S3_LF_STRADDLE
#define S3_LF_STRADDLE          (0)
#endif
#if S3_LF_STRADDLE
#include "lf_straddle.h"                /* straddle line-following core (D14) */
#endif
#include "../subsystem1/subsystem1.h"   /* tel_set_ir_raw (IF-12, dashboard integration) */

/** Boot calibration: samples per sensor, one per LF_SAMPLE_PERIOD_MS
 *  (32 x 5 ms = 160 ms in the start pose). */
#define LF_CAL_SAMPLES          (32U)
/** The test waits at most this long for a recalibration: it takes 160 ms. */
#define LF_RECAL_WAIT_MS        (1000U)

/* The pure modules' state.  Static, not on the 1.5 KB task stack.  The
 * straddle build (S3_LF_STRADDLE) runs lf_straddle.c's core in
 * place of lf_core.c's, behind the same inputs and outputs. */
#if S3_LF_STRADDLE
static lf_sd_t          s_core;
#else
static lf_core_t        s_core;
#endif
static lf_link_t        s_link;
static motion_state_t   s_ms;           /* last good Motion state copy */
static lf_task_status_t s_st;           /* written by this task only */

#if APP_TEST
/* Written by the test task (priority 11), read once per sample here: a
 * single aligned byte, so no lock; volatile so each sample re-reads it. */
static volatile bool    s_armed = false;
/* Set by the test task, cleared here when the calibration has run. */
static volatile bool    s_recal = false;
/* A calibration from the test task (two-pose, lf_cal_from_two_poses):
 * s_cal_new is written by the test task with dispatching disabled, then
 * s_cal_set; this task applies it at its next sample and clears the flag. */
static lf_cal_t         s_cal_new;
static volatile bool    s_cal_set = false;
#else
#define s_armed         (true)
#endif
static bool             s_was_armed = false;    /* task only */

/** State names for the console (lf_state_t order). */
static char const *const k_state_name[LF_ST_COUNT] =
{
    "IDLE", "ON_LINE", "JUNCTION", "LINE_LOST", "REACQUIRING", "SUSPENDED"
};

/* ---------------------------------------------------------------------------
 * Inputs
 * ------------------------------------------------------------------------- */

/* The car is in autonomous navigation: every NAV substate, and also
 * OBSTACLE_AVOIDANCE.  Without the avoidance state the core would go IDLE
 * during a bypass instead of SUSPENDED_BY_AVOIDANCE, and the LINE_FOUND /
 * LINE_REACQUIRED handoff (design.md 6.1, FR13) would break. */
static bool lf_vc_running(uint32_t vc_state)
{
    return ((vc_state >= VC_SYS_NAV_LINE_TRACKING) && (vc_state <= VC_SYS_NAV_TURN_RIGHT))
           || (vc_state == VC_SYS_OBSTACLE_AVOIDANCE);
}

/* The junction turn the VC's state says is running (lf_link watches it
 * until the wheels stop). */
static lf_manoeuvre_t lf_vc_manoeuvre(uint32_t vc_state)
{
    lf_manoeuvre_t man = LF_MAN_NONE;

    if ((vc_state == VC_SYS_NAV_TURN_LEFT) || (vc_state == VC_SYS_NAV_TURN_RIGHT))
    {
        man = LF_MAN_TURN;
    }
    else if (vc_state == VC_SYS_NAV_U_TURN)
    {
        man = LF_MAN_UTURN;
    }
    else
    {
        /* LINE_TRACKING, HUMP_SENSING, GO_STRAIGHT, or not navigating */
    }
    return man;
}

static uint32_t lf_read_nav_flags(void)
{
    T_RFLG ref;

    if (tk_ref_flg(EVF_NAV, &ref) < (ER)E_OK)
    {
        return 0U;
    }
    return (uint32_t)ref.flgptn;
}

/* ---------------------------------------------------------------------------
 * Outputs
 * ------------------------------------------------------------------------- */

static void lf_set_line_found(bool found)
{
    ER er;

    if (found)
    {
        er = tk_set_flg(EVF_NAV, (UINT)EVF_NAV_LINE_FOUND);
    }
    else
    {
        er = tk_clr_flg(EVF_NAV, ~(UINT)EVF_NAV_LINE_FOUND);
    }
    if (er != E_OK)
    {
        dbg_put_err("[LF] EVF_NAV LINE_FOUND", (int32_t)er);
    }
    s_st.line_found = found;
}

/* One-off event: wait up to NAV_MBF_SEND_TMO_MS (design.md 3.3).  A
 * timeout costs at most one 5 ms sample; app_period_wait() then counts the
 * missed period. */
static bool lf_send_event(uint32_t type)
{
    nav_input_t const m = drep_nav_line_event(type);
    ER const er = tk_snd_mbf(MBF_NAV_IN, &m, (INT)sizeof(m), (TMO)NAV_MBF_SEND_TMO_MS);

    if (er != E_OK)
    {
        s_st.ev_send_fail++;
        dbg_put_err("[LF] event send to MBF_NAV_IN", (int32_t)er);
    }
    return (er == E_OK);
}

/* Steering: never wait (TMO_POL); the next post replaces a dropped one. */
static void lf_send_steer(int32_t steer_q7, bool in_slot)
{
    nav_input_t const m = drep_nav_steer(steer_q7);

    if (tk_snd_mbf(MBF_NAV_IN, &m, (INT)sizeof(m), TMO_POL) == E_OK)
    {
        s_st.last_steer_q7 = steer_q7;
        if (in_slot)
        {
            s_st.posts_slot++;
        }
        else
        {
            s_st.posts_entry++;
        }
    }
    else
    {
        s_st.posts_dropped++;
    }
}

/* ---------------------------------------------------------------------------
 * Boot calibration
 * ------------------------------------------------------------------------- */

/* LF_CAL_SAMPLES of each sensor in the start pose, then the core's
 * calibration rule (lf_cal_from_start_pose: refuses a bad sensor or too
 * little contrast).  The core is set up either way; refused means it never
 * steers and reports TRACK_LOST once at run start, so the VC stops. */
static void lf_calibrate(void)
{
    lf_adc_stats_t sc;
    lf_adc_stats_t sr;
    app_period_t   per;
    lf_cal_t       cal;

    lf_adc_stats_reset(&sc);
    lf_adc_stats_reset(&sr);
    app_period_init(&per, LF_SAMPLE_PERIOD_MS);
    for (uint32_t i = 0U; i < LF_CAL_SAMPLES; i++)
    {
        (void)app_period_wait(&per);
        lf_adc_stats_add(&sc, drv_ir_read(IR_C_ADC_CH));
        lf_adc_stats_add(&sr, drv_ir_read(IR_R_ADC_CH));
    }

    cal.c.white = 0U;
    cal.c.black = 0U;
    cal.r = cal.c;
    s_st.cal_status = lf_cal_from_start_pose(&sc, &sr, &cal);
    s_st.cal = cal;
    s_st.cal_c_avg = lf_adc_stats_avg(&sc);
    s_st.cal_r_avg = lf_adc_stats_avg(&sr);
#if S3_LF_STRADDLE
    lf_sd_init(&s_core, (s_st.cal_status == LF_CAL_OK) ? &cal : NULL);
#else
    lf_core_init(&s_core, (s_st.cal_status == LF_CAL_OK) ? &cal : NULL);
#endif

    dbg_printf("[LF] boot calibration %s: C avg %u (%u..%u), R avg %u (%u..%u)",
               (s_st.cal_status == LF_CAL_OK) ? "OK"
               : ((s_st.cal_status == LF_CAL_BAD_SENSOR) ? "REFUSED bad sensor"
                  : "REFUSED no contrast"),
               s_st.cal_c_avg, sc.min, sc.max, s_st.cal_r_avg, sr.min, sr.max);
    if (s_st.cal_status == LF_CAL_OK)
    {
        dbg_printf("[LF] cal white %u black %u (both sensors)", cal.c.white, cal.c.black);
    }
}

/* ---------------------------------------------------------------------------
 * One sample
 * ------------------------------------------------------------------------- */

static void lf_sample(void)
{
    lf_in_t        in;
    lf_out_t       out;
    lf_link_out_t  lk;
    uint32_t       now_ms;
    uint32_t       vc_state;
    uint32_t       flags;
    bool const     armed = s_armed;
    lf_state_t const prev_state = s_st.state;

    /* 1. Motion's state, then the time (see the file header). */
    if (motion_get_state(&s_ms) != E_OK)
    {
        /* keep the previous copy: counts unchanged, slot still predicted */
    }
    now_ms = app_period_now_ms();

    /* 2. Inputs, then the pure modules. */
    vc_state = vc_get_system_state();
    flags = lf_read_nav_flags();

    in.raw_c = drv_ir_read(IR_C_ADC_CH);
    in.raw_r = drv_ir_read(IR_R_ADC_CH);
    in.running = armed && lf_vc_running(vc_state);
    in.avoid_active = ((flags & EVF_NAV_AVOID_ACTIVE) != 0U);
    in.reacquired = ((flags & EVF_NAV_LINE_REACQUIRED) != 0U);

    lf_link_step(&s_link, lf_vc_manoeuvre(vc_state), s_ms.enc_count, &lk);
    in.manoeuvre = lk.manoeuvre;
    in.fwd_cnt = lk.fwd_cnt;
    in.yaw_cnt = lk.yaw_cnt;
    in.post_slot = lf_link_post_slot(s_ms.tick_ms, now_ms);

#if S3_LF_STRADDLE
    lf_sd_step(&s_core, &in, &out);
#else
    lf_core_step(&s_core, &in, &out);
#endif

    /* 3. Act.  While disarmed (APP_TEST) the core is IDLE: nothing to send. */
    if (armed)
    {
        if (out.event == LF_EV_JUNCTION)
        {
            if (lf_send_event(DREP_NAV_LINE_JUNCTION))
            {
                s_st.n_junction++;
            }
        }
        else if (out.event == LF_EV_TRACK_LOST)
        {
            if (lf_send_event(DREP_NAV_LINE_TRACK_LOST))
            {
                s_st.n_track_lost++;
                s_st.t_track_lost_ms = now_ms;
            }
        }
        else
        {
            /* LF_EV_NONE */
        }
        if (out.post_steer)
        {
            lf_send_steer(out.steer_q7, in.post_slot);
        }
        if (out.line_found != s_st.line_found)
        {
            lf_set_line_found(out.line_found);
        }
    }
    else if (s_was_armed && s_st.line_found)
    {
        lf_set_line_found(false);           /* just disarmed (APP_TEST) */
    }
    else
    {
        /* disarmed: EVF_NAV left alone */
    }
    s_was_armed = armed;

    if (out.state != prev_state)
    {
        dbg_printf("[LF] %s -> %s  c=%u r=%u dc=%u dr=%u vc=%lu",
                   k_state_name[prev_state], k_state_name[out.state],
                   in.raw_c, in.raw_r, out.d_c_pm, out.d_r_pm, (unsigned long)vc_state);
    }

    /* 4. Status: no blocking call from here to the end of the sample. */
    s_st.releases++;
    s_st.raw_c = in.raw_c;
    s_st.raw_r = in.raw_r;
    tel_set_ir_raw(in.raw_r, in.raw_c);     /* IF-12: newest raws for the 200 ms sensor frame; one store */
    s_st.state = out.state;
    s_st.vc_state = vc_state;
    if (out.state != prev_state)
    {
        s_st.t_entry_ms[out.state] = now_ms;
    }
}

/* ---------------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------------- */

static void lf_task(INT stacd, void *p_exinf)
{
    app_period_t per;

    (void)stacd;
    (void)p_exinf;

    lf_link_init(&s_link);
    lf_calibrate();
    app_period_init(&per, LF_SAMPLE_PERIOD_MS);

    for (;;)
    {
        uint32_t const missed = app_period_wait(&per);

        if (missed > 0U)
        {
            s_st.missed += missed;
            dbg_printf("[LF] period overrun: %lu missed", (unsigned long)missed);
        }
#if APP_TEST
        if (s_recal)
        {
            /* Runs its own 5 ms grid for 160 ms: restart ours afterwards, or
             * those periods would count as missed. */
            lf_calibrate();
            app_period_init(&per, LF_SAMPLE_PERIOD_MS);
            s_recal = false;
        }
        if (s_cal_set)
        {
            s_st.cal = s_cal_new;
            s_st.cal_status = LF_CAL_OK;
#if S3_LF_STRADDLE
            lf_sd_init(&s_core, &s_cal_new);
#else
            lf_core_init(&s_core, &s_cal_new);
#endif
            dbg_printf("[LF] two-pose calibration in use: C white %u black %u, R white %u black %u",
                       s_cal_new.c.white, s_cal_new.c.black, s_cal_new.r.white, s_cal_new.r.black);
            s_cal_set = false;
        }
#endif
        lf_sample();
    }
    /* Never returns, never calls tk_ext_tsk() (rule R6). */
}

int32_t lf_task_start(void)
{
    s_st.state = LF_ST_IDLE;
    s_st.cal_status = LF_CAL_NO_CONTRAST;   /* until the boot calibration ran */
    return app_start_task(lf_task, TASK_PRI_LF, TASK_STACK_LF_BYTES, "lf");
}

void lf_task_get_status(lf_task_status_t *p_out)
{
    if (p_out != NULL)
    {
        (void)tk_dis_dsp();
        *p_out = s_st;
        (void)tk_ena_dsp();
    }
}

#if APP_TEST
void lf_task_test_arm(bool armed)
{
    s_armed = armed;    /* the task clears LINE_FOUND itself on disarming */
}

void lf_task_test_set_cal(lf_cal_t const *p_cal)
{
    uint32_t waited_ms = 0U;

    if (p_cal != NULL)
    {
        (void)tk_dis_dsp();
        s_cal_new = *p_cal;
        s_cal_set = true;
        (void)tk_ena_dsp();
        while (s_cal_set && (waited_ms < LF_RECAL_WAIT_MS))
        {
            (void)tk_dly_tsk((RELTIM)LF_SAMPLE_PERIOD_MS);
            waited_ms += LF_SAMPLE_PERIOD_MS;
        }
    }
}

void lf_task_test_recalibrate(void)
{
    uint32_t waited_ms = 0U;

    s_recal = true;
    while (s_recal && (waited_ms < LF_RECAL_WAIT_MS))
    {
        (void)tk_dly_tsk((RELTIM)LF_SAMPLE_PERIOD_MS);
        waited_ms += LF_SAMPLE_PERIOD_MS;
    }
}
#endif /* APP_TEST */
