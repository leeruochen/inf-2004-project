/*
 *  task_lf.h
 *  Subsystem 3 -- Line-Following Control task (priority 4, 5 ms).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side: kernel headers only (rule R1).  Included by subsystem3.c,
 *  task_lf.c and the on-target tests; not part of the public interface.
 *
 *  The task is the thin shell around the pure, host-tested modules: every
 *  LF_SAMPLE_PERIOD_MS it reads both line sensors (drv_ir.h), Motion's
 *  published state (IF-08) and the Vehicle Controller's system state, runs
 *  lf_link_step(), lf_link_post_slot() and lf_core_step(), and acts on the
 *  result:
 *    - steering -> MBF_NAV_IN (IF-01), TMO_POL (design.md 3.3: a dropped
 *      update is replaced 20 ms later);
 *    - JUNCTION / TRACK_LOST -> MBF_NAV_IN, NAV_MBF_SEND_TMO_MS (one-off
 *      events: wait a little, log if it still fails);
 *    - EVF_NAV_LINE_FOUND (IF-07): set / cleared when it changes; this task
 *      is the flag's only setter.
 *  It calibrates once, at boot, before its first sample: the car must be in
 *  the start pose (CENTRE over the middle of the black line, RIGHT over
 *  white) when it is powered up or reset.
 *
 *  After it starts, this task is the only reader of the line-sensor ADC
 *  (the LF-1 measurement test S3-U09 is therefore not run any more; see
 *  subsystem3_test.c).
 */

#ifndef TASK_LF_H
#define TASK_LF_H

#include "subsystem3.h"

#include "lf_core.h"

/** What the task has done so far (for the console and the on-target
 *  tests).  Written only by the Line-Following task. */
typedef struct
{
    uint32_t        releases;       /**< samples run */
    uint32_t        missed;         /**< periods missed (app_period_wait) */
    lf_cal_status_t cal_status;     /**< boot calibration verdict */
    lf_cal_t        cal;            /**< the calibration (if LF_CAL_OK) */
    uint16_t        cal_c_avg;      /**< boot: CENTRE average (on the line) */
    uint16_t        cal_r_avg;      /**< boot: RIGHT average (on white) */
    uint16_t        raw_c;          /**< last CENTRE sample */
    uint16_t        raw_r;          /**< last RIGHT sample */
    lf_state_t      state;          /**< core state after the last sample */
    uint32_t        vc_state;       /**< VC system state seen (VC_SYS_*) */
    int32_t         last_steer_q7;  /**< last steering posted */
    uint32_t        posts_slot;     /**< steering posts made in Motion's post slot */
    uint32_t        posts_entry;    /**< steering posts made on state entry, off the slot */
    uint32_t        posts_dropped;  /**< steering posts the buffer refused (TMO_POL) */
    uint32_t        n_junction;     /**< JUNCTION events sent */
    uint32_t        n_track_lost;   /**< TRACK_LOST events sent */
    uint32_t        ev_send_fail;   /**< events not sent within NAV_MBF_SEND_TMO_MS */
    bool            line_found;     /**< EVF_NAV_LINE_FOUND as last set */
    uint32_t        t_entry_ms[LF_ST_COUNT]; /**< kernel time each state was last entered */
    uint32_t        t_track_lost_ms;/**< kernel time of the last TRACK_LOST */
} lf_task_status_t;

/* Start the Line-Following task (subsystem3_start()).  Returns the task ID,
 * or a negative kernel error code. */
int32_t lf_task_start(void);

/* Copy the task's status.  Safe from any task of lower priority: the copy
 * runs with dispatch disabled, so the Line-Following task cannot update the
 * status halfway through it. */
void lf_task_get_status(lf_task_status_t *p_out);

#if APP_TEST
/*
 * APP_TEST builds only.  The other subsystems' on-target tests drive the
 * Vehicle Controller directly (start, stop, fault) before Subsystem 3's
 * tests run, so until armed the task samples but treats the car as not
 * running: it sends nothing and leaves EVF_NAV alone.  Normal builds are
 * always armed.  Disarming clears LINE_FOUND.
 */
void lf_task_test_arm(bool armed);

/*
 * Run the boot calibration again, now, and return when it is done.  The task
 * does it, so nothing else touches the core or the ADC.  Only with the VC
 * READY and the car in the start pose: it resets the core.  The power-up
 * reading is taken with the car still being handled and has read up to 40 %
 * high, so the on-track test calibrates at pose A instead.
 */
void lf_task_test_recalibrate(void);

/* APP_TEST only: replace the calibration with *p_cal (e.g. the two-pose
 * calibration, lf_cal_from_two_poses) and restart the core with it, at the
 * task's next sample.  Waits until applied (at most LF_RECAL_WAIT_MS). */
void lf_task_test_set_cal(lf_cal_t const *p_cal);
#endif /* APP_TEST */

#endif /* TASK_LF_H */
