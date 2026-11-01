/*
 *  task_obs_avoid.h
 *  Subsystem 5 -- internal interface to the Obstacle Avoidance task (P7).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Kernel side only: included by task_obs_avoid.c, subsystem5.c and
 *  subsystem5_test.c.  Never included by a drv_*.c (rule R1).
 *
 *  The task consumes obstacle_profile_t from MBF_OBSTACLE (IF-03), decides
 *  which way to go round, and drives the manoeuvre by sending nav_input_t
 *  bypass commands into MBF_NAV_IN (IF-01) for the Vehicle Controller (S2)
 *  to execute.  It is the single setter of EVF_NAV_AVOID_ACTIVE and
 *  EVF_NAV_LINE_REACQUIRED (IF-07).
 *
 *  The manoeuvre is the classic four-turn bypass, each turn being one
 *  DREP_BYPASS_LEFT / DREP_BYPASS_RIGHT (the Vehicle Controller turns
 *  45 degrees per bypass turn command):
 *
 *      line ----X(obstacle)----------------------------
 *              /                        \
 *      -------/  out    along            \  back  -----
 *                    (parallel, offset)
 *
 *  TURN_OUT -> LEG_OUT -> TURN_IN1 -> LEG_ALONG -> TURN_IN2 -> LEG_BACK
 *  -> TURN_STRAIGHT -> RELEASE.  LEG_BACK ends as soon as subsystem 3 sets
 *  EVF_NAV_LINE_FOUND, or after LF_T_SEARCH_MS.
 *
 *  The whole manoeuvre is bounded by OBS_BYPASS_BUDGET_MS + LF_T_SEARCH_MS
 *  (= OBS_AVOID_WORST_CASE_MS, NFR14's 5 s hard limit).  Running out of
 *  budget stops the car and releases authority rather than carrying on.
 */

#ifndef TASK_OBS_AVOID_H
#define TASK_OBS_AVOID_H

#include "subsystem5.h"

/** Manoeuvre states, in order.  Also the payload of obs_avoid_get_state(). */
#define OBS_AV_IDLE             (0U)    /**< no obstacle; following the line */
#define OBS_AV_SLOW             (1U)    /**< obstacle seen, car slowed, deciding */
#define OBS_AV_TURN_OUT         (2U)
#define OBS_AV_LEG_OUT          (3U)
#define OBS_AV_TURN_IN1         (4U)
#define OBS_AV_LEG_ALONG        (5U)
#define OBS_AV_TURN_IN2         (6U)
#define OBS_AV_LEG_BACK         (7U)
#define OBS_AV_TURN_STRAIGHT    (8U)
#define OBS_AV_BLOCKED          (9U)    /**< no side has room: stop and back off */
#define OBS_AV_STATE_COUNT      (10U)

/** Bypass side. */
#define OBS_SIDE_LEFT           (0U)
#define OBS_SIDE_RIGHT          (1U)

/*
 * Obstacle Avoidance RTOS task entry (TASK_PRI_OBS_AVOID = 7).
 * Blocks on MBF_OBSTACLE with a timeout equal to the time left in the
 * current manoeuvre step, so it reacts to a new profile at once and never
 * burns a period polling.  Never calls tk_ext_tsk() (rule R6).
 */
void obs_avoid_task_entry(INT stacd, void *p_exinf);

/** Reset the manoeuvre state machine.  Called by subsystem5_init(). */
void obs_avoid_init(void);

/** True once obs_avoid_task_entry() has started. */
bool obs_avoid_is_running(void);

/* ---------------------------------------------------------------------------
 * Pure decisions.  No kernel calls: the on-target tests (S5-U15..S5-U20)
 * drive these directly.
 * ------------------------------------------------------------------------- */

/*
 * Choose which side to pass an obstacle, from one IF-03 profile.
 *   *p_side  OBS_SIDE_LEFT or OBS_SIDE_RIGHT (only written when true)
 * Returns false when neither side has OBS_BYPASS_CLEAR_MIN_MM of room, i.e.
 * the car is blocked.  A side needs that much clearance; when both qualify
 * the car goes the way the obstacle is NOT (bearing left of centre means
 * pass on the right).
 */
bool obs_choose_side(obstacle_profile_t prof, uint32_t *p_side);

/*
 * Pure: how far the car must travel on the diagonal out-leg, in
 * millimetres, to clear an obstacle of this width with
 * OBS_SIDE_MARGIN_MM to spare at a 45 degree bypass angle.
 */
uint32_t obs_leg_out_mm(uint32_t width_mm);

/** Pure: milliseconds to cover distance_mm at MOTION_SPEED_BYPASS_CMPS. */
uint32_t obs_mm_to_ms(uint32_t distance_mm);

/* ---------------------------------------------------------------------------
 * Test read-backs
 * ------------------------------------------------------------------------- */

/** Current manoeuvre state (OBS_AV_*). */
uint32_t obs_avoid_get_state(void);

/** Side chosen for the manoeuvre in progress (OBS_SIDE_*). */
uint32_t obs_avoid_get_side(void);

/** Completed bypass manoeuvres since start-up. */
uint32_t obs_avoid_get_bypass_count(void);

/** Manoeuvres abandoned because the budget ran out. */
uint32_t obs_avoid_get_abort_count(void);

/** Profiles received from MBF_OBSTACLE since start-up. */
uint32_t obs_avoid_get_profile_count(void);

/** nav_input_t messages successfully sent to MBF_NAV_IN. */
uint32_t obs_avoid_get_sent_count(void);

#endif /* TASK_OBS_AVOID_H */
