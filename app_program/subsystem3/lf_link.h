/*
 *  lf_link.h
 *  Subsystem 3 -- what the line-following core needs from Motion Control's
 *  published state (pure C): forward distance, and when a junction turn
 *  has finished.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and pure project
 *  headers only, so tests/host/test_lf_link.c builds it unchanged.  The
 *  Line-Following task calls lf_link_step() once per 5 ms sample with the
 *  encoder counts from motion_get_state() (IF-08) and the manoeuvre implied
 *  by the Vehicle Controller's system state, then passes the result to
 *  lf_core_step().
 *
 *  Why the turn-done detector exists: on a JUNCTION with a barcode command
 *  pending, the Vehicle Controller enters NAV_TURN_LEFT / _RIGHT / U_TURN
 *  and sends a bounded turn to Motion.  Motion stops the wheels when the
 *  angle is reached, but nothing tells Line-Following; the Vehicle
 *  Controller stays in the turn state until the next steering post, and
 *  the core must not post steering while the turn runs (a post replaces
 *  the turn).  So the turn is "done" when the published encoder counts
 *  have moved and then stood still for LF_T_TURN_STILL_MS: Motion has
 *  stopped the wheels.  LF_T_TURN_MAX_MS bounds the wait if they never
 *  stop.
 */

#ifndef LF_LINK_H
#define LF_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"

/* ---------------------------------------------------------------------------
 * Tunables
 * ------------------------------------------------------------------------- */

/** Encoder counts unchanged this long after moving -> the turn is done.
 *  Motion publishes every MOTION_PERIOD_MS, so this is 3 publications
 *  without a change. TUNE */
#define LF_T_TURN_STILL_MS      (3U * MOTION_PERIOD_MS)
/** A turn is taken as done after this long even if the wheels never
 *  stopped (a 180 degree pivot at slow speed takes about 2 s). TUNE */
#define LF_T_TURN_MAX_MS        (5000U)

/* ---------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

/** Link state.  Private: fields are read only by lf_link.c (and tests). */
typedef struct
{
    bool           primed;      /**< false until the first sample */
    int32_t        prev[2];     /**< last encoder counts, [0] left, [1] right */
    uint32_t       fwd_cnt;     /**< forward counts so far (free-running, wraps) */
    uint32_t       half;        /**< 0 or 1: half a count carried over */
    lf_manoeuvre_t man;         /**< manoeuvre being watched (NONE = none) */
    bool           moved;       /**< the counts changed since it started */
    bool           done;        /**< the turn has finished */
    uint32_t       still_n;     /**< samples since the counts last changed */
    uint32_t       turn_n;      /**< samples since the turn started */
} lf_link_t;

/** Result of one sample, for lf_in_t. */
typedef struct
{
    lf_manoeuvre_t manoeuvre;   /**< vc_man, or NONE once the turn is done */
    uint32_t       fwd_cnt;     /**< forward travel, encoder counts (wraps) */
} lf_link_out_t;

/* ---------------------------------------------------------------------------
 * Functions
 * ------------------------------------------------------------------------- */

/* Reset: no counts seen yet, no turn watched. */
void lf_link_init(lf_link_t *p_link);

/*
 * One sample.
 *   vc_man     the manoeuvre the Vehicle Controller's state says is running
 *              (LF_MAN_NONE outside NAV_TURN_LEFT / _RIGHT / U_TURN);
 *   enc_count  Motion's published counts, [0] left, [1] right (IF-08,
 *              free-running int32, forward counts up).
 * Forward travel grows by the mean of the two wheels' changes when that
 * mean is positive, so a pivot (wheels opposite) or reversing adds nothing.
 * Counts are compared as unsigned differences, so their wrap is harmless.
 */
void lf_link_step(lf_link_t *p_link, lf_manoeuvre_t vc_man,
                  int32_t const enc_count[2], lf_link_out_t *p_out);

#endif /* LF_LINK_H */
