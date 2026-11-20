/*
 *  task_bc.h
 *  Subsystem 3 -- Barcode Decode task (priority TASK_PRI_BC = 5).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Kernel side only: included by task_bc.c, subsystem3.c and
 *  subsystem3_test.c, never by a drv_*.c (rule R1).
 *
 *  BC-2A: sensor -> decoded letter.  The task logs every read (letter or
 *  reason) and its raw gaps to the debug console and keeps counters.  It
 *  sends nothing to other tasks yet: the letter -> movement mapping (MQTT)
 *  and the IF-01 message are BC-2B.
 */

#ifndef TASK_BC_H
#define TASK_BC_H

#include "subsystem3.h"
#include "bc_frame.h"

/** Letters A-Z. */
#define BC_STATS_LETTERS        (26U)
/** bc_frame_result_t values (BC_FRAME_OK .. BC_FRAME_AMBIGUOUS). */
#define BC_STATS_RESULTS        (6U)

/** Counters since boot.  Written by the Barcode task only. */
typedef struct
{
    uint32_t captures;                      /**< captures decided */
    uint32_t results[BC_STATS_RESULTS];     /**< per bc_frame_result_t */
    uint32_t letters[BC_STATS_LETTERS];     /**< BC_FRAME_OK reads per letter */
    uint32_t max_gaps_per_poll;             /**< most gaps one poll popped */
    uint32_t max_poll_gap_us;               /**< longest time between two polls */
    char     last_letter;                   /**< last OK letter, '?' before any */
} bc_stats_t;

/** Task body (app_start_task()).  Never returns (rule R6). */
void bc_task_entry(INT stacd, void *p_exinf);

/** True once the task has entered its loop. */
bool bc_task_is_running(void);

/** Consistent copy of the counters, from any task (dispatch disabled during
 *  the copy, so the Barcode task cannot update them half-way). */
void bc_task_get_stats(bc_stats_t *p_out);

#if APP_TEST
/*
 * Bench tests only (S3-U04, S3-U06): use this speed (mm/s) for every gap
 * instead of Motion's.  On the bench the wheels do not turn, so Motion
 * publishes 0 or its commanded setpoint; a hand slide at a roughly steady
 * speed decodes with any constant value, because the decoder classifies by
 * width ratio.  0 = use Motion (normal).
 */
void bc_task_set_speed_override(int32_t speed_mmps);
#endif /* APP_TEST */

#endif /* TASK_BC_H */
