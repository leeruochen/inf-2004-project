/*
 *  task_motion.h
 *  Subsystem 2 -- internal interface to the Motion Control task (P3, 20 ms).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Kernel side only: included by task_motion.c, subsystem2.c and
 *  subsystem2_test.c.  Never included by drv_*.c (rule R1).
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "subsystem2.h"

/*
 * Reset Motion Control internal state and initialise the motor and encoder
 * drivers.  Safe to call from subsystem2_init() and from unit tests.
 */
void motion_control_init(void);

/*
 * Motion Control RTOS task entry (TASK_PRI_MOTION = 3, period
 * MOTION_PERIOD_MS = 20 ms via app_period_wait()).
 * Never calls tk_ext_tsk() (rule R6).
 */
void motion_task_entry(INT stacd, void *p_exinf);

/*
 * Execute one 20 ms control-loop cycle (poll MBF_MOTION_CMD, update hold
 * timer and PID/outputs, publish motion_state_t under MTX_MOTION).
 * Exposed so unit tests can step deterministically when the task is not
 * running, or run timed app_period_wait() cycles in motion_test_run_periods().
 */
void motion_step_cycle(uint32_t now_ms);

/*
 * Run `periods` drift-free 20 ms cycles using app_period_init() and
 * app_period_wait() (S2-U12..S2-U16).  If the background Motion task is
 * already running, waits `periods * MOTION_PERIOD_MS` ms instead.
 */
void motion_test_run_periods(uint32_t periods);

/** True once motion_task_entry() has started. */
bool motion_is_task_running(void);

/** Total periodic releases completed by Motion Control. */
uint32_t motion_get_release_count(void);

/** Total missed periodic releases reported by app_period_wait(). */
uint32_t motion_get_missed_count(void);

/** Total invalid motion_cmd_t messages rejected from MBF_MOTION_CMD. */
uint32_t motion_get_reject_count(void);

/** Current active speed setpoint in cm/s (0 when stopped or after T_hold). */
uint32_t motion_get_setpoint_cmps(void);

/** Current active command type (DREP_MC_*). */
uint32_t motion_get_cmd_type(void);

#endif /* TASK_MOTION_H */

