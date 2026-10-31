/*
 *  task_vc.h
 *  Subsystem 2 -- internal interface to the Vehicle Controller task (P2).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Kernel side only: included by task_vc.c, subsystem2.c and
 *  subsystem2_test.c.  Never included by drv_*.c (rule R1).
 */

#ifndef TASK_VC_H
#define TASK_VC_H

#include "subsystem2.h"

/* VC authority mode (1 bit, DREP_FSM_VC_AUTH_STATES = 2). */
#define VC_AUTH_LINE            (0U)
#define VC_AUTH_AVOID           (1U)

/*
 * Reset the Vehicle Controller state machine to VC_SYS_INITIALISING and
 * authority to VC_AUTH_LINE.
 */
void vc_init(void);

/*
 * Complete the power-up self-test transition from VC_SYS_INITIALISING:
 *   passed == true  -> VC_SYS_READY_WAITING
 *   passed == false -> VC_SYS_SAFETY_FAULT_LATCHED
 */
void vc_complete_selftest(bool passed);

/*
 * Vehicle Controller RTOS task entry (TASK_PRI_VC = 2).
 * Sole reader of MBF_NAV_IN (IF-01); arbitrates Safety > Avoid > Barcode >
 * Line and sends resolved motion_cmd_t commands to MBF_MOTION_CMD (IF-02).
 * Never calls tk_ext_tsk() (rule R6).
 */
void vc_task_entry(INT stacd, void *p_exinf);

/*
 * Drain all currently queued messages in MBF_NAV_IN (with TMO_POL) and
 * update the safety latch / stationary timer at kernel time `now_ms`.
 * Used by vc_task_entry() after waking on a message and by unit tests
 * (S2-U22..S2-U27).
 */
void vc_drain_and_step(uint32_t now_ms);

/** True once vc_task_entry() has started. */
bool vc_is_task_running(void);

/** Current authority mode: VC_AUTH_LINE (0) or VC_AUTH_AVOID (1). */
uint32_t vc_get_authority(void);

/** Force authority mode for unit testing (S2-U23). */
void vc_set_authority(uint32_t auth);

/** Total invalid nav_input_t messages rejected from MBF_NAV_IN. */
uint32_t vc_get_reject_count(void);

/** Total motion_cmd_t messages sent by VC to MBF_MOTION_CMD. */
uint32_t vc_get_sent_cmd_count(void);

/** Last motion_cmd_t sent by VC to MBF_MOTION_CMD. */
motion_cmd_t vc_get_last_motion_cmd(void);

#endif /* TASK_VC_H */
