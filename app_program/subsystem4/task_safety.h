/*
 *  task_safety.h
 *  Subsystem 4 -- internal interface to the Safety task (P1).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Kernel side only: included by task_safety.c, subsystem4.c and
 *  subsystem4_test.c.  Never included by drv_*.c (rule R1).
 */

#ifndef TASK_SAFETY_H
#define TASK_SAFETY_H

#include "subsystem4.h"

/*
 * Safety RTOS task entry (TASK_PRI_SAFETY = 1).
 * Waits on EVF_SAFETY for COLLISION | IMU_ALIVE with TWF_ORW | TWF_BITCLR
 * (never TWF_CLR).  Brakes only through motion_emergency_brake() (IF-11).
 * Never takes MTX_MOTION.  Never calls tk_ext_tsk() (rule R6).
 */
void safety_task_entry(INT stacd, void *p_exinf);

/** True once safety_task_entry() has started. */
bool safety_is_task_running(void);

/** Total times the emergency brake was triggered since boot. */
uint32_t safety_get_brake_count(void);

/** Total IMU watchdog timeouts since boot. */
uint32_t safety_get_wdog_timeout_count(void);

#endif /* TASK_SAFETY_H */
