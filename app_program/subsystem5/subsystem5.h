/*
 *  subsystem5.h
 *  Subsystem 5 -- obstacle scanning and obstacle avoidance / recovery.
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  This header is subsystem 5's PUBLIC interface: only what other subsystems
 *  are allowed to use.  Everything else stays static in the .c files.  Other
 *  subsystems include it as "../subsystem5/subsystem5.h".
 *
 *  Include rule (check_rules R1): drv_*.c include SDK headers only and never
 *  include this header.  task_*.c, subsystem5.c and subsystem5_test.c include
 *  kernel headers only.
 *
 *  Tasks (config/priorities.h):
 *    Obstacle Scanning    TASK_PRI_OBS_SCAN (6)   TASK_STACK_OBS_SCAN_BYTES
 *    Obstacle Avoidance   TASK_PRI_OBS_AVOID (7)  TASK_STACK_OBS_AVOID_BYTES
 *
 *  Interfaces (TEAM.md section 5).  Every IPC object is created centrally by
 *  ipc_init() in usermain step 1 (Batch B), so no object IDs are declared
 *  here.
 *    IF-03  MBF_OBSTACLE    Scanning -> Avoidance, internal (owner)
 *    IF-05  SEM_ECHO        echo ISR -> Obstacle Scanning (owner)
 *    IF-07  EVF_NAV         sets AVOID_ACTIVE and LINE_REACQUIRED (owner)
 *    IF-01  MBF_NAV_IN      sends bypass commands (5 ms timeout) to the
 *                           Vehicle Controller (S2)
 *    IF-06  EVF_SAFETY      sets and clears STATUS_NO_ECHO (b7, owner);
 *                           reads COLLISION and FAULT_LATCHED to abort
 *
 *  Internal layout:
 *    drv_servo.c/.h        SDK side, PWM slice 7 B on GP15
 *    drv_ultrasonic.c/.h   SDK side, GP4 trigger / GP5 echo + echo ISR
 *    task_obs_scan.c/.h    the servo sweep and obstacle profiling
 *    task_obs_avoid.c/.h   the bypass manoeuvre and line reacquisition
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

/*
 * Power-up sensor check, for the Vehicle Controller's self-test gate
 * (vc_complete_selftest(), subsystem 2).  True when the servo PWM came up at
 * SERVO_PWM_FREQ_HZ, the echo handler registered, and the echo line was not
 * stuck HIGH at start-up (which means the sensor is unpowered, miswired, or
 * missing its 3.3 V divider).  Valid only after subsystem5_init().
 */
bool subsystem5_sensors_ok(void);

/*
 * Current obstacle-avoidance manoeuvre state (OBS_AV_* in task_obs_avoid.h),
 * for Telemetry (subsystem 1).  OBS_AV_IDLE (0) when the car is not avoiding.
 */
uint32_t subsystem5_get_avoid_state(void);

/*
 * True while a bypass manoeuvre has authority over navigation, i.e. while
 * EVF_NAV_AVOID_ACTIVE is set by this subsystem.  Readers may equally poll
 * EVF_NAV themselves; this is the cheaper call.
 */
bool subsystem5_avoid_is_active(void);

#endif /* SUBSYSTEM5_H */
