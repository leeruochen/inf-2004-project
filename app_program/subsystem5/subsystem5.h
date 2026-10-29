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
 *  ipc_init() in usermain step 1 (Batch B), so no object IDs are declared yet.
 *    IF-03  MBF_OBSTACLE    Scanning -> Avoidance, internal (owner)
 *    IF-05  SEM_ECHO        echo ISR -> Obstacle Scanning (owner)
 *    IF-07  EVF_NAV         sets AVOID_ACTIVE and LINE_REACQUIRED (owner)
 *    IF-01  MBF_NAV_IN      sends bypass commands (5 ms timeout) to the
 *                           Vehicle Controller (S2)
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#endif /* SUBSYSTEM5_H */
