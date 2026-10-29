/*
 *  subsystem3.h
 *  Subsystem 3 -- line following and barcode decode.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  This header is subsystem 3's PUBLIC interface: only what other subsystems
 *  are allowed to use.  Everything else stays static in the .c files.  Other
 *  subsystems include it as "../subsystem3/subsystem3.h".
 *
 *  Include rule (check_rules R1): drv_*.c include SDK headers only and never
 *  include this header.  task_*.c, subsystem3.c and subsystem3_test.c include
 *  kernel headers only.
 *
 *  Tasks (config/priorities.h):
 *    Line-Following   TASK_PRI_LF (4)   TASK_STACK_LF_BYTES
 *    Barcode Decode   TASK_PRI_BC (5)   TASK_STACK_BC_BYTES
 *
 *  Interfaces (TEAM.md section 5).  Every IPC object is created centrally by
 *  ipc_init() in usermain step 1 (Batch B), so no object IDs are declared yet.
 *    IF-01  MBF_NAV_IN      sends steering updates (TMO_POL) and barcode
 *                           commands (5 ms timeout) to the Vehicle Controller
 *    IF-07  EVF_NAV         sets LINE_FOUND
 *    IF-08  MTX_MOTION      Barcode Decode copies motion_state_t (S2)
 *    IF-09  barcode edge ring, barcode ISR -> Barcode Decode, internal (owner)
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#endif /* SUBSYSTEM3_H */
