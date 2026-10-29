/*
 *  subsystem1.h
 *  Subsystem 1 -- telemetry (WiFi, MQTT, dashboard link).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  This header is subsystem 1's PUBLIC interface: only what other subsystems
 *  are allowed to use.  Everything else stays static in the .c files.  Other
 *  subsystems include it as "../subsystem1/subsystem1.h".
 *
 *  Include rule (check_rules R1): drv_*.c include SDK headers only and never
 *  include this header.  task_*.c, subsystem1.c and subsystem1_test.c include
 *  kernel headers only.
 *
 *  Tasks (config/priorities.h):
 *    Telemetry   TASK_PRI_TEL (9)   TASK_STACK_TEL_BYTES
 *
 *  Interfaces (TEAM.md section 5).  Every IPC object is created centrally by
 *  ipc_init() in usermain step 1 (Batch B), so no object IDs are declared yet.
 *    IF-04  MBF_TELEMETRY   receives tel_evt_t from every subsystem (owner)
 *    IF-01  MBF_NAV_IN      sends parsed remote commands (start, stop, reset)
 *                           to the Vehicle Controller (S2)
 *    IF-08  MTX_MOTION      copies motion_state_t (S2) and releases at once
 *
 *  TODO(owner-to-confirm, buddy 1): delta D2 (20 ms telemetry wake) is NOT
 *  approved; the wake period stays open (config/timing.h, ARCHITECTURE.md).
 *  Phase 2 WiFi: the template's CYW43/lwIP/MQTT code is reference only
 *  (docs/TEMPLATE_NOTES.md).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#endif /* SUBSYSTEM1_H */
