/*
 *  subsystem2.h
 *  Subsystem 2 -- motion control and Vehicle Controller.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *  design.md 3.1: Vehicle Controller owner = Buddy 2; Motion Control = Buddy 2.
 *
 *  This header is subsystem 2's PUBLIC interface: only what other subsystems
 *  are allowed to use.  Everything else stays static in the .c files.  Other
 *  subsystems include it as "../subsystem2/subsystem2.h".
 *
 *  Include rule (check_rules R1): drv_*.c include SDK headers only and never
 *  include this header.  task_*.c, subsystem2.c and subsystem2_test.c include
 *  kernel headers only.
 *
 *  Tasks (config/priorities.h):
 *    Vehicle Controller  TASK_PRI_VC (2)      TASK_STACK_VC_BYTES
 *    Motion Control      TASK_PRI_MOTION (3)  TASK_STACK_MOTION_BYTES
 *
 *  Interfaces (TEAM.md section 5).  Every IPC object is created centrally by
 *  ipc_init() in usermain step 1 (Batch B), so no object IDs are declared yet.
 *    IF-01  MBF_NAV_IN      Vehicle Controller receives nav_input_t (owner)
 *    IF-02  MBF_MOTION_CMD  VC -> Motion, internal (owner)
 *    IF-08  MTX_MOTION      Motion writes motion_state_t (owner)
 *    IF-10  encoder counts  owned by Motion Control (owner)
 *    IF-06  EVF_SAFETY      VC clears FAULT_LATCHED (reset while stationary)
 *    IF-11  motion_emergency_brake(), called by Safety (S4), below
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/*
 * IF-11: emergency brake, called by the Safety task (subsystem 4) ONLY
 * through this header.  Drives all four motor channels LOW (ROBO PICO brake)
 * inside a critical section, safe from any task and before motor_init().
 */
void motion_emergency_brake(void);

/*
 * IF-08 / NFR6: shared motion state guarded by MTX_MOTION.
 * Readers may either call motion_get_state(&copy) or lock MTX_MOTION, copy
 * g_motion_state, and unlock MTX_MOTION at once.  Safety never takes
 * MTX_MOTION.
 */
extern motion_state_t g_motion_state;
ER motion_get_state(motion_state_t *p_out);

/* ---------------------------------------------------------------------------
 * 10-state System FSM (4 bits flat, docs/DATA_REPRESENTATION.md section 4,
 * design.md section 5).  Owned by the Vehicle Controller (roadmap Q1);
 * Telemetry reads vc_get_system_state() for the heartbeat frame.
 * ------------------------------------------------------------------------- */
#define VC_SYS_INITIALISING             (0U)
#define VC_SYS_READY_WAITING            (1U)
#define VC_SYS_OBSTACLE_AVOIDANCE       (2U)
#define VC_SYS_SAFETY_FAULT_LATCHED     (3U)
#define VC_SYS_NAV_LINE_TRACKING        (4U)
#define VC_SYS_NAV_HUMP_SENSING         (5U)
#define VC_SYS_NAV_TURN_LEFT            (6U)
#define VC_SYS_NAV_GO_STRAIGHT          (7U)
#define VC_SYS_NAV_U_TURN               (8U)
#define VC_SYS_NAV_TURN_RIGHT           (9U)
#define VC_SYS_STATE_COUNT              (10U)

_Static_assert(VC_SYS_STATE_COUNT == DREP_FSM_SYSTEM_STATES,
               "System FSM state count must match DREP_FSM_SYSTEM_STATES (10)");

/** Current 4-bit system state (VC_SYS_*). */
uint32_t vc_get_system_state(void);

#endif /* SUBSYSTEM2_H */
