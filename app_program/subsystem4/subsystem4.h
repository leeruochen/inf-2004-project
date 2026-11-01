/*
 *  subsystem4.h
 *  Subsystem 4 -- IMU processing and safety / collision response.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  This header is subsystem 4's PUBLIC interface: only what other subsystems
 *  are allowed to use.  Everything else stays static in the .c files.  Other
 *  subsystems include it as "../subsystem4/subsystem4.h".
 *
 *  Include rule (check_rules R1): drv_*.c include SDK headers only and never
 *  include this header.  task_*.c, subsystem4.c and subsystem4_test.c include
 *  kernel headers only.
 *
 *  Tasks (config/priorities.h):
 *    Safety           TASK_PRI_SAFETY (1)  TASK_STACK_SAFETY_BYTES
 *    IMU Processing   TASK_PRI_IMU (8)     TASK_STACK_IMU_BYTES
 *
 *  task_safety.c lives in this folder.  It calls motion_emergency_brake()
 *  ONLY through ../subsystem2/subsystem2.h (IF-11); never drv_motor.* or
 *  PWM directly.  Safety never takes MTX_MOTION.
 *
 *  Interfaces (TEAM.md section 5).
 *    IF-06  EVF_SAFETY      owner.  IMU sets COLLISION and IMU_ALIVE; only
 *                           Safety sets FAULT_LATCHED; only the Vehicle
 *                           Controller (S2) clears it
 *    IF-04  MBF_TELEMETRY   sends hump and fault events (TMO_POL)
 *    IF-11  motion_emergency_brake() in subsystem 2
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/*
 * Latest pitch reading from the IMU in half-degree units (Q1).
 * value / 2 = degrees; positive = nose up.
 */
int32_t subsystem4_get_pitch_half_deg(void);

/*
 * Latest acceleration magnitude in milli-g.
 */
uint32_t subsystem4_get_accel_mag_mg(void);

/*
 * True if the IMU hardware was found on the I2C bus during init.
 */
bool subsystem4_imu_found(void);

#endif /* SUBSYSTEM4_H */
