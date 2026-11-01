/*
 *  task_imu.h
 *  Subsystem 4 -- internal interface to the IMU Processing task (P8, 10 ms).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Kernel side only: included by task_imu.c, subsystem4.c and
 *  subsystem4_test.c.  Never included by drv_*.c (rule R1).
 */

#ifndef TASK_IMU_H
#define TASK_IMU_H

#include "subsystem4.h"

/*
 * Initialise the IMU driver (drv_imu.c) and reset internal state.
 * Safe to call from subsystem4_init() and from unit tests.
 * Returns true if the IMU hardware was found on the bus.
 */
bool imu_task_init(void);

/*
 * IMU Processing RTOS task entry (TASK_PRI_IMU = 8, period
 * IMU_SAMPLE_PERIOD_MS = 10 ms via app_period_wait()).
 * Never calls tk_ext_tsk() (rule R6).
 */
void imu_task_entry(INT stacd, void *p_exinf);

/** True once imu_task_entry() has started. */
bool imu_is_task_running(void);

/** Latest pitch in half-degree units (Q1: value / 2 = degrees). */
int32_t imu_get_pitch_half_deg(void);

/** Latest acceleration magnitude in milli-g. */
uint32_t imu_get_accel_mag_mg(void);

/** Total periodic releases completed by the IMU task. */
uint32_t imu_get_release_count(void);

/** Total I2C read failures since boot. */
uint32_t imu_get_i2c_error_count(void);

#endif /* TASK_IMU_H */
