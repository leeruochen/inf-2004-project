/*
 *  imu_lift.h
 *  Subsystem 4 -- lift / overturn detector (pure C).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and pure project
 *  headers only, so tests/host/test_imu_lift.c builds it unchanged.  The IMU
 *  Processing task calls imu_lift_step() once per 10 ms sample and sets
 *  EVF_SAFETY_LIFTED on an event; nothing here touches the kernel or I2C.
 *
 *  Why (requirement change, professor's update of 11 Nov 2026): collision
 *  detection with the IMU is removed.  The IMU must instead detect that the
 *  robot has been lifted or overturned and stop the motors at once; lifting
 *  the robot ends the run at the demonstration.
 *
 *  How:  The accelerometer has no gyro, so a car held still and level in the
 *  air reads 1 g, exactly like a car on the floor.  What can be seen is the
 *  act of picking it up or turning it over.  Compared with the "up" vector
 *  (gravity measured when the car stood flat and still at boot), a sample
 *  is suspicious when
 *    TILTED  the car leans more than IMU_LIFT_TILT_DEG (30 deg) from level
 *            in any direction.  Hands almost always tilt a car that is
 *            picked up, and an overturned car is tilted past 90 deg, so
 *            this also covers "overturned".  Humps tilt it ~10-15 deg.
 *    RAISED  the acceleration along "up" is more than IMU_LIFT_ACCEL_G_X1000
 *            (0.3 g) above 1 g: the car is being pulled upwards.
 *  The caller passes the low-passed (80 ms) vector, so motor vibration is
 *  already smoothed out.  IMU_LIFT_CONFIRM_SAMPLES suspicious samples in a
 *  row give ONE event.  The detector then disarms until the car has stood
 *  level and still again for IMU_LIFT_REARM_SAMPLES (put back down), so one
 *  pick-up never reports twice.
 */

#ifndef IMU_LIFT_H
#define IMU_LIFT_H

#include <stdbool.h>
#include <stdint.h>

#include "config/thresholds.h"

/** Suspicious samples in a row before an event (5 x 10 ms = 50 ms). */
#define IMU_LIFT_CONFIRM_SAMPLES    (5U)
/** Level and still samples in a row before re-arming (50 x 10 ms = 0.5 s). */
#define IMU_LIFT_REARM_SAMPLES      (50U)
/** "Level again": within 15 deg of the up vector (cos 15 deg = 0.966). */
#define IMU_LIFT_REARM_COS_X1000    (966)
/** "Still again": acceleration along up within +-0.15 g of rest. */
#define IMU_LIFT_REARM_ACCEL_MG     (150)

typedef enum
{
    IMU_LIFT_NONE = 0,      /**< nothing new */
    IMU_LIFT_TILTED,        /**< tilted past IMU_LIFT_TILT_DEG (lifted or overturned) */
    IMU_LIFT_RAISED         /**< pulled upwards (lifted) */
} imu_lift_event_t;

typedef struct
{
    int32_t  up_x;          /**< gravity at rest, mg */
    int32_t  up_y;
    int32_t  up_z;
    uint32_t up_mag;        /**< |up| in mg; 0 = no reference yet (detector off) */
    bool     armed;
    uint32_t confirm;
    uint32_t settle;
} imu_lift_t;

/** What the last imu_lift_step() measured (for the log). */
typedef struct
{
    int32_t cos_tilt_x1000; /**< cos(angle to up) x 1000: 1000 level, < 0 past 90 deg */
    int32_t up_accel_mg;    /**< acceleration along up minus rest (|up|), mg */
} imu_lift_meas_t;

/** Reset; the detector stays off until imu_lift_set_up(). */
void imu_lift_init(imu_lift_t *p_lift);

/** Set the up vector (car flat and still), mg; arms the detector.
 *  A zero vector leaves it off. */
void imu_lift_set_up(imu_lift_t *p_lift, int32_t x_mg, int32_t y_mg, int32_t z_mg);

/** One low-passed sample, mg.  Returns an event at most once per pick-up.
 *  p_meas may be NULL. */
imu_lift_event_t imu_lift_step(imu_lift_t *p_lift, int32_t x_mg, int32_t y_mg, int32_t z_mg,
                               imu_lift_meas_t *p_meas);

#endif /* IMU_LIFT_H */
