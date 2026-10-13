/**
 * @file thresholds.h
 * @brief Sensor thresholds, speeds, PWM settings and power notes.
 *
 * Source: design.md section 3.2 plus the Phase 1 brief. Pure macros: no SDK or
 * kernel includes. Tunables are tagged TUNE. Unknowns carry
 * TODO(owner-to-confirm, buddy N). Fixed-point: _X1000 = value x 1000.
 */

#ifndef THRESHOLDS_H
#define THRESHOLDS_H

/* ---------------------------------------------------------------------------
 * IMU (Buddy 4)
 * ------------------------------------------------------------------------- */

/** a_move: acceleration that means "the car started moving" (0.05 g). TUNE */
#define IMU_A_MOVE_G_X1000          (50U)
/** degree_hump: pitch that, with a vertical spike, means a hump. TUNE */
#define IMU_HUMP_PITCH_DEG          (5U)
/** g_collision: acceleration spike that latches a collision fault (2.5 g). TUNE */
#define IMU_G_COLLISION_X1000       (2500U)
/** Accelerometer full scale. Must exceed g_collision, so not 2 g. TUNE */
#define IMU_ACCEL_FS_G              (4U)
/** IMU output data rate. TUNE */
#define IMU_ODR_HZ                  (400U)
/** 1 = the IMU has a gyro.
 *  TODO(owner-to-confirm, buddy 4): design.md says "Accel + Gyro"; the 0x19/0x1E
 *  addresses suggest an accel+mag part. */
#define IMU_HAS_GYRO                (0U)
/** 1 = use the magnetometer for heading. Off in Phase 1. */
#define IMU_MAG_HEADING_ENABLE      (0U)
/** Accelerometer I2C address.
 *  TODO(owner-to-confirm, buddy 4): VERIFY by I2C bus scan. */
#define IMU_ACCEL_I2C_ADDR          (0x19U)
/** Magnetometer I2C address.
 *  TODO(owner-to-confirm, buddy 4): VERIFY by I2C bus scan. */
#define IMU_MAG_I2C_ADDR            (0x1EU)

/* ---------------------------------------------------------------------------
 * Obstacle (Buddy 5)
 * ------------------------------------------------------------------------- */

/** D_detect: a coarse reading closer than this triggers the fine scan. TUNE */
#define OBS_D_DETECT_CM             (25U)
/** Consecutive detections needed before the fine scan. TUNE */
#define OBS_DETECT_CONFIRM_COUNT    (2U)
/** Side clearance margin beyond the car width. TUNE */
#define OBS_SIDE_MARGIN_MM          (50U)
/** Car width. ASSUMPTION.
 *  TODO(owner-to-confirm, buddy 5): measure the car. */
#define CAR_WIDTH_MM                (120U)
/** Minimum free gap for a bypass: car width + margin. */
#define OBS_BYPASS_CLEAR_MIN_MM     (CAR_WIDTH_MM + OBS_SIDE_MARGIN_MM)
/** Ultrasonic minimum valid range. TUNE */
#define US_MIN_RANGE_MM             (20U)

/* ---------------------------------------------------------------------------
 * Barcode (Buddy 3)
 * ------------------------------------------------------------------------- */

/** D_cmd: a pending barcode command expires after this distance without a junction. TUNE */
#define BC_D_CMD_CM                 (30U)
/** Ignore a repeat of the same barcode within this distance. Not in design.md. TUNE */
#define BC_REPEAT_IGNORE_CM         (10U)

/* ---------------------------------------------------------------------------
 * Speeds (Buddy 2). Two sets: nominal and bring-up. All TUNE.
 * ------------------------------------------------------------------------- */

/** Nominal cruise speed. TUNE */
#define MOTION_SPEED_CRUISE_NOMINAL_CMPS    (30U)
/** Nominal slow speed (junctions, search). TUNE */
#define MOTION_SPEED_SLOW_NOMINAL_CMPS      (15U)
/** Nominal bypass speed. TUNE */
#define MOTION_SPEED_BYPASS_NOMINAL_CMPS    (20U)
/** Bring-up cruise speed. TUNE */
#define MOTION_SPEED_CRUISE_BRINGUP_CMPS    (20U)
/** Bring-up slow speed. TUNE */
#define MOTION_SPEED_SLOW_BRINGUP_CMPS      (10U)
/** Bring-up bypass speed. TUNE */
#define MOTION_SPEED_BYPASS_BRINGUP_CMPS    (15U)

/** 1 = use the bring-up speed set (default for Phase 1). */
#ifndef MOTION_BRINGUP_SPEEDS
#define MOTION_BRINGUP_SPEEDS               (1U)
#endif

#if MOTION_BRINGUP_SPEEDS
/** Selected cruise speed. */
#define MOTION_SPEED_CRUISE_CMPS    MOTION_SPEED_CRUISE_BRINGUP_CMPS
/** Selected slow speed. */
#define MOTION_SPEED_SLOW_CMPS      MOTION_SPEED_SLOW_BRINGUP_CMPS
/** Selected bypass speed. */
#define MOTION_SPEED_BYPASS_CMPS    MOTION_SPEED_BYPASS_BRINGUP_CMPS
#else
/** Selected cruise speed. */
#define MOTION_SPEED_CRUISE_CMPS    MOTION_SPEED_CRUISE_NOMINAL_CMPS
/** Selected slow speed. */
#define MOTION_SPEED_SLOW_CMPS      MOTION_SPEED_SLOW_NOMINAL_CMPS
/** Selected bypass speed. */
#define MOTION_SPEED_BYPASS_CMPS    MOTION_SPEED_BYPASS_NOMINAL_CMPS
#endif

/** Hard speed cap (NFR2). */
#define MOTION_SPEED_MAX_CMPS       (50U)
/** Minimum bypass speed. */
#define MOTION_SPEED_BYPASS_MIN_CMPS (15U)

/* ---------------------------------------------------------------------------
 * PWM
 * ------------------------------------------------------------------------- */

/** Motor PWM frequency. */
#define MOTOR_PWM_FREQ_HZ           (20000UL)
/** Motor duty cap. TUNE
 *  TODO(owner-to-confirm, buddy 2): provisional duty cap. */
#define MOTOR_DUTY_MAX_PCT          (80U)
/** Servo PWM frequency. */
#define SERVO_PWM_FREQ_HZ           (50U)
/** Servo minimum pulse width. TUNE */
#define SERVO_PULSE_MIN_US          (1000U)
/** Servo maximum pulse width. TUNE */
#define SERVO_PULSE_MAX_US          (2000U)

/* ---------------------------------------------------------------------------
 * Power notes. The power bank supplies the motor driver and the Pico W.
 * ------------------------------------------------------------------------- */

/** Minimum VIN (design.md section 2). */
#define POWER_VIN_MIN_MV            (3600U)
/** Maximum VIN (design.md section 2). */
#define POWER_VIN_MAX_MV            (6000U)
/** 3.3 V budget for Grove devices.
 *  TODO(owner-to-confirm, buddy 2): verify 300 mA against the ROBO PICO datasheet. */
#define POWER_GROVE_3V3_BUDGET_MA   (300U)

/* ---------------------------------------------------------------------------
 * Compile-time checks
 * ------------------------------------------------------------------------- */

#if IMU_ACCEL_FS_G == 2U
#error "thresholds.h: 2 g full scale cannot measure g_collision"
#endif

#if (IMU_ACCEL_FS_G * 1000U) <= IMU_G_COLLISION_X1000
#error "thresholds.h: IMU full scale must exceed g_collision"
#endif

#if (MOTION_SPEED_CRUISE_NOMINAL_CMPS > MOTION_SPEED_MAX_CMPS)  \
    || (MOTION_SPEED_SLOW_NOMINAL_CMPS > MOTION_SPEED_MAX_CMPS)  \
    || (MOTION_SPEED_BYPASS_NOMINAL_CMPS > MOTION_SPEED_MAX_CMPS) \
    || (MOTION_SPEED_CRUISE_BRINGUP_CMPS > MOTION_SPEED_MAX_CMPS) \
    || (MOTION_SPEED_SLOW_BRINGUP_CMPS > MOTION_SPEED_MAX_CMPS)   \
    || (MOTION_SPEED_BYPASS_BRINGUP_CMPS > MOTION_SPEED_MAX_CMPS)
#error "thresholds.h: a speed exceeds MOTION_SPEED_MAX_CMPS (NFR2)"
#endif

#if (MOTION_SPEED_BYPASS_NOMINAL_CMPS < MOTION_SPEED_BYPASS_MIN_CMPS) \
    || (MOTION_SPEED_BYPASS_BRINGUP_CMPS < MOTION_SPEED_BYPASS_MIN_CMPS)
#error "thresholds.h: bypass speed must be >= 15 cm/s"
#endif

#if MOTION_SPEED_BYPASS_MIN_CMPS < 15U
#error "thresholds.h: MOTION_SPEED_BYPASS_MIN_CMPS must be >= 15 cm/s"
#endif

#if (MOTION_SPEED_SLOW_NOMINAL_CMPS > MOTION_SPEED_CRUISE_NOMINAL_CMPS) \
    || (MOTION_SPEED_SLOW_BRINGUP_CMPS > MOTION_SPEED_CRUISE_BRINGUP_CMPS)
#error "thresholds.h: slow speed must be <= cruise speed"
#endif

#if MOTOR_DUTY_MAX_PCT > 100U
#error "thresholds.h: MOTOR_DUTY_MAX_PCT > 100"
#endif

#if !((SERVO_PULSE_MIN_US < SERVO_PULSE_MAX_US) && (SERVO_PULSE_MAX_US < (1000000UL / SERVO_PWM_FREQ_HZ)))
#error "thresholds.h: servo pulse range invalid for SERVO_PWM_FREQ_HZ"
#endif

#endif /* THRESHOLDS_H */
