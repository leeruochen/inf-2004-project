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
 * Line sensor ADC sanity check at boot (Buddy 3), used by
 * hal_adc_sanity_check() on GP26 (IR_R, ADC0). GP26 also has an untaped lead
 * on Grove 6 (config/pins.h). A boot reading below the low rail limit or above
 * the high rail limit, or a peak-to-peak spread above the noise limit, is a
 * sensor fault: logged and counted, never acts on the motors.
 * All values PROVISIONAL. TUNE.
 * TODO(owner-to-confirm, buddy 3): measure on the car.
 * ------------------------------------------------------------------------- */

/** ADC full scale (12-bit RP2040 ADC). */
#define ADC_FULL_SCALE_RAW          (4095U)
/** Samples taken by the boot sanity check. PROVISIONAL. TUNE
 *  TODO(owner-to-confirm, buddy 3). */
#define ADC_SANITY_SAMPLES          (16U)
/** Every sample at or below this = stuck at the low rail. PROVISIONAL. TUNE
 *  TODO(owner-to-confirm, buddy 3). */
#define ADC_STUCK_LOW_RAW           (16U)
/** Every sample at or above this = stuck at the high rail. PROVISIONAL. TUNE
 *  TODO(owner-to-confirm, buddy 3). */
#define ADC_STUCK_HIGH_RAW          (4079U)
/** Peak-to-peak spread above this = abnormally noisy. PROVISIONAL. TUNE
 *  TODO(owner-to-confirm, buddy 3). */
#define ADC_NOISE_MAX_RAW           (400U)

/* ---------------------------------------------------------------------------
 * Encoder geometry (Buddy 2). PIO0 quadrature decoding, X4 (delta D7).
 * 0 = not yet measured. Never invent values: Batch B code that uses them must
 * #error while either is 0.
 *
 * Overflow bound of the free-running int32_t count (documentation only):
 *   counts per second at top speed
 *     r = ENC_QUAD_MULT x ENC_CPR x (MOTION_SPEED_MAX_CMPS x 10) / WHEEL_CIRC_MM
 *   counts = ENC_QUAD_MULT x ENC_CPR x distance_mm / WHEEL_CIRC_MM, so the
 *   count wraps after (2^31 - 1) x WHEEL_CIRC_MM / (ENC_QUAD_MULT x ENC_CPR) mm
 *   of one-way driving, i.e. after (2^31 - 1) / r seconds at 50 cm/s.
 *   CPR-independent hardware bound: the PIO program counts at most
 *   sysclk / 10 = 12.5 Msteps/s, so a wrap takes at least
 *   2,147,483,647 / 12,500,000 = 171.8 s even at the decoder's limit.
 *   Motion uses wrap-safe two's-complement deltas per 20 ms period, exact while
 *   one period's delta stays below 2^31.
 * ------------------------------------------------------------------------- */

/** Quadrature decoding multiplier (X4: every edge of A and B). */
#define ENC_QUAD_MULT               (4U)
/** Encoder cycles per wheel revolution, per channel, gearbox included.
 *  TODO(owner-to-confirm, buddy 2): from the encoder spec or a measured turn. */
#define ENC_CPR                     (0U)
/** Wheel circumference.
 *  TODO(owner-to-confirm, buddy 2): measure the wheel. */
#define WHEEL_CIRC_MM               (0U)

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

#if ADC_STUCK_LOW_RAW >= ADC_STUCK_HIGH_RAW
#error "thresholds.h: ADC_STUCK_LOW_RAW must be below ADC_STUCK_HIGH_RAW"
#endif

#if ADC_STUCK_HIGH_RAW > ADC_FULL_SCALE_RAW
#error "thresholds.h: ADC_STUCK_HIGH_RAW exceeds the ADC full scale"
#endif

#if ADC_SANITY_SAMPLES == 0U
#error "thresholds.h: ADC_SANITY_SAMPLES must be > 0"
#endif

#if ADC_NOISE_MAX_RAW >= ADC_FULL_SCALE_RAW
#error "thresholds.h: ADC_NOISE_MAX_RAW must be below the ADC full scale"
#endif

/* Once ENC_CPR and WHEEL_CIRC_MM are measured: one second of counts at top
 * speed (and so any 20 ms delta) must fit an int32_t. */
#if (ENC_CPR != 0U) && (WHEEL_CIRC_MM != 0U)
#if ((ENC_QUAD_MULT * ENC_CPR * MOTION_SPEED_MAX_CMPS * 10U) / WHEEL_CIRC_MM) > 2147483647UL
#error "thresholds.h: encoder counts per second at top speed overflow int32_t"
#endif
#endif

#endif /* THRESHOLDS_H */
