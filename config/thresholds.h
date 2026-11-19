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
/** degree_hump: pitch that means a hump.  Was 5 deg (design.md); the 10 mm
 *  track hump tilts the car only ~4-4.5 deg (S4-U26 trace, 19 Nov 2026), and
 *  the car's cruise noise is within ~+-2 deg.  TUNE */
#define IMU_HUMP_PITCH_DEG          (3U)
/** Distance between the car's two ground contacts along its length (driven
 *  wheel axle to the front contact), for the hump height (imu_hump.h):
 *  height = wheelbase x sin(pitch) while one end is on the hump.
 *  PROVISIONAL: 120 mm is the line sensors' distance in front of the axle
 *  (Buddy 3, ruler, 4 Nov 2026, lf_core.h LF_D_PIVOT_MM), the closest value
 *  on record.  TODO(owner-to-confirm, buddy 4): measure axle centre to the
 *  front contact with a ruler, or calibrate with S4-U25 (wheelbase =
 *  object height / sin(pitch)). */
#define IMU_HUMP_WHEELBASE_MM       (120U)
/* Lift / overturn detection (replaces g_collision; requirement change of
 * 11 Nov 2026, app_program/subsystem4/imu_lift.h).  Either one, held for
 * 50 ms, brakes the motors and latches the fault. */
/** theta_lift: tilt from level that means lifted or overturned (30 deg).
 *  Must stay well above hump pitch (~10-15 deg measured). TUNE */
#define IMU_LIFT_TILT_DEG           (30U)
/** cos(IMU_LIFT_TILT_DEG) x 1000, used by the integer code. Update together. */
#define IMU_LIFT_COS_TILT_X1000     (866U)
/** v_lift: upward speed that means lifted (0.10 m/s), estimated by integrating
 *  the acceleration along "up". A smooth hand lift of 10 cm reaches ~0.15-0.4
 *  m/s; climbing a 2 cm hump at 25 cm/s ~0.05 m/s. Replaced a 0.3 g / 50 ms
 *  acceleration rule that missed a gentle level lift (S4-U18, 13 Nov 2026). TUNE */
#define IMU_LIFT_V_MMPS             (100U)
/** Accelerometer full scale (headroom for bumps and handling). TUNE */
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
/** Car width at its widest point, wheels included.
 *  MEASURED 2026-11-19 (Buddy 5): 130 mm, was an assumed 120 mm.
 *  Feeds obs_leg_out_mm() (how far the bypass swings sideways) and
 *  OBS_BYPASS_CLEAR_MIN_MM (the gap a bypass needs). */
#define CAR_WIDTH_MM                (130U)
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
 *  Measured on car (Buddy 2, S2-U10/S2-U18, floor_run3.txt: 2464 quad counts/rev). */
#define ENC_CPR                     (616U)
/** Wheel circumference in mm.
 *  Measured on car (Buddy 2, S2-U11: 66 mm wheel diameter -> pi * 66 = 207.3 mm). */
#define WHEEL_CIRC_MM               (207U)

/* ---------------------------------------------------------------------------
 * Barcode (Buddy 3)
 * ------------------------------------------------------------------------- */

/** D_cmd: a pending barcode command expires after this distance without a junction. TUNE */
#define BC_D_CMD_CM                 (30U)
/** Ignore a repeat of the same barcode within this distance. Not in design.md. TUNE */
#define BC_REPEAT_IGNORE_CM         (10U)
/** Wide/narrow threshold (NFR9): an element is wide when its width is more
 *  than this x the median element width / 100.  Code 39 wide:narrow is 2:1 to
 *  3:1 and most elements are narrow, so the median is a narrow width and the
 *  value must lie between 100 and 200 (checked below); 150 is the midpoint at
 *  the worst-case 2:1.  Set from measured hardware ratios. Not in design.md. TUNE */
#define BC_WIDE_RATIO_X100          (150U)
/** Slowest speed at which a read is trusted; any element below it makes the
 *  read a stall (design.md 6.2).  Half the slowest drive speed
 *  (MOTION_SPEED_SLOW_BRINGUP_CMPS, 10 cm/s): below it one encoder count is a
 *  large part of Motion's 20 ms speed estimate. Not in design.md. TUNE */
#define BC_MIN_SPEED_MMPS           (50U)

/* ---------------------------------------------------------------------------
 * Speeds (Buddy 2). Two sets: nominal and bring-up. All TUNE.
 * ------------------------------------------------------------------------- */

/** 1 = the slow bring-up bypass (5 cm/s) is in use; NFR14 cannot pass
 *  while it is set.  Defined in both thresholds.h and timing.h with the
 *  same default so either header can be included alone; timing.h carries
 *  the full explanation and the list of what to put back. */
#ifndef OBS_BYPASS_SLOW_BRINGUP
#define OBS_BYPASS_SLOW_BRINGUP             (1U)
#endif

/** Nominal cruise speed. TUNE */
#define MOTION_SPEED_CRUISE_NOMINAL_CMPS    (30U)
/** Nominal slow speed (junctions, search). TUNE */
#define MOTION_SPEED_SLOW_NOMINAL_CMPS      (15U)
/** Nominal bypass speed. Closed to 25 cm/s on 2026-11-04 for NFR14. */
#define MOTION_SPEED_BYPASS_NOMINAL_CMPS    (25U)
/** Bring-up cruise speed. TUNE
 *  2026-11-19: 15 -> 5 cm/s so the obstacle run can be watched and the car
 *  caught.  Needs LF_T_JCT_MAX_MS raised to match (lf_core.h): the
 *  assertion there ties the junction time cap to this speed.  Buddy 3
 *  agreed to the change. */
#define MOTION_SPEED_CRUISE_BRINGUP_CMPS    (5U)
/** Bring-up slow speed. TUNE */
#define MOTION_SPEED_SLOW_BRINGUP_CMPS      (5U)
/** Bring-up bypass speed.
 *  2026-11-19: 25 -> 5 cm/s, matching the bring-up cruise speed.
 *  REASON: at 25 cm/s the car crossed the line and drove past it between
 *  EVF_NAV_LINE_FOUND being set and Line Following resuming -- measured
 *  on the car, reacquired=1 followed by REACQUIRING -> IDLE with CENTRE
 *  back on white (150 raw).  At 5 cm/s there is five times as long for
 *  the handover to land while the sensor is still over the line.
 *  BREAKS NFR14: see OBS_BYPASS_SLOW_BRINGUP in timing.h.  Set this back
 *  to 25 before the NFR14 timing run. */
#define MOTION_SPEED_BYPASS_BRINGUP_CMPS    (5U)

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
/** Minimum bypass speed.
 *  2026-11-19: 15 -> 5 cm/s for the slow bring-up bypass (see
 *  MOTION_SPEED_BYPASS_BRINGUP_CMPS).  15 cm/s is the floor NFR14 needs;
 *  restore it with the bypass speed. */
#define MOTION_SPEED_BYPASS_MIN_CMPS (5U)

/* ---------------------------------------------------------------------------
 * PWM
 * ------------------------------------------------------------------------- */

/** Motor PWM frequency. */
#define MOTOR_PWM_FREQ_HZ           (20000UL)
/** Motor duty cap. TUNE
 *  Confirmed on car (Buddy 2, S2-U02 / S2-U21). */
#define MOTOR_DUTY_MAX_PCT          (95U)
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
 *  Verified on car (Buddy 2, MOTION_ACCURACY_EVALUATION.md section 1.2:
 *  72 mA typical / 110 mA peak across all 7 Grove devices < 300 mA LDO budget). */
#define POWER_GROVE_3V3_BUDGET_MA   (300U)

/* ---------------------------------------------------------------------------
 * Compile-time checks
 * ------------------------------------------------------------------------- */

#if !((IMU_LIFT_TILT_DEG == 30U) && (IMU_LIFT_COS_TILT_X1000 == 866U))     \
    && !((IMU_LIFT_TILT_DEG == 45U) && (IMU_LIFT_COS_TILT_X1000 == 707U))  \
    && !((IMU_LIFT_TILT_DEG == 60U) && (IMU_LIFT_COS_TILT_X1000 == 500U))
#error "thresholds.h: IMU_LIFT_COS_TILT_X1000 must be cos(IMU_LIFT_TILT_DEG) x 1000 (30/866, 45/707, 60/500)"
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
#error "thresholds.h: bypass speed is below MOTION_SPEED_BYPASS_MIN_CMPS"
#endif

/* The 15 cm/s floor is what NFR14's 5.0 s needs.  It is relaxed only for
 * the slow bring-up bypass; OBS_BYPASS_SLOW_BRINGUP in timing.h is the one
 * switch that has to be cleared to put both back. */
#if !OBS_BYPASS_SLOW_BRINGUP && (MOTION_SPEED_BYPASS_MIN_CMPS < 15U)
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

#if (BC_WIDE_RATIO_X100 <= 100U) || (BC_WIDE_RATIO_X100 >= 200U)
#error "thresholds.h: BC_WIDE_RATIO_X100 must lie between 100 and 200 (2:1 bars)"
#endif

#if (BC_MIN_SPEED_MMPS == 0U) || (BC_MIN_SPEED_MMPS > (MOTION_SPEED_SLOW_BRINGUP_CMPS * 10U))
#error "thresholds.h: BC_MIN_SPEED_MMPS must be > 0 and not above the slowest drive speed"
#endif

/* Once ENC_CPR and WHEEL_CIRC_MM are measured: one second of counts at top
 * speed (and so any 20 ms delta) must fit an int32_t. */
#if (ENC_CPR != 0U) && (WHEEL_CIRC_MM != 0U)
#if ((ENC_QUAD_MULT * ENC_CPR * MOTION_SPEED_MAX_CMPS * 10U) / WHEEL_CIRC_MM) > 2147483647UL
#error "thresholds.h: encoder counts per second at top speed overflow int32_t"
#endif
#endif

#endif /* THRESHOLDS_H */
