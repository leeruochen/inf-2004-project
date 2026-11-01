/*
 *  drv_servo.h
 *  Subsystem 5 -- scanning servo driver interface (kernel-free).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  <stdbool.h> and <stdint.h> only, so both drv_servo.c (SDK side) and the
 *  Obstacle Scanning task (kernel side) may include it (rule R1).
 *
 *  Hardware: one hobby servo on SERVO_PIN (GP15) = PWM slice 7 channel B,
 *  driven at SERVO_PWM_FREQ_HZ (50 Hz) with a pulse of SERVO_PULSE_MIN_US..
 *  SERVO_PULSE_MAX_US (config/pins.h, config/thresholds.h).
 *
 *  Bearing convention (matches docs/DATA_REPRESENTATION.md 2.3, where an
 *  obstacle bearing is 30 + 10 x index degrees):
 *      0 deg   = full RIGHT      pulse SERVO_PULSE_MIN_US
 *      90 deg  = straight AHEAD  pulse mid-range
 *      180 deg = full LEFT       pulse SERVO_PULSE_MAX_US
 *  If the horn turns the other way once mounted, define SERVO_DIR_INVERT=1
 *  (a build option; drv_servo.c defaults it to 0) rather than editing the
 *  shared config/pins.h.  TODO(owner-to-confirm, buddy 5): check on the car.
 *
 *  Slice 7 is shared with GP14 (TEAM.md section 2: "50 Hz PWM or plain GPIO
 *  only"), because wrap and clock divider belong to the whole slice.  This
 *  driver sets slice 7 to 50 Hz and holds channel A at level 0 (always LOW),
 *  so GP14 stays safe for its owner.
 *
 *  Ownership: the Obstacle Scanning task is the only caller.  Every call is
 *  non-blocking: moving the horn takes time, so the caller must wait
 *  OBS_SERVO_SETTLE_MS before trusting a ping taken after a move.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdbool.h>
#include <stdint.h>

/** Full-right bearing in degrees. */
#define SERVO_DEG_MIN           (0U)
/** Straight-ahead bearing in degrees. */
#define SERVO_DEG_CENTRE        (90U)
/** Full-left bearing in degrees. */
#define SERVO_DEG_MAX           (180U)

/*
 * Set up PWM slice 7 at SERVO_PWM_FREQ_HZ and park the horn at
 * SERVO_DEG_CENTRE.  Idempotent: a second call does nothing.
 */
void servo_init(void);

/** True once servo_init() has run. */
bool servo_is_ready(void);

/*
 * Command the horn to a bearing, SERVO_DEG_MIN..SERVO_DEG_MAX.  Out-of-range
 * values are clamped.  Ignored before servo_init().  Non-blocking: the horn
 * needs up to OBS_SERVO_SETTLE_MS to get there.
 */
void servo_set_deg(uint32_t deg);

/** Last bearing commanded through servo_set_deg() (centre before any call). */
uint32_t servo_get_deg(void);

/** Shorthand for servo_set_deg(SERVO_DEG_CENTRE). */
void servo_centre(void);

/*
 * Pure: pulse width in microseconds for a bearing, after clamping and the
 * SERVO_DIR_INVERT build option.  Exposed for on-target tests (S5-U01).
 */
uint32_t servo_deg_to_pulse_us(uint32_t deg);

/*
 * Test read-back (S5-U02): PWM frequency of slice 7 in Hz, computed from the
 * slice's TOP and DIV registers and clk_sys.  0 before servo_init().
 */
uint32_t servo_get_pwm_freq_hz(void);

/*
 * Test read-back (S5-U02): pulse width currently programmed on channel B, in
 * microseconds, read back from the PWM compare register.  0 before
 * servo_init().
 */
uint32_t servo_get_pulse_us(void);

/*
 * Test read-back (S5-U03): true while slice 7 channel A (GP14, another
 * owner's spare pin) is held at compare level 0, i.e. always LOW.
 */
bool servo_chan_a_is_low(void);

#endif /* DRV_SERVO_H */
