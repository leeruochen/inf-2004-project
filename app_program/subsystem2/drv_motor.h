/*
 *  drv_motor.h
 *  Subsystem 2 -- drive motor driver interface (kernel-free).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  <stdbool.h> and <stdint.h> only, so both drv_motor.c (SDK side) and the
 *  Motion Control task (kernel side) may include it.
 *
 *  Hardware: onboard ROBO PICO motor driver, PWM slices 4 (left, GP8/GP9) and
 *  5 (right, GP10/GP11) at MOTOR_PWM_FREQ_HZ (config/pins.h, thresholds.h).
 *  Truth table per motor (config/pins.h):
 *    forward  PWM on the forward input, reverse input LOW
 *    reverse  PWM on the reverse input, forward input LOW
 *    brake    both inputs LOW
 *    coast    both inputs HIGH
 *  The forward/reverse drive pattern is to be confirmed on the board
 *  (roadmap Q4, test S2-U03).  MOTOR_x_DIR_INVERT swaps the inputs.
 *
 *  Ownership: the Motion Control task is the only caller, except
 *  motor_brake_all(), which the emergency brake (IF-11) may call from any
 *  context.  Duty is in permille (per thousand) of the PWM period.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/** Motor index: left motor (GP8/GP9, PWM slice 4). */
#define MOTOR_LEFT              (0U)
/** Motor index: right motor (GP10/GP11, PWM slice 5). */
#define MOTOR_RIGHT             (1U)
/** Number of drive motors. */
#define MOTOR_COUNT             (2U)

/** Full-scale duty (100 %) in permille. */
#define MOTOR_DUTY_FULL_PM      (1000)

/*
 * Set up PWM slices 4 and 5 at MOTOR_PWM_FREQ_HZ with both motors braked
 * (all four inputs LOW).  Idempotent: a second call does nothing.
 */
void motor_init(void);

/** True once motor_init() has run. */
bool motor_is_ready(void);

/*
 * Drive one motor.
 *   motor    MOTOR_LEFT or MOTOR_RIGHT (anything else is ignored)
 *   duty_pm  -1000..+1000; > 0 forward, < 0 reverse, 0 = brake.  Clamped to
 *            +-MOTOR_DUTY_MAX_PCT (motor_duty_clamp_pm()).
 * Ignored before motor_init().  Non-blocking.
 */
void motor_set_duty(uint32_t motor, int32_t duty_pm);

/** Brake one motor: both inputs LOW. */
void motor_brake(uint32_t motor);

/** Coast one motor: both inputs HIGH. */
void motor_coast(uint32_t motor);

/*
 * Brake both motors at once (all four inputs LOW) inside a critical
 * section.  Safe from any task and before motor_init(): if PWM is not set up
 * yet, the four pins are driven LOW as plain GPIO outputs.  With PWM running
 * the new compare value takes effect at the end of the current PWM period
 * (at most 1 / MOTOR_PWM_FREQ_HZ = 50 us).  Used by motion_emergency_brake().
 */
void motor_brake_all(void);

/*
 * Pure: clamp a duty request to +-(MOTOR_DUTY_MAX_PCT x 10) permille.
 * Used by motor_set_duty(); exposed for host and on-target tests (S2-U02).
 */
int32_t motor_duty_clamp_pm(int32_t duty_pm);

/*
 * Test read-back (S2-U01): PWM frequency of one motor's slice in Hz,
 * computed from the slice's TOP and DIV registers and clk_sys.
 * Returns 0 for a bad index or before motor_init().
 */
uint32_t motor_get_pwm_freq_hz(uint32_t motor);

/*
 * Test read-back: current compare levels of one motor's forward and reverse
 * inputs, in PWM counts (0 = always LOW, wrap + 1 = always HIGH), and the
 * period in counts (wrap + 1).  Any pointer may be NULL.
 */
void motor_get_levels(uint32_t motor, uint32_t *p_fwd, uint32_t *p_rev, uint32_t *p_period);

/*
 * Test read-back (S2-U06): true if all four motor channels are currently
 * driven LOW (either PWM compare levels 0 on both slices after motor_init(),
 * or SIO outputs LOW on GP8-GP11 before motor_init()).
 */
bool motor_are_all_braked(void);

#endif /* DRV_MOTOR_H */

