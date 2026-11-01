/*
 *  drv_ultrasonic.h
 *  Subsystem 5 -- HC-SR04 ultrasonic range finder interface (kernel-free).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  <stdbool.h> and <stdint.h> only, so both drv_ultrasonic.c (SDK side) and
 *  the Obstacle Scanning task (kernel side) may include it (rule R1).
 *
 *  Hardware: US_TRIG_PIN (GP4) output, US_ECHO_PIN (GP5) input on Grove 3.
 *  The echo line is 5 V on a bare HC-SR04 and MUST be brought down to 3.3 V
 *  before it reaches GP5 (TEAM.md section 2, ARCHITECTURE.md A8); a divider
 *  or a 3.3 V-native module is required or the pin will be damaged.
 *
 *  How a measurement works (IF-05):
 *    1. us_trigger() arms the capture and emits a US_TRIG_PULSE_US pulse.
 *    2. The module drives echo HIGH, then LOW after a time proportional to
 *       the round-trip distance.  Both edges raise IO_IRQ_BANK0; the handler
 *       registered by us_init() timestamps the rising edge, computes the
 *       width on the falling edge and signals the semaphore passed to
 *       us_set_echo_sem() through hal_isr_sig() (never a kernel call from
 *       SDK code directly).
 *    3. The task waits on that semaphore for OBS_ECHO_TIMEOUT_MS.  A timeout
 *       means "nothing in range" (design.md 3.3), not a fault.
 *    4. us_take_echo_us() hands over the captured width, once.
 *
 *  Ownership: the Obstacle Scanning task is the only caller.  Every call is
 *  non-blocking apart from the US_TRIG_PULSE_US (10 us) pulse itself, which
 *  is far below one kernel tick.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

/** Trigger pulse width required by the HC-SR04 (datasheet: at least 10 us). */
#define US_TRIG_PULSE_US        (10U)

/** Longest echo accepted: about 4 m round trip.  Anything longer is noise. */
#define US_ECHO_MAX_US          (25000U)

/*
 * Minimum gap between two triggers, so the previous burst has died away
 * before the next one is sent (HC-SR04 datasheet recommends a 60 ms
 * measurement cycle at full range; 10 ms is ample for the sub-metre ranges
 * this subsystem cares about).  TUNE.
 */
#define US_PING_GAP_MS          (10U)

/*
 * Set up the trigger and echo pins and register the echo edge handler on
 * IO_IRQ_BANK0 through hal_gpio_irq_register().  Idempotent.
 * Returns HAL_OK, or a negative HAL_E_* code if the handler could not be
 * registered (then no measurement will ever complete).
 */
int32_t us_init(void);

/** True once us_init() has registered the echo handler. */
bool us_is_ready(void);

/*
 * Tell the driver which semaphore the echo handler should signal (IF-05
 * SEM_ECHO).  Called once from kernel side, after ipc_init(), because SDK
 * code may not name kernel objects.  Pass 0 to disable signalling and poll
 * us_echo_ready() instead.
 */
void us_set_echo_sem(int32_t semid);

/*
 * Arm the capture and emit the trigger pulse.  Discards any previous
 * un-taken result, so the caller always waits for a fresh echo.
 * Returns false (and does nothing) before us_init().
 */
bool us_trigger(void);

/** True if a complete echo has been captured and not yet taken. */
bool us_echo_ready(void);

/*
 * Hand over the captured echo width in microseconds and clear the ready
 * flag.  Returns false if no complete echo is waiting (p_echo_us untouched).
 */
bool us_take_echo_us(uint32_t *p_echo_us);

/*
 * Pure: round-trip echo width in microseconds to a one-way distance in
 * millimetres, using 343 m/s (20 C).  Returns 0 for 0 or for a width above
 * US_ECHO_MAX_US.  Exposed for on-target tests (S5-U04).
 */
uint32_t us_echo_us_to_mm(uint32_t echo_us);

/** Echo widths captured since us_init(), for health checks and tests. */
uint32_t us_echo_count(void);

/*
 * Triggers that were still armed when the next one was issued, i.e. no
 * falling edge ever arrived.  Counts "nothing in range" as well as a broken
 * or unwired sensor, so compare it against us_echo_count().
 */
uint32_t us_miss_count(void);

/*
 * True if the echo line is HIGH right now.  A line stuck HIGH means the
 * sensor is unpowered, miswired, or the divider is missing; checked once at
 * start-up (S5-U05).
 */
bool us_echo_pin_is_high(void);

#endif /* DRV_ULTRASONIC_H */
