/*
 *  drv_servo.c
 *  Subsystem 5 -- scanning servo driver.  SDK side: Pico SDK headers only,
 *  never kernel headers or subsystem5.h (rule R1).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  See drv_servo.h for the interface and the bearing convention.
 *
 *  PWM set-up: the clock divider is chosen so that one PWM count is one
 *  microsecond (SERVO_TICK_HZ), which makes the compare level equal to the
 *  pulse width in microseconds and keeps the arithmetic exact.  At the SDK
 *  default clk_sys of 125 MHz this is div 125, wrap 19999 (20 000 us = 50 Hz).
 *  If clk_sys is not a whole number of MHz the divider still gives the
 *  closest tick rate and servo_tick_hz_ scales the conversion.
 *
 *  Slice 7 channel A (GP14) is held at compare level 0 so that another
 *  owner's spare pin is never driven HIGH by this driver.
 */

#include "drv_servo.h"

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* ---------------------------------------------------------------------------
 * Compile-time checks: the pin really is slice 7 channel B, the pulse range
 * fits inside one 50 Hz period, and the servo slice is not a motor slice.
 * ------------------------------------------------------------------------- */

#if PIN_PWM_SLICE(SERVO_PIN) != SERVO_PWM_SLICE
#error "drv_servo.c: SERVO_PIN is not on SERVO_PWM_SLICE"
#endif
#if (SERVO_PIN & 1U) != SERVO_PWM_CHAN
#error "drv_servo.c: SERVO_PIN is not on SERVO_PWM_CHAN"
#endif
#if SERVO_PWM_FREQ_HZ == 0U
#error "drv_servo.c: SERVO_PWM_FREQ_HZ must not be 0"
#endif
#if SERVO_PULSE_MIN_US >= SERVO_PULSE_MAX_US
#error "drv_servo.c: SERVO_PULSE_MIN_US must be below SERVO_PULSE_MAX_US"
#endif
#if SERVO_PULSE_MAX_US >= (1000000U / SERVO_PWM_FREQ_HZ)
#error "drv_servo.c: SERVO_PULSE_MAX_US does not fit in one PWM period"
#endif

/* Horn direction.  Build option, defaulted here so that the shared
 * config/pins.h stays untouched (drv_servo.h). */
#ifndef SERVO_DIR_INVERT
#define SERVO_DIR_INVERT        (0U)
#endif

/** Target PWM tick rate: one count per microsecond. */
#define SERVO_TICK_HZ           (1000000UL)

/** Largest integer clock divider of a PWM slice. */
#define SERVO_DIV_INT_MAX       (255UL)

/** Largest PWM period in counts (16-bit counter: wrap 0xFFFF). */
#define SERVO_PERIOD_MAX        (65536UL)

/** Nominal PWM period in microseconds (20 000 us at 50 Hz). */
#define SERVO_PERIOD_US         (1000000UL / SERVO_PWM_FREQ_HZ)

/** Pulse span in microseconds between full right and full left. */
#define SERVO_PULSE_SPAN_US     ((uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US)

static bool     s_ready = false;
static uint32_t s_deg = SERVO_DEG_CENTRE;
static uint32_t s_tick_hz = SERVO_TICK_HZ;  /* actual counts per second */
static uint32_t s_wrap = 0U;

/** Compare counts for a pulse width, with the actual tick rate. */
static uint32_t pulse_us_to_counts(uint32_t pulse_us)
{
    uint64_t const counts = ((uint64_t)pulse_us * (uint64_t)s_tick_hz) / 1000000ULL;

    return (counts > (uint64_t)s_wrap) ? s_wrap : (uint32_t)counts;
}

uint32_t servo_deg_to_pulse_us(uint32_t deg)
{
    uint32_t eff = (deg > SERVO_DEG_MAX) ? SERVO_DEG_MAX : deg;

    if (SERVO_DIR_INVERT != 0U)
    {
        eff = SERVO_DEG_MAX - eff;
    }

    /* Round to nearest microsecond: + half a step before the divide. */
    return (uint32_t)SERVO_PULSE_MIN_US
           + (((eff * SERVO_PULSE_SPAN_US) + (SERVO_DEG_MAX / 2U)) / SERVO_DEG_MAX);
}

void servo_init(void)
{
    if (s_ready)
    {
        return;
    }

    uint32_t const clk_hz = clock_get_hz(clk_sys);
    uint32_t       div = (clk_hz + (SERVO_TICK_HZ / 2UL)) / SERVO_TICK_HZ;

    if (div == 0UL)
    {
        div = 1UL;
    }
    if (div > SERVO_DIV_INT_MAX)
    {
        div = SERVO_DIV_INT_MAX;
    }

    uint32_t tick_hz = clk_hz / div;
    uint32_t period = tick_hz / (uint32_t)SERVO_PWM_FREQ_HZ;

    /* A 16-bit counter cannot hold the period: slow the clock down until it
     * can.  Cannot happen at 125 MHz / 50 Hz, but keep the driver honest if
     * clk_sys is ever raised. */
    while ((period > SERVO_PERIOD_MAX) && (div < SERVO_DIV_INT_MAX))
    {
        div++;
        tick_hz = clk_hz / div;
        period = tick_hz / (uint32_t)SERVO_PWM_FREQ_HZ;
    }
    if (period == 0UL)
    {
        period = 1UL;
    }
    if (period > SERVO_PERIOD_MAX)
    {
        period = SERVO_PERIOD_MAX;
    }

    s_tick_hz = tick_hz;
    s_wrap = period - 1UL;

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(SERVO_PWM_SLICE, (uint8_t)div, 0U);
    pwm_set_wrap(SERVO_PWM_SLICE, (uint16_t)s_wrap);

    /* GP14 shares this slice: hold channel A permanently LOW (drv_servo.h). */
    pwm_set_chan_level(SERVO_PWM_SLICE, PWM_CHAN_A, 0U);

    s_deg = SERVO_DEG_CENTRE;
    pwm_set_chan_level(SERVO_PWM_SLICE, PWM_CHAN_B,
                       (uint16_t)pulse_us_to_counts(servo_deg_to_pulse_us(s_deg)));
    pwm_set_enabled(SERVO_PWM_SLICE, true);

    s_ready = true;
}

bool servo_is_ready(void)
{
    return s_ready;
}

void servo_set_deg(uint32_t deg)
{
    if (!s_ready)
    {
        return;
    }

    s_deg = (deg > SERVO_DEG_MAX) ? SERVO_DEG_MAX : deg;
    pwm_set_chan_level(SERVO_PWM_SLICE, PWM_CHAN_B,
                       (uint16_t)pulse_us_to_counts(servo_deg_to_pulse_us(s_deg)));
}

uint32_t servo_get_deg(void)
{
    return s_deg;
}

void servo_centre(void)
{
    servo_set_deg(SERVO_DEG_CENTRE);
}

uint32_t servo_get_pwm_freq_hz(void)
{
    if (!s_ready)
    {
        return 0U;
    }

    uint32_t const top = (uint32_t)pwm_hw->slice[SERVO_PWM_SLICE].top + 1UL;
    uint32_t const div = pwm_hw->slice[SERVO_PWM_SLICE].div;   /* 8.4 fixed point */
    uint32_t const clk_hz = clock_get_hz(clk_sys);

    if ((top == 0UL) || (div == 0UL))
    {
        return 0U;
    }

    /* clk_sys / ((div / 16) x top), rounded to the nearest hertz. */
    uint64_t const denom = (uint64_t)top * (uint64_t)div;

    return (uint32_t)(((uint64_t)clk_hz * 16ULL + (denom / 2ULL)) / denom);
}

uint32_t servo_get_pulse_us(void)
{
    if (!s_ready)
    {
        return 0U;
    }

    uint32_t const cc = pwm_hw->slice[SERVO_PWM_SLICE].cc;
    uint32_t const level = (cc >> PWM_CH0_CC_B_LSB) & 0xFFFFU;

    if (s_tick_hz == 0UL)
    {
        return 0U;
    }

    return (uint32_t)((((uint64_t)level * 1000000ULL) + ((uint64_t)s_tick_hz / 2ULL))
                      / (uint64_t)s_tick_hz);
}

bool servo_chan_a_is_low(void)
{
    if (!s_ready)
    {
        return false;
    }

    return (((pwm_hw->slice[SERVO_PWM_SLICE].cc >> PWM_CH0_CC_A_LSB) & 0xFFFFU) == 0U);
}
