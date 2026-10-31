/*
 *  drv_motor.c
 *  Subsystem 2 -- drive motor driver.  SDK side: Pico SDK headers only,
 *  never kernel headers or subsystem2.h.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  See drv_motor.h for the interface, truth table and ownership.
 *
 *  PWM set-up: one integer clock divider and one wrap for both slices, so
 *  that clk_sys / (div x (wrap + 1)) = MOTOR_PWM_FREQ_HZ.  At the SDK default
 *  clk_sys of 125 MHz this is div 1, wrap 6249 (6250 counts per period).
 *  A compare level of 0 holds the output LOW; wrap + 1 holds it HIGH.
 */

#include "drv_motor.h"

#include <stddef.h>

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

/* ---------------------------------------------------------------------------
 * Compile-time checks: each motor's two inputs are the A and B channels of
 * one PWM slice, and the duty cap is a percentage.
 * ------------------------------------------------------------------------- */

#if PIN_PWM_SLICE(MOTOR_L_A_PIN) != PIN_PWM_SLICE(MOTOR_L_B_PIN)
#error "drv_motor.c: left motor inputs must share one PWM slice"
#endif
#if PIN_PWM_SLICE(MOTOR_R_A_PIN) != PIN_PWM_SLICE(MOTOR_R_B_PIN)
#error "drv_motor.c: right motor inputs must share one PWM slice"
#endif
#if PIN_PWM_SLICE(MOTOR_L_A_PIN) == PIN_PWM_SLICE(MOTOR_R_A_PIN)
#error "drv_motor.c: left and right motors must use different PWM slices"
#endif
#if (MOTOR_DUTY_MAX_PCT == 0U) || (MOTOR_DUTY_MAX_PCT > 100U)
#error "drv_motor.c: MOTOR_DUTY_MAX_PCT must be 1..100"
#endif
#if MOTOR_PWM_FREQ_HZ == 0UL
#error "drv_motor.c: MOTOR_PWM_FREQ_HZ must not be 0"
#endif

/** Duty cap in permille. */
#define MOTOR_DUTY_MAX_PM       ((int32_t)MOTOR_DUTY_MAX_PCT * 10)

/** Largest PWM period in counts (16-bit counter: wrap 0xFFFF). */
#define MOTOR_PERIOD_MAX        (65536UL)

/** Largest integer clock divider of a PWM slice. */
#define MOTOR_DIV_INT_MAX       (255UL)

/* Forward and reverse input pin of each motor, after MOTOR_x_DIR_INVERT. */
typedef struct
{
    uint32_t pin_fwd;
    uint32_t pin_rev;
} motor_pins_t;

static motor_pins_t const k_motor_pins[MOTOR_COUNT] =
{
    [MOTOR_LEFT] =
    {
        (MOTOR_L_DIR_INVERT != 0U) ? MOTOR_L_B_PIN : MOTOR_L_A_PIN,
        (MOTOR_L_DIR_INVERT != 0U) ? MOTOR_L_A_PIN : MOTOR_L_B_PIN
    },
    [MOTOR_RIGHT] =
    {
        (MOTOR_R_DIR_INVERT != 0U) ? MOTOR_R_B_PIN : MOTOR_R_A_PIN,
        (MOTOR_R_DIR_INVERT != 0U) ? MOTOR_R_A_PIN : MOTOR_R_B_PIN
    }
};

/* All four motor pins, for the emergency brake before PWM is set up. */
static uint32_t const k_all_pins[4] =
{
    MOTOR_L_A_PIN, MOTOR_L_B_PIN, MOTOR_R_A_PIN, MOTOR_R_B_PIN
};

static volatile bool s_ready = false;
static uint32_t      s_period = 0U;     /* wrap + 1, counts */

/* Write both compare levels of one motor in a single CC register write, so
 * the two inputs never disagree for a PWM period. */
static void motor_write_levels(uint32_t motor, uint32_t level_fwd, uint32_t level_rev)
{
    motor_pins_t const *p_pins = &k_motor_pins[motor];
    uint32_t const      slice = pwm_gpio_to_slice_num(p_pins->pin_fwd);
    uint32_t            level_a;
    uint32_t            level_b;

    if (pwm_gpio_to_channel(p_pins->pin_fwd) == PWM_CHAN_A)
    {
        level_a = level_fwd;
        level_b = level_rev;
    }
    else
    {
        level_a = level_rev;
        level_b = level_fwd;
    }
    pwm_set_both_levels(slice, (uint16_t)level_a, (uint16_t)level_b);
}

void motor_init(void)
{
    uint32_t   sys_hz;
    uint32_t   counts;
    uint32_t   div = 1U;
    pwm_config cfg;

    if (s_ready)
    {
        return;
    }

    /* Smallest integer divider that fits the period into 16 bits. */
    sys_hz = clock_get_hz(clk_sys);
    counts = sys_hz / (uint32_t)MOTOR_PWM_FREQ_HZ;
    while (((counts / div) > MOTOR_PERIOD_MAX) && (div < MOTOR_DIV_INT_MAX))
    {
        div++;
    }
    s_period = counts / div;
    if (s_period > MOTOR_PERIOD_MAX)
    {
        s_period = MOTOR_PERIOD_MAX;
    }

    cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, div);
    pwm_config_set_wrap(&cfg, (uint16_t)(s_period - 1U));

    for (uint32_t m = 0U; m < MOTOR_COUNT; m++)
    {
        uint32_t const slice = pwm_gpio_to_slice_num(k_motor_pins[m].pin_fwd);

        /* Levels 0 before the pins switch to PWM: the motor stays braked. */
        pwm_init(slice, &cfg, false);
        pwm_set_both_levels(slice, 0U, 0U);
    }
    for (uint32_t i = 0U; i < 4U; i++)
    {
        gpio_set_function(k_all_pins[i], GPIO_FUNC_PWM);
    }
    for (uint32_t m = 0U; m < MOTOR_COUNT; m++)
    {
        pwm_set_enabled(pwm_gpio_to_slice_num(k_motor_pins[m].pin_fwd), true);
    }
    s_ready = true;
}

bool motor_is_ready(void)
{
    return s_ready;
}

int32_t motor_duty_clamp_pm(int32_t duty_pm)
{
    if (duty_pm > MOTOR_DUTY_MAX_PM)
    {
        return MOTOR_DUTY_MAX_PM;
    }
    if (duty_pm < -MOTOR_DUTY_MAX_PM)
    {
        return -MOTOR_DUTY_MAX_PM;
    }
    return duty_pm;
}

void motor_set_duty(uint32_t motor, int32_t duty_pm)
{
    int32_t  duty;
    uint32_t mag;
    uint32_t level;

    if ((!s_ready) || (motor >= MOTOR_COUNT))
    {
        return;
    }
    duty = motor_duty_clamp_pm(duty_pm);
    mag = (duty < 0) ? (uint32_t)(-duty) : (uint32_t)duty;
    level = (mag * s_period) / (uint32_t)MOTOR_DUTY_FULL_PM;

    if (duty > 0)
    {
        motor_write_levels(motor, level, 0U);
    }
    else if (duty < 0)
    {
        motor_write_levels(motor, 0U, level);
    }
    else
    {
        motor_write_levels(motor, 0U, 0U);     /* 0 = brake */
    }
}

void motor_brake(uint32_t motor)
{
    if ((!s_ready) || (motor >= MOTOR_COUNT))
    {
        return;
    }
    motor_write_levels(motor, 0U, 0U);
}

void motor_coast(uint32_t motor)
{
    if ((!s_ready) || (motor >= MOTOR_COUNT))
    {
        return;
    }
    motor_write_levels(motor, s_period, s_period);
}

void motor_brake_all(void)
{
    uint32_t const irq_state = save_and_disable_interrupts();

    if (s_ready)
    {
        for (uint32_t m = 0U; m < MOTOR_COUNT; m++)
        {
            pwm_set_both_levels(pwm_gpio_to_slice_num(k_motor_pins[m].pin_fwd), 0U, 0U);
        }
    }
    else
    {
        for (uint32_t i = 0U; i < 4U; i++)
        {
            gpio_init(k_all_pins[i]);
            gpio_put(k_all_pins[i], false);
            gpio_set_dir(k_all_pins[i], GPIO_OUT);
        }
    }
    restore_interrupts(irq_state);
}

uint32_t motor_get_pwm_freq_hz(uint32_t motor)
{
    uint32_t slice;
    uint64_t div_x16;
    uint64_t top;

    if ((!s_ready) || (motor >= MOTOR_COUNT))
    {
        return 0U;
    }
    slice = pwm_gpio_to_slice_num(k_motor_pins[motor].pin_fwd);
    div_x16 = (uint64_t)(pwm_hw->slice[slice].div & 0xFFFU);    /* INT 11:4, FRAC 3:0 */
    top = (uint64_t)(pwm_hw->slice[slice].top & 0xFFFFU);
    if (div_x16 == 0U)
    {
        return 0U;
    }
    return (uint32_t)(((uint64_t)clock_get_hz(clk_sys) * 16U) / (div_x16 * (top + 1U)));
}

void motor_get_levels(uint32_t motor, uint32_t *p_fwd, uint32_t *p_rev, uint32_t *p_period)
{
    uint32_t level_fwd = 0U;
    uint32_t level_rev = 0U;

    if (s_ready && (motor < MOTOR_COUNT))
    {
        motor_pins_t const *p_pins = &k_motor_pins[motor];
        uint32_t const      cc = pwm_hw->slice[pwm_gpio_to_slice_num(p_pins->pin_fwd)].cc;
        uint32_t const      level_a = cc & 0xFFFFU;
        uint32_t const      level_b = cc >> 16;

        if (pwm_gpio_to_channel(p_pins->pin_fwd) == PWM_CHAN_A)
        {
            level_fwd = level_a;
            level_rev = level_b;
        }
        else
        {
            level_fwd = level_b;
            level_rev = level_a;
        }
    }
    if (p_fwd != NULL)
    {
        *p_fwd = level_fwd;
    }
    if (p_rev != NULL)
    {
        *p_rev = level_rev;
    }
    if (p_period != NULL)
    {
        *p_period = s_ready ? s_period : 0U;
    }
}

bool motor_are_all_braked(void)
{
    if (s_ready)
    {
        for (uint32_t i = 0U; i < 4U; i++)
        {
            if (gpio_get_function(k_all_pins[i]) != GPIO_FUNC_PWM)
            {
                return false;
            }
        }
        for (uint32_t m = 0U; m < MOTOR_COUNT; m++)
        {
            uint32_t const slice = pwm_gpio_to_slice_num(k_motor_pins[m].pin_fwd);

            if (pwm_hw->slice[slice].cc != 0U)
            {
                return false;
            }
        }
        return true;
    }

    for (uint32_t i = 0U; i < 4U; i++)
    {
        uint32_t const pin = k_all_pins[i];

        if ((gpio_get_function(pin) != GPIO_FUNC_SIO)
            || (!gpio_is_dir_out(pin))
            || gpio_get_out_level(pin))
        {
            return false;
        }
    }
    return true;
}

