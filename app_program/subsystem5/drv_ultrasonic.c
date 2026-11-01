/*
 *  drv_ultrasonic.c
 *  Subsystem 5 -- HC-SR04 ultrasonic range finder.  SDK side: Pico SDK
 *  headers plus the pure platform header src/rtos/port/hal.h only, never
 *  kernel headers or subsystem5.h (rule R1).
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  See drv_ultrasonic.h for the interface and the measurement sequence.
 *
 *  Why an ISR and not a polled pulse width: at 4 m the echo is 23 ms long.
 *  Polling it would block the Obstacle Scanning task for 23 of its 60 ms
 *  period and would break rule R2 (no blocking SDK waits).  Both echo edges
 *  therefore raise IO_IRQ_BANK0 through the port's shared dispatcher, and the
 *  task waits on SEM_ECHO instead.
 *
 *  The only spin in this file is the US_TRIG_PULSE_US (10 us) trigger pulse,
 *  which is 1/100 of a kernel tick.  It is written against time_us_32()
 *  rather than busy_wait_us() so that rule R2 stays satisfied in letter and
 *  in spirit.
 */

#include "drv_ultrasonic.h"

#include <stddef.h>

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "src/rtos/port/hal.h"

#if US_ECHO_PIN == US_TRIG_PIN
#error "drv_ultrasonic.c: US_TRIG_PIN and US_ECHO_PIN must differ"
#endif
#if US_TRIG_PULSE_US == 0U
#error "drv_ultrasonic.c: US_TRIG_PULSE_US must not be 0"
#endif

/** Speed of sound at 20 C, in millimetres per second / 1000 (343 mm/ms). */
#define US_SOUND_MM_PER_MS      (343U)

/* Shared with the echo handler: read and written under a critical section on
 * the task side, written only by the handler on the interrupt side. */
static volatile uint32_t s_rise_us = 0U;
static volatile uint32_t s_echo_us = 0U;
static volatile bool     s_have_rise = false;
static volatile bool     s_echo_ready = false;
static volatile bool     s_armed = false;
static volatile uint32_t s_echo_count = 0U;
static volatile uint32_t s_miss_count = 0U;

static volatile int32_t  s_echo_sem = 0;
static bool              s_ready = false;

/*
 * Echo edge handler.  Registered for both edges of US_ECHO_PIN through
 * hal_gpio_irq_register(), so it must read and acknowledge its own pin's
 * events (hal.h).  Keeps to timestamp arithmetic and one hal_isr_sig().
 */
static void echo_isr(void)
{
    uint32_t const events = gpio_get_irq_event_mask(US_ECHO_PIN);
    uint32_t const now_us = time_us_32();

    if ((events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) == 0U)
    {
        return;                         /* not ours */
    }
    gpio_acknowledge_irq(US_ECHO_PIN, events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL));

    if ((events & GPIO_IRQ_EDGE_RISE) != 0U)
    {
        s_rise_us = now_us;
        s_have_rise = true;
    }

    if (((events & GPIO_IRQ_EDGE_FALL) != 0U) && s_armed && s_have_rise)
    {
        uint32_t const width = now_us - s_rise_us;      /* wrap-safe */

        s_armed = false;
        s_have_rise = false;

        /* A width above US_ECHO_MAX_US is reverberation or a missed rising
         * edge: record it as "nothing in range" (0) rather than a wild
         * distance, but still release the waiting task. */
        s_echo_us = (width > (uint32_t)US_ECHO_MAX_US) ? 0U : width;
        s_echo_ready = true;
        s_echo_count++;

        if (s_echo_sem > 0)
        {
            (void)hal_isr_sig(s_echo_sem);
        }
    }
}

/*
 * Bounded spin for the trigger pulse only.  Written against time_us_32()
 * instead of busy_wait_us() (rule R2); 10 us at 125 MHz is about 1250 cycles.
 */
static void trig_spin_us(uint32_t us)
{
    uint32_t const start = time_us_32();

    while ((time_us_32() - start) < us)
    {
        tight_loop_contents();
    }
}

int32_t us_init(void)
{
    if (s_ready)
    {
        return HAL_OK;
    }

    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);

    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);
    /* No pull: the module drives the line.  A pull-up here would read as a
     * permanent echo when the sensor is unplugged, hiding the fault that
     * us_echo_pin_is_high() is there to catch. */
    gpio_disable_pulls(US_ECHO_PIN);

    int32_t const er = hal_gpio_irq_register(US_ECHO_PIN,
                                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                             echo_isr);
    if (er < HAL_OK)
    {
        return er;
    }

    s_ready = true;
    return HAL_OK;
}

bool us_is_ready(void)
{
    return s_ready;
}

void us_set_echo_sem(int32_t semid)
{
    s_echo_sem = semid;
}

bool us_trigger(void)
{
    if (!s_ready)
    {
        return false;
    }

    uint32_t const save = save_and_disable_interrupts();

    if (s_armed)
    {
        s_miss_count++;                 /* the previous ping never completed */
    }
    s_echo_ready = false;
    s_have_rise = false;
    s_echo_us = 0U;
    s_armed = true;
    restore_interrupts(save);

    gpio_put(US_TRIG_PIN, 1);
    trig_spin_us((uint32_t)US_TRIG_PULSE_US);
    gpio_put(US_TRIG_PIN, 0);

    return true;
}

bool us_echo_ready(void)
{
    return s_echo_ready;
}

bool us_take_echo_us(uint32_t *p_echo_us)
{
    bool           taken = false;
    uint32_t const save = save_and_disable_interrupts();

    if (s_echo_ready)
    {
        if (p_echo_us != NULL)
        {
            *p_echo_us = s_echo_us;
        }
        s_echo_ready = false;
        taken = true;
    }
    restore_interrupts(save);

    return taken;
}

uint32_t us_echo_us_to_mm(uint32_t echo_us)
{
    if ((echo_us == 0U) || (echo_us > (uint32_t)US_ECHO_MAX_US))
    {
        return 0U;
    }

    /* One way: (echo_us x 343 mm/ms) / (1000 us/ms x 2). */
    return (uint32_t)(((uint64_t)echo_us * (uint64_t)US_SOUND_MM_PER_MS) / 2000ULL);
}

uint32_t us_echo_count(void)
{
    return s_echo_count;
}

uint32_t us_miss_count(void)
{
    return s_miss_count;
}

bool us_echo_pin_is_high(void)
{
    return s_ready && gpio_get(US_ECHO_PIN);
}
