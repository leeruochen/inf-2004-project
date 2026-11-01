/*
 *  drv_bc.c
 *  Subsystem 3 -- barcode IR sensor edge capture on GP27.  SDK side: Pico
 *  SDK headers plus pure project headers only, never kernel headers or
 *  subsystem3.h (rule R1).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Why an interrupt: one narrow element lasts 5.9 ms at the top speed, and
 *  the decoder needs its length to ~1 %.  A 2 ms task poll of the pin could
 *  be ~35 % off; the edge interrupt timestamps each edge to a few us.
 *
 *  The handler does only: acknowledge, timestamp, push one gap, wake the
 *  Barcode task.  No decoding, no logging (dbg_ is tasks only).
 *
 *  Concurrency (single core RP2040, Cortex-M0+: no cache, in-order; the
 *  handler runs to completion before the task resumes):
 *    - The edge ring is single-producer (this handler) / single-consumer
 *      (the task).  drep_edge_push() stores the slot, then a compiler
 *      barrier, then head; drep_edge_pop() reads head, then a barrier, then
 *      the slot, then a barrier, then tail.  head and tail are volatile
 *      uint8_t (one byte store each, never torn).
 *    - s_last_edge_us and the counters are aligned 32-bit words written only
 *      here: one STR each, so a task read is never torn.  volatile makes
 *      every task read a fresh load.
 *    - s_wake_tid and the ring are set up before the interrupt is enabled.
 */

#include "drv_bc.h"

#include <stddef.h>

#include "config/pins.h"            /* IR_BC_PIN */
#include "hardware/gpio.h"
#include "pico/time.h"
#include "src/common/data_repr.h"   /* drep_edge_ring_t (IF-09) */
#include "src/rtos/port/hal.h"

/** Both edge types: every bar/space boundary. */
#define BC_EDGE_EVENTS          (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)

static drep_edge_ring_t     s_ring;                 /* IF-09 */
static volatile uint32_t    s_last_edge_us = 0U;
static volatile uint32_t    s_edge_count = 0U;
static volatile uint32_t    s_double_count = 0U;
static volatile int32_t     s_wake_tid = 0;
static bool                 s_ready = false;

/*
 * GP27 edge handler, registered through hal_gpio_irq_register(), so it must
 * read and acknowledge its own pin's events (hal.h).
 */
static void bc_edge_isr(void)
{
    uint32_t const now_us = time_us_32();           /* = hal_time_us() */
    uint32_t const events = gpio_get_irq_event_mask(IR_BC_PIN) & BC_EDGE_EVENTS;

    if (events != 0U)
    {
        gpio_acknowledge_irq(IR_BC_PIN, events);

        if (events == BC_EDGE_EVENTS)
        {
            /* Rise and fall both latched: a pulse shorter than the interrupt
             * latency.  The level is back where it was, so pushing nothing
             * keeps bars and spaces alternating; the element goes on. */
            s_double_count++;
        }
        else
        {
            uint32_t const gap_us = now_us - s_last_edge_us;    /* wrap-safe */

            s_last_edge_us = now_us;
            (void)drep_edge_push(&s_ring, gap_us);  /* full: counted in s_ring.overflow */
            s_edge_count++;
            if (s_wake_tid > 0)
            {
                (void)hal_isr_wake(s_wake_tid);     /* E_QOVR is harmless: already queued */
            }
        }
    }
}

int32_t drv_bc_init(void)
{
    int32_t er = HAL_OK;

    if (!s_ready)
    {
        drep_edge_init(&s_ring);
        s_last_edge_us = time_us_32();

        /* Digital input.  Never adc_gpio_init() here: it turns the digital
         * input buffer off (config/pins.h).  Pull-up: the module drives DO,
         * so the pull changes nothing when connected, and an unplugged lead
         * reads a steady high instead of floating into noise edges. */
        gpio_init(IR_BC_PIN);
        gpio_set_dir(IR_BC_PIN, GPIO_IN);
        gpio_pull_up(IR_BC_PIN);

        er = hal_gpio_irq_register(IR_BC_PIN, BC_EDGE_EVENTS, bc_edge_isr);
        if (er >= HAL_OK)
        {
            s_ready = true;
        }
    }

    return er;
}

bool drv_bc_is_ready(void)
{
    return s_ready;
}

void drv_bc_set_wake_task(int32_t tskid)
{
    s_wake_tid = tskid;
}

bool drv_bc_pop(uint32_t *p_gap_us)
{
    return (p_gap_us != NULL) && drep_edge_pop(&s_ring, p_gap_us);
}

uint32_t drv_bc_pending(void)
{
    return drep_edge_count(&s_ring);
}

uint32_t drv_bc_last_edge_us(void)
{
    return s_last_edge_us;
}

uint32_t drv_bc_overflow_count(void)
{
    return (uint32_t)s_ring.overflow;
}

uint32_t drv_bc_edge_count(void)
{
    return s_edge_count;
}

uint32_t drv_bc_double_count(void)
{
    return s_double_count;
}

bool drv_bc_level(void)
{
    return gpio_get(IR_BC_PIN);
}
