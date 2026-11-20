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
 *  Why a glitch filter (bc_edge.c): the sensor's comparator flickers at
 *  every black/white edge (first hardware runs, 1 Nov 2026: hundreds of
 *  flips of 20-800 us per pass).  The handler gives each change of the pin's
 *  LEVEL to the filter; only confirmed bars and spaces reach the ring.
 *
 *  The handler does only: acknowledge, timestamp, read the level, run the
 *  filter (a few comparisons), and for a confirmed gap: push it and wake
 *  the Barcode task.
 *  No decoding, no logging (dbg_ is tasks only).
 *
 *  Concurrency (single core RP2040, Cortex-M0+: no cache, in-order; the
 *  handler runs to completion before any task resumes):
 *    - s_filter is changed by the handler, and by the task only inside
 *      save_and_disable_interrupts() .. restore_interrupts() (drv_bc_settle,
 *      drv_bc_last_edge_us), so the two never interleave.
 *    - The edge ring has two producers: the handler, and drv_bc_settle()
 *      with interrupts disabled.  They cannot run at the same time, so for
 *      the ring there is still one producer at a time and one consumer (the
 *      task, drv_bc_pop).  drep_edge_push() stores the slot, a compiler
 *      barrier, then head; drep_edge_pop() reads head, a barrier, the slot,
 *      a barrier, then tail.  head and tail are volatile uint8_t.
 *    - The counters are aligned 32-bit words written only with interrupts
 *      off or in the handler: one STR each, so a task read is never torn.
 *    - s_wake_tid, the filter and the ring are set up before the interrupt
 *      is enabled.
 */

#include "drv_bc.h"

#include <stddef.h>

#include "config/pins.h"            /* IR_BC_PIN */
#include "config/timing.h"          /* BC_GLITCH_US */
#include "hardware/gpio.h"
#include "hardware/sync.h"          /* save_and_disable_interrupts() */
#include "pico/time.h"
#include "src/common/data_repr.h"   /* drep_edge_ring_t (IF-09) */
#include "src/rtos/port/hal.h"
#include "bc_edge.h"

/** Both edge types: every bar/space boundary. */
#define BC_EDGE_EVENTS          (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)

static drep_edge_ring_t     s_ring;                 /* IF-09 */
static bc_edge_t            s_filter;               /* handler; task with IRQs off */
static volatile uint32_t    s_irq_count = 0U;
static volatile uint32_t    s_edge_count = 0U;      /* gaps pushed */
static volatile uint32_t    s_glitch_count = 0U;
static volatile uint32_t    s_ignored_count = 0U;
static volatile int32_t     s_wake_tid = 0;
static bool                 s_ready = false;

/* Push one confirmed gap.  Handler, or task with interrupts disabled. */
static void bc_push(uint32_t gap_us)
{
    (void)drep_edge_push(&s_ring, gap_us);  /* full: counted in s_ring.overflow */
    s_edge_count++;
}

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
        uint32_t gap_us = 0U;

        gpio_acknowledge_irq(IR_BC_PIN, events);
        s_irq_count++;

        /* The level now, not the event bits: a rise and a fall latched
         * together leave the level unchanged, and the filter ignores that. */
        switch (bc_edge_on_change(&s_filter, gpio_get(IR_BC_PIN), now_us, &gap_us))
        {
            case BC_EDGE_CONFIRMED:
                bc_push(gap_us);
                if (s_wake_tid > 0)
                {
                    /* Only a confirmed gap is work for the task: flicker
                     * must not turn into thousands of kernel calls. */
                    (void)hal_isr_wake(s_wake_tid);     /* E_QOVR is harmless */
                }
                break;
            case BC_EDGE_GLITCH:
                s_glitch_count++;
                break;
            case BC_EDGE_IGNORED:
                s_ignored_count++;
                break;
            case BC_EDGE_TENTATIVE:
            default:
                break;
        }
    }
}

int32_t drv_bc_init(void)
{
    int32_t er = HAL_OK;

    if (!s_ready)
    {
        drep_edge_init(&s_ring);

        /* Digital input.  Never adc_gpio_init() here: it turns the digital
         * input buffer off (config/pins.h).  Pull-up: the module drives DO,
         * so the pull changes nothing when connected, and an unplugged lead
         * reads a steady high instead of floating into noise edges. */
        gpio_init(IR_BC_PIN);
        gpio_set_dir(IR_BC_PIN, GPIO_IN);
        gpio_pull_up(IR_BC_PIN);
        bc_edge_init(&s_filter, gpio_get(IR_BC_PIN), time_us_32(), BC_GLITCH_US);

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

bool drv_bc_settle(void)
{
    uint32_t gap_us = 0U;
    uint32_t const irq = save_and_disable_interrupts();
    bool const     done = bc_edge_settle(&s_filter, time_us_32(), &gap_us);

    if (done)
    {
        bc_push(gap_us);
    }
    restore_interrupts(irq);

    return done;
}

uint32_t drv_bc_last_edge_us(void)
{
    uint32_t const irq = save_and_disable_interrupts();
    uint32_t const t = bc_edge_last_us(&s_filter);

    restore_interrupts(irq);
    return t;
}

uint32_t drv_bc_overflow_count(void)
{
    return (uint32_t)s_ring.overflow;
}

uint32_t drv_bc_edge_count(void)
{
    return s_edge_count;
}

uint32_t drv_bc_glitch_count(void)
{
    return s_glitch_count;
}

uint32_t drv_bc_ignored_count(void)
{
    return s_ignored_count;
}

uint32_t drv_bc_irq_count(void)
{
    return s_irq_count;
}

bool drv_bc_level(void)
{
    return gpio_get(IR_BC_PIN);
}
