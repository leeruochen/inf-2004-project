/*
 *  port_irq.c
 *  Interrupt registration through the SDK IRQ API, with every handler
 *  wrapped so kernel calls from it work (hal.h).
 *  Owner: platform (Batch B, stage B3).  SDK side.
 *
 *  Each handler gets one of HAL_IRQ_SLOTS fixed wrappers:
 *      port_isr_enter(); handler(); port_isr_leave();
 *  port_isr_enter/leave (port_kernel_internal.c) mark the kernel's
 *  task-independent part (knl_taskindp), exactly as the kernel's own ISR
 *  entry does.  GPIO handlers stay on the SDK's shared IO_IRQ_BANK0 chain
 *  (gpio_add_raw_irq_handler_masked), so several pins can each have one.
 *  Never tk_def_int: the SDK owns the vector table.
 *
 *  Registration runs in the init task (subsystemN_init/start), before the
 *  interrupts it enables can fire, so the slot table needs no lock.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "src/rtos/port/hal.h"
#include "src/rtos/port/port.h"

static hal_isr_fn_t s_slot_fn[HAL_IRQ_SLOTS];
static uint32_t     s_slots_used;

static void run_slot(uint32_t slot)
{
    port_isr_enter();
    s_slot_fn[slot]();
    port_isr_leave();
}

#define PORT_IRQ_WRAPPER(n)                     \
    static void port_irq_wrap_##n(void)         \
    {                                           \
        run_slot(n##U);                         \
    }

PORT_IRQ_WRAPPER(0)
PORT_IRQ_WRAPPER(1)
PORT_IRQ_WRAPPER(2)
PORT_IRQ_WRAPPER(3)
PORT_IRQ_WRAPPER(4)
PORT_IRQ_WRAPPER(5)
PORT_IRQ_WRAPPER(6)
PORT_IRQ_WRAPPER(7)

static irq_handler_t const k_wrappers[HAL_IRQ_SLOTS] =
{
    port_irq_wrap_0, port_irq_wrap_1, port_irq_wrap_2, port_irq_wrap_3,
    port_irq_wrap_4, port_irq_wrap_5, port_irq_wrap_6, port_irq_wrap_7,
};

/* Take a free slot for fn; returns its wrapper, or NULL when all are used. */
static irq_handler_t slot_take(hal_isr_fn_t fn)
{
    irq_handler_t wrapper;

    if (s_slots_used >= HAL_IRQ_SLOTS)
    {
        return NULL;
    }
    s_slot_fn[s_slots_used] = fn;
    wrapper = k_wrappers[s_slots_used];
    s_slots_used++;
    return wrapper;
}

int32_t hal_irq_register(uint32_t irq, hal_isr_fn_t fn, uint32_t level)
{
    irq_handler_t wrapper;

    if ((fn == NULL) || (irq >= NUM_IRQS) || (irq == IO_IRQ_BANK0) || (level > 3U))
    {
        return HAL_E_PAR;
    }
    wrapper = slot_take(fn);
    if (wrapper == NULL)
    {
        return HAL_E_LIMIT;
    }
    irq_set_exclusive_handler(irq, wrapper);
    irq_set_priority(irq, (uint8_t)(level << 6));
    irq_set_enabled(irq, true);
    return HAL_OK;
}

int32_t hal_gpio_irq_register(uint32_t gpio, uint32_t events, hal_isr_fn_t fn)
{
    irq_handler_t wrapper;

    if ((fn == NULL) || (gpio >= NUM_BANK0_GPIOS) || (events == 0U))
    {
        return HAL_E_PAR;
    }
    wrapper = slot_take(fn);
    if (wrapper == NULL)
    {
        return HAL_E_LIMIT;
    }
    gpio_add_raw_irq_handler_masked(1U << gpio, wrapper);
    gpio_set_irq_enabled(gpio, events, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
    return HAL_OK;
}
