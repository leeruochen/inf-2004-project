/*
 *  drv_bc.h
 *  Subsystem 3 -- barcode IR sensor edge capture on GP27 (kernel-free).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  <stdint.h> and <stdbool.h> only, so both drv_bc.c (SDK side, owns the
 *  interrupt handler) and the Barcode task (kernel side) include it (R1).
 *
 *  Data path (IF-09, TEAM.md section 5):
 *    GP27 edge -> ISR: gap = now - last edge; push the gap into the edge ring
 *                 (drep_edge_ring_t, 64 x 20-bit gaps); wake the task
 *    task      -> drv_bc_pop() every gap, in edge order
 *
 *  Ownership (single core, so only compiler ordering matters; see drv_bc.c):
 *    ISR only:  writes the ring slots and head, the overflow count, the
 *               last-edge time and the edge/double counters.
 *    Task only: writes the ring tail (drv_bc_pop).
 *    Init only: everything, before the interrupt is enabled.
 */

#ifndef DRV_BC_H
#define DRV_BC_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Configure IR_BC_PIN (GP27) as a digital input with the pull-up on, empty
 * the edge ring and register the handler for both edges on IO_IRQ_BANK0
 * (hal_gpio_irq_register).  Edges are buffered from then on; nobody is woken
 * until drv_bc_set_wake_task().  Init task only; idempotent.
 * Returns HAL_OK (0) or the negative hal_gpio_irq_register() error.
 */
int32_t drv_bc_init(void);

/** True once drv_bc_init() has succeeded. */
bool drv_bc_is_ready(void);

/*
 * Task the handler wakes (tk_wup_tsk through hal_isr_wake) on every edge.
 * 0 = wake nobody.  Called by the init task once the Barcode task exists.
 */
void drv_bc_set_wake_task(int32_t tskid);

/** Task side: take the oldest gap (microseconds, saturated at 0xFFFFF).
 *  False when the ring is empty. */
bool drv_bc_pop(uint32_t *p_gap_us);

/** Gaps waiting in the ring (0..64). */
uint32_t drv_bc_pending(void);

/** hal_time_us() value of the newest edge (wraps every ~71 minutes). */
uint32_t drv_bc_last_edge_us(void);

/** Edges the ring refused because it was full, since boot (wraps at 256:
 *  compare two readings, never the value itself). */
uint32_t drv_bc_overflow_count(void);

/** Edges handled since boot (each pushed or refused). */
uint32_t drv_bc_edge_count(void);

/** Interrupts that saw both a rising and a falling edge at once: a pulse
 *  shorter than the interrupt latency, treated as noise (no gap pushed). */
uint32_t drv_bc_double_count(void);

/** GP27 level now (true = high).  Which level is a bar is measured in
 *  S3-U03; the decoder does not need it. */
bool drv_bc_level(void);

#endif /* DRV_BC_H */
