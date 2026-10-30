/*
 *  port.h
 *  Calls between the port's own files (src/rtos/port/ only), across the
 *  kernel/SDK split.  Not for application code: use hal.h.
 *  Owner: platform (Batch B, stage B3).  Pure header: <stdint.h> only.
 */

#ifndef PORT_H
#define PORT_H

#include <stdint.h>

/* port_kernel_internal.c (kernel side): the kernel's task-independent
 * marker (knl_taskindp), around every wrapped ISR. */
void port_isr_enter(void);
void port_isr_leave(void);

/* port_fault.c (SDK side). */
void port_fault_boot(void);                                 /* at start: count resets */
void port_fault_shutdown(void) __attribute__((noreturn));   /* knl_shutdown_hw */

/* port_boot.c (SDK side): text into the system-message ring. */
void port_sysmsg_puts(char const *p_text);

#endif /* PORT_H */
