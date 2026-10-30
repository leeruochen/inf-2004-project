/*
 *  port_kernel_internal.c
 *  Kernel side of the port: the symbols libmtk3 leaves for the port
 *  (cmake/mtk3.cmake), and the ISR -> kernel calls.
 *  Owner: platform (Batch B, stage B3).  Kernel side: kernel headers only.
 *
 *  Every definition that must match a kernel declaration cites it, so
 *  review can check the types (W = signed long, ER = W).
 */

#include <stddef.h>                 /* size_t from the C library (see app.h) */
#include <stdint.h>
#define PROHIBIT_DEF_SIZE_T
#include <tk/tkernel.h>

#include "config/priorities.h"
#include "src/rtos/port/hal.h"
#include "src/rtos/port/port.h"

/* ---------------------------------------------------------------------------
 * Kernel memory pool (Imalloc): task stacks, message buffers, ...
 * KERNEL_POOL_BYTES and its budget check: config/priorities.h.
 * ------------------------------------------------------------------------- */
_Static_assert((KERNEL_POOL_BYTES % 8U) == 0U, "KERNEL_POOL_BYTES must be a multiple of 8");

static uint64_t s_kernel_pool[KERNEL_POOL_BYTES / 8U];

/* kernel/knlinc/kernel.h:209  IMPORT void *knl_lowmem_top, *knl_lowmem_limit; */
void *knl_lowmem_top = &s_kernel_pool[0];
void *knl_lowmem_limit = &s_kernel_pool[KERNEL_POOL_BYTES / 8U];

/* ---------------------------------------------------------------------------
 * Hardware start/stop hooks
 * ------------------------------------------------------------------------- */

/* kernel/knlinc/kernel.h:215  IMPORT void knl_shutdown_hw( void );
 * Called when the kernel shuts down (usermain returned, or a start error in
 * sysinit.c).  Recorded as a fault, then reset or halt (O7). */
void knl_shutdown_hw(void)
{
    port_fault_shutdown();
}

/* kernel/knlinc/kernel.h:216  IMPORT ER knl_restart_hw( W mode ); */
ER knl_restart_hw(W mode)
{
    (void)mode;
    return E_NOSPT;
}

/* ---------------------------------------------------------------------------
 * ISR wrapper support (port_irq.c).  The kernel's own ISR entry does only
 * this (armv6m/cpu_status.h:71-78); a kernel call made inside pends PendSV
 * itself (END_CRITICAL_SECTION, cpu_status.h:30-35).
 * ------------------------------------------------------------------------- */

/* kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c:30  EXPORT W knl_taskindp = 0; */
extern W knl_taskindp;

void port_isr_enter(void)
{
    knl_taskindp++;
}

void port_isr_leave(void)
{
    knl_taskindp--;
}

/* ---------------------------------------------------------------------------
 * Kernel calls for registered ISRs (hal.h)
 * ------------------------------------------------------------------------- */

int32_t hal_isr_wake(int32_t tskid)
{
    return (int32_t)tk_wup_tsk((ID)tskid);
}

int32_t hal_isr_sig(int32_t semid)
{
    return (int32_t)tk_sig_sem((ID)semid, 1);
}

int32_t hal_isr_setflg(int32_t flgid, uint32_t ptn)
{
    return (int32_t)tk_set_flg((ID)flgid, (UINT)ptn);
}
