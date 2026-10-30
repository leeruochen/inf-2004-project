/*
 *  port_knl_view.c
 *  Read-only views of kernel state that the public API does not offer.
 *  Owner: platform (Batch B, stage B6).  Kernel internals only: compiled
 *  INTO the mtk3 library (cmake/mtk3.cmake) with the kernel's internal
 *  headers, so the TCB layout comes from the kernel itself, never from a
 *  hard-coded offset.  Declared for project code in src/rtos/port/hal.h.
 */

#include "kernel.h"                 /* kernel/knlinc: TCB, knl_ctxtsk */

void *hal_task_stack_top(void);

/* Initial stack pointer (tcb->isstack) of the running task: the top of its
 * stack.  Called from task context only (knl_ctxtsk is the caller). */
void *hal_task_stack_top(void)
{
    return (knl_ctxtsk != NULL) ? knl_ctxtsk->isstack : NULL;
}
