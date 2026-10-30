/*
 *  stack_mon.h
 *  Task stack painting for the high-water report (batchB-plan O8).
 *  Owner: platform (Batch B, stage B2).  Kernel side: kernel headers only.
 *
 *  Stacks stay kernel-allocated.  app_start_task() registers each task here
 *  and starts it through stack_mon_trampoline(), which takes the exact stack
 *  top from SP at the task's first instruction, paints the unused part of
 *  the stack with STACK_MON_PATTERN, then calls the real entry.  The B6 scan
 *  counts the pattern words still untouched (stack_mon_unused_bytes()).
 *
 *  Why the bounds are exact: the kernel builds the first context just below
 *  tcb->isstack and the exception return pops all of it, so SP at the first
 *  instruction == isstack (armv6m/cpu_task.h knl_setup_context()).  The stack
 *  is round8(stksz + DEFAULT_SYS_STKSZ) bytes below it
 *  (kernel/tkernel/task_manage.c:62-65, 93).
 *
 *  A task ends by returning from its function: stack_mon_task_start() then
 *  calls tk_ext_tsk() with enough stack headroom (see stack_mon.c).  Calling
 *  tk_ext_tsk() or tk_exd_tsk() anywhere else is rule R6.
 *
 *  The 1 KB kernel init task (usermain) has no known top, so only a window
 *  of STACK_MON_INIT_WINDOW_BYTES below usermain's frame is painted
 *  (stack_mon_paint_init_window()): it measures how deep usermain goes
 *  below its entry, and never touches memory outside the init stack.
 */

#ifndef STACK_MON_H
#define STACK_MON_H

#include <stddef.h>                 /* size_t from the C library (see app.h) */
#include <stdint.h>
#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif
#include <tk/tkernel.h>

#define STACK_MON_PATTERN           (0xA5A5A5A5UL)
/** Painted window below usermain's frame on the init stack. */
#define STACK_MON_INIT_WINDOW_BYTES (512U)
/** Entries: every task app_start_task() can start, plus the init window. */
#define STACK_MON_MAX_ENTRIES       (16U)

typedef struct
{
    void      (*p_entry)(INT stacd, void *p_exinf);
    char const *p_name;
    uint32_t    stack_bytes;        /* as passed to tk_cre_tsk (stksz) */
    uint32_t   *p_bottom;           /* lowest stack word; NULL until painted */
    uint32_t   *p_top;              /* one past the highest stack word */
} stack_mon_entry_t;

/**
 * Reserve an entry for a task before tk_cre_tsk.  Returns the pointer to
 * pass as exinf (with task = stack_mon_trampoline), or NULL if the table
 * is full; app_start_task() then refuses the task.
 */
void *stack_mon_register(void (*p_entry)(INT stacd, void *p_exinf),
                         uint32_t stack_bytes, char const *p_name);

/** Task start address for registered tasks.  Never call it directly. */
void stack_mon_trampoline(INT stacd, void *p_exinf);

/** Called once at the start of usermain(), on the init stack. */
void stack_mon_paint_init_window(void);

/** Entries in use, and read access for the report. */
uint32_t stack_mon_count(void);
stack_mon_entry_t const *stack_mon_entry(uint32_t index);

/** Bytes at the bottom of an entry's stack never written since painting. */
uint32_t stack_mon_unused_bytes(stack_mon_entry_t const *p_entry);

#endif /* STACK_MON_H */
