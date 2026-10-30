/*
 *  stack_mon.c
 *  Task stack painting (batchB-plan O8).  See stack_mon.h for the bounds.
 *  Owner: platform (Batch B, stage B2).  Kernel side: kernel headers only.
 *
 *  Registration runs only in the init task (app_start_task(), before
 *  tk_sta_tsk), so the table needs no lock.  Each task then writes only its
 *  own entry, once, at its first instruction.
 */

#include "src/common/stack_mon.h"

#include <sys/inittask.h>           /* INITTASK_STKSZ */

#include "config/priorities.h"
#include "src/rtos/port/hal.h"

_Static_assert(KERNEL_INIT_TASK_STACK_BYTES == INITTASK_STKSZ,
               "config/priorities.h KERNEL_INIT_TASK_STACK_BYTES != kernel INITTASK_STKSZ");

/* Never paint closer than this to the painting function's own frame. */
#define STACK_MON_GUARD_BYTES   (64U)

/*
 * Stack kept free below stack_mon_task_start() before tk_ext_tsk() runs.
 * tk_ext_tsk() -> knl_make_dormant() rebuilds the first context of the task
 * on the stack it is still running on: it writes the word at isstack - 68
 * (SStackFrame.exp_ret, armv6m/cpu_task.h knl_setup_context()).  The kernel
 * reserves only DORMANT_STACK_SIZE (28 B) in tk_ext_tsk(), which is enough
 * only when tk_ext_tsk() is called from the very top of the stack.  Called
 * 24 B below the top (stack_mon_task_start 16 B + an 8 B task function),
 * the write lands on knl_make_dormant()'s saved LR, and its return jumps to
 * 0xFFFFFFF8: HardFault (B4b stage 3, 30 Oct 2026).  64 B more puts every
 * live frame of the exit path below isstack - 68.
 */
#define STACK_MON_EXIT_PAD_WORDS (16U)

static stack_mon_entry_t s_entries[STACK_MON_MAX_ENTRIES];
static uint32_t          s_count;
/* Tasks whose kernel stack top (hal_task_stack_top) differed from SP at
 * their first instruction: must stay 0, or the init-stack bounds are wrong. */
static uint32_t          s_top_mismatches;

/* Defined below; reached from the trampoline's assembly only. */
void stack_mon_task_start(INT stacd, void *p_exinf, uint32_t *p_top)
    __attribute__((noreturn, used));

/* Ends the calling task with STACK_MON_EXIT_PAD_WORDS of headroom. */
static void __attribute__((noinline, noreturn)) exit_task_with_headroom(void)
{
    volatile uint32_t pad[STACK_MON_EXIT_PAD_WORDS];

    pad[0] = 0U;
    tk_ext_tsk();

    for (;;)
    {
        /* tk_ext_tsk() does not return.  Reading pad keeps the frame alive
         * across the call (no tail call that would drop the headroom). */
        (void)pad[0];
    }
}

/* Fill [p_bottom, p_limit) with the pattern, word by word. */
static void __attribute__((noinline)) paint_words(uint32_t *p_bottom, uint32_t *p_limit)
{
    volatile uint32_t *p = p_bottom;

    while (p < p_limit)
    {
        *p = STACK_MON_PATTERN;
        p++;
    }
}

/* Paint from p_bottom up to just below this call's own stack frame. */
static void __attribute__((noinline)) paint_below_here(uint32_t *p_bottom)
{
    uint32_t marker = 0U;
    uint32_t limit = ((uint32_t)(uintptr_t)&marker - STACK_MON_GUARD_BYTES) & ~3UL;

    if ((uint32_t)(uintptr_t)p_bottom < limit)
    {
        paint_words(p_bottom, (uint32_t *)(uintptr_t)limit);
    }
}

void *stack_mon_register(void (*p_entry)(INT stacd, void *p_exinf),
                         uint32_t stack_bytes, char const *p_name)
{
    stack_mon_entry_t *p_ent;

    if ((p_entry == NULL) || (s_count >= STACK_MON_MAX_ENTRIES))
    {
        return NULL;
    }
    p_ent = &s_entries[s_count];
    s_count++;

    p_ent->p_entry = p_entry;
    p_ent->p_name = p_name;
    p_ent->stack_bytes = stack_bytes;
    p_ent->p_bottom = NULL;
    p_ent->p_top = NULL;
    return p_ent;
}

/*
 * Task start address.  Naked: no prologue, so SP is still the stack top the
 * kernel set (tcb->isstack).  Passes it as the third argument (r2) and
 * branches with BL (ARMv6-M has no long B); stack_mon_task_start never
 * returns, so the clobbered LR does not matter.  stacd and p_exinf stay in
 * r0/r1 and reach stack_mon_task_start unchanged.
 */
__attribute__((naked)) void stack_mon_trampoline(INT stacd __attribute__((unused)),
                                                 void *p_exinf __attribute__((unused)))
{
    __asm volatile(
        "mov    r2, sp                  \n"
        "bl     stack_mon_task_start    \n");
}

void stack_mon_task_start(INT stacd, void *p_exinf, uint32_t *p_top)
{
    stack_mon_entry_t *p_ent = (stack_mon_entry_t *)p_exinf;
    /* Same size the kernel allocated: round8(stksz + DEFAULT_SYS_STKSZ). */
    uint32_t const alloc_bytes = (p_ent->stack_bytes + (uint32_t)DEFAULT_SYS_STKSZ + 7U) & ~7UL;

    p_ent->p_top = p_top;
    p_ent->p_bottom = (uint32_t *)(uintptr_t)((uint32_t)(uintptr_t)p_top - alloc_bytes);
    if (hal_task_stack_top() != (void *)p_top)
    {
        s_top_mismatches++;
    }
    paint_below_here(p_ent->p_bottom);

    p_ent->p_entry(stacd, NULL);

    /* The task function returned: the only way a task ends (rule R6). */
    exit_task_with_headroom();
}

void stack_mon_paint_init_window(void)
{
    uint32_t           marker = 0U;
    uint32_t const     here = (uint32_t)(uintptr_t)&marker & ~3UL;
    /* The kernel allocated the init stack like any task's:
     * round8(INITTASK_STKSZ + DEFAULT_SYS_STKSZ) below tcb->isstack. */
    uint32_t const     alloc_bytes = (KERNEL_INIT_TASK_STACK_BYTES + (uint32_t)DEFAULT_SYS_STKSZ + 7U) & ~7UL;
    uint32_t const     top = (uint32_t)(uintptr_t)hal_task_stack_top();
    stack_mon_entry_t *p_ent;

    if (s_count >= STACK_MON_MAX_ENTRIES)
    {
        return;
    }
    p_ent = &s_entries[s_count];
    s_count++;
    p_ent->p_entry = NULL;

    if ((top > here) && ((top - here) < alloc_bytes))
    {
        /* The whole init stack (stage B6). */
        p_ent->p_name = "init";
        p_ent->stack_bytes = KERNEL_INIT_TASK_STACK_BYTES;
        p_ent->p_top = (uint32_t *)(uintptr_t)top;
        p_ent->p_bottom = (uint32_t *)(uintptr_t)(top - alloc_bytes);
    }
    else
    {
        /* Top not plausible: only a window below usermain's frame. */
        p_ent->p_name = "init (usermain window)";
        p_ent->stack_bytes = STACK_MON_INIT_WINDOW_BYTES;
        p_ent->p_top = (uint32_t *)(uintptr_t)here;
        p_ent->p_bottom = (uint32_t *)(uintptr_t)(here - STACK_MON_INIT_WINDOW_BYTES);
    }
    paint_below_here(p_ent->p_bottom);
}

uint32_t stack_mon_top_mismatches(void)
{
    return s_top_mismatches;
}

uint32_t stack_mon_count(void)
{
    return s_count;
}

stack_mon_entry_t const *stack_mon_entry(uint32_t index)
{
    return (index < s_count) ? &s_entries[index] : NULL;
}

uint32_t stack_mon_unused_bytes(stack_mon_entry_t const *p_entry)
{
    volatile uint32_t const *p;
    uint32_t                 unused = 0U;

    if ((p_entry == NULL) || (p_entry->p_bottom == NULL))
    {
        return 0U;
    }
    p = p_entry->p_bottom;
    while ((p < p_entry->p_top) && (*p == STACK_MON_PATTERN))
    {
        unused += 4U;
        p++;
    }
    return unused;
}
