/*
 *  app.h
 *  Definitions shared by the whole team.  Changed only by team agreement.
 *  Owner: team (shared).  Kernel side only.
 *
 *  Ported from the course template (ProjectTemplate/app_program/app.h) and
 *  rewritten for this project: no <tm/tmonitor.h>, no <bsp/libbsp.h>, no
 *  TASK_STACK_SIZE (stacks come from config/priorities.h), no core argument
 *  (single core, no SMP).
 *
 *  Anything that belongs to ONE subsystem goes in that subsystem's own
 *  header instead (app_program/subsystemN/subsystemN.h).
 *
 *  Include rule (enforced by check_rules R1):
 *    - drv_*.c include Pico SDK headers only, plus their own kernel-free
 *      drv_*.h (<stdint.h> only).
 *    - task_*.c, subsystemN.c and subsystemN_test.c include kernel headers
 *      only (this header, subsystemN.h, <tk/tkernel.h>).
 *    - subsystemN.h and this header are never included by a drv_*.c.
 */

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <tk/tkernel.h>             /* micro T-Kernel API (tk_xxx_yyy) */

#include "config/priorities.h"      /* TASK_PRI_*, TASK_STACK_*_BYTES */

/* ---------------------------------------------------------------------------
 * Pins: every pin is defined in config/pins.h and claimed in TEAM.md
 * section 2 before it is wired or coded.  Never use GP23/24/25/29: they are
 * wired to the Pico W radio (CYW43).
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * Task priorities: 1 is the highest.  Ours are 1-10 (11 with APP_TEST),
 * defined in config/priorities.h and listed in TEAM.md section 4.  There are
 * no kernel service tasks above the application tasks.
 * ------------------------------------------------------------------------- */

/*
 * Kernel entry point, called by the kernel init task (priority 1, 1 KB
 * stack).  Defined in app_main.c.  The kernel declares it only in an internal
 * header (kernel/knlinc/kernel.h), which is not on our include path.
 */
INT usermain(void);

/*
 * Create and start one task.  Defined in app_main.c.
 *   p_entry      task function; the kernel passes stacd and exinf
 *   priority     TASK_PRI_* from config/priorities.h
 *   stack_bytes  TASK_STACK_*_BYTES from config/priorities.h
 *   p_name       short name for logs
 * Returns the task ID (> 0) or a negative kernel error code.
 */
int32_t app_start_task(void (*p_entry)(INT stacd, void *p_exinf),
                       uint32_t priority,
                       uint32_t stack_bytes,
                       char const *p_name);

/* ---------------------------------------------------------------------------
 * Two-phase start.  usermain() first creates every IPC object (ipc_init(),
 * Batch B), then calls ALL the _init() functions, and only then ALL the
 * _start() functions, so every kernel object exists before any task can use
 * it.
 *   subsystemN_init()   create this subsystem's private kernel objects
 *                       (cyclic handlers); never starts a task
 *   subsystemN_start()  create and start this subsystem's tasks
 *   subsystemN_test()   on-target tests, only in APP_TEST builds; run by the
 *                       test task at TASK_PRI_TEST
 * _init() and _start() return E_OK or a negative kernel error code.
 * ------------------------------------------------------------------------- */
int32_t subsystem1_init(void);
int32_t subsystem2_init(void);
int32_t subsystem3_init(void);
int32_t subsystem4_init(void);
int32_t subsystem5_init(void);

int32_t subsystem1_start(void);
int32_t subsystem2_start(void);
int32_t subsystem3_start(void);
int32_t subsystem4_start(void);
int32_t subsystem5_start(void);

#if APP_TEST
void subsystem1_test(void);
void subsystem2_test(void);
void subsystem3_test(void);
void subsystem4_test(void);
void subsystem5_test(void);
#endif /* APP_TEST */

/*
 * Report a failed kernel call (negative error code).  Evaluates er once.
 * Replaces the template's CHECK(), which called tm_printf from a task.
 */
#define APP_CHECK(er, p_what)                                               \
    do                                                                      \
    {                                                                       \
        int32_t app_check_er_ = (int32_t)(er);                              \
        if (app_check_er_ < (int32_t)E_OK)                                  \
        {                                                                   \
            (void)(p_what);                                                 \
            /* TODO(phase 1 batch B, buddy 2): dbg_ log line with p_what */ \
        }                                                                   \
    } while (0)

#endif /* APP_H */
