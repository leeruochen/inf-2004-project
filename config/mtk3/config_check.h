/* robocar project file (not part of the BSP). Compile-time checks of the kernel config. */

/*
 * config_check.h
 * Static checks that the micro T-Kernel configuration in config/mtk3/config.h
 * provides every kernel object the robocar design needs. Included from the end
 * of config/mtk3/config.h, so every kernel translation unit runs these checks.
 *
 * dispatch.S reaches config.h through <sys/machine.h>, so all C-only content is
 * guarded by __ASSEMBLER__.
 */

#ifndef MTK3_CONFIG_CHECK_H
#define MTK3_CONFIG_CHECK_H

#if !defined(__ASSEMBLER__)

#include "../priorities.h"
#include "../timing.h"
#include "../thresholds.h"

/* Tick: 1 ms system timer, and it must match the project tick constant. */
_Static_assert(CNF_TIMER_PERIOD == 1, "config: CNF_TIMER_PERIOD must be 1 ms");
_Static_assert(CNF_TIMER_PERIOD == KERNEL_TICK_MS, "config: CNF_TIMER_PERIOD != KERNEL_TICK_MS");

/* Cyclic handlers: Motion 20 ms, Line 5 ms, ObsScan 60 ms, IMU 10 ms, Telemetry 20 ms. */
_Static_assert(CNF_MAX_CYCID >= 5, "config: need >= 5 cyclic handlers");

/* Message buffers: MBF_NAV_IN, MBF_MOTION_CMD, MBF_OBSTACLE, MBF_TELEMETRY. */
_Static_assert(CNF_MAX_MBFID >= 4, "config: need >= 4 message buffers");

/* Tasks: the owner asked for >= 10. There are 11: nine design tasks, the kernel
 * init task and the debug-only drain task. Check the larger number. */
_Static_assert(CNF_MAX_TSKID >= 11, "config: need >= 11 tasks (9 + init + debug drain)");
_Static_assert(CNF_MAX_TSKID >= TASK_COUNT_TOTAL, "config: CNF_MAX_TSKID < TASK_COUNT_TOTAL");

/* Eventflags: EVF_SAFETY, EVF_NAV. */
_Static_assert(CNF_MAX_FLGID >= 2, "config: need >= 2 eventflags");

/* Semaphores: SEM_ECHO. */
_Static_assert(CNF_MAX_SEMID >= 1, "config: need >= 1 semaphore");

/* Mutexes: MTX_MOTION (TA_INHERIT). */
_Static_assert(CNF_MAX_MTXID >= 1, "config: need >= 1 mutex");

/* Priorities: P1..P9 plus the debug drain task at 10. */
_Static_assert(CNF_MAX_TSKPRI >= 10, "config: need >= 10 task priorities");
_Static_assert(CNF_MAX_TSKPRI >= TASK_PRI_DBG_DRAIN, "config: CNF_MAX_TSKPRI < TASK_PRI_DBG_DRAIN");

/* Ownership: PWM IRQ and the sample drivers belong to the SDK, not the kernel. */
_Static_assert(USE_PTMR == 0, "config: USE_PTMR must be 0 (PWM is SDK-owned)");
_Static_assert(USE_SDEV_DRV == 0, "config: USE_SDEV_DRV must be 0 (SDK owns ADC/I2C)");
_Static_assert(USE_DEVICE == 0, "config: USE_DEVICE must be 0 (no device manager)");

/* The port supplies a static pool for Imalloc (task stacks, message buffers). */
_Static_assert(USE_IMALLOC == 1, "config: USE_IMALLOC must be 1 (port supplies the pool)");

/* Mirrors of the #error checks in priorities.h / thresholds.h as _Static_asserts. */
_Static_assert((TASK_STACK_TOTAL_BYTES + KERNEL_POOL_MBF_RESERVE_BYTES) <= KERNEL_POOL_BYTES,
               "config: task stacks (+128 B system stack each) do not fit KERNEL_POOL_BYTES");
_Static_assert(MOTION_SPEED_BYPASS_CMPS >= 15U, "config: bypass speed must be >= 15 cm/s");
_Static_assert(MOTION_SPEED_BYPASS_CMPS >= MOTION_SPEED_BYPASS_MIN_CMPS,
               "config: bypass speed below MOTION_SPEED_BYPASS_MIN_CMPS");

#endif /* !__ASSEMBLER__ */

#endif /* MTK3_CONFIG_CHECK_H */
