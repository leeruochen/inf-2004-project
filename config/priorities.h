/**
 * @file priorities.h
 * @brief Task priorities, stack sizes and the kernel memory pool budget.
 *
 * Kernel priority = design priority (design.md section 3: P1 highest .. P9).
 * Design delta: nine design tasks plus one debug-only drain task at priority 10
 * (DBG_ENABLE builds only). Recorded in ARCHITECTURE.md.
 *
 * Pure macros: no SDK or kernel includes.
 */

#ifndef PRIORITIES_H
#define PRIORITIES_H

/* ---------------------------------------------------------------------------
 * Priorities (1 = highest)
 * ------------------------------------------------------------------------- */

/** Kernel init task (INITTASK_ITSKPRI). Creates every object and task, then tk_slp_tsk(TMO_FEVR). */
#define TASK_PRI_INIT           (1U)
/** Safety / Collision Response. */
#define TASK_PRI_SAFETY         (1U)
/** Vehicle Controller. */
#define TASK_PRI_VC             (2U)
/** Motion Control (PID). */
#define TASK_PRI_MOTION         (3U)
/** Line-Following Control. */
#define TASK_PRI_LF             (4U)
/** Barcode Decode. */
#define TASK_PRI_BC             (5U)
/** Obstacle Scanning. */
#define TASK_PRI_OBS_SCAN       (6U)
/** Obstacle Avoidance and Recovery. */
#define TASK_PRI_OBS_AVOID      (7U)
/** IMU Processing. */
#define TASK_PRI_IMU            (8U)
/** Telemetry / MQTT. */
#define TASK_PRI_TEL            (9U)
/** Debug drain (debug-only). The ONLY caller of stdio. */
#define TASK_PRI_DBG_DRAIN      (10U)

/** Design tasks. */
#define TASK_COUNT_APP          (9U)
/** All tasks: 9 design + init + debug drain (counted even when DBG_ENABLE is OFF). */
#define TASK_COUNT_TOTAL        (11U)

/* ---------------------------------------------------------------------------
 * Stacks. Tasks run on MSP, so ISR frames land on the running task's stack:
 * final size = base need + STACK_IRQ_MARGIN_BYTES. Do not shrink until the
 * stack-painting high-water marks have been measured (Batch B).
 * ------------------------------------------------------------------------- */

/** Margin added to every task stack for interrupt frames. */
#define STACK_IRQ_MARGIN_BYTES              (512U)

/** Base stack need, Safety. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_SAFETY_BYTES        (1024U)
/** Base stack need, Vehicle Controller. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_VC_BYTES            (1024U)
/** Base stack need, Motion Control. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_MOTION_BYTES        (1024U)
/** Base stack need, Line-Following. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_LF_BYTES            (1024U)
/** Base stack need, Barcode Decode. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_BC_BYTES            (1024U)
/** Base stack need, Obstacle Scanning. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_OBS_SCAN_BYTES      (1024U)
/** Base stack need, Obstacle Avoidance. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_OBS_AVOID_BYTES     (1024U)
/** Base stack need, IMU Processing. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_IMU_BYTES           (1536U)
/** Base stack need, Telemetry. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_TEL_BYTES           (4096U)
/** Base stack need, debug drain. ASSUMPTION. TUNE */
#define TASK_STACK_BASE_DBG_DRAIN_BYTES     (1024U)

/** Final stack, Safety. */
#define TASK_STACK_SAFETY_BYTES     (TASK_STACK_BASE_SAFETY_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Vehicle Controller. */
#define TASK_STACK_VC_BYTES         (TASK_STACK_BASE_VC_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Motion Control. */
#define TASK_STACK_MOTION_BYTES     (TASK_STACK_BASE_MOTION_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Line-Following. */
#define TASK_STACK_LF_BYTES         (TASK_STACK_BASE_LF_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Barcode Decode. */
#define TASK_STACK_BC_BYTES         (TASK_STACK_BASE_BC_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Obstacle Scanning. */
#define TASK_STACK_OBS_SCAN_BYTES   (TASK_STACK_BASE_OBS_SCAN_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Obstacle Avoidance. */
#define TASK_STACK_OBS_AVOID_BYTES  (TASK_STACK_BASE_OBS_AVOID_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, IMU Processing. */
#define TASK_STACK_IMU_BYTES        (TASK_STACK_BASE_IMU_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, Telemetry. */
#define TASK_STACK_TEL_BYTES        (TASK_STACK_BASE_TEL_BYTES + STACK_IRQ_MARGIN_BYTES)
/** Final stack, debug drain. */
#define TASK_STACK_DBG_DRAIN_BYTES  (TASK_STACK_BASE_DBG_DRAIN_BYTES + STACK_IRQ_MARGIN_BYTES)

/* ---------------------------------------------------------------------------
 * Kernel memory pool (static array handed to Imalloc by the port)
 * ------------------------------------------------------------------------- */

/** System stack the kernel adds to every tk_cre_tsk allocation (DEFAULT_SYS_STKSZ). */
#define KERNEL_SYS_STACK_BYTES              (128U)
/**
 * Kernel init task stack (INITTASK_STKSZ). Known exception to the
 * +STACK_IRQ_MARGIN_BYTES rule: the kernel fixes it at 1 KB in vendored
 * include/sys/inittask.h (no #ifndef, not overridable without editing vendored
 * code). usermain runs on it with interrupts live, so usermain must stay
 * stack-light (no large locals, no stdio), and the init task is included in the
 * Batch B high-water report. Recorded in ARCHITECTURE.md.
 */
#define KERNEL_INIT_TASK_STACK_BYTES        (1024U)
/** Static kernel pool. Do not shrink yet. */
#define KERNEL_POOL_BYTES                   (96U * 1024U)
/** Per-block Imalloc overhead (estimate). TUNE */
#define KERNEL_POOL_ALLOC_OVERHEAD_BYTES    (16U)
/** Pool reserved for message buffers (exact sizes from MBF_BUFSZ in Batch B). TUNE */
#define KERNEL_POOL_MBF_RESERVE_BYTES       (4096U)

/** Every task stack plus kernel overheads, as allocated from the pool. */
#define TASK_STACK_TOTAL_BYTES  ( TASK_STACK_SAFETY_BYTES + TASK_STACK_VC_BYTES              \
                                + TASK_STACK_MOTION_BYTES + TASK_STACK_LF_BYTES              \
                                + TASK_STACK_BC_BYTES + TASK_STACK_OBS_SCAN_BYTES            \
                                + TASK_STACK_OBS_AVOID_BYTES + TASK_STACK_IMU_BYTES          \
                                + TASK_STACK_TEL_BYTES + TASK_STACK_DBG_DRAIN_BYTES          \
                                + (TASK_COUNT_TOTAL * (KERNEL_SYS_STACK_BYTES                \
                                                       + KERNEL_POOL_ALLOC_OVERHEAD_BYTES))  \
                                + KERNEL_INIT_TASK_STACK_BYTES )

/* ---------------------------------------------------------------------------
 * Compile-time checks
 * ------------------------------------------------------------------------- */

#if (TASK_STACK_TOTAL_BYTES + KERNEL_POOL_MBF_RESERVE_BYTES) > KERNEL_POOL_BYTES
#error "priorities.h: task stacks + MBF reserve do not fit KERNEL_POOL_BYTES"
#endif

#if ((TASK_STACK_SAFETY_BYTES % 8U) != 0U) || ((TASK_STACK_VC_BYTES % 8U) != 0U)          \
    || ((TASK_STACK_MOTION_BYTES % 8U) != 0U) || ((TASK_STACK_LF_BYTES % 8U) != 0U)       \
    || ((TASK_STACK_BC_BYTES % 8U) != 0U) || ((TASK_STACK_OBS_SCAN_BYTES % 8U) != 0U)     \
    || ((TASK_STACK_OBS_AVOID_BYTES % 8U) != 0U) || ((TASK_STACK_IMU_BYTES % 8U) != 0U)   \
    || ((TASK_STACK_TEL_BYTES % 8U) != 0U) || ((TASK_STACK_DBG_DRAIN_BYTES % 8U) != 0U)
#error "priorities.h: every task stack must be a multiple of 8 bytes"
#endif

#if TASK_PRI_DBG_DRAIN <= TASK_PRI_TEL
#error "priorities.h: the debug drain must be the lowest-priority task"
#endif

#endif /* PRIORITIES_H */
