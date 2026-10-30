/*
 *  hal.h
 *  Platform services between the Pico SDK and micro T-Kernel.
 *  Owner: platform (Batch B, stage B3).  Pure header: <stdint.h> and
 *  <stdbool.h> only, so both sides may include it (rule R1).
 *
 *  Implemented in src/rtos/port/: port_boot.c (start, time, system-message
 *  ring), port_fault.c (fault record), port_irq.c (interrupts, SDK side),
 *  port_kernel_internal.c (ISR -> kernel calls, kernel side).
 *  hal_console_connected() is defined by src/app/main.c (stage B4).
 */

#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Return codes: the same values as the kernel's E_* codes. */
#define HAL_OK          (0)
#define HAL_E_PAR       (-17)       /**< E_PAR: bad argument */
#define HAL_E_LIMIT     (-34)       /**< E_LIMIT: no free handler slot */

/* ---------------------------------------------------------------------------
 * Start and time
 * ------------------------------------------------------------------------- */

/**
 * Start micro T-Kernel.  Called once by the SDK main() after SDK init.
 * Sets the exception priorities and the PendSV/SysTick handlers, then runs
 * the kernel's start sequence.  Never returns.
 */
void hal_kernel_start(void) __attribute__((noreturn));

/** Microseconds since boot (wraps after about 71 minutes). */
uint32_t hal_time_us(void);

/** True while a USB CDC host has the console open (stage B4, src/app/main.c). */
bool hal_console_connected(void);

/* ---------------------------------------------------------------------------
 * Interrupts.  Every ISR goes through these, never irq_set_exclusive_handler
 * or tk_def_int directly: the port wraps each handler so kernel calls from
 * it work (knl_taskindp).  At most HAL_IRQ_SLOTS handlers in total.
 * ------------------------------------------------------------------------- */

#define HAL_IRQ_SLOTS   (8U)

typedef void (*hal_isr_fn_t)(void);

/**
 * Install fn as the only handler of NVIC interrupt irq (not IO_IRQ_BANK0:
 * use hal_gpio_irq_register) at priority level 0 (highest) .. 3 (lowest),
 * and enable it.  Use level 1 or 2: level 0 shares the kernel tick's
 * priority, level 3 the dispatcher's.
 */
int32_t hal_irq_register(uint32_t irq, hal_isr_fn_t fn, uint32_t level);

/**
 * Handle GPIO events (GPIO_IRQ_* masks) on one pin through the SDK's shared
 * IO_IRQ_BANK0 chain.  fn must read and acknowledge its own pin's events
 * (gpio_get_irq_event_mask / gpio_acknowledge_irq).
 */
int32_t hal_gpio_irq_register(uint32_t gpio, uint32_t events, hal_isr_fn_t fn);

/** Kernel calls allowed from a registered ISR (IDs from ipc.h / task IDs). */
int32_t hal_isr_wake(int32_t tskid);                    /**< tk_wup_tsk */
int32_t hal_isr_sig(int32_t semid);                     /**< tk_sig_sem(semid, 1) */
int32_t hal_isr_setflg(int32_t flgid, uint32_t ptn);    /**< tk_set_flg */

/* ---------------------------------------------------------------------------
 * System-message ring (C3, batchB-plan O5): the kernel's own text
 * (tm_printf banner, system messages) via tm_snd_dat.  Read by the drain.
 * ------------------------------------------------------------------------- */

/**
 * Copy up to buf_bytes - 1 waiting bytes into p_buf, NUL-terminated.
 * Returns the number of bytes copied (0 = empty).  Drain task only.
 */
uint32_t hal_sysmsg_read(char *p_buf, uint32_t buf_bytes);

/** Bytes dropped because the ring was full, since boot. */
uint32_t hal_sysmsg_dropped(void);

/* ---------------------------------------------------------------------------
 * RAM fault record (C3, batchB-plan O4/O7, docs/DEBUG.md).  Faults only.
 * ------------------------------------------------------------------------- */

#define HAL_FAULT_MAGIC             (0x464C5452UL)  /**< "FLTR"; bytes "RTLF" in memory */
#define HAL_FAULT_VERSION           (1UL)
#define HAL_FAULT_TYPE_HARDFAULT    (1UL)
#define HAL_FAULT_TYPE_SHUTDOWN     (2UL)           /**< kernel shut down (knl_shutdown_hw) */

/** Payload covered by the CRC (7 words, O4). */
typedef struct
{
    uint32_t version;       /**< HAL_FAULT_VERSION */
    uint32_t type;          /**< HAL_FAULT_TYPE_* */
    uint32_t pc;            /**< stacked PC (0 if the stack pointer was invalid) */
    uint32_t lr;            /**< stacked LR */
    uint32_t xpsr;          /**< stacked xPSR */
    uint32_t kernel_err;    /**< kernel error code, if any (0 for a HardFault) */
    uint32_t boot_count;    /**< resets since the fault was recorded */
} hal_fault_payload_t;

/** Copy a valid record (magic and CRC match) into p_out.  False if none. */
bool hal_fault_get(hal_fault_payload_t *p_out);

/** Invalidate the record (the drain calls it after a delivered print). */
void hal_fault_clear(void);

/** True if a valid record was already there when this boot started. */
bool hal_fault_seen_at_boot(void);

/* ---------------------------------------------------------------------------
 * Platform self-test hooks (PLATFORM_SELFTEST builds only, stage B4b).
 * The stage survives watchdog resets (watchdog scratch register 0) and is
 * cleared by a power cycle, so the sequence runs once per power-up.
 * ------------------------------------------------------------------------- */

#define HAL_SELFTEST_STAGE_START        (0UL)   /**< after power-up */
#define HAL_SELFTEST_STAGE_AFTER_FAULT  (1UL)   /**< after the forced HardFault */
#define HAL_SELFTEST_STAGE_AFTER_WARM   (2UL)   /**< after the plain watchdog reset */
#define HAL_SELFTEST_STAGE_DONE         (3UL)

uint32_t hal_selftest_stage(void);
void     hal_selftest_set_stage(uint32_t stage);
/** Execute an undefined instruction inside this function: a HardFault. */
void     hal_selftest_force_fault(void) __attribute__((noreturn));
/** Plain watchdog reset (no fault). */
void     hal_selftest_warm_reboot(void) __attribute__((noreturn));
/** One line of kernel text through tm_printf, after the drain is running. */
void     hal_selftest_late_sysmsg(void);

#endif /* HAL_H */
