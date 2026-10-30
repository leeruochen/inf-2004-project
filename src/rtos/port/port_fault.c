/*
 *  port_fault.c
 *  RAM fault record and the HardFault handler (C3, batchB-plan O4/O7,
 *  docs/DEBUG.md).  Owner: platform (Batch B, stage B3).  SDK side.
 *
 *  The record lives in .uninitialized_data (NOLOAD, never zeroed by crt0,
 *  cmake/memmap_mtk3.ld:191-194), so it survives the reset that follows a
 *  fault.  Layout: magic, 7 payload words, CRC-32 over the payload.  The
 *  drain prints it on the next boot only if magic and CRC match; random RAM
 *  after power-on or flashing is rejected.
 *
 *  After recording: reset the whole chip with the watchdog (SYSRESETREQ
 *  resets only the core on RP2040), or halt for a debugger when the RAM flag
 *  port_fault_halt is set (from gdb: set var port_fault_halt = 1) or the
 *  build defines PORT_FAULT_HALT=1.  Cortex-M0+ code cannot read DHCSR, and a
 *  bkpt with no debugger attached would lock the core up (O7).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/platform.h"

#include "src/common/crc32.h"
#include "src/rtos/port/hal.h"
#include "src/rtos/port/port.h"

#ifndef PORT_FAULT_HALT
#define PORT_FAULT_HALT             (0)
#endif

/* Watchdog delay before the reset; long enough for nothing, short enough
 * that the record is printed promptly. */
#define PORT_FAULT_RESET_DELAY_MS   (10U)

typedef struct
{
    uint32_t            magic;
    hal_fault_payload_t payload;
    uint32_t            crc;
} port_fault_record_t;

_Static_assert(sizeof(hal_fault_payload_t) == (7U * 4U), "O4: 7 payload words");
_Static_assert(sizeof(port_fault_record_t) == (9U * 4U), "O4: 9 words in total");

static port_fault_record_t __uninitialized_ram(s_record);

/* Set to 1 from a debugger to halt on a fault instead of resetting. */
volatile uint32_t port_fault_halt;

/* A valid record was there when this boot started (port_fault_boot). */
static bool s_seen_at_boot;

/* Reached from isr_hardfault's assembly only. */
void port_fault_from_handler(uint32_t const *p_frame) __attribute__((noreturn, used));

static uint32_t payload_crc(hal_fault_payload_t const *p_payload)
{
    return crc32_ieee(p_payload, (uint32_t)sizeof(*p_payload));
}

static bool record_valid(void)
{
    return (s_record.magic == HAL_FAULT_MAGIC) && (s_record.crc == payload_crc(&s_record.payload));
}

/* Fill the record.  p_frame: the stacked exception frame, or NULL. */
static void record_fault(uint32_t type, uint32_t const *p_frame, uint32_t kernel_err)
{
    uint32_t const frame = (uint32_t)(uintptr_t)p_frame;

    s_record.payload.version = HAL_FAULT_VERSION;
    s_record.payload.type = type;
    s_record.payload.pc = 0U;
    s_record.payload.lr = 0U;
    s_record.payload.xpsr = 0U;
    /* Read the frame only if it lies in SRAM: after a stack overflow SP may
     * point anywhere, and a second fault here would lock the core up. */
    if ((p_frame != NULL) && (frame >= SRAM_BASE) && (frame <= (SRAM_END - (8U * 4U)))
        && ((frame & 3U) == 0U))
    {
        s_record.payload.lr = p_frame[5];       /* r0 r1 r2 r3 r12 lr pc xpsr */
        s_record.payload.pc = p_frame[6];
        s_record.payload.xpsr = p_frame[7];
    }
    s_record.payload.kernel_err = kernel_err;
    s_record.payload.boot_count = 0U;
    s_record.crc = payload_crc(&s_record.payload);
    s_record.magic = HAL_FAULT_MAGIC;
}

/* Reset (default) or halt for the debugger (O7).  Never returns. */
static void __attribute__((noreturn)) after_record(void)
{
    if ((PORT_FAULT_HALT != 0) || (port_fault_halt != 0U))
    {
        for (;;)
        {
            __breakpoint();
        }
    }
    watchdog_reboot(0U, 0U, PORT_FAULT_RESET_DELAY_MS);
    for (;;)
    {
        /* Wait for the watchdog. */
    }
}

/*
 * HardFault entry (overrides the SDK crt0's weak isr_hardfault).  Naked:
 * picks the stack the frame was pushed on (EXC_RETURN bit 2) and passes it
 * in r0.  BL is used because ARMv6-M has no long B; the handler never
 * returns, so LR does not matter.
 */
__attribute__((naked)) void isr_hardfault(void)
{
    __asm volatile(
        "movs   r0, #4                      \n"
        "mov    r1, lr                      \n"
        "tst    r0, r1                      \n"
        "beq    1f                          \n"
        "mrs    r0, psp                     \n"
        "b      2f                          \n"
        "1:                                 \n"
        "mrs    r0, msp                     \n"
        "2:                                 \n"
        "bl     port_fault_from_handler     \n");
}

void port_fault_from_handler(uint32_t const *p_frame)
{
    record_fault(HAL_FAULT_TYPE_HARDFAULT, p_frame, 0U);
    after_record();
}

void port_fault_shutdown(void)
{
    (void)save_and_disable_interrupts();
    record_fault(HAL_FAULT_TYPE_SHUTDOWN, NULL, 0U);
    after_record();
}

void port_fault_boot(void)
{
    s_seen_at_boot = record_valid();
    if (s_seen_at_boot)
    {
        s_record.payload.boot_count++;
        s_record.crc = payload_crc(&s_record.payload);
    }
}

bool hal_fault_seen_at_boot(void)
{
    return s_seen_at_boot;
}

bool hal_fault_get(hal_fault_payload_t *p_out)
{
    if ((p_out == NULL) || !record_valid())
    {
        return false;
    }
    *p_out = s_record.payload;
    return true;
}

void hal_fault_clear(void)
{
    s_record.magic = 0U;
}

#if PLATFORM_SELFTEST
/* ---------------------------------------------------------------------------
 * Self-test hooks (stage B4b).  The stage lives in watchdog scratch register
 * 0, which keeps its value through a watchdog reset and is cleared by a power
 * cycle (the bootrom uses scratch 4-7 only).
 * ------------------------------------------------------------------------- */

#define SELFTEST_STAGE_MAGIC    (0x5E1F0000UL)
#define SELFTEST_STAGE_MASK     (0x0000FFFFUL)

uint32_t hal_selftest_stage(void)
{
    uint32_t const raw = watchdog_hw->scratch[0];

    return ((raw & ~SELFTEST_STAGE_MASK) == SELFTEST_STAGE_MAGIC)
         ? (raw & SELFTEST_STAGE_MASK) : HAL_SELFTEST_STAGE_START;
}

void hal_selftest_set_stage(uint32_t stage)
{
    watchdog_hw->scratch[0] = SELFTEST_STAGE_MAGIC | (stage & SELFTEST_STAGE_MASK);
}

/* The recorded PC must be inside this function (the udf instruction). */
void __attribute__((noinline)) hal_selftest_force_fault(void)
{
    __asm volatile("udf #0");
    for (;;)
    {
        /* Not reached: the HardFault handler resets the chip. */
    }
}

void hal_selftest_warm_reboot(void)
{
    watchdog_reboot(0U, 0U, PORT_FAULT_RESET_DELAY_MS);
    for (;;)
    {
        /* Wait for the watchdog. */
    }
}
#endif /* PLATFORM_SELFTEST */
