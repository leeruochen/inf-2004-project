/*
 *  port_boot.c
 *  Kernel start (hal_kernel_start), time, and the system-message ring that
 *  takes the kernel's own console text (tm_snd_dat).
 *  Owner: platform (Batch B, stage B3).  SDK side: SDK headers only; kernel
 *  functions are declared here as bare prototypes, each citing its kernel
 *  declaration.
 *
 *  Replaces the excluded kernel reset_hdl.c, step by step:
 *    - .data/.bss init, VTOR: done by the SDK crt0/runtime (approach (a)).
 *    - knl_lowmem_top/limit: static pool in port_kernel_internal.c.
 *    - SHPR2/SHPR3: here (reset_hdl.c:100-102; SVC 0, SysTick 0, PendSV 3).
 *    - PendSV/SysTick vectors: entries of the SDK's RAM vector table.
 *    - main(): the kernel's sysinit.c main, renamed mtk3_kernel_main.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/m0plus.h"
#include "hardware/timer.h"
#include "pico/platform.h"

#include "src/rtos/port/hal.h"
#include "src/rtos/port/port.h"

/* kernel/sysinit/sysinit.c:26  EXPORT INT main( void ), compiled with
 * -Dmain=mtk3_kernel_main (cmake/mtk3.cmake).  Never returns on success. */
extern int mtk3_kernel_main(void);

/* kernel/sysdepend/cpu/core/armv6m/dispatch.S:58-60  knl_dispatch_entry (PendSV) */
extern void knl_dispatch_entry(void);

/* kernel/sysdepend/cpu/core/armv6m/interrupt.c:60  EXPORT void knl_systim_inthdr(void) */
extern void knl_systim_inthdr(void);

/* The kernel programs SysTick for this clock (include/sys/sysdepend/
 * pico_rp2040/sysdef.h:39-41, TMCLK_KHz = 125000); SDK 1.5.1 default. */
#define PORT_SYSCLK_HZ          (125000000UL)

/* Exception priorities, as reset_hdl.c:100-102 with rp2040 sysdef.h:72-74:
 * 2 priority bits in the top of each byte. */
#define PORT_PRI_SVC            (0UL)
#define PORT_PRI_SYSTICK        (0UL)
#define PORT_PRI_PENDSV         (3UL)
#define PORT_SHPR2_VAL          (PORT_PRI_SVC << 30)
#define PORT_SHPR3_VAL          ((PORT_PRI_SYSTICK << 30) | (PORT_PRI_PENDSV << 22))

/* ---------------------------------------------------------------------------
 * System-message ring (C3, batchB-plan O5).  One writer (tm_snd_dat; libtm
 * wraps every call in DI/EI, lib/libtm/libtm.c:89-94, 108-112) and one
 * reader (the drain).  Free-running 32-bit indices: the writer owns s_head,
 * the reader owns s_tail, so no lock is needed on one core.
 * ------------------------------------------------------------------------- */

#define SYSMSG_BYTES            (512U)
#define SYSMSG_MASK             (SYSMSG_BYTES - 1U)
_Static_assert((SYSMSG_BYTES & SYSMSG_MASK) == 0U, "SYSMSG_BYTES must be a power of two");

static char              s_sysmsg[SYSMSG_BYTES];
static volatile uint32_t s_sysmsg_head;     /* written by the writer only */
static volatile uint32_t s_sysmsg_tail;     /* written by the reader only */
static volatile uint32_t s_sysmsg_dropped;  /* written by the writer only */

static void sysmsg_write(char const *p_buf, uint32_t len)
{
    uint32_t head = s_sysmsg_head;
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        if ((head - s_sysmsg_tail) >= SYSMSG_BYTES)
        {
            s_sysmsg_dropped += len - i;
            break;
        }
        s_sysmsg[head & SYSMSG_MASK] = p_buf[i];
        head++;
    }
    __compiler_memory_barrier();    /* bytes stored before the reader sees head */
    s_sysmsg_head = head;
}

/* lib/libtm/libtm.h:23  IMPORT void tm_com_init(void); */
void tm_com_init(void)
{
    /* The ring is zero-initialised .bss; nothing to set up. */
}

/* lib/libtm/libtm.h:25  IMPORT void tm_snd_dat( const UB* buf, INT size );
 * Never blocks and never writes to stdio (C3). */
void tm_snd_dat(const unsigned char *buf, int size)
{
    if ((buf != NULL) && (size > 0))
    {
        sysmsg_write((char const *)buf, (uint32_t)size);
    }
}

/* lib/libtm/libtm.h:26  IMPORT void tm_rcv_dat( UB* buf, INT size );
 * There is no console input: return NUL bytes. */
void tm_rcv_dat(unsigned char *buf, int size)
{
    int i;

    for (i = 0; (buf != NULL) && (i < size); i++)
    {
        buf[i] = 0U;
    }
}

void port_sysmsg_puts(char const *p_text)
{
    uint32_t len = 0U;

    if (p_text == NULL)
    {
        return;
    }
    while (p_text[len] != '\0')
    {
        len++;
    }
    sysmsg_write(p_text, len);
}

uint32_t hal_sysmsg_read(char *p_buf, uint32_t buf_bytes)
{
    uint32_t const head = s_sysmsg_head;    /* read once */
    uint32_t       tail = s_sysmsg_tail;
    uint32_t       n = 0U;

    if ((p_buf == NULL) || (buf_bytes == 0U))
    {
        return 0U;
    }
    __compiler_memory_barrier();            /* head read before the bytes */
    while ((tail != head) && (n < (buf_bytes - 1U)))
    {
        p_buf[n] = s_sysmsg[tail & SYSMSG_MASK];
        n++;
        tail++;
    }
    p_buf[n] = '\0';
    __compiler_memory_barrier();            /* bytes copied before the slot is freed */
    s_sysmsg_tail = tail;
    return n;
}

uint32_t hal_sysmsg_dropped(void)
{
    return s_sysmsg_dropped;
}

/* ---------------------------------------------------------------------------
 * Time
 * ------------------------------------------------------------------------- */

uint32_t hal_time_us(void)
{
    return time_us_32();
}

/* ---------------------------------------------------------------------------
 * Kernel start
 * ------------------------------------------------------------------------- */

void hal_kernel_start(void)
{
    if (clock_get_hz(clk_sys) != PORT_SYSCLK_HZ)
    {
        port_sysmsg_puts("port: clk_sys is not 125 MHz; kernel ticks are not 1 ms\n");
    }

    port_fault_boot();

    *(io_rw_32 *)(PPB_BASE + M0PLUS_SHPR2_OFFSET) = PORT_SHPR2_VAL;
    *(io_rw_32 *)(PPB_BASE + M0PLUS_SHPR3_OFFSET) = PORT_SHPR3_VAL;

    (void)exception_set_exclusive_handler(PENDSV_EXCEPTION, knl_dispatch_entry);
    (void)exception_set_exclusive_handler(SYSTICK_EXCEPTION, knl_systim_inthdr);

    (void)mtk3_kernel_main();

    /* mtk3_kernel_main() returns only after the kernel shut down, which
     * knl_shutdown_hw() already handled (record, reset or halt). */
    port_fault_shutdown();
}
