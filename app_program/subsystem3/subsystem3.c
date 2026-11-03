/*
 *  subsystem3.c
 *  Subsystem 3 -- line following and barcode decode: creates its objects
 *  and starts its tasks.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Stub (Phase 1 skeleton).  See subsystem3.h for the tasks and interfaces.
 */

#include "subsystem3.h"

#include "config/pins.h"         /* IR_C_ADC_CH, IR_R_ADC_CH */
#include "drv_ir.h"             /* kernel-free driver interface */

/** Sanity verdict names for the boot log (lf_adc_status_t order). */
static char const *const k_ir_verdict[5] =
{
    "OK", "STUCK LOW", "STUCK HIGH", "NOISY", "NO SAMPLES"
};

/* Boot check of one line sensor: dbg_puts only, the init task's stack is
 * 1 KB (dbg.h: no dbg_printf here).  A fault is logged, never acted on. */
static void s3_ir_boot_check(char const *p_line_ok, char const *p_line_bad, uint32_t ch)
{
    lf_adc_status_t const st = drv_ir_sanity(ch, NULL);

    if (st == LF_ADC_OK)
    {
        dbg_puts(p_line_ok);
    }
    else
    {
        dbg_puts(p_line_bad);
        dbg_puts(k_ir_verdict[(uint32_t)st]);
    }
}

int32_t subsystem3_init(void)
{
    /* Line sensors (LF-1): ADC set-up, then the boot sanity check of both.
     * hal_adc_sanity_check() (drv_ir.h) is the GP26 case of the same check. */
    drv_ir_init();
    s3_ir_boot_check("[S3] IR_R (GP26/ADC0) ADC sanity OK", "[S3] IR_R (GP26/ADC0) ADC sanity FAULT:",
                     IR_R_ADC_CH);
    s3_ir_boot_check("[S3] IR_C (GP28/ADC2) ADC sanity OK", "[S3] IR_C (GP28/ADC2) ADC sanity FAULT:",
                     IR_C_ADC_CH);

    /* TODO(phase 2, buddy 3): Line-Following periodic wake (LF_SAMPLE_PERIOD_MS,
     * 5 ms): cyclic handler or app_period_wait(), open decision (LF-3). */
    return (int32_t)E_OK;
}

int32_t subsystem3_start(void)
{
    /* TODO(phase 2, buddy 3): app_start_task(LF task, TASK_PRI_LF,
     * TASK_STACK_LF_BYTES, "lf") and app_start_task(BC task, TASK_PRI_BC,
     * TASK_STACK_BC_BYTES, "bc"), then start the cyclic handler. */
    return (int32_t)E_OK;
}
