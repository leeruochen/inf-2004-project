/*
 *  subsystem3.c
 *  Subsystem 3 -- line following and barcode decode: brings up the barcode
 *  sensor and starts its tasks.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  See subsystem3.h for the tasks and interfaces.  Two-phase start (app.h):
 *    subsystem3_init()   line-sensor ADC and the GP27 input + edge handler
 *                        (drv_bc_init).  Edges are buffered in the IF-09 ring from
 *                        here on, nobody is woken yet.
 *    subsystem3_start()  the Line-Following task, then the Barcode Decode task,
 *                        then its ID is handed to the edge handler.  The init task
 *                        (priority 1) runs both, so the Barcode task (5) cannot run
 *                        in between.
 */

#include "subsystem3.h"

#include "config/pins.h"         /* IR_C_ADC_CH, IR_R_ADC_CH */
#include "drv_ir.h"             /* kernel-free driver interface */
#include "drv_bc.h"             /* kernel-free driver interface */
#include "task_bc.h"
#include "task_lf.h"

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

    /* Barcode (GP27): the input and the edge handler.  Edges are buffered from here on. */
    {
        int32_t const er = drv_bc_init();

        APP_CHECK(er, "s3 drv_bc_init");
    }

    /* The Line-Following task paces itself with app_period_wait()
     * (ARCHITECTURE.md T8): no cyclic handler to create here. */
    return (int32_t)E_OK;
}

int32_t subsystem3_start(void)
{
    /* Line-Following: calibrates at once (the car must be in the start
     * pose at power-up), then samples every 5 ms. */
    int32_t const lf_id = lf_task_start();
    int32_t       bc_id;

    if (lf_id < 0)
    {
        return lf_id;
    }
    /* Barcode Decode (BC-2A.2), then hand its ID to the edge handler. */
    bc_id = app_start_task(bc_task_entry, TASK_PRI_BC, TASK_STACK_BC_BYTES, "bc");
    if (bc_id < (int32_t)E_OK)
    {
        return bc_id;
    }
    drv_bc_set_wake_task(bc_id);
    return (int32_t)E_OK;
}
