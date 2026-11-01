/*
 *  subsystem3.c
 *  Subsystem 3 -- line following and barcode decode: brings up the barcode
 *  sensor and starts its tasks.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Two-phase start (app.h):
 *    subsystem3_init()   GP27 input + edge handler (drv_bc_init).  Edges are
 *                        buffered in the IF-09 ring from here on, nobody is
 *                        woken yet.
 *    subsystem3_start()  Barcode Decode task, then hand its ID to the edge
 *                        handler.  The init task (priority 1) runs both, so
 *                        the Barcode task (5) cannot run in between.
 *  Line Following (LF-1..LF-3) is not written yet.
 */

#include "subsystem3.h"

#include "drv_bc.h"             /* kernel-free driver interface */
#include "task_bc.h"

int32_t subsystem3_init(void)
{
    int32_t const er = drv_bc_init();

    APP_CHECK(er, "s3 drv_bc_init");
    /* TODO(phase 2, buddy 3): Line-Following periodic wake (LF_SAMPLE_PERIOD_MS,
     * 5 ms): cyclic handler or app_period_wait(), open decision. */
    return (int32_t)E_OK;
}

int32_t subsystem3_start(void)
{
    int32_t const bc_id = app_start_task(bc_task_entry, TASK_PRI_BC, TASK_STACK_BC_BYTES, "bc");
    int32_t       result = (int32_t)E_OK;

    if (bc_id < (int32_t)E_OK)
    {
        result = bc_id;
    }
    else
    {
        drv_bc_set_wake_task(bc_id);
    }
    /* TODO(phase 2, buddy 3): app_start_task(LF task, TASK_PRI_LF,
     * TASK_STACK_LF_BYTES, "lf"). */
    return result;
}
