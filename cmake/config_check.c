/**
 * @file config_check.c
 * @brief Build-time check of every project config header.
 *
 * Compiled as the OBJECT library `config_check` with -Wall -Wextra -Werror.
 * It is never linked into firmware. Including the headers runs their #error
 * checks and the _Static_asserts in config/mtk3/config_check.h.
 * Pure-macro headers only: no SDK or kernel API headers here.
 */

#include "config/pins.h"
#include "config/timing.h"
#include "config/priorities.h"
#include "config/thresholds.h"
#include "config/mtk3/config.h"        /* kernel config; pulls in config_check.h at its end */
#include "config/mtk3/config_check.h"  /* already included above (guarded); listed for completeness */

/** Touch a few values so the TU is not empty and the macros expand as C. */
unsigned long robocar_config_check(void);

unsigned long robocar_config_check(void)
{
    return PINS_ALLOCATED_MASK
         + (unsigned long)TASK_STACK_TOTAL_BYTES
         + (unsigned long)LF_SAMPLE_PERIOD_MS
         + (unsigned long)MOTION_SPEED_BYPASS_CMPS
         + (unsigned long)CNF_MAX_TSKID;
}
