/*
 *  drv_ir.c
 *  Subsystem 3 -- line sensor ADC driver.  SDK side: Pico SDK headers only,
 *  never kernel headers or subsystem3.h.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Stub (Phase 1 skeleton).  Compile-checked in app_program_check_sdk.
 */

#include "drv_ir.h"

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/adc.h"

int32_t hal_adc_sanity_check(void)
{
    /* TODO(phase 2, buddy 3): flag stuck-rail or abnormally noisy GP26 at boot
     * as a sensor fault; logged and counted, never acts on motors.  Sample
     * IR_R_ADC_CH ADC_SANITY_SAMPLES times and compare with ADC_STUCK_LOW_RAW,
     * ADC_STUCK_HIGH_RAW and ADC_NOISE_MAX_RAW. */
    return 0;
}
