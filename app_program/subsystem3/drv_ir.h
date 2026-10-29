/*
 *  drv_ir.h
 *  Subsystem 3 -- line sensor ADC driver interface (kernel-free).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  <stdint.h> only, so both drv_ir.c (SDK side) and the Line-Following task
 *  (kernel side) may include it.
 */

#ifndef DRV_IR_H
#define DRV_IR_H

#include <stdint.h>

/*
 * Boot-time sanity check of GP26 (IR_R, ADC0).  GP26 also has an untaped lead
 * on Grove 6 (config/pins.h), so a floating, shorted or double-driven pin must
 * be caught before the line sensor is trusted.
 *   Takes ADC_SANITY_SAMPLES samples and compares them with
 *   ADC_STUCK_LOW_RAW, ADC_STUCK_HIGH_RAW and ADC_NOISE_MAX_RAW
 *   (config/thresholds.h).
 * Returns 0 if the readings are plausible, 1 if GP26 is stuck at a rail or
 * abnormally noisy.  The caller logs and counts a 1 as a sensor fault; it
 * never acts on the motors.
 * Called once by the ADC owner (Line-Following) before its first sample.
 */
int32_t hal_adc_sanity_check(void);

#endif /* DRV_IR_H */
