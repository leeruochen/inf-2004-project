/*
 *  drv_ir.h
 *  Subsystem 3 -- line sensor ADC driver interface (kernel-free).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  <stdint.h> and pure project headers only, so both drv_ir.c (SDK side)
 *  and kernel-side callers (subsystem3.c, the tests, later the
 *  Line-Following task) may include it.
 *
 *  Sensors (config/pins.h, ARCHITECTURE.md D11), analogue output AO only:
 *    CENTRE  IR_C_PIN = GP28 = ADC2 (Grove 7; the VBAT-sense jumper must be OPEN)
 *    RIGHT   IR_R_PIN = GP26 = ADC0 (Grove 5; Grove 6 has a second GP26 lead)
 *
 *  Ownership: Subsystem 3 owns the ADC (TEAM.md section 3).  adc_select_input()
 *  then adc_read() is not atomic, so only ONE task may call drv_ir_read() at a
 *  time: in LF-1 the boot check (init task) and then the on-target test task;
 *  from LF-3 the Line-Following task only.
 */

#ifndef DRV_IR_H
#define DRV_IR_H

#include <stdint.h>

#include "app_program/subsystem3/lf_sanity.h"

/*
 * Set up the ADC and both sensor pins (adc_gpio_init: analogue input, no
 * pulls, digital input buffer off).  Init task, before any read; idempotent.
 */
void drv_ir_init(void);

/*
 * One raw 12-bit sample (0..4095) of ADC channel ch: IR_C_ADC_CH or
 * IR_R_ADC_CH.  Any other channel returns 0.  About 2 us (one conversion).
 * Which level means black is measured on the car (S3-U09), not assumed here.
 */
uint16_t drv_ir_read(uint32_t ch);

/*
 * Take ADC_SANITY_SAMPLES samples of channel ch and judge them (lf_sanity.h).
 * If p_stats is not NULL it receives the statistics.
 */
lf_adc_status_t drv_ir_sanity(uint32_t ch, lf_adc_stats_t *p_stats);

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
 * Called once by the ADC owner before its first sample (subsystem3_init in
 * LF-1).  Calls drv_ir_init() itself if needed.
 */
int32_t hal_adc_sanity_check(void);

#endif /* DRV_IR_H */
