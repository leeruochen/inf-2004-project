/*
 *  drv_ir.c
 *  Subsystem 3 -- line sensor ADC driver (see drv_ir.h).  SDK side: Pico SDK
 *  headers plus pure project headers only, never kernel headers or
 *  subsystem3.h (rule R1).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  adc_read() starts one conversion and spins until it is done (about 2 us
 *  at the 48 MHz ADC clock): far below one kernel tick, and not one of the
 *  blocking waits rule R2 forbids.  No interrupt, no DMA, no FIFO.
 */

#include "drv_ir.h"

#include <stdbool.h>
#include <stddef.h>

#include "config/pins.h"            /* IR_C_PIN, IR_C_ADC_CH, IR_R_PIN, IR_R_ADC_CH */
#include "config/thresholds.h"      /* ADC_SANITY_SAMPLES */
#include "hardware/adc.h"

static bool s_ready = false;

void drv_ir_init(void)
{
    if (!s_ready)
    {
        adc_init();
        /* Analogue inputs: pulls off, digital input buffer off. */
        adc_gpio_init(IR_C_PIN);
        adc_gpio_init(IR_R_PIN);
        s_ready = true;
    }
}

uint16_t drv_ir_read(uint32_t ch)
{
    uint16_t raw = 0U;

    if (s_ready && ((ch == IR_C_ADC_CH) || (ch == IR_R_ADC_CH)))
    {
        adc_select_input(ch);
        raw = adc_read();
    }

    return raw;
}

lf_adc_status_t drv_ir_sanity(uint32_t ch, lf_adc_stats_t *p_stats)
{
    lf_adc_stats_t s;
    uint32_t       i;

    drv_ir_init();
    lf_adc_stats_reset(&s);
    for (i = 0U; i < ADC_SANITY_SAMPLES; i++)
    {
        lf_adc_stats_add(&s, drv_ir_read(ch));
    }
    if (p_stats != NULL)
    {
        *p_stats = s;
    }

    return lf_adc_check(&s);
}

int32_t hal_adc_sanity_check(void)
{
    return (drv_ir_sanity(IR_R_ADC_CH, NULL) == LF_ADC_OK) ? 0 : 1;
}
