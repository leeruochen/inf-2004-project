/*
 *  drv_encoder.c
 *  Subsystem 2 -- wheel encoder driver on PIO0.  SDK side: Pico SDK headers
 *  only, never kernel headers or subsystem2.h.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Architecture (ARCHITECTURE.md D7, config/pins.h):
 *    - One copy of quadrature_encoder.pio (pico-examples tag sdk-1.5.1,
 *      24 instructions) loaded at offset 0 on PIO0.
 *    - SM0 decodes the right wheel (GP0/GP1); SM1 decodes the left wheel
 *      (GP16/GP17).
 *    - Non-blocking reader (rule R2): drains stale words from the 4-deep RX
 *      FIFO with pio_sm_get(), polls a bounded number of iterations for the
 *      fresh count pushed by the state machine (~10 clk_sys cycles at clkdiv
 *      1), and falls back to the last sample if none arrives.
 */

#include "drv_encoder.h"

#include <stdbool.h>

#include "config/pins.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

#if ENC_PIO_NUM != 0U
#error "drv_encoder.c: ENC_PIO_NUM must be 0 (PIO0)"
#endif
#if ENC_R_B_PIN != (ENC_R_A_PIN + 1U)
#error "drv_encoder.c: right encoder pins must be consecutive (B = A + 1)"
#endif
#if ENC_L_B_PIN != (ENC_L_A_PIN + 1U)
#error "drv_encoder.c: left encoder pins must be consecutive (B = A + 1)"
#endif
#if ENC_R_PIO_SM == ENC_L_PIO_SM
#error "drv_encoder.c: left and right encoders must use distinct PIO SMs"
#endif

/** Bounded poll iterations for a fresh push after draining the 4-word FIFO. */
#define ENC_FRESH_POLL_MAX      (32U)

typedef struct
{
    uint32_t sm;
    uint32_t pin_a;
    uint32_t invert;
} enc_wheel_cfg_t;

static enc_wheel_cfg_t const k_enc_wheels[ENC_WHEEL_COUNT] =
{
    [ENC_WHEEL_LEFT] =
    {
        ENC_L_PIO_SM,
        ENC_L_A_PIN,
        ENC_L_DIR_INVERT
    },
    [ENC_WHEEL_RIGHT] =
    {
        ENC_R_PIO_SM,
        ENC_R_A_PIN,
        ENC_R_DIR_INVERT
    }
};

static volatile uint32_t s_ready = 0U;
static int32_t           s_last_raw[ENC_WHEEL_COUNT] = { 0, 0 };

static void enc_init_sm(enc_wheel_cfg_t const *p_cfg)
{
    PIO const     pio = pio0;
    uint32_t const sm = p_cfg->sm;
    uint32_t const pin_a = p_cfg->pin_a;
    pio_sm_config c;

    pio_sm_claim(pio, sm);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_a, 2U, false);
    gpio_pull_up(pin_a);
    gpio_pull_up(pin_a + 1U);

    c = quadrature_encoder_program_get_default_config(0U);
    sm_config_set_in_pins(&c, pin_a);
    sm_config_set_jmp_pin(&c, pin_a);
    sm_config_set_in_shift(&c, false, false, 32U);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    sm_config_set_clkdiv(&c, 1.0f);

    pio_sm_init(pio, sm, 0U, &c);

    /* Seed OSR with the current 2-bit pin state and zero Y so a non-00 idle
     * state at boot does not cause a synthetic +1/-1 step on the first poll. */
    pio_sm_exec(pio, sm, pio_encode_in(pio_pins, 2U));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_osr, pio_isr));
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0U));
    pio_sm_clear_fifos(pio, sm);

    pio_sm_set_enabled(pio, sm, true);
}

void enc_init(void)
{
    if (s_ready != 0U)
    {
        return;
    }
    if (!pio_can_add_program_at_offset(pio0, &quadrature_encoder_program, 0U))
    {
        return;
    }
    pio_add_program_at_offset(pio0, &quadrature_encoder_program, 0U);

    for (uint32_t w = 0U; w < ENC_WHEEL_COUNT; w++)
    {
        s_last_raw[w] = 0;
        enc_init_sm(&k_enc_wheels[w]);
    }
    s_ready = 1U;
}

uint32_t enc_is_ready(void)
{
    return s_ready;
}

int32_t enc_get_count(uint32_t wheel)
{
    uint32_t sm;
    int32_t  raw;
    bool     got = false;

    if (wheel >= ENC_WHEEL_COUNT)
    {
        return 0;
    }
    if (s_ready == 0U)
    {
        enc_init();
        if (s_ready == 0U)
        {
            return 0;
        }
    }

    sm = k_enc_wheels[wheel].sm;
    raw = s_last_raw[wheel];

    /* Drain any stale values sitting in the 4-word RX FIFO. */
    while (!pio_sm_is_rx_fifo_empty(pio0, sm))
    {
        raw = (int32_t)pio_sm_get(pio0, sm);
        got = true;
    }

    /* Wait a bounded number of iterations (~10 clk_sys cycles) for the state
     * machine's next PUSH noblock so we return the live Y register. */
    for (uint32_t i = 0U; i < ENC_FRESH_POLL_MAX; i++)
    {
        if (!pio_sm_is_rx_fifo_empty(pio0, sm))
        {
            raw = (int32_t)pio_sm_get(pio0, sm);
            got = true;
            break;
        }
    }

    if (got)
    {
        s_last_raw[wheel] = raw;
    }

    if (k_enc_wheels[wheel].invert != 0U)
    {
        return (int32_t)(0U - (uint32_t)raw);
    }
    return raw;
}

