/*
 *  drv_encoder.h
 *  Subsystem 2 -- wheel encoder driver interface (kernel-free).
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  <stdint.h> only, so both the driver (SDK side) and the Motion Control task
 *  (kernel side) may include it.  Week 8: quadrature on PIO0, one state
 *  machine per wheel (ARCHITECTURE.md D7).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

/** Wheel index for enc_get_count(): left wheel (GP16/GP17, PIO0 SM1). */
#define ENC_WHEEL_LEFT          (0U)
/** Wheel index for enc_get_count(): right wheel (GP0/GP1, PIO0 SM0). */
#define ENC_WHEEL_RIGHT         (1U)
/** Number of encoder wheels. */
#define ENC_WHEEL_COUNT         (2U)

/*
 * Load quadrature_encoder.pio at offset 0 on PIO0 and start SM0 (right,
 * GP0/GP1) and SM1 (left, GP16/GP17).  Idempotent: a second call does nothing.
 */
void enc_init(void);

/** Non-zero once enc_init() has succeeded at offset 0 on PIO0. */
uint32_t enc_is_ready(void);

/*
 * Current encoder count of one wheel.
 *   wheel  ENC_WHEEL_LEFT or ENC_WHEEL_RIGHT
 * Returns the free-running signed X4 count (ENC_x_DIR_INVERT applied).  The
 * reader never clears it; Motion Control takes wrap-safe two's-complement
 * deltas.  Non-blocking: never pio_sm_get_blocking() (rule R2).  Callable
 * from the Motion task, the only writer of the published count words.
 * Implementable by PIO0 (D7) or by a GPIO-interrupt decoder without
 * changing callers.
 */
int32_t enc_get_count(uint32_t wheel);

#endif /* DRV_ENCODER_H */
