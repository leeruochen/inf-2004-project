/**
 * @file pins.h
 * @brief GPIO allocation for the robocar (Pico W on a Cytron ROBO PICO).
 *
 * Week 8 pin map v2 (ARCHITECTURE.md delta D5; supersedes design.md section 2,
 * and the Week 8 document still lists the old pins). TEAM.md section 2 is the
 * human-readable claim table; this header is the compile-time source of truth.
 *
 * Console: USB CDC is the ONLY console. The Raspberry Pi Debug Probe is used
 * for SWD only. UART0 is removed: GP0/GP1 are the right wheel encoder (delta D6).
 *
 * Pure macros: no SDK or kernel includes, so this header may be included from
 * either side of the port layer.
 *
 * Compile-time checks at the end reject two pins sharing a GPIO, any use of the
 * CYW43-owned pins (GP23, GP24, GP25, GP29), an allocated spare pin, an encoder
 * pair the PIO program cannot read, a wrong ADC channel, a servo PWM slice that
 * collides with the motors, and a UART0 console build.
 */

#ifndef PINS_H
#define PINS_H

/* ---------------------------------------------------------------------------
 * Drive motors (onboard driver, PWM slices 4 and 5, 20 kHz). Buddy 2.
 * ROBO PICO driver truth table: both inputs LOW = brake, both inputs HIGH = coast.
 * ------------------------------------------------------------------------- */

/** Left motor input A (M1A), PWM slice 4 A. */
#define MOTOR_L_A_PIN           (8U)
/** Left motor input B (M1B), PWM slice 4 B. */
#define MOTOR_L_B_PIN           (9U)
/** Right motor input A (M2A), PWM slice 5 A. */
#define MOTOR_R_A_PIN           (10U)
/** Right motor input B (M2B), PWM slice 5 B. */
#define MOTOR_R_B_PIN           (11U)

/* ---------------------------------------------------------------------------
 * Wheel encoders: quadrature, PIO0, one state machine per wheel (delta D7).
 * Buddy 2. One copy of the pico-examples quadrature program reads IN PINS, 2,
 * i.e. the A pin and A + 1, so each pair must be adjacent with A on the even,
 * lower pin.
 *
 * "A on the lower pin" is a PROVISIONAL label. The wiring table's black/white
 * A/B labels are not authoritative; the harness colours are green and red.
 * The sign is fixed in firmware by ENC_x_DIR_INVERT.
 *
 * 6-pin motor/encoder connector (per motor):
 *   pin 1     GND
 *   pins 2/3  encoder channels (phase A/B order not specified)
 *   pin 4     3.3 V encoder supply
 *   pins 5/6  motor power, to the ROBO PICO motor screw terminals
 * The encoder is powered from 3V3 only, never VIN or the servo header V pins.
 *
 * Bring-up order: first set MOTOR_x_DIR_INVERT so a "forward" command drives
 * each wheel forward, then set ENC_x_DIR_INVERT so forward motion counts up
 * (hand-spin test, then a forward-drive test).
 * ------------------------------------------------------------------------- */

/** Right wheel encoder phase A (Grove 1). PROVISIONAL label. */
#define ENC_R_A_PIN             (0U)
/** Right wheel encoder phase B (Grove 1). PROVISIONAL label. */
#define ENC_R_B_PIN             (1U)
/** Left wheel encoder phase A (Grove 4). PROVISIONAL label. */
#define ENC_L_A_PIN             (16U)
/** Left wheel encoder phase B (Grove 4). PROVISIONAL label. */
#define ENC_L_B_PIN             (17U)

/** 1 = swap A/B for the left motor so "forward" drives forward. TUNE */
#define MOTOR_L_DIR_INVERT      (0U)
/** 1 = swap A/B for the right motor so "forward" drives forward. TUNE */
#define MOTOR_R_DIR_INVERT      (0U)
/** 1 = negate the left encoder count so forward motion counts up. TUNE
 *  Set by the hand-spin and forward-drive test. */
#define ENC_L_DIR_INVERT        (0U)
/** 1 = negate the right encoder count so forward motion counts up. TUNE
 *  Set by the hand-spin and forward-drive test. */
#define ENC_R_DIR_INVERT        (0U)

/** PIO block used by the encoders (PIO0; PIO1 is reserved for the CYW43). */
#define ENC_PIO_NUM             (0U)
/** PIO0 state machine for the right encoder. */
#define ENC_R_PIO_SM            (0U)
/** PIO0 state machine for the left encoder. */
#define ENC_L_PIO_SM            (1U)

/* ---------------------------------------------------------------------------
 * Line-following IR sensors (ADC). Buddy 3.
 * CENTER + RIGHT: the mirror of design.md's left + centre (delta D11). Buddy 3
 * must REDEFINE the steering sign, not copy it from design.md.
 * TODO(owner-to-confirm, buddy 3): proposal: steer_err > 0 = line to the car's
 * right = steer right.
 * Only VCC, GND and AO are wired on both line sensors; DO is not wired.
 *
 * GP26 is declared ONCE, as an ADC input with pull-up and pull-down disabled,
 * never an output. Grove 6's GP26 lead is the SAME pin as Grove 5's GP26 lead
 * and is not taped off; firmware cannot disconnect a physical lead. Hardware
 * checklist (TEAM.md): tape or cap the Grove 6 GP26 lead.
 * hal_adc_sanity_check() flags a stuck or abnormally noisy GP26 at boot.
 * ------------------------------------------------------------------------- */

/** Centre line sensor analogue output (Grove 7). The GP28 VBAT-sense jumper
 *  must stay OPEN, or ADC2 reads the battery divider. */
#define IR_C_PIN                (28U)
/** ADC channel for IR_C_PIN (GP28 = ADC2). */
#define IR_C_ADC_CH             (2U)
/** Right line sensor analogue output, the car's RIGHT (Grove 5). */
#define IR_R_PIN                (26U)
/** ADC channel for IR_R_PIN (GP26 = ADC0). */
#define IR_R_ADC_CH             (0U)

/* ---------------------------------------------------------------------------
 * Barcode IR sensor (digital GPIO edge interrupt, edge timing). Buddy 3.
 * The barcode board has VCC, GND and DO only. Never call adc_gpio_init() on
 * GP27: it disables the digital input buffer.
 * ------------------------------------------------------------------------- */

/** Barcode IR sensor digital output (Grove 6). */
#define IR_BC_PIN               (27U)

/* ---------------------------------------------------------------------------
 * IMU (GY-511 LSM303DLHC, I2C1 via the Maker/Qwiic port). Buddy 4.
 * GP2/GP3 are shared with Grove 2, so Grove 2 must stay unused.
 * Accel 0x19 and mag 0x1E (thresholds.h): VERIFY by I2C bus scan.
 * INT1, INT2 and SAO are not connected.
 * ------------------------------------------------------------------------- */

/** I2C controller number used by the IMU (I2C1). */
#define IMU_I2C_PORT_NUM        (1U)
/** IMU I2C data line. */
#define IMU_I2C_SDA_PIN         (2U)
/** IMU I2C clock line. */
#define IMU_I2C_SCL_PIN         (3U)

/* ---------------------------------------------------------------------------
 * Ultrasonic sensor (Grove 3). Buddy 5.
 * ECHO must be 3.3 V-safe for GP5 (assumption A8).
 * ------------------------------------------------------------------------- */

/** Ultrasonic trigger output. */
#define US_TRIG_PIN             (4U)
/** Ultrasonic echo input (GPIO IRQ). */
#define US_ECHO_PIN             (5U)

/* ---------------------------------------------------------------------------
 * Scanning servo (servo header, PWM 50 Hz). Buddy 5.
 * GP15 CONFIRMED by the owner: PWM slice 7 channel B. Never use GP12 for the
 * servo. GP14 (slice 7 A) shares the slice: 50 Hz PWM or plain GPIO only.
 * ------------------------------------------------------------------------- */

/** Scanning servo signal. */
#define SERVO_PIN               (15U)
/** PWM slice of SERVO_PIN ((pin >> 1) & 7). */
#define SERVO_PWM_SLICE         (7U)
/** PWM channel of SERVO_PIN (pin & 1; 1 = B). */
#define SERVO_PWM_CHAN          (1U)

/* ---------------------------------------------------------------------------
 * Reserved onboard functions, unused in Phase 1.
 * ------------------------------------------------------------------------- */

/** Onboard NeoPixel (WS2812 via PIO). Reserved for a later visible status. Not implemented. */
#define NEOPIXEL_PIN            (18U)
/** Onboard programmable button 0. Reserved. */
#define BTN0_PIN                (20U)
/** Onboard programmable button 1. Reserved. */
#define BTN1_PIN                (21U)
/** Onboard piezo buzzer. Reserved. */
#define BUZZER_PIN              (22U)

/* ---------------------------------------------------------------------------
 * CYW43-owned pins on the Pico W. NEVER USE from application code.
 * ------------------------------------------------------------------------- */

/** CYW43 WL_REG_ON. */
#define PIN_CYW43_WL_REG_ON         (23U)
/** CYW43 SPI data and host-wake IRQ. */
#define PIN_CYW43_WL_DATA_IRQ       (24U)
/** CYW43 SPI chip select. */
#define PIN_CYW43_WL_CS             (25U)
/** CYW43 SPI clock, shared with VSYS sense on ADC3. */
#define PIN_CYW43_WL_CLK_VSYS_ADC3  (29U)

/* ---------------------------------------------------------------------------
 * Compile-time checks.
 * ------------------------------------------------------------------------- */

/** Bit for GPIO p in a 32-bit pin mask. */
#define PIN_BIT(p)              (1UL << (p))

/** PWM slice of GPIO p (RP2040: (p >> 1) & 7). */
#define PIN_PWM_SLICE(p)        (((p) >> 1U) & 7U)

/** Mask of every allocated or reserved pin (20 pins). */
#define PINS_ALLOCATED_MASK  ( PIN_BIT(ENC_R_A_PIN)      | PIN_BIT(ENC_R_B_PIN)      \
                             | PIN_BIT(IMU_I2C_SDA_PIN)  | PIN_BIT(IMU_I2C_SCL_PIN)  \
                             | PIN_BIT(US_TRIG_PIN)      | PIN_BIT(US_ECHO_PIN)      \
                             | PIN_BIT(ENC_L_A_PIN)      | PIN_BIT(ENC_L_B_PIN)      \
                             | PIN_BIT(MOTOR_L_A_PIN)    | PIN_BIT(MOTOR_L_B_PIN)    \
                             | PIN_BIT(MOTOR_R_A_PIN)    | PIN_BIT(MOTOR_R_B_PIN)    \
                             | PIN_BIT(SERVO_PIN)        | PIN_BIT(NEOPIXEL_PIN)     \
                             | PIN_BIT(BTN0_PIN)         | PIN_BIT(BTN1_PIN)         \
                             | PIN_BIT(BUZZER_PIN)       | PIN_BIT(IR_R_PIN)         \
                             | PIN_BIT(IR_C_PIN)         | PIN_BIT(IR_BC_PIN) )

/** Same terms as PINS_ALLOCATED_MASK, added. Equals the mask only if all bits are distinct. */
#define PINS_ALLOCATED_SUM   ( PIN_BIT(ENC_R_A_PIN)      + PIN_BIT(ENC_R_B_PIN)      \
                             + PIN_BIT(IMU_I2C_SDA_PIN)  + PIN_BIT(IMU_I2C_SCL_PIN)  \
                             + PIN_BIT(US_TRIG_PIN)      + PIN_BIT(US_ECHO_PIN)      \
                             + PIN_BIT(ENC_L_A_PIN)      + PIN_BIT(ENC_L_B_PIN)      \
                             + PIN_BIT(MOTOR_L_A_PIN)    + PIN_BIT(MOTOR_L_B_PIN)    \
                             + PIN_BIT(MOTOR_R_A_PIN)    + PIN_BIT(MOTOR_R_B_PIN)    \
                             + PIN_BIT(SERVO_PIN)        + PIN_BIT(NEOPIXEL_PIN)     \
                             + PIN_BIT(BTN0_PIN)         + PIN_BIT(BTN1_PIN)         \
                             + PIN_BIT(BUZZER_PIN)       + PIN_BIT(IR_R_PIN)         \
                             + PIN_BIT(IR_C_PIN)         + PIN_BIT(IR_BC_PIN) )

/** CYW43-owned pins (GP23, GP24, GP25, GP29). */
#define PINS_CYW43_MASK      ( PIN_BIT(PIN_CYW43_WL_REG_ON) | PIN_BIT(PIN_CYW43_WL_DATA_IRQ) \
                             | PIN_BIT(PIN_CYW43_WL_CS)     | PIN_BIT(PIN_CYW43_WL_CLK_VSYS_ADC3) )

/** Unallocated spares: GP6, GP7 (second leads of Grove 5 / Grove 7, where the
 *  unwired line-sensor DO would land: inputs only), GP12, GP13 (servo headers),
 *  GP14 (servo header, shares PWM slice 7 with the servo: 50 Hz PWM or GPIO only). */
#define PINS_SPARE_MASK      ( PIN_BIT(6U)  | PIN_BIT(7U)  | PIN_BIT(12U) \
                             | PIN_BIT(13U) | PIN_BIT(14U) )

#if PINS_ALLOCATED_SUM != PINS_ALLOCATED_MASK
#error "pins.h: two pins share a GPIO"
#endif

#if (PINS_ALLOCATED_MASK & PINS_CYW43_MASK) != 0UL
#error "pins.h: a pin uses GP23/GP24/GP25/GP29, which the CYW43 owns"
#endif

#if (PINS_ALLOCATED_MASK & PINS_SPARE_MASK) != 0UL
#error "pins.h: a spare pin is allocated; update PINS_SPARE_MASK"
#endif

/* Encoders: one PIO program reads IN PINS, 2 = A and A + 1. */
#if (ENC_R_B_PIN != (ENC_R_A_PIN + 1U)) || (ENC_L_B_PIN != (ENC_L_A_PIN + 1U))
#error "pins.h: each encoder pair must be adjacent with B = A + 1 (PIO IN PINS, 2)"
#endif

#if ((ENC_R_A_PIN & 1U) != 0U) || ((ENC_L_A_PIN & 1U) != 0U)
#error "pins.h: each encoder A pin must be even (A on the lower pin of the pair)"
#endif

#if ENC_R_PIO_SM == ENC_L_PIO_SM
#error "pins.h: each encoder needs its own PIO state machine"
#endif

/* Line sensors: GP26..GP28 = ADC0..ADC2. */
#if (IR_R_PIN - 26U) != IR_R_ADC_CH
#error "pins.h: IR_R_ADC_CH does not match IR_R_PIN"
#endif

#if (IR_C_PIN - 26U) != IR_C_ADC_CH
#error "pins.h: IR_C_ADC_CH does not match IR_C_PIN"
#endif

/* Servo: slice and channel match the pin, and the 50 Hz servo slice never
 * shares a slice with the 20 kHz motors. */
#if PIN_PWM_SLICE(SERVO_PIN) != SERVO_PWM_SLICE
#error "pins.h: SERVO_PWM_SLICE does not match SERVO_PIN"
#endif

#if (SERVO_PIN & 1U) != SERVO_PWM_CHAN
#error "pins.h: SERVO_PWM_CHAN does not match SERVO_PIN"
#endif

#if (SERVO_PWM_SLICE == PIN_PWM_SLICE(MOTOR_L_A_PIN)) || (SERVO_PWM_SLICE == PIN_PWM_SLICE(MOTOR_L_B_PIN)) \
    || (SERVO_PWM_SLICE == PIN_PWM_SLICE(MOTOR_R_A_PIN)) || (SERVO_PWM_SLICE == PIN_PWM_SLICE(MOTOR_R_B_PIN))
#error "pins.h: the servo PWM slice collides with a motor PWM slice"
#endif

/* Console backstop: the CMake guard rejects DBG_BACKEND=UART0 at configure. */
#if defined(DBG_BACKEND_UART0) && ((PINS_ALLOCATED_MASK & (PIN_BIT(0U) | PIN_BIT(1U))) != 0UL)
#error "pins.h: UART0 console needs GP0/GP1, which are the right encoder (ENC_R_A/ENC_R_B)"
#endif

#endif /* PINS_H */
