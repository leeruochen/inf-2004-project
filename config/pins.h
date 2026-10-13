/**
 * @file pins.h
 * @brief GPIO allocation for the robocar (Pico W on a Cytron ROBO PICO).
 *
 * Source: design.md section 2 plus the Phase 1 debug-probe decision (UART0 on
 * GP0/GP1, Grove 1 reserved). Pure macros: no SDK or kernel includes, so this
 * header may be included from either side of the port layer.
 *
 * Compile-time checks at the end reject two pins sharing a GPIO and any use of
 * the CYW43-owned pins (GP23, GP24, GP25, GP29).
 */

#ifndef PINS_H
#define PINS_H

/* ---------------------------------------------------------------------------
 * Drive motors (onboard driver, PWM). Buddy 2.
 * ROBO PICO driver truth table: both inputs LOW = brake, both inputs HIGH = coast.
 * ------------------------------------------------------------------------- */

/** Left motor input A (M1A), PWM. */
#define MOTOR_L_A_PIN           (8U)
/** Left motor input B (M1B), PWM. */
#define MOTOR_L_B_PIN           (9U)
/** Right motor input A (M2A), PWM. */
#define MOTOR_R_A_PIN           (10U)
/** Right motor input B (M2B), PWM. */
#define MOTOR_R_B_PIN           (11U)
/** 1 = swap A/B for the left motor so "forward" drives forward. TUNE */
#define MOTOR_L_DIR_INVERT      (0U)
/** 1 = swap A/B for the right motor so "forward" drives forward. TUNE */
#define MOTOR_R_DIR_INVERT      (0U)

/* ---------------------------------------------------------------------------
 * Wheel encoders (GPIO IRQ, shared IO_IRQ_BANK0). Buddy 2.
 * ------------------------------------------------------------------------- */

/** Left wheel encoder output (Grove 5). */
#define ENC_L_PIN               (6U)
/** Right wheel encoder output (Grove 7). */
#define ENC_R_PIN               (7U)

/* ---------------------------------------------------------------------------
 * Line-following IR sensors (ADC, Grove 6). Buddy 3.
 * ------------------------------------------------------------------------- */

/** Left track IR sensor analogue output. */
#define IR_L_PIN                (26U)
/** Centre track IR sensor analogue output. */
#define IR_C_PIN                (27U)
/** ADC channel for IR_L_PIN (GP26 = ADC0). */
#define IR_L_ADC_CH             (0U)
/** ADC channel for IR_C_PIN (GP27 = ADC1). */
#define IR_C_ADC_CH             (1U)

/* ---------------------------------------------------------------------------
 * Barcode IR sensor (digital GPIO IRQ, edge timing). Buddy 3.
 * The VBAT-sense jumper (GP28-ADC2) must stay open (unsoldered).
 * ------------------------------------------------------------------------- */

/** Barcode IR sensor digital output. */
#define IR_BC_PIN               (28U)

/* ---------------------------------------------------------------------------
 * IMU (I2C1 via the Maker/Qwiic port). Buddy 4.
 * GP2/GP3 are shared with Grove 2, so Grove 2 must stay unused.
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
 * Scanning servo (Servo port 1, PWM). Buddy 5.
 * ------------------------------------------------------------------------- */

/** Scanning servo signal. */
#define SERVO_PIN               (12U)

/* ---------------------------------------------------------------------------
 * Debug console: UART0 on Grove 1, reserved for the Raspberry Pi Debug Probe.
 * Wiring: probe TX -> GP1 (Pico RX), probe RX -> GP0 (Pico TX), common GND.
 * 115200 8N1. Boot banner and fault messages use polled UART0.
 * ------------------------------------------------------------------------- */

/** UART0 TX (Pico -> probe RX). */
#define DBG_UART0_TX_PIN        (0U)
/** UART0 RX (probe TX -> Pico). */
#define DBG_UART0_RX_PIN        (1U)
/** UART0 baud rate, 8N1. */
#define DBG_UART0_BAUD_BPS      (115200UL)

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

/** Mask of every allocated or reserved pin (20 pins). */
#define PINS_ALLOCATED_MASK  ( PIN_BIT(DBG_UART0_TX_PIN) | PIN_BIT(DBG_UART0_RX_PIN) \
                             | PIN_BIT(IMU_I2C_SDA_PIN)  | PIN_BIT(IMU_I2C_SCL_PIN)  \
                             | PIN_BIT(US_TRIG_PIN)      | PIN_BIT(US_ECHO_PIN)      \
                             | PIN_BIT(ENC_L_PIN)        | PIN_BIT(ENC_R_PIN)        \
                             | PIN_BIT(MOTOR_L_A_PIN)    | PIN_BIT(MOTOR_L_B_PIN)    \
                             | PIN_BIT(MOTOR_R_A_PIN)    | PIN_BIT(MOTOR_R_B_PIN)    \
                             | PIN_BIT(SERVO_PIN)        | PIN_BIT(NEOPIXEL_PIN)     \
                             | PIN_BIT(BTN0_PIN)         | PIN_BIT(BTN1_PIN)         \
                             | PIN_BIT(BUZZER_PIN)       | PIN_BIT(IR_L_PIN)         \
                             | PIN_BIT(IR_C_PIN)         | PIN_BIT(IR_BC_PIN) )

/** Same terms as PINS_ALLOCATED_MASK, added. Equals the mask only if all bits are distinct. */
#define PINS_ALLOCATED_SUM   ( PIN_BIT(DBG_UART0_TX_PIN) + PIN_BIT(DBG_UART0_RX_PIN) \
                             + PIN_BIT(IMU_I2C_SDA_PIN)  + PIN_BIT(IMU_I2C_SCL_PIN)  \
                             + PIN_BIT(US_TRIG_PIN)      + PIN_BIT(US_ECHO_PIN)      \
                             + PIN_BIT(ENC_L_PIN)        + PIN_BIT(ENC_R_PIN)        \
                             + PIN_BIT(MOTOR_L_A_PIN)    + PIN_BIT(MOTOR_L_B_PIN)    \
                             + PIN_BIT(MOTOR_R_A_PIN)    + PIN_BIT(MOTOR_R_B_PIN)    \
                             + PIN_BIT(SERVO_PIN)        + PIN_BIT(NEOPIXEL_PIN)     \
                             + PIN_BIT(BTN0_PIN)         + PIN_BIT(BTN1_PIN)         \
                             + PIN_BIT(BUZZER_PIN)       + PIN_BIT(IR_L_PIN)         \
                             + PIN_BIT(IR_C_PIN)         + PIN_BIT(IR_BC_PIN) )

/** CYW43-owned pins (GP23, GP24, GP25, GP29). */
#define PINS_CYW43_MASK      ( PIN_BIT(PIN_CYW43_WL_REG_ON) | PIN_BIT(PIN_CYW43_WL_DATA_IRQ) \
                             | PIN_BIT(PIN_CYW43_WL_CS)     | PIN_BIT(PIN_CYW43_WL_CLK_VSYS_ADC3) )

/** Unallocated spares: GP13-GP15 (servo ports 2-4), GP16/GP17 (Grove 4). */
#define PINS_SPARE_MASK      ( PIN_BIT(13U) | PIN_BIT(14U) | PIN_BIT(15U) \
                             | PIN_BIT(16U) | PIN_BIT(17U) )

#if PINS_ALLOCATED_SUM != PINS_ALLOCATED_MASK
#error "pins.h: two pins share a GPIO"
#endif

#if (PINS_ALLOCATED_MASK & PINS_CYW43_MASK) != 0UL
#error "pins.h: a pin uses GP23/GP24/GP25/GP29, which the CYW43 owns"
#endif

#if (PINS_ALLOCATED_MASK & PINS_SPARE_MASK) != 0UL
#error "pins.h: a spare pin is allocated; update PINS_SPARE_MASK"
#endif

#if (IR_L_PIN - 26U) != IR_L_ADC_CH
#error "pins.h: IR_L_ADC_CH does not match IR_L_PIN"
#endif

#if (IR_C_PIN - 26U) != IR_C_ADC_CH
#error "pins.h: IR_C_ADC_CH does not match IR_C_PIN"
#endif

#endif /* PINS_H */
