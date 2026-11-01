/*
 *  drv_imu.c
 *  Subsystem 4 -- IMU driver.  SDK side: Pico SDK headers only,
 *  never kernel headers or subsystem4.h.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  See drv_imu.h for the interface.
 *
 *  Hardware: GY-511 LSM303DLHC accelerometer on I2C1 (SDA GP2, SCL GP3).
 *  The LSM303DLHC accelerometer registers are at I2C address 0x19.
 *  Data is left-justified 12-bit in a 16-bit register (high-resolution mode).
 */

#include "drv_imu.h"

#include <stddef.h>

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

/* ---------------------------------------------------------------------------
 * LSM303DLHC accelerometer register addresses
 * ------------------------------------------------------------------------- */

#define LSM303_CTRL_REG1_A      (0x20U)
#define LSM303_CTRL_REG4_A      (0x23U)
#define LSM303_STATUS_REG_A     (0x27U)
#define LSM303_OUT_X_L_A        (0x28U)

/* CTRL_REG1_A: ODR and axis enable bits */
#define CTRL_REG1_ODR_400HZ     (0x70U)
#define CTRL_REG1_XYZ_EN        (0x07U)

/* CTRL_REG4_A: full-scale range and high-resolution mode */
#define CTRL_REG4_HR            (0x08U)
#if IMU_ACCEL_FS_G == 2U
#define CTRL_REG4_FS            (0x00U)
#define ACCEL_SENSITIVITY_MG    (1U)     /* 1 mg/LSB at 12-bit */
#elif IMU_ACCEL_FS_G == 4U
#define CTRL_REG4_FS            (0x10U)
#define ACCEL_SENSITIVITY_MG    (2U)     /* 2 mg/LSB at 12-bit */
#elif IMU_ACCEL_FS_G == 8U
#define CTRL_REG4_FS            (0x20U)
#define ACCEL_SENSITIVITY_MG    (4U)     /* 4 mg/LSB at 12-bit */
#elif IMU_ACCEL_FS_G == 16U
#define CTRL_REG4_FS            (0x30U)
#define ACCEL_SENSITIVITY_MG    (12U)    /* 12 mg/LSB at 12-bit */
#else
#error "drv_imu.c: unsupported IMU_ACCEL_FS_G"
#endif

/* I2C timeout in microseconds (10 ms: generous for a single register read). */
#define I2C_TIMEOUT_US          (10000)

/* I2C bus speed: 400 kHz fast mode. */
#define I2C_SPEED_HZ            (400U * 1000U)

/* Multi-byte read: set MSB of sub-address for auto-increment. */
#define AUTO_INC                (0x80U)

/* ---------------------------------------------------------------------------
 * I2C1 instance from the port number
 * ------------------------------------------------------------------------- */

#if IMU_I2C_PORT_NUM == 0U
#define IMU_I2C_INST            i2c0
#elif IMU_I2C_PORT_NUM == 1U
#define IMU_I2C_INST            i2c1
#else
#error "drv_imu.c: unsupported IMU_I2C_PORT_NUM"
#endif

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */

static bool s_ready = false;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static bool write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int const ret = i2c_write_timeout_us(IMU_I2C_INST, addr, buf, 2,
                                         false, I2C_TIMEOUT_US);
    return (ret == 2);
}

static bool i2c_probe(uint8_t addr)
{
    uint8_t dummy = 0U;
    int const ret = i2c_read_timeout_us(IMU_I2C_INST, addr, &dummy, 1,
                                        false, I2C_TIMEOUT_US);
    return (ret >= 0);
}

/* ---------------------------------------------------------------------------
 * Integer square root (Babylonian / Newton)
 * ------------------------------------------------------------------------- */

static uint32_t isqrt32(uint32_t n)
{
    if (n == 0U)
    {
        return 0U;
    }

    uint32_t x = n;
    uint32_t y = (x + 1U) / 2U;

    while (y < x)
    {
        x = y;
        y = (x + n / x) / 2U;
    }

    return x;
}

/* ---------------------------------------------------------------------------
 * Integer atan2 approximation returning degrees x 2 (Q1 half-degrees).
 * Uses the rational minimax atan(r) ~ r * (45 + 17.7 * (|r| - 1)) / ...
 * simplified to an integer-friendly piecewise-linear approach.
 * Returns [-180, +180] in half-degree units (i.e. [-90.0, +90.0] degrees).
 * ------------------------------------------------------------------------- */

static int32_t atan2_half_deg(int32_t y, int32_t x)
{
    if ((x == 0) && (y == 0))
    {
        return 0;
    }

    int32_t abs_x = (x >= 0) ? x : -x;
    int32_t abs_y = (y >= 0) ? y : -y;

    /* atan(y/x) in [0, 45 deg] using the identity:
     * atan(r) ~ r * 45 degrees (small angle, scaled). */
    int32_t angle_half;

    if (abs_x >= abs_y)
    {
        /* |angle| <= 45 deg; atan(y/x) ~ (y/x) * 90 half-deg */
        angle_half = (int32_t)((abs_y * 90L) / abs_x);
    }
    else
    {
        /* |angle| > 45 deg; atan(y/x) = 90 - atan(x/y) */
        angle_half = 180 - (int32_t)((abs_x * 90L) / abs_y);
    }

    /* Quadrant adjustment (result in half-degrees): pitch is atan2(x, z)
     * so we treat the first argument as y, second as x. */
    if (x < 0)
    {
        angle_half = 360 - angle_half;
    }

    if (y < 0)
    {
        angle_half = -angle_half;
    }

    /* Wrap to [-180, +180] half-degrees = [-90, +90] degrees */
    if (angle_half > 180)
    {
        angle_half -= 360;
    }
    else if (angle_half < -180)
    {
        angle_half += 360;
    }

    return angle_half;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool imu_bus_scan_accel(void)
{
    return i2c_probe((uint8_t)IMU_ACCEL_I2C_ADDR);
}

bool imu_bus_scan_mag(void)
{
    return i2c_probe((uint8_t)IMU_MAG_I2C_ADDR);
}

bool imu_init(void)
{
    if (s_ready)
    {
        return true;
    }

    /* Configure I2C1 pins and bus speed. */
    i2c_init(IMU_I2C_INST, I2C_SPEED_HZ);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    /* Verify the accelerometer is on the bus. */
    if (!imu_bus_scan_accel())
    {
        return false;
    }

    /* CTRL_REG1_A: ODR 400 Hz, all axes enabled. */
    if (!write_reg((uint8_t)IMU_ACCEL_I2C_ADDR, LSM303_CTRL_REG1_A,
                   CTRL_REG1_ODR_400HZ | CTRL_REG1_XYZ_EN))
    {
        return false;
    }

    /* CTRL_REG4_A: full-scale range, high-resolution mode, BDU off. */
    if (!write_reg((uint8_t)IMU_ACCEL_I2C_ADDR, LSM303_CTRL_REG4_A,
                   CTRL_REG4_FS | CTRL_REG4_HR))
    {
        return false;
    }

    s_ready = true;
    return true;
}

bool imu_is_ready(void)
{
    return s_ready;
}

bool imu_read_accel(imu_accel_raw_t *p_out)
{
    if (!s_ready || (p_out == NULL))
    {
        return false;
    }

    /* Read 6 bytes starting at OUT_X_L_A with auto-increment. */
    uint8_t reg = (uint8_t)(LSM303_OUT_X_L_A | AUTO_INC);
    int ret = i2c_write_timeout_us(IMU_I2C_INST, (uint8_t)IMU_ACCEL_I2C_ADDR,
                                   &reg, 1, true, I2C_TIMEOUT_US);
    if (ret != 1)
    {
        return false;
    }

    uint8_t buf[6];

    ret = i2c_read_timeout_us(IMU_I2C_INST, (uint8_t)IMU_ACCEL_I2C_ADDR,
                              buf, 6, false, I2C_TIMEOUT_US);
    if (ret != 6)
    {
        return false;
    }

    /* LSM303DLHC output is little-endian, left-justified 12-bit in 16-bit.
     * Shift right by 4 to get the 12-bit signed value. */
    p_out->x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    p_out->y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    p_out->z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));

    return true;
}

void imu_raw_to_mg(imu_accel_raw_t const *p_raw,
                   int32_t *p_x_mg, int32_t *p_y_mg, int32_t *p_z_mg)
{
    /* The raw value is left-justified 12-bit in a 16-bit register.
     * Shift right by 4 to get the signed 12-bit count, then multiply
     * by the sensitivity. */
    *p_x_mg = ((int32_t)p_raw->x >> 4) * (int32_t)ACCEL_SENSITIVITY_MG;
    *p_y_mg = ((int32_t)p_raw->y >> 4) * (int32_t)ACCEL_SENSITIVITY_MG;
    *p_z_mg = ((int32_t)p_raw->z >> 4) * (int32_t)ACCEL_SENSITIVITY_MG;
}

int32_t imu_pitch_half_deg(int32_t x_mg, int32_t z_mg)
{
    return atan2_half_deg(x_mg, z_mg);
}

uint32_t imu_accel_magnitude_mg(int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    uint32_t const sq = (uint32_t)((int64_t)x_mg * x_mg
                                 + (int64_t)y_mg * y_mg
                                 + (int64_t)z_mg * z_mg);

    return isqrt32(sq);
}
