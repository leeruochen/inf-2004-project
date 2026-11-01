/*
 *  drv_imu.c
 *  Subsystem 4 -- IMU driver.  SDK side: Pico SDK headers only,
 *  never kernel headers or subsystem4.h.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  See drv_imu.h for the interface and ownership.
 *
 *  Hardware: GY-511 LSM303DLHC accelerometer on I2C1 (SDA GP2, SCL GP3).
 *  Output is a 12-bit value left-justified in 16 bits (high-resolution mode).
 *  TODO(owner-to-confirm, buddy 4): the code assumes the board's +X axis
 *  points to the front of the car (pitch sign).  Check by tilting the nose up.
 */

#include "drv_imu.h"

#include <stddef.h>

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

/* ---------------------------------------------------------------------------
 * LSM303DLHC accelerometer registers
 * ------------------------------------------------------------------------- */

#define LSM303_CTRL_REG1_A      (0x20U)
#define LSM303_CTRL_REG4_A      (0x23U)
#define LSM303_OUT_X_L_A        (0x28U)

/* CTRL_REG1_A: ODR 400 Hz (ODR[3:0] = 0111), X/Y/Z enabled. */
#define CTRL_REG1_ODR_400HZ     (0x70U)
#define CTRL_REG1_XYZ_EN        (0x07U)

#if IMU_ODR_HZ != 400U
#error "drv_imu.c: CTRL_REG1_A is set for 400 Hz; update it with IMU_ODR_HZ"
#endif

/* CTRL_REG4_A: block data update (no torn high/low bytes), high resolution. */
#define CTRL_REG4_BDU           (0x80U)
#define CTRL_REG4_HR            (0x08U)
#if IMU_ACCEL_FS_G == 4U
#define CTRL_REG4_FS            (0x10U)
#define ACCEL_SENSITIVITY_MG    (2)      /* 2 mg/LSB at 12-bit */
#elif IMU_ACCEL_FS_G == 8U
#define CTRL_REG4_FS            (0x20U)
#define ACCEL_SENSITIVITY_MG    (4)      /* 4 mg/LSB at 12-bit */
#elif IMU_ACCEL_FS_G == 16U
#define CTRL_REG4_FS            (0x30U)
#define ACCEL_SENSITIVITY_MG    (12)     /* 12 mg/LSB at 12-bit */
#else
#error "drv_imu.c: unsupported IMU_ACCEL_FS_G (4, 8 or 16)"
#endif

/* Sub-address MSB set = auto-increment for multi-byte reads. */
#define AUTO_INC                (0x80U)

/* One 7-byte transfer at 400 kHz takes about 200 us; 2 ms is a stuck bus. */
#define I2C_TIMEOUT_US          (2000U)

#define I2C_SPEED_HZ            (400U * 1000U)

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

static bool s_ready       = false;
static bool s_accel_found = false;
static bool s_mag_found   = false;

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static bool write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t const buf[2] = { reg, val };

    return i2c_write_timeout_us(IMU_I2C_INST, addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool i2c_probe(uint8_t addr)
{
    uint8_t dummy = 0U;

    return i2c_read_timeout_us(IMU_I2C_INST, addr, &dummy, 1, false, I2C_TIMEOUT_US) >= 0;
}

static uint32_t isqrt32(uint32_t n)
{
    if (n == 0U)
    {
        return 0U;
    }

    uint32_t x = n;
    uint32_t y = (x / 2U) + 1U;

    while (y < x)
    {
        x = y;
        y = (x + (n / x)) / 2U;
    }

    return x;
}

/*
 * atan(num / den) in half-degrees for 0 <= num <= den, den > 0.
 * atan(r) ~ 45 r + 15.64 r (1 - r) degrees (max error 0.22 deg).
 */
static int32_t atan_ratio_half_deg(int64_t num, int64_t den)
{
    int64_t const d2 = 100 * den * den;

    /* (90 r + 31.28 r (1 - r)) x d2, rounded to the nearest half-degree. */
    return (int32_t)(((9000 * num * den) + (3128 * num * (den - num)) + (d2 / 2)) / d2);
}

/* atan2(y, x) in half-degrees, range +-360 (+-180 deg). */
static int32_t atan2_half_deg(int32_t y, int32_t x)
{
    int64_t const ax = (x >= 0) ? (int64_t)x : -(int64_t)x;
    int64_t const ay = (y >= 0) ? (int64_t)y : -(int64_t)y;
    int32_t       a;

    if ((ax == 0) && (ay == 0))
    {
        return 0;
    }

    if (ax >= ay)
    {
        a = atan_ratio_half_deg(ay, ax);
    }
    else
    {
        a = 180 - atan_ratio_half_deg(ax, ay);
    }

    if (x < 0)
    {
        a = 360 - a;
    }

    return (y < 0) ? -a : a;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool imu_init(void)
{
    if (s_ready)
    {
        return true;
    }

    i2c_init(IMU_I2C_INST, I2C_SPEED_HZ);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    s_accel_found = i2c_probe((uint8_t)IMU_ACCEL_I2C_ADDR);
    s_mag_found   = i2c_probe((uint8_t)IMU_MAG_I2C_ADDR);

    if (!s_accel_found)
    {
        return false;
    }

    if (!write_reg((uint8_t)IMU_ACCEL_I2C_ADDR, LSM303_CTRL_REG1_A,
                   CTRL_REG1_ODR_400HZ | CTRL_REG1_XYZ_EN))
    {
        return false;
    }

    if (!write_reg((uint8_t)IMU_ACCEL_I2C_ADDR, LSM303_CTRL_REG4_A,
                   CTRL_REG4_BDU | CTRL_REG4_FS | CTRL_REG4_HR))
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

bool imu_accel_found(void)
{
    return s_accel_found;
}

bool imu_mag_found(void)
{
    return s_mag_found;
}

bool imu_read_accel(imu_accel_raw_t *p_out)
{
    if (!s_ready || (p_out == NULL))
    {
        return false;
    }

    uint8_t const reg = (uint8_t)(LSM303_OUT_X_L_A | AUTO_INC);
    uint8_t       buf[6];

    if (i2c_write_timeout_us(IMU_I2C_INST, (uint8_t)IMU_ACCEL_I2C_ADDR,
                             &reg, 1, true, I2C_TIMEOUT_US) != 1)
    {
        return false;
    }

    if (i2c_read_timeout_us(IMU_I2C_INST, (uint8_t)IMU_ACCEL_I2C_ADDR,
                            buf, 6, false, I2C_TIMEOUT_US) != 6)
    {
        return false;
    }

    /* Little-endian: low byte first. */
    p_out->x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    p_out->y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    p_out->z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));

    return true;
}

void imu_raw_to_mg(imu_accel_raw_t const *p_raw,
                   int32_t *p_x_mg, int32_t *p_y_mg, int32_t *p_z_mg)
{
    /* Arithmetic shift keeps the sign (GCC); 16 = 4-bit left justification. */
    *p_x_mg = ((int32_t)p_raw->x >> 4) * ACCEL_SENSITIVITY_MG;
    *p_y_mg = ((int32_t)p_raw->y >> 4) * ACCEL_SENSITIVITY_MG;
    *p_z_mg = ((int32_t)p_raw->z >> 4) * ACCEL_SENSITIVITY_MG;
}

int32_t imu_pitch_half_deg(int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    uint32_t const yz = isqrt32((uint32_t)(((int64_t)y_mg * y_mg) + ((int64_t)z_mg * z_mg)));

    return atan2_half_deg(x_mg, (int32_t)yz);
}

uint32_t imu_accel_magnitude_mg(int32_t x_mg, int32_t y_mg, int32_t z_mg)
{
    /* Worst case (16 g scale) 3 x 24564^2 = 1.81e9 mg^2: fits uint32_t. */
    uint32_t const sq = (uint32_t)(((int64_t)x_mg * x_mg)
                                 + ((int64_t)y_mg * y_mg)
                                 + ((int64_t)z_mg * z_mg));

    return isqrt32(sq);
}
