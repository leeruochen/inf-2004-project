/*
 *  drv_imu.c
 *  Subsystem 4 -- IMU driver.  SDK side: Pico SDK headers only,
 *  never kernel headers or subsystem4.h.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  See drv_imu.h for the interface and ownership.
 *
 *  "GY-511" boards carry one of two ST chips, detected at boot:
 *    LSM303DLHC  accel 0x19 + mag 0x1E; 12-bit accel, left-justified.
 *    LSM303D     accel + mag in one chip at 0x1D (or 0x1E), WHO_AM_I 0x49;
 *                16-bit accel.  Found on the team's car (I2C scan: 0x1D only).
 *  TODO(owner-to-confirm, buddy 4): the code assumes the board's +X axis
 *  points to the front of the car (pitch sign).  Check by tilting the nose up.
 */

#include "drv_imu.h"

#include <stddef.h>

#include "config/pins.h"
#include "config/thresholds.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#if IMU_ODR_HZ != 400U
#error "drv_imu.c: the control registers are set for 400 Hz; update them with IMU_ODR_HZ"
#endif

/* ---------------------------------------------------------------------------
 * Common register map (both chips)
 * ------------------------------------------------------------------------- */

#define REG_WHO_AM_I            (0x0FU)
#define REG_CTRL1               (0x20U)     /* DLHC CTRL_REG1_A, D CTRL1 */
#define REG_OUT_X_L_A           (0x28U)
#define AUTO_INC                (0x80U)     /* sub-address MSB: auto-increment */

/* ---------------------------------------------------------------------------
 * LSM303DLHC accelerometer
 * ------------------------------------------------------------------------- */

#define DLHC_CTRL_REG4_A        (0x23U)
#define DLHC_CTRL1_400HZ_XYZ    (0x77U)     /* ODR 0111 = 400 Hz, X/Y/Z on */
#define DLHC_REG4_BDU_HR        (0x88U)     /* block data update, high resolution */
#if IMU_ACCEL_FS_G == 4U
#define DLHC_REG4_FS            (0x10U)
#define DLHC_MG_PER_LSB         (2)         /* 12-bit */
#elif IMU_ACCEL_FS_G == 8U
#define DLHC_REG4_FS            (0x20U)
#define DLHC_MG_PER_LSB         (4)
#elif IMU_ACCEL_FS_G == 16U
#define DLHC_REG4_FS            (0x30U)
#define DLHC_MG_PER_LSB         (12)
#else
#error "drv_imu.c: unsupported IMU_ACCEL_FS_G (4, 8 or 16)"
#endif

/* ---------------------------------------------------------------------------
 * LSM303D (accel + mag in one chip)
 * ------------------------------------------------------------------------- */

#define LSM303D_ADDR_SA0_HIGH   (0x1DU)
#define LSM303D_ADDR_SA0_LOW    (0x1EU)
#define LSM303D_WHO_AM_I_VALUE  (0x49U)
#define LSM303D_CTRL2           (0x21U)
#define LSM303D_CTRL1_400HZ     (0x8FU)     /* AODR 1000 = 400 Hz, BDU, X/Y/Z on */
#define LSM303D_CTRL2_ABW_50HZ  (0xC0U)     /* anti-alias filter 50 Hz (ABW 11): cuts vibration */
#if IMU_ACCEL_FS_G == 4U
#define LSM303D_CTRL2_FS        (0x08U)     /* AFS 001 */
#define LSM303D_UG_PER_LSB      (122)       /* 0.122 mg/LSB, 16-bit */
#elif IMU_ACCEL_FS_G == 8U
#define LSM303D_CTRL2_FS        (0x18U)     /* AFS 011 */
#define LSM303D_UG_PER_LSB      (244)
#elif IMU_ACCEL_FS_G == 16U
#define LSM303D_CTRL2_FS        (0x20U)     /* AFS 100 */
#define LSM303D_UG_PER_LSB      (732)
#endif

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

static bool     s_ready       = false;
static bool     s_accel_found = false;
static bool     s_mag_found   = false;
static uint32_t s_chip        = IMU_CHIP_NONE;
static uint8_t  s_accel_addr  = 0U;
static uint32_t s_scan_map[4] = { 0U, 0U, 0U, 0U };    /* bit per 7-bit address */

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static bool write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t const buf[2] = { reg, val };

    return i2c_write_timeout_us(IMU_I2C_INST, addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool read_regs(uint8_t addr, uint8_t reg, uint8_t *p_dst, size_t len)
{
    if (i2c_write_timeout_us(IMU_I2C_INST, addr, &reg, 1, true, I2C_TIMEOUT_US) != 1)
    {
        return false;
    }

    return i2c_read_timeout_us(IMU_I2C_INST, addr, p_dst, len, false, I2C_TIMEOUT_US) == (int)len;
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

static void scan_bus(void)
{
    for (uint32_t addr = 0x08U; addr <= 0x77U; addr++)
    {
        if (i2c_probe((uint8_t)addr))
        {
            s_scan_map[addr >> 5U] |= (1UL << (addr & 31U));
        }
    }
}

static bool is_lsm303d(uint8_t addr)
{
    uint8_t who = 0U;

    return imu_scan_found(addr)
           && read_regs(addr, REG_WHO_AM_I, &who, 1U)
           && (who == LSM303D_WHO_AM_I_VALUE);
}

static bool configure(void)
{
    if (s_chip == IMU_CHIP_LSM303DLHC)
    {
        return write_reg(s_accel_addr, REG_CTRL1, DLHC_CTRL1_400HZ_XYZ)
               && write_reg(s_accel_addr, DLHC_CTRL_REG4_A, DLHC_REG4_BDU_HR | DLHC_REG4_FS);
    }

    if (s_chip == IMU_CHIP_LSM303D)
    {
        return write_reg(s_accel_addr, REG_CTRL1, LSM303D_CTRL1_400HZ)
               && write_reg(s_accel_addr, LSM303D_CTRL2, LSM303D_CTRL2_ABW_50HZ | LSM303D_CTRL2_FS);
    }

    return false;
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

    scan_bus();

    if (imu_scan_found(IMU_ACCEL_I2C_ADDR))
    {
        s_chip       = IMU_CHIP_LSM303DLHC;
        s_accel_addr = (uint8_t)IMU_ACCEL_I2C_ADDR;
        s_mag_found  = imu_scan_found(IMU_MAG_I2C_ADDR);
    }
    else if (is_lsm303d((uint8_t)LSM303D_ADDR_SA0_HIGH))
    {
        s_chip       = IMU_CHIP_LSM303D;
        s_accel_addr = (uint8_t)LSM303D_ADDR_SA0_HIGH;
        s_mag_found  = true;    /* magnetometer is in the same chip */
    }
    else if (is_lsm303d((uint8_t)LSM303D_ADDR_SA0_LOW))
    {
        s_chip       = IMU_CHIP_LSM303D;
        s_accel_addr = (uint8_t)LSM303D_ADDR_SA0_LOW;
        s_mag_found  = true;
    }
    else
    {
        return false;
    }

    s_accel_found = true;
    s_ready       = configure();
    return s_ready;
}

bool imu_is_ready(void)
{
    return s_ready;
}

bool imu_reconfigure(void)
{
    return s_ready && configure();
}

bool imu_accel_found(void)
{
    return s_accel_found;
}

bool imu_mag_found(void)
{
    return s_mag_found;
}

uint32_t imu_chip(void)
{
    return s_chip;
}

uint32_t imu_accel_addr(void)
{
    return s_accel_addr;
}

bool imu_scan_found(uint32_t addr)
{
    if (addr > 0x7FU)
    {
        return false;
    }

    return (s_scan_map[addr >> 5U] & (1UL << (addr & 31U))) != 0U;
}

int32_t imu_read_id_reg(uint32_t addr, uint32_t reg)
{
    uint8_t val = 0U;

    if (!read_regs((uint8_t)addr, (uint8_t)reg, &val, 1U))
    {
        return -1;
    }

    return (int32_t)val;
}

bool imu_read_accel(imu_accel_raw_t *p_out)
{
    uint8_t buf[6];

    if (!s_ready || (p_out == NULL)
        || !read_regs(s_accel_addr, (uint8_t)(REG_OUT_X_L_A | AUTO_INC), buf, sizeof buf))
    {
        return false;
    }

    /* Little-endian: low byte first (both chips). */
    p_out->x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    p_out->y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    p_out->z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));

    return true;
}

void imu_raw_to_mg(imu_accel_raw_t const *p_raw,
                   int32_t *p_x_mg, int32_t *p_y_mg, int32_t *p_z_mg)
{
    if (s_chip == IMU_CHIP_LSM303D)
    {
        *p_x_mg = ((int32_t)p_raw->x * LSM303D_UG_PER_LSB) / 1000;
        *p_y_mg = ((int32_t)p_raw->y * LSM303D_UG_PER_LSB) / 1000;
        *p_z_mg = ((int32_t)p_raw->z * LSM303D_UG_PER_LSB) / 1000;
    }
    else
    {
        /* DLHC: 12 bits left-justified; arithmetic shift keeps the sign (GCC). */
        *p_x_mg = ((int32_t)p_raw->x >> 4) * DLHC_MG_PER_LSB;
        *p_y_mg = ((int32_t)p_raw->y >> 4) * DLHC_MG_PER_LSB;
        *p_z_mg = ((int32_t)p_raw->z >> 4) * DLHC_MG_PER_LSB;
    }
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
