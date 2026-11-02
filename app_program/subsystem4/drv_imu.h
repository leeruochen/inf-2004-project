/*
 *  drv_imu.h
 *  Subsystem 4 -- IMU driver interface (kernel-free).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  <stdbool.h> and <stdint.h> only, so both drv_imu.c (SDK side) and the
 *  IMU Processing / Safety tasks (kernel side) may include it.
 *
 *  Hardware: GY-511 on I2C1 (SDA GP2, SCL GP3) with either an LSM303DLHC
 *  (accel 0x19, mag 0x1E) or an LSM303D (accel + mag at 0x1D), detected at
 *  boot.  Accelerometer-only pitch in Phase 1 (IMU_HAS_GYRO = 0).
 *
 *  Ownership: I2C1 has one owner.  imu_init() runs in subsystem4_init()
 *  before any task starts; after that only the IMU Processing task calls
 *  imu_read_accel().  Other tasks (tests included) use the cached results
 *  (imu_accel_found(), imu_mag_found(), the task_imu.h accessors).
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdbool.h>
#include <stdint.h>

/** Raw accelerometer reading (12-bit value left-justified in 16 bits). */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_raw_t;

/*
 * Initialise I2C1 at 400 kHz, probe the accelerometer and magnetometer
 * addresses, and configure the accelerometer (400 Hz, IMU_ACCEL_FS_G,
 * high resolution, block data update).  Returns true if the accelerometer
 * answered and was configured.
 */
bool imu_init(void);

/** True once imu_init() has configured the accelerometer. */
bool imu_is_ready(void);

/*
 * Write the configuration again to the chip found by imu_init().  A sensor
 * that lost power (cable unplugged) wakes up powered down and must be
 * configured before its readings mean anything.  IMU Processing task only.
 */
bool imu_reconfigure(void);

/** Bus-scan results recorded by imu_init() (no I2C traffic). */
bool imu_accel_found(void);
bool imu_mag_found(void);

/*
 * True if any device answered at addr (0x08..0x77) in the full I2C1 scan
 * done by imu_init().  For wiring checks: tells a missing or miswired IMU
 * from a different chip at another address.
 */
bool imu_scan_found(uint32_t addr);

/** Chip found by imu_init(). */
#define IMU_CHIP_NONE           (0U)
#define IMU_CHIP_LSM303DLHC     (1U)    /* accel 0x19, mag 0x1E */
#define IMU_CHIP_LSM303D        (2U)    /* accel + mag at 0x1D or 0x1E */

uint32_t imu_chip(void);

/** I2C address the accelerometer is read from (0 if none). */
uint32_t imu_accel_addr(void);

/*
 * Read one register for identification logs: the value 0..255, or -1 on an
 * I2C error.  IMU Processing task only (it owns I2C1 after init).
 */
int32_t imu_read_id_reg(uint32_t addr, uint32_t reg);

/*
 * Read the latest accelerometer sample.  Returns false on an I2C error or
 * before imu_init().  IMU Processing task only.
 */
bool imu_read_accel(imu_accel_raw_t *p_out);

/* Convert a raw reading to milli-g per axis (IMU_ACCEL_FS_G sensitivity). */
void imu_raw_to_mg(imu_accel_raw_t const *p_raw,
                   int32_t *p_x_mg, int32_t *p_y_mg, int32_t *p_z_mg);

/*
 * Pitch from the accelerometer, in half-degrees (Q1), positive = nose up:
 * atan2(x, sqrt(y^2 + z^2)), so roll does not change it.  Range +-180
 * (+-90 deg).  Integer only, max error about 0.25 deg.
 */
int32_t imu_pitch_half_deg(int32_t x_mg, int32_t y_mg, int32_t z_mg);

/* Magnitude of the acceleration vector in milli-g. */
uint32_t imu_accel_magnitude_mg(int32_t x_mg, int32_t y_mg, int32_t z_mg);

#endif /* DRV_IMU_H */
