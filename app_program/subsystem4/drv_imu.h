/*
 *  drv_imu.h
 *  Subsystem 4 -- IMU driver interface (kernel-free).
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  <stdbool.h> and <stdint.h> only, so both drv_imu.c (SDK side) and the
 *  IMU Processing / Safety tasks (kernel side) may include it.
 *
 *  Hardware: GY-511 LSM303DLHC, accelerometer at 0x19 and magnetometer at
 *  0x1E on I2C1 (SDA GP2, SCL GP3).  Accelerometer-only pitch in Phase 1
 *  (IMU_HAS_GYRO = 0).  Ownership: the IMU Processing task is the only
 *  caller after imu_init().
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdbool.h>
#include <stdint.h>

/** Raw accelerometer reading (signed 16-bit, left-justified in the register). */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_raw_t;

/*
 * Initialise I2C1 at 400 kHz, configure the LSM303DLHC accelerometer
 * (ODR, full scale, high-resolution mode).  Returns true on success,
 * false if the I2C bus scan fails to find the expected address.
 */
bool imu_init(void);

/** True once imu_init() has run successfully. */
bool imu_is_ready(void);

/*
 * Read the latest accelerometer sample.  Returns true on success (data
 * valid), false on an I2C error.  Non-blocking: reads the most recent
 * output registers.
 */
bool imu_read_accel(imu_accel_raw_t *p_out);

/*
 * Convert a raw accelerometer reading to milli-g (mg) per axis, based on
 * the configured full-scale range (IMU_ACCEL_FS_G).
 */
void imu_raw_to_mg(imu_accel_raw_t const *p_raw,
                   int32_t *p_x_mg, int32_t *p_y_mg, int32_t *p_z_mg);

/*
 * Compute pitch angle from accelerometer milli-g values.
 * Returns pitch in units of 0.5 degrees (Q1 fixed-point), positive = nose up.
 * Uses atan2 approximation; no floating point.
 */
int32_t imu_pitch_half_deg(int32_t x_mg, int32_t z_mg);

/*
 * Compute the magnitude of the acceleration vector in milli-g.
 * Uses an integer square-root approximation.
 */
uint32_t imu_accel_magnitude_mg(int32_t x_mg, int32_t y_mg, int32_t z_mg);

/*
 * Scan I2C1 for the accelerometer address.  Returns true if the device
 * ACKs.  Safe to call before imu_init() for bus diagnostics.
 */
bool imu_bus_scan_accel(void);

/*
 * Scan I2C1 for the magnetometer address.  Returns true if the device
 * ACKs.  Safe to call before imu_init() for bus diagnostics.
 */
bool imu_bus_scan_mag(void);

#endif /* DRV_IMU_H */
