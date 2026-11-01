/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4 (IMU processing and safety).
 *  APP_TEST builds only.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem4/logs/ and record the result in
 *  testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U01, S4-U02, ...
 */

#include "subsystem4.h"
#include "task_imu.h"
#include "task_safety.h"
#include "drv_imu.h"
#include "../test_report.h"

#if APP_TEST

void subsystem4_test(void)
{
    TEST_INIT();

    /* S4-U01: I2C bus scan finds the accelerometer at 0x19. */
    TEST_CHECK("S4-U01", imu_bus_scan_accel(), "accel I2C scan");

    /* S4-U02: I2C bus scan finds the magnetometer at 0x1E. */
    TEST_CHECK("S4-U02", imu_bus_scan_mag(), "mag I2C scan");

    /* S4-U03: IMU driver initialised successfully. */
    TEST_CHECK("S4-U03", imu_is_ready(), "imu_is_ready");

    /* S4-U04: accelerometer read succeeds. */
    imu_accel_raw_t raw;
    bool const read_ok = imu_read_accel(&raw);

    TEST_CHECK("S4-U04", read_ok, "accel read");

    /* S4-U05: at rest, Z-axis should read approximately 1 g (800-1200 mg
     * at 4 g full-scale). */
    if (read_ok)
    {
        int32_t x_mg;
        int32_t y_mg;
        int32_t z_mg;

        imu_raw_to_mg(&raw, &x_mg, &y_mg, &z_mg);

        int32_t const abs_z = (z_mg >= 0) ? z_mg : -z_mg;

        TEST_CHECK("S4-U05", (abs_z > 800) && (abs_z < 1200),
                   "z-axis ~1g at rest");

        /* S4-U06: magnitude should be approximately 1 g at rest. */
        uint32_t const mag = imu_accel_magnitude_mg(x_mg, y_mg, z_mg);

        TEST_CHECK("S4-U06", (mag > 800U) && (mag < 1200U),
                   "magnitude ~1g at rest");

        /* S4-U07: pitch at rest (car flat) should be near 0. */
        int32_t const pitch = imu_pitch_half_deg(x_mg, z_mg);
        int32_t const abs_pitch = (pitch >= 0) ? pitch : -pitch;

        TEST_CHECK("S4-U07", abs_pitch < 20, "pitch ~0 at rest");
    }

    /* S4-U08: IMU task started. */
    TEST_CHECK("S4-U08", imu_is_task_running(), "IMU task running");

    /* S4-U09: Safety task started. */
    TEST_CHECK("S4-U09", safety_is_task_running(), "Safety task running");

    /* S4-U10: after a few cycles, release count should be > 0. */
    (void)tk_dly_tsk(50);
    TEST_CHECK("S4-U10", imu_get_release_count() > 0U,
               "IMU releases > 0");

    /* S4-U11: no I2C errors after initial cycles. */
    TEST_CHECK("S4-U11", imu_get_i2c_error_count() == 0U,
               "no I2C errors");

    /* S4-U12: subsystem4 public pitch accessor works. */
    int32_t const pub_pitch = subsystem4_get_pitch_half_deg();
    int32_t const abs_pub_pitch = (pub_pitch >= 0) ? pub_pitch : -pub_pitch;

    TEST_CHECK("S4-U12", abs_pub_pitch < 60, "public pitch accessor");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
