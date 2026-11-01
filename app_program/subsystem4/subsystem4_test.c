/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4 (IMU processing and safety).
 *  APP_TEST builds only.
 *  Owner: Buddy 4, Student ID: 2500709 (Vincent).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem4/logs/ and record the result in
 *  testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U01, S4-U02, ...
 *
 *  The IMU task owns I2C1 while these tests run, so they never touch the bus:
 *  they read the init-time scan results and the task's cached samples.
 *  Keep the car still and flat on the bench.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "task_imu.h"
#include "task_safety.h"
#include "../test_report.h"

#if APP_TEST

/* Wait for this many 10 ms IMU cycles before checking the samples. */
#define S4_TEST_SETTLE_MS   (100U)

void subsystem4_test(void)
{
    TEST_INIT();

    TEST_CHECK("S4-U01", imu_accel_found(), "accel answers at IMU_ACCEL_I2C_ADDR");
    TEST_CHECK("S4-U02", imu_mag_found(), "mag answers at IMU_MAG_I2C_ADDR");
    TEST_CHECK("S4-U03", imu_is_ready(), "accel configured (imu_is_ready)");

    uint32_t const good_before = imu_get_good_sample_count();

    (void)tk_dly_tsk(S4_TEST_SETTLE_MS);

    uint32_t const good_after = imu_get_good_sample_count();

    TEST_CHECK("S4-U04", good_after > good_before, "IMU task reads new samples");

    int32_t const  z_mg  = imu_get_z_mg();
    int32_t const  abs_z = (z_mg >= 0) ? z_mg : -z_mg;
    uint32_t const mag   = imu_get_accel_mag_mg();
    int32_t const  pitch = imu_get_pitch_half_deg();
    int32_t const  abs_p = (pitch >= 0) ? pitch : -pitch;

    dbg_printf("[S4] z=%ld mg mag=%lu mg pitch=%ld half-deg",
               (long)z_mg, (unsigned long)mag, (long)pitch);

    TEST_CHECK("S4-U05", (abs_z > 800) && (abs_z < 1200), "z-axis ~1 g at rest");
    TEST_CHECK("S4-U06", (mag > 800U) && (mag < 1200U), "magnitude ~1 g at rest");
    TEST_CHECK("S4-U07", abs_p < 20, "pitch within 10 deg when flat");

    TEST_CHECK("S4-U08", imu_is_task_running(), "IMU task running");
    TEST_CHECK("S4-U09", safety_is_task_running(), "Safety task running");

    /* 100 ms at 10 ms: expect about 10 releases; allow scheduling slack. */
    TEST_CHECK("S4-U10", (good_after - good_before) >= 8U, "IMU period ~10 ms");
    TEST_CHECK("S4-U11", imu_get_i2c_error_count() == 0U, "no I2C errors");
    TEST_CHECK("S4-U12", safety_get_wdog_timeout_count() == 0U,
               "no IMU watchdog timeouts");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
