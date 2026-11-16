/*
 *  subsystem3_test.c
 *  On-target tests for subsystem 3 (line following and barcode decode).
 *  APP_TEST builds only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem3/logs/ and record the result in
 *  testing/subsystem3/TEST_LOG.md.  Test IDs: S3-U01, S3-U02, ...
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "lf_hw_test.h"
#include "lf_run_test.h"
#include "lf_step_test.h"

/* S3-U09 (LF-1 measurement, closed 3 Nov 2026 by its WAIVED row) reads the
 * ADC from the test task.  The Line-Following task now owns the ADC, so the
 * two must not run together: S3-U09 is built but not run.  To repeat it,
 * set this to 1 and comment out lf_task_start() in subsystem3_start(). */
#define S3_RUN_LF1_TEST     (0)

/* S3-U16, the steering step diagnostic on the open floor, in place of the
 * track test: configure with -DS3_STEP_DIAG=ON (sources.cmake).  0 = the track. */
#ifndef S3_STEP_DIAG
#define S3_STEP_DIAG        (0)
#endif

void subsystem3_test(void)
{
    TEST_INIT();

#if S3_RUN_LF1_TEST
    /* S3-U09: LF-1 line sensor measurement (operator-guided). */
    lf_hw_test(&test_pass_, &test_fail_);
#endif

#if S3_STEP_DIAG
    /* S3-U16: what the wheels do after a strong steering command (operator-guided). */
    lf_step_test(&test_pass_, &test_fail_);
#else
    /* S3-U12: LF-3.2 Line-Following task on the track, motors live (operator-guided). */
    lf_run_test(&test_pass_, &test_fail_);
#endif

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
