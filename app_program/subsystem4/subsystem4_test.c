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
#include "../test_report.h"

#if APP_TEST

void subsystem4_test(void)
{
    TEST_INIT();

    /* TODO(phase 2, buddy 4): S4-U01 ... (define the IDs in
     * testing/subsystem4/TEST_LOG.md first). */

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
