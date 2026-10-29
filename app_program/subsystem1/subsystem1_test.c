/*
 *  subsystem1_test.c
 *  On-target tests for subsystem 1 (telemetry).  APP_TEST builds only.
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem1/logs/ and record the result in
 *  testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

void subsystem1_test(void)
{
    TEST_INIT();

    /* TODO(phase 2, buddy 1): S1-U01 ... (define the IDs in
     * testing/subsystem1/TEST_LOG.md first). */

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
