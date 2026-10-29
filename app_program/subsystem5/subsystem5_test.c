/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5 (obstacle scanning and avoidance).
 *  APP_TEST builds only.
 *  Owner: Buddy 5, Student ID: 2500596 (Chay Han).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem5/logs/ and record the result in
 *  testing/subsystem5/TEST_LOG.md.  Test IDs: S5-U01, S5-U02, ...
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

void subsystem5_test(void)
{
    TEST_INIT();

    /* TODO(phase 2, buddy 5): S5-U01 ... (define the IDs in
     * testing/subsystem5/TEST_LOG.md first). */

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
