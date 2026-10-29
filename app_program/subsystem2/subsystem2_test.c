/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2 (motion control and Vehicle Controller).
 *  APP_TEST builds only.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 *
 *  Each TEST_CHECK prints a PASS/FAIL line through the dbg_ ring.  Save the
 *  console output to testing/subsystem2/logs/ and record the result in
 *  testing/subsystem2/TEST_LOG.md.  Test IDs: S2-U01, S2-U02, ...
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

void subsystem2_test(void)
{
    TEST_INIT();

    /* TODO(phase 2, buddy 2): S2-U01 ... (define the IDs in
     * testing/subsystem2/TEST_LOG.md first). */

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
