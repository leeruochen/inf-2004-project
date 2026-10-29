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

void subsystem3_test(void)
{
    TEST_INIT();

    /* TODO(phase 2, buddy 3): S3-U01 ... (define the IDs in
     * testing/subsystem3/TEST_LOG.md first). */

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
