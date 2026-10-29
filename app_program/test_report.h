/*
 *  test_report.h
 *  Minimal on-target test helpers.  Owner: team (shared).  Kernel side.
 *
 *  Ported from the course template (ProjectTemplate/app_program/test_report.h).
 *  Same macro names and the same one-line format, but the output goes
 *  through the dbg_ ring (drained to USB CDC by the debug drain task), never
 *  tm_printf from a task.
 *
 *  Every check prints ONE line in a fixed format, so a saved console log can
 *  be searched for results (Select-String '\[TEST\]' log.txt):
 *
 *      [TEST] S1-U01 PASS  ADC reads in range
 *      [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *      [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 *  Usage, inside subsystemN_test():
 *      TEST_INIT();
 *      TEST_CHECK("S1-U01", value <= 4095U, "ADC reads in range");
 *      TEST_SUMMARY("subsystem1");
 *
 *  Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md:
 *  unit S<N>-U<nn>, integration INT-<nn>, system SYS-<nn>; never reused.
 *
 *  Only APP_TEST builds use these macros (test task, TASK_PRI_TEST).
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <stdint.h>

#if APP_TEST

#error "APP_TEST=ON needs the dbg_ ring API: TODO(phase 1 batch B, buddy 2)"

/*
 * Intended expansion once the dbg_ ring exists (Batch B):
 *   TEST_INIT()               uint32_t test_pass_ = 0U, test_fail_ = 0U
 *   TEST_CHECK(id, cond, d)   pass: test_pass_++,
 *                                   dbg_ line "[TEST] %s PASS  %s" (id, d)
 *                             fail: test_fail_++,
 *                                   dbg_ line "[TEST] %s FAIL  %s   (%s:%d)"
 *                                   (id, d, __FILE__, __LINE__)
 *   TEST_SUMMARY(name)        dbg_ line "[TEST] %s SUMMARY %u passed, %u failed"
 */
#define TEST_INIT()                 uint32_t test_pass_ = 0U, test_fail_ = 0U

#define TEST_CHECK(id, cond, desc)                                          \
    do                                                                      \
    {                                                                       \
        if (cond)                                                           \
        {                                                                   \
            test_pass_++;                                                   \
            (void)(id);                                                     \
            (void)(desc);                                                   \
        }                                                                   \
        else                                                                \
        {                                                                   \
            test_fail_++;                                                   \
            (void)(id);                                                     \
            (void)(desc);                                                   \
        }                                                                   \
    } while (0)

#define TEST_SUMMARY(name)                                                  \
    do                                                                      \
    {                                                                       \
        (void)(name);                                                       \
        (void)test_pass_;                                                   \
        (void)test_fail_;                                                   \
    } while (0)

#endif /* APP_TEST */

#endif /* TEST_REPORT_H */
