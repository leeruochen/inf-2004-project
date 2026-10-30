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

#include "src/debug/dbg.h"

#if !DBG_ENABLE
#error "APP_TEST=ON needs DBG_ENABLE=ON (the [TEST] lines go through the dbg_ ring)"
#endif

/* File name without its directories, so a FAIL line keeps its line number
 * within DBG_LINE_MAX (__FILE__ is an absolute path). */
static inline char const *test_report_basename_(char const *p_path)
{
    char const *p_name = p_path;

    for (char const *p = p_path; *p != '\0'; p++)
    {
        if ((*p == '/') || (*p == '\\'))
        {
            p_name = p + 1;
        }
    }
    return p_name;
}

#define TEST_INIT()                 uint32_t test_pass_ = 0U, test_fail_ = 0U

#define TEST_CHECK(id, cond, desc)                                          \
    do                                                                      \
    {                                                                       \
        if (cond)                                                           \
        {                                                                   \
            test_pass_++;                                                   \
            dbg_printf("[TEST] %s PASS  %s", (id), (desc));                 \
        }                                                                   \
        else                                                                \
        {                                                                   \
            test_fail_++;                                                   \
            dbg_printf("[TEST] %s FAIL  %s   (%s:%d)", (id), (desc),        \
                       test_report_basename_(__FILE__), __LINE__);          \
        }                                                                   \
    } while (0)

#define TEST_SUMMARY(name)                                                  \
    dbg_printf("[TEST] %s SUMMARY %lu passed, %lu failed", (name),          \
               (unsigned long)test_pass_, (unsigned long)test_fail_)

#endif /* APP_TEST */

#endif /* TEST_REPORT_H */
