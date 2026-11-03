/*
 *  lf_run_test.h
 *  Subsystem 3 -- LF-3.2 Line-Following task test on the car (S3-U12),
 *  APP_TEST only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side only: included by subsystem3_test.c and lf_run_test.c.
 */

#ifndef LF_RUN_TEST_H
#define LF_RUN_TEST_H

#include <stdint.h>

#if APP_TEST
/*
 * Operator-guided test of the running Line-Following task (prompts
 * "[LF] >>>"; guide .agents/tasks/s3-lf32-test.md).  Car on its wheels,
 * battery OFF, a paper strip with the line under the sensors.  Adds
 * its TEST_CHECK results to the caller's counters.
 */
void lf_run_test(uint32_t *p_pass, uint32_t *p_fail);
#endif /* APP_TEST */

#endif /* LF_RUN_TEST_H */
