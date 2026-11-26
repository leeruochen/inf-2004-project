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
 * Operator-guided test of the running Line-Following task on the track,
 * motors live (prompts "[LF] >>>"; see testing/subsystem3/TEST_LOG.md S3-U12).
 * Static levels, then six bounded drives (straight, three recovery starts,
 * a cross junction, a line end); the test STOPs each drive at its limit.
 * Adds its TEST_CHECK results to the caller's counters.
 */
void lf_run_test(uint32_t *p_pass, uint32_t *p_fail);
#endif /* APP_TEST */

#endif /* LF_RUN_TEST_H */
