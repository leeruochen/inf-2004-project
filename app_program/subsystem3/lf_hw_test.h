/*
 *  lf_hw_test.h
 *  Subsystem 3 -- LF-1 line sensor measurement test (S3-U09), APP_TEST only.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side only: included by subsystem3_test.c and lf_hw_test.c.
 */

#ifndef LF_HW_TEST_H
#define LF_HW_TEST_H

#include <stdint.h>

#if APP_TEST
/*
 * Operator-guided measurement of both line sensors (prompts "[LF] >>>"):
 * white, line under CENTRE, line under RIGHT, both over black, then a slow
 * sideways sweep.  Prints the raw statistics ("[LF] ...") that LF-2's
 * thresholds will come from, and adds its TEST_CHECK results to the
 * caller's counters.
 */
void lf_hw_test(uint32_t *p_pass, uint32_t *p_fail);
#endif /* APP_TEST */

#endif /* LF_HW_TEST_H */
