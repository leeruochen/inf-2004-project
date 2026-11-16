/*
 *  lf_step_test.h
 *  Subsystem 3 -- S3-U16, steering step diagnostic on the open floor,
 *  APP_TEST only, run only with -DS3_STEP_DIAG=ON.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Kernel side only: included by subsystem3_test.c and lf_step_test.c.
 */

#ifndef LF_STEP_TEST_H
#define LF_STEP_TEST_H

#include <stdint.h>

#if APP_TEST
/*
 * Operator-guided diagnostic, motors live, no line needed: the
 * Line-Following task stays disarmed and the test posts the steering in its
 * place (prompts "[LF] >>>").  Six short drives; the test STOPs each one.
 * Adds its TEST_CHECK result to the caller's counters.
 */
void lf_step_test(uint32_t *p_pass, uint32_t *p_fail);

/*
 * S3-U18 (-DS3_TURN_DIAG=ON): fixed steering from standing until the wheels
 * say 180 degrees, eight drives (+-127, +-64); the operator measures the
 * front tip's start and end marks with a ruler.  Adds its TEST_CHECK result
 * to the caller's counters.
 */
void lf_turn_test(uint32_t *p_pass, uint32_t *p_fail);
#endif /* APP_TEST */

#endif /* LF_STEP_TEST_H */
