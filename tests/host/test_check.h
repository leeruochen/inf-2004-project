/*
 *  Minimal check macros for host tests.  A failed check prints file:line and
 *  the expression; main() returns the failure count (ctest: non-zero = fail).
 */

#ifndef TEST_CHECK_H
#define TEST_CHECK_H

#include <stdio.h>

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                         \
    do                                                                      \
    {                                                                       \
        g_checks++;                                                         \
        if (!(cond))                                                        \
        {                                                                   \
            g_failures++;                                                   \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
        }                                                                   \
    } while (0)

#define CHECK_STR(actual, expected)                                         \
    do                                                                      \
    {                                                                       \
        g_checks++;                                                         \
        if (strcmp((actual), (expected)) != 0)                              \
        {                                                                   \
            g_failures++;                                                   \
            printf("FAIL %s:%d: got \"%s\", expected \"%s\"\n",             \
                   __FILE__, __LINE__, (actual), (expected));               \
        }                                                                   \
    } while (0)

#define TEST_RESULT(name)                                                   \
    (printf("%s: %d checks, %d failed\n", (name), g_checks, g_failures),    \
     (g_failures == 0) ? 0 : 1)

#endif /* TEST_CHECK_H */
