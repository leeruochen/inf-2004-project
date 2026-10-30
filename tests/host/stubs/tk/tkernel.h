/*
 *  Host-test stand-in for <tk/tkernel.h>: only what the platform files under
 *  test use.  A host test runs on one thread, so DI/EI do nothing.
 */

#ifndef HOST_STUB_TK_TKERNEL_H
#define HOST_STUB_TK_TKERNEL_H

typedef unsigned long UW;
typedef long          W;
typedef int           INT;
typedef INT           ER;
typedef UW            RELTIM;

typedef struct
{
    W  hi;
    UW lo;
} SYSTIM;

#define E_OK            (0)

/* Time services: defined by the host test that needs them (test_period.c). */
ER tk_get_otm(SYSTIM *pk_tim);
ER tk_dly_tsk(RELTIM dlytim);

#define DI(intsts)      ((intsts) = 0UL)
#define EI(intsts)      ((void)(intsts))

#endif /* HOST_STUB_TK_TKERNEL_H */
