/*
 *  Host-test stand-in for <tk/tkernel.h>: only what the platform files under
 *  test use.  A host test runs on one thread, so DI/EI do nothing.
 */

#ifndef HOST_STUB_TK_TKERNEL_H
#define HOST_STUB_TK_TKERNEL_H

typedef unsigned long UW;

#define DI(intsts)      ((intsts) = 0UL)
#define EI(intsts)      ((void)(intsts))

#endif /* HOST_STUB_TK_TKERNEL_H */
