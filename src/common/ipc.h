/*
 *  ipc.h
 *  The IPC objects of design.md 3.3, created once by ipc_init() in
 *  usermain() step 1, before any subsystemN_init() (ARCHITECTURE.md T5).
 *  Owner: platform (Batch B, stage B2).  Kernel side: kernel headers only.
 *  app.h includes it, so every subsystem sees the object names.
 *
 *  Use the TEAM.md section 5 names directly, for example
 *      tk_snd_mbf(MBF_NAV_IN, &msg, sizeof(msg), TMO_POL);
 *  They are valid only after ipc_init(), so never use them in a static
 *  initialiser.
 */

#ifndef IPC_H
#define IPC_H

#include <stddef.h>                 /* size_t from the C library (see app.h) */
#include <stdint.h>
#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif
#include <tk/tkernel.h>

#include "src/common/ipc_types.h"

/* Object IDs, set by ipc_init().  Use the macros below, not these names. */
extern ID ipc_mbf_nav_in;
extern ID ipc_mbf_motion_cmd;
extern ID ipc_mbf_obstacle;
extern ID ipc_mbf_telemetry;
extern ID ipc_sem_echo;
extern ID ipc_evf_safety;
extern ID ipc_evf_nav;
extern ID ipc_mtx_motion;

#define MBF_NAV_IN          (ipc_mbf_nav_in)        /**< IF-01, 8 x nav_input_t */
#define MBF_MOTION_CMD      (ipc_mbf_motion_cmd)    /**< IF-02, 4 x motion_cmd_t */
#define MBF_OBSTACLE        (ipc_mbf_obstacle)      /**< IF-03, 2 x obstacle_profile_t */
#define MBF_TELEMETRY       (ipc_mbf_telemetry)     /**< IF-04, 32 x tel_evt_t */
#define SEM_ECHO            (ipc_sem_echo)          /**< IF-05, count 0, max 1 */
#define EVF_SAFETY          (ipc_evf_safety)        /**< IF-06, TA_WSGL */
#define EVF_NAV             (ipc_evf_nav)           /**< IF-07, TA_WMUL */
#define MTX_MOTION          (ipc_mtx_motion)        /**< IF-08, TA_INHERIT */

/**
 * Create every IPC object, check each message buffer's depth (N messages
 * fit, N+1 do not), and check the kernel version.  Called once, by
 * usermain() step 1, on the 1 KB init stack.  Logs through the dbg_ ring.
 * Returns E_OK, or the first negative error (every object is still tried).
 */
ER ipc_init(void);

#endif /* IPC_H */
