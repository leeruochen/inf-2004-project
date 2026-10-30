/*
 *  mbf_size.h
 *  Message-buffer sizing.  Owner: platform (Batch B, stage B2).
 *  Pure header: <stdint.h> only.
 *
 *  micro T-Kernel stores every message as a 4-byte header plus the message
 *  rounded up to 4 bytes (kernel/tkernel/messagebuf.h:58-62, HEADERSZ and
 *  ROUNDSZ; messagebuf.c:65).  So a buffer that must hold exactly N messages
 *  of type T needs N * (4 + round4(sizeof(T))) bytes.  The kernel checks
 *  free space with the unrounded size (messagebuf.h:68-71), so message N+1
 *  never fits: ipc_init() proves this at boot (depth self-check).
 */

#ifndef MBF_SIZE_H
#define MBF_SIZE_H

#include <stdint.h>

#include "src/common/ipc_types.h"

#define MBF_HEADER_BYTES            (4U)
#define MBF_SLOT_BYTES(msg_bytes)   (MBF_HEADER_BYTES + (((uint32_t)(msg_bytes) + 3U) & ~3U))
#define MBF_BUFSZ(n, T)             ((uint32_t)(n) * MBF_SLOT_BYTES(sizeof(T)))

/* Expected RAM per buffer, inside the kernel pool (docs/DATA_REPRESENTATION.md section 6). */
_Static_assert(MBF_BUFSZ(MBF_NAV_IN_DEPTH, nav_input_t) == 64U, "MBF_NAV_IN is 64 B");
_Static_assert(MBF_BUFSZ(MBF_MOTION_CMD_DEPTH, motion_cmd_t) == 64U, "MBF_MOTION_CMD is 64 B");
_Static_assert(MBF_BUFSZ(MBF_OBSTACLE_DEPTH, obstacle_profile_t) == 40U, "MBF_OBSTACLE is 40 B");
_Static_assert(MBF_BUFSZ(MBF_TELEMETRY_DEPTH, tel_evt_t) == 640U, "MBF_TELEMETRY is 640 B");

#endif /* MBF_SIZE_H */
