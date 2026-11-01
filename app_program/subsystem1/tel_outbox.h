/*
 *  tel_outbox.h
 *  Subsystem 1 -- ring buffer of EVENT frames waiting to be published
 *  (DEGRADED_BUFFERING in design.md statechart 6.5, NFR1).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  PURE: <stdbool.h>, <stdint.h> and src/common/data_repr.h only.
 *
 *  Policy:
 *    - Only event frames (barcode, hump, obstacle, fault) are buffered.
 *      Periodic frames (motion, sensor, heartbeat) are stale 200 ms later,
 *      so they are sent when the link is up and skipped otherwise.
 *    - When full, the OLDEST frame is dropped and counted (design 6.5), so
 *      the newest events survive a long outage.
 *    - Single user: the Telemetry task only (no locking).
 *
 *  RAM: TEL_OUTBOX_LEN x 19 B + indices = 308 B.
 */

#ifndef TEL_OUTBOX_H
#define TEL_OUTBOX_H

#include <stdbool.h>
#include <stdint.h>

#include "src/common/data_repr.h"

/** Slots.  Power of two.  16 events cover a 30 s outage at the expected event rate. */
#define TEL_OUTBOX_LEN      (16U)

typedef struct
{
    uint8_t len;                            /**< frame bytes (3 + payload) */
    uint8_t buf[DREP_FRAME_MAX_BYTES];      /**< the frame, as built by drep_frame_*() */
} tel_frame_t;

typedef struct
{
    tel_frame_t slot[TEL_OUTBOX_LEN];
    uint32_t    head;       /**< free-running write index */
    uint32_t    tail;       /**< free-running read index */
    uint32_t    dropped;    /**< frames dropped (oldest) because the ring was full */
    uint32_t    peak;       /**< highest fill level seen */
} tel_outbox_t;

void tel_outbox_init(tel_outbox_t *p_box);

/**
 * Copy a frame in (len 3..DREP_FRAME_MAX_BYTES).  When full, the oldest frame
 * is dropped first and counted.  Returns false only for a bad argument.
 */
bool tel_outbox_push(tel_outbox_t *p_box, uint8_t const *p_frame, uint32_t len);

/** Oldest frame, or NULL when empty.  Stays in the ring until tel_outbox_pop(). */
tel_frame_t const *tel_outbox_peek(tel_outbox_t const *p_box);

/** Remove the oldest frame (after it was published). */
void tel_outbox_pop(tel_outbox_t *p_box);

uint32_t tel_outbox_count(tel_outbox_t const *p_box);

#endif /* TEL_OUTBOX_H */
