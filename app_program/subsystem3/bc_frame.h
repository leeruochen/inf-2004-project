/*
 *  bc_frame.h
 *  Subsystem 3 -- barcode capture framing (pure C).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Turns the gaps of one capture (time between barcode-sensor edges, each
 *  paired with Motion's speed) into one decoded letter or a reason why not:
 *    - drops the first gap of a capture (it ends at the first edge, so it is
 *      the time since an earlier, unrelated edge, not an element);
 *    - decides when the capture has ended (no edge for long enough);
 *    - finds the 29-element symbol among the captured elements and decodes it
 *      with bc_decode() (BC-1).
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and pure project
 *  headers only, so tests/host/test_bc_frame.c builds it unchanged.  The
 *  Barcode Decode task (BC-2A.2) feeds it; the GP27 ISR never calls it.
 */

#ifndef BC_FRAME_H
#define BC_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#include "app_program/subsystem3/bc_decode.h"

/** Elements kept per capture: one symbol (29) plus up to 2 extra elements at
 *  each end.  The printed sheet's edge is ~4 mm from the bars, so if the
 *  track around it reads dark, a pass adds a quiet-zone space and a track
 *  "bar" before and/or after the symbol. */
#define BC_FRAME_MAX_ELEMENTS       (BC_SYMBOL_ELEMENTS + 4U)

/** One capture in progress.  Owned by one task; reset before each capture. */
typedef struct
{
    bc_element_t elem[BC_FRAME_MAX_ELEMENTS];   /**< elements in crossing order */
    uint32_t     count;         /**< elements stored (<= BC_FRAME_MAX_ELEMENTS) */
    uint32_t     seen;          /**< elements offered, stored or not */
    uint32_t     widest_us;     /**< longest element gap so far (the start gap excluded) */
    bool         started;       /**< the start gap has been received and dropped */
    bool         overflow;      /**< the edge ring dropped edges during this capture */
} bc_frame_t;

/** Capture outcome.  Every value but BC_FRAME_OK is an invalid read. */
typedef enum
{
    BC_FRAME_OK = 0,            /**< exactly one window decoded: letter valid */
    BC_FRAME_OVERFLOW,          /**< edges were lost: elements cannot be trusted */
    BC_FRAME_TOO_LONG,          /**< more than BC_FRAME_MAX_ELEMENTS elements */
    BC_FRAME_INCOMPLETE,        /**< fewer than BC_SYMBOL_ELEMENTS elements */
    BC_FRAME_NO_SYMBOL,         /**< no 29-element window decoded (see detail) */
    BC_FRAME_AMBIGUOUS          /**< more than one window decoded: refuse to guess */
} bc_frame_result_t;

/** What bc_frame_finish() found, for the result log line. */
typedef struct
{
    char        letter;         /**< 'A'..'Z' on BC_FRAME_OK, else '?' */
    uint32_t    window;         /**< first element of the decoded window (BC_FRAME_OK) */
    uint32_t    windows_ok;     /**< windows that decoded (0, 1, or more) */
    bc_result_t detail;         /**< BC_FRAME_NO_SYMBOL: the BC-1 reject of the window
                                     that got furthest through bc_decode()'s checks */
} bc_frame_out_t;

/** Start a new capture: no elements, nothing seen, no overflow. */
void bc_frame_reset(bc_frame_t *p_frame);

/*
 * Add the next gap of the capture, in the order the edges happened.
 * The first gap after bc_frame_reset() is the start gap and is dropped.
 * speed_mmps: Motion's speed (IF-08, signed mm/s) that the task paired with
 * this gap; its magnitude is stored, because a width is positive whichever
 * way the car drives (INT32_MIN, never produced by IF-08, is stored as 0).
 * Elements beyond BC_FRAME_MAX_ELEMENTS are counted but not stored.
 */
void bc_frame_add(bc_frame_t *p_frame, uint32_t gap_us, int32_t speed_mmps);

/** The edge ring dropped edges during this capture (its overflow count moved). */
void bc_frame_mark_overflow(bc_frame_t *p_frame);

/*
 * True when the capture is over: no edge for long enough.  Call only when no
 * gap is waiting in the edge ring.  quiet = now_us - last_edge_us (wraps
 * correctly through 2^32).  The limit is BC_CAPTURE_TIMEOUT_MS; once at least
 * BC_SYMBOL_ELEMENTS elements have been seen it is lowered to T_end =
 * BC_T_END_WIDEST_MULT x the widest element, if that is shorter (design.md
 * 6.2: "a quiet gap longer than T_end ... and the expected element count is
 * reached").  Before 29 elements T_end is not used: a symbol starts narrow
 * bar, wide space (3 x as long), so "3 x widest so far" would end the capture
 * inside the symbol.
 */
bool bc_frame_ended(bc_frame_t const *p_frame, uint32_t last_edge_us, uint32_t now_us);

/*
 * Decide the capture.  Checks, in order: overflow, too many elements, too
 * few, then every window of BC_SYMBOL_ELEMENTS consecutive elements through
 * bc_decode().  OK only if exactly one window decodes: two can (for example
 * *D* forwards at window 0 and *I* backwards at window 2 of the same 31
 * elements), and then the read is refused rather than guessed.
 * p_out is filled whenever it is not NULL; letter and window mean something
 * only on BC_FRAME_OK.
 */
bc_frame_result_t bc_frame_finish(bc_frame_t const *p_frame, bc_cfg_t const *p_cfg,
                                  bc_frame_out_t *p_out);

#endif /* BC_FRAME_H */
