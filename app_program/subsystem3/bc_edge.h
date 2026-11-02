/*
 *  bc_edge.h
 *  Subsystem 3 -- barcode edge glitch filter (pure C).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Why: the barcode sensor's comparator has no hysteresis.  While a bar edge
 *  passes under it, its output flickers for a moment: on the car (1 Nov 2026)
 *  one slow pass gave hundreds of flips of 20-800 us.  Each flip is a real
 *  GPIO edge, so the raw edge times are not the bars and spaces.
 *
 *  What: the GP27 handler gives every pin change (time + level) to this
 *  filter.  An element (bar or space) shorter than glitch_us is a glitch: it
 *  is deleted together with the two edges around it, and the element before
 *  it simply continues.  Bars and spaces therefore still alternate.  A run of
 *  flicker pulses at one edge collapses to its last crossing, so a boundary
 *  moves by at most the flicker's length (sub-ms against 5.9 ms or more).
 *
 *  How: an element is only CONFIRMED (its gap handed on) once the element
 *  after it has lasted glitch_us.  The newest edge is TENTATIVE until then.
 *  The last element of a barcode has no edge after it: bc_edge_settle()
 *  confirms it once the pin has stayed put for glitch_us.
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> only.  The handler
 *  (drv_bc.c) owns the instance; the Barcode task reaches it only with
 *  interrupts disabled (drv_bc_settle, drv_bc_last_edge_us).
 */

#ifndef BC_EDGE_H
#define BC_EDGE_H

#include <stdbool.h>
#include <stdint.h>

/** Filter state.  Times are hal_time_us() values (wrap after ~71 min). */
typedef struct
{
    uint32_t glitch_us;     /**< shorter elements are glitches */
    uint32_t start_us;      /**< start of the element in progress (confirmed boundary) */
    uint32_t edge_us;       /**< newest edge, tentative (valid if have_edge) */
    bool     have_edge;     /**< an unconfirmed edge exists */
    bool     level;         /**< pin level after the newest accepted change */
} bc_edge_t;

/** What one pin change did. */
typedef enum
{
    BC_EDGE_IGNORED = 0,    /**< level unchanged (a pulse shorter than the
                                 interrupt latency, or a stale event) */
    BC_EDGE_TENTATIVE,      /**< a new edge, not yet confirmed */
    BC_EDGE_GLITCH,         /**< the element just ended was a glitch: removed */
    BC_EDGE_CONFIRMED       /**< *p_gap_us = an element now confirmed */
} bc_edge_event_t;

/** Start with the pin at level, at time now_us (no edge yet). */
void bc_edge_init(bc_edge_t *p_f, bool level, uint32_t now_us, uint32_t glitch_us);

/*
 * The pin was seen at level at time now_us (after an edge interrupt).
 * On BC_EDGE_CONFIRMED, *p_gap_us = length of the element that is now
 * final, in microseconds; otherwise *p_gap_us is untouched.
 */
bc_edge_event_t bc_edge_on_change(bc_edge_t *p_f, bool level, uint32_t now_us,
                                  uint32_t *p_gap_us);

/*
 * No change for a while: if the tentative edge is at least glitch_us old,
 * the element before it is final.  Returns true and writes *p_gap_us then.
 */
bool bc_edge_settle(bc_edge_t *p_f, uint32_t now_us, uint32_t *p_gap_us);

/** Time of the newest edge still standing (the start of the quiet time). */
uint32_t bc_edge_last_us(bc_edge_t const *p_f);

#endif /* BC_EDGE_H */
