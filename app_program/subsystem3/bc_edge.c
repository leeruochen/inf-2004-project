/*
 *  bc_edge.c
 *  Subsystem 3 -- barcode edge glitch filter (see bc_edge.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).  Called from the GP27
 *  interrupt handler, so every function is short, bounded and never blocks.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_program/subsystem3/bc_edge.h"

void bc_edge_init(bc_edge_t *p_f, bool level, uint32_t now_us, uint32_t glitch_us)
{
    if (p_f != NULL)
    {
        p_f->glitch_us = glitch_us;
        p_f->start_us = now_us;
        p_f->edge_us = now_us;
        p_f->have_edge = false;
        p_f->level = level;
    }
}

bc_edge_event_t bc_edge_on_change(bc_edge_t *p_f, bool level, uint32_t now_us,
                                  uint32_t *p_gap_us)
{
    bc_edge_event_t ev = BC_EDGE_IGNORED;

    if ((p_f != NULL) && (p_gap_us != NULL) && (level != p_f->level))
    {
        p_f->level = level;
        if (!p_f->have_edge)
        {
            /* The element in progress (since start_us) may end here. */
            p_f->edge_us = now_us;
            p_f->have_edge = true;
            ev = BC_EDGE_TENTATIVE;
        }
        else if ((now_us - p_f->edge_us) < p_f->glitch_us)      /* wrap-safe */
        {
            /* The element since edge_us was too short: delete it and both
             * of its edges.  The element since start_us goes on. */
            p_f->have_edge = false;
            ev = BC_EDGE_GLITCH;
        }
        else
        {
            /* The element since edge_us is real, so the one before it ended
             * at edge_us for good.  This edge becomes the tentative one. */
            *p_gap_us = p_f->edge_us - p_f->start_us;
            p_f->start_us = p_f->edge_us;
            p_f->edge_us = now_us;
            ev = BC_EDGE_CONFIRMED;
        }
    }

    return ev;
}

bool bc_edge_settle(bc_edge_t *p_f, uint32_t now_us, uint32_t *p_gap_us)
{
    bool done = false;

    if ((p_f != NULL) && (p_gap_us != NULL) && p_f->have_edge
        && ((now_us - p_f->edge_us) >= p_f->glitch_us))
    {
        /* Quiet since the tentative edge: the element before it is final,
         * and the quiet element now in progress starts at that edge. */
        *p_gap_us = p_f->edge_us - p_f->start_us;
        p_f->start_us = p_f->edge_us;
        p_f->have_edge = false;
        done = true;
    }

    return done;
}

uint32_t bc_edge_last_us(bc_edge_t const *p_f)
{
    uint32_t t = 0U;

    if (p_f != NULL)
    {
        t = p_f->have_edge ? p_f->edge_us : p_f->start_us;
    }

    return t;
}
