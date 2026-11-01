/*
 *  bc_frame.c
 *  Subsystem 3 -- barcode capture framing (see bc_frame.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/timing.h"          /* BC_CAPTURE_TIMEOUT_MS, BC_T_END_WIDEST_MULT */
#include "app_program/subsystem3/bc_decode.h"
#include "app_program/subsystem3/bc_frame.h"

/** Microseconds per millisecond. */
#define BC_US_PER_MS                (1000U)

void bc_frame_reset(bc_frame_t *p_frame)
{
    if (p_frame != NULL)
    {
        p_frame->count = 0U;
        p_frame->seen = 0U;
        p_frame->widest_us = 0U;
        p_frame->started = false;
        p_frame->overflow = false;
    }
}

void bc_frame_add(bc_frame_t *p_frame, uint32_t gap_us, int32_t speed_mmps)
{
    if (p_frame != NULL)
    {
        if (!p_frame->started)
        {
            /* The start gap: from an earlier edge to this capture's first. */
            p_frame->started = true;
        }
        else
        {
            if (gap_us > p_frame->widest_us)
            {
                p_frame->widest_us = gap_us;
            }
            if (p_frame->count < BC_FRAME_MAX_ELEMENTS)
            {
                /* Magnitude: -v is safe for every IF-08 value (+-511). */
                int32_t const v = (speed_mmps == INT32_MIN) ? 0
                                  : ((speed_mmps < 0) ? -speed_mmps : speed_mmps);

                p_frame->elem[p_frame->count].gap_us = gap_us;
                p_frame->elem[p_frame->count].speed_mmps = v;
                p_frame->count++;
            }
            if (p_frame->seen < UINT32_MAX)
            {
                p_frame->seen++;
            }
        }
    }
}

void bc_frame_mark_overflow(bc_frame_t *p_frame)
{
    if (p_frame != NULL)
    {
        p_frame->overflow = true;
    }
}

bool bc_frame_ended(bc_frame_t const *p_frame, uint32_t last_edge_us, uint32_t now_us)
{
    /* Unsigned subtraction: correct across the 32-bit wrap (~71 min). */
    uint32_t const quiet_us = now_us - last_edge_us;
    uint32_t       limit_us = BC_CAPTURE_TIMEOUT_MS * BC_US_PER_MS;

    if ((p_frame != NULL) && (p_frame->seen >= BC_SYMBOL_ELEMENTS))
    {
        /* widest_us <= the 20-bit gap maximum, so x 3 fits 32 bits. */
        uint32_t const t_end_us = p_frame->widest_us * BC_T_END_WIDEST_MULT;

        if (t_end_us < limit_us)
        {
            limit_us = t_end_us;
        }
    }

    return quiet_us >= limit_us;
}

bc_frame_result_t bc_frame_finish(bc_frame_t const *p_frame, bc_cfg_t const *p_cfg,
                                  bc_frame_out_t *p_out)
{
    bc_frame_result_t result = BC_FRAME_NO_SYMBOL;
    bc_frame_out_t    out = { '?', 0U, 0U, BC_ERR_COUNT };
    uint32_t          w;

    if ((p_frame == NULL) || (p_cfg == NULL) || (p_out == NULL))
    {
        result = BC_FRAME_INCOMPLETE;
    }
    else if (p_frame->overflow)
    {
        result = BC_FRAME_OVERFLOW;
    }
    else if (p_frame->seen > BC_FRAME_MAX_ELEMENTS)
    {
        result = BC_FRAME_TOO_LONG;
    }
    else if (p_frame->count < BC_SYMBOL_ELEMENTS)
    {
        result = BC_FRAME_INCOMPLETE;
    }
    else
    {
        /* Every window of 29 consecutive elements.  The window that decodes
         * must be the only one; otherwise keep the reject that got furthest
         * (bc_result_t values are in bc_decode()'s check order). */
        for (w = 0U; (w + BC_SYMBOL_ELEMENTS) <= p_frame->count; w++)
        {
            char              ch = '?';
            bc_result_t const r = bc_decode(&p_frame->elem[w], BC_SYMBOL_ELEMENTS, p_cfg, &ch);

            if (r == BC_OK)
            {
                out.windows_ok++;
                out.letter = ch;
                out.window = w;
            }
            else if (r > out.detail)
            {
                out.detail = r;     /* BC_ERR_COUNT (start value) is the lowest */
            }
            else
            {
                /* An earlier window got further: keep its reject. */
            }
        }

        if (out.windows_ok == 1U)
        {
            out.detail = BC_OK;
            result = BC_FRAME_OK;
        }
        else if (out.windows_ok > 1U)
        {
            out.letter = '?';
            out.window = 0U;
            result = BC_FRAME_AMBIGUOUS;
        }
        else
        {
            result = BC_FRAME_NO_SYMBOL;
        }
    }

    if (p_out != NULL)
    {
        *p_out = out;
    }

    return result;
}
