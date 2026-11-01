/*
 *  tel_outbox.c
 *  Subsystem 1 -- event frame ring buffer, drop-oldest (tel_outbox.h).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Pure logic: no kernel or SDK headers, so it builds on the host too.
 */

#include "tel_outbox.h"

#include <stddef.h>

_Static_assert((TEL_OUTBOX_LEN & (TEL_OUTBOX_LEN - 1U)) == 0U, "TEL_OUTBOX_LEN must be a power of two");
_Static_assert(DREP_FRAME_MAX_BYTES <= 255U, "frame length fits tel_frame_t.len");

void tel_outbox_init(tel_outbox_t *p_box)
{
    if (p_box == NULL)
    {
        return;
    }
    p_box->head = 0U;
    p_box->tail = 0U;
    p_box->dropped = 0U;
    p_box->peak = 0U;
}

uint32_t tel_outbox_count(tel_outbox_t const *p_box)
{
    return (p_box != NULL) ? (p_box->head - p_box->tail) : 0U;
}

bool tel_outbox_push(tel_outbox_t *p_box, uint8_t const *p_frame, uint32_t len)
{
    tel_frame_t *p_slot;
    uint32_t     i;
    uint32_t     count;

    if ((p_box == NULL) || (p_frame == NULL) || (len < DREP_FRAME_HDR_BYTES) || (len > DREP_FRAME_MAX_BYTES))
    {
        return false;
    }
    if (tel_outbox_count(p_box) >= TEL_OUTBOX_LEN)
    {
        p_box->tail++;          /* drop the oldest */
        p_box->dropped++;
    }
    p_slot = &p_box->slot[p_box->head & (TEL_OUTBOX_LEN - 1U)];
    for (i = 0U; i < len; i++)
    {
        p_slot->buf[i] = p_frame[i];
    }
    p_slot->len = (uint8_t)len;
    p_box->head++;

    count = tel_outbox_count(p_box);
    if (count > p_box->peak)
    {
        p_box->peak = count;
    }
    return true;
}

tel_frame_t const *tel_outbox_peek(tel_outbox_t const *p_box)
{
    if (tel_outbox_count(p_box) == 0U)
    {
        return NULL;
    }
    return &p_box->slot[p_box->tail & (TEL_OUTBOX_LEN - 1U)];
}

void tel_outbox_pop(tel_outbox_t *p_box)
{
    if (tel_outbox_count(p_box) != 0U)
    {
        p_box->tail++;
    }
}
