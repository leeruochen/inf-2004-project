/*
 *  tel_link.c
 *  Subsystem 1 -- WiFi / MQTT connection state machine (see tel_link.h).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Pure logic: no kernel or SDK headers, so it builds on the host too.
 */

#include "tel_link.h"

#include <stddef.h>

/* Wrap-safe "a is at or after b" for kernel millisecond times. */
static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void enter(tel_link_t *p_link, uint32_t state, uint32_t now_ms)
{
    p_link->state = state;
    p_link->entered_ms = now_ms;
}

/* Start a backoff wait with the current backoff, then double it (capped). */
static void start_backoff(tel_link_t *p_link, uint32_t now_ms)
{
    uint32_t next;

    p_link->last_wait_ms = p_link->backoff_ms;
    p_link->retry_at_ms = now_ms + p_link->backoff_ms;
    p_link->failures++;

    next = p_link->backoff_ms * 2U;
    if ((next < p_link->backoff_ms) || (next > p_link->cfg.backoff_max_ms))
    {
        next = p_link->cfg.backoff_max_ms;  /* overflow or cap */
    }
    p_link->backoff_ms = next;
    enter(p_link, TEL_LINK_RECONNECTING, now_ms);
}

void tel_link_init(tel_link_t *p_link, tel_link_cfg_t const *p_cfg, uint32_t now_ms)
{
    if ((p_link == NULL) || (p_cfg == NULL))
    {
        return;
    }
    p_link->cfg = *p_cfg;
    if (p_link->cfg.backoff_min_ms == 0U)
    {
        p_link->cfg.backoff_min_ms = 1U;
    }
    if (p_link->cfg.backoff_max_ms < p_link->cfg.backoff_min_ms)
    {
        p_link->cfg.backoff_max_ms = p_link->cfg.backoff_min_ms;
    }
    p_link->backoff_ms = p_link->cfg.backoff_min_ms;
    p_link->retry_at_ms = now_ms;
    p_link->last_wait_ms = 0U;
    p_link->ever_connected = false;
    p_link->last_net_up = false;
    p_link->connects = 0U;
    p_link->failures = 0U;
    enter(p_link, TEL_LINK_DISCONNECTED, now_ms);
}

uint32_t tel_link_step(tel_link_t *p_link, uint32_t now_ms, tel_link_in_t const *p_in)
{
    uint32_t act = TEL_LINK_ACT_NONE;

    if ((p_link == NULL) || (p_in == NULL))
    {
        return TEL_LINK_ACT_NONE;
    }
    p_link->last_net_up = p_in->net_up;

    switch (p_link->state)
    {
        case TEL_LINK_DISCONNECTED:
            enter(p_link, TEL_LINK_CONNECTING, now_ms);
            act = TEL_LINK_ACT_CONNECT;
            break;

        case TEL_LINK_CONNECTING:
            if (p_in->net_up)
            {
                p_link->backoff_ms = p_link->cfg.backoff_min_ms;
                p_link->ever_connected = true;
                p_link->connects++;
                enter(p_link, TEL_LINK_CONNECTED, now_ms);
            }
            else if (p_in->attempt_failed
                     || time_reached(now_ms, p_link->entered_ms + p_link->cfg.connect_timeout_ms))
            {
                start_backoff(p_link, now_ms);
                act = TEL_LINK_ACT_TEARDOWN;
            }
            else
            {
                /* still joining / resolving / waiting for CONNACK */
            }
            break;

        case TEL_LINK_CONNECTED:
            if ((!p_in->net_up) || p_in->publish_failed)
            {
                enter(p_link, TEL_LINK_DEGRADED, now_ms);
            }
            break;

        case TEL_LINK_DEGRADED:
            if (p_in->net_up && (!p_in->publish_failed) && p_in->outbox_empty)
            {
                enter(p_link, TEL_LINK_CONNECTED, now_ms);
            }
            else if (p_in->net_up && (!p_in->publish_failed))
            {
                /* recovering: the outbox is being flushed; T_ka restarts */
                p_link->entered_ms = now_ms;
            }
            else if (time_reached(now_ms, p_link->entered_ms + p_link->cfg.degrade_timeout_ms))
            {
                start_backoff(p_link, now_ms);
                act = TEL_LINK_ACT_TEARDOWN;
            }
            else
            {
                /* buffering, waiting for the link or the output buffer */
            }
            break;

        case TEL_LINK_RECONNECTING:
            if (time_reached(now_ms, p_link->retry_at_ms))
            {
                enter(p_link, TEL_LINK_CONNECTING, now_ms);
                act = TEL_LINK_ACT_CONNECT;
            }
            break;

        default:
            /* corrupted state: start over (fail safe, never stuck) */
            enter(p_link, TEL_LINK_DISCONNECTED, now_ms);
            break;
    }
    return act;
}

uint32_t tel_link_state(tel_link_t const *p_link)
{
    return (p_link != NULL) ? p_link->state : TEL_LINK_DISCONNECTED;
}

bool tel_link_is_lost(tel_link_t const *p_link)
{
    if ((p_link == NULL) || (!p_link->ever_connected))
    {
        return false;
    }
    return (p_link->state != TEL_LINK_CONNECTED) && (!p_link->last_net_up);
}

bool tel_link_can_send(tel_link_t const *p_link)
{
    if (p_link == NULL)
    {
        return false;
    }
    return (p_link->state == TEL_LINK_CONNECTED)
           || ((p_link->state == TEL_LINK_DEGRADED) && p_link->last_net_up);
}

char const *tel_link_state_name(uint32_t state)
{
    static char const *const k_names[TEL_LINK_STATE_COUNT] =
    {
        "DISCONNECTED", "CONNECTING", "CONNECTED", "DEGRADED", "RECONNECTING"
    };

    return (state < TEL_LINK_STATE_COUNT) ? k_names[state] : "?";
}
