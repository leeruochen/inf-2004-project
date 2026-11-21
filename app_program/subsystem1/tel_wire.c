/*
 *  tel_wire.c
 *  Subsystem 1 -- MQTT topic tree and command receive check (tel_wire.h).
 *  Owner: Buddy 1, Student ID: 2501299 (Damian).
 *
 *  Pure logic: no kernel or SDK headers, so it builds on the host too.
 */

#include "tel_wire.h"

#include <stddef.h>

/* Index = frame type.  Types 8..11 are commands (received, never published). */
static char const *const k_topics[DREP_FT_CMD_START] =
{
    NULL,
    "car/telemetry/motion",
    "car/telemetry/sensor",
    "car/telemetry/barcode",
    "car/telemetry/hump",
    "car/telemetry/obstacle",
    "car/telemetry/fault",
    "car/telemetry/heartbeat",
};

/* VC navigation command topic suffixes, index = frame type - DREP_FT_CMD_START. */
#define TEL_NAV_CMD_COUNT           (3U)
static char const *const k_cmd_suffix[TEL_NAV_CMD_COUNT] =
{
    "start", "stop", "reset"
};

/* strcmp without <string.h> (keeps the module free of C library state). */
static bool str_equal(char const *p_a, char const *p_b)
{
    while ((*p_a != '\0') && (*p_a == *p_b))
    {
        p_a++;
        p_b++;
    }
    return *p_a == *p_b;
}

/* If p_str starts with p_prefix, return the rest, else NULL. */
static char const *after_prefix(char const *p_str, char const *p_prefix)
{
    while (*p_prefix != '\0')
    {
        if (*p_str != *p_prefix)
        {
            return NULL;
        }
        p_str++;
        p_prefix++;
    }
    return p_str;
}

char const *tel_wire_topic(uint32_t frame_type)
{
    return (frame_type < DREP_FT_CMD_START) ? k_topics[frame_type] : NULL;
}

bool tel_wire_is_periodic(uint32_t frame_type)
{
    return (frame_type == DREP_FT_TELEMETRY_MOTION) || (frame_type == DREP_FT_TELEMETRY_SENSOR)
           || (frame_type == DREP_FT_HEARTBEAT);
}

int32_t tel_wire_cmd(char const *p_topic, uint8_t const *p_buf, uint32_t n, nav_input_t *p_nav)
{
    char const *p_suffix;
    nav_input_t nav;
    int32_t     rc;
    uint32_t    i;
    uint32_t    topic_type = 0U;

    if ((p_topic == NULL) || (p_buf == NULL) || (p_nav == NULL))
    {
        return TEL_WIRE_BAD_TOPIC;
    }

    p_suffix = after_prefix(p_topic, TEL_TOPIC_CMD_PREFIX);
    if (p_suffix == NULL)
    {
        return TEL_WIRE_BAD_TOPIC;
    }
    for (i = 0U; i < TEL_NAV_CMD_COUNT; i++)
    {
        if (str_equal(p_suffix, k_cmd_suffix[i]))
        {
            topic_type = DREP_FT_CMD_START + i;
        }
    }
    if (topic_type == 0U)
    {
        return TEL_WIRE_BAD_TOPIC;
    }

    nav.w = 0U;
    rc = drep_cmd_to_nav(p_buf, n, &nav);
    if (rc != DREP_FRAME_OK)
    {
        return rc;
    }
    if ((uint32_t)p_buf[1] != topic_type)
    {
        return TEL_WIRE_TOPIC_MISMATCH;
    }
    *p_nav = nav;
    return DREP_FRAME_OK;
}

int32_t tel_wire_bc_map(char const *p_topic, uint8_t const *p_buf, uint32_t n,
                        char out_map[DREP_BC_CMD_COUNT])
{
    int32_t  rc;
    uint32_t i;
    uint32_t j;

    if ((p_topic == NULL) || (p_buf == NULL) || (out_map == NULL)
        || !str_equal(p_topic, TEL_TOPIC_CMD_BC_MAP))
    {
        return TEL_WIRE_BAD_TOPIC;
    }

    rc = drep_frame_check(p_buf, n);
    if (rc != DREP_FRAME_OK)
    {
        return rc;
    }
    if ((uint32_t)p_buf[1] != DREP_FT_CMD_BC_MAP)
    {
        if ((p_buf[1] >= DREP_FT_CMD_START) && (p_buf[1] < DREP_FT_COUNT))
        {
            return TEL_WIRE_TOPIC_MISMATCH;
        }
        return DREP_FRAME_NOT_COMMAND;
    }

    for (i = 0U; i < DREP_BC_CMD_COUNT; i++)
    {
        uint8_t const ch = p_buf[DREP_FRAME_HDR_BYTES + i];

        if ((ch < (uint8_t)'A') || (ch > (uint8_t)'Z'))
        {
            return TEL_WIRE_BAD_PAYLOAD;
        }
        for (j = i + 1U; j < DREP_BC_CMD_COUNT; j++)
        {
            if (p_buf[DREP_FRAME_HDR_BYTES + j] == ch)
            {
                return TEL_WIRE_BAD_PAYLOAD;
            }
        }
    }

    for (i = 0U; i < DREP_BC_CMD_COUNT; i++)
    {
        out_map[i] = (char)p_buf[DREP_FRAME_HDR_BYTES + i];
    }
    return DREP_FRAME_OK;
}

char const *tel_wire_reject_name(int32_t rc)
{
    switch (rc)
    {
        case DREP_FRAME_OK:             return "OK";
        case DREP_FRAME_SHORT:          return "SHORT";
        case DREP_FRAME_BAD_VERSION:    return "BAD_VERSION";
        case DREP_FRAME_UNKNOWN_TYPE:   return "UNKNOWN_TYPE";
        case DREP_FRAME_BAD_LENGTH:     return "BAD_LENGTH";
        case DREP_FRAME_NOT_COMMAND:    return "NOT_COMMAND";
        case TEL_WIRE_BAD_TOPIC:        return "BAD_TOPIC";
        case TEL_WIRE_TOPIC_MISMATCH:   return "TOPIC_MISMATCH";
        case TEL_WIRE_BAD_PAYLOAD:      return "BAD_PAYLOAD";
        default:                        return "?";
    }
}

uint32_t tel_wire_hex(char *p_out, uint32_t out_bytes, uint8_t const *p_buf, uint32_t n)
{
    static char const k_digits[] = "0123456789abcdef";
    uint32_t          w = 0U;
    uint32_t          i;

    if ((p_out == NULL) || (out_bytes == 0U))
    {
        return 0U;
    }
    for (i = 0U; (i < n) && ((w + 2U) < out_bytes); i++)
    {
        p_out[w] = k_digits[(p_buf[i] >> 4) & 0x0FU];
        p_out[w + 1U] = k_digits[p_buf[i] & 0x0FU];
        w += 2U;
    }
    p_out[w] = '\0';
    return w;
}
