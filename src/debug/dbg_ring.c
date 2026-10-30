/*
 *  dbg_ring.c
 *  Debug log ring (dbg_): the writer side and the drain's read call.
 *  Owner: platform (Batch B, stage B1).  Kernel side: kernel headers only.
 *  Built only when DBG_ENABLE=ON (src/debug/sources.cmake).
 *
 *  Storage: one byte array used as a ring of records, back to back:
 *      [length (1 byte, 0..DBG_LINE_MAX)][length bytes of text]
 *  A record may wrap around the end of the array.  s_used counts the bytes
 *  in use, so the oldest record starts at (s_head - s_used) mod size.
 *
 *  Concurrency: many writer tasks, one reader (the drain).  Every access to
 *  the ring state runs inside DI/EI (PRIMASK), and holds it only for one
 *  copy of at most DBG_LINE_MAX + 1 bytes (a few microseconds).  Formatting
 *  (vsnprintf) runs before that, with interrupts enabled.
 *
 *  <stdio.h> is included for vsnprintf only: formatting into a buffer, not
 *  console output (rule R3a allows it; check_rules_selftest.cmake).
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* The C library defines size_t above; stop tk/syslib.h from redefining it. */
#define PROHIBIT_DEF_SIZE_T
#include <tk/tkernel.h>

#include "src/debug/dbg.h"

_Static_assert((DBG_RING_BYTES & (DBG_RING_BYTES - 1U)) == 0U, "DBG_RING_BYTES must be a power of two");
_Static_assert(DBG_LINE_MAX <= 255U, "a record length must fit in one byte");
_Static_assert(DBG_RING_BYTES > (DBG_LINE_MAX + 1U), "the ring must hold at least one full line");

#define DBG_RING_MASK       (DBG_RING_BYTES - 1U)

static uint8_t  s_ring[DBG_RING_BYTES];
static uint32_t s_head;         /* next write index */
static uint32_t s_used;         /* bytes in use (records + length bytes) */
static uint32_t s_overflow;     /* lines dropped because the ring was full */

/* Copy len bytes into the ring at index idx, wrapping at the end.  DI held. */
static void ring_copy_in(uint32_t idx, uint8_t const *p_src, uint32_t len)
{
    uint32_t first = DBG_RING_BYTES - idx;

    if (first > len)
    {
        first = len;
    }
    (void)memcpy(&s_ring[idx], p_src, first);
    (void)memcpy(&s_ring[0], &p_src[first], len - first);
}

/* Copy len bytes out of the ring from index idx, wrapping at the end.  DI held. */
static void ring_copy_out(uint32_t idx, uint8_t *p_dst, uint32_t len)
{
    uint32_t first = DBG_RING_BYTES - idx;

    if (first > len)
    {
        first = len;
    }
    (void)memcpy(p_dst, &s_ring[idx], first);
    (void)memcpy(&p_dst[first], &s_ring[0], len - first);
}

/* Append one line (len already cut to DBG_LINE_MAX), or drop it if full. */
static void ring_push(char const *p_text, uint32_t len)
{
    UW      imask;
    uint8_t len_byte;

    /* Drop line ends: the drain adds its own. */
    while ((len > 0U) && ((p_text[len - 1U] == '\n') || (p_text[len - 1U] == '\r')))
    {
        len--;
    }
    /* Empty lines are not stored: dbg_ring_read() returns 0 only for "ring empty". */
    if (len == 0U)
    {
        return;
    }
    len_byte = (uint8_t)len;

    DI(imask);
    if ((DBG_RING_BYTES - s_used) < (len + 1U))
    {
        s_overflow++;
    }
    else
    {
        s_ring[s_head] = len_byte;
        ring_copy_in((s_head + 1U) & DBG_RING_MASK, (uint8_t const *)p_text, len);
        s_head = (s_head + len + 1U) & DBG_RING_MASK;
        s_used += len + 1U;
    }
    EI(imask);
}

/* Length of p_str, at most max characters. */
static uint32_t bounded_len(char const *p_str, uint32_t max)
{
    uint32_t len = 0U;

    while ((len < max) && (p_str[len] != '\0'))
    {
        len++;
    }
    return len;
}

/* Append the decimal form of value at p_out; returns characters written (<= 11). */
static uint32_t put_dec(char *p_out, int32_t value)
{
    char     digits[10];
    uint32_t magnitude = (value < 0) ? (0U - (uint32_t)value) : (uint32_t)value;
    uint32_t n_digits = 0U;
    uint32_t n_out = 0U;

    do
    {
        digits[n_digits] = (char)('0' + (char)(magnitude % 10U));
        n_digits++;
        magnitude /= 10U;
    } while (magnitude != 0U);

    if (value < 0)
    {
        p_out[n_out] = '-';
        n_out++;
    }
    while (n_digits > 0U)
    {
        n_digits--;
        p_out[n_out] = digits[n_digits];
        n_out++;
    }
    return n_out;
}

void dbg_puts(char const *p_line)
{
    if (p_line != NULL)
    {
        ring_push(p_line, bounded_len(p_line, DBG_LINE_MAX));
    }
}

void dbg_printf(char const *p_fmt, ...)
{
    char    line[DBG_LINE_MAX + 1U];
    va_list args;
    int     n;

    if (p_fmt == NULL)
    {
        return;
    }

    va_start(args, p_fmt);
    n = vsnprintf(line, sizeof(line), p_fmt, args);
    va_end(args);

    if (n < 0)
    {
        dbg_puts("[ERR] dbg_printf: format error");
    }
    else
    {
        ring_push(line, ((uint32_t)n > DBG_LINE_MAX) ? DBG_LINE_MAX : (uint32_t)n);
    }
}

void dbg_put_err(char const *p_what, int32_t er)
{
    static char const k_prefix[] = "[ERR] ";
    static char const k_mid[] = " er=";
    char     line[DBG_LINE_MAX + 1U];
    uint32_t len = 0U;
    uint32_t what_len;

    (void)memcpy(&line[len], k_prefix, sizeof(k_prefix) - 1U);
    len += (uint32_t)(sizeof(k_prefix) - 1U);

    /* Room left for " er=" and up to 11 characters of the code. */
    what_len = (p_what != NULL)
             ? bounded_len(p_what, DBG_LINE_MAX - len - (uint32_t)(sizeof(k_mid) - 1U) - 11U)
             : 0U;
    if (what_len > 0U)
    {
        (void)memcpy(&line[len], p_what, what_len);
        len += what_len;
    }

    (void)memcpy(&line[len], k_mid, sizeof(k_mid) - 1U);
    len += (uint32_t)(sizeof(k_mid) - 1U);

    len += put_dec(&line[len], er);

    ring_push(line, len);
}

uint32_t dbg_ring_read(char *p_buf, uint32_t buf_bytes)
{
    UW       imask;
    uint32_t tail;
    uint32_t rec_len;
    uint32_t copy_len = 0U;

    if ((p_buf == NULL) || (buf_bytes == 0U))
    {
        return 0U;
    }

    DI(imask);
    if (s_used != 0U)
    {
        tail = (s_head - s_used) & DBG_RING_MASK;
        rec_len = s_ring[tail];
        copy_len = (rec_len < (buf_bytes - 1U)) ? rec_len : (buf_bytes - 1U);
        ring_copy_out((tail + 1U) & DBG_RING_MASK, (uint8_t *)p_buf, copy_len);
        s_used -= rec_len + 1U;
    }
    EI(imask);

    p_buf[copy_len] = '\0';
    return copy_len;
}

uint32_t dbg_overflow_count(void)
{
    return s_overflow;
}
