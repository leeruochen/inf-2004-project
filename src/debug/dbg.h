/*
 *  dbg.h
 *  Debug log ring (dbg_).  Owner: platform (Batch B, stage B1).
 *  Pure header: <stdint.h> only, no kernel or SDK headers.
 *
 *  Tasks never print (rule R3a).  They write one line at a time into the
 *  dbg_ ring.  The debug drain task (src/debug/dbg_drain.c, priority 10,
 *  stage B4) is the only reader and the only caller of stdio
 *  (ARCHITECTURE.md D1).
 *
 *  Rules for callers:
 *    - One call = one line.  Do not end the text with '\n': the drain adds
 *      the line end.  A trailing '\n' or '\r' is dropped.
 *    - Lines longer than DBG_LINE_MAX bytes are cut.
 *    - When the ring is full the NEW line is dropped and counted
 *      (dbg_overflow_count()); older lines are kept (batchB-plan O6).
 *    - Tasks only.  No logging from ISRs in Phase 1.
 *    - dbg_printf() formats on the caller's stack (DBG_LINE_MAX + 1 bytes
 *      plus vsnprintf).  usermain() runs on the 1 KB init stack: use
 *      dbg_puts() / dbg_put_err() there, never dbg_printf().
 *    - DBG_ENABLE=0 (Release): every call compiles to nothing.  The
 *      arguments are still type-checked, so Release builds see the same
 *      warnings.
 */

#ifndef DBG_H
#define DBG_H

#include <stdint.h>

#if !defined(DBG_ENABLE)
#error "dbg.h: DBG_ENABLE must be defined by the build (robocar_options)"
#endif

/** Ring size in bytes.  Power of two. */
#define DBG_RING_BYTES      (2048U)
/** Longest line kept, in characters, without the line end. */
#define DBG_LINE_MAX        (96U)

#if DBG_ENABLE

/** Log one line of plain text (no formatting; safe on small stacks).  Empty lines are not logged. */
void dbg_puts(char const *p_line);

/** Log one formatted line (vsnprintf into a DBG_LINE_MAX + 1 byte stack buffer). */
void dbg_printf(char const *p_fmt, ...) __attribute__((format(printf, 1, 2)));

/** Log "[ERR] <p_what> er=<er>" for a failed kernel call (no formatting library). */
void dbg_put_err(char const *p_what, int32_t er);

/**
 * Debug drain only.  Removes the oldest line from the ring and copies it into
 * p_buf as a NUL-terminated string (cut to buf_bytes - 1 characters).
 * Returns the number of characters copied, or 0 when the ring is empty.
 */
uint32_t dbg_ring_read(char *p_buf, uint32_t buf_bytes);

/** Lines dropped because the ring was full, since boot. */
uint32_t dbg_overflow_count(void);

#else /* DBG_ENABLE */

static inline void dbg_puts(char const *p_line)
{
    (void)p_line;
}

static inline __attribute__((format(printf, 1, 2))) void dbg_printf(char const *p_fmt, ...)
{
    (void)p_fmt;
}

static inline void dbg_put_err(char const *p_what, int32_t er)
{
    (void)p_what;
    (void)er;
}

static inline uint32_t dbg_ring_read(char *p_buf, uint32_t buf_bytes)
{
    (void)p_buf;
    (void)buf_bytes;
    return 0U;
}

static inline uint32_t dbg_overflow_count(void)
{
    return 0U;
}

#endif /* DBG_ENABLE */

#endif /* DBG_H */
