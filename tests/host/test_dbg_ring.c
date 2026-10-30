/*
 *  test_dbg_ring.c
 *  Host tests for the dbg_ ring (src/debug/dbg_ring.c, batchB-plan O6).
 *  The source file is included directly, so each test can reset the ring's
 *  static state.
 */

#include <stdint.h>
#include <string.h>

#include "test_check.h"

#include "src/debug/dbg_ring.c"

static void ring_reset(void)
{
    (void)memset(s_ring, 0xA5, sizeof(s_ring));
    s_head = 0U;
    s_used = 0U;
    s_overflow = 0U;
}

/* Place the empty ring's write index at idx, to test wrap-around. */
static void ring_reset_at(uint32_t idx)
{
    ring_reset();
    s_head = idx & DBG_RING_MASK;
}

static void test_empty(void)
{
    char buf[DBG_LINE_MAX + 1U];

    ring_reset();
    buf[0] = 'x';
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 0U);
    CHECK(buf[0] == '\0');
    CHECK(dbg_ring_read(NULL, 10U) == 0U);
    CHECK(dbg_ring_read(buf, 0U) == 0U);
    CHECK(dbg_overflow_count() == 0U);
}

static void test_roundtrip_and_order(void)
{
    char buf[DBG_LINE_MAX + 1U];

    ring_reset();
    dbg_puts("first");
    dbg_puts("second\n");       /* line end dropped */
    dbg_puts("third\r\n");
    dbg_puts("");               /* not stored */
    dbg_puts("\n");             /* empty after the line end: not stored */
    dbg_puts(NULL);             /* ignored */

    CHECK(dbg_ring_read(buf, sizeof(buf)) == 5U);
    CHECK_STR(buf, "first");
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 6U);
    CHECK_STR(buf, "second");
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 5U);
    CHECK_STR(buf, "third");
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 0U);
    CHECK(s_used == 0U);
}

static void test_long_line_is_cut(void)
{
    char src[200];
    char buf[DBG_LINE_MAX + 1U];

    ring_reset();
    (void)memset(src, 'L', sizeof(src) - 1U);
    src[sizeof(src) - 1U] = '\0';

    dbg_puts(src);
    CHECK(dbg_ring_read(buf, sizeof(buf)) == DBG_LINE_MAX);
    CHECK(strlen(buf) == DBG_LINE_MAX);

    dbg_printf("%s", src);
    CHECK(dbg_ring_read(buf, sizeof(buf)) == DBG_LINE_MAX);
    CHECK(buf[DBG_LINE_MAX - 1U] == 'L');
}

static void test_printf(void)
{
    char buf[DBG_LINE_MAX + 1U];

    ring_reset();
    dbg_printf("[TEST] %s PASS  %s", "S3-U01", "ADC in range");
    dbg_printf("v=%d u=%lu x=%02x", -42, 4000000000UL, 0xAU);
    CHECK(dbg_ring_read(buf, sizeof(buf)) > 0U);
    CHECK_STR(buf, "[TEST] S3-U01 PASS  ADC in range");
    CHECK(dbg_ring_read(buf, sizeof(buf)) > 0U);
    CHECK_STR(buf, "v=-42 u=4000000000 x=0a");
}

static void test_put_err(void)
{
    char buf[DBG_LINE_MAX + 1U];
    char what[200];

    ring_reset();
    dbg_put_err("subsystem1_init", -17);
    dbg_put_err("min", INT32_MIN);
    dbg_put_err("zero", 0);
    dbg_put_err(NULL, -1);

    (void)dbg_ring_read(buf, sizeof(buf));
    CHECK_STR(buf, "[ERR] subsystem1_init er=-17");
    (void)dbg_ring_read(buf, sizeof(buf));
    CHECK_STR(buf, "[ERR] min er=-2147483648");
    (void)dbg_ring_read(buf, sizeof(buf));
    CHECK_STR(buf, "[ERR] zero er=0");
    (void)dbg_ring_read(buf, sizeof(buf));
    CHECK_STR(buf, "[ERR]  er=-1");

    /* A very long name is cut, but the error code always fits. */
    (void)memset(what, 'w', sizeof(what) - 1U);
    what[sizeof(what) - 1U] = '\0';
    dbg_put_err(what, INT32_MIN);
    CHECK(dbg_ring_read(buf, sizeof(buf)) <= DBG_LINE_MAX);
    CHECK(strstr(buf, " er=-2147483648") != NULL);
}

static void test_full_drops_newest(void)
{
    char     buf[DBG_LINE_MAX + 1U];
    char     line[16];
    uint32_t stored = 0U;
    uint32_t i;

    /* Each record is 1 length byte + 9 characters = 10 bytes. */
    ring_reset();
    for (i = 0U; i < 1000U; i++)
    {
        (void)snprintf(line, sizeof(line), "line%05u", (unsigned)i);
        dbg_puts(line);
    }
    stored = DBG_RING_BYTES / 10U;
    CHECK(dbg_overflow_count() == (1000U - stored));
    CHECK(s_used == (stored * 10U));

    /* The OLDEST lines are kept, in order. */
    for (i = 0U; i < stored; i++)
    {
        (void)snprintf(line, sizeof(line), "line%05u", (unsigned)i);
        CHECK(dbg_ring_read(buf, sizeof(buf)) == 9U);
        CHECK_STR(buf, line);
    }
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 0U);

    /* Space is usable again after the drain read it. */
    dbg_puts("after");
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 5U);
    CHECK_STR(buf, "after");
    CHECK(dbg_overflow_count() == (1000U - stored));
}

static void test_exact_fit(void)
{
    char     buf[DBG_LINE_MAX + 1U];
    char     line[DBG_LINE_MAX + 1U];
    uint32_t free_bytes;

    ring_reset();
    (void)memset(line, 'f', DBG_LINE_MAX);
    line[DBG_LINE_MAX] = '\0';
    while ((DBG_RING_BYTES - s_used) >= (DBG_LINE_MAX + 1U))
    {
        dbg_puts(line);
    }
    free_bytes = DBG_RING_BYTES - s_used;
    CHECK(free_bytes < (DBG_LINE_MAX + 1U));

    /* A line that needs exactly the free space fits; one more byte does not. */
    if (free_bytes >= 2U)
    {
        line[free_bytes - 1U] = '\0';
        dbg_puts(line);
        CHECK(s_used == DBG_RING_BYTES);
        CHECK(dbg_overflow_count() == 0U);
        dbg_puts("x");
        CHECK(dbg_overflow_count() == 1U);
    }
    while (dbg_ring_read(buf, sizeof(buf)) != 0U)
    {
    }
    CHECK(s_used == 0U);
}

static void test_wrap_around(void)
{
    char     buf[DBG_LINE_MAX + 1U];
    char     line[DBG_LINE_MAX + 1U];
    uint32_t start;
    uint32_t len;
    uint32_t k;

    /* Start the write index at every position near the end of the array, with
     * every length, so both the length byte and the text cross the end. */
    for (start = DBG_RING_BYTES - (DBG_LINE_MAX + 2U); start < (DBG_RING_BYTES + 2U); start++)
    {
        for (len = 1U; len <= DBG_LINE_MAX; len += 7U)
        {
            ring_reset_at(start);
            for (k = 0U; k < len; k++)
            {
                line[k] = (char)('A' + (char)((k + start) % 26U));
            }
            line[len] = '\0';
            dbg_puts(line);
            CHECK(dbg_ring_read(buf, sizeof(buf)) == len);
            CHECK(strcmp(buf, line) == 0);
            CHECK(s_used == 0U);
        }
    }
}

static void test_long_run(void)
{
    char     buf[DBG_LINE_MAX + 1U];
    char     line[DBG_LINE_MAX + 1U];
    uint32_t written = 0U;
    uint32_t read = 0U;
    uint32_t round;
    uint32_t n;

    /* Many write/read rounds with varying lengths: everything that was
     * stored comes back intact and in order.  The reader keeps up (up to 8
     * lines per round, writers at most 7), as the drain does, so nothing is
     * dropped while the ring wraps around many times. */
    ring_reset();
    for (round = 0U; round < 500U; round++)
    {
        for (n = 0U; n < (round % 7U) + 1U; n++)
        {
            (void)snprintf(line, sizeof(line), "%u:%.*s", (unsigned)written,
                           (int)((written * 13U) % 80U), "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzab");
            dbg_puts(line);
            written++;
        }
        for (n = 0U; n < 8U; n++)
        {
            uint32_t got = dbg_ring_read(buf, sizeof(buf));
            if (got == 0U)
            {
                break;
            }
            (void)snprintf(line, sizeof(line), "%u:%.*s", (unsigned)read,
                           (int)((read * 13U) % 80U), "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzab");
            CHECK(strcmp(buf, line) == 0);
            read++;
        }
    }
    CHECK(dbg_overflow_count() == 0U);
    while (dbg_ring_read(buf, sizeof(buf)) != 0U)
    {
        read++;
    }
    CHECK(read == written);
}

static void test_small_read_buffer(void)
{
    char buf[4];

    ring_reset();
    dbg_puts("abcdef");
    dbg_puts("gh");
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 3U);   /* cut, but consumed */
    CHECK_STR(buf, "abc");
    CHECK(dbg_ring_read(buf, sizeof(buf)) == 2U);
    CHECK_STR(buf, "gh");
    CHECK(s_used == 0U);
}

int main(void)
{
    test_empty();
    test_roundtrip_and_order();
    test_long_line_is_cut();
    test_printf();
    test_put_err();
    test_full_drops_newest();
    test_exact_fit();
    test_wrap_around();
    test_long_run();
    test_small_read_buffer();
    return TEST_RESULT("dbg_ring");
}
