/*
 *  test_crc32.c
 *  Host tests for src/common/crc32.h (fault record CRC, batchB-plan O4).
 */

#include <stdint.h>
#include <string.h>

#include "test_check.h"

#include "src/common/crc32.h"

int main(void)
{
    static char const k_check[] = "123456789";
    uint8_t const     zeros[4] = { 0U, 0U, 0U, 0U };
    uint32_t          words[7] = { 1U, 1U, 0x10000340UL, 0x100002F1UL, 0x61000000UL, 0U, 0U };
    uint32_t          before;

    /* Standard CRC-32/IEEE check value. */
    CHECK(crc32_ieee(k_check, 9U) == CRC32_CHECK_VALUE);
    CHECK(crc32_ieee(k_check, 9U) == 0xCBF43926UL);
    /* Known values: empty input, four zero bytes. */
    CHECK(crc32_ieee(k_check, 0U) == 0x00000000UL);
    CHECK(crc32_ieee(zeros, 4U) == 0x2144DF1CUL);
    /* Any single-bit change in a fault payload changes the CRC. */
    before = crc32_ieee(words, (uint32_t)sizeof(words));
    for (uint32_t bit = 0U; bit < (uint32_t)(sizeof(words) * 8U); bit++)
    {
        words[bit / 32U] ^= (1UL << (bit % 32U));
        CHECK(crc32_ieee(words, (uint32_t)sizeof(words)) != before);
        words[bit / 32U] ^= (1UL << (bit % 32U));
    }
    CHECK(crc32_ieee(words, (uint32_t)sizeof(words)) == before);
    return TEST_RESULT("crc32");
}
