/*
 *  crc32.h
 *  CRC-32/IEEE (reflected, polynomial 0xEDB88320, init and xorout
 *  0xFFFFFFFF), bitwise, no table.  Used for the RAM fault record
 *  (batchB-plan O4).  Check value: crc32_ieee("123456789", 9) == 0xCBF43926.
 *  Owner: platform (Batch B, stage B3).  Pure header: <stdint.h> only, so
 *  the SDK-side fault handler and the host tests can both use it.
 *  Safe in a fault handler: no tables, no library calls, stack-light.
 */

#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>

#define CRC32_CHECK_VALUE   (0xCBF43926UL)  /**< crc32_ieee("123456789", 9) */

static inline uint32_t crc32_ieee(void const *p_data, uint32_t len)
{
    uint8_t const *p = (uint8_t const *)p_data;
    uint32_t       crc = 0xFFFFFFFFUL;
    uint32_t       bit;

    while (len > 0U)
    {
        crc ^= (uint32_t)*p;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 1UL) != 0UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
        p++;
        len--;
    }
    return crc ^ 0xFFFFFFFFUL;
}

#endif /* CRC32_H */
