/*
 *  bc_decode.c
 *  Subsystem 3 -- Code 39 barcode decoder core (see bc_decode.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/thresholds.h"      /* BC_WIDE_RATIO_X100, BC_MIN_SPEED_MMPS */
#include "src/common/data_repr.h"   /* DREP_BC_CMD_*, DREP_EDGE_GAP_MAX, DREP_MS_SPEED_X10_MAX */
#include "app_program/subsystem3/bc_decode.h"

/** Micrometres per (microsecond x millimetre per second). */
#define BC_US_MMPS_PER_UM           (1000U)
/** Characters per symbol: start '*', the command, stop '*'. */
#define BC_SYMBOL_CHARS             (3U)
/** Elements per character: 5 bars and 4 spaces. */
#define BC_CHAR_ELEMENTS            (9U)
/** Character positions in a symbol. */
#define BC_CHAR_START               (0U)
#define BC_CHAR_CMD                 (1U)
#define BC_CHAR_STOP                (2U)
/** A character plus the gap that follows it. */
#define BC_CHAR_STRIDE              (BC_CHAR_ELEMENTS + 1U)
/** Elements that carry data (the 2 gaps between characters excluded). */
#define BC_CODED_ELEMENTS           (BC_SYMBOL_CHARS * BC_CHAR_ELEMENTS)
/** Fixed-point scale of bc_cfg_t.wide_ratio_x100. */
#define BC_RATIO_SCALE              (100U)

/* Code 39 character patterns (ISO/IEC 16388 table): the 9 elements of a
 * character in crossing order, first element in bit 8, 1 = wide. */
#define BC_PATTERN_START_STOP       (0x094U)    /* '*' */
#define BC_PATTERN_A                (0x109U)
#define BC_PATTERN_B                (0x049U)
#define BC_PATTERN_C                (0x148U)
#define BC_PATTERN_D                (0x019U)
#define BC_COMMANDS                 (4U)
/** Every Code 39 character has exactly 3 wide elements of 9 ("3 of 9"). */
#define BC_WIDE_PER_CHAR            (3U)

/** The four command characters and what each means (FR9). */
static uint16_t const s_cmd_pattern[BC_COMMANDS] =
{
    BC_PATTERN_A, BC_PATTERN_B, BC_PATTERN_C, BC_PATTERN_D
};
static uint32_t const s_cmd_value[BC_COMMANDS] =
{
    DREP_BC_CMD_LEFT, DREP_BC_CMD_RIGHT, DREP_BC_CMD_STRAIGHT, DREP_BC_CMD_UTURN
};

/* Median of the coded widths: an insertion-sorted copy, middle entry.  With
 * 27 elements of which 9 are wide, the median is always a narrow width. */
static uint32_t bc_median_um(uint32_t const p_width[BC_CODED_ELEMENTS])
{
    uint32_t sorted[BC_CODED_ELEMENTS];
    uint32_t i;

    for (i = 0U; i < BC_CODED_ELEMENTS; i++)
    {
        uint32_t const w = p_width[i];
        uint32_t       j = i;

        while ((j > 0U) && (sorted[j - 1U] > w))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = w;
    }

    return sorted[BC_CODED_ELEMENTS / 2U];
}

/* Wide if width / median > ratio / 100.  64-bit products: the configuration
 * is a run-time input, so its range is not trusted to keep 32 bits. */
static bool bc_is_wide(uint32_t width_um, uint32_t median_um, uint32_t ratio_x100)
{
    return ((uint64_t)width_um * BC_RATIO_SCALE) > ((uint64_t)median_um * ratio_x100);
}

/* Number of wide elements (set bits) in a 9-bit character pattern. */
static uint32_t bc_wide_count(uint32_t pattern)
{
    uint32_t n = 0U;
    uint32_t e;

    for (e = 0U; e < BC_CHAR_ELEMENTS; e++)
    {
        n += (pattern >> e) & 1U;
    }

    return n;
}

/* Width of each of the 27 coded elements (each with its own speed, NFR9),
 * then narrow/wide by ratio to their median, packed first element first.
 * The gap after each character is skipped. */
static void bc_classify(bc_element_t const p_elem[BC_SYMBOL_ELEMENTS], uint32_t ratio_x100,
                        uint32_t p_pattern[BC_SYMBOL_CHARS])
{
    uint32_t width_um[BC_CODED_ELEMENTS];
    uint32_t median_um;
    uint32_t c;
    uint32_t e;

    for (c = 0U; c < BC_SYMBOL_CHARS; c++)
    {
        for (e = 0U; e < BC_CHAR_ELEMENTS; e++)
        {
            bc_element_t const *p_el = &p_elem[(c * BC_CHAR_STRIDE) + e];

            width_um[(c * BC_CHAR_ELEMENTS) + e] = bc_gap_to_width_um(p_el->gap_us, p_el->speed_mmps);
        }
    }

    median_um = bc_median_um(width_um);
    for (c = 0U; c < BC_SYMBOL_CHARS; c++)
    {
        p_pattern[c] = 0U;
        for (e = 0U; e < BC_CHAR_ELEMENTS; e++)
        {
            bool const wide = bc_is_wide(width_um[(c * BC_CHAR_ELEMENTS) + e], median_um, ratio_x100);

            p_pattern[c] = (p_pattern[c] << 1U) | (wide ? 1U : 0U);
        }
    }
}

void bc_cfg_default(bc_cfg_t *p_cfg)
{
    if (p_cfg != NULL)
    {
        p_cfg->wide_ratio_x100 = BC_WIDE_RATIO_X100;
        p_cfg->min_speed_mmps = BC_MIN_SPEED_MMPS;
    }
}

bc_result_t bc_decode(bc_element_t const p_elem[], uint32_t count,
                      bc_cfg_t const *p_cfg, uint32_t *p_cmd)
{
    uint32_t    pattern[BC_SYMBOL_CHARS] = { 0U, 0U, 0U };
    uint32_t    cmd = 0U;
    uint32_t    i;
    bc_result_t result = BC_OK;

    if ((p_elem == NULL) || (p_cfg == NULL) || (p_cmd == NULL) || (count != BC_SYMBOL_ELEMENTS))
    {
        result = BC_ERR_COUNT;
    }

    /* Every element, the gaps between characters included, needs a usable
     * speed: the car stalling mid-read invalidates the read (design.md 6.2). */
    for (i = 0U; (result == BC_OK) && (i < BC_SYMBOL_ELEMENTS); i++)
    {
        int32_t const v = p_elem[i].speed_mmps;

        if ((v <= 0) || ((uint32_t)v < p_cfg->min_speed_mmps))
        {
            result = BC_ERR_STALL;
        }
    }

    /* Code 39 structure: 3 wide of 9 in every character (NFR8). */
    if (result == BC_OK)
    {
        bc_classify(p_elem, p_cfg->wide_ratio_x100, pattern);
        for (i = 0U; i < BC_SYMBOL_CHARS; i++)
        {
            if (bc_wide_count(pattern[i]) != BC_WIDE_PER_CHAR)
            {
                result = BC_ERR_PATTERN;
            }
        }
    }

    if ((result == BC_OK)
        && ((pattern[BC_CHAR_START] != BC_PATTERN_START_STOP) || (pattern[BC_CHAR_STOP] != BC_PATTERN_START_STOP)))
    {
        result = BC_ERR_START_STOP;
    }

    /* Only A-D are commands (FR9); every other character is rejected. */
    if (result == BC_OK)
    {
        result = BC_ERR_NOT_CMD;
        for (i = 0U; i < BC_COMMANDS; i++)
        {
            if (pattern[BC_CHAR_CMD] == s_cmd_pattern[i])
            {
                cmd = s_cmd_value[i];
                result = BC_OK;
            }
        }
    }

    if (result == BC_OK)
    {
        *p_cmd = cmd;
    }

    return result;
}

uint32_t bc_gap_to_width_um(uint32_t gap_us, int32_t speed_mmps)
{
    uint32_t width_um = 0U;

    if (speed_mmps > 0)
    {
        /* Clamped inputs: 0xFFFFF x 511 = 535 821 825 < 2^32. */
        uint32_t const gap = (gap_us > DREP_EDGE_GAP_MAX) ? DREP_EDGE_GAP_MAX : gap_us;
        uint32_t const speed = (speed_mmps > DREP_MS_SPEED_X10_MAX)
                               ? (uint32_t)DREP_MS_SPEED_X10_MAX : (uint32_t)speed_mmps;

        width_um = (gap * speed) / BC_US_MMPS_PER_UM;
    }

    return width_um;
}
