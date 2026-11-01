/*
 *  bc_decode.h
 *  Subsystem 3 -- Code 39 barcode decoder core (pure C).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Pure module: includes only <stdint.h>, <stdbool.h> and pure project
 *  headers, never a kernel or SDK header and never subsystem3.h (check_rules
 *  R1), so the host tests (tests/host/test_bc_decode.c) build it unchanged.
 *  The Barcode Decode task (BC-2) collects the inputs and acts on the result.
 */

#ifndef BC_DECODE_H
#define BC_DECODE_H

#include <stdint.h>

/** Elements in one symbol: '*' + character + '*' (9 each) + 2 gaps between. */
#define BC_SYMBOL_ELEMENTS          (29U)

/** One bar or space as the Barcode task sees it. */
typedef struct
{
    uint32_t gap_us;        /**< how long it lasted (IF-09 gap, microseconds) */
    int32_t  speed_mmps;    /**< Motion's speed (IF-08, mm/s) published most
                                 recently when the task read this gap */
} bc_element_t;

/** Tunables, filled by bc_cfg_default() from config/thresholds.h. */
typedef struct
{
    uint32_t wide_ratio_x100;   /**< wide if width x 100 > ratio x median */
    uint32_t min_speed_mmps;    /**< any element slower: the read is a stall */
} bc_cfg_t;

/** Decode outcome.  Every value but BC_OK leads to INVALID_READ_RESET. */
typedef enum
{
    BC_OK = 0,              /**< valid *A* .. *D*: command written */
    BC_ERR_COUNT,           /**< NULL pointer, or not BC_SYMBOL_ELEMENTS elements */
    BC_ERR_STALL,           /**< an element's speed below min_speed_mmps (or <= 0) */
    BC_ERR_PATTERN,         /**< a character without exactly 3 wide of 9 */
    BC_ERR_START_STOP,      /**< first or last character is not '*' */
    BC_ERR_NOT_CMD          /**< middle character is not A, B, C or D */
} bc_result_t;

/** Fill *p_cfg with the defaults BC_WIDE_RATIO_X100 and BC_MIN_SPEED_MMPS. */
void bc_cfg_default(bc_cfg_t *p_cfg);

/*
 * Decode one Code 39 symbol (FR9, NFR8, NFR9).
 * p_elem: count elements in crossing order, first one a bar.  Each width is
 * gap_us x speed_mmps (bc_gap_to_width_um); the 27 character elements are
 * classified wide or narrow by their ratio to the median of those 27, so a
 * constant speed cancels and a changing one is corrected element by element.
 * The 2 gaps between characters are not checked (Code 39 allows them wider).
 * On BC_OK, *p_cmd = DREP_BC_CMD_LEFT/RIGHT/STRAIGHT/UTURN for A/B/C/D;
 * otherwise *p_cmd is left unchanged.
 */
bc_result_t bc_decode(bc_element_t const p_elem[], uint32_t count,
                      bc_cfg_t const *p_cfg, uint32_t *p_cmd);

/*
 * Width of one bar or space from how long it lasted and how fast the car was
 * moving (NFR9): width_um = gap_us x speed_mmps / 1000, truncated.
 * gap_us is an IF-09 gap (saturates at DREP_EDGE_GAP_MAX); speed_mmps is the
 * IF-08 speed (drep_ms_speed_x10(), clamped to DREP_MS_SPEED_X10_MAX).  Both
 * are clamped to those limits here, so the product always fits 32 bits.
 * Returns 0 when speed_mmps <= 0 (no usable speed).
 */
uint32_t bc_gap_to_width_um(uint32_t gap_us, int32_t speed_mmps);

#endif /* BC_DECODE_H */
