/*
 *  lf_straddle.h
 *  Subsystem 3 -- EXPERIMENT (branch buddy3/lf-straddle, not for merge until
 *  the car shows it works): straddle line following, a second core behind
 *  the same inputs and outputs as lf_core.h (lf_in_t, lf_out_t, lf_state_t,
 *  lf_event_t), so task_lf.c, the Vehicle Controller (IF-01), LINE_FOUND
 *  (IF-07) and telemetry see no difference.  Built in with -DS3_LF_STRADDLE=ON
 *  (sources.cmake); the edge-following core stays the default.
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *
 *  Pure module (check_rules R1): <stdint.h>, <stdbool.h> and pure project
 *  headers only, so tests/host/test_lf_straddle.c builds it unchanged.
 *
 *  Mounting.  The two existing sensors sit one each side of the car's
 *  centreline, the same distance (LF_D_PIVOT_MM) ahead of the axle:
 *    the CENTRE module (IR_C, GP28) becomes LEFT; RIGHT (IR_R, GP26) stays
 *    RIGHT.  In lf_in_t, raw_c is LEFT and raw_r is RIGHT: no wiring change.
 *  Spacing S, centre to centre, against the 18 mm line:
 *    - S = 26 mm (today's bracket, moved 13 mm to the car's left): both read
 *      white when the car is centred, with a dead band of about
 *      (26 - 18) / 2 - spot = +-2.8 mm each side;
 *    - S = 16-18 mm (each sensor over one edge of the line; stagger the two
 *      boards along the car if they are too wide to sit side by side): both
 *      read part-dark when centred, so "both white" only ever means "off the
 *      line".  Preferred where the boards allow it.
 *
 *  Steering, every 5 ms sample, darkness dl (LEFT) and dr (RIGHT) 0..1000:
 *    - contact (either >= LF_LIGHT_MAX_PM): steer = LF_SD_KP_Q7 x (dr - dl)
 *      / 1000 (> 0 = right: the darker side is the side the line is on);
 *    - both DARK for LF_SD_JCT_N confirmed samples: junction, handled as in
 *      lf_core (travel LF_D_JCT_POST_MM, post JUNCTION; the Vehicle
 *      Controller turns on a pending barcode command or goes straight).
 *
 *  Both white is the limit of two straddling sensors: "centred between them"
 *  and "the line is gone" read exactly the same, so no single sample can
 *  tell them apart.  This core never assumes either for long:
 *    - after a shallow contact (the darker sensor never reached
 *      LF_DARK_MIN_PM) the line went back between the sensors: steer 0, but
 *      only for LF_SD_CONF_MM of travel; then PROBE: steer LF_SD_PROBE_Q7
 *      towards the side last touched for at most LF_SD_PROBE_MM.  Centred,
 *      that sensor touches the line within a few mm; gone, LINE_LOST.
 *    - after a deep one-sided contact (the line's body under one sensor, the
 *      other white: the car at least ~6 mm off) the line most likely went on
 *      past that sensor (a bend): HOLD towards it at LF_SD_HOLD_Q7 for at
 *      most LF_SD_HOLD_MM.  If it had come back between the sensors after
 *      all, the other sensor touches it within a few mm and the
 *      proportional rule takes over.  Then LINE_LOST.
 *  So blind travel is bounded (at most LF_SD_CONF_MM + LF_SD_PROBE_MM, or
 *  LF_SD_HOLD_MM, then the bounded search) and a long, well-centred straight
 *  costs one gentle probe every LF_SD_CONF_MM, not a false stop.
 *
 *  LINE_LOST and the search are as in lf_core (stop on Motion's hold, arc
 *  towards the last side, swap, T_search; LF_D_SEARCH_MAX_MM after a loss
 *  while driving), the line counting as found when either sensor touches it.
 *  A sensor at an ADC rail (lf_sanity.h ADC_STUCK_*) for LF_SD_FAULT_N
 *  samples while running is a fault: TRACK_LOST, so the VC stops the car.
 */

#ifndef LF_STRADDLE_H
#define LF_STRADDLE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_program/subsystem3/lf_core.h"

/* ---------------------------------------------------------------------------
 * Tunables (host model first, then the car).  #ifndef only so the host
 * studies can sweep them; the firmware uses these values.
 * ------------------------------------------------------------------------- */

#ifndef LF_SD_KP_Q7
/** Steering for one sensor fully on black and the other white, Q7.  Full
 *  steering: on the car (20 Nov, the track's U-bend) 96 overshot and lost the
 *  line, and slowing the car did not help in the host model (below ~100 mm/s
 *  the U-bend failed at every radius).  Host model, U-bend R 130-180 over
 *  three motor models: Kp 64 -> 8/108 passes, 80 -> 38, 96 -> 70, 127 -> 80;
 *  with the hold at 127 too: U-bends 117/120 at the fitted model, 90 degree
 *  bends 119/120, straights clean. TUNE */
#define LF_SD_KP_Q7             (127)
#endif
#ifndef LF_SD_HOLD_Q7
/** Steering held towards the side of a deep contact once both are white
 *  (see LF_SD_KP_Q7). TUNE */
#define LF_SD_HOLD_Q7           (127)
#endif
#ifndef LF_SD_HOLD_MM
/** The hold lasts at most this much travel, then LINE_LOST. TUNE */
#define LF_SD_HOLD_MM           (150U)
#endif
#ifndef LF_SD_HOLD_DEG
/** ... or until the car has turned this far since the hold began.  A line
 *  END reads exactly like a sharp bend (the line vanishes past one sensor);
 *  bounded only by distance, the host model's car swung 78 degrees and
 *  stopped 36 cm from a line end at 150 mm/s.  A real bend is caught again
 *  well inside this. TUNE */
#define LF_SD_HOLD_DEG          (60U)
#endif
#ifndef LF_SD_FLIP
/** 0 (off; no gain in the host model fitted to the car, below).  1: when the hold ends without finding the line, steer the OTHER way for
 *  the probe distance before declaring the line lost.  At the end of a bend
 *  the line is behind, on the other side, and the hold has turned the car
 *  away from it (20 Nov, new track: 60 degrees left, then lost).  Host model
 *  fitted to the car (steering dead zone 56, gain 1.19), 7 cm/s, bends of
 *  45-180 degrees at R 110-300: hold 60 degrees with or without it 104/160;
 *  hold 30 degrees 94 -> 97; hold 45 degrees 104.  Kept off. TUNE */
#define LF_SD_FLIP              (0)
#endif
#ifndef LF_SD_FLIP_Q7
/** Steering of that reverse probe, Q7: above the car's dead zone, where small
 *  steering does nothing (host model fitted to the car: ~56 of 127). */
#define LF_SD_FLIP_Q7           (96)
#endif
#ifndef LF_SD_COMP_DZ
/** EXPERIMENT: steering dead-zone compensation, Q7.  The car ignores the
 *  first ~56 of 127 steering units at 7 cm/s (fitted from two measurements:
 *  127 turned it 0.46 deg/mm, 96 only 0.26), so small corrections do nothing
 *  and the next big one overshoots (the wobble).  When non-zero, any steering
 *  is lifted past that: out = dz + |s| (127 - dz) / 127, same sign.  0 = off.
 *  It works round Motion's speed controller (Buddy 2's code; the real fix is
 *  per-wheel targets there) and depends on the speed.  Host model: offset
 *  starts failing 6/18 -> 0/18, bends and the wobble unchanged.  Kept off. */
#define LF_SD_COMP_DZ           (0)
#endif
#ifndef LF_SD_CONF_MM
/** Both white after a shallow contact: straight on for this much travel.
 *  Host model: 250 mm drove 19 cm past a line end before the probe; 120 mm
 *  keeps every straight clean and probes about every 33 cm on a perfectly
 *  trimmed one (the car's measured drift touches a sensor sooner). TUNE */
#define LF_SD_CONF_MM           (120U)
#endif
#ifndef LF_SD_PROBE_Q7
/** Then the probe towards the side last touched, Q7. TUNE */
#define LF_SD_PROBE_Q7          (24)
#endif
#ifndef LF_SD_PROBE_MM
/** The probe lasts at most this much travel, then LINE_LOST.  Centred, a
 *  sensor touches the line within ~20 mm of it (host model). TUNE */
#define LF_SD_PROBE_MM          (60U)
#endif
#ifndef LF_SD_DEEP_PM
/** A contact is "deep" (the line's body under one sensor, the other white)
 *  when the near sensor reads at least this ... */
#define LF_SD_DEEP_PM           (950U)
#endif
#ifndef LF_SD_DEEP_MM
/** ... for at least this much travel.  A brief touch of the edge on a
 *  straight is not a bend: with a looser test the host model held the turn
 *  after every touch and zig-zagged +-4 mm down a straight. TUNE */
#define LF_SD_DEEP_MM           (6U)
#endif
#ifndef LF_SD_OVER_RAW
/** A reading above this is not the tape: it is the floor beyond the mat (or a
 *  lifted car).  Tape peaks seen on the car, 4 logs and 3 mounts, 19-20 Nov:
 *  2249-2706 raw; the floor read 3179 at first contact and 3500-3950 after
 *  (20 Nov, 01:58, both sensors for 1.7 m).  An absolute ADC level, because
 *  calibrated black does not scale with it (2520 on a black of 1178). TUNE */
#define LF_SD_OVER_RAW          (3000U)
#endif
#ifndef LF_SD_OVER_N
/** Either sensor above LF_SD_OVER_RAW for this many samples (50 ms): the
 *  car is off the track: TRACK_LOST, the VC stops it. */
#define LF_SD_OVER_N            (10U)
#endif
#ifndef LF_SD_BOTHDARK_MM
/** Both sensors DARK outside a junction for this much travel is not a line
 *  (a cross line is 18 mm wide and the junction travel already covers it):
 *  LINE_LOST.  Without it both-DARK sat in ON_LINE with steering 0 for
 *  1.7 m (20 Nov). */
#define LF_SD_BOTHDARK_MM       (60U)
#endif
#ifndef LF_SD_MEM_Q7
/** A graze while the car has been steering one way: the steering averaged
 *  over LF_SD_MEM_MM of travel is at least this (Q7) -> treat it as a bend:
 *  hold that way (as after a deep contact) instead of going straight.  A
 *  weave on a straight averages near 0. TUNE */
#define LF_SD_MEM_Q7            (40)
#endif
#ifndef LF_SD_MEM_MM
#define LF_SD_MEM_MM            (60U)
#endif
#ifndef LF_SD_JCT_N
/** Both confirmed DARK for this many samples is a junction (30 ms). TUNE */
#define LF_SD_JCT_N             (6U)
#endif
/** A sensor at an ADC rail this many samples while running is a fault (200 ms). */
#define LF_SD_FAULT_N           (40U)

/** The distances in encoder counts (ENC_QUAD_MULT x ENC_CPR per WHEEL_CIRC_MM). */
#define LF_SD_HOLD_CNT          ((LF_SD_HOLD_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
#define LF_SD_CONF_CNT          ((LF_SD_CONF_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
#define LF_SD_PROBE_CNT         ((LF_SD_PROBE_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
#define LF_SD_DEEP_CNT          ((LF_SD_DEEP_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
#define LF_SD_BOTHDARK_CNT      ((LF_SD_BOTHDARK_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
#define LF_SD_MEM_CNT           ((LF_SD_MEM_MM * ENC_QUAD_MULT * ENC_CPR) / WHEEL_CIRC_MM)
/** LF_SD_HOLD_DEG as left-minus-right counts, as LF_PIVOT_HALF_CNT is 45 deg:
 *  track 114 mm x 2464 / 207 counts per mm = 1357 counts per radian. */
#define LF_SD_HOLD_YAW_CNT      ((LF_SD_HOLD_DEG * LF_PIVOT_HALF_CNT) / 45U)

/* ---------------------------------------------------------------------------
 * Types
 * ------------------------------------------------------------------------- */

/** What the core believes while both sensors are white. */
typedef enum
{
    LF_SD_BW_NONE = 0,      /**< a sensor is touching the line */
    LF_SD_BW_CENTRED,       /**< after a shallow contact: straight on */
    LF_SD_BW_PROBE,         /**< checking it: a gentle turn towards the last side */
    LF_SD_BW_HOLD           /**< after a deep contact: keep turning towards it */
} lf_sd_bw_t;

/** Core state.  Private: fields are read only by lf_straddle.c (and tests). */
typedef struct
{
    lf_cal_t       cal;
    bool           cal_ok;
    lf_state_t     state;
    lf_zone_filt_t zl;              /**< LEFT zone filter */
    lf_zone_filt_t zr;              /**< RIGHT zone filter */
    bool           prev_running;
    uint32_t       prev_fwd;        /**< fwd_cnt at the previous sample */
    uint32_t       timer_n;         /**< samples in the current state */
    bool           post_now;        /**< post at the next sample that may (state entry) */
    int32_t        steer_q7;        /**< last computed steering */
    lf_side_t      side;            /**< the side last touched (the darker sensor) */
    bool           in_contact;      /**< a sensor touched the line last sample */
    bool           deep;            /**< this contact went one-sided deep (LF_SD_DEEP_*) */
    uint32_t       deep_cnt;        /**< forward counts of it so far */
    lf_sd_bw_t     bw;              /**< belief while both are white */
    uint32_t       bw_cnt;          /**< forward counts since the last contact */
    int32_t        hold_yaw0;       /**< yaw_cnt when the hold began */
    bool           flipped;         /**< the hold failed and the reverse probe has begun */
    bool           jct_armed;       /**< not both DARK since the last junction */
    uint32_t       jct_n;           /**< consecutive both-DARK confirmed samples */
    bool           jct_pending;     /**< junction detected, JUNCTION not posted yet */
    uint32_t       jct_cnt0;        /**< fwd_cnt at detection */
    lf_manoeuvre_t man_seen;        /**< manoeuvre the VC ran after the junction */
    bool           lost_driving;    /**< this LINE_LOST came while driving the line */
    bool           srch_capped;     /**< this search ends at LF_D_SEARCH_MAX_CNT */
    uint32_t       srch_cnt;        /**< its forward counts so far */
    uint32_t       fault_n;         /**< consecutive samples with a sensor at a rail */
    uint32_t       over_n;          /**< consecutive samples with a sensor above LF_SD_OVER_RAW */
    uint32_t       bd_cnt;          /**< forward counts with both sensors DARK in ON_LINE */
    int32_t        mem_q15;         /**< steering averaged over ~LF_SD_MEM_MM of travel, Q7 x 256 (an integer
                                     *   average of Q7 values stalls 59 short of its target) */
} lf_sd_t;

/* ---------------------------------------------------------------------------
 * Functions
 * ------------------------------------------------------------------------- */

/*
 * Set up the core.  p_cal NULL, or a calibration without LF_MIN_SPAN_RAW on
 * either sensor: the core never steers, and reports TRACK_LOST once at run
 * start (as lf_core_init).
 */
void lf_sd_init(lf_sd_t *p_sd, lf_cal_t const *p_cal);

/* One 5 ms sample: inputs in, decision out (all of *p_out written). */
void lf_sd_step(lf_sd_t *p_sd, lf_in_t const *p_in, lf_out_t *p_out);

#endif /* LF_STRADDLE_H */
