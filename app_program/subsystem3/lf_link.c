/*
 *  lf_link.c
 *  Subsystem 3 -- line-following link to Motion Control's published state
 *  (see lf_link.h).
 *  Owner: Buddy 3, Student ID: 2503319 (Jin Yong).
 *  Pure C: no kernel or SDK header (check_rules R1).
 *
 *  Integer widths: a count change is taken as the unsigned difference of
 *  two int32 counts, cast back to int32, so the free-running counts may
 *  wrap.  Per sample the wheels move a few counts (at 50 cm/s, the top
 *  speed, about 30 counts in 5 ms), so the sum of the two changes never
 *  comes near the int32 limits.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config/timing.h"
#include "app_program/subsystem3/lf_core.h"
#include "app_program/subsystem3/lf_link.h"

/* Sample counts (5 ms each). */
#define LF_N_TURN_STILL     LF_SAMPLES(LF_T_TURN_STILL_MS)     /* 12   = 60 ms */
#define LF_N_TURN_MAX       LF_SAMPLES(LF_T_TURN_MAX_MS)       /* 1000 = 5 s   */

#define LF_ENC_LEFT         (0U)
#define LF_ENC_RIGHT        (1U)

_Static_assert(LF_T_TURN_STILL_MS > MOTION_PERIOD_MS,
               "stillness spans more than one Motion publication");
_Static_assert(LF_N_TURN_STILL < LF_N_TURN_MAX, "still rule before the cap");
_Static_assert((MOTION_PERIOD_MS > LF_SAMPLE_PERIOD_MS)
               && ((MOTION_PERIOD_MS % LF_SAMPLE_PERIOD_MS) == 0U),
               "a whole number of samples per Motion period: one post slot each");

static int32_t count_change(int32_t now, int32_t before)
{
    return (int32_t)((uint32_t)now - (uint32_t)before);
}

void lf_link_init(lf_link_t *p_link)
{
    if (p_link != NULL)
    {
        p_link->primed = false;
        p_link->prev[LF_ENC_LEFT] = 0;
        p_link->prev[LF_ENC_RIGHT] = 0;
        p_link->fwd_cnt = 0U;
        p_link->half = 0U;
        p_link->man = LF_MAN_NONE;
        p_link->moved = false;
        p_link->done = false;
        p_link->still_n = 0U;
        p_link->turn_n = 0U;
    }
}

/* Forward travel: the mean of the two wheels' changes, when positive.  An
 * odd sum leaves half a count, carried to the next sample. */
static void odometer(lf_link_t *p_link, int32_t dl, int32_t dr)
{
    int32_t const sum = dl + dr + (int32_t)p_link->half;

    if (sum > 0)
    {
        p_link->fwd_cnt += (uint32_t)sum / 2U;
        p_link->half = (uint32_t)sum % 2U;
    }
    else
    {
        p_link->half = 0U;
    }
}

/* Turn-done detector: returns the manoeuvre the core should see. */
static lf_manoeuvre_t turn_watch(lf_link_t *p_link, lf_manoeuvre_t vc_man, bool changed)
{
    if (vc_man != p_link->man)
    {
        /* The VC started (or ended, or swapped) a manoeuvre: watch afresh. */
        p_link->man = vc_man;
        p_link->moved = false;
        p_link->done = false;
        p_link->still_n = 0U;
        p_link->turn_n = 0U;
    }

    if ((vc_man != LF_MAN_NONE) && !p_link->done)
    {
        p_link->turn_n++;
        if (changed)
        {
            p_link->moved = true;
            p_link->still_n = 0U;
        }
        else
        {
            p_link->still_n++;
        }
        if ((p_link->moved && (p_link->still_n >= LF_N_TURN_STILL))
            || (p_link->turn_n >= LF_N_TURN_MAX))
        {
            p_link->done = true;
        }
    }

    return p_link->done ? LF_MAN_NONE : vc_man;
}

void lf_link_step(lf_link_t *p_link, lf_manoeuvre_t vc_man,
                  int32_t const enc_count[2], lf_link_out_t *p_out)
{
    if ((p_link != NULL) && (enc_count != NULL) && (p_out != NULL))
    {
        bool changed = false;

        if (p_link->primed)
        {
            int32_t const dl = count_change(enc_count[LF_ENC_LEFT], p_link->prev[LF_ENC_LEFT]);
            int32_t const dr = count_change(enc_count[LF_ENC_RIGHT], p_link->prev[LF_ENC_RIGHT]);

            changed = (dl != 0) || (dr != 0);
            odometer(p_link, dl, dr);
        }
        p_link->primed = true;
        p_link->prev[LF_ENC_LEFT] = enc_count[LF_ENC_LEFT];
        p_link->prev[LF_ENC_RIGHT] = enc_count[LF_ENC_RIGHT];

        p_out->manoeuvre = turn_watch(p_link, vc_man, changed);
        p_out->fwd_cnt = p_link->fwd_cnt;
        p_out->yaw_cnt = (int32_t)((uint32_t)enc_count[LF_ENC_LEFT] - (uint32_t)enc_count[LF_ENC_RIGHT]);
    }
}

/*
 * Steering post slot.  Motion drains MBF_MOTION_CMD once per period, at its
 * release, and publishes that release time as tick_ms; its next reads are
 * tick_ms + k x MOTION_PERIOD_MS.  A post made in the slot (the last sample
 * before a read) waits 1..5 ms for Motion instead of 0..20 ms, so Motion
 * acts on steering a fixed, short time after the sensors saw the line.
 *
 * age = now - tick, as unsigned, so the kernel time may wrap.  Cast to
 * int32: negative means Motion published after this sample's time was read
 * (it preempted the task), so it has just read the queue: not the slot.
 * age % 20 = time since Motion's latest read on its schedule (a stale
 * tick_ms still predicts it); the slot is when that leaves 1..5 ms to the
 * next read.  At age % 20 == 0 Motion is due now: with its higher priority
 * it has read already (or reads before this post).  The 4 samples of each
 * period fall on 4 different remainders 5 apart, so exactly one is the slot.
 */
bool lf_link_post_slot(uint32_t motion_tick_ms, uint32_t now_ms)
{
    uint32_t const age = now_ms - motion_tick_ms;
    bool slot = false;

    if ((int32_t)age >= 0)
    {
        uint32_t const since_read = age % MOTION_PERIOD_MS;

        slot = (since_read >= (MOTION_PERIOD_MS - LF_SAMPLE_PERIOD_MS));
    }
    return slot;
}
