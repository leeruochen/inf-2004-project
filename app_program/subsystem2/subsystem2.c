/*
 *  subsystem2.c
 *  Subsystem 2 -- motion control and Vehicle Controller: creates its
 *  objects, starts its tasks, and provides the public motion API.
 *  Owner: Buddy 2, Student ID: 2500530 (Ruo Chen).
 */

#include "subsystem2.h"
#include "config/thresholds.h"
#include "drv_encoder.h"    /* kernel-free driver interface: enc_get_count() */
#include "drv_motor.h"      /* kernel-free driver interface: motor_brake_all() */
#include "task_motion.h"
#include "task_vc.h"

static bool s_pre_start_brake_ok = false;

static ER send_motion_api_cmd(motion_cmd_t cmd)
{
    return tk_snd_mbf(MBF_MOTION_CMD, &cmd, (INT)sizeof(cmd), TMO_POL);
}

void motion_emergency_brake(void)
{
    motor_brake_all();
}

bool subsystem2_pre_start_brake_ok(void)
{
    return s_pre_start_brake_ok;
}

ER motion_move_forward(int32_t distance_cm)
{
    return send_motion_api_cmd(drep_mc_pack(DREP_MC_FWD,
                                            (int32_t)MOTION_SPEED_CRUISE_CMPS,
                                            distance_cm));
}

ER motion_move_backward(int32_t distance_cm)
{
    return send_motion_api_cmd(drep_mc_pack(DREP_MC_BACK,
                                            (int32_t)MOTION_SPEED_BYPASS_CMPS,
                                            distance_cm));
}

ER motion_turn_left(int32_t angle_deg)
{
    return send_motion_api_cmd(drep_mc_pack(DREP_MC_TURN_L,
                                            (int32_t)MOTION_SPEED_SLOW_CMPS,
                                            angle_deg));
}

ER motion_turn_right(int32_t angle_deg)
{
    return send_motion_api_cmd(drep_mc_pack(DREP_MC_TURN_R,
                                            (int32_t)MOTION_SPEED_SLOW_CMPS,
                                            angle_deg));
}

ER motion_stop(void)
{
    return send_motion_api_cmd(drep_mc_pack(DREP_MC_STOP, 0, 0));
}

int32_t subsystem2_init(void)
{
    /* Verify and enforce all four motor channels LOW before motor_init()
     * and before the Motion task starts (IF-11, S2-U06).
     * Motion Control uses app_period_wait() (MOTION_PERIOD_MS, 20 ms,
     * HANDOVER section 3 step 7), so no cyclic handler is created. */
    motion_emergency_brake();
    s_pre_start_brake_ok = motor_are_all_braked();
    vc_init();
    return (int32_t)E_OK;
}

int32_t subsystem2_start(void)
{
    int32_t const vc_id = app_start_task(vc_task_entry,
                                         TASK_PRI_VC,
                                         TASK_STACK_VC_BYTES,
                                         "vc");
    if (vc_id < (int32_t)E_OK)
    {
        return vc_id;
    }

    int32_t const mot_id = app_start_task(motion_task_entry,
                                          TASK_PRI_MOTION,
                                          TASK_STACK_MOTION_BYTES,
                                          "motion");
    if (mot_id < (int32_t)E_OK)
    {
        return mot_id;
    }

    return (int32_t)E_OK;
}
