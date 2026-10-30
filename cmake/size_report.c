/**
 * @file size_report.c
 * @brief Build-time size report of the message types (Batch B stage B6).
 *
 * Compiled as the OBJECT library `size_report` with -Wall -Wextra -Werror,
 * never linked into firmware (like config_check.c). Every object below is
 * exactly as many bytes as the quantity in its name, so the symbol sizes
 * ARE the measurement:
 *
 *   arm-none-eabi-nm -S --size-sort <build>/CMakeFiles/size_report.dir/cmake/size_report.c.obj
 *
 * prints, per line, the address, the size in hex, and the name:
 *   sizeof_<type>   sizeof(type) (the "Baseline bytes" of DATA_REPRESENTATION.md)
 *   slot_<type>     bytes one message takes in a kernel message buffer
 *   mbf_<buffer>    MBF_BUFSZ of each message buffer (depth x slot)
 *   edge_ring       the IF-09 barcode edge ring (BC_EDGE_RING_LEN x bc_edge_t)
 * Pure headers only: no SDK or kernel API headers here.
 */

#include <stdint.h>

#include "config/timing.h"
#include "src/common/ipc_types.h"
#include "src/common/mbf_size.h"

#define SIZE_REPORT(name, bytes)                    \
    extern uint8_t const name[(bytes)];             \
    uint8_t const name[(bytes)] = { 0U }

SIZE_REPORT(sizeof_nav_input_t, sizeof(nav_input_t));
SIZE_REPORT(sizeof_motion_cmd_t, sizeof(motion_cmd_t));
SIZE_REPORT(sizeof_obstacle_profile_t, sizeof(obstacle_profile_t));
SIZE_REPORT(sizeof_tel_evt_t, sizeof(tel_evt_t));
SIZE_REPORT(sizeof_motion_state_t, sizeof(motion_state_t));
SIZE_REPORT(sizeof_bc_edge_t, sizeof(bc_edge_t));

SIZE_REPORT(slot_nav_input_t, MBF_SLOT_BYTES(sizeof(nav_input_t)));
SIZE_REPORT(slot_motion_cmd_t, MBF_SLOT_BYTES(sizeof(motion_cmd_t)));
SIZE_REPORT(slot_obstacle_profile_t, MBF_SLOT_BYTES(sizeof(obstacle_profile_t)));
SIZE_REPORT(slot_tel_evt_t, MBF_SLOT_BYTES(sizeof(tel_evt_t)));

SIZE_REPORT(mbf_nav_in, MBF_BUFSZ(MBF_NAV_IN_DEPTH, nav_input_t));
SIZE_REPORT(mbf_motion_cmd, MBF_BUFSZ(MBF_MOTION_CMD_DEPTH, motion_cmd_t));
SIZE_REPORT(mbf_obstacle, MBF_BUFSZ(MBF_OBSTACLE_DEPTH, obstacle_profile_t));
SIZE_REPORT(mbf_telemetry, MBF_BUFSZ(MBF_TELEMETRY_DEPTH, tel_evt_t));

SIZE_REPORT(edge_ring, BC_EDGE_RING_LEN * sizeof(bc_edge_t));
