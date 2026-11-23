# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Line-Following steering into Vehicle Controller and Motion Control | IF-01 (`MBF_NAV_IN`), IF-02 (`MBF_MOTION_CMD`), IF-07 (`EVF_NAV_LINE_FOUND`), NFR7 | auto / manual | In `NAV_LINE_TRACKING`, S3 samples IR at `5 ms` and posts `drep_nav_line_steer(err_q7)` at `20 ms` aligned slots to `MBF_NAV_IN`; inspect `motion_cmd_t` and wheel duties | VC forwards `DREP_MC_STEER` to Motion; positive `err_q7` steers right; stopping commands for `> MOTION_T_HOLD_MS` (`100 ms`) halts both motors |
| INT-02 | Barcode Decode junction manoeuvre, `car/cmd/bc_map` mapping, and distance gating | IF-01 (`MBF_NAV_IN`), IF-08 (`MTX_MOTION`, `motion_state_t`), IF-12 | auto / manual | S3 reads speed/distance via `motion_get_state()` under `MTX_MOTION`, decodes `'A'..'Z'`, calls `tel_post_barcode_letter()`, and posts `drep_nav_barcode(DREP_BC_CMD_LEFT/RIGHT/STRAIGHT/UTURN)` for mapped letters within `BC_D_CMD_CM` (`60 cm`) | VC transitions to `VC_SYS_NAV_TURN_LEFT` / `RIGHT` / `GO_STRAIGHT` / `U_TURN` at the junction and posts the corresponding `motion_cmd_t` to `MBF_MOTION_CMD` |
| INT-03 | Obstacle Avoidance override and handoff back to Line-Following | IF-01 (`MBF_NAV_IN`), IF-07 (`EVF_NAV_AVOID_ACTIVE`, `EVF_NAV_LINE_REACQUIRED`, `EVF_NAV_LINE_FOUND`) | auto / manual | S5 sets `EVF_NAV_AVOID_ACTIVE` and posts `drep_nav_avoid(DREP_NAV_AVOID_SLOW/BYPASS, DREP_BYPASS_*)` while S3 is active; on `EVF_NAV_LINE_FOUND`, S5 sets `EVF_NAV_LINE_REACQUIRED`, clears `AVOID_ACTIVE`, and posts `drep_nav_avoid(DREP_NAV_AVOID_RELEASE, DREP_BYPASS_STOP)` | VC switches authority to `VC_AUTH_AVOID` (`VC_SYS_OBSTACLE_AVOIDANCE`), ignores S3 steering during bypass, and restores `VC_AUTH_LINE` (`VC_SYS_NAV_LINE_TRACKING`) on release |
| INT-04 | Emergency brake and safety fault latch / stationary reset | IF-06 (`EVF_SAFETY`), IF-11 (`motion_emergency_brake()`), FR12, NFR10 | auto / manual | S4 Safety task calls `motion_emergency_brake()` and sets `EVF_SAFETY_FAULT_LATCHED` on `LIFTED` (`> 30°` or `> 0.10 m/s` for `50 ms`) or IMU watchdog timeout; S1 posts remote `RESET` while moving, then after `500 ms` (`SAFETY_T_STILL_MS`) stationary | Motors brake within `200 ms` (all 4 channels LOW); VC latches `VC_SYS_SAFETY_FAULT_LATCHED` and rejects drive commands; `EVF_SAFETY_FAULT_LATCHED` clears only after `500 ms` at zero speed |
| INT-05 | Telemetry remote commands, `car/cmd/bc_map`, state events, snapshots, and motion telemetry | IF-01 (`MBF_NAV_IN`), IF-04 (`MBF_TELEMETRY`), IF-08 (`MTX_MOTION`), IF-12 | auto / manual | S1 posts remote `START` / `STOP` / `RESET` to `MBF_NAV_IN`, applies framed non-retained `car/cmd/bc_map` (`DREP_FT_CMD_BC_MAP = 11`), drains `tel_evt_t` (`HUMP`, `BARCODE`, `FAULT`, `STATE`), reads IF-12 snapshots (`IR`, `range`, `attitude`, `obstacle`) + `motion_get_state()` + `vc_get_system_state()` | System transitions `READY_WAITING` -> `NAV_LINE_TRACKING` -> `READY_WAITING`; `car/cmd/bc_map` updates `bc_task_set_map()`; `DREP_TEL_STATE` emits single buffered heartbeats; `telemetry_motion`/`sensor` (`ir_valid`/`range_valid`/`att_valid`) and event frames decode cleanly on dashboard |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|

## Current status (as of 22 Nov 2026)

Live track runs over Wi-Fi/MQTT and USB console have exercised `INT-01..INT-05` end to end (see dashboard Demo Rubric captures and `testing/subsystem5/logs/2026-11-19_s5_fullchain.txt`). Formal console logs in `testing/integration/logs/` and unlisted YouTube links in `videos/README.md` still need to be saved and committed as PASS rows above before hand-in.
