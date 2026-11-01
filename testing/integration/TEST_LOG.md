# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results, and **`INTEGRATION_PHASES.md`** for the step-by-step pairwise integration guide and code contracts.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | Line-Following steering into Vehicle Controller and Motion Control | IF-01 (`MBF_NAV_IN`), IF-02 (`MBF_MOTION_CMD`), IF-07 (`EVF_NAV_LINE_FOUND`) | auto / manual | In `NAV_LINE_TRACKING`, S3 posts `drep_nav_steer(err_q7)` at 5 ms intervals to `MBF_NAV_IN`; inspect `motion_cmd_t` and wheel duties | VC forwards `DREP_MC_STEER` to Motion; positive `err_q7` steers right; stopping commands for > `MOTION_T_HOLD_MS` (100 ms) halts both motors |
| INT-02 | Barcode Decode junction manoeuvre and distance gating | IF-01 (`MBF_NAV_IN`), IF-08 (`MTX_MOTION`, `motion_state_t`) | auto / manual | S3 reads cumulative distance via `motion_get_state()` under `MTX_MOTION` and posts `drep_nav_barcode(DREP_BC_LEFT/RIGHT/UTURN/STOP)` to `MBF_NAV_IN` | VC transitions to `VC_SYS_NAV_TURN_LEFT` / `RIGHT` / `U_TURN` / `READY_WAITING` and posts the corresponding `motion_cmd_t` to `MBF_MOTION_CMD` |
| INT-03 | Obstacle Avoidance override and handoff back to Line-Following | IF-01 (`MBF_NAV_IN`), IF-07 (`EVF_NAV_AVOID_ACTIVE`, `EVF_NAV_LINE_REACQUIRED`) | auto / manual | S5 sets `EVF_NAV_AVOID_ACTIVE` and posts `drep_nav_avoid(bypass, seq)` while S3 posts `drep_nav_steer()`; then S5 sets `EVF_NAV_LINE_REACQUIRED` and posts `DREP_BYPASS_RELEASE` | VC switches authority to `VC_AUTH_AVOID` (`VC_SYS_OBSTACLE_AVOIDANCE`), ignores S3 steering during bypass, and restores `VC_AUTH_LINE` (`VC_SYS_NAV_LINE_TRACKING`) on release |
| INT-04 | Emergency brake and safety fault latch / stationary reset | IF-06 (`EVF_SAFETY`), IF-11 (`motion_emergency_brake()`), FR12 | auto / manual | S4 Safety task calls `motion_emergency_brake()` and sets `EVF_SAFETY_FAULT_LATCHED` while driving; S1 posts remote `RESET` while moving, then after 500 ms (`SAFETY_T_STILL_MS`) stationary | Motors brake immediately (all 4 channels LOW); VC latches `VC_SYS_SAFETY_FAULT_LATCHED` and rejects drive commands; `EVF_SAFETY_FAULT_LATCHED` clears only after 500 ms at zero speed |
| INT-05 | Telemetry remote commands, state events, and motion telemetry | IF-01 (`MBF_NAV_IN`), IF-04 (`MBF_TELEMETRY`), IF-08 (`MTX_MOTION`) | auto / manual | S1 posts remote `START` and `STOP` to `MBF_NAV_IN`, drains `tel_evt_t` (`DREP_TEL_STATE`) from `MBF_TELEMETRY`, and reads `motion_get_state()` + `vc_get_system_state()` | System transitions `READY_WAITING` -> `NAV_LINE_TRACKING` -> `READY_WAITING`; `DREP_TEL_STATE` events match `vc_get_system_state()`; `motion_state_t` fields pack cleanly into `telemetry_motion` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
