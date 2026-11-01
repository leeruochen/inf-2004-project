# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results, and **`INTEGRATION_PHASES.md`** for the step-by-step lab course testing guide.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Platform: USB CDC bring-up test (Batch B stage B5, milestone M1) | Phase 1 acceptance (HANDOVER section 4); `docs/DEBUG.md` "USB CDC bring-up test" | auto (log checker) + manual (replug) | `.agents/tasks/batchB-b5-test.md`: `PLATFORM_BRINGUP=ON` build, 800 s capture (10 min + 60 s stress at 660-720 s), then terminal close/reopen and USB replug; `tools/bringup_check.ps1` on both logs | nine heartbeat tasks; no sequence gaps; no `[FAULT]`/`[ERR]`; every period within 1 ms (control tasks also during stress); drop counter rises only during stress; keeps running after reopen and replug; checker PASS on both logs |
| SYS-02 | End-to-end line-following lap on lab course | FR2, FR5, FR6, NFR1 | manual + log + video | Place car at course start; send remote `START` via MQTT; run a full lap of straights and curves; send remote `STOP` at finish | Car stays on the line for the entire lap, reports live speed/distance over MQTT (`telemetry_motion`), and stops cleanly at finish |
| SYS-03 | Barcode-guided junction turns on lab course | FR3, FR4 | manual + log + video | Place barcode cards (`LEFT`, `RIGHT`, `U_TURN`, `STOP`) before course junctions; run car along the line | Car decodes each barcode within `BC_D_CMD_CM` (30 cm), executes the commanded 90°/180° turn or stop at the junction, and sends `event_barcode` to MQTT |
| SYS-04 | Hump traversal & pitch detection on lab course | FR7, FR8 | manual + log + video | Drive car over the lab course hump/ramp during autonomous line tracking | IMU detects pitch >= 5° (`IMU_HUMP_PITCH_DEG`); VC enters `VC_SYS_NAV_HUMP_SENSING` (slow speed), traverses hump without losing the line, and sends `event_hump` |
| SYS-05 | Obstacle detection, servo scan & bypass on lab course | FR9, FR10, FR11 | manual + log + video | Place an obstacle box on the track line; run car toward the obstacle | Car stops at < 25 cm, sweeps servo (`GP15`), sends `event_obstacle`, steers around the clearer side (`VC_AUTH_AVOID`), reacquires the black line, and resumes `NAV_LINE_TRACKING` |
| SYS-06 | Collision fault latch & stationary reset | FR12, IF-06, IF-11 | manual + log + video | Trigger a collision (> 2.5 g) or run off-track (`TRACK_LOST`); attempt remote `RESET` while moving, then after 500 ms stationary | Motors brake immediately (`motion_emergency_brake()`); `event_fault` sent; moving `RESET` rejected; stationary `RESET` clears latch and returns to `READY_WAITING` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | SYS-01 | `a7393a7` | Buddy 2 (Ruo Chen) | PASS | `logs/2026-10-31_b5-bringup-run.txt`, `logs/2026-10-31_b5-bringup-replug.txt`; report on PR #5 | Batch B stage B5, milestone M1. 798 s, nine tasks, no gaps, no faults; worst \|dt − period\| 10 µs (safety) to 566 µs (tel), limit 1000 µs; stress dropped 633 lines, all inside 660-720 s, no heartbeat lost; alive difference constant (1026 ms, 79 lines); kept running with the terminal closed; restarted after replug. `bringup_check` PASS on both logs (Buddy 2, re-run by the helper) |
