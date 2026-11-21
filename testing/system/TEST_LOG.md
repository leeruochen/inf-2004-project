# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results, and **`INTEGRATION_PHASES.md`** for the step-by-step lab course testing guide.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Platform: USB CDC bring-up test (Batch B stage B5, milestone M1) | Phase 1 acceptance (HANDOVER section 4); `docs/DEBUG.md` "USB CDC bring-up test" | auto (log checker) + manual (replug) | `.agents/tasks/batchB-b5-test.md`: `PLATFORM_BRINGUP=ON` build, 800 s capture (10 min + 60 s stress at 660-720 s), then terminal close/reopen and USB replug; `tools/bringup_check.ps1` on both logs | nine heartbeat tasks; no sequence gaps; no `[FAULT]`/`[ERR]`; every period within 1 ms (control tasks also during stress); drop counter rises only during stress; keeps running after reopen and replug; checker PASS on both logs |
| SYS-02 | End-to-end line-following lap on lab course | FR1, FR4–FR8, FR10, NFR1, NFR7 | manual + log + video | Place car at course start; send remote `START` via MQTT; run a full lap of straights and curves; send remote `STOP` at finish | Car stays on the line for the entire lap, reports live speed/distance/IR over MQTT (`telemetry_motion`, `telemetry_sensor`), and stops cleanly at finish |
| SYS-03 | Barcode-guided junction turns on lab course | FR9, NFR8, NFR9 | manual + log + video | Configure `car/cmd/bc_map` on the dashboard Barcode tab (default `A=LEFT, B=RIGHT, C=STRAIGHT, D=U-TURN`); place Code 39 cards before course junctions; run car along the line | Car decodes `'A'..'Z'` cards while moving (`BC_MIN_SPEED_MMPS = 20 mm/s`), sends `event_barcode` to MQTT, and executes the mapped LEFT/RIGHT/STRAIGHT/UTURN at the next junction within `BC_D_CMD_CM` (`60 cm`) |
| SYS-04 | Hump traversal & pitch detection on lab course | FR11, NFR11, NFR12 | manual + log + video | Drive car over the lab course hump/ramp during autonomous line tracking | IMU detects pitch `>= 3°` (`IMU_HUMP_PITCH_DEG`, calibrated `95 mm` wheelbase), Motion applies gradual hump climb assist without losing the line, no false lift fires, and `event_hump` reports height within NFR11 (`±20 %` or `±5 mm`) + peak hump |
| SYS-05 | Obstacle detection, servo scan & bypass on lab course | FR13, FR14, NFR14, NFR15 | manual + log + video | Place an obstacle box on the track line; run car toward the obstacle (`OBS_BYPASS_SLOW_BRINGUP = 0` for NFR14 timing run, or `1` at `5 cm/s` for slow bring-up demo) | Car halts at `< 25 cm`, runs stationary `5°` sweep (`50°–130°`), sends `event_obstacle` with width and clearances (`CAR_WIDTH_MM = 130 mm`), steers around the clearer side (`VC_AUTH_AVOID`), reacquires the black line (`LINE_FOUND`), and resumes `NAV_LINE_TRACKING` |
| SYS-06 | Lift/overturn fault latch & stationary reset (was collision until 11 Nov 2026; no results recorded before the change) | FR12, NFR10, IF-06, IF-11 | manual + log + video | Lift the robot (`> 30°` or `> 0.10 m/s` for `50 ms`) off the track mid-run (the demo's end-of-run signal) or tip it over; attempt remote `RESET` while moving, then after `500 ms` stationary | Motors brake within `200 ms` (`motion_emergency_brake()`); `event_fault` sent; moving `RESET` rejected; stationary `RESET` clears `FAULT_LATCHED` and returns to `READY_WAITING` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-31 | SYS-01 | `a7393a7` | Buddy 2 (Ruo Chen) | PASS | `logs/2026-10-31_b5-bringup-run.txt`, `logs/2026-10-31_b5-bringup-replug.txt`; report on PR #5 | Batch B stage B5, milestone M1. 798 s, nine tasks, no gaps, no faults; worst \|dt − period\| 10 µs (safety) to 566 µs (tel), limit 1000 µs; stress dropped 633 lines, all inside 660-720 s, no heartbeat lost; alive difference constant (1026 ms, 79 lines); kept running with the terminal closed; restarted after replug. `bringup_check` PASS on both logs (Buddy 2, re-run by the helper) |

## Current status (as of 22 Nov 2026)

- Live Wi-Fi/MQTT track runs have exercised line following, barcode decoding/junction turns, hump height reporting, obstacle scan/bypass, and pickup stop (shown on the dashboard Demo Rubric in `tel_web.py`).
- Formal saved logs in `testing/system/logs/` and unlisted YouTube links in `videos/README.md` are still needed to add official `PASS` rows for `SYS-02..SYS-06` (remember to set `OBS_BYPASS_SLOW_BRINGUP = 0U` and `MOTION_SPEED_BYPASS_* = 25U` when logging nominal `NFR14` timing for `SYS-05`).
