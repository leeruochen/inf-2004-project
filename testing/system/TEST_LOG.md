# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Platform: USB CDC bring-up test (Batch B stage B5, milestone M1) | Phase 1 acceptance (HANDOVER section 4); `docs/DEBUG.md` "USB CDC bring-up test" | auto (log checker) + manual (replug) | `.agents/tasks/batchB-b5-test.md`: `PLATFORM_BRINGUP=ON` build, 800 s capture (10 min + 60 s stress at 660-720 s), then terminal close/reopen and USB replug; `tools/bringup_check.ps1` on both logs | nine heartbeat tasks; no sequence gaps; no `[FAULT]`/`[ERR]`; every period within 1 ms (control tasks also during stress); drop counter rises only during stress; keeps running after reopen and replug; checker PASS on both logs |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
