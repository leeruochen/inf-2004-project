# Subsystem 3 — test log

**Subsystem:** Line following and barcode decode  ·  **Student ID:** 2503319 (Jin Yong)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 decoder core (`bc_decode.c`) on the host: gap x speed -> width, narrow/wide by ratio to the median, `*X*` -> letter A-Z read in either direction (no movement mapping: the four movement letters are set at the demo over MQTT, module clarification 1 Nov 2026), and every reject | FR7, FR9 (decode part), NFR8, NFR9; IF-08 speed (mm/s), IF-09 gap (us) | auto (host) | `ctest -R bc_decode` (`tests/host/test_bc_decode.c`, tests 1-16). Inputs come from a barcode + physics + Motion-speed model, not from the decoder's own arithmetic; the test's Code 39 patterns come from the generator the module refers to (barcode.tec-it.com), not from the decoder's table. Test 16 uses the bar widths measured from the official samples (`*A*`, `*F*`, `*W*`, `*X*`, `*Y*`, `*Z*` in Barcode Sample.pdf and BARCODE.pptx) scaled to the specified 141 mm (3 mm narrow, 9 mm wide), forwards and reversed | `bc_decode: N checks, 0 failed`, ctest PASS; every letter decodes to itself in both directions |
| S3-U02 | Barcode capture framing (`bc_frame.c`) on the host: start gap dropped, end of capture (`BC_CAPTURE_TIMEOUT_MS` = no edge for this long; T_end = 3 x widest only after 29 elements), the 29-element window found among 29-33 elements (sheet-edge extras), exactly one window or refused, and the Barcode task's speed rule (one Motion reading per 2 ms poll shared by every gap popped in it) | FR7, NFR8, NFR9; design.md 6.2 (T_end, capture timeout, INVALID_READ_RESET causes); IF-08 speed, IF-09 gap | auto (host) | `ctest -R bc_frame` (`tests/host/test_bc_frame.c`, tests 1-14). Captures simulated with the S3-U01 generator (`tests/host/bc_fixture.h`) and the task's poll rule: A-Z both directions with 29/30/31/33 elements, official samples, 60 and 50 mm/s, ramps, bars +-0.5 mm, an ambiguous *D*/*I* capture, partial passes, too long, overflow, stalled and reversed speeds, 0.2 mm glitches, 50-200 ms task delays, timer wrap | `bc_frame: N checks, 0 failed`, ctest PASS; never a wrong letter |
| S3-U03 | GP27 barcode edge path, no barcode: edge handler registered, Barcode task running, quiet over white and black, one edge per strip boundary | IF-09 (edge ring), FR7; `config/pins.h` IR_BC_PIN = GP27 | auto (on-target, operator-guided) | APP_TEST build, car on the stand, follow the `[S3] >>>` prompts (`.agents/tasks/s3-bc2a-test.md` step 4): 10 s still over white, 10 s still over black tape, 5 slides of a 20 mm black strip | 4 `[TEST] S3-U03 PASS` lines: (a) handler + task, (b) 0 edges over white, (c) 0 edges over black and the level differs, (d) exactly 10 edges for 5 strips. The white/black levels are printed (which level is a bar) |
| S3-U04 | Barcode read end to end (GP27 ISR -> ring -> task -> `bc_frame` -> `bc_decode`) with hand-slid official cards, both directions, at a fixed bench speed (APP_TEST speed override 300 mm/s: the wheels do not turn on the stand) | FR7, NFR8, NFR9 (ratio decision with a constant speed), IF-09 | auto (on-target, operator-guided) | Car on the stand; cards *A*, *Z*, *F* printed at 141 mm (3 mm narrow); per card 10 slides left-to-right then 10 right-to-left in 60 s windows | 6 `[TEST] S3-U04 PASS` lines: per window >= 9 correct letters, 0 wrong letters, 0 ring overflow; every capture logged with its raw gaps |
| S3-U05 | Barcode read while driving, Motion's real speed paired with every gap (IF-08), forward and reverse crossings | FR7, NFR9, IF-08, IF-09 | auto (on-target, operator-guided) | Car on the floor, card flat 15 cm ahead of the sensor; the test drives `motion_move_forward(40)` then `motion_move_backward(40)` 5 times per card; cards *A*, *Z*, *F*, *W*, *X*, *Y* | 6 `[TEST] S3-U05 PASS` lines: per card the drives complete, >= 9 of 10 correct, 0 wrong, 0 overflow |
| S3-U06 | Invalid reads never give a letter and the task recovers (design.md 6.2 INVALID_READ_RESET: incomplete read, car stopped mid-read, no edges while parked) | FR7, NFR8 | auto (on-target, operator-guided) | (a) bench: slide card *A* halfway and back, 3 times; (b) bench: stop char covered with paper, 3 slides; (c) floor: drive 22 cm so the car stops on the card; (d) parked 10 s; (e) back off, then one full 40 cm pass | 5 `[TEST] S3-U06 PASS` lines: (a)-(c) no letter and at least one refused read logged with its reason; (d) no edges, no read; (e) the next full pass reads A |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S3-U01 | `0542c43` | 2503319 | PASS | `testing/subsystem3/logs/2026-11-01_s3_u01_host.txt` | Host only: 5/5 ctest, `bc_decode: 683 checks, 0 failed` (tests 1-16, incl. official `*A*`/`*F*`/`*Z*` samples forwards and reversed). Firmware gates 7/7 exit 0, 0 warnings |
| 2026-11-01 | S3-U01 | `e063471` | 2503319 | PASS | `testing/subsystem3/logs/2026-11-01_s3_u01_host_az.txt` | Revised definition (A-Z, both directions, no movement mapping). Host only: 5/5 ctest, `bc_decode: 1980 checks, 0 failed` (tests 1-16, incl. official `*A*`/`*F*`/`*W*`/`*X*`/`*Y*`/`*Z*` both ways). Firmware gates 7/7 exit 0, 0 warnings |
