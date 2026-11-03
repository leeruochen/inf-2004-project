# Subsystem 3 — test log

**Subsystem:** Line following and barcode decode  ·  **Student ID:** 2503319 (Jin Yong)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 decoder core (`bc_decode.c`) on the host: gap x speed -> width, narrow/wide by ratio to the median, `*X*` -> letter A-Z read in either direction (no movement mapping: the four movement letters are set at the demo over MQTT, module clarification 1 Nov 2026), and every reject | FR7, FR9 (decode part), NFR8, NFR9; IF-08 speed (mm/s), IF-09 gap (us) | auto (host) | `ctest -R bc_decode` (`tests/host/test_bc_decode.c`, tests 1-16). Inputs come from a barcode + physics + Motion-speed model, not from the decoder's own arithmetic; the test's Code 39 patterns come from the generator the module refers to (barcode.tec-it.com), not from the decoder's table. Test 16 uses the bar widths measured from the official samples (`*A*`, `*F*`, `*W*`, `*X*`, `*Y*`, `*Z*` in Barcode Sample.pdf and BARCODE.pptx) scaled to the specified 141 mm (3 mm narrow, 9 mm wide), forwards and reversed | `bc_decode: N checks, 0 failed`, ctest PASS; every letter decodes to itself in both directions |
| S3-U02 | Barcode capture framing (`bc_frame.c`) on the host: start gap dropped, end of capture (`BC_CAPTURE_TIMEOUT_MS` = no edge for this long; T_end = 3 x widest only after 29 elements), the 29-element window found among 29-33 elements (sheet-edge extras), exactly one window or refused, and the Barcode task's speed rule (one Motion reading per 2 ms poll shared by every gap popped in it) | FR7, NFR8, NFR9; design.md 6.2 (T_end, capture timeout, INVALID_READ_RESET causes); IF-08 speed, IF-09 gap | auto (host) | `ctest -R bc_frame` (`tests/host/test_bc_frame.c`, tests 1-14). Captures simulated with the S3-U01 generator (`tests/host/bc_fixture.h`) and the task's poll rule: A-Z both directions with 29/30/31/33 elements, official samples, 60 and 50 mm/s, ramps, bars +-0.5 mm, an ambiguous *D*/*I* capture, partial passes, too long, overflow, stalled and reversed speeds, 0.2 mm glitches, 50-200 ms task delays, timer wrap | `bc_frame: N checks, 0 failed`, ctest PASS; never a wrong letter |
| S3-U08 | Line sensor ADC statistics and sanity decision (`lf_sanity.c`) on the host: min/max/average/spread, stuck low (every sample <= `ADC_STUCK_LOW_RAW`), stuck high (every sample >= `ADC_STUCK_HIGH_RAW`), noisy (spread > `ADC_NOISE_MAX_RAW`), the boundaries of each | `hal_adc_sanity_check()` contract (`drv_ir.h`, `config/thresholds.h`), FR8 (input plausibility) | auto (host) | `ctest -R lf_sanity` (`tests/host/test_lf_sanity.c`, tests 1-5), limits typed in as numbers | `lf_sanity: N checks, 0 failed`, ctest PASS |
| S3-U09 | LF-1 line sensor measurement on the car: CENTRE (GP28/ADC2) and RIGHT (GP26/ADC0) raw readings over white, line under CENTRE, line under RIGHT, both over black, and a slow sideways sweep; boot sanity check of both | FR8, NFR7 (sampled at `LF_SAMPLE_PERIOD_MS`), ARCHITECTURE.md D11; input to LF-2 thresholds and polarity | auto + measurement (on-target, operator-guided) | APP_TEST build, car on the desk, motors unpowered; follow the `[LF] >>>` prompts (`.agents/tasks/s3-lf1-test.md`): 5 s still per position (1000 samples at 5 ms), 15 s sweep logged every 50 ms | `[S3] IR_R ... sanity OK` and `IR_C ... sanity OK` at boot; 4 `[TEST] S3-U09 PASS` lines: (a) both sanity OK, (b) CENTRE and (c) RIGHT line differs from white by >= 400 raw (provisional), (d) spread <= 400 while still. The printed `[LF]` statistics and sweep are the measurement |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S3-U01 | `0542c43` | 2503319 | PASS | `testing/subsystem3/logs/2026-11-01_s3_u01_host.txt` | Host only: 5/5 ctest, `bc_decode: 683 checks, 0 failed` (tests 1-16, incl. official `*A*`/`*F*`/`*Z*` samples forwards and reversed). Firmware gates 7/7 exit 0, 0 warnings |
| 2026-11-01 | S3-U01 | `e063471` | 2503319 | PASS | `testing/subsystem3/logs/2026-11-01_s3_u01_host_az.txt` | Revised definition (A-Z, both directions, no movement mapping). Host only: 5/5 ctest, `bc_decode: 1980 checks, 0 failed` (tests 1-16, incl. official `*A*`/`*F*`/`*W*`/`*X*`/`*Y*`/`*Z*` both ways). Firmware gates 7/7 exit 0, 0 warnings |
| 2026-11-03 | S3-U09 | LF-1 working tree before commit (the LF-1 commit's code: every LF-1 source file predates the firmware build `robocar-s3-lf1-wip.uf2`, 2026-11-02 23:31) | 2503319 | FAIL | `testing/subsystem3/logs/2026-11-03_s3_lf1_run2.txt` | Desk, motors unpowered, black 3D-printed block as the line. (a) both sanity OK PASS; (b) CENTRE contrast PASS; (c) FAIL: in the "line under RIGHT" step the block sat under CENTRE (C 1152, R 144), operator placement; (d) FAIL: the block moved during "line under CENTRE" (C 104..1828). Measurements used by LF-2: white C 140 / R 144 (noise +-35), black block under CENTRE 1144-1152, black reads HIGHER than white; the sweep crossed CENTRE then RIGHT with clean edges. Rerun with correct placement still to do (no thin tape available). |
