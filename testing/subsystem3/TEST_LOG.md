# Subsystem 3 — test log

**Subsystem:** Line following and barcode decode  ·  **Student ID:** 2503319 (Jin Yong)  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 decoder core (`bc_decode.c`) on the host: gap x speed -> width, narrow/wide by ratio to the median, `*X*` -> A-D, and every reject | FR7, FR9, NFR8, NFR9; IF-01 payload (`DREP_BC_CMD_*`), IF-08 speed (mm/s), IF-09 gap (us) | auto (host) | `ctest -R bc_decode` (`tests/host/test_bc_decode.c`, tests 1-16). Inputs come from a barcode + physics + Motion-speed model, not from the decoder's own arithmetic; test 16 uses the bar widths measured from the official samples (`*A*`, `*F*`, `*Z*` in Barcode Sample.pdf and BARCODE.pptx) scaled to the specified 141 mm (3 mm narrow, 9 mm wide), forwards and reversed | `bc_decode: N checks, 0 failed`, ctest PASS |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S3-U01 | `0542c43` | 2503319 | PASS | `testing/subsystem3/logs/2026-11-01_s3_u01_host.txt` | Host only: 5/5 ctest, `bc_decode: 683 checks, 0 failed` (tests 1-16, incl. official `*A*`/`*F*`/`*Z*` samples forwards and reversed). Firmware gates 7/7 exit 0, 0 warnings |
