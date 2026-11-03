# Subsystem 3 LF-1: line sensor measurement (S3-U09)

Written 2 Nov 2026 by Buddy 3 (Jin Yong). Tests: `testing/subsystem3/TEST_LOG.md` S3-U08 (host) and S3-U09 (this run). Code: `app_program/subsystem3/drv_ir.c` (ADC), `lf_sanity.c` (sanity check), `lf_hw_test.c` (this test).

## Purpose

LF-1 only reads the two line sensors. The point of this run is to **measure** them, so that LF-2's thresholds and polarity come from real numbers:
- CENTRE = GP28 / ADC2 (Grove 7);
- RIGHT = GP26 / ADC0 (Grove 5).

Nothing steers, and the car does not move.

## Build (from the repo root, after the HANDOVER.md section 5 step 3 environment block)

```powershell
git switch buddy3/lf1-adc; git pull --ff-only; git log --oneline -1
& $CM -S . -B build/s3-lf1 @common -DDBG_BACKEND=USB -DAPP_TEST=ON
& $CM --build build/s3-lf1        # expect 0 warnings, check_rules: PASS
```

Flash `build/s3-lf1/robocar.uf2` (BOOTSEL), then capture:
`powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 600 -Out build/s3-lf1-run.txt`.

## Physical setup (measure and write down)

- Car on the desk, **battery off**: Subsystem 2's tests run first and would spin the wheels.
- A piece of the real track: its white/grey surface plus the 18 mm black line. Otherwise, black electrical tape of about 18 mm on white paper; say which.
- The black block or a wide black tape area, for "both over black".
- With a ruler:
  - each sensor's height above the surface (mm);
  - the distance between the CENTRE and RIGHT sensor centres (mm);
  - the line width (mm).
- A photo of the two sensors from the front, with the ruler.
- The GP28 VBAT jumper must be OPEN, and the Grove 6 GP26 lead taped.

## Steps (follow the `[LF] >>>` prompts after the S1/S2 tests)

| Prompt | Do | Logged |
|---|---|---|
| boot | nothing | `[S3] IR_R (GP26/ADC0) ADC sanity OK`, `[S3] IR_C (GP28/ADC2) ADC sanity OK` |
| `BOTH line sensors over WHITE` (15 s), then `measuring WHITE under both` (5 s) | white track surface under both; keep still | `[LF] sanity: ...`, S3-U09 (a); `[LF] WHITE under both: C min/avg/max ... R ...` |
| `black LINE under the CENTRE sensor only` (15 s + 5 s) | line under CENTRE, RIGHT on white; keep still | `[LF] line under CENTRE: ...` |
| `black LINE under the RIGHT sensor only` (15 s + 5 s) | line under RIGHT, CENTRE on white; keep still | `[LF] line under RIGHT: ...` |
| `BOTH sensors over black` (15 s + 5 s) | block or wide tape under both; keep still | `[LF] BLACK under both: ...` |
| `sweep next` (15 s), then `sweep` (15 s) | line just left of CENTRE, then slide it SLOWLY sideways (about 1 cm/s) past CENTRE and then past RIGHT | `[LF] sweep <ms> C=<raw> R=<raw>`, 300 lines |
| end | nothing | `[LF] polarity: ...`, S3-U09 (b), (c), (d), `[TEST] subsystem3 SUMMARY 4 passed, 0 failed` |

The PASS rules are provisional: contrast >= 400 raw, and spread <= 400 while still. **The numbers are the real result.**

## Send back

- the whole log `build/s3-lf1-run.txt` and the commit hash;
- the measured heights, sensor spacing and line width;
- the surface used, and a note on the lighting;
- the photo.

## Stop conditions

A boot `sanity FAULT`, `[S3] ... STUCK` or `NOISY` line: finish the run, then check the sensor's cable, the Grove port and the VBAT jumper. Send the log.
