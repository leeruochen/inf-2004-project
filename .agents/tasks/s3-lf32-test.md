# Subsystem 3 LF-3.2: the Line-Following task on the car (S3-U12)

Written 3 Nov 2026 by Buddy 3 (Jin Yong). Test: `testing/subsystem3/TEST_LOG.md` S3-U12. Code: `app_program/subsystem3/task_lf.c` (the task), `lf_run_test.c` (this test), plus the host-tested cores `lf_core.c` (S3-U10) and `lf_link.c` (S3-U11).

## Purpose

This is the first time the line-following code runs as a real task, next to the real Vehicle Controller and Motion tasks. The run answers these questions:

| Check | Question |
|---|---|
| (a) | Does the task sample every 5 ms without missing a period? |
| (b) | Does the boot calibration accept the start pose? |
| (c) | While the VC waits for START, does Line-Following stay silent? |
| (d1–d3) | After START, does it track (ON_LINE), post exactly one steering command per 20 ms Motion period in Motion's post slot, steer right while CENTRE is on the line body, and set LINE_FOUND? |
| (e) | With CENTRE on the line's right edge, is the steering near 0? |
| (f1, f2) | With the line gone, does it go LINE_LOST → search → one TRACK_LOST exactly 2120 ms later, and does the VC stop? |
| (g) | Does RIGHT read solid black as DARK at ride height? This is the residual risk from the S3-U09 waiver. Junctions depend on it. |
| (h) | Over the whole run: no missed period, no dropped post, no failed event? |

**The wheels do not move: the battery stays OFF for the whole run.** Motion and the VC run normally, but the motors get no power. This is safe: a stall sets only `EVF_SAFETY_FAULT_STALL`, which latches nothing.

## Build

From the repo root, after the HANDOVER.md section 5 step 3 environment block:

```powershell
git status --short                 # only untracked session files
git switch buddy3/lf3-link
git log --oneline -1               # note the hash (it goes in the report)
if (Test-Path build/s3-lf32) { Remove-Item -Recurse -Force build/s3-lf32 }
& $CM -S . -B build/s3-lf32 @common -DDBG_BACKEND=USB -DAPP_TEST=ON 2>&1 | Tee-Object build/s3-lf32-configure.txt
& $CM --build build/s3-lf32 2>&1 | Tee-Object build/s3-lf32-build.txt
Select-String -Path build/s3-lf32-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/s3-lf32-configure.txt -Pattern 'check_rules: PASS'                                         # expect a match
```

In Jin Yong's workspace the firmware is already built: `build/robocar-s3-lf32-wip-126af05.uf2` (the LF-3.2 working tree on `126af05`). The script `build/s3-lf32-run.ps1` flashes it and captures the console into `build/s3-lf32-run.txt`.

## Physical setup (measure and write down, BEFORE flashing)

1. **Battery OFF.** USB only. The wheels must stay still all through the run.
2. **The test strip.** Cut a strip of white paper about 8 cm long (the car's travel direction) and 25 cm wide. Along its middle, in the travel direction, stick an 18 mm black line: black electrical tape (18–19 mm) or a piece of the real track. Say which one you used.
3. **Lay it under the front of the car, under the two line sensors,** with the black line parallel to the car's heading. Nothing of the car (wheels, caster) may stand on the strip: it must slide sideways freely. The sensors then read it at their real ride height.
4. **Ride height.** Measure each sensor face to the paper (mm). Measure the CENTRE–RIGHT spacing, centre to centre (mm).
5. **Start pose (the boot calibration reads this pose):**
   - CENTRE over the **middle** of the black line;
   - RIGHT over white.
   Keep the strip in this pose through the flash and the first 2 minutes.
6. **Lighting:** normal room light, no direct sun. Note it.
7. **Photo:** the two sensors over the strip, from the front, with the ruler.
8. GP28 VBAT jumper OPEN. The GY-511 IMU must be connected (Grove 2). If it is missing, Safety latches a fault and steps (c)–(h) cannot run.

## Flash and capture

1. Unplug the Pico W. Hold BOOTSEL, plug it in, release BOOTSEL. `RPI-RP2` appears.
2. Copy the firmware and start the capture **at once**. The capture also shows every line on screen.

```powershell
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
Copy-Item build/s3-lf32/robocar.uf2 "$($v.DriveLetter):\"
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 900 -Out build/s3-lf32-run.txt
```

The boot calibration runs about 0.2 s after the kernel starts. **Do not touch the strip until the first `[LF] >>>` prompt.**

## Steps

Subsystems 1 and 2 run their tests first, about 1–3 minutes. With the battery off, S2's motion tests print FAIL lines. That is expected and not part of this test.

Early in the log, before the S1/S2 tests:

- `[LF] boot calibration OK: C avg ~1100 (…), R avg ~140 (…)`
- `[LF] cal white … black …`

Then `[LF] S3-U12 LF task, battery OFF: follow the [LF] >>> prompts`.

| # | Prompt / when | What the person does | Expected lines | PASS rule |
|---|---|---|---|---|
| 1 | (automatic, 1 s) | nothing | `[LF] 1 s: 200 samples, 0 missed so far`, `[TEST] S3-U12 PASS (a) …`, `[LF] boot cal: status 0 …`, `(b)` PASS | 195–205 samples, 0 missed; cal status 0 |
| 2 | (automatic, 1 s) | nothing | `(c)` PASS | no posts, no events, IDLE |
| 3 | `[LF] >>> START POSE …`, then `keep still: START comes next …` (10 s) | Check the start pose again: CENTRE on the middle of the line, RIGHT on white. Touch nothing. | `[LF] IDLE -> ON_LINE …`, `[LF] started: st=1 vc=4 …`, `[LF] after 2 s: …`, `[LF] 2 s: 100 slot posts, 0 or 1 entry posts since START, 0 dropped`, then `(d1)`, `(d2)`, `(d3)` PASS | VC state 4, LF ON_LINE, 99–101 slot posts, steering ≥ +32 (about +64), LINE_FOUND set |
| 4 | `[LF] >>> NOW slide the strip SLOWLY to the car's LEFT (~1 cm/s)` | Slide the strip **to the car's left**, slowly and evenly: about 1 cm per second, about 3 cm in total. The line then moves out from under CENTRE (CENTRE crosses the line's right edge). Then hold still with both sensors over white. | `[LF] slide: …` every 0.2 s (dc falls from ~1000 to 0; steering falls from +64 through 0); `[LF] ON_LINE -> LINE_LOST`, `-> REACQUIRING`, `-> IDLE`; `[LF] at the edge: <n> polls, largest abs(steer) <k>`; `[LF] LINE_LOST at …, TRACK_LOST at …: 2120 ms`; then `(e)`, `(f1)`, `(f2)` PASS | (e): ≥ 3 polls at the edge, abs(steer) ≤ 24. (f1): one TRACK_LOST, 2110–2135 ms after LINE_LOST. (f2): VC READY (1), LF IDLE, LINE_FOUND clear |
| 5 | `[LF] >>> strip under RIGHT ONLY (CENTRE on white), keep still` (10 s, then 3 s measuring) | Slide the strip so the black line is **under RIGHT only**: middle of the line under RIGHT, CENTRE on white. Hold still. | `[LF] RIGHT on black: raw <r> = <d> pm (DARK > 750)`, `[LF] CENTRE on white: raw …`, `(g)` PASS or FAIL | d > 750 |
| 6 | (automatic) | nothing | `[LF] totals: …`, `(h)` PASS, then `[TEST] subsystem3 SUMMARY 11 passed, 0 failed` | exactly that summary |

After that, Subsystems 4 and 5 run their tests. Let the capture run on until `[STACK]` report lines have appeared at least twice after the SUMMARY.

**If step 4 (e) fails with "at the edge: 0 polls" or 1–2 polls, the strip was slid too fast.** Note it, finish the run, and run it once more from the flash.

**If (g) FAILs, that is a real finding, not a mistake.** RIGHT reads black lower than CENTRE, and junctions would never fire on the track. Record the numbers: LF-3.3 fixes the calibration from them.

## Check the log

```powershell
Select-String build/s3-lf32-run.txt -Pattern '\[TEST\] S3-U12|\[TEST\] subsystem3'   # 11 S3-U12 lines + SUMMARY
Select-String build/s3-lf32-run.txt -Pattern '\[LF\] (boot|cal|1 s|2 s|at the edge|LINE_LOST at|RIGHT on|CENTRE on|totals)'
Select-String build/s3-lf32-run.txt -Pattern '\[LF\] [A-Z_]+ -> '                    # state changes
Select-String build/s3-lf32-run.txt -Pattern '\[STACK\] lf' | Select-Object -Last 2
Select-String build/s3-lf32-run.txt -Pattern '\[LF\] (period overrun|event send|EVF_NAV)'   # expect none
```

| # | Expected | Pass rule |
|---|---|---|
| 1 | `[TEST] subsystem3 SUMMARY 11 passed, 0 failed` | exactly |
| 2 | `[STACK] lf: <used> of 1536 B used, <free> never used` | at least 256 B never used, and not `LOW` |
| 3 | no `[LF] period overrun`, `event send` or `EVF_NAV` error line | none |

## Stop conditions

- The build has warnings or no `check_rules: PASS`: stop and send the build output.
- `[LF] boot calibration REFUSED …`: the start pose was wrong (CENTRE not on black, or RIGHT not on white) or a sensor is unplugged. Fix the pose and flash again. Don't continue.
- `(c-h) not run: VC not READY …` with `vc=3`: Safety latched a fault (look for `[SAFETY]` and `[S4] IMU` lines). Stop and send the log. It is not a Subsystem 3 problem.
- The board resets (`[capture] port lost`, `FAULT`): stop and send the log.
- The wheels turn at any time: the battery is on. Switch it off, flash again.

## What to send back

Post this on the LF-3 PR when it exists; until then, send it to Jin Yong:

```text
S3 LF-3.2 hardware run (S3-U12), <date>, by <name>
Commit: <hash from Build>
Build: warnings=<n>, check_rules=<PASS/FAIL>
Setup: battery OFF; strip = <tape / real track>; sensor height C=<mm> R=<mm>; spacing C-R=<mm>; lighting <...>
Result: [TEST] subsystem3 SUMMARY <p> passed, <f> failed
FAILs: (each [TEST] S3-U12 FAIL line and the [LF] lines just above it)
Numbers: (the [LF] boot calibration, 1 s, 2 s, at the edge, LINE_LOST at, RIGHT on black, CENTRE on white, totals lines; the last [STACK] lf line)
Notes: (anything that went wrong, with the step number)
Log: build/s3-lf32-run.txt (attach the whole file)
Photo: sensors over the strip, front view, with the ruler
```

Jin Yong then saves the log as `testing/subsystem3/logs/<date>_s3_lf32_u12.txt`, adds the S3-U12 result row to `TEST_LOG.md`, and only then counts LF-3.2 as passed on hardware.
