# Subsystem 3 LF-3.2: the Line-Following task driving the car (S3-U12)

Written 3 Nov 2026 by Buddy 3 (Jin Yong); rewritten 4 Nov 2026 for a moving car. Test: `testing/subsystem3/TEST_LOG.md` S3-U12. Code: `app_program/subsystem3/task_lf.c` (the task), `lf_run_test.c` (this test), plus the host-tested cores `lf_core.c` (S3-U10) and `lf_link.c` (S3-U11).

The 3 Nov version of this test assumed that with the battery off the motors stay off. That is false: on the Robo Pico, USB alone powers the motors. Both runs of that version are recorded as INVALID in the TEST_LOG.

## Purpose

This is the first time the line-following code drives the car, together with the real Vehicle Controller (VC) and Motion tasks. The speed is the build's cruise speed: **200 mm/s** with the default bring-up speed set. 300 mm/s comes later, in a separate build, only after a clean 200 mm/s run.

| Check | Question |
|---|---|
| (a) | Does the task sample every 5 ms without missing a period? |
| (b) | Does the boot calibration accept the start pose? |
| (c) | While the VC waits for START, does Line-Following stay silent? |
| (g) | With the car standing still, does RIGHT read the black line as DARK at ride height? This is the residual risk from the S3-U09 waiver, and junctions depend on it. The test also prints both sensors' static levels and noise. |
| (d1) | Straight drive: is the first steering to the right, and does CENTRE reach the line's right edge within 150 mm? This checks the steering direction. |
| (d2) | Is exactly one steering command posted per 20 ms Motion period, in Motion's post slot, with none dropped? |
| (d3) | Does it track the edge stably (at least 90 % of samples in the band), never lose the line, and set LINE_FOUND? |
| (n) | With the motors running, does RIGHT stay LIGHT over white, with no false junction? The test prints RIGHT's moving noise against the static noise. |
| (r1–r3) | From a small offset, does it find the line within 1 s and settle without losing it? |
| (j) | At a cross junction: exactly one JUNCTION, RIGHT DARK on the cross line, and tracking again after it? |
| (f1, f2) | At a line end: LINE_LOST, the search, exactly one TRACK_LOST 2120 ms later, the VC stops the car, and the wheels stand still? |
| (h) | Over the whole run: no missed period, no dropped post, no failed event? |

## Safety: what drives the car and what stops it

- **The motors are live from the moment the board is powered.** Subsystem 2's tests spin the wheels about 10–14 s after boot, before this test starts.
- **The test drives the car six times.** Each drive starts at the end of a 10 s count and **stops by itself**. The test sends STOP when the wheel travel or the time reaches the drive's limit:

  | Drive | Limit |
  |---|---|
  | Straight | 700 mm |
  | r1, r2, r3 | 500 mm each |
  | Junction | 800 mm |
  | Line end | 1200 mm / 7 s |

  The time limit of each drive is its distance at cruise speed plus 1.5 s.
- **The VC stops the car** on TRACK_LOST. Safety brakes on a collision spike above 2.5 g.
- **To stop the car by hand, catch it and lift it by the chassis**, keeping your fingers away from the wheels. Lifting does not stop the motors: in the air the sensors read "black", and the wheels keep spinning at cruise speed until the drive's distance limit ends the drive. That takes at most a few seconds, 6 s in the worst case. A hard catch can latch a collision fault. The test then sends RESET before the next drive; keep the car still during the count, because the VC accepts RESET only after 0.5 s with the wheels still.
- **Last resort:** unplug the USB cable. With USB as the only power source this stops everything.
- **Obstacle avoidance (Subsystem 5) is live.** Anything within 25 cm ahead of the ultrasonic sensor (30–150° sweep) makes the VC hand the car to avoidance. The test then sends STOP and that drive fails. Walk **behind** the car, never in front of it.
- Between drives the VC is READY and the motors are stopped. Hands near the car are safe then: the VC ignores avoidance while READY.

## Build

From the repo root, after the HANDOVER.md section 5 step 3 environment block:

```powershell
git status --short                 # only untracked session files
git switch buddy3/lf3-link
git log --oneline -1               # note the hash (it goes in the report)
if (Test-Path build/s3-u12) { Remove-Item -Recurse -Force build/s3-u12 }
& $CM -S . -B build/s3-u12 @common -DDBG_BACKEND=USB -DAPP_TEST=ON 2>&1 | Tee-Object build/s3-u12-configure.txt
& $CM --build build/s3-u12 2>&1 | Tee-Object build/s3-u12-build.txt
Select-String -Path build/s3-u12-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/s3-u12-configure.txt -Pattern 'check_rules: PASS'                                         # expect a match
```

Do not set `MOTION_BRINGUP_SPEEDS`: the default (1) gives 200 mm/s.

In Jin Yong's workspace the firmware is already built: `build/robocar-s3-u12v2-wip-3a5d76a.uf2`. The script `build/s3-u12v2-run.ps1` flashes it and captures the console into a new file `build/s3-u12v2-run-<date>-<time>.txt` each time.

## Physical setup (before flashing)

1. **The track.** It needs:
   - **a straight of at least 1 m** of the 18 mm black line. The straight drive and the three recovery drives all start at its beginning.
   - **a cross junction** with at least 40 cm of straight line before it and 40 cm after it. The car must arrive square to the cross line.
   - **a line end.** If the track has none, make one: at least 50 cm of 18–19 mm black electrical tape on white paper or board, lying flat at floor level, with at least 30 cm of clear white around the end. At the end the car pivots left and right for about 2 s while it searches.
2. **Clear space.** Nothing within 40 cm of the car's path: no walls, boxes, bags or feet.
3. **Power and cable.** A USB cable of at least 2 m from the laptop. Hold it loosely, above and behind the car, so it never pulls the car. Write down the power setup: the battery (connected or not) and the position of the board's power switch.
4. **Optional switch check (1 minute).** Do it if you want to know whether the switch cuts the motors. Hold the car in the air and power it from USB only, with the switch OFF. Watch the wheels about 10 s after boot. Repeat with the switch ON. Note the result.
5. **Ride height.** Measure each sensor face to the floor (mm), and the CENTRE–RIGHT spacing, centre to centre (mm).
6. **Lighting:** normal room light, no direct sun. Note it.
7. **Photo:** the two sensors over the line, from the front, with the ruler.
8. GP28 VBAT jumper OPEN. The GY-511 IMU must be connected (Grove 2). Without it Safety latches a fault and steps 3–10 cannot run.

## Flash and capture

1. Put the car on the track in the **start pose** at the beginning of the straight:
   - car parallel to the line;
   - CENTRE over the **middle** of the black line;
   - RIGHT over white.

   The boot calibration reads this pose about 0.2 s after power-up.
2. In PowerShell, from the repo root:

   ```powershell
   powershell -ExecutionPolicy Bypass -File build\s3-u12v2-run.ps1
   ```

3. When the script asks, unplug the Pico W, hold BOOTSEL, plug it in and release BOOTSEL. The script flashes the firmware and starts the capture. Every line also shows on screen.

## Steps

| # | Prompt / when | What the person does | Expected lines | PASS rule |
|---|---|---|---|---|
| 1 | Boot | Keep the car still in the start pose until `[LF] boot calibration OK …` **and** `[IMU] level reference …` have appeared (about 1 s). Then **lift the car**, held level with the wheels free, and keep it up. | `[LF] boot calibration OK: C avg …, R avg …`. Subsystem 1 and 2 tests (the wheels spin about 10–14 s after boot). Then `[LF] S3-U12 on the track, motors LIVE, cruise 200 mm/s` | Calibration OK; **cruise 200 mm/s** |
| 2 | (automatic, 2 s) | Keep holding the car | `[LF] 1 s: 200 samples, 0 missed so far`; `(a)`, `(b)`, `(c)` PASS | 195–205 samples, 0 missed; cal status 0; no output while READY |
| 3 | `[LF] >>> pose A: put the car down in the START POSE, hands off` (10 s, then 3 s measuring) | Put the car down in the start pose. Hands off by the end of the count. | `[LF] static pose A: C raw … = … pm`, `R raw … = … pm` | Measurement: C about 1000 pm, R about 0 pm |
| 4 | `[LF] >>> pose B: RIGHT over the line middle, CENTRE on white, hands off` | Move the car **left** by the CENTRE–RIGHT spacing, so the line's middle is under RIGHT and CENTRE is on white. Hands off. | `[LF] static pose B: …`; `(g)` PASS or FAIL | RIGHT > 750 pm. A FAIL is a finding, not a mistake: write the numbers down |
| 5 | `[LF] >>> straight: START POSE at the start of >= 1 m of straight line`, then `hands off at 0: the car DRIVES` | Start pose at the beginning of the straight. Hands off at 0, then walk behind the car. | `[LF] IDLE -> ON_LINE`; `[LF] d t=… mm=…` every 100 ms; `[LF] straight: end distance limit at … ms, 700 mm …`; speed, settle, R white moving vs static; `(d1)`, `(d2)`, `(d3)`, `(n)` PASS | First steer about +64; settle ≤ 150 mm; measured speed about 200 mm/s; R white never DARK |
| 6 | `r1: car parallel to the line, CENTRE ~1 cm RIGHT of it (on white)` | Start pose, then move the car about 2 cm to the **right**, so CENTRE is 1 cm right of the line's right edge, over white. Car parallel. | `[LF] IDLE -> REACQUIRING`, then `-> ON_LINE`; `[LF] r1: …`; `(r1)` PASS | First steer negative (left); ON_LINE within 1 s |
| 7 | `r2: nose ~15 deg LEFT of the line, CENTRE on the line middle` | CENTRE on the line middle, then turn the car about 15° nose-left around CENTRE | `(r2)` PASS | ON_LINE within 1 s, settles, never lost |
| 8 | `r3: nose ~15 deg RIGHT of the line, CENTRE on the line middle` | Same, about 15° nose-right | `(r3)` PASS | Same |
| 9 | `junction: START POSE, a cross junction 40 cm ahead, square to it` | Start pose 40 cm before a cross junction, car square to the cross line | `[LF] ON_LINE -> JUNCTION`, later `-> ON_LINE`; `[LF] junction: … jct 1 …`; `R peak raw … = … pm`; `(j)` PASS | Exactly one JUNCTION; RIGHT peak > 750 pm; ON_LINE at the end |
| 10 | `line end: START POSE, the line's END 30 cm ahead (temporary line OK)` | Start pose 30 cm before the line's end (track or temporary line). Let the car search: it pivots left and right for about 2 s, then stops by itself. | `-> LINE_LOST`, `-> REACQUIRING`, `-> IDLE`; `[LF] line end: end TRACK_LOST …`; `[LF] line end: LINE_LOST to TRACK_LOST 2120 ms`; `(f1)`, `(f2)` PASS | 2110–2135 ms; VC READY; wheels still |
| 11 | (automatic) | Nothing. Keep the car still. | `[LF] totals: …`, `(h)` PASS, `[LF] S3-U12 done`, then `[TEST] subsystem3 SUMMARY 15 passed, 0 failed` | Exactly that summary |

After that, Subsystems 4 and 5 run their tests; they don't drive the car. Let the capture run until `[STACK]` report lines have appeared at least twice after the SUMMARY.

All the S3 steps together take about 3 minutes. Each prompt gives 10 s: place the car in the first 5–6 s, then take your hands off.

## Stop conditions

- **The car heads for an edge or an obstacle:** catch it and lift it. Note the drive's name. The drive fails; the test goes on to the next prompt.
- **The car clearly steers away from the line on the straight** (a sign error): catch it, then stop the capture with Ctrl+C and send the log. Don't continue.
- **`[capture] port lost` while the car is driving:** a brownout or the cable. Stop and send the log.
- **`[LF] boot calibration REFUSED …`:** the start pose was wrong at power-up. Fix it and flash again.
- **`(c-h) not run: VC not READY …` with `vc=3`:** Safety latched a fault before the test (look for `[SAFETY]` and `[IMU]` lines). Send the log.
- **`[LF] <drive>: VC state 3, not READY: no START`:** a latched fault could not be RESET because the car moved during the count. That drive fails. Keep the car still during the next count.
- **`end avoidance`:** something was within 25 cm ahead of the sensor. Clear the area. That drive fails.

## Check the log

```powershell
$log = (Get-ChildItem build\s3-u12v2-run-*.txt | Sort-Object LastWriteTime | Select-Object -Last 1).FullName
Select-String $log -Pattern '\[TEST\] S3-U12|\[TEST\] subsystem3'      # 15 S3-U12 lines + SUMMARY
Select-String $log -Pattern '\[LF\] (boot|cal|S3-U12 on|1 s|static|straight|r1|r2|r3|junction|line end|totals)'
Select-String $log -Pattern '\[LF\] [A-Z_]+ -> '                       # state changes
Select-String $log -Pattern '\[STACK\] (lf|app_test)' | Select-Object -Last 2
Select-String $log -Pattern '\[LF\] (period overrun|event send|EVF_NAV)|capture\] port lost'   # expect none
```

| # | Expected | Pass rule |
|---|---|---|
| 1 | `[TEST] subsystem3 SUMMARY 15 passed, 0 failed` | Exactly |
| 2 | `[STACK] lf: <used> of 1664 B used, <free> never used`, and the same for `app_test` | At least 256 B never used, and not `LOW` |
| 3 | No `[LF] period overrun`, `event send` or `EVF_NAV` error line, and no `port lost` | None |

## What to send back

Post this on the LF-3 PR when it exists; until then, send it to Jin Yong:

```text
S3 LF-3.2 hardware run (S3-U12, moving), <date>, by <name>
Commit / firmware: <hash or UF2 name>
Build: warnings=<n>, check_rules=<PASS/FAIL>
Setup: power <USB only / USB + battery>, switch <ON/OFF>; switch check <result or not done>;
       sensor height C=<mm> R=<mm>; spacing C-R=<mm>; line end <track / temporary tape>; lighting <...>
Result: [TEST] subsystem3 SUMMARY <p> passed, <f> failed
FAILs: (each [TEST] S3-U12 FAIL line and the [LF] summary lines of that drive)
Numbers: (the [LF] boot calibration, static pose A/B, each drive's summary lines, totals; the last [STACK] lf and app_test lines)
Notes: (anything that went wrong, with the step number; any drive you had to catch)
Log: build\s3-u12v2-run-<date>-<time>.txt (attach the whole file)
Photo: sensors over the line, front view, with the ruler
```

Jin Yong then saves the log as `testing/subsystem3/logs/<date>_s3_lf32_u12.txt`, adds the S3-U12 result row to `TEST_LOG.md`, and only then counts LF-3.2 as passed on hardware.
