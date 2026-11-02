# Subsystem 3 BC-2A: barcode sensor → letter, hardware tests (for Buddy 2 and his AI agent)

Written 2 Nov 2026 by Buddy 3 (Jin Yong). Tests: `testing/subsystem3/TEST_LOG.md`, S3-U03 to S3-U06. Code: `app_program/subsystem3/drv_bc.c` (GP27 edge interrupt), `task_bc.c` (Barcode task), `bc_frame.c` + `bc_decode.c` (host-tested: S3-U01, S3-U02).

## What this is

This is the first time the barcode code meets the real sensor. The chain under test is:

GP27 edge → interrupt → edge ring → Barcode task → Motion speed per gap → framing → Code 39 decoder → letter A–Z.

The firmware prints every read as a `[BC]` line: the letter, or the reason it was refused. Each line is followed by the raw gap times, so a failure can be replayed on a PC. The test itself counts the letters and prints `[TEST] S3-U0x PASS/FAIL`. **Nothing here moves the car on its own, except S3-U05 and S3-U06 c–e, which drive 40 cm forwards and back on the floor.** No letter is turned into a movement yet; that comes later (BC-2B).

| Test | Question it answers | Where |
|---|---|---|
| S3-U03 | Does GP27 give clean edges: none when still, exactly 2 per black strip? Which level is black? | stand |
| S3-U04 | Do hand-slid cards read as the right letter, both directions? | stand |
| S3-U06 a, b | Are half passes and covered cards refused (no letter)? | stand |
| S3-U05 | Do cards read correctly while the car drives over them, forwards and backwards, with Motion's real speed? | floor |
| S3-U06 c, d, e | Car stops on a card / parks on it / drives on: refused, silent, then reads again? | floor |

## Rules for the agent

Same as before:
- Never edit, commit or push.
- Steps marked **PERSON** need the person.
- Run commands from the repo root in PowerShell.
- On a failure, stop and report (see "Stop conditions").
- About 45 minutes, most of it the person following prompts.

## What the person needs

- The car with the barcode sensor on **Grove 6** (VCC, GND, DO only; AO not wired), and the car's stand (wheels off the ground).
- **Tape over Grove 6's GP26 lead** (TEAM.md hardware checklist). It is the same pin as the right line sensor.
- A multimeter, a ruler, white A4 paper, a small piece of white paper (about 15 × 50 mm) to cover the end of a card, and sticky tape.
- About 1 m of clear, flat floor in a straight line, and a phone for photos.
- The cards: print `testing/subsystem3/cards/bc2a_cards_p1.svg` and `bc2a_cards_p2.svg`. Open them in a browser and print at **100 % / actual size** (turn off "fit to page").
  - Check with a ruler: the scale bar at the top of each page must measure 100 mm, and each barcode 141 mm from its first to its last bar.
  - Write the measured lengths down.
  - Cut out the six cards (*A*, *Z*, *F*, *W*, *X*, *Y*), the 20 mm black strip and the black square.
  - The cards carry the official module patterns (3 mm narrow, 9 mm wide).

## Steps

### 1. Get the code (agent)

```powershell
git status --short              # must be empty (untracked session files are fine)
git fetch origin
git switch buddy3/bc2a-hw
git pull --ff-only
git log --oneline -3            # note the top hash
git submodule status            # expect " 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp"
```

### 2. Build the APP_TEST firmware (agent)

Run the HANDOVER.md section 5 step 3 environment block first. Then:

```powershell
if (Test-Path build/s3-bc2a) { Remove-Item -Recurse -Force build/s3-bc2a }
& $CM -S . -B build/s3-bc2a @common -DDBG_BACKEND=USB -DAPP_TEST=ON 2>&1 | Tee-Object build/s3-bc2a-configure.txt
& $CM --build build/s3-bc2a 2>&1 | Tee-Object build/s3-bc2a-build.txt
Select-String -Path build/s3-bc2a-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/s3-bc2a-configure.txt -Pattern 'check_rules: PASS'                                         # expect one match
Test-Path build/s3-bc2a/robocar.uf2                                                                                    # expect True
```

### 3. Physical setup (PERSON, before flashing)

Measure and write down each value. It goes in the report.

1. **Sensor voltage (safety: GP27 is not 5 V tolerant).** Power the car with the USB cable only. Measure the sensor's DO pin to GND with the multimeter, once with the sensor over white paper and once over black. Both readings must be **between 0 and 3.4 V**. If either is above 3.4 V, **stop**: the sensor must be powered from 3.3 V.
2. **Sensor height.** Measure the distance from the sensor's face to the surface it reads, on the stand and on the floor. Typical is 3–8 mm.
3. **Sensor threshold.** The sensor board has two LEDs: power (always on) and signal (the one next to `D0`/`A0`, which is ON over white and OFF over black). Set the switching point to the **50 % point**: put the sensor, at its working height, exactly over the edge between a thick black bar and white (half over each) and turn the blue screw slowly until the signal LED is right at the point of switching. Then check: white alone → LED on, black alone → LED off. Flicker right at an edge is fine (the firmware filters it). Note "adjusted" or "not adjusted". (Desk runs 2 Nov: a wrong setting gave 0 edges; a setting biased towards white made narrow bars look 4 ms long against 20 ms spaces.)
4. **Photos:**
   - (a) the sensor mount from the side, with the ruler showing the height;
   - (b) the car from below, showing where the barcode sensor sits;
   - (c) each printed card next to the ruler.
5. **Lighting:** normal room light. Note it (for example "ceiling LEDs, no sun").
6. Put the car **on the stand**. Subsystem 2's tests run first after boot and spin the wheels.

### 4. Flash and capture (PERSON, then agent)

1. **PERSON:** unplug the Pico W, hold BOOTSEL, plug it in, release BOOTSEL. `RPI-RP2` appears. Say "done".
2. **Agent:** copy the firmware, then start the capture **at once**. It prints every line on the screen as well, and the person follows those lines.

```powershell
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
if (-not $v) { throw "RPI-RP2 drive not found" }
Copy-Item build/s3-bc2a/robocar.uf2 "$($v.DriveLetter):\"
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 2700 -Out build/s3-bc2a-run.txt
```

### 5. The run (PERSON follows the screen)

Subsystems 1 and 2 run their tests first (about 1–2 minutes; the wheels spin). Then:

`[S3] BC-2A hardware tests: follow the [S3] >>> prompts`

From then on, every `[S3] >>> <step>: <what to do> (<n> s)` line tells the person what to do and for how long. `[S3] <<< <step>: stop` ends that window. Prep windows (`... prep`) are time to get ready. The test counts only what happens inside the named windows.

| Window(s) | What the person does | Expected lines | PASS rule |
|---|---|---|---|
| — | nothing | `[TEST] S3-U03 PASS (a) GP27 handler registered, Barcode task running` | PASS |
| `U03 white` (10 s) | White paper under the sensor; touch nothing | `[S3] white: edges=0 level=…` then `[TEST] S3-U03 PASS (b) …` | 0 edges |
| `U03 black` (10 s) | The black square under the sensor; touch nothing | `[S3] black: edges=0 level=…` then `(c)` PASS | 0 edges, and the level differs from white |
| `U03 strip` (counted: 5 reads, max 60 s) | Slide the 20 mm strip across under the sensor **5 times, always the same way** (lift it to bring it back) | `[S3] strip: edges=10 glitches=<n>`, `(d)` PASS; five `[BC] … FAIL INCOMPLETE` lines are normal. `glitches` counts sensor flicker the filter removed: any number is fine | exactly 10 edges |
| `U04 slide` ×6 (counted: 10 reads each, max 120 s) | **Use the card the prompt names** (other letters count as wrong). Each slide prints `[S3] k/10: OK <letter>`; the step ends after the 10th read. Cards in the order the prompt names (*A*, *Z*, *F*). Each card twice: **10 slides left-to-right**, then **10 slides right-to-left**. Move the card along its **141 mm length**, so the sensor crosses all 15 bars one after another (not along a bar). Card flat, about 1 s per pass, one pass every 4 s, lift it to bring it back. | per slide a `[BC] #n OK A …` line; after each window `[S3] card A: correct=10 wrong=0 refused=0 ovf=0` | per window **≥ 9 correct and 0 wrong**, 0 overflow |
| `U06 half` (counted: 3 reads, max 60 s) | Card *A*: slide it only halfway under the sensor and pull it back; 3 times | `[BC] … FAIL INCOMPLETE …` lines, no `OK` | no letter, ≥ 1 refused |
| `U06 cover` (counted: 3 reads, max 60 s) | Tape the small white paper over the **last 10 mm** of card *A* (the stop `*`). Slide it fully across, 3 times. | `FAIL` lines, no `OK` | no letter, ≥ 1 refused |
| `FLOOR prep` (60 s) | Lift the car **off the stand onto the floor**, pointing along the clear 1 m. Mark the start position with tape. | — | — |
| `U05 prep` ×6 | Tape the named card flat on the floor **15 cm ahead of the barcode sensor**, bars across the car's path, centred under the sensor's track. Put the car back on its start mark if it has moved. | — | — |
| (drives) | Nothing. The car drives 40 cm forwards and 40 cm back, 5 times per card (cards *A*, *Z*, *F*, *W*, *X*, *Y*). | `[BC] #n OK …` per crossing, then `[S3] card X: correct=10 wrong=0 …` | per card **≥ 9 correct and 0 wrong**, drives complete |
| `U06 prep` (30 s) | Card *A* again, 15 cm ahead; car on its start mark | — | — |
| (drive 22 cm) | Nothing. The car stops with the sensor over the card. | `[S3] stop mid-card: ok=0 refused=1` | no letter, ≥ 1 refused |
| `U06 park` (10 s) | Touch nothing | `(d)` PASS | 0 edges, no read |
| (back 22, forward 40) | Nothing | `(e)` PASS | the next full pass reads A |

Then `[S3] totals: …` (3 lines) and `[TEST] subsystem3 SUMMARY 21 passed, 0 failed`. Subsystems 4 and 5 run their tests after that: leave the car on the floor and let the capture finish.

If a step went wrong because of the person (a slide skipped, the card moved), **don't restart**. Note it with the window and the time, and carry on.

### 6. Check the log (agent)

```powershell
Select-String build/s3-bc2a-run.txt -Pattern '\[TEST\] S3-|\[TEST\] subsystem3'      # 21 S3 lines + SUMMARY
Select-String build/s3-bc2a-run.txt -Pattern '\[S3\] (white|black|strip|card|half|covered|stop|totals)'
Select-String build/s3-bc2a-run.txt -Pattern '\[BC\] #\d+ (OK|FAIL)' | Measure-Object | Select-Object -ExpandProperty Count
Select-String build/s3-bc2a-run.txt -Pattern '\[STACK\].*\bbc\b|\[STACK\].*LOW' | Select-Object -Last 3
```

| # | Expected | Pass rule |
|---|---|---|
| 1 | `[TEST] subsystem3 SUMMARY 21 passed, 0 failed` | exactly |
| 2 | `[S3] totals: … dbg_dropped=0` | 0, so no log line was lost |
| 3 | `[S3] totals: … max_poll=…us` | below 10000 µs (the task was never late by more than 10 ms) |
| 4 | the `[STACK]` line for `bc` | not `LOW` |

## Stop conditions

- DO above 3.4 V in step 3.1: stop, don't connect GP27.
- The build has warnings or no `check_rules: PASS`: stop and send the build output.
- No `[S3]` lines after 5 minutes, or the board resets (`[capture] port lost`, `FAULT`): stop, send the log.
- S3-U03 (b) or (c) FAIL with many edges while still: the sensor or wiring is noisy. Finish the run anyway (the later tests show how bad it is), but say so in the report.
- The car does not move in S3-U05 (`[S3] drive fwd 40 cm: not reached`): let the run finish, send the log. That is a Motion or setup problem, not barcode.
- The car heads towards something: lift it, note the time, and let the run continue on the stand.

## What to send back

Post this on the BC-2A.2 PR and send it to Jin Yong:

```text
S3 BC-2A hardware run, <date>, by <name>
Commit: <hash from step 1>
Build: warnings=<n>, check_rules=<PASS/FAIL>
Setup: DO white=<V> black=<V>; sensor height stand=<mm> floor=<mm>; pot <adjusted/not>; lighting <...>
Cards measured: A=<mm> Z=<mm> F=<mm> W=<mm> X=<mm> Y=<mm>; scale bar=<mm>
Result: [TEST] subsystem3 SUMMARY <p> passed, <f> failed
FAILs: (paste each [TEST] S3 FAIL line and the [S3] line above it)
Totals: (paste the 3 [S3] totals lines and the [STACK] bc line)
Notes: (anything that went wrong while following the prompts, with the window name and time)
Log: build/s3-bc2a-run.txt (attach the whole file)
Photos: sensor side view with ruler, underside, each card with ruler
```

The whole log matters, not only the summary. Every `[BC] #n r..` line is a raw capture that Jin Yong can replay on a PC to find a failure's cause.

Jin Yong then saves the log as `testing/subsystem3/logs/<date>_s3_bc2a_hw.txt`, adds one result row per test to `TEST_LOG.md`, and only then counts BC-2A as passed.
