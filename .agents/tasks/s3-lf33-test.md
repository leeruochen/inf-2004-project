# Subsystem 3 LF-3.3: straight tracking on the car at 200 mm/s (S3-U13)

Written 4 Nov 2026 by Buddy 3 (Jin Yong). Test: `testing/subsystem3/TEST_LOG.md` S3-U13. Code: `app_program/subsystem3/lf_core.c`, `lf_run_test.c`.

## Purpose

One question: **does the car now follow a straight line at 200 mm/s, repeatably?** Junctions, curves, recovery and 300 mm/s are not tested here.

The firmware does the static poses, then the same straight drive three times.

## What should look different from the 4 Nov runs

| | Before | Now |
|---|---|---|
| Start | Turns right firmly | Very gentle drift right: `s=4` |
| Middle of the drive | Swings across the line, full steer | Small corrections near the edge, `s` between about -20 and +4 |
| End | Runs off, curves right, gets caught | Stops by itself after 900 mm |

## Build

In Jin Yong's workspace the firmware is already built from the commit under test: `build/robocar-s3-u13-<hash>.uf2`. The script `build/s3-u13-run.ps1` flashes it and captures the console into a new file `build/s3-u13-run-<date>-<time>.txt`.

To rebuild from source: use the `s3-lf32-test.md` build steps, with no `MOTION_BRINGUP_SPEEDS` setting (the default gives 200 mm/s).

## Physical setup

1. **Track:** at least 1.2 m of straight 18 mm black line, white either side, nothing crossing it.
2. **Power:** battery disconnected, USB only, so unplugging stops everything. Use a long cable, held loosely above and behind the car.
3. **IMU:** GY-511 connected on Grove 2.
4. **Space:** clear 40 cm around the path. Stay behind the car: anything within 25 cm in front triggers obstacle avoidance.
5. **Lighting:** normal room light, no direct sun.

## Where the sensors go

| Prompt | CENTRE | RIGHT | Car |
|---|---|---|---|
| Flashing | Black, middle of the line | White | Parallel to the line, on the floor, still |
| `pose A` | Black, middle of the line | White | Parallel |
| `pose B` | White | Black, middle of the line | Parallel |
| `straight` (3 times) | Black, middle of the line | White | Parallel, at the start of the straight |

Look straight down from above the sensors when placing the car.

## Steps

| # | Prompt / when | What to do | Check |
|---|---|---|---|
| 1 | Before flashing | Car on the floor in the flashing pose. Run the script. | |
| 2 | Script waiting | Unplug the Pico W, hold BOOTSEL, plug in, release. Do not move the car. | `[LF] boot calibration OK` |
| 3 | After `[IMU] level reference` | Lift the car by the chassis, wheels free. Keep holding it. | Both wheels spin at about 10 s |
| 4 | `[MOT] S2-U16` line | Read `encL` and `encR`. | Both rise by about 430. **If either stays near 0, or a wheel does not spin: unplug the USB and stop.** |
| 5 | `>>> pose A` | Put the car down in pose A, hands off by the end of the count. | `(g2)` PASS. **If it fails: unplug, go back to step 1.** The boot calibration is wrong and the drives would be meaningless |
| 6 | `>>> pose B` | Slide the car left into pose B, hands off. | `(g)` PASS |
| 7 | `>>> straight` (run 1) | Put the car back in the start pose, at the start of the straight. Hands off by 0. Walk behind it. | Follows the line, stops by itself after 900 mm |
| 8 | `[LF] tr` lines | Wait: the trace prints for about 3 s after the car stops. | `tr straight end: ring dropped 0 lines` |
| 9 | `>>> straight` (runs 2, 3) | Same as step 7, back at the start of the straight. | Same |
| 10 | End | Keep the car still. Press Ctrl+C after two more `[STACK]` lines. | `[TEST] subsystem3 SUMMARY 18 passed, 0 failed` |

## Stop conditions

- **Car leaves the line or heads for something:** catch it by the chassis, fingers away from the wheels. The wheels can keep spinning for a few seconds. The trace of that drive still prints, and it is the useful part: let it finish.
- **Car clearly steers away from the line from the start:** catch it and unplug the USB. Do not continue.
- **`[LF] boot calibration REFUSED`:** CENTRE was not on black at power-up. Fix the pose and flash again.
- **`[capture] port lost` while driving:** a cable or power problem. Stop and send the log.
- **Emergency:** unplug the USB. Ctrl+C only stops the logging, not the car.

## What to send back

The log file `build/s3-u13-run-<date>-<time>.txt`, and for each straight run: did it stop by itself, and did you have to catch it.
