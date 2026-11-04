# Subsystem 3 LF-3.3: junction steps 1-3 on the junction mat (S3-U15)

Written 4 Nov 2026 by Buddy 3 (Jin Yong). Test: `testing/subsystem3/TEST_LOG.md` S3-U15. Code: `app_program/subsystem3/lf_run_test.c`, `lf_core.c`.

## Purpose

Three questions, at 200 mm/s, with F = 120 mm:

1. Does the car detect the cross?
2. Does the JUNCTION event come when the axle is on the cross?
3. Does it carry on straight, then stop by itself?

It also checks whether the wheel distance is accurate. Left, right, U-turn, curves and barcode are not tested: the mat's branches are too short.

## Setup

1. **Track:** standalone junction mat, approaching from the long side. The line ends about 140 mm past the cross.
2. **Power:** battery disconnected, USB only, so unplugging stops everything. Long cable, held loosely above and behind the car.
3. **Space:** keep hands and objects 25 cm clear in front of the car. Cables clear of the sonar.
4. **Ruler marks:** have tape, a pen and a ruler ready.

## Start pose

- **CENTRE on black, RIGHT on white.**
- CENTRE sensing point 250-270 mm before the centre of the cross, the same distance every drive.
- CENTRE slightly right of the line's middle, as in the S3-U13i run.
- Car parallel to the line.

Boot, pose A and every drive use this pose. Pose B: **CENTRE on white, RIGHT on black.**

## Each drive

1. Place the car. Put a tape mark on the mat beside a fixed point of the car, such as the front edge of the CENTRE sensor board.
2. Hands off by 0. Stay ready to catch it.
3. The car stops by itself just after the cross. Do not touch it. The log prints `MEASURE NOW` and gives you 45 s.
4. Mark the same fixed point again. Write down the distance between the two marks: **real travel**.
5. Measure from the left wheel hub to the centre of the cross arm, along the direction of travel, then the same for the right wheel. Past the cross is +, before it is -. The average is **A**. Averaging cancels a cross arm that is not square to the line.
6. Put the car back when the next prompt asks.

## What to read in the log

`[LF] RULER run N: wheel travel START to standstill X mm, after JUNCTION Y mm`

- **Distance ratio** = X / real travel.
- **Axle at the event** = A - Y (rough: Y is in wheel mm).

## PASS

All of these, on all 3 drives:

- `[TEST] subsystem3 SUMMARY 21 passed, 0 failed`.
- `tr jmat end: ring dropped 0 lines`.
- Distance ratio 0.97 to 1.03 (NFR4 allows +-3 %).
- Axle at the event within +-9 mm of the cross centre, the half width of the cross line.

## What a failure means

| Result | Meaning |
|---|---|
| `(j0)` FAIL only | Early line loss came back: the launch problem, not a junction problem |
| `(j1)` FAIL, no JUNCTION | Detection missed: the car stops by itself at 430 mm or on LINE_LOST |
| `(j2)` FAIL | The delay is wrong, or the time cap ended it (encoder fault) |
| `(j3)` FAIL | The car left the line past the cross |
| `(j4)` FAIL | The stop came late, or the car was caught |
| Ratio about 1.08 to 1.16 | The wheel distance reads long: the software passes but the real pivot is short. Stop and report; do not retune F |

## Stop conditions

- **Car leaves the mat or heads for something:** catch it by the chassis, fingers away from the wheels.
- **`encL` or `encR` stays near 0 at `[MOT] S2-U16`:** unplug the USB and stop.
- **`(g2)` fails:** unplug and restart in the start pose.
