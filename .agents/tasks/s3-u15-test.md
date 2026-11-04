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
2. **Power:** battery disconnected, USB only, so unplugging stops everything. Long cable, **held slack, overhead and behind the car, on every drive.** It must never pull on the car.
3. **Wires:** tie them clear, so nothing can touch or pull on the right wheel. With the cable loose, the right wheel ran about 45 mm/s slow past the cross; held clear, it did not.
4. **Space:** keep hands and objects 25 cm clear in front of the car. Cables clear of the sonar.
5. **Ruler marks:** have tape, a pen and a ruler ready.

## Start position

- **Boot and pose A:** CENTRE on black, in the line's middle, RIGHT on white.
- **Pose B:** CENTRE on white, RIGHT on black.
- **Each drive:** CENTRE on the right half of the black line, not on the black/white border. RIGHT on white. Car parallel to the line, CENTRE sensing point 250-270 mm before the centre of the cross.

Set this once, then put tape on the mat against the back and side of both wheels. Every drive goes back against the tape, so all three use the same position.

## Each drive

1. At `>>> jmat`, lift the car and put it against the tape. Take your hands off.
2. The start guard watches CENTRE and RIGHT. Nothing moves until CENTRE is steady on black and RIGHT on white. `NOT READY` means reposition the car.
3. `HANDS OFF` comes 3 s before the car drives. If the car moves in that time, the start is cancelled.
4. The car crosses the junction, carries on straight and stops by itself. There is no turn in this test.
5. Take your time. The guard waits up to 3 minutes, and only starts after the car has been lifted.

**Once, on a clean drive:** mark a fixed point of the car before and after, and measure the distance between the marks: **real travel**. Also measure left and right wheel hub to the centre of the cross arm, along the direction of travel, past it +, before it -. The average is **A**.

## What to read in the log

`[LF] RULER run N: wheel travel START to standstill X mm, after JUNCTION Y mm`

- **Distance ratio** = X / real travel.
- **Axle at the event** = A - Y (rough: Y is in wheel mm).

## PASS

Started by the guard on all 3 drives, and on all 3:

- `(j1)` `(j2)` `(j3)` `(j4)` PASS.
- `tr jmat end: ring dropped 0 lines`.

That closes the straight-through junction stage. `(j0)` is logged but is a launch result, not a junction one: the launch loss from a start on black is known and tracked separately. A drive that stops before the cross did not test the junction: repeat that run.

The ruler numbers do not decide this test, but the next stage needs them: distance ratio 0.97 to 1.03 (NFR4 allows +-3 %), axle at the event within +-9 mm of the cross centre.

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
