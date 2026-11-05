# Subsystem 3 LF-3.4: junction turn on the course (S3-U14)

Written 5 Nov 2026 by Buddy 3 (Jin Yong). Test: `testing/subsystem3/TEST_LOG.md` S3-U14. Code: `app_program/subsystem3/lf_run_test.c`.

## Purpose

With a turn command waiting, does the car pivot at the cross, find the new line and follow it? The command is injected by the test: no barcode is read. Left passed on 5 Nov. This build tests the right turn; U-turn comes last.

## Setup

1. **Track:** a cross with at least 30 cm of straight line before it, and **at least 50 cm of straight line on the side the car turns to**. The car lands about 12 cm along the new line and follows it 30 cm more. Add a board if the line is shorter: too short a line fails (t5).
2. **Power:** battery disconnected, USB only. From pose A on, lift the laptop and follow the car, cable slack and overhead. It must never pull on the car. Wires clear of the right wheel.
3. **Space:** keep the laptop and everything else at least 25 cm from the car's path, on both the approach and the new line.

## Position

- **Power-up:** nothing special. The test no longer relies on the reading taken at power-up.
- **Pose A:** CENTRE on black in the line's middle, RIGHT on white, flat, hands off. **The car calibrates here.** If it says `pose A not good`, fix the position: you get 3 tries.
- **Pose B:** CENTRE on white, RIGHT on black.
- **Each drive:** CENTRE on the right half of the black line, RIGHT on white, parallel to the line, CENTRE 30 cm before the cross. Set it once and tape the wheels' back and side.

## Each drive

1. Lift the car and set it against the tape, hands off.
2. It starts by itself after `HANDS OFF`.
3. It crosses the junction line, stops, pivots right, steers gently onto the new line and follows it about 30 cm, then stops.
4. Note what you see: did it pivot about a quarter turn, did it end on the new line?

`LINE_LOST` after the pivot is allowed, but should now be rare: the car then searches and finds the line.

## PASS

`[TEST] subsystem3 SUMMARY 21 passed, 0 failed`.

## Stop conditions

- Car heads off the mat: catch it by the chassis, fingers away from the wheels.
- `STOP: no drives`: pose A failed 3 times: check the car is on black and RIGHT on white, then flash again.
