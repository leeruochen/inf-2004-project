# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-26 | All (S1–S5) | `SYS-02`, `SYS-03`, `SYS-04`, `SYS-05`, `SYS-07` (Full Autonomous Course Demo) | `05e2ffc` | `2500530` (with team `2501299`, `2503319`, `2500709`, `2500596`) | https://www.youtube.com/watch?v=eDlNAmzyx_U | Unlisted full-run demo on the physical track: (1) decodes Code 39 barcode `Q` mapped to `STRAIGHT` (move forward via `car/cmd/bc_map`) and continues straight through the junction; (2) autonomous IR straddle line following (`GP28`/`GP26`); (3) detects obstacle `< 25 cm`, halts for stationary `3°` servo ultrasonic sweep (`51°–129°`), executes encoder-closed 4-turn `45°` bypass, and reacquires the line; (4) detects `15 mm` hump via IMU pitch (`>= 3°`) and applies gradual IMU-pitch-assisted speed/torque climb boost (`MOTION_HUMP_*`) to clear the hump. |

## Supplementary Dashboard Screenshots (Corroborating Telemetry, No Paired Video)

These two screenshots were captured from the live Robocar Web Dashboard ([`tools/dashboard/tel_web.py`](../tools/dashboard/tel_web.py)) during full-rubric bench/track sessions.

> **Evidentiary note:** Neither screenshot has a paired camera video recording or a committed raw `.txt` MQTT broker capture in `testing/system/logs/`. They are retained strictly as supplementary telemetry/dashboard evidence corroborating live MQTT frame decoding, demo rubric tracking (`10 / 10`), turn/U-turn barcode execution, obstacle bypass telemetry, hump measurement, and pickup safety latch transitions.

| File | Session / Date | Rubric | Observed Telemetry & Events | Limitation |
|---|---|---|---|---|
| [`all_pass1.jpeg`](all_pass1.jpeg) | Live MQTT (`build\dash-mqtt-20261120-213250.txt`), `2026-11-20` | `10 / 10` DONE (`ACTIVE` MQTT telemetry) | Chronological timeline from `CMD START` (`21:33:45.870`) $\rightarrow$ `NAV_LINE_TRACKING` (`fwd 147 cm`) $\rightarrow$ `OBSTACLE` (`24 cm @ 80 deg`, `width 95 mm`, `clr L 635 / R 635 mm`) $\rightarrow$ `REJOIN` (`24.6 s`, steady `>= 3 s`) $\rightarrow$ `BARCODE` `U-TURN (F)` (`21:34:43.570`) $\rightarrow$ `JUNCTION -> NAV_U_TURN -> NAV_LINE_TRACKING` $\rightarrow$ `HUMP` (`height 16 mm`, `pitch -10.0 deg`) $\rightarrow$ `PICKUP / SAFETY_FAULT_LATCHED` (`21:35:45.884`); `1407` MQTT frames, `134` heartbeats. | No paired camera video or committed raw `.txt` log file |
| [`all_pass2.jpeg`](all_pass2.jpeg) | Live MQTT (`2026-11-20..26`) | `10 / 10` DONE (`ACTIVE` MQTT telemetry) | `3` barcodes detected (`LEFT (F)`, `LETTER Z`, `LETTER Q`), `LEFT (F) -> NAV_TURN_LEFT` (`13:52:45.754`), `2` obstacle profiles (`24 cm @ 80 deg`, `width 160 mm`, `L 635 / R 635 mm`, `rejoined 27.4 s`, `bypass complete 2x`), `peak 9 mm` hump, `2475` MQTT frames, `232` heartbeats, `STOPPED & LATCHED` (`FAULT_LATCHED`). | No paired camera video or committed raw `.txt` log file |
