# Data representation (minimum-bit, Week 8)

Agreed field table for every inter-task message, shared word, state machine,
eventflag bit and the MQTT wire format (ARCHITECTURE.md D8, D9, D10). It is
**language-neutral**: the dashboard language is not decided, and no decoder
exists. Nothing here is implemented yet. Batch B-baseline builds the unpacked
Week 6 types and measures them; Batch B-packed adds the shared header and host
tests (section 10).

## 1. General rules

- Masks and shifts only. No C bit-fields anywhere.
- One shared, pure header (`<stdint.h>` only) will hold every `*_BITS`,
  `*_SHIFT`, `*_MASK`, the wire version and the pack/unpack functions
  (proposed `src/common/data_repr.h`, prefix `drep_`, Batch B-packed). It is
  included by SDK-side drivers, kernel-side tasks and host tests.
- The header separates the **wire / message formats** (fixed, versioned) from
  **private task state**. Private state sits in its own section with its own
  accessors, so Buddy 1 can later switch private state to bit-fields (if the
  Barr C check allows it) without any wire or message change.
- Clamp before packing; a value out of range is clamped and the message's OOR
  (out-of-range) bit is set. Signed fields are two's complement and are
  sign-extended on unpack.
- Every multi-byte wire field is little-endian.
- Single writer per field; the writer is named in every table row.

"Baseline" = the Week 6 natural `<stdint.h>` struct (ARM EABI alignment),
estimated; B-baseline measures it. "MBF slot" = message-buffer bytes per
message: 4-byte header + size rounded up to 4 (`messagebuf.h:58-62`,
`messagebuf.c:65`).

## 2. Messages

Columns: Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy.

### 2.1 `nav_input_t` (MBF_NAV_IN, 8 slots)

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nav_input_t | source | LINE, BARCODE, AVOID, REMOTE | 1 | no | 2 | 4 (whole struct) | 4 / 8 (whole) | 4 sources | reject unknown | sender | VC | 2 |
| nav_input_t | type | per source: STEER, JUNCTION, TRACK_LOST; CMD; SLOW, BYPASS, RELEASE, BLOCKED; START, STOP, RESET | 1 | no | 3 | | | ≤ 8 types per source | reject unknown | sender | VC | 2 |
| nav_input_t | payload | steer −128..+127 (Q7, ±1.0 normalised), or barcode command 2 bits, or remote command 2 bits | 1/128 for steer | steer yes | 8 | | | steer error is a normalised ratio | clamp ±1.0, OOR | sender | VC | 2 |
| nav_input_t | bypass command | stop, continue, left, right, reverse (with type BYPASS) | 1 | no | 3 | | | 5 values; replaces "bypass state" from `obstacle_profile_t`. TODO(owner-to-confirm, buddy 5) | reject 5-7 | Obstacle Avoidance | VC | 5 |
| nav_input_t | OOR | 0/1 | | no | 1 | | | clamp marker | | sender | VC | 2 |

Total 2 + 3 + 8 + 3 + 1 = 17 bits → `uint32_t` (the bypass command may share
the payload byte instead, which gives 14 bits → `uint16_t`; either way the MBF
slot stays 8 B, so the RAM saving is 0 B).

### 2.2 `motion_cmd_t` (MBF_MOTION_CMD, 4 slots)

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| motion_cmd_t | type | STOP, FWD, BACK, TURN_L, TURN_R, UTURN, STEER, BRAKE | 1 | no | 3 | 10 (slot 16) | 4 / 8 | 8 commands | reject unknown | VC | Motion | 2 |
| motion_cmd_t | speed | 0-50 cm/s | 1 cm/s | no | 6 | | | NFR2 cap 50 cm/s; 51 values ≤ 64 | clamp to 50, OOR | VC | Motion | 2 |
| motion_cmd_t | arg (by type) | FWD/BACK distance 0-511 cm (0 = continuous); TURN_L/TURN_R angle 0-180°; STEER steer −256..+255 | 1 cm / 1° / 1 step | STEER only | 9 | | | one argument per command type covers the missing distance and steering fields | per-type range check (below), OOR | VC | Motion | 2 |
| motion_cmd_t | OOR | 0/1 | | no | 1 | | | | | VC | Motion | 2 |

Total 19 bits → `uint32_t`. Per-type range check: distance ≤ 511, angle ≤ 180,
steer −256..+255, STOP/UTURN/BRAKE arg = 0. Planned host tests (Batch
B-packed): one per type, IDs `HOST-MC-01`..`HOST-MC-08` (STOP, FWD, BACK,
TURN_L, TURN_R, UTURN, STEER, BRAKE), each at min, max, min−1 and max+1.

### 2.3 `obstacle_profile_t` (MBF_OBSTACLE, 2 slots)

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| obstacle_profile_t | distance | 2-400 cm; 0 = no echo | 1 cm | no | 9 | 16 (slot 20) | 5 / 12 | HC-SR04 range; raw 0-511 fits 9 bits; 0 replaces `valid` | clamp to 400, OOR | Obstacle Scanning | Obstacle Avoidance | 5 |
| obstacle_profile_t | bearing index | 0-12 (30° + 10° × i) | 10° | no | 4 | | | 13 values ≤ 16 | clamp 0..12, OOR (fine-scan ±30° around 30°/150° can reach 0°/180°) | Obstacle Scanning | Obstacle Avoidance | 5 |
| obstacle_profile_t | width | 0-400 mm | 5 mm | no | 7 | | | 81 values ≤ 128 | clamp, OOR | Obstacle Scanning | Obstacle Avoidance | 5 |
| obstacle_profile_t | clear_L | 0-635 mm | 5 mm | no | 7 | | | specs.md left-side clearance; 127 = "≥ 635, clear" | saturate at 127 | Obstacle Scanning | Obstacle Avoidance | 5 |
| obstacle_profile_t | clear_R | 0-635 mm | 5 mm | no | 7 | | | specs.md right-side clearance; saturating | saturate at 127 | Obstacle Scanning | Obstacle Avoidance | 5 |
| obstacle_profile_t | sequence (optional) | 0-3, wraps | 1 | no | 2 | | | lets Avoidance spot a stale or repeated profile; kept until the owner decides. TODO(owner-to-confirm, buddy 5) | wraps | Obstacle Scanning | Obstacle Avoidance | 5 |
| obstacle_profile_t | OOR | 0/1 | | no | 1 | | | | | Obstacle Scanning | Obstacle Avoidance | 5 |

Total 9 + 4 + 7 + 7 + 7 + 2 (sequence) + 1 (OOR) = 37 bits → 5 B on the wire
(12 B MBF slot); 35 bits if the sequence is dropped, still 5 B. The timestamp
is **dropped**, and "bypass state" is **removed** (it is Avoidance's own state;
the 3-bit bypass command goes in `nav_input_t` instead, flagged for Buddy 5).

**Newest-wins needs drain-then-send.** design.md 3.3 says "Obstacle Scanning
sends with TMO_POL (newest profile wins)". A `tk_snd_mbf` with `TMO_POL` on a
full buffer does **not** overwrite: it returns `E_TMOUT` (free-space check at
`external/mtk3_bsp/kernel/tkernel/messagebuf.c:315`, `ercd = E_TMOUT` at :319,
wait only `if ( tmout != TMO_POL )` at :320; range 315-321). So for newest-wins the sender
first drains with `tk_rcv_mbf(..., TMO_POL)` until it returns `E_TMOUT`, then
sends. Notes:

- Race: if Avoidance is already waiting in `tk_rcv_mbf(TMO_FEVR)`, the buffer
  is empty, so the drain finds nothing and the send hands the message straight
  to the waiting receiver. If Avoidance (P7) becomes ready between the drain
  and the send it cannot run before Scanning (P6) finishes the send, because
  Scanning has the higher priority; the only loss is an old profile, which is
  the intent.
- With 2 slots the drain is needed only when the buffer is full: try the send
  first, and drain + resend only on `E_TMOUT`.
- Sequence-number analysis: with drain-then-send, Avoidance never sees an
  older profile after a newer one, so the 2-bit sequence adds detection of a
  *repeated* reading (no new scan since the last receive) and of a lost
  profile (gap of 2). Without drain-then-send it is the only way to tell that
  a received profile is stale. It costs 2 bits and no MBF bytes (the slot is
  12 B either way). Recommendation for Buddy 5: keep it until the drain is
  implemented and tested, then decide. TODO(owner-to-confirm, buddy 5).

### 2.4 `motion_state_t` (MTX_MOTION; shared data, not a queue)

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| motion_state_t | speed | ±51.1 cm/s | 0.1 cm/s | yes | 10 | 20 (whole) | 16 (whole) / n/a | NFR2 cap 50 cm/s | clamp, OOR | Motion | BC, Telemetry | 2 |
| motion_state_t | distance | 0-65535 cm | 1 cm | no | 16 | | | wraps at 655 m (one run < 655 m, assumption) | wraps | Motion | BC, Telemetry | 2 |
| motion_state_t | spare / OOR | | | | 6 | | | | | Motion | | 2 |
| motion_state_t | enc_count[2] | int32 | 1 count | yes | 2 × 32, unpacked | | | see 2.8 | wraps (deltas) | Motion | BC, Telemetry | 2 |
| motion_state_t | tick | ms | 1 ms | no | 32 | | | kernel time of the sample | wraps | Motion | BC, Telemetry | 2 |

### 2.5 `tel_evt_t` (MBF_TELEMETRY, 32 slots)

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| tel_evt_t | type | ≤ 32 event types | 1 | no | 5 | 16 (slot 20) | 8 / 12 | | reject unknown | any producer | Telemetry | 1 |
| tel_evt_t | timestamp | 0-16,777,215 ms (4.66 h, wraps) | 1 ms | no | 24 | | | dashboard handles the wrap | wraps | any producer | Telemetry | 1 |
| tel_evt_t | flags | | | no | 3 | | | OOR + 2 spare | | any producer | Telemetry | 1 |
| tel_evt_t | payload | per type (hump, pitch, barcode, fault bits, ...) | | | 32 | | | one word per type | per field | any producer | Telemetry | 1 |

### 2.6 Small fields

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hump report | height | 0-100 mm | 1 mm | no | 7 | in tel_evt_t | in payload | 101 values ≤ 128 | clamp 100, OOR | IMU | Telemetry | 4 |
| pitch | pitch | ±45° | 0.5° | yes | 8 | in tel_evt_t | in payload | int8 Q1 covers −64.0..+63.5°; ±45° = 181 values | clamp ±45°, OOR; sign-extend on unpack | IMU | Safety, Telemetry | 4 |
| barcode command | cmd | A-D (LEFT, RIGHT, STRAIGHT, U-TURN) | 1 | no | 2 | 1 | in nav_input_t payload | invalid reads are never posted | n/a | Barcode Decode | VC | 3 |
| steering sign | steer_err | > 0 = line to the car's RIGHT = steer right (proposal) | | yes | | | | sensors are CENTRE + RIGHT (D11), the mirror of design.md; redefined, not copied. TODO(owner-to-confirm, buddy 3) | | Line-Following | VC | 3 |

### 2.7 Barcode edge ring (IF-09)

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| edge ring | gap | 0-1,048,575 µs | 1 µs | no | 20 (`BC_GAP_BITS`) | 64 × 8 = 512 | 160 (lo16[64] = 128 B + hi-nibble[32] = 32 B) | `BC_CAPTURE_TIMEOUT_MS` 1000 × 1000 = 1,000,000 µs < 2^20 = 1,048,576 (4.6 % margin), checked by `#error` in `timing.h` and `_Static_assert` in `config_check.h` | saturate at 0xFFFFF = start of capture / timeout | barcode ISR | Barcode Decode | 3 |
| edge ring | head | 0-255 (mod 64 index) | | no | 8 | | | free-running | | barcode ISR | Barcode Decode | 3 |
| edge ring | tail | 0-255 | | no | 8 | | | free-running | | Barcode Decode | barcode ISR | 3 |
| edge ring | overflow count | 0-255 | | no | 8 | | | task compares with its snapshot → INVALID_READ_RESET | wraps | barcode ISR | Barcode Decode | 3 |

### 2.8 Encoder count words (IF-10): the unpacked exception

| Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| encoder counts | count[left], count[right] | int32, free-running | 1 X4 count | yes | 32 each, aligned | 8 | 8 | single aligned 32-bit store/load is atomic on the M0+; wrap bound below | wraps; two's-complement deltas | Motion task only (reads PIO0 via `enc_get_count()`) | BC, Telemetry (via motion_state_t) | 2 |

Wrap bound (`config/thresholds.h`): counts = `ENC_QUAD_MULT × ENC_CPR ×
distance_mm / WHEEL_CIRC_MM`, so a count wraps after
`(2^31 − 1) × WHEEL_CIRC_MM / (4 × ENC_CPR)` mm of one-way driving. `ENC_CPR`
and `WHEEL_CIRC_MM` are not measured yet (TODO(owner-to-confirm, buddy 2)).
CPR-independent bound: the PIO decoder counts at most 12.5 Msteps/s, so a wrap
takes at least 171.8 s even at the decoder limit; per-period deltas stay exact.

## 3. EVF_SAFETY bits (D9)

One eventflag, 32-bit pattern (`UINT`), about 10 bits used. Bit positions are
proposed; Batch B fixes them in the shared header.

| Bit | Name | Setter (single) | Clearer | In Safety's wait mask | Counted as fault |
|---|---|---|---|---|---|
| b0 | COLLISION | IMU Processing | Safety (`TWF_BITCLR` on release) | yes | yes (latched) |
| b1 | IMU_ALIVE | IMU Processing (every sample) | Safety (`TWF_BITCLR`) | yes | no (watchdog input) |
| b2 | FAULT_LATCHED (was "collision latch") | Safety only: collision or IMU watchdog trip | Vehicle Controller only (`car/cmd/reset` while stationary) | no | yes |
| b3 | FAULT_STALL | Motion | Motion | no | yes |
| b4 | FAULT_ENCODER | Motion | Motion | no | yes |
| b5 | FAULT_IMU_I2C | IMU Processing | IMU Processing | no | yes |
| b6 | FAULT_IMU_WDOG | Safety (`SAFETY_IMU_WDOG_MS` timeout) | Safety | no | yes (also sets FAULT_LATCHED) |
| b7 | STATUS_NO_ECHO | Obstacle Scanning | Obstacle Scanning | no | **no**: informational ("no echo = nothing in range", design.md 3.3) |
| b8 | FAULT_WIFI_LOSS | Telemetry | Telemetry | no | yes (never stops the car) |
| b9 | FAULT_IR_SENSOR | Line-Following (`hal_adc_sanity_check()` at boot) | Line-Following | no | yes (logged and counted, never acts on motors) |

- Safety: `tk_wai_flg(EVF_SAFETY, COLLISION | IMU_ALIVE, TWF_ORW | TWF_BITCLR, &ptn, SAFETY_IMU_WDOG_MS)`.
  `TWF_BITCLR` clears only the waited bits (`flgptn &= ~waiptn`,
  eventflag.c:170-171, 260-261) and the returned pattern is captured before the
  clear (eventflag.c:165, 257). Never `TWF_CLR`, which zeroes the whole word.
- The flag is `TA_WSGL`: a second `tk_wai_flg` (even `TMO_POL`) while Safety
  waits returns `E_OBJ` (eventflag.c:249-251). VC, Motion and every other
  reader use `tk_ref_flg`.
- To clear bit B: `tk_clr_flg(EVF_SAFETY, ~B)` (it ANDs, eventflag.c:204).

EVF_NAV is unchanged: AVOID_ACTIVE, LINE_REACQUIRED (Obstacle Avoidance),
LINE_FOUND (Line-Following); 3 bits; readers poll.

## 4. State machine widths

| FSM | States | Count | Bits |
|---|---|---|---|
| System (design.md section 5, `[image2]`) | top level: INITIALISING, READY_WAITING_FOR_START, OBSTACLE_AVOIDANCE/BYPASS, SAFETY_COLLISION_LATCHED_FAULT; AUTONOMOUS_NAVIGATION sub-states: LINE_TRACKING, HUMP_SENSING, TURN_LEFT, GO_STRAIGHT, U_TURN, TURN_RIGHT | 10 | **4 flat** |
| Line-following (6.1) | ON_LINE, JUNCTION_DETECTED, LINE_LOST, REACQUIRING_TRACK, SUSPENDED_BY_AVOIDANCE, TRACK_LOST | 6 | 3 |
| Barcode (6.2) | IDLE, EDGE_CAPTURE_READING, VALIDATING_CHECKSUM, COMMAND_READY, INVALID_READ_RESET | 5 | 3 |
| Obstacle (6.3) | SWEEPING_COARSE, FINE_SCAN_LOCALIZATION, BYPASS_DECISION, LEAVE_LINE, PASS_OBSTACLE, RETURN_TO_LINE, LINE_REACQUIRED_HANDOFF, BLOCKED_STOP (+ composite EXECUTING_BYPASS) | 8-9 | 4 |
| IMU / terrain (6.4) | STATIONARY, ACCELERATING, CRUISING, TURNING, HUMP_DETECTED, COLLISION_FAULT | 6 | 3 |
| WiFi / MQTT (6.5) | DISCONNECTED, CONNECTING, CONNECTED, DEGRADED_BUFFERING, RECONNECTING | 5 | 3 |
| VC authority | LINE, AVOID | 2 | 1 |
| Avoidance plan | stop, continue, left, right, reverse | 5 | 3 |

System FSM recount (Week 8 step, owner decision: 3 bits only if 8 or fewer
states). The statechart was extracted from design.md (`[image2]`, PNG, native
size 660 × 409 px; no higher resolution exists in the document) and counted
at native resolution. POWER_OFF is excluded: the legend marks it "Power
state", not a software state. AUTONOMOUS_NAVIGATION is a composite, so its six
sub-states are counted instead; the diamond is a choice pseudo-state, not a
state. Result: 4 + 6 = **10 states > 8, so 4 bits flat**. The small labels
are legible at native size only approximately (for example HUMP_SENSING is
read as "HUMP_SENSING: slow, log peak"); the count does not depend on the
exact wording. Obstacle states span two tasks (Scanning P6, Avoidance P7): each
task keeps its own state private, and Avoidance composes the 4-bit reported
state (single writer).

## 5. RAM fault record (Batch B)

| Field | Bits | Notes |
|---|---|---|
| magic | 32 | first word; fixed constant chosen in Batch B |
| payload | n × 32 | fault type, PC/LR/xPSR at the fault, kernel error code, boot count; layout fixed in Batch B |
| crc32 | 32 | CRC-32 over the payload |

Kept in the SDK's `.uninitialized_data` (`__uninitialized_ram()`, NOLOAD,
never zeroed by crt0; not `.noinit`). Read over SWD at any time. On the next
boot the debug drain prints it over USB CDC and clears it **only if the magic
and the CRC both match**; otherwise RAM is random (power-on, flashing) and the
record is silently re-initialised (docs/DEBUG.md, ARCHITECTURE.md D6).

## 6. RAM: what bit-packing changes (Week 13)

Bit-packing barely changes RAM: message-buffer slots round up to whole words
plus a 4-byte header, and the buffers live inside the fixed 96 KB kernel pool.

Only what the map file shows is reported. No firmware map exists yet (firmware
target skipped until Batch B), so no RAM figures are reported now. Expected
from the arithmetic above, for planning only: MBF_NAV_IN 64 → 64 B,
MBF_MOTION_CMD 64 → 32 B, MBF_OBSTACLE 40 → 24 B, MBF_TELEMETRY 640 → 384 B
(all inside the pool, so the map does not move); edge ring 512 → 160 B (a
static array, visible in the map).

## 7. Baseline plan

1. **B-baseline:** port layer, `main.c`, `usermain`, `dbg_` ring and drain on
   USB CDC, all nine tasks as stubs at design priorities and periods, every
   kernel object at its Week 6 depth, the Week 6 **unpacked** types in one
   header, the 64 × 64-bit edge ring. Measure `sizeof` per type
   (`arm-none-eabi-nm -S` on a never-linked size-report object), the map file
   (`arm-none-eabi-size -A`), MBF bytes and pool use. Tag
   `baseline/week6-unpacked` locally; the owner pushes it.
2. **B-packed:** add the shared header and pack/unpack functions, switch every
   type, add host tests (round trip at min, max, ±limit, out of range; sign
   extension; per-type `motion_cmd_t`; obstacle sentinel and saturation; edge
   ring wrap and overflow; frame byte order and reject rules), then re-measure
   with the same commands and fill in the "packed" columns.

## 8. MQTT binary frames (D10)

Frame: `ver u8 | type u8 | len u8 | payload[len]`, little-endian, masks and
shifts. A received command is accepted only if `ver` equals the wire version,
`type` is known, `len` equals the expected length for that type and the MQTT
payload length equals `3 + len`. Anything else is logged through `dbg_`,
counted, and never posted to MBF_NAV_IN, so it never reaches the Vehicle
Controller.

Planned payloads (each starts with a 24-bit millisecond timestamp, except the
heartbeat and the commands):

| Type | Binary payload (bits → bytes) | Fields |
|---|---|---|
| telemetry_motion | 24 + 10 + 16 + 2 × 32 = 114 → 15 | ts, speed, distance, enc_l, enc_r |
| telemetry_sensor | 24 + 12 + 12 + 9 + 8 + 8 = 73 → 10 | ts, IR_R raw, IR_C raw, ultrasonic cm, pitch, roll (roll uses the pitch format: ASSUMPTION) |
| event_barcode | 24 + 2 = 26 → 4 | ts, command |
| event_hump | 24 + 7 + 8 + 1 = 40 → 5 | ts, height, pitch, OOR |
| event_obstacle | 24 + 37 = 61 → 8 | ts, obstacle_profile_t fields incl. the 2-bit sequence and OOR |
| event_fault | 24 + 10 = 34 → 5 | ts, EVF_SAFETY bits b0-b9 |
| heartbeat | 32 + 4 + 10 = 46 → 6 | uptime ms, system state, fault bits |
| cmd_start, cmd_stop, cmd_reset | 0 | type only |

### Reference JSON baseline

`tools/json_baseline/*.json` holds one minified reference JSON per message
type. They are a **reference baseline invented for size comparison, not a
protocol**: design.md defines no JSON schema (it names only FR1 "motion,
sensor, and event data", `car/telemetry/*`, heartbeat and `car/cmd/start|stop|reset`).
Field names are snake_case and values are at their widest printed form. Output
of `cmake -P tools/json_sizes.cmake`:

| type | JSON bytes | binary frame bytes | JSON / binary |
|---|---|---|---|
| cmd_reset | 15 | 3 | 5.0 |
| cmd_start | 15 | 3 | 5.0 |
| cmd_stop | 14 | 3 | 4.7 |
| event_barcode | 52 | 7 | 7.4 |
| event_fault | 66 | 8 | 8.3 |
| event_hump | 66 | 8 | 8.3 |
| event_obstacle | 128 | 11 | 11.6 |
| heartbeat | 90 | 9 | 10.0 |
| telemetry_motion | 113 | 18 | 6.3 |
| telemetry_sensor | 105 | 13 | 8.1 |
| total | 664 | 83 | 8.0 |

## 9. Dashboard

The dashboard language is not decided. This document is the field table it
must follow (sections 2 and 8); no decoder is written. Risk R3 (dashboard and
firmware drifting apart) is handled by generating or checking the dashboard
against the shared header once both exist.

## 10. Status

Documentation only. No packed types, pack/unpack code or encoder code exist
yet (Batch B).
