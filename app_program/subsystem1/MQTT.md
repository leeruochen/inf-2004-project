# Subsystem 1: communication, MQTT and telemetry

Owner: Buddy 1, Damian (2501299). Code: `app_program/subsystem1/`. Tests:
`testing/subsystem1/`, `tests/host/test_tel.c`, `tools/dashboard/test_tel_decode.py`.

This document covers the `specs.md` deliverables for Buddy 1: the
communication API, the telemetry framework, the MQTT documentation and how to
demonstrate telemetry reporting. It covers FR1 to FR3, NFR1, assumptions A3
and A9, statechart 6.5 and deltas D2/D10.

---

## 1. What runs where

```
 any task ── tel_post_*() ──► MBF_TELEMETRY (32) ─┐            TMO_POL, never waits
 S3/S4/S5 ── tel_set_*()  ──► snapshot words  ────┤            single aligned stores
 S5 ──────── tel_post_obstacle() ► seq-locked slot┤
 S2 Motion ─ motion_get_state() (MTX_MOTION) ─────┤
                                                  ▼
                       ┌──────────── Telemetry task, P9, wakes every 20 ms ────────────┐
                       │ frames (drep_frame_*) → outbox (events) / direct (periodic)   │
                       │ link state machine (tel_link.c) → connect / teardown / backoff│
                       │ backend: console "[TEL] <topic> <hex>"   or   drv_wifi.c       │
                       └───────────────────────────────┬───────────────────────────────┘
                                                       │ CYW43 + lwIP, poll mode
                                     WiFi ◄────────────┘ (all network work inside this task)
 MQTT broker ◄──► dashboard (tools/dashboard/tel_dashboard.py)
 car/cmd/* ──► Telemetry task: tel_wire_cmd() check ──► MBF_NAV_IN (5 ms) ──► Vehicle Controller
```

| File | Side | What it does |
|---|---|---|
| `subsystem1.h` | public | The `tel_` API other buddies call (section 2) |
| `subsystem1.c` | kernel | `subsystem1_init()` (nothing to create) and `subsystem1_start()` (starts the task) |
| `task_tel.c/.h` | kernel | The Telemetry task, the producer API, the snapshot, the command receive path |
| `tel_link.c/.h` | pure C | Connection state machine and backoff (statechart 6.5), host-tested |
| `tel_outbox.c/.h` | pure C | Event frame ring, drop-oldest, host-tested |
| `tel_wire.c/.h` | pure C | Topic tree, command check, hex, host-tested |
| `drv_wifi.c/.h` | SDK | CYW43 + lwIP MQTT client (only with `TEL_BACKEND_WIFI=ON`) |
| `lwipopts.h`, `tel_cyw43_config.h` | SDK config | lwIP sizing; CYW43 driver with its `printf` silenced |

The frame layouts themselves (`drep_frame_*`) are platform code in
`src/common/data_repr.h`, from Batch B stage B7.

## 2. Communication API (`subsystem1.h`)

All calls work from any task, never from an ISR, and **never wait** (NFR1).

| Call | Who calls it | Effect |
|---|---|---|
| `tel_post(evt)` / `tel_post_event(type, payload, oor)` | anyone | Queue a `tel_evt_t` (`TMO_POL`). Returns `false` and counts a drop if the queue is full |
| `tel_post_hump(height_mm, pitch_half_deg)` | Buddy 4 | Hump event (0-100 mm, ±45° in 0.5° steps, clamped with OOR) |
| `tel_post_barcode(DREP_BC_CMD_*)` | Buddy 3 | A validated barcode command |
| `tel_post_fault(evf_bits)` | Buddy 4 / anyone | `EVF_SAFETY` bits at the moment a fault was raised |
| `tel_set_ir_raw(ir_r, ir_c)` | Buddy 3, Line-Following | Latest raw ADC values (12 bit) for the 200 ms sensor frame |
| `tel_set_range_cm(cm)` | Buddy 5, Scanning | Latest forward range, 0-400 cm (0 = no echo) |
| `tel_set_attitude(pitch, roll)` | Buddy 4, IMU | Latest pitch and roll in 0.5° units |
| `tel_post_obstacle(prof)` | Buddy 5, one task | Newest obstacle profile, all 37 bits; published once |
| `tel_get_stats(&st)`, `tel_is_running()` | anyone | Counters (section 7) |

The Vehicle Controller already sends `DREP_TEL_STATE` events. The Telemetry
task turns each one into an immediate heartbeat, which carries the 4-bit
system state.

**Proposed interface IF-12 (needs the team's OK in `TEAM.md` section 5).**
The `tel_set_*()` snapshot replaces "post every sample into MBF_TELEMETRY".
That would push 200 IR samples a second into a 32-slot queue. Each setter
does one aligned 32-bit store (atomic on the M0+, one writer per word), and the
Telemetry task reads the newest value every 200 ms. `tel_post_obstacle()`
settles the open question in `docs/DATA_REPRESENTATION.md` section 10: a
37-bit profile does not fit the 32-bit `tel_evt_t` payload, so it goes through
a sequence-locked two-word slot instead, with no bits dropped and no second
event.

## 3. Topic tree, QoS and payloads

| Topic | Direction | Frame type | When | QoS | Retain |
|---|---|---|---|---|---|
| `car/telemetry/motion` | car → | 1 | every 200 ms | 0 | no |
| `car/telemetry/sensor` | car → | 2 | every 200 ms, once a producer has written a value | 0 | no |
| `car/telemetry/barcode` | car → | 3 | event | 0 | no |
| `car/telemetry/hump` | car → | 4 | event | 0 | no |
| `car/telemetry/obstacle` | car → | 5 | event | 0 | no |
| `car/telemetry/fault` | car → | 6 | event, also when the link is lost | 0 | no |
| `car/telemetry/heartbeat` | car → | 7 | every 1 s, and at once on a system state change | 0 | no |
| `car/status` | car → | 1 byte: `'1'` online, `'0'` offline | on connect; `'0'` is the MQTT last will | 1 | yes |
| `car/cmd/start`, `/stop`, `/reset` | → car | 8, 9, 10 (3 bytes) | operator | 1 (subscription) | no |

**Why QoS 0 for telemetry.** A periodic frame is stale 200 ms later, and the
broker is on the same LAN. QoS 1 would hold a request slot per message until
PUBACK. QoS 2 was already rejected in Week 8 (extra handshakes). Events are
protected against outages by the outbox (section 4), not by MQTT
acknowledgements.

**Why QoS 1 for commands.** At-least-once delivery. A duplicate START, STOP
or RESET is harmless.

**Payloads** are the binary frames of `docs/DATA_REPRESENTATION.md` section 8
(ARCHITECTURE.md D10): `ver u8 | type u8 | len u8 | payload`. Fields are
packed LSB-first and little-endian, with masks and shifts only.

| Type | Payload fields, in bit order | Bytes on the wire |
|---|---|---|
| 1 motion | ts 24, speed 10 (signed, 0.1 cm/s), distance 16 (cm), enc_l 32, enc_r 32 (signed) | 18 |
| 2 sensor | ts 24, IR_R 12, IR_C 12, range 9 (cm), pitch 8, roll 8 (signed, 0.5°) | 13 |
| 3 barcode | ts 24, command 2 (0 L, 1 R, 2 straight, 3 U-turn) | 7 |
| 4 hump | ts 24, height 7 (mm), pitch 8 (signed, 0.5°), OOR 1 | 8 |
| 5 obstacle | ts 24, distance 9, bearing index 4 (30° + 10° × i), width 7, clear_L 7 (×5 mm), seq 2, OOR 1, clear_R 7 (×5 mm) | 11 |
| 6 fault | ts 24, EVF_SAFETY b0-b9 | 8 |
| 7 heartbeat | uptime 32 (ms), system state 4, EVF_SAFETY b0-b9 | 9 |
| 8/9/10 command | none | 3 |

`ts` is the kernel time in ms, 24 bits (it wraps every 4.66 h; the dashboard
handles that).

**Receive check (FR2, D10).** A message on `car/cmd/...` reaches the Vehicle
Controller only if **all** of these hold:

- the version byte is 1;
- the type is known and is a command;
- the `len` field and the received length match;
- the topic matches the command (a STOP frame on `car/cmd/start` is rejected).

Anything else is logged (`[TEL] cmd rejected: BAD_VERSION ...`), counted, and
never posted. The check is `tel_wire_cmd()`, built on `drep_cmd_to_nav()`.

**Bytes per second.** One MQTT PUBLISH costs 2 bytes of fixed header, 2 bytes
of topic length, the topic, then the frame.

| Stream | Binary (topic + frame) | JSON baseline (same topic) |
|---|---|---|
| motion, 5/s | 5 × 42 = 210 B/s | 5 × 137 = 685 B/s |
| sensor, 5/s | 5 × 37 = 185 B/s | 5 × 129 = 645 B/s |
| heartbeat, 1/s | 36 B/s | 117 B/s |
| **total** | **431 B/s (3.4 kbit/s)** | **1447 B/s** |

Binary frames cut the steady-state traffic by 70 %. The topic strings are now
more than half of what is left. Shortening them (for example `c/m` instead of
`car/telemetry/motion`) would save about 45 % more, at the cost of a less
readable broker. They are one table in `tel_wire.c` plus `TOPIC` in the
dashboard, if the team wants that.

## 4. Connection handling (FR3, statechart 6.5)

Five states, encoded in 3 bits (`DREP_FSM_WIFI_STATES`), in `tel_link.c`:

| From | Event | To | Action |
|---|---|---|---|
| DISCONNECTED | task start | CONNECTING | join WiFi → DHCP → DNS → MQTT CONNECT |
| CONNECTING | CONNACK accepted | CONNECTED | backoff reset to 1 s; subscribe `car/cmd/#`; `car/status` = `'1'` |
| CONNECTING | definite failure (bad password, no AP, broker refused), or 15 s without success | RECONNECTING | teardown; wait the backoff |
| CONNECTED | session or WiFi lost, or a publish refused | DEGRADED_BUFFERING | events go to the outbox; periodic frames are skipped |
| DEGRADED | network up, publishes accepted, outbox empty | CONNECTED | none |
| DEGRADED | no progress for T_ka = 5 s | RECONNECTING | teardown (leave the AP if WiFi is down) |
| RECONNECTING | backoff expired | CONNECTING | next attempt |

The backoff is 1 s, 2 s, 4 s, 8 s, 16 s, then 30 s, repeating at 30 s.

| Setting | Value | Where |
|---|---|---|
| Wake period | 20 ms | `TEL_WAKE_PERIOD_MS`, `config/timing.h` |
| Publish / heartbeat | every 10th / 50th wake = 200 ms / 1 s | `config/timing.h` |
| MQTT keep-alive, T_ka | 5 s | `TEL_KEEPALIVE_MS` |
| Backoff | 1 s to 30 s | `TEL_BACKOFF_MIN_MS`, `TEL_BACKOFF_MAX_MS` |
| Connect timeout | 15 s | `TEL_CONNECT_TIMEOUT_MS`, `task_tel.h` |
| Outbox | 16 event frames, the oldest dropped first | `TEL_OUTBOX_LEN`, `tel_outbox.h` |

**Outbox policy.** Only events (barcode, hump, obstacle, fault) are buffered.
They matter after an outage: the peak hump has to be reported. Motion, sensor
and heartbeat frames are not buffered, because the next one replaces them
within 200 ms. When the link is lost, a fault frame with `WIFI_LOSS` set goes
into the outbox, so the dashboard learns when the outage started.

**`EVF_SAFETY` b8 FAULT_WIFI_LOSS.** Telemetry is its only setter and only
clearer. It is set when the link drops after having been up once, and cleared
on reconnect. No control task acts on it. Link state never changes how the
car drives (accepted risk: the car cannot be stopped remotely while WiFi is
down; see design.md section 8.3).

## 5. Why the network can never stall the car (NFR1, A3, A9)

1. **Producers never wait.** Every post uses `TMO_POL`, and a full queue is a
   counted drop. Test S1-U04 times 64 posts at under 2 ms in total.
2. **One task owns the network.** `pico_cyw43_arch_lwip_poll`: lwIP and the
   CYW43 driver do their work only inside `cyw43_arch_poll()`, called by the
   Telemetry task. Every lwIP callback runs in that task. No interrupt handler
   calls the kernel. The CYW43 GPIO24 interrupt only marks work pending, on
   the SDK's shared IO_IRQ_BANK0 chain.
3. **Lowest application priority.** The task runs at P9. The one slow call,
   `net_init()` (radio firmware download, about 0.3-1 s at boot), is busy
   waiting that P1-P8 preempt at every tick.
4. **No `printf` on the network path.** A `printf` from P9 would contend for
   the stdio lock with the debug drain (P10) and could deadlock both. The
   CYW43 driver's `CYW43_PRINTF` is compiled out (`tel_cyw43_config.h`), and so
   are the arch debug output and lwIP diagnostics. A disassembly check of the
   WiFi build shows the only remaining `printf` callers outside the drain are
   SDK error paths (section 9).
5. **Bounded work per wake:** at most 32 events, 8 outbox frames and 4
   commands.

CYW43 claims PIO1 and two DMA channels at run time, as reserved in `TEAM.md`
section 3. PIO0 holds the wheel encoders. The WiFi build raises
`PICO_MAX_SHARED_IRQ_HANDLERS` from 4 to 8. USB, echo, barcode and CYW43
already need 4 shared handlers, so one more would otherwise panic at boot.

## 6. Building, flashing and running

**Console backend (default, no radio).** The normal build:

```powershell
& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
& $CM --build build/usb
```

The USB console shows `[TEL] car/telemetry/motion 01010f...` lines. To keep
the shared console readable, periodic frames are printed every 5th time
(`TEL_CONSOLE_PERIODIC_DIV`). Events are always printed. Save the console
and decode it with:

```
python tools/dashboard/tel_dashboard.py decode-log saved_log.txt
```

**WiFi backend:**

1. Copy `config/secrets.h.example` to `config/secrets.h` and fill in the
   SSID, password and broker address. The file is git-ignored; never commit
   it. The network must be **2.4 GHz**.
2. Start a broker on the laptop, on the same network. For example, Mosquitto
   with a `mosquitto.conf` containing `listener 1883 0.0.0.0` and
   `allow_anonymous true`, run as `mosquitto -c mosquitto.conf -v`. Allow port
   1883 through the laptop firewall.
3. Build in a fresh directory and flash `build/wifi/robocar.uf2`:

   ```powershell
   & $CM -S . -B build/wifi @common -DDBG_BACKEND=USB -DTEL_BACKEND_WIFI=ON
   & $CM --build build/wifi
   ```

4. Run the dashboard (needs `pip install paho-mqtt`):

   ```
   python tools/dashboard/tel_dashboard.py listen --broker <laptop IP> [--quiet]
   python tools/dashboard/tel_dashboard.py send start --broker <laptop IP>
   ```

   Ctrl+C prints the run summary, including the **peak hump** (mission
   requirement 5).

Console lines to expect from the WiFi build:

```
[TEL] backend: WiFi/MQTT
[TEL] link DISCONNECTED -> CONNECTING
[NET] WiFi up, IP 192.168.1.42
[NET] MQTT session up
[TEL] link CONNECTING -> CONNECTED
```

A status line follows every 60 s: `[TEL] CONNECTED tx=... txf=... off=... ev=... bad=... qd=... od=... cmd=a/r`.

## 7. Counters (`tel_get_stats`, 60 s status line)

| Field | Status line | Meaning |
|---|---|---|
| `frames_sent` | `tx` | frames handed to the network or console |
| `publish_fail` | `txf` | publishes refused (MQTT output buffer full, no session) |
| `offline_skipped` | `off` | periodic frames skipped while the link was down |
| `evt_rx` / `evt_invalid` | `ev` / `bad` | events received / rejected |
| `evt_post_drops` | `qd` | producer posts refused because MBF_TELEMETRY was full |
| `outbox_dropped` | `od` | buffered events dropped (oldest) during a long outage |
| `cmd_accepted` / `cmd_rejected` | `cmd=a/r` | remote commands |
| `connects`, `link_failures`, `wake_overruns` | | link history; missed 20 ms releases |

## 8. Resource use (measured, GCC 10.3.1, Debug)

| Build | Flash (text) | RAM (bss) | Change |
|---|---|---|---|
| Before subsystem 1 (repo as of 31 Oct) | 58,628 B | 111,644 B | |
| Console backend | 64,668 B | 112,132 B | +6.0 KB flash, +0.5 KB RAM |
| WiFi backend | 365,668 B | 134,744 B | +301 KB flash (of which about 230 KB is the CYW43 firmware blob), +22.6 KB RAM |

Inside the WiFi RAM: lwIP receive pool 12 KB (8 pbufs), lwIP heap 4 KB,
`cyw43_state` 2.4 KB, the static MQTT client with its 1 KB output ring 1.4 KB,
and the telemetry task state about 0.6 KB. The Telemetry stack stays at 4 KB +
512 B (`config/priorities.h`). **Check the `[STACK] tel` high-water line in
the 60 s report on the first WiFi run.**

## 9. Open items and residual risks

| # | Item | Status |
|---|---|---|
| D2 | 20 ms wake with a non-blocking poll (instead of a 200 ms blocking receive) | **Implemented. Recommend approving it.** Poll-mode lwIP must be serviced every few tens of ms, so a 200 ms blocking receive would delay TCP ACKs and keep-alives. FR1 rates are unchanged. Buddy 1's call: record it in ARCHITECTURE.md D2 |
| IF-12 | `tel_set_*()` snapshot and `tel_post_obstacle()` | Proposed; needs a row in TEAM.md section 5 and an OK from Buddies 3, 4 and 5 |
| A9 | micro T-Kernel + CYW43/lwIP coexistence | Designed for it (section 5), and both builds compile with 0 warnings. **Needs the 30-minute soak on the car (S1-M04).** |
| A3 | Recovery after a WiFi drop without control jitter | Needs S1-M03 on the car |
| R-SDK | SDK error paths can still `printf`: `cyw43_read_bytes error`, `F2 not ready` (SPI bus faults) and lwIP assertions (`panic`) | Only on a hardware fault or a bug. If one fires, Telemetry and the debug drain can stall; control tasks P1-P8 keep running. The soak test watches for it |
| Topics | Topic strings are more than half of the wire bytes | Optional shortening (section 3) |
| Budget | TEAM.md section 6 budgets 1 cyclic handler for S1 | Not needed (`app_period_wait`); can be released |
