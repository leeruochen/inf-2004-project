**INF2004 Embedded Systems Programming**

**Week 8 Tribunal 1 Template**

*Architecture Defence & Integration Readiness \| AI ideation, live update, AI-vetted gap check*

| Item | Details |
| :---- | :---- |
| Purpose | Defend Week 6 architecture, justify changes, and show readiness for partial integration. |
| Core workflow | Pre-submission AI ideation \-\> live Zoom-updated version \-\> final AI-vetted gap check |
| Assessment focus | Evidence, engineering judgement, negotiation, and ability to identify missing work |
| Submission format | Completed document plus Zoom recording and evidence links |

| Important: this submission has THREE versions inside one document Version A: Pre-tribunal AI ideation. Each buddy uses AI to ideate their subsystem position before Zoom. Version B: Live Zoom-updated version. Design decisions are updated during the Zoom discussion through challenge, negotiation, and constraint response. Version C: Final AI-vetted gap check. After discussion, AI is used as a reviewer to identify missing assumptions, risks, tests, or unclear decisions. Marks are not awarded for blindly following AI. Marks are awarded for critical evaluation and human engineering judgement; meaning, I want to see you guys discuss during zoom. |
| :---- |

# **Team Information**

| Item | Details |
| :---- | :---- |
| Group ID | IS16 |
| Repository Link | https\://github.com/leeruochen/inf-2004-project |

| Buddy | Student ID | Primary subsystem / persona |
| :---- | :---- | :---- |
| Buddy 1 | 2501299 (Damian) | Network Gatekeeper: WiFi, MQTT, telemetry |
| Buddy 2 | 2500530 (Ruo Chen) | Real-Time Taskmaster: motors, encoders, PID |
| Buddy 3 | 2503319 (Jin Yong) | Track Purist: IR sensors, line following, barcode |
| Buddy 4 | 2500709 (Vincent) | Physics Watchdog: IMU, terrain, hump/motion events |
| Buddy 5 | 2500596 (Chay Han) | Spatial Optimizer: ultrasonic, servo, obstacle profiling |

#  **Version A: Pre-Tribunal AI Ideation**

| Instructions • Each buddy must use AI before Zoom session to prepare subsystem-specific arguments, risks, and negotiation points. • The AI output must be critiqued. Identify what is useful, unrealistic, incomplete, or requires validation. • For Tribunal 1, compare against the Week 6 architecture freeze. For Tribunal 2, compare against actual integration evidence. |
| :---- |

| Buddy | AI prompt used | Useful AI ideas | Weak / unrealistic AI ideas | Position before Zoom |
| :---- | :---- | :---- | :---- | :---- |
| Buddy 1 | Act as a strict embedded systems reviewer. Our Pico W car runs micro T-Kernel. Telemetry/MQTT is the lowest-priority task (P9) and is the only task that calls lwIP. It publishes every 200 ms, sends a heartbeat every 1 s, uses a 32-slot queue that drops messages when full, and reconnects with 1–30 s backoff. Which requirement changes would break this design, and what would each cost in CPU time, RAM and timing? | 1\) A faster publish rate (e.g. 50 ms) multiplies lwIP/CYW43 work by 4; at P9 the task may simply never get CPU time under load, so measure the cost of one publish before agreeing to any rate. 2\) If a remote command becomes safety-relevant (e.g. a remote emergency stop), its receive path runs at P9 and waits behind publishing, so it needs a faster path. 3\) State the RAM cost of the 32 × tel\_evt\_t queue in bytes, so a request for “more telemetry” can be answered with numbers. | 1\) Raise Telemetry above the control tasks to guarantee the publish rate: rejected, because network jitter would then delay motion and line-following (breaks NFR1). 2\) Use MQTT QoS 2 for all telemetry: rejected, because the extra handshakes and buffering are wasted on data that is stale after 200 ms. | Defend: P9 priority, drop-when-full queue, and the rule that link loss never changes vehicle state. Open to: the 20 ms wake / non-blocking poll scheme recorded as delta D2 in ARCHITECTURE.md (not yet approved). Concede: the car cannot be stopped remotely while WiFi is down (accepted risk in Week 6 §8.3). |
| Buddy 2 | Our Motion Control task runs a per-wheel PID every 20 ms at P3, below Safety (P1) and the Vehicle Controller (P2). Wheel encoders on GP6/GP7 use GPIO interrupts, and Safety can brake through motion\_emergency\_brake(). If the requirements change (higher speed, tighter turn accuracy, U-turns, new manoeuvres), what happens to the encoder interrupt load, the PID period, the CPU budget and the 200 ms stop? | 1\) Encoder interrupt rate \= speed × pulses per cm, so it grows with speed; compute it at MOTION\_SPEED\_MAX (50 cm/s) once the encoder resolution is measured. 2\) A faster PID (e.g. 10 ms) halves the pulses counted per sample, which makes the speed estimate coarser; timing the interval between pulses may be better than shortening the period. 3\) The U-turn reuses the encoder-based turn, but wheel slip during a 180° pivot may exceed the ±5° turn tolerance (NFR4). | 1\) Run the PID inside the encoder ISR for lower latency: rejected, because ISRs must stay minimal and the loop rate would then depend on wheel speed. 2\) Add a dedicated brake GPIO: not possible, because the ROBO PICO brakes by driving both motor inputs LOW; there is no separate brake pin. | Defend: 20 ms PID at P3; Motion Control is the single PWM owner, with motion\_emergency\_brake() as the only exception. Keep the reduced bring-up speeds in config/thresholds.h (20 / 10 / 15 cm/s) until encoder counts are validated (A2). Still needed: measured encoder resolution and stopping distance (NFR10). |
| Buddy 3 | Line-Following samples two ADC IR sensors every 5 ms at P4. The barcode IR sensor on GP28 is timestamped in an ISR and decoded at P5 (Code 39, A–D). Which requirement changes (faster car, new barcode characters, different bar widths, new junction types, U-turns) would stress the sampling rate, the 64-edge buffer, CPU time or decoding reliability? | 1\) Narrow-bar time \= bar width ÷ speed; a faster car shortens it, but µs ISR timestamps still resolve it. The real limit is draining the 64-edge buffer before it overflows (A10). 2\) Adding a barcode command costs only a table entry, as long as validation still rejects every other character. 3\) Two line sensors cannot tell a left branch from a right branch at a junction; a new junction type could expose this. | 1\) Read the barcode sensor through a third ADC channel on GP28: rejected, because it conflicts with edge timing and with the board’s GP28 VBAT-sense option. 2\) Sample the line sensors every 1 ms: rejected, because it costs 5× the CPU time while steering is only posted every 20 ms. | Defend: ISR timestamps plus ratio-based narrow/wide classification, which makes decoding independent of speed. Raise at Zoom: the repo skeleton polls every 2 ms while capturing (BC\_POLL\_PERIOD\_MS) instead of being woken by the ISR, a delta from Week 6 that is not yet in ARCHITECTURE.md. Decide: whether a U-turn runs at the next junction (current design) or immediately after decoding. |
| Buddy 4 | IMU Processing runs every 10 ms at P8 and also detects collisions; Safety (P1) brakes on a COLLISION flag or when the 50 ms IMU heartbeat stops. Our config suggests the IMU may be an accelerometer \+ magnetometer with no gyro. How does that affect pitch estimation, turn detection, hump height and collision latency, and which requirement changes would stress this subsystem? | 1\) Without a gyro a complementary filter is impossible; pitch from the accelerometer alone is corrupted by acceleration and vibration, so low-pass it and only trust it while \|a\| ≈ 1 g. 2\) TURNING would have to come from the encoder wheel-speed difference (or the magnetometer) instead of gyro z. 3\) Worst-case collision detection \= 10 ms period \+ preemption by P1–P7; it must be measured, not assumed (A7). | 1\) Estimate hump height by double-integrating vertical acceleration: already rejected in Week 6 (≈1.2 cm drift in 0.5 s from a 0.01 g bias). 2\) Move IMU Processing to P1: rejected, because its I2C reads every 10 ms would then delay motion control. | Before Zoom: confirm the IMU part by I2C bus scan (TODO in config/thresholds.h, IMU\_HAS\_GYRO \= 0). If there is no gyro, propose: pitch from filtered accelerometer data, turn detection from encoders, and correcting the Week 6 “Accel \+ Gyro” wording. Defend: the 50 ms IMU watchdog and the latched collision fault. |
| Buddy 5 | Obstacle Scanning (P6) sweeps a servo on GP12 across 5 bearings (60 ms per step, median of 3 ultrasonic pings), then fine-scans; Obstacle Avoidance (P7) bypasses and hands control back through LINE\_REACQUIRED. If the speed, obstacle size or detection distance requirements change, does a scan still finish before the car reaches the obstacle? | 1\) Worst case per bearing \= 50 ms servo settle \+ 3 pings × 30 ms echo timeout \= 140 ms, so a full coarse sweep can take 700 ms. At 30 cm/s the car moves 21 cm in that time, close to the 25 cm D\_detect. 2\) The forward bearing is revisited only once per sweep; checking it between side bearings would shorten the blind interval. 3\) The bypass clearance depends on CAR\_WIDTH\_MM \= 120 mm, which is an assumption until the car is measured. | 1\) Run a continuous fine scan instead of coarse-then-fine: already rejected in Week 6 as too slow. 2\) Add a second ultrasonic sensor: rejected, because it is not in the hardware kit and would need another timed-pulse ISR. | Defend: coarse-then-fine scanning and the single LINE\_REACQUIRED handoff exit. Propose: re-check the 90° bearing between side bearings if the 700 ms worst case is confirmed. Before Zoom: measure the car width and the ECHO voltage level (A8). |

# **Week 6 Architecture Defence Snapshot Before Zoom**

Select only **5 to 7** important Week 6 architecture claims that affect integration readiness. Do not repeat the entire Week 6 document. Each row should defend one claim using evidence, remaining weakness, and the buddy responsible. One smart-toaster example is shown; replace it with your own project claim.

| Week 6 artefact | What the team claimed in Week 6 | Evidence available now | Still weak or untested | Buddy defending it |
| :---- | :---- | :---- | :---- | :---- |
| RTOS task table / Safety statechart (Week 6 §3, §6.4) | Safety / Collision Response at P1, backed by a 50 ms IMU heartbeat watchdog, stops the car within 200 ms of a collision (NFR10, NFR13). | Week 6 §3 and §3.1 set and justify the P1–P9 priorities; NFR10 sets the 200 ms stop target and NFR13 the 50 ms IMU watchdog; the §6.4 statechart shows the latched collision fault and its reset condition. | No timing measurement yet. Collision detection itself still runs at P8. The IMU part is unconfirmed (IMU\_HAS\_GYRO \= 0), and the spike-to-stop time has not been measured (A7). | Buddy 4 |
| Task interaction diagram (Week 6 §4) | Only Motion Control drives motor PWM; Line-Following, Barcode Decode and Obstacle Avoidance post to MBF\_NAV\_IN, and the Vehicle Controller is the single arbiter. | Week 6 §4 diagram and §3.3 queue table; the §2 pin table lists Motion Control as the only owner of GP8–GP11; NFR17 requires a defined depth and timeout for every queue. | The arbiter is not implemented. Queue depths and the discarding of stale AVOID commands after RELEASE are untested. | Buddy 2 |
| Hardware pin layout (Week 6 §2) | Every peripheral has a conflict-free pin verified against the ROBO PICO datasheet, including the split Grove 7 port (ENC\_R on GP7, IR\_BC on GP28). | Week 6 §2 pin table, cross-checked against the ROBO PICO datasheet (Grove port pin pairs and the motor-driver truth table). | The Grove 7 split cable, the open GP28 VBAT-sense jumper and the ECHO voltage level (A8) have not been checked on the physical car. | Buddy 3 |
| Barcode decoder statechart (Week 6 §6.2) | Code 39 decoding is independent of speed: ISR edge timestamps plus ratio-based narrow/wide classification; only A–D are accepted. | Week 6 §6.2 statechart; §3.2 constants (64-edge buffer, T\_end \= 3 × widest element, D\_cmd \= 30 cm); NFR8 and NFR9. | No barcode has been read yet. The skeleton polls every 2 ms instead of being ISR-woken (undocumented delta). Unknown whether the track barcodes carry a mod-43 check character. | Buddy 3 |
| IMU / Terrain statechart (Week 6 §6.4) | Pitch comes from a complementary filter (accelerometer \+ gyro), TURNING from the gyro rate, and hump height from Σ sin(pitch) × encoder distance. | Week 6 §6.4 and assumption A6. | config/thresholds.h suggests an accelerometer \+ magnetometer part with no gyro. If confirmed, the filter and gyro-based TURNING must be redesigned before Week 10\. | Buddy 4 |
| Obstacle avoidance statechart (Week 6 §6.3) | The coarse scan detects an obstacle in time to stop; the bypass takes ≤ 3.0 s with a 5.0 s worst case, and LINE\_REACQUIRED\_HANDOFF is the only exit. | Week 6 §6.3 statechart; §3.2 constants (D\_detect 25 cm; 3.0 s bypass budget \+ 2.0 s search \= 5.0 s worst case); NFR14. | The worst-case coarse sweep (≈700 ms) lets the car travel ≈21 cm against a 25 cm detection distance. CAR\_WIDTH\_MM \= 120 is an assumption. Nothing has been tested. | Buddy 5 |
| WiFi / MQTT statechart (Week 6 §6.5) | Network problems never block a control task: all lwIP calls stay in the P9 Telemetry task and producers post without waiting (NFR1). | Week 6 §3.3 and §6.5; ARCHITECTURE.md delta D2 (non-blocking 20 ms telemetry wake); build rule R3 limits stdio to the debug drain task. | micro T-Kernel \+ CYW43/lwIP coexistence is unproven (A9). D2 is not yet approved. No WiFi-drop test has been run. | Buddy 1 |

## 

## **Shared Resource Allocation Matrix**

Enter at least **3 shared resources**, or more if the project has additional shared buses, timers, interrupts, actuators, sensors, queues, event groups, or shared data. A resource is “shared” if more than one task, module, interrupt, or buddy can affect it. 

| Shared resource | Used by task(s) | Owner task / module | Access type | Contention risk | Mitigation | Evidence needed |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| Motor PWM outputs GP8–GP11 (PWM slices 4 and 5\) | Motion Control; Safety (emergency brake only) | Motion Control (src/motion) | Write | Safety’s brake could be overwritten by the next PID write in the same 20 ms tick. | Safety only calls motion\_emergency\_brake(), which runs in a critical section; Motion Control checks FAULT\_LATCHED before every PWM write. | Logic-analyser trace of a brake issued while the PID is running: no PWM edge after the latch. |
| GPIO interrupt (single IO\_BANK0 IRQ for GP5 echo, GP6/GP7 encoders, GP28 barcode) | Encoder ISR, barcode edge ISR, ultrasonic echo ISR | Shared ISR dispatch (src/common) | Interrupt | At high speed, encoder edges delay the barcode and echo timestamps, distorting bar widths and ranges. | Keep each handler to a timestamp or counter update; take the timestamp first; no kernel calls except a wake-up. | Measure ISR duration and barcode timing jitter at maximum speed with all three sources active. |
| motion\_state\_t (guarded by MTX\_MOTION) | Motion Control (writes); Barcode Decode, Telemetry (read) | Motion Control | Read / write | Priority inversion: Telemetry (P9) holding the mutex could delay Motion Control (P3). | TA\_INHERIT mutex; readers copy the struct and release at once; Safety never takes it. | Log the maximum mutex hold time and confirm Motion Control never misses its 20 ms period. |
| MBF\_NAV\_IN message buffer (8 × nav\_input\_t) | Line-Following, Barcode Decode, Obstacle Avoidance, Telemetry → Vehicle Controller | Vehicle Controller | Queue (many writers, one reader) | Overflow could drop a barcode command, or delay a remote STOP behind steering messages. | Steering is sent with TMO\_POL (droppable); other senders wait up to 5 ms (NAV\_MBF\_SEND\_TMO\_MS) and log on failure. | Peak occupancy and drop counters logged during a full-load run. |
| EVF\_SAFETY eventflag (COLLISION, IMU\_ALIVE, FAULT\_LATCHED) | IMU Processing, Safety, Vehicle Controller | Safety | Set / clear flags | Wrong task clears the latch; IMU preempted for over 50 ms causes a false watchdog stop. | Single setter and single clearer per flag (Safety sets FAULT\_LATCHED, only the Vehicle Controller clears it); measure IMU jitter under load. | Fault injection: suspend IMU Processing and confirm the car stops within 50 ms; confirm no false trips in a 10-minute run. |
| Grove 7 connector (GP7 \= ENC\_R, GP28 \= IR\_BC) | Motion Control (Buddy 2), Barcode Decode (Buddy 3\) | Hardware wiring (shared by two buddies) | Physical connector | Wiring mix-up between the encoder and barcode sensor; soldering the GP28 VBAT jumper would corrupt barcode reads. | Labelled split cable; VBAT-sense jumper left open; pin check in config/pins.h. | Photo of the wiring plus a pin test: each sensor toggles only its own GPIO. |

## **Subsystem Interface and Code Structure Map**

Enter at least **one interface or module per buddy**. Each row should identify who provides the module, who consumes it, what inputs and outputs it exposes, and how it will be validated. 

| Module / file / interface | Provider | Consumer(s) | Purpose | Inputs | Outputs | Interface rule | Validation rule |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| src/telemetry (tel\_ API) | Buddy 1 | All tasks (event producers); operator dashboard | Publish telemetry and parse remote commands | tel\_evt\_t via MBF\_TELEMETRY; MQTT messages on car/cmd/\* | car/telemetry/\* topics; nav\_input\_t (src \= REMOTE) to MBF\_NAV\_IN | Producers post with TMO\_POL only and never call lwIP; only the Telemetry task touches CYW43/lwIP. | Force a WiFi drop mid-run: control-loop timing unchanged, link back within 30 s (A3). |
| src/motion (motion\_ API) | Buddy 2 | Vehicle Controller (commands); Safety (brake); Barcode Decode and Telemetry (state) | Closed-loop wheel control and motion state | motion\_cmd\_t from MBF\_MOTION\_CMD; encoder counts | PWM on GP8–GP11; motion\_state\_t (speed, distance, counts) | Only Motion Control writes PWM; motion\_emergency\_brake() is the only exception; state read by copy under MTX\_MOTION. | 1 m runs within ±3% (NFR4); brake during active PID stops the output. |
| src/vc (Vehicle Controller) | Buddy 2 | Motion Control | Arbitrate navigation inputs into one command | nav\_input\_t {src, payload} from MBF\_NAV\_IN; FAULT\_LATCHED | One motion\_cmd\_t per decision; status events | Fixed precedence Safety › Avoid › Barcode › Line; AVOID messages discarded after RELEASE. | Inject conflicting LINE and AVOID inputs: log shows only the authority holder’s command executed. |
| src/line (lf\_ API) | Buddy 3 | Vehicle Controller; Obstacle Avoidance | Line tracking, junction and line-loss detection | IR\_L / IR\_C on ADC0 / ADC1 | Steering error every 20 ms; junction and TRACK\_LOST events; LINE\_FOUND flag | Never calls motion functions directly; posts steering with TMO\_POL. | Serial log of line error against a measured physical offset; junction detected on the test track. |
| src/barcode (bc\_ API) | Buddy 3 | Vehicle Controller | Decode Code 39 navigation barcodes | GP28 edge timestamps; vehicle velocity from motion\_state\_t | nav\_input\_t (src \= BARCODE, A–D) | Only validated A–D characters are posted; invalid reads are logged, never posted. | 20 passes per barcode at two speeds: correct command every time. |
| src/imu and src/safety (imu\_, safety\_ API) | Buddy 4 | Safety; Telemetry; Vehicle Controller | Motion and terrain monitoring; collision stop | Accelerometer (+ magnetometer) on I2C1, GP2 / GP3 | COLLISION and IMU\_ALIVE flags; hump events; FAULT\_LATCHED | IMU refreshes IMU\_ALIVE every sample; only Safety sets FAULT\_LATCHED and only the Vehicle Controller clears it. | Staged bump stops the car within 200 ms; suspending IMU Processing stops it within 50 ms. |
| src/obstacle (obs\_ API) | Buddy 5 | Vehicle Controller; Line-Following (handoff) | Scan, profile and bypass obstacles | Echo pulse width (GP5); servo angle (GP12); LINE\_FOUND flag | obstacle\_profile\_t; bypass commands and RELEASE; AVOID\_ACTIVE / LINE\_REACQUIRED | Avoidance reads LINE\_FOUND and never reads the IR ADC directly; the only exit is RELEASE \+ LINE\_REACQUIRED. | Known obstacle at 25 cm: width error ≤ 10%; full bypass ≤ 3.0 s. |

## **Week 6 Assumption Closure**

Enter at least **one assumption per buddy**, preferably copied from Week 6\. Each row should show whether the assumption is still open, validated, deferred, or converted into a risk. 

| Week 6 assumption ID | Original assumption | Current status | Evidence collected | Remaining risk | Next action |
| :---- | :---- | :---- | :---- | :---- | :---- |
| A1 (Buddy 3\) | Track IR sensors give stable readings after startup calibration across expected lighting. | Open | None yet; pins and ADC channels fixed in config/pins.h. | Erratic line-following or false junctions under different lighting. | Log IR ADC values under bright, dim and mixed light; pass if the black/white gap is at least 3× the noise band. |
| A2 (Buddy 2\) | Wheel encoder counts reflect rotation with no missed or double counts at maximum speed. | Open | None yet; bring-up speeds capped at 20 cm/s in config/thresholds.h until validated. | PID drift and wrong distance-based moves, including U-turn angle. | Ten 1 m runs at maximum speed; pass if error ≤ ±3% (NFR4). Measure encoder resolution first. |
| A3 (Buddy 1\) | The MQTT link recovers after a WiFi drop without blocking the control loop. | Open | Design only; non-blocking telemetry wake proposed as delta D2 in ARCHITECTURE.md. | Car freezes waiting on network I/O, or telemetry is lost. | Force a WiFi drop mid-run; pass if Motion Control jitter stays below 1 ms and the link recovers within 30 s. |
| A4 (Buddy 4\) | Hump and collision thresholds can be told apart from normal vibration and turning. | Open | Initial values only (5°, 2.5 g) in config/thresholds.h. | False collision stops, or missed humps. | Log 20 runs each on flat track, humps, turns and staged bumps; tune until there are zero false collisions. |
| A5 (Buddy 5\) | Ultrasonic ranging is reliable at the servo’s angular steps and free of reflection echoes. | Open | None yet; scan constants in config/timing.h. | Wrong obstacle width or side, leading to a bad bypass. | Scan known obstacles at 15–40 cm and ±60°; pass if width error ≤ 10% and detection ≥ 95%. |
| A7 (Buddy 4\) | Collision detection at P8 plus the 50 ms watchdog keeps the full stop within 200 ms under load. | Needs validation | None yet; this is the main open safety question. | Stop slower than NFR10; impact damage. | GPIO toggles at the IMU spike and at the brake call, measured with a logic analyser under full load. |
| A9 (Buddy 1\) | micro T-Kernel runs alongside the Pico SDK CYW43/lwIP driver when every lwIP call is made from the Telemetry task. | Needs validation | Kernel integrated as a static library behind a port layer (ARCHITECTURE.md); WiFi not yet brought up. | Network-stack crashes or lockups; telemetry unusable. | 30-minute soak test with 200 ms telemetry and forced reconnects; pass with no hard faults or stalls. |
| New (Buddy 4\) | The IMU provides a gyro (assumed by the Week 6 complementary filter and gyro-based TURNING). | Needs validation (likely converted into a risk) | config/thresholds.h sets IMU\_HAS\_GYRO \= 0: the 0x19 / 0x1E I2C addresses suggest an accelerometer \+ magnetometer part. | Pitch estimate and turn detection in §6.4 do not work as designed. | Run an I2C bus scan and check the part number; if no gyro, redesign pitch and turn detection before Week 10\. |

# **Version B: Live Zoom-Updated Version**

| Instructions • During the recorded Zoom session, update this section live or immediately from discussion notes. • Record what changed from Version A and why. A change without justification is weak evidence. • Every buddy must contribute at least one challenge, defence, trade-off, or correction. • Any use of AI during the Zoom session must be screen-shared and visible to all participants at the point it is being discussed. • If AI-generated content is introduced during the live discussion, the team must explain the prompt used, the output received, and why the output is being considered. • AI suggestions must be challenged, validated, refined, or rejected through human engineering discussion; “AI said so” is not sufficient justification. |
| :---- |

**Acceptable evidence examples:** GitHub commit link, short test log, video timestamp, oscilloscope or serial monitor screenshot, pin test result, task timing measurement, AI prompt and critique, meeting decision note, or photo of hardware wiring.

**AI-use declaration:** Do not ask AI to complete this template on behalf of the team. Use AI as a reviewer, challenger, or brainstorming partner. The team confirms that all final entries were reviewed, corrected, and accepted by the team, and that every member can explain the decisions related to their subsystem.

## **Buddy Contributions During Zoom**

Each buddy must complete **one row**. The contribution must be a real technical challenge, defence, trade-off, or correction raised during Zoom, not only an attendance statement.

| Buddy | Key challenge, concern, or argument raised | Challenge received | Decision or compromise reached |
| :---- | :---- | :---- | :---- |
| Buddy 1 | Proposed replacing JSON telemetry and commands with a compact binary payload: version byte first, fixed little-endian layout packed with masks and shifts, decoded on the dashboard. | Buddy 3: binary messages are unreadable, so how do we debug them, and what happens when the format changes? | Version byte lets the dashboard reject or adapt to old formats; the dashboard decoder prints every field; local debug logs stay as text over the UART debug drain, outside MQTT. |
| Buddy 2 | Argued encoder counters should stay in their own aligned 32-bit word, while motion\_cmd\_t and motion\_state\_t messages are packed (type 3 bits, speed 6 bits, angle 8 bits). | Buddy 4: does that break the minimum-bit rule, and is a 32-bit write really atomic on the M0+? | An aligned 32-bit load or store is a single instruction on the M0+, so the ISR (the only writer) never needs a read-modify-write; the width is justified by the longest run in the data representation table. |
| Buddy 3 | Proposed storing 20-bit gaps between barcode edges instead of 64-bit absolute timestamps, cutting the 64-entry ring from 512 B to 160 B packed. | Buddy 2: what is the largest gap we could see at our slowest speed, and do 20 bits cover it? | The 1 s capture timeout bounds any gap at 1,000,000 µs, which is below 2^20; a slower read already ends in INVALID\_READ\_RESET, so 20 bits are enough. |
| Buddy 4 | Proposed moving all fault flags (stall, encoder, IMU, no echo, WiFi loss, collision) into one bitmask held in a eventflag. | Buddy 1: if two tasks set different fault bits at the same moment, can one overwrite the other? | No: tk\_set\_flg and tk\_clr\_flg are atomic kernel calls and can be used from interrupt handlers; each bit has one owner that sets it, and only the Vehicle Controller clears the collision latch. |
| Buddy 5 | Proposed packing obstacle\_profile\_t: distance 2–400 cm at 1 cm (9 bits), bearing index 0–12 (4 bits), width 0–400 mm at 5 mm (7 bits), bypass state 4 bits. | Buddy 2: is 1 cm resolution good enough for the bypass clearance check, given ultrasonic noise? | Kept 1 cm: the sensor noise is about that size, so finer bits would store noise; clearance is computed in millimetres before packing, so the bypass decision is unaffected. |

## **Live Decision Change Log**

Enter at least **3 changes AND 3 confirmed no-change decisions** from the Zoom discussion. Include the change type and evidence link or timestamp whenever possible. One row may state “No change” if the team defended the Week 6 design successfully.

| Original idea / Week 6 design | Updated decision | Change type | Reason for change | Evidence link / timestamp | Owner / follow-up |
| :---- | :---- | :---- | :---- | :---- | :---- |
| MQTT telemetry payloads as JSON | compact binary fields  | Major redesign | requirements needs us to use compact binary |  | Buddy 1 |
| separate fault conditions and booleans | individual bits within a shared fault bitmask | Simplified | tk\_set\_flg / tk\_clr\_flg are atomic so no read/modify/write race between tasks and  isr |  | buddy 4 |
| barcode edge ring stores 64-bit absolute timestamps | 20 bit gaps between edges | Clarified | gap between edge is used for decoding, this cuts the ring from 512 bytes to 160 bytes |  | buddy 3 |
| RTOS task table priorities kept | Kept as designed | no change | new requirements only affect data representation |  | all |
| pin layout and hardware | kept  | no change | requirements only changed software related stuff |  | all |
| queue depths and timeout | kept | no change | same number of messages, only bytes per message shrink |  | all |
| message buffers like motion\_cmd\_t, nav\_input\_t, obstacle\_profile\_t | make it to a minimum of how many bits used with masks and shifts | major redesign | requirements covers inter task messages so our buffers needed change |  | buddy 2, buddy 5, buddy 3 |

## **Resource and Assumption Review**

| Item challenged | Evidence presented | Decision made | Impact on other buddies |
| :---- | :---- | :---- | :---- |
| RAM saved from bit minimisation | task stacks are small so requirements change doesnt really affect | accept change and report the measurements later | all buddies, changes to be measured |
| cpu costs by masks and shifts | m0+ has no bitfield extract instruction so packed access adds shifts and AND instructions | negligible at 5 to 20 ms periods for our ISRs, need to measure duration to make sure its within RTOS limits | buddy 2, 3, 5 |
| C bit fields | bit field layout is left of the compiler | bit fields for private states, masks and shifts the queue messages for MQT paylods | buddy 1’s wire format and queue |

## **Risk Assessment and Subsystem Test Plan**

### **Integration Risk Register**

Enter at least **3 integration risks**. Include at least one timing or scheduling risk, one hardware or wiring risk, and one interface or team-integration risk. 

| Risk ID | Risk | Cause | Impact | Likelihood | Severity | Mitigation | Owner | Status |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| R1 | Extra mask and shift inside ISRs adds timestamp jitters | extra instructions in the ISRs of the encoder, barcode, echo handlers | high speeds might make it miss edges if barcode is distorted | low | medium | measure ISR | buddy 3 | open |
| R2 | packed fields are too narrow for real sensor values | resolution chosen from nominal range only | lost in accuracy in line position, pitch, hump height, distance | medium | medium | size each field from measured sensor noise | buddy 5 | open |
| R3 | dashboard disagrees on layout | inconsistent changes | data gets corrupted due to inconsistency | medium | medium | one header shared | buddy 1 | open |

### **Subsystem Test Plan**

Enter **at least 4 cross-subsystem tests**. Each test must involve the work of **at least two buddies** and must include a success criterion and evidence type, such as a video, serial log, screenshot, timing result, or commit link. At least **TWO** test should involve timing, safety, or shared-resource concern. 

| Subsystem | Test case | Success criteria | Evidence required | Owner | Status |
| :---- | :---- | :---- | :---- | :---- | :---- |
| IMU \+ safety \+ motion | an isr and two tasks set. it clears fault bits continuously | no fault bit is lost for 10 mins | serial log of set/clear counts compared with expected counts | buddy 4/ buddy 2 | open |
| all subsystems | build the firmware according to our week 6 first to get a baseline | RAM and message sizes measured and compared | RAM figures from map file for both builds | all buddies | open |
| Barcode and motion timing | read barcode 20 times at maximum speed | 20/20 correct, barcode ISR not worse than baseline | log of decoded characters and ISR timing measurement | buddy 3/2 | open |
| telemetry \+ all subsystems | send every telemetry and command message to the dashboard decoder | every field decodes to the value sent, message sizes recorded before and after | decoder output screenshot, message-size comparison table | all/buddy1 | open |

## **Constraint Injection / Curveball Response**

| Instructions • Record the injected constraint exactly as given. • Explain how it affects all five buddies, even if the impact differs by subsystem. • A good response shows prioritisation, trade-off reasoning, fallback behaviour, and evidence-aware decision making. |
| :---- |

| Injected constraint | Buddy affected | Immediate impact | Adaptation / compromise |
| :---- | :---- | :---- | :---- |
| minimum bit data representation, firmware must minimise number of bits used to represent application data such as motion control and encoder data. | Buddy 1 | MQTT’s initial telemetry changed from JSON to compact binary fields  | Fixed little-endian layout packed with masks and shifts, version byte first, decoder on the dashboard side. |
| same as above | Buddy 2 | message buffer motion\_cmd\_t and encoder data must be compact. | Command type 3 bits, speed 0–50 cm/s 6 bits, angle 0–180 8 bits; encoder counter kept in its own aligned word. |
| same as above | Buddy 3 | barcode command can fit in the compacted message and barcode edge ring | Store 20-bit gaps between edges; LF and barcode states 3 bits each. |
| same as above | Buddy 4 | fault flags and hump values should be packed. tasks set fault bits | All faults as bits in an eventflag; hump 0–100 mm at 1 mm \= 7 bits; pitch 45 at 0.5 \= 8 bits. |
| same as above | Buddy 5 | obstacle\_profile\_t buffer should be compact | Distance 2–400 cm at 1 cm \= 9 bits; bearing index 0–12 \= 4 bits; width 0–400 mm at 5 mm \= 7 bits; bypass state 4 bits. |

## **Integration Readiness Self-Assessment**

Complete all rows. Use **Green** only if there is evidence, **Yellow** if partly ready or untested, and **Red** if the area is unclear or blocking Week 10 integration.

| Readiness area | Status | Evidence | Concern |
| :---- | :---- | :---- | :---- |
| Hardware wiring known | Green | car is built | requirements unchanged |
| RTOS task ownership clear | Green | rtos table from week 6 document | none |
| Shared resources identified | amber | shared resource matrix | new compact shared words add read/modify/write problems not tested |
| Critical statecharts implementable | amber | week 6 document statecharts | state encoding must be fixed to new minimum bits |
| Safety or fault handling defined | green | week 6 NFR’s covered safety such as NFR10 and NFR13 | defined but untested |
| Week 10 partial integration feasible | amber | week 6 design | no codes yet and baseline have not been measured |

**Updated readiness position before the Week 10 partial integration demo:**

application data changed from requirements not task structures or timings

1. measure RAM and message size baseline before code is changed  
2. shared header defines every field width and mask

# 

# **Version C: Final AI-Vetted Gap Check**

| Instructions • After the live discussion, give the updated Version B content to AI and ask AI to act as a strict embedded systems reviewer. • Ask AI to identify missing assumptions, weak evidence, unclear interfaces, untested failure modes, and resource conflicts. • For every AI-identified gap, decide whether to fix, reject, or defer it, and explain why. |
| :---- |

| AI reviewer prompt used |  |
| :---- | :---- |
| Act like a strict embedded system reviewer and help me verify the report’s version b, and identify the gaps for it.  |  |

| AI-identified gap | Team response | Action taken | Evidence / reason |
| :---- | :---- | :---- | :---- |
| No concurrency risk in the risk register, although the requirement explicitly says read-modify-write on shared packed data must be race-free. | Fix | Added risk R4: lost updates on packed words shared with ISRs or tasks. Mitigation: interrupts off around ISR-shared updates, mutex for task-shared data, eventflag for faults. | Concurrency is the one hazard the requirement names directly; the M0+ has no LDREX/STREX, so a plain read-modify-write is not atomic. |
| The test plan has no safety-timing test: nothing shows the collision stop is still within 200 ms after packing. | Fix | Added test: staged bump after packing; motors braked within 200 ms (NFR10); logic-analyser trace of spike and brake GPIOs. Owners Buddy 4 / Buddy 2\. | Week 13 requires evidence of no unacceptable timing degradation, and the safety stop is the most critical timing in the system. |
| The encoder-counter exception (own aligned 32-bit word, not packed) appears in Buddy Contributions and Constraint Injection but not in the change log, so the decision is not traceable. | Fix | Recorded here as a Clarified decision; it will get its own row in the Week 13 data representation table with the longest-run justification. | An unrecorded exception looks like non-compliance with “minimum bits” at Week 13\. |
| Readiness says state encodings must change to minimum bits, but no change-log row records the chosen widths. | Fix | Recorded here: system, line-following and barcode FSMs 3 bits; obstacle FSM 4 bits (up to 9 states). | Makes the readiness concern traceable to a decision with an owner. |
| Out-of-range and signed values are undefined: nothing says whether packed fields clamp or wrap, or how signed fields (pitch, line error) are sign-extended when unpacked. | Fix | Rule added: clamp before packing and set an out-of-range flag; sign-extend on unpack. Host unit tests at minimum, maximum, ±limits and just outside range. | A wrapped steering error flips direction; a wrongly unpacked negative pitch corrupts hump reports. |
| Eventflag clear semantics are not defined: a wait that clears all bits on release could wipe other buddies’ fault bits. | Fix | Safety waits with TWF\_BITCLR (or no clear); each fault bit has one setter; only the Vehicle Controller clears the collision latch. | Prevents one task silently clearing faults it does not own. |
| Binary MQTT commands are not validated: a malformed or wrong-version command could be acted on. | Fix | Commands are rejected unless the version byte and length match; a rejected command is logged and never reaches the Vehicle Controller. | With text JSON a bad command failed parsing; a binary byte pattern can look valid by accident. |
| Hardware wiring is rated Green with “car is built” as the only evidence. | Partially fixed | Kept Green, but will attach a wiring photo and a pin test (each sensor toggles only its own GPIO) as evidence. | The template allows Green only with evidence; a built car is evidence of assembly, not of correct wiring. |
| The “before” baseline is not defined: there is no subsystem code yet, so it is unclear what build the RAM comparison starts from. | Deferred | Baseline defined as the first working Week 6-design build before any packing; measured and committed before conversion starts. | Cannot be measured until that build exists; deferred to the start of implementation, with Buddy 2 as owner. |
| Encoder counters are left unpacked, which may not meet “minimum bits”; consider packing them, and using C11 atomics for packed shared data. | Rejected | No change. | Packing the counters would force a read-modify-write inside the ISR; on the M0+, C11 atomics are implemented by disabling interrupts anyway, and the eventflag already covers fault bits. |
| Whether Barr C permits C bit-fields for private state is unconfirmed. | Needs validation | Buddy 1 to check the Barr C rules before any bit-field is written; fall back to masks and shifts everywhere if not allowed. | The design uses bit-fields for private state, so compliance with NFR16 depends on it. |

# **5\. Final Tribunal Summary**

| Final question | Team answer |
| :---- | :---- |
| What is the strongest part of the current design? | Clear single ownership: every task has one owner and one priority, the Vehicle Controller is the only arbiter of motion, and fault bits sit in an atomic kernel eventflag. The minimum-bit change therefore touches data representation only, not task structure or timing. |
| What is most likely to fail? | Mismatched packed layouts between producers, consumers and the MQTT dashboard, and read-modify-write races on packed data shared with ISRs. Both fail silently, so they need the unit tests and stress test planned in Version C. |
| What evidence supports readiness? | The Week 6 design (task table, statecharts, NFR10/NFR13), the built car, and bit-width calculations checked by hand (2 bits for A–D; 2^20 \> 1,000,000 µs). No measured timing or RAM results exist yet, so readiness is mainly design evidence. |
| What did AI miss that human discussion revealed? | That the car is already built, so hardware wiring could be rated Green, and that the RAM baseline should come from a build of our Week 6 design before any packing. The AI had no way to know the physical state of the car. |
| What did AI correctly identify after discussion? | The missing concurrency risk and safety-timing test, the unrecorded encoder-counter exception, and the undefined clamping and eventflag clear rules. |

# 

