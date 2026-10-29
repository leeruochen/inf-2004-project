# Original task brief: template merge + Week 8 Step 2

This is the brief the paused workflow was given (the owner's approved decisions, as sent). `merge-week8-plan.md` refers to it as "the brief", and its line 187 says the implementer "gets the full Step 3 text from the brief": **this file is that text.** Precedence when anything conflicts: the OWNER CORRECTIONS at the top of `week8-impact-report.md` first, then this brief, then the plan.

Already done (do not redo): STEP 0 (unblock check_rules) and STEP 1 (same-name gate: README.md and .gitignore diffs approved and applied). Remaining: STEP 2, verify merge, STEP 3, final verification.

---

TASK: Execute (1) the approved TEMPLATE MERGE (Option C, Step 4), (2) verify it, (3) the approved WEEK 8 CHANGE REQUEST Step 2 (pins, console, docs only), (4) verify again. Work directly in the repo (NO worktree). NEVER git commit, never push. Use git mv for the five rule-fixture moves. Never delete or move ProjectTemplate/ or anything in it. Never edit 'INF 2004 Embedded Systems.md' (the team's assessed submission, read only), design.md or specs.md. Never edit anything under external/mtk3_bsp. Do NOT start Batch B (no ipc module, no port layer, no dbg ring/drain implementation, no packed types, no encoder code, no main.c).

NOTICE.md has an owner edit: check `git diff NOTICE.md` / `git log -p NOTICE.md` and PRESERVE it; append only.

READ FIRST: .agents/tasks/template-merge-report.md (sections 2-3 = the approved Option C plan), .agents/tasks/week8-impact-report.md (sections A-F), .agents/tasks/batchA-summary.md (build preamble and verification standard), .agents/tasks/mtk3-rp2040-integration-report.md, ARCHITECTURE.md, CMakeLists.txt, cmake/*.cmake, config/*.h, config/mtk3/*.h, docs/*.md, tests/rules_fixtures/**, design.md, 'INF 2004 Embedded Systems.md' (Team Information table), ProjectTemplate/TEAM.md, ProjectTemplate/app_program/**, ProjectTemplate/testing/**, ProjectTemplate/videos/README.md.

NAMES/IDS: only from the Team Information table in 'INF 2004 Embedded Systems.md', copied exactly. Never from design.md. Never invent. GitHub usernames are TODO(owner-to-confirm).

NON-NEGOTIABLES:
- The SDK owns startup, linker and vector table. The kernel is the static lib mtk3, behind a port.
- A .c file includes kernel headers (<tk/...>, <tm/...>, <sys/...>) OR SDK headers (pico/, hardware/), never both.
- One owner per peripheral.
- No blocking SDK calls in tasks.
- Only the debug drain task calls stdio.
- ISRs go through the SDK IRQ API.
- -Wall -Wextra -Werror on our code only.
- Barr C naming: module prefixes, _t types, stdint types, braces on every block, no magic numbers, Barr-style header guards.
- No C bit-fields anywhere.
- Keep every TODO(owner-to-confirm) marker and open item.
- Stubs carry /* TODO(phase N, buddy X): ... */.

=== STEP 2: TEMPLATE MERGE (Option C, per template-merge-report section 3.2-3.3) ===
- Team code goes in app_program/subsystem1..5:
  - 1 = telemetry (Buddy 1)
  - 2 = motion + Vehicle Controller (Buddy 2; design.md 3.1 confirms VC = Buddy 2)
  - 3 = line + barcode (Buddy 3)
  - 4 = IMU + safety (Buddy 4). task_safety.c lives in subsystem4 and calls motion_emergency_brake() ONLY via subsystem2/subsystem2.h.
  - 5 = obstacle scan + avoid (Buddy 5)
- Platform code stays in src/ (src/rtos/port, src/debug, src/common, src/app/main.c).
  - Keep src/app/main.c as the planned SDK main() that starts the kernel. It is NOT created now; it comes in Batch B.
  - Add app_program/app_main.c as the kernel-side usermain.
  - Explain both roles in file headers and ARCHITECTURE.md.
- Imported files: copy each, then rewrite it as a STUB (Barr C naming, template file-header block):
  - **app_program/app.h:** the two-phase contract and the subsystemN_init/start/test prototypes. Kernel side only: no <bsp/libbsp.h>, no TASK_STACK_SIZE (stacks come from config/priorities.h), no prc/TP_PRC args.
  - **app_program/app_main.c (usermain), in this order:**
    1. create all IPC objects (placeholder TODO(phase 1 batch B, buddy 2): ipc_init(); the module is not created yet);
    2. every subsystemN_init();
    3. every subsystemN_start();
    4. only if APP_TEST, start the test task;
    5. tk_slp_tsk(TMO_FEVR).
  - **app_program/test_report.h:** TEST_INIT/TEST_CHECK/TEST_SUMMARY, same `[TEST] ID PASS|FAIL` format, output through the dbg_ ring API. That API is Batch B, so reference it with a TODO. The APP_TEST=ON compile is deferred to Batch B.
  - **app_program/subsystem1..5/{subsystemN.c,subsystemN.h,subsystemN_test.c}:** stubs.
- Do NOT port the demo tasks (task_led, task_producer, task_consumer, task_monitor). They stay in ProjectTemplate/ as reference.
- Write docs/TEMPLATE_NOTES.md:
  - patterns worth keeping: two-phase init, the cyclic-handler wake pattern, IF-NN publishing in subsystemN.h, the test ID scheme, the TEAM.md budget tables;
  - why each demo is not ported: tm_printf from tasks, libbsp, TP_PRC*, SMP;
  - Phase 2 WiFi: the template's CYW43/lwIP/MQTT is reference only.
- Includes inside a subsystem:
  - drv_*.c include SDK headers only, plus their own kernel-free drv_*.h (stdint only).
  - task_*.c, subsystemN.c and subsystemN_test.c include kernel headers only.
  - subsystemN.h is never included by a drv_*.c.
- APP_TEST:
  - Add CMake option(APP_TEST OFF).
  - Tests run in a dedicated LOWEST-priority test task: kernel priority 11, below the debug drain at 10. It is created only when APP_TEST=ON, and its output goes via dbg_.
  - APP_TEST=ON with DBG_ENABLE=OFF -> FATAL_ERROR at configure.
  - Raise the task-count assert by one (CNF_MAX_TSKID >= 12).
  - Add TASK_PRI_TEST 11 and its stack to config/priorities.h and to the pool-fit assert.
- CMake:
  - Module registration covers src/common, src/rtos, src/debug, src/app, app_program and app_program/subsystem1..5.
  - Extend the unregistered sources.cmake guard to app_program/** (never ProjectTemplate).
  - The robocar firmware link stays SKIPPED until Batch B.
  - Add compile-only OBJECT libraries in ALL with 0 warnings: kernel-side stubs with mtk3_headers, and SDK-side drv_ stubs separately.
- Rule fixtures:
  - git mv the five files (src/motion/r1_mixed.c, src/motion/r2_blocking.c, src/line/r3_stdio.c, src/vc/r4_libbsp.c, src/safety/r5_core1.c -> app_program/subsystem{2,2,3,2,4}/...).
  - Update the check_rules_selftest expectations.
  - Add pio_sm_get_blocking and pio_sm_put_blocking to R2, with a fixture.
- TEAM.md at the root, ported from ProjectTemplate/TEAM.md, with OUR data:
  - pins from config/pins.h (the new Week 8 map);
  - peripherals, one owner each (PIO0 = Motion Control encoders; PIO1 reserved for CYW43 in Phase 2);
  - OUR priorities 1-10 (11 with APP_TEST);
  - IF rows from design.md 3.3 (MBF_NAV_IN, MBF_MOTION_CMD, MBF_OBSTACLE, MBF_TELEMETRY, SEM_ECHO, EVF_SAFETY, EVF_NAV, MTX_MOTION, edge ring, encoder words);
  - the object budget from config/mtk3/config.h;
  - remove the cyw43_utk_task rows (leave a TODO for Phase 2).
- testing/:
  - README.md, plus TEST_LOG.md and logs/.gitkeep for subsystem1..5, integration and system;
  - our CMake -DAPP_TEST=ON command;
  - USB CDC capture instructions on Windows;
  - example rows cleared.
- videos/README.md: copy verbatim, except the example row.
- Docs:
  - ARCHITECTURE.md: two source roots, the usermain location, and the template deviations delta.
  - NOTICE.md: append the credit "INF2004 course project template provided by the module instructor", author TODO(owner-to-confirm). The T-License credit is unchanged.
  - docs/DEBUG.md.
  - cmake/mtk3.cmake: fix the comment so it says app_program there means the BSP's own sample.

=== VERIFY MERGE (Batch A standard) before Step 3 ===

=== STEP 3: WEEK 8 CHANGE REQUEST: pins, console, docs ONLY ===

**config/pins.h.** Barr C style, compile-time collision checks, CYW43 never-use pins GP23/24/25/29.

Encoders:
- ENC_R_A_PIN 0, ENC_R_B_PIN 1 (Grove 1); ENC_L_A_PIN 16, ENC_L_B_PIN 17 (Grove 4). QUADRATURE.
- "A on the lower pin" is PROVISIONAL. The wiring table's black/white labels are not authoritative; the harness colours are green/red.
- ENC_R_DIR_INVERT and ENC_L_DIR_INVERT: 0U, TUNE, set by a hand-spin plus forward-drive test.
- MOTOR_L_DIR_INVERT and MOTOR_R_DIR_INVERT go next to them.
- Bring-up order comment: motor direction first, then encoder sign.
- Compile-time checks: each pair is adjacent with A = B-1 (the single PIO program reads base and base+1), and ENC_*_A is even.
- 6-pin motor/encoder connector comment block:
  - pin 1 GND;
  - pins 2/3 encoder channels;
  - pin 4 3.3 V;
  - pins 5/6 motor power to the screw terminals.
  - Phase A/B is not specified.
  - The encoder is powered from 3V3 only, never VIN or the servo header.

Ultrasonic and IMU:
- US_TRIG 4, US_ECHO 5 (Grove 3).
- IMU SDA 2, SCL 3 (Grove 2, I2C1). Accel 0x19 and mag 0x1E: VERIFY by bus scan. INT1/INT2/SAO are not connected.

Line sensors:
- IR_C_PIN 28 = ADC2 (Grove 7, CENTER); IR_R_PIN 26 = ADC0 (Grove 5, the car's RIGHT).
- Explicit IR_C_ADC_CH 2U and IR_R_ADC_CH 0U, plus a (pin-26)==ch check.
- This is the mirror of design.md's left+centre. Buddy 3 must REDEFINE the steering sign: TODO(owner-to-confirm, buddy 3). Proposal: steer_err > 0 = line to the car's right.
- Only VCC/GND/AO are wired on the line sensors; DO is not wired.

GP26:
- Declared ONCE as an ADC input, pulls disabled, never an output.
- Grove 6's GP26 lead is the SAME pin as Grove 5's GP26 and is not taped off. Firmware cannot disconnect it; do NOT claim otherwise.
- Hardware checklist item: tape or cap the Grove 6 GP26 lead.

Barcode, GP28, servo, motors:
- IR_BC_PIN 27 (Grove 6), digital GPIO interrupt. The barcode board has VCC/GND/DO only. Never call adc_gpio_init() on GP27.
- The GP28 VBAT-sense jumper must stay OPEN.
- SERVO_PIN 15, CONFIRMED (PWM slice 7 channel B). Never use GP12 for the servo.
- Motors GP8-11 on slices 4/5 are unchanged. Static check: servo slice != motor slices.

Spare, reserved and removed pins:
- Spare: GP6, GP7, GP12, GP13, GP14 (update PINS_SPARE_MASK).
- Reserved: GP18 NeoPixel, GP20/21 buttons, GP22 buzzer.
- Remove the DBG_UART0_* pins.

UART0 guard:
- In pins.h: `#if defined(DBG_BACKEND_UART0)` and GP0/GP1 allocated -> `#error "UART0 console needs GP0/GP1, which are the right encoder"`.
- In CMake: DBG_DEFAULT=USB, and DBG_BACKEND=UART0 -> FATAL_ERROR giving that reason.

**thresholds.h:**
- GP26 ADC sanity constants: ADC_STUCK_LOW_RAW, ADC_STUCK_HIGH_RAW, ADC_NOISE_MAX_RAW, ADC_SANITY_SAMPLES. Values TODO(owner-to-confirm, buddy 3), TUNE.
- ENC_CPR and WHEEL_CIRC_MM: symbolic placeholders, TODO(owner-to-confirm, buddy 2). Never invent values, and config_check must still compile.
- A doc-only worst-case overflow bound for the int32 count: counts = ENC_CPR*4*distance/WHEEL_CIRC, and the 2^31 limit gives the maximum run distance.

**timing.h:** BC_CAPTURE_TIMEOUT_MS 1000 stays TUNE + TODO(owner-to-confirm, buddy 3), with `_Static_assert(BC_CAPTURE_TIMEOUT_MS*1000UL < (1UL<<20))`.

**Stubs:**
- hal_adc_sanity_check(): an empty body with /* TODO(phase 2, buddy 3): flag stuck-rail or abnormally noisy GP26 at boot as a sensor fault; logged and counted, never acts on motors */. SDK-side file, compile-only.
- enc_get_count(): a PROTOTYPE ONLY in app_program/subsystem2/drv_encoder.h, with no implementation.

**Console:**
- USB CDC is the ONLY console; the Debug Probe is SWD only.
- Rewrite docs/DEBUG.md:
  - SWD flashing;
  - opening the USB CDC COM port;
  - the mandatory USB CDC bring-up test (integration report section 14);
  - the 3 s pre-kernel stdio_usb_connected wait.
- Boot/fault output: a RAM fault record in the SDK's .uninitialized_data (NOLOAD, not zeroed; NOT .noinit), carrying a magic number AND a CRC.
  - It is read over SWD.
  - The dbg drain prints and clears it on the next boot over USB CDC, only when both match.
  - Implementation is Batch B; document it now.

**ARCHITECTURE.md deltas** (after D1, D2):

| Delta | Content |
|---|---|
| D3 | BC_POLL_PERIOD_MS 2 ms poll (the design has it ISR-woken) |
| D4 | IMU_HAS_GYRO=0: accel-only pitch, encoder-difference turning |
| D5 | Pin map v2. Supersedes design.md section 2; note that the Week 8 document still lists the old pins (not edited) |
| D6 | USB CDC-only console + RAM fault record (magic + CRC) |
| D7 | PIO0 quadrature encoders: one SM per wheel, one copy of the pico-examples sdk-1.5.1 program at offset 0 (24/32 instructions), the enc_get_count API, the deviation from NFR2's wording, and the proposed NFR2 wording as TODO(owner-to-confirm, team) with the top-speed zero-missed-counts test |
| D8 | Minimum-bit representation |
| D9 | EVF_SAFETY fault bits |
| D10 | Binary MQTT |
| D11 | Template merge deviations |

D2 stays TODO(owner-to-confirm, buddy 1); it is NOT approved.

**NOTICE.md:** append the BSD-3-Clause notice for the pico-examples quadrature_encoder program (sdk-1.5.1). Preserve the owner's edits.

**docs/DATA_REPRESENTATION.md.** Columns: Item | Field | Range | Resolution | Signed | Bits | Baseline bytes | Packed bytes (wire / MBF slot) | Justification | Clamp/OOR | Writer | Readers | Owner buddy. Content:
- General: masks and shifts only. Structure the header so that Buddy 1 can later switch PRIVATE state to bit-fields without any wire change.
- motion_cmd_t: type 3, speed 6 (0-50 cm/s), arg 9 shared by type (distance cm / angle 0-180 / signed steer), OOR 1. 19 bits, with a per-type range check.
- obstacle_profile_t:
  - fields: distance 9 (2-400 cm, 0 = no echo), bearing index 4 (0-12), width 7 (0-400 mm at 5 mm), clear_L 7, clear_R 7, OOR, and an OPTIONAL 2-bit seq (kept until the owner decides, TODO buddy 5);
  - the timestamp is DROPPED, and "bypass state" is REMOVED (flag for Buddy 5);
  - document drain-then-send for newest-wins, because a TMO_POL send fails on a full buffer (messagebuf.c:315-321).
- nav_input_t gains a 3-bit bypass command (stop, continue, left, right, reverse); flag for Buddy 5.
- Hump: 0-100 mm, 7 bits.
- Pitch: signed ±45° at 0.5°, 8 bits, clamp + OOR.
- Barcode command: 2 bits.
- Encoder counts: unpacked aligned int32, written only by the Motion task.
- Barcode edge ring: 20-bit gaps, 64 entries = 160 B (lo16[64] + hi-nibble[32]) vs the 512 B baseline.
- FSM widths:

  | FSM | Bits |
  |---|---|
  | System | 4 flat (RE-COUNT design.md section 5 at FULL resolution; use 3 if there are 8 or fewer states; record the count and the state list) |
  | Line | 3 |
  | Barcode | 3 |
  | Obstacle | 4 |
  | IMU | 3 |
  | WiFi | 3 |
  | VC authority | 1 |
  | Avoidance plan | 3 |

- EVF_SAFETY fault bit table:
  - Safety waits on COLLISION|IMU_ALIVE with TWF_ORW|TWF_BITCLR, never TWF_CLR.
  - FAULT_LATCHED latches both collisions and IMU watchdog trips; rename it in the docs. Only Safety sets it; only VC clears it.
  - "No echo" is informational only.
  - VC/Motion use tk_ref_flg.
- MQTT: binary frames, little-endian, version byte first. A command is rejected unless the version and length match; rejects are logged and never reach the VC.
- A language-neutral field table for the dashboard (no decoder).
- Week 13 statement: bit-packing barely changes RAM; report only what the map file shows.
- Baseline plan: B-baseline, then B-packed.
- Reference JSON per telemetry type, derived from design.md, plus a host CMake -P script (tools/json_sizes.cmake). Put its results in the doc. tools/ must be scanned by check_rules, or excluded with a reason.

=== FINAL VERIFICATION (exact commands + results) ===
1. Delete build/ first.
2. USB build: mtk3 (0 warnings, 0 errors), config_check (0 warnings, -Werror) and ALL (0 warnings in our code; check_rules PASS).
3. UART0 configure must FAIL with the guard message. Also prove the pins.h #error fires by compiling config_check.c with -DDBG_BACKEND_UART0=1.
4. check_rules and check_rules_selftest PASS.
5. Negative tests (temporary files deleted afterwards):
   - app_program/subsystem6/sources.cmake -> configure fails;
   - tm_printf in app_program/subsystem2/x.c -> R3b;
   - sleep_ms in the same file -> R2;
   - sleep_ms in src/foo/x.c -> still fires;
   - pio_sm_get_blocking -> R2;
   - a violation planted in ProjectTemplate/ -> NOT reported.
6. Release configure shows DBG_ENABLE=OFF; Release + APP_TEST=ON fails; Debug + APP_TEST=ON configures.
7. The submodule is clean at 15ed232c08f2d89e54e514db79de150e224307e3.
8. git log --follow shows the fixture renames.
9. git status shows no changes under ProjectTemplate/, 'INF 2004 Embedded Systems.md', design.md or specs.md.

Write .agents/tasks/merge-week8-summary.md and .agents/tasks/merge-week8-diffs.patch. Then STOP and wait for the owner's approval before Batch B.
