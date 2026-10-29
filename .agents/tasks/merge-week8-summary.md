# Summary: template merge + Week 8 pins/console/docs

Status: Stage 1 (template merge), Stage 2 (Week 8) and Stage 3 (final verification) are done
and verified. Nothing was committed or pushed. **Waiting for owner approval before Batch B.
Batch B was not started.**

Device: repo at `C:\Users\asus\Downloads\inf-2004-project`, SDK at the default path. HEAD
`67f9b00` (owner's WIP commit). The submodule folder was empty at start and was filled with
`git submodule update --init` (now clean at `15ed232c08f2d89e54e514db79de150e224307e3`).

Evidence: `merge-verify.md` + `merge-review.md/.json` (Stage 1), `week8-verify.md` +
`week8-review.md/.json` (Stage 2), `final-review.json` (Stage 3), this file, and
`merge-week8-diffs.patch`.

## 1. Files

Modified (tracked): `ARCHITECTURE.md`, `CMakeLists.txt`, `NOTICE.md` (append only),
`cmake/check_rules.cmake`, `cmake/check_rules_selftest.cmake`, `cmake/mtk3.cmake` (comment),
`config/mtk3/config_check.h`, `config/pins.h`, `config/priorities.h`, `config/thresholds.h`,
`config/timing.h`, `docs/DEBUG.md`, `HANDOVER.md` (section 2 status).

Moved (`git mv`, staged, R100):
`tests/rules_fixtures/src/motion/r1_mixed.c` -> `app_program/subsystem2/r1_mixed.c`,
`src/motion/r2_blocking.c` -> `app_program/subsystem2/r2_blocking.c`,
`src/line/r3_stdio.c` -> `app_program/subsystem3/r3_stdio.c`,
`src/vc/r4_libbsp.c` -> `app_program/subsystem2/r4_libbsp.c`,
`src/safety/r5_core1.c` -> `app_program/subsystem4/r5_core1.c` (all under `tests/rules_fixtures/`).

Created:
- `app_program/`: `app.h`, `app_main.c`, `test_report.h`, `sources.cmake`;
  `subsystem1..5/{subsystemN.c, subsystemN.h, subsystemN_test.c, sources.cmake}`;
  `subsystem2/drv_encoder.h`; `subsystem3/drv_ir.h`, `subsystem3/drv_ir.c`.
- `TEAM.md`; `testing/README.md` + `testing/{subsystem1..5,integration,system}/{TEST_LOG.md, logs/.gitkeep}`;
  `videos/README.md`.
- `docs/TEMPLATE_NOTES.md`, `docs/DATA_REPRESENTATION.md`.
- `tools/json_sizes.cmake`, `tools/json_baseline/*.json` (10 files).
- Fixtures `tests/rules_fixtures/app_program/subsystem2/r2_pio_blocking.c`, `r1_drv_subsystem_hdr.c`.
- `.agents/tasks/`: `merge-verify.md`, `merge-review.md/.json`, `week8-verify.md`,
  `week8-review.md/.json`, `final-review.json`, `merge-week8-summary.md`, `merge-week8-diffs.patch`.

Never edited: `ProjectTemplate/**`, `external/mtk3_bsp/**`, `INF 2004 Embedded Systems.md`,
`design.md`, `specs.md`, `README.md`, `.gitignore` (`git status` empty for all of them).

## 2. Item status

M-01..M-17 done (table in `merge-verify.md`). W-01..W-15 done (table in `week8-verify.md`).
MV-1..MV-8, WV-1..WV-8, FV-1..FV-4 pass.

## 3. Final verification (FV), `build/` deleted first

Preamble: HANDOVER section 3. Logs: `build/final-logs/`. Warnings/errors = `warning:` / `error:`
lines per log.

| # | Command | Exit | Warnings | Errors | Result |
|---|---|---|---|---|---|
| 1 | `Remove-Item -Recurse -Force build` | 0 | | | `build` absent |
| 2 | `& $CM -S . -B build/usb @common -DDBG_BACKEND=USB` | 0 | 0 | 0 | `robocar: DBG_ENABLE=ON DBG_BACKEND=USB TEL_BACKEND_WIFI=OFF APP_TEST=OFF`; `mtk3: ... pinned at 15ed232..., clean`; `robocar: src/app/main.c not present (Batch B); firmware target skipped, app_program compile-checked` |
| 2 | `& $CM --build build/usb --target mtk3` | 0 | 0 | 0 | 0 warnings in mtk3 |
| 2 | `--target config_check` | 0 | 0 | 0 | `-Werror` |
| 2 | `--target app_program_check` | 0 | 0 | 0 | kernel-side stubs, `-Werror` |
| 2 | `--target app_program_check_sdk` | 0 | 0 | 0 | `drv_ir.c`, `-Werror` |
| 2 | `& $CM --build build/usb` (ALL) | 0 | 0 | 0 | `check_rules: PASS (42 files scanned)` |
| 2 | `& $CM -S . -B build/default @common` | 0 | | | `DBG_BACKEND=USB` (new default) |
| 3 | `& $CM -S . -B build/uart0 @common -DDBG_BACKEND=UART0` | 1 | | | `DBG_BACKEND=UART0 was removed (Week 8 pin map v2): GP0/GP1 are ENC_R_A/ENC_R_B in config/pins.h. USB CDC is the only console (docs/DEBUG.md).` |
| 3 | `build/uart0c`: `-DDBG_BACKEND=USB "-DCMAKE_C_FLAGS=-DDBG_BACKEND_UART0=1"`, `--target config_check` | 1 | | 1 | `pins.h:279:2: error: #error "pins.h: UART0 console needs GP0/GP1, which are the right encoder (ENC_R_A/ENC_R_B)"` |
| 3b | mutations `SERVO_PIN (12U)` / `ENC_L_B_PIN (18U)` / `BC_CAPTURE_TIMEOUT_MS (1049U)` / extra GP13 in the mask, `--target config_check` | 1 each | | 3 / 2 / 2 / 2 | spare + slice + channel; duplicate + adjacency; 20-bit gap `#error` + static assert; spare (+ duplicate). Reverted; `config_check` 0 |
| 4 | `--target check_rules_selftest`; `& $CM -P cmake/check_rules.cmake`; `& $CM -P cmake/check_rules_selftest.cmake` | 0 / 0 / 0 | | | PASS / PASS (42 files) / PASS |
| 4 | `& $CM -DRULES_ROOT=<root>/tests/rules_fixtures -P cmake/check_rules.cmake` | 1 | | | `check_rules: 13 violation(s)`, all expected, none in `ProjectTemplate/`, `docs/`, `src/app/main.c` |
| 5 | temp `app_program/subsystem6/sources.cmake`, configure | 1 | | | `... is not registered.` (also `app_program/subsystem2/extra/`, `src/newmod/`: 1 each) |
| 5 | temp `app_program/subsystem2/x.c` with `tm_printf` / `sleep_ms` / `pio_sm_get_blocking`, `-P check_rules.cmake` | 1 each | | | `[R3b]` / `[R2]` / `[R2]` |
| 5 | temp `app_program/subsystem5/drv_y.c` (`subsystem5.h` + `hardware/gpio.h`) | 1 | | | `[R1]` |
| 5 | temp `src/foo/x.c` with `sleep_ms` | 1 | | | `[R2]` |
| 5 | violation inside `ProjectTemplate/` | | | | not reported: fixture `tests/rules_fixtures/ProjectTemplate/app_program/subsystem1/excluded.c` (`tm_printf` + `sleep_ms`) is never listed, and the self-test fails on any `ProjectTemplate/` line. No file was planted in the real `ProjectTemplate/` (never-edit rule) |
| 5 | cleanup, reconfigure + rebuild | 0 | 0 | 0 | temp files and dirs gone, `src/` absent |
| 6 | Release configure | 0 | | | `DBG_ENABLE=OFF` |
| 6 | Release + `-DAPP_TEST=ON` | 1 | | | `APP_TEST=ON needs DBG_ENABLE=ON: test results are printed through the dbg_ ring` |
| 6 | Debug + `-DAPP_TEST=ON` configure / build | 0 / 1 | 0 | 5 | configures with `APP_TEST=ON`; the build stops only at the deliberate `test_report.h:35` `#error` (Batch B deferral) |
| 6b | `& $CM -P tools/json_sizes.cmake` | 0 | | | output contained verbatim in DATA_REPRESENTATION.md section 8 |
| 7 | `git -C external/mtk3_bsp status --porcelain`; `git submodule status` | 0 | | | empty; ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)` |
| 8 | `git diff --cached -M --name-status` | 0 | | | five `R100` fixture renames. `git log --follow` cannot show them yet: renames are staged, not committed; after the owner commits, `git log --follow` will |
| 9 | `git status --porcelain -- ProjectTemplate design.md specs.md "INF 2004 Embedded Systems.md" README.md .gitignore` | 0 | | | empty |

## 4. Facts carried forward

- F-1 Vehicle Controller owner = Buddy 2 (design.md 3.1), so subsystem2 = motion + VC.
- F-2/F-3 Student IDs only from the Team Information table; `ProjectTemplate/TEAM.md` is still
  blank; GitHub usernames are not on disk.
- F-4 the fault record needs no linker change: SDK `.uninitialized_data` is NOLOAD and never
  zeroed (`cmake/memmap_mtk3.ld:191`); `.noinit` is zero-filled (`:217-220`, ASSERT `:275`).
- `messagebuf.c` on disk: free-space check :315, `E_TMOUT` :319, `TMO_POL` wait test :320.

## 5. System-FSM recount

design.md `[image2]` (PNG, native 660 x 409 px, the only resolution in the document) counted at
native size. POWER_OFF excluded (legend: "Power state"). States: INITIALISING,
READY_WAITING_FOR_START, OBSTACLE_AVOIDANCE/BYPASS, SAFETY_COLLISION_LATCHED_FAULT, and the
AUTONOMOUS_NAVIGATION sub-states LINE_TRACKING, HUMP_SENSING, TURN_LEFT, GO_STRAIGHT, U_TURN,
TURN_RIGHT (the diamond is a choice pseudo-state). **10 states > 8, so 4 bits flat.** Small
labels are only approximately legible; the count does not depend on the wording.

## 6. Sequence-number analysis (`obstacle_profile_t`)

A `TMO_POL` send on a full buffer fails with `E_TMOUT` and never overwrites, so "newest profile
wins" needs drain-then-send (drain with `tk_rcv_mbf(TMO_POL)` until empty, then send; with
2 slots, only after a failed send). Scanning (P6) outranks Avoidance (P7), so Avoidance cannot
run between the drain and the send. With drain-then-send in place the 2-bit sequence only adds
detection of a repeated reading (no new scan) or a lost profile; without it, it is the only
staleness check. It costs 2 bits and no MBF bytes (12 B slot either way). It is **kept**,
TODO(owner-to-confirm, buddy 5).

## 7. NFR2 proposed wording (ARCHITECTURE.md D7, TODO(owner-to-confirm, team))

"Wheel encoder pulses shall be captured by hardware (PIO quadrature decoding) on both wheels
independently, with zero missed counts under normal operating speed (target: up to 50 cm/s)."
Reason: PIO decoding counts with zero CPU and keeps counting through kernel interrupt-masked
sections. "Zero missed counts" needs a top-speed test: drive a measured distance at 50 cm/s and
compare counts with expected, both wheels. The current NFR2 (`design.md:102`) says "captured
using GPIO interrupts", so the team does need to update it. No team document was edited.

## 8. Assumptions

1. App files were written as full rewrites of the template files (no template bytes kept).
   `TEAM.md` was copied, then rewritten in full; `testing/` and `videos/` were copied and edited
   with UTF-8 (no BOM) writes, never `Get-Content | Set-Content`.
2. W-03 constant names follow the brief (`ADC_SANITY_SAMPLES`, `ADC_STUCK_LOW_RAW`,
   `ADC_STUCK_HIGH_RAW`, `ADC_NOISE_MAX_RAW`, plus `ADC_FULL_SCALE_RAW`), not the plan's
   `IR_SANITY_*`, because the brief takes precedence. Values 16 / 16 / 4079 / 400 are provisional.
3. The encoder overflow `#error` checks one second of counts at top speed (covers the 20 ms delta).
4. EVF_SAFETY bit positions, separate `FAULT_IMU_I2C` / `FAULT_IMU_WDOG` bits and
   `FAULT_IR_SENSOR` (b9) are proposals.
5. JSON reference files and binary payload layouts (24-bit timestamp + packed fields; roll uses
   the pitch format) are planning figures for the size comparison, not a protocol.
6. TEAM.md budget table has an extra "Platform" column for the init, drain and test tasks.
7. `app_program_check_sdk` links `hardware_*` INTERFACE libraries, so their SDK sources also
   compile inside it (no `-Werror`, 0 warnings, no crt0/stdio).
8. The plan's "HEAD still df67420" check was read as "HEAD unchanged": HEAD is the WIP commit
   `67f9b00` throughout.

## 9. Open TODO(owner-to-confirm) items

GitHub usernames (TEAM.md); template author (NOTICE.md); instructor's build preference
(README.md, ARCHITECTURE.md T2); platform (`src/`) owner (TEAM.md); D2 telemetry wake (buddy 1);
`ENC_CPR`, `WHEEL_CIRC_MM` (buddy 2); `BC_CAPTURE_TIMEOUT_MS` (buddy 3); ADC sanity limits
(buddy 3); steering sign (buddy 3); bypass command and 2-bit sequence (buddy 5); NFR2 wording
(team); every IF "Agreed by" cell (TEAM.md); IMU addresses (buddy 4); `CAR_WIDTH_MM` (buddy 5);
servo pulse range; motor duty cap and 3V3 budget (buddy 2).

## 10. Owner questions (with suggested answers)

| # | Question | Suggested answer |
|---|---|---|
| 1 | GitHub usernames for TEAM.md | Each buddy fills in their own row; keep the TODO until then |
| 2 | `ProjectTemplate/TEAM.md` is still blank although it was said to be filled in | Commit the filled-in copy if it exists; otherwise the Team Information table stays the source |
| 3 | Instructor's build preference (make/WSL vs CMake/Ninja) | Ask before the Week 10 demo; keep the README TODO |
| 4 | D2: 20 ms telemetry wake | Buddy 1 decides; FR1 rates hold either way |
| 5 | Keep the 2-bit sequence number in `obstacle_profile_t`? | Keep until drain-then-send is implemented and tested, then Buddy 5 decides (section 6) |
| 6 | Approve the NFR2 wording (section 7)? | Approve after the top-speed zero-missed-counts test on both wheels |
| 7 | `ENC_CPR`, `WHEEL_CIRC_MM`, `CAR_WIDTH_MM`, IMU addresses, servo pulse range | Measure on the car (encoder spec or a counted wheel turn, ruler, I2C bus scan, servo sweep) |
| 8 | Barr C: bit-fields for private state later? | Buddy 1 checks the standard; masks and shifts until then (the header layout already allows the switch) |
| 9 | Accept the proposed EVF_SAFETY bit table (DATA_REPRESENTATION.md section 3), incl. `FAULT_IR_SENSOR`? | Yes, fix positions in Batch B-packed |
| 10 | Platform (`src/`) owner | Buddy 2 (all platform TODOs are labelled buddy 2); confirm |
| 11 | Template author for NOTICE.md | Use the module instructor's name as given on the course page |
| 12 | Batch B approval | Approve after reviewing this summary and the patch |

## 11. Next step

Owner reviews this summary and `merge-week8-diffs.patch`, decides on the commit, and approves
Batch B (B-baseline, then B-packed). Batch B was not started.
