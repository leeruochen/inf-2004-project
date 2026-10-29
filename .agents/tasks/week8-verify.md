# Week 8 verification (Stage 2: W-01..W-15, WV-1..WV-8)

Started after `merge-review.json` = APPROVED. Repo root `C:\Users\asus\Downloads\inf-2004-project`,
HEAD `67f9b00`, no commit. Logs: `build/week8-logs/` (git-ignored).

## W-items

| Item | Status | Notes |
|---|---|---|
| W-01 | Done | `config/pins.h` v2: ENC_R_A/B 0/1, ENC_L_A/B 16/17, DIR_INVERTs (motor + encoder, bring-up order), `ENC_PIO_NUM 0`, SM 0 right / 1 left; B = A + 1 and A even checks; 6-pin connector block; IR_C 28/ADC2, IR_R 26/ADC0 with `(pin-26)==ch`; GP26 note (Grove 6 lead same pin, not taped, firmware cannot disconnect); IR_BC 27 (never `adc_gpio_init`); GP28 VBAT jumper open; SERVO 15 slice 7 B with slice/channel checks and servo-vs-motor slice check, "never GP12"; spares GP6, GP7, GP12-GP14; reserved GP18/20/21/22; DBG_UART0_* deleted; UART0 `#error` backstop. P-8 line removed from TEAM.md |
| W-02 | Done | `DBG_DEFAULT "USB"`, STRINGS `USB`, UART0 FATAL_ERROR with reason, other values "use USB"; only `DBG_BACKEND_USB=1`; firmware block always USB stdio; `hardware_pio` in robocar and `app_program_check_sdk`; header examples USB only |
| W-03 | Done | GP26 sanity constants with the **brief's names** `ADC_SANITY_SAMPLES`, `ADC_STUCK_LOW_RAW`, `ADC_STUCK_HIGH_RAW`, `ADC_NOISE_MAX_RAW` (+ `ADC_FULL_SCALE_RAW`); the plan's `IR_SANITY_*` names were not used because the brief takes precedence over the plan. Values PROVISIONAL, TUNE, TODO(owner-to-confirm, buddy 3). `ENC_QUAD_MULT 4`, `ENC_CPR 0`, `WHEEL_CIRC_MM 0` TODO(owner-to-confirm, buddy 2); overflow bound comment; guarded check (counts per second at top speed fit `int32_t`; stricter than the plan's 20 ms delta and needs no timing.h include) |
| W-04 | Done | `BC_GAP_BITS 20`; `#error` in timing.h; `_Static_assert` mirror in config_check.h; `BC_CAPTURE_TIMEOUT_MS 1000` stays TUNE + TODO(owner-to-confirm, buddy 3); D2 text unchanged |
| W-05 | Done | `app_program/subsystem3/drv_ir.h` / `drv_ir.c` (SDK side, `hardware/adc.h`), registered in `ROBOCAR_SOURCES`; body is the brief's TODO text |
| W-06 | Done | `app_program/subsystem2/drv_encoder.h` (prototype only, `ENC_WHEEL_LEFT/RIGHT`), included from `subsystem2.c`; no `drv_encoder.c` |
| W-07 | Done | TEAM.md "Hardware checklist" |
| W-08 | Done | `docs/DEBUG.md` rewritten: USB CDC only, probe SWD only, COM port and re-enumeration, 3 s wait, RAM fault record (`.uninitialized_data`, magic + CRC-32, SWD read, print/clear rule), options (UART0 fails), mandatory USB CDC bring-up test, APP_TEST section kept |
| W-09 | Done | ARCHITECTURE.md D3-D11; D2 unchanged (PROPOSED, TODO(owner-to-confirm)) |
| W-10 | Done | NOTICE.md: pico-examples `quadrature_encoder.pio` (sdk-1.5.1) notice + full BSD-3 text. Text checked against the fetched `LICENSE.TXT` at tag sdk-1.5.1 and the `.pio` header (2023 copyright, SPDX BSD-3-Clause) |
| W-11 | Done | `[image2]` extracted to `%TEMP%\design_image2.png` (PNG 660 x 409, native; outside the repo). 10 software states (4 top level excluding POWER_OFF + 6 AUTONOMOUS_NAVIGATION sub-states) > 8 -> 4 bits flat. Recorded in DATA_REPRESENTATION.md section 4 |
| W-12 | Done | `docs/DATA_REPRESENTATION.md` |
| W-13 | Done | `tools/json_baseline/*.json` (10 files, minified, no trailing newline) |
| W-14 | Done | `tools/json_sizes.cmake`; output pasted verbatim |
| W-15 | Done | answer below |

## Commands and results

| # | Command | Exit | Warnings | Errors | Result |
|---|---|---|---|---|---|
| WV-1 | `& $CM -S . -B build/usb @common -DDBG_BACKEND=USB` (fresh) | 0 | 0 | 0 | `DBG_BACKEND=USB ... APP_TEST=OFF`; pinned 15ed232, clean; skip line |
| WV-1 | `--target mtk3` / `config_check` / `app_program_check` / `app_program_check_sdk` / ALL | 0 each | 0 each | 0 each | `check_rules: PASS (42 files scanned)` (38 + drv_ir.c, drv_ir.h, drv_encoder.h, tools/json_sizes.cmake) |
| WV-1 | `& $CM -S . -B build/default @common` (no `-DDBG_BACKEND`) | 0 | | | `DBG_BACKEND=USB` |
| WV-2 | `& $CM -S . -B build/uart0 @common -DDBG_BACKEND=UART0` | 1 | | | `DBG_BACKEND=UART0 was removed (Week 8 pin map v2): GP0/GP1 are ENC_R_A/ENC_R_B in config/pins.h. USB CDC is the only console (docs/DEBUG.md).` |
| WV-2 | `build/uart0c` with `-DDBG_BACKEND=USB "-DCMAKE_C_FLAGS=-DDBG_BACKEND_UART0=1"`, `--target config_check` | 1 | | 1 | `pins.h:279:2: error: #error "pins.h: UART0 console needs GP0/GP1, which are the right encoder (ENC_R_A/ENC_R_B)"` |
| WV-3 | `SERVO_PIN (12U)` | 1 | | 3 | spare pin allocated; SERVO_PWM_SLICE / SERVO_PWM_CHAN mismatch |
| WV-3 | `ENC_L_B_PIN (18U)` | 1 | | 2 | two pins share a GPIO; pair must be adjacent B = A + 1 |
| WV-3 | `BC_CAPTURE_TIMEOUT_MS (1049U)` | 1 | | 2 | `timing.h: BC_CAPTURE_TIMEOUT_MS no longer fits a 20-bit microsecond gap` + the config_check.h `_Static_assert` |
| WV-3 | extra `PIN_BIT(13U)` term in `PINS_ALLOCATED_MASK` | 1 | | 2 | spare pin allocated (and SUM != MASK) |
| WV-3 | all reverted, `--target config_check` | 0 | 0 | 0 | |
| WV-4 | `--target check_rules_selftest`, `-P check_rules.cmake`, `-P check_rules_selftest.cmake` | 0 | | | PASS, PASS (42 files), PASS |
| WV-4 | fixtures run | 1 | | | 13 violations (same list as MV-2) |
| WV-4 | MV-3/MV-4 negatives repeated (subsystem6, subsystem2/extra, src/newmod; R3b, R2 sleep_ms, R2 pio_sm_get_blocking, R1 drv + subsystem5.h, R2 src/foo) | 1 each | | | expected line each; temp files removed; reconfigure + rebuild 0 |
| WV-5 | Release configure | 0 | | | `DBG_ENABLE=OFF` |
| WV-5 | Release + `-DAPP_TEST=ON` | 1 | | | `APP_TEST=ON needs DBG_ENABLE=ON: ...` |
| WV-5 | Debug + `-DAPP_TEST=ON` configure / build | 0 / 1 | 0 | 5 | all 5 errors are the P-4 `#error` in `test_report.h:35` |
| WV-6 | `& $CM -P tools/json_sizes.cmake` | 0 | | | output contained verbatim in DATA_REPRESENTATION.md section 8 (checked with `String.Contains`) |
| WV-7 | submodule | | | | clean, ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)` |
| WV-7 | `git status --porcelain -- ProjectTemplate design.md specs.md "INF 2004 Embedded Systems.md" README.md .gitignore` | | | | empty |
| WV-7 | staged | | | | only the five R100 renames; HEAD `67f9b00` |
| WV-7 | `Select-String config,app_program,docs,TEAM.md,ARCHITECTURE.md -Pattern "DBG_UART0|GP12.*servo|IR_L_|UART0"` | | | | only "removed / never / guard" explanations (pins.h header and guard, DEBUG.md, TEMPLATE_NOTES.md, TEAM.md "unused" and "Never the servo pin", ARCHITECTURE.md D6). No `DBG_UART0_*` or `IR_L_*` macro remains |

## W-15: does the team need to update the NFR2 wording?

Yes. NFR2 says encoder pulses "shall be captured using GPIO interrupts on both
wheels independently" (`design.md:102`), and delta D7 counts them with PIO0
instead. The proposed wording is recorded only in ARCHITECTURE.md D7 as
TODO(owner-to-confirm, team): "Wheel encoder pulses shall be captured by
hardware (PIO quadrature decoding) on both wheels independently, with zero
missed counts under normal operating speed (target: up to 50 cm/s)." Reason:
PIO decoding counts with zero CPU and keeps counting through kernel
interrupt-masked sections. "Zero missed counts" needs a top-speed test (drive a
measured distance at 50 cm/s, compare counts with expected, both wheels). No
team document was edited.

## Observations

- `messagebuf.c` line numbers on disk: free-space check :315, `ercd = E_TMOUT` :319, the
  `TMO_POL` wait test :320 (the plan's F-7 said :320/:321). DATA_REPRESENTATION.md cites the
  on-disk lines and the owner's range 315-321.
- EVF_SAFETY bit b9 `FAULT_IR_SENSOR` (for `hal_adc_sanity_check()`) and the separate
  `FAULT_IMU_I2C` / `FAULT_IMU_WDOG` bits are proposals (one setter per bit); positions are
  fixed in Batch B.
