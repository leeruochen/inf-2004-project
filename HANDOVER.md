# HANDOVER: continuing Phase 1 on another device

Written when the work was paused, partway through the "template merge + Week 8 pins/console" task. Read this file first, then the reports in `.agents/tasks/` (listed in section 7). Those reports hold all the evidence, decisions and line-level plans; this file only tells you where things stand and what to do next.

---

## 0. AGENT QUICK START (follow this when the owner says "read the handover.md and continue the work")

You are continuing an interrupted task. **Do not re-plan, re-investigate or re-ask questions that are already answered.** Every decision is recorded here and in the files below. Do exactly this:

1. **Find the repo root.** It is the folder containing this file. Paths inside `.agents/tasks/*.md` are written as `C:/Users/Admin/Downloads/Projects/inf-2004-project/...`; if the root is different on this device, substitute it. If the SDK is not at `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1`, find it and change `$R` (section 3), and tell the owner.
2. **Read, in this order:**
   1. this file, fully;
   2. the **"OWNER CORRECTIONS"** block at the top of `.agents/tasks/week8-impact-report.md` (binding; it overrides everything else);
   3. `.agents/tasks/merge-week8-brief.md`, fully. This is the original task brief. When the plan says "the brief" or "gets the full Step 3 text from the brief", it means this file;
   4. `.agents/tasks/merge-week8-plan.md`, fully. **This is your checklist.** Items M-01..M-17, MV-1..MV-8, W-01..W-15, WV-1..WV-8 and FV-1..FV-4, with exact files, contents and commands;
   5. as needed: `step0-verify.md`, `merge-samename-diffs.md`, `template-merge-report.md` (section 3), `week8-impact-report.md` (sections A-F), `batchA-summary.md` (verification standard), `mtk3-rp2040-integration-report.md`.
3. **Check the starting state** (section 2 describes it):
   - `git log --oneline -n 3`: HEAD should be the owner's WIP commit containing Steps 0-1 and this file.
   - `app_program/` must **not** exist yet. `git submodule status` must show `15ed232c08f2d89e54e514db79de150e224307e3`. If the submodule folder is empty, run `git submodule update --init`.
   - Run the sanity build in section 3, step 4. It must pass before you change anything.
   - If the state differs (for example `app_program/` already exists, or the build fails), **stop and report to the owner** instead of guessing.
4. **Execute the remaining work in this exact order.** Do not skip, reorder or merge stages:
   - **Stage 1, template merge:** plan items **M-01..M-17**, then the checks **MV-1..MV-8**. Fix until every MV check passes.
   - **Stage 2, Week 8 pins/console/docs:** items **W-01..W-15**, then **WV-1..WV-8**.
   - **Stage 3, final verification:** **FV-1..FV-4**, deleting `build/` first (section 4.3).
   - The plan names review gate files (`merge-review.json`, `week8-review.json`, `final-review.json`). After each stage, review your own diff against the plan, the brief and section 5 as a strict code reviewer. Write the matching `.agents/tasks/<stage>-review.md` and the gate file `{"verdict":"APPROVED"}` (or `"CHANGES_REQUESTED"` plus the findings). Start the next stage only after an APPROVED verdict.
   - The plan's merge check MV-1 configures a `uart0` build. That check is correct for Stage 1, because the UART0 backend is only removed in Stage 2 (W-items). After Stage 2, UART0 configure must FAIL.
   - If tools for delegated workflows are available, you may run each stage as an implement-then-review loop, but keep the order and the gates.
5. **Gates. Stop and ask the owner (do not proceed) only if:**
   - a change to `README.md` or `.gitignore` seems necessary (show the full diff first; they are approved same-name files);
   - a plan item would contradict section 5, section 6 or the OWNER CORRECTIONS;
   - a verification check cannot be made to pass without changing an approved decision;
   - you would need to edit `ProjectTemplate/`, `external/mtk3_bsp/`, `INF 2004 Embedded Systems.md`, `design.md` or `specs.md` (never allowed).
   Anything else that is a judgement call: pick the option the plan or report recommends, and record it as an assumption.
6. **Finish:**
   - write `.agents/tasks/merge-week8-summary.md` with: files changed/created/moved, assumptions, open items, every owner question with a suggested answer, the NFR2 proposed wording, the system-FSM recount result, the sequence-number analysis, and the full table of commands and results (Batch A format);
   - save `git diff` of every modified tracked file to `.agents/tasks/merge-week8-diffs.patch`;
   - update section 2 of this file to show the new status;
   - **do not commit or push.** Report the summary to the owner and **wait for approval before Batch B. Never start Batch B in this task.**

---

## 1. Project in one paragraph

Autonomous line-following car on a Raspberry Pi **Pico W (RP2040)** with a Cytron Robo Pico board, programmed in **C** with **Pico SDK 1.5.1** and **micro T-Kernel 3.0** (official single-core `tron-forum/mtk3_bsp`, branch `pico_rp2040`, pinned submodule at `external/mtk3_bsp`, commit `15ed232c08f2d89e54e514db79de150e224307e3`). It builds with CMake and Ninja on Windows and follows the Barr C standard. The Pico SDK owns startup, linker and vector table; the kernel is a static library (`mtk3`) behind a port layer. Sources of truth: `design.md` (team design), `specs.md` (brief), and `INF 2004 Embedded Systems.md` (the team's Week 8 submission; **read only**). Phase 1 = a skeleton only: every task is a stub with `/* TODO(phase N, buddy X): ... */`.

## 2. Where things stand

| Item | Status |
|---|---|
| Kernel integration report | Done, approved (`.agents/tasks/mtk3-rp2040-integration-report.md`) |
| Batch A (build system + config headers) | Done, reviewer-approved, **committed** (commit `56826c7`, also branch `backup/pre-template-merge`) |
| Professor's template | Committed as `ProjectTemplate/` (commit `79bc417`). **Keep it tracked and in place. Never move, delete or edit it.** It is reference only. |
| Template merge, Step 0 (unblock `check_rules`) | **Done, uncommitted.** `ProjectTemplate/`, `testing/`, `videos/` are excluded from `check_rules`; the CMake guard skips `ProjectTemplate/`. Evidence: `.agents/tasks/step0-verify.md` |
| Template merge, Step 1 (same-name gate) | **Done, uncommitted.** Owner approved the `README.md` and `.gitignore` diffs (`.agents/tasks/merge-samename-diffs.md`); both applied as proposed. |
| Merge + Week 8 implementation plan | **Written:** `.agents/tasks/merge-week8-plan.md` (items M-01..M-17, MV-1..8, W-01..W-15, WV-1..8, FV-1..4). This is the to-do list for the rest of the task. |
| Template merge, Step 2 (M-items) | **NOT done.** The coding step was paused before it changed any file. |
| Week 8 Step 3 (W-items: pins, console, docs) | **NOT done.** |
| Final verification (FV-items) | **NOT done.** |
| Batch B (common, rtos/port, debug) | **Not started. Do not start it** until the owner approves, after this task is verified. |

Uncommitted changes in the working tree at handover (made by Steps 0 and 1, plus this handover):

```
 M .gitignore                      (Step 1 merge + .agents/tasks now shared)
 M CMakeLists.txt                  (Step 0: sources.cmake guard skips ProjectTemplate)
 M README.md                       (Step 1: approved rewrite)
 M cmake/check_rules.cmake         (Step 0: exclusions)
 M cmake/check_rules_selftest.cmake (Step 0: new fixture expectations)
?? tests/rules_fixtures/ProjectTemplate/app_program/subsystem1/excluded.c
?? tests/rules_fixtures/app_program/subsystem1/r3_app_program.c
?? HANDOVER.md
?? .agents/tasks/*                 (shared reports, see section 6)
```

**To move this work to the other device, commit and push it from this device first** (for example `git add -A` then `git commit -m "WIP: template merge steps 0-1 + handover"` then `git push`). Check `git status` afterwards: `ProjectTemplate/` must show no changes, and `config/secrets.h` must not be staged.

## 3. Setting up the other device

1. Install the **Raspberry Pi Pico SDK v1.5.1 Windows installer** at `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1` (it bundles GCC 10.3.1, CMake 3.25.2, Ninja 1.11.1, Python, Git). Use the same path, or change `$R` below.
2. Clone with the submodule: `git clone --recursive https://github.com/leeruochen/inf-2004-project.git`. If already cloned: `git submodule update --init`. Check: `git submodule status` must show `15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp`. Details: `docs/SUBMODULE.md`.
3. In each new PowerShell window, from the repo root, run the build preamble:

```powershell
$R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"
$env:PATH = "$R\cmake\bin;$R\gcc-arm-none-eabi\bin;$R\ninja;$R\python;$R\git\cmd;$env:PATH"
$env:PICO_SDK_PATH = "$R\pico-sdk"
$CM = "$R\cmake\bin\cmake.exe"
$common = @("-G","Ninja","-DCMAKE_BUILD_TYPE=Debug",
  "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk",
  "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools",
  "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe",
  "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi")
```

4. Sanity check that Steps 0-1 still build. **Note:** the CMake default is still `DBG_BACKEND=UART0` until Week 8 item W-02 changes it, so pass USB explicitly:

```powershell
& $CM -S . -B build/usb @common -DDBG_BACKEND=USB
& $CM --build build/usb            # expect: check_rules PASS, 0 warnings
& $CM -P cmake/check_rules_selftest.cmake   # expect: PASS
```

## 4. What remains to be done (in this order)

The exact items, file lists and checks are in **`.agents/tasks/merge-week8-plan.md`**. Follow it item by item. Summary:

### 4.1 Template merge, Step 2 (plan items M-01..M-17), then verify (MV-1..MV-8)
- Create `app_program/` from the template **as stubs** (Barr C naming, template file-header block): `app.h` (two-phase contract, kernel side only), `app_main.c` (= kernel `usermain()`), `test_report.h` (output through the future `dbg_` API), and `subsystem1..5/{subsystemN.c,.h,_test.c}`.
  - Mapping: 1 = telemetry (Buddy 1), 2 = motion + Vehicle Controller (Buddy 2), 3 = line + barcode (Buddy 3), 4 = IMU + safety (Buddy 4; `task_safety.c` here, calling `motion_emergency_brake()` only through `subsystem2.h`), 5 = obstacle scan + avoid (Buddy 5).
  - `usermain` order: create all IPC objects (TODO, Batch B), every `subsystemN_init()`, every `subsystemN_start()`, the test task only when `APP_TEST`, then `tk_slp_tsk(TMO_FEVR)`.
  - `src/app/main.c` stays the planned SDK `main()` (created in Batch B). Explain both roles in file headers and `ARCHITECTURE.md`.
  - Do **not** port the template demo tasks (`task_led`, `task_producer`, `task_consumer`, `task_monitor`). Write `docs/TEMPLATE_NOTES.md` with the patterns worth keeping.
- `drv_*.c` include SDK headers only; `task_*.c`, `subsystemN.c`, `subsystemN_test.c` include kernel headers only.
- `APP_TEST` CMake option: tests run in a dedicated test task at priority 11 (below the debug drain at 10), output via `dbg_`. `APP_TEST=ON` with `DBG_ENABLE=OFF` fails at configure. Task-count assert goes up by one.
- The firmware still cannot link (no port layer / `main.c` until Batch B), so add a **compile-only** object target for the stubs, built in ALL with `-Werror`.
- `git mv` the five rule fixtures (`tests/rules_fixtures/src/{motion,line,vc,safety}/...` to `tests/rules_fixtures/app_program/subsystem{2,3,2,4}/...`) and update the self-test. Add `pio_sm_get_blocking` / `pio_sm_put_blocking` to rule R2.
- New root records: `TEAM.md` (ported from `ProjectTemplate/TEAM.md` with OUR pins, peripherals, priorities 1-10, IF rows from design.md 3.3, object budget), `testing/` and `videos/` (from the template, our commands, example rows cleared).
- Docs: `ARCHITECTURE.md` (two source roots), `NOTICE.md` (append the template credit), `docs/DEBUG.md`.

### 4.2 Week 8 change request, Step 3 (plan items W-01..W-15), then verify (WV-1..WV-8)
Pins, console and documentation **only**. No packed types and no encoder code yet.
- `config/pins.h` v2: right encoder GP0/GP1 (Grove 1), left encoder GP16/GP17 (Grove 4), quadrature, A on the lower pin is provisional, `ENC_R/L_DIR_INVERT` and `MOTOR_L/R_DIR_INVERT`; IR_C on GP28 = ADC2 (Grove 7, centre), IR_R on GP26 = ADC0 (Grove 5, the car's right); barcode IR_BC GP27 (Grove 6) digital IRQ; servo **GP15** (slice 7 B, confirmed); spare GP6, GP7, GP12-GP14; GP28 VBAT jumper must stay open; GP26 note (Grove 6's GP26 lead is the same pin and is not taped off); 6-pin motor/encoder connector comment; remove the UART0 pins.
- UART0 guard: `DBG_BACKEND=UART0` fails at configure; `pins.h` `#error` backstop. Default becomes USB.
- `thresholds.h`: GP26 ADC sanity constants; `ENC_CPR` and `WHEEL_CIRC_MM` placeholders (TODO, buddy 2, never invent values). `timing.h`: static assert that `BC_CAPTURE_TIMEOUT_MS` fits the 20-bit gap.
- Stubs: `hal_adc_sanity_check()` (TODO phase 2, buddy 3) and the `enc_get_count()` prototype only.
- `docs/DEBUG.md`: USB CDC is the only console; Debug Probe is SWD only; the USB CDC bring-up test is mandatory.
- `ARCHITECTURE.md` deltas D3-D11 (D3 barcode 2 ms poll, D4 no gyro, D5 pin map v2, D6 USB-only console + RAM fault record, D7 PIO0 encoders + proposed NFR2 wording, D8 minimum-bit representation, D9 fault bits, D10 binary MQTT, D11 template deviations).
- `NOTICE.md`: append the BSD-3 notice for the pico-examples quadrature program.
- `docs/DATA_REPRESENTATION.md` (the agreed field table) and a host CMake script for reference-JSON byte sizes.

### 4.3 Final verification (FV-1..FV-4)
Delete `build/` first, then:
- USB build: `mtk3`, `config_check` and ALL build with **0 warnings** in our code and in mtk3; `check_rules` passes.
- `-DDBG_BACKEND=UART0` configure **must fail** with the guard message (negative test). The `pins.h` `#error` must also fire when compiled with `-DDBG_BACKEND_UART0=1`.
- `check_rules` self-test passes; negative tests fire (an unregistered `app_program/subsystem6/sources.cmake`, `tm_printf` / `sleep_ms` / `pio_sm_get_blocking` in `app_program/`, `src/foo/x.c`), and a violation inside `ProjectTemplate/` is **not** reported. Delete temporary files afterwards.
- A Release configure shows `DBG_ENABLE=OFF`; Release + `-DAPP_TEST=ON` fails; Debug + `APP_TEST=ON` configures.
- `git -C external/mtk3_bsp status --porcelain` is empty and the submodule is still at `15ed232...`.
- `git status` shows no changes in `ProjectTemplate/`, `INF 2004 Embedded Systems.md`, `design.md`, `specs.md`.
- Write a summary (files changed, assumptions, open items, owner questions with a suggested answer each, and the full command/result table) to `.agents/tasks/merge-week8-summary.md`.

Then **stop and wait for the owner's approval before Batch B.**

## 5. Rules that must not be broken

- Never commit without the owner deciding to. Never `git rm --cached`, `reset --hard`, `clean`, or force-push.
- Never edit: `ProjectTemplate/**`, `external/mtk3_bsp/**`, `INF 2004 Embedded Systems.md`, `design.md`, `specs.md`. Do not edit `README.md` or `.gitignore` again without showing the diff to the owner first (same-name files).
- `NOTICE.md` contains an owner edit: preserve it; only append.
- Names and student IDs come **only** from the Team Information table in `INF 2004 Embedded Systems.md` (lines 17-28), copied exactly: Buddy 1 `2501299 (Damian)`, Buddy 2 `2500530 (Ruo Chen)`, Buddy 3 `2503319 (Jin Yong)`, Buddy 4 `2500709 (Vincent)`, Buddy 5 `2500596 (Chay Han)`. GitHub usernames stay `TODO(owner-to-confirm)`. (`ProjectTemplate/TEAM.md` is still blank.)
- A `.c` file includes kernel headers (`<tk/...>`, `<tm/...>`, `<sys/...>`) **or** SDK headers (`pico/`, `hardware/`), never both.
- No blocking SDK calls from tasks (`sleep_ms`, `mutex_enter_blocking`, `sem_acquire*`, `pio_sm_get_blocking`); use `tk_dly_tsk`. Only the debug drain task calls stdio; `tm_printf` only for boot/fault in the port. Every ISR goes through the SDK IRQ API with the `knl_taskindp` wrapper. One owner per peripheral. Core 1 unused.
- `-Wall -Wextra -Werror` on our code only, never on mtk3 or the SDK.
- No C bit-fields anywhere (masks and shifts only).
- Template files are UTF-8 with non-ASCII characters: copy them with `Copy-Item` and edit with an editor; never round-trip them through `Get-Content | Set-Content`.
- Keep every `TODO(owner-to-confirm)` marker and open item.

## 6. Owner decisions already made (binding)

Full text: the "OWNER CORRECTIONS" section at the top of `.agents/tasks/week8-impact-report.md`, plus the owner's replies recorded in the plan's sections 0-2. Key points:

- Template merge Option C (ours as base; adopt `app_program/subsystemN/`, `TEAM.md`, `testing/`, `videos/`). No make shim; README states our CMake/Ninja flow, with a TODO for the instructor's build preference. TEAM.md lists our priorities 1-10 (11 with `APP_TEST`). Template credit: "INF2004 course project template provided by the module instructor", author TODO(owner-to-confirm). Template WiFi code is reference only.
- Servo on **GP15** (confirmed). Encoders via **PIO0**, one state machine per wheel (delta D7); keep an `enc_get_count()` API. NFR2 wording: **do not edit team documents**; record the proposed wording in D7 as `TODO(owner-to-confirm, team)`, noting that "zero missed counts" needs a top-speed test.
- Pitch signed ±45° at 0.5° (8 bits). `motion_cmd_t` = type 3 + speed 6 + shared 9-bit arg (distance / angle / signed steer) + OOR = 19 bits. `obstacle_profile_t` adds left/right clearance (7 bits each), drops the timestamp, removes "bypass state" (a 3-bit bypass command goes in `nav_input_t`, flag Buddy 5), and **keeps the optional 2-bit sequence number** until the owner decides. `TMO_POL` send fails on a full buffer (verified, `messagebuf.c:315-321`), so newest-wins needs drain-then-send: document it.
- System FSM: 4 bits flat; recount the states in design.md section 5 at full resolution, and use 3 bits if there are 8 or fewer.
- Faults: extend EVF_SAFETY; Safety waits on `COLLISION|IMU_ALIVE` with `TWF_ORW|TWF_BITCLR`, never `TWF_CLR`; FAULT_LATCHED covers collisions and IMU watchdog trips; "no echo" is informational only, not a fault; VC/Motion use `tk_ref_flg`.
- RAM fault record: in the SDK's `.uninitialized_data` (NOLOAD, not zeroed; **not** `.noinit`), with a **magic number and a CRC**. It is printed and cleared at boot only when both match. Implemented in Batch B.
- **D2 (20 ms telemetry wake) is NOT approved**: it stays `TODO(owner-to-confirm, buddy 1)` everywhere.
- `BC_CAPTURE_TIMEOUT_MS = 1000` stays TUNE with a static assert, TODO(owner-to-confirm, buddy 3). `ENC_CPR` and `WHEEL_CIRC_MM` are symbolic TODOs (buddy 2). Dashboard language is undecided: write a language-neutral field table only.
- Week 13 note: bit-packing barely changes RAM, because message-buffer slots round up to whole words and live inside the fixed kernel pool. Report only what the map file shows.
- Batch B will be split into B-baseline (unpacked, measured, tagged `baseline/week6-unpacked`) and B-packed (shared header `src/common/data_repr.h` + host tests).

## 7. Reports in `.agents/tasks/` (now tracked in git)

| File | What it is |
|---|---|
| `mtk3-rp2040-integration-report.md` | Kernel integration analysis (approach (a), port layer, interrupts, tick, message-buffer formula, USB CDC test procedure) |
| `batchA-plan.md`, `batchA-summary.md`, `batchA-review.md/.json` | Batch A plan, evidence (exact build commands and results) and review |
| `template-merge-report.md` | Template inventory, conflict table, Option C merge plan |
| `step0-verify.md` | Step 0 evidence (check_rules unblocked) and handoff notes |
| `merge-samename-diffs.md` | Approved README.md / .gitignore diffs |
| `week8-impact-report.md` | Week 8 impact analysis (pins, quadrature, data representation, deltas) with the binding OWNER CORRECTIONS at the top |
| `merge-week8-brief.md` | **The original task brief** (all approved decisions for Steps 2-3 and the final verification) |
| `merge-week8-plan.md` | **The implementation plan for everything still to do** (M-, MV-, W-, WV-, FV- items) |

## 8. Open owner questions (with suggested answers)

| # | Question | Suggested answer |
|---|---|---|
| 1 | GitHub usernames for TEAM.md | Each buddy adds their own; leave TODO until then |
| 2 | `ProjectTemplate/TEAM.md` was said to be filled in, but the committed copy is blank | Save and commit the filled-in copy if it exists; otherwise the Team Information table is used |
| 3 | Instructor's build preference (make/WSL vs our CMake/Ninja) | Ask the instructor before the Week 10 demo; keep the README TODO until then |
| 4 | D2: 20 ms telemetry wake | Buddy 1 to decide; FR1 rates are met either way |
| 5 | Keep the optional 2-bit sequence number in `obstacle_profile_t`? | Buddy 5 to decide after the drain-then-send analysis |
| 6 | NFR2 proposed wording (D7) | Team to approve, after a top-speed zero-missed-counts test |
| 7 | `ENC_CPR`, `WHEEL_CIRC_MM`, `CAR_WIDTH_MM`, IMU I2C addresses, servo pulse range | Measure on the car (encoder spec / bus scan / ruler) |
| 8 | Barr C: allow bit-fields for private state later? | Buddy 1 checks the standard; masks and shifts until then |
| 9 | Batch B approval | After the final verification summary is reviewed |
