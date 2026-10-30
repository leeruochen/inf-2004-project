# Batch B plan: platform (boot, kernel port, debug console, IPC objects)

Status: B0 done (docs only, 29-30 Oct 2026), waiting for Buddy 2's review and official gate run. BM deferred (Rev 5). No code is implemented.
Written 29 Oct 2026 for the Batch B helper (Buddy 3, Jin Yong), with Buddy 2's go-ahead.
Rev 2 (29 Oct 2026, after review):
- the stack model is decided before B2 (O8);
- O11 is no longer a blocker;
- the boot buffer became a continuously drained system-message ring (C3/O5);
- milestones M1/M2 were added (§3).

Rev 4 (29 Oct 2026; **toolchain move WITHDRAWN in Rev 5**, kept for history):
- Buddy 2 moved the project to **Pico SDK 2.3.1 + Arm GNU GCC 10.3.1 (arm-none-eabi)**.
- New stage BM (SDK/toolchain migration) between B0 and B1.
- The linker script is rewritten as 2.3.1 override files in B3 and checked in B4.
- O11 is resolved.

Rev 5 (30 Oct 2026, helper's proposal; Buddy 2 confirms in the B0 review):
- **Workflow (C7):** the helper writes each stage and pushes its branch; Buddy 2 (Ruo Chen) pulls it and runs the official gates in the team's verified environment (HANDOVER §5); from B4 on he also flashes the Pico W and runs the hardware/USB tests. A stage passes only after his verification is recorded (§2). The helper's local builds are feedback only.
- **Toolchain stays (C8):** Pico SDK 1.5.1 installer + GCC 10.3.1, as in HANDOVER §5 and the committed repo (`CMakeLists.txt:1`, `cmake/memmap_mtk3.ld:1`, `cmake/warnings.cmake:10`, README, NOTICE). The move to 2.3.1 came only from the helper's local setup, so it is not needed. Stage BM is deferred (§3).
- B3 needs no linker work: `cmake/memmap_mtk3.ld` (Batch A, 1.5.1 copy) already has `.data_vector (NOLOAD)` and `.noinit` inside the zero-filled `__bss_start__..__bss_end__` range, with `ASSERT`s (`memmap_mtk3.ld:129-136, 213-231, 274-275`). B4 checks it at the first link.
- Rev 4 called the 2.3.1 move an owner decision. **If Buddy 2 has already moved his PC to 2.3.1, he says so in the B0 review, and BM comes back** before B1.

## 0. Baseline and confirmed decisions

**Official toolchain (Rev 5, C8):** Pico SDK **1.5.1** Windows installer, GCC **10.3.1** (HANDOVER §5), on Buddy 2's PC. Batch B does not change it. Every authoritative gate and every measurement (B6 sizes depend on the compiler) comes from this environment.

**Helper's local feedback environment** (helper's PC; SDK 2.3.1 + GCC 15.2.Rel1 + CMake 4.3.4 + Ninja 1.13.2 from the VS Code extension). Its results are **feedback only, never acceptance evidence**. Trial on a throwaway copy of `8d083ca`, 29 Oct 2026:
- configure OK (`mtk3 ... pinned ... clean`, `firmware target skipped`);
- build 97/97: our code 0 warnings, `check_rules: PASS (42 files scanned)`, `check_rules_selftest: PASS`; Release builds; `APP_TEST=ON` stops only at the deliberate `test_report.h:35` `#error` (same as on 1.5.1);
- one warning, in vendored `lib/libtm/libtm_printf.c:50` (`-Wunterminated-string-initialization`, a GCC 15-only check; not expected with 10.3.1);
- checked unchanged in the 2.3.1 source:
  - RAM vector table installed at start-up (`pico_runtime_init/runtime_init.c:224`);
  - crt0 ISR names `isr_hardfault`/`isr_pendsv`/`isr_systick` (`pico_crt0/crt0.S:349-352`);
  - `__uninitialized_ram` (`pico/platform/sections.h:183`);
  - USB stdio low-priority IRQ + alarm (`pico_stdio_usb/stdio_usb.c:62-97`);
  - `irq_set_exclusive_handler` and `gpio_add_raw_irq_handler_masked`;
  - SDK mutex owner = core number (`pico/lock_core.h:112`);
  - the SDK runtime never writes SHPR.
- differs: 2.3.1 builds its linker script from include files with override hooks (`rp2040/pico_platform/memmap_default.ld`), while our `cmake/memmap_mtk3.ld` is a 1.5.1 copy. So from B4 on, a local link on 2.3.1 may fail or produce a different layout. That is a local limitation, not a defect; only the official 1.5.1 link counts.
- GCC 15 adds warnings in vendored `libtm_printf.c` (B0 result). They are not in the official Debug baseline (0 warnings); Buddy 2's official Release run shows whether GCC 10.3.1 gives any.

**Verified baseline = the official environment** (Buddy 2's device, 29 Oct 2026):
- `8d083ca` = `main` = `origin/main`; working tree clean except an untracked `session.md`.
- Submodule ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp`.
- SDK 1.5.1, GCC 10.3.1.
- Fresh `build/fresh`: 84/84 steps, no warnings, `check_rules: PASS (42 files scanned)`, `check_rules_selftest: PASS`, `firmware target skipped`.
- Pico W board and cable: BOOTSEL shows `RPI-RP2`.

**Confirmed decisions:**

| # | Decision | Source |
|---|---|---|
| C1 | Buddy 2 approves Batch B work by the helper | owner, verbal; recorded in B0 (HANDOVER §0, §7) |
| C2 | Each member writes the tests for their own subsystem (`subsystemN_test.c`) | team |
| C3 | The kernel's own boot text (`tm_printf` banner, system messages) is kept **separate** from the RAM fault record. It goes into a system-message ring that the drain empties every cycle, so no `tm_*` output is lost after boot either. The fault record holds faults only. | owner, this session |
| C4 | Two parts: B-baseline (Week 6 unpacked types, measured, tagged) then B-packed | HANDOVER §4 |
| C5 | Approach (a): the SDK owns startup; the kernel is the `mtk3` library behind `src/rtos/port/` | ARCHITECTURE.md |
| C6 | `ipc_init()` creates every IPC object centrally, before any `subsystemN_init()` (T5) | ARCHITECTURE.md T5 |
| C7 | The helper writes and pushes each stage; Buddy 2 runs the official gates (and from B4 the flashing and hardware/USB tests). A stage passes only after his verification. The helper's local builds are feedback only | helper, Rev 5; Buddy 2 confirms in the B0 review |
| C8 | The toolchain stays Pico SDK 1.5.1 + GCC 10.3.1 (HANDOVER §5); BM deferred | helper, Rev 5; Buddy 2 confirms in the B0 review |

**Binding rules** (HANDOVER §6, ARCHITECTURE.md "Rules enforced by the build"), enforced in every stage:
- R1: one file includes kernel headers or SDK headers, never both.
- R2: no blocking SDK waits in tasks; only `src/app/main.c` lines marked `RULE-ALLOW: pre-kernel`.
- R3a/R3b: stdio only in `src/debug/dbg_drain.c` (and marked `main.c` lines); `tm_*` only in `port_boot.c`/`port_fault.c`.
- R4: no `libbsp_pwm`. R5: no core 1.
- `-Wall -Wextra -Werror` on our code. No C bit-fields.
- Never edit `ProjectTemplate/**`, `external/mtk3_bsp/**`, `design.md`, `specs.md`, `INF 2004 Embedded Systems.md`.
- Never edit another member's `app_program/subsystemN/`.
- Shared files (`app.h`, `app_main.c`, `test_report.h`, `CMakeLists.txt`, `config/*.h`, `check_rules.cmake`) change only with the team told (say so in the PR).

**Layout facts the plan depends on** (checked in the code):
- `src/rtos/port/sources.cmake` is NOT allowed (configure fails: only `src/rtos` is in `ROBOCAR_MODULE_DIRS`). Register the port files from `src/rtos/sources.cmake`.
- **The firmware switch.** Creating `src/app/main.c` switches CMake from compile-check to full link (`CMakeLists.txt:150`). It is created in B4 only, when everything it needs exists.
- Before B4, every registered `src/` source is compiled by `app_program_check` (kernel side, kernel include paths only) or `app_program_check_sdk` (SDK side, `hardware_*` include paths only). `pico_stdlib`/`pico_stdio` headers are NOT on that path, so `main.c` cannot be compile-checked early.
- `mtk3` leaves 8 symbols for the port to define: `knl_lowmem_top`, `knl_lowmem_limit`, `knl_shutdown_hw`, `knl_restart_hw`, `tm_com_init`, `tm_snd_dat`, `tm_rcv_dat`, `usermain` (`cmake/mtk3.cmake:15-18`). Kernel prototypes:
  - `void knl_shutdown_hw(void)` and `ER knl_restart_hw(W mode)` (`kernel/knlinc/kernel.h:215-216`);
  - `void tm_snd_dat(const UB*, INT)` (`lib/libtm/libtm.h:25-28`);
  - `W knl_taskindp` (`armv6m/cpu_cntl.c:30`);
  - `INT main(void)` in `sysinit.c:26`, renamed to `mtk3_kernel_main`.
- The kernel ISR entry only does `knl_taskindp++` / `--` (`armv6m/cpu_status.h:71-78`). A kernel call from an ISR pends PendSV through `END_CRITICAL_SECTION` (`cpu_status.h:30-35`), so the wrapper needs nothing more.
- `knl_init_interrupt()` does not touch VTOR. Only the excluded `reset_hdl.c:61` does, so the SDK RAM vector table stays active.
- The kernel banner is printed by `tm_printf` in `inittask.c:119` on every boot (this is why C3 exists). It is the only reachable `tm_*` caller among the compiled kernel sources today. The other callers are the kernel fault handlers in `exc_hdr.c:28,56,83`, which are never installed. Our `port_boot.c`/`port_fault.c` may call `tm_printf` too (R3b), so `tm_snd_dat` can run after boot.
- `TA_USERBUF`: the caller's buffer is the whole stack (`stksz >= MIN_SYS_STACK_SIZE` = 128); no Imalloc, and no +128 B `DEFAULT_SYS_STKSZ` (`task_manage.c:55-60`). The kernel init task stack (1 KB) is always kernel-allocated.
- Kernel `HardFault_Handler` (`exc_hdr.c:45`) is never installed, because the SDK vector table uses `isr_hardfault`. The port must provide the HardFault handler.

## 1. Open decisions (flagged, not guessed)

**Rev 3 (29 Oct 2026):**
- **Accepted as proposed:** O1, O2, O4, O7, O8, O9 (the helper answered for the team; tell Buddy 2 in the B0 PR).
- **Debug Probe:** the team has one, so SWD debugging is available from B4.
- **Still open:** O3 (default: out of scope), O5/O6 (defaults apply unless changed in review), O10 (B7), the B6 init-task measurement method. (SDK version: Rev 5, stays 1.5.1 + GCC 10.3.1, C8.)

Each has a proposed default and the stage it blocks. **Bold** = must be answered before that stage starts.

| # | Question | Proposed default | Blocks |
|---|---|---|---|
| **O1** | Acceptance needs "nine heartbeat lines" (DEBUG.md). Members write their own tests (C2), but they may not be ready. Who provides the 9 heartbeat tasks? | Temporary platform bring-up tasks in `src/debug/dbg_bringup.c`, built only with a new CMake option `PLATFORM_BRINGUP=ON` (default OFF), at the design priorities and periods. Deleted once the members' real tasks exist. Needs a `CMakeLists.txt` change (shared). | **B5** |
| **O2** | The Week 6 unpacked C types (`nav_input_t`, `motion_cmd_t`, `obstacle_profile_t`, `tel_evt_t`, `motion_state_t`, edge ring) exist nowhere as C. Only field lists and estimated sizes exist (DATA_REPRESENTATION.md §2: 4, 10, 16, 16, 20 B; ring 64 × 8 B). Who writes the struct definitions, and where? | Platform writes them once in `src/common/ipc_types.h` (pure, `<stdint.h>` only), each field from DATA_REPRESENTATION §2, every struct marked `TODO(owner-to-confirm, buddy N)` for its IF owner | **B2** |
| O3 | `drv_encoder.c` is tagged `TODO(phase 1 batch B)` (`drv_encoder.h:9`), but HANDOVER §3 lists it as Buddy 2's subsystem work | Out of Batch B; Buddy 2 writes it in `subsystem2/` | none |
| **O4** | Fault record layout: magic value, payload words, CRC variant | magic `0x464C5452` ("FLTR"); payload = version, fault type, PC, LR, xPSR, kernel error code, boot count (7 words); CRC-32 IEEE (reflected, poly `0xEDB88320`, init/xorout `0xFFFFFFFF`) over the payload | **B3** |
| O5 | System-message ring (C3): size and overflow policy | 512 B `.bss` ring, single writer (`tm_snd_dat`) and single reader (the drain); 32-bit head/tail indices, no lock; `tm_snd_dat` never blocks; on full, drop new bytes and count them; the drain empties it every cycle. Caveat: text written during a HardFault dies with the reset, so the fault record stays the authority for crashes. | B3 |
| O6 | `dbg_` ring: size, max line, overflow policy, ISR use | 2 KB ring, 96 B max line, drop the newest line and count it; tasks only (no ISR logging in Phase 1); every call compiles to nothing when `DBG_ENABLE=0` | B1 |
| O7 | After a HardFault: reset or halt? | Record, then reset through AIRCR `SYSRESETREQ`, so the next boot prints the record. Halt instead (`bkpt`) if a debugger is attached (`DHCSR.C_DEBUGEN`). | B3 |
| **O8** | Stack allocation model (decided BEFORE B2, so B4/B5/B6 all run the same layout). Stack painting needs known stack addresses. Kernel-allocated stacks have no public address (`USE_DBGSPT 0`). | (a) `app_start_task()` uses `TA_USERBUF` with stacks cut from one static arena (8-byte aligned, painted at creation). `priorities.h` takes the task stacks out of `KERNEL_POOL_BYTES`, and stack sizes lose the +128 B `KERNEL_SYS_STACK_BYTES` term (shared file). Or (b) keep kernel allocation and read the TCB through kernel-internal headers (new include path, more coupling). Recommend (a). Still open in either case: how to measure the kernel-allocated init task stack (B6). | **B2** |
| O9 | `ipc.h` (kernel object IDs) includes `<tk/tkernel.h>` but is not in the R1 project-header regex, so a `drv_*.c` including it would slip past R1 | Add `ipc` (and `dbg`/`hal` if they ever include kernel headers) to `_RE_KINC_PROJ` in `check_rules.cmake`, plus a fixture in `check_rules_selftest.cmake` (shared) | B2 |
| O10 | Host compiler for host tests (B7): the SDK installer has none | Use the MSYS2 GCC already on Buddy 2's PC (`C:/msys64`), or install one; record the path in HANDOVER §5 | B7 |
| O11 | Helper's own build environment | RESOLVED (Rev 5): the helper keeps SDK 2.3.1 + GCC 15.2 for feedback only, with no install needed. Optional, for closer feedback: install the SDK 1.5.1 Windows installer too. The Pico W is with Buddy 2, so he flashes and runs every hardware test from B4 on (C7). | none |

## 2. Gate commands (run at the end of every stage)

**Who runs what (C7):**
1. The helper finishes the stage on `batchB/bN-<name>`, runs the gates locally as feedback, and pushes the branch. The PR (or the stage's result block below) gives the commit hash and the local results, labelled "local feedback, SDK 2.3.1 + GCC 15.2".
2. Buddy 2 pulls that exact commit and runs the gates below in the official environment (HANDOVER §5, SDK 1.5.1 + GCC 10.3.1), in fresh build directories. From B4 on he also flashes the Pico W and runs the stage's hardware/USB checks.
3. He posts the result on the PR: commit hash, environment, each gate's outcome, and the warning count; for hardware checks, the console log, which goes into `testing/system/logs/` (B5 on). The helper copies the summary into the stage's result block and cites it.
4. **A stage passes only when Buddy 2's verification is recorded.** Only then is it merged, and only then does the next stage start from it. A local pass that fails officially means the stage has failed.
5. If his run fails, he posts the log; the helper fixes it on the same branch, and the stage goes back to step 1.

Run in PowerShell from the repo root, after the HANDOVER §5 step 3 environment block. Use a **fresh** build directory per stage (`build/bN`), never an old one: the `[1/1]` incremental build proved nothing.

```powershell
& $CM -S . -B build/bN @common -DDBG_BACKEND=USB
& $CM --build build/bN                                            # 0 warnings, check_rules: PASS
& $CM -P cmake/check_rules_selftest.cmake                         # PASS
& $CM -S . -B build/bN-test @common -DDBG_BACKEND=USB -DAPP_TEST=ON
& $CM --build build/bN-test                                       # from B1 on: must build
& $CM -S . -B build/bN-rel @common -DCMAKE_BUILD_TYPE=Release     # DBG_ENABLE=OFF
& $CM --build build/bN-rel                                        # must build
git status --short                                                # only intended files
```

**General stop condition, every stage:** any warning, any `check_rules` violation, a self-test FAIL, a change to a forbidden file, or a need to touch another member's folder. Stop, report, do not work around it.

**One branch and one PR per stage** (`batchB/bN-<name>`), opened to `main`. The helper commits only their own work; Buddy 2 commits his own logs. The PR states which shared files changed. After each stage passes Buddy 2's verification, update HANDOVER §1 and §4.

---

## 3. Stages

**Milestones for the other members:**

| Milestone | After | What members can rely on | Still changing |
|---|---|---|---|
| **M1: platform works** | B5 | flash and run their own tasks; object IDs (`ipc.h`), `app_start_task()`, `dbg_`, `hal_*`, `APP_TEST` builds | message types in `ipc_types.h` (PROVISIONAL), EVF_SAFETY bit positions |
| **M2: messages final** | B7 | packed message types (`data_repr.h`), EVF_SAFETY bits, host tests | nothing in Batch B |

Between M1 and M2, member code that reads or writes message fields may need edits. HANDOVER §1/§3 must say so at M1.

### B0: records and prerequisites (docs only)

- **Goal:** the repo matches reality before any code changes.
- **Files:**
  - `HANDOVER.md`: §1 commit status is stale; record the C1 go-ahead and decisions C2/C3; update §4 to point at this plan.
  - `ARCHITECTURE.md`: D6 gets the separate system-message ring (C3).
  - `docs/DEBUG.md`: the banner line now says the kernel banner arrives through the system-message ring, not the `dbg_` ring.
  - this plan.
- **Depends on:** nothing.
- **Work:**
  1. Fix the stale lines above.
  2. Do NOT edit evidence reports in `.agents/tasks/`. Instead list their stale parts here, in §5.
  3. Helper clones the repo on Windows.
  4. Get answers to O1, O2, O4, O8, and ideally O9.
- **Verify:** gate commands (no code change, so expect the same result as the baseline). Diff is docs only. Official run: Buddy 2 (C7).
- **Stop if:** Buddy 2 disagrees with any recorded decision.
- **B0 result (29-30 Oct 2026, helper's PC, branch `batchB/b0-records`, not committed). B0 has NOT passed yet: Buddy 2's official run is pending (C7).**
  - Files changed: `HANDOVER.md` (§0 item 3-4, §1 Batch B row + commit status + M1/M2 note, §3 Buddy 2/3 rows, §4 rewritten around this plan with a stage table and the C3 `tm_snd_dat` fix, §7 D6 bullet + Batch B decisions C1-C8/O answers, §9 index row), `ARCHITECTURE.md` D6 (C3), `docs/DEBUG.md` (section split into system-message ring + fault record; bring-up test step 1 banner line), this plan (status, C1 source, §5 additions; Rev 5: workflow C7, toolchain C8, BM deferred, B3 linker row, §2 roles, B4-B6 roles). No code, no CMake, no forbidden file, no other member's folder.
  - Work 3: the helper's clone is `D:\Github\inf-2004-project` (Windows drive, edited from WSL). Windows git rejects the WSL-created submodule ("dubious ownership"), so `mtk3.cmake`'s pin check failed at first. Gates were run with a session-only `GIT_CONFIG_COUNT=1 / safe.directory=*` (no global git config change). `git status` from WSL shows ` m external/mtk3_bsp`: CRLF-only differences (`git diff --ignore-cr-at-eol` is empty); Windows git reports it `clean`. Pre-existing, not touched.
  - Work 4: O1, O2, O4, O7, O8, O9 accepted (Rev 3); owner approved the plan's decisions (29 Oct 2026). Recorded in HANDOVER §7.
  - **Local feedback** (not acceptance evidence, C7), SDK 2.3.1 + GCC 15.2.Rel1 + CMake 4.3.4 + Ninja 1.13.2, fresh `build/b0`, `build/b0-test`, `build/b0-rel`:

    | Gate | Result |
    |---|---|
    | configure `build/b0` | OK: `mtk3 ... pinned at 15ed232..., clean`, `firmware target skipped, app_program compile-checked` |
    | build `build/b0` | exit 0, 97/97, `check_rules: PASS (42 files scanned)`; 0 warnings in our code; 1 vendored warning `libtm_printf.c:50` `-Wunterminated-string-initialization` (as in the Rev 4 trial) |
    | `check_rules_selftest` | PASS |
    | `build/b0-test` (`APP_TEST=ON`) | stops only at the deliberate `test_report.h:35` `#error`, all five `subsystemN_test.c` (expected until B1) |
    | `build/b0-rel` (Release) | exit 0, 97/97, `check_rules: PASS`; 0 warnings in our code; vendored warnings: `libtm_printf.c:50` once and **`libtm_printf.c:277` `-Warray-bounds=` ×17 (new: not in the Rev 4 trial notes)** |

  - The vendored warnings come from GCC 15 checks and do not appear in the official Debug baseline. If Buddy 2's official Release run shows `libtm_printf.c:277` with GCC 10.3.1 too, the fix is a per-file warning option for `mtk3` in our CMake (never a vendored edit), with his agreement.
  - **Official run (Buddy 2, pending):** the §2 gate commands on the pushed `batchB/b0-records` commit, SDK 1.5.1 + GCC 10.3.1. Expected: the same as the verified baseline (Debug 84/84, no warnings, `check_rules: PASS (42 files scanned)`, self-test PASS); `APP_TEST=ON` stops at `test_report.h:35`; Release builds. Plus his answers to C7/C8 (and whether he has already moved to 2.3.1).

### BM: SDK/toolchain migration: DEFERRED (Rev 5)

- **Not done.** The project stays on Pico SDK 1.5.1 + GCC 10.3.1 (C8). Buddy 2 runs every official gate there (C7), so the team never needs the helper's local toolchain.
- **Reinstated only if** Buddy 2 has already moved to 2.3.1, or the team later chooses to move (for example for the template's 2.2.0/2.3.0 line, ARCHITECTURE.md T2). It then becomes its own stage before the next unstarted one, and it would cover:
  - a version guard in `CMakeLists.txt`;
  - the 2.3.1 `pico_sdk_import.cmake`;
  - the `warnings.cmake` comment;
  - the install/path blocks in README (team sees the diff first), HANDOVER §2/§5 and DEBUG.md;
  - ARCHITECTURE T2 and a NOTICE.md line;
  - replacing `cmake/memmap_mtk3.ld` with 2.3.1 linker override files (`.data_vector`, `.noinit` zero-filled);
  - re-checking pico-examples `quadrature_encoder.pio` against the `sdk-1.5.1` tag credited in NOTICE.md.
- First gate if reinstated: Buddy 2 builds the unchanged tree on the new pairing. Nobody patches SDK or kernel files to make it fit.

### B1: `dbg_` ring API + shared-file hooks

- **Goal:** tasks can log without stdio, and `APP_TEST=ON` compiles.
- **Files (new):**
  - `src/debug/sources.cmake`;
  - `src/debug/dbg.h`: pure (`<stdint.h>`, `<stdarg.h>`). API: `dbg_puts(const char *)`, `dbg_printf(const char *, ...)`, `dbg_put_err(const char *what, int32_t er)`, `dbg_ring_read(...)`, `dbg_overflow_count(void)`. Macro no-ops when `DBG_ENABLE=0`.
  - `src/debug/dbg_ring.c`: kernel side. A short critical section with `DI`/`EI` from `<tk/tkernel.h>` (`armv6m/syslib.h:43-45`); `vsnprintf` into a caller-stack buffer (allowed: `snprintf` is not an R3a call, see `check_rules_selftest.cmake:42`).
- **Files (changed, shared):**
  - `app_program/app.h`: `APP_CHECK` calls `dbg_put_err(p_what, er)`, with no formatting, because it runs on the 1 KB init stack.
  - `app_program/test_report.h`: remove the `#error`; `TEST_CHECK`/`TEST_SUMMARY` use `dbg_printf` with the fixed `[TEST]` formats already documented in the file.
- **Depends on:** B0 passed (Buddy 2's official run), O6.
- **Work:**
  1. Ring of O6 size, one writer at a time under `DI`/`EI`, line-granular, drop-newest plus counter.
  2. The drain (reader) is NOT written yet.
- **Verify:**
  - gate commands; `APP_TEST=ON` now builds (before: stopped at `test_report.h:35`);
  - Release build: `dbg_` calls compile to nothing;
  - `check_rules` PASS: `dbg_ring.c` has kernel headers only and no stdio calls.
- **Stop if:**
  - `vsnprintf` pulls in something R3a flags;
  - the `APP_TEST` build needs members' code to change.

### B2: IPC objects + task start (`src/common`)

- **Goal:** `usermain()` creates every design.md 3.3 object at its design depth and can start tasks.
- **Files (new):**
  - `src/common/sources.cmake`;
  - `src/common/ipc_types.h`: Week 6 unpacked types (O2); header comment "PROVISIONAL until M2 (B7)";
  - `src/common/mbf_size.h`: pure; `MBF_BUFSZ(N, T) = N * (4 + ((sizeof(T) + 3) & ~3))` plus `_Static_assert`s (integration report §9);
  - `src/common/ipc.h`: kernel side; `extern ID` for each object, `ipc_init()` prototype;
  - `src/common/ipc.c`.
- **Files (changed, shared):**
  - `app_program/app_main.c`: `app_start_task()` = `T_CTSK{TA_HLNG|TA_RNG0|TA_USERBUF (per O8), itskpri, stksz, bufptr}` + `tk_cre_tsk` + `tk_sta_tsk`, failures through `APP_CHECK`; step 1 calls `ipc_init()`.
  - `config/priorities.h` (per O8): task stacks leave `KERNEL_POOL_BYTES`; drop the `KERNEL_SYS_STACK_BYTES` term for `TA_USERBUF` tasks; keep the init task in the pool.
  - stack arena: in `src/common/` (kernel side) or `app_main.c`, wherever O8 puts it.
  - `cmake/check_rules.cmake` + `cmake/check_rules_selftest.cmake` + one new fixture, if O9 is accepted.
- **Depends on:** B1 (`APP_CHECK` logs), O2, **O8**, O9.
- **Work:**
  1. Create the 8 objects:

     | Object | Created as |
     |---|---|
     | `MBF_NAV_IN` | 8 × `nav_input_t` |
     | `MBF_MOTION_CMD` | 4 × `motion_cmd_t` |
     | `MBF_OBSTACLE` | 2 × `obstacle_profile_t` |
     | `MBF_TELEMETRY` | 32 × `tel_evt_t` |
     | `SEM_ECHO` | count 0 |
     | `EVF_SAFETY` | `TA_WSGL`, initial 0 |
     | `EVF_NAV` | initial 0 |
     | `MTX_MOTION` | `TA_INHERIT` |

  2. Depth self-check per message buffer, at init (integration report §9):
     - `tk_ref_mbf`: `frbufsz == bufsz`;
     - send N dummies with `TMO_POL` → all `E_OK`;
     - send N+1 → `E_TMOUT`;
     - drain.

     Keep it stack-light: one local of the largest type, no more than 20 B.
  3. Kernel version self-test: `tk_ref_ver`, `spver == TK_SPECVER` (`0x0300`, `include/sys/profile.h:47`), logged.
  4. EVF_SAFETY bit positions stay as proposed (fixed in B7).
  5. Stack arena (O8): one static, 8-byte-aligned array sized from `TASK_STACK_*_BYTES`. `app_start_task()` cuts each stack from it and paints it with a fixed pattern before `tk_cre_tsk`; it fails with `E_NOMEM` if the arena runs out. Painting happens here; reporting is in B6.
- **Verify:**
  - gate commands (compile only: nothing runs until B4);
  - the `_Static_assert`s hold;
  - the `priorities.h` pool `#error` check still passes with the new budget;
  - every subsystem still compiles untouched.
- **Stop if:**
  - an IF owner disputes a type;
  - the self-check grows `usermain`'s stack use past about 256 B.

### B3: kernel port layer (`src/rtos/port`)

- **Goal:** every symbol `mtk3` needs is defined, and the SDK and kernel are glued without breaking R1.
- **Files (new):**

  | File | Side | Contents |
  |---|---|---|
  | `src/rtos/sources.cmake` | – | registers the four port `.c` files |
  | `src/rtos/port/hal.h` | pure (`<stdint.h>`) | `hal_kernel_start(void)` (no return); `hal_irq_register(irq, fn, level)`; `hal_gpio_irq_register(gpio, events, fn)`; `hal_isr_wake/hal_isr_sig/hal_isr_setflg` (IDs as `int32_t`); `hal_time_us(void)` (for the B5 jitter test); fault-record and system-message-ring accessors for the drain |
  | `port_kernel_internal.c` | kernel | the 96 KB pool (`KERNEL_POOL_BYTES`, 8-byte aligned); defines `knl_lowmem_top/limit`; `knl_shutdown_hw` (`disint` + spin); `knl_restart_hw` (return `E_NOSPT`); `port_isr_enter/leave` (`knl_taskindp++/--`, `extern W knl_taskindp`); `hal_isr_*` → `tk_wup_tsk`/`tk_sig_sem`/`tk_set_flg` |
  | `port_boot.c` | SDK | `hal_kernel_start()`: set pool bounds; SHPR2 = SVC 0, SHPR3 = SysTick 0, PendSV `0xC0` (as `reset_hdl.c:100-102`); write `((uint32_t *)scb_hw->vtor)[14] = knl_dispatch_entry`, `[15] = knl_systim_inthdr`; call `mtk3_kernel_main()`. Also `tm_com_init`/`tm_snd_dat` (→ system-message ring, C3/O5; never blocks, callable from any context)/`tm_rcv_dat` (returns nothing). Kernel functions declared as bare `extern` prototypes, no kernel headers. |
  | linker script | – | **No change (Rev 5).** `cmake/memmap_mtk3.ld` (1.5.1 copy, Batch A) already places the kernel's `exchdr_tbl` in `.data_vector (NOLOAD)` and `.noinit` (kernel control blocks, which MUST be zeroed) inside crt0's zero-filled `__bss_start__..__bss_end__` range, with `ASSERT`s (`memmap_mtk3.ld:129-136, 213-231, 274-275`). It is first used at the B4 link and checked there. |
  | `port_fault.c` | SDK | fault record in `__uninitialized_ram()` (`.uninitialized_data`, `memmap_mtk3.ld:191`), CRC-32 (O4), `isr_hardfault` override (naked entry, reads the stacked PC/LR/xPSR), reset or halt per O7, boot-count update, accessors `valid/get/clear` |
  | `port_irq.c` | SDK | `hal_irq_register` → `irq_set_exclusive_handler` + `irq_set_priority(level << 6)` + `irq_set_enabled`, through a wrapper table (`port_isr_enter(); fn(); port_isr_leave();`). `hal_gpio_irq_register` → `gpio_set_irq_enabled` + `gpio_add_raw_irq_handler_masked` with the same wrapper, so `IO_IRQ_BANK0` stays on the SDK's shared chain (TEAM.md:145 dispatcher). Never `tk_def_int`. |

- **Depends on:** B2 (`ipc.h` IDs for `hal_isr_*`), O4, O5, O7.
- **Work:** as in the table.
  - Every `extern` prototype must match the kernel declaration exactly (types `W`=`int32_t`, `UB`, `INT`). Each one gets a comment citing its kernel file:line, so review can check them.
  - `tm_printf` is not called by our code. R3b allows it in `port_boot.c`/`port_fault.c` if needed.
- **Verify:**
  1. gate commands (still compile-only);
  2. symbol check: after the build, run `arm-none-eabi-nm` on the port `.obj` files under `build/b3/CMakeFiles/app_program_check*.dir/src/rtos/port/` and confirm all 7 port symbols are defined (`T`/`B`/`D`): `knl_lowmem_top`, `knl_lowmem_limit`, `knl_shutdown_hw`, `knl_restart_hw`, `tm_com_init`, `tm_snd_dat`, `tm_rcv_dat`. `usermain` is already in `app_main.c`.
  3. `check_rules` PASS, with no R1 hit on any port file.
- **Stop if:**
  - any port file needs both kernel and SDK headers (split it instead);
  - a needed kernel symbol is `static` or `Inline`, so it can't be linked.

### B4: SDK `main()` + debug drain = first firmware (first flash)

- **Goal:** the firmware links, boots on the Pico W and prints over USB CDC.
- **Files (new):**
  - `src/app/sources.cmake`;
  - `src/app/main.c`: SDK side. `stdio_init_all()` (USB only, as `CMakeLists.txt` sets); wait up to `DBG_USB_CONNECT_WAIT_MS` for `stdio_usb_connected()`, sleep lines marked `RULE-ALLOW: pre-kernel`; one pre-kernel `printf("pre-kernel ok")` marked the same way (a sign of life before the kernel exists); `hal_kernel_start()`.
  - `src/debug/dbg_drain.c`: kernel side + `<stdio.h>`. Task at `TASK_PRI_DBG_DRAIN`, period `DBG_DRAIN_PERIOD_MS` via `tk_dly_tsk`. At start: the fault record if valid (then clears it). Every cycle: empty the system-message ring (C3), then the `dbg_` ring lines; print both overflow counters when they change. `dbg_drain_start()`.
- **Files (changed, shared):** `app_program/app_main.c` calls `dbg_drain_start()` after `ipc_init()` when `DBG_ENABLE`.
- **Depends on:** B1, B2, B3 all merged. **This stage turns the firmware switch on.**
- **Work:** as listed.
  - **Buddy 2 builds the official `.uf2` and flashes it** (C7) by BOOTSEL: copy `build/b4/robocar.uf2` to `RPI-RP2`. The helper's local 2.3.1 link is feedback only, and may fail because the linker script is the 1.5.1 copy (§0).
  - Checks 2 and 4 need temporary debug-only code. The helper pushes it as a separate commit on the branch, Buddy 2 flashes and runs it, and the helper removes it before merge.
  - Open the COM port within 3 s (`docs/DEBUG.md`).
- **Verify (hardware, Buddy 2 on the Pico W; console logs posted on the PR):**
  1. `pre-kernel ok`, then the kernel banner (via the system-message ring), then `ipc_init` OK lines and the version self-test line.
  2. Late system message: a debug-only `tm_printf` from `port_boot.c` after the drain is running (removed before merge) appears on the console.
  3. No HardFault for 2 minutes; drain alive.
  4. Fault-record test, in a debug-only build with a temporary forced fault (a `PORT_FAULT_SELFTEST` define, removed before merge): fault → reset → record printed once with a correct PC; next reset → nothing printed; power-cycle → nothing printed (random RAM rejected).
  5. Gate commands; now `robocar.elf`/`.uf2` exist; record `arm-none-eabi-size build/b4/robocar.elf`.
  6. Linker check (`cmake/memmap_mtk3.ld`, official build): `arm-none-eabi-size -A` and `robocar.elf.map` show `.data_vector` and `.uninitialized_data` as NOLOAD RAM sections, and every `.noinit` object between `__bss_start__` and `__bss_end__` (zero-filled). No orphan-section warnings at link.
- **Stop if:**
  - no USB COM port: check `pre-kernel ok` first; if absent, the fault is before the kernel;
  - HardFault at first dispatch: check the vector writes, SHPR values and pool bounds;
  - the banner shows up inside the fault record (C3 broken).

  Without a Debug Probe a silent hang is hard to diagnose. Ask whether the team has one (`docs/DEBUG.md` assumes SWD).

### B5: heartbeats + mandatory USB CDC bring-up test (B-baseline acceptance)

- **Goal:** meet HANDOVER §4 acceptance.
- **Files:**
  - per O1: `src/debug/dbg_bringup.c` + `PLATFORM_BRINGUP` option in `CMakeLists.txt` (shared);
  - `testing/system/TEST_LOG.md` (test entry);
  - `testing/system/logs/` (saved console log).
- **Depends on:** B4, **O1**.
- **Work:**
  1. Nine tasks at the design priorities and periods. Each logs `id, seq, dt_min/max` (µs, from `hal_time_us()`).
  2. Buddy 2 runs the four steps in `docs/DEBUG.md` "USB CDC bring-up test" on the Pico W (C7):
     - boot + 9 heartbeat lines;
     - 10 minutes;
     - 10× log stress (overflow counter rises, periods unaffected);
     - unplug/replug and open/close the terminal.
- **Verify:** pass criteria exactly as in DEBUG.md: no sequence gaps, no HardFault, jitter within 1 ms. Buddy 2 saves the log in `testing/system/logs/` (his commit), referenced in TEST_LOG.md. Then declare **M1** in HANDOVER, and state that message types stay provisional until M2.
- **Stop if:** jitter exceeds 1 ms or sequence gaps appear. Investigate PRIMASK time (SysTick timer queue, `tm_putstring`) before changing priorities.

### B6: B-baseline measurement + tag

- **Goal:** Week 6 "before" numbers for the data-representation work.
- **Files:**
  - `cmake/size_report.c` + an OBJECT library in `CMakeLists.txt` (shared; never linked, like `config_check`);
  - `docs/measurements/week6-baseline.txt`;
  - `docs/DATA_REPRESENTATION.md` (baseline columns);
  - high-water report only. The stacks are painted since B2 (O8); B6 adds the scan, which counts untouched pattern bytes per task and prints through `dbg_`.
- **Depends on:** B5; the init-task measurement method (still open from O8).
- **Work** (week8-impact-report §D). Every number comes from Buddy 2's official build and board (sizes depend on the compiler, C8); the helper's local figures are not reported:
  - `arm-none-eabi-nm -S --size-sort` on `size_report.c.obj` for each `sizeof`;
  - `arm-none-eabi-size -A build/b6/robocar.elf`;
  - `MBF_BUFSZ` bytes;
  - pool use (`tk_ref_mbf` free sizes);
  - stack high-water marks for every task, including the 1 KB init task;
  - then the local annotated tag `baseline/week6-unpacked`. **Buddy 2 pushes it.**
- **Verify:** numbers reproducible from a fresh build dir; every stack high-water below its size with margin.
- **Stop if:** any stack high-water is within 128 B of its size, or the init task overflows.

### B7: B-packed

- **Goal:** minimum-bit types (D8), host-tested, re-measured.
- **Files:**
  - `src/common/data_repr.h` (pure, prefix `drep_`, masks and shifts, `_Static_assert`s);
  - `src/common/ipc_types.h` switched to packed types;
  - `tests/CMakeLists.txt` + host test sources (must `return()` when `CMAKE_CROSSCOMPILING`; must not glob `tests/rules_fixtures/`);
  - DATA_REPRESENTATION.md packed columns;
  - EVF_SAFETY bit positions fixed (merge-week8-summary Q9).
- **Depends on:** B6, O10, and the owner answers still open in DATA_REPRESENTATION.md (2-bit sequence and bypass command: buddy 5; steering sign: buddy 3).
- **Work:** the host-test list in week8-impact-report §E.3 (round trips at limits, sign extension, per-type `motion_cmd_t` `HOST-MC-01..08`, edge ring wrap, frame reject rules). Then re-measure with the B6 commands.
- **Verify:** host tests pass (`cmake -S tests -B build/host -G Ninja` with the host compiler); firmware gate commands; B5 bring-up test re-run briefly (2 minutes). Firmware gates and the re-run are Buddy 2's (C7); host tests run on his PC too (O10). Then declare **M2** in HANDOVER.
- **Stop if:** a packed type changes an interface some member already uses without their agreement.

## 4. Out of scope

`drv_encoder.c` and all other subsystem drivers and tasks (O3, C2); WiFi/MQTT (`TEL_BACKEND_WIFI` stays OFF); any `design.md` fix (Buddy 2 only).

## 5. Stale parts of the evidence reports (not edited; read with care)

- `mtk3-rp2040-integration-report.md`:
  - pins GP6/GP7 encoders, GP28 barcode, GP12 servo;
  - "UART0 fallback", `tm_com` on UART0, §14 step 5;
  - +256 B stack margin (now 512);
  - `knl_shutdown_hw` "returns `E_NOSPT`" (it is `void`).
- `batchA-summary.md`:
  - item 2 (`TSKID >= 11`, now 12);
  - item 13 (UART0).
- `week8-impact-report.md` §D.1: "requires `ProjectTemplate/` to be moved or excluded first" is already done (`check_rules` excludes it).
- `session.md` §5 / HANDOVER §1: "nothing is committed" is out of date. It is committed (`559f479`, `8d083ca`). HANDOVER §1 fixed in B0; `session.md` is untracked on Buddy 2's device only.
- C3 (found in B0): these say `tm_snd_dat` boot/fault output goes into the RAM fault record. Since C3, kernel text goes to the system-message ring and the record holds faults only:
  - `week8-impact-report.md` §B item 2 (line 88: also says `.noinit`, later corrected to `.uninitialized_data`), §D6 summary (line 270), Q16 (line 344);
  - `template-merge-report.md:243` and `merge-samename-diffs.md:33`: "banner through `dbg_`" (the kernel banner comes through the system-message ring; a `usermain` banner, if any, would use `dbg_`).
- `mtk3-rp2040-integration-report.md:300`, bring-up test step 1: "kernel banner (via `tm_*` → port)" is right in spirit; the port puts it in the system-message ring (DEBUG.md updated in B0).
- SDK line numbers in `mtk3-rp2040-integration-report.md` and `week8-impact-report.md` (e.g. `runtime.c:146-148`, `stdio_usb.c:195-208`, `irq.c:30-44`, `memmap_default.ld:120-128`) refer to SDK **1.5.1**. Since Rev 5 (C8) that is again the official SDK, so they are current. Only the helper's local 2.3.1 tree has moved file names (e.g. `runtime.c` became `pico_runtime_init/runtime_init.c`, §0).
