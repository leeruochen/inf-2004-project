# Batch B plan: platform (boot, kernel port, debug console, IPC objects)

Status: B0 reviewed by Buddy 2 (`batchB-b0-review-report.md`); Rev 6 fixes pushed (`af53004`), merge after his confirmation. B1-B3 code written on `batchB/b1-platform-code`, all compile gates and host tests pass (results in each stage). Next: B4 (first firmware, first flash).
Written 29 Oct 2026 for the Batch B helper (Buddy 3, Jin Yong), with Buddy 2's go-ahead.

Rev 2 (29 Oct 2026, after review):
- the stack model is decided before B2 (O8);
- the boot buffer became a continuously drained system-message ring (C3/O5);
- milestones M1/M2 were added (§3).

Rev 6 (30 Oct 2026, after Buddy 2's B0 review; Buddy 2 in chat, 30 Oct: "proceed based on the findings"):
- Toolchain: Pico SDK 1.5.1 installer + GCC 10.3.1 only (C8). No other SDK is used, by anyone.
- Workflow (C7, revised): the helper installs the same SDK 1.5.1 installer and runs the official compile gates themselves; Buddy 2 verifies the hardware stages.
- B1, B2 and B3 ship as one PR (all compile-only).
- O2: the B2 baseline uses the Week 6 field set.
- O7: reset by `watchdog_reboot`, halt only on a RAM flag or build option.
- O8: trampoline stack painting replaces the `TA_USERBUF` arena.
- B4: self-test hooks behind a default-OFF CMake option; the fault record is cleared only after a delivered print.
- New milestone M1-lite after B4.
- Wording fixes: `.data_vector`, `CMakeLists.txt:161`, provenance labels.

## 0. Baseline and decisions

**Toolchain (C8):** Pico SDK **1.5.1** Windows installer with its GCC **10.3.1**, CMake 3.25.2 and Ninja 1.11.1, exactly as in HANDOVER §5. Every gate, every `.uf2` and every measurement comes from this environment, on whichever PC runs it.

**Verified baseline** (Buddy 2's device, 29 Oct 2026):
- `8d083ca` = `main` = `origin/main`; working tree clean except an untracked `session.md`.
- Submodule ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp`.
- SDK 1.5.1, GCC 10.3.1.
- Fresh `build/fresh`: 84/84 steps, no warnings, `check_rules: PASS (42 files scanned)`, `check_rules_selftest: PASS`, `firmware target skipped`.
- Pico W board and cable: BOOTSEL shows `RPI-RP2`.

**Decisions.** "Confirmed" means Buddy 2 has said yes. "Chat" means his informal yes in the team chat; he confirms on the B0 PR before merge.

| # | Decision | Status |
|---|---|---|
| C1 | Buddy 2 approves Batch B work by the helper (Buddy 3) | owner, verbal; confirm on the B0 PR |
| C2 | Each member writes the tests for their own subsystem (`subsystemN_test.c`) | team |
| C3 | The kernel's own boot text (`tm_printf` banner, system messages) is kept **separate** from the RAM fault record. It goes into a system-message ring that the drain empties every cycle, so no `tm_*` output is lost after boot either. The fault record holds faults only. | confirmed (owner; agreed in the review) |
| C4 | Two parts: B-baseline (Week 6 unpacked types, measured, tagged) then B-packed | HANDOVER §4 |
| C5 | Approach (a): the SDK owns startup; the kernel is the `mtk3` library behind `src/rtos/port/` | ARCHITECTURE.md |
| C6 | `ipc_init()` creates every IPC object centrally, before any `subsystemN_init()` (T5) | ARCHITECTURE.md T5 |
| C7 | Workflow, §2: the helper writes each stage and runs the official compile gates on their own SDK 1.5.1 install; Buddy 2 reviews and merges, and runs every hardware stage (flash, USB, measurements). A stage passes only with official-toolchain evidence, and hardware stages only with Buddy 2's run | chat, 30 Oct 2026 (review §2a) |
| C8 | Toolchain: SDK 1.5.1 + GCC 10.3.1 only. No SDK migration stage | chat, 30 Oct 2026 |

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
- **The firmware switch.** Creating `src/app/main.c` switches CMake from compile-check to full link (`CMakeLists.txt:161`). It is created in B4 only, when everything it needs exists.
- Before B4, every registered `src/` source is compiled by `app_program_check` (kernel side, kernel include paths only) or `app_program_check_sdk` (SDK side, `hardware_*` include paths only). `pico_stdlib`/`pico_stdio` headers are NOT on that path, so `main.c` cannot be compile-checked early.
- `mtk3` leaves 8 symbols for the port to define: `knl_lowmem_top`, `knl_lowmem_limit`, `knl_shutdown_hw`, `knl_restart_hw`, `tm_com_init`, `tm_snd_dat`, `tm_rcv_dat`, `usermain` (`cmake/mtk3.cmake:15-18`). Kernel prototypes:
  - `void knl_shutdown_hw(void)` and `ER knl_restart_hw(W mode)` (`kernel/knlinc/kernel.h:215-216`);
  - `void tm_snd_dat(const UB*, INT)` (`lib/libtm/libtm.h:25-28`);
  - `W knl_taskindp` (`armv6m/cpu_cntl.c:30`);
  - `INT main(void)` in `sysinit.c:26`, renamed to `mtk3_kernel_main`.
- The kernel ISR entry only does `knl_taskindp++` / `--` (`armv6m/cpu_status.h:71-78`). A kernel call from an ISR pends PendSV through `END_CRITICAL_SECTION` (`cpu_status.h:30-35`), so the wrapper needs nothing more.
- `knl_init_interrupt()` does not touch VTOR. Only the excluded `reset_hdl.c:61` does, so the SDK RAM vector table stays active.
- The kernel banner is printed by `tm_printf` in `inittask.c:119` on every boot (this is why C3 exists). It is the only reachable `tm_*` caller among the compiled kernel sources today. The other callers are the kernel fault handlers in `exc_hdr.c:28,56,83`, which are never installed. Our `port_boot.c`/`port_fault.c` may call `tm_printf` too (R3b), so `tm_snd_dat` can run after boot. `tm_putchar`/`tm_putstring` wrap every `tm_snd_dat` call in `DI`/`EI` (`libtm.c:89-94, 108-112`).
- Kernel stacks: `knl_Imalloc(stksz + DEFAULT_SYS_STKSZ)` rounded to 8, `DEFAULT_SYS_STKSZ` = 128 (`task_manage.c:60-65`, `armv6m/sysdef.h:114,120`); one stack per task, initial frame at `stack + sstksz` (`task_manage.c:93`). The init task stack (1 KB, `include/sys/inittask.h:26-27`) is always kernel-allocated.
- Linker script (`cmake/memmap_mtk3.ld`, Batch A copy of the SDK 1.5.1 default): `.noinit` (kernel `Noinit()` control blocks, which MUST be zeroed, `USE_NOINIT 0`) is inside crt0's zero-filled `__bss_start__..__bss_end__` range (`:217-231`, `ASSERT` at `:275`). `.data_vector (NOLOAD)` (`:126-137`, before `.data`, not zeroed) holds the kernel's `exchdr_tbl`, which is dead storage because the SDK owns VTOR and we never call `tk_def_int`. `.uninitialized_data` (`:191-194`) is NOLOAD and not zeroed (fault record). **No linker work is needed in Batch B.**
- Kernel `HardFault_Handler` (`exc_hdr.c:45`) is never installed, because the SDK vector table uses `isr_hardfault`. The port must provide the HardFault handler.
- `size_t` clash (found in B1): `tk/syslib.h:124-126` typedefs `size_t` as `SZ` (`signed long`) unless `PROHIBIT_DEF_SIZE_T` is defined, which clashes with the C library's `size_t` (`<string.h>`, `<stdio.h>`). With the guard, a C library header that declares `size_t` must come BEFORE `<tk/tkernel.h>`. Since B1, `app.h` includes `<stddef.h>` and sets the guard, so kernel-side files include `app.h` (or `subsystemN.h`) first; platform files that include `<tk/tkernel.h>` directly put the C library headers and the guard first (`dbg_ring.c`).
- Tasks run on MSP (`config/priorities.h:54`). A task stack overflow can make exception stacking fault and LOCKUP the core, where no handler runs. Known gap: only a watchdog recovers it (open question Q9, §1).

## 1. Decisions per question

All have an answer; open items are marked **open**. "Review" = `batchB-b0-review-report.md` §2/§2a.

| # | Question | Answer | Stage |
|---|---|---|---|
| O1 | Acceptance needs "nine heartbeat lines" (DEBUG.md). Who provides the 9 heartbeat tasks? | Temporary platform bring-up tasks in `src/debug/dbg_bringup.c`, built only with a new CMake option `PLATFORM_BRINGUP=ON` (default OFF), at the design priorities and periods. Deleted once the members' real tasks exist. Its sibling `PLATFORM_SELFTEST` (B4) holds the self-test hooks. Shared `CMakeLists.txt` change. | B4/B5 |
| O2 | The Week 6 unpacked C types exist nowhere as C. Who writes them, and where? | Platform writes them once in `src/common/ipc_types.h` (pure, `<stdint.h>` only), every struct marked `TODO(owner-to-confirm, buddy N)` for its IF owner. **The B2 baseline uses the Week 6 field set** (design.md 3.3: natural fields, timestamp kept; sizes as DATA_REPRESENTATION.md §2 "Baseline bytes"). The Week 8 additions (clearances, bypass command, sequence number, dropped timestamp; ARCHITECTURE D8) arrive in B7. | B2 |
| O3 | `drv_encoder.c` is tagged `TODO(phase 1 batch B)` (`drv_encoder.h:9`), but HANDOVER §3 lists it as Buddy 2's work | Out of Batch B; Buddy 2 writes it in `subsystem2/` | none |
| O4 | Fault record layout | Magic `0x464C5452` ("FLTR" as a number; in memory the bytes read "RTLF", little-endian, so `x/wx` shows `0x464c5452`). Payload: version, fault type, PC, LR, xPSR, kernel error code, boot count (7 words). CRC-32 IEEE (reflected, poly `0xEDB88320`, init/xorout `0xFFFFFFFF`) over the payload. Total 9 words = 36 B. CRC in a pure header `src/common/crc32.h`, checked against `0xCBF43926` for ASCII "123456789" in the B4 self-test and the B7 host tests. **Open (Q8):** add SP and current task id; keep or drop boot count (it does not survive the clear). | B3 |
| O5 | System-message ring (C3): size and overflow policy | 512 B `.bss` ring, single writer (`tm_snd_dat`, already serialised by `DI`/`EI` in `libtm.c`) and single reader (the drain). `volatile uint32_t` head (writer only) and tail (reader only), no lock; a compiler barrier, no DMB (single core). `tm_snd_dat` never blocks. On full, drop new bytes and count them (writer-only counter). The drain reads head once per cycle, copies, then writes tail. The banner is about 34 B. Text written during a HardFault dies with the reset, so the fault record is the authority for crashes. | B3 |
| O6 | `dbg_` ring: size, max line, overflow policy, ISR use | 2 KB ring, 96 B max line, drop the newest line and count it. `vsnprintf` into a stack buffer first, then reserve+copy inside `DI`/`EI`. Tasks only (no ISR logging in Phase 1). Every call compiles to nothing when `DBG_ENABLE=0`. Measure `vsnprintf` stack depth in B6. | B1 |
| O7 | After a HardFault: reset or halt? | Record, then reset the whole chip with `watchdog_reboot(0, 0, <small non-zero ms>)` (register writes only, safe in the handler). Not `SYSRESETREQ`: on RP2040 it resets only the core. Halt (`bkpt`) only if the RAM flag `port_fault_halt` is set (from gdb before a debug session) or the build option `PORT_FAULT_HALT` is ON; default = reset. Cortex-M0+ code cannot read `DHCSR`, so debugger detection is not used. A `bkpt` with no debugger attached would LOCKUP. | B3 |
| O8 | Stack model for painting and the B6 high-water report | **Trampoline painting** (review §2a(iii)). Stacks stay kernel-allocated; no `TA_USERBUF`, no `config/priorities.h` change. `app_start_task()` starts every task through one trampoline that takes `__builtin_frame_address(0)` as the stack top. It computes the bottom as top − (`stksz` + 128, rounded to 8), paints from just below its own frame to the bottom, records top/bottom in a static table, then calls the real entry. `usermain()` does the same for the 1 KB init task, which answers the init-task measurement question. The top is approximate by a few dozen bytes; a one-time check runs in B4 (PLATFORM_SELFTEST). B6 scans the table. | B2 |
| O9 | `ipc.h` includes `<tk/tkernel.h>` but is not in the R1 project-header regex | Add `ipc` to `_RE_KINC_PROJ` (`check_rules.cmake:61`) plus a fixture under `tests/rules_fixtures/` (pattern: `app_program/subsystem2/r1_drv_subsystem_hdr.c`) and the matching `check_rules_selftest.cmake` entry. `ipc.h` is kernel side only; SDK-side files reach IDs through `hal_*`. `ipc_types.h`, `mbf_size.h`, `crc32.h` stay pure. | B2 |
| O10 | Host compiler for host tests (B7) | Any host GCC: Buddy 2's MSYS2 (`C:/msys64`) or the helper's. Record the path in HANDOVER §5 | B7 |
| O11 | Helper's build environment | The helper installs the **Pico SDK 1.5.1 Windows installer** (HANDOVER §5) and builds from PowerShell. If the clone was made from WSL, Windows git refuses the submodule ("dubious ownership"); run once `git config --global --add safe.directory <repo>/external/mtk3_bsp`. The Pico W is with Buddy 2, so he flashes and runs every hardware test. | before B1 |

**Open owner questions** (review §6; none blocks B1-B3):

| # | Question | Recommendation | Blocks |
|---|---|---|---|
| Q3 | Will Buddy 2 write any Batch B code while the helper does? | No concurrent `src/` work. Optionally Buddy 2 (or pairing) takes the hardware-sensitive B3/B4 parts; at minimum he may push fixup commits to the B1-B3 and B4 branches | B1-B3 PR |
| Q7 | Run the 10-minute USB CDC test (B5) after M1-lite, in parallel with subsystem work? | Yes; tell the team (timing change, not a test change) | B5 |
| Q8 | Fault record: add SP and task id; keep boot count? | Add SP and task id; drop boot count unless stored outside the cleared record | B3 |
| Q9 | A watchdog (B5 or later) to recover LOCKUP from stack overflow? | Yes, as a follow-up item, off the critical path | after B5 |

## 2. Workflow and gate commands

**Who runs what (C7):**

| Stage | Official gate run by | Hardware |
|---|---|---|
| B0 (docs) | Buddy 2 (done, review §4) | – |
| B1-B3 (one PR, compile-only) | helper, on SDK 1.5.1; Buddy 2 reviews, may spot-check | – |
| B4 | helper (build gates) | Buddy 2 flashes and runs the B4 checks |
| B5, B6 | Buddy 2 | Buddy 2 (USB test, measurements) |
| B7 | helper (host tests + firmware gates) | Buddy 2 (2-minute re-run) |

1. The helper finishes the stage on `batchB/<stage>-<name>`, runs the gates below on SDK 1.5.1 in fresh build directories, and pushes. The PR states the commit hash, the environment (`arm-none-eabi-gcc --version`), each gate's outcome and the warning count, and which shared files changed.
2. Buddy 2 reviews. For hardware stages he pulls that exact commit, builds the official `.uf2` and runs the stage's hardware checks, building with `-DPLATFORM_SELFTEST=ON` / `-DPLATFORM_BRINGUP=ON` where the stage says so. The tested commit is the merged commit; no temporary commits.
3. Logs: B4 console logs go in a PR comment. The B5 log is committed by Buddy 2 on `main` after the stage merges (`testing/system/logs/`), and the helper cites its hash.
4. **A stage passes only with official-toolchain evidence; a hardware stage only with Buddy 2's run.** Docs-only follow-ups after verification (result blocks, HANDOVER §1/§4) need no re-run and go into the same PR before merge. HANDOVER edits come from the helper's stage PRs only, to avoid conflicts.
5. If a run fails, the log goes on the PR. The helper (or Buddy 2, by fixup commit, Q3) fixes it on the same branch, and the stage repeats from step 1. The next stage may start on top of an unmerged one; stages are verified and merged in order.

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

---

## 3. Stages

**Milestones for the other members:**

| Milestone | After | What members can rely on | Still changing |
|---|---|---|---|
| **M1-lite: firmware boots** | B4 | flash and run their own tasks: object IDs (`ipc.h`), `app_start_task()`, `dbg_`, `hal_*`, `APP_TEST` builds | message types in `ipc_types.h` (PROVISIONAL), EVF_SAFETY bit positions; B5 acceptance not yet run |
| **M1: platform accepted** | B5 | the mandatory USB CDC bring-up test passed (Phase 1 acceptance) | as above |
| **M2: messages final** | B7 | packed message types (`data_repr.h`), EVF_SAFETY bits, host tests | nothing in Batch B |

Members start subsystem tasks at M1-lite; B5 and B6 run in parallel with that work (Q7). Between M1-lite and M2, member code that reads or writes message fields may need edits. HANDOVER §1/§3 must say so at M1-lite.

Critical path: B0 merge → B1-B3 PR → B4 flash = M1-lite.

### B0: records and prerequisites (docs only)

- **Goal:** the repo matches reality before any code changes.
- **Files:** `HANDOVER.md`, `ARCHITECTURE.md` D6, `docs/DEBUG.md`, this plan, and Buddy 2's review report `batchB-b0-review-report.md`.
- **Work:**
  1. Fix the stale lines.
  2. Do NOT edit evidence reports in `.agents/tasks/`; list their stale parts in §5.
  3. Get answers to O1-O9.
- **Verify:** gate commands (no code change: expect the baseline). Diff is docs only.
- **Stop if:** Buddy 2 disagrees with any recorded decision.
- **B0 result:**
  - `fae43bf` (branch `batchB/b0-records`, pushed 30 Oct 2026): HANDOVER (commit status, stage table, decisions, workflow), ARCHITECTURE D6 + DEBUG.md (C3 system-message ring), this plan.
  - **Official run, Buddy 2's review** (review §4; SDK 1.5.1 + GCC 10.3.1, fresh clone of `fae43bf`):
    - `build/b0`: exit 0, 84/84, 0 warnings, `check_rules: PASS (42 files scanned)`;
    - `check_rules_selftest`: PASS;
    - `build/b0-test` (`APP_TEST=ON`): stops only at `test_report.h:35` in all five `subsystemN_test.c`, as expected;
    - `build/b0-rel` (Release): exit 0, 84/84, 0 warnings;
    - only the 4 docs files changed; fast-forward onto `main`.
  - Verdict: merge after small fixes.
  - Rev 6 fix commit (on the same branch) applies review §5 items 1-7 and the §2a recommendations. Docs only, so no gate re-run is needed (§2 step 4). Merge after Buddy 2 confirms C1, C7, C8 and the O answers on the PR.

### B1-B3: platform code (one PR: `batchB/b1-platform-code`, three commits)

All three are compile-only (nothing runs until B4) and use the same gate, so they ship as one PR with one commit per part. `check_rules` must be clean at every commit. The helper runs the official gates on the final commit (C7). Depends on: B0 merged, O11 (helper's SDK 1.5.1 install).

#### B1: `dbg_` ring API + shared-file hooks

- **Goal:** tasks can log without stdio, and `APP_TEST=ON` compiles.
- **Files (new):**
  - `src/debug/sources.cmake`;
  - `src/debug/dbg.h`: pure (`<stdint.h>`, `<stdarg.h>`). API: `dbg_puts(const char *)`, `dbg_printf(const char *, ...)`, `dbg_put_err(const char *what, int32_t er)`, `dbg_ring_read(...)`, `dbg_overflow_count(void)`. Macro no-ops when `DBG_ENABLE=0`.
  - `src/debug/dbg_ring.c`: kernel side. `vsnprintf` into a caller-stack buffer (allowed: `snprintf` is not an R3a call, see `check_rules_selftest.cmake:42`), then reserve+copy in a short `DI`/`EI` section from `<tk/tkernel.h>` (`armv6m/syslib.h:43-46`).
- **Files (changed, shared):**
  - `app_program/app.h`: `APP_CHECK` calls `dbg_put_err(p_what, er)`, with no formatting, because it runs on the 1 KB init stack.
  - `app_program/test_report.h`: remove the `#error`; `TEST_CHECK`/`TEST_SUMMARY` use `dbg_printf` with the fixed `[TEST]` formats already documented in the file.
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
- **B1 result (helper, branch `batchB/b1-platform-code`; SDK 1.5.1 + GCC 10.3.1 in WSL, the same versions as the installer):**
  - Files: `src/debug/{dbg.h, dbg_ring.c, sources.cmake}`; shared: `app_program/app.h` (`APP_CHECK` → `dbg_put_err`; `size_t` guard), `app_program/test_report.h` (`#error` removed, `[TEST]` lines via `dbg_printf`, file base name in FAIL lines); host tests `tests/CMakeLists.txt`, `tests/host/` (planned for B7; added early, no firmware effect).
  - Ring: records `[len][text]`, 2 KB, line ≤ 96 B, `DI`/`EI` around every ring access, drop newest + counter, empty lines not stored, `dbg_ring_read()` for the drain. `dbg_ring.c` is built only with `DBG_ENABLE=ON`; with OFF every call is an empty `static inline` (arguments still type-checked).
  - Gates (fresh `build/b1*`): Debug exit 0, 0 warnings, `check_rules: PASS (45 files)`; self-test PASS; **`APP_TEST=ON` builds** (was stopped at `test_report.h:35`); Release exit 0, 0 warnings, no `dbg_ring.c` object.
  - Host tests: `test_dbg_ring` 6647 checks, 0 failed (empty ring, order, line ends, cut at 96, printf, `put_err` incl. `INT32_MIN`, full ring drops newest and keeps oldest, exact fit, wrap at every end position, 500-round run, small read buffer); also clean under `-fsanitize=address,undefined`.

#### B2: IPC objects + task start (`src/common`)

- **Goal:** `usermain()` creates every design.md 3.3 object at its design depth and can start tasks.
- **Files (new):**
  - `src/common/sources.cmake`;
  - `src/common/ipc_types.h`: Week 6 unpacked types (O2); header comment "PROVISIONAL until M2 (B7)";
  - `src/common/mbf_size.h`: pure; `MBF_BUFSZ(N, T) = N * (4 + ((sizeof(T) + 3) & ~3))` plus `_Static_assert`s (integration report §9);
  - `src/common/ipc.h`: kernel side; `extern ID` for each object, `ipc_init()` prototype;
  - `src/common/ipc.c`;
  - the stack trampoline and its top/bottom table (O8), kernel side, in `src/common/`.
- **Files (changed, shared):**
  - `app_program/app_main.c`: `app_start_task()` = `T_CTSK{TA_HLNG|TA_RNG0, itskpri, stksz}` with the O8 trampoline as `task`, plus `tk_cre_tsk` + `tk_sta_tsk`, failures through `APP_CHECK`. Step 1 calls `ipc_init()`. `usermain()` paints the init task stack (O8).
  - `cmake/check_rules.cmake` + `cmake/check_rules_selftest.cmake` + one new fixture (O9).
  - `config/priorities.h`: **unchanged** (O8).
- **Depends on:** B1 (`APP_CHECK` logs).
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
  3. Kernel version self-test: `tk_ref_ver`, `spver == (TK_SPECVER_MAGIC << 12) | TK_SPECVER` = `0x6300`, logged. (Corrected in B2: `tk_ref_ver` reports `0x6000 | (major << 8) | minor`, `sys/knldef.h:230`, `misc_calls.c:67`; comparing with `TK_SPECVER` = `0x0300` alone would fail on every boot.)
  4. EVF_SAFETY bit positions stay as proposed (fixed in B7).
  5. Stack painting via the O8 trampoline. Painting happens here; reporting is in B6.
- **Verify:**
  - gate commands (compile only);
  - the `_Static_assert`s hold;
  - the `priorities.h` pool `#error` check still passes (file unchanged);
  - every subsystem still compiles untouched.
- **Stop if:**
  - an IF owner disputes a type;
  - the self-check grows `usermain`'s stack use past about 256 B.
- **B2 result (helper, same branch and toolchain as B1):**
  - Files: `src/common/{ipc_types.h, mbf_size.h, ipc.h, ipc.c, stack_mon.h, stack_mon.c, sources.cmake}`; shared: `app_program/app.h` (includes `ipc.h`, so every subsystem sees `MBF_NAV_IN` ... `MTX_MOTION`, the message types and the PROVISIONAL `EVF_*` bit masks), `app_program/app_main.c` (`app_start_task()`, `ipc_init()` in step 1, init-stack window), `cmake/check_rules.cmake` + `cmake/check_rules_selftest.cmake` + fixture `tests/rules_fixtures/app_program/subsystem2/r1_drv_ipc_hdr.c` (O9: `ipc` and `stack_mon` count as kernel-side headers).
  - Object names: `ipc.h` defines `MBF_NAV_IN` etc. as macros for runtime IDs (`ipc_mbf_nav_in` ...), so code uses the TEAM.md names directly. Attributes: MBFs `TA_TFIFO`; `SEM_ECHO` count 0, max 1 (TODO buddy 5); `EVF_SAFETY` `TA_WSGL`; `EVF_NAV` `TA_WMUL` (polling readers never get `E_OBJ`); `MTX_MOTION` `TA_INHERIT`.
  - `ipc_init()` tries every object, logs each failure through `dbg_put_err`, returns the first error. Depth self-check per MBF as in work item 2 (16-byte dummy + one `T_RMBF` on the init stack).
  - O8 as built: `stack_mon_trampoline` is naked (`mov r2, sp; bl stack_mon_task_start`), checked in the disassembly; bottom = top − `round8(stksz + 128)`. Init task: a 512 B window below `usermain`'s frame (`stack_mon_paint_init_window()`), because its top is unknown; safe while the kernel frames above `usermain` stay under 640 B of the 1152 B init stack. Table: 16 entries.
  - Sizes: `_Static_assert`s on the baseline sizes (4, 10, 16, 16, 20 B; edge 8 B) and on `MBF_BUFSZ` (64, 64, 40, 640 B) all hold.
  - Gates (fresh `~/robocar-build/b2*`): Debug, `APP_TEST=ON` and Release exit 0, 0 warnings, `check_rules: PASS (52 files)`, self-test PASS (new fixture fires). `config/priorities.h` unchanged.
  - Build note: fresh configures on the Windows drive from WSL (`/mnt/d`, DrvFs) intermittently lose files; WSL builds go to the Linux disk. Windows builds (PowerShell) are unaffected.

#### B3: kernel port layer (`src/rtos/port`)

- **Goal:** every symbol `mtk3` needs is defined, and the SDK and kernel are glued without breaking R1.
- **Files (new):**

  | File | Side | Contents |
  |---|---|---|
  | `src/rtos/sources.cmake` | – | registers the four port `.c` files |
  | `src/rtos/port/hal.h` | pure (`<stdint.h>`) | `hal_kernel_start(void)` (no return); `hal_irq_register(irq, fn, level)`; `hal_gpio_irq_register(gpio, events, fn)`; `hal_isr_wake/hal_isr_sig/hal_isr_setflg` (IDs as `int32_t`); `hal_time_us(void)` (for the B5 jitter test); `hal_console_connected(void)` (wraps `stdio_usb_connected()`, for the drain, B4); fault-record and system-message-ring accessors for the drain |
  | `port_kernel_internal.c` | kernel | the 96 KB pool (`KERNEL_POOL_BYTES`, 8-byte aligned); defines `knl_lowmem_top/limit`; `knl_shutdown_hw` (`disint` + spin); `knl_restart_hw` (return `E_NOSPT`); `port_isr_enter/leave` (`knl_taskindp++/--`, `extern W knl_taskindp`); `hal_isr_*` → `tk_wup_tsk`/`tk_sig_sem`/`tk_set_flg` |
  | `port_boot.c` | SDK | `hal_kernel_start()`: set pool bounds; SHPR2 = SVC 0, SHPR3 = SysTick 0, PendSV `0xC0` (as `reset_hdl.c:100-102`); write `((uint32_t *)scb_hw->vtor)[14] = knl_dispatch_entry`, `[15] = knl_systim_inthdr`; call `mtk3_kernel_main()`. Also `tm_com_init`/`tm_snd_dat` (→ system-message ring, C3/O5; never blocks, callable from any context)/`tm_rcv_dat` (returns nothing). `hal_console_connected()`. Kernel functions declared as bare `extern` prototypes, no kernel headers. |
  | `port_fault.c` | SDK | fault record in `__uninitialized_ram()` (`.uninitialized_data`, `memmap_mtk3.ld:191-194`), CRC-32 from `crc32.h` (O4), `isr_hardfault` override (naked entry, reads the stacked PC/LR/xPSR), then `watchdog_reboot` or halt per O7, accessors `valid/get/clear` |
  | `port_irq.c` | SDK | `hal_irq_register` → `irq_set_exclusive_handler` + `irq_set_priority(level << 6)` + `irq_set_enabled`, through a wrapper table (`port_isr_enter(); fn(); port_isr_leave();`). `hal_gpio_irq_register` → `gpio_set_irq_enabled` + `gpio_add_raw_irq_handler_masked` with the same wrapper, so `IO_IRQ_BANK0` stays on the SDK's shared chain (TEAM.md:145 dispatcher). Never `tk_def_int`. |
  | `src/common/crc32.h` | pure | CRC-32 IEEE (O4) |

  The linker script is unchanged (§0 layout facts).
- **Depends on:** B2 (`ipc.h` IDs for `hal_isr_*`), O4, O5, O7 (Q8 may add two record words).
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
- **B3 result (helper, same branch and toolchain):**
  - Files: `src/rtos/sources.cmake`; `src/rtos/port/{hal.h, port.h, port_kernel_internal.c, port_boot.c, port_fault.c, port_irq.c}`; `src/common/crc32.h`; shared: `CMakeLists.txt` adds `hardware_exception`, `hardware_clocks`, `hardware_timer`, `hardware_watchdog` to `app_program_check_sdk` and to the `robocar` link list. `port.h` (pure) carries the port's own calls across the kernel/SDK split (`port_isr_enter/leave`, `port_fault_boot/shutdown`, `port_sysmsg_puts`). `hal_console_connected()` is declared in `hal.h` and defined in B4 (`src/app/main.c`, the only file with `pico/stdio_usb.h`).
  - Start: `hal_kernel_start()` checks `clk_sys` = 125 MHz (the kernel's `TMCLK_KHz`; a mismatch is reported in the system-message ring), counts resets for a valid fault record, writes SHPR2 = 0 and SHPR3 = `0x00C00000` (SVC 0, SysTick 0, PendSV 3) at `PPB_BASE + M0PLUS_SHPRn_OFFSET` (SDK 1.5.1 has no exception-priority API), installs PendSV/SysTick with `exception_set_exclusive_handler()` (RAM vector table), calls `mtk3_kernel_main()`.
  - Pool: `static uint64_t s_kernel_pool[KERNEL_POOL_BYTES / 8]`; `knl_lowmem_top/limit` are static initialisers (no runtime code). `knl_shutdown_hw` records a SHUTDOWN fault, then resets or halts (O7); `knl_restart_hw` returns `E_NOSPT`.
  - System-message ring: 512 B, free-running `head` (writer) / `tail` (reader), compiler barriers, drop-and-count when full; `tm_rcv_dat` returns NUL bytes.
  - Fault record: 9 words in `.uninitialized_data.s_record` (O4); naked `isr_hardfault` picks MSP/PSP from EXC_RETURN and `bl`s the C part (checked in the disassembly); the stacked frame is read only if it lies in SRAM and is aligned; reset = `watchdog_reboot(0, 0, 10)`; halt = `bkpt` only with `port_fault_halt` (RAM, set from gdb) or `PORT_FAULT_HALT=1`. `boot_count` counts resets since the fault (incremented in `hal_kernel_start`).
  - IRQs: 8 fixed wrapper slots (`port_isr_enter(); fn(); port_isr_leave();`); `hal_irq_register` refuses `IO_IRQ_BANK0`, priority = level << 6; `hal_gpio_irq_register` uses `gpio_add_raw_irq_handler_masked` (handler acknowledges its own pin). Return codes match `E_PAR`/`E_LIMIT`.
  - Gates (fresh `~/robocar-build/b3*`): Debug, `APP_TEST=ON`, Release exit 0, 0 warnings, `check_rules: PASS (60 files)`, no R1 hit on any port file; self-test PASS.
  - Symbol check: every symbol `libmtk3.a` needs from outside is defined by our objects (`knl_lowmem_top/limit`, `knl_shutdown_hw`, `knl_restart_hw`, `tm_com_init`, `tm_snd_dat`, `tm_rcv_dat`; `usermain` in `app_main.c`), except `__aeabi_*` and `memcpy`, which the SDK/newlib provide at link.
  - Host tests: `test_crc32` (check value `0xCBF43926`, empty input, zeros, every single-bit flip of a payload changes the CRC) and `test_dbg_ring`: 2/2 pass.

### B4: SDK `main()` + debug drain = first firmware (first flash) → M1-lite

Split into two flashable steps (helper + Buddy 2, 30 Oct 2026), each tested on the board before the next:
- **B4a (branch `batchB/b4-first-firmware`):** `main.c`, the drain, the alive line. Hardware checks 1, 3, 6, 7 below.
- **B4b:** the `PLATFORM_SELFTEST` option and its checks 2, 4, 5.

- **Goal:** the firmware links, boots on the Pico W and prints over USB CDC.
- **Files (new):**
  - `src/app/sources.cmake`;
  - `src/app/main.c`: SDK side. `stdio_init_all()` (USB only, as `CMakeLists.txt` sets); wait up to `DBG_USB_CONNECT_WAIT_MS` for `stdio_usb_connected()`, sleep lines marked `RULE-ALLOW: pre-kernel`; one pre-kernel `printf("pre-kernel ok")` marked the same way (a sign of life before the kernel exists); `hal_kernel_start()`.
  - `src/debug/dbg_drain.c`: kernel side + `<stdio.h>`. Task at `TASK_PRI_DBG_DRAIN`, period `DBG_DRAIN_PERIOD_MS` via `tk_dly_tsk`. Fault record: if valid, print it directly (not through the `dbg_` ring), and clear it **only after a print while `hal_console_connected()` is true**, because `stdio_usb` silently drops output with no host (`stdio_usb.c:116-119`). Every cycle: empty the system-message ring (C3), then the `dbg_` ring lines; print both overflow counters when they change. `dbg_drain_start()`.
- **Files (changed, shared):**
  - `app_program/app_main.c` calls `dbg_drain_start()` after `ipc_init()` when `DBG_ENABLE`;
  - `CMakeLists.txt`: option `PLATFORM_SELFTEST` (default OFF, needs `DBG_ENABLE`), which compiles in:
    - a late `tm_printf` from `port_boot.c` after the drain runs;
    - the `PORT_FAULT_SELFTEST` forced fault (after a delay, or on a command);
    - the CRC-32 check value;
    - the one-time O8 stack-top check.
- **Depends on:** the B1-B3 PR merged. **This stage turns the firmware switch on.**
- **Work:** as listed.
  - The helper runs the build gates (now `robocar.elf`/`.uf2` exist).
  - **Buddy 2 builds the official `.uf2` from the PR commit and flashes it** by BOOTSEL (copy `build/b4/robocar.uf2` to `RPI-RP2`), once normally and once with `-DPLATFORM_SELFTEST=ON`.
  - Open the COM port within 3 s (`docs/DEBUG.md`). Fault-record checks need the terminal open before the reset completes, or the print is held until a host connects.
- **Verify (hardware, Buddy 2 on the Pico W; console logs posted on the PR):**
  1. `pre-kernel ok`, then the kernel banner (via the system-message ring), then `ipc_init` OK lines and the version self-test line.
  2. Late system message (`PLATFORM_SELFTEST`) appears on the console.
  3. No HardFault for 2 minutes; drain alive.
  4. Fault-record test (`PLATFORM_SELFTEST`):
     - forced fault → watchdog reset → record printed once with a correct PC;
     - next reset → nothing printed;
     - power-cycle → nothing printed (random RAM rejected);
     - a fault with no terminal open → printed after the terminal connects, not lost.
  5. CRC check value and the O8 stack-top check pass (`PLATFORM_SELFTEST`).
  6. Record `arm-none-eabi-size build/b4/robocar.elf`.
  7. Linker check (`cmake/memmap_mtk3.ld`): `arm-none-eabi-size -A` and `robocar.elf.map` show:
     - `.data_vector` and `.uninitialized_data` as NOLOAD RAM sections;
     - every `.noinit` object between `__bss_start__` and `__bss_end__` (zero-filled);
     - no orphan-section warnings at link.
- **Then:** declare **M1-lite** in HANDOVER: members may add tasks; message types stay provisional until M2.
- **Stop if:**
  - no USB COM port: check `pre-kernel ok` first; if absent, the fault is before the kernel;
  - HardFault at first dispatch: check the vector writes, SHPR values and pool bounds (SWD with the team's Debug Probe, `docs/DEBUG.md`);
  - the banner shows up inside the fault record (C3 broken).
- **B4a result (helper; SDK 1.5.1 + GCC 10.3.1 in WSL):**
  - Files: `src/app/{main.c, sources.cmake}`, `src/debug/dbg_drain.c`; `src/debug/{dbg.h, sources.cmake}` (`dbg_drain_start()`); shared: `app_program/app_main.c` (`dbg_drain_start()` after `ipc_init()`).
  - Drain as built: reads nothing while no terminal is connected (USB CDC would drop it), so boot text waits in the rings and appears when the terminal opens; prints the fault record directly and clears it after a connected print; every 10 s prints `[DBG] alive: SDK timer <ms>, kernel time <ms>, difference <ms>` (the difference must stay constant: proves the 1 ms tick). `hal_console_connected()` = `stdio_usb_connected()` in `main.c`.
  - **The firmware links** (the firmware switch is on): Debug, `APP_TEST=ON`, Release exit 0, 0 warnings, `check_rules: PASS (63 files)`, self-test PASS. Sizes: Debug text 49,780 B, bss 111,472 B; Release text 36,452 B.
  - Linker checks (`robocar.elf`, map): `.noinit` 6,360 B at `0x20000CD8` = `__bss_start__` = `__noinit_start` (zero-filled with `.bss`, which ends at `__bss_end__` = `0x2001BF64`); `knl_tcb_table`, `knl_tmp_stack` inside it; `.uninitialized_data` 36 B (`s_record`) flags `ALLOC` only (NOLOAD); `.data_vector` 0 B (`exchdr_tbl` removed by `--gc-sections`: never referenced without `tk_def_int`); RAM ends at `0x2001C764` (about 140 KB free). No orphan-section warnings.
  - Vector table (flash `0x10000100`): HardFault = `0x10000E85` = our `isr_hardfault`; PendSV/SysTick = SDK weak defaults, replaced at run time by `hal_kernel_start()`; initial SP `0x20042000`. Kernel idle `low_pow()` is empty (`pico_rp2040/power_save.c`): no clock changes.
  - **Hardware (Buddy 2): pending.** Checklist: B4a hardware test below.
- **B4b result (helper, branch `batchB/b4b-selftest`, on top of B4a):**
  - Files: `src/debug/dbg_selftest.c` (task at `TASK_PRI_TEST`, `TASK_STACK_TEST_BYTES`); `src/rtos/port/{hal.h, port_fault.c, port_boot.c}` (self-test hooks; `hal_fault_seen_at_boot()` in every build); `src/debug/{dbg.h, sources.cmake}`; shared: `CMakeLists.txt` (option `PLATFORM_SELFTEST`, default OFF; refused with Release or with `APP_TEST`, so the 12-task budget holds), `app_program/app_main.c` (`dbg_selftest_start()`, a no-op unless the option is ON).
  - Sequence once per power-up, stage in watchdog scratch 0 (kept through watchdog resets, cleared by power-on): checks (no record at power-up, CRC check value, own stack bounds = round8(stksz + 128), every started stack has unused pattern, per-task unused bytes, late `tm_printf`) then a forced HardFault (`udf #0`, the first instruction of `hal_selftest_force_fault`); after the reset the record must have been there (the drain prints it once); then a plain watchdog reset; after it the record must be gone.
  - Gates: Debug, `APP_TEST=ON`, Release and Debug + `PLATFORM_SELFTEST=ON` exit 0, 0 warnings, `check_rules: PASS (64 files)`; Release + self-test and `APP_TEST` + self-test are refused at configure. The normal firmware contains no self-test symbol.
  - Hardware test for Buddy 2: `.agents/tasks/batchB-b4b-test.md` (after B4a passes). Not pushed until B4a passes on the board.
- **B4a hardware test (Buddy 2, Pico W, Debug build):** step by step for an AI agent in `.agents/tasks/batchB-b4a-test.md` (capture with `tools/serial_capture.ps1`).
  1. `git switch batchB/b4-first-firmware`, fresh `build/b4` (HANDOVER §5), `& $CM --build build/b4`; expect 0 warnings and `build/b4/robocar.uf2`.
  2. Hold BOOTSEL, plug in USB, copy `robocar.uf2` to `RPI-RP2`. Open the new COM port (PuTTY, 115200) within 3 s; if later, the boot text still appears when the port opens.
  3. Expect, in this order: `pre-kernel ok`; `microT-Kernel Version 3.00`; `[IPC] kernel spec version 3.00 OK`; `[IPC] 8 objects created, MBF depth self-check OK`. No `[ERR]` and no `[FAULT]` line. (`pre-kernel ok` is lost if the port opens after the 3 s wait: that is fine.)
  4. Leave it 2 minutes: an `[DBG] alive` line every 10 s, and the `difference` value never changes by more than 1-2 ms.
  5. Close and reopen the terminal, then unplug and replug USB: it keeps running (after a replug the board restarts: the boot lines come again).
  6. Post the console log on the PR. Stop and report if: no COM port, no `pre-kernel ok` (fault before the kernel), banner but no `[IPC]` lines (first dispatch failed), any `[ERR]`/`[FAULT]` line, or the difference drifts.

### B5: heartbeats + mandatory USB CDC bring-up test (B-baseline acceptance) → M1

- **Goal:** meet HANDOVER §4 acceptance. Runs after M1-lite, in parallel with subsystem work (Q7).
- **Files:**
  - per O1: `src/debug/dbg_bringup.c` + `PLATFORM_BRINGUP` option in `CMakeLists.txt` (shared);
  - `testing/system/TEST_LOG.md` (test entry);
  - `testing/system/logs/` (saved console log, Buddy 2's commit on `main`, §2 step 3).
- **Depends on:** B4.
- **Work:**
  1. Nine tasks at the design priorities and periods. Each logs `id, seq, dt_min/max` (µs, from `hal_time_us()`).
  2. Buddy 2 runs the four steps in `docs/DEBUG.md` "USB CDC bring-up test" on the Pico W, built with `-DPLATFORM_BRINGUP=ON`:
     - boot + 9 heartbeat lines;
     - 10 minutes;
     - 10× log stress (overflow counter rises, periods unaffected);
     - unplug/replug and open/close the terminal.
- **Verify:** pass criteria exactly as in DEBUG.md: no sequence gaps, no HardFault, jitter within 1 ms. Log referenced in TEST_LOG.md. Then declare **M1**.
- **Stop if:** jitter exceeds 1 ms or sequence gaps appear. Investigate PRIMASK time (SysTick timer queue, `tm_putstring`) before changing priorities.

### B6: B-baseline measurement + tag

- **Goal:** Week 6 "before" numbers for the data-representation work.
- **Files:**
  - `cmake/size_report.c` + an OBJECT library in `CMakeLists.txt` (shared; never linked, like `config_check`);
  - `docs/measurements/week6-baseline.txt`;
  - `docs/DATA_REPRESENTATION.md` (baseline columns);
  - high-water report: the B6 scan reads the O8 table and counts untouched pattern bytes per task, including the init task, printed through `dbg_`.
- **Depends on:** B5 (can overlap subsystem work).
- **Work** (week8-impact-report §D). Run-time numbers come from Buddy 2's board:
  - `arm-none-eabi-nm -S --size-sort` on `size_report.c.obj` for each `sizeof`;
  - `arm-none-eabi-size -A build/b6/robocar.elf`;
  - `MBF_BUFSZ` bytes;
  - pool use (`tk_ref_mbf` free sizes);
  - stack high-water marks for every task, including the 1 KB init task and `vsnprintf` users (O6);
  - then the local annotated tag `baseline/week6-unpacked`. **Buddy 2 pushes it.**
- **Verify:** numbers reproducible from a fresh build dir; every stack high-water below its size with margin.
- **Stop if:** any stack high-water is within 128 B of its size, or the init task overflows.

### B7: B-packed → M2

- **Goal:** minimum-bit types (D8), with the Week 8 fields, host-tested, re-measured.
- **Files:**
  - `src/common/data_repr.h` (pure, prefix `drep_`, masks and shifts, `_Static_assert`s);
  - `src/common/ipc_types.h` switched to packed types with the Week 8 fields (O2);
  - `tests/CMakeLists.txt` + host test sources (must `return()` when `CMAKE_CROSSCOMPILING`; must not glob `tests/rules_fixtures/`), including the CRC-32 check value (O4);
  - DATA_REPRESENTATION.md packed columns;
  - EVF_SAFETY bit positions fixed (merge-week8-summary Q9).
- **Depends on:** B6, O10, and the owner answers still open in DATA_REPRESENTATION.md (2-bit sequence and bypass command: buddy 5; steering sign: buddy 3).
- **Work:** the host-test list in week8-impact-report §E.3 (round trips at limits, sign extension, per-type `motion_cmd_t` `HOST-MC-01..08`, edge ring wrap, frame reject rules). Then re-measure with the B6 commands.
- **Verify:**
  - host tests pass (`cmake -S tests -B build/host -G Ninja` with the host compiler; helper);
  - firmware gate commands (helper);
  - B5 bring-up test re-run briefly, 2 minutes (Buddy 2).

  Then declare **M2** in HANDOVER.
- **Stop if:** a packed type changes an interface some member already uses without their agreement.

## 4. Out of scope

`drv_encoder.c` and all other subsystem drivers and tasks (O3, C2); WiFi/MQTT (`TEL_BACKEND_WIFI` stays OFF); any `design.md` fix (Buddy 2 only); any SDK or toolchain change (C8).

## 5. Stale parts of the evidence reports (not edited; read with care)

- `mtk3-rp2040-integration-report.md`:
  - pins GP6/GP7 encoders, GP28 barcode, GP12 servo;
  - "UART0 fallback", `tm_com` on UART0, §14 step 5;
  - +256 B stack margin (now 512);
  - `knl_shutdown_hw` "returns `E_NOSPT`" (it is `void`);
  - line 300, bring-up test step 1, "kernel banner (via `tm_*` → port)": right in spirit; the port puts it in the system-message ring.
- `batchA-summary.md`:
  - item 2 (`TSKID >= 11`, now 12);
  - item 13 (UART0).
- `week8-impact-report.md`:
  - §D.1: "requires `ProjectTemplate/` to be moved or excluded first" is already done (`check_rules` excludes it);
  - C3: §B item 2 (line 88, which also says `.noinit`, later corrected to `.uninitialized_data`), the D6 summary (line 270) and Q16 (line 344) say `tm_snd_dat` output goes into the fault record. Since C3 it goes to the system-message ring, and the record holds faults only.
- `template-merge-report.md:243` and `merge-samename-diffs.md:33`: "banner through `dbg_`". The kernel banner comes through the system-message ring; a `usermain` banner, if any, would use `dbg_`.
- `session.md` §5: "nothing is committed" is out of date (committed in `559f479`, `8d083ca`). `session.md` is untracked, on Buddy 2's device only.
- O7 in any earlier text ("halt if `DHCSR.C_DEBUGEN`", "`SYSRESETREQ`") is replaced by the Rev 6 O7.
