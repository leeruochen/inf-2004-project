# Phase 1 Batch A (pass 2): `src/` layout, fail-closed rule checks and debug option defaults

This pass checks the fixes for the seven pass-1 findings. Module sources now live under `src/<module>/`, matching the owner's `src/rtos/port/port_kernel_internal.c`. The rule checker scans the whole tree, and configure fails on any `sources.cmake` it would otherwise ignore. `DBG_ENABLE` now defaults from the build type. The cache-default behaviour, the init-task stack exception and the proposed telemetry delta D2 are documented. The kernel library, config copies, linker script and remaining config headers are unchanged since pass 1, which verified them against report section 11 and the owner's decisions. The build evidence is complete:

- Both backends configure and build `mtk3`, `config_check` and ALL with 0 warnings.
- Each new guard has a negative test.
- A Release configure shows `DBG_ENABLE=OFF`.

I spot-checked four things: `check_rules` passes (14 files scanned), its self-test passes, the vendored tree is clean at `15ed232`, and HEAD `a3ebae1` is unchanged, so nothing was committed.

Watch for: no blocking concerns remain. The open items the coder carries (D2 approval, the obstacle timing conflict, `IMU_HAS_GYRO`, attribution wording) wait on the owner and do not block Batch A.

**Verdict**: APPROVED

## High-level view

The `src/` layout is applied in every place pass 1 named: `ROBOCAR_SRC_DIR` and module registration in `CMakeLists.txt`, the allow-lists and `RULES_PORT_DIR_REGEX` in `check_rules.cmake`, the self-test fixtures, and a new "Source layout" section in ARCHITECTURE.md. `config/` stays at the top level, as the owner approved for `config/mtk3/`.

The rule checker now fails closed. It skips only the top-level `external`, `build*`, `tests`, `docs`, `.git`, `.agents` and `.vscode`. New self-test fixtures prove it reaches an unlisted directory and a root-level file. The module-registration guard rejects any top-level `*/sources.cmake` or unlisted `src/**/sources.cmake`.

`DBG_ENABLE` is evaluated after `pico_sdk_init()`, so an empty build type has already been defaulted by the SDK's `pico_pre_load_toolchain.cmake` and resolves to OFF. The init-stack exception is recorded in ARCHITECTURE.md and `priorities.h`. D2 is marked TODO(owner-to-confirm) in ARCHITECTURE.md and `timing.h`.

<details>
<summary>Issues (0)</summary>

No open action items in Batch A. The owner still has to decide the items the coder already carries: D2 approval, the obstacle budget (60 ms step vs settle plus 3 × 30 ms echo), `IMU_HAS_GYRO`, the NOTICE/README wording, and whether R2 should cover `i2c_*_blocking`.

</details>

<details>
<summary>Details</summary>

### Carry-forward for Batch B

The allow-lists in `check_rules.cmake:31-34` fix the Batch B file names: `src/debug/dbg_drain.c`, `src/rtos/port/port_boot.c`, `src/rtos/port/port_fault.c` and `src/app/main.c`. Batch B must use these names or update the variables, or its legitimate stdio and `tm_*` calls will fail the build. `exchdr_tbl` sits in `.data_vector (NOLOAD)` and is never dispatched, because the SDK owns VTOR. Every ISR must go through the SDK irq API, since a `tk_def_int` registration would silently never fire (summary open item 15).

</details>

<details>
<summary>File map</summary>

- `CMakeLists.txt`: `src/` module registration with an unlisted-`sources.cmake` guard; `DBG_ENABLE` default by build type; cache-default comment.
- `cmake/check_rules.cmake`: whole-tree scan with top-level excludes; `src/` allow-list paths.
- `cmake/check_rules_selftest.cmake`, `tests/rules_fixtures/**` (9 files): fixtures moved under `src/`; new unlisted-dir, root-file and excluded-dir cases.
- `ARCHITECTURE.md`: source layout, init-stack exception, D2 marked PROPOSED.
- `config/priorities.h`, `config/timing.h`: init-stack exception note; D2 TODO(owner-to-confirm).
- `docs/DEBUG.md`: cache-default note, fresh-build-dir and Release examples.
- Unchanged since pass 1: `cmake/mtk3.cmake` (comment only), `cmake/memmap_mtk3.ld`, `cmake/warnings.cmake`, `config/mtk3/*`, `config/pins.h`, `config/thresholds.h`, `docs/SUBMODULE.md`, `NOTICE.md`, `README.md`, `.gitmodules`, `external/mtk3_bsp`.

Full diff: `git status` / `git diff` in the repo root (all untracked or staged; HEAD `a3ebae1`).

</details>
