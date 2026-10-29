# Step 0 verification: check_rules unblock (ProjectTemplate/ excluded)

Nothing is committed. Repo HEAD at the time of this step is `df67420` ("status", which adds `INF 2004 Embedded Systems.md` and edits `NOTICE.md`). Its parent is `79bc417`, the commit that added `ProjectTemplate/`. ProjectTemplate/ is tracked (643 files) and unchanged by this step.

Housekeeping: `merge-review.json`, `week8-review.json` and `final-review.json` did not exist (`Test-Path` returned False for all three), so nothing was deleted.

## Files changed

| File | Change |
|---|---|
| `cmake/check_rules.cmake` | `RULES_EXCLUDE_TOP_REGEX` now also skips the top-level `ProjectTemplate`, `testing` and `videos`. The header comment lists them. |
| `CMakeLists.txt` | The unregistered-`sources.cmake` guard drops any match under `ProjectTemplate/` (new `ROBOCAR_NEVER_SCAN_REGEX "^ProjectTemplate/"`) before the check, so a template file can never be included or make configure fail. |
| `cmake/check_rules_selftest.cmake` | Expects `app_program/subsystem1/r3_app_program.c:2: [R3b]`. Fails if any `ProjectTemplate/` path is reported. |
| `tests/rules_fixtures/ProjectTemplate/app_program/subsystem1/excluded.c` (new) | `tm_printf` + `sleep_ms` inside an excluded ProjectTemplate path: must not be reported. |
| `tests/rules_fixtures/app_program/subsystem1/r3_app_program.c` (new) | `tm_printf` in `app_program/`: R3b must fire on line 2. |

No same-name file was touched. ProjectTemplate/ has no `CMakeLists.txt` or `*.cmake`, and the changed files have no counterpart inside it. README.md, .gitignore, NOTICE.md, design.md, specs.md, `INF 2004 Embedded Systems.md` and `external/mtk3_bsp` were not edited.

## Commands (PowerShell, from the repo root)

Preamble, exactly as in `batchA-summary.md`:

```
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

`build/usb` was deleted first, so this is a from-scratch build. Each command wrote to `build/step0-logs/<name>.log`. Warnings and errors were counted with `Select-String "warning:"` / `"error:"`.

| Command | Result |
|---|---|
| (before the fix) `& $CM -DRULES_ROOT=<repo> -P <HEAD:cmake/check_rules.cmake>` | exit 1, `check_rules: 94 violation(s)`, all 94 under `ProjectTemplate/` (reproduces the reported failure) |
| `& $CM -S . -B build/usb @common -DDBG_BACKEND=USB` | exit 0; `DBG_ENABLE=ON DBG_BACKEND=USB TEL_BACKEND_WIFI=OFF`, `mtk3: ... pinned at 15ed232..., clean`, `no module sources yet (Batch A); firmware target skipped`; 0 warnings |
| `& $CM --build build/usb --target mtk3` | exit 0, `[44/44] Linking C static library libmtk3.a`, **0 warnings, 0 errors** |
| `& $CM --build build/usb --target config_check` | exit 0, **0 warnings, 0 errors** |
| `& $CM --build build/usb --target check_rules` | exit 0, `check_rules: PASS (14 files scanned)`, **0 violations** |
| `& $CM --build build/usb --target check_rules_selftest` | exit 0, `check_rules_selftest: PASS` |
| `& $CM --build build/usb` (ALL) | exit 0, `check_rules: PASS (14 files scanned)`, 0 warnings, 0 errors |
| `& $CM -P cmake/check_rules.cmake` | exit 0, `check_rules: PASS (14 files scanned)` (the same 14 files as Batch A, so nothing under ProjectTemplate/ is read) |
| `& $CM -P cmake/check_rules_selftest.cmake` | exit 0, `check_rules_selftest: PASS` |
| `& $CM -DRULES_ROOT=<repo>/tests/rules_fixtures -P cmake/check_rules.cmake` | exit 1 (expected), `check_rules: 10 violation(s)`. The output includes `app_program/subsystem1/r3_app_program.c:2: [R3b]` and contains no `ProjectTemplate/` line |
| `git status --porcelain -- ProjectTemplate` | **empty**, exit 0 |
| `git -C external/mtk3_bsp status --porcelain` | empty |
| `git submodule status` | ` 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp (v1.00.00.B5-pico_rp2040)` |
| `git status --porcelain` | ` M CMakeLists.txt`, ` M cmake/check_rules.cmake`, ` M cmake/check_rules_selftest.cmake`, `?? tests/rules_fixtures/ProjectTemplate/`, `?? tests/rules_fixtures/app_program/` |

Warnings in our code: **0**. Warnings in mtk3: **0**.

## Every GLOB, and why none reaches ProjectTemplate/

`Select-String -Path cmake/*.cmake, CMakeLists.txt, pico_sdk_import.cmake -Pattern GLOB`:

```
check_rules.cmake:69: file(GLOB _top LIST_DIRECTORIES true RELATIVE "${_root}" "${_root}/*")
check_rules.cmake:76: file(GLOB_RECURSE _found "${_root}/${_e}/${_pat}")
check_rules.cmake:90: # Regex form of each glob pattern, for files directly in the root.   (comment)
CMakeLists.txt:98: # A file is in exactly one list. No globbing of sources.            (comment)
CMakeLists.txt:111: file(GLOB _found_top "${PROJECT_SOURCE_DIR}/*/sources.cmake")
CMakeLists.txt:114: file(GLOB_RECURSE _found_src "${ROBOCAR_SRC_DIR}/sources.cmake")
CMakeLists.txt:182: # glob tests/rules_fixtures/ (rule-check fixtures, never compiled). (comment)
```

`cmake/mtk3.cmake`, `cmake/warnings.cmake` and `pico_sdk_import.cmake` have no GLOB.

- `check_rules.cmake:69` is non-recursive. It lists only the top-level entry names. `ProjectTemplate` matches `RULES_EXCLUDE_TOP_REGEX`, and the loop `continue()`s before any file operation on it.
- `check_rules.cmake:76` runs only for top-level entries that passed that exclusion, so it never descends into `ProjectTemplate/`. The unchanged count of 14 scanned files is the evidence.
- `CMakeLists.txt:111` is one level deep (`<top>/sources.cmake`), so it could only match `ProjectTemplate/sources.cmake`, which does not exist. The template has no `CMakeLists.txt`, `*.cmake` or `sources.cmake` at any depth (`Get-ChildItem -Recurse -Force -Include sources.cmake,CMakeLists.txt,*.cmake` count = 0). Any future match is dropped by `ROBOCAR_NEVER_SCAN_REGEX "^ProjectTemplate/"` before the registration check, and only the registered `src/<module>/sources.cmake` files are ever `include()`d.
- `CMakeLists.txt:114` is rooted at `${PROJECT_SOURCE_DIR}/src`, so it cannot reach `ProjectTemplate/`.
- No `add_subdirectory` or `include` names ProjectTemplate. The only `add_subdirectory` is `tests`, and only if `tests/CMakeLists.txt` exists (it does not).

## Handoff for later steps

The owner added corrections at the top of `.agents/tasks/week8-impact-report.md` (section "OWNER CORRECTIONS"). They override earlier text and do not affect this step:
- D2 stays TODO(owner-to-confirm, buddy 1).
- The RAM fault record needs a magic number plus a CRC.
- Keep the optional 2-bit sequence number in `obstacle_profile_t`, and document the drain-then-send pattern.
- The NFR2 wording change goes in ARCHITECTURE.md D7 only, as a proposal.

## Notes

- `testing/` and `videos/` are excluded from check_rules as the brief asked. Neither exists yet, and no fixture covers them; the self-test covers ProjectTemplate/ only.
- HEAD is `df67420`, not `79bc417` as the brief says. The extra commit touches only `INF 2004 Embedded Systems.md` and `NOTICE.md`, and ProjectTemplate/ is still tracked, so this step was not affected.
