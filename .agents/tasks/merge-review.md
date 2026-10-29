# Merge review (Stage 1 gate)

Self-review of the Stage 1 diff (M-01..M-17) against `merge-week8-plan.md`, the brief
(`merge-week8-brief.md`), HANDOVER section 5 and the OWNER CORRECTIONS. Evidence:
`merge-verify.md`.

## Checked

| Area | Finding |
|---|---|
| Never-edit files | `ProjectTemplate/`, `external/mtk3_bsp/`, `INF 2004 Embedded Systems.md`, `design.md`, `specs.md`: no change (`git status` empty, submodule clean at 15ed232). `README.md`, `.gitignore`: not touched |
| NOTICE.md | only appended; `owner: 16` kept |
| Names / IDs | only the five Team Information values, byte-exact; GitHub usernames `TODO(owner-to-confirm)` |
| Header mixing | kernel-side files include only `app.h` / `subsystemN.h` / `test_report.h` / `<tk/tkernel.h>` / `<stdint.h>` / `config/priorities.h`. Compile command for `app_program_check` has no SDK include path |
| Blocking calls, stdio, tm_printf | none in `app_program/` (`check_rules` PASS) |
| Bit-fields | none |
| Barr C | module prefixes, `_t`-free (no new types), `<stdint.h>` types in our interfaces, braces on every block, no tabs, Barr-style guards `APP_H`, `SUBSYSTEMN_H`, `TEST_REPORT_H`, `(void)` casts for ignored returns |
| TODO markers | platform `TODO(phase 1 batch B, buddy 2)`; subsystem logic `TODO(phase 2, buddy N)`; D2 `TODO(owner-to-confirm, buddy 1)` in subsystem1 and TEAM.md, never called approved; D1/D2 rows of ARCHITECTURE.md unchanged |
| usermain order | ipc_init TODO -> 5 x init -> 5 x start -> APP_TEST test task -> `tk_slp_tsk(TMO_FEVR)` |
| APP_TEST | option, configure guard, define, priority 11 + stack + count 12, `#error` deferral; Release + APP_TEST fails |
| CMake | repo-relative module dirs; guard covers `app_program/**` and `src/**`, never `ProjectTemplate/`; firmware block unchanged inside the `src/app/main.c` gate; both check targets in ALL with `-Werror` and `check_rules` dependency |
| Fixtures | five R100 renames staged, nothing else staged; two new fixtures; self-test PASS; 13 violations |
| Records | TEAM.md pins match the Week 8 list (W-01) with the P-8 interim line; priorities match `config/priorities.h`; limits match `config/mtk3/config.h`; no `cyw43_utk_task`, `TP_PRC`, `CRASH_REPORT` |
| Not started | no ipc module, port, dbg ring, `src/app/main.c`, packed types, encoder/PIO code |

## Findings fixed during review

1. `TEAM.md` named `cyw43_utk_task` in the Phase 2 TODO (plan verify forbids it). Changed to
   "cyw43_utk/lwIP/MQTT" as in plan M-12.
2. `cmake/check_rules.cmake` header comment had one over-long line. Reflowed; self-test and
   real-tree check re-run: PASS.
3. ARCHITECTURE.md T5 reason first cited NFR17 as if it were the reason for central creation.
   Reworded to the recorded owner decision.

## Open (not blocking, carried to the summary)

- The plan's HEAD `df67420` is outdated; HEAD is the owner's WIP commit `67f9b00`.
- `check_rules` scans 38 files (plan estimated 39).

Verdict: APPROVED.
