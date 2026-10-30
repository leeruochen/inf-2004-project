# Batch B: what Ruo Chen (Buddy 2) needs to do, step by step

Written 30 Oct 2026 by Jin Yong (Buddy 3, the Batch B helper). This is the **one guide to follow**: it covers every task, in order, and points to the two hardware test guides at the right moments. Nothing else is needed.

Repo: `https://github.com/leeruochen/inf-2004-project`. All Batch B branches are pushed there.

---

## 0. The short version

| Part | What | Time | Needs the Pico W? |
|---|---|---|---|
| A | One-time setup check | 5 min | no |
| B | Confirm the decisions on the B0 PR, merge B0 | 10 min | no |
| C | Hardware test B4a (first firmware) | 15 min | yes |
| D | Hardware test B4b (self-test) | 15 min | yes |
| E | Send the results; merge B1-B3, B4a, B4b in order | 10 min | no |
| F | Tell the team (milestone M1-lite) | 2 min | no |
| G | Answer 4 open questions (any time) | 5 min | no |

Stop at the first part that fails and send the result to Jin Yong (part E explains what to send). Do not try to fix code: Jin Yong fixes it on the branch and you re-run that part.

---

## Using your AI agent

You can give this whole guide to your AI agent. Paste this prompt:

```text
Read .agents/tasks/batchB-ruochen-guide.md (on branch batchB/b4b-selftest of this repo) and carry
it out with me, part by part, in order. Rules:
- Never edit, commit or push any file in the repo.
- For steps marked PERSON, tell me exactly what to do and wait for me to say "done".
- For steps marked DECISION, show me the question and the recommended answer, and wait for my answer.
- Run every command in PowerShell from the repo root, show me the output, and check it against the
  expected result written in the guide.
- Stop at the first failure and help me write the report from part E.
```

To read the guide before switching branches: open it on GitHub, branch `batchB/b4b-selftest`, file `.agents/tasks/batchB-ruochen-guide.md`.

---

## Part A: one-time setup check (agent)

A1. **Toolchain.** The Pico SDK 1.5.1 Windows installer must be installed at `C:\Program Files\Raspberry Pi\Pico SDK v1.5.1` (HANDOVER.md section 5, step 1). Check:

```powershell
Test-Path "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe"   # expect True
& "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe" --version | Select-Object -First 1
# expect: arm-none-eabi-gcc.exe (GNU Arm Embedded Toolchain 10.3-2021.10) 10.3.1 ...
```

A2. **No uncommitted work in the repo.** The tests switch branches.

```powershell
git status --short
```

If anything other than untracked `session.md` is listed, **PERSON**: commit or stash your own work first (`git stash push -m "before batchB tests"`), and restore it at the end with `git stash pop`.

A3. **Get all branches.**

```powershell
git fetch origin
git branch -r | Select-String 'batchB'
# expect: origin/batchB/b0-records, origin/batchB/b1-platform-code,
#         origin/batchB/b4-first-firmware, origin/batchB/b4b-selftest
```

A4. **Environment block.** Every later part that builds needs this, in the same PowerShell window (HANDOVER.md section 5, step 3):

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

If PowerShell is closed, run A4 again before building.

A5. **Hardware at hand (PERSON).** You need:
- the Pico W;
- a USB data cable (not charge-only);
- no motors or other wiring needed;
- the Debug Probe, only if a test fails and you want to look inside.

Close any terminal that has the Pico's COM port open (PuTTY, Arduino, VS Code serial monitor).

---

## Part B: the B0 PR (decisions), then merge B0

B0 is documents only: it records decisions and the Batch B plan. Your earlier review (`.agents/tasks/batchB-b0-review-report.md`) passed the build and asked for fixes. Those fixes are in commit `af53004` (plan Rev 6).

B1. On GitHub, open the PR for branch `batchB/b0-records` → `main`. If there is no PR yet, create one (base `main`, compare `batchB/b0-records`).

B2. **DECISION.** Confirm or change each item below, in a PR comment. The full text is in `.agents/tasks/batchB-plan.md` sections 0 and 1. Recommended answer: confirm all.

| # | Decision | Recommended |
|---|---|---|
| C1 | Jin Yong is the Batch B helper (your verbal OK, recorded in writing) | confirm |
| C7 | Workflow: Jin Yong writes each stage and runs the compile checks with SDK 1.5.1; you review, merge, and run every hardware test | confirm |
| C8 | Toolchain stays Pico SDK 1.5.1 + GCC 10.3.1 for everyone; no SDK migration | confirm |
| O1 | Temporary heartbeat tasks behind a `PLATFORM_BRINGUP` option (default OFF) for the B5 USB test | confirm |
| O2 | Platform writes the message structs in `src/common/ipc_types.h`: Week 6 fields now, Week 8 packed fields in B7 | confirm |
| O4 | Fault record: magic `0x464C5452`, 7 words, CRC-32 | confirm |
| O5/O6 | Ring sizes: 512 B kernel text, 2 KB debug log | confirm |
| O7 | After a crash: record it, then reset with the watchdog; halt only if a RAM flag is set from the debugger | confirm |
| O8 | Task stacks stay kernel-allocated; painted by a start-up trampoline for the stack report | confirm |
| O9 | `ipc.h` and `stack_mon.h` count as kernel-side headers in the include rule | confirm |

Suggested comment:

```text
Confirmed: C1, C7, C8, O1, O2, O4, O5, O6, O7, O8, O9 as in batchB-plan.md Rev 6. - Ruo Chen
```

B3. **Merge B0.** On the PR, choose **"Create a merge commit"**. **Do NOT use "Squash and merge" or "Rebase and merge"** on any Batch B PR: the later branches are built on top of these exact commits, and squashing or rebasing would make their PRs show duplicate changes and conflicts.

---

## Part C: hardware test B4a (the first firmware)

C1. **Follow `.agents/tasks/batchB-b4a-test.md` from top to bottom.** It is on branch `batchB/b4-first-firmware` and covers:
- getting the code;
- the build;
- flashing (**PERSON**: BOOTSEL);
- a 150 s console capture;
- the checks;
- an unplug test (**PERSON**);
- the report block.

C2. What PASS looks like, in short: `pre-kernel ok`, then `microT-Kernel Version 3.00`, then `[IPC] kernel spec version 3.00 OK` and `[IPC] 8 objects created, MBF depth self-check OK`, then a `[DBG] alive ... difference <d> ms` line every 10 s with `<d>` not changing. No `[ERR]`, no `[FAULT]`.

C3. If B4a FAILS: stop here, go to part E and send the report. Do not run part D.

---

## Part D: hardware test B4b (the self-test), only if B4a passed

D1. **Follow `.agents/tasks/batchB-b4b-test.md` from top to bottom.** It is on branch `batchB/b4b-selftest`. The board resets **twice by itself** in this test: that is the point (a forced crash, then a plain reset). Exactly one `[FAULT]` line must appear.

D2. After the test, put normal firmware back on the board so nobody is surprised by the self-test later: flash `build/b4a/robocar.uf2` from part C again (**PERSON**: BOOTSEL, then copy the file to `RPI-RP2`).

---

## Part E: send the results, then merge in order

E1. **Send to Jin Yong** (chat or PR comment) for each test you ran:
- the report block from the test guide, filled in;
- the log files: `build/b4a-console.txt`, `build/b4a-replug.txt`, and for B4b `build/b4b-console.txt`, `build/b4b-powercycle.txt`;
- if a build failed: `build/b4a-build.txt` or `build/b4b-build.txt`.

E2. **Only if the tests passed**, merge the code PRs, **in this order**, each with **"Create a merge commit"**:

| Order | PR | Base | Contents | Review focus |
|---|---|---|---|---|
| 1 | `batchB/b1-platform-code` | `main` | B1 debug log ring, B2 IPC objects + task start, B3 kernel port layer (3 commits) | shared files changed: `app_program/app.h`, `app_main.c`, `test_report.h`, `CMakeLists.txt`, `cmake/check_rules*.cmake` |
| 2 | `batchB/b4-first-firmware` | `batchB/b1-platform-code` (GitHub retargets it to `main` after PR 1 merges) | B4a: `src/app/main.c`, debug drain, test guide, capture script | `main.c`, `dbg_drain.c` |
| 3 | `batchB/b4b-selftest` | `batchB/b4-first-firmware` | B4b: `PLATFORM_SELFTEST` option | default OFF; normal firmware unchanged |

If a PR does not exist yet, create it with the base shown. Each PR description and commit message lists what changed. The plan has the full result of every stage: `.agents/tasks/batchB-plan.md`, section 3, "B1 result" to "B4b result".

E3. If you have review comments, write them on the PR instead of merging; Jin Yong fixes them on the same branch and the tests do not need to be repeated unless the firmware code changed.

E4. Restore your own work if you stashed it in A2: `git switch main; git pull; git stash pop`.

---

## Part F: tell the team (after B4a and B4b passed and are merged)

F1. Post to the team (suggested text):

```text
Batch B milestone M1-lite: the platform boots on the Pico W. From main you can now:
- start your tasks with app_start_task(entry, TASK_PRI_x, TASK_STACK_x_BYTES, "name") in subsystemN_start();
- use the IPC objects by their TEAM.md names (MBF_NAV_IN, EVF_SAFETY, ...) via app.h;
- log with dbg_puts / dbg_printf (never printf), and register ISRs with hal_irq_register /
  hal_gpio_irq_register (src/rtos/port/hal.h);
- build APP_TEST=ON and see your [TEST] lines on the USB console.
Include app.h (or your subsystemN.h) FIRST in kernel-side files.
Message structs in src/common/ipc_types.h are PROVISIONAL until milestone M2 (stage B7).
Console and debugging: docs/DEBUG.md.
```

---

## Part G: open questions (DECISION; answer any time, in a PR comment or chat)

None of these blocks the tests. Recommended answers in brackets; the full text is in `.agents/tasks/batchB-plan.md` section 1.

| # | Question | Recommended |
|---|---|---|
| Q3 | Will you write any Batch B code yourself while Jin Yong does? | No; you may push small fix commits to a branch if a hardware test needs it |
| Q7 | Run the 10-minute USB test (stage B5) while teammates already work on their parts? | Yes (tell the team) |
| Q8 | Fault record: add the stack pointer and task ID; drop the boot count? | Add SP and task ID; keep the reset count |
| Q9 | Add a watchdog later to recover from a stack overflow lock-up? | Yes, as a follow-up after B5 |

---

## What comes after (no action now)

| Stage | Jin Yong writes | You will be asked to |
|---|---|---|
| B5 | 9 temporary heartbeat tasks (`PLATFORM_BRINGUP`) | run the mandatory 10-minute USB test from `docs/DEBUG.md` and commit the log to `testing/system/logs/` (about 30 min) → milestone M1 |
| B6 | size and stack reports | run them, send the numbers, push the tag `baseline/week6-unpacked` (about 20 min) |
| B7 | packed message types + host tests | a 2-minute re-run → milestone M2. Needs team decisions first (Buddy 5: sequence number and bypass command; Buddy 3: steering sign) |

Each stage will come with its own test guide like the B4a and B4b ones.
