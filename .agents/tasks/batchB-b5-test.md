# Batch B stage B5: USB CDC bring-up test, milestone M1 (for Buddy 2 and his AI agent)

Written 31 Oct 2026 by the Batch B helper (Buddy 3). Plan: `.agents/tasks/batchB-plan.md`, section B5. Test definition: `testing/system/TEST_LOG.md`, **SYS-01**. Pass rules: `docs/DEBUG.md`, "USB CDC bring-up test".

## What this is

The mandatory Phase 1 acceptance test of the platform. The build option `PLATFORM_BRINGUP=ON` (Debug only) replaces the subsystems' tasks with nine heartbeat tasks:

| id | task | priority | period |
|---|---|---|---|
| 1 | safety | 1 | 50 ms (IMU watchdog) |
| 2 | vc | 2 | 20 ms (stand-in; event-driven in the design) |
| 3 | motion | 3 | 20 ms |
| 4 | lf | 4 | 5 ms |
| 5 | bc | 5 | 2 ms (barcode poll) |
| 6 | obs_scan | 6 | 60 ms |
| 7 | obs_avoid | 7 | 100 ms (stand-in; event-driven) |
| 8 | imu | 8 | 10 ms |
| 9 | tel | 9 | 20 ms |

Each task uses its design priority and stack size. Once a second it prints one line:

```text
[HB] id=4 lf seq=12 cycles=200 period=5000 dt_min=4990 dt_max=5012 missed=0
```

- `dt_min`/`dt_max` are the shortest and longest times between two releases over that second, in µs.
- `seq` counts that task's lines.

**The stress step runs by itself.** From 660 s to 720 s after boot, every task logs 10× more lines. The log shows `[BRINGUP] stress ON` and `[BRINGUP] stress OFF` around it. The 2 KB log ring must overflow during this window (`[DBG] dropped so far` lines) without slowing the tasks.

`tools/bringup_check.ps1` checks a log against every pass rule, so the agent does not judge timing by eye.

What it proves, if it passes:
- the kernel holds nine tasks' periods within 1 ms for 10 minutes;
- log bursts are dropped, never block a task;
- the console survives closing and reopening the terminal, and a USB replug.

After it passes: **milestone M1**.

## Rules for the agent

Same as B4:
- Never edit, commit or push.
- Steps marked **PERSON** need the person.
- Run commands from the repo root in PowerShell.
- On a failure, stop and report.
- **Total time:** about 20 minutes, mostly waiting.
- **The console must stay connected:** during the 800 s capture nothing else may open the COM port, and the PC must not sleep.

## Steps

### 1. Get the code (agent)

```powershell
git status --short              # must be empty (untracked session.md is fine)
git fetch origin
git switch batchB/b5-bringup
git pull --ff-only
git log --oneline -3            # note the top hash
git submodule status            # expect " 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp"
```

### 2. Build the bring-up firmware (agent)

Run the HANDOVER.md section 5 step 3 environment block first. Then:

```powershell
if (Test-Path build/b5) { Remove-Item -Recurse -Force build/b5 }
& $CM -S . -B build/b5 @common -DDBG_BACKEND=USB -DPLATFORM_BRINGUP=ON 2>&1 | Tee-Object build/b5-configure.txt
& $CM --build build/b5 2>&1 | Tee-Object build/b5-build.txt
Select-String -Path build/b5-configure.txt -Pattern 'PLATFORM_BRINGUP=ON'                                         # expect one match
Select-String -Path build/b5-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/b5-build.txt -Pattern 'check_rules: PASS'                                               # expect one match
Test-Path build/b5/robocar.uf2                                                                                    # expect True
```

### 3. Flash (PERSON, then agent)

1. **PERSON:** unplug the Pico W, hold BOOTSEL, plug it in, release BOOTSEL. `RPI-RP2` appears. Say "done".
2. **Agent:** copy the firmware, then start step 4 **at once**. The stress window is timed from boot, so a late start can miss it.

```powershell
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
if (-not $v) { throw "RPI-RP2 drive not found" }
Copy-Item build/b5/robocar.uf2 "$($v.DriveLetter):\"
```

### 4. Main run: capture 800 s (agent)

This covers steps 1-3 of DEBUG.md: boot, 10 minutes, then the stress window at 660-720 s.

```powershell
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 800 -Out build/b5-run.txt
powershell -ExecutionPolicy Bypass -File tools/bringup_check.ps1 -Log build/b5-run.txt
```

Expected: `bringup_check: PASS`, and every task has at least 600 s of heartbeats. If it says FAIL, stop and report; each FAIL line names the rule (F1-F8 at the top of the script) and the log line.

### 5. Terminal close/reopen and USB replug (PERSON + agent)

The step 4 capture has ended, so the terminal is closed now. The board keeps running and its log ring fills up.

1. Agent: wait 30 s, then start `powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 150 -Out build/b5-replug.txt`. This reopens the terminal.
2. **PERSON** (while it runs, after about 60 s): unplug USB, wait 5 s, plug it in again (no BOOTSEL). The board restarts.
3. Agent, after the capture ends: `powershell -ExecutionPolicy Bypass -File tools/bringup_check.ps1 -Log build/b5-replug.txt -Replug`

Expected in `build/b5-replug.txt`:
- **At the start:** heartbeat lines continue with high `seq` numbers, with no new kernel banner. So the board kept running while the terminal was closed. Older lines and a `[DBG] dropped so far` line come first; that is expected.
- **After the replug:** `[capture] reconnected`, then the boot lines, then heartbeats from `seq=1`.
- **Checker:** `bringup_check: PASS`.

## If it fails: what to look at

| Symptom | Most likely place | What to collect |
|---|---|---|
| No `[HB]` lines | configure did not set `PLATFORM_BRINGUP=ON`, or an `[ERR] dbg_bringup_start` line (a task could not start) | `build/b5-configure.txt`, the log |
| F3 on a control task (id 1-4) | something holds interrupts off too long (PRIMASK: SysTick timer queue, `tm_putstring`), or a USB interrupt storm | the log; do not change priorities before the helper looks |
| F3 only on id 5-9, only in stress | a WARN, not a FAIL: the low-priority tasks wait behind the burst | nothing |
| F4 (gap with no drop counted) | a lost line outside the ring's accounting | the whole log |
| F5 (overflow outside stress) | the drain cannot keep up with normal logging (USB slow, or terminal flow control) | the whole log; which program held the port |
| F1/F2 (`[FAULT]`, reboot) | a crash | the whole log; `arm-none-eabi-addr2line -f -e build/b5/robocar.elf <pc>` |
| F7 "no stress ON line" | the capture started late or ended early | rerun steps 3-4, starting the capture at once |

## Report (post on the B5 PR, and give it to the helper)

```text
B5 USB CDC bring-up test (SYS-01), <date>, by <name>
Commit: <hash>
Build: warnings=<n>, check_rules=<PASS/FAIL>, PLATFORM_BRINGUP=ON confirmed
Main run (b5-run.txt): bringup_check <PASS/FAIL>; worst |dt - period| per task: <paste the 9 summary lines>
Stress: ON/OFF lines seen <yes/no>; dropped count before/after: <a>/<b>
Replug (b5-replug.txt): bringup_check <PASS/FAIL>; kept running while closed <yes/no>; restarted after replug <yes/no>
Verdict: PASS / FAIL
Logs: (attach build/b5-run.txt and build/b5-replug.txt)
```

If it passes: **milestone M1**. The helper then:
- commits both logs to `testing/system/logs/`;
- adds the SYS-01 result row to `testing/system/TEST_LOG.md`;
- adds the result to the plan;
- merges the PR, with your OK on file (plan C7).
