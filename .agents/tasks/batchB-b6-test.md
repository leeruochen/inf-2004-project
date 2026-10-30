# Batch B stage B6: Week 6 baseline measurement + tag (for Buddy 2 and his AI agent)

Written 31 Oct 2026 by the Batch B helper (Buddy 3). Plan: `.agents/tasks/batchB-plan.md`, section B6. Results file: `docs/measurements/week6-baseline.txt`.
**Run this after B5 passed** (`.agents/tasks/batchB-b5-test.md`). It can run in the same sitting.

## What this is

B6 records the "before" numbers for the data-representation work: the Week 6 **unpacked** message types, measured. B7 then packs the types and measures again with the same commands. Nothing here needs subsystem code: it measures the platform alone.

There are two parts:
- **Part A, compile time.** Type and buffer sizes, plus firmware sections. The helper has already filled these in `docs/measurements/week6-baseline.txt`. Your build must give the same numbers, because the toolchain is the same.
- **Part B, run time.** Stack high-water marks for every task, including the whole 1 KB kernel init stack (new in B6), and message-buffer free space.
  - Every debug build now prints a report every 60 s: `[STACK] <name>: <used> of <size> B used, <free> never used` and `[MBF] <name>: ...`.
  - The baseline is the **last** report of an 800 s bring-up run, because that run includes the stress window.

After it passes:
- the helper fills part B into the results file and merges the PR;
- then you push the tag `baseline/week6-unpacked` (step 6).

## Rules for the agent

Same as B4 and B5:
- Never edit, commit or push, except the tag in step 6, which the person runs.
- Steps marked **PERSON** need the person.
- Run commands from the repo root in PowerShell.
- On a failure, stop and report.
- About 20 minutes.

## Steps

### 1. Get the code (agent)

```powershell
git status --short              # must be empty (untracked session.md is fine)
git fetch origin
git switch batchB/b6-baseline
git pull --ff-only
git log --oneline -3            # note the top hash
git submodule status            # expect " 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp"
```

### 2. Build three configurations, fresh (agent)

Run the HANDOVER.md section 5 step 3 environment block first. Then:

```powershell
foreach ($d in 'build/b6','build/b6-rel','build/b6-bu') { if (Test-Path $d) { Remove-Item -Recurse -Force $d } }
& $CM -S . -B build/b6     @common -DDBG_BACKEND=USB                          2>&1 | Tee-Object build/b6-configure.txt
& $CM -S . -B build/b6-rel @common -DCMAKE_BUILD_TYPE=Release                 2>&1 | Tee-Object build/b6-rel-configure.txt
& $CM -S . -B build/b6-bu  @common -DDBG_BACKEND=USB -DPLATFORM_BRINGUP=ON    2>&1 | Tee-Object build/b6-bu-configure.txt
foreach ($d in 'b6','b6-rel','b6-bu') { & $CM --build "build/$d" 2>&1 | Tee-Object "build/$d-build.txt" }
foreach ($d in 'b6','b6-rel','b6-bu') { "$d warnings: " + (Select-String -Path "build/$d-build.txt" -Pattern 'warning' | Measure-Object).Count }   # expect 0 each
Select-String -Path build/b6-build.txt -Pattern 'check_rules: PASS'                                                 # expect one match
```

`@common` sets Debug. The Release line overrides it, which works because the later `-D` wins.

### 3. Part A: compare with the results file (agent)

```powershell
$B = "$R\gcc-arm-none-eabi\bin"
& "$B\arm-none-eabi-nm.exe" -S --size-sort build/b6/CMakeFiles/size_report.dir/cmake/size_report.c.obj
& "$B\arm-none-eabi-size.exe" -A build/b6/robocar.elf
& "$B\arm-none-eabi-size.exe" build/b6/robocar.elf build/b6-rel/robocar.elf build/b6-bu/robocar.elf
```

`nm` prints sizes in hex: for example `00000280` is 640. Compare every number with sections A1 and A2 of `docs/measurements/week6-baseline.txt`.
- **Pass:** all equal.
- **If anything differs:** list which numbers differ, but do not stop. The helper decides; a different `.text` usually means a different toolchain.

### 4. Part B: flash the bring-up build and capture 800 s (PERSON, then agent)

1. **PERSON:** unplug the Pico W, hold BOOTSEL, plug it in, release BOOTSEL. Say "done".
2. **Agent:** copy `build/b6-bu/robocar.uf2` to `RPI-RP2` (as in the B5 guide), then **at once**:

```powershell
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 800 -Out build/b6-run.txt
powershell -ExecutionPolicy Bypass -File tools/bringup_check.ps1 -Log build/b6-run.txt     # expect PASS (B5 rules still hold)
```

### 5. Read the last report (agent)

```powershell
$lines = Get-Content build/b6-run.txt
$last  = ($lines | Select-String '\[STACK\] stack top check' | Select-Object -Last 1).LineNumber
$first = $last; while ($first -gt 1 -and $lines[$first - 2] -match '\[STACK\] ') { $first-- }
$lines[($first - 1)..($last + 3)]     # the last [STACK] block and the 4 [MBF] lines after it
```

Expected, and the pass rules:

| # | Line | Pass rule |
|---|---|---|
| 1 | `[STACK] init: <u> of 1152 B used, <f> never used` | the name is `init`, not `init (usermain window)`, so the kernel stack top was found; `<f>` ≥ 128 (no ` LOW`) |
| 2 | `[STACK] dbg_drain: ...` and one line each for the nine heartbeat tasks (`safety` ... `tel`) | 10 lines, none with ` LOW` |
| 3 | `[STACK] stack top check: 0 mismatches` | exactly 0 |
| 4 | `[MBF] nav_in: 64 B buffer, 64 B free`, `motion_cmd: 64 ... 64`, `obstacle: 40 ... 40`, `telemetry: 640 ... 640` | buffer sizes equal A1; free = buffer (nothing uses them in this build) |

**Stop if:**
- a stack shows ` LOW`, i.e. less than 128 B never used (the plan's B6 stop rule);
- the init line says `usermain window`;
- the mismatch count is not 0.

### 6. After the helper merged the PR: push the tag (PERSON)

The helper tells you the merge commit. Then:

```powershell
git fetch origin
git switch main; git pull --ff-only
git log --oneline -1                     # must be the B6 merge commit the helper named
git tag -a baseline/week6-unpacked -m "Week 6 unpacked baseline (Batch B B6): docs/measurements/week6-baseline.txt"
git push origin baseline/week6-unpacked
```

## Report (post on the B6 PR, and give it to the helper)

```text
B6 baseline measurement, <date>, by <name>
Commit: <hash>
Build: warnings b6/b6-rel/b6-bu = <n>/<n>/<n>, check_rules=<PASS/FAIL>
Part A equal to the results file: <yes / list the differences>
Bring-up run (b6-run.txt): bringup_check <PASS/FAIL>
Last report: (paste the [STACK] block and the 4 [MBF] lines)
Verdict: PASS / FAIL
Log: (attach build/b6-run.txt)
```
