# Batch B stage B7: packed formats, 2-minute re-run → M2 (for Buddy 2 and his AI agent)

Written 31 Oct 2026 by the Batch B helper (Buddy 3). Plan: `.agents/tasks/batchB-plan.md`, section B7. Results file: `docs/measurements/week8-packed.txt`.

## What this is

B7 switches the message types to the packed minimum-bit formats (`src/common/data_repr.h`, `src/common/ipc_types.h`). The helper has already run:
- the host tests (every field at its limits, sign extension, the edge ring, the MQTT frames and their reject rules);
- the firmware gates.

What is left is the plan's board check: a **short B5 re-run (2 minutes)**. It confirms that the platform still boots and runs with the new types. The `[MBF]` report also shows the smaller message buffers. After it passes: **milestone M2**, the message formats are final.

The firmware's code size does not change. Nothing calls the pack functions yet, and the buffers live inside the kernel pool. So the run should look exactly like B6, except for the four `[MBF]` lines.

## Rules for the agent

Same as before:
- Never edit, commit or push.
- Steps marked **PERSON** need the person.
- Run commands from the repo root in PowerShell.
- On a failure, stop and report.
- About 10 minutes.

## Steps

### 1. Get the code (agent)

```powershell
git status --short              # must be empty (untracked session.md is fine)
git fetch origin
git switch batchB/b7-packed
git pull --ff-only
git log --oneline -3            # note the top hash
git submodule status            # expect " 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp"
```

### 2. Build the bring-up firmware, and one size check (agent)

Run the HANDOVER.md section 5 step 3 environment block first. Then:

```powershell
if (Test-Path build/b7-bu) { Remove-Item -Recurse -Force build/b7-bu }
& $CM -S . -B build/b7-bu @common -DDBG_BACKEND=USB -DPLATFORM_BRINGUP=ON 2>&1 | Tee-Object build/b7-configure.txt
& $CM --build build/b7-bu 2>&1 | Tee-Object build/b7-build.txt
Select-String -Path build/b7-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/b7-build.txt -Pattern 'check_rules: PASS'                                               # expect one match
& "$R\gcc-arm-none-eabi\bin\arm-none-eabi-nm.exe" -S --size-sort build/b7-bu/CMakeFiles/size_report.dir/cmake/size_report.c.obj
```

The `nm` sizes are in hex. Expect:

| Symbol | Hex | Bytes |
|---|---|---|
| `mbf_motion_cmd` | `20` | 32 |
| `mbf_obstacle` | `18` | 24 |
| `mbf_telemetry` | `180` | 384 |
| `edge_ring` | `a0` | 160 |

Compare the whole list with section A1 of `docs/measurements/week8-packed.txt`.

### 3. Flash and capture 3 minutes (PERSON, then agent)

1. **PERSON:** unplug the Pico W, hold BOOTSEL, plug it in, release BOOTSEL. Say "done".
2. **Agent:** copy `build/b7-bu/robocar.uf2` to `RPI-RP2` (as in the B5 guide), then at once:

```powershell
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 180 -Out build/b7-run.txt
powershell -ExecutionPolicy Bypass -File tools/bringup_check.ps1 -Log build/b7-run.txt -MinMinutes 2 -NoStress
```

`-NoStress` is needed because a 3-minute run ends before the stress window (660 s). Expected: `bringup_check: PASS`.

### 4. Read the last report (agent)

```powershell
Select-String build/b7-run.txt -Pattern '\[MBF\]|\[STACK\]' | Select-Object -Last 16
```

| # | Expected | Pass rule |
|---|---|---|
| 1 | `[MBF] nav_in: 64 B buffer, 64 B free`, `motion_cmd: 32 ... 32`, `obstacle: 24 ... 24`, `telemetry: 384 ... 384` | exactly these sizes (B6 was 64 / 64 / 40 / 640) |
| 2 | `[STACK] init`, `dbg_drain` and the nine heartbeat lines | none with ` LOW`; close to the B6 numbers (`docs/measurements/week6-baseline.txt` B1) |
| 3 | `[STACK] stack top check: 0 mismatches` | exactly 0 |

## Report (post on the B7 PR, and give it to the helper)

```text
B7 packed formats, 2-minute re-run, <date>, by <name>
Commit: <hash>
Build: warnings=<n>, check_rules=<PASS/FAIL>; size_report equal to week8-packed.txt A1: <yes / differences>
Run (b7-run.txt): bringup_check -MinMinutes 2 -NoStress <PASS/FAIL>
Last report: (paste the [STACK] block and the 4 [MBF] lines)
Verdict: PASS / FAIL
Log: (attach build/b7-run.txt)
```

If it passes, the helper:
- records the run;
- merges the PR, which declares **M2** in HANDOVER.
