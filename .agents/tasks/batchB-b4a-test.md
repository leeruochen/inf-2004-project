# Batch B stage B4a: hardware test (for Buddy 2 and his AI agent)

Written 30 Oct 2026 by the Batch B helper (Buddy 3). Plan: `.agents/tasks/batchB-plan.md`, section B4.

## What this is

B4a is the first firmware of the project: SDK `main()`, micro T-Kernel start, the IPC objects, and the debug drain on USB CDC. It builds and links with 0 warnings on SDK 1.5.1 + GCC 10.3.1, and the linker layout was checked. **It has never run on a board.** This test is the first run.

What it proves, if it passes:
- the SDK hands over to the kernel;
- the first task switch works;
- the kernel tick is 1 ms;
- `ipc_init()` creates all 8 objects at their design depths;
- the debug console works.

Teammates can start running their own tasks only after this test and B4b pass (milestone M1-lite).

## Rules for the agent

- **Do not edit, commit or push anything.** This is a test only. Report the results; the helper fixes problems on the branch.
- Two steps need the person (marked **PERSON**): putting the board into BOOTSEL mode, and unplugging it. Ask, wait for "done", then continue.
- Run every command in PowerShell from the repo root.
- If any step fails, stop, and go to "Report" with what you have.

## Steps

### 1. Get the code (agent)

```powershell
git status --short              # must be empty (untracked session.md is fine); otherwise stop and ask
git fetch origin
git switch batchB/b4-first-firmware
git pull --ff-only
git log --oneline -3            # note the top hash for the report
git submodule status            # expect " 15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp"
```

### 2. Build (agent)

Use the HANDOVER.md section 5 step 3 environment block (sets `$R`, `$env:PATH`, `$env:PICO_SDK_PATH`, `$CM`, `$common`). Then, in a fresh build folder:

```powershell
if (Test-Path build/b4a) { Remove-Item -Recurse -Force build/b4a }
& $CM -S . -B build/b4a @common -DDBG_BACKEND=USB 2>&1 | Tee-Object build/b4a-configure.txt
& $CM --build build/b4a 2>&1 | Tee-Object build/b4a-build.txt
Select-String -Path build/b4a-build.txt -Pattern 'warning' | Measure-Object | Select-Object -ExpandProperty Count   # expect 0
Select-String -Path build/b4a-build.txt -Pattern 'check_rules: PASS'                                               # expect one match
Test-Path build/b4a/robocar.uf2                                                                                    # expect True
& "$R\gcc-arm-none-eabi\bin\arm-none-eabi-size.exe" build/b4a/robocar.elf                                          # note for the report
```

The configure output must NOT say `firmware target skipped` any more (B4a has `src/app/main.c`).

### 3. Flash (PERSON, then agent)

1. **PERSON:** unplug the Pico W, hold the BOOTSEL button, plug the USB cable in, release BOOTSEL. A drive called `RPI-RP2` appears. Say "done".
2. **Agent:** copy the firmware. Then START THE CAPTURE IMMEDIATELY (step 4), because the board reboots at once:

```powershell
$v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
if (-not $v) { throw "RPI-RP2 drive not found" }
Copy-Item build/b4a/robocar.uf2 "$($v.DriveLetter):\"
```

### 4. Capture 150 s of console output (agent)

```powershell
powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 150 -Out build/b4a-console.txt
```

If a terminal (PuTTY, VS Code serial monitor) has the COM port open, close it first: only one program can open the port.

### 5. Check the log (agent)

Expected lines in `build/b4a-console.txt`, in this order. The `HH:mm:ss.fff` timestamp at the front is added by the capture script.

| # | Expected | Pass rule |
|---|---|---|
| 1 | `pre-kernel ok` | Present. It may be missing only if the capture started more than 3 s after the reboot (the line is printed before the kernel and not stored); then rerun step 3-4 faster. Not a failure by itself if line 2 appears. |
| 2 | `microT-Kernel Version 3.00` | Present (the kernel banner, via the system-message ring) |
| 3 | `[IPC] kernel spec version 3.00 OK` | Present |
| 4 | `[IPC] 8 objects created, MBF depth self-check OK` | Present |
| 5 | `[DBG] alive: SDK timer <a> ms, kernel time <b> ms, difference <d> ms` | One about every 10 s, so about 13-14 lines in 150 s, with no gap over 12 s |
| 6 | the `difference` value | Constant: max minus min over all alive lines is at most 2 ms (it is the boot delay, about 0-3000 ms, and must not grow) |
| 7 | no `[ERR]` line, no `[FAULT]` line, no `[DBG] dropped` line | None |
| 8 | no `[capture] port lost` line | None (the board must not reset by itself) |

Commands that help:

```powershell
Select-String build/b4a-console.txt -Pattern 'pre-kernel ok|microT-Kernel|\[IPC\]|\[ERR\]|\[FAULT\]|dropped|port lost'
Select-String build/b4a-console.txt -Pattern 'difference (-?\d+) ms' | ForEach-Object { [int]$_.Matches[0].Groups[1].Value } | Measure-Object -Minimum -Maximum -Count
```

### 6. Unplug and reconnect (PERSON + agent)

1. Agent: start a 60 s capture: `powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 60 -Out build/b4a-replug.txt`
2. **PERSON** (while it runs): unplug the USB cable, wait 5 s, plug it back in (no BOOTSEL).
3. Pass: after `[capture] reconnected`, the boot lines 2-4 appear again (unplugging removes power, so the board restarts), then alive lines. No `[FAULT]` line: a power cycle must never show a fault record (random RAM is rejected by the CRC).

## If it fails: what to look at

| Symptom | Most likely place | What to collect |
|---|---|---|
| No COM port at all (capture exits with code 2) | before the kernel: `src/app/main.c`, USB init | Device Manager: is there an unknown device? Does `RPI-RP2` come back on its own (a crash loop)? |
| `pre-kernel ok` but no banner | kernel start: `hal_kernel_start()` (`src/rtos/port/port_boot.c`), first dispatch | With the Debug Probe: `arm-none-eabi-gdb build/b4a/robocar.elf`, `target extended-remote :3333`, `monitor reset halt`, `continue`, then Ctrl-C and `bt`, `info registers`, `print knl_taskindp` |
| Banner but no `[IPC]` lines | init task, `ipc_init()`, or the drain task start | gdb as above; `print s_record` (fault record), `print ipc_mbf_nav_in` |
| `[ERR] ...` line | the named call failed; the number is the kernel error code | the full line |
| `[FAULT] ...` line on first boot | a crash happened, then a reset | the full line (pc, lr); then `arm-none-eabi-addr2line -e build/b4a/robocar.elf <pc>` |
| `difference` drifts | kernel tick is not 1 ms (clock) | the alive lines; look for a `port: clk_sys is not 125 MHz` line |
| `[capture] port lost` with no unplugging | a reset loop (watchdog after a fault) | the whole log |

## Report (post on the B4a PR, and give it to the helper)

```text
B4a hardware test, <date>, by <name>
Commit: <hash from step 1>
Toolchain: SDK 1.5.1 installer, GCC 10.3.1
Build: warnings=<n>, check_rules=<PASS/FAIL>, size: text=<n> bss=<n>
Flash: OK / problem: ...
Expected lines 1-8: <PASS/FAIL each>
alive lines: <count>, difference min/max: <a>/<b> ms
Replug test: PASS/FAIL
Verdict: PASS / FAIL
Log: (attach build/b4a-console.txt and build/b4a-replug.txt)
```

If all pass, the helper moves on to B4b (the self-test build: forced fault, reset, fault record printed once).
