# testing/subsystem4/s4_test.ps1 - Subsystem 4 (Buddy 4): build, flash and record the
# APP_TEST firmware with the operator-guided tests S4-U14..U16 and S4-U18..U20 (S4_MANUAL_TESTS=ON).
# Needs the Pico SDK v1.5.1 Windows installer (HANDOVER.md section 5).
# Run from the repo root:
#   powershell -ExecutionPolicy Bypass -File testing\subsystem4\s4_test.ps1
# Options:
#   -SkipBuild        flash the existing firmware instead of rebuilding.
#   -AutoOnly         unattended run: build WITHOUT the operator-guided tests (only
#                     S4-U01..U12, in build/s4auto), wait for the robot to be plugged in,
#                     flash it without BOOTSEL if it already runs USB-console firmware,
#                     and stop by itself after the summary line. Exit code 0 = all passed.
#   -WaitMinutes 10   how long to wait for the robot to be plugged in.
# Stop: Ctrl+C.
param(
    [string]$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path,
    [switch]$SkipBuild,
    [switch]$AutoOnly,
    [int]$WaitMinutes = 10
)

$R = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"
$env:PATH = "$R\cmake\bin;$R\gcc-arm-none-eabi\bin;$R\ninja;$R\python;$R\git\cmd;$env:PATH"
$env:PICO_SDK_PATH = "$R\pico-sdk"
$CM = "$R\cmake\bin\cmake.exe"
$common = @("-G", "Ninja", "-DCMAKE_BUILD_TYPE=Debug",
    "-DPICO_SDK_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk",
    "-Dpico-sdk-tools_DIR=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk-tools",
    "-DCMAKE_MAKE_PROGRAM=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe",
    "-DPICO_TOOLCHAIN_PATH=C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi")

function Step($text) { Write-Host ""; Write-Host "== $text" -ForegroundColor Cyan }
function Ask($text)  { Write-Host ""; Write-Host ">> YOU: $text" -ForegroundColor Yellow }
function Stop-Here($text) { Write-Host ""; Write-Host "STOPPED: $text" -ForegroundColor Red; exit 1 }

if (-not (Test-Path "$R\pico-sdk")) { Stop-Here "Pico SDK v1.5.1 not found at $R" }
if (-not (Test-Path $Repo)) { Stop-Here "Repo not found at $Repo" }
Set-Location $Repo

$commit = (git rev-parse --short HEAD)
$branch = (git rev-parse --abbrev-ref HEAD)
Step "Repo $Repo on $branch at $commit"

# --- 1. Build ---------------------------------------------------------------
if ($AutoOnly) { $bdir = "build/s4auto"; $manual = "OFF" } else { $bdir = "build/s4test"; $manual = "ON" }
$uf2 = Join-Path $Repo ($bdir.Replace('/', '\') + "\robocar.uf2")
if (-not $SkipBuild) {
    Step "1/4 Building the test firmware (APP_TEST=ON, S4 guided tests $manual) in $bdir"
    # A build folder configured with CMAKE_C_FLAGS by an earlier script lost the
    # toolchain's CPU flags; start that one fresh.
    $cache = "$bdir/CMakeCache.txt"
    if ((Test-Path $cache) -and (Select-String -Path $cache -Pattern '^CMAKE_C_FLAGS:STRING=-DS4' -Quiet)) {
        Write-Host "Removing the old $bdir folder (wrong compiler flags)" -ForegroundColor DarkYellow
        Remove-Item -Recurse -Force $bdir
    }
    & $CM -S . -B $bdir @common -DDBG_BACKEND=USB -DAPP_TEST=ON "-DS4_MANUAL_TESTS=$manual"
    if ($LASTEXITCODE -ne 0) { Stop-Here "CMake configure failed (scroll up for the first error)" }
    & $CM --build $bdir
    if ($LASTEXITCODE -ne 0) { Stop-Here "Build failed (scroll up for the first error)" }
}
if (-not (Test-Path $uf2)) { Stop-Here "$uf2 was not produced" }
Write-Host "Firmware: $uf2" -ForegroundColor Green

# --- 2. Wait for BOOTSEL ------------------------------------------------------
Step "2/4 Flashing"
function Find-BootDrive {
    $d = Get-CimInstance Win32_LogicalDisk -Filter "VolumeName='RPI-RP2'" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($d) { return $d.DeviceID } else { return $null }
}
# A Pico running firmware with the SDK USB console (VID 2E8A, PID 000A).
function Find-PicoConsole {
    foreach ($e in @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
            Where-Object { ($_.PNPDeviceID -match 'VID_2E8A&PID_000A') -and ($_.Name -match '\(COM\d+\)') })) {
        if ($e.Name -match '\((COM\d+)\)') { return $Matches[1] }
    }
    return $null
}
function Wait-BootDrive($seconds) {
    $deadline = (Get-Date).AddSeconds($seconds)
    while ((Get-Date) -lt $deadline) {
        $d = Find-BootDrive
        if ($d) { return $d }
        Start-Sleep -Milliseconds 300
    }
    return $null
}

Ask "Car on a stand, wheels OFF the desk, flat and still, IMU plugged in."
Ask "Plug the Pico W USB cable in (no need to hold BOOTSEL if it already runs our firmware)."
Write-Host "Waiting up to $WaitMinutes min for the robot..."
$drive = $null
$deadline = (Get-Date).AddMinutes($WaitMinutes)
while (-not $drive) {
    $drive = Find-BootDrive
    if ($drive) { break }
    $cp = Find-PicoConsole
    if ($cp) {
        # The SDK USB console reboots into BOOTSEL when the host sets 1200 baud.
        Write-Host "Robot found on $cp running firmware; asking it to reboot into BOOTSEL..." -ForegroundColor Green
        Start-Sleep -Seconds 1
        try {
            $t = New-Object System.IO.Ports.SerialPort $cp, 1200
            $t.Open(); Start-Sleep -Milliseconds 200; $t.Close()
        } catch { Write-Host "(1200-baud touch: $($_.Exception.Message))" -ForegroundColor DarkGray }
        $drive = Wait-BootDrive 15
        if (-not $drive) {
            Ask "Automatic reboot did not work. Unplug the USB cable, HOLD the white BOOTSEL button, plug it back in, then release."
            $drive = Wait-BootDrive 180
            if (-not $drive) { Stop-Here "No RPI-RP2 drive. Try BOOTSEL again, or another USB cable (some are charge-only)." }
        }
        break
    }
    if ((Get-Date) -gt $deadline) { Stop-Here "No robot within $WaitMinutes min. Check the USB cable (some are charge-only), or plug in while holding BOOTSEL." }
    Start-Sleep -Milliseconds 300
}
Write-Host "Found RPI-RP2 at $drive" -ForegroundColor Green

$portsBefore = @([System.IO.Ports.SerialPort]::GetPortNames())
try { Copy-Item $uf2 "$drive\" -ErrorAction Stop } catch { Write-Host "(copy reported: $($_.Exception.Message); the Pico usually reboots anyway)" }
Write-Host "Firmware copied. The Pico is restarting..." -ForegroundColor Green

# --- 3. Find the new COM port -----------------------------------------------
Step "3/4 Waiting for the Pico's USB console (COM port)"
$port = $null
$deadline = (Get-Date).AddSeconds(30)
while (-not $port) {
    $new = @([System.IO.Ports.SerialPort]::GetPortNames() | Where-Object { $portsBefore -notcontains $_ })
    if ($new.Count -gt 0) { $port = $new[0] }
    elseif ((Get-Date) -gt $deadline) { Stop-Here "No new COM port in 30 s. Check Device Manager > Ports, then re-run with -SkipBuild." }
    else { Start-Sleep -Milliseconds 100 }
}
Write-Host "Console on $port" -ForegroundColor Green

# --- 4. Read and log --------------------------------------------------------
$logDir = Join-Path $Repo "testing\subsystem4\logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$log = Join-Path $logDir ("{0}_s4_unit.txt" -f (Get-Date -Format "yyyy-MM-dd_HHmm"))
"# Subsystem 4 APP_TEST run, $branch at $commit, $(Get-Date -Format 's')" | Set-Content -Path $log -Encoding UTF8

function Open-Console($preferred, $ignore, $seconds) {
    $deadline = (Get-Date).AddSeconds($seconds)
    while ((Get-Date) -lt $deadline) {
        $names = @([System.IO.Ports.SerialPort]::GetPortNames())
        $pick = $null
        if ($names -contains $preferred) { $pick = $preferred }
        else { $pick = @($names | Where-Object { $ignore -notcontains $_ }) | Select-Object -First 1 }
        if ($pick) {
            $s = New-Object System.IO.Ports.SerialPort $pick, 115200
            $s.DtrEnable = $true     # the Pico only sends once the host asserts DTR
            $s.RtsEnable = $true
            try { $s.Open(); return $s } catch { $s.Dispose() }
        }
        Start-Sleep -Milliseconds 250
    }
    return $null
}

function Show-Prompt($line) {
    Write-Host ""
    Write-Host ">> DO THIS NOW: $($line -replace '^\[S4\] MANUAL ', '')" -ForegroundColor Black -BackgroundColor Yellow
    try { [console]::Beep(1000, 300) } catch { }
}

$sp = Open-Console $port $portsBefore 10
if (-not $sp) { Stop-Here "Could not open $port (is PuTTY or another program using it?)" }

Step "4/4 Running. Do not touch the car until a yellow DO THIS NOW line appears (wheels may spin before that)."
Write-Host "Log: $log"
Write-Host "If the Pico drops off USB, this window reconnects by itself. Ctrl+C to stop." -ForegroundColor DarkGray

$pass = 0; $fail = 0; $hinted = $false; $buf = ""; $drops = 0
$stopAt = $null
$runDeadline = (Get-Date).AddMinutes(10)
try {
    while ($true) {
        if ($AutoOnly) {
            if ($stopAt -and ((Get-Date) -gt $stopAt)) { break }
            if ((Get-Date) -gt $runDeadline) {
                Write-Host "[script] no subsystem4 SUMMARY within 10 min; stopping." -ForegroundColor Red
                "# [script] timed out waiting for the subsystem4 SUMMARY" | Add-Content -Path $log -Encoding UTF8
                break
            }
        }
        $lost = $false
        try {
            if (-not $sp.IsOpen) { $lost = $true }
            elseif ($sp.BytesToRead -gt 0) { $buf += $sp.ReadExisting() }
            else { Start-Sleep -Milliseconds 20 }
        }
        catch { $lost = $true }

        while (($nl = $buf.IndexOf("`n")) -ge 0) {
            $line = $buf.Substring(0, $nl).TrimEnd("`r")
            $buf = $buf.Substring($nl + 1)
            Add-Content -Path $log -Value $line -Encoding UTF8

            if ($line -match '\[TEST\] S4-\S+ FAIL') { $fail++; Write-Host $line -ForegroundColor Red }
            elseif ($line -match '\[TEST\] S4-\S+ PASS') { $pass++; Write-Host $line -ForegroundColor Green }
            elseif ($line -match 'FAIL|\[FAULT\]|\[ERR\]') { Write-Host $line -ForegroundColor DarkYellow }
            elseif ($line -match '^\[(S4|IMU|SAFETY)\]') { Write-Host $line -ForegroundColor Cyan }
            else { Write-Host $line }

            if ($line -match '\[S4\] MANUAL S4-') { Show-Prompt $line }
            if ($line -match 'watchdog tripped: plug the IMU back in') { Show-Prompt "Plug the IMU cable back in" }
            if (-not $hinted -and $line -match 'subsystem4 SUMMARY') {
                $hinted = $true
                Write-Host ""
                if ($AutoOnly) {
                    Write-Host "Subsystem 4 done: $pass passed, $fail failed. Finishing in 3 s." -ForegroundColor Magenta
                    $stopAt = (Get-Date).AddSeconds(3)
                } else {
                    Write-Host "Subsystem 4 done: $pass passed, $fail failed. You can press Ctrl+C now." -ForegroundColor Magenta
                }
            }
        }

        if ($lost) {
            $drops++
            $stamp = Get-Date -Format "HH:mm:ss"
            "# [script] $stamp console lost (drop $drops), reconnecting" | Add-Content -Path $log -Encoding UTF8
            Write-Host ""
            Write-Host "[script] $stamp console lost (drop $drops): the Pico left USB. Reconnecting..." -ForegroundColor DarkYellow
            try { $sp.Dispose() } catch { }
            $buf = ""
            $sp = Open-Console $port $portsBefore 60
            if (-not $sp) { Write-Host "[script] no console for 60 s; stopping." -ForegroundColor Red; break }
            $stamp = Get-Date -Format "HH:mm:ss"
            "# [script] $stamp reconnected on $($sp.PortName)" | Add-Content -Path $log -Encoding UTF8
            Write-Host "[script] $stamp reconnected on $($sp.PortName). Lines after this are from a NEW boot." -ForegroundColor DarkYellow
        }
    }
}
finally {
    if ($sp -and $sp.IsOpen) { $sp.Close() }
    Write-Host ""
    Write-Host "Saved log: $log" -ForegroundColor Green
    Write-Host "S4 automatic checks seen: $pass passed, $fail failed. Console drops: $drops. Commit tested: $commit"
}
if ($hinted -and ($fail -eq 0) -and ($pass -gt 0)) { exit 0 } else { exit 1 }
