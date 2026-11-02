# testing/subsystem4/s4_test.ps1 - Subsystem 4 (Buddy 4): build, flash and record the
# APP_TEST firmware with the operator-guided tests S4-U13..U16 (S4_MANUAL_TESTS=ON).
# Needs the Pico SDK v1.5.1 Windows installer (HANDOVER.md section 5).
# Run from the repo root:
#   powershell -ExecutionPolicy Bypass -File testing\subsystem4\s4_test.ps1
# Options: -SkipBuild (flash the existing build/s4test firmware).  Stop: Ctrl+C.
param(
    [string]$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path,
    [switch]$SkipBuild
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
$uf2 = Join-Path $Repo "build\s4test\robocar.uf2"
if (-not $SkipBuild) {
    Step "1/4 Building the test firmware (APP_TEST=ON, S4 guided tests on) in build/s4test"
    # A build folder configured with CMAKE_C_FLAGS by an earlier script lost the
    # toolchain's CPU flags; start that one fresh.
    $cache = "build/s4test/CMakeCache.txt"
    if ((Test-Path $cache) -and (Select-String -Path $cache -Pattern '^CMAKE_C_FLAGS:STRING=-DS4' -Quiet)) {
        Write-Host "Removing the old build/s4test folder (wrong compiler flags)" -ForegroundColor DarkYellow
        Remove-Item -Recurse -Force build/s4test
    }
    & $CM -S . -B build/s4test @common -DDBG_BACKEND=USB -DAPP_TEST=ON -DS4_MANUAL_TESTS=ON
    if ($LASTEXITCODE -ne 0) { Stop-Here "CMake configure failed (scroll up for the first error)" }
    & $CM --build build/s4test
    if ($LASTEXITCODE -ne 0) { Stop-Here "Build failed (scroll up for the first error)" }
}
if (-not (Test-Path $uf2)) { Stop-Here "$uf2 was not produced" }
Write-Host "Firmware: $uf2" -ForegroundColor Green

# --- 2. Wait for BOOTSEL ------------------------------------------------------
Step "2/4 Flashing"
Ask "Car on a stand, wheels OFF the desk, IMU plugged in."
Ask "Unplug the Pico W USB cable, HOLD the white BOOTSEL button, plug the cable back in, then release."
Write-Host "Waiting for the RPI-RP2 drive..."
$drive = $null
$deadline = (Get-Date).AddMinutes(3)
while (-not $drive) {
    $d = Get-CimInstance Win32_LogicalDisk -Filter "VolumeName='RPI-RP2'" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($d) { $drive = $d.DeviceID }
    elseif ((Get-Date) -gt $deadline) { Stop-Here "No RPI-RP2 drive after 3 minutes. Try BOOTSEL again, or another USB cable (some are charge-only)." }
    else { Start-Sleep -Milliseconds 300 }
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
try {
    while ($true) {
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
                Write-Host "Subsystem 4 done: $pass passed, $fail failed. You can press Ctrl+C now." -ForegroundColor Magenta
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
