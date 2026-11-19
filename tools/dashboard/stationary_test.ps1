# Stationary subsystem test with the telemetry dashboard (dashboard integration, 19 Nov 2026).
#   powershell -ExecutionPolicy Bypass -File tools\dashboard\stationary_test.ps1 -Subsystem imu
#   powershell -ExecutionPolicy Bypass -File tools\dashboard\stationary_test.ps1 -Subsystem obstacle -ExpectCm 20,50 -ExpectWidthMm 100
# Flashes the stationary firmware (it never commands the drive motors: the VC stays in
# READY_WAITING and no start command can reach it over USB), opens a second window with the
# live dashboard view, captures the USB log, then judges it with "tel_dashboard.py check".
# Procedures: tools/dashboard/SUBSYSTEM_TESTS.md.
param(
    [Parameter(Mandatory = $true)][ValidateSet('imu', 'obstacle')][string]$Subsystem,
    [string]$Uf2 = '',              # default: newest build\robocar-dash-stationary-*.uf2
    [int]$Minutes = 4,
    [string]$ExpectCm = '',         # obstacle: tape-measured box distances, e.g. 20,50
    [int]$ExpectWidthMm = 0,        # obstacle: the box's real width
    [switch]$NoFlash                # the firmware is already on the car: just capture
)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $repo

$py = $null
foreach ($c in 'python', 'py') { if (Get-Command $c -ErrorAction SilentlyContinue) { $py = $c; break } }
if (-not $py) { throw 'Python 3.8+ not found (python or py on PATH)' }

if (-not $NoFlash) {
    if ($Uf2 -eq '') {
        $f = Get-ChildItem 'build\robocar-dash-stationary-*.uf2' -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if (-not $f) { throw 'no build\robocar-dash-stationary-*.uf2: build it first (SUBSYSTEM_TESTS.md)' }
        $Uf2 = $f.FullName
    }
    if (-not (Test-Path $Uf2)) { throw "$Uf2 not found" }
}
$out = "build\dash-$Subsystem-" + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.txt'

Write-Host ''
Write-Host "STATIONARY $($Subsystem.ToUpper()) TEST ($Minutes min). The drive motors are never commanded." -ForegroundColor Cyan
if ($Subsystem -eq 'imu') {
    Write-Host ' 0. Car flat on the desk, still. Have a book about 4 cm thick ready.'
    Write-Host ' 1. After flashing: hands off, car flat and still for 30 s (level reference, then flat readings).'
    Write-Host ' 2. Lift the FRONT about 15 deg (front wheels on the book), hold 3 s, put it back flat. Wait 5 s.'
    Write-Host '    In the live window pitch should go POSITIVE. A HUMP event here is expected (pitch-only detector).'
    Write-Host ' 3. Lift the LEFT side about 15 deg (left wheels on the book), hold 3 s, put it back flat. Wait 5 s.'
    Write-Host '    Note the sign of roll in the live window (left side up should read positive; not yet confirmed).'
    Write-Host ' 4. LAST: lift the whole car straight up about 5 cm, hold 1 s, put it down flat.'
    Write-Host '    Expect [IMU] LIFTED and [SAFETY] BRAKE. The fault stays latched until you replug USB: that is correct.'
    Write-Host ' 5. Leave it still until the capture ends (or Ctrl+C), then read the check.'
}
else {
    Write-Host ' 0. Car on the desk, nothing within 60 cm in front or to the sides (the sensor sweeps 30-150 deg).'
    Write-Host '    Have a box, a tape measure, and the box width measured. The scanner servo WILL sweep.'
    Write-Host ' 1. After flashing: 20 s with nothing in front.'
    Write-Host ' 2. Box straight ahead at 50 cm from the sensor face, square to it: hold 15 s.'
    Write-Host ' 3. Box straight ahead at 20 cm: hold 15 s (inside 25 cm, so the scanner should report an obstacle).'
    Write-Host ' 4. Remove the box: 15 s. Leave it until the capture ends (or Ctrl+C).'
    Write-Host '    [S5] avoid lines may appear: the VC ignores them while it is not driving (motors stay off).'
}

if (-not $NoFlash) {
    Write-Host ''
    Write-Host 'Hold BOOTSEL, plug in the USB, release. Waiting up to 2 minutes for the RPI-RP2 drive...'
    $v = $null
    for ($i = 0; ($i -lt 120) -and (-not $v); $i++) {
        $v = Get-Volume | Where-Object { $_.FileSystemLabel -eq 'RPI-RP2' }
        if (-not $v) { Start-Sleep -Seconds 1 }
    }
    if (-not $v) { throw 'RPI-RP2 drive not found: plug in again with BOOTSEL held' }
    Copy-Item $Uf2 "$($v.DriveLetter):\"
    Write-Host "Flashed $Uf2"
}

# Live view in a second window; it waits until the capture creates the log.
$live = "Set-Location '$repo'; $py tools\dashboard\tel_dashboard.py decode-log '$out' --view $Subsystem --follow"
Start-Process powershell -ArgumentList '-NoExit', '-Command', $live

$check = "$py tools\dashboard\tel_dashboard.py check $Subsystem $out"
if ($Subsystem -eq 'obstacle') {
    if ($ExpectCm -ne '') { $check += " --expect-cm $ExpectCm" }
    if ($ExpectWidthMm -gt 0) { $check += " --expect-width-mm $ExpectWidthMm" }
}
Write-Host "Capturing $Minutes min into $out (Ctrl+C ends early; the check still runs)."
try {
    powershell -ExecutionPolicy Bypass -File tools\serial_capture.ps1 -Seconds ($Minutes * 60) -Out $out
}
finally {
    Write-Host ''
    Invoke-Expression $check
    Write-Host ''
    Write-Host "Judge it again later:  $check"
}
