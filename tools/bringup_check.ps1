# tools/bringup_check.ps1
#
# Checks a console log of the PLATFORM_BRINGUP firmware (Batch B stage B5)
# against the USB CDC bring-up test in docs/DEBUG.md. Windows PowerShell 5.1
# or later, no extra installs. The log comes from tools/serial_capture.ps1
# ("HH:mm:ss.fff  <text>" per line).
#
# Usage, from the repo root:
#   powershell -ExecutionPolicy Bypass -File tools/bringup_check.ps1 -Log build/b5-run.txt
#   powershell -ExecutionPolicy Bypass -File tools/bringup_check.ps1 -Log build/b5-replug.txt -Replug
#
# Main run (default): one boot, at least -MinMinutes of heartbeats from all
# nine tasks, the stress window (ON and OFF lines) with the drop counter
# rising inside it.
# -Replug: the step 4 log. Reboots (after unplugging) and line loss while
# the terminal was closed are expected; heartbeats must come back.
#
# Rules (FAIL unless noted):
#   F1  [FAULT] or [ERR] line.
#   F2  Main run: the board rebooted (a second kernel banner).
#   F3  A heartbeat dt_min/dt_max more than 1000 us from the period, or
#       missed > 0. Inside the stress window this is a FAIL only for the
#       control tasks (id 1-4: safety, vc, motion, lf); WARN for the others.
#   F4  A heartbeat sequence gap with no "[DBG] dropped" line between the
#       two lines (a line lost without the ring counting it).
#   F5  Ring overflow ("[DBG] dropped" line) outside the stress window and
#       more than 10 s after the capture started or reconnected.
#   F6  The "[DBG] alive" difference drifts by more than 2 ms within a boot.
#   F7  Main run: fewer than -MinMinutes of heartbeats from any task, a task
#       missing, no stress ON/OFF, or no drop-counter rise inside it.
#   F8  -Replug: no heartbeat line at all, or none after the last reconnect.
# Exit code: 0 = PASS, 1 = FAIL, 2 = log not found.

param(
    [Parameter(Mandatory = $true)][string]$Log,
    [double]$MinMinutes = 10,
    [switch]$Replug
)

$names = @{ 1 = 'safety'; 2 = 'vc'; 3 = 'motion'; 4 = 'lf'; 5 = 'bc';
            6 = 'obs_scan'; 7 = 'obs_avoid'; 8 = 'imu'; 9 = 'tel' }
$controlIds = @(1, 2, 3, 4)
$jitterUs = 1000
$graceS = 10

$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Log)) { Write-Host "log not found: $Log"; exit 2 }
$logPath = (Resolve-Path -LiteralPath $Log).ProviderPath

$reLine  = '^(\d\d):(\d\d):(\d\d)\.(\d{3})  (.*)$'
$reHb    = '^\[HB\] id=(\d+) (\S+) seq=(\d+) cycles=(\d+) period=(\d+) dt_min=(\d+) dt_max=(\d+) missed=(\d+)$'
$reAlive = 'difference (-?\d+) ms'
$reDrop  = '^\[DBG\] dropped so far: (\d+) dbg_ lines'

$fails = New-Object System.Collections.ArrayList
$warns = New-Object System.Collections.ArrayList
function Fail([string]$m) { [void]$fails.Add($m) }
function Warn([string]$m) { [void]$warns.Add($m) }

# Per task, per boot.
function New-TaskState {
    return @{ lines = 0; lastSeq = -1; dropSince = $false; seconds = 0.0; stressLeft = 0 }
}

$boot = 0
$segment = 0                # log parts between kernel banners, for messages
$tasks = @{}
$best = @{}                 # id -> most seconds of heartbeats in one boot
$worst = @{}                # id -> largest |dt - period| seen, us
$aliveMin = $null; $aliveMax = $null
$inStress = $false; $stressOn = $false; $stressOff = $false
$dropAtOn = $null; $dropInStress = $false; $lastDrop = 0
$lastCaptureT = -1000.0
$hbAfterReconnect = $false; $reconnects = 0; $hbTotal = 0
$dayOffset = 0.0; $prevT = -1.0
$lineNo = 0

function Close-Boot {
    $script:segment++
    foreach ($id in $tasks.Keys) {
        $s = $tasks[$id].seconds
        if ((-not $best.ContainsKey($id)) -or ($s -gt $best[$id])) { $best[$id] = $s }
    }
    if (($null -ne $aliveMin) -and (($aliveMax - $aliveMin) -gt 2)) {
        Fail ("F6 log part {0}: alive difference drifts {1}..{2} ms" -f $script:segment, $aliveMin, $aliveMax)
    }
}

foreach ($raw in [System.IO.File]::ReadLines($logPath)) {
    $lineNo++
    $m = [regex]::Match($raw, $reLine)
    if (-not $m.Success) { continue }
    $t = [int]$m.Groups[1].Value * 3600 + [int]$m.Groups[2].Value * 60 + [int]$m.Groups[3].Value + [int]$m.Groups[4].Value / 1000.0
    if (($prevT -ge 0) -and (($t + $dayOffset) -lt ($prevT - 43200))) { $dayOffset += 86400 }
    $t += $dayOffset
    $prevT = $t
    $text = $m.Groups[5].Value

    if ($text.StartsWith('[capture] ')) {
        if (($text -match '^\[capture\] COM\d+ for') -or ($text -match '^\[capture\] reconnected')) {
            $lastCaptureT = $t
            if ($text -match 'reconnected') { $reconnects++; $hbAfterReconnect = $false }
        }
        continue
    }
    if ($text -match '^microT-Kernel Version') {
        Close-Boot          # also the part before the first banner (replug log)
        if (($boot -gt 0) -and (-not $Replug)) { Fail ("F2 line {0}: the board rebooted" -f $lineNo) }
        $boot++
        $tasks = @{}
        $aliveMin = $null; $aliveMax = $null
        $inStress = $false
        continue
    }
    if ($text.StartsWith('[FAULT]') -or $text.StartsWith('[ERR]')) {
        Fail ("F1 line {0}: {1}" -f $lineNo, $text)
        continue
    }
    $md = [regex]::Match($text, $reDrop)
    if ($md.Success) {
        $n = [long]$md.Groups[1].Value
        foreach ($id in $tasks.Keys) { $tasks[$id].dropSince = $true }
        $stressNow = $inStress
        foreach ($id in $tasks.Keys) { if ($tasks[$id].stressLeft -gt 0) { $stressNow = $true } }
        if ($stressNow) {
            if (($null -ne $dropAtOn) -and ($n -gt $dropAtOn)) { $dropInStress = $true }
        } elseif (($t - $lastCaptureT) -gt $graceS) {
            Fail ("F5 line {0}: ring overflow in normal operation: {1}" -f $lineNo, $text)
        }
        $lastDrop = $n
        continue
    }
    if ($text -match '^\[BRINGUP\] stress ON') {
        $inStress = $true; $stressOn = $true; $dropAtOn = $lastDrop
        continue
    }
    if ($text -match '^\[BRINGUP\] stress OFF') {
        $inStress = $false; $stressOff = $true
        foreach ($id in $tasks.Keys) { $tasks[$id].stressLeft = 2 }
        continue
    }
    $ma = [regex]::Match($text, $reAlive)
    if ($text.StartsWith('[DBG] alive') -and $ma.Success) {
        $d = [int]$ma.Groups[1].Value
        if (($null -eq $aliveMin) -or ($d -lt $aliveMin)) { $aliveMin = $d }
        if (($null -eq $aliveMax) -or ($d -gt $aliveMax)) { $aliveMax = $d }
        continue
    }
    $mh = [regex]::Match($text, $reHb)
    if (-not $mh.Success) { continue }

    $id = [int]$mh.Groups[1].Value
    $name = $mh.Groups[2].Value
    $seq = [long]$mh.Groups[3].Value
    $cycles = [long]$mh.Groups[4].Value
    $period = [long]$mh.Groups[5].Value
    $dtMin = [long]$mh.Groups[6].Value
    $dtMax = [long]$mh.Groups[7].Value
    $missed = [long]$mh.Groups[8].Value
    $hbAfterReconnect = $true
    $hbTotal++

    if ((-not $names.ContainsKey($id)) -or ($names[$id] -ne $name)) {
        Fail ("F7 line {0}: unknown task id={1} {2}" -f $lineNo, $id, $name)
        continue
    }
    if (-not $tasks.ContainsKey($id)) { $tasks[$id] = New-TaskState }
    $st = $tasks[$id]
    $stress = $inStress -or ($st.stressLeft -gt 0)
    if ($st.stressLeft -gt 0) { $st.stressLeft-- }

    if ($st.lastSeq -ge 0) {
        if ($seq -ne ($st.lastSeq + 1)) {
            if (-not $st.dropSince) {
                Fail ("F4 line {0}: id={1} {2} seq {3} -> {4} with no drop counted" -f $lineNo, $id, $name, $st.lastSeq, $seq)
            } elseif ($stress) {
                Warn ("line {0}: id={1} {2} lost heartbeat lines in the stress window (seq {3} -> {4})" -f $lineNo, $id, $name, $st.lastSeq, $seq)
            }
            # Outside stress, the drop line itself is judged by F5.
        }
    }
    $st.lastSeq = $seq
    $st.dropSince = $false
    $st.lines++
    $st.seconds += ($cycles * $period) / 1000000.0

    $dev = [Math]::Max([Math]::Abs($dtMin - $period), [Math]::Abs($dtMax - $period))
    if ((-not $worst.ContainsKey($id)) -or ($dev -gt $worst[$id])) { $worst[$id] = $dev }
    if (($dev -gt $jitterUs) -or ($missed -gt 0)) {
        $msg = "line {0}: id={1} {2} period {3} us, dt {4}..{5} us, missed {6}" -f $lineNo, $id, $name, $period, $dtMin, $dtMax, $missed
        if ($stress -and -not ($controlIds -contains $id)) {
            Warn ("stress, non-control task: " + $msg)
        } else {
            Fail ("F3 " + $msg)
        }
    }
}
Close-Boot

if ($Replug) {
    if ($hbTotal -eq 0) { Fail "F8 no heartbeat line at all" }
    if ($reconnects -eq 0) { Warn "no reconnect in this log (was the cable unplugged or the terminal reopened?)" }
    elseif (-not $hbAfterReconnect) { Fail "F8 no heartbeat line after the last reconnect" }
} else {
    if ($boot -eq 0) { Fail "F7 no kernel banner (start the capture right after flashing)" }
    foreach ($id in 1..9) {
        if (-not $best.ContainsKey($id)) {
            Fail ("F7 task id={0} {1}: no heartbeat line" -f $id, $names[$id])
        } elseif ($best[$id] -lt ($MinMinutes * 60)) {
            Fail ("F7 task id={0} {1}: only {2:N0} s of heartbeats (need {3} min)" -f $id, $names[$id], $best[$id], $MinMinutes)
        }
    }
    if (-not $stressOn) { Fail "F7 no '[BRINGUP] stress ON' line (capture too short?)" }
    if (-not $stressOff) { Fail "F7 no '[BRINGUP] stress OFF' line (capture too short?)" }
    if ($stressOn -and -not $dropInStress) { Fail "F7 the drop counter did not rise during the stress window" }
}

Write-Host ("bringup_check: {0}{1}" -f $Log, $(if ($Replug) { ' (replug)' } else { '' }))
Write-Host ("  boots: {0}, reconnects: {1}" -f $boot, $reconnects)
foreach ($id in 1..9) {
    $s = 0.0; $w = 0
    if ($best.ContainsKey($id)) { $s = $best[$id] }
    if ($worst.ContainsKey($id)) { $w = $worst[$id] }
    Write-Host ("  id={0} {1,-9} {2,6:N0} s of heartbeats, worst |dt - period| {3} us" -f $id, $names[$id], $s, $w)
}
foreach ($w in $warns) { Write-Host ("  WARN " + $w) }
foreach ($f in $fails) { Write-Host ("  FAIL " + $f) }
if ($fails.Count -eq 0) {
    Write-Host "bringup_check: PASS"
    exit 0
}
Write-Host ("bringup_check: FAIL ({0} problems)" -f $fails.Count)
exit 1
