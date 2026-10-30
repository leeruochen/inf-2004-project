# tools/serial_capture.ps1
#
# Record the Pico W's USB CDC console to a file, with a timestamp per line.
# Windows PowerShell 5.1 or later, no extra installs (uses .NET SerialPort).
# Opening the port sets DTR, which is what the firmware's stdio_usb_connected()
# (and so the debug drain) waits for.
#
# Usage, from the repo root:
#   powershell -ExecutionPolicy Bypass -File tools/serial_capture.ps1 -Seconds 150 -Out build/console.txt
# Options:
#   -Port COM5            use this port instead of finding the Pico (USB VID 2E8A)
#   -WaitPortSeconds 30   how long to wait for the port to appear (after flashing)
#
# If the board resets or is unplugged during the capture, the script writes
# "[capture] port lost" and reconnects when the port comes back, so a
# fault -> reset -> fault-record sequence ends up in one log.
# Exit code: 0 = captured, 2 = no port found.

param(
    [int]$Seconds = 150,
    [string]$Out = "console.txt",
    [string]$Port = "",
    [int]$WaitPortSeconds = 30
)

function Find-PicoPort {
    $devices = Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
        Where-Object { ($_.PNPDeviceID -match 'VID_2E8A') -and ($_.Name -match '\(COM\d+\)') }
    foreach ($d in $devices) {
        if ($d.Name -match '\((COM\d+)\)') { return $Matches[1] }
    }
    return ""
}

function Wait-Port([string]$fixed, [datetime]$until) {
    while ((Get-Date) -lt $until) {
        $p = if ($fixed -ne "") { $fixed } else { Find-PicoPort }
        if (($p -ne "") -and ([System.IO.Ports.SerialPort]::GetPortNames() -contains $p)) { return $p }
        Start-Sleep -Milliseconds 200
    }
    return ""
}

function Open-Port([string]$name) {
    $sp = New-Object System.IO.Ports.SerialPort $name, 115200
    $sp.DtrEnable = $true
    $sp.RtsEnable = $true
    $sp.ReadTimeout = 200
    $sp.NewLine = "`n"
    for ($i = 0; $i -lt 25; $i++) {
        try { $sp.Open(); return $sp } catch { Start-Sleep -Milliseconds 200 }
    }
    return $null
}

$end = (Get-Date).AddSeconds($Seconds)
$writer = New-Object System.IO.StreamWriter($Out, $false)

function Log([string]$text) {
    $line = ("{0:HH:mm:ss.fff}  {1}" -f (Get-Date), $text)
    $writer.WriteLine($line)
    $writer.Flush()
    Write-Host $line
}

$fixedPort = $Port
$name = Wait-Port $fixedPort ((Get-Date).AddSeconds($WaitPortSeconds))
if ($name -eq "") {
    Log "[capture] no Pico USB serial port (VID 2E8A) within $WaitPortSeconds s"
    $writer.Close()
    exit 2
}

Log "[capture] $name for $Seconds s -> $Out"
$sp = Open-Port $name
try {
    while ((Get-Date) -lt $end) {
        if (($null -eq $sp) -or (-not $sp.IsOpen)) {
            $name = Wait-Port $fixedPort $end
            if ($name -eq "") { break }
            $sp = Open-Port $name
            if ($null -ne $sp) { Log "[capture] reconnected to $name" }
            continue
        }
        try {
            $text = $sp.ReadLine()
            Log ($text.TrimEnd("`r"))
        } catch [System.TimeoutException] {
            continue
        } catch {
            Log "[capture] port lost ($($_.Exception.GetType().Name)); waiting for it to come back"
            try { $sp.Close() } catch { }
            $sp = $null
        }
    }
} finally {
    if (($null -ne $sp) -and $sp.IsOpen) { $sp.Close() }
    Log "[capture] done"
    $writer.Close()
}
exit 0
