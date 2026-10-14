<#
 record.ps1 -- serial capture to a file (lossless binary, USB-glitch tolerant)

 Usage:
  powershell -NoProfile -ExecutionPolicy Bypass -File record.ps1 -Secs 60
  powershell -NoProfile -ExecutionPolicy Bypass -File record.ps1 -Secs 300 -Port COM5
  powershell -NoProfile -ExecutionPolicy Bypass -File record.ps1 -Secs 60 -Port auto -Baud 115200

 Ports:
  -Port COM3  WCH-Link debug UART (USART1) -- text log / V3F+startup
  -Port COM4  board USB CDC (CH34x-class usbser) -- binary justfloat stream
  -Port COMx  explicit port
  -Port auto  pick automatically (default): the HIGHEST COM number wins,
         so the board's CDC is preferred over the lower numbered
         debug port. Names come from the union of GetPortNames() and
         HARDWARE\DEVICEMAP\SERIALCOMM (GetPortNames can lag).

 Params:
  -Secs    capture duration in seconds (required; wall clock)
  -Baud    baud rate (default 921600; CDC ignores baud but the value must be valid)
  -Out     output file (default R:\rec_<stamp>.bin)
  -IdleMs  idle watchdog: ms without a new byte before dropout+reconnect (default 500)
  -RetryMs reconnect poll interval (default 25 ms, fast polling)
  -NoReconnect disable auto-reconnect (old behavior, for comparison)

 * Why an idle watchdog is needed:
   On a momentary USB signal-line glitch, SerialPort on Windows often raises
   NO exception; BytesToRead just keeps returning 0. A pure try/catch reconnect
   would then never fire, and the symptom is "the file stops growing while the
   script is still alive". So you must ALSO use "no new bytes for a while"
   to detect the dropout.

 * The output file is NOT closed on disconnect:
   The file stream is opened exactly once (Create truncate); a reconnect only
   reopens the port. Bytes before and after a reconnect land in the same file;
   nothing is truncated.

 * Event log: written to <Out>.events.log, with each dropout/recovery time and
   downtime in ms, so it can be cross-checked against frame-count gaps
   (magmap gap_report) afterwards.

 * NOTE: this file deliberately AVOIDS backtick line-continuation -- a trailing
#>
param(
  [Parameter(Mandatory=$true)][int]$Secs,
  [string]$Port = 'auto',
  [int]$Baud = 921600,
  [string]$Out = '',
  [int]$IdleMs = 500,
  [int]$RetryMs = 25,
  [switch]$NoReconnect
)

$ErrorActionPreference = 'Stop'

function Get-Ports {
  $all = [System.IO.Ports.SerialPort]::GetPortNames()
  $list = @()
  foreach ($p in $all) {
    $name = ''
    try {
      $q = Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue | Where-Object { $_.FriendlyName -like "*($p)*" }
      if ($q) { $name = $q[0].FriendlyName }
    } catch { }
    $list += [pscustomobject]@{ Port = $p; Name = $name }
  }
  return $list
}

function Get-PortNum([string]$name) {
  $m = [regex]::Match($name, '(\d+)\s*$')
  if ($m.Success) { return [int]$m.Groups[1].Value }
  return -1
}

# Port-name union: GetPortNames() can lag / miss a just-enumerated port,
# while HKLM HARDWARE\DEVICEMAP\SERIALCOMM is usually already updated.
# NOTE: keep this file ASCII-only. PowerShell 5.1 mis-parses UTF-8 without
#    BOM, and non-ASCII comments break the whole script (see header note).
function Get-PortNamesAll {
  $set = New-Object 'System.Collections.Generic.HashSet[string]'
  try {
    foreach ($p in [System.IO.Ports.SerialPort]::GetPortNames()) { [void]$set.Add($p) }
  } catch { }
  try {
    $rk = 'HKLM:\HARDWARE\DEVICEMAP\SERIALCOMM'
    if (Test-Path $rk) {
      $it = Get-ItemProperty $rk
      foreach ($pr in $it.PSObject.Properties) {
        if ($pr.Name -like 'PS*') { continue }
        $v = [string]$pr.Value
        if ($v -match '^COM\d+$') { [void]$set.Add($v) }
      }
    }
  } catch { }
  return @($set)
}

function Pick-Port([string]$hint) {
  $names = Get-PortNamesAll
  if ($hint -ne 'auto') {
    if ($names -contains $hint) { return $hint }
    return $null
  }
  if ($names.Count -eq 0) { return $null }
  # Highest COM number wins: after re-enumeration the board usually takes
  # a higher number, so a stale low-numbered virtual/debug port can not
  # steal the selection.
  $best = $null
  $bestNum = -2
  foreach ($p in $names) {
    $k = Get-PortNum $p
    if ($k -gt $bestNum) { $bestNum = $k; $best = $p }
  }
  return $best
}

function Open-Port([string]$name, [int]$baud) {
  try {
    $sp = New-Object System.IO.Ports.SerialPort $name, $baud, 'None', 8, 'One'
    $sp.ReadTimeout  = 200
    $sp.WriteTimeout  = 200
    $sp.ReadBufferSize = 4MB
    $sp.DtrEnable   = $false
    $sp.RtsEnable   = $false
    $sp.Open()
    return $sp
  } catch {
    return $null
  }
}

if ($Out -eq '') { $Out = ('R:\rec_{0}.bin' -f (Get-Date -Format 'yyyyMMdd_HHmmss')) }
$logPath = "$Out.events.log"

$target = Pick-Port $Port
if ($target) { Write-Host ('port  : {0}' -f $target) } else { Write-Host 'port  : (no port available yet, waiting to reconnect)' }

try {
  $fs = [IO.File]::Create($Out)
} catch {
  Write-Host ''
  Write-Host ('ERROR: cannot create output file: {0}' -f $Out)
  Write-Host ('    reason: {0}' -f $_.Exception.Message)
  Write-Host '    check: 1) is R: mounted? (RAM disk disappears on power loss)'
  Write-Host '       2) is the file locked by another process?'
  Write-Host '       3) write permission on that directory?'
  Write-Host '    or pass an explicit path as the 6th arg to record.cmd'
  exit 2
}
$ev = New-Object System.IO.StreamWriter($logPath, $false)
$ev.AutoFlush = $true
$buf = New-Object byte[] 262144

function Log-Event([string]$m) { $ev.WriteLine(('{0:HH:mm:ss.fff} {1}' -f (Get-Date), $m)) }

$sw     = [System.Diagnostics.Stopwatch]::StartNew()
$total   = 0L
$nextReport = 2
$nDrop   = 0
$dropMsTot = 0L

$sp = $null
$everConnected = $false
if ($target) { $sp = Open-Port $target $Baud }
if ($sp) { $everConnected = $true }
$lastData = $sw.ElapsedMilliseconds
$dropAtMs = $lastData

Write-Host ('secs  : {0}  idle={1}ms retry={2}ms reconnect={3}' -f $Secs, $IdleMs, $RetryMs, (-not $NoReconnect))
Write-Host ('out  : {0}' -f $Out)
Write-Host ('events : {0}' -f $logPath)
Write-Host ''

try {
  while ($sw.Elapsed.TotalSeconds -lt $Secs) {

    # ---------- not connected: fast reconnect polling ----------
    if ($null -eq $sp) {
      if ($NoReconnect) { break }
      Start-Sleep -Milliseconds $RetryMs
      $t = Pick-Port $Port
      if ($t) {
        $sp = Open-Port $t $Baud
        if ($sp) {
          $target  = $t
          $everConnected = $true
          $lastData = $sw.ElapsedMilliseconds
          $downMs  = $lastData - $dropAtMs
          $dropMsTot += $downMs
          Log-Event ('REOPEN ok port={0} elapsed={1:N1}s down={2}ms' -f $t, $sw.Elapsed.TotalSeconds, $downMs)
          Write-Host (' [reconnect] {0} reconnected @ {1:N1}s (downtime {2} ms)' -f $t, $sw.Elapsed.TotalSeconds, $downMs)
        }
      }
      continue
    }

    # ---------- connected: read ----------
    $n = 0
    $failed = $false
    try {
      $n = $sp.BytesToRead
      if ($n -gt 0) {
        if ($n -gt $buf.Length) { $n = $buf.Length }
        $r = $sp.Read($buf, 0, $n)
        if ($r -gt 0) {
          $fs.Write($buf, 0, $r)
          $total += $r
          $lastData = $sw.ElapsedMilliseconds
        }
      }
    } catch {
      $failed = $true
    }

    # idle watchdog: no exception but no data for long -> treat as dropout
    $idle = $sw.ElapsedMilliseconds - $lastData
    if ($failed -or $idle -gt $IdleMs) {
      $nDrop++
      $why = 'idle-watchdog'
      if ($failed) { $why = 'exception' }
      try { $fs.Flush() } catch { }
      try { $sp.Close() } catch { }
      try { $sp.Dispose() } catch { }
      $sp = $null
      $dropAtMs = $sw.ElapsedMilliseconds
      Log-Event ('DROPOUT #{0} reason={1} idle={2}ms elapsed={3:N1}s bytes={4}' -f $nDrop, $why, $idle, $sw.Elapsed.TotalSeconds, $total)
      Write-Host (' [dropout #{0}] {1} @ {2:N1}s (idle {3}ms), starting reconnect' -f $nDrop, $why, $sw.Elapsed.TotalSeconds, $idle)
      continue
    }

    if ($n -le 0) { Start-Sleep -Milliseconds 1 }

    # ---------- progress ----------
    $el = $sw.Elapsed.TotalSeconds
    if ($el -ge $nextReport) {
      Write-Host (' {0,4}s : {1,12:N0} B' -f [int]$el, $total)
      $nextReport += 2
    }
  }
} finally {
  try { $fs.Flush() } catch { }
  try { $fs.Close() } catch { }
  if ($sp) { try { $sp.Close() } catch { }; try { $sp.Dispose() } catch { } }
  $sw.Stop()
  Log-Event ('STOP total={0} bytes dropouts={1} downtime={2}ms wall={3:N1}s' -f $total, $nDrop, $dropMsTot, $sw.Elapsed.TotalSeconds)
  $ev.Close()
}

Write-Host ''
Write-Host ('done : {0:N0} B' -f $total)
if ($total -eq 0) {
  Write-Host ''
  if (-not $everConnected) {
    Write-Host 'ERROR: ERROR: could never open any serial port -- this file is EMPTY, do not analyze it.'
    Write-Host '    check: 1) is the board powered, USB is the USB cable seated, is the port number -Port wrong, '
    Write-Host '       is another program (serial terminal / /VOFA+) holding the port.'
  } else {
    Write-Host 'ERROR: ERROR: opened a port but received ZERO bytes -- this file is EMPTY, do not analyze it.'
    Write-Host '    check: is the board really streaming (USART1 log prints imu=/mmc= counters), '
    Write-Host '       baud rate / /does the frame type match.'
  }
  Write-Host ('    details in {0}' -f $logPath)
  exit 3
}
if ($nDrop -gt 0) {
  Write-Host ('glitch: {0} dropout(s), total downtime {1} ms(see {2})' -f $nDrop, $dropMsTot, $logPath)
  Write-Host '    Bytes lost during a dropout cannot be recovered; use magmap of gap_report() to check the gaps'
}
Get-Item -LiteralPath $Out | ForEach-Object {
  $rate = $_.Length / ([math]::Max($Secs,1)) / 1KB
  Write-Host ('file : {0}' -f $_.FullName)
  Write-Host ('size : {0:N0} B ({1:N1} KB, {2:N1} KB/s)' -f $_.Length, ($_.Length/1KB), $rate)
}
