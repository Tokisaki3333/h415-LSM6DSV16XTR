@echo off
rem record.cmd -- serial capture to R:\ root
rem   Justfloat binary stream on the board USB CDC:
rem     type 1 ATT frame : 16 float =  64 B  @ ~100 Hz   (attitude, for VOFA+)
rem     type 2 FULL frame: 30 float = 120 B  @ 7670 Hz   (raw counts + q + flag2)
rem   FULL mode ~900 KB/s. End of capture prints bytes, rate and glitch count.
rem
rem   usage: record.cmd <secs> [port] [baud] [idleMs] [retryMs]
rem     port: COM3 = WCH-Link debug UART (text)
rem           COM4 = board USB CDC (justfloat binary; CH34x-class usbser)
rem           COMx = explicit port
rem           auto = pick automatically (default): the HIGHEST COM number
rem                  wins, so the board's CDC is preferred over the lower
rem                  numbered debug port.
rem     idleMs : idle watchdog threshold in ms (default 500).
rem              No new byte for this long => treat as dropout and reconnect.
rem              USB signal-line glitches often do NOT raise an exception on
rem              Windows, so try/catch alone would never fire.
rem     retryMs: reconnect poll interval in ms (default 25, high-speed polling)
rem   e.g.  record.cmd 20
rem         record.cmd 300 COM3
rem         record.cmd 20 COM5 115200
rem         record.cmd 120 auto 921600 500 25
rem
rem   NOTE: keep this file ASCII-only. cmd.exe codepage handling of UTF-8
rem         without BOM is not reliable; non-ASCII comments can break parsing.
setlocal
if "%~1"=="" goto usage
set SECS=%~1
set PORT=%~2
set BAUD=%~3
set IDLE=%~4
set RETRY=%~5
set OUTF=%~6
if "%PORT%"==""  set PORT=auto
if "%BAUD%"==""  set BAUD=921600
if "%IDLE%"==""  set IDLE=500
if "%RETRY%"=="" set RETRY=25
if "%OUTF%"=="" goto run
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0record.ps1" -Secs %SECS% -Port %PORT% -Baud %BAUD% -IdleMs %IDLE% -RetryMs %RETRY% -Out "%OUTF%"
goto :eof
:run
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0record.ps1" -Secs %SECS% -Port %PORT% -Baud %BAUD% -IdleMs %IDLE% -RetryMs %RETRY%
goto :eof
:usage
echo usage: record.cmd ^<secs^> [port] [baud] [idleMs] [retryMs] [outfile]
echo   port: COM3 = WCH-Link debug UART (text)
echo         COMx = board USB CDC (justfloat binary)
echo         auto = pick automatically (default)
echo   idleMs : idle watchdog threshold, default 500 ms
echo   retryMs: reconnect poll interval, default 25 ms
echo   outfile: default R:\rec_^<stamp^>.bin
echo   e.g.  record.cmd 120 auto 921600 500 25
exit /b 1
