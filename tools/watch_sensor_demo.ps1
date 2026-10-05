# Windows PowerShell, Bluetooth SPP or USB-UART, 115200 8N1.
# Unit-bearing text firmware. Earlier raw CSV logs are left untouched.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^COM[0-9]+$')]
    [string]$Port,
    [string]$LogPath = (Join-Path $PSScriptRoot '..\dataset\sensor_demo.txt')
)
$ErrorActionPreference = 'Stop'
$serial = New-Object System.IO.Ports.SerialPort($Port, 115200,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.Encoding = [System.Text.Encoding]::ASCII
$serial.NewLine = "`n"
$serial.ReadTimeout = 100
$serial.WriteTimeout = 1000
$writer = $null
try {
    $serial.Open()
    $resolvedLog = [System.IO.Path]::GetFullPath($LogPath)
    if ([System.IO.Path]::GetExtension($resolvedLog) -eq '.csv') {
        throw 'This firmware emits text. Use a .txt -LogPath to preserve earlier CSV records.'
    }
    [System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($resolvedLog)) | Out-Null
    $writer = New-Object System.IO.StreamWriter($resolvedLog, $true,
        (New-Object System.Text.UTF8Encoding($false)))
    $writer.AutoFlush = $true
    Write-Host "Opened $Port at 115200 8N1. Log: $resolvedLog"
    Write-Host 'P=ping, F=20Hz, S=5Hz, N=10Hz, T=set RTC from PC local time, Q=quit.'
    while ($true) {
        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true).KeyChar.ToString().ToUpperInvariant()
            if ($key -eq 'Q') { break }
            if ($key -in @('P', 'F', 'S', 'N')) {
                $serial.Write($key)
                Write-Host "Sent $key; check CMD and LAST."
            } elseif ($key -eq 'T') {
                $now = Get-Date
                $frame = 'T{0}{1}{2};' -f $now.ToString('yyMMdd'), ([int]$now.DayOfWeek), $now.ToString('HHmmss')
                $serial.Write($frame)
                Write-Host "Sent $frame. Check actual HH-MM-SS on the board display."
            }
        }
        try { $line = $serial.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        # Ignore an incomplete first line after opening halfway through a report.
        if ($line -notmatch '^I:(OK|E[0-9]{3}) +U:(OK|E[0-9]{3}) ' -or
            $line -notmatch ' RXERR=[0-9]+ LAST=[PFSN-]$') { continue }
        $writer.WriteLine(('{0} {1}' -f [DateTime]::UtcNow.ToString('o'), $line))
        Write-Host $line
    }
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
