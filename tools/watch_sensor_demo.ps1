# Run in Windows PowerShell, not WSL: Bluetooth SPP appears as a Windows COM port.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^COM[0-9]+$')]
    [string]$Port,
    [string]$LogPath = (Join-Path $PSScriptRoot '..\dataset\sensor_demo.csv')
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
$header = 'host_time_utc,seq,imu_valid,imu_who,imu_error,imu_samples,gx,gy,gz,ax,ay,az,us_valid,us_error,us_samples,distance_mm,command_count,last_command,rx_errors'
try {
    $serial.Open()
    $resolvedLog = [System.IO.Path]::GetFullPath($LogPath)
    $logDirectory = [System.IO.Path]::GetDirectoryName($resolvedLog)
    [System.IO.Directory]::CreateDirectory($logDirectory) | Out-Null
    $newLog = !(Test-Path -LiteralPath $resolvedLog) -or (Get-Item -LiteralPath $resolvedLog).Length -eq 0
    if (!$newLog) {
        $existingHeader = Get-Content -LiteralPath $resolvedLog -TotalCount 1
        if ($existingHeader -ne $header) {
            throw "Existing log has a different format. Choose another -LogPath: $resolvedLog"
        }
    }
    # Append repeated runs to the same log; never discard an earlier measurement.
    $writer = New-Object System.IO.StreamWriter($resolvedLog, $true,
        (New-Object System.Text.UTF8Encoding($false)))
    $writer.AutoFlush = $true
    if ($newLog) { $writer.WriteLine($header) }
    Write-Host "Opened $Port at 115200 8N1. Log: $resolvedLog"
    Write-Host 'Keys: P=ping, F=20Hz telemetry, S=5Hz, N=10Hz, Q=quit. These do not control motors.'
    while ($true) {
        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true).KeyChar.ToString().ToUpperInvariant()
            if ($key -eq 'Q') { break }
            if ($key -in @('P', 'F', 'S', 'N')) {
                $serial.Write($key)
                Write-Host "Sent $key; check cmd_count / last_cmd in the next telemetry lines."
            }
        }
        try { $line = $serial.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        $fields = $line.Split(',')
        if ($fields.Count -ne 18) { continue } # First line may start mid-packet.
        $numbers = New-Object 'long[]' 18
        $wellFormed = $true
        for ($i = 0; $i -lt 18; $i++) {
            [long]$number = 0
            if (![long]::TryParse($fields[$i], [ref]$number)) { $wellFormed = $false; break }
            $numbers[$i] = $number
            if ($i -ge 5 -and $i -le 10) {
                if ($number -lt -32768 -or $number -gt 32767) { $wellFormed = $false }
            } elseif ($i -eq 14) {
                if ($number -lt -1 -or $number -gt 65535) { $wellFormed = $false }
            } elseif ($number -lt 0 -or $number -gt 4294967295) { $wellFormed = $false }
        }
        if (!$wellFormed -or $numbers[1] -notin @(0, 1) -or $numbers[11] -notin @(0, 1)) { continue }
        if ($numbers[2] -gt 255 -or $numbers[3] -gt 255 -or $numbers[12] -gt 255 -or $numbers[16] -gt 255) { continue }
        $writer.WriteLine(('{0},{1}' -f [DateTime]::UtcNow.ToString('o'), $line))
        if ($numbers[1] -eq 1) {
            $imuText = 'IMU OK G[dps]=({0:F2},{1:F2},{2:F2}) A[mg]=({3:F1},{4:F1},{5:F1}) samples={6}' -f (
                $numbers[5] * 0.070), ($numbers[6] * 0.070), ($numbers[7] * 0.070), (
                $numbers[8] * 0.122), ($numbers[9] * 0.122), ($numbers[10] * 0.122), $numbers[4]
        } else { $imuText = 'IMU INVALID id=0x{0:X2} error={1} samples={2}' -f $numbers[2], $numbers[3], $numbers[4] }
        if ($numbers[11] -eq 1) { $usText = 'US={0}mm samples={1}' -f $numbers[14], $numbers[13] }
        else { $usText = 'US INVALID error={0} samples={1}' -f $numbers[12], $numbers[13] }
        $commandText = if ($numbers[16] -eq 0) { '-' } else { ([char]$numbers[16]).ToString() }
        Write-Host ('seq={0} | {1} | {2} | cmd_count={3} last_cmd={4} rx_errors={5}' -f
            $numbers[0], $imuText, $usText, $numbers[15], $commandText, $numbers[17])
    }
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
