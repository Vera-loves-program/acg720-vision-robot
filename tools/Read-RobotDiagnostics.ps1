<#
Read-only operation10 snapshots using Windows/.NET; no Python packages required.
Close VOFA before running. Does not issue motion, reset, PID or clear commands.
Use -SelfTest to check decoding without opening a serial port.
#>
[CmdletBinding()]
param(
    [string]$Port,
    [ValidateRange(1,20)][int]$Count=3,
    [ValidateRange(1,30)][int]$Interval=1,
    [string]$Output='dataset/field_20261103/navigation.jsonl',
    [switch]$SelfTest
)
$ErrorActionPreference='Stop'
function Convert-Signed32([uint32]$Value) {
    if ($Value -ge 2147483648) { return ([int64]$Value-4294967296) }
    return [int64]$Value
}
function Convert-Ack([string]$Line,[int]$Sequence) {
    if ($Line -notmatch '^\[ACK (OK|ERR)\]\s*(\d+),(\d+),(\d+),(\d+),(\d+),(-?\d+)\s*$') { return $null }
    if ([int]$Matches[2] -ne $Sequence) { return $null }
    if ($Matches[1] -ne 'OK' -or [int]$Matches[3] -ne 10 -or [int]$Matches[6] -ne 0) {
        throw "Diagnostic rejected: $Line"
    }
    return [uint32]([int64]$Matches[7] -band [int64]4294967295)
}
function Convert-Snapshot([uint32[]]$Words) {
    if ($Words.Count -ne 48 -or $Words[0] -ne 0x4c440101) { throw 'Snapshot version mismatch; check the FPGA candidate.' }
    $reasonText=@('No live cause; inspect mission/planner','Scan not ready','Scan expired',
        'Gyro not calibrated/valid','Gyro expired','Processing overrun','Direction measurement UNKNOWN',
        'Near obstacle','Virtual contact','Map not world','Waiting map integration','Start UNKNOWN',
        'Start OCCUPIED','Goal UNKNOWN','Goal OCCUPIED','No known route','Confirmed lookahead hit')
    $reason=[int]($Words[39] -band 255)
    $text='Unclassified'; if ($reason -lt $reasonText.Count) { $text=$reasonText[$reason] }
    return [ordered]@{
        wall_time=[DateTimeOffset]::Now.ToString('o'); reason_code=$reason; reason=$text
        nav_status=($Words[1] -band 15); nav_fault=($Words[1] -shr 24)
        mission_error=(($Words[1] -shr 16) -band 255); planner_error=(($Words[1] -shr 8) -band 255)
        nav_measurement_unknown=[bool]($Words[8] -band 2097152)
        gyro_fresh=[bool]($Words[2] -band 512); gyro_calibrated=[bool]($Words[2] -band 256)
        lidar_ready=[bool]($Words[2] -band 32); avoid_lidar_ready=[bool]($Words[2] -band 16)
        body_guard_ready=[bool]($Words[2] -band 8)
        scan_id=($Words[4] -shr 16); integrated_id=($Words[4] -band 65535)
        scan_age_ticks=(($Words[3] -shr 4) -band 63); work_bad=[bool]($Words[3] -band 8)
        required_coverage=(($Words[8] -shr 17) -band 15); sampled_visibility=(($Words[8] -shr 13) -band 15)
        pose=@((Convert-Signed32 $Words[13]),(Convert-Signed32 $Words[14]),(Convert-Signed32 $Words[15]))
        goal=@((Convert-Signed32 $Words[34]),(Convert-Signed32 $Words[35]))
        map_local=[bool]($Words[17] -band 128); map_frozen=[bool]($Words[17] -band 2)
        rejected_cell_xy=@((Convert-Signed32 $Words[18]),(Convert-Signed32 $Words[19]))
        rejected_cell=(($Words[20] -shr 3) -band 3); reject_reason=(($Words[20] -shr 5) -band 15)
        mixed_epoch=[bool]($Words[47] -band 65536); words=$Words
    }
}
if ($SelfTest) {
    $fixture=New-Object 'uint32[]' 48
    $fixture[0]=0x4c440101; $fixture[1]=1441800; $fixture[8]=2097152
    $fixture[13]=4294967295; $fixture[39]=6
    $decoded=Convert-Snapshot $fixture
    if ($decoded.mission_error -ne 22 -or $decoded.nav_status -ne 8 -or !$decoded.nav_measurement_unknown -or $decoded.pose[0] -ne -1) { throw 'Snapshot self-test failed' }
    if ((Convert-Ack '[ACK OK] 100,10,0,0,0,-1' 100) -ne 4294967295) { throw 'ACK signed word failed' }
    Write-Output 'PASS: snapshot/ACK decode, signed32, mission22; no serial port opened.'
    exit 0
}
if (!$Port) { throw 'Specify -Port COM8 (replace with the actual Bluetooth serial port).' }
function Read-Word($Serial,[int]$Sequence,[int]$Field,[int]$Value) {
    $Serial.WriteLine("#$Sequence,10,0,$Field,$Value")
    $deadline=[DateTime]::UtcNow.AddSeconds(3)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line=$Serial.ReadLine().Trim() } catch [System.TimeoutException] { continue }
        $word=Convert-Ack $line $Sequence
        if ($null -ne $word) { return $word }
    }
    throw 'No diagnostic ACK. Close VOFA and check serial port/FPGA build; no movement was requested.'
}
$absoluteOutput=$ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Output)
[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($absoluteOutput)) | Out-Null
$serial=New-Object System.IO.Ports.SerialPort($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$serial.ReadTimeout=200; $serial.WriteTimeout=1000; $serial.NewLine="`n"
$writer=$null
try {
    $serial.Open()
    $writer=New-Object IO.StreamWriter($absoluteOutput,$true,(New-Object Text.UTF8Encoding($false)))
    $sequence=10000
    for ($sample=0;$sample -lt $Count;$sample++) {
        $timer=[Diagnostics.Stopwatch]::StartNew()
        Read-Word $serial $sequence 1 0 | Out-Null; $sequence++
        $words=New-Object 'uint32[]' 48
        for ($index=0;$index -lt 48;$index++) {
            $words[$index]=Read-Word $serial $sequence 0 $index; $sequence++
        }
        $record=Convert-Snapshot $words
        $json=ConvertTo-Json -InputObject $record -Compress -Depth 8
        $writer.WriteLine($json); $writer.Flush(); Write-Output $json
        $delay=[Math]::Max(0,$Interval*1000-$timer.ElapsedMilliseconds)
        if ($delay -gt 0) { Start-Sleep -Milliseconds $delay }
    }
    Write-Host "Saved: $absoluteOutput"
} finally {
    if ($writer) { $writer.Dispose() }
    $serial.Dispose()
}
