<#!
Read-only Bluetooth serial-port mapping. No pairing, COM opening, device reset,
AT command or robot command is performed. Uses registry rather than CIM/WMI.
!#>
$ErrorActionPreference = 'Stop'
$deviceRoot = 'HKLM:\SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices'
$knownNames = @{}
if (Test-Path -LiteralPath $deviceRoot) {
    foreach ($deviceKey in Get-ChildItem -LiteralPath $deviceRoot) {
        $properties = Get-ItemProperty -LiteralPath $deviceKey.PSPath
        $label = if ($properties.Name -is [byte[]]) {
            [Text.Encoding]::UTF8.GetString($properties.Name).Trim([char]0)
        } else { [string]$properties.Name }
        $knownNames[$deviceKey.PSChildName.ToLowerInvariant()] = $label
    }
}
$ports = @()
$enumRoot = 'HKLM:\SYSTEM\CurrentControlSet\Enum\BTHENUM'
if (Test-Path -LiteralPath $enumRoot) {
    foreach ($serviceKey in Get-ChildItem -LiteralPath $enumRoot | Where-Object { $_.PSChildName -like '*00001101*' }) {
        foreach ($instanceKey in Get-ChildItem -LiteralPath $serviceKey.PSPath) {
            $parameters = Get-ItemProperty -LiteralPath ($instanceKey.PSPath + '\Device Parameters') -ErrorAction SilentlyContinue
            if (-not $parameters.PortName) { continue }
            $address = $null
            $direction = 'Unknown: verify COM Ports tab'
            if ($instanceKey.PSChildName -match '([0-9a-fA-F]{12})_C[0-9a-fA-F]+$') {
                $address = $Matches[1].ToLowerInvariant()
                $direction = 'Remote-bound (usually Outgoing)'
            } elseif ($instanceKey.PSChildName -match '000000000000_00000000$') {
                $direction = 'Local listener (usually Incoming)'
            }
            $ports += [pscustomobject]@{
                Port = $parameters.PortName
                Mapping = $direction
                RemoteAddress = $address
                RemoteName = if ($address -and $knownNames.ContainsKey($address)) { $knownNames[$address] } else { '' }
            }
        }
    }
}
if ($ports.Count) { $ports | Sort-Object Port | Format-Table -AutoSize }
else { Write-Output 'No registered Bluetooth SPP serial ports found.' }
Write-Output 'Paired-device records (may include stale devices):'
$knownNames.GetEnumerator() | Sort-Object Value | ForEach-Object {
    [pscustomobject]@{ Address = $_.Key; Name = $_.Value }
} | Format-Table -AutoSize
Write-Output 'This reads stored mappings, not live connection state. Verify the robot name/address before selecting a port.'
