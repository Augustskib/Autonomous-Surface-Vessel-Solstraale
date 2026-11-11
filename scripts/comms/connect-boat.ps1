<# Connect the operator browser and Foxglove to the boat over SSH.
   Does not change router settings, start propulsion, or disable host-key checks.
   Example: ./connect-boat.ps1 -BoatHost asv-pi.local -PiUser august
   Keep this terminal open. Ctrl+C closes the tunnel.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9.:-]*$')]
    [string]$BoatHost,
    [Parameter(Mandatory=$true)]
    [ValidatePattern('^[A-Za-z_][A-Za-z0-9_-]*$')]
    [string]$PiUser,
    [switch]$DryRun
)
$ErrorActionPreference = 'Stop'
$sshArgs = @(
    '-N', '-T',
    '-o', 'ExitOnForwardFailure=yes',
    '-o', 'ServerAliveInterval=5',
    '-o', 'ServerAliveCountMax=3',
    '-L', '127.0.0.1:5000:127.0.0.1:5000',
    '-L', '127.0.0.1:8765:127.0.0.1:8765',
    '-l', $PiUser, $BoatHost
)
Write-Host 'Dashboard: http://127.0.0.1:5000'
Write-Host 'Foxglove connection: ws://127.0.0.1:8765'
Write-Host 'The Pi must be reachable and its boat services must already be running.'
if ($DryRun) {
    Write-Output ('ssh ' + ($sshArgs -join ' '))
    return
}
$ssh = Get-Command ssh -CommandType Application -ErrorAction Stop
& $ssh.Source @sshArgs
if ($LASTEXITCODE -ne 0) {
    throw "SSH tunnel ended with code $LASTEXITCODE. Check the boat address, credentials, network, and whether local ports are already in use."
}
