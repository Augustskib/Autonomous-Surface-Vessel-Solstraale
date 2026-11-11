<# Connect the operator browser and Foxglove to the boat over SSH.
   Does not change router settings, start propulsion, or disable host-key checks.
   Example: ./connect-boat.ps1 -BoatHost asv-pi.local -PiUser august
   Keep this terminal open. Ctrl+C closes the tunnel.
   If the link drops, the tunnel is reopened automatically. With SSH keys set
   up this needs no input; with a password you are asked again each time.
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
while ($true) {
    & $ssh.Source @sshArgs
    Write-Warning "Tunnel closed (code $LASTEXITCODE). Reconnecting in 2 s. Ctrl+C to stop."
    Start-Sleep -Seconds 2
}
