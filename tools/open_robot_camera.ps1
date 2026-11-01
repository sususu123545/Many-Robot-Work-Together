[CmdletBinding()]
param(
    [string]$RobotIp = '100.102.57.19',
    [string]$SshUser = 'pi',
    [int]$LocalPort = 18080,
    [int]$RemotePort = 8080
)

$ErrorActionPreference = 'Stop'

# Windows Forms is available in Windows PowerShell 5.1. Relaunch there when
# started from PowerShell 7, using the same visible console.
if ($PSVersionTable.PSEdition -ne 'Desktop') {
    $windowsPowerShell = Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
    if (-not (Test-Path -LiteralPath $windowsPowerShell)) {
        throw 'Windows PowerShell 5.1 is required for the camera window.'
    }
    & $windowsPowerShell -NoProfile -STA -ExecutionPolicy Bypass -File $PSCommandPath `
        -RobotIp $RobotIp -SshUser $SshUser -LocalPort $LocalPort -RemotePort $RemotePort
    exit $LASTEXITCODE
}

$sshPath = Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'
$viewerSourcePath = Join-Path $PSScriptRoot 'RobotCameraViewer.cs'
if (-not (Test-Path -LiteralPath $sshPath)) { throw "OpenSSH not found: $sshPath" }
if (-not (Test-Path -LiteralPath $viewerSourcePath)) { throw "Viewer source not found: $viewerSourcePath" }

function Test-LocalPort([int]$Port) {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $pending = $client.BeginConnect('127.0.0.1', $Port, $null, $null)
        if (-not $pending.AsyncWaitHandle.WaitOne(700)) { return $false }
        $client.EndConnect($pending)
        return $client.Connected
    } catch {
        return $false
    } finally {
        $client.Dispose()
    }
}

$tunnelProcess = $null
if (-not (Test-LocalPort $LocalPort)) {
    $forward = "${LocalPort}:127.0.0.1:${RemotePort}"
    $sshArgs = @(
        '-N', '-T', '-L', $forward,
        '-o', 'ExitOnForwardFailure=yes',
        '-o', 'ServerAliveInterval=30',
        '-o', 'ServerAliveCountMax=3',
        '-o', 'HostKeyAlias=raspberrypi',
        '-o', 'StrictHostKeyChecking=yes',
        "$SshUser@$RobotIp"
    )

    Write-Host "Connecting to $RobotIp. Enter the SSH password if prompted. Keep this window open while viewing."
    $tunnelProcess = Start-Process -FilePath $sshPath -ArgumentList $sshArgs -PassThru -NoNewWindow
    $ready = $false
    for ($i = 0; $i -lt 120; $i++) {
        if ($tunnelProcess.HasExited) { throw "SSH tunnel exited: $($tunnelProcess.ExitCode)" }
        if (Test-LocalPort $LocalPort) { $ready = $true; break }
        Start-Sleep -Seconds 1
    }
    if (-not $ready) {
        Stop-Process -Id $tunnelProcess.Id -Force -ErrorAction SilentlyContinue
        throw "Timed out waiting for local port $LocalPort. Check the car IP, Wi-Fi, and SSH login."
    }
} else {
    Write-Host "Using the existing local camera tunnel on port $LocalPort."
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -Path $viewerSourcePath -ReferencedAssemblies @('System.Windows.Forms', 'System.Drawing')
$streamUrl = "http://127.0.0.1:${LocalPort}/stream?topic=%2Frear_camera%2Fimage_raw&type=mjpeg&qos_profile=sensor_data"

try {
    [RobotCameraViewer]::Run($streamUrl)
} finally {
    if ($tunnelProcess -and -not $tunnelProcess.HasExited) {
        Stop-Process -Id $tunnelProcess.Id -Force -ErrorAction SilentlyContinue
    }
}
