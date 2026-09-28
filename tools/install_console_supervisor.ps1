param(
    [string]$CarHost = "192.168.19.178",
    [string]$User = "pi",
    [string]$Password = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Python = "C:\Users\46361\AppData\Local\Programs\Python312-embed\python.exe"
$Supervisor = Join-Path $PSScriptRoot "console_supervisor.py"
$ConfigDir = Join-Path $env:LOCALAPPDATA "ArmPiConsole"
$Config = Join-Path $ConfigDir "console_supervisor.json"
$Runner = Join-Path $ConfigDir "run_console_supervisor.ps1"
$RunKey = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"

if (-not (Test-Path -LiteralPath $Python)) {
    throw "Python runtime not found: $Python"
}
if (-not (Test-Path -LiteralPath $Supervisor)) {
    throw "Supervisor script not found: $Supervisor"
}
if (-not $Password) {
    $secure = Read-Host "SSH password for $User@$CarHost" -AsSecureString
    $ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
    try { $Password = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
}

New-Item -ItemType Directory -Force -Path $ConfigDir | Out-Null
@{
    host = $CarHost
    user = $User
    password = $Password
} | ConvertTo-Json | Set-Content -LiteralPath $Config -Encoding UTF8

# Keep the local credential file out of the repository and readable only by
# the interactive user and Windows SYSTEM.
$identity = "$env:USERDOMAIN\$env:USERNAME"
icacls $Config /inheritance:r /grant:r "${identity}:(F)" "SYSTEM:(F)" | Out-Null

# Keep the task action itself ASCII-only. Task Scheduler on this machine
# returns error 123 when a cmd.exe action contains the Chinese workspace path.
# The generated runner can still use the full Unicode path after PowerShell
# has started.
@"
`$ErrorActionPreference = "Continue"
Set-Location -LiteralPath '$Root'
& '$Python' '$Supervisor' --config '$Config' >> '$ConfigDir\runner.log' 2>&1
exit `$LASTEXITCODE
"@ | Set-Content -LiteralPath $Runner -Encoding Unicode

$powershell = Join-Path $env:SystemRoot "System32\WindowsPowerShell\v1.0\powershell.exe"
$runCommand = "`"$powershell`" -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$Runner`""

# Use the per-user Run key instead of an elevated scheduled task. It starts
# after the desktop/network are available, works with the Unicode workspace
# path, and avoids storing a second task that can be left in a failed state.
New-Item -Path $RunKey -Force | Out-Null
New-ItemProperty -Path $RunKey -Name "ArmPiConsoleSupervisor" -Value $runCommand -PropertyType String -Force | Out-Null
Get-ScheduledTask -TaskName "ArmPi Console Supervisor" -ErrorAction SilentlyContinue |
    Unregister-ScheduledTask -Confirm:$false -ErrorAction SilentlyContinue
Start-Process -WindowStyle Hidden -FilePath $powershell -ArgumentList @(
    "-NoProfile", "-WindowStyle", "Hidden", "-ExecutionPolicy", "Bypass", "-File", $Runner
)

Write-Output "Installed and started: ArmPiConsoleSupervisor (HKCU Run)"
Write-Output "Config: $Config"
