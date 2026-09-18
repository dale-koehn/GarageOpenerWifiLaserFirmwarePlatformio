<#
.SYNOPSIS
  Build, flash, and monitor the GarageOpenerWifiLaser firmware via PlatformIO.

.DESCRIPTION
  Wraps the PlatformIO CLI so you don't have to remember environment names and
  upload-port flags. Run it with no arguments for an interactive menu, or pass
  switches to drive it non-interactively (e.g. from another script or CI).

.PARAMETER Build
  Compile the firmware without flashing it.

.PARAMETER FlashSerial
  Build and flash over USB/serial. Prompts for a COM port if -Port isn't given.

.PARAMETER FlashOTA
  Build and flash over WiFi (ArduinoOTA). Prompts for an IP/hostname if
  -IPAddress isn't given.

.PARAMETER Monitor
  Open the serial monitor. Prompts for a COM port if -Port isn't given.

.PARAMETER Port
  COM port to use for -FlashSerial or -Monitor (e.g. COM8). Skips the picker.

.PARAMETER IPAddress
  Device IP address or mDNS hostname to use for -FlashOTA
  (e.g. 192.168.1.90 or garage-opener_v3-esp8266.local). Skips the prompt.

.PARAMETER Environment
  Override the PlatformIO environment name (defaults: "esp12e" for
  Build/FlashSerial/Monitor, "esp12e_ota" for FlashOTA).

.EXAMPLE
  .\deploy.ps1
  Shows the interactive menu.

.EXAMPLE
  .\deploy.ps1 -Build

.EXAMPLE
  .\deploy.ps1 -FlashSerial -Port COM8

.EXAMPLE
  .\deploy.ps1 -FlashOTA -IPAddress 192.168.1.93

.EXAMPLE
  .\deploy.ps1 -Monitor -Port COM8
#>

[CmdletBinding()]
param(
    [switch]$Build,
    [switch]$FlashSerial,
    [switch]$FlashOTA,
    [switch]$Monitor,
    [string]$Port,
    [string]$IPAddress,
    [string]$Environment
)

$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot

$DefaultBuildEnv  = "esp12e"
$DefaultSerialEnv = "esp12e"
$DefaultOtaEnv    = "esp12e_ota"

function Get-PioExe {
    $cmd = Get-Command pio -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $fallback = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
    if (Test-Path $fallback) { return $fallback }

    Write-Error "Could not find the PlatformIO CLI ('pio'). Install PlatformIO or add it to PATH."
    exit 1
}

$Pio = Get-PioExe

function Invoke-Pio {
    param([string[]]$PioArgs)
    Write-Host ">> pio $($PioArgs -join ' ')" -ForegroundColor DarkGray
    # Pipe through Out-Host: without it, assigning this function's result to a
    # variable (as the callers below do, to capture the exit code) silently
    # swallows pio's own console output into that variable instead of
    # displaying it - a well-known PowerShell pipeline-capture gotcha.
    & $Pio @PioArgs | Out-Host
    return $LASTEXITCODE
}

function Get-SerialPorts {
    [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
}

function Select-SerialPort {
    $ports = @(Get-SerialPorts)
    if ($ports.Count -eq 0) {
        Write-Warning "No serial ports detected."
    } else {
        Write-Host "Available serial ports:"
        for ($i = 0; $i -lt $ports.Count; $i++) {
            Write-Host ("  [{0}] {1}" -f ($i + 1), $ports[$i])
        }
    }

    $choice = Read-Host "Enter a port number from the list above, or type a port name (e.g. COM8)"
    $index = 0
    if ([int]::TryParse($choice, [ref]$index) -and $index -ge 1 -and $index -le $ports.Count) {
        return $ports[$index - 1]
    }
    if ([string]::IsNullOrWhiteSpace($choice)) {
        return $null
    }
    return $choice
}

function Invoke-Build {
    param([string]$TargetEnv)
    Write-Host "`nBuilding '$TargetEnv'..." -ForegroundColor Cyan
    return Invoke-Pio @("run", "-e", $TargetEnv)
}

function Invoke-FlashSerial {
    param([string]$TargetEnv, [string]$TargetPort)
    if (-not $TargetPort) {
        $TargetPort = Select-SerialPort
    }
    if (-not $TargetPort) {
        Write-Error "No serial port selected."
        return 1
    }
    Write-Host "`nFlashing '$TargetEnv' over serial on $TargetPort..." -ForegroundColor Cyan
    return Invoke-Pio @("run", "-e", $TargetEnv, "-t", "upload", "--upload-port", $TargetPort)
}

function Invoke-FlashOTA {
    param([string]$TargetEnv, [string]$TargetIP)
    if (-not $TargetIP) {
        $TargetIP = Read-Host "Enter device IP address or hostname (e.g. 192.168.1.90 or garage-opener_v3-esp8266.local)"
    }
    if ([string]::IsNullOrWhiteSpace($TargetIP)) {
        Write-Error "No IP address / hostname provided."
        return 1
    }
    Write-Host "`nFlashing '$TargetEnv' over OTA to $TargetIP..." -ForegroundColor Cyan
    return Invoke-Pio @("run", "-e", $TargetEnv, "-t", "upload", "--upload-port", $TargetIP)
}

function Invoke-Monitor {
    # Deliberately does NOT go through Invoke-Pio and does not return/capture
    # anything: `pio device monitor` needs a real interactive terminal, and
    # piping or capturing a native command's output anywhere in the call
    # chain (even just `| Out-Null` at the call site) makes PowerShell
    # redirect its stdio, which pio then rejects with
    # "requires an interactive terminal on stdin". Callers must invoke this
    # as a bare statement - never `$x = Invoke-Monitor ...` or
    # `Invoke-Monitor ... | Out-Null`.
    param([string]$TargetEnv, [string]$TargetPort)
    if (-not $TargetPort) {
        $TargetPort = Select-SerialPort
    }
    if (-not $TargetPort) {
        Write-Error "No serial port selected."
        return
    }
    $MonitorArgs = @("device", "monitor", "-e", $TargetEnv, "--port", $TargetPort)
    Write-Host "`nOpening serial monitor on $TargetPort (Ctrl+C to exit)..." -ForegroundColor Cyan
    Write-Host ">> pio $($MonitorArgs -join ' ')" -ForegroundColor DarkGray
    & $Pio @MonitorArgs
}

function Show-Menu {
    Write-Host ""
    Write-Host "=== GarageOpenerWifiLaser deploy ===" -ForegroundColor Yellow
    Write-Host "  1) Build"
    Write-Host "  2) Flash via serial"
    Write-Host "  3) Flash via OTA"
    Write-Host "  4) Monitor serial"
    Write-Host "  5) Exit"
    Write-Host ""
    Write-Host "Tip: skip this menu next time with -Build, -FlashSerial [-Port COMx]," -ForegroundColor DarkGray
    Write-Host "     -FlashOTA [-IPAddress x.x.x.x], or -Monitor [-Port COMx]." -ForegroundColor DarkGray
}

function Start-Menu {
    while ($true) {
        Show-Menu
        $choice = Read-Host "Choose an option"
        switch ($choice) {
            "1" { Invoke-Build -TargetEnv $DefaultBuildEnv | Out-Null }
            "2" { Invoke-FlashSerial -TargetEnv $DefaultSerialEnv -TargetPort $null | Out-Null }
            "3" { Invoke-FlashOTA -TargetEnv $DefaultOtaEnv -TargetIP $null | Out-Null }
            "4" { Invoke-Monitor -TargetEnv $DefaultSerialEnv -TargetPort $null }
            "5" { return }
            default { Write-Warning "Invalid choice: $choice" }
        }
    }
}

$RequestedAction = $Build -or $FlashSerial -or $FlashOTA -or $Monitor

if (-not $RequestedAction) {
    Start-Menu
    exit 0
}

$BuildEnv  = if ($Environment) { $Environment } else { $DefaultBuildEnv }
$SerialEnv = if ($Environment) { $Environment } else { $DefaultSerialEnv }
$OtaEnv    = if ($Environment) { $Environment } else { $DefaultOtaEnv }

if ($Build) {
    $code = Invoke-Build -TargetEnv $BuildEnv
    if ($code -ne 0) { exit $code }
}
if ($FlashSerial) {
    $code = Invoke-FlashSerial -TargetEnv $SerialEnv -TargetPort $Port
    if ($code -ne 0) { exit $code }
}
if ($FlashOTA) {
    $code = Invoke-FlashOTA -TargetEnv $OtaEnv -TargetIP $IPAddress
    if ($code -ne 0) { exit $code }
}
if ($Monitor) {
    Invoke-Monitor -TargetEnv $SerialEnv -TargetPort $Port
}

exit 0
