# Start ONLY the CityLife simulator on this desktop for a controller that runs
# on the Jetson (deploy/orin/run_follow_controller.sh). Nothing flies from here.
# It prints the address Project AirSim listens on: 0.0.0.0 (or this PC's LAN
# address) is reachable from the Jetson; 127.0.0.1 is not.
#
#   .\deploy\orin\start_sim_for_jetson.ps1                 # window, uncapped
#   .\deploy\orin\start_sim_for_jetson.ps1 -SimMaxFps 30 -SimOffscreen
#   .\deploy\orin\start_sim_for_jetson.ps1 -Stop           # stop it again
#
# Windows Firewall must allow inbound TCP 8989-8990 to UnrealEditor on the
# network the Jetson is on (allow it when Windows asks the first time, or add
# a rule yourself). Like the other launchers, this restarts a running game
# simulator of Blocks.uproject and refuses while an EDITOR is open on it.
param(
    [int]$SimWidth = 960,
    [int]$SimHeight = 540,
    [int]$SimMaxFps = 0,
    [switch]$SimOffscreen,
    [switch]$Stop
)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
. "$Root/tools/Resolve-UnrealEngine.ps1"
$UE = Get-UnrealEditor -UProject $Proj
$Map = "/Game/CityLife/Maps/CityLife_Day"

function Stop-OurGameSim {
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and $_.CommandLine -like '*-game*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}
if ($Stop) { Stop-OurGameSim; Write-Host "==> simulator stopped"; return }

$editors = @(Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
             Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and $_.CommandLine -notlike '*-game*' })
if ($editors.Count -gt 0) { throw "an Unreal EDITOR is open on this project (PID $($editors[0].ProcessId)); save and close it" }

Stop-OurGameSim
Start-Sleep -Seconds 4
$ueArgs = @("`"$Proj`"", $Map, '-game', '-windowed', "-ResX=$SimWidth", "-ResY=$SimHeight")
if ($SimMaxFps -gt 0) { $ueArgs += "-ExecCmds=`"t.MaxFPS $SimMaxFps`"" }
if ($SimOffscreen) { $ueArgs += '-RenderOffscreen' }
Start-Process -FilePath $UE -ArgumentList $ueArgs | Out-Null
Write-Host "==> simulator starting on $Map ..."
for ($i = 0; $i -lt 90; $i++) {
    Start-Sleep -Seconds 5
    $l = @(Get-NetTCPConnection -LocalPort 8989 -State Listen -ErrorAction SilentlyContinue)
    if ($l.Count -gt 0) {
        Start-Sleep -Seconds 8
        $addr = ($l | ForEach-Object { $_.LocalAddress }) -join ", "
        Write-Host "==> simulator ready; Project AirSim listens on $addr (ports 8989, 8990)"
        if ($addr -match '^(127\.0\.0\.1|::1)(, (127\.0\.0\.1|::1))*$') {
            Write-Warning "it listens on localhost only: the Jetson cannot reach it"
        }
        $ips = Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' }
        Write-Host "==> this PC's addresses: $(($ips | ForEach-Object { "$($_.IPAddress) ($($_.InterfaceAlias))" }) -join '; ')"
        Write-Host "==> on the Jetson: SIM=<one of these> bash deploy/orin/run_follow_controller.sh"
        return
    }
}
throw "simulator did not open port 8989"
