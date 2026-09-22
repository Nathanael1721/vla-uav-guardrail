# Fly the follow mission inside CityLife_Day, against the level's own crowd.
#
# The difference from run_retarget_demo.ps1 is what supplies the scene. There,
# the client spawns everything it follows: baked GLB figures it teleports once
# per tick, and a scripted car. Here the LEVEL walks 16 pedestrians and drives 8
# cars on its own, so the client spawns nothing, spends no per-tick RPC on
# scenery, and reads ground truth back out of the simulator with
# --level-peds (see demo/level_actors.py).
#
#     .\scripts\run_citylife_follow.ps1
#     .\scripts\run_citylife_follow.ps1 -Seconds 300 -Tag citylife_long
#
# WARNING: like every demo script here, this restarts the simulator, which kills
# any process holding Blocks.uproject - INCLUDING an editor you have open. Close
# the editor first. The match is on the .uproject, not the process name, so
# unrelated Unreal projects are left alone.

param(
    [string]$Object  = "a person",      # the 10 m pedestrian rule, from t=0
    [int]$Seconds    = 180,
    [int]$LevelPeds  = 16,              # Ped_00..Ped_15, by TAG (see below)
    [string]$Tag     = "citylife_follow",
    [int]$SimWidth   = 960,
    [int]$SimHeight  = 540,
    [switch]$SkipSim
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
. "$Root/tools/Resolve-UnrealEngine.ps1"
$UE   = Get-UnrealEditor -UProject $Proj
# The ONLY thing that chooses the level is this command-line argument - the
# .jsonc configs under demo/pas_config attach actors to a running sim, they do
# not open a map.
$Map  = "/Game/CityLife/Maps/CityLife_Day"

function Say($m) { Write-Host "==> $m" -ForegroundColor Cyan }
foreach ($p in @($Py, $UE, $Proj)) { if (-not (Test-Path $p)) { throw "not found: $p" } }
Set-Location $Root

function Test-SimUp { (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded }

function Stop-OurSim {
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

function Start-Sim {
    Stop-OurSim
    Start-Sleep -Seconds 4
    Start-Process -FilePath $UE -ArgumentList "`"$Proj`"", $Map, '-game', '-windowed', "-ResX=$SimWidth", "-ResY=$SimHeight" | Out-Null
    Say "simulator starting on $Map ..."
    for ($i = 0; $i -lt 90; $i++) {
        Start-Sleep -Seconds 5
        if (Test-SimUp) { Start-Sleep -Seconds 8; Say "simulator ready"; return }
    }
    throw "simulator did not open port 8989"
}

$editors = @(Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
             Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and
                            $_.CommandLine -notlike '*-game*' })
if ($editors.Count -gt 0 -and -not $SkipSim) {
    Write-Host ""
    Write-Host "  An Unreal EDITOR is open on this project (PID $($editors[0].ProcessId))." -ForegroundColor Yellow
    Write-Host "  Starting the sim will close it. Save and close it first, or pass -SkipSim" -ForegroundColor Yellow
    Write-Host "  if a sim is already listening on 8989." -ForegroundColor Yellow
    Write-Host ""
    throw "editor open - refusing to kill it without being asked"
}

if ($SkipSim -and -not (Test-SimUp)) { throw "-SkipSim given but nothing on 8989" }
if (-not $SkipSim) { Start-Sim }

Say "following `"$Object`" for ${Seconds}s against $LevelPeds level pedestrians"

# --det-thresh 0.008 is the value every recorded pedestrian flight used. It
# matters here too: OWL-ViT scored the crowd figures 0.05-0.11 at 10 m, no
# better than the chrome mannequins they replaced, so a low threshold is the
# measured condition, not a thumb on the scale.
#
# No --pedestrians and no --parked: the level already walks its own, and
# spawning more would put two crowds in one street and spend RPC doing it.
# --no-car for the same reason - the level drives eight.
$a = @("demo\follow_vlm.py",
       "--object", $Object,
       "--tag", $Tag,
       "--policy", "policies\follow_pedestrian.yaml",
       "--max-s", "$Seconds",
       "--det-thresh", "0.008",
       "--want-width", "0.16",
       "--cruise-alt", "8",
       "--lock-target",
       "--no-car",
       "--level-peds", "$LevelPeds",
       "--save-view")
& $Py @a

Say "done. Read the result from the artefact, not the screen:"
Write-Host "    demo\out\$Tag\metrics.json     -> det_hz, frac_on_target, pedestrian_truth"
Write-Host "    demo\out\$Tag\kpi.json         -> p0_violation_escape_rate must be 0.0"
Write-Host "  then build the video:"
Write-Host "    python tools\make_demo_video.py --tag $Tag --height 720"
