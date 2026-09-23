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
# THE CAR MISSION. Follow one car the LEVEL drives, named by colour:
#
#     .\scripts\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
#         -Seconds 240 -Tag citylife_redcar
#
# The truth is then that ONE car (by tag), so the tracking score has a single
# subject and its null cannot saturate the way forty pedestrians did.
#
# WARNING: like every demo script here, this restarts the simulator, which kills
# any process holding Blocks.uproject - INCLUDING an editor you have open. Close
# the editor first. The match is on the .uproject, not the process name, so
# unrelated Unreal projects are left alone.

param(
    [string]$Object  = "a person",      # the 10 m pedestrian rule, from t=0
    [int]$Seconds    = 180,
    [int]$LevelPeds  = 40,              # Ped_00..Ped_39, by TAG (see below)
    [double]$WantRange = 12.0,          # metres; 0 would derive it from --want-width
    [double]$TruthPeriod = 0.5,         # s between pose polls; see the note below
    [string]$Tag     = "citylife_follow",
    [string]$LevelCar = "",             # e.g. Car_10: follow a car the level drives
    [int]$StartWhenSeen = 5,            # consecutive plausible boxes before t0; 0 = off
    [double]$StartTimeout = 240,
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
# --level-truth-period 0.5, not the 0.1 default: the simulator loops the names
# on the game thread, so 40 names at 10 Hz is 400 game-thread round trips a
# second and the control loop pays for every one of them. Measured: 4.04 Hz
# loop at 0.1 s. Ground truth at 2 Hz is still finer than the 1.4 m/s subject
# moves between polls.
#
# --want-range, not --want-width: 0.16 of frame width was calibrated for a 4 m
# car, and on a 0.5 m person it asks for a 2 m stand-off - inside the camera's
# 6.9 m blind spot at 8 m altitude, so the subject leaves frame before the
# aircraft arrives. 12 m sits outside both that and the policy's 10 m ring.
#
# No --pedestrians and no --parked: the level already walks its own, and
# spawning more would put two crowds in one street and spend RPC doing it.
# --no-car for the same reason - the level drives eight.
# --presence-gates-control: a box the presence verdict rejects (a "person" 2.9 m
# wide at 75 m, a wall-sized box, the wrong colour) may not steer. Without it the
# aircraft chased building facades on citylife_city while its own check said
# ABSENT on 83 % of ticks.
#
# --start-when-seen: the level's traffic does not wait for the aircraft, so the
# mission clock starts when the DETECTOR has produced that many plausible boxes
# in a row - the evidence the controller steers by - not when truth says so.
#
# Policy: follow_pedestrian.yaml for both missions. Besides the 10 m pedestrian
# ring it carries the catch-all 5 m SubjectStandoff (subject_class "*"), which
# is the rule that binds a car; the retarget demo followed its car under it.
$a = @("demo\follow_vlm.py",
       "--object", $Object,
       "--tag", $Tag,
       "--policy", "policies\follow_pedestrian.yaml",
       "--max-s", "$Seconds",
       "--det-thresh", "0.008",
       "--cruise-alt", "8",
       "--lock-target",
       "--no-car",
       "--presence-gates-control",
       "--start-when-seen", "$StartWhenSeen",
       "--start-timeout-s", "$StartTimeout",
       "--level-peds", "$LevelPeds",
       "--level-truth-period", "$TruthPeriod",
       "--save-view")
if ($LevelCar) {
    # A car: 0.16 of frame width is the calibration made for a 4 m car (15.8 m
    # stand-off), and the subject's truth is the one tag.
    $a += @("--level-car", $LevelCar, "--want-width", "0.16")
} else {
    $a += @("--want-range", "$WantRange")
}
& $Py @a

Say "done. Read the result from the artefact, not the screen:"
Write-Host "    demo\out\$Tag\metrics.json     -> det_hz, frac_on_target (+ _chance), instance_score, start_gate, stage_ms_median"
Write-Host "    demo\out\$Tag\kpi.json         -> p0_violation_escape_rate must be 0.0"
Write-Host "  then build the video:"
Write-Host "    python tools\make_demo_video.py --tag $Tag --height 720"
