# Fly the follow mission inside CityLife_Day, against the level's own crowd.
#
# The difference from run_retarget_demo.ps1 is what supplies the scene. There,
# the client spawns everything it follows: baked GLB figures it teleports once
# per tick, and a scripted car. Here the LEVEL walks 40 pedestrians and drives 24
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
    [double]$StartMaxRange = 45,        # car mode: acquire only within this range
    [double]$CruiseAlt = 0,             # 0 = the default, 8 m (see below)
    [int]$SimWidth   = 960,
    [int]$SimHeight  = 540,
    [switch]$NoTrail,                   # car mode: fly along the nose, as before 09-24
    [switch]$KeepSim,                   # leave the simulator running afterwards
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

# The teardown's kill: only the -game simulator. An editor opened on the project
# DURING the flight (to look at a car, say) is somebody's work, and the start-up
# guard below only protects the one that was open before.
function Stop-OurGameSim {
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and $_.CommandLine -like '*-game*' } |
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

# THE MAP OF THIS LEVEL, not Demo_day's. Every map under demo\out\citymap is the
# 160 m cube surveyed on Demo_day; CityLife's loops run 30-120 m past it, and off
# that map the Shield read a junction entrance as 35 m inside a building. Built by
# build_voxel_map.py over NED x -140..220, y -60..140 (the FINDING's fourth part).
# Refuse to fly without it rather than fall back to the wrong level's map - and
# refuse BEFORE starting a simulator, not after, or the throw leaves it running.
$CityMap = Join-Path $Root "demo\out\citymap_citylife\occ_day.npz"
if (-not (Test-Path $CityMap)) {
    throw ("no CityLife obstacle map at $CityMap. With the sim running CityLife_Day: " +
           "python demo\build_voxel_map.py --x-min -140 --x-max 220 --y-min -60 --y-max 140 " +
           "--out-dir demo\out\citymap_citylife --bands ground_0to2:0:2,ground_2to4:2:4," +
           "occ_day_flightband_6to14:6:14,occ_day_highband_15to55:15:55 " +
           "--alias occ_day=occ_day_flightband_6to14; then " +
           "python demo\build_street_mask.py --dir demo\out\citymap_citylife")
}

if ($SkipSim -and -not (Test-SimUp)) { throw "-SkipSim given but nothing on 8989" }

# Everything from here runs under try/finally: a -game simulator left running
# keeps simulating. The one left up after the 2026-09-23 flights logged 12 349
# pedestrian contacts overnight, which read like evidence from the flight until
# the timestamps were checked. A throw or a Ctrl+C must not leave one behind.
try {
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
# --no-car for the same reason - the level drives its own 24.
# --presence-gates-control: a box the presence verdict rejects (a "person" 2.9 m
# wide at 75 m, a wall-sized box, the wrong colour) may not steer. Without it the
# aircraft chased building facades on citylife_city while its own check said
# ABSENT on 83 % of ticks.
#
# --start-when-seen: the level's traffic does not wait for the aircraft, so the
# mission clock starts when the DETECTOR has produced that many plausible boxes
# in a row - the evidence the controller steers by - not when truth says so.
#
# Policy: follow_pedestrian.yaml for a person; follow_car_citylife.yaml for the
# car. The car mission first flew under the pedestrian policy for its catch-all
# 5 m stand-off, and inherited a 3.0 m/s cap against a 3.2 m/s car and a 5 m
# clearance ring a mapped obstacle violates at the start point: it could not
# keep up by construction. follow_car_citylife.yaml keeps both stand-offs and
# takes follow_car.yaml's envelope (5 m/s, 3 m clearance, 6-14 m).
$Policy = if ($LevelCar) { "policies\follow_car_citylife.yaml" } else { "policies\follow_pedestrian.yaml" }
# Cruise altitude: 8 m for both missions. 12 m was flown (citylife_redcar_high)
# and overshot the car policy's 14 m ceiling for 3.1 s (peak 14.32 m), each climb
# during a phantom-wall repair (docs/FINDING-crowd-pedestrians-and-traffic.md,
# third part), and the detector's hit rate fell from 0.71 to 0.32.
if ($CruiseAlt -le 0) { $CruiseAlt = 8 }
$a = @("demo\follow_vlm.py",
       "--object", $Object,
       "--tag", $Tag,
       "--policy", $Policy,
       "--max-s", "$Seconds",
       "--det-thresh", "0.008",
       "--cruise-alt", "$CruiseAlt",
       "--lock-target",
       "--no-car",
       "--presence-gates-control",
       "--start-when-seen", "$StartWhenSeen",
       "--start-timeout-s", "$StartTimeout",
       "--level-peds", "$LevelPeds",
       "--level-truth-period", "$TruthPeriod",
       "--citymap", $CityMap,
       "--save-view")
if ($LevelCar) {
    # A car: 0.16 of frame width is the calibration made for a 4 m car (15.8 m
    # stand-off), and the subject's truth is the one tag.
    $a += @("--level-car", $LevelCar, "--want-width", "0.16")
    # Only a NEAR car is a start: acquired at 140 m it was 16 px, driving away
    # at 3.2 m/s from an aircraft capped at 4 m/s, and was never closed on.
    # The car passes the start point once a lap, so the gate waits for that.
    $a += @("--start-max-range-m", "$StartMaxRange")
    # Fly the car's TRAIL, not the nose: at a corner the nose points across the
    # corner block at a car that has already turned (demo/trail.py).
    if (-not $NoTrail) { $a += @("--trail-follow") }
    # The policy's speed cap, not the controller's 4 m/s default. At 4 m/s
    # against a 3.2 m/s car the follow closes at 0.8 m/s, so the 15-25 m a
    # corner costs takes 20-30 s to win back; citylife_redcar_trail was still
    # 40 m behind when the car turned the third corner, and lost it there.
    $a += @("--speed-max", "5.0")
} else {
    $a += @("--want-range", "$WantRange")
}
# A metrics.json left by an EARLIER flight under this tag would otherwise be
# what the closing lines point at when this one fails before writing its own.
Remove-Item (Join-Path $Root "demo\out\$Tag\metrics.json") -ErrorAction SilentlyContinue
& $Py @a
$rc = $LASTEXITCODE
}
finally {
    if (-not $KeepSim -and -not $SkipSim) {
        Stop-OurGameSim
        Say "simulator stopped (pass -KeepSim to leave it running)"
    }
}

$metrics = Join-Path $Root "demo\out\$Tag\metrics.json"
if ($rc -ne 0 -or -not (Test-Path $metrics)) {
    Write-Host ""
    Write-Host "  *** FLIGHT FAILED (exit $rc)." -ForegroundColor Red
    if (Test-Path $metrics) {
        # follow_vlm writes metrics.json before the KPI, manifest and report
        # steps, so a failure after it leaves a PARTIAL file (no kpi fields).
        Write-Host "  $metrics is PARTIAL: written before the failure, no KPI fields." -ForegroundColor Red
    } else {
        Write-Host "  No metrics.json was written for $Tag; read the console above." -ForegroundColor Red
    }
    exit $(if ($rc) { $rc } else { 1 })
}

Say "done. Read the result from the artefact, not the screen:"
Write-Host "    demo\out\$Tag\metrics.json     -> det_hz, frac_on_target (+ _chance), instance_score, start_gate, stage_ms_median, collisions, off_map_ticks, trail"
Write-Host "    demo\out\$Tag\kpi.json         -> p0_violation_escape_rate must be 0.0"
Write-Host "  then build the video:"
Write-Host "    python tools\make_demo_video.py --tag $Tag --height 720"
