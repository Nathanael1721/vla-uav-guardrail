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
# RECORDING IT FOR AERIALVLA (-RecordVla, wave FT-D 2026-10-23): the same
# flight, with AerialVLA's two cameras added to the airframe and every
# front+down pair written with its pose, the box, the command and the track
# (demo/vla_record.py), to C: through a junction like the frames; then
# python tools\build_follow_dataset.py demo\out\<Tag> ... --out <dataset>:
#
#     .\scripts\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
#         -PolicyFile policies\follow_car_citylife_nfz.yaml -Seconds 300 `
#         -Tag ft_expert_01 -RecordVla
#
# -RecordPerturb (with -RecordVla only) adds DART-style noise so the data
# shows an off-centre car being brought back: every 5-8 s, for 1.0-1.5 s, a
# +-0.3-0.5 rad/s yaw-rate offset on the executed command, before the Shield
# (follow_vlm --record-perturb; -PerturbSpeed also scales the speed 0.5-1.0).
# The schedule follows the seed: give each recording its own -Seed. The
# expert's yaw servo fights an ADDED offset, so it gives turn labels but few
# left/right phrases (vla_record.Perturber); -PerturbReplaceYaw (the offset
# instead of the expert's yaw rate while it lasts) or -PerturbYawRate 0.6,0.9
# push the car further off-centre.
#
#     .\scripts\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
#         -PolicyFile policies\follow_car_citylife_nfz.yaml -Seconds 300 `
#         -Tag ft_expert_p01 -RecordVla -RecordPerturb -Seed 101
#
# -RecordPerturbVla HOST:PORT (with -RecordVla only; not with -RecordPerturb)
# is DAgger (demo/vla_dagger.py): the fine-tuned AerialVLA, served by a
# running demo\vla_server_v2.py, flies windows of the recording - every
# 6-10 s, for 2.5-4.0 s (-PerturbVlaEvery / -PerturbVlaDur "LO,HI"), the
# executed command is vla_fly's follow step tracker on AerialVLA's latest
# step, before the Shield - and the expert's recovery from wherever it left
# the aircraft becomes the label. Start the server first (vla-real env); this
# script refuses to start the simulator when nothing listens on HOST:PORT,
# and follow_vlm refuses a server that does not answer hello as AerialVLA
# (-PerturbVlaAllowStandIn: the scripted stand-in, a plumbing dry run):
#
#     python demo\vla_server_v2.py serve --precision bf16 --attn sdpa --merge `
#         --missing-down refuse --adapter D:\models\aerialvla-ft-follow\<run>\epoch<N>
#     .\scripts\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
#         -PolicyFile policies\follow_car_citylife_nfz.yaml -Seconds 300 `
#         -Tag ft_dagger_01 -RecordVla -RecordPerturbVla 127.0.0.1:8790 -Seed 201
#
# A checkpoint trained on a prev_front dataset (the mosaic's lower tile is the
# front frame 1 s earlier) flies its windows with that tile, and its steps as
# rates if it was flown so (vla_fly --vla-bottom-tile / --step-as-rate):
# -PerturbVlaBottomTile prev_front [-PerturbVlaPrevDtS 1.0] -PerturbVlaStepAsRate 1.0.
# follow_vlm refuses a server whose adapter was trained on another tile. One
# trained on a --speed-words dataset asks with the aircraft's own speed as
# words after the object (vla_fly --vla-speed-words): -PerturbVlaSpeedWords;
# follow_vlm refuses a server whose adapter was trained otherwise.
#
# THE CEILING FLIGHT (-DecideEveryS P [-DecideDelayS D], 2026-10-24): the same
# expert follow, but what it executes is held to a VLA's decision rate - every
# P s its own command from D s before, held in between, after its controller
# and before the Shield (follow_vlm --decide-every-s / --decide-delay-s,
# demo/decide_rate.py). Its frac_within_30m reads against the expert's 0.963:
# near it, a better-trained VLA can close the gap; far below it, the decision
# rate is the bottleneck. Not with -RecordVla, -RecordPerturb or
# -RecordPerturbVla:
#
#     .\scripts\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
#         -PolicyFile policies\follow_car_citylife_nfz.yaml -Seconds 420 `
#         -Tag ceiling_p10_d08 -DecideEveryS 1.0 -DecideDelayS 0.8
#
# HIERARCHICAL CONTROL (-VlaHierarchy HOST:PORT, 2026-10-25): the fine-tuned
# AerialVLA decides the follow's intent at its own rate and this controller
# executes it at 10 Hz (follow_vlm --vla-hierarchy, demo/vla_hierarchy.py).
# With the car locked the controller flies as always, its stand-off at the
# VLA's desired range (22 m at a zero step to 12 m at a 5 m step,
# -VlaHierarchyRange "NEAR,FAR"); with the car lost the VLA's step tracker
# flies the search; a LAND holds; an answer older than 1.5 s hands everything
# back to the controller. The Shield filters every command. The airframe
# gets AerialVLA's two cameras and nothing is recorded. Start the server
# first (this script refuses to start the simulator when nothing listens on
# HOST:PORT); the tile, rate and speed-words flags are the served adapter's,
# as for -RecordPerturbVla. Not with -RecordVla, -RecordPerturb,
# -RecordPerturbVla or -DecideEveryS:
#
#     python demo\vla_server_v2.py serve --precision bf16 --attn sdpa --merge `
#         --missing-down refuse --adapter D:\models\aerialvla-ft-follow\follow_v4_r3\epoch1
#     .\scripts\run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 `
#         -PolicyFile policies\follow_car_citylife_nfz.yaml -Seconds 420 `
#         -Tag hier_v4r3_01 -VlaHierarchy 127.0.0.1:8790 -VlaHierarchyBottomTile prev_front `
#         -VlaHierarchyStepAsRate 1.0 -SimOffscreen -SimMaxFps 30
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
    [double]$StartTimeout = 330,        # red lights lengthen the car's lap (2026-09-29)
    [double]$StartMaxRange = 30,        # car mode: acquire only within this range.
                                        # 45 m let every flight start at 43.9 m,
                                        # the car 20 px wide and driving away.
    [double]$CruiseAlt = 0,             # 0 = the default, 8 m (see below)
    [int]$SimWidth   = 960,
    [int]$SimHeight  = 540,
    [int]$SimMaxFps  = 0,               # t.MaxFPS for the simulator (0 = uncapped, as before)
    [switch]$SimOffscreen,             # -RenderOffscreen: no game window (the cameras still render)
    [switch]$NoTrail,                   # car mode: fly along the nose, as before 09-24
    [switch]$Identity,                  # --identity for a person mission too (see below)
    [switch]$NoIdentity,                # the pre-2026-09-29 selection (no identity
                                        # tiers, lenient lock, no re-acquire gate,
                                        # no junction planner, no landing site)
    # Frames go to C: through a junction: D: is nearly full, and a flight's
    # frames are 1-2 GB. Empty string = write under demo\out as before.
    [string]$OffloadRoot = "D:\vla_drone_offload\demo_out",   # outside OneDrive; C: filled up on 23 Oct
    [string]$PolicyFile = "",          # override the rule set, e.g.
                                        # policies\follow_car_citylife_nfz.yaml
    [switch]$DevYaml,                   # fly the policy YAML itself (unsigned, a
                                        # development run) even when its signed
                                        # bundle is in bundles\
    [switch]$NoPolicyHud,               # the old one-line HUD (A/B of the indicator's cost)
    [switch]$RecordVla,                 # record what AerialVLA would see (--record-vla),
                                        # to <OffloadRoot>\<Tag>\vla_record (see the header)
    [string]$RecordVlaFrontSource = "", # with -RecordVla: "detector" records FrontCamera as the front frame (AIOT-VLA)
    [switch]$RecordPerturb,             # with -RecordVla: DART noise on the executed command
    [switch]$PerturbSpeed,              # with -RecordPerturb: also scale the speed 0.5-1.0
    [switch]$PerturbReplaceYaw,         # with -RecordPerturb: the offset replaces the
                                        # expert's yaw rate while it lasts (stronger)
    [string]$PerturbYawRate = "",       # with -RecordPerturb: "LO,HI" rad/s (default 0.3,0.5)
    [string]$RecordPerturbVla = "",     # with -RecordVla: HOST:PORT of a running AerialVLA
                                        # server that flies windows of the recording (DAgger)
    [string]$PerturbVlaEvery = "",      # with -RecordPerturbVla: "LO,HI" s onset to onset (6,10)
    [string]$PerturbVlaDur = "",        # with -RecordPerturbVla: "LO,HI" s per window (2.5,4.0)
    [switch]$PerturbVlaAllowStandIn,    # with -RecordPerturbVla: a server that is not AerialVLA
    [string]$PerturbVlaBottomTile = "", # with -RecordPerturbVla: "prev_front" (default down)
    [double]$PerturbVlaPrevDtS = 0,     # with -PerturbVlaBottomTile prev_front: s earlier (1.0)
    [double]$PerturbVlaStepAsRate = 0,  # with -RecordPerturbVla: steps as rates over H s (off)
    [switch]$PerturbVlaSpeedWords,      # with -RecordPerturbVla: own speed as words (off)
    [string]$PerturbVlaRateYaw = "",    # with -PerturbVlaStepAsRate: "rate" flies dpsi/H as the yaw rate (aiotvla)
    [double]$DecideEveryS = 0,          # the ceiling flight: a decision every P s (off)
    [double]$DecideDelayS = 0,          # with -DecideEveryS: each carries the command from D s before
    [string]$VlaHierarchy = "",         # HOST:PORT of a running AerialVLA server that decides
                                        # the follow's intent (hierarchical control; see the header)
    [string]$VlaHierarchyBottomTile = "", # with -VlaHierarchy: "prev_front" (default down)
    [double]$VlaHierarchyPrevDtS = 0,   # with -VlaHierarchyBottomTile prev_front: s earlier (1.0)
    [double]$VlaHierarchyStepAsRate = 0, # with -VlaHierarchy: the search's steps as rates over H s (off)
    [switch]$VlaHierarchySpeedWords,    # with -VlaHierarchy: own speed as words (off)
    [string]$VlaHierarchyFrontSource = "", # with -VlaHierarchy: "detector" sends FrontCamera
                                        # (a foreign server that reads it; default VLAFront)
    [string]$VlaHierarchyRange = "",    # with -VlaHierarchy: "NEAR,FAR" stand-off metres (12,22)
    [switch]$VlaHierarchyAllowStandIn,  # with -VlaHierarchy: a server that is not AerialVLA
    [int]$Seed = 0,                  # follow_vlm --seed (0 = its default); the
                                        # perturbation schedule follows it
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
    $ueArgs = @("`"$Proj`"", $Map, '-game', '-windowed', "-ResX=$SimWidth", "-ResY=$SimHeight")
    if ($SimMaxFps -gt 0) { $ueArgs += "-ExecCmds=`"t.MaxFPS $SimMaxFps`"" }
    if ($SimOffscreen) { $ueArgs += '-RenderOffscreen' }
    Start-Process -FilePath $UE -ArgumentList $ueArgs | Out-Null
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
# Refused here, before a simulator starts (follow_vlm refuses the same):
# the perturbation exists only to be recorded as training data.
if ($RecordPerturb -and -not $RecordVla) { throw "-RecordPerturb needs -RecordVla" }
if ($RecordVlaFrontSource -and -not $RecordVla) { throw "-RecordVlaFrontSource needs -RecordVla" }
if ($PerturbSpeed -and -not $RecordPerturb) { throw "-PerturbSpeed needs -RecordPerturb" }
if (($PerturbReplaceYaw -or $PerturbYawRate) -and -not $RecordPerturb) {
    throw "-PerturbReplaceYaw and -PerturbYawRate need -RecordPerturb"
}
if ($RecordPerturbVla -and -not $RecordVla) { throw "-RecordPerturbVla needs -RecordVla" }
if ($RecordPerturbVla -and $RecordPerturb) {
    throw "-RecordPerturbVla and -RecordPerturb: one perturbation at a time"
}
if (($PerturbVlaEvery -or $PerturbVlaDur -or $PerturbVlaAllowStandIn) -and -not $RecordPerturbVla) {
    throw "-PerturbVlaEvery, -PerturbVlaDur and -PerturbVlaAllowStandIn need -RecordPerturbVla"
}
if (($PerturbVlaBottomTile -or $PerturbVlaPrevDtS -or $PerturbVlaStepAsRate) -and -not $RecordPerturbVla) {
    throw "-PerturbVlaBottomTile, -PerturbVlaPrevDtS and -PerturbVlaStepAsRate need -RecordPerturbVla"
}
if ($PerturbVlaSpeedWords -and -not $RecordPerturbVla) {
    throw "-PerturbVlaSpeedWords needs -RecordPerturbVla"
}
if ($PerturbVlaRateYaw -and -not $PerturbVlaStepAsRate) {
    throw "-PerturbVlaRateYaw needs -PerturbVlaStepAsRate"
}
# The ceiling flight (see the header), refused as follow_vlm refuses it. The
# bounds are follow_vlm's (demo/decide_rate.py): a 0 typed on purpose is not
# taken for "off", nor 800 (milliseconds) for seconds.
if ($PSBoundParameters.ContainsKey('DecideEveryS') -and -not ($DecideEveryS -gt 0 -and $DecideEveryS -le 10)) {
    throw "-DecideEveryS is the seconds between decisions: more than 0, at most 10"
}
if ($PSBoundParameters.ContainsKey('DecideDelayS') -and -not ($DecideDelayS -ge 0 -and $DecideDelayS -le 10)) {
    throw "-DecideDelayS is seconds: 0 to 10"
}
if ($PSBoundParameters.ContainsKey('DecideDelayS') -and -not $DecideEveryS) {
    throw "-DecideDelayS needs -DecideEveryS"
}
if ($DecideEveryS -and ($RecordPerturb -or $RecordPerturbVla)) {
    throw "-DecideEveryS and -RecordPerturb / -RecordPerturbVla: one replacement of the command at a time"
}
if ($DecideEveryS -and $RecordVla) {
    throw "-DecideEveryS and -RecordVla: a ceiling flight is a measurement, not training data"
}
if ($RecordPerturbVla) {
    # The VLA server must already listen: follow_vlm refuses a DAgger flight
    # whose server does not answer hello, but only after a simulator has been
    # started (minutes) for nothing.
    $i = $RecordPerturbVla.LastIndexOf(':')
    $vlaPort = 0
    if ($i -lt 1 -or -not [int]::TryParse($RecordPerturbVla.Substring($i + 1), [ref]$vlaPort)) {
        throw "-RecordPerturbVla $RecordPerturbVla is not HOST:PORT"
    }
    $vlaHost = $RecordPerturbVla.Substring(0, $i)
    if (-not (Test-NetConnection $vlaHost -Port $vlaPort -WarningAction SilentlyContinue).TcpTestSucceeded) {
        throw ("nothing listens on ${RecordPerturbVla}: start the VLA server first (vla-real env), " +
               "e.g. python demo\vla_server_v2.py serve --precision bf16 --attn sdpa --merge " +
               "--missing-down refuse --adapter D:\models\aerialvla-ft-follow\<run>\epoch<N>")
    }
    Say "VLA server listening on $RecordPerturbVla (follow_vlm checks its hello)"
}
# Hierarchical control (see the header), refused as follow_vlm refuses it:
# one replacement of the executed command at a time, and never training data.
if (($VlaHierarchyBottomTile -or $VlaHierarchyPrevDtS -or $VlaHierarchyStepAsRate -or
     $VlaHierarchySpeedWords -or $VlaHierarchyRange -or $VlaHierarchyAllowStandIn -or
     $VlaHierarchyFrontSource) -and -not $VlaHierarchy) {
    throw ("-VlaHierarchyBottomTile, -VlaHierarchyPrevDtS, -VlaHierarchyStepAsRate, " +
           "-VlaHierarchySpeedWords, -VlaHierarchyRange, -VlaHierarchyAllowStandIn and " +
           "-VlaHierarchyFrontSource need -VlaHierarchy")
}
if ($VlaHierarchy -and ($RecordPerturb -or $RecordPerturbVla -or $DecideEveryS)) {
    throw "-VlaHierarchy and -RecordPerturb / -RecordPerturbVla / -DecideEveryS: one replacement of the executed command at a time"
}
if ($VlaHierarchy -and $RecordVla) {
    throw "-VlaHierarchy and -RecordVla: a hierarchical flight flies the VLA's intent, not training data"
}
if ($VlaHierarchy) {
    # The VLA server must already listen: follow_vlm refuses a server that
    # does not answer hello, but only after a simulator has been started.
    $i = $VlaHierarchy.LastIndexOf(':')
    $hierPort = 0
    if ($i -lt 1 -or -not [int]::TryParse($VlaHierarchy.Substring($i + 1), [ref]$hierPort)) {
        throw "-VlaHierarchy $VlaHierarchy is not HOST:PORT"
    }
    $hierHost = $VlaHierarchy.Substring(0, $i)
    if (-not (Test-NetConnection $hierHost -Port $hierPort -WarningAction SilentlyContinue).TcpTestSucceeded) {
        throw ("nothing listens on ${VlaHierarchy}: start the VLA server first (vla-real env), " +
               "e.g. python demo\vla_server_v2.py serve --precision bf16 --attn sdpa --merge " +
               "--missing-down refuse --adapter D:\models\aerialvla-ft-follow\<run>\epoch<N>")
    }
    Say "VLA server listening on $VlaHierarchy (follow_vlm checks its hello)"
}

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
# --identity: every candidate judged physically at its own frame's pose and
# depth (demo/identity.py), a strict lock, and a lost subject re-acquired only
# through a gate - citylife_redcar_trail ended TARGET LOCKED on a red
# pedestrian signal 110 m from the car without them. --land-site: descend on a
# pavement cell, not wherever the mission ended (a hedge, a parked car).
$Policy = if ($LevelCar) { "policies\follow_car_citylife.yaml" } else { "policies\follow_pedestrian.yaml" }
# -PolicyFile swaps the rule set only (the CityLife no-fly zone, 2026-10-03);
# everything else about the mission stays as above.
if ($PolicyFile) {
    $pf = if ([IO.Path]::IsPathRooted($PolicyFile)) { $PolicyFile } else { Join-Path $Root $PolicyFile }
    if (-not (Test-Path $pf)) { throw "no policy at $PolicyFile" }
    $Policy = $PolicyFile
}
# THE SIGNED BUNDLE, else the YAML under --dev-yaml (ARCH-29, 2026-10-21):
# follow_vlm flies the signed bundle of --policy and refuses the YAML without
# --dev-yaml; tools\Get-FlightPolicyDev.ps1 checks the bundle first and says
# which flies (-DevYaml forces the YAML).
. "$Root/tools/Get-FlightPolicyDev.ps1"
$PolicyDev = @(Get-FlightPolicyDev $Root $Py $Policy -DevYaml:$DevYaml)
# Cruise altitude: 8 m for both missions. 12 m was flown (citylife_redcar_high)
# and overshot the car policy's 14 m ceiling for 3.1 s (peak 14.32 m), each climb
# during a phantom-wall repair (docs/FINDING-crowd-pedestrians-and-traffic.md,
# third part), and the detector's hit rate fell from 0.71 to 0.32.
if ($CruiseAlt -le 0) { $CruiseAlt = 8 }
$a = @("demo\follow_vlm.py",
       "--object", $Object,
       "--tag", $Tag,
       "--policy", $Policy) + $PolicyDev + @(
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
if ($NoPolicyHud) { $a += @("--no-policy-hud") }
# --identity was tuned on the red-car flights only (tools/replay_identity.py).
# For people it supersedes the presence gate's 0.2-1.5 m width test, and the
# first pedestrian flight under it (citylife_ped_id, 2026-09-30) followed
# person boxes ranged on the facades behind them, 40-130 m out: within 30 m
# 0.972, but its estimate within 6 m of anybody on 46 % of ticks. Until the
# person rules are tuned the same way, a person mission flies the
# presence-gated selection; -Identity forces identity for it.
if (-not $NoIdentity) {
    if ($LevelCar -or $Identity) { $a += @("--identity") }
    $a += @("--land-site")
}
if ($LevelCar) {
    # After the coast and the search sweep: pursue the car's street to the next
    # junction and watch it (demo/search.py), not rotate in place.
    if (-not $NoIdentity) { $a += @("--search-planner") }
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
if ($OffloadRoot) {
    $viewLink = Join-Path $Root "demo\out\$Tag\view"
    $viewDest = Join-Path $OffloadRoot "$Tag\view"
    # Get-Item on the link itself, not Test-Path. Test-Path is True for a
    # junction whose target was deleted (the offload folder cleared to make
    # room), so nothing was recreated, follow_vlm's mkdir of view\tps failed,
    # and its handler printed "no Chase camera": a whole flight with no frames.
    $view = Get-Item -Force -LiteralPath $viewLink -ErrorAction SilentlyContinue
    if (-not $view) {
        New-Item -ItemType Directory -Force (Split-Path $viewLink) | Out-Null
        New-Item -ItemType Directory -Force $viewDest | Out-Null
        New-Item -ItemType Junction -Path $viewLink -Target $viewDest | Out-Null
        Say "frames -> $viewDest (junction)"
    } elseif ($view.LinkType) {
        # Windows PowerShell 5.1 returns Target as string[], 7 as a string.
        $target = @($view.Target)[0]
        if (-not [IO.Path]::IsPathRooted($target)) {
            $target = Join-Path (Split-Path $viewLink) $target
        }
        if (-not (Test-Path -LiteralPath $target)) {
            New-Item -ItemType Directory -Force $target | Out-Null
            Say "frames -> $target (junction; its target was missing, recreated)"
        } else {
            Say "frames -> $target (junction)"
        }
        $under = [IO.Path]::GetFullPath($OffloadRoot).TrimEnd('\') + '\'
        if (-not [IO.Path]::GetFullPath($target).StartsWith($under, [StringComparison]::OrdinalIgnoreCase)) {
            Write-Warning "$viewLink points at $target, outside -OffloadRoot $OffloadRoot. The frames go there."
        }
    } else {
        # A real folder: a flight from before the offload, or one run with
        # -OffloadRoot "". Only files or links count as content - follow_vlm
        # makes view\tps and view\fpv before the first frame, so a flight that
        # failed early leaves those two empty folders and nothing else.
        $content = @(Get-ChildItem -Force -Recurse -LiteralPath $viewLink |
                     Where-Object { -not $_.PSIsContainer -or $_.LinkType })
        if ($content.Count -eq 0) {
            Remove-Item -Recurse -Force -LiteralPath $viewLink
            New-Item -ItemType Directory -Force $viewDest | Out-Null
            New-Item -ItemType Junction -Path $viewLink -Target $viewDest | Out-Null
            Say "frames -> $viewDest (junction; replaced an empty view folder)"
        } else {
            # Never delete frames here, and never send them to D: silently: the
            # 13 CityLife flights offloaded so far wrote 1.3-1.7 GB each
            # (measured 2026-09-30) and D: had 2.3 GB free that day. The FLIGHT
            # still deletes them: FrameRecorder clears view\tps and view\fpv of
            # *.jpg when it starts (demo/recorder.py, _clear), so "nothing was
            # deleted" would be true only until then, and the warning says so.
            $free = (Get-PSDrive (Split-Path -Qualifier $viewLink).TrimEnd(':')).Free / 1GB
            Write-Warning ("$viewLink is a real folder that already holds frames, so this " +
                           "flight writes to $(Split-Path -Qualifier $viewLink) ($('{0:N1}' -f $free) GB free; " +
                           "a flight is 1-2 GB), not to $viewDest, and its recorder deletes the old " +
                           "tps/fpv frames there when it starts. To keep them, stop this run (Ctrl+C) " +
                           "and move the folder's contents to $viewDest, or fly under a new -Tag.")
        }
    }
}
# -RecordVla: demo\out\<Tag>\vla_record, a junction to <OffloadRoot>\<Tag>\
# vla_record (D: is nearly full; a front+down pair is about 50 KB, 10 a second,
# ~150 MB for a 300 s flight). The recorder rewrites its three files and keeps
# frames\ (content-addressed), so a reused tag needs nothing cleared here.
if ($RecordVla) {
    $recLink = Join-Path $Root "demo\out\$Tag\vla_record"
    if ($OffloadRoot) {
        $recDest = Join-Path $OffloadRoot "$Tag\vla_record"
        $rec = Get-Item -Force -LiteralPath $recLink -ErrorAction SilentlyContinue
        if (-not $rec) {
            New-Item -ItemType Directory -Force (Split-Path $recLink) | Out-Null
            New-Item -ItemType Directory -Force $recDest | Out-Null
            New-Item -ItemType Junction -Path $recLink -Target $recDest | Out-Null
            Say "VLA recording -> $recDest (junction)"
        } elseif ($rec.LinkType) {
            $recTarget = @($rec.Target)[0]
            if (-not [IO.Path]::IsPathRooted($recTarget)) {
                $recTarget = Join-Path (Split-Path $recLink) $recTarget
            }
            if (-not (Test-Path -LiteralPath $recTarget)) {
                New-Item -ItemType Directory -Force $recTarget | Out-Null
            }
            Say "VLA recording -> $recTarget (junction)"
        } else {
            Write-Warning ("$recLink is a real folder, so this recording writes to " +
                           "$(Split-Path -Qualifier $recLink), not to $recDest. To keep D: free, " +
                           "stop this run (Ctrl+C) and move the folder to $recDest, or fly under a new -Tag.")
        }
    }
    $a += @("--record-vla", "demo\out\$Tag\vla_record")
    if ($RecordVlaFrontSource) {
        $a += @("--record-vla-front-source", $RecordVlaFrontSource)
    }
    if ($RecordPerturb) {
        $a += @("--record-perturb")
        if ($PerturbSpeed) {
            $a += @("--perturb-speed")
        }
        if ($PerturbReplaceYaw) {
            $a += @("--perturb-replace-yaw")
        }
        if ($PerturbYawRate) {
            $a += @("--perturb-yaw-rate", $PerturbYawRate)
        }
    }
    if ($RecordPerturbVla) {
        $a += @("--record-perturb-vla", $RecordPerturbVla)
        # An unquoted -PerturbVlaEvery 6,10 binds to a [string] as "6 10".
        if ($PerturbVlaEvery) {
            $a += @("--perturb-vla-every", ($PerturbVlaEvery.Trim() -replace '\s+', ','))
        }
        if ($PerturbVlaDur) {
            $a += @("--perturb-vla-dur", ($PerturbVlaDur.Trim() -replace '\s+', ','))
        }
        if ($PerturbVlaAllowStandIn) {
            $a += @("--perturb-vla-allow-stand-in")
        }
        if ($PerturbVlaBottomTile) {
            $a += @("--perturb-vla-bottom-tile", $PerturbVlaBottomTile)
        }
        if ($PerturbVlaPrevDtS) {
            $a += @("--perturb-vla-prev-dt-s", "$PerturbVlaPrevDtS")
        }
        if ($PerturbVlaStepAsRate) {
            $a += @("--perturb-vla-step-as-rate", "$PerturbVlaStepAsRate")
        }
        if ($PerturbVlaRateYaw) {
            $a += @("--perturb-vla-rate-yaw", "$PerturbVlaRateYaw")
        }
        if ($PerturbVlaSpeedWords) {
            $a += @("--perturb-vla-speed-words")
        }
    }
}
# The ceiling flight (see the header): the executed command held to a VLA's
# decision rate. Refused above with a recording or a perturbation.
if ($DecideEveryS) {
    $a += @("--decide-every-s", "$DecideEveryS")
    if ($DecideDelayS) {
        $a += @("--decide-delay-s", "$DecideDelayS")
    }
}
# Hierarchical control (see the header): the VLA decides the intent, this
# controller executes it. Refused above with a recording or another mode.
if ($VlaHierarchy) {
    $a += @("--vla-hierarchy", $VlaHierarchy)
    if ($VlaHierarchyBottomTile) {
        $a += @("--vla-hierarchy-bottom-tile", $VlaHierarchyBottomTile)
    }
    if ($VlaHierarchyPrevDtS) {
        $a += @("--vla-hierarchy-prev-dt-s", "$VlaHierarchyPrevDtS")
    }
    if ($VlaHierarchyStepAsRate) {
        $a += @("--vla-hierarchy-step-as-rate", "$VlaHierarchyStepAsRate")
    }
    if ($VlaHierarchySpeedWords) {
        $a += @("--vla-hierarchy-speed-words")
    }
    # An unquoted -VlaHierarchyRange 12,22 binds to a [string] as "12 22".
    if ($VlaHierarchyRange) {
        $a += @("--vla-hierarchy-range", ($VlaHierarchyRange.Trim() -replace '\s+', ','))
    }
    if ($VlaHierarchyAllowStandIn) {
        $a += @("--vla-hierarchy-allow-stand-in")
    }
    if ($VlaHierarchyFrontSource) {
        $a += @("--vla-hierarchy-front-source", $VlaHierarchyFrontSource)
    }
}
if ($Seed -ne 0) { $a += @("--seed", "$Seed") }
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
Write-Host "    demo\out\$Tag\metrics.json     -> det_hz, frac_on_target (+ _chance), instance_score, start_gate, stage_ms_median, collisions, off_map_ticks, trail, decide_rate (a ceiling flight), vla_hierarchy (a hierarchical flight)"
Write-Host "    demo\out\$Tag\kpi.json         -> p0_violation_escape_rate must be 0.0"
Write-Host "  then build the video:"
Write-Host "    python tools\make_demo_video.py --tag $Tag --height 720"
