<#
.SYNOPSIS
    Drone follows a named moving object using vision and language, through the guardrail.

.DESCRIPTION
    Starts the simulator, spawns a car that drives down a real street, and flies a
    mission whose only steering input is "find the thing I named, in the camera".
    No target coordinates reach the controller.

    Records two views for the demo - the drone's own camera with the detection
    box and telemetry drawn on it, and a third-person chase view - and stitches
    them into one side-by-side video.

    Measured 2026-08-11, all three demos:
      DEMO 1  tracking          100% of the flight within 30 m, mean 13.4 m
      DEMO 2  + 3 distractors   100% within 30 m, mean 17.7 m, hit rate 97.9%
      DEMO 3  no-fly zone       34.9% within 30 m by design, 0 Shield overrides
    NFZ time and altitude escape are 0.0 s on every flight ever recorded.

    Takes about 18 minutes: three flights, each preceded by a simulator restart,
    then three videos. Add -Controls for two more flights.

.PARAMETER Object
    What to follow, in words. Be specific: "a car" alone makes it chase city
    clutter. Default "a white car" - the target is SKM_SportsCar painted with
    M_Orange, which renders WHITE on that mesh (measured; a material is a shader,
    not a colour). Avoid "a blue car": the asphalt in this map sits in the blue
    hue band above the saturation floor.

.PARAMETER Controls
    Also fly the two control conditions (wrong colour word, and no car present).

.EXAMPLE
    .\run_follow_vlm.ps1
    .\run_follow_vlm.ps1 -Controls
    .\run_follow_vlm.ps1 -SkipSim
#>
[CmdletBinding()]
param(
    [string]$Object = "a white car",
    [switch]$SkipSim,
    [switch]$Controls,
    [switch]$NoVideo
)
# Flight durations are per-demo and set at the call sites below: the tracking run
# is sized to the car's 48 s route plus its two stops, the fenced run to the
# point where the car has escaped. A single -Seconds knob would make one of them
# wrong.

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$UE   = "C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
$Map  = "/Game/JapaneseCity/Maps/Demo_day"

function Say($m) { Write-Host "==> $m" -ForegroundColor Cyan }
foreach ($p in @($Py, $UE, $Proj)) { if (-not (Test-Path $p)) { throw "not found: $p" } }
Set-Location $Root

function Test-SimUp { (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded }

function Stop-OurSim {
    # Matched on the .uproject, not the process name: killing every UnrealEditor
    # would take unrelated projects, and unsaved work, with it.
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

function Start-Sim {
    Stop-OurSim
    Start-Sleep -Seconds 4
    # PowerShell, not Git Bash: bash rewrites the /Game/... map argument into a
    # Windows path, the map is not found, and the engine crashes on the fallback.
    Start-Process -FilePath $UE -ArgumentList "`"$Proj`"", $Map, '-game', '-windowed', '-ResX=1280', '-ResY=720' | Out-Null
    Say "simulator starting ..."
    for ($i = 0; $i -lt 90; $i++) {
        Start-Sleep -Seconds 5
        if (Test-SimUp) { Start-Sleep -Seconds 8; Say "simulator ready"; return }
    }
    throw "simulator did not open port 8989"
}

function Fly($tag, $obj, $policy, $secs, $stopS, $traffic, [switch]$NoCar, [switch]$Record) {
    # The simulator is restarted before every flight. Measured repeatedly: it
    # refuses the next connection after a flight disconnects, so reusing it
    # silently costs a run.
    if (-not $SkipSim) { Start-Sim }
    $a = @("demo\follow_vlm.py", "--object", $obj, "--tag", $tag,
           "--max-s", "$secs", "--det-thresh", "0.008",
           "--car-speed", "2.0", "--car-stop-s", "$stopS",
           "--policy", $policy, "--straight")
    if ($traffic -gt 0) { $a += @("--traffic", "$traffic") }
    if ($NoCar)  { $a += "--no-car" }
    if ($Record) { $a += "--save-view" }
    & $Py @a
}

# Fly() starts the simulator itself before each flight, so do not start one here
# as well - that was an extra two-minute restart before the first run.
if ($SkipSim -and -not (Test-SimUp)) { throw "-SkipSim given but nothing on 8989" }

Say "DEMO 1: follow `"$Object`" - no fence, pure tracking"
Say "        measured 100% of the flight within 30 m, on two separate flights"
Fly "demo_follow" $Object "policies\follow_car.yaml" 70 6 0 -Record

# --want-width is COUPLED to --cruise-alt. It is an angular stand-off, so the
# same value is a much larger ground distance from higher up. 0.10 suits the 9 m
# cruise these demos fly. follow_car_gap.yaml forces 13 m and needs about 0.20,
# or the aircraft holds a 30 m stand-off and the "within 30 m" metric reads that
# as failure - it scored 0.26 for that reason alone, and 0.759 once corrected.
# See docs/FINDING-gapfence-was-never-the-fence.md.
Say "DEMO 2: three MORE cars, same mesh, only the colour differs"
Say "        the noun cannot separate them - only the colour test can"
Fly "demo_traffic" $Object "policies\follow_car.yaml" 70 6 3 -Record

Say "DEMO 3: same mission with a no-fly zone across the route"
Say "        the car drives through it, the drone must not"
Fly "demo_nfz" $Object "policies\follow_car_nfz.yaml" 70 6 0 -Record

if ($Controls) {
    Say "CONTROL 1: same car, WRONG colour word - should NOT follow"
    # "a red car", not "a blue car": the asphalt reads blue above the saturation
    # floor, so a blue query can score on the road itself and the control looks
    # weaker than the colour gate really is. Red has no such background overlap.
    Fly "demo_wrongcolour" "a red car" "policies\follow_car.yaml" 70 6 0
    Say "CONTROL 2: right words, NO car in the scene"
    Fly "demo_nocar" $Object "policies\follow_car.yaml" 70 6 0 -NoCar
}

if (-not $NoVideo) {
    Say "building the side-by-side demo videos"
    & $Py "tools\make_demo_video.py" "--tag" "demo_follow" "--fps" "10"
    & $Py "tools\make_demo_video.py" "--tag" "demo_traffic" "--fps" "10"
    & $Py "tools\make_demo_video.py" "--tag" "demo_nfz" "--fps" "10"
}

Say "summary"
& $Py -c @"
import json, pathlib
tags = ['demo_follow', 'demo_traffic', 'demo_nfz',
        'demo_wrongcolour', 'demo_nocar']
cols = ['tag','object','det_hit_rate','sep_mean_m','frac_within_30m',
        'interventions','nfz_s','alt_violation_s','frac_absent']
print('  ' + ' | '.join(f'{c:>16}' for c in cols))
for t in tags:
    f = pathlib.Path('demo/out') / t / 'metrics.json'
    if not f.exists():
        continue
    m = json.loads(f.read_text(encoding='utf-8'))
    print('  ' + ' | '.join(f'{str(m.get(c)):>16}' for c in cols))
print()
print('  sep_mean and frac_within_30m are the following metrics.')
print('  sep_min alone is not: a drone that never moves still records ~12 m,')
print('  because the car drives past it.')
"@

Stop-OurSim
Say "done."
Write-Host "    videos  : demo\out\demo_follow\demo_follow_demo.mp4   (tracking, car stops twice)"
Write-Host "              demo\out\demo_traffic\demo_traffic_demo.mp4 (picks one of four identical cars)"
Write-Host "              demo\out\demo_nfz\demo_nfz_demo.mp4         (guardrail holds it outside)"
Write-Host "    metrics : demo\out\<tag>\metrics.json"
