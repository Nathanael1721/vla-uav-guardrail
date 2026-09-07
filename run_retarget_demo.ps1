# The 2026-09-02 review's question, flown.
#
# Prof. Lai's point was fair: BoT-SORT and ByteTrack associate detections across
# frames better than we do, so "we track well" is not a claim worth making. But a
# tracker must be given a BOX and cannot be given a NOUN. It returns an id, never
# a class.
#
# The stand-off rule is written PER CLASS - policies/follow_pedestrian.yaml holds
# 10 m from a pedestrian and 5 m from anything else - so the policy has to know
# the subject IS a person before it can pick the rule. That is not a
# tracking-quality contest we might lose; it is a question a tracker cannot be
# asked at all.
#
# So: one flight, one policy, one aircraft. Partway through, the operator changes
# a PHRASE. The enforced ring moves and the drone backs off. Nothing else changes.
#
#     .\run_retarget_demo.ps1
#     .\run_retarget_demo.ps1 -At 25 -To "a person"
#
# WARNING: like every demo script here, this restarts the simulator, which kills
# any process holding Blocks.uproject - INCLUDING an editor you have open. Close
# the editor first. The match is on the .uproject, not the process name, so
# unrelated Unreal projects are left alone.

param(
    [string]$Object = "a yellow car",   # what it follows first  -> the 5 m rule
    [string]$To     = "a person",       # what it is retargeted to -> the 10 m rule
    [double]$At     = 30,               # seconds into the flight
    [int]$Seconds   = 70,
    [string]$Tag    = "retarget_demo",
    [int]$SimWidth  = 960,
    [int]$SimHeight = 540,
    [switch]$SkipSim
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
# Engine read from the .uproject rather than hard-coded - see tools/ue_engine.py.
. "$Root/tools/Resolve-UnrealEngine.ps1"
$UE   = Get-UnrealEditor -UProject $Proj
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
    Start-Process -FilePath $UE -ArgumentList "`"$Proj`"", $Map, '-game', '-windowed', "-ResX=$SimWidth", "-ResY=$SimHeight" | Out-Null
    Say "simulator starting ..."
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

Say "following `"$Object`" (the 5 m rule), retargeting to `"$To`" at t+$($At)s (the 10 m rule)"

# --det-thresh 0.008 matches the city_locked flight that produced the last demo
# video. It matters more than usual here: OWL-ViT scores our pedestrian meshes at
# about 0.06 against the taxi's 0.18, so the person half of this flight is the
# part most likely to struggle. If it does, that is a measured result about the
# detector - not a reason to quietly raise the threshold until it looks good.
$a = @("demo\follow_vlm.py",
       "--object", $Object,
       "--retarget", "$($At):$To",
       "--tag", $Tag,
       "--policy", "policies\follow_pedestrian.yaml",
       "--max-s", "$Seconds",
       "--det-thresh", "0.008",
       "--want-width", "0.16",
       "--cruise-alt", "8",
       "--lock-target",
       "--pedestrians", "12",
       "--parked", "8",
       "--save-view")
& $Py @a

Say "done. Read the result from the artefact, not the screen:"
Write-Host "    demo\out\$Tag\metrics.json     -> `"retargets`", sep_min_m, target_lock"
Write-Host "    demo\out\$Tag\kpi.json         -> p0_violation_escape_rate must be 0.0"
