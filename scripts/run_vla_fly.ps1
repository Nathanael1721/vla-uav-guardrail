# AerialVLA flies the drone in CityLife_Day, behind the Safety Shield.
#
# Starts the simulator on CityLife_Day, then demo\vla_fly.py, which starts the
# VLA server (demo\vla_server_v2.py, vla-real env) with an EXPLICIT model
# configuration, waits for its hello, flies the mission and stops the server;
# then this script stops the simulator. Pattern of run_citylife_follow.ps1.
#
#     .\scripts\run_vla_fly.ps1                                   # M1 GOTO, 180 s, bf16
#     .\scripts\run_vla_fly.ps1 -Mission goto -Seconds 240 -Tag vla_goto_02
#     .\scripts\run_vla_fly.ps1 -Mission goto -Goal 43,100        # beside the city NFZ
#     .\scripts\run_vla_fly.ps1 -Mission approach -Tag vla_appr_01   # the parked orange car
#     .\scripts\run_vla_fly.ps1 -Mission follow -Seconds 240 -Tag vla_follow_01 `
#         -Cadence pipelined -CruiseMps 2.5 -SpeedCapMps 3   # chase the level's red car (Car_10)
#     .\scripts\run_vla_fly.ps1 -Mission goto -Precision nf4      # the low-VRAM option
#     .\scripts\run_vla_fly.ps1 -Mission goto -Shield off -Tag vla_goto_off   # control arm
#     .\scripts\run_vla_fly.ps1 -VlaServer 127.0.0.1:8790         # attach to a running server
#     .\scripts\run_vla_fly.ps1 -FakeOutput "96 48 49</s>" -Tag vla_dry   # no model: plumbing only
#     .\scripts\run_vla_fly.ps1 -Mission goto -Cadence pipelined  # the pipelined arm (labelled)
#     .\scripts\run_vla_fly.ps1 -Mission goto -Goal 43,100 -PrintArgs   # show the flight's arguments, start nothing
#
# -Goal / -ParkAt take X,Y in metres, written either way: 43,100 (PowerShell
# binds that as two numbers) or "43,100" (one string). Both reach the flight
# as "43,100"; anything that is not exactly two finite numbers is refused
# here, and the flight's own parser checks the whole argument list
# (demo\vla_fly.py --check-args) - both BEFORE the simulator starts. Until
# 2026-10-23 -Goal 43,100 bound to a [string] as "43 100" and the flight
# refused it only after the simulator was up.
#
# -Cadence stop_and_go (default) is upstream's: the next image only after the
# step is flown and held 0.5 s. pipelined sends the next image while a step
# is flown, which applies each answer's turn and climb at the decision rate
# (docs\DESIGN-vla-pilot.md section 13 has the measured difference).
#
# -Precision bf16 (default) is the FIDELITY option: --vla-precision bf16
# --vla-attn sdpa --no-vla-merge (agrees with the upstream-precision reference
# on 96 % of the RTX 4090 probe, about 17 GB). -Precision nf4 is the measured
# LOW-VRAM option, nf4/sdpa/merge (25-33 % agreement, about 7.9 GB). Whether
# bf16 fits beside CityLife on the 24 GB card is not yet measured; if the
# server dies loading, the flight refuses to take off and this says so.
#
# WARNING: like every demo script here, this restarts the simulator, which
# kills any process holding Blocks.uproject - INCLUDING an editor you have
# open. Close the editor first (the script refuses while one is open).

param(
    [ValidateSet("goto", "approach", "follow")][string]$Mission = "goto",
    [int]$Seconds      = 180,
    [string]$Tag       = "",
    [ValidateSet("bf16", "nf4")][string]$Precision = "bf16",
    [string[]]$Goal    = @(),           # M1: X,Y (default 43,40: vla_mission.py)
    [string]$Object    = "",            # M2: OWL-ViT's query (default "an orange car")
    [string[]]$ParkAt  = @(),           # M2: where the scenario car parks (default 48,12)
    [ValidateSet("", "on", "off")][string]$Shield = "",
    [ValidateSet("stop_and_go", "pipelined")][string]$Cadence = "stop_and_go",
    [string]$LevelCar  = "",            # M3 follow: the level's car, truth for scoring (default Car_10)
    [double]$CruiseMps = 0,             # step tracker cruise speed (0 = vla_fly's default, 1 m/s)
    [double]$SpeedCapMps = 0,           # step tracker speed cap (0 = vla_fly's default, 2 m/s)
    [string[]]$ExtraArgs = @(),         # passed to demo\vla_fly.py as given, last (checked by its --check-args)
    [string]$VlaServer = "",            # host:port: attach instead of starting one
    [string[]]$FakeOutput = @(),        # the scripted stand-in (no GPU; never a VLA run)
    [string]$PolicyFile = "",           # default policies\vla_goto_citylife.yaml
    [int]$Port         = 8790,
    [int]$SimWidth     = 960,
    [int]$SimHeight    = 540,
    # Frames go to C: through a junction (run_citylife_follow.ps1 says why);
    # empty string = write under demo\out.
    [string]$OffloadRoot = "D:\vla_drone_offload\demo_out",   # outside OneDrive; C: filled up on 23 Oct
    [switch]$KeepSim,
    [switch]$SkipSim,
    # Print the flight's arguments, one per line, and exit: nothing is
    # started (tests\test_vla_fly.py binds the real parameters this way).
    [switch]$PrintArgs
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
$Map  = "/Game/CityLife/Maps/CityLife_Day"
if (-not $Tag) { $Tag = "vla_$Mission" }
$CityMap = Join-Path $Root "demo\out\citymap_citylife\occ_day.npz"

function Say($m)  { Write-Host "==> $m" -ForegroundColor Cyan }
function Warn($m) { Write-Host "  ! $m" -ForegroundColor Yellow }

# ---- the policy: its signed bundle, or the YAML under --dev-yaml ----------
# run_pas_ardupilot.ps1's rule: the bundle when it is on disk and still the
# YAML's policy; otherwise the YAML, flown as a development run (recorded so,
# never KPI-grade). python tools\build_flight_bundles.py vla_goto_citylife
# builds the bundle where the lab key is.
function Get-PolicyArgs([string]$BundlesDir = (Join-Path $Root "bundles")) {
    $yaml = if ($PolicyFile) { $PolicyFile } else { Join-Path $Root "policies\vla_goto_citylife.yaml" }
    $code = "import json, sys; sys.path.insert(0, sys.argv[1]); from guardrail.manifest import flight_bundle_status; print(json.dumps(flight_bundle_status(sys.argv[2], bundles_dir=sys.argv[3])))"
    $out = (& $Py -c $code $Root $yaml $BundlesDir | Select-Object -Last 1)
    if ($LASTEXITCODE -ne 0 -or -not $out) { throw "could not check the signed bundle of $yaml" }
    $st = "$out" | ConvertFrom-Json
    if ($st.present -and $st.matches -ne $false) {
        Say "policy: the signed bundle $($st.bundle)"
        return @("--bundle", $st.bundle)
    }
    if ($st.present) {
        Warn "the signed bundle $($st.bundle) is STALE ($($st.bundle_hash) vs $($st.yaml_hash))"
    } else {
        Warn "no signed bundle of $yaml at $($st.bundle) (python tools\build_flight_bundles.py vla_goto_citylife)"
    }
    Warn "flying the YAML with --dev-yaml - a development run, recorded as such"
    return @("--policy", $yaml, "--dev-yaml")
}

# ---- X,Y as the flight reads it -------------------------------------------
# -Goal 43,100 binds as two values, -Goal "43,100" as one: join them with
# ",", then require exactly two finite numbers (invariant culture, no
# thousands separators - "43,100" must never become 43100) and write them
# back as "X,Y". Throws, before anything starts, otherwise.
function ConvertTo-XYArg([string[]]$Value, [string]$Name) {
    $joined = (@($Value) | ForEach-Object { "$_".Trim() }) -join ","
    $parts = @($joined -split "," | ForEach-Object { $_.Trim() })
    $inv = [Globalization.CultureInfo]::InvariantCulture
    $style = [Globalization.NumberStyles]::Float
    $nums = @()
    foreach ($p in $parts) {
        $d = 0.0
        if (-not [double]::TryParse($p, $style, $inv, [ref]$d) -or [double]::IsNaN($d) -or
            [double]::IsInfinity($d)) { $nums = $null; break }
        $nums += $d
    }
    if ($null -eq $nums -or $nums.Count -ne 2) {
        throw "$Name '$joined' is not X,Y: two finite numbers in metres (NED x North, y East), e.g. $Name 43,100"
    }
    return ($nums | ForEach-Object { $_.ToString("R", $inv) }) -join ","
}

# ---- the flight's arguments (tests\test_vla_fly.py runs this function) ----
# Reads the parameters above plus $Root, $CityMap and $PolicyArgs. The model
# configuration is ALWAYS explicit: the server has no default, because the
# precision changes what AerialVLA says (demo\vla_server_v2.py).
function Get-VlaFlyArgs {
    $a = @((Join-Path $Root "demo\vla_fly.py"),
           "--mission", $Mission,
           "--tag", $Tag,
           "--max-s", "$Seconds",
           "--cadence", $Cadence,
           "--citymap", $CityMap)
    if (@($Goal).Count -gt 0)   { $a += @("--goal", (ConvertTo-XYArg $Goal "-Goal")) }
    if ($Object) { $a += @("--object", $Object) }
    if (@($ParkAt).Count -gt 0) { $a += @("--park-at", (ConvertTo-XYArg $ParkAt "-ParkAt")) }
    if ($Shield) { $a += @("--shield", $Shield) }
    if ($LevelCar) { $a += @("--level-car", $LevelCar) }
    if ($CruiseMps -gt 0) { $a += @("--cruise-mps", "$CruiseMps") }
    if ($SpeedCapMps -gt 0) { $a += @("--speed-cap-mps", "$SpeedCapMps") }
    if ($VlaServer) {
        $a += @("--vla-server", $VlaServer)
    } elseif ($FakeOutput -and @($FakeOutput).Count -gt 0) {
        $a += @("--vla-port", "$Port", "--vla-fake-output") + @($FakeOutput)
    } elseif ($Precision -eq "nf4") {
        $a += @("--vla-port", "$Port", "--vla-low-vram")
    } else {
        $a += @("--vla-port", "$Port", "--vla-precision", "bf16", "--vla-attn", "sdpa", "--no-vla-merge")
    }
    if ($PolicyArgs) { $a += $PolicyArgs }
    if (@($ExtraArgs).Count -gt 0) { $a += @($ExtraArgs) }
    return $a
}
# ---- end of Get-VlaFlyArgs

if ($PrintArgs) {
    # The real parameter binding, nothing started (no simulator, no server,
    # no Python): the arguments as they would reach demo\vla_fly.py, minus
    # the policy choice (Get-PolicyArgs runs Python).
    $PolicyArgs = @()
    Get-VlaFlyArgs | ForEach-Object { Write-Output "$_" }
    exit 0
}

. "$Root/tools/Resolve-UnrealEngine.ps1"
$UE = Get-UnrealEditor -UProject $Proj
foreach ($p in @($Py, $UE, $Proj)) { if (-not (Test-Path $p)) { throw "not found: $p" } }
Set-Location $Root

function Test-SimUp { (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded }

function Stop-OurSim {
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

function Stop-OurGameSim {
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and $_.CommandLine -like '*-game*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}

# vla_fly.py stops the server it started (op shutdown, then terminate). If the
# flight process itself was killed, its server would keep 15 GB on the GPU:
# kill only a vla_server_v2.py whose --log names THIS tag's folder.
function Stop-OurVlaServer {
    Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
        Where-Object { $_.CommandLine -like '*vla_server_v2.py*' -and
                       $_.CommandLine -like "*demo\out\$Tag\vla_server.jsonl*" } |
        ForEach-Object {
            Warn "a VLA server of $Tag outlived its flight (PID $($_.ProcessId)); stopping it"
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
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
    throw "an Unreal EDITOR is open on this project (PID $($editors[0].ProcessId)); save and close it, or pass -SkipSim"
}
if (-not (Test-Path $CityMap)) {
    throw "no CityLife obstacle map at $CityMap (run_citylife_follow.ps1 says how to build it)"
}
if ($SkipSim -and -not (Test-SimUp)) { throw "-SkipSim given but nothing on 8989" }

$PolicyArgs = Get-PolicyArgs
$a = Get-VlaFlyArgs
# The flight's own parser over the whole list, BEFORE the simulator starts:
# a refusal here costs nothing (no simulator, no server, no GPU).
$eap = $ErrorActionPreference
$ErrorActionPreference = "Continue"     # argparse writes its refusal to stderr
try { $check = @(& $Py @a --check-args 2>&1 | ForEach-Object { "$_" }) }
finally { $ErrorActionPreference = $eap }
if ($LASTEXITCODE -ne 0) {
    $check | ForEach-Object { Write-Host "    $_" }
    throw "demo\vla_fly.py refuses these arguments (above); nothing was started"
}
Say "arguments checked: mission $Mission, cadence $Cadence"

Remove-Item (Join-Path $Root "demo\out\$Tag\metrics.json") -ErrorAction SilentlyContinue
if ($OffloadRoot) {
    # The frames junction, as run_citylife_follow.ps1 (its comments say why
    # each branch exists): create it, repair a dangling one, or warn about a
    # real folder that already holds frames.
    $viewLink = Join-Path $Root "demo\out\$Tag\view"
    $viewDest = Join-Path $OffloadRoot "$Tag\view"
    $view = Get-Item -Force -LiteralPath $viewLink -ErrorAction SilentlyContinue
    if (-not $view) {
        New-Item -ItemType Directory -Force (Split-Path $viewLink) | Out-Null
        New-Item -ItemType Directory -Force $viewDest | Out-Null
        New-Item -ItemType Junction -Path $viewLink -Target $viewDest | Out-Null
        Say "frames -> $viewDest (junction)"
    } elseif ($view.LinkType) {
        $target = @($view.Target)[0]
        if (-not [IO.Path]::IsPathRooted($target)) { $target = Join-Path (Split-Path $viewLink) $target }
        if (-not (Test-Path -LiteralPath $target)) {
            New-Item -ItemType Directory -Force $target | Out-Null
            Say "frames -> $target (junction; its target was missing, recreated)"
        } else {
            Say "frames -> $target (junction)"
        }
    } else {
        $content = @(Get-ChildItem -Force -Recurse -LiteralPath $viewLink |
                     Where-Object { -not $_.PSIsContainer -or $_.LinkType })
        if ($content.Count -eq 0) {
            Remove-Item -Recurse -Force -LiteralPath $viewLink
            New-Item -ItemType Directory -Force $viewDest | Out-Null
            New-Item -ItemType Junction -Path $viewLink -Target $viewDest | Out-Null
            Say "frames -> $viewDest (junction; replaced an empty view folder)"
        } else {
            Warn "$viewLink is a real folder that already holds frames: this flight writes there (D:), and its recorder clears the old tps/fpv frames"
        }
    }
}

# Everything from here under try/finally: a -game simulator or a VLA server
# left running keeps the GPU busy (and a simulator keeps simulating).
$rc = 1
try {
    if (-not $SkipSim) { Start-Sim }
    Say "AerialVLA flies: mission $Mission, $Seconds s, tag $Tag, precision $Precision, cadence $Cadence"
    & $Py @a
    $rc = $LASTEXITCODE
}
finally {
    Stop-OurVlaServer
    if (-not $KeepSim -and -not $SkipSim) {
        Stop-OurGameSim
        Say "simulator stopped (pass -KeepSim to leave it running)"
    }
}

$metrics = Join-Path $Root "demo\out\$Tag\metrics.json"
if ($rc -eq 3) {
    Write-Host "  *** the mission ended on a VLA FAILURE (server dead, slow or erroring, malformed answers or no decision); the aircraft landed. Read end_reason in $metrics" -ForegroundColor Red
    exit 3
}
if ($rc -eq 4) {
    Write-Host "  *** REFUSED before the mission: the warm-up inference was too slow or unanswered (never took off), or the transfer missed the mission start (landed). Read refused / warmup in $metrics" -ForegroundColor Red
    exit 4
}
if ($rc -eq 130) {
    Write-Host "  *** INTERRUPTED (Ctrl+C): the aircraft landed where it was; the records say interrupted ($metrics)" -ForegroundColor Red
    exit 130
}
if ($rc -ne 0 -or -not (Test-Path $metrics)) {
    Write-Host "  *** FLIGHT FAILED (exit $rc). Read the console above." -ForegroundColor Red
    exit $(if ($rc) { $rc } else { 1 })
}
Say "done. Read the result from the artefacts, not the screen:"
Write-Host "    demo\out\$Tag\metrics.json   -> end_reason, reached_goal, pilot.summary (decision_hz, latency, authorship), land"
Write-Host "    demo\out\$Tag\vla_steps.jsonl -> one line per request: the server's answer and what the tracker made of it"
Write-Host "    demo\out\$Tag\kpi.json       -> p0_violation_escape_rate must be 0.0"
Write-Host "  then: python tools\vla_replay_gate.py --run demo\out\$Tag --vla-server 127.0.0.1:$Port  (with a server up)"
Write-Host "        python tools\make_demo_video.py --tag $Tag --height 720"
