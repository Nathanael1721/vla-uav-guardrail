# Fly the CSP campaign: OpenVLA-7B reads the Constraint Summary Pack, A/B and
# paraphrases (WP2-01, WP2-09, WP4-15, WP4-09). PowerShell 7.
#
# WHAT IT FLIES, IN ORDER (the plan is tools\score_csp_flights.py plan's):
#   1. the A/B: for each -Seeds (1001, 1002, 1003), demo\real_vla_demo.py with
#      --csp off, then --csp on. Same policy, instruction, precision, Shield and
#      length; every flight starts from the simulator's spawn, because the
#      simulator is restarted before each one (AirSim's reset is not used:
#      it leaves the simulator command-deaf on this build).
#   2. the paraphrase set, CSP on, at the first seed: per -Families
#      (verb_phrase, utterance) the first -PerFamily stored paraphrases of the
#      instruction (4 each), by --paraphrase-index; utterances with
#      --paraphrase-allow-utterance (the grammar confound is recorded).
# Each run is demo\out\<Prefix>_ab_s<seed>_<off|on> or
# demo\out\<Prefix>_pp_<family>_<index>; then the scorer prints the WP4-15
# deltas and the WP4-09 outcome stability and writes demo\out\<Prefix>_score.json.
#
# THE SIMULATOR IS CLASSIC AIRSIM, AirSimNH - NOT CityLife. real_vla_demo talks
# to AirSim through the `airsim` package (msgpack-rpc, port 41451) and flies
# urban_demo_policy.yaml, which was drawn for AirSimNH. CityLife is Project
# AirSim (port 8989), another protocol this demo cannot speak. The lab's
# Documents\AirSim\settings.json declares an ArduCopter vehicle (the SITL rail,
# since September), which ignores AirSim's velocity API; this script never
# touches that file: it starts AirSimNH with tools\airsim_settings_real_vla.json
# (SimpleFlight, 1280x720 scene camera as in July) through AirSim's -settings=
# argument, copied first to a path without spaces. real_vla_demo checks the
# vehicle before its model loads and refuses anything but SimpleFlight.
#
# THE EXACT SEQUENCE (vla-real env; nothing else on the GPU; no tools\ci_local.py
# running - it slows the model and the renderer, so the script refuses then):
#
#   cd "D:\OneDrive\College\S2-TaipeiTech\Lab\VLA Drone"
#   # 0. dry run: every flight's prompt, tokens, CSP hash and configuration,
#   #    no model, no simulator (seconds)
#   pwsh -File tools\run_csp_flights.ps1 -Prefix cspab_20261107 -DryRun
#   # 1. one flight first, to see the rail work end to end (~6-8 min)
#   pwsh -File tools\run_csp_flights.ps1 -Prefix cspsmoke_20261107 -Phase ab -Seeds 1001 -MaxS 30
#   # 2. the campaign: 6 A/B flights then 8 paraphrase flights, scored at the end
#   pwsh -File tools\run_csp_flights.ps1 -Prefix cspab_20261107
#   #    (-Precision bf16 for the unquantized model on the 24 GB card;
#   #     -SkipDone resumes a campaign without re-flying finished runs)
#   # 3. score again at any time (no model, no simulator)
#   C:\Users\natha\.conda\envs\vla-real\python.exe tools\score_csp_flights.py score --prefix cspab_20261107 --json demo\out\cspab_20261107_score.json
#   # 4. the offline replay of each run (WP2-09; the GPU model, one run at a time;
#   #    the scorer prints the line for every finished run)
#   C:\Users\natha\.conda\envs\vla-real\python.exe tools\prefix_eval.py demo\out\cspab_20261107_ab_s1001_on --out demo\out\cspab_20261107_ab_s1001_on\prefix_eval.json
#
# EXPECTED DURATIONS (estimates: nothing here has flown). Restarting AirSimNH
# 0.5-1 min; loading OpenVLA 2-5 min at nf4 (289 s in July) and about 2 min at
# bf16 (117 s in the 7 Nov smoke check: 15 GB read from D:, an SMR disk that
# slows under long copies); take-off and climb to 20 m ~0.3 min; the flight
# -MaxS (60 s); the landing ~0.5 min. About 5-8 min a flight: the A/B (6
# flights) 30-50 min, the paraphrase set (8) 40-65 min, the campaign 1.2-2 h.
# A decision takes ~0.9 s at nf4 (July) and 0.55-0.61 s at bf16 with no
# simulator running (7 Nov); the renderer sharing the GPU slows both. A replay
# with prefix_eval is a model load plus 3 prompts x the run's frames (~60-100
# frames), about 4-8 min a run.
#
# WARNING: like the other demo scripts, this kills every running AirSimNH,
# Blocks and LandscapeMountains process before each flight.

param(
    [string]$Prefix = "cspab_$(Get-Date -Format yyyyMMdd)",
    [ValidateSet("all", "ab", "paraphrase")][string]$Phase = "all",
    [int[]]$Seeds = @(1001, 1002, 1003),
    [ValidateSet("nf4", "bf16")][string]$Precision = "nf4",
    [string]$Instruction = "fly forward and avoid restricted areas",
    [string]$PolicyFile = "policies/urban_demo_policy.yaml",
    [string[]]$Families = @("verb_phrase", "utterance"),
    [int]$PerFamily = 4,
    [double]$MaxS = 60,
    [string]$SimExe = "D:\AirSim\AirSimNH\WindowsNoEditor\AirSimNH.exe",
    [int]$SimWidth = 1280,
    [int]$SimHeight = 720,
    [int]$SimReadyTimeoutS = 180,
    # The camera frames each run keeps (1280x720 PNGs, about one a second) go
    # outside OneDrive through a junction demo\out\<tag>\frames, as the city
    # launchers do with view\; "" writes them under demo\out.
    [string]$OffloadRoot = "D:\vla_drone_offload\demo_out",
    [switch]$DryRun,                    # check every flight's prompt and configuration; fly nothing
    [switch]$SkipDone,                  # leave runs that already have kpi.json
    [switch]$ContinueOnRefusal,         # a refused flight (exit 3) does not stop the campaign
    [switch]$IgnoreBusy,                # fly while tools\ci_local.py runs (not comparable)
    [switch]$NoScore,
    [switch]$KeepSim
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Py = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$Settings = Join-Path $Root "tools\airsim_settings_real_vla.json"

function Say($m) { Write-Host "==> $m" -ForegroundColor Cyan }
function Warn($m) { Write-Host "  ! $m" -ForegroundColor Yellow }

foreach ($p in @($Py, $Settings)) { if (-not (Test-Path $p)) { throw "not found: $p" } }
Set-Location $Root

# The plan's own arguments, the same for plan, check and the scorer's tags.
$planArgs = @("--prefix", $Prefix, "--phase", $Phase,
              "--seeds", ($Seeds -join ","), "--precision", $Precision,
              "--instruction", $Instruction, "--policy", $PolicyFile,
              "--families", ($Families -join ","), "--per-family", "$PerFamily",
              "--max-s", "$MaxS")

if ($DryRun) {
    Say "dry run of $Prefix ($Phase): every flight's --print-prompt, no model, no simulator"
    & $Py tools\score_csp_flights.py check @planArgs --json (Join-Path $Root "demo\out\${Prefix}_dryrun.json")
    exit $LASTEXITCODE
}

if (-not (Test-Path $SimExe)) { throw "not found: $SimExe (the AirSimNH binary, v1.8.1-windows)" }
$busy = @(Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
          Where-Object { $_.CommandLine -like '*ci_local.py*' })
if ($busy.Count -gt 0 -and -not $IgnoreBusy) {
    throw ("tools\ci_local.py is running (PID $($busy[0].ProcessId)): it shares the CPU and " +
           "slows the model and the renderer, so the flights would not be comparable. " +
           "Wait for it, or pass -IgnoreBusy.")
}

$plan = @(& $Py tools\score_csp_flights.py plan @planArgs --json | ConvertFrom-Json)
if ($LASTEXITCODE -ne 0 -or $plan.Count -eq 0) { throw "the plan was refused (above)" }
Say "$($plan.Count) flight(s), prefix $Prefix, precision $Precision, $MaxS s each"

# The settings file, at a path without spaces (it then needs no quoting), and
# -settings= last on the command line. Whether this AirSimNH build honoured it
# is checked by real_vla_demo before its model loads (the vehicle must be
# SimpleFlight); the working directory holds the same file as settings.json.
$SimDir = Join-Path $env:LOCALAPPDATA "vla-guardrail\airsim"
New-Item -ItemType Directory -Force $SimDir | Out-Null
$SimSettings = Join-Path $SimDir "settings.json"
Copy-Item -Force $Settings $SimSettings
if ($SimSettings -match '\s') { throw "the settings path has a space: $SimSettings" }

function Test-SimUp {
    [bool](Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue)
}

function Stop-Sim {
    Get-Process -Name AirSimNH, Blocks, LandscapeMountains -ErrorAction SilentlyContinue |
        Stop-Process -Force -ErrorAction SilentlyContinue
}

function Start-Sim {
    Stop-Sim
    Start-Sleep -Seconds 5
    $simArgs = @("-ResX=$SimWidth", "-ResY=$SimHeight", "-windowed", "-settings=$SimSettings")
    Start-Process -FilePath $SimExe -ArgumentList $simArgs -WorkingDirectory $SimDir | Out-Null
    Say "AirSimNH starting (SimpleFlight settings $SimSettings) ..."
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $SimReadyTimeoutS) {
        Start-Sleep -Seconds 3
        if (Test-SimUp) { Start-Sleep -Seconds 8; Say "simulator ready on 41451"; return }
    }
    throw "AirSimNH did not open port 41451 within $SimReadyTimeoutS s (a 'Choose Vehicle' dialog means the settings were not read)"
}

# demo\out\<tag>\frames -> <OffloadRoot>\<tag>\frames: create it, repair a
# dangling one, or say where the frames go when it is a real folder.
function Set-FramesJunction([string]$Tag) {
    if (-not $OffloadRoot) { return }
    $link = Join-Path $Root "demo\out\$Tag\frames"
    $dest = Join-Path $OffloadRoot "$Tag\frames"
    $item = Get-Item -Force -LiteralPath $link -ErrorAction SilentlyContinue
    if (-not $item) {
        New-Item -ItemType Directory -Force (Split-Path $link) | Out-Null
        New-Item -ItemType Directory -Force $dest | Out-Null
        New-Item -ItemType Junction -Path $link -Target $dest | Out-Null
        Say "frames -> $dest (junction)"
    } elseif ($item.LinkType) {
        $target = @($item.Target)[0]
        if (-not (Test-Path -LiteralPath $target)) { New-Item -ItemType Directory -Force $target | Out-Null }
        Say "frames -> $target (junction)"
    } else {
        Warn "$link is a real folder: this run's frames go there (OneDrive), not to $dest"
    }
}

$results = @()
$stopped = $null
try {
    foreach ($f in $plan) {
        $kpi = Join-Path $Root "demo\out\$($f.tag)\kpi.json"
        if ($SkipDone -and (Test-Path $kpi)) {
            Say "$($f.tag): done already (kpi.json), skipped"
            $results += [pscustomobject]@{ tag = $f.tag; rc = "skipped" }
            continue
        }
        Set-FramesJunction $f.tag
        Start-Sim
        Say "flight $($f.tag): csp $($f.csp), seed $($f.seed)$(if ($null -ne $f.paraphrase_index) { ", paraphrase $($f.family) #$($f.paraphrase_index)" })"
        $t1 = Get-Date
        & $Py demo\real_vla_demo.py @($f.args)
        $rc = $LASTEXITCODE
        $mins = [math]::Round(((Get-Date) - $t1).TotalMinutes, 1)
        $results += [pscustomobject]@{ tag = $f.tag; rc = $rc; minutes = $mins }
        if ($rc -eq 3) {
            Warn "$($f.tag) was REFUSED (exit 3): read the [sim] / [prompt] / [flight] lines above"
            if (-not $ContinueOnRefusal) { $stopped = "refusal"; break }
        } elseif ($rc -ne 0) {
            Warn "$($f.tag) FAILED (exit $rc)"
            $stopped = "failure"
            break
        }
    }
}
finally {
    if (-not $KeepSim) { Stop-Sim; Say "simulator stopped (pass -KeepSim to leave it running)" }
}

$results | Format-Table -AutoSize | Out-String | Write-Host
if ($stopped) { Warn "the campaign stopped early ($stopped); -SkipDone resumes it after the fix" }
if (-not $NoScore) {
    & $Py tools\score_csp_flights.py score --prefix $Prefix --json (Join-Path $Root "demo\out\${Prefix}_score.json")
}
if ($stopped) { exit 1 }
exit 0
