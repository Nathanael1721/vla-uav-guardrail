# Project AirSim (CityLife) flown by ArduPilot SITL, with Mission Planner attached.
#
#   .\scripts\run_pas_ardupilot.ps1 -Step check          # addresses, firewall, deps, rendered configs
#   .\scripts\run_pas_ardupilot.ps1 -Step g1             # gate G1: three square-mission runs + verdict
#   .\scripts\run_pas_ardupilot.ps1 -Step node -Object "a red car" -Seconds 120
#   .\scripts\run_pas_ardupilot.ps1 -Step node -SaveView # ... and record the demo video
#   .\scripts\run_pas_ardupilot.ps1 -Step scene          # load the scene only, leave it running
#   .\scripts\run_pas_ardupilot.ps1 -Step sitl           # SITL + router (+ -Mavros) after -Step scene
#
# What runs where (docs/DESIGN-projectairsim-ardupilot.md has the diagram):
#   Windows : Unreal + Project AirSim (CityLife_Day), Mission Planner, the node / G1 script
#   WSL2    : ArduPilot SITL (--model airsim-copter), mavlink-router (or a MAVProxy
#             fallback), optionally MAVROS 2 (-Mavros, or -Link mavros)
#
# MISSION PLANNER: open it, choose UDP, port 14550, Connect. If Windows Firewall
# drops the UDP stream, choose TCP instead: host 127.0.0.1, port 5790 with
# mavlink-router, 5791 with the MAVProxy fallback (start_ardupilot.sh prints
# which; a MAVProxy TCP output serves one client, so the node keeps 5790).
# WSL forwards TCP from Windows' localhost, never UDP.
#
# ORDER: the scene is loaded first and ArduPilot started second. -Step node and
# -Step g1 do it themselves (--start-sitl). -Step node is the pymavlink node in
# this Windows env (not a grant topology). For the MAVROS node - dev in WSL, hil
# on the Jetson Orin - run -Step scene, then -Step sitl -Mavros (dev) or
# -Step sitl -Orin <orin ip> (hil), then the node there with --link mavros
# --scene-mode attach (the design doc has the exact commands). Starting SITL
# before a scene (re)load lets the simulator's clock reset run ArduPilot's
# clock backwards.
#
# POLICY: -Step node (and -Step check's dry run) fly the SIGNED bundle of the
# policy (-PolicyFile, default policies\follow_car_citylife.yaml) when
# tools\build_flight_bundles.py has built it into bundles\; otherwise they pass
# the YAML with --dev-yaml and say so: a development run, recorded as such and
# never KPI-grade. -ClockStart scripts the rule clock (ISO 8601); -HoldMode
# LOITER flies the FSM's Loiter as ArduCopter's LOITER with the throttle
# centred by the node (SITL only; BRAKE is the default).
#
# VIDEO: -Step node -SaveView records the demo video as follow_vlm does
# (perception_node.py --save-view): the first-person frame with the policy HUD
# (rule panel, the Shield / escalation-FSM banner, minimap) to
# demo\out\<tag>\view\fpv, the Chase camera to view\tps, and view\recorder.json.
# view\ is a junction to -OffloadRoot\<tag>\view on C:, as run_citylife_follow.ps1
# makes it (its CityLife flights wrote 1-2 GB of frames each, more than D: had
# free at the time; -OffloadRoot "" keeps them under demo\out). The closing lines print the make_demo_video command; tools\make_compare_video.py
# puts a Shield-on and a Shield-off run (-NodeArgs "--shield","off") in one.
#
# WARNING, as in every launcher here: starting the simulator closes any process
# holding PASBlocks\Blocks.uproject. An open EDITOR is refused, not killed.

param(
    [ValidateSet("check", "sim", "scene", "sitl", "node", "g1", "g1-summary")]
    [string]$Step    = "check",
    [string]$Distro  = "Ubuntu",
    [ValidateSet("auto", "nat", "mirrored")]
    [string]$Network = "auto",
    [ValidateSet("pymavlink", "mavros")]
    [string]$Link    = "pymavlink",
    [string]$Object  = "a red car",
    [int]$Seconds    = 120,
    [string]$Tag     = "",
    [int]$Runs       = 3,
    [int]$Seed       = 20261006,
    [double]$SideM   = 10,
    [double]$AltM    = 10,
    [string]$PolicyFile = "",
    [string]$ClockStart = "",           # ISO 8601: the rule clock's start (default: wall clock, +08:00)
    [ValidateSet("BRAKE", "LOITER")]
    [string]$HoldMode   = "BRAKE",
    [string]$G1Verdict  = "",           # default: the newest demo\out\g1_*\g1_verdict.json
    [string[]]$NodeArgs = @(),          # -Step node: passed on to perception_node.py as given (e.g. "--want-width","0.5","--shield","off")
    [string]$Orin    = "",              # Jetson Orin address: adds a MAVLink UDP output to it
    [int]$SimWidth   = 960,
    [int]$SimHeight  = 540,
    [switch]$SaveView,                  # -Step node: record the demo video (--save-view)
    # Frames go to C: through a junction, as run_citylife_follow.ps1; "" = under demo\out.
    [string]$OffloadRoot = "C:\Users\natha\vla_drone_offload\demo_out",
    [switch]$Mavros,                    # -Step sitl: also start MAVROS 2 in WSL (dev)
    [switch]$SkipSim,
    [switch]$KeepSim
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Py   = "C:\Users\natha\.conda\envs\vla-real\python.exe"
$Proj = Join-Path $Root "PASBlocks\Blocks.uproject"
$Map  = "/Game/CityLife/Maps/CityLife_Day"
$Node = Join-Path $Root "demo\pas_ardupilot\perception_node.py"
$G1   = Join-Path $Root "demo\pas_ardupilot\g1_check.py"
$Out  = Join-Path $Root "demo\out"
Set-Location $Root

function Say($m) { Write-Host "==> $m" -ForegroundColor Cyan }
function Warn($m) { Write-Host "  ! $m" -ForegroundColor Yellow }

# D:\a\b -> /mnt/d/a/b (works on Windows PowerShell 5.1 and PowerShell 7)
function ConvertTo-WslPath([string]$p) {
    $p = $p -replace '\\', '/'
    if ($p -match '^([A-Za-z]):(.*)$') { return "/mnt/" + $Matches[1].ToLower() + $Matches[2] }
    return $p
}

# ---- the policy: its signed bundle, else the YAML under --dev-yaml ----------
# The node refuses a YAML policy without --dev-yaml (guardrail.manifest.
# flight_policy_choice); the bundle's name comes from the same function the
# node and tools\build_flight_bundles.py use. A bundle is named by policy id
# and version only, so one python call also checks it still flies THIS
# YAML's policy (guardrail.manifest.flight_bundle_status): an edited YAML
# whose version was not bumped keeps a stale bundle's name.
function Get-PolicyArgs([string]$BundlesDir = (Join-Path $Root "bundles")) {
    $yaml = if ($PolicyFile) { $PolicyFile } else { Join-Path $Root "policies\follow_car_citylife.yaml" }
    $code = "import json, sys; sys.path.insert(0, sys.argv[1]); from guardrail.manifest import flight_bundle_status; print(json.dumps(flight_bundle_status(sys.argv[2], bundles_dir=sys.argv[3])))"
    $out = (& $Py -c $code $Root $yaml $BundlesDir | Select-Object -Last 1)
    if ($LASTEXITCODE -ne 0 -or -not $out) { throw "could not check the signed bundle of $yaml" }
    $st = "$out" | ConvertFrom-Json
    if ($st.present -and $st.matches -ne $false) {
        Say "policy: the signed bundle $($st.bundle)"
        return @("--bundle", $st.bundle)
    }
    if ($st.present) {
        Warn "the signed bundle $($st.bundle) is STALE: it flies $($st.bundle_hash),"
        Warn "and $yaml is now $($st.yaml_hash) (python tools\build_flight_bundles.py rebuilds it):"
    } else {
        Warn "no signed bundle of $yaml at $($st.bundle) (python tools\build_flight_bundles.py builds it):"
    }
    Warn "flying the YAML with --dev-yaml - a development run, recorded as such and never KPI-grade"
    return @("--policy", $yaml, "--dev-yaml")
}

# ---- WSL2 networking: which address each side must use --------------------
function Get-NetworkMode {
    if ($Network -ne "auto") { return $Network }
    $cfg = Join-Path $env:USERPROFILE ".wslconfig"
    if (Test-Path $cfg) {
        # An uncommented networkingMode=mirrored; '#' lines are comments.
        $hit = Get-Content $cfg | Where-Object { $_ -match '^\s*networkingMode\s*=\s*mirrored\s*$' }
        if ($hit) { return "mirrored" }
    }
    return "nat"
}

function Get-WslAddresses([string]$mode) {
    if ($mode -eq "mirrored") { return @{ Windows = "127.0.0.1"; Wsl = "127.0.0.1" } }
    $wsl = ((wsl.exe -d $Distro -- hostname -I) -join " ").Trim().Split(" ")[0]
    $route = ((wsl.exe -d $Distro -- ip route show default) -join " ").Trim().Split(" ")
    $win = $route[2]
    if (-not $wsl -or -not $win) { throw "could not read WSL addresses (is the $Distro distro installed?)" }
    return @{ Windows = $win; Wsl = $wsl }
}

# Read only. Port rules only: a program rule (e.g. one Windows created for
# UnrealEditor.exe when it first listened) is reported separately, because
# whether it covers this UDP port and the WSL adapter's profile cannot be told
# from here. A missing port rule is therefore a warning, not a verdict.
function Test-FirewallUdp([int]$port) {
    $f = Get-NetFirewallPortFilter -Protocol UDP -ErrorAction SilentlyContinue |
         Where-Object { $_.LocalPort -eq "$port" -or $_.LocalPort -eq "Any" }
    foreach ($x in $f) {
        $r = $x | Get-NetFirewallRule -ErrorAction SilentlyContinue
        if ($r -and $r.Enabled -eq "True" -and $r.Direction -eq "Inbound" -and $r.Action -eq "Allow") { return $true }
    }
    return $false
}
function Get-ProgramAllowRules([string]$exeName) {
    Get-NetFirewallApplicationFilter -ErrorAction SilentlyContinue |
        Where-Object { $_.Program -like "*\$exeName" } |
        ForEach-Object { $_ | Get-NetFirewallRule -ErrorAction SilentlyContinue } |
        Where-Object { $_.Enabled -eq "True" -and $_.Direction -eq "Inbound" -and $_.Action -eq "Allow" }
}

# ---- the simulator (as run_citylife_follow.ps1) ----------------------------
function Test-SimUp { (Test-NetConnection 127.0.0.1 -Port 8989 -WarningAction SilentlyContinue).TcpTestSucceeded }
function Stop-OurGameSim {
    Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
        Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and $_.CommandLine -like '*-game*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}
function Start-Sim {
    . "$Root/tools/Resolve-UnrealEngine.ps1"
    $UE = Get-UnrealEditor -UProject $Proj
    $editors = @(Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" |
                 Where-Object { $_.CommandLine -like '*Blocks.uproject*' -and $_.CommandLine -notlike '*-game*' })
    if ($editors.Count -gt 0) {
        throw "an Unreal EDITOR is open on this project (PID $($editors[0].ProcessId)); save and close it, or pass -SkipSim"
    }
    Stop-OurGameSim
    Start-Sleep -Seconds 4
    Start-Process -FilePath $UE -ArgumentList "`"$Proj`"", $Map, '-game', '-windowed', "-ResX=$SimWidth", "-ResY=$SimHeight" | Out-Null
    Say "simulator starting on $Map ..."
    for ($i = 0; $i -lt 90; $i++) {
        Start-Sleep -Seconds 5
        if (Test-SimUp) { Start-Sleep -Seconds 8; Say "simulator ready"; return }
    }
    throw "simulator did not open port 8989"
}

# ---- -SaveView: demo\out\<tag>\view as a junction to the C: offload folder --
# The same rules as run_citylife_follow.ps1, which has the history behind each.
function Set-ViewJunction([string]$RunTag) {
    $viewLink = Join-Path $Out "$RunTag\view"
    if (-not $OffloadRoot) { Say "frames -> $viewLink (-OffloadRoot `"`": no junction)"; return }
    $viewDest = Join-Path $OffloadRoot "$RunTag\view"
    # Get-Item on the link itself, not Test-Path: Test-Path is True for a
    # junction whose target was deleted, and nothing would recreate it.
    $view = Get-Item -Force -LiteralPath $viewLink -ErrorAction SilentlyContinue
    if (-not $view) {
        New-Item -ItemType Directory -Force (Split-Path $viewLink) | Out-Null
        New-Item -ItemType Directory -Force $viewDest | Out-Null
        New-Item -ItemType Junction -Path $viewLink -Target $viewDest | Out-Null
        Say "frames -> $viewDest (junction)"
    } elseif ($view.LinkType) {
        # Windows PowerShell 5.1 returns Target as string[], 7 as a string.
        $target = @($view.Target)[0]
        if (-not [IO.Path]::IsPathRooted($target)) { $target = Join-Path (Split-Path $viewLink) $target }
        if (-not (Test-Path -LiteralPath $target)) {
            New-Item -ItemType Directory -Force $target | Out-Null
            Say "frames -> $target (junction; its target was missing, recreated)"
        } else {
            Say "frames -> $target (junction)"
        }
        $under = [IO.Path]::GetFullPath($OffloadRoot).TrimEnd('\') + '\'
        if (-not [IO.Path]::GetFullPath($target).StartsWith($under, [StringComparison]::OrdinalIgnoreCase)) {
            Warn "$viewLink points at $target, outside -OffloadRoot $OffloadRoot. The frames go there."
        }
    } else {
        # A real folder. Only files or links count as content: a run that
        # failed early leaves the empty view\tps and view\fpv and nothing else.
        $content = @(Get-ChildItem -Force -Recurse -LiteralPath $viewLink |
                     Where-Object { -not $_.PSIsContainer -or $_.LinkType })
        if ($content.Count -eq 0) {
            Remove-Item -Recurse -Force -LiteralPath $viewLink
            New-Item -ItemType Directory -Force $viewDest | Out-Null
            New-Item -ItemType Junction -Path $viewLink -Target $viewDest | Out-Null
            Say "frames -> $viewDest (junction; replaced an empty view folder)"
        } else {
            # Never delete frames here and never send them to D: silently. The
            # recorder itself clears view\tps and view\fpv of *.jpg when it
            # starts (demo/recorder.py, _clear), and the warning says so.
            $free = (Get-PSDrive (Split-Path -Qualifier $viewLink).TrimEnd(':')).Free / 1GB
            Warn ("$viewLink is a real folder that already holds frames, so this run writes to " +
                  "$(Split-Path -Qualifier $viewLink) ($('{0:N1}' -f $free) GB free; a flight is 1-2 GB), " +
                  "not to $viewDest, and its recorder deletes the old tps/fpv frames there when it " +
                  "starts. To keep them, stop this run (Ctrl+C) and move the folder's contents to " +
                  "$viewDest, or fly under a new -Tag.")
        }
    }
}

if ($SaveView -and $Step -ne "node") {
    throw "-SaveView records the perception node's flight; it needs -Step node"
}

if ($Step -eq "g1-summary") {
    # Summarising a tag nobody named would summarise an empty, fresh folder.
    if (-not $Tag) { throw "-Step g1-summary needs -Tag <the g1_... tag of the runs to summarise>" }
    & $Py $G1 summarize --tag $Tag --runs $Runs
    exit $LASTEXITCODE
}

if ($Step -eq "node" -and $Link -eq "mavros") {
    # Refused before anything starts (no WSL query, no simulator, no SITL):
    # the MAVROS link needs ROS 2 (rclpy, mavros_msgs), which this Windows env
    # does not have. The MAVROS node runs in WSL (dev) or on the Jetson Orin
    # (hil) and ATTACHES to a scene the desktop loaded; the design doc has both.
    throw ("-Step node runs the node in this Windows env, which has no ROS 2; " +
           "-Link mavros runs in WSL or on the Orin: -Step scene, then -Step sitl -Mavros " +
           "(dev) or -Step sitl -Orin <ip> (hil), then perception_node.py --link mavros " +
           "--scene-mode attach there (docs\DESIGN-projectairsim-ardupilot.md)")
}

$mode = Get-NetworkMode
$ip = Get-WslAddresses $mode
$net = @("--network", $mode, "--windows-ip", $ip.Windows, "--wsl-ip", $ip.Wsl, "--wsl-distro", $Distro)
if (-not $Tag) {
    $prefix = @{ "g1" = "g1_"; "scene" = "pas_scene_" }[$Step]
    if (-not $prefix) { $prefix = "pas_ap_" }
    $Tag = $prefix + (Get-Date -Format "yyyyMMdd_HHmmss")
}
Say "WSL2 networking: $mode | Windows (as WSL sees it) $($ip.Windows) | WSL $($ip.Wsl)"

if ($Step -eq "check") {
    Say "firewall (read only; this script changes no firewall setting)"
    # 9002: ArduPilot's PWM into Project AirSim. 14550: the router's stream to
    # Mission Planner. 14551: the router's stream to a node listening on UDP
    # (unused by default: the node connects to the router's TCP port).
    foreach ($p in @(9002, 14550, 14551)) {
        if ($mode -eq "nat" -and -not (Test-FirewallUdp $p)) {
            Warn "no inbound-allow PORT rule found for UDP $p. If the step fails with no PWM (9002) or"
            Warn "Mission Planner sees nothing on 14550, an administrator can add one:"
            Warn "  New-NetFirewallRule -DisplayName 'PAS ArduPilot UDP $p' -Direction Inbound -Protocol UDP -LocalPort $p -Action Allow"
        } else { Say "UDP $p : ok (port rule)" }
    }
    foreach ($exe in @("UnrealEditor.exe", "MissionPlanner.exe")) {
        $pr = @(Get-ProgramAllowRules $exe)
        if ($pr.Count -gt 0) { Say "$exe has $($pr.Count) inbound-allow program rule(s) (may cover its UDP port too; not checked)" }
    }
    Say "WSL side"
    # the router is built into ~/.local/bin, which start_ardupilot.sh puts on PATH
    $rt = (wsl.exe -d $Distro -- sh -c 'command -v mavlink-routerd || test -x ~/.local/bin/mavlink-routerd && echo found || echo MISSING') -join ""
    if ($rt -match "MISSING") { Warn "mavlink-routerd not installed in ${Distro}: the MAVProxy fallback will be used (not the grant's router; Mission Planner TCP is then 5791)" }
    else { Say "mavlink-routerd: $rt" }
    wsl.exe -d $Distro -- bash "$(ConvertTo-WslPath $Root)/sitl/setup_sitl.sh" --verify
    Say "node dry run (renders this run's configs, lists missing Python packages)"
    $pol = Get-PolicyArgs
    & $Py $Node --dry-run --tag "$Tag" --link $Link @net @pol
    exit $LASTEXITCODE
}

if ($Step -eq "g1") {
    $tagDir = Join-Path $Out $Tag
    if (Test-Path (Join-Path $tagDir "run_*")) {
        throw "$tagDir already holds G1 runs; a G1 tag is one invocation. Omit -Tag for a fresh one."
    }
}

# -Step scene leaves the scene loaded for ArduPilot and an attaching node, so
# the simulator must outlive this script.
if ($Step -eq "scene") { $KeepSim = $true }

$startedSim = $false
try {
    if (-not $SkipSim -and $Step -ne "sitl") { Start-Sim; $startedSim = $true }
    elseif (-not (Test-SimUp) -and $Step -ne "sitl") { throw "-SkipSim given but nothing listens on 8989" }

    switch ($Step) {
        "sim" { Say "simulator up; Ctrl+C to stop"; while ($true) { Start-Sleep 5 } }
        "scene" {
            & $Py $Node --scene-only --tag $Tag @net
            if ($LASTEXITCODE -ne 0) { throw "scene load exited $LASTEXITCODE" }
            Say "scene loaded and left running. Next: -Step sitl -SkipSim (ArduPilot AFTER the scene, never before a reload)"
        }
        "sitl" {
            # Needs a scene loaded first (-Step scene); reloading it
            # while this runs drives ArduPilot's clock backwards.
            if (-not (Test-SimUp)) { throw "nothing listens on 8989: load the scene first (-Step scene)" }
            $wslScript = (ConvertTo-WslPath $Root) + "/demo/pas_ardupilot/start_ardupilot.sh"
            $extra = @()
            if ($Orin) { $extra += @("--orin", $Orin) }
            if ($Mavros -or $Link -eq "mavros") { $extra += @("--mavros") }
            wsl.exe -d $Distro -- bash $wslScript --sim-address $ip.Windows --gcs-address $ip.Windows --node-address $ip.Windows @extra
        }
        "node" {
            $a = @($Node, "--object", $Object, "--tag", $Tag, "--max-s", "$Seconds", "--start-sitl", "--link", $Link) + $net
            $a += Get-PolicyArgs
            $a += @("--hold-mode", $HoldMode)
            if ($ClockStart) { $a += @("--clock-start", $ClockStart) }
            if ($NodeArgs) { Say "node args: $($NodeArgs -join ' ')"; $a += $NodeArgs }
            if (-not $G1Verdict) {
                $v = Get-ChildItem (Join-Path $Out "g1_*\g1_verdict.json") -ErrorAction SilentlyContinue |
                     Sort-Object LastWriteTime -Descending | Select-Object -First 1
                if ($v) { $G1Verdict = $v.FullName }
            }
            if ($G1Verdict) { Say "G1 verdict: $G1Verdict"; $a += @("--g1-verdict", $G1Verdict) }
            else { Warn "no G1 verdict under demo\out\g1_*: this run cannot be KPI-grade (run -Step g1 first)" }
            if ($SaveView) {
                Set-ViewJunction $Tag
                $a += @("--save-view")
            }
            & $Py @a
            if ($LASTEXITCODE -ne 0) { throw "perception node exited $LASTEXITCODE" }
            if ($SaveView) {
                $side = Join-Path $Out "$Tag\view\recorder.json"
                if (Test-Path -LiteralPath $side) {
                    Say "frames and recorder.json in demo\out\$Tag\view. Build the video:"
                } else {
                    Warn "no demo\out\$Tag\view\recorder.json: make_demo_video falls back to a guessed rate"
                }
                Write-Host "    python tools\make_demo_video.py --tag $Tag --height 720"
                Write-Host "  or, beside a run flown with -NodeArgs `"--shield`",`"off`":"
                Write-Host "    python tools\make_compare_video.py --a $Tag --b <shield-off tag>"
            }
        }
        "g1" {
            for ($r = 1; $r -le $Runs; $r++) {
                # A fresh simulator per run. With the ardupilot-api controller the
                # steppable clock waits for ArduPilot's PWM; once the previous
                # run's ArduPilot is gone, a scene reload never returns
                # (2026-10-08: runs 2 and 3 timed out in /Sim/LoadScene).
                if ($r -gt 1 -and $startedSim) {
                    wsl.exe -d $Distro -- bash -c "pkill -9 -f '[a]rducopter --model airsim' ; pkill -9 -f '[m]avlink-routerd -c' ; pkill -9 -f '[m]avproxy.py --master=tcp:127.0.0.1:5760' ; true" 2>$null
                    Say "restarting the simulator for run $r"
                    Start-Sim
                }
                Say "G1 run $r of $Runs (seed $Seed)"
                & $Py $G1 run --tag $Tag --run $r --seed $Seed --side-m $SideM --alt-m $AltM --start-sitl @net
                if ($LASTEXITCODE -ne 0) { Warn "run $r exited $LASTEXITCODE; summarize counts it as a FAILED run (demo\out\$Tag\run_$r)" }
                # A fresh scene per run: the next run's World() reload resets the drone,
                # and its own --start-sitl starts a fresh ArduPilot after it.
            }
            & $Py $G1 summarize --tag $Tag --runs $Runs
            Say "verdict: demo\out\$Tag\g1_verdict.md"
        }
    }
}
finally {
    if ($startedSim -and -not $KeepSim) { Say "stopping the simulator"; Stop-OurGameSim }
    if ($Step -ne "scene") {
        wsl.exe -d $Distro -- bash -c "pkill -9 -f '[a]rducopter --model airsim' ; pkill -9 -f '[m]avlink-routerd -c' ; pkill -9 -f '[m]avproxy.py --master=tcp:127.0.0.1:5760' ; pkill -9 -f '[m]avros_node.*14555' ; true" 2>$null
    }
}
