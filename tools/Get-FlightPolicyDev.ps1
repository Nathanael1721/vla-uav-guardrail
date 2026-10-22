# The city rail's policy: its SIGNED bundle, else the YAML under --dev-yaml.
#
# Dot-source it, then:   $a += Get-FlightPolicyDev $Root $Py $PolicyYaml [-DevYaml]
#
# demo/follow_vlm.py flies the signed bundle of --policy P from bundles\
# (python tools\build_flight_bundles.py builds it) and refuses P itself without
# --dev-yaml (follow_vlm.city_policy_choice, audit card ARCH-29, 2026-10-21).
# This answers, before the simulator starts, whether that bundle is on disk and
# still P's policy (guardrail.manifest.flight_bundle_status - a bundle is named
# by policy id and version only, so an edit without a version bump leaves a
# stale one), as run_pas_ardupilot.ps1's Get-PolicyArgs does for the ArduPilot
# rail. It returns @() when the bundle flies, and @("--dev-yaml") - saying why -
# when it does not, or when -DevYaml asks for the YAML: a development run,
# recorded as such and never KPI-grade.
function Get-FlightPolicyDev([string]$Root, [string]$Py, [string]$Policy, [switch]$DevYaml,
                             [string]$BundlesDir = (Join-Path $Root "bundles")) {
    if ($DevYaml) {
        Write-Warning "-DevYaml: flying $Policy unsigned (--dev-yaml), a development run"
        return @("--dev-yaml")
    }
    $path = if ([IO.Path]::IsPathRooted($Policy)) { $Policy } else { Join-Path $Root $Policy }
    $code = "import json, sys; sys.path.insert(0, sys.argv[1]); from guardrail.manifest import flight_bundle_status; print(json.dumps(flight_bundle_status(sys.argv[2], bundles_dir=sys.argv[3])))"
    $out = (& $Py -c $code $Root $path $BundlesDir | Select-Object -Last 1)
    if ($LASTEXITCODE -ne 0 -or -not $out) { throw "could not check the signed bundle of $Policy" }
    $st = "$out" | ConvertFrom-Json
    if ($st.present -and $st.matches -ne $false) {
        Write-Host "==> policy: the signed bundle $($st.bundle)" -ForegroundColor Cyan
        return @()
    }
    if ($st.present) {
        Write-Warning ("the signed bundle $($st.bundle) is STALE: it flies $($st.bundle_hash), " +
                       "and $Policy is now $($st.yaml_hash) (python tools\build_flight_bundles.py rebuilds it)")
    } else {
        Write-Warning "no signed bundle of $Policy at $($st.bundle) (python tools\build_flight_bundles.py builds it)"
    }
    Write-Warning "flying the YAML with --dev-yaml - a development run, recorded as such and never KPI-grade"
    return @("--dev-yaml")
}
