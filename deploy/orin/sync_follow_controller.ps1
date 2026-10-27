# Copy what the Jetson needs to run the hand-written follow
# (deploy/orin/run_follow_controller.sh) from this desktop over SSH:
#   * the repository at HEAD (git archive: tracked files only, so the
#     gitignored private signing key policies/keys/*.pem never leaves),
#   * the signed policy bundles the follow flies (bundles/follow-car-citylife*.tar.gz,
#     gitignored, built by tools/build_flight_bundles.py),
#   * the CityLife map folder (demo/out/citymap_citylife/: occupancy bands,
#     street mask, voxels; gitignored).
# Uncommitted changes are NOT sent; the script warns when the tree is dirty and
# writes the commit it sent to <Dest>/DEPLOYED_COMMIT.
#
#   .\deploy\orin\sync_follow_controller.ps1 -JetsonHost nathan@192.168.50.2
#
# Needs Windows' OpenSSH client (ssh, scp) and an SSH login on the Jetson.
param(
    [Parameter(Mandatory = $true)][string]$JetsonHost,   # user@address
    [string]$Dest = "~/vla-drone"
)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
Set-Location $Root
$commit = (git rev-parse HEAD).Trim()
if (git status --porcelain --untracked-files=no) {
    Write-Warning "the working tree has uncommitted changes: they are NOT sent (HEAD $commit is)"
}
$tmp = Join-Path $env:TEMP "vla-drone-$($commit.Substring(0,8)).tar"
git archive --format=tar -o $tmp HEAD
$extras = @(Get-ChildItem bundles -Filter "follow-car-citylife*.tar.gz" | ForEach-Object { "bundles/$($_.Name)" })
# The WHOLE CityLife map folder, not just occ_day.npz: the altitude-band
# occupancy maps, street.npz (street mask, junction planner, landing sites)
# and the voxels. orin_follow_01 (27 Oct) had occ_day.npz alone and flew with
# ObstacleClearance inert, no street mask, no junction planner, no landing site.
$extras += @(Get-ChildItem demo/out/citymap_citylife -File | ForEach-Object { "demo/out/citymap_citylife/$($_.Name)" })
foreach ($e in $extras) { if (-not (Test-Path $e)) { throw "missing $e" } }

Write-Host "==> sending HEAD $commit ($([math]::Round((Get-Item $tmp).Length/1MB,1)) MB) to ${JetsonHost}:$Dest"
ssh $JetsonHost "mkdir -p $Dest/bundles $Dest/demo/out/citymap_citylife"
if ($LASTEXITCODE -ne 0) { throw "ssh $JetsonHost failed" }
scp $tmp "${JetsonHost}:/tmp/vla-drone.tar"
if ($LASTEXITCODE -ne 0) { throw "scp failed" }
ssh $JetsonHost "tar -xf /tmp/vla-drone.tar -C $Dest && rm /tmp/vla-drone.tar && echo $commit > $Dest/DEPLOYED_COMMIT"
if ($LASTEXITCODE -ne 0) { throw "extract failed" }
foreach ($e in $extras) {
    scp $e "${JetsonHost}:$Dest/$e"
    if ($LASTEXITCODE -ne 0) { throw "scp $e failed" }
}
Remove-Item $tmp
Write-Host "==> done: $Dest at $commit, plus $($extras.Count) gitignored files"
