# Copy what the Jetson needs to run the hand-written follow
# (deploy/orin/run_follow_controller.sh) from this desktop over SSH:
#   * the repository at HEAD as a git checkout (a git bundle fetched into
#     <Dest> and checked out there), so guardrail.manifest.code_revision on the
#     Jetson names the commit and says -dirty when the tree changed. Until
#     2026-10-28 this sent `git archive` (no .git): every Jetson run recorded
#     code_revision "unversioned", which the KPI gate refuses. History only:
#     the gitignored private signing key policies/keys/*.pem was never
#     committed, so it never leaves,
#   * the signed policy bundles the follow flies (bundles/follow-car-citylife*.tar.gz,
#     gitignored, built by tools/build_flight_bundles.py),
#   * the CityLife map folder (demo/out/citymap_citylife/: occupancy bands,
#     street mask, voxels; gitignored).
# Uncommitted changes are NOT sent; the script warns when the tree is dirty and
# prints the Jetson checkout's commit and `git status` at the end.
#
#   .\deploy\orin\sync_follow_controller.ps1 -JetsonHost nathan@10.42.0.2
#   # where Windows' ssh.exe refuses the key, Git for Windows' client:
#   .\deploy\orin\sync_follow_controller.ps1 -JetsonHost nathan@10.42.0.2 `
#       -Ssh "C:\Program Files\Git\usr\bin\ssh.exe" -Scp "C:\Program Files\Git\usr\bin\scp.exe"
#
# Needs an OpenSSH client (ssh, scp), an SSH login on the Jetson and git there.
param(
    [Parameter(Mandatory = $true)][string]$JetsonHost,   # user@address
    [string]$Dest = "~/vla-drone",
    [string]$Ssh = "ssh",
    [string]$Scp = "scp"
)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
Set-Location $Root
$commit = (git rev-parse HEAD).Trim()
if (git status --porcelain --untracked-files=no) {
    Write-Warning "the working tree has uncommitted changes: they are NOT sent (HEAD $commit is)"
}
$tmp = Join-Path $env:TEMP "vla-drone-$($commit.Substring(0,8)).bundle"
git bundle create $tmp HEAD
if ($LASTEXITCODE -ne 0) { throw "git bundle failed" }
$extras = @(Get-ChildItem bundles -Filter "follow-car-citylife*.tar.gz" | ForEach-Object { "bundles/$($_.Name)" })
# The WHOLE CityLife map folder, not just occ_day.npz: the altitude-band
# occupancy maps, street.npz (street mask, junction planner, landing sites)
# and the voxels. orin_follow_01 (27 Oct) had occ_day.npz alone and flew with
# ObstacleClearance inert, no street mask, no junction planner, no landing site.
$extras += @(Get-ChildItem demo/out/citymap_citylife -File | ForEach-Object { "demo/out/citymap_citylife/$($_.Name)" })
foreach ($e in $extras) { if (-not (Test-Path $e)) { throw "missing $e" } }

Write-Host "==> sending HEAD $commit ($([math]::Round((Get-Item $tmp).Length/1MB,1)) MB bundle) to ${JetsonHost}:$Dest"
& $Ssh $JetsonHost "mkdir -p $Dest/bundles $Dest/demo/out/citymap_citylife"
if ($LASTEXITCODE -ne 0) { throw "ssh $JetsonHost failed" }
& $Scp $tmp "${JetsonHost}:/tmp/vla-drone.bundle"
if ($LASTEXITCODE -ne 0) { throw "scp failed" }
# A first sync turns an older git-archive copy into a checkout: files the
# archive left that HEAD no longer tracks show up as untracked (listed, never
# deleted here). DEPLOYED_COMMIT was the archive's stand-in for .git.
$remote = @"
set -e
cd $Dest
[ -d .git ] || git init -q
git fetch -q /tmp/vla-drone.bundle HEAD
git checkout -q -f --detach FETCH_HEAD
rm -f /tmp/vla-drone.bundle DEPLOYED_COMMIT
echo "checkout: `$(git rev-parse --short=12 HEAD)"
git status --porcelain | head -20
"@
# One argument, not stdin: PowerShell ends every piped line with CRLF, and
# bash then reads "head -20" plus a carriage return.
& $Ssh $JetsonHost ($remote -replace "`r", "")
if ($LASTEXITCODE -ne 0) { throw "checkout on the Jetson failed" }
foreach ($e in $extras) {
    & $Scp $e "${JetsonHost}:$Dest/$e"
    if ($LASTEXITCODE -ne 0) { throw "scp $e failed" }
}
Remove-Item $tmp
Write-Host "==> done: $Dest checked out at $commit, plus $($extras.Count) gitignored files"
Write-Host "    (any path listed above under checkout is untracked or modified there: a run then records -dirty)"
