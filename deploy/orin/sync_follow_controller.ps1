# Copy what the Jetson needs to run the hand-written follow
# (deploy/orin/run_follow_controller.sh) from this desktop over SSH:
#   * the repository at HEAD (git archive: tracked files only, so the
#     gitignored private signing key policies/keys/*.pem never leaves),
#   * the signed policy bundles the follow flies (bundles/follow-car-citylife*.tar.gz,
#     gitignored, built by tools/build_flight_bundles.py),
#   * the CityLife obstacle map (demo/out/citymap_citylife/occ_day.npz, gitignored).
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
$extras += "demo/out/citymap_citylife/occ_day.npz"
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
