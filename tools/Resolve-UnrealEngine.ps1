# Resolve the Unreal editor for a project, from the project itself.
#
# PowerShell twin of tools/ue_engine.py. The logic is duplicated in two
# languages on purpose: the single source of truth is Blocks.uproject, not this
# file, and making four PowerShell launchers shell out to a conda Python just to
# read one JSON field would couple them to an environment they otherwise do not
# need.
#
# WHY IT EXISTS: eleven files here hard-coded UE_5.7. The PASBlocks project was
# migrated to 5.8 on 2026-09-03 and its modules rebuilt against 5.8's BuildId.
# Launching a 5.8-built project with the 5.7 editor prompts a rebuild against
# 5.7, which would destroy that build. Reading the version from the .uproject
# means the next migration needs no edits here at all.
#
# Usage, from a launcher in the repo root:
#     . "$PSScriptRoot\tools\Resolve-UnrealEngine.ps1"
#     $UE = Get-UnrealEditor -UProject $UPROJ
#
# Get-UnrealEditor THROWS on a BuildId mismatch rather than returning a path
# that would trigger a silent rebuild.

function Get-UnrealEngineRoot {
    param([Parameter(Mandatory)][string]$UProject)

    if (-not (Test-Path $UProject)) { throw "no such .uproject: $UProject" }
    $assoc = (Get-Content $UProject -Raw | ConvertFrom-Json).EngineAssociation
    if (-not $assoc) { throw "$UProject declares no EngineAssociation" }
    if ($assoc -notmatch '^\d+\.\d+$') {
        throw ("EngineAssociation '$assoc' is not a launcher version - this " +
               "looks like a source build. Point the script at its Engine/ " +
               "directly instead of using this resolver.")
    }
    $root = Join-Path "C:\Program Files\Epic Games" "UE_$assoc"
    if (-not (Test-Path $root)) {
        $have = (Get-ChildItem "C:\Program Files\Epic Games" -Directory -ErrorAction SilentlyContinue |
                 Where-Object Name -like 'UE_*' | Select-Object -ExpandProperty Name) -join ', '
        throw "project wants engine $assoc, not installed at $root. Installed: $have"
    }
    return $root
}

function Get-UnrealBuildId {
    param([Parameter(Mandatory)][string]$ModulesPath)
    if (-not (Test-Path $ModulesPath)) { return $null }
    try { return (Get-Content $ModulesPath -Raw | ConvertFrom-Json).BuildId }
    catch { return $null }
}

function Get-UnrealEditor {
    param(
        [Parameter(Mandatory)][string]$UProject,
        [switch]$Cmd,            # UnrealEditor-Cmd.exe instead of UnrealEditor.exe
        [switch]$SkipBuildCheck
    )

    $root = Get-UnrealEngineRoot -UProject $UProject
    $exe  = if ($Cmd) { 'UnrealEditor-Cmd.exe' } else { 'UnrealEditor.exe' }
    $path = Join-Path $root "Engine\Binaries\Win64\$exe"
    if (-not (Test-Path $path)) { throw "engine found but $exe is missing: $path" }

    if (-not $SkipBuildCheck) {
        $engineId  = Get-UnrealBuildId (Join-Path $root 'Engine\Binaries\Win64\UnrealEditor.modules')
        $projectId = Get-UnrealBuildId (Join-Path (Split-Path $UProject) 'Binaries\Win64\UnrealEditor.modules')
        # No project modules yet is fine - nothing has been built, so there is
        # nothing to invalidate. Only a DIFFERING id is dangerous.
        if ($projectId -and $engineId -and ($projectId -ne $engineId)) {
            throw ("BuildId mismatch: engine $engineId vs project $projectId. " +
                   "Launching would rebuild the project's modules and " +
                   "invalidate the existing build. Refusing.")
        }
    }
    return $path
}
