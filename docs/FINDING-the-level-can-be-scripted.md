# The level can be edited by script, and the edit survives a reload

**Date:** 2026-09-04
**Status:** proven end to end. Receipt: `docs/data/ue_env_trial.json`.

## What was in doubt

`docs/DESIGN-unreal-native-environment.md` argued that authoring the environment
in Unreal would buy visual fidelity and cost reproducibility: a hand-authored
level is a 27 GB binary that cannot be diffed, cannot go in the repository, and
cannot be regenerated. That objection was the main reason for deferring the work.

It rested on an assumption that was never tested — that authoring means *hand*
authoring. If a script can do it, the objection mostly dissolves: a script goes
in the repository, is reviewable, and can be re-run.

## What was run

`tools/ue_env_trial.py`, executed inside the editor:

```
UnrealEditor-Cmd.exe PASBlocks/Blocks.uproject -run=pythonscript -script=tools/ue_env_trial.py
```

Duplicate `Demo_day` → `Demo_day_Env` through the asset API, open the copy,
spawn a `StaticMeshActor` at a chosen transform, tag it, save — then **bounce to
a different level and reload the copy from disk**, and look the actor up again by
tag.

| Step | Result |
|---|---|
| duplicated | yes |
| opened the copy | yes |
| spawned | yes |
| saved | yes |
| reopened from disk | yes |
| **found after reload** | **yes** |
| location error | **0.0 cm** |
| rotation error | **0.0°** |

Engine `5.8.1-56057345`. 37 seconds, unattended, no GUI, no cursor.

## Why the reload is the whole test

A spawn call returns a live object whether or not anything reached disk. A script
that spawned and reported success would prove nothing about persistence. Bouncing
to another level and re-opening the copy is what makes the result mean *the file
on disk contains the actor* — which is precisely what a scenery-placement script
needs to be able to claim.

Looking the actor up by **tag** rather than by name or label matters for the same
reason: object names get mangled on load and labels are not unique, so either
could produce a false positive.

## Safety, and what it cost

`Demo_day.umap` was hashed before and after: **byte-identical**
(`efe015f9eb6b4e17…`). The trial only ever wrote to the duplicate. Given
PASBlocks is 27 GB and gitignored, that check is not ceremony — there is no
`git checkout` to recover with.

Duplication went through `EditorAssetLibrary.duplicate_asset`, never a file copy.
A copied `.umap` keeps internal references pointing at the original, which is the
kind of corruption that only shows up later.

## What this changes

The reproducibility objection in the design note no longer applies to the parts a
script can place. The environment can be authored the way everything else here is
authored — as code that is reviewed, versioned and re-runnable — with the level
as its build output rather than its source.

What still cannot be scripted usefully is judgement: whether a placement *looks*
right, whether lighting reads well. That remains a human-in-the-editor task, and
it is a much smaller one than placing everything by hand.

## Unreal MCP, separately

The MCP server is live in the editor and speaks the protocol. A JSON-RPC
`initialize` against `http://127.0.0.1:8000/mcp` returns protocol version
`2024-11-05` with a `tools` capability. Setup was already complete before this
session: `ModelContextProtocol` and `AllToolsets` are enabled in
`Blocks.uproject`, `bAutoStartServer=True` is set, and `.mcp.json` exists.

It could not be *used* as a tool this session, for a reason that has nothing to
do with Unreal: Claude Code reads `.mcp.json` at startup, so a server added
mid-session is not callable until the session restarts. `.mcp.json` now also
exists at the repository root, where this session's client looks for it.

For the work at hand this is not a limitation. The Python commandlet path is
headless, scriptable and verifiable, and produced the result above without any
live connection. MCP is the better interface for *interactive* work — asking the
editor questions, iterating on a placement — and is worth having for that.
