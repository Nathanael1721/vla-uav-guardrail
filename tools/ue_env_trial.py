"""Trial: prove a level can be edited by SCRIPT, and that the edit survives a reload.

Runs INSIDE Unreal, not in the repo's Python:

    UnrealEditor-Cmd.exe <Blocks.uproject> -run=pythonscript -script="tools/ue_env_trial.py"

(resolve the engine with `python tools/ue_engine.py --cmd`, never a hard-coded
version — see that file for why.)

WHAT THIS IS FOR

The plan is to stop spawning scenery at run time and author it into the level
instead. Before placing anything, one question has to be answered: can the level
be changed by a script, deterministically, and does the change actually persist?

Doing it by cursor would answer neither. A hand-placed level cannot be
regenerated, cannot be diffed, and - since PASBlocks is 27 GB and gitignored -
cannot be recovered if it goes wrong. A script can be re-run, reviewed and
corrected.

WHY IT REOPENS THE LEVEL

Spawning an actor and reporting success proves almost nothing: the spawn call
returns a live object whether or not anything was written to disk. So the script
saves, RELOADS the level from disk, and looks the actor up again by tag. Only
that round trip proves the edit is real - and it is exactly the round trip a
future scenery script depends on.

SAFETY

`Demo_day` is the map the delivered demos fly. It is never opened for writing:
the script duplicates it through the asset API (never a file copy, which would
leave internal references pointing at the original) and edits the copy. The
caller checks Demo_day's hash before and after.
"""
import json
import traceback

import unreal

SRC_MAP = "/Game/JapaneseCity/Maps/Demo_day"
DST_MAP = "/Game/JapaneseCity/Maps/Demo_day_Env"
CUBE = "/Engine/BasicShapes/Cube"

# A tag is how the actor is found again after the reload. Labels are not unique
# and object names get mangled on load; a tag survives both.
TAG = "guardrail_env_trial"
WANT_LOCATION = unreal.Vector(1200.0, -450.0, 300.0)
WANT_ROTATION = unreal.Rotator(0.0, 0.0, 45.0)


def receipt_path() -> str:
    # project_dir() is .../PASBlocks/ ; the repo root is its parent.
    proj = unreal.Paths.project_dir()
    root = unreal.Paths.combine([proj, ".."])
    return unreal.Paths.combine([root, "docs", "data", "ue_env_trial.json"])


def log(msg: str) -> None:
    unreal.log(f"[env-trial] {msg}")


def load_level(path: str) -> bool:
    """Load a level, tolerating the API move between UE versions."""
    try:
        les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        return bool(les.load_level(path))
    except Exception:                                            # noqa: BLE001
        return bool(unreal.EditorLevelLibrary.load_level(path))


def save_level() -> bool:
    try:
        les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
        return bool(les.save_current_level())
    except Exception:                                            # noqa: BLE001
        return bool(unreal.EditorLevelLibrary.save_current_level())


def all_actors():
    try:
        eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        return list(eas.get_all_level_actors())
    except Exception:                                            # noqa: BLE001
        return list(unreal.EditorLevelLibrary.get_all_level_actors())


def spawn_cube():
    try:
        eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        return eas.spawn_actor_from_class(
            unreal.StaticMeshActor, WANT_LOCATION, WANT_ROTATION)
    except Exception:                                            # noqa: BLE001
        return unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, WANT_LOCATION, WANT_ROTATION)


def find_tagged():
    for a in all_actors():
        try:
            if TAG in [str(t) for t in a.tags]:
                return a
        except Exception:                                        # noqa: BLE001
            continue
    return None


def run() -> dict:
    out = {"_what": "Trial that a level edit can be made by script and survives "
                    "a reload. See tools/ue_env_trial.py.",
           "source_map": SRC_MAP, "target_map": DST_MAP,
           "engine": unreal.SystemLibrary.get_engine_version(),
           "steps": {}}

    # 1. duplicate, through the asset API
    if unreal.EditorAssetLibrary.does_asset_exist(DST_MAP):
        log(f"{DST_MAP} already exists - deleting so the trial starts clean")
        unreal.EditorAssetLibrary.delete_asset(DST_MAP)
    dup = unreal.EditorAssetLibrary.duplicate_asset(SRC_MAP, DST_MAP)
    out["steps"]["duplicated"] = bool(dup) and \
        unreal.EditorAssetLibrary.does_asset_exist(DST_MAP)
    log(f"duplicate -> {out['steps']['duplicated']}")
    if not out["steps"]["duplicated"]:
        out["ok"] = False
        out["error"] = "duplicate_asset failed"
        return out

    # 2. open the COPY and place one actor at a known transform
    out["steps"]["opened_copy"] = load_level(DST_MAP)
    actor = spawn_cube()
    if actor is None:
        out["ok"] = False
        out["error"] = "spawn returned None"
        return out
    mesh = unreal.EditorAssetLibrary.load_asset(CUBE)
    if mesh:
        actor.static_mesh_component.set_static_mesh(mesh)
    actor.set_actor_label("EnvTrial_Cube")
    actor.tags = [TAG]
    out["steps"]["spawned"] = True

    # 3. save
    out["steps"]["saved"] = save_level()
    log(f"saved -> {out['steps']['saved']}")

    # 4. THE POINT: reload from disk and look it up again
    load_level(SRC_MAP)                     # bounce away so the reload is real
    out["steps"]["reopened"] = load_level(DST_MAP)
    found = find_tagged()
    out["steps"]["found_after_reload"] = found is not None

    if found is not None:
        loc, rot = found.get_actor_location(), found.get_actor_rotation()
        out["actor"] = {
            "label": str(found.get_actor_label()),
            "class": str(type(found).__name__),
            "location": [round(loc.x, 3), round(loc.y, 3), round(loc.z, 3)],
            "rotation": [round(rot.roll, 3), round(rot.pitch, 3), round(rot.yaw, 3)],
        }
        out["wanted"] = {
            "location": [WANT_LOCATION.x, WANT_LOCATION.y, WANT_LOCATION.z],
            "rotation": [WANT_ROTATION.roll, WANT_ROTATION.pitch, WANT_ROTATION.yaw],
        }
        dl = max(abs(a - b) for a, b in
                 zip(out["actor"]["location"], out["wanted"]["location"]))
        dr = max(abs(a - b) for a, b in
                 zip(out["actor"]["rotation"], out["wanted"]["rotation"]))
        out["max_location_error_cm"] = round(dl, 4)
        out["max_rotation_error_deg"] = round(dr, 4)
        out["transform_matches"] = dl < 0.5 and dr < 0.5

    out["ok"] = bool(out["steps"].get("found_after_reload")
                     and out.get("transform_matches"))
    return out


def main() -> None:
    try:
        res = run()
    except Exception:                                            # noqa: BLE001
        res = {"ok": False, "error": traceback.format_exc()}
    p = receipt_path()
    try:
        with open(p, "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=2)
        log(f"receipt -> {p}")
    except Exception as e:                                       # noqa: BLE001
        log(f"could not write receipt to {p}: {e}")
    log("RESULT ok=" + str(res.get("ok")))


main()
