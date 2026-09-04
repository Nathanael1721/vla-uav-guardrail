r"""Resolve the Unreal editor for a project, from the project itself.

    python tools/ue_engine.py --editor     -> ...\UE_5.8\...\UnrealEditor.exe
    python tools/ue_engine.py --cmd        -> ...\UnrealEditor-Cmd.exe
    python tools/ue_engine.py --check      -> verify the build matches

WHY THIS EXISTS

Eleven files in this repository hard-coded
`C:\\Program Files\\Epic Games\\UE_5.7\\...`. On 2026-09-03 the PASBlocks project
was migrated to UE 5.8 - `Blocks.uproject` now declares
`"EngineAssociation": "5.8"`, and its modules were rebuilt against 5.8's BuildId
`55116800`. Every one of those eleven files was left pointing at 5.7.

That is not a cosmetic mismatch. Launching a 5.8-built project with the 5.7
editor prompts a rebuild against 5.7, which would undo the 5.8 build the whole
environment now depends on - eleven scripts, any one of which could do it, two
weeks before a demo.

Editing eleven paths to say 5.8 would fix today and recreate the trap for the
next migration. So the version is READ FROM THE PROJECT. `.uproject` is the only
place that knows which engine it belongs to, and it is the file that changes when
somebody migrates.

THE BUILD-ID CHECK IS THE POINT

Resolving the path is half the job. `check()` compares the BuildId in the
engine's `UnrealEditor.modules` against the one in the project's, because those
must match for the editor to load the project's modules without rebuilding. A
mismatch is exactly the silent-rebuild situation above, and it is far better to
stop with a message than to let an editor "helpfully" recompile.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UPROJECT = ROOT / "PASBlocks" / "Blocks.uproject"

# Where the Epic launcher installs engines on this machine. A source build is
# identified by a GUID association instead of a version string and is handled
# with an explicit error rather than a guess.
LAUNCHER_ROOT = Path(r"C:\Program Files\Epic Games")


def engine_association(uproject: Path) -> str:
    raw = json.loads(Path(uproject).read_text(encoding="utf-8-sig"))
    assoc = str(raw.get("EngineAssociation", "")).strip()
    if not assoc:
        raise SystemExit(f"{uproject} declares no EngineAssociation")
    return assoc


def engine_root(uproject: Path = DEFAULT_UPROJECT) -> Path:
    assoc = engine_association(uproject)
    if not re.fullmatch(r"\d+\.\d+", assoc):
        raise SystemExit(
            f"EngineAssociation {assoc!r} is not a launcher version. This looks "
            f"like a source build; point the scripts at its Engine/ directly "
            f"rather than through this resolver.")
    root = LAUNCHER_ROOT / f"UE_{assoc}"
    if not root.is_dir():
        installed = sorted(p.name for p in LAUNCHER_ROOT.glob("UE_*")) \
            if LAUNCHER_ROOT.is_dir() else []
        raise SystemExit(
            f"{uproject.name} wants engine {assoc}, which is not installed at "
            f"{root}. Installed: {', '.join(installed) or 'none found'}")
    return root


def editor_exe(uproject: Path = DEFAULT_UPROJECT) -> Path:
    return engine_root(uproject) / "Engine/Binaries/Win64/UnrealEditor.exe"


def cmd_exe(uproject: Path = DEFAULT_UPROJECT) -> Path:
    return engine_root(uproject) / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"


def _build_id(modules: Path) -> str | None:
    if not modules.is_file():
        return None
    try:
        return str(json.loads(modules.read_text(encoding="utf-8")).get("BuildId"))
    except Exception:                                            # noqa: BLE001
        return None


def check(uproject: Path = DEFAULT_UPROJECT) -> dict:
    """Would this engine load this project's modules without rebuilding them?"""
    root = engine_root(uproject)
    engine_id = _build_id(root / "Engine/Binaries/Win64/UnrealEditor.modules")
    proj_mod = Path(uproject).parent / "Binaries/Win64/UnrealEditor.modules"
    project_id = _build_id(proj_mod)
    return {
        "uproject": str(uproject),
        "engine_association": engine_association(uproject),
        "engine_root": str(root),
        "engine_build_id": engine_id,
        "project_build_id": project_id,
        # No project modules yet is NOT a mismatch - nothing has been built, so
        # there is nothing to invalidate. Only a differing id is a problem.
        "match": project_id is None or engine_id == project_id,
        "project_built": project_id is not None,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uproject", default=str(DEFAULT_UPROJECT))
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--editor", action="store_true")
    g.add_argument("--cmd", action="store_true")
    g.add_argument("--root", action="store_true")
    g.add_argument("--check", action="store_true")
    a = ap.parse_args()
    up = Path(a.uproject)

    if a.editor:
        print(editor_exe(up))
    elif a.cmd:
        print(cmd_exe(up))
    elif a.root:
        print(engine_root(up))
    else:
        info = check(up)
        for k, v in info.items():
            print(f"  {k:<20} {v}")
        if not info["match"]:
            print("\nMISMATCH: launching this engine would rebuild the project's "
                  "modules and invalidate the existing build. Refusing to "
                  "recommend it.")
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
