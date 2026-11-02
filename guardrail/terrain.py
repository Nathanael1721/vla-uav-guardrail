"""Ground elevation: what an MSL height is above the ground (X-11, WP1-07).

WHY THIS EXISTS

The grant's DSL lets a geometry carry `altitude_ref: MSL` (mean sea level) as
an opt-in beside the default AGL (above ground level). The Shield measures
height above ground (models.State.up), so until 2026-11-03 an MSL rule was
declarable and refused for flight: the Shield had no ground-elevation source
to turn one height into the other, and judging a 120-300 m MSL zone as metres
above ground over 100 m ground lets an aircraft at 50 m AGL fly through it
with no violation (CHANGELOG 2026-10-19).

This module is that source. The Shield takes one (Shield(..., ground=...))
and enforces an MSL band as the AGL band it is over the ground:

    floor_agl = floor_msl - elevation_msl(x, y)

WHAT IS IMPLEMENTED, AND WHAT IS NOT

Only a FLAT ground: one stated elevation over the whole mission, per rail.
That is exact where the ground is flat - every simulated world here is - and
the simulators state it themselves, as the altitude of their home point:

  * ArduPilot SITL rails: --home -35.363261,149.165230,584,0 (CMAC;
    sitl/start_sitl.sh, deploy/docker/sitl-entrypoint.sh): 584 m.
  * Project AirSim scenes: home-geo-point altitude 40.0 m (demo/pas_config/
    *.jsonc, demo/pas_ardupilot/scene_citylife_ardupilot.jsonc).

A real flight over real terrain needs a digital elevation model; a ground
whose elevation varies with position is refused by the Shield
(`FlatGround.flat` is the contract), not approximated. Nothing here reads a
DEM, and no rail in this repository has flown an MSL rule.

The ground is NOT part of the policy (it is the world's, not the rule's), so
it never enters policy_hash; a flight records it beside its results
(`FlatGround.record`).
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FlatGround:
    """The ground at one stated elevation, metres above mean sea level,
    everywhere. `source` says where the number comes from - it is recorded
    with every flight that uses it."""

    flat = True
    __slots__ = ("elevation_msl_m", "source")

    def __init__(self, elevation_msl_m: float, source: str):
        e = float(elevation_msl_m)
        if not math.isfinite(e):
            raise ValueError(f"ground elevation {elevation_msl_m!r} is not finite")
        if not source:
            raise ValueError("a ground elevation must say where it comes from (source)")
        self.elevation_msl_m = e
        self.source = str(source)

    def elevation_at(self, x: float | None = None, y: float | None = None) -> float:
        """The ground's elevation (m MSL) under (x, y) metres of the policy
        frame: the stated one, wherever."""
        return self.elevation_msl_m

    def record(self) -> dict:
        return {"kind": "flat", "elevation_msl_m": self.elevation_msl_m,
                "source": self.source}

    def __repr__(self) -> str:
        return f"FlatGround({self.elevation_msl_m:g} m MSL, {self.source!r})"


# The rails' stated ground: the simulators' own home altitude.
SITL_CMAC = FlatGround(584.0, "ArduPilot SITL --home altitude (CMAC, sitl/start_sitl.sh)")
PAS_SCENE = FlatGround(40.0, "Project AirSim scenes' home-geo-point altitude "
                             "(demo/pas_config/*.jsonc)")
RAIL_GROUND: dict[str, FlatGround] = {"ardupilot-sitl": SITL_CMAC,
                                      "projectairsim": PAS_SCENE}


def scene_ground(scene: dict | str | Path) -> FlatGround:
    """The flat ground of a Project AirSim scene: its home-geo-point
    altitude (a scene dict, or the path of a .jsonc scene file)."""
    src = "a Project AirSim scene"
    if not isinstance(scene, dict):
        p = Path(scene)
        src = str(p)
        text = re.sub(r"^\s*//.*$", "", p.read_text(encoding="utf-8"), flags=re.M)
        scene = json.loads(text)
    home = scene.get("home-geo-point") or {}
    if "altitude" not in home:
        raise ValueError(f"{src}: no home-geo-point altitude, so no ground elevation")
    return FlatGround(float(home["altitude"]), f"home-geo-point altitude of {src}")


# Rule fields that hold a height, per altitude-banded rule type.
_BANDS = {"altitude_envelope": ("alt_min_m", "alt_max_m")}
_GEOM_BAND = ("altitude_floor_m", "altitude_ceiling_m")


def is_msl(rule) -> bool:
    return getattr(rule, "altitude_ref", None) == "MSL"


def ground_problem(ground) -> str | None:
    """Why `ground` cannot convert an MSL height for the Shield, or None."""
    if ground is None:
        return ("the Shield measures height above ground and was given no "
                "ground-elevation source to convert mean-sea-level heights "
                "(Shield(ground=...), guardrail/terrain.py)")
    if not getattr(ground, "flat", False) or not callable(getattr(ground, "elevation_at", None)):
        return (f"{ground!r} is not a flat ground: only a stated elevation per rail "
                f"(terrain.FlatGround) is implemented; a ground that varies with "
                f"position is refused, not approximated")
    return None


def agl_rule(rule, ground):
    """`rule` with its MSL band as the AGL band over `ground` (a copy; the
    rule itself is unchanged), or `rule` when it is AGL. Same id, same
    everything else: the Shield enforces the copy under the declared rule's
    name."""
    if not is_msl(rule):
        return rule
    why = ground_problem(ground)
    if why:
        raise ValueError(f"{rule.id}: altitude_ref MSL - {why}")
    e = ground.elevation_at()
    fields = _BANDS.get(rule.type, _GEOM_BAND)
    upd = {f: float(getattr(rule, f)) - e for f in fields}
    upd["altitude_ref"] = None
    return rule.model_copy(update=upd)
