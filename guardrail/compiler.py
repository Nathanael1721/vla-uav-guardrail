"""
Constraint Compiler (mini) — the "before the VLA" half of the Guardrail.

Meeting architecture slot:

    User Command -> [Constraint Compiler] -> YAML Prompt -> VLA -> ...

Takes a natural-language command, resolves it into a structured Mission, and
renders a YAML prompt that bundles the mission WITH a summary of the active
policy constraints (the grant calls this the Constraint Summary Pack — CSP).

v0 parsing is deliberately simple: a named-place registry + coordinate regex.
A real deployment would use map data / an LLM here; the *interface* (text in,
validated Mission + prompt out) is what matters, and that stays stable.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import yaml
from pydantic import BaseModel

from .models import (AltitudeEnvelope, Corridor, KinematicEnvelope,
                     ObstacleClearance, Policy, PolygonFence, SubjectStandoff)


def _count_types(policy: Policy) -> dict:
    """How many rules of each kind, so a reader can see at a glance whether a
    class they expected is missing entirely."""
    out: dict = {}
    for c in policy.constraints:
        out[c.type] = out.get(c.type, 0) + 1
    return out


# Named places the operator may refer to (stand-in for a map service).
PLACES = {
    "northeast pad": (30.0, 30.0),
    "north pad": (35.0, 0.0),
    "east pad": (0.0, 35.0),
    "home": (0.0, 0.0),
}


class Mission(BaseModel):
    task_text: str                 # the operator's original words
    target_x: float
    target_y: float
    cruise_alt_m: float
    speed_pref_mps: float          # what the operator ASKED for (may be illegal!)


class ConstraintCompiler:
    def __init__(self, policy: Policy):
        self.policy = policy

    # ---------------- command -> Mission ---------------- #

    def parse_command(self, text: str, default_alt: float = 15.0,
                      default_speed: float = 6.0) -> Mission:
        """Resolve ambiguous human text into an exact, structured mission."""
        low = text.lower()

        target = None
        for name, xy in PLACES.items():
            if name in low:
                target = xy
                break
        m = re.search(r"\(?\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)?", low)
        if target is None and m:
            target = (float(m.group(1)), float(m.group(2)))
        if target is None:
            raise ValueError(f"cannot resolve a target from: {text!r} "
                             f"(known places: {list(PLACES)})")

        # Parse speed FIRST and strip it, so "6 m/s" can never be mistaken
        # for an altitude (bit us once: "... 6 m/s altitude 20" parsed alt=6).
        speed = default_speed
        m = re.search(r"(\d+(?:\.\d+)?)\s*m/s", low)
        if m:
            speed = float(m.group(1))
            low = low.replace(m.group(0), " ")

        alt = default_alt
        m = re.search(r"(?:alt|altitude|height|tinggi)\D{0,3}(\d+(?:\.\d+)?)|"
                      r"(\d+(?:\.\d+)?)\s*m\s+(?:alt|altitude|height)", low)
        if m:
            alt = float(m.group(1) or m.group(2))

        return Mission(task_text=text, target_x=target[0], target_y=target[1],
                       cruise_alt_m=alt, speed_pref_mps=speed)

    # ---------------- Mission + policy -> YAML prompt ---------------- #

    def summary_pack(self) -> dict:
        """The Constraint Summary Pack — WP2's named deliverable.

        Every rule in force, in one structure, with the hash that identifies the
        policy it came from. Named once in the grant's deliverable list and never
        produced until now; `build_prompt` renders from it, so the prompt the VLA
        reads and the pack a reviewer reads cannot disagree.

        EVERY rule type appears here, which is the part that was wrong before.
        `build_prompt` used to emit only fences, the altitude envelope and the
        speed cap. `obstacle_clearance` and `subject_standoff` were silently
        absent — so the pilot was never told about the 10 m pedestrian stand-off
        that the Shield enforces against it. A constraint the planner cannot see
        is one it can only discover by being repaired, which is exactly the
        "tell the model the rules up front" argument the prefix compiler exists
        to make.

        Nothing is summarised away. A rule that cannot be rendered as a sentence
        still appears in `rules` with its full parameters; the natural-language
        half is a convenience for the model, not the record.
        """
        rules = []
        for c in self.policy.constraints:
            d = c.model_dump(mode="json")
            rules.append(d)
        return {
            "policy_id": self.policy.policy_id,
            "version": self.policy.version,
            "generation": self.policy.generation,
            "policy_hash": self.policy.policy_hash,
            "origin": (self.policy.origin.model_dump()
                       if self.policy.origin else None),
            "n_rules": len(rules),
            "rules_by_type": _count_types(self.policy),
            "rules": rules,
        }

    def _sentences(self) -> list[str]:
        """One plain sentence per rule, for the model rather than the reviewer."""
        nl = []
        for f in self.policy.by_type(PolygonFence):
            nl.append(f"Never enter zone '{f.id}'.")
        for c in self.policy.by_type(Corridor):
            nl.append(f"Stay inside corridor '{c.id}', within "
                      f"{c.half_width_m:g} m of its centerline and between "
                      f"{c.altitude_floor_m:g} m and {c.altitude_ceiling_m:g} m.")
        for e in self.policy.by_type(AltitudeEnvelope):
            nl.append(f"Stay between {e.alt_min_m:g} m and {e.alt_max_m:g} m altitude.")
        for k in self.policy.by_type(KinematicEnvelope):
            nl.append(f"Keep speed at or below {k.speed_max_mps:g} m/s, climb rate "
                      f"below {k.climb_rate_max_mps:g} m/s and turn rate below "
                      f"{k.yaw_rate_max_dps:g} deg/s.")
        for o in self.policy.by_type(ObstacleClearance):
            nl.append(f"Keep at least {o.min_clearance_m:g} m from any building.")
        for s in self.policy.by_type(SubjectStandoff):
            who = "anything you are following" if s.subject_class == "*" \
                else f"any {s.subject_class}"
            nl.append(f"Keep at least {s.min_range_m:g} m away from {who}.")
        return nl

    def build_prompt(self, mission: Mission) -> str:
        """Render the structured YAML prompt the VLA receives.
        Templated, never free-form — same rule as the grant's NL adapter."""
        pack = self.summary_pack()
        alts = self.policy.by_type(AltitudeEnvelope)
        kins = self.policy.by_type(KinematicEnvelope)

        doc = {
            "mission": {
                "task": mission.task_text,
                "target": {"x": mission.target_x, "y": mission.target_y},
                "cruise_alt_m": mission.cruise_alt_m,
            },
            "constraints": {
                "policy_id": pack["policy_id"],
                "policy_hash": pack["policy_hash"],
                # The shorthands the demos already read, kept so this stays a
                # drop-in replacement...
                "no_fly_zones": [
                    {"id": f.id, "vertices": [{"x": v.x, "y": v.y} for v in f.vertices]}
                    for f in self.policy.by_type(PolygonFence)],
                "altitude_band_m": (
                    [alts[0].alt_min_m, alts[0].alt_max_m] if alts else None),
                "speed_max_mps": kins[0].speed_max_mps if kins else None,
                # ...and the full pack underneath them, so no rule is invisible
                # merely because it has no shorthand.
                "summary_pack": pack,
            },
        }
        doc["natural_language_prompt"] = " ".join(self._sentences())
        return yaml.safe_dump(doc, sort_keys=False, allow_unicode=True)

    def write_summary_pack(self, path: str | Path) -> Path:
        """Emit the CSP as a file, so it can travel with a flight's artefacts."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.summary_pack(), indent=2) + "\n",
                     encoding="utf-8")
        return p
