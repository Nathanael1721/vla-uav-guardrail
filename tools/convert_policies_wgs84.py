"""Rewrite the shipped policies from local metres to WGS84, and prove each one
enforces the same geometry as before.

    python tools/convert_policies_wgs84.py --check      # prove, write nothing
    python tools/convert_policies_wgs84.py              # convert policies/*.yaml
    python tools/convert_policies_wgs84.py --report docs/data/wgs84_conversion.json

WHY THIS EXISTS (WP1-14)

The grant (Policy DSL page, "Locked design choices"): "WGS84 latitude /
longitude is canonical ... the DSL itself never carries projected
coordinates." Until 2026-11-03, 31 of the 32 policies in policies/ were
written, stored and hashed in local metres (x North, y East of a frame origin
nobody stated); the one WGS84 policy (wgs84_taipei.yaml) has never flown. This
tool converts the 31: each gains `origin: {lat, lon}` - the WGS84 position its
rail puts at the local frame's (0, 0) - and every point is rewritten as
{lat, lon} about that origin. Nothing else in the file changes but `version:`
(the patch number is raised: the content changed, and an `id@version` names
one content forever - policies/policy.lock.json).

WHERE THE ORIGIN COMES FROM (ANCHORS)

The simulated worlds have no surveyed position; each rail states one, and
that is the anchor:

  * ArduPilot SITL rails (ROS 2 + MAVROS 2, pymavlink, Gazebo): SITL's
    --home, -35.363261, 149.165230 (CMAC; sitl/start_sitl.sh). The MAVLink
    adapter refuses to upload the zones of a policy whose origin is more
    than 5 m from the autopilot's home (sitl/mavlink_adapter_node.py,
    ORIGIN_TOLERANCE_M), so the three policies flown there are anchored at
    it: fase3-sim-demo, sitl-pedestrian-standoff, poly-test.
  * Every other policy flies the AirSim worlds (CityLife on Project AirSim
    and on classic AirSim, Blocks, AirSimNH) or the headless sweep: the
    Project AirSim scenes' home-geo-point, 35.6895, 139.6917
    (demo/pas_config/*.jsonc, demo/pas_ardupilot/scene_citylife_ardupilot.jsonc),
    which the PAS rail puts at the frame origin (demo/pas_ardupilot/
    autopilot.py FrameAnchor: the policy's origin first, else the scene's
    home). Classic AirSim and the headless sweep work in metres and read no
    origin; for them the anchor only says where the frame is.

WHAT IS PROVED, PER POLICY (`--check` does all of it and writes nothing)

  1. Round trip of every point: lat/lon are written from the metres with
     projection.LocalProjection.to_latlon about the origin (12 decimal
     places), and projected back with to_local. The unrounded error is
     reported (max over every point); it must be under TOL_M (1 um). The
     policy's local view (projection.local_metres, rounded to 1 um) must give
     back the original metres EXACTLY, float for float.
  2. The same IR: the converted file's metre form (models.Policy.metre_form:
     the rules in metres about the origin, no origin) at the OLD version has
     the old file's canonical bytes - so the same policy_hash, IR 2.x hashes
     and legacy 16-hex hashes. This is also how the lock reads the old
     version back (guardrail/bundle.py, rewind / FRAME_FORM).
  3. The same enforcement inputs: every zone, corridor and circle is the same
     metres, float for float, in the Shield's view (point.x / point.y), the
     same start conflicts, flight problems and runtime notes.

The sweep's P0 numbers are replayed separately (experiments/sweep_scenarios.py
before and after; the CHANGELOG entry of 2026-11-03 (F) has the figures).
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.bundle import as_version                          # noqa: E402
from guardrail.models import (Policy, _rule_points, load_policy,  # noqa: E402
                              parse_policy_text)
from guardrail.projection import LocalProjection, local_metres   # noqa: E402

TOL_M = 1e-6                      # the stated round-trip tolerance (1 um)
LATLON_DECIMALS = 12              # ~0.1 um of latitude

CMAC = (-35.363261, 149.165230,
        "ArduPilot SITL --home (CMAC, sitl/start_sitl.sh)",
        "the ArduPilot SITL rails' home; the MAVLink adapter refuses zones whose "
        "origin is more than 5 m from it (sitl/mavlink_adapter_node.py)")
PAS_HOME = (35.6895, 139.6917,
            "the Project AirSim scenes' home-geo-point",
            "demo/pas_config/*.jsonc, demo/pas_ardupilot/scene_citylife_ardupilot."
            "jsonc; the PAS rail puts it at the frame origin, classic AirSim and the "
            "headless sweep read no origin")
# The policies flown on the ArduPilot SITL rails (stored runs: ros2_*, sitl_*,
# gazebo_*); every other one is anchored at PAS_HOME.
SITL_FILES = frozenset({"sim_demo_policy.yaml", "sitl_pedestrian.yaml",
                        "poly_test.yaml"})


def anchor_for(name: str) -> tuple[float, float, str]:
    return CMAC if name in SITL_FILES else PAS_HOME


def _num(s: str) -> str:
    """A coordinate as YAML text: 12 decimal places, trailing zeros dropped."""
    t = f"{float(s):.{LATLON_DECIMALS}f}".rstrip("0")
    return t + "0" if t.endswith(".") else t


_FLOW = re.compile(r"\{\s*x:\s*(-?\d+(?:\.\d+)?)\s*,\s*y:\s*(-?\d+(?:\.\d+)?)\s*\}")
_BLOCK = re.compile(r"^(\s*-\s+)x:\s*(-?\d+(?:\.\d+)?)\s*\n(\s*)y:\s*(-?\d+(?:\.\d+)?)\s*$",
                    re.M)
_VERSION = re.compile(r"^version:[ \t]*(\d+)\.(\d+)\.(\d+)[ \t]*$", re.M)


def _split_comment(line: str) -> tuple[str, str]:
    """(code, comment) of one YAML line; '#' inside a quoted string is code."""
    q = None
    for i, ch in enumerate(line):
        if ch in "'\"":
            q = None if q == ch else (ch if q is None else q)
        elif ch == "#" and q is None and (i == 0 or line[i - 1] in " \t"):
            return line[:i], line[i:]
    return line, ""


def convert_text(text: str, origin: tuple, bump: bool = True,
                 note: str | None = None) -> tuple[str, list]:
    """The file's text with every point in WGS84 and `origin` stated, and the
    list of (x, y, lat, lon) it wrote. Comments are left as written (they
    describe the local frame, which is the policy's view). `bump` raises the
    patch version (a converted file: new content); `note` replaces the
    provenance comment's first two lines (demo/gen_random_scenario.py writes
    a new file in WGS84 this way, at the version it is given)."""
    proj = LocalProjection(origin[0], origin[1])
    pts: list = []

    def ll(x: str, y: str) -> tuple[str, str]:
        lat, lon = proj.to_latlon(float(x), float(y))
        a, b = _num(lat), _num(lon)
        pts.append((float(x), float(y), float(a), float(b)))
        return a, b

    def flow(m):
        a, b = ll(m.group(1), m.group(2))
        return f"{{lat: {a}, lon: {b}}}"

    out_lines = []
    for line in text.split("\n"):
        code, comment = _split_comment(line)
        if code.strip():
            code = _FLOW.sub(flow, code)
        out_lines.append(code + comment)
    text = "\n".join(out_lines)

    def block(m):
        a, b = ll(m.group(2), m.group(4))
        return f"{m.group(1)}lat: {a}\n{m.group(3)}lon: {b}"
    text = _BLOCK.sub(block, text)

    m = _VERSION.search(text)
    if m is None:
        raise ValueError("no top-level `version:` line")
    old_ver = f"{m.group(1)}.{m.group(2)}.{m.group(3)}"
    new_ver = f"{m.group(1)}.{m.group(2)}.{int(m.group(3)) + int(bump)}"
    head = note or (f"WGS84 since 2026-11-03 (tools/convert_policies_wgs84.py, from the\n"
                    f"# metres of {old_ver})")
    block_txt = (f"version: {new_ver}\n"
                 f"# {head}. Frame origin: {origin[2]}.\n"
                 f"# Points are lat/lon; the x North / y East metres about this origin\n"
                 f"# that comments here use are the view the Shield computes from them.\n"
                 f"origin:\n  lat: {_num(origin[0])}\n  lon: {_num(origin[1])}")
    text = text[:m.start()] + block_txt + text[m.end():]
    return text, pts


def _geometry(pol: Policy) -> list:
    """Every zone, tube and circle of `pol` as the Shield reads it."""
    out = []
    for c in pol.constraints:
        for name in ("vertices", "centerline"):
            pts = getattr(c, name, None)
            if pts:
                out.append((c.id, name, [(v.x, v.y) for v in pts]))
        if getattr(c, "center", None) is not None:
            out.append((c.id, "center", [(c.center.x, c.center.y)]))
    return out


def convert_one(path: Path) -> dict:
    """Convert one file in memory and prove it; returns the record (with the
    new text under `_text`). Raises if any proof fails."""
    with open(path, encoding="utf-8", newline="") as fh:
        raw_text = fh.read()
    # The repository mixes LF and CRLF; the file keeps its own.
    eol = "\r\n" if "\r\n" in raw_text else "\n"
    text = raw_text.replace("\r\n", "\n")
    old = load_policy(path, runtime=False)
    rec = {"file": path.name, "old": f"{old.policy_id}@{old.version}",
           "old_policy_hash": old.policy_hash}
    if old.origin is not None or old.is_geographic:
        return {**rec, "skipped": "already WGS84 with a stated origin"}
    origin = anchor_for(path.name)
    new_text, pts = convert_text(text, origin)
    new = Policy.model_validate(parse_policy_text(new_text, path.name))
    bad = new.lint()
    if bad:
        raise ValueError(f"{path.name}: the converted policy fails the lint: {bad}")
    if (new.origin.lat, new.origin.lon) != (float(_num(origin[0])), float(_num(origin[1]))):
        raise ValueError(f"{path.name}: origin not written as stated")
    n_metre = sum(1 for pts_ in _rule_points(new, all_points=True)
                  for v in pts_ if not v.geographic)
    if n_metre:
        raise ValueError(f"{path.name}: {n_metre} point(s) still in metres")

    # 1. the round trip of every point
    proj = LocalProjection(new.origin.lat, new.origin.lon)
    worst = 0.0
    for x, y, lat, lon in pts:
        n, e = proj.to_local(lat, lon)
        worst = max(worst, abs(n - x), abs(e - y))
        if local_metres((new.origin.lat, new.origin.lon), lat, lon) != (x, y):
            raise ValueError(f"{path.name}: ({x}, {y}) -> ({lat}, {lon}) does not give "
                             f"back its metres exactly in the policy's view")
    if worst > TOL_M:
        raise ValueError(f"{path.name}: round-trip error {worst:.3g} m > {TOL_M} m")

    # 2. the same IR, read back through the metre form at the old version
    back = as_version(new.metre_form(), old.version)
    if back.canonical_bytes() != old.canonical_bytes():
        raise ValueError(f"{path.name}: the metre form is not the old IR")
    if back.policy_hash != old.policy_hash or back.ir2_hashes() != old.ir2_hashes() \
            or back.legacy_hashes() != old.legacy_hashes():
        raise ValueError(f"{path.name}: the metre form does not reproduce every old hash")

    # 3. the same enforcement inputs
    if _geometry(new) != _geometry(old):
        raise ValueError(f"{path.name}: the Shield's geometry moved")
    for what in ("start_conflicts", "runtime_notes", "unenforced_rules",
                 "unmeasured_rules"):
        if getattr(new, what)() != getattr(old, what)():
            raise ValueError(f"{path.name}: {what} changed")
    if new.frame_problem() is not None:
        raise ValueError(f"{path.name}: {new.frame_problem()}")

    return {**rec, "new": f"{new.policy_id}@{new.version}",
            "new_policy_hash": new.policy_hash,
            "origin": {"lat": new.origin.lat, "lon": new.origin.lon,
                       "source": origin[2], "why": origin[3]},
            "points": len(pts),
            "max_roundtrip_error_m": worst,
            "tolerance_m": TOL_M,
            "local_view_exact": True,
            "metre_form_is_old_ir": True,
            "_text": new_text.replace("\n", eol)}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--policies", default=str(ROOT / "policies"))
    ap.add_argument("--check", action="store_true",
                    help="prove the conversion of every file, write nothing")
    ap.add_argument("--report", default=None,
                    help="write the per-policy record (JSON) here")
    args = ap.parse_args(argv)
    recs = []
    for f in sorted(Path(args.policies).glob("*.yaml")):
        recs.append(convert_one(f))
    done = [r for r in recs if "skipped" not in r]
    for r in recs:
        if "skipped" in r:
            print(f"  skip  {r['file']:32s} {r['old']}: {r['skipped']}")
        else:
            print(f"  {'check' if args.check else 'write'} {r['file']:32s} {r['old']} -> "
                  f"{r['new']}  {r['points']:2d} pts  max round trip "
                  f"{r['max_roundtrip_error_m']:.2e} m  origin "
                  f"({r['origin']['lat']}, {r['origin']['lon']})")
    if not args.check:
        for r in done:
            p = Path(args.policies) / r["file"]
            with open(p, "w", encoding="utf-8", newline="") as fh:
                fh.write(r["_text"])
    if args.report:
        rows = [{k: v for k, v in r.items() if k != "_text"} for r in recs]
        doc = {"tool": "tools/convert_policies_wgs84.py",
               "tolerance_m": TOL_M, "latlon_decimals": LATLON_DECIMALS,
               "converted": len(done), "skipped": len(recs) - len(done),
               "max_roundtrip_error_m": max((r["max_roundtrip_error_m"] for r in done),
                                            default=0.0),
               "policies": rows}
        Path(args.report).write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n",
                                     encoding="utf-8", newline="\n")
    worst = max((r["max_roundtrip_error_m"] for r in done), default=0.0)
    print(f"{len(done)} converted, {len(recs) - len(done)} skipped; every proof held; "
          f"max round-trip error {worst:.3g} m (tolerance {TOL_M} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
