"""
Generate a RANDOM waypoint mission with a no-fly-zone placed on EACH leg, so the
global planner must detour around every NFZ (and the buildings). Writes a policy
YAML + prints the --route string for aerialvla_pas_demo.py.

Run:  python demo/gen_random_scenario.py --seed 7 --n 4

Each output file gets its own `policy_id` (WP1-22, 2026-10-21): the default
`policies/random_scenario.yaml` keeps `random-scenario`, any other file is
`random-scenario-<file stem>` (`hard_3.yaml` -> `random-scenario-hard-3`), and
`--policy-id` overrides it. The generator refuses an id another policy in the
output folder already claims. Until then every file it wrote said
`policy_id: random-scenario`, so nine files with nine contents shared
`random-scenario@0.1.0` and that name identified no single policy
(policies/policy.lock.json records the old collision; docs/DESIGN-policy-identity.md).
"""
from __future__ import annotations

import argparse
import math
import random
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

POLICIES = ROOT / "policies"
DEFAULT_OUT = POLICIES / "random_scenario.yaml"
BASE_ID = "random-scenario"


def default_policy_id(out: str | Path) -> str:
    """The policy_id a generated file gets unless --policy-id says otherwise:
    the default output keeps the historical `random-scenario`; any other file
    is `random-scenario-<stem>`, lower case, '_' and spaces as '-'.

    NOT unique by construction: `hard_3.yaml` and `hard-3.yaml` (or
    `Hard 3.yaml`) both map to `random-scenario-hard-3`. The ids are unique
    because main() refuses an id another file already claims
    (id_claimed_elsewhere), not because file names are; this docstring said
    the opposite until the review of 3660383 (P7)."""
    stem = re.sub(r"[_\s]+", "-", Path(out).stem.strip().lower())
    return BASE_ID if stem == BASE_ID else f"{BASE_ID}-{stem}"


def _claimed_id(path: Path) -> str | None:
    """The policy_id a policy file claims, as the loader reads it; a file the
    loader refuses is read by its top-level `policy_id:` line."""
    try:
        from guardrail.models import load_policy
        return load_policy(path, runtime=False).policy_id
    except Exception:                                     # noqa: BLE001
        m = re.search(r"^policy_id:\s*['\"]?([^'\"\s#]+)", path.read_text(encoding="utf-8"),
                      re.MULTILINE)
        return m.group(1) if m else None


def id_claimed_elsewhere(policy_id: str, out: str | Path) -> list[str]:
    """The other policy files that already claim `policy_id`: in `out`'s
    folder (by name), and in the repository's policies/ when `out` is
    elsewhere (as `policies/<name>`). Regenerating the same file is not a
    clash.

    Why policies/ too (review of 3660383, P7): with only the output folder
    scanned, `--out scratch/hard_3.yaml` minted `random-scenario-hard-3` -
    the id policies/hard_3.yaml already carries - for different content, and
    nothing said so until the file reached policies/ and the lock check found
    two contents under one id@version."""
    out = Path(out).resolve()
    folders = [out.parent]
    if POLICIES.resolve() != out.parent:
        folders.append(POLICIES.resolve())
    hits = []
    for folder in folders:
        for p in folder.glob("*.yaml"):
            if p.resolve() != out and _claimed_id(p) == policy_id:
                hits.append(p.name if folder == out.parent else f"policies/{p.name}")
    return sorted(hits)


def policy_yaml(policy_id: str, nfzs, band_lo: float, band_hi: float,
                version: str = "0.1.0") -> str:
    """The generated policy's text: one 12 m keep-out square per leg, the
    altitude band and the kinematic caps. The breach action is the grant's
    `project_fix` (IR schema 3.0.0; it was this repository's `repair` until
    2026-11-02, when the shipped files moved to 0.1.1 under the same rules).
    Every rule states `scope: mission` (IR schema 4.0.0 requires it; the
    shipped files moved to 0.1.3 on 2026-11-06): each zone, the band and
    the caps are this mission's, written for its legs, and none is a
    regulation's or a site's."""
    lines = [f"policy_id: {policy_id}", f"version: {version}", "constraints:"]
    for i, (xmn, xmx, ymn, ymx) in enumerate(nfzs):
        lines += [
            f"  - id: nfz-leg-{i+1}", "    type: polygon_fence",
            "    constraint_type: hard", "    scope: mission", "    priority: P0",
            "    violation_action: project_fix", "    vertices:",
            f"      - {{x: {xmn:.1f}, y: {ymn:.1f}}}",
            f"      - {{x: {xmx:.1f}, y: {ymn:.1f}}}",
            f"      - {{x: {xmx:.1f}, y: {ymx:.1f}}}",
            f"      - {{x: {xmn:.1f}, y: {ymx:.1f}}}",
            "    altitude_floor_m: 0", "    altitude_ceiling_m: 60",
            "    margin_m: 1.0",
        ]
    lines += [
        "  - id: alt-band", "    type: altitude_envelope",
        "    constraint_type: hard", "    scope: mission", "    priority: P0",
        "    violation_action: project_fix",
        f"    alt_min_m: {band_lo:.0f}", f"    alt_max_m: {band_hi:.0f}",
        "  - id: kin-caps", "    type: kinematic_envelope",
        "    constraint_type: hard", "    scope: mission", "    priority: P1",
        "    violation_action: project_fix",
        "    speed_max_mps: 4.0", "    climb_rate_max_mps: 2.0",
        "    yaw_rate_max_dps: 45.0",
    ]
    return "\n".join(lines) + "\n"


def wgs84_policy_yaml(policy_id: str, nfzs, band_lo: float, band_hi: float,
                      version: str = "0.1.0") -> str:
    """policy_yaml in the grant's canonical frame (WP1-14, 2026-11-03): the
    same rules with `origin` stated - the Project AirSim scenes' home-geo-
    point, where every CityLife policy is anchored - and every point in
    WGS84, written by tools/convert_policies_wgs84.py's own conversion, so a
    generated file and a converted one say the same thing the same way."""
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from convert_policies_wgs84 import PAS_HOME, convert_text
    finally:
        sys.path.remove(str(ROOT / "tools"))
    text, _ = convert_text(policy_yaml(policy_id, nfzs, band_lo, band_hi, version),
                           PAS_HOME, bump=False,
                           note="Generated by demo/gen_random_scenario.py in WGS84")
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--n", type=int, default=4, help="number of waypoints")
    ap.add_argument("--citymap", default=str(ROOT / "demo/out/citymap/occ_day.npz"))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--policy-id", default=None,
                    help="the policy's id (default: random-scenario for the default "
                         "--out, random-scenario-<file stem> for any other file)")
    ap.add_argument("--band-lo", type=float, default=35.0)
    ap.add_argument("--band-hi", type=float, default=55.0)
    ap.add_argument("--nfz", type=float, default=12.0, help="NFZ side length (m)")
    ap.add_argument("--metres", action="store_true",
                    help="DEPRECATED (WP1-14): write the legacy local-metre form "
                         "(no origin) instead of WGS84; the grant's DSL never "
                         "carries projected coordinates")
    args = ap.parse_args()
    policy_id = args.policy_id or default_policy_id(args.out)
    clash = id_claimed_elsewhere(policy_id, args.out)
    if clash:
        ap.error(f"policy_id {policy_id!r} is already claimed by {', '.join(clash)} "
                 f"(output folder {Path(args.out).resolve().parent}); pass --policy-id "
                 f"or another --out")
    import city_planner as cp  # noqa: E402  (numpy; only the generation needs it)
    random.seed(args.seed)

    m = cp.load_occ(args.citymap)
    occ, res, ox, oy = m["occ"], m["res"], m["ox"], m["oy"]
    grid = cp.inflate(occ, res, 5.0)

    def free_pt(margin=55.0):
        for _ in range(2000):
            x = random.uniform(-margin, margin)
            y = random.uniform(-margin, margin)
            if not cp.is_blocked(grid, res, ox, oy, x, y):
                return (round(x), round(y))
        return (0, 0)

    # spawn is (35, -20); pick N spread-out free waypoints
    wps = []
    while len(wps) < args.n:
        p = free_pt()
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 30 for q in wps):
            wps.append(p)

    SPAWN = (35.0, -20.0)
    stops = [SPAWN] + wps
    nfzs = []
    for i, (a, b) in enumerate(zip(stops[:-1], stops[1:])):
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)      # midpoint of the leg
        h = args.nfz / 2
        # NEVER let the NFZ box contain the spawn or any waypoint (with a 4 m
        # buffer) — otherwise the drone starts/sits inside it during takeoff and
        # racks up unavoidable NFZ time (the Shield can't undo a spawn-inside).
        if any(abs(mid[0] - s[0]) < h + 4 and abs(mid[1] - s[1]) < h + 4
               for s in stops):
            continue
        # only keep the NFZ if a detour still exists (planner finds a path)
        test = grid.copy()
        for ii in range(occ.shape[0]):
            wx = ox + ii * res
            if abs(wx - mid[0]) > h:
                continue
            for jj in range(occ.shape[1]):
                wy = oy + jj * res
                if abs(wy - mid[1]) <= h:
                    test[ii, jj] = 1
        if cp.plan(test, res, ox, oy, a, b) is not None:
            nfzs.append((mid[0] - h, mid[0] + h, mid[1] - h, mid[1] + h))

    # write the policy YAML (WGS84 since 2026-11-03; --metres for the legacy form)
    write = policy_yaml if args.metres else wgs84_policy_yaml
    if args.metres:
        print("WARNING: --metres is deprecated (WP1-14): it writes the legacy "
              "local-metre form, which loads but is not the canonical WGS84 form",
              file=sys.stderr)
    Path(args.out).write_text(write(policy_id, nfzs, args.band_lo, args.band_hi),
                              encoding="utf-8")

    route = "; ".join(f"{x},{y}" for x, y in wps)
    print(f"WAYPOINTS: {wps}")
    print(f"NFZS ({len(nfzs)}): {[tuple(round(v) for v in n) for n in nfzs]}")
    print(f"POLICY: {args.out} (policy_id {policy_id}; lock it with "
          f"`python -m guardrail.bundle lock --update`)")
    print(f"ROUTE={route}")


if __name__ == "__main__":
    main()
