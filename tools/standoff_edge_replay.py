"""Open-loop replay of a stored follow flight through FenceGuard before and
after THE STAND-OFF EDGE (demo/follow_vlm.py, 2026-11-20).

Run:  python tools/standoff_edge_replay.py [RUN] [--root DIR] [--policy P]
                                           [--json OUT] [--txt OUT]

Default run nfz3b_agx_hier_aerovla_wgs84_20261119 under
policies/follow_car_citylife_nfz3b.yaml (the policy it flew, as a WGS84
bundle), the CityLife day map as fly() loads it (onepath_guard_replay.
flight_setup). `--root` is the checkout whose demo/out holds the run and the
map (the lab checkout: demo/out is gitignored).

What is compared, honestly:
  * BEFORE is `LegacyStandoffGuard`: FenceGuard with the stand-off edge taken
    out - inside a zone's stand-off gate() holds every velocity (scale 0,
    blocked, cause "fence"), and the loop does not call standoff_admit. That
    is gate() and the loop as they were at 6fb8312; everything else
    (building half, slide, the Shield's rulers) is today's.
  * AFTER is FenceGuard as fly() builds it now, with the loop's
    standoff_admit before gate().
  * The decision is the flight loop's (clear / near / hold / skirt).
  * HOLD TICKS: the log keeps guard_in = 0 on a hold, so the command before
    the guard is NOT in the log. It is rebuilt: the forward direction is the
    logged trail direction (the heading when there is none), re-aimed by
    clear_aim (lookahead 10 m) where the log says fence_aim, at nominal
    speeds 1, 2 and 4 m/s. The BEFORE replay's agreement with the logged
    "hold" is printed: it is the measure of how far the rebuilt commands are
    from the flown ones, and every AFTER figure on those ticks inherits it.
  * OTHER TICKS: the command is guard_in / fence_scale (fence_scale >= 0.05,
    no skirt: exact up to the log's rounding), and the two guards' decisions
    are compared tick by tick.
  * Open-loop: a changed decision would have changed the flight from that
    tick on, which no replay of a stored log can show. Nothing here flies,
    and nothing it prints is a flight result.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from follow_vlm import FenceGuard                                  # noqa: E402

RUN = "nfz3b_agx_hier_aerovla_wgs84_20261119"
POLICY = ROOT / "policies" / "follow_car_citylife_nfz3b.yaml"
SPEEDS = (1.0, 2.0, 4.0)
AIM_LOOKAHEAD_M = 10.0          # follow_vlm --trail-lookahead-m default
HOLD_WINDOW_S = (79.7, 120.0)   # the 403-tick hold (diagnosis section 2)


class LegacyStandoffGuard(FenceGuard):
    """DIAGNOSTIC ONLY: FenceGuard without the stand-off edge (6fb8312):
    inside a zone's stand-off every velocity holds, and nothing is admitted."""

    def _gate_inside(self, ruler, x, y, ux, uy, horizon, d, o_scale, o_blocked, o_cause):
        self.last_cause = "fence"
        return 0.0, d, True

    def standoff_admit(self, x, y, vx, vy):
        return vx, vy, 0.0


def decide(g, x: float, y: float, cvx: float, cvy: float):
    """(mode, scale, admitted command, cut) as the flight loop decides."""
    cvx, cvy, cut = g.standoff_admit(x, y, cvx, cvy)
    fscale, fdist, fblocked = g.gate(x, y, cvx, cvy)
    approaching = fdist is not None and fdist < g.brake_m and fscale < 1.0
    if fblocked or approaching:
        sx, sy, _ = g.slide(x, y, cvx, cvy)
        if sx or sy:
            return "skirt", fscale, (cvx, cvy), cut
        return ("hold" if fblocked else "near"), fscale, (cvx, cvy), cut
    return ("clear" if fdist is None or fdist > 20 else "near"), fscale, (cvx, cvy), cut


def rebuilt_direction(g, r: dict) -> tuple[float, float]:
    x, y = float(r["x"]), float(r["y"])
    tr = r.get("trail") or {}
    a = math.radians(tr["dir_deg"]) if tr.get("dir_deg") is not None else float(r["psi"])
    ux, uy = math.cos(a), math.sin(a)
    if r.get("fence_aim"):
        aim = g.clear_aim(x, y, ux, uy, AIM_LOOKAHEAD_M)
        if aim is not None:
            ux, uy = aim
    return ux, uy


def replay(rows: list, old, new) -> dict:
    holds = [r for r in rows if r.get("fence_hold") and r.get("fence_cause") == "fence"]
    window = [r for r in holds if HOLD_WINDOW_S[0] <= r["t"] <= HOLD_WINDOW_S[1]]
    out = {"ticks_logged": len(rows), "fence_hold_ticks_logged": len(holds),
           "hold_window_s": list(HOLD_WINDOW_S), "hold_window_ticks": len(window),
           "hold_ticks": {}}
    for label, sel in (("all_fence_holds", holds), ("hold_window", window)):
        by = {}
        for spd in SPEEDS:
            o_modes, n_modes, cut, moving = {}, {}, 0, []
            for r in sel:
                x, y = float(r["x"]), float(r["y"])
                ux, uy = rebuilt_direction(new, r)
                o = decide(old, x, y, spd * ux, spd * uy)
                n = decide(new, x, y, spd * ux, spd * uy)
                o_modes[o[0]] = o_modes.get(o[0], 0) + 1
                n_modes[n[0]] = n_modes.get(n[0], 0) + 1
                cut += int(n[3] > 0.0)
                if n[0] != "hold":
                    moving.append(math.hypot(*n[2]) * n[1])
            by[str(spd)] = {
                "before": o_modes, "after": n_modes,
                "before_agrees_with_logged_hold": o_modes.get("hold", 0),
                "after_inward_part_cut": cut,
                "after_mean_speed_when_not_held": (round(sum(moving) / len(moving), 3)
                                                   if moving else None)}
        out["hold_ticks"][label] = by
    same = changed = inside = 0
    examples = []
    for r in rows:
        gi, sc, fm = r.get("guard_in"), r.get("fence_scale"), r.get("fence_mode")
        if gi is None or sc is None or sc < 0.05 or fm == "skirt":
            continue
        x, y = float(r["x"]), float(r["y"])
        cvx, cvy = gi[0] / sc, gi[1] / sc
        o = decide(old, x, y, cvx, cvy)
        n = decide(new, x, y, cvx, cvy)
        inside += int(new._ruler().distance(x, y) <= new.stand_off_m)
        if o[0] == n[0] and abs(o[1] - n[1]) < 1e-9:
            same += 1
        else:
            changed += 1
            if len(examples) < 10:
                examples.append({"t": r["t"], "before": o[0], "after": n[0]})
    out["other_ticks"] = {"replayed": same + changed, "decision_same": same,
                          "decision_changed": changed, "inside_a_stand_off": inside,
                          "examples": examples}
    return out


def text(res: dict, run: str) -> str:
    lines = [f"{run}: open-loop replay, FenceGuard before (6fb8312) and after the stand-off edge",
             f"ticks logged {res['ticks_logged']}; logged fence-cause holds "
             f"{res['fence_hold_ticks_logged']}; the {HOLD_WINDOW_S[0]}-{HOLD_WINDOW_S[1]} s "
             f"hold: {res['hold_window_ticks']} ticks (logged hold on every one)",
             "HOLD TICKS (command REBUILT from trail direction + clear_aim, not logged):"]
    for label, by in res["hold_ticks"].items():
        for spd, v in by.items():
            lines.append(f"  {label} @ {spd} m/s: before {v['before']} "
                         f"(reproduces the logged hold on {v['before_agrees_with_logged_hold']}); "
                         f"after {v['after']}, inward part cut on {v['after_inward_part_cut']}, "
                         f"mean admitted speed when not held {v['after_mean_speed_when_not_held']}")
    o = res["other_ticks"]
    lines.append(f"OTHER TICKS (command = guard_in / fence_scale): {o['replayed']} replayed, "
                 f"decision changed on {o['decision_changed']} "
                 f"({o['inside_a_stand_off']} inside a stand-off)")
    lines.append("Open-loop; nothing here flew.")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run", nargs="?", default=RUN)
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--policy", default=str(POLICY))
    ap.add_argument("--citymap", default="citymap_citylife")
    ap.add_argument("--json", default=None)
    ap.add_argument("--txt", default=None)
    a = ap.parse_args(argv)
    from onepath_guard_replay import flight_setup
    lab = Path(a.root)
    log = lab / "demo" / "out" / a.run / "flight_log.jsonl"
    if not log.is_file():
        print(f"{a.run}: no flight_log.jsonl under {lab / 'demo' / 'out'}")
        return 2
    pol, smap, _grid, new = flight_setup(lab, Path(a.policy), a.citymap)
    old = LegacyStandoffGuard(pol, obstacle_map=smap, min_clearance_m=new.min_clearance_m,
                              street_mask=new.street, shield=new.shield)
    rows = [json.loads(line) for line in log.open(encoding="utf-8")]
    res = replay(rows, old, new)
    res.update(run=a.run, policy=Path(a.policy).name, citymap=a.citymap,
               speeds_mps=list(SPEEDS), aim_lookahead_m=AIM_LOOKAHEAD_M)
    t = text(res, a.run)
    print(t, end="")
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1) + "\n", encoding="utf-8")
    if a.txt:
        Path(a.txt).write_text(t, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
