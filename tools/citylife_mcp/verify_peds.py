"""Measure the CityLife pedestrians inside a running Simulate.

    python -m tools.citylife_mcp.verify_peds          # one read of every figure
    python -m tools.citylife_mcp.verify_peds --watch 6 --every 20

Per figure (tag Ped_NN), from BP_CityPed's own variables (ped_walk.py):
PState (0 walk, 1 pause, 2 wait at a kerb, 3 cross), Idx, Crossings (walk
starts), KerbWaitMax (s), Spd (its XY speed, cm/s, written every tick), and
its position. And the things a viewer would call unnatural:

  on_zebra_not_crossing  per read: a figure inside a zebra box that is not
                   crossing (PState != 3) or is standing (Spd < 10 cm/s) - the
                   old Roam idled there, and the model's ZebraIdleS is 0.
                   Counted by distinct tag over all reads, beside how many
                   figure-reads were in a zebra box at all: 0 of 0 measured
                   nothing. Spd is read on its own - get_properties refuses a
                   whole call over one unknown name - and a BP_CityPed that
                   predates it is checked on PState only, and says so. The
                   speed cannot come from positions instead: every editor
                   call lets the game advance a frame, so two positions read
                   in one script are not a known time apart;
  on_zebra_standing_between_reads  the same figure within 10 cm of where it
                   was one read earlier, inside a zebra box. Needs two reads:
                   with one it is None, not 0;
  displaced        straight-line displacement between reads, against the path
                   length it could have walked: a pacing figure goes nowhere;
  off_route        per read, a figure farther than OFF_ROUTE_CM from where its
                   state puts it: walking or crossing, from the leg between
                   Route[Idx-1] and Route[Idx]; waiting or pausing, from
                   Route[Idx-1], the point it reached. The check that was
                   missing when the stuck rule cascaded figures off their
                   tours (2026-09-30: 21 of 40 frozen in CROSS tens of metres
                   from any zebra, while every zebra count read clean);
  detours, teleports  BP_CityPed's own counts of the stuck rule's sidesteps
                   and set-downs (ped_walk.py, STUCK), summed over figures.

The car side of the crossing (PedViol, EStops, PedClose) is verify_drive.py.
"""
import json
import math
import sys
import time

from tools import citylife_routes as R
from tools.citylife_mcp import ue_rpc

READ = r'''
import json
ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
PROPS = ["PState", "Idx", "Crossings", "KerbWaitMax", "bUseNavMesh"]


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    out = {}
    for a in T(ST + "find_actors", name="", tag="", collision_channels=[]):
        if "UEDPIE" not in a["refPath"] or "CityPed" not in a["refPath"]:
            continue
        tags = [t for t in T(AC + "get_tags", actor=a) if t.startswith("Ped_")]
        if not tags:
            continue
        try:
            p = json.loads(T(OT + "get_properties", instance=a, properties=PROPS))
        except RuntimeError:
            p = {}
        for extra in (["Spd"], ["Route"], ["Detours", "Teleports"]):
            try:
                p.update(json.loads(T(OT + "get_properties", instance=a, properties=extra)))
            except RuntimeError:
                pass
        xf = T(AC + "get_actor_transform", actor=a)
        p["x"] = round(xf["location"]["x"])
        p["y"] = round(xf["location"]["y"])
        out[tags[0]] = p
    t = None
    for a in T(ST + "find_actors", name="SignalController", tag="", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            t = json.loads(T(OT + "get_properties", instance=a, properties=["NowT"]))["NowT"]
    return {"t": t, "peds": out}
'''

STILL_CMS = 10.0
OFF_ROUTE_CM = 500.0


def in_zebra(x, y) -> bool:
    for cx, cy in R.CROSSINGS:
        jx = round((cx - 4100) / 8200) * 8200 + 4100
        jy = round((cy - 4100) / 8200) * 8200 + 4100
        if abs(cx - jx) >= abs(cy - jy):          # on the NS road: band spans Y
            if abs(x - cx) < 300 and abs(y - cy) < 800:
                return True
        elif abs(y - cy) < 300 and abs(x - cx) < 800:
            return True
    return False


def _low(p: dict) -> dict:
    """Property names as the tool returns them may differ in case (curSpeed
    for CurSpeed in verify_drive.py): read them case-blind."""
    return {k.lower(): v for k, v in p.items()}


def on_zebra_not_crossing(reads: list) -> dict:
    """The per-read check: which figures were ever inside a zebra box while
    not crossing or standing, out of how many figure-reads in a box - and,
    per such figure, what it was doing (`why`: read, state, target index,
    speed, place), since the tag alone cannot tell a figure that walked onto
    a zebra from one that stopped mid-crossing."""
    tags, in_box, no_state, no_speed = set(), 0, 0, 0
    why = {}
    for k, rd in enumerate(reads):
        for tag, p in rd["peds"].items():
            q = _low(p)
            if not in_zebra(q["x"], q["y"]):
                continue
            in_box += 1
            st, spd = q.get("pstate"), q.get("spd")
            no_state += st is None
            no_speed += spd is None
            if (st is not None and st != 3) or (spd is not None and spd < STILL_CMS):
                tags.add(tag)
                why.setdefault(tag, []).append(
                    {"read": k, "pstate": st, "idx": q.get("idx"),
                     "spd": None if spd is None else round(spd, 1),
                     "x": q["x"], "y": q["y"]})
    return {"tags": sorted(tags), "figure_reads_in_a_box": in_box,
            "state_unread": no_state, "speed_unread": no_speed, "why": why}


def _seg_dist(px, py, a, b) -> float:
    ax, ay, bx, by = a["x"], a["y"], b["x"], b["y"]
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / L2))
    return math.hypot(px - (ax + u * (bx - ax)), py - (ay + u * (by - ay)))


def off_route(reads: list, lim: float = OFF_ROUTE_CM) -> dict:
    """Figures ever farther than `lim` from where their state puts them (see
    the module docstring), by distinct tag, with the first such read; and
    how many figure-reads could be checked at all (Route, Idx, PState read)."""
    tags, checked = {}, 0
    for k, rd in enumerate(reads):
        for tag, p in rd["peds"].items():
            q = _low(p)
            r, i, st = q.get("route"), q.get("idx"), q.get("pstate")
            if not r or i is None or st is None:
                continue
            checked += 1
            prev, cur = r[(i - 1) % len(r)], r[i % len(r)]
            d = (_seg_dist(q["x"], q["y"], prev, cur) if st in (0, 3)
                 else math.hypot(q["x"] - prev["x"], q["y"] - prev["y"]))
            if d > lim and tag not in tags:
                tags[tag] = {"read": k, "pstate": st, "idx": i, "cm": round(d)}
    return {"tags": tags, "checked": checked}


def main():
    n = int(sys.argv[sys.argv.index("--watch") + 1]) if "--watch" in sys.argv else 1
    every = float(sys.argv[sys.argv.index("--every") + 1]) if "--every" in sys.argv else 20.0
    reads = []
    for k in range(n):
        if k:
            time.sleep(every)
        reads.append(ue_rpc.run_script(READ))
    last = reads[-1]
    peds = {tag: _low(p) for tag, p in last["peds"].items()}
    states = {}
    for p in peds.values():
        states[p.get("pstate")] = states.get(p.get("pstate"), 0) + 1
    zebra_still = 0
    moves = []
    for a, b in zip(reads, reads[1:]):
        for tag, q in b["peds"].items():
            if tag in a["peds"]:
                d = math.hypot(q["x"] - a["peds"][tag]["x"], q["y"] - a["peds"][tag]["y"])
                moves.append(d)
                if d < 10 and in_zebra(q["x"], q["y"]):
                    zebra_still += 1
    zc = on_zebra_not_crossing(reads)
    orr = off_route(reads)
    if zc["speed_unread"]:
        check = "PState only: %d figure-reads in a box had no Spd (re-run ped_walk.py)" % zc["speed_unread"]
    else:
        check = "PState and Spd"
    print(json.dumps({
        "t_game": last["t"], "reads": len(reads), "figures": len(peds),
        "on_routes": sum(1 for p in peds.values() if p.get("busenavmesh") is False),
        "state_counts": {("walk", "pause", "wait", "cross").__getitem__(k) if isinstance(k, int) and 0 <= k < 4 else str(k): v
                         for k, v in states.items()},
        "crossings_started": sum(p.get("crossings") or 0 for p in peds.values()),
        "kerb_wait_max_s": round(max((p.get("kerbwaitmax") or 0.0) for p in peds.values()), 1) if peds else None,
        "on_zebra_not_crossing": len(zc["tags"]),
        "on_zebra_not_crossing_tags": zc["tags"],
        "on_zebra_not_crossing_why": zc["why"],
        "on_zebra_figure_reads": zc["figure_reads_in_a_box"],
        "on_zebra_state_unread": zc["state_unread"],
        "on_zebra_check": check,
        "on_zebra_standing_between_reads": zebra_still if len(reads) >= 2 else None,
        "median_move_cm_between_reads": (round(sorted(moves)[len(moves) // 2]) if moves else None),
        "off_route": len(orr["tags"]),
        "off_route_why": orr["tags"],
        "off_route_checked_reads": orr["checked"],
        "detours": (sum(p.get("detours") or 0 for p in peds.values())
                    if any("detours" in p for p in peds.values()) else None),
        "teleports": (sum(p.get("teleports") or 0 for p in peds.values())
                      if any("teleports" in p for p in peds.values()) else None),
    }, indent=1))


if __name__ == "__main__":
    main()
