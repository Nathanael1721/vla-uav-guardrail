"""Write the generated driving paths into the CityLife cars and place them.

    python -m tools.citylife_mcp.apply_routes            # write + place all 24
    python -m tools.citylife_mcp.apply_routes --dry-run  # print the plan only

The level is gitignored, so this file is the record of what every car was
given. Per car it writes, through the editor's MCP endpoint:

  Route / RouteSpeed / RouteKappa   the dense lane-centre path of its loop from
                                    tools/citylife_routes.py (keep-left, filleted
                                    corners, speed limit and curvature per point)
  Junctions                         every junction the loop drives through;
                                    z = 1 where it must give way: its path
                                    crosses another loop's there and it is the
                                    one turning across (give_way_junctions)
  Crossings                         the pedestrian crossings its PATH drives
                                    over (crossings_on); z = the path index
                                    nearest the crossing, so a car measures
                                    the distance to it along its own path
  Idx, transform                    the car is placed ON its path, facing along
                                    it, cars of one loop evenly spaced
  JExit / JOff                      per Junctions entry: the exit lane point level
                                    with the centre (z = exit yaw, deg), and the
                                    start of the junction's NS green (s) - what
                                    UpdateEffSpeed's signal rules read
                                    (drive_signals.py; citylife_traffic_model
                                    .jexit_rows, citylife_signals.offset)

Arrays are written as [] first and then in full: resizing a non-empty array
property and filling it in one write is the case this toolset handles badly.

THE SUBJECT. Car_10 is the one red car and the drone's subject. It drives loop
A at 3.2 m/s, below the aircraft's 4 m/s --speed-max, so a follow can keep
station. It is placed on A's eastbound leg just past the corner at (4100,
-4100), i.e. about 10 m behind the drone's spawn (UE 3500, -2000) and facing
the same way, so it drives into the camera's view from behind.
"""
import json
import math
import sys

from tools import citylife_routes as R
from tools.citylife_mcp import ue_rpc

ASSIGN = {
    "A": ["Car_10", "Car_11", "Car_01", "Car_02", "Car_12", "Car_03", "Car_22",
          "Car_04", "Car_23", "Car_05", "Car_08", "Car_09", "Car_00"],
    "B": ["Car_06", "Car_13", "Car_07", "Car_14", "Car_15"],
    "C": ["Car_16", "Car_17", "Car_18", "Car_19", "Car_20", "Car_21"],
}
SUBJECT, SUBJECT_SPEED = "Car_10", 320.0
SUBJECT_AT = (4450.0, -3000.0)           # on A's eastbound lane, behind the drone
Z = 5.0


def junction_flags(name: str, give_way: set):
    """(x, y, flag): flag 1 where this loop must give way."""
    return [(j[0], j[1], 1.0 if j in give_way else 0.0) for j in R.junctions_on(R.LOOPS[name])]


def placements(path: R.Path, cars, anchor=None):
    """Even spacing by arc length; the first car at the path point nearest
    `anchor` (or at point 0). Returns [(car, idx_next, x, y, yaw_deg)]."""
    pts, n = path.pts, len(path.pts)
    cum = [0.0]
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = cum[-1]
    i0 = 0
    if anchor is not None:
        i0 = min(range(n), key=lambda i: math.hypot(pts[i][0] - anchor[0], pts[i][1] - anchor[1]))
    out = []
    for k, car in enumerate(cars):
        s = (cum[i0] + k * total / len(cars)) % total
        i = max(j for j in range(n) if cum[j] <= s)
        a, b = pts[i], pts[(i + 1) % n]
        f = (s - cum[i]) / max(cum[i + 1] - cum[i], 1e-9)
        x, y = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        yaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))   # X north, Y east
        out.append((car, (i + 1) % n, x, y, yaw))
    return out


def plan():
    paths = R.build_all()
    give_way = R.give_way_junctions(paths)
    cars = {}
    for loop, names in ASSIGN.items():
        p = paths[loop]
        anchor = SUBJECT_AT if SUBJECT in names else None
        for car, idx, x, y, yaw in placements(p, names, anchor):
            cars[car] = {
                "loop": loop, "idx": idx, "x": round(x, 1), "y": round(y, 1), "yaw": round(yaw, 3),
                "speed": SUBJECT_SPEED if car == SUBJECT else None,
            }
    data = {
        "loops": {k: {"route": [[round(x, 1), round(y, 1)] for x, y in p.pts],
                      "speed": [round(v, 1) for v in p.speed],
                      "kappa": [-c for c in p.curvature],      # car sign: + right
                      "junctions": junction_flags(k, give_way[k]),
                      "crossings": [[x, y, float(i)] for x, y, i in R.crossings_on(p)]}
                  for k, p in paths.items()},
        "cars": cars,
        "z": Z,
    }
    return data


EDITOR = r'''
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
DATA = json.loads(__DATA__)


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    z = DATA["z"]
    by_tag = {}
    for a in T(ST + "find_actors", name="", tag="citylife.car", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            continue
        for t in T(AC + "get_tags", actor=a):
            if t.startswith("Car_"):
                by_tag[t] = a
    done = {}
    for car, c in DATA["cars"].items():
        if car not in by_tag:
            done[car] = "MISSING"
            continue
        a = by_tag[car]
        L = DATA["loops"][c["loop"]]
        full = {
            "route": [{"x": p[0], "y": p[1], "z": z} for p in L["route"]],
            "routeSpeed": L["speed"],
            "routeKappa": L["kappa"],
            "junctions": [{"x": j[0], "y": j[1], "z": j[2]} for j in L["junctions"]],
            "crossings": [{"x": q[0], "y": q[1], "z": q[2]} for q in L["crossings"]],
            "jExit": [{"x": e[0], "y": e[1], "z": e[2]} for e in L["jexit"]],
            "jOff": L["joff"],
        }
        T(OT + "set_properties", instance=a, values=json.dumps({k: [] for k in full}))
        T(OT + "set_properties", instance=a, values=json.dumps(full))
        extra = {"idx": c["idx"]}
        if c["speed"] is not None:
            extra["speedCmS"] = c["speed"]
        T(OT + "set_properties", instance=a, values=json.dumps(extra))
        T(AC + "set_actor_transform", actor=a,
          xform={"location": {"x": c["x"], "y": c["y"], "z": z},
                 "rotation": {"pitch": 0.0, "yaw": c["yaw"], "roll": 0.0}},
          worldspace=True)
        back = json.loads(T(OT + "get_properties", instance=a,
                            properties=["route", "routeSpeed", "routeKappa", "junctions",
                                        "crossings", "idx", "speedCmS", "jExit", "jOff"]))
        done[car] = [c["loop"], len(back["route"]), len(back["routeSpeed"]), len(back["routeKappa"]),
                     len(back["junctions"]), len(back["crossings"]), back["idx"], back["speedCmS"],
                     len(back["jExit"]), len(back["jOff"])]
    return {"cars": done}
'''


def with_signal_rows(data: dict) -> dict:
    """Add each loop's JExit rows and junction offsets (see the docstring)."""
    from tools import citylife_signals as S
    from tools.citylife_traffic_model import jexit_rows
    rows = jexit_rows(data)
    for k, L in data["loops"].items():
        L["jexit"] = [[round(x, 1), round(y, 1), yaw] for x, y, yaw in rows[k]]
        L["joff"] = [S.offset(j[0], j[1]) for j in L["junctions"]]
    return data


def main():
    data = with_signal_rows(plan())
    if "--dry-run" in sys.argv:
        for car, c in sorted(data["cars"].items()):
            print(car, c)
        for k, L in data["loops"].items():
            print(k, len(L["route"]), "junctions", L["junctions"], "crossings", L["crossings"])
        return
    code = EDITOR.replace("__DATA__", json.dumps(json.dumps(data)))
    res = ue_rpc.run_script(code)
    bad = {}
    for car, row in res["cars"].items():
        if row == "MISSING":
            bad[car] = row
            continue
        loop = data["loops"][row[0]]
        want = [len(loop["route"])] * 3 + [len(loop["junctions"]), len(loop["crossings"])]
        if (row[1:6] != want or row[6] != data["cars"][car]["idx"]
                or row[8:10] != [len(loop["junctions"])] * 2):
            bad[car] = (row, want)
    for car, row in sorted(res["cars"].items()):
        print(car, row)
    print("MISMATCH" if bad else "all readbacks match", bad or "")


if __name__ == "__main__":
    main()
