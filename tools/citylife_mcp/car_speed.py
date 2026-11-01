"""Read or set a CityLife car's SpeedCmS (its top speed, cm/s) in the open editor.

    python -m tools.citylife_mcp.car_speed                # print every car's SpeedCmS
    python -m tools.citylife_mcp.car_speed Car_07 320     # set one car and save the level

The level is gitignored, so every change made with this is recorded in the
CHANGELOG with the value it replaced. Setting prints the old value first, then
saves and checks the .umap's file date: save_assets can report success while the
file stays locked (it did once after the disk ran full).

2026-11-01: Car_07, the only black car (loop B), set from its old value to 320
(Car_10's speed) for the AIOT-VLA black-car recordings, because it drove faster
than the aircraft's 5 m/s cap. Put back to 560 the same afternoon, after the
black-car and person work.
"""
import json
import os
import sys

from tools.citylife_mcp import ue_rpc

UMAP = os.path.join("PASBlocks", "Content", "CityLife", "Maps", "CityLife_Day.umap")

EDITOR = r'''
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
WANT = json.loads(__WANT__)


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    by_tag = {}
    for a in T(ST + "find_actors", name="", tag="citylife.car", collision_channels=[]):
        if "UEDPIE" in a["refPath"]:
            continue
        for t in T(AC + "get_tags", actor=a):
            if t.startswith("Car_"):
                by_tag[t] = a
    for car in WANT:
        if car not in by_tag:
            return {"error": "MISSING " + car}
    for car, v in WANT.items():
        T(OT + "set_properties", instance=by_tag[car], values=json.dumps({"speedCmS": v}))
    speed = {}
    for car, a in by_tag.items():
        speed[car] = json.loads(T(OT + "get_properties", instance=a, properties=["speedCmS"]))["speedCmS"]
    saved = None
    if WANT:
        saved = execute_tool("editor_toolset.toolsets.asset.AssetTools.save_assets",
                             json.dumps({"asset_paths": []}))["returnValue"]
    return {"speed": speed, "saved": saved}
'''


def run(want: dict) -> dict:
    res = ue_rpc.run_script(EDITOR.replace("__WANT__", json.dumps(json.dumps(want))))
    if "error" in res:
        raise SystemExit(res["error"])
    return res


def main():
    before = run({})["speed"]
    if len(sys.argv) < 3:
        for car in sorted(before):
            print(car, before[car])
        return
    car, value = sys.argv[1], float(sys.argv[2])
    if car not in before:
        raise SystemExit(f"no car tagged {car}")
    print(f"{car} SpeedCmS was {before[car]}")
    mtime = os.path.getmtime(UMAP)
    res = run({car: value})
    print(f"{car} SpeedCmS now {res['speed'][car]}; save_assets returned {res['saved']}")
    others = {c: v for c, v in res["speed"].items() if c != car and v != before[c]}
    if res["speed"][car] != value or others:
        raise SystemExit(f"READBACK MISMATCH {res['speed'][car]} {others}")
    if os.path.getmtime(UMAP) <= mtime:
        raise SystemExit(f"{UMAP} was NOT rewritten: the save did not reach the disk")
    print(f"{UMAP} rewritten")


if __name__ == "__main__":
    main()
