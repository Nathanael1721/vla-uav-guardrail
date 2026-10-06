"""Measure the CityLife traffic inside the running engine (Simulate / PIE).

    python -m tools.citylife_mcp.verify_drive start       # save all, start Simulate
    python -m tools.citylife_mcp.verify_drive read        # counters of every car
    python -m tools.citylife_mcp.verify_drive stop

`read` prints, per car, the counters DriveTick and UpdateEffSpeed keep:

  eyMax       worst lateral distance from its lane centre (cm) after warm-up
  alatMax     worst v^2 * kappa (cm/s^2): 180 is the design, 200 the ceiling
  minEver     closest centre-to-centre distance to any other car (cm). Cars in
              a queue stop 650 apart and oncoming lanes are 700 apart, so a
              value well under 500 means two cars went through each other
  yieldTicks  half-rate ticks spent waiting at a junction
  pedTicks    half-rate ticks spent waiting for a pedestrian on a crossing
  stopRunMax  longest continuous stand-still (s): a deadlock grows without bound
  pedViol     half-rate ticks driving over a crossing at > 50 cm/s while a
              MOVING pedestrian is anywhere on the zebra. A gate until
              2026-09-24; now context - a car already on the crossing drives on
              past a pedestrian in the other lane by design
  pedStill    the same past a pedestrian standing still, allowed after 6 s
  eStops      half-rate ticks of the emergency stop for a pedestrian in the
              car's own path once it is already on the zebra (2026-09-24)
  pedCorr     those of them still above 50 cm/s, i.e. while braking
  pedClose    half-rate ticks above 50 cm/s with a MOVING pedestrian in the
              car's own path within 4 m of its centre. Includes figures that
              step out inside braking distance, so not a pass/fail gate

These are read from the car itself, not sampled from outside: an outside
snapshot of 24 cars takes about 70 editor calls, each of which lets the game
advance a frame, so it is not a snapshot at all.
"""
import json
import sys

from tools.citylife_mcp import ue_rpc

START = r'''
import json

def run():
    saved = execute_tool("editor_toolset.toolsets.asset.AssetTools.save_assets",
                         json.dumps({"asset_paths": []}))["returnValue"]
    running = execute_tool("EditorToolset.EditorAppToolset.IsPIERunning", "{}")["returnValue"]
    if not running:
        execute_tool("EditorToolset.EditorAppToolset.StartPIE", json.dumps(
            {"options": {"bSimulate": True, "playMode": "PlayMode_InViewPort",
                         "warmupSeconds": 5}}))
    return {"saved": saved, "was_running": running}
'''

STOP = r'''
def run():
    running = execute_tool("EditorToolset.EditorAppToolset.IsPIERunning", "{}")["returnValue"]
    if running:
        execute_tool("EditorToolset.EditorAppToolset.StopPIE", "{}")
    return {"was_running": running}
'''

READ = r'''
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
PROPS = ["eyMax", "alatMax", "minEver", "closeTicks", "yieldTicks", "pedTicks",
         "stopRunMax", "stopRun", "curSpeed", "speedCmS", "ticks", "idx",
         "pedViol", "pedPassStill"]
# Read on their own: get_properties refuses the WHOLE call if one name is
# unknown, so asking for these alongside the rest made a BP_CityCar that
# predates them unreadable instead of "not measured" (review round 4).
NEW = ["eStops", "pedCorr", "pedClose"]
# drive_signals.py (2026-09-29): the signal rules' own counters.
SIG = ["redViol", "zebraWait", "sigHolds", "sneaks", "lateHold", "boxStill"]


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    out = {}
    for a in T(ST + "find_actors", name="", tag="citylife.car", collision_channels=[]):
        if "UEDPIE" not in a["refPath"]:
            continue
        tag = [t for t in T(AC + "get_tags", actor=a) if t.startswith("Car_")][0]
        p = json.loads(T(OT + "get_properties", instance=a, properties=PROPS))
        try:
            p.update(json.loads(T(OT + "get_properties", instance=a, properties=NEW)))
        except RuntimeError:
            pass
        try:
            p.update(json.loads(T(OT + "get_properties", instance=a, properties=SIG)))
        except RuntimeError:
            pass
        loc = T(AC + "get_actor_transform", actor=a)
        p["x"] = round(loc["location"]["x"])
        p["y"] = round(loc["location"]["y"])
        p["yaw"] = round(loc["rotation"]["yaw"], 1)
        out[tag] = p
    return {"cars": out}
'''


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "read"
    if mode == "start":
        print(json.dumps(ue_rpc.run_script(START)))
        return
    if mode == "stop":
        print(json.dumps(ue_rpc.run_script(STOP)))
        return
    cars = ue_rpc.run_script(READ)["cars"]
    if "--json" in sys.argv:
        print(json.dumps(cars))
        return
    hdr = "car     eyMax alatMax minEver yieldT pedT viol still stopMax stopNow  v/cap   ticks  pos"
    print(hdr)
    for k in sorted(cars):
        c = cars[k]
        print("%-7s %5.1f %7.1f %7.0f %6d %4d %4d %5d %7.1f %7.1f %4.0f/%-4.0f %6d  (%d, %d) %.0f" % (
            k, c["eyMax"], c["alatMax"], c["minEver"], c["yieldTicks"], c["pedTicks"],
            c["pedViol"], c["pedPassStill"],
            c["stopRunMax"], c["stopRun"], c["curSpeed"], c["speedCmS"], c["ticks"],
            c["x"], c["y"], c["yaw"]))
    vals = list(cars.values())
    if vals:
        print("worst: eyMax %.1f cm, alatMax %.1f cm/s^2, minEver %.0f cm, stopRunMax %.1f s; "
              "yield ticks %d, ped ticks %d, ped violations %d, passed-still %d" % (
                  max(c["eyMax"] for c in vals), max(c["alatMax"] for c in vals),
                  min(c["minEver"] for c in vals), max(c["stopRunMax"] for c in vals),
                  sum(c["yieldTicks"] for c in vals), sum(c["pedTicks"] for c in vals),
                  sum(c["pedViol"] for c in vals), sum(c["pedPassStill"] for c in vals)))
        new = ("eStops", "pedCorr", "pedClose")
        if not all(isinstance(c.get(k), (int, float)) for c in vals for k in new):
            # A BP_CityCar that predates drive_tick.py's 2026-09-24 pass has no
            # such counters; a 0 printed for them would read as a pass.
            print("on-the-zebra counters NOT MEASURED: BP_CityCar predates "
                  "drive_tick.py (run it, then Simulate again)")
        else:
            print("on-the-zebra emergency stops %d half-rate ticks, %d of them still above 50 cm/s; "
                  "pedestrian within 4 m ahead at speed (incl. step-outs): %d" % (
                      sum(c["eStops"] for c in vals), sum(c["pedCorr"] for c in vals),
                      sum(c["pedClose"] for c in vals)))
        sig = ("redViol", "zebraWait", "sigHolds", "sneaks", "lateHold", "boxStill")
        if not all(isinstance(c.get(k), (int, float)) for c in vals for k in sig):
            print("signal counters NOT MEASURED: UpdateEffSpeed predates drive_signals.py")
        else:
            # zebraWait / sigHolds are half-rate ticks; boxStill is seconds.
            print("signals: RedViol %d, ZebraWait %d half-rate ticks, signal holds %d, "
                  "sneaks %d, late holds %d, stood in a box %.1f s (worst car %.1f s)" % (
                      sum(c["redViol"] for c in vals), sum(c["zebraWait"] for c in vals),
                      sum(c["sigHolds"] for c in vals), sum(c["sneaks"] for c in vals),
                      sum(c["lateHold"] for c in vals), sum(c["boxStill"] for c in vals),
                      max(c["boxStill"] for c in vals)))


if __name__ == "__main__":
    main()
