"""BP_CityCar.UpdateEffSpeed WITH SIGNALS: the Blueprint port of tools/citylife_traffic_model.py.

    python -m tools.citylife_mcp.drive_signals             # write the graph, compile
    python -m tools.citylife_mcp.drive_signals --render    # print the DSL only

drive_tick.py wrote UpdateEffSpeed (and still writes DriveTick, unchanged).
This replaces UpdateEffSpeed with mode="signals" of the reference model,
which was measured before a line of DSL was written: 30 simulated minutes of
all 24 cars at 3/10/30 fps, RedViol 0, ZebraWait 0, MinGap 650 cm, no
deadlock (tests/test_citylife_traffic_model.py). The two problems it fixes:

  (a) the cars ignored the signal heads (and the heads showed nothing: see
      signals.py for the lamps, driven by the same plan);
  (b) a car waiting at a junction stopped with its front half ON the zebra:
      the old give-way target JEdge - 300 put its front 1170 cm from the
      centre, inside the zebra band 800-1400.

The rules, node for node with update_eff() (read its docstring for the why
of every number):

  signal      phase of MY axis of travel at NextJ: JOff[k] (the junction's NS
              green start, citylife_signals.offset) + 25 s for EW, from
              GetGameTimeInSeconds; 0 green / 1 yellow / 2 red.
  stop line   DLine = along-distance to the centre - 1730 (zebra outer edge
              1400 + 100 + car half length 230).
  red         hold at the line while DLine > -50 (past that: committed).
  yellow      hold only if DLine >= v^2/500 - 50.
  green       hold for: box (a non-parallel car inside the box, except my
              platoon leader heading into my exit quarter or an oncoming car
              7 m past the centre), giveway (at a z = 1 junction: an ONCOMING
              car 1100-3600 cm out), exit (a car slower than 150 cm/s on my
              exit lane up to 2280 cm past the centre: JExit), exitped (a
              walker on my exit zebra during the first 8 s of green); a hold
              the car can no longer make at 4 m/s^2 commits, except box, which
              falls back to JEdge - 300 (Hold 7, the model's "late").
  committed   DLine <= -50: no signal hold; a car in the box still stops it
              at JEdge - 300, also Hold 7 (the model's `elif box_conf`).
  sneak      a turner held >= 4 s on green goes in the first 1.5 s of yellow
              if no oncoming car is unable to stop and box/exit are clear;
              held again before its line, it drops the sneak.
  queue       same-way car within 1300 cm (was 900); a leader < 90 deg round a
              bend in my corridor counts as a queue leader, not a slow-down.

Per car it needs two new instance arrays, written by apply_routes.py:
JExit (x, y of the exit lane level with the centre, z = exit yaw deg) and
JOff (s), parallel to Junctions. The other car's speed comes from SpeedOf, a
one-line function returning CurSpeed, called on the `_car` target: a
Blueprint cannot read another instance's variable of its OWN class through
the DSL (Class|BPCityCar|GetCurSpeed exists only in other classes, and the
self-class getter has no target pin), and GetVelocity is 0 for a car moved
by SetActorLocation.

New counters, read by verify_drive.py: RedViol (a MOVING car's centre went
more than 5 cm past its line-point between two updates while red and not
sneaking - the model's RED_TOL_CM; without the tolerance and the speed test a
car standing at the line flipped DLine's sign by millimetres and counted 35
in the first Simulate), ZebraWait (below), SigHolds, Sneaks, LateHold,
BoxStill (s stopped inside a junction box).

ZEBRAWAIT. Half-rate ticks a junction hold keeps a car standing with its
body on NextJ's approach zebra. Until 2026-09-30 the test sat inside the
held branch (DLine > -50), where the centre is more than 1680 cm out and the
front more than 1450, 50 cm outside the zebra's outer edge: it could not
fire, and the 0 a Simulate read was a rule that never ran. A junction rule
leaves the front ON the zebra only through the box rule's JEdge - 300 target
(centre 1400 out, front 1170, inside the band 800-1400): Hold 7, set in the
held branch when a box hold can no longer be made and in the committed
branch. So the test now runs after both branches, for any held stop:

    Hold > 0 and CurSpeed < V_STOPPED (10 cm/s), the hold binds (Target <=
    StopCm <= GapCm - 650: neither a crossing stop nor the queue is the
    tighter one), and the body overlaps the band: front (JEdge + 1100) - 230
    < ZEBRA_OUTER 1400 and rear (JEdge + 1100) + 230 > ZEBRA_INNER 800.

What still differs from simulate()'s zebra_wait (v < V_STOPPED, why in
HOLD_REASONS, stop <= gap - QUEUE_CM, _on_zebra):
  - the band is where NextJ's approach zebra would be, measured along my
    heading, with no width or yaw, and it is tested whether or not that leg
    HAS a zebra; the model overlaps the rotated 460 x 180 footprint with the
    seven zebra boxes that exist (R.CROSSINGS: all four legs of (4100, 4100),
    three of (4100, -4100), none at the other eleven junctions). On a leg
    with a zebra the two agree on every stop a hold can cause (the test below
    holds them equal on all four approaches of (4100, 4100)): a car with a
    NextJ (centre 1100-3600 cm out, not in a box) is on its straight
    approach, where no other zebra box reaches its body - NextJ's cross-road
    zebras lie 750 cm or more to its side (lane 350 from the centre line, box
    300 deep, car 90 half wide) and the previous junction's exit zebra over
    3500 cm behind (junctions 8200 apart). On a leg without one the DSL
    counts a late box hold the model does not (pinned by the test at
    (12300, 12300)). A 30 min model run (10 fps, seed 1) had no stopped,
    binding hold inside the band on any leg, so both read 0 there; a
    Simulate that counts ZebraWait needs its junction checked against
    R.CROSSINGS before it is read as a car on a zebra;
  - the model takes `why` from a hold only when its target is STRICTLY below
    the crossing stop; the DSL's `Target <= StopCm` also counts a tie;
  - the DSL counts at UpdateEffSpeed, every other tick, on the position and
    CurSpeed the previous DriveTick left; the model adds dt every frame after
    all cars moved. ZebraWait x 2 x frame time is comparable with
    zebra_wait_s; zebra_wait_events has no DSL counterpart.
tests/test_drive_signals_dsl.py runs the rendered junction block: the
committed late hold counts and the band equals the model's _on_zebra on the
approaches of (4100, 4100). The same run of the old graph counted 0 at every
distance from 1105 to 3600 cm.

The legacy graph is drive_tick.EFF; `python -m tools.citylife_mcp.ue_rpc
script tools/citylife_mcp/drive_tick.py` restores it.
"""
import json
import re
import sys
from pathlib import Path

from tools import citylife_traffic_model as M
from tools.citylife_mcp import ue_rpc

HERE = Path(__file__).resolve().parent
DSL_FILE = HERE / "eff_signals.dsl"

CAR = "/Game/CityLife/Blueprints/BP_CityCar.BP_CityCar_C"
PED = "/Game/CityLife/Blueprints/BP_CityPed.BP_CityPed_C"
BPP = "/Game/CityLife/Blueprints/BP_CityCar.BP_CityCar"

NEW_VARS = [
    ("NowT", "float", None), ("Ph", "float", None), ("OV", "float", None),
    ("TA2", "float", None), ("OLat", "float", None), ("TmpC", "float", None),
    ("DLine", "float", None), ("Target", "float", None), ("HeldTurn", "float", None),
    ("PrevDL", "float", None), ("BoxStill", "float", None),
    ("Sig", "int", None), ("NJi", "int", None), ("SneakJ", "int", None),
    ("Hold", "int", None), ("PrevNJ", "int", None), ("Sneaks", "int", None),
    ("LateHold", "int", None), ("SigHolds", "int", None), ("ZebraWait", "int", None),
    ("RedViol", "int", None),
    ("CheckJ", "bool", None), ("Give", "bool", None), ("OncGo", "bool", None),
    ("ExitBlk", "bool", None), ("ExitPed", "bool", None), ("Inside", "bool", None),
    ("IsExit", "bool", None), ("InWin", "bool", None), ("Sneaking", "bool", None),
    ("JOut", "Vector", None), ("UOut", "Vector", None),
    ("JExit", "Vector", "ARRAY"), ("JOff", "float", "ARRAY"),
]


def constants() -> dict:
    c = M.dsl_constants()
    S = {k[4:]: v for k, v in c.items() if k.startswith("SIG_")}
    return {
        "BOX": c["BOX_HALF_CM"], "NEG_BOX": -c["BOX_HALF_CM"],
        "JNEAR": c["J_SCAN_NEAR_CM"], "JFAR": c["J_SCAN_FAR_CM"],
        "HALF": S["HALF_CYCLE_S"], "CYCLE": S["CYCLE_S"], "PHASE_PAD": 2.0 * S["CYCLE_S"],
        "GREEN": S["GREEN_S"], "AMBER_END": S["GREEN_S"] + S["YELLOW_S"], "WALK": S["WALK_S"],
        "LANE": c["LANE_HALF_CM"], "ARC_LEADER": c["ARC_LEADER_CM"],
        "EXIT_QUARTER": c["EXIT_QUARTER"], "NEG_EXIT_QUARTER": -c["EXIT_QUARTER"],
        "NEG_ONCOMING_PASSED": -c["ONCOMING_PASSED_CM"], "GIVEWAY_FAR": c["GIVEWAY_FAR_CM"],
        "V_STOPPED": c["V_STOPPED"], "V_SLOW": c["V_SLOW"], "STOP": c["STOP_CM"],
        "TWO_STOP_DECEL": 2.0 * c["STOP_DECEL"], "TWO_MAX_DECEL": 2.0 * c["MAX_DECEL"],
        "SNEAK_SLACK": c["SNEAK_MARGIN_CM"] - c["COMMIT_CM"],
        "EXIT_CLEAR": c["EXIT_CLEAR_CM"], "EXIT_AHEAD": int(c["EXIT_XING_AHEAD_PTS"]),
        "NEG_EXIT_BACK": -c["EXIT_XING_BACK_CM"], "EXIT_NEAR": c["EXIT_XING_NEAR_CM"],
        "EXIT_PED_SPAN": c["EXIT_PED_SPAN_CM"], "COMMIT": c["COMMIT_CM"],
        "NEG_COMMIT": -c["COMMIT_CM"], "SNEAK_HELD": c["SNEAK_HELD_S"],
        "SNEAK_LATEST": c["SNEAK_LATEST_S"], "CAR_HALF_LEN": M.CAR_HALF_LEN_CM,
        "ZEBRA_OUTER": c["ZEBRA_OUTER_CM"], "NEG_RED_TOL": -M.RED_TOL_CM,
        "ZEBRA_INNER": M.ZEBRA_CENTRE_CM - M.ZEBRA_HALF_DEPTH_CM, "QUEUE": c["QUEUE_CM"],
    }


def _num(v):
    return str(int(v)) if isinstance(v, int) else ("%.1f" % v if float(v) == round(float(v), 1)
                                                    else "%.4f" % v)


def render(src: str = None) -> str:
    src = src if src is not None else DSL_FILE.read_text(encoding="utf-8")
    k = constants()
    out = re.sub(r"@([A-Z_]+)@", lambda m: _num(k[m.group(1)]), src)
    out = re.sub(r"\{([A-Za-z0-9_]+)\}", lambda m: "(Variables|Default|Get%s)" % m.group(1), out)
    return out.replace("__CAR__", CAR).replace("__PED__", PED)


def check_parens(src: str) -> None:
    depth = 0
    for i, ch in enumerate(src):
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth < 0:
                raise ValueError(f"unbalanced ')' at char {i}")
    if depth:
        raise ValueError(f"{depth} unclosed '('")


EDITOR = r'''
import json

BT = "editor_toolset.toolsets.blueprint.BlueprintTools."
BPP = "__BPP__"
BP = {"refPath": BPP}
NEW_VARS = json.loads(__VARS__)
EFF = json.loads(__EFF__)


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    have = [v if isinstance(v, str) else v["name"] for v in T(BT + "list_variables", blueprint=BP)]
    added = []
    for name, typ, cont in NEW_VARS:
        if name in have:
            continue
        kw = {"blueprint": BP, "name": name, "type_name": typ}
        if cont:
            kw["container_type"] = cont
        T(BT + "add_variable", **kw)
        added.append(name)
    for name, _, cont in NEW_VARS:
        if cont:
            T(BT + "set_variable_instance_editable", blueprint=BP, variable_name=name,
              instance_editable=True)
    graphs = [x["refPath"].split(":")[-1] for x in T(BT + "list_graphs", blueprint=BP)]
    if "SpeedOf" not in graphs:
        gs = T(BT + "add_function_graph", blueprint=BP, graph_name="SpeedOf")
        T(BT + "add_function_param", graph=gs, param_name="Speed", param_type="float",
          input_param=False)
    T(BT + "write_graph_dsl", graph={"refPath": BPP + ":SpeedOf"},
      code="(fn SpeedOf () (return (Variables|Default|GetCurSpeed)))")
    gr = {"refPath": BPP + ":UpdateEffSpeed"}
    T(BT + "write_graph_dsl", graph=gr, code=EFF)
    back = T(BT + "read_graph_dsl", graph=gr)
    return {"added": added, "dsl_len": len(back), "compile": T(BT + "compile_blueprint", blueprint=BP),
            "has_sig": "GetSig" in back, "has_speed_of": "SpeedOf" in back}
'''


def main():
    eff = render()
    check_parens(eff)
    if "--render" in sys.argv:
        print(eff)
        return
    code = (EDITOR.replace("__BPP__", BPP)
            .replace("__VARS__", json.dumps(json.dumps(NEW_VARS)))
            .replace("__EFF__", json.dumps(json.dumps(eff))))
    print(json.dumps(ue_rpc.run_script(code), indent=1))


if __name__ == "__main__":
    main()
