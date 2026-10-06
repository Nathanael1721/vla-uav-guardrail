"""BP_CityPed: walk a route, wait at the kerb, cross on the walk phase with a gap.

    python -m tools.citylife_mcp.ped_walk             # write the graph + all 40 routes
    python -m tools.citylife_mcp.ped_walk --dry-run   # the plan only
    python -m tools.citylife_mcp.ped_walk --roam      # put the figures back on Roam

WHY. The 40 figures ROAMED (tools/citylife_mcp/ped_roam_v1.dsl is the graph
as found): 14 m random hops, back home past 20 m, a pause at any point -
including on a zebra - and no look at the cars. From the air that reads as
people pacing on the spot, and on a crossing it froze a junction for 33 s.
tools/citylife_peds.py builds what replaces it and measured it in Python
first (300 s, 40 figures, 18 cars: WalkViol 0, GapViol 0, ZebraIdleS 0,
CarOverlap 0): closed tours on a pavement graph whose only road-crossing
edges are the zebras, and a WALK / WAIT / CROSS rule.

WHAT IT WRITES

  per figure (tag Ped_00..Ped_39): Route (x, y, z = node kind: 0 pavement,
      1 kerb-wait, 2 crossing exit), WOff (per point; at an exit: the start of
      the green that walks WITH that crossing, citylife_signals), Idx 0,
      bUseNavMesh false (so BeginPlay starts no Roam timer), and the figure
      moved onto its route's first point (its own height kept);
  BP_CityPed EventTick, the route follower it already had (AddMovementInput
      toward Route[Idx], 80 cm arrival) grown into a state machine:
        0 WALK   toward the next point; at a pavement node a 10 % chance of a
                 2-6 s pause; at a kerb-wait node -> 2; at an exit -> 0.
        1 PAUSE  until NextMoveTime.
        2 WAIT   at the kerb until the walk phase (the first 8 s of the
                 parallel green, the same arithmetic as the lamps) AND no car
                 moving above 50 cm/s would reach the zebra within 4 s or is
                 on it - then 3. Never on the zebra: the kerb node is 200 cm
                 back from the carriageway.
        3 CROSS  to the exit without stopping (no pause).
      STUCK, in WALK or CROSS: under 10 cm/s for 1.5 s. Within 300 cm of the
      point it is heading for, that IS arriving (a crowd at a kerb or a node
      stands on the spot itself). Anywhere else the figure keeps its target
      and steps aside for 1 s - a little forward, mostly to one side, then
      the other, alternately - and tries again; after 6 such detours without
      arriving it is set down on the point (SetActorLocation) and arrives.
      Idx only ever moves at the point it names.
      The rule it replaces SKIPPED to the next point when stuck short of a
      pavement node, and went to WAIT where it stood short of a kerb node.
      Two figures meeting head-on on the same node line stall; the skip
      then aimed at a point round a building corner, stalled again, skipped
      again, and so on until a kerb node sent the figure to wait - 55 m from
      its zebra - and then across, through a building, for ever (Simulate
      2026-09-30, t = 1933 s: 21 of 40 figures frozen in CROSS far from any
      zebra, median move between reads 1.8 m). Detours and set-downs are
      counted (Detours, Teleports) for verify_peds.py.
      Pavement nodes are also moved per figure by -25..+25 cm away from the
      road (LANE_JITTER_CM): with every figure on the same line, two walking
      opposite ways met exactly head-on, the one collision capsules cannot
      slide out of.      Spd, the figure's XY speed, is written every tick for verify_peds.py.

THE CAR TEST is citylife_peds.time_to_zebra read off the car's own path,
which a figure CAN see: every BP_CityCar carries Crossings (x, y, z = the
path index nearest the zebra; apply_routes.py), Idx (the path point it
drives toward) and NPts. The zebra is the midpoint of the kerb and exit
nodes (the CROSSINGS point exactly, for all five usable crossings), not of
the figure and the exit: a figure that stopped short of its kerb node would
move it. For a car above 50 cm/s:

  its Crossings has an entry within 100 cm of that point -> its path drives
      over this zebra. CAh = (CIdx - Idx + NPts) % NPts points ahead, 150 cm
      a point (the car's own crossing rule's figure; the loops measure
      145.8-156.8). It blocks when CAh >= NPts - 4 (passed by at most four
      points: its body is still on the zebra, as the car's own rule counts
      it) or (150 CAh - 700) / v < 4 s. 700 is the model's reach (550 = zebra
      half depth 300 + half a 500 cm car) plus one point: 150 CAh overstates
      the distance by up to 68 cm (the crossing projects up to 50 cm short of
      its CIdx, and a 145.8 cm arc point is not 150), and with 550 alone the
      test's sweep let through cars 3.94-4.0 s from the zebra;
  none -> its path misses this zebra, and it blocks only while its 500 x 250
      footprint overlaps the zebra box (time_to_zebra's 0-or-inf for such a
      car), tested on the zebra's two axes with its heading: a superset of
      the exact overlap, never less.

The heading rule this replaces judged a car by where it pointed: a car
turning INTO the zebra pointed along the road until the arc, and on loop A's
turn onto (4100, -3000) at Car_10's 3.2 m/s it first counted one 0.4 s from
the zebra (1.1 s at 5.6 m/s on loop B at (4100, 3000)). `car_blocks` below
is the same arithmetic in Python; tests/test_ped_walk_plan.py holds it to
block every car, every 25 cm of every loop, that time_to_zebra puts under
4 s from every usable crossing, and runs the rendered graph against it.

Rollback: --roam rewrites the found graph and sets bUseNavMesh true.
"""
import json
import math
import sys
from pathlib import Path

from tools import citylife_peds as P
from tools import citylife_routes as R
from tools import citylife_signals as S
from tools.citylife_mcp import ue_rpc

HERE = Path(__file__).resolve().parent
PEDP = "/Game/CityLife/Blueprints/BP_CityPed.BP_CityPed"
CAR = "/Game/CityLife/Blueprints/BP_CityCar.BP_CityCar_C"
N_PEDS = 40

# The car test (module docstring, THE CAR TEST).
MATCH_CM = 100.0                                   # a Crossings entry this near the zebra centre is it
PT_CM = R.SPACING_CM                               # 150 cm a path point, as the car's own rule counts
PASSED_PTS = 4                                     # passed by up to this many points: still on the zebra
REACH_CM = P.ZEBRA_HALF_CM + 0.5 * P.CAR_LEN_CM    # 550: time_to_zebra's reach
XBAND_CM = REACH_CM + PT_CM                        # 700: plus one point for what 150 CAh overstates

# The stuck rule (module docstring, STUCK).
STILL_CMS = P.STILL_PED_CMS                        # 10: under this a figure is not moving
STUCK_S = 1.5                                      # this long still: stuck
NEAR_CM = 300.0                                    # stuck this near its point: arrived
DETOUR_S = 1.0                                     # a sidestep lasts this long
DETOUR_FWD = 0.3                                   # its forward share (the side's is 1)
TELEPORT_N = 6                                     # detours before it is set down on the point
LANE_JITTER_CM = 25.0                              # pavement nodes, +- this, away from the road

NEW_VARS = [("WOff", "float", "ARRAY"), ("PState", "int", None), ("StuckT", "float", None),
            ("NowT", "float", None), ("XC", "Vector", None), ("XW", "Vector", None),
            ("XR", "Vector", None), ("CanGo", "bool", None), ("WalkOn", "bool", None),
            ("CRel", "Vector", None), ("CF", "Vector", None), ("CV", "float", None),
            ("Crossings", "int", None), ("KerbWaitMax", "float", None),
            ("WaitT", "float", None), ("CAh", "int", None), ("CNp", "int", None),
            ("CHas", "bool", None), ("Spd", "float", None), ("NKind", "int", None),
            ("Arr", "bool", None), ("StuckN", "int", None), ("DetT", "float", None),
            ("DetS", "float", None), ("Detours", "int", None), ("Teleports", "int", None)]


def walk_offset(kx, ky, ex, ey) -> float:
    """WOff for the crossing from kerb (kx, ky) to exit (ex, ey): the start of
    the green of the axis the walkers move ALONG (citylife_signals.walk)."""
    cx, cy = 0.5 * (kx + ex), 0.5 * (ky + ey)
    jx, jy = S.nearest_junction(cx, cy)
    road = S.crossing_road_axis(cx, cy, jx, jy)
    par = S.other(road)
    return S.offset(jx, jy) + (S.HALF_CYCLE_S if par == S.EW else 0.0)


def _away(v: float) -> int:
    """+1 / -1 when coordinate v lies on a pavement line (PAVEMENT_OFF_CM, or
    the 1000 cm of a node beside a kerb, from a grid road's centre): the side
    away from that road. 0 when it does not."""
    g = P.GRID_ORIGIN_CM + round((v - P.GRID_ORIGIN_CM) / R.GRID_CM) * R.GRID_CM
    d = v - g
    return (1 if d > 0 else -1) if abs(abs(d) - P.PAVEMENT_OFF_CM) < 60.0 else 0


def lane_jitter(k: int) -> float:
    """Figure k's pavement offset, cm, -LANE_JITTER_CM..+LANE_JITTER_CM in 5 steps."""
    return -LANE_JITTER_CM + 2.0 * LANE_JITTER_CM * (k % 5) / 4.0


def jitter_route(r, j: float):
    """Pavement nodes moved j cm away from their road (both coordinates at a
    corner); kerb-wait and exit nodes kept: they fix the zebra."""
    out = []
    for x, y, kind in r:
        if int(kind) == P.KIND_PAVEMENT:
            x, y = x + _away(x) * j, y + _away(y) * j
        out.append((x, y, kind))
    return out


def plan(n=N_PEDS):
    routes = [jitter_route(r, lane_jitter(k)) for k, r in enumerate(P.make_routes(n_peds=n))]
    out = []
    for k, r in enumerate(routes):
        woff = []
        for i, (x, y, kind) in enumerate(r):
            if int(kind) == P.KIND_CROSS_EXIT:
                px, py, _ = r[i - 1]
                woff.append(round(walk_offset(px, py, x, y), 3))
            else:
                woff.append(0.0)
        out.append({"tag": "Ped_%02d" % k, "route": [[round(x, 1), round(y, 1), float(kind)] for x, y, kind in r],
                    "woff": woff})
    return out


def is_walk(t: float, woff: float) -> bool:
    """The Blueprint's test, in Python (C-style fmod)."""
    return math.fmod(t - woff + 2 * S.CYCLE_S, S.CYCLE_S) < S.WALK_S


def car_blocks(xc, xw, car_xy, car_fwd, cv, crossings, idx, npts) -> bool:
    """The Blueprint's car test for ONE car, in Python: True if it stops the
    figure at the kerb. xc zebra centre, xw unit walking direction; the car's
    position, unit heading, CurSpeed, Crossings rows (x, y, path index), Idx
    and NPts. The Blueprint's `>` against MOVING and C-style % are kept."""
    if not cv > P.MOVING_CAR_CMS:
        return False
    has, block = False, False
    for x, y, ci in crossings:
        if math.hypot(x - xc[0], y - xc[1]) < MATCH_CM:
            has = True
            cah = (int(ci) - idx + npts) % npts
            if cah >= npts - PASSED_PTS or (PT_CM * cah - XBAND_CM) / cv < P.GAP_S:
                block = True
    if has:
        return block
    xr = (-xw[1], xw[0])
    rx, ry = car_xy[0] - xc[0], car_xy[1] - xc[1]
    f_r = abs(car_fwd[0] * xr[0] + car_fwd[1] * xr[1])
    f_w = abs(car_fwd[0] * xw[0] + car_fwd[1] * xw[1])
    hl, hw = 0.5 * P.CAR_LEN_CM, 0.5 * P.CAR_WID_CM
    # <=: touching counts, as in car_on_zebra's separating-axis test
    return (abs(rx * xr[0] + ry * xr[1]) <= P.ZEBRA_HALF_CM + hl * f_r + hw * f_w
            and abs(rx * xw[0] + ry * xw[1]) <= R.CARRIAGEWAY_HALF_CM + hl * f_w + hw * f_r)


V = "Variables|Default|"


def _g(n):
    return "(%sGet%s)" % (V, n)


class _G(dict):
    def __missing__(self, k):
        return _g(k)


TICK = """(event EventBeginPlay
  (Variables|Default|SetHomeLocation (Transformation|GetActorLocation))
  (Variables|Default|SetPState 0)
  (if (Variables|Default|GetUseNavMesh)
    (Utilities|Time|SetTimerbyFunctionName self "Roam" 1.0 true)))

(event Collision|EventActorBeginOverlap (OtherActor))

(event EventTick (DeltaSeconds)
  (Variables|Default|SetSpd (Math|Vector|VectorLengthXY (Transformation|GetVelocity)))
  (if (not (Variables|Default|GetUseNavMesh))
    (bind _n (Utilities|Array|Length {Route}))
    (if (> _n 1)
      (Variables|Default|SetNowT (Utilities|Time|GetGameTimeInSeconds))
      (bind _me (Transformation|GetActorLocation))
      (bind _tgt (Utilities|Array|Get(acopy) {Route} {Idx}))
      (bind _d (Math|Vector|MakeVector (- (.x _tgt) (.x _me)) (- (.y _tgt) (.y _me)) 0.0))
      (if (== {PState} 1)
        (if (>= {NowT} {NextMoveTime})
          (Variables|Default|SetPState 0))
        (elif (== {PState} 2)
          (Variables|Default|SetWaitT (+ {WaitT} DeltaSeconds))
          (Variables|Default|SetKerbWaitMax (Math|Float|Max(Float) {KerbWaitMax} {WaitT}))
          (Variables|Default|SetWalkOn (< (Math|Float|%(Float) (+ (- {NowT} (Utilities|Array|Get(acopy) {WOff} {Idx})) @PAD@) @CYCLE@) @WALK@))
          (if {WalkOn}
            (bind _kb (Utilities|Array|Get(acopy) {Route} (Math|Integer|%(Integer) (+ {Idx} (- _n 1)) _n)))
            (Variables|Default|SetXC (Math|Vector|MakeVector (* 0.5 (+ (.x _tgt) (.x _kb))) (* 0.5 (+ (.y _tgt) (.y _kb))) 0.0))
            (Variables|Default|SetXW (Math|Vector|Normalize (Math|Vector|MakeVector (- (.x _tgt) (.x _kb)) (- (.y _tgt) (.y _kb)) 0.0)))
            (Variables|Default|SetXR (Math|Vector|MakeVector (* -1.0 (.y {XW})) (.x {XW}) 0.0))
            (Variables|Default|SetCanGo true)
            (bind _cars (Actor|GetAllActorsOfClass "__CAR__"))
            (for _car _cars
              (bind _cc (Utilities|Casting|CastToBP_CityCar :Object _car)
                (:then
                  (Variables|Default|SetCV (Class|BPCityCar|GetCurSpeed :self _cc))
                  (if (> {CV} @MOVING@)
                    (Variables|Default|SetCHas false)
                    (Variables|Default|SetCNp (Class|BPCityCar|GetNPts :self _cc))
                    (bind _cx (Class|BPCityCar|GetCrossings :self _cc))
                    (for _zc _cx
                      (if (< (Math|Vector|VectorLengthXY (Math|Vector|MakeVector (- (.x _zc) (.x {XC})) (- (.y _zc) (.y {XC})) 0.0)) @MATCH@)
                        (Variables|Default|SetCHas true)
                        (Variables|Default|SetCAh (Math|Integer|%(Integer) (+ (- (Math|Float|Truncate (.z _zc)) (Class|BPCityCar|GetIdx :self _cc)) {CNp}) {CNp}))
                        (if (or (>= {CAh} (- {CNp} @PASSED_PTS@))
                                (< (/ (- (* @PT@ {CAh}) @XBAND@) {CV}) @GAP@))
                          (Variables|Default|SetCanGo false))))
                    (if (not {CHas})
                      (bind _cl (Transformation|GetActorLocation :self _car))
                      (Variables|Default|SetCRel (Math|Vector|MakeVector (- (.x _cl) (.x {XC})) (- (.y _cl) (.y {XC})) 0.0))
                      (Variables|Default|SetCF (Transformation|GetActorForwardVector :self _car))
                      (if (and (<= (Math|Float|Absolute(Float) (Math|Vector|DotProduct {CRel} {XR}))
                                  (+ @ZDEPTH@ (+ (* @HALF_LEN@ (Math|Float|Absolute(Float) (Math|Vector|DotProduct {CF} {XR})))
                                                 (* @HALF_WID@ (Math|Float|Absolute(Float) (Math|Vector|DotProduct {CF} {XW}))))))
                               (<= (Math|Float|Absolute(Float) (Math|Vector|DotProduct {CRel} {XW}))
                                  (+ @ZSPAN@ (+ (* @HALF_LEN@ (Math|Float|Absolute(Float) (Math|Vector|DotProduct {CF} {XW})))
                                                (* @HALF_WID@ (Math|Float|Absolute(Float) (Math|Vector|DotProduct {CF} {XR})))))))
                        (Variables|Default|SetCanGo false)))))
                (:CastFailed)))
            (if {CanGo}
              (Variables|Default|SetPState 3)
              (Variables|Default|SetCrossings (+ {Crossings} 1))
              (Variables|Default|SetWaitT 0.0)))
          (else
            ; STUCK (module docstring): still for @STUCK_S@ s near the point
            ; is arriving; anywhere else a detour, the target kept.
            (if (< {Spd} @STILL@)
              (Variables|Default|SetStuckT (+ {StuckT} DeltaSeconds))
              (else
                (Variables|Default|SetStuckT 0.0)))
            (Variables|Default|SetArr (or (< (Math|Vector|VectorLengthXY _d) @ARRIVE@)
                                          (and (> {StuckT} @STUCK_S@)
                                               (< (Math|Vector|VectorLengthXY _d) @NEAR@))))
            (if (and (not {Arr}) (> {StuckT} @STUCK_S@))
              (Variables|Default|SetStuckT 0.0)
              (Variables|Default|SetStuckN (+ {StuckN} 1))
              (Variables|Default|SetDetours (+ {Detours} 1))
              (Variables|Default|SetDetT (+ {NowT} @DETOUR_S@))
              (if (== (Math|Integer|%(Integer) {StuckN} 2) 1)
                (Variables|Default|SetDetS 1.0)
                (else
                  (Variables|Default|SetDetS -1.0)))
              (if (>= {StuckN} @TELEPORT_N@)
                (Transformation|SetActorLocation :NewLocation (Math|Vector|MakeVector (.x _tgt) (.y _tgt) (.z _me)))
                (Variables|Default|SetTeleports (+ {Teleports} 1))
                (Variables|Default|SetArr true)))
            (if {Arr}
              ; LATCHED before Idx moves: _tgt is a pure Get(Route, Idx), and
              ; a pure node is re-evaluated at every use - read after SetIdx
              ; it is the NEXT point's kind. That sent a figure arriving at
              ; its kerb across the road in WALK, with no walk-phase and no
              ; gap test (Simulate, 2026-09-30: figures mid-carriageway).
              (Variables|Default|SetNKind (Math|Float|Truncate (.z _tgt)))
              (Variables|Default|SetIdx (Math|Integer|%(Integer) (+ {Idx} 1) _n))
              (Variables|Default|SetStuckT 0.0)
              (Variables|Default|SetStuckN 0)
              (Variables|Default|SetDetT 0.0)
              (if (== {NKind} 1)
                (Variables|Default|SetPState 2)
                (Variables|Default|SetWaitT 0.0)
                (elif (== {NKind} 2)
                  (Variables|Default|SetPState 0)
                  (else
                    (Variables|Default|SetPState 0)
                    (if (Math|Random|RandomBoolWithWeight @PAUSE_P@)
                      (Variables|Default|SetNextMoveTime (+ {NowT} (Math|Random|RandomFloatInRange @PAUSE_LO@ @PAUSE_HI@)))
                      (Variables|Default|SetPState 1)))))
              (else
                (if (< {NowT} {DetT})
                  (Pawn|Input|AddMovementInput :WorldDirection (Math|Vector|Normalize (Math|Vector|MakeVector (- (* @DETOUR_FWD@ (.x _d)) (* {DetS} (.y _d))) (+ (* @DETOUR_FWD@ (.y _d)) (* {DetS} (.x _d))) 0.0)) :ScaleValue 1.0)
                  (else
                    (Pawn|Input|AddMovementInput :WorldDirection (Math|Vector|Normalize _d) :ScaleValue 1.0)))))))))))
"""


def render(src: str = TICK) -> str:
    rep = {"PAD": 2 * S.CYCLE_S, "CYCLE": S.CYCLE_S, "WALK": S.WALK_S,
           "MOVING": P.MOVING_CAR_CMS, "GAP": P.GAP_S, "ARRIVE": 80.0,
           "MATCH": MATCH_CM, "PT": PT_CM, "XBAND": XBAND_CM,
           "PASSED_PTS": PASSED_PTS,                        # int: compared with an int
           "ZDEPTH": P.ZEBRA_HALF_CM, "ZSPAN": R.CARRIAGEWAY_HALF_CM,
           "HALF_LEN": 0.5 * P.CAR_LEN_CM, "HALF_WID": 0.5 * P.CAR_WID_CM,
           "PAUSE_P": P.PAUSE_P, "PAUSE_LO": P.PAUSE_S[0], "PAUSE_HI": P.PAUSE_S[1],
           "STILL": STILL_CMS, "STUCK_S": STUCK_S, "NEAR": NEAR_CM, "DETOUR_S": DETOUR_S,
           "DETOUR_FWD": DETOUR_FWD, "TELEPORT_N": TELEPORT_N}      # int: compared with an int
    out = src.format_map(_G())
    for k, v in rep.items():
        out = out.replace("@%s@" % k, str(v) if isinstance(v, int) else "%.3f" % v)
    return out.replace("__CAR__", CAR)


EDITOR = r'''
import json

ST = "editor_toolset.toolsets.scene.SceneTools."
AC = "editor_toolset.toolsets.actor.ActorTools."
OT = "editor_toolset.toolsets.object.ObjectTools."
BT = "editor_toolset.toolsets.blueprint.BlueprintTools."
BPP = "__PEDP__"
BP = {"refPath": BPP}
NEW_VARS = json.loads(__VARS__)
PLAN = json.loads(__PLAN__)
GRAPH = json.loads(__GRAPH__)
ROAM = __ROAM__


def T(_tool, **kw):
    return execute_tool(_tool, json.dumps(kw))["returnValue"]


def run():
    out = {}
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
    for name in ("WOff", "Route", "Idx", "bUseNavMesh"):
        T(BT + "set_variable_instance_editable", blueprint=BP, variable_name=name,
          instance_editable=True)
    out["added"] = added
    T(BT + "write_graph_dsl", graph={"refPath": BPP + ":EventGraph"}, code=GRAPH)
    out["compile"] = T(BT + "compile_blueprint", blueprint=BP)
    by_tag = {}
    for a in T(ST + "find_actors", name="", tag="", collision_channels=[]):
        if "UEDPIE" in a["refPath"] or "CityPed" not in a["refPath"]:
            continue
        for t in T(AC + "get_tags", actor=a):
            if t.startswith("Ped_"):
                by_tag[t] = a
    done = {}
    for p in PLAN:
        a = by_tag[p["tag"]] if p["tag"] in by_tag else None
        if a is None:
            done[p["tag"]] = "MISSING"
            continue
        if ROAM:
            T(OT + "set_properties", instance=a, values=json.dumps({"bUseNavMesh": True}))
            done[p["tag"]] = "roam"
            continue
        full = {"route": [{"x": q[0], "y": q[1], "z": q[2]} for q in p["route"]],
                "wOff": p["woff"]}
        T(OT + "set_properties", instance=a, values=json.dumps({"route": [], "wOff": []}))
        T(OT + "set_properties", instance=a, values=json.dumps(full))
        T(OT + "set_properties", instance=a, values=json.dumps({"idx": 0, "bUseNavMesh": False}))
        xf = T(AC + "get_actor_transform", actor=a)
        T(AC + "set_actor_transform", actor=a,
          xform={"location": {"x": p["route"][0][0], "y": p["route"][0][1],
                              "z": xf["location"]["z"]}},
          worldspace=True)
        back = json.loads(T(OT + "get_properties", instance=a, properties=["route", "wOff", "bUseNavMesh"]))
        done[p["tag"]] = [len(back["route"]), len(back["wOff"]), back["bUseNavMesh"]]
    out["peds"] = done
    return out
'''


def main():
    roam = "--roam" in sys.argv
    plan_ = plan()
    n_x = sum(1 for p in plan_ for q in p["route"] if int(q[2]) == P.KIND_CROSS_EXIT)
    print(f"{len(plan_)} routes, {n_x} crossing legs")
    if "--dry-run" in sys.argv:
        for p in plan_[:3]:
            print(p["tag"], len(p["route"]), p["route"][:4], [w for w in p["woff"] if w][:4])
        return
    graph = (HERE / "ped_roam_v1.dsl").read_text(encoding="utf-8") if roam else render()
    if roam:
        graph = graph.split("; ---- EventGraph", 1)[1].split("\n", 1)[1]
    code = (EDITOR.replace("__PEDP__", PEDP)
            .replace("__VARS__", json.dumps(json.dumps(NEW_VARS)))
            .replace("__PLAN__", json.dumps(json.dumps(plan_)))
            .replace("__GRAPH__", json.dumps(json.dumps(graph)))
            .replace("__ROAM__", "True" if roam else "False"))
    res = ue_rpc.run_script(code)
    missing = [k for k, v in res["peds"].items() if v == "MISSING"]
    print(json.dumps({k: v for k, v in res.items() if k != "peds"}, indent=1))
    print("peds written:", sum(1 for v in res["peds"].values() if v != "MISSING"),
          "missing:", missing)


if __name__ == "__main__":
    main()
