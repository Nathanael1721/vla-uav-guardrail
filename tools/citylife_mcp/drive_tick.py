"""BP_CityCar: follow a dense path at v/R, and yield at junctions and crossings.

Run inside the editor (ProgrammaticToolset) with

    python -m tools.citylife_mcp.ue_rpc script tools/citylife_mcp/drive_tick.py

WHAT IT WRITES

`DriveTick` is `tools/citylife_routes.py::follow_step`, node for node: the car
steers by the path's own curvature plus a small correction of its lateral and
heading error, so on an arc its yaw rate is v/R and it does not pivot at a
waypoint. The path (`Route`, `RouteSpeed`, `RouteKappa`) is written per car by
`apply_routes.py`; this script only builds the logic.

`UpdateEffSpeed` runs every other tick (it walks every car in the world) and
now decides three speeds:

  queue      a car ahead in my direction (in my corridor, or within 9 m when
             a curve puts it off-axis): stop 6.5 m behind it, centre to centre,
             on a 2.5 m/s^2 ramp. The old rule never went below 60 cm/s and so
             crept into a stopped car. A car ahead that is NOT going my way
             counts only inside my 2.5 m corridor: the 9 m allowance once
             caught oncoming cars in the other lane and slowed the subject to
             0.6 m/s.
  junction   my next junction (from `Junctions`; z = 1 where I must give
             way: my path crosses another loop's there and I am the one
             turning across). I wait at the box edge while any non-parallel
             car is inside the box. Where I give way, I also wait for any
             non-parallel car within 20 m of the box. A car already inside the
             box never waits, so the rule cannot deadlock: the waiting car's
             condition always clears.
  crossing   a pedestrian crossing on MY PATH (`Crossings`, z = its path
             index) 3-17 path points ahead (4.5-25 m) with a pedestrian on the
             carriageway part of it: stop 6.5 m short of its centre. Measured
             along the path, because a car about to turn must stop for the
             crossing round the corner, not the one straight ahead of it. The
             pedestrian scan runs only then, so it costs nothing most ticks.
             A pedestrian standing still counts only for the first 6 s of the
             wait: roaming figures pick random nav points, the crossings are
             nav, and one that idles or sticks there froze a junction for 33 s.
             Closer than 3 points (the car is already on the zebra) the stop
             line is behind it, and it used to drive on whoever stepped out:
             5 half-rate ticks in 4 min of Simulate (2026-09-23). Now a
             pedestrian in the car's own path - 3 m either side of its centre
             line, from 1 m behind its centre to 9 m ahead - stops it where it
             is (StopCm 0), under the same 6 s rule for one standing still.
             A pedestrian elsewhere on the zebra does not: the car is on the
             crossing already, and stopping there blocks it for everyone.
             "On the zebra" runs from 3 points before the crossing's path point
             to 4 after it (CAhead >= NPts-4): the zebra reaches 3 m past its
             centre line, and CAhead wraps to NPts-1 the moment the car's
             centre passes that point - which cancelled a stop half-way through
             its braking (found in review, 2026-09-24).
             THE 6 s RULE HAS ITS OWN CLOCK, StillRun: time spent stopped
             while a standing pedestrian is in the way, reset only when none
             is. It used StopRun, which DriveTick zeroes the moment the car
             reaches 10 cm/s - so the release after 6 s re-armed after about
             a centimetre of creep and held the car for as long as the figure
             idled (found in review, 2026-09-24). The clock belongs to the
             crossing that armed it (StillC): a release earned at one zebra
             used to carry over to the next, 7-16 path points on, and the car
             drove over an idle figure there without stopping (same review).
             "Armed it" means the NEAREST crossing with a standing figure
             (CA2: path points ahead, negative once past it), and a figure
             anywhere in the crossing's box counts, in both windows: latching
             the last crossing in array order re-armed the near one's stop
             from the far one's figure, and the narrower path box on the
             zebra threw away the 6 s already waited (review round 3).
             Crossings AHEAD win the latch (a crossing behind the car ranks
             after every one ahead: toggling figures behind a car waiting at
             the next line flipped the latch and held it for good), and a
             release exempts only the crossing it was earned at (StillDone,
             forgotten once that crossing leaves the window) - with crossings
             7 points apart the release at one still carried to the next
             (review round 4).
             PedViol and PedPassStill count only at CAhead 0-2, the window
             they had when the baseline of 5 was measured.

StopCm and GapCm are measured every other tick and then shrunk by the distance
driven in every DriveTick sub-step, so a slow frame rate does not let a car run
past a stop line or into the car ahead before the next measurement.

DriveTick cuts a long frame into sub-steps of at most 50 ms (see
citylife_routes.follow): an editor hitch of 0.5 s otherwise drives the car 1.8 m
on the wrong curvature at the end of an arc.

Every intermediate is latched into a member variable by an impure Set: a pure
node chain is re-evaluated each time it is read, and GetActorLocation read
after SetActorLocation would otherwise return the NEW position.

Verification counters, readable in PIE: EyMax (worst lateral error, cm),
AlatMax (worst v^2 kappa, cm/s^2), MinEver (closest approach to another car),
YieldTicks, PedTicks, StopRunMax (longest stand-still, s), PedViol (half-rate
ticks a car drove over a crossing at > 50 cm/s with a MOVING pedestrian on the
zebra - must stay 0) and PedPassStill (the same with a pedestrian standing still,
which the 6 s rule allows). Since 2026-09-24 PedViol is CONTEXT, not a gate:
it still counts a moving pedestrian ANYWHERE on the zebra, as it did when the 5
were measured, and a car already on the crossing now drives on past one in the
other lane by design. PedClose counts half-rate ticks a car was above
50 cm/s with a MOVING pedestrian in its own path within 4 m of its centre
(about 2 m past its bumper). It is not a pass/fail gate: a figure that steps
out inside the car's braking distance (1.3 m from 3.2 m/s) counts too, while
the car brakes as hard as it can - read it beside EStops. EStops counts half-rate ticks of the
on-the-zebra emergency stop and PedCorr those still above 50 cm/s (braking,
0.8 s from 3.2 m/s); each counts once per tick, however many figures.
"""
import json

BT = "editor_toolset.toolsets.blueprint.BlueprintTools."
BPP = "/Game/CityLife/Blueprints/BP_CityCar.BP_CityCar"
BP = {"refPath": BPP}
CAR = "/Game/CityLife/Blueprints/BP_CityCar.BP_CityCar_C"
PED = "/Game/CityLife/Blueprints/BP_CityPed.BP_CityPed_C"

NEW_VARS = [
    ("RouteSpeed", "float", "ARRAY"), ("RouteKappa", "float", "ARRAY"),
    ("Junctions", "Vector", "ARRAY"), ("Crossings", "Vector", "ARRAY"),
    ("Dt", "float", None), ("Ey", "float", None), ("Eps", "float", None),
    ("Kff", "float", None), ("SegLen", "float", None), ("Kappa", "float", None),
    ("StopCm", "float", None), ("JEdge", "float", None), ("TmpB", "float", None),
    ("Par", "float", None), ("EyMax", "float", None), ("AlatMax", "float", None),
    ("StopRun", "float", None), ("StopRunMax", "float", None),
    ("Pos", "Vector", None), ("Fwd", "Vector", None), ("Rgt", "Vector", None),
    ("SegA", "Vector", None), ("SegT", "Vector", None), ("NextJ", "Vector", None),
    ("TmpE", "Vector", None), ("OFwd", "Vector", None),
    ("NPts", "int", None), ("IPrev", "int", None), ("YieldTicks", "int", None),
    ("CIdx", "int", None), ("CAhead", "int", None), ("NSub", "int", None),
    ("SubDt", "float", None), ("PedSpd", "float", None),
    ("PedViol", "int", None), ("PedPassStill", "int", None),
    ("PedTicks", "int", None), ("EStops", "int", None), ("PedCorr", "int", None),
    ("PedClose", "int", None), ("StillRun", "float", None),
    ("StillC", "int", None), ("StillCPrev", "int", None),
    ("CA2", "int", None), ("StillBest", "int", None), ("StillDone", "int", None),
    ("Exempt", "bool", None),
    ("StillSeen", "bool", None), ("EHit", "bool", None), ("CloseHit", "bool", None),
    ("InBox", "bool", None), ("HasJ", "bool", None), ("Conflict", "bool", None),
]

V = "Variables|Default|"


def g(name):
    return "(%sGet%s)" % (V, name)


DRIVE = """(fn DriveTick ()
  (Variables|Default|SetDt (Utilities|Time|GetWorldDeltaSeconds))
  (Variables|Default|SetNPts (Utilities|Array|Length {Route}))
  (if (> {NPts} 2)
    (Variables|Default|SetNSub (Math|Integer|Clamp(Integer) (+ (Math|Float|Truncate (* {Dt} 20.0)) 1) 1 20))
    (Variables|Default|SetSubDt (/ {Dt} {NSub}))
    (for _s (range {NSub})
      (Variables|Default|SetPos (Transformation|GetActorLocation))
      (Variables|Default|SetFwd (Transformation|GetActorForwardVector))
      (for _i (range 4)
        (if (<= (Math|Vector|DotProduct (- (Utilities|Array|Get(acopy) {Route} {Idx}) {Pos}) {Fwd}) 0.0)
          (Variables|Default|SetIdx (Math|Integer|%(Integer) (+ {Idx} 1) {NPts}))))
      (Variables|Default|SetIPrev (Math|Integer|%(Integer) (+ {Idx} (- {NPts} 1)) {NPts}))
      (Variables|Default|SetSegA (Utilities|Array|Get(acopy) {Route} {IPrev}))
      (Variables|Default|SetTmpE (- (Utilities|Array|Get(acopy) {Route} {Idx}) {SegA}))
      (Variables|Default|SetSegLen (Math|Vector|VectorLengthXY {TmpE}))
      (Variables|Default|SetSegT (Math|Vector|Normalize (Math|Vector|MakeVector (.x {TmpE}) (.y {TmpE}) 0.0)))
      (Variables|Default|SetTmpE (- {Pos} {SegA}))
      (Variables|Default|SetEy (- (* (.y {TmpE}) (.x {SegT})) (* (.x {TmpE}) (.y {SegT}))))
      (Variables|Default|SetKff (Utilities|Array|Get(acopy) {RouteKappa} {IPrev}))
      (Variables|Default|SetEps (- (- (* (.y {Fwd}) (.x {SegT})) (* (.x {Fwd}) (.y {SegT})))
                                   (* (- (+ (* (.x {TmpE}) (.x {SegT})) (* (.y {TmpE}) (.y {SegT})))
                                         (* 0.5 {SegLen}))
                                      {Kff})))
      (Variables|Default|SetKappa (Math|Float|Clamp(Float)
        (- (- {Kff} (* 0.000004 {Ey})) (* 0.0036 {Eps})) -0.002 0.002))
      (Variables|Default|SetTargetSpeed (Math|Float|Min(Float)
        (Math|Float|Min(Float) {EffSpeed} (Utilities|Array|Get(acopy) {RouteSpeed} {IPrev}))
        (Utilities|Array|Get(acopy) {RouteSpeed} (Math|Integer|%(Integer) (+ {Idx} 2) {NPts}))))
      (Variables|Default|SetTargetSpeed (Math|Float|Min(Float) {TargetSpeed}
        (Math|Float|Sqrt (* 500.0 (Math|Float|Max(Float) 0.0
          (Math|Float|Min(Float) {StopCm} (- {GapCm} 650.0)))))))
      (Variables|Default|SetCurSpeed (+ {CurSpeed} (Math|Float|Clamp(Float) (- {TargetSpeed} {CurSpeed})
        (* -400.0 {SubDt}) (* 200.0 {SubDt}))))
      (Transformation|SetActorRotation :NewRotation (Math|Rotator|MakeRotator 0.0 0.0
        (+ (.yaw (Transformation|GetActorRotation)) (* 57.2957795 (* (* {CurSpeed} {Kappa}) {SubDt})))))
      (Variables|Default|SetFwd (Transformation|GetActorForwardVector))
      (Transformation|SetActorLocation :NewLocation (+ {Pos} (* {Fwd} (* {CurSpeed} {SubDt}))))
      (Variables|Default|SetSpin (Math|Float|%(Float) (+ {Spin} (* (/ (* {CurSpeed} {SubDt}) {WheelRadius}) 57.2957795)) 360.0))
      (Variables|Default|SetStopCm (- {StopCm} (* {CurSpeed} {SubDt})))
      (Variables|Default|SetGapCm (- {GapCm} (* {CurSpeed} {SubDt})))
      (if (> {Ticks} 240)
        (Variables|Default|SetEyMax (Math|Float|Max(Float) {EyMax} (Math|Float|Absolute(Float) {Ey})))
        (Variables|Default|SetAlatMax (Math|Float|Max(Float) {AlatMax}
          (Math|Float|Absolute(Float) (* (* {CurSpeed} {CurSpeed}) {Kappa}))))))
    (Variables|Default|SetSteer (Math|Float|Clamp(Float) (* 15470.0 {Kappa}) -35.0 35.0))
    (if (> {Ticks} 240)
      (if (< {CurSpeed} 10.0)
        (Variables|Default|SetStopRun (+ {StopRun} {Dt}))
        (Variables|Default|SetStopRunMax (Math|Float|Max(Float) {StopRunMax} {StopRun}))
        (else
          (Variables|Default|SetStopRun 0.0))))))
"""

EFF = """(fn UpdateEffSpeed ()
  (Variables|Default|SetTicks (+ {Ticks} 1))
  (if (== (Math|Integer|%(Integer) {Ticks} 2) 0)
    (Variables|Default|SetDbgMe (Transformation|GetActorLocation))
    (Variables|Default|SetDbgFwd (Transformation|GetActorForwardVector))
    (Variables|Default|SetRgt (Transformation|GetActorRightVector))
    (Variables|Default|SetInBox false)
    (Variables|Default|SetHasJ false)
    (Variables|Default|SetConflict false)
    (Variables|Default|SetJEdge 99999.0)
    (for _j {Junctions}
      (Variables|Default|SetTmpD (- _j {DbgMe}))
      (if (and (< (Math|Float|Absolute(Float) (.x {TmpD})) 1100.0)
               (< (Math|Float|Absolute(Float) (.y {TmpD})) 1100.0))
        (Variables|Default|SetInBox true))
      (Variables|Default|SetTmpA (Math|Vector|DotProduct {TmpD} {DbgFwd}))
      (if (and (and (> {TmpA} 1100.0) (< (- {TmpA} 1100.0) {JEdge}))
               (and (< {TmpA} 3600.0)
                    (< (Math|Float|Absolute(Float) (Math|Vector|DotProduct {TmpD} {Rgt})) 1100.0)))
        (Variables|Default|SetJEdge (- {TmpA} 1100.0))
        (Variables|Default|SetNextJ _j)
        (Variables|Default|SetHasJ true)))
    (Variables|Default|SetGapCm 99999.0)
    (Variables|Default|SetTmpB 99999.0)
    (Variables|Default|SetMinNow 99999.0)
    (bind _cars (Actor|GetAllActorsOfClass "__CAR__"))
    (for _car _cars
      (Variables|Default|SetTmpE (Transformation|GetActorLocation :self _car))
      (Variables|Default|SetTmpD (- {TmpE} {DbgMe}))
      (if (> (Math|Vector|VectorLengthXY {TmpD}) 1.0)
        (Variables|Default|SetMinNow (Math|Float|Min(Float) {MinNow} (Math|Vector|VectorLengthXY {TmpD})))
        (Variables|Default|SetOFwd (Transformation|GetActorForwardVector :self _car))
        (Variables|Default|SetTmpA (Math|Vector|DotProduct {TmpD} {DbgFwd}))
        (Variables|Default|SetPar (Math|Vector|DotProduct {OFwd} {DbgFwd}))
        (if (> {TmpA} 0.0)
          (if (> {Par} 0.5)
            (if (or (< (Math|Float|Absolute(Float) (Math|Vector|DotProduct {TmpD} {Rgt})) 250.0)
                    (< (Math|Vector|VectorLengthXY {TmpD}) 900.0))
              (Variables|Default|SetGapCm (Math|Float|Min(Float) {GapCm} {TmpA})))
            (else
              (if (and (< {TmpA} 900.0)
                       (< (Math|Float|Absolute(Float) (Math|Vector|DotProduct {TmpD} {Rgt})) 250.0))
                (Variables|Default|SetTmpB (Math|Float|Min(Float) {TmpB} {TmpA}))))))
        (if (and {HasJ} (not {InBox}))
          (if (< {Par} 0.7)
            (Variables|Default|SetTmpD (- {TmpE} {NextJ}))
            (if (and (< (Math|Float|Absolute(Float) (.x {TmpD})) 1100.0)
                     (< (Math|Float|Absolute(Float) (.y {TmpD})) 1100.0))
              (Variables|Default|SetConflict true)
              (else
                (if (> (.z {NextJ}) 0.5)
                  (Variables|Default|SetTmpA (* -1.0 (Math|Vector|DotProduct {TmpD} {OFwd})))
                  (if (and (and (> {TmpA} 1100.0) (< {TmpA} 3100.0))
                           (< (Math|Float|Absolute(Float)
                                (Math|Vector|DotProduct {TmpD} (Transformation|GetActorRightVector :self _car)))
                              1100.0))
                    (Variables|Default|SetConflict true)))))))))
    (Variables|Default|SetStopCm 99999.0)
    (if {Conflict}
      (Variables|Default|SetStopCm (- {JEdge} 300.0))
      (Variables|Default|SetYieldTicks (+ {YieldTicks} 1)))
    (if (< {Ticks} 3)
      (Variables|Default|SetStillDone -1)
      (Variables|Default|SetStillCPrev -1))
    (Variables|Default|SetStillSeen false)
    (Variables|Default|SetStillBest 99999)
    (Variables|Default|SetEHit false)
    (Variables|Default|SetCloseHit false)
    (for _c {Crossings}
      (Variables|Default|SetCIdx (Math|Float|Truncate (.z _c)))
      (Variables|Default|SetCAhead (Math|Integer|%(Integer) (+ (- {CIdx} {Idx}) {NPts}) {NPts}))
      (Variables|Default|SetCA2 {CAhead})
      (if (> {CAhead} 17)
        (Variables|Default|SetCA2 (+ 1000 (- {NPts} {CAhead}))))
      (if (and (== {CIdx} {StillDone})
               (and (> {CAhead} 17) (< {CAhead} (- {NPts} 4))))
        (Variables|Default|SetStillDone -1))
      (Variables|Default|SetExempt (or (== {CIdx} {StillDone})
                                       (and (== {CIdx} {StillCPrev}) (>= {StillRun} 6.0))))
      (if (or (<= {CAhead} 17) (>= {CAhead} (- {NPts} 4)))
        (Variables|Default|SetTmpE (- (Utilities|Array|Get(acopy) {Route} (Math|Integer|%(Integer) (+ {CIdx} 1) {NPts}))
                                      (Utilities|Array|Get(acopy) {Route} {CIdx})))
        (Variables|Default|SetOFwd (Math|Vector|Normalize (Math|Vector|MakeVector (.x {TmpE}) (.y {TmpE}) 0.0)))
        (bind _peds (Actor|GetAllActorsOfClass "__PED__"))
        (for _p _peds
          (Variables|Default|SetTmpE (- (Transformation|GetActorLocation :self _p) _c))
          (if (and (< (Math|Float|Absolute(Float)
                        (+ (* (.x {TmpE}) (.x {OFwd})) (* (.y {TmpE}) (.y {OFwd})))) 300.0)
                   (< (Math|Float|Absolute(Float)
                        (- (* (.y {TmpE}) (.x {OFwd})) (* (.x {TmpE}) (.y {OFwd})))) 800.0))
            (Variables|Default|SetPedSpd (Math|Vector|VectorLengthXY (Transformation|GetVelocity :self _p)))
            (if (< {PedSpd} 20.0)
              (Variables|Default|SetStillSeen true)
              (if (< {CA2} {StillBest})
                (Variables|Default|SetStillBest {CA2})
                (Variables|Default|SetStillC {CIdx})))
            (if (and (>= {CAhead} 3) (<= {CAhead} 17))
              (if (or (> {PedSpd} 20.0) (not {Exempt}))
                (Variables|Default|SetStopCm (Math|Float|Min(Float) {StopCm} (- (* 150.0 {CAhead}) 650.0)))
                (Variables|Default|SetPedTicks (+ {PedTicks} 1)))
              (else
                (Variables|Default|SetTmpD (- (Transformation|GetActorLocation :self _p) {DbgMe}))
                (Variables|Default|SetTmpA (Math|Vector|DotProduct {TmpD} {DbgFwd}))
                (if (and (> {TmpA} -100.0)
                         (and (< {TmpA} 900.0)
                              (< (Math|Float|Absolute(Float) (Math|Vector|DotProduct {TmpD} {Rgt})) 300.0)))
                  (if (or (> {PedSpd} 20.0) (not {Exempt}))
                    (Variables|Default|SetStopCm 0.0)
                    (Variables|Default|SetEHit true)
                    (if (and (> {PedSpd} 20.0) (< {TmpA} 400.0))
                      (Variables|Default|SetCloseHit true))))
                (if (and (<= {CAhead} 2) (> {CurSpeed} 50.0))
                  (if (> {PedSpd} 20.0)
                    (Variables|Default|SetPedViol (+ {PedViol} 1))
                    (else
                      (Variables|Default|SetPedPassStill (+ {PedPassStill} 1)))))))))))
    (if {StillSeen}
      (if (not (== {StillC} {StillCPrev}))
        (Variables|Default|SetStillRun 0.0)
        (Variables|Default|SetStillCPrev {StillC}))
      (if (< {CurSpeed} 10.0)
        (Variables|Default|SetStillRun (+ {StillRun} (* 2.0 {Dt}))))
      (if (>= {StillRun} 6.0)
        (Variables|Default|SetStillDone {StillCPrev}))
      (else
        (Variables|Default|SetStillRun 0.0)
        (Variables|Default|SetStillCPrev -1)))
    (if {EHit}
      (Variables|Default|SetEStops (+ {EStops} 1))
      (if (> {CurSpeed} 50.0)
        (Variables|Default|SetPedCorr (+ {PedCorr} 1))))
    (if (and {CloseHit} (> {CurSpeed} 50.0))
      (Variables|Default|SetPedClose (+ {PedClose} 1)))
    (if (< {Ticks} 3)
      (Variables|Default|SetMinEver 99999.0))
    (Variables|Default|SetMinEver (Math|Float|Min(Float) {MinEver} {MinNow}))
    (if (< {MinNow} 400.0)
      (Variables|Default|SetCloseTicks (+ {CloseTicks} 1)))
    (Variables|Default|SetTmpA (Math|Float|Clamp(Float) (/ {TmpB} (+ 700.0 (* 1.8 {CurSpeed}))) 0.0 1.0))
    (Variables|Default|SetEffSpeed (Math|Float|Min(Float) {SpeedCmS}
      (Math|Float|Max(Float) 60.0 (* {SpeedCmS} (* {TmpA} {TmpA})))))))
"""


class _G(dict):
    def __missing__(self, k):
        return g(k)


def render(src):
    return src.format_map(_G()).replace("__CAR__", CAR).replace("__PED__", PED)


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
    # The per-car data arrays are written by apply_routes.py through the object
    # API, which refuses any property that is not instance-editable.
    for name, _, cont in NEW_VARS:
        if cont:
            T(BT + "set_variable_instance_editable", blueprint=BP, variable_name=name,
              instance_editable=True)
    out = {"added": added}
    graphs = [x["refPath"].split(":")[-1] for x in T(BT + "list_graphs", blueprint=BP)]
    for name, src in (("DriveTick", DRIVE), ("UpdateEffSpeed", EFF)):
        if name in graphs:
            gr = {"refPath": BPP + ":" + name}
        else:
            gr = T(BT + "add_function_graph", blueprint=BP, graph_name=name)
        T(BT + "write_graph_dsl", graph=gr, code=render(src))
        out[name] = len(T(BT + "read_graph_dsl", graph=gr))
    out["compile"] = T(BT + "compile_blueprint", blueprint=BP)
    return out
