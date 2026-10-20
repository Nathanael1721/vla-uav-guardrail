"""Offline closed loop for the own-velocity forecast: the Shield, the rail's
escalation FSM and its hold, flying a simulated ArduCopter.

docs/DESIGN-own-velocity-forecast.md, "The closed loop". A replay of a log is
open loop: after the first tick where the new Shield would have commanded
something else, the logged trajectory is the old Shield's. This puts the
Shield back in the loop with an aircraft model, flies the recorded pilot's
raw actions (the logged `raw`, tick for tick - the follow controller's own
commands, which do not react to where the simulated aircraft is), and
measures what the rails record: ticks inside a hard rule (`unsafe`, the
position), the deepest incursion, P0 escapes (the Shield's re-check of what
was flown, as guardrail.replay.flown_fields counts them - with the velocity
when the Shield was given one - and on the OLD basis, the flown action
re-checked on the commanded forecast from the position, which is what every
stored run's count means) and the FSM edges.

THE AIRCRAFT IS THE PHYSICAL FORM. The simulated aircraft's velocity follows
the setpoint sent `reaction_s` ago, at most `accel_mps2` (a vector rate
limit), integrated at 5 ms - the form the response was identified in
(tools/fit_velocity_response.py), with the setpoints the loop actually sends.
The Shield forecasts with what it can know: the setpoints already sent
(note_flown) and then the candidate, held. `--robust` also flies a slower
aircraft than the fit (reaction 0.45 s, 1.75 m/s^2), so the Shield's model is
optimistic about it. The measured velocity the Shield is given is the
aircraft's plus 0.05 m/s of seeded noise.

ArduCopter BRAKE (the rail's Loiter) is flown as a zero setpoint at the
ARDUCOPTER_BRAKE envelope's 0.75 m/s^2 behind the same pipe. RTL is a GUIDED
stop held RTL_HOLD_S (both recorded RTLs from inside the margin stayed inside
it 5.65 and 6.44 s while they climbed), after which the run ends. Altitude is
held (the recorded approaches are level). Validation: flown with the
commanded forecast, the simulated aircraft is inside the margin on 73 ticks
of each recorded approach; the flights logged 71 and 78.

Run:
  python tools/own_velocity_closed_loop.py                  # the two recorded close approaches
  python tools/own_velocity_closed_loop.py --robust
  python tools/own_velocity_closed_loop.py --all-runs <checkout>/demo/out --robust
                                                            # every recorded pas_ap run, open
                                                            # and closed loop (read only)
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import random
import sys
from collections import Counter, deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "demo" / "pas_ardupilot"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np                                                      # noqa: E402

from guardrail import fsm as F                                          # noqa: E402
from guardrail import response as R                                     # noqa: E402
from guardrail.models import Action4D, Policy, State                    # noqa: E402
from guardrail.replay import rail_shield                                # noqa: E402

FIXTURES = ROOT / "tests" / "fixtures"
CITY_MAP = FIXTURES / "citylife_occ_flightband_6to14.npz"
APPROACH = FIXTURES / "pas_ap_close_approaches.slim.jsonl.gz"
# The two recorded close approaches to the same building on the CityLife
# red-car follow: the 14 Oct recording (Shield on, BRAKE hold) and the 13 Oct
# close run (Shield on, Loiter flown as BRAKE). Windows: from ~3 s before the
# Shield first acted to ~6 s after it.
WINDOWS = {"pas_ap_video_on_20261014": (330, 440), "pas_ap_close_on": (350, 460)}
BRAKE_PLANT_ACCEL = R.ARDUCOPTER_BRAKE.accel_mps2
RTL_HOLD_S = 6.0
VEL_NOISE_MPS = 0.05


class Aircraft:
    """ArduCopter GUIDED, physical form: the velocity follows the setpoint
    sent `lag` seconds ago, rate-limited at `accel`."""

    def __init__(self, x, y, up, vx, vy, *, lag: float, accel: float, dt: float = 0.005):
        self.x, self.y, self.up, self.vx, self.vy = x, y, up, vx, vy
        self.lag, self.accel, self.dt = lag, accel, dt
        self.t = 0.0
        self.cmds = deque([(-math.inf, (vx, vy), accel)])
        self.trace = []

    def command(self, t: float, vx: float, vy: float, accel: float | None = None) -> None:
        self.cmds.append((t, (vx, vy), self.accel if accel is None else accel))

    def advance(self, t_end: float, on_step=None) -> None:
        while self.t < t_end - 1e-12:
            h = min(self.dt, t_end - self.t)
            tq = self.t - self.lag
            while len(self.cmds) > 1 and self.cmds[1][0] <= tq:
                self.cmds.popleft()
            (cx, cy), acc = self.cmds[0][1], self.cmds[0][2]
            ddx, ddy = cx - self.vx, cy - self.vy
            m = math.hypot(ddx, ddy)
            lim = acc * h
            if m > lim:
                ddx, ddy = ddx * lim / m, ddy * lim / m
            self.vx += ddx
            self.vy += ddy
            self.x += self.vx * h
            self.y += self.vy * h
            self.t += h
            if on_step is not None:
                on_step(self.x, self.y)


def load_approaches(path: Path = APPROACH) -> tuple[dict, dict]:
    """{run: rows}, {run: policy} from the committed slim copy."""
    rows: dict[str, list] = {}
    pols: dict[str, Policy] = {}
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        for line in fh:
            r = json.loads(line)
            if "policy" in r:
                pols[r["run"]] = Policy.model_validate(r["policy"])
            else:
                rows.setdefault(r["run"], []).append(r)
    return rows, pols


def city_map() -> dict:
    with np.load(CITY_MAP) as z:
        return {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["ox"]),
                "oy": float(z["oy"])}


def _ls_velocity(rows, i, half=2):
    lo, hi = max(0, i - half), min(len(rows) - 1, i + half)
    ts = [rows[k]["t"] for k in range(lo, hi + 1)]
    out = []
    for f in ("x", "y"):
        vs = [rows[k][f] for k in range(lo, hi + 1)]
        mt, mv = sum(ts) / len(ts), sum(vs) / len(vs)
        den = sum((t - mt) ** 2 for t in ts)
        out.append(sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / den)
    return out


def fly(run: str, rows: list, policy: Policy, smap: dict, *, own_velocity: bool,
        lag: float = R.ARDUCOPTER_GUIDED.reaction_s,
        accel: float = R.ARDUCOPTER_GUIDED.accel_mps2, seed: int = 20261019,
        window: tuple | None = None) -> dict:
    """One closed-loop run over the run's window (WINDOWS, or `window`).
    `own_velocity` False is the rail as flown (commanded forecast, Loiter as
    BRAKE); True gives the Shield the measured velocity and flies the hold
    that stops clear."""
    lo, hi = WINDOWS[run] if window is None else window
    win = [r for r in rows if lo <= r["tick"] <= hi]
    i0 = next(k for k, r in enumerate(rows) if r["tick"] == lo)
    vx0, vy0 = _ls_velocity(rows, i0)
    r0 = win[0]
    ac = Aircraft(r0["x"], r0["y"], r0["up"], vx0, vy0, lag=lag, accel=accel)
    ac.t = r0["t"]
    sh = rail_shield(policy, lookahead_s=5.0, dt=0.1, obstacle_map=smap)
    fsm = F.EscalationFSM()
    rng = random.Random(seed)
    p0 = {c.id for c in policy.constraints if c.priority == "P0"}
    hard = {c.id for c in policy.constraints if c.constraint_type == "hard"}
    ap_mode, deferred = "GUIDED", False
    unsafe_ticks, escapes, edges = 0, 0, Counter()
    escapes_cmd = 0
    failsafe, unsafe_before_failsafe, rtl_until = None, None, math.inf
    min_clear = [math.inf]
    holds, out_rows = [], []

    def note(x, y):
        min_clear[0] = min(min_clear[0], sh.clearance_at(x, y))

    for r in win:
        t = r["t"]
        ac.advance(t, note)
        vel = None
        if own_velocity:
            vel = (ac.vx + rng.gauss(0, VEL_NOISE_MPS), ac.vy + rng.gauss(0, VEL_NOISE_MPS), 0.0)
        st = State(x=ac.x, y=ac.y, up=ac.up, yaw_deg=r["yaw_deg"], velocity=vel)
        s = r["subject"]
        if s is None:
            sh.set_subject(None)
        else:
            sh.set_subject(s[0], s[1], "vehicle")
        raw = Action4D(**r["raw"])
        d = sh.filter(st, raw, t=t)
        unsafe = sh.state_is_unsafe(st)
        unsafe_hard = [v for v in unsafe if v.rule_id in hard]
        unsafe_ticks += bool(unsafe_hard)
        o = fsm.step(F.tick_input_from_decision(
            t, d, policy, horizon_s=None, stop_illegal=unsafe,
            unsafe_depth_m=sh.unsafe_depth(st, unsafe), caps=sh.applied_caps or False))
        if o.edge:
            edges[o.edge] += 1
        # The rail's order (demo/pas_ardupilot/perception_node.py): the FSM's
        # mode request, the hold that stops clear, the stop through the
        # Shield, then what is sent - and the Shield told what was sent.
        setpoint, flown_dec, flown, request = o.setpoint, d, None, None
        if o.set_mode == "LOITER":
            plan = R.hold_plan(sh, st) if own_velocity else {"choice": "brake", "why": "flag off"}
            holds.append({"tick": r["tick"], **{k: plan.get(k) for k in ("choice", "why")}})
            deferred = plan["choice"] == "guided"
            request = None if deferred else "BRAKE"
        elif o.set_mode in ("RTL", "LAND"):
            request, deferred = o.set_mode, False
        elif o.set_mode == "GUIDED":
            request, deferred = "GUIDED", False
        elif deferred and fsm.state == F.FSMState.LOITER:
            plan = R.hold_plan(sh, st)
            if plan["choice"] == "brake":
                deferred, request = False, "BRAKE"
                holds.append({"tick": r["tick"], "choice": "brake (handed over)",
                              "why": plan["why"]})
        if deferred and fsm.state == F.FSMState.LOITER:
            flown_dec, setpoint = sh.advise(st, Action4D()), "pass"
        elif own_velocity and setpoint == "brake":
            flown_dec, setpoint = sh.advise(st, Action4D()), "pass"
        if ap_mode == "GUIDED":
            if setpoint == "pass":
                flown = flown_dec.emitted
            elif setpoint == "brake":
                flown = Action4D()
        if flown is not None:
            ac.command(t, flown.vx, flown.vy)
            esc = [v for v in (flown_dec.emitted_violations if setpoint == "pass"
                               else unsafe) if v.rule_id in p0]
            escapes += bool(esc)
            # The same count on the OLD basis: the flown action re-checked
            # from the position with no velocity, on the commanded forecast -
            # what every stored run's P0 escape count means. With the flag on
            # the rail's re-check (flown_fields: decision.emitted_violations)
            # uses the forecast the decision used; this shows the KPI does
            # not move because of the basis. Identical to `escapes` with the
            # flag off (no velocity on the state either way).
            if setpoint == "pass" and own_velocity:
                pos = State(x=st.x, y=st.y, up=st.up, yaw_deg=st.yaw_deg)
                escapes_cmd += any(v.rule_id in p0 for v in sh._check(pos, flown))
            else:
                escapes_cmd += bool(esc)
        sh.note_flown(flown, t)
        if request == "BRAKE":
            ap_mode = "BRAKE"
            ac.command(t, 0.0, 0.0, BRAKE_PLANT_ACCEL)
        elif request in ("RTL", "LAND"):
            ap_mode = request
            ac.command(t, 0.0, 0.0)
        elif request == "GUIDED":
            ap_mode = "GUIDED"
        out_rows.append({"tick": r["tick"], "t": t, "x": round(ac.x, 3), "y": round(ac.y, 3),
                         "v": [round(ac.vx, 3), round(ac.vy, 3)],
                         "clear_m": round(sh.clearance_at(ac.x, ac.y), 3),
                         "unsafe": [v.rule_id for v in unsafe_hard], "fsm": o.state.value,
                         "edge": o.edge, "ap_mode": ap_mode, "deferred": deferred,
                         "flown": None if flown is None else [round(flown.vx, 3), round(flown.vy, 3)],
                         "violations": [v.rule_id for v in d.violations]})
        # A fail-safe: ArduCopter's RTL first climbs where it is, then flies
        # home, which this level model does not fly. On both recorded RTLs
        # from inside the margin the aircraft stayed inside for 5.65 s and
        # 6.44 s of the climb (pas_ap_video_on_20261014 tick 388 -> 440,
        # pas_ap_close_on 408 -> 467), so the simulation holds the aircraft
        # (stopped) RTL_HOLD_S, counting, and ends there.
        if request in ("RTL", "LAND") and failsafe is None:
            failsafe, unsafe_before_failsafe = request, unsafe_ticks
            rtl_until = t + RTL_HOLD_S
        if fsm.terminal or (failsafe is not None and t >= rtl_until):
            break
    clear_rule = min((c.min_clearance_m for c in policy.constraints
                      if c.type == "obstacle_clearance"), default=None)
    return {"run": run, "own_velocity": own_velocity, "lag": lag, "accel": accel,
            "failsafe": failsafe,
            "ticks": len(out_rows), "unsafe_ticks": unsafe_ticks, "p0_escapes": escapes,
            "p0_escapes_commanded_basis": escapes_cmd,
            "unsafe_before_failsafe": (unsafe_ticks if unsafe_before_failsafe is None
                                       else unsafe_before_failsafe),
            "min_clearance_m": round(min_clear[0], 3),
            "deepest_in_margin_m": (None if clear_rule is None
                                    else round(max(0.0, clear_rule - min_clear[0]), 3)),
            "edges": dict(edges), "holds": holds,
            "first_clearance_flag": next((x["tick"] for x in out_rows
                                          if any("clearance" in v for v in x["violations"])), None),
            "rows": out_rows}


def report(robust: bool = False) -> list[dict]:
    rows, pols = load_approaches()
    smap = city_map()
    out = []
    planes = [(R.ARDUCOPTER_GUIDED.reaction_s, R.ARDUCOPTER_GUIDED.accel_mps2)]
    if robust:
        planes.append((0.45, 1.75))
    for run in WINDOWS:
        for lag, acc in planes:
            for own in (False, True):
                out.append(fly(run, rows[run], pols[run], smap, own_velocity=own,
                               lag=lag, accel=acc))
    return out


ALL_RUNS = ("pas_ap_20261008_224440", "pas_ap_20261012_033329", "pas_ap_close_off",
            "pas_ap_close_on", "pas_ap_video_off_20261014", "pas_ap_video_on_20261014")


def load_run(out_root: Path, run: str) -> tuple[list, Policy]:
    """A recorded run's rows and policy from its folder (gitignored; read only)."""
    folder = Path(out_root) / run
    rows = [json.loads(l) for l in (folder / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines() if l.strip()]
    pol = Policy.model_validate(json.loads((folder / "policy_g0.json").read_text(
        encoding="utf-8")))
    return rows, pol


def causal_velocity(rows: list, i: int, n: int = 3) -> tuple[float, float, float]:
    """What a rail without the autopilot's velocity could have measured at row
    i: the least-squares slope of the last n + 1 logged positions (lags by
    about half its span, ~0.16 s; the EKF's own velocity does not)."""
    lo = max(0, i - n)
    ts = [rows[k]["t"] for k in range(lo, i + 1)]
    if len(ts) < 2:
        return (0.0, 0.0, 0.0)
    out = []
    for f in ("x", "y", "up"):
        vs = [rows[k][f] for k in range(lo, i + 1)]
        mt, mv = sum(ts) / len(ts), sum(vs) / len(vs)
        den = sum((t - mt) ** 2 for t in ts)
        out.append(sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / den if den else 0.0)
    return tuple(out)


def replay_escapes(rows: list, policy: Policy, smap: dict, *, own_velocity: bool) -> dict:
    """OPEN LOOP: every logged tick through the Shield, with or without the
    measured velocity (causal_velocity) and the pipe (the logged flown
    action, note_flown). Counts the Shield's own P0 escapes (emitted
    violations of a P0 rule) and the ticks it touched."""
    sh = rail_shield(policy, lookahead_s=5.0, dt=0.1, obstacle_map=smap)
    p0 = {c.id for c in policy.constraints if c.priority == "P0"}
    esc, touched, braked = [], 0, 0
    esc_cmd: list = []
    for i, r in enumerate(rows):
        vel = causal_velocity(rows, i) if own_velocity else None
        st = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"], velocity=vel)
        s = r.get("subject")
        if s is None:
            sh.set_subject(None)
        elif math.hypot(s[0] - r["x"], s[1] - r["y"]) < 0.01:
            sh.set_subject(r["x"], r["y"], "vehicle")
        else:
            sh.set_subject(s[0], s[1], "vehicle")
        d = sh.filter(st, Action4D(**r["raw"]), t=r["t"])
        if any(v.rule_id in p0 for v in d.emitted_violations):
            esc.append(r["tick"])
        if vel is not None:
            # The old basis (fly(), escapes_cmd): the emitted action re-checked
            # from the position without the velocity, commanded forecast.
            pos = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"])
            if any(v.rule_id in p0 for v in sh._check(pos, d.emitted)):
                esc_cmd.append(r["tick"])
        elif esc and esc[-1] == r["tick"]:
            esc_cmd.append(r["tick"])
        touched += bool(d.touched)
        braked += bool(d.braked)
        fa = r.get("flown_action")
        sh.note_flown(None if fa is None else Action4D(**fa), r["t"])
    return {"ticks": len(rows), "p0_escape_ticks": esc,
            "p0_escape_ticks_commanded_basis": esc_cmd,
            "touched": touched, "braked": braked}


def sweep(out_root: Path, robust: bool = False) -> list[dict]:
    """Every recorded pas_ap run: the open-loop escape replay, and the whole
    run in closed loop, commanded forecast against own-velocity."""
    smap = city_map()
    out = []
    planes = [(R.ARDUCOPTER_GUIDED.reaction_s, R.ARDUCOPTER_GUIDED.accel_mps2)]
    if robust:
        planes.append((0.45, 1.75))
    for run in ALL_RUNS:
        rows, pol = load_run(out_root, run)
        rec = {"run": run, "ticks": len(rows),
               "replay": {m: replay_escapes(rows, pol, smap, own_velocity=m == "own")
                          for m in ("commanded", "own")}, "closed_loop": []}
        win = (rows[3]["tick"], rows[-1]["tick"])
        for lag, acc in planes:
            for own in (False, True):
                r = fly(run, rows, pol, smap, own_velocity=own, lag=lag, accel=acc, window=win)
                r.pop("rows")
                rec["closed_loop"].append(r)
        out.append(rec)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--robust", action="store_true",
                    help="also fly a slower aircraft (0.45 s, 1.75 m/s^2)")
    ap.add_argument("--all-runs", default=None, metavar="DEMO_OUT",
                    help="sweep every recorded pas_ap run in this demo/out (read only)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    if args.all_runs:
        res = sweep(Path(args.all_runs), args.robust)
        for rec in res:
            rc, ro = rec["replay"]["commanded"], rec["replay"]["own"]
            print(f"{rec['run']:28s} open loop: P0 escapes commanded "
                  f"{len(rc['p0_escape_ticks'])} {rc['p0_escape_ticks'][:6]} / own-velocity "
                  f"{len(ro['p0_escape_ticks'])} {ro['p0_escape_ticks'][:6]} (old basis "
                  f"{len(ro['p0_escape_ticks_commanded_basis'])}); touched "
                  f"{rc['touched']} / {ro['touched']}; braked {rc['braked']} / {ro['braked']}")
            for r in rec["closed_loop"]:
                print(f"    closed loop {r['lag']:.2f} s/{r['accel']:.2f}  "
                      f"{'own-velocity' if r['own_velocity'] else 'commanded   '}  ticks "
                      f"{r['ticks']}  failsafe {r['failsafe']}  unsafe {r['unsafe_ticks']}  "
                      f"P0 escapes {r['p0_escapes']} (old basis "
                      f"{r['p0_escapes_commanded_basis']})  "
                      f"min clearance {r['min_clearance_m']:.2f}  edges {r['edges']}")
        if args.out:
            Path(args.out).write_text(json.dumps(res, indent=1), encoding="utf-8")
        return 0
    res = report(args.robust)
    for r in res:
        print(f"{r['run']:28s} aircraft {r['lag']:.2f} s/{r['accel']:.2f} m/s^2  "
              f"{'own-velocity' if r['own_velocity'] else 'commanded   '}  "
              f"ticks {r['ticks']:3d}  failsafe {r['failsafe']}  unsafe {r['unsafe_ticks']:3d}  P0 escapes "
              f"{r['p0_escapes']} (old basis {r['p0_escapes_commanded_basis']})  "
              f"min clearance {r['min_clearance_m']:.2f} m  first "
              f"flag {r['first_clearance_flag']}  edges {r['edges']}  holds "
              f"{[h['choice'] for h in r['holds']]}")
    if args.out:
        Path(args.out).write_text(json.dumps(res, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
