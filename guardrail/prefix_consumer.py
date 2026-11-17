"""A pilot that CONSUMES the Prefix Compiler's output: the grant's default
per-consumer adapter, "Action-set adapter (default)" (Prefix Compiler page).

WHY THIS EXISTS (audit card WP4-15, 2026-11-16)

"Always-on A/B during stress runs. A scenario can be flagged for A/B; the
harness runs it twice (Prefix on / off) with the same seed, and the KPI
report shows the delta." The harness had the key (`ab: true`) and the report
had the section, but no pilot it flies read the Prefix: the three laws of
guardrail/vla_stub.py are rule-ignorant on purpose, so the two arms were one
flight and the section read n/m. This module is the missing consumer.

WHAT IT IS, AND WHAT IT IS NOT

`PrefixConsumer` wraps a pilot law (anything with `act(state) -> Action4D`,
or a callable) and a CSP. With a CSP it changes the law's candidate in two
steps. The ranges and the band come from the CSP's action sets; the polygon
and corridor test resolves the CSP's rule ids to the POLICY's compiled
geometry, the Shield's own margin rings included - the CSP carries the ids,
not the shapes:

  1. CLAMP to `allowed_action_set`: the horizontal speed cap (and the vx / vy
     box), the climb range, the yaw-rate cap (the CSP states deg/s, Action4D
     rad/s), and the altitude band - the climb is held so the altitude one
     lookahead ahead stays inside the band, and from outside the band it is
     the fastest climb the range allows toward it.
  2. TURN: when the clamped candidate would translate into a polygon of
     `no_translate_into_polygons` or out of a corridor of
     `no_translate_out_of_corridors` within the lookahead, the nearest
     heading of a fixed fan (FAN_DEG, 15 degree steps to either side, the
     side last taken first) that would not is flown at the same speed; when
     no heading of the fan is admissible it stops (zero horizontal velocity)
     if stopping is admissible, and otherwise hands the clamped candidate on
     unchanged - an aircraft already inside a zone is the Shield's to
     recover, not this adapter's.

The admissibility test is `guardrail.compiler.action_violations` - the
Shield's own point verdicts over the CSP's rule ids, against
compiled_ir(policy): the zones and corridors with the Shield's buffered
margins - at the lookahead and at every HORIZON_STEP_S before it (an
end-point test alone steps over a zone narrower than speed x lookahead).
The whole straight path to the lookahead is tested as well (since
2026-11-16): a path that starts inside a corridor's tube must stay in it,
and one that starts outside a zone's margin ring must not cross it (when its
altitudes meet the zone's band). The tube is the Shield's own
(Shield._corridor_tube: buffered by half / cos(pi/64) with 16 segments a
quadrant, so it contains every point the Shield's distance check accepts; a
plain buffer's chords cut inside it), the ring the IR's. A sampled test steps
over a short cut: the hil prefix A/B pab1 (2026-11-16) flew a 0.21 s, 0.25 m
cut across a dog-leg's inner corner between two 0.5 s samples, which the
Shield's 0.1 s forecast caught and CorridorReturn sized at 2.48 m along the
nearest leg, over theta: Loiter, then RTL on the loiter timeout; headless,
nfz-head-on's prefix-on arm clipped the zone's ring the same way, 19 repairs
and two Loiters. "Do not translate into / out of" is a statement about the
path, so the path is what is tested. A path from inside a ring or from
outside a tube is the Shield's to recover and is judged as before. No other
geometry is written here, but the turn reads the Shield's geometry, so the
action-set adapter's ids and ranges alone would not steer this pilot: that
is part of why its on / off delta holds by construction.

THE CORRIDOR MARGIN (since 2026-11-17). On a rail the consumer decides on
the pose it last received, and the Shield judges that action from the pose
of its own tick, which is newer: the aircraft has flown on. The fan's
nearest admissible heading is by construction the one whose path grazes the
wall, so the action could already leave the tube from the Shield's pose.
On a straight leg CorridorReturn repairs that (a Brake on G1). Where the
exit is through the round cap at the corridor's END, it cannot: the
aircraft's radial rate toward the nearest centerline point (the end vertex)
is still inward, so the operator leaves the action unchanged, and the Shield
brakes on X1, "no operator converged". The hil prefix A/B pab2 (2026-11-17)
did this on 34 ticks of its 4 corridor-drift-out prefix-on flights, every
one an exit through the end cap 0.005 to 0.30 m deep at t+5 s, and
pab2_011 ended in RTL (N-in-T in Loiter); the logged actions are
reproduced from the pose of the same 0.1 s tick or the one before
(docs/data/kpi_prefix_ab_20261117/corridor_finding.md). So a path that
starts inside the tube must also stay the margin (corridor_margin_m)
inside its wall: then the same commanded straight path, from any pose up to
that far away, stays inside the tube. That is geometry alone: the Shield
also forecasts from the aircraft's measured velocity (its own-velocity
branch), which this does not cover. A path that starts in the margin band
(inside the tube, within the margin of the wall) must end back inside the
margin. The margin also closes a smaller gap: the wall test reads the
Shield's own tube, which CONTAINS the true one (20.024 m for a 20 m
half-width - the right side for the Shield's exit test, the wrong one for
admissibility), and pab2_010 flew a corner pass 20.021 m out that the
Shield's distance check flagged (X1, "repair over theta"). Both branches
close it: from inside the margin the path stays in the inner tube, and from
the band it must stay in the true tube as well (buffered by the plain
half-width, whose chords cut inside the circle), so a band start at a
dog-leg's inner corner cannot pass the corner 20.0-20.024 m out either. The
Shield is not changed by this.

Without a CSP (`csp=None`, the A/B's prefix-off arm) the law's action is
returned untouched: the two arms are the same pilot, with and without its
Prefix.

It is a hand-written adapter, not a learned VLA. It reads the polygons the
Shield enforces, so part of any on / off delta holds by construction; a
report that quotes one must say so and name this pilot (tools/kpi_report.py
does). It is deterministic: the same states give the same actions, on any
host.

ITS OWN MODULE, NOT guardrail/vla_stub.py: that file's source is the
`vla_model_hash` of the three stub pilots (guardrail.manifest.model_hash
pins a code-only pilot by its module's bytes), and every KPI-grade campaign
flown so far carries it. This pilot announces its own id, MODEL_ID, pinned
by THIS file's source; the law it wraps is pinned beside it, by that law's
own source, in its record (`wraps_model_hash`, consumer_record).

STATIC POLICIES ONLY (`static_csp`). The CSP is compiled once, before
take-off, from the policy the pilot's node loaded, with the spatial filter
off (one CSP serves the whole mission, so every rule in force is in its
action sets, not only those near the first lookahead window). A policy whose
action sets change during the mission (a schedule edge) is refused, and so
is a scenario with mid-flight events by the harness
(experiments/sweep_scenarios.py prefix_consumer_refusal): generation N's CSP
reaching the pilot is audit card WP4-19's plumbing, not built here.
"""
from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any

import shapely
from shapely.geometry import LineString, Point

from .compiler import (DEFAULT_LOOKAHEAD_S, ConstraintCompiler, Mission,
                       action_set_adapter, action_violations, compiled_ir)
from .csp import CSP, csp_content_hash
from .geometry import area_covers, area_meets
from .models import Action4D, State

# The id this pilot announces (/vla/identity `model_id`, the manifest's
# vla_model_hash): a code-only pilot, pinned by this module's source.
MODEL_ID = "guardrail.prefix_consumer.PrefixConsumer"
# The per-consumer adapter it reads (the grant's default), the name a stress
# profile asks for it by (`prefix_consumer: action_set`).
ADAPTER = "action_set"
ADAPTERS = (ADAPTER,)
# The heading fan: degrees either side of the law's own heading, nearest
# first. 180 (straight back) is tried once.
FAN_STEP_DEG = 15.0
FAN_DEG = tuple(FAN_STEP_DEG * k for k in range(1, 13))
# The horizons the admissibility test is made at, up to the CSP's lookahead.
HORIZON_STEP_S = 0.5
# Zones and corridors are tested along the whole straight path to the
# lookahead as well (module docstring): the run's consumer record says which
# test it flew ("segment_margin" since 2026-11-17: the path also keeps
# CORRIDOR_MARGIN_M inside a corridor's wall; "segment" 2026-11-16, the wall
# itself; a record without the key is the 0.5 s samples alone).
CORRIDOR_PATH_TEST = "segment_margin"
# How far inside a corridor's wall a path from inside the tube must stay
# (module docstring, THE CORRIDOR MARGIN). The Shield judges the action from
# a pose newer than the one the consumer decided on; a path that keeps this
# distance from the wall stays inside the tube from any pose up to this far
# away. It is the larger of CORRIDOR_MARGIN_M and the CSP's horizontal speed
# cap times CORRIDOR_LAG_BUDGET_S (corridor_margin_m), so a faster cell keeps
# the same lag cover; the consumer record carries the value flown
# (`corridor_margin_m`). 1.0 m is the 4 m/s cap of the cells flown for a
# quarter second (the hil prefix A/B pab2 exceeded the wall within one 0.1 s
# tick; corridor_finding.md's headless lag sweep repaired nothing up to
# 0.2 s of lag and repaired at 0.3 s).
CORRIDOR_MARGIN_M = 1.0
CORRIDOR_LAG_BUDGET_S = 0.25
# The file the pilot's node writes its CSP to, under <run dir>/PREFIX_DIR/:
# apart from the Shield node's own csp_g<N>.json in the run directory (the
# episode record's, compiled at the Shield's clock with the spatial filter).
PREFIX_DIR = "vla_prefix"
CSP_FILE = "csp_g0.json"
_GEO = ("polygon:", "corridor:")


def consumer_mission(start, target, speed_mps: float,
                     cruise_alt_m: float | None = None) -> Mission:
    """The mission context the consumer's CSP is compiled for, built the same
    way wherever it is compiled (the pilot's node, the stress harness), so
    one cell and seed give one CSP on every backend.

    `start` is (x, y) or (x, y, up); `target` the same, or None for a pilot
    that flies to none (the mission's target is then its start, as the Shield
    node writes a target-less mission's). The cruise altitude is
    `cruise_alt_m`, else the target's, else the start's, else 15 m
    (ConstraintCompiler.parse_command's default). No task text: this pilot
    reads no words, and the text would be the only part of the mission id two
    backends could spell differently."""
    s = tuple(float(v) for v in start)
    t = None if target is None else tuple(None if v is None else float(v) for v in target)
    if len(s) not in (2, 3) or (t is not None and len(t) not in (2, 3)):
        raise ValueError(f"start and target are (x, y) or (x, y, up): {start!r}, {target!r}")
    alt = cruise_alt_m
    if alt is None and t is not None and len(t) == 3 and t[2] is not None:
        alt = t[2]
    if alt is None and len(s) == 3:
        alt = s[2]
    if alt is None:
        alt = 15.0
    tx, ty = (s[0], s[1]) if t is None else (t[0], t[1])
    vals = (s[0], s[1], tx, ty, float(alt), float(speed_mps))
    if not all(math.isfinite(v) for v in vals):
        raise ValueError(f"the consumer's mission must be finite: start {start!r}, "
                         f"target {target!r}, speed {speed_mps!r}, altitude {alt!r}")
    return Mission(task_text="", target_x=tx, target_y=ty, cruise_alt_m=float(alt),
                   speed_pref_mps=float(speed_mps), start_x=s[0], start_y=s[1])


def _instants(now: datetime, duration_s: float) -> list[datetime]:
    """`now`, every minute boundary inside the mission, and its end: the
    instants a weekly schedule (minute resolution) can change at."""
    end = now + timedelta(seconds=float(duration_s))
    out = [now]
    t = now.replace(second=0, microsecond=0) + timedelta(minutes=1)
    while t < end:
        out.append(t)
        t += timedelta(minutes=1)
    out.append(end)
    return out


def static_csp(policy, mission: Mission, *, now: datetime | None = None,
               duration_s: float | None = None,
               lookahead_s: float = DEFAULT_LOOKAHEAD_S) -> CSP:
    """The one CSP a consumer flies a whole mission on: compile_csp for
    `mission` at `now` (None: no clock, every rule in force) with the spatial
    filter off.

    Raises ValueError when the action sets are not the same at every instant
    of the mission (`now` .. `now` + `duration_s`, sampled at each minute
    boundary): a rule that switches in flight needs the next generation's
    CSP, which this pilot is not given (module docstring). With no clock, or
    no duration, nothing can switch and nothing is sampled. compile_csp's own
    refusals (CSPBudgetExceeded, CSPUnenforcedRules, CSPAltitudeConflict)
    pass through."""
    comp = ConstraintCompiler(policy)
    csp = comp.compile_csp(mission, lookahead_s, now, region_margin_m=None)
    if now is not None and duration_s:
        first = action_set_adapter(csp)
        for t in _instants(now, duration_s)[1:]:
            later = action_set_adapter(
                comp.compile_csp(mission, lookahead_s, t, region_margin_m=None))
            if later != first:
                raise ValueError(
                    f"{policy.policy_id}: the CSP's action sets at "
                    f"{t.isoformat(timespec='seconds')} are not those at "
                    f"{now.isoformat(timespec='seconds')} (a rule switches during the "
                    f"mission): the prefix consumer flies one CSP, compiled before "
                    f"take-off, and is for static policies only")
    return csp


def action_set_sha256(csp: CSP) -> str:
    """SHA-256 over what the consumer READS of a CSP: the policy hash, the
    generation, the lookahead and the spec's flat action-set dict
    (compiler.action_set_adapter). Every number in it is the policy's own, so
    two hosts that compiled the same policy agree on it to the last bit - the
    binding the stress harness checks a rail flight's announced CSP against.
    (csp_content_hash covers the prose and the risk grades too, whose
    distances are computed.)"""
    doc = {"policy_hash": csp.policy_hash, "generation": csp.generation,
           "lookahead_s": csp.lookahead_s, "action_set": action_set_adapter(csp)}
    canon = json.dumps(doc, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canon.encode("utf-8")).hexdigest()


def csp_record(csp: CSP | None) -> dict[str, Any]:
    """The identity of the CSP a consumer holds, for the arm record and
    /vla/identity: its three hashes, its policy and generation. Every value
    None for no CSP (the prefix-off arm)."""
    if csp is None:
        return {"policy_hash": None, "generation": None, "lookahead_s": None,
                "action_set_sha256": None, "csp_content_hash": None}
    return {"policy_hash": csp.policy_hash, "generation": csp.generation,
            "lookahead_s": csp.lookahead_s,
            "action_set_sha256": action_set_sha256(csp),
            "csp_content_hash": csp_content_hash(csp)}


def corridor_margin_m(csp: CSP | None) -> float | None:
    """The corridor margin a consumer of `csp` keeps (module docstring, THE
    CORRIDOR MARGIN): CORRIDOR_MARGIN_M, or the CSP's horizontal speed cap
    times CORRIDOR_LAG_BUDGET_S where that is larger. None for no CSP."""
    if csp is None:
        return None
    cap = csp.allowed_action_set.horizontal_speed_max_mps
    if cap is None or not math.isfinite(float(cap)):
        return float(CORRIDOR_MARGIN_M)
    return max(float(CORRIDOR_MARGIN_M), float(cap) * CORRIDOR_LAG_BUDGET_S)


def consumer_record(arm: str, csp: CSP | None, wraps: str | None = None) -> dict[str, Any]:
    """What a prefix arm's pilot is, as the harness's arm record and the
    node's /vla/identity both write it: this module's id and adapter, the arm
    ("on": the CSP is consumed; "off": the same pilot without it), the law it
    wraps and that law's own source pin (`wraps_model_hash`:
    guardrail.manifest.model_hash of its id), the fan and horizons, and the
    CSP's identity (csp_record).

    The pin is here because a consumer flight announces THIS module's id, so
    the manifest's vla_model_hash pins prefix_consumer.py only: the wrapped
    law (guardrail/vla_stub.py), which flies every action of the prefix-off
    arm and the candidate of the prefix-on arm, would otherwise be pinned by
    name alone, and a goto cell and a constant cell would share one
    vla_model_hash."""
    if arm not in ("on", "off"):
        raise ValueError(f"prefix arm {arm!r} is not on or off")
    if (arm == "on") != (csp is not None):
        raise ValueError(f"prefix arm {arm!r} with {'a' if csp is not None else 'no'} CSP: "
                         f"the on arm consumes one, the off arm none")
    from .manifest import model_hash
    return {"id": MODEL_ID, "adapter": ADAPTER, "arm": arm, "wraps": wraps,
            "wraps_model_hash": model_hash(wraps) if wraps else None,
            "fan_deg": list(FAN_DEG), "horizon_step_s": HORIZON_STEP_S,
            "corridor_path_test": CORRIDOR_PATH_TEST,
            "corridor_margin_m": corridor_margin_m(csp),
            **csp_record(csp)}


def write_csp(run_dir: str | Path, csp: CSP) -> Path:
    """The consumed CSP as <run_dir>/vla_prefix/csp_g0.json (the form
    ConstraintCompiler.write_csp writes: indent 2, LF). An earlier flight's
    file under the same run directory is replaced."""
    p = Path(run_dir) / PREFIX_DIR / CSP_FILE
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(csp.model_dump_json(indent=2) + "\n")
    return p


def _clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _turned(a: Action4D, deg: float) -> Action4D:
    """`a` with its horizontal velocity turned `deg` clockwise seen from
    above (x North, y East), its speed, climb and yaw rate kept."""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return Action4D(vx=a.vx * c - a.vy * s, vy=a.vx * s + a.vy * c,
                    vz_up=a.vz_up, yaw_rate=a.yaw_rate)


class PrefixConsumer:
    """A pilot law flown through the CSP's action sets (module docstring).

        pilot = PrefixConsumer(GotoPilot(...), csp, policy)   # the on arm
        pilot = PrefixConsumer(GotoPilot(...))                # the off arm
        action = pilot.act(state)                             # world frame

    `law` has `act(state)` or is a callable of the state. `policy` is the
    policy the CSP was compiled from (the zone and corridor geometry behind
    the CSP's rule ids; a CSP of another policy is refused here, before
    anything flies). `stats` counts what the consumer did, tick by tick:
    `clamped` (a range or the band changed the candidate), `turned` (a fan
    heading was flown), `stopped`, `no_admissible` (the candidate handed on
    though inadmissible), `nonfinite_passed` (a non-finite candidate is the
    Shield's contract to refuse, never repaired here)."""

    def __init__(self, law, csp: CSP | None = None, policy=None, *,
                 wraps: str | None = None) -> None:
        self._law = law.act if hasattr(law, "act") else law
        if not callable(self._law):
            raise TypeError("a pilot law has act(state) or is callable")
        self.csp = csp
        self.policy = policy
        self.wraps = wraps
        self.arm = "off" if csp is None else "on"
        self._side = 1.0
        self.stats: dict[str, Any] = {"ticks": 0, "clamped": 0, "turned": 0, "stopped": 0,
                                      "no_admissible": 0, "nonfinite_passed": 0,
                                      "max_turn_deg": 0.0}
        self._ir = None
        self._horizons: tuple[float, ...] = ()
        self._tubes: tuple = ()
        self._zones: tuple = ()
        if csp is not None:
            if policy is None:
                raise ValueError("a CSP is consumed with the policy it was compiled from "
                                 "(its rule ids name that policy's zones and corridors)")
            if not policy.matches_hash(csp.policy_hash):
                raise ValueError(f"CSP is for {csp.policy_hash}, policy is "
                                 f"{policy.policy_hash}")
            self._ir = compiled_ir(policy)
            look = float(csp.lookahead_s)
            n = max(1, int(math.ceil(look / HORIZON_STEP_S - 1e-9)))
            self._horizons = tuple(look * (k + 1) / n for k in range(n))
            by_id = {c.id: c for c in policy.constraints}
            margin = corridor_margin_m(csp)
            tubes = []
            for cid in csp.forbidden_action_set.no_translate_out_of_corridors:
                c = by_id[cid]
                # The Shield's tube (Shield._corridor_tube), which contains
                # every point its distance check accepts.
                line = LineString(c.points())
                tube = line.buffer(c.half_width_m / math.cos(math.pi / 64.0), quad_segs=16)
                shapely.prepare(tube)
                # The margin's inner tube and the true tube: buffered by the
                # plain distance, so their chords cut INSIDE the circle and
                # every point of the inner one is at most half - margin from
                # the centerline, of the true one at most half (module
                # docstring, THE CORRIDOR MARGIN). Both None for a tube no
                # wider than twice the margin: the Shield's tube alone is
                # tested there (the "segment" test as pab2 flew it).
                inner = true = None
                if c.half_width_m - margin > 0.0:
                    inner = line.buffer(c.half_width_m - margin, quad_segs=16)
                    true = line.buffer(c.half_width_m, quad_segs=16)
                    shapely.prepare(inner)
                    shapely.prepare(true)
                tubes.append((cid, tube, inner, true))
            self._tubes = tuple(tubes)
            rings = {r.rule.id: r.buffered for r in self._ir.fences}
            zones = []
            for fid in csp.forbidden_action_set.no_translate_into_polygons:
                z = by_id[fid]
                ring = rings.get(fid)
                if ring is None:
                    # A zone the IR has no ring for keeps the sampled test.
                    continue
                shapely.prepare(ring)
                zones.append((fid, ring,
                              float(getattr(z, "altitude_floor_m", -math.inf)),
                              float(getattr(z, "altitude_ceiling_m", math.inf))))
            self._zones = tuple(zones)

    def record(self) -> dict[str, Any]:
        return consumer_record(self.arm, self.csp, self.wraps)

    # ------------------------------------------------------------------ #

    def _clamp(self, a: Action4D, state: State) -> Action4D:
        al = self.csp.allowed_action_set
        vx, vy, vz, yr = a.vx, a.vy, a.vz_up, a.yaw_rate
        cap = al.horizontal_speed_max_mps
        speed = math.hypot(vx, vy)
        if cap is not None and speed > cap:
            vx, vy = vx * cap / speed, vy * cap / speed
        if al.vx_range_mps is not None:
            vx = _clip(vx, *al.vx_range_mps)
        if al.vy_range_mps is not None:
            vy = _clip(vy, *al.vy_range_mps)
        band = al.altitude_band_m_agl
        rng = al.vz_range_mps
        if band is not None and math.isfinite(state.up):
            look = float(self.csp.lookahead_s)
            lo_v, hi_v = (band[0] - state.up) / look, (band[1] - state.up) / look
            # Inside the band one lookahead ahead...
            vz = _clip(vz, lo_v, hi_v)
            if rng is not None:
                # ...and from outside it, toward it as fast as the range
                # allows (never past its far edge within the lookahead).
                if state.up < band[0]:
                    vz = max(vz, min(rng[1], hi_v))
                elif state.up > band[1]:
                    vz = min(vz, max(rng[0], lo_v))
        if rng is not None:
            # After the band: where the two disagree the range wins.
            vz = _clip(vz, *rng)
        if al.yaw_rate_max_dps is not None:
            lim = math.radians(al.yaw_rate_max_dps)
            yr = _clip(yr, -lim, lim)
        if (vx, vy, vz, yr) == (a.vx, a.vy, a.vz_up, a.yaw_rate):
            return a
        return Action4D(vx=vx, vy=vy, vz_up=vz, yaw_rate=yr)

    def inadmissible(self, a: Action4D, state: State) -> list[str]:
        """The polygons `a` would translate into and the corridors it would
        leave within the lookahead ("polygon:<id>" / "corridor:<id>", as
        action_violations names them), at the first horizon that has any;
        then, along the straight path to the lookahead, the zones whose ring
        it crosses from outside (where its altitudes meet the zone's band)
        and the corridors whose tube it leaves from inside (module docstring:
        a path from inside a ring or outside a tube is the Shield's to
        recover, and is judged as before)."""
        for h in self._horizons:
            geo = [v for v in action_violations(a, self.csp, state, self.policy,
                                                dt=h, ir=self._ir)
                   if v.startswith(_GEO)]
            if geo:
                return geo
        if not (self._tubes or self._zones) or not all(
                math.isfinite(v) for v in (state.x, state.y, state.up)):
            return []
        look = float(self.csp.lookahead_s)
        x1, y1 = state.x + a.vx * look, state.y + a.vy * look
        if math.hypot(x1 - state.x, y1 - state.y) <= 1e-9:
            return []
        start = Point(state.x, state.y)
        path = LineString([(state.x, state.y), (x1, y1)])
        up0, up1 = state.up, state.up + a.vz_up * look
        lo, hi = min(up0, up1), max(up0, up1)
        out = [f"polygon:{fid}" for fid, ring, floor, ceil in self._zones
               if lo <= ceil and hi >= floor
               and not area_covers(ring, start) and area_meets(path, ring)]
        for cid, tube, inner, true in self._tubes:
            if not area_covers(tube, start):
                continue                    # outside: the Shield's to recover
            if not area_covers(tube, path):
                out.append(f"corridor:{cid}")
            elif inner is None:
                continue
            elif area_covers(inner, start):
                # Inside the margin: the path keeps it.
                if not area_covers(inner, path):
                    out.append(f"corridor:{cid}")
            elif not area_covers(inner, Point(x1, y1)):
                # Already in the margin band: the path ends back inside it...
                out.append(f"corridor:{cid}")
            elif area_covers(true, start) and not area_covers(true, path):
                # ...and on the way never passes the true wall, which the
                # Shield's tube (a superset) would let it do at a dog-leg's
                # inner corner (module docstring). A start already past the
                # true wall (in the sliver between it and the Shield's
                # tube, 20.0-20.024 m for a 20 m half-width) cannot
                # keep it and is judged by the end point alone.
                out.append(f"corridor:{cid}")
        return out

    def act(self, state: State) -> Action4D:
        a = self._law(state)
        self.stats["ticks"] += 1
        if self.csp is None:
            return a
        if not all(math.isfinite(v) for v in (a.vx, a.vy, a.vz_up, a.yaw_rate)):
            self.stats["nonfinite_passed"] += 1
            return a
        b = self._clamp(a, state)
        if b is not a:
            self.stats["clamped"] += 1
        if not self.inadmissible(b, state):
            return b
        moving = math.hypot(b.vx, b.vy) > 1e-9
        if moving:
            for deg in FAN_DEG:
                sides = (self._side,) if deg >= 180.0 else (self._side, -self._side)
                for side in sides:
                    c = _turned(b, side * deg)
                    if not self.inadmissible(c, state):
                        self._side = side
                        self.stats["turned"] += 1
                        self.stats["max_turn_deg"] = max(self.stats["max_turn_deg"], deg)
                        return c
            stop = Action4D(vx=0.0, vy=0.0, vz_up=b.vz_up, yaw_rate=b.yaw_rate)
            if not self.inadmissible(stop, state):
                self.stats["stopped"] += 1
                return stop
        self.stats["no_admissible"] += 1
        return b
