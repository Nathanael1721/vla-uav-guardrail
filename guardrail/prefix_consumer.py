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
steps, both read from the CSP's action sets and nothing else:

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
Shield's own point verdicts over the CSP's rule ids - at the lookahead and at
every HORIZON_STEP_S before it (an end-point test alone steps over a zone
narrower than speed x lookahead). No geometry is written here.

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

from .compiler import (DEFAULT_LOOKAHEAD_S, ConstraintCompiler, Mission,
                       action_set_adapter, action_violations, compiled_ir)
from .csp import CSP, csp_content_hash
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
        action_violations names them), at the first horizon that has any."""
        for h in self._horizons:
            geo = [v for v in action_violations(a, self.csp, state, self.policy,
                                                dt=h, ir=self._ir)
                   if v.startswith(_GEO)]
            if geo:
                return geo
        return []

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
