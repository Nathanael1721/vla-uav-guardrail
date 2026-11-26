"""Generation N's CSP from the Shield to the pilot that consumes it (audit
card WP4-19, 2026-11-26).

WHAT THE GRANT ASKS

The final demo gate (recommendation.md): "All four components in the
implementation plan [Policy DSL, Prefix Compiler, Safety Shield, Stress
Testing], exercised by a non-trivial scenario from the Stress Testing
scenario suite". The suite's templates (stress-testing.md: dynamic_nfz_movement,
time_window_switch, radius_scaling, corridor_swap) all change the rules in
flight, so the Prefix Compiler's output has to follow a hot-apply to the
pilot: until this module the prefix consumer (guardrail/prefix_consumer.py)
flew the CSP compiled before take-off, and the stress harness refused every
consumer arm of a scenario with a mid-flight event.

WHAT TRAVELS, AND WHO COMPILES WHAT

  /shield/csp       (std_msgs/String JSON, latched: reliable, transient_local,
                    depth 1) - published by the Shield node after it has
                    written each generation's policy_g<N>.json and
                    csp_g<N>.json (take-off and every accepted hot-apply):
                    `shield_csp_message`. It carries the generation, the
                    policy hash, the generation's POLICY (canonical JSON, the
                    bytes of policy_g<N>.json), the Shield's own CSP for it
                    (the csp_g<N>.json document, or None where the compile
                    failed), the rule clock's reading the generation was
                    recorded at and the mission time of the hot-apply.
  /vla/prefix_csp   (std_msgs/String JSON, latched) - published by the VLA
                    stub's prefix-on arm at take-off and after every swap or
                    refusal: `GenerationFollower`. The Shield node stamps
                    each of its rows with the generation last announced
                    (`pilot_csp_generation`) and keeps the announcements in
                    metrics.json, so the harness can check the pilot followed.

The pilot needs the POLICY, not only the CSP: the consumer resolves the CSP's
rule ids to the policy's compiled geometry (a spawned zone's polygon is in
the policy, not in the CSP). From that policy it compiles its own view of
generation N with the Prefix Compiler (`generation_csp`:
prefix_consumer.static_csp, the spatial filter off, at the generation's rule
clock reading, static over the rest of the flight) - the same call it makes
for generation 0 at take-off and the same call the headless harness makes at
the event, so one generation gives one action set on every backend. The
Shield's own CSP in the message (compiled with the mission's spatial filter)
is recorded beside it (`shield_csp_content_hash`), not flown: a CSP filtered
to the mission's box at the hot-apply instant is not the whole-flight view
the consumer flies.

WHAT IS CHECKED (`tracking_problems`, the stress harness's backstop)

  * every Shield generation N > 0 is reached by the pilot: the first Shield
    row whose `pilot_csp_generation` is N or more comes at most LAG_MAX_S
    after the first row of generation N (a flight that ends sooner than that
    after the hot-apply is not held to it);
  * the pilot is never ahead of the Shield;
  * the pilot refused no generation (a refusal leaves it on the old CSP);
  * each announced generation's CSP is the one the harness compiles from the
    run's own policy_g<N>.json at the same clock reading (its action sets'
    SHA-256): the binding generation 0 already has (prefix_flight_problems).

LAG_MAX_S, 1.0 s, is ten Shield ticks: the Shield writes generation N's
files (its CSP compile among them) after the hot-apply tick's setpoint, the
message crosses DDS, the stub compiles its own CSP (one compile per minute
boundary left in the flight) and announces it, and the Shield reads the
announcement on a later tick. For the cells of
experiments/profiles/prefix_dyn.yaml each compile took 1-2 ms (median of 20)
on the desktop, 45 ms at most on a cold first call (measured 2026-11-26);
1.0 s leaves room for a slower computer (the Orin, not measured here), the
DDS hop and the tick quantisation on both nodes. Every flight records its measured lag per generation
(`generation_lag`), so a report quotes the number flown, not this bound.
The Shield enforces generation N from its hot-apply tick whatever the pilot
holds: a stale CSP at the pilot costs the A/B measurement, never safety.

HEADLESS. The stress harness swaps the consumer's CSP at the event, before
the tick's pilot call (experiments/sweep_scenarios.py run_episode), so the
lag there is 0 ticks by construction.
"""
from __future__ import annotations

import json
from datetime import datetime
from typing import Any, Callable

from . import prefix_consumer as PC
from .csp import CSP, csp_content_hash

CSP_TOPIC = "/shield/csp"
PILOT_CSP_TOPIC = "/vla/prefix_csp"
SCHEMA = "shield-csp/1"
PILOT_SCHEMA = "vla-prefix-csp/1"
# The latest generation N's CSP may reach the pilot after the Shield's
# hot-apply tick (module docstring).
LAG_MAX_S = 1.0
# The flag both nodes read (each gets every argument): "on" makes the Shield
# node publish CSP_TOPIC and read PILOT_CSP_TOPIC, and the stub's prefix-on
# arm read CSP_TOPIC and publish PILOT_CSP_TOPIC. Default off: nothing here
# runs and both nodes fly as they did.
FLAG = "--csp-topic"


def clock_reading(now: datetime | None) -> str | None:
    """The rule clock's reading as a generation records it
    (guardrail.replay.EpisodeRecord.write_generation: ISO, to the second)."""
    return None if now is None else now.isoformat(timespec="seconds")


def parse_clock(reading: str | None) -> datetime | None:
    if reading is None or reading == "":
        return None
    return datetime.fromisoformat(str(reading))


def shield_csp_message(row: dict, policy, *, t: float | None = None,
                       csp_doc: dict | None = None) -> dict:
    """The /shield/csp document of one generation: `row` is the
    generation's record (EpisodeRecord.write_generation's: generation,
    policy_hash, reason, rule_clock_reading), `policy` the generation's
    policy as written to policy_g<N>.json, `csp_doc` the csp_g<N>.json
    document (None when it could not be compiled), `t` the mission time of
    the hot-apply (None at take-off)."""
    if int(row["generation"]) != int(policy.generation):
        raise ValueError(f"the generation row ({row['generation']}) is not the policy's "
                         f"({policy.generation})")
    return {"schema": SCHEMA, "generation": int(row["generation"]),
            "policy_hash": policy.policy_hash, "reason": row.get("reason"),
            "rule_clock_reading": row.get("rule_clock_reading"),
            "t": None if t is None else round(float(t), 4),
            "policy": json.loads(policy.canonical_bytes()),
            "csp": csp_doc}


def read_policy(doc: dict):
    """The Policy a /shield/csp document carries, read as a run's
    policy_g<N>.json is (no runtime gate: a hot-applied zone is a
    dynamic_nfz, which a policy FILE may not fly). Raises ValueError when
    it is not the generation and hash the document names."""
    from .models import policy_from_raw
    if not isinstance(doc.get("policy"), dict):
        raise ValueError("the message carries no policy")
    pol = policy_from_raw(doc["policy"], source=f"{CSP_TOPIC} g{doc.get('generation')}",
                          runtime=False)
    if not pol.matches_hash(str(doc.get("policy_hash"))):
        raise ValueError(f"the message's policy is {pol.policy_hash}, it names "
                         f"{doc.get('policy_hash')}")
    if int(pol.generation) != int(doc.get("generation")):
        raise ValueError(f"the message's policy is generation {pol.generation}, it names "
                         f"{doc.get('generation')}")
    return pol


def generation_csp(policy, mission, *, clock: str | datetime | None,
                   t_s: float | None, max_s: float | None) -> CSP:
    """The CSP the consumer flies generation `policy.generation` on:
    prefix_consumer.static_csp for `mission` (consumer_mission's), at the
    generation's rule clock reading, static over the rest of the flight
    (`max_s` - `t_s`; over the whole `max_s` when the hot-apply's time is
    not known). Raises ValueError where no such CSP exists: a rule that
    still switches with the clock after the generation (a windowed switch),
    a moving zone."""
    now = parse_clock(clock) if not isinstance(clock, datetime) else clock
    if max_s is None:
        rest = None
    elif t_s is None:
        rest = float(max_s)
    else:
        rest = max(0.0, float(max_s) - float(t_s))
    return PC.static_csp(policy, mission, now=now, duration_s=rest)


class GenerationFollower:
    """The pilot's side: generation N's CSP into a PrefixConsumer.

        f = GenerationFollower(consumer, mission, max_s=...)
        publish(f.announcement())           # at take-off: generation 0
        ann = f.take(json.loads(msg.data))  # each /shield/csp message
        if ann is not None: publish(ann)

    A message of a generation not above the one flown is ignored (None: the
    latched take-off message, a repeat). Otherwise the generation's policy
    is read, its CSP compiled (generation_csp) and swapped in
    (PrefixConsumer.swap: one assignment, the next tick decides on it); a
    message that cannot be flown is announced as refused and the consumer
    keeps the CSP it holds. Single-threaded by design: a ROS node calls
    take() from its executor, which runs its ticks too."""

    def __init__(self, consumer: PC.PrefixConsumer, mission, *,
                 max_s: float | None = None) -> None:
        if consumer.arm != "on":
            raise ValueError("only the prefix-on arm follows the Shield's generations")
        self.consumer = consumer
        self.mission = mission
        self.max_s = max_s
        self.announced: list[dict] = []
        self.refused: list[dict] = []

    def announcement(self) -> dict:
        """What the pilot flies now, as PILOT_CSP_TOPIC carries it."""
        c = self.consumer
        rec = PC.csp_record(c.csp)
        last = c.swaps[-1] if c.swaps else None
        return {"schema": PILOT_SCHEMA, "generation": c.generation,
                "policy_hash": rec["policy_hash"],
                "action_set_sha256": rec["action_set_sha256"],
                "csp_content_hash": rec["csp_content_hash"],
                "source": "take-off" if last is None else CSP_TOPIC,
                "rule_clock_reading": None if last is None else last.get("rule_clock_reading"),
                "t_shield": None if last is None else last.get("t"),
                "shield_csp_content_hash": (None if last is None
                                            else last.get("shield_csp_content_hash"))}

    def take(self, doc: Any) -> dict | None:
        if not isinstance(doc, dict) or doc.get("schema") != SCHEMA:
            return self._refuse(None, f"not a {SCHEMA} document")
        g = doc.get("generation")
        if not isinstance(g, int) or isinstance(g, bool):
            return self._refuse(None, f"generation {g!r} is not an integer")
        if g <= int(self.consumer.generation):
            return None
        try:
            pol = read_policy(doc)
            csp = generation_csp(pol, self.mission, clock=doc.get("rule_clock_reading"),
                                 t_s=doc.get("t"), max_s=self.max_s)
            shield_hash = None
            if isinstance(doc.get("csp"), dict):
                shield_hash = csp_content_hash(CSP.model_validate(doc["csp"]))
            rec = self.consumer.swap(csp, pol, t=doc.get("t"), source=CSP_TOPIC)
        except Exception as e:                                   # noqa: BLE001
            return self._refuse(g, f"{type(e).__name__}: {e}")
        rec.update(rule_clock_reading=doc.get("rule_clock_reading"),
                   shield_csp_content_hash=shield_hash)
        ann = self.announcement()
        self.announced.append(ann)
        return ann

    def _refuse(self, g, why: str) -> dict:
        ann = {"schema": PILOT_SCHEMA, "generation": g, "refused": why,
               "flying_generation": self.consumer.generation}
        self.refused.append(ann)
        return ann


def _gen(v) -> int | None:
    return v if isinstance(v, int) and not isinstance(v, bool) else None


def generation_lag(rows) -> dict:
    """How the pilot's CSP generation followed the Shield's, over rows that
    carry both (`generation`, the Shield's; `pilot_csp_generation`, the
    pilot's last announced): per Shield generation N > 0, the first row of
    N (`shield_t`), the first row at or after it whose pilot generation is N
    or more (`pilot_t`, None if none), and the lag between them in seconds
    and rows; `stale_rows` (pilot behind the Shield) and `ahead_rows` (pilot
    ahead: never expected)."""
    rows = [r for r in rows if isinstance(r, dict)]
    out: dict[str, Any] = {"rows": len(rows),
                           "pilot_rows": sum(1 for r in rows if "pilot_csp_generation" in r),
                           "generations": [], "stale_rows": 0, "ahead_rows": 0}
    for r in rows:
        g, p = _gen(r.get("generation")), _gen(r.get("pilot_csp_generation"))
        if g is not None and p is not None:
            out["stale_rows"] += p < g
            out["ahead_rows"] += p > g
    seen = sorted({g for g in (_gen(r.get("generation")) for r in rows)
                   if g is not None and g > 0})
    for n in seen:
        k0 = next(i for i, r in enumerate(rows)
                  if _gen(r.get("generation")) is not None and r["generation"] >= n)
        k1 = next((i for i in range(k0, len(rows))
                   if _gen(rows[i].get("pilot_csp_generation")) is not None
                   and rows[i]["pilot_csp_generation"] >= n), None)
        t0 = rows[k0].get("t")
        t1 = None if k1 is None else rows[k1].get("t")
        out["generations"].append({
            "generation": n, "shield_t": t0, "pilot_t": t1,
            "lag_s": (None if t1 is None or t0 is None else round(float(t1) - float(t0), 4)),
            "lag_rows": None if k1 is None else k1 - k0})
    return out


def tracking_problems(rows, *, announcements=None, generations: dict | None = None,
                      csp_for: Callable[[Any, Any, Any], CSP] | None = None,
                      lag_max_s: float = LAG_MAX_S) -> list[str]:
    """Why the pilot did not follow the Shield's generations (module
    docstring, WHAT IS CHECKED); [] when it did. `announcements`: the
    pilot's PILOT_CSP_TOPIC documents as the Shield node kept them;
    `generations`: {N: Policy} from the run's policy_g<N>.json;
    `csp_for(policy, rule_clock_reading, t_shield)`: the harness's own
    generation CSP for the binding check (none: not checked)."""
    rows = [r for r in rows if isinstance(r, dict)]
    if not any("pilot_csp_generation" in r for r in rows):
        return [f"no row carries the pilot's CSP generation: the VLA node announced none "
                f"on {PILOT_CSP_TOPIC} (a node without {FLAG}, the flag not sent, or the "
                f"topic refused), so generation N's CSP is not shown to reach the pilot"]
    out: list[str] = []
    lag = generation_lag(rows)
    end_t = next((r.get("t") for r in reversed(rows) if r.get("t") is not None), None)
    for e in lag["generations"]:
        if e["pilot_t"] is None:
            if end_t is None or e["shield_t"] is None or \
                    float(end_t) - float(e["shield_t"]) >= lag_max_s - 1e-9:
                out.append(f"the pilot never flew generation {e['generation']}'s CSP (the "
                           f"Shield's from t={e['shield_t']} s)")
        elif e["lag_s"] is not None and e["lag_s"] > lag_max_s + 1e-9:
            out.append(f"generation {e['generation']}'s CSP reached the pilot {e['lag_s']} s "
                       f"after the Shield's hot-apply (over {lag_max_s:g} s)")
    if lag["ahead_rows"]:
        out.append(f"on {lag['ahead_rows']} row(s) the pilot announced a generation above "
                   f"the Shield's")
    for a in announcements or ():
        if not isinstance(a, dict):
            continue
        if a.get("refused"):
            out.append(f"the pilot refused generation {a.get('generation')}'s CSP "
                       f"({a['refused']}) and flew on generation "
                       f"{a.get('flying_generation')}'s")
            continue
        g = _gen(a.get("generation"))
        if g is None or g <= 0 or generations is None or csp_for is None:
            continue
        pol = generations.get(g)
        if pol is None:
            out.append(f"the pilot flew generation {g}, which the run's policy files do not "
                       f"hold")
            continue
        if not pol.matches_hash(str(a.get("policy_hash"))):
            out.append(f"the pilot's generation {g} CSP is for {a.get('policy_hash')}, the "
                       f"Shield's policy_g{g}.json is {pol.policy_hash}")
            continue
        try:
            want = PC.action_set_sha256(csp_for(pol, a.get("rule_clock_reading"),
                                                a.get("t_shield")))
        except Exception as e:                                   # noqa: BLE001
            out.append(f"generation {g}'s CSP cannot be compiled here ({type(e).__name__}: "
                       f"{e})")
            continue
        if a.get("action_set_sha256") != want:
            out.append(f"the pilot's generation {g} CSP ({a.get('action_set_sha256')}) is not "
                       f"the one the run's policy_g{g}.json compiles to ({want})")
    return out
