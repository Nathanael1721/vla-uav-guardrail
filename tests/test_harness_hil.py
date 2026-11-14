"""A harness-driven hil episode (2026-11-14; audit cards WP4-03, WP4-05,
WP4-11): the scenario's own zone events forwarded by the harness over REST,
EmitEvent on a rail episode, and the dry run of tools/harness_hil_episode.py.

    python tests/test_harness_hil.py

NOTHING HERE FLIES. The rig is tests/test_harness_rail.py's (`_Rig`: the fake
hil rig tests/test_sweep.py flies its campaign on, whose mission also serves
guardrail/api.py's own handlers on a live Shield when launched with
--hotapply-port), with one change: the fake writes generation 0 as the policy
the mission STARTED with (a node writes policy_g0.json at its start), where
the campaign's fake writes the policy it ended with, which for a scenario
with events is another generation. These tests show what the harness
launches, forwards, refuses, records and bundles - nothing about ArduPilot,
MAVROS, the Orin, the real listener or its timing.

  * every library event with a REST form is applied by the REST handler
    exactly as the mission node applies it from --events
    (guardrail.scenario_events.apply_event with shield_t=None): the same
    policy hash and generation after each event, on every library cell whose
    events all have one; the null: the spawn body without the defaults the
    mission writes is refused by the endpoint;
  * a scenario's REST timeline is its events in flight order with their
    routes (a rotate / scale of a zone as PATCH /dynamic_nfz/<id>); what has
    no REST form (a zone placed against the vehicle, a change of motion, an
    expected refusal) is named and refused;
  * StartEpisode drive_scenario_events launches the mission WITHOUT the
    scenario's events (no --events, no events.json pushed) and forwards each
    at its mission time; the bundle carries the grant's six names, every
    generation, and each forward marked `scenario`;
  * a scenario event that never reaches the flying Shield fails the episode,
    unscored and unbundled, naming it;
  * EmitEvent on a rail episode is forwarded as its REST body (translate,
    rotate, scale as PATCH), refused when it has no REST form;
  * the dry run plans what StartEpisode launches, writes and sends nothing;
  * tools/harness_hil_episode.py flies dyn-nfz-rotate-step over gRPC on the
    fake rig and checks the six names.
"""
from __future__ import annotations

import contextlib
import dataclasses
import importlib.util
import io
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("GRPC_VERBOSITY", "ERROR")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import api                                          # noqa: E402
from guardrail import harness_service as H                         # noqa: E402
from guardrail import scenario_events as SE                        # noqa: E402
from guardrail.scenario_spec import ScenarioEvent                  # noqa: E402
from guardrail.shield import Action4D, Shield, State               # noqa: E402


def _load(name: str, file: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tests" / file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


TR = _load("test_harness_rail_for_hil", "test_harness_rail.py")
TS, S = TR.TS, TR.S
SKIP = "SKIP"
CAMPAIGN = "tsthh"
_TMP: list[Path] = []

FAST = """
scenario_id: harness-driven-zone
template: dynamic_nfz_movement
description: "Open ground at 15 m; a zone off the route spawned, moved, turned and grown."
mission:
  start_pose: {x: -20.0, y: -20.0, up: 15.0}
  target: {x: 40.0, y: -20.0, up: 15.0}
  pilot: {type: goto, speed: 4.0}
policy: {path: policies/sim_demo_policy.yaml}
events:
  - at_sim_t: 0.2
    type: spawn_polygon_fence
    payload: {id: hz, vertices: [{x: 210, y: 230}, {x: 220, y: 230},
                                 {x: 220, y: 250}, {x: 210, y: 250}]}
  - at_sim_t: 0.5
    type: translate_polygon_fence
    payload: {dx_m: 5.0, dy_m: -2.0}
  - at_sim_t: 0.8
    type: rotate_polygon_fence
    payload: {id: hz, angle_deg: 30.0}
  - at_sim_t: 1.1
    type: scale_radius
    payload: {id: hz, factor: 1.5}
expected_kpis:
  P0_escape_rate: 0
"""


def _tmp(prefix: str) -> Path:
    import tempfile
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


class _G0:
    """A policy stand-in whose canonical bytes are generation 0's."""

    def __init__(self, blob: bytes):
        self._b = blob

    def canonical_bytes(self) -> bytes:
        return self._b


class _HilRig(TR._Rig):
    """TR._Rig, writing policy_g0.json as the policy the mission started
    with (EpisodeRun.generations[0]) - what the Shield node writes - instead
    of the policy the headless flight ended with. Records the files the
    mission was pushed."""

    def run_mission(self, args, tag, shield, timeout_s):
        self.pushed = sorted(p.name for p in (self.remote / S.CAMPAIGN_DIR / tag).iterdir())
        saved = TS.S.run_episode

        def first_generation(*a, **kw):
            run = saved(*a, **kw)
            return dataclasses.replace(run, policy=_G0(run.generations[0]))
        TS.S.run_episode = first_generation
        try:
            return super().run_mission(args, tag, shield, timeout_s)
        finally:
            TS.S.run_episode = saved


def _hil_harness(**rig_kw):
    td = _tmp("hhil_")
    rigs: list = []
    cfg = S.HilConfig(store=str(td / "store"), bundles_dir=str(TS._signed_like_bundles(td)))
    rig = H.RailRig(backends=("hil",), hil_config=cfg, campaign=CAMPAIGN,
                    transport=lambda c: rigs.append(_HilRig(c, **rig_kw)) or rigs[-1])
    return H.Harness(runs_dir=td / "runs", rail=rig), rigs, td


@contextlib.contextmanager
def _family(h, text: str):
    """A family the fake rig's "Orin" can find by its id (it flies the
    library cell its pushed meta names)."""
    fam = h._family_from_yaml(text)
    TS.FAMILIES[fam.scenario_id] = fam
    try:
        yield fam
    finally:
        TS.FAMILIES.pop(fam.scenario_id, None)


def _cleanup():
    for base in (S.CAMPAIGN_DIR, S.RAIL_STRESS_DIR):
        for d in (ROOT / base).glob(CAMPAIGN + "*"):
            shutil.rmtree(d, ignore_errors=True)
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)
    TR._cleanup()


def _ev(at, typ, payload) -> ScenarioEvent:
    return ScenarioEvent.model_validate({"at_sim_t": at, "type": typ, "payload": payload})


# --------------------------------------------------------------------------- #
# ScenarioEvent -> REST body
# --------------------------------------------------------------------------- #

def test_each_event_form_maps_to_the_rest_body_and_route_the_endpoint_takes():
    """rest_body_of_event: the routes, the defaults the mission writes made
    explicit, the zone ids resolved as the mission's EventState would, and
    the forms with no REST body refused with why."""
    verts = [{"x": 1, "y": 2}, {"x": 3, "y": 2}, {"x": 3, "y": 5}]
    spawn = H.rest_body_of_event(_ev(1, "spawn_polygon_fence", {"vertices": verts}), zone_k=2)
    assert spawn == {"type": "dynamic_nfz", "id": "dyn-nfz-2",
                     "vertices": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 2.0},
                                  {"x": 3.0, "y": 5.0}],
                     "priority": "P0", "constraint_type": "hard", "violation_action": "repair",
                     "scope": "mission", "altitude_floor_m": 0.0,
                     "altitude_ceiling_m": 1000.0, "margin_m": 1.0}, spawn
    assert H.rest_route(spawn)[:2] == ("POST", "/hot_apply")
    moving = H.rest_body_of_event(_ev(1, "spawn_polygon_fence", {
        "id": "m", "vertices": verts, "motion": {"vx_mps": 1.0}, "margin_m": 2}))
    assert moving["motion"] == {"vx_mps": 1.0} and moving["margin_m"] == 2.0
    cases = [
        (_ev(2, "translate_polygon_fence", {"dx_m": 4, "dy_m": -1}),
         ("PATCH", "/dynamic_nfz/z9", {"op": "translate", "dx": 4.0, "dy": -1.0})),
        (_ev(2, "rotate_polygon_fence", {"id": "z1", "angle_deg": 45}),
         ("PATCH", "/dynamic_nfz/z1", {"op": "rotate", "angle_deg": 45.0})),
        (_ev(2, "scale_radius", {"id": "z1", "factor": 0.5}),
         ("PATCH", "/dynamic_nfz/z1", {"op": "scale", "factor": 0.5})),
    ]
    for ev, want in cases:
        assert H.rest_route(H.rest_body_of_event(ev, last_zone="z9")) == want, ev
    off = H.rest_body_of_event(_ev(3, "deactivate_rule", {"rule_id": "nfz-1"}))
    assert off == {"type": "time_window_switch", "constraint_type": "hard", "priority": "P0",
                   "violation_action": "project_fix", "scope": "mission",
                   "target_id": "nfz-1", "active": False}, off
    win = {"days": ["Mon"], "start_time": "08:00", "end_time": "09:00"}
    on = H.rest_body_of_event(_ev(3, "activate_rule", {"rule_id": "nfz-1", "window": win}))
    assert on["id"] == "sw-nfz-1-on" and on["valid_time"] == {"recurrence": win}, on
    swap = H.rest_body_of_event(_ev(4, "swap_corridor", {
        "target_id": "c1", "centerline": [[0, 0], {"x": 10, "y": 0}], "width_m": 6}))
    assert swap["id"] == "swap-c1" and swap["centerline"] == [{"x": 0.0, "y": 0.0},
                                                               {"x": 10.0, "y": 0.0}], swap
    # no REST form, with why; nothing to name the zone by
    for ev, code, word in (
            (_ev(1, "spawn_polygon_fence", {"width_m": 10, "height_m": 5,
                                            "ahead_of_vehicle_m": 20}), "UNIMPLEMENTED",
             "vehicle"),
            (_ev(1, "translate_polygon_fence", {"id": "z", "mps": 2,
                                                "direction": "along"}), "UNIMPLEMENTED",
             "motion"),
            (_ev(1, "rotate_polygon_fence", {"id": "z", "deg_per_s": 5}), "UNIMPLEMENTED",
             "motion"),
            (_ev(1, "deactivate_rule", {"rule_id": "r", "expect_refused": True}),
             "UNIMPLEMENTED", "expect_refused"),
            (_ev(1, "scale_radius", {"factor": 2}), "INVALID_ARGUMENT", "`id`"),
            (_ev(1, "spawn_polygon_fence", {"vertices": verts}), "INVALID_ARGUMENT", "`id`"),
            (_ev(1, "spawn_polygon_fence", {"id": "z", "vertices": verts[:2]}),
             "INVALID_ARGUMENT", "three")):
        code_, why = TR._refused(H.rest_body_of_event, ev)
        assert code_ == code and word in why, (ev.type, code_, why)


def test_the_rest_handler_applies_each_library_event_as_the_mission_node_does():
    """For every library cell whose events all have a REST form, a Shield
    given the events as the mission node gives them (scenario_events.
    apply_event, shield_t=None: the Shield's own clock) and a twin given the
    REST bodies through guardrail/api.py's handlers hold the same policy -
    hash and generation - after each event. The null: the spawn body
    without the fields the mission writes for it is refused by the endpoint
    (a REST dynamic_nfz states its violation_action)."""
    flown = refused = 0
    for sid, fam in TS.FAMILIES.items():
        for cell in fam.cells(None, 1):
            spec = cell.spec
            if not spec.events:
                continue
            try:
                timeline = H.scenario_rest_timeline(spec)
            except H.HarnessError as e:
                assert e.code == "FAILED_PRECONDITION", (cell.episode_id, e)
                refused += 1
                continue
            pol = spec.policy.load(ROOT)
            node = Shield(pol.model_copy(deep=True), escalation=False)
            rest = Shield(pol.model_copy(deep=True), escalation=False)
            st = State(x=0.0, y=0.0, up=15.0)
            for sh in (node, rest):
                sh.filter(st, Action4D())                 # the mission has started
            es = SE.EventState(shields=[node])
            order = sorted(spec.events, key=lambda e: e.at_sim_t)
            assert [x["type"] for x in timeline] == [e.type for e in order]
            for ev, x in zip(order, timeline):
                SE.apply_event(ev, es, st, (1.0, 0.0), ev.at_sim_t, shield_t=None)
                method, path, payload = H.rest_route(x["body"])
                if method == "POST":
                    api.apply_event(rest, x["body"])
                else:
                    api.apply_nfz_edit(rest, x["body"]["id"], payload)
                assert (rest.policy.generation, rest.policy.policy_hash) == \
                    (node.policy.generation, node.policy.policy_hash), (cell.episode_id, ev)
            flown += 1
    assert flown >= 10 and refused >= 10, (flown, refused)
    # the null
    step = TS.FAMILIES["dyn-nfz-rotate-step"].first.spec
    body = H.scenario_rest_timeline(step)[0]["body"]
    bare = {k: v for k, v in body.items() if k not in ("violation_action", "margin_m")}
    sh = Shield(step.policy.load(ROOT), escalation=False)
    try:
        api.apply_event(sh, bare)
    except api.ApiError as e:
        assert e.status == 422 and "violation_action" in e.detail, e.detail
    else:
        raise AssertionError("the endpoint took a dynamic_nfz with no violation_action")


def test_a_scenario_timeline_is_its_events_in_flight_order_with_their_routes():
    step = TS.FAMILIES["dyn-nfz-rotate-step"].first.spec
    tl = H.scenario_rest_timeline(step)
    assert [(x["at_sim_t"], x["type"], x["method"], x["path"]) for x in tl] == [
        (3.0, "spawn_polygon_fence", "POST", "/hot_apply"),
        (8.0, "rotate_polygon_fence", "PATCH", "/dynamic_nfz/bar")], tl
    assert H.rest_route(tl[1]["body"])[2] == {"op": "rotate", "angle_deg": 90.0}
    shrink = next(c for c in TS.FAMILIES["radius-scaling-shrink"].cells(None, 1)
                  if c.episode_id.endswith("factor=0.5")).spec
    assert [H.rest_route(x["body"])[2].get("op") for x in H.scenario_rest_timeline(shrink)] \
        == [None, "scale"]
    spin = TS.FAMILIES["dyn-nfz-rotate-spin"].first.spec
    code, why = TR._refused(H.scenario_rest_timeline, spin)
    assert code == "FAILED_PRECONDITION" and "#0 spawn_polygon_fence" in why \
        and "#1 rotate_polygon_fence" in why and "--events" in why, why
    code, why = TR._refused(H.scenario_rest_timeline, TS.BY_ID["speed-cap"])
    assert code == "INVALID_ARGUMENT" and "no events" in why, why
    # written out of order, and an edit with no id: flight order, the last spawn
    fam = H.Harness(runs_dir=_tmp("hhil_tl_"))._family_from_yaml(FAST.replace(
        "at_sim_t: 0.2", "at_sim_t: 0.45"))
    tl = H.scenario_rest_timeline(fam.first.spec)
    assert [(x["index"], x["at_sim_t"]) for x in tl] == [(0, 0.45), (1, 0.5), (2, 0.8),
                                                         (3, 1.1)], tl
    assert tl[1]["path"] == "/dynamic_nfz/hz", tl[1]


# --------------------------------------------------------------------------- #
# StartEpisode drive_scenario_events on the fake rig
# --------------------------------------------------------------------------- #

def test_a_harness_driven_hil_episode_flies_the_scenarios_events_over_rest():
    """The mission is launched with the scenario's own arguments minus its
    events (no --events, no events.json pushed) plus the listener's flag;
    the harness forwards the four events at their mission times (spawn
    POST, translate / rotate / scale PATCH, the translate with no id on the
    zone spawned last); the run is scored as the campaign scores it, and the
    bundle holds the grant's six names, generations 0-4 and each forward
    marked `scenario`. The null: the same cell launched with the mission
    applying its events carries --events and its events.json."""
    h, rigs, td = _hil_harness(expect=4)
    port = TR._free_port()
    url = f"http://127.0.0.1:{port}"
    with _family(h, FAST) as fam, TR._fast_polls():
        s = h.start_session(backend="hil", rest_endpoint=url, seed=1001)["session_id"]
        ref = h.start_episode(scenario_yaml=FAST, session_id=s, drive_scenario_events=True)
        st = h.status(ref["episode_id"])
        b = h.end_episode(ref["episode_id"])
        h.stop_session(s)
    spec = fam.first.spec
    rig = rigs[0]
    assert ref["scenario_events_via"] == "harness" == st["scenario_events_via"], ref
    assert [q["source"] for q in st["events"]] == ["scenario"] * 4, st["events"]

    # launched without its events
    run = next(c for c in rig.calls if c[0] == "run_mission")
    tag = f"{CAMPAIGN}_001_harness-driven-zone"
    bare = spec.model_copy(update={"events": []})
    own = S.rail_node_args(bare, 1001, TS.DEFAULTS, backend="hil",
                           meta=f"{S.CAMPAIGN_DIR}/{tag}/scenario.json",
                           bundles_dir=str(td / "bundles"))
    assert run[3] == own + [f"--hotapply-port={port}"], (run[3], own)
    assert not any(a.startswith("--events") for a in run[3]) and \
        rig.pushed == ["scenario.json"], rig.pushed

    # forwarded at their mission times, by route
    want = [H.rest_route(x["body"]) for x in H.scenario_rest_timeline(spec)]
    assert [(m, p, b_) for m, p, b_ in rig.requests] == want, (rig.requests, want)
    assert want[1] == ("PATCH", "/dynamic_nfz/hz", {"op": "translate", "dx": 5.0, "dy": -2.0})

    # scored as the campaign scores it; the grant's six names
    run_dir = td / "store" / tag
    score = json.loads((run_dir / S.SCENARIO_SCORE).read_text(encoding="utf-8"))
    d = Path(b["bundle_path"])
    kpi = json.loads((d / "kpi.json").read_text(encoding="utf-8"))
    assert b["status"] == score["status"] == kpi["status"] == "pass", (b["status"], score)
    assert set(b["bundle_files"]) == {"manifest.json", "policy_bundle.tar.gz", "episode.bag2/",
                                      "shield_audit.jsonl", "harness_events.jsonl",
                                      "kpi.json"}
    assert all(v.startswith("written") for v in b["bundle_files"].values()), b["bundle_files"]
    _, _, idx = TR._policy_bundle(d)
    assert [g["generation"] for g in idx["generations"]] == [0, 1, 2, 3, 4] and \
        idx["complete"], idx["generations"]
    rail = kpi["harness"]["rail"]
    assert rail["scenario_events_via"] == "harness" and \
        [x["type"] for x in rail["scenario_events"]] == [e.type for e in spec.events], rail
    ev = TR._events(d)
    assert ev[0]["scenario_events_via"] == "harness"
    fw = [e for e in ev if e["type"] == "hot_apply"]
    assert [(e["source"], e["scenario_event"], e["method"], e["generation"]) for e in fw] == [
        ("scenario", "spawn_polygon_fence", "POST", 1),
        ("scenario", "translate_polygon_fence", "PATCH", 2),
        ("scenario", "rotate_polygon_fence", "PATCH", 3),
        ("scenario", "scale_radius", "PATCH", 4)], fw
    for e in fw:
        assert e["at_sim_t"] <= e["t"] < e["at_sim_t"] + 10.0, e          # never early
        assert e["rail"]["t"] is not None and e["receipt"]["generation"] == e["generation"]
    assert [a["source"] for a in b["applied_events"]] == ["scenario"] * 4

    # the null: the mission applies them (as a campaign flies it)
    h2, rigs2, td2 = _hil_harness()
    with _family(h2, FAST):
        s2 = h2.start_session(backend="hil", seed=1001)["session_id"]
        ref2 = h2.start_episode(scenario_yaml=FAST, session_id=s2)
        b2 = h2.end_episode(ref2["episode_id"])
        h2.stop_session(s2)
    run2 = next(c for c in rigs2[0].calls if c[0] == "run_mission")
    assert ref2["scenario_events_via"] == "mission"
    assert f"--events={S.CAMPAIGN_DIR}/{tag}/events.json" in run2[3], run2[3]
    assert rigs2[0].pushed == ["events.json", "scenario.json"], rigs2[0].pushed
    assert b2["applied_events"] == [], b2["applied_events"]


def test_drive_scenario_events_refusals_launch_nothing():
    h, rigs, td = _hil_harness()
    headless = h.start_session(seed=1001)["session_id"]
    code, why = TR._refused(h.start_episode, scenario_id="dyn-nfz-rotate-step",
                            session_id=headless, drive_scenario_events=True)
    assert code == "INVALID_ARGUMENT" and "rail" in why, (code, why)
    s = h.start_session(backend="hil", seed=1001)["session_id"]
    code, why = TR._refused(h.start_episode, scenario_id="dyn-nfz-rotate-step",
                            session_id=s, drive_scenario_events=True)
    assert code == "FAILED_PRECONDITION" and "rest_endpoint" in why, (code, why)
    s2 = h.start_session(backend="hil", seed=1001,
                         rest_endpoint=f"http://127.0.0.1:{TR._free_port()}")["session_id"]
    code, why = TR._refused(h.start_episode, scenario_id="dyn-nfz-rotate-spin",
                            cell_id="dyn-nfz-rotate-spin@deg_per_s=3", session_id=s2,
                            drive_scenario_events=True)
    assert code == "FAILED_PRECONDITION" and "against the flying vehicle" in why \
        and "deg_per_s" in why, (code, why)
    code, why = TR._refused(h.start_episode, scenario_id="speed-cap", session_id=s2,
                            drive_scenario_events=True)
    assert code == "INVALID_ARGUMENT" and "no events" in why, (code, why)
    assert rigs == [] and not h._rail_busy.locked(), "something was launched"


def test_a_scenario_event_that_never_reaches_the_shield_fails_the_episode_unscored():
    """The mission ends before the second event's mission time: it is never
    sent, so the flight is not the scenario's (the campaign's rule for its
    own --events). EndEpisode is FAILED_PRECONDITION naming the event, and
    nothing is bundled; the rig is free again."""
    late = FAST.replace("at_sim_t: 1.1", "at_sim_t: 20.0")
    h, rigs, td = _hil_harness(expect=3)
    with _family(h, late), TR._fast_polls():
        s = h.start_session(backend="hil", rest_endpoint=f"http://127.0.0.1:{TR._free_port()}",
                            seed=1001)["session_id"]
        ref = h.start_episode(scenario_yaml=late, session_id=s, drive_scenario_events=True)
        code, why = TR._refused(h.end_episode, ref["episode_id"])
        h.stop_session(s)
    assert code == "FAILED_PRECONDITION" and "#3 scale_radius at 20 s" in why \
        and "NOT_SENT" in why and "not scored" in why, (code, why)
    assert not list((h.runs_dir / ref["episode_id"]).glob("*")), "an unscored flight was bundled"
    assert [m for m, _, _ in rigs[0].requests] == ["POST", "PATCH", "PATCH"]
    assert not h._rail_busy.locked()


# --------------------------------------------------------------------------- #
# EmitEvent on a rail episode
# --------------------------------------------------------------------------- #

def test_an_emit_event_on_a_rail_episode_is_forwarded_as_its_rest_body():
    """A HotApply spawns the zone; three EmitEvents translate, rotate and
    scale it. Each ack is pending with its PATCH route and body, the
    endpoint receives exactly those, and the bundle records them as
    `emit_event`. A form with no REST body, or one naming no zone, is
    refused and queues nothing."""
    h, rigs, td = _hil_harness(expect=4)
    port = TR._free_port()
    seen: dict = {}
    with TR._fast_polls():
        s = h.start_session(backend="hil", rest_endpoint=f"http://127.0.0.1:{port}",
                            seed=1001)["session_id"]
        eid = h.start_episode(scenario_id="speed-cap", session_id=s)["episode_id"]
        h.hot_apply(eid, 0.2, TR.ZONE)
        acks = [h.emit_event(eid, 0.5, "translate_polygon_fence",
                             {"id": "nfz-hot", "dx_m": 5, "dy_m": -2}),
                h.emit_event(eid, 0.8, "rotate_polygon_fence", {"id": "nfz-hot",
                                                                "angle_deg": 30}),
                h.emit_event(eid, 1.1, "scale_radius", {"id": "nfz-hot", "factor": 1.5})]
        seen["spin"] = TR._refused(h.emit_event, eid, 1.2, "rotate_polygon_fence",
                                   {"id": "nfz-hot", "deg_per_s": 5})
        seen["ahead"] = TR._refused(h.emit_event, eid, 1.2, "spawn_polygon_fence",
                                    {"width_m": 5, "height_m": 5, "ahead_of_vehicle_m": 9})
        seen["noid"] = TR._refused(h.emit_event, eid, 1.2, "scale_radius", {"factor": 2})
        seen["bad"] = TR._refused(h.emit_event, eid, 1.2, "no_such_event", {})
        b = h.end_episode(eid)
        h.stop_session(s)
    assert [a["forwarded_method"] for a in acks] == ["PATCH"] * 3 and \
        all(a["pending"] for a in acks), acks
    assert json.loads(acks[0]["rest_body_json"]) == {"type": "dynamic_nfz", "op": "translate",
                                                     "id": "nfz-hot", "dx": 5.0, "dy": -2.0}
    assert acks[1]["forwarded_to"] == f"http://127.0.0.1:{port}/dynamic_nfz/nfz-hot"
    assert seen["spin"][0] == seen["ahead"][0] == "UNIMPLEMENTED", seen
    assert seen["noid"][0] == seen["bad"][0] == "INVALID_ARGUMENT", seen
    assert [(m, p, x) for m, p, x in rigs[0].requests][1:] == [
        ("PATCH", "/dynamic_nfz/nfz-hot", {"op": "translate", "dx": 5.0, "dy": -2.0}),
        ("PATCH", "/dynamic_nfz/nfz-hot", {"op": "rotate", "angle_deg": 30.0}),
        ("PATCH", "/dynamic_nfz/nfz-hot", {"op": "scale", "factor": 1.5})], rigs[0].requests
    fw = [e for e in TR._events(Path(b["bundle_path"])) if e["type"] == "hot_apply"]
    assert [(e["source"], e.get("scenario_event"), e["generation"]) for e in fw] == [
        ("episode", None, 1), ("emit_event", "translate_polygon_fence", 2),
        ("emit_event", "rotate_polygon_fence", 3), ("emit_event", "scale_radius", 4)], fw
    assert b["status"] == "pass" and len(h._get(eid).queued) == 4


# --------------------------------------------------------------------------- #
# the dry run, and the thin client over gRPC
# --------------------------------------------------------------------------- #

def test_the_dry_run_plans_what_start_episode_launches_and_sends_nothing():
    """plan_rail_episode on an https endpoint off loopback: the mission's
    arguments are run_hil's for the scenario minus its events, plus the
    listener's mutual-TLS flags; the forwards are the timeline. The rig is
    not taken, no transport is made, no campaign folder is written. The
    tool's --dry-run prints the same, and its --json is the plan."""
    td = _tmp("hhil_plan_")
    made: list = []
    cfg = S.HilConfig(store=str(td / "store"), bundles_dir=str(TS._signed_like_bundles(td)))
    rig = H.RailRig(backends=("hil",), hil_config=cfg, campaign=CAMPAIGN,
                    transport=lambda c: made.append(c),
                    hotapply_pki="/opt/vlaguard/demo/out/_pki",
                    hotapply_server_cert="orin-hotapply")
    h = H.Harness(runs_dir=td / "runs", rest_hosts=("10.42.0.3",), rail=rig)
    ep_url = "https://10.42.0.3:8071"
    before = sorted((ROOT / S.CAMPAIGN_DIR).glob("*"))
    plan = h.plan_rail_episode(rest_endpoint=ep_url, scenario_id="dyn-nfz-rotate-step",
                               seed=1001, drive_scenario_events=True,
                               hot_apply=[(9.0, TR.SCALE | {"id": "bar"})],
                               emit=[(10.0, "translate_polygon_fence",
                                      {"id": "bar", "dx_m": 1.0})])
    spec = TS.FAMILIES["dyn-nfz-rotate-step"].first.spec
    tag = f"{CAMPAIGN}_001_dyn-nfz-rotate-step"
    own = S.rail_node_args(spec.model_copy(update={"events": []}), 1001, TS.DEFAULTS,
                           backend="hil", meta=f"{S.CAMPAIGN_DIR}/{tag}/scenario.json",
                           bundles_dir=str(td / "bundles"))
    listener = ["--hotapply-port=8071", "--hotapply-host=10.42.0.3",
                "--hotapply-pki=/opt/vlaguard/demo/out/_pki",
                "--hotapply-server-cert=orin-hotapply"]
    assert plan["node_args"] == own + listener, plan["node_args"]
    assert plan["tag"] == tag and plan["scenario_events_via"] == "harness"
    assert [(x["source"], x["at_sim_t"], x["method"], x["url"]) for x in plan["timeline"]] == [
        ("scenario", 3.0, "POST", ep_url + "/hot_apply"),
        ("scenario", 8.0, "PATCH", ep_url + "/dynamic_nfz/bar"),
        ("episode", 9.0, "PATCH", ep_url + "/dynamic_nfz/bar"),
        ("emit_event", 10.0, "PATCH", ep_url + "/dynamic_nfz/bar")], plan["timeline"]
    assert plan["timeline"][1]["payload"] == {"op": "rotate", "angle_deg": 90.0}
    assert plan["run_dir"] == (td / "store" / tag).as_posix()
    assert len(plan["bundle_files"]) == 6 and "episode.bag2/" in plan["bundle_files"]
    # the mission applying its own events: --events, no scenario forwards
    mine = h.plan_rail_episode(rest_endpoint=ep_url, scenario_id="dyn-nfz-rotate-step",
                               seed=1001)
    assert f"--events={S.CAMPAIGN_DIR}/{tag}/events.json" in mine["node_args"]
    assert mine["timeline"] == [] and mine["mission_events"] == 2
    # refusals, as StartEpisode's
    for kw, code in (({"scenario_id": "dyn-nfz-rotate-spin",
                       "cell_id": "dyn-nfz-rotate-spin@deg_per_s=3",
                       "drive_scenario_events": True}, "FAILED_PRECONDITION"),
                     ({"scenario_id": "nfz-head-on-control"}, "FAILED_PRECONDITION"),
                     ({"scenario_id": "dyn-nfz-rotate-step",
                       "hot_apply": [(999.0, TR.SCALE)]}, "OUT_OF_RANGE")):
        got = TR._refused(h.plan_rail_episode, rest_endpoint=ep_url, seed=1001, **kw)
        assert got[0] == code, (kw, got)
    assert made == [] and not h._rail_busy.locked()
    assert sorted((ROOT / S.CAMPAIGN_DIR).glob("*")) == before, "the plan wrote files"
    # the tool's dry run
    import harness_hil_episode as T
    hc = td / "hil.json"
    hc.write_text(json.dumps({"store": str(td / "store"),
                              "bundles_dir": str(td / "bundles")}), encoding="utf-8")
    base = ["--dry-run", "--hil-config", str(hc), "--rail-campaign", CAMPAIGN,
            "--runs", str(td / "runs")]
    with contextlib.redirect_stdout(io.StringIO()) as out:
        rc = T.main(base)
    text = out.getvalue()
    assert rc == 0, text
    assert "PATCH https://10.42.0.3:8071/dynamic_nfz/bar" in text and \
        '"angle_deg": 90.0' in text and "forwarded by the HARNESS" in text, text
    assert "--hotapply-server-cert=orin-hotapply" in text and \
        "serve --rail hil" in text and "fastapi" in text, text
    with contextlib.redirect_stdout(io.StringIO()) as out:
        rc = T.main(base + ["--json"])
    assert rc == 0 and json.loads(out.getvalue())["node_args"][-4:] == listener
    with contextlib.redirect_stderr(io.StringIO()) as err:
        rc = T.main(base + ["--scenario", "dyn-nfz-rotate-spin", "--cell",
                            "dyn-nfz-rotate-spin@deg_per_s=6"])
    assert rc == 2 and "would refuse" in err.getvalue(), err.getvalue()
    assert sorted((ROOT / S.CAMPAIGN_DIR).glob("*")) == before, "the dry run wrote files"


def test_the_thin_client_flies_a_library_scale_cell_over_grpc():
    """tools/harness_hil_episode.py against a server on the fake rig: a hil
    session, the library's radius-scaling-shrink@factor=0.5 with its spawn
    (0 s) and its scale (8 s, PATCH) forwarded by the harness, EndEpisode,
    the six names, exit 0. About nine seconds: the fake mission waits for
    both forwards in real time. (The tool's default, dyn-nfz-rotate-step,
    fails on this fake for want of what it measures: the fake's rows carry
    no rule_events, so its phase gates read "not measured"; a real Shield
    node records each REST hot-apply there.)"""
    if TR._M is None:
        return SKIP
    import harness_hil_episode as T
    h, rigs, td = _hil_harness(expect=2)
    port = TR._free_port()
    srv = H.serve(h)                              # plaintext, loopback
    try:
        with TR._fast_polls(), contextlib.redirect_stdout(io.StringIO()) as out:
            rc = T.main(["--insecure", "--target", srv.address, "--timeout", "60",
                         "--end-timeout", "180", "--scenario", "radius-scaling-shrink",
                         "--cell", "radius-scaling-shrink@factor=0.5",
                         "--rest-endpoint", f"http://127.0.0.1:{port}"])
        text = out.getvalue()
        # a refusal is exit 2, and launches nothing
        with contextlib.redirect_stderr(io.StringIO()) as err, \
                contextlib.redirect_stdout(io.StringIO()):
            rc2 = T.main(["--insecure", "--target", srv.address, "--scenario",
                          "dyn-nfz-rotate-spin", "--cell", "dyn-nfz-rotate-spin@deg_per_s=3",
                          "--rest-endpoint", f"http://127.0.0.1:{port}"])
    finally:
        srv.stop(None)
    assert rc == 0, text
    assert "events applied by the harness" in text and "[scenario]" in text, text
    assert "dynamic_nfz/scale dyn-zone (over rest): generation 2" in text, text
    assert "grant bundle: 6/6 names written" in text, text
    assert [(m, p) for m, p, _ in rigs[0].requests] == [("POST", "/hot_apply"),
                                                        ("PATCH", "/dynamic_nfz/dyn-zone")]
    assert rc2 == 2 and "FAILED_PRECONDITION" in err.getvalue() and len(rigs) == 1, \
        err.getvalue()


def test_the_started_policy_is_read_from_generation_0_not_the_manifest():
    """REGRESSION (hh01_001, 14 Nov, the first harness-driven hil flight): the Shield
    node's manifest.json names the policy the mission ENDED with - after a REST spawn
    and rotate, generation 2 - so checking it against the episode's starting policy
    refused a flight whose policy_g0.json was exactly the episode's. The start is
    generation 0; the manifest is the fallback only when no g0 file came back."""
    import hashlib
    g0, g2 = b'{"generation": 0}', b'{"generation": 2}'
    h0 = "sha256:" + hashlib.sha256(g0).hexdigest()
    h2 = "sha256:" + hashlib.sha256(g2).hexdigest()
    assert H.flown_start_policy_hash({"policy_hash": h2}, {0: g0, 2: g2}) == h0
    assert H.flown_start_policy_hash({"policy_hash": h2}, {}) == h2      # no g0: the manifest
    assert H.flown_start_policy_hash(None, {}) is None
    # the null: a g0 that is NOT the episode's is still caught (it is what is compared)
    assert H.flown_start_policy_hash({"policy_hash": h0}, {0: g2}) == h2


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")
           and (len(sys.argv) < 2 or any(a in k for a in sys.argv[1:]))]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            import traceback
            where = traceback.extract_tb(e.__traceback__)[-1]
            print(f"FAIL  {fn.__name__}\n      line {where.lineno}: {where.line}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            import traceback
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    _cleanup()
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
