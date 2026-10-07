"""The Shield tick profiler: budgets, the topology claim, and its own nulls.

Run either way:
    pytest tests/test_profile_shield_tick.py -v
    python tests/test_profile_shield_tick.py

WHAT IS BEING CLAIMED

tools/profile_shield_tick.py times the Shield against the grant's two budgets
(5 ms monitor query at 50 rules, 100 ms tick) on real flight states, and writes
a JSON that a hil or flight run can attach. Three things would make that number
worthless while still looking fine, and each has a test here that breaks it on
purpose:

  * a desktop number labelled `hil` - the profiler must refuse the label on a
    host that is not the grant's Orin by guardrail.manifest's own rule, and
    must not take "aarch64", or a Xavier, as proof of one;
  * a design-load arm that never came near a fence - a Shield that skipped its
    fences would post the best time; the profiler must FAIL the run, and a
    test swaps in exactly such a Shield to see it do so;
  * a pack that is not the flight - an edited policy, a replay that no longer
    reproduces what the rail logged, or one that drops the subject;
  * the measured window - a clock that only moves inside Shield._check must
    show up as exactly that time, so _check cannot drift out of its window;
  * the conditions of the headline - the budgets, the horizon and the 50
    rules are pinned to the grant's text here, so an edit to any of them
    fails a test instead of quietly moving the goalposts;
  * a host that changed speed mid-run - the verdict is then undetermined.

Tests that need artefacts outside git (demo/out) print SKIP, never PASS.
"""
from __future__ import annotations

import copy
import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import guardrail.shield as shield_mod                               # noqa: E402
import profile_shield_tick as P                                     # noqa: E402
from guardrail import load_policy                                   # noqa: E402
from guardrail.models import (Action4D, PolygonFence, Policy,        # noqa: E402
                              State)
from guardrail.shield import Shield                                 # noqa: E402

SKIP = "SKIP"
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
PED = ROOT / "policies" / "sitl_pedestrian.yaml"
COMMITTED_PACK = ROOT / "deploy" / "evidence" / "shield_tick_pack.json"
COMMITTED_DEV_PROFILE = ROOT / "deploy" / "evidence" / "dev" / "shield_tick_profile.json"
GRANT_TEXT = ROOT / "kuanting-vla-uav-guardrail" / "docs"

ORIN_MODEL = "NVIDIA Jetson AGX Orin Developer Kit"
TEGRA_R36 = ("# R36 (release), REVISION: 4.0, GCID: 37537400, BOARD: generic, "
             "EABI: aarch64, DATE: Fri Sep 13 04:36:44 UTC 2024")


def _entry(tag: str, pol, rows: list) -> dict:
    dump = pol.model_dump(mode="json", exclude_none=True)
    return {"tag": tag, "rail": "dev", "rail_recorded": "dev", "stride": 1,
            "source_sha256": None, "policy": dump, "policy_sha256": P.policy_digest(dump),
            "policy_hash": pol.policy_hash, "policy_label": tag,
            "obstacle_map": None, "columns": list(P.COLUMNS), "rows": rows,
            "_policy": pol}


def _mini_flight(n: int = 120) -> dict:
    """A straight 4 m/s leg from the origin past the demo NFZ, logged the way
    a rail logs it: the violated rule ids come from a real Shield at the flown
    horizon, so a faithful replay must agree on every tick."""
    pol = load_policy(DEMO)
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    rows = []
    x = y = 0.0
    for k in range(n):
        st = State(x=x, y=y, up=15.0)
        act = Action4D(vx=2.83, vy=2.83, vz_up=0.0, yaw_rate=0.0)
        logged = sorted({v.rule_id for v in sh.filter(st, act).violations})
        rows.append([round(0.1 * k, 3), x, y, 15.0, 0.0, 2.83, 2.83, 0.0, 0.0,
                     None, None, None, logged, None])
        x += 0.283
        y += 0.283
    return _entry("mini", pol, rows)


def _standoff_flight(n: int = 120) -> dict:
    """A 3 m/s leg east towards a pedestrian standing at (25, 0), logged by a
    real Shield that was given the subject every tick (as the rail does with
    Shield.set_subject). The stand-off rule fires as the forecast reaches
    10 m, so a replay that drops the subject cannot reproduce the log."""
    pol = load_policy(PED)
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    rows = []
    for k in range(n):
        x = 0.3 * k
        st = State(x=x, y=0.0, up=15.0)
        act = Action4D(vx=3.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
        sh.set_subject(25.0, 0.0, "pedestrian")
        logged = sorted({v.rule_id for v in sh.filter(st, act).violations})
        rows.append([round(0.1 * k, 3), x, 0.0, 15.0, 0.0, 3.0, 0.0, 0.0, 0.0,
                     25.0, 0.0, "pedestrian", logged, None])
    return _entry("standoff", pol, rows)


def _mini_pack(n: int = 120) -> dict:
    return {"schema": P.PACK_SCHEMA, "flights": [_mini_flight(n)]}


def _facts(is_orin: bool, machine: str = "x86_64") -> dict:
    return {"machine": machine,
            "jetson": {"is_orin": is_orin, "device_model": None,
                       "missing": [] if is_orin else ["not an Orin (test host)"]}}


def _host(files: dict, machine: str = "aarch64", jetpack=None) -> dict:
    return P.jetson_facts(read=lambda p: files.get(p), machine=lambda: machine,
                          jetpack=lambda: jetpack)


class _SteadyClock:
    """calibrate() stand-in: the same reference before and after."""

    def __init__(self, before: float, after: float):
        self.vals = [before, after]

    def __call__(self, repeats: int = 5, clock=None):
        v = self.vals.pop(0) if self.vals else 0.0
        return {"workload": "test", "median_ms": v, "min_ms": v, "max_ms": v,
                "repeats": repeats, "_check": (0, 0)}


# --------------------------------------------------------------------------- #
# the grant's numbers, pinned
# --------------------------------------------------------------------------- #
def test_the_budgets_horizon_and_load_are_the_grants():
    """Safety Shield page: "Monitor tick budget <= 100 ms (10 Hz) end-to-end",
    "5 s of predicted trajectory at 10 Hz = 50 future poses"; Policy DSL page:
    "Monitor query latency at 10 Hz with 50 active rules <= 5 ms". The headline
    must be measured at exactly that condition. Where the reference design's
    text is on this machine (it is git-ignored), the numbers are read from it
    as well, so the test follows the source rather than a copy of it."""
    assert P.QUERY_BUDGET_MS == 5.0
    assert P.TICK_BUDGET_MS == 100.0
    assert P.DESIGN_LOAD_RULES == 50
    assert P.HORIZONS["grant"] == (5.0, 0.1)
    assert P.HEADLINE == ("design_load", "grant")
    lookahead, dt = P.HORIZONS["grant"]
    assert round(lookahead / dt) == 50, "the grant horizon is not 50 future poses"
    shield_md = GRANT_TEXT / "02-implementation" / "safety-shield.md"
    dsl_md = GRANT_TEXT / "02-implementation" / "policy-dsl.md"
    if shield_md.is_file() and dsl_md.is_file():
        import re
        s = shield_md.read_text(encoding="utf-8")
        d = dsl_md.read_text(encoding="utf-8")
        m = re.search(r"Monitor tick budget \| ≤ (\d+) ms", s)
        assert m and float(m.group(1)) == P.TICK_BUDGET_MS, m
        m = re.search(r"(\d+) s of predicted trajectory at (\d+) Hz = (\d+) future poses", s)
        assert m and (float(m.group(1)), 1 / float(m.group(2))) == P.HORIZONS["grant"], m
        assert int(m.group(3)) == round(lookahead / dt)
        m = re.search(r"with (\d+) active rules \| ≤ (\d+) ms", d)
        assert m and int(m.group(1)) == P.DESIGN_LOAD_RULES, m
        assert float(m.group(2)) == P.QUERY_BUDGET_MS, m


def test_the_headline_is_taken_at_the_grant_condition():
    """Swap the pooled cells: the headline must follow design_load/grant and
    nothing else."""
    prof = P.profile(_mini_pack(30), "dev", facts=_facts(False), warmup=0)
    want = prof["pooled"]["design_load/grant"]
    assert prof["headline"]["check_p99_ms"] == want["check"]["p99_ms"]
    assert prof["headline"]["filter_p99_ms"] == want["filter"]["p99_ms"]
    assert prof["headline"]["ticks"] == want["check"]["n"]
    assert "p99" in prof["headline"]["verdict_basis"]
    assert prof["headline"]["check_max_ms"] == want["check"]["max_ms"]
    assert prof["headline"]["check_over_budget"] == want["check"]["over_budget"]


# --------------------------------------------------------------------------- #
# statistics and the budget verdict
# --------------------------------------------------------------------------- #
def test_check_time_is_the_time_spent_inside_check():
    """A clock that moves ONLY inside Shield._check (3 ms per call). Every
    _check time must be exactly 3 ms, and every filter time a non-zero multiple
    of 3 ms (filter calls _check at least once). Moving _check out of its timed
    window, or timing the wrong call, breaks one of the two."""
    t = [0.0]

    class TimedShield(Shield):
        def _check(self, state, action):
            t[0] += 0.003
            return super()._check(state, action)

    real = shield_mod.Shield
    shield_mod.Shield = TimedShield
    try:
        res = P.replay(_mini_flight(20), load_policy(DEMO), P.HORIZONS["flown"],
                       clock=lambda: t[0])
    finally:
        shield_mod.Shield = real
    assert len(res["check_ms"]) == 20
    assert all(math.isclose(v, 3.0, abs_tol=1e-9) for v in res["check_ms"]), res["check_ms"]
    for v in res["filter_ms"]:
        assert v >= 3.0 - 1e-9, res["filter_ms"]
        assert math.isclose(v / 3.0, round(v / 3.0), abs_tol=1e-6), v


def test_stats_are_nearest_rank_and_count_ticks_over_budget():
    s = P.stats([float(v) for v in range(1, 101)], budget_ms=90.0)
    assert s["n"] == 100
    assert s["median_ms"] == 50.5
    assert s["p95_ms"] == 95.0 and s["p99_ms"] == 99.0, s
    assert s["max_ms"] == 100.0
    assert s["over_budget"] == 10, s
    assert s["verdict"] == "over"
    assert P.stats([1.0, 2.0], budget_ms=5.0)["verdict"] == "within"
    assert P.stats([]) == {"n": 0}


def test_a_slow_clock_is_reported_over_budget():
    """Inject a clock that makes every call take 120 ms: both calls must be
    reported over their budgets, every tick counted."""
    t = [0.0]

    def clock():
        t[0] += 0.120
        return t[0]

    res = P.replay(_mini_flight(30), load_policy(DEMO), P.HORIZONS["flown"], clock=clock)
    chk = P.stats(res["check_ms"], P.QUERY_BUDGET_MS)
    flt = P.stats(res["filter_ms"], P.TICK_BUDGET_MS)
    assert chk["verdict"] == "over" and chk["over_budget"] == 30, chk
    assert flt["verdict"] == "over" and flt["over_budget"] == 30, flt
    assert math.isclose(flt["median_ms"], 120.0, rel_tol=1e-9), flt


def test_pinning_really_restricts_the_process():
    """--cpus must change where the process runs, and report the set now in
    force, not the set it was asked for."""
    import os
    try:
        import psutil
        get = lambda: sorted(psutil.Process().cpu_affinity())        # noqa: E731
        put = lambda cpus: psutil.Process().cpu_affinity(cpus)       # noqa: E731
    except ImportError:
        if not hasattr(os, "sched_getaffinity"):
            return SKIP
        get = lambda: sorted(os.sched_getaffinity(0))                # noqa: E731
        put = lambda cpus: os.sched_setaffinity(0, set(cpus))        # noqa: E731
    before = get()
    if len(before) < 2:
        return SKIP
    try:
        got = P.pin_cpus([before[0]])
        assert got == [before[0]], got
        assert get() == [before[0]], "pin_cpus reported a pin it did not make"
    finally:
        put(before)
    assert get() == before


def test_write_lf_writes_lf_on_every_os():
    d = Path(tempfile.mkdtemp())
    P.write_lf(d / "x.json", "a\nb\n")
    assert (d / "x.json").read_bytes() == b"a\nb\n"


def test_a_design_load_arm_without_fifty_rules_fails_the_run():
    real = P.design_load_policy

    def short(policy, xy, total=P.DESIGN_LOAD_RULES):
        return real(policy, xy, total=40)

    P.design_load_policy = short
    try:
        prof = P.profile(_mini_pack(20), "dev", facts=_facts(False), warmup=0,
                         calibrate_host=False)
    finally:
        P.design_load_policy = real
    assert any("not 50" in m for m in prof["failures"]), prof["failures"]


def test_cpu_lists_parse_and_bad_ones_are_refused():
    assert P.parse_cpus("0-3,8, 10-11") == [0, 1, 2, 3, 8, 10, 11]
    assert P.parse_cpus("5") == [5]
    for bad in ("", "7-3", "a-b"):
        try:
            P.parse_cpus(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")


def test_the_host_speed_reference_flags_a_host_that_changed_speed():
    """A shared desktop measured the same Shield code 2-3x apart an hour apart.
    The calibration before and after a run is what makes that visible."""
    cal = P.calibrate(repeats=2)
    assert cal["median_ms"] > 0 and cal["repeats"] == 2
    steady = P.host_speed_record({"median_ms": 100.0}, {"median_ms": 120.0})
    assert steady["steady"] is True
    slowed = P.host_speed_record({"median_ms": 100.0}, {"median_ms": 210.0})
    assert slowed["steady"] is False and math.isclose(slowed["after_over_before"], 2.1)
    sped = P.host_speed_record({"median_ms": 100.0}, {"median_ms": 40.0})
    assert sped["steady"] is False
    prof = P.profile(_mini_pack(20), "dev", facts=_facts(False), warmup=0,
                     arms=("as_flown",), horizons=("flown",))
    assert prof["host_speed"]["before"]["median_ms"] > 0, prof["host_speed"]


def test_a_host_that_changed_speed_fails_the_run_and_leaves_the_verdict_open():
    """Measured on the shared desktop: the reference went 41 -> 66 ms in one
    run while the headline read 'over'. Such a run mixes two host speeds; it
    must FAIL, and `attach` must say neither within nor over."""
    real = P.calibrate
    try:
        P.calibrate = _SteadyClock(41.0, 66.5)
        prof = P.profile(_mini_pack(30), "dev", facts=_facts(False), warmup=0)
        P.calibrate = _SteadyClock(41.0, 42.0)
        steady = P.profile(_mini_pack(30), "dev", facts=_facts(False), warmup=0)
    finally:
        P.calibrate = real
    assert any("host-speed reference moved" in m for m in prof["failures"]), prof["failures"]
    assert prof["headline"]["host_steady"] is False
    assert P.attach_summary(prof)["within_budget"] is None
    assert not any("host-speed" in m for m in steady["failures"]), steady["failures"]
    assert P.attach_summary(steady)["within_budget"] in (True, False)
    no_ref = P.profile(_mini_pack(10), "dev", facts=_facts(False), warmup=0,
                       calibrate_host=False)
    assert P.attach_summary(no_ref)["within_budget"] is None, \
        "a run with no host-speed reference cannot be called within budget"


# --------------------------------------------------------------------------- #
# the topology claim: guardrail.manifest's Orin rule, nothing looser
# --------------------------------------------------------------------------- #
def test_hil_and_flight_are_refused_on_a_host_that_is_not_an_orin():
    for topo in ("hil", "flight"):
        why = P.check_topology_claim(topo, _facts(False))
        assert why and "Jetson Orin" in why[0], why
    assert P.check_topology_claim("dev", _facts(False)) == []


def test_aarch64_alone_does_not_make_an_orin():
    """A Raspberry Pi or an ARM cloud VM is aarch64 too."""
    j = _host({}, machine="aarch64")
    assert j["is_orin"] is False and j["missing"], j
    why = P.check_topology_claim("hil", {"machine": "aarch64", "jetson": j})
    assert why, "an aarch64 host with no Jetson evidence was accepted as hil"


def test_a_jetson_that_is_not_an_orin_is_refused():
    """A Xavier runs L4T too. The grant's hil machine is an Orin, and the
    manifest would label the Xavier's runs dev; the profile must agree."""
    j = _host({"/etc/nv_tegra_release": TEGRA_R36.replace("R36", "R35"),
               "/proc/device-tree/model": "NVIDIA Jetson AGX Xavier Developer Kit"})
    assert j["is_orin"] is False and any("Orin" in m for m in j["missing"]), j
    assert P.check_topology_claim("hil", {"jetson": j})


def test_an_orin_with_l4t_is_accepted_and_without_a_release_is_not():
    files = {"/etc/nv_tegra_release": TEGRA_R36, "/proc/device-tree/model": ORIN_MODEL}
    j = _host(files)
    assert j["is_orin"] is True and j["missing"] == [], j
    assert j["l4t_release"] == "R36.4.0" and j["device_model"] == ORIN_MODEL, j
    assert "hostname" not in j, "a committed profile must not carry the host's name"
    assert P.check_topology_claim("hil", {"jetson": j}) == []
    assert P.check_topology_claim("flight", {"jetson": j}) == []
    # The device tree inside a default Docker container is masked: the board
    # name then reads as missing, and the claim must fall.
    no_model = _host({"/etc/nv_tegra_release": TEGRA_R36})
    assert no_model["is_orin"] is False, no_model
    no_release = _host({"/proc/device-tree/model": ORIN_MODEL})
    assert no_release["is_orin"] is False, no_release
    assert _host(files, jetpack="6.1")["is_orin"] is True


def test_the_profiler_and_the_manifest_never_disagree_about_a_host():
    """The same synthetic hosts through both: the profile's is_orin must be
    exactly guardrail.manifest's verdict on the same evidence."""
    from guardrail import manifest as M
    hosts = [({}, "x86_64"), ({}, "aarch64"),
             ({"/proc/device-tree/model": ORIN_MODEL}, "aarch64"),
             ({"/etc/nv_tegra_release": TEGRA_R36, "/proc/device-tree/model": ORIN_MODEL},
              "aarch64"),
             ({"/etc/nv_tegra_release": TEGRA_R36, "/proc/device-tree/model": ORIN_MODEL},
              "x86_64"),
             ({"/etc/nv_tegra_release": TEGRA_R36,
               "/sys/firmware/devicetree/base/model": "NVIDIA Jetson Orin Nano Developer Kit"},
              "aarch64")]
    # The manifest's public name for its Orin rule, or the private one it had
    # before; the profiler resolves it the same way.
    orin_rule = getattr(M, "orin_missing", None) or M._orin_missing
    for files, machine in hosts:
        ev = M.collect_host_evidence(read=lambda p, f=files: f.get(p),
                                     machine=lambda m=machine: m, jetpack=lambda: None)
        assert _host(files, machine)["is_orin"] == (not orin_rule(ev)), (files, machine)
    assert not hasattr(P, "_jetson_facts"), "a private copy of the Jetson rule is back"


def test_hil_is_accepted_on_an_orin_and_unknown_labels_are_refused():
    assert P.check_topology_claim("hil", _facts(True, "aarch64")) == []
    assert P.check_topology_claim("canonical-hil", _facts(True))
    assert P.check_topology_claim(None, _facts(True))


def test_cli_refuses_hil_here_unless_this_host_is_an_orin():
    if P.host_facts()["jetson"]["is_orin"]:
        return SKIP
    d = Path(tempfile.mkdtemp())
    pack = d / "pack.json"
    pack.write_text("{}", encoding="utf-8")
    rc = P.main(["profile", "--topology", "hil", "--pack", str(pack),
                 "--out", str(d / "out.json")])
    assert rc == 2, rc
    assert not (d / "out.json").exists(), "a refused claim still wrote a profile"


# --------------------------------------------------------------------------- #
# the three arms
# --------------------------------------------------------------------------- #
def test_design_load_has_exactly_fifty_rules_and_never_contains_the_path():
    from shapely.geometry import Point, box
    pol = load_policy(DEMO)
    xy = [(0.283 * k, 0.283 * k) for k in range(120)]
    dl, added = P.design_load_policy(pol, xy)
    assert len(dl.constraints) == P.DESIGN_LOAD_RULES, len(dl.constraints)
    assert added == P.DESIGN_LOAD_RULES - len(pol.constraints)
    rings = []
    for c in dl.constraints:
        if not c.id.startswith("design-load-"):
            continue
        xs = [v.x for v in c.vertices]
        ys = [v.y for v in c.vertices]
        ring = box(min(xs) - c.margin_m, min(ys) - c.margin_m,
                   max(xs) + c.margin_m, max(ys) + c.margin_m)
        for px, py in xy:
            assert not ring.contains(Point(px, py)), f"{c.id} holds a logged position"
        rings.append(ring)
    for i, a in enumerate(rings):
        for b in rings[i + 1:]:
            assert a.intersection(b).area < 1e-9, "two design-load zones overlap"
    again, _ = P.design_load_policy(pol, xy)
    assert again.policy_hash == dl.policy_hash, "placement is not deterministic"
    assert len(pol.constraints) == 3, "the flight's own policy was modified in place"


def test_design_load_leaves_a_policy_that_already_has_fifty_rules_alone():
    pol = load_policy(DEMO)
    extra = [PolygonFence(id=f"z{k}", type="polygon_fence",
                          vertices=[{"x": 500 + 30 * k, "y": 0}, {"x": 510 + 30 * k, "y": 0},
                                    {"x": 510 + 30 * k, "y": 10}])
             for k in range(60)]
    big = pol.model_copy(deep=True)
    big.constraints = list(big.constraints) + extra
    out, added = P.design_load_policy(big, [(0.0, 0.0)])
    assert added == 0 and len(out.constraints) == 63


def test_floor_removes_the_fences_and_nothing_else():
    pol = load_policy(DEMO)
    fl = P.floor_policy(pol)
    assert [c.id for c in fl.constraints] == ["alt-band", "kin-caps"], fl.constraints
    assert len(pol.constraints) == 3


# --------------------------------------------------------------------------- #
# the nulls must be able to fail
# --------------------------------------------------------------------------- #
def test_a_profile_of_a_faithful_replay_passes_its_own_checks():
    prof = P.profile(_mini_pack(), "dev", facts=_facts(False), warmup=0,
                     calibrate_host=False)
    assert prof["failures"] == [], prof["failures"]
    f = prof["flights"][0]
    ag = f["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert ag["identical_rule_sets"] == ag["compared"] == 120, ag
    # The split, and its null: the informative part is the ticks that logged
    # a violation; a never-flagging replay would match the rest.
    assert ag["logged_nonempty"] > 0 and ag["identical_nonempty"] == ag["logged_nonempty"], ag
    assert ag["never_flagging_null"] == ag["compared"] - ag["logged_nonempty"], ag
    assert ag["informative"] is True and "agreement_note" not in f
    assert prof["logged_agreement"]["identical_nonempty"] == ag["identical_nonempty"]
    assert f["rules"]["design_load"] == 50
    assert prof["headline"]["check_verdict"] in ("within", "over")
    assert prof["pooled"]["design_load/grant"]["check"]["n"] == 120
    assert prof["gc_enabled"] is True, "the garbage collector runs in a live node; time it so"


def test_a_replay_that_never_flags_matches_only_the_null():
    """The null made concrete: a Shield that reports no violation at all
    agrees on every empty tick and on none of the informative ones."""
    class SilentShield(Shield):
        def filter(self, state, raw):
            dec = super().filter(state, raw)
            return dec.model_copy(update={"violations": []})

    real = shield_mod.Shield
    shield_mod.Shield = SilentShield
    try:
        prof = P.profile(_mini_pack(), "dev", facts=_facts(False), warmup=0,
                         arms=("as_flown",), horizons=("flown",))
    finally:
        shield_mod.Shield = real
    ag = prof["flights"][0]["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert ag["identical_nonempty"] == 0, ag
    assert ag["identical_rule_sets"] == ag["never_flagging_null"], ag


def test_a_flight_that_logged_nothing_is_marked_as_verifying_nothing():
    """The city flight logged no violation in 2,396 ticks: its agreement is
    all null. The profile must say so rather than count it as reproduced."""
    pol = load_policy(DEMO)
    rows = [[0.1 * k, -200.0 - k, -200.0, 15.0, 0.0, -1.0, 0.0, 0.0, 0.0,
             None, None, None, [], None] for k in range(30)]
    pack = {"schema": P.PACK_SCHEMA, "flights": [_entry("quiet", pol, rows)]}
    prof = P.profile(pack, "dev", facts=_facts(False), warmup=0,
                     arms=("as_flown",), horizons=("flown",))
    f = prof["flights"][0]
    ag = f["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert ag["informative"] is False and ag["logged_nonempty"] == 0, ag
    assert "verifies nothing" in f["agreement_note"], f
    assert prof["logged_agreement"]["flights_with_no_logged_violation"] == ["quiet"]


def test_a_replay_that_drops_the_subject_is_caught():
    """The stand-off rule needs the subject the rail passed to set_subject.
    A faithful replay reproduces every logged stand-off tick; a Shield that
    never receives the subject reproduces none of them."""
    flight = _standoff_flight()
    logged_standoff = sum(1 for r in flight["rows"] if "standoff-pedestrian" in r[12])
    assert logged_standoff >= 5, "the test flight never came near the pedestrian"
    pack = {"schema": P.PACK_SCHEMA, "flights": [flight]}
    prof = P.profile(pack, "dev", facts=_facts(False), warmup=0,
                     arms=("as_flown",), horizons=("flown",))
    ag = prof["flights"][0]["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert ag["identical_nonempty"] == ag["logged_nonempty"] >= logged_standoff, ag

    class SubjectBlindShield(Shield):
        def set_subject(self, x, y=None, subject_class=None):
            return super().set_subject(None)

    real = shield_mod.Shield
    shield_mod.Shield = SubjectBlindShield
    try:
        blind = P.profile(pack, "dev", facts=_facts(False), warmup=0,
                          arms=("as_flown",), horizons=("flown",))
    finally:
        shield_mod.Shield = real
    bag = blind["flights"][0]["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert bag["identical_nonempty"] <= ag["identical_nonempty"] - logged_standoff, (ag, bag)


def test_a_shield_that_skips_its_fences_is_caught():
    """The failure this whole check exists for: fences silently ignored, best
    time in the table. Swap in such a Shield and require a FAIL."""
    class BlindShield(Shield):
        def _fence_records(self, ir):
            return []

    real = shield_mod.Shield
    shield_mod.Shield = BlindShield
    try:
        prof = P.profile(_mini_pack(), "dev", facts=_facts(False), warmup=0)
    finally:
        shield_mod.Shield = real
    fails = [m for m in prof["failures"] if "design_load" in m and "geofence" in m]
    assert fails, f"a fence-blind Shield passed: {prof['failures']}"


def test_a_floor_that_still_has_fences_is_caught():
    real = P.floor_policy
    P.floor_policy = lambda pol: pol.model_copy(deep=True)
    try:
        prof = P.profile(_mini_pack(), "dev", facts=_facts(False), warmup=0)
    finally:
        P.floor_policy = real
    assert any("floor" in m for m in prof["failures"]), prof["failures"]


def test_a_replay_that_is_not_the_flight_shows_in_the_agreement():
    """Corrupt the logged rule sets on half the ticks: the agreement count
    must drop by exactly that many, not stay at 100 %."""
    pack = _mini_pack()
    rows = pack["flights"][0]["rows"]
    for r in rows[::2]:
        r[12] = ["no-such-rule"]
    prof = P.profile(pack, "dev", facts=_facts(False), warmup=0,
                     arms=("as_flown",), horizons=("flown",))
    ag = prof["flights"][0]["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert ag["identical_rule_sets"] == 60 and ag["compared"] == 120, ag


# --------------------------------------------------------------------------- #
# the pack
# --------------------------------------------------------------------------- #
def _write_pack(path: Path, pack: dict) -> None:
    clean = copy.deepcopy({k: v for k, v in pack.items()})
    for f in clean["flights"]:
        f.pop("_policy", None)
    path.write_text(json.dumps(clean), encoding="utf-8")


def test_load_pack_refuses_an_edited_policy():
    d = Path(tempfile.mkdtemp())
    pack = _mini_pack(10)
    p = d / "pack.json"
    _write_pack(p, pack)
    P.load_pack(p)                                     # untouched: loads
    raw = json.loads(p.read_text(encoding="utf-8"))
    raw["flights"][0]["policy"]["constraints"][0]["vertices"][0]["x"] = 8.0
    p.write_text(json.dumps(raw), encoding="utf-8")
    try:
        P.load_pack(p)
    except ValueError as exc:
        assert "was edited" in str(exc), exc
    else:
        raise AssertionError("an edited policy loaded as if it were the flight's")


def test_a_changed_fingerprint_function_warns_instead_of_refusing():
    """The document is intact; only Policy.policy_hash moved (as it did once
    when the hash stopped covering empty fields). That is not an edited pack."""
    d = Path(tempfile.mkdtemp())
    pack = _mini_pack(10)
    pack["flights"][0]["policy_hash"] = "sha256:" + "f" * 64
    p = d / "pack.json"
    _write_pack(p, pack)
    loaded = P.load_pack(p)
    assert loaded["warnings"] and "fingerprint function" in loaded["warnings"][0], \
        loaded["warnings"]


def test_load_pack_refuses_a_wrong_schema_and_an_empty_pack():
    d = Path(tempfile.mkdtemp())
    for bad in ({"schema": "something/else", "flights": [1]},
                {"schema": P.PACK_SCHEMA, "flights": []}):
        p = d / "bad.json"
        p.write_text(json.dumps(bad), encoding="utf-8")
        try:
            P.load_pack(p)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


def test_load_pack_refuses_a_retired_rail_label():
    d = Path(tempfile.mkdtemp())
    pack = _mini_pack(10)
    pack["flights"][0]["rail"] = "canonical-hil"
    p = d / "pack.json"
    _write_pack(p, pack)
    try:
        P.load_pack(p)
    except ValueError as exc:
        assert "retired label" in str(exc) and "'dev'" in str(exc), exc
    else:
        raise AssertionError("a pack that calls the dev rail 'canonical-hil' was loaded")


def _flight_dir(policy_hash: str, n: int = 40, topology: str = "dev") -> Path:
    d = Path(tempfile.mkdtemp()) / "fake_flight"
    d.mkdir()
    (d / "manifest.json").write_text(json.dumps({"policy_hash": policy_hash,
                                                 "topology": topology}), encoding="utf-8")
    (d / "metrics.json").write_text(json.dumps({"tag": d.name}), encoding="utf-8")
    pol = load_policy(DEMO)
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    with (d / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
        for k in range(n):
            st = State(x=0.3 * k, y=0.3 * k, up=15.0)
            raw = Action4D(vx=3.0, vy=3.0)
            dec = sh.filter(st, raw)
            fh.write(json.dumps({"t": 0.1 * k, "tick": k + 1, "x": st.x, "y": st.y,
                                 "up": st.up, "raw": raw.model_dump(),
                                 "violations": [v.model_dump() for v in dec.violations]})
                     + "\n")
    return d


def test_pack_flight_resolves_the_policy_by_hash_and_round_trips():
    pol = load_policy(DEMO)
    d = _flight_dir(pol.policy_hash)
    entry = P.pack_flight(d, max_ticks=0)
    assert entry["policy_hash"] == pol.policy_hash
    assert len(entry["rows"]) == 40 and entry["stride"] == 1
    assert entry["action_field"] == "raw" and entry["subject_source"] == "none"
    pack = {"schema": P.PACK_SCHEMA, "flights": [entry]}
    p = d.parent / "pack.json"
    p.write_text(json.dumps(pack), encoding="utf-8")
    loaded = P.load_pack(p)
    prof = P.profile(loaded, "dev", facts=_facts(False), warmup=0,
                     arms=("as_flown",), horizons=("flown",))
    ag = prof["flights"][0]["arms"]["as_flown"]["flown"]["logged_agreement"]
    assert ag["identical_rule_sets"] == 40, ag


def test_pack_flight_reads_the_retired_label_as_dev_and_keeps_what_was_recorded():
    """Three dev-rail flights recorded 'canonical-hil', the retired name of
    dev. The pack must say dev and keep the recorded label beside it."""
    pol = load_policy(DEMO)
    entry = P.pack_flight(_flight_dir(pol.policy_hash, n=10, topology="canonical-hil"),
                          max_ticks=0)
    assert entry["rail"] == "dev" and entry["rail_recorded"] == "canonical-hil", entry
    other = P.pack_flight(_flight_dir(pol.policy_hash, n=10,
                                      topology="projectairsim-single-host"), max_ticks=0)
    assert other["rail"] == other["rail_recorded"] == "projectairsim-single-host"


def test_pack_flight_refuses_a_policy_it_cannot_resolve():
    d = _flight_dir("sha256:" + "0" * 64)
    try:
        P.pack_flight(d)
    except P.PackRefused as exc:
        assert "matches no policy" in str(exc), exc
    else:
        raise AssertionError("an unresolvable policy was packed (nearest-policy guess?)")


def test_pack_flight_samples_long_flights_evenly():
    pol = load_policy(DEMO)
    d = _flight_dir(pol.policy_hash, n=40)
    entry = P.pack_flight(d, max_ticks=15)
    assert entry["stride"] == 3 and len(entry["rows"]) == 14, (entry["stride"], len(entry["rows"]))
    assert entry["rows_in_source"] == 40


def test_rail_logged_tick_period_is_withheld_for_a_sampled_pack():
    f = _mini_flight(20)
    tp = P._rail_logged(f)["tick_period"]
    assert tp["n"] == 19 and "verdict" not in tp, "a 10 Hz period is not an overrun"
    assert tp["late_ticks_over_150ms"] == 0
    f["rows"][10][0] += 0.3                          # one tick arrives 300 ms late
    assert P._rail_logged(f)["tick_period"]["late_ticks_over_150ms"] == 1
    f["stride"] = 4
    out = P._rail_logged(f)
    assert out["tick_period"] is None and "every 4th" in out["tick_period_why"], out


# --------------------------------------------------------------------------- #
# the attachment
# --------------------------------------------------------------------------- #
def test_attach_summary_is_never_within_budget_when_the_run_failed():
    real = P.calibrate
    try:
        P.calibrate = _SteadyClock(40.0, 40.0)
        prof = P.profile(_mini_pack(), "dev", facts=_facts(False), warmup=0)
    finally:
        P.calibrate = real
    good = P.attach_summary(prof)
    assert good["within_budget"] == (prof["headline"]["check_verdict"] == "within"
                                     and prof["headline"]["filter_verdict"] == "within")
    assert good["host_is_orin"] is False and good["host_steady"] is True
    prof["failures"] = ["something"]
    assert P.attach_summary(prof)["within_budget"] is False
    d = Path(tempfile.mkdtemp())
    p = d / "prof.json"
    p.write_text(json.dumps(prof), encoding="utf-8")
    s = P.attach_summary(prof, p)
    assert s["sha256"] == P.sha256_file(p) and s["failures"] == 1


def test_the_default_output_goes_where_the_environment_says():
    """The Orin writes into deploy/evidence/incoming/ (git-ignored), so a run
    there never modifies a tracked file and never makes the checkout dirty."""
    import os
    old = os.environ.get(P.EVIDENCE_OUT_ENV)
    try:
        os.environ.pop(P.EVIDENCE_OUT_ENV, None)
        assert P.evidence_root() == ROOT / "deploy" / "evidence"
        os.environ[P.EVIDENCE_OUT_ENV] = "/opt/vlaguard/deploy/evidence/incoming"
        assert P.evidence_root().as_posix().endswith("deploy/evidence/incoming")
    finally:
        if old is None:
            os.environ.pop(P.EVIDENCE_OUT_ENV, None)
        else:
            os.environ[P.EVIDENCE_OUT_ENV] = old


# --------------------------------------------------------------------------- #
# the committed evidence
# --------------------------------------------------------------------------- #
def test_the_committed_pack_loads_and_still_matches_its_sources():
    """Every source still on this host must hash to what was packed. On a
    clean clone (no demo/out) nothing can be checked, and that is a SKIP, not
    a PASS."""
    if not COMMITTED_PACK.is_file():
        return SKIP
    pack = P.load_pack(COMMITTED_PACK)
    assert len(pack["flights"]) >= 3, [f["tag"] for f in pack["flights"]]
    for f in pack["flights"]:
        assert f["rows"], f["tag"]
        assert f["rail"] in ("dev", "projectairsim-single-host"), (f["tag"], f["rail"])
        assert "rail_recorded" in f, f["tag"]
    rails = {f["rail"] for f in pack["flights"]}
    assert len(rails) >= 2, f"one rail only: {rails}"
    checked = 0
    for f in pack["flights"]:
        src = ROOT / f["source"]
        if src.is_file():
            assert P.sha256_file(src) == f["source_sha256"], \
                f"{f['source']} changed since it was packed"
            checked += 1
    print(f"      {checked} of {len(pack['flights'])} pack sources verified on this host")
    if checked == 0:
        return SKIP


def test_the_committed_desktop_baselines_match_the_committed_pack():
    profiles = sorted(COMMITTED_DEV_PROFILE.parent.glob("shield_tick_profile*.json"))
    if not (profiles and COMMITTED_PACK.is_file()):
        return SKIP
    assert COMMITTED_DEV_PROFILE in profiles
    pack_sha = P.sha256_file(COMMITTED_PACK)
    n_rows = sum(len(f["rows"]) for f in P.load_pack(COMMITTED_PACK)["flights"])
    for path in profiles:
        prof = json.loads(path.read_text(encoding="utf-8"))
        assert prof["schema"] == P.PROFILE_SCHEMA, path.name
        assert prof["topology"] == "dev"
        assert prof["host"]["jetson"]["is_orin"] is False
        assert "hostname" not in prof["host"]["jetson"], path.name
        assert prof["failures"] == [], (path.name, prof["failures"])
        assert prof["pack"]["sha256"] == pack_sha, \
            f"{path.name} was taken on a different pack than the one committed"
        assert prof["pooled"]["design_load/grant"]["check"]["n"] == n_rows
        assert prof["host_speed"]["steady"], \
            f"{path.name}: the host changed speed during the run"
        assert prof["cpu_affinity"], f"{path.name}: not pinned"
        for f in prof["flights"]:
            ag = f["arms"]["as_flown"]["flown"]["logged_agreement"]
            if ag["informative"]:
                # The informative part, not the null-inflated total.
                assert ag["identical_nonempty"] >= 0.95 * ag["logged_nonempty"], (f["tag"], ag)
            else:
                assert "verifies nothing" in f.get("agreement_note", ""), f["tag"]
            for hz, cell in f["arms"]["design_load"].items():
                assert cell["geofence_ticks"] > 0, (f["tag"], hz)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs the committed evidence and its sources, "
                      f"a way to pin CPUs (psutil or os.sched_setaffinity), or a "
                      f"host that is not an Orin)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                        # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
