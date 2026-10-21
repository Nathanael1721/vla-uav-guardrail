"""The project's to-do list rules: tools/todo.py and tracker/data/todo.json.

Run either way:
    python tests/test_todo.py
    pytest tests/test_todo.py -v

Like the update log, the to-do list is in three languages and is read by the
Dashboard. These tests pin that `check` refuses each way it could quietly go
wrong - a missing language, a Simplified character, an owner that is a
person's name instead of a role, an item marked done with nothing that closed
it - by breaking one rule of a valid fixture at a time, and that the "first
VLA flight" milestone turns done only on a run whose manifest names a real
VLA (never the OWL-ViT detector, the StubVLA or a hand-written controller)
AND whose own records show the VLA server answered and its steps were flown
(review of 8035d0a, P2: a manifest that only names AerialVLA, whose server
never answered, counted as the first VLA flight), with at least half of the
requests answered and half of the loop ticks flown by the VLA (review of
d180e01, P3: a server that died after ten answers counted), by the model the
manifest configured (P7: the bare OpenVLA base serving an AerialVLA run
counted).
tracker/data/ is local-only (.gitignore): the real-file check SKIPs visibly
on a fresh clone.
"""
import copy
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import todo as T                                                # noqa: E402

SKIP = "SKIP"
CARDS = {"ARCH-20", "ARCH-21", "WP3-15"}
CHANGELOG = "# Changelog\n\n## [Unreleased]\n\n### 2026-10-21 — a wave\n\n#### Added\n- x\n"


def item(**kw):
    it = {"id": "VLA-01", "kind": "milestone", "wp": "ARCH", "owner": "agent", "status": "open",
          "cards": ["ARCH-20"], "due": None, "branch": None, "closed_by": None, "milestone": "vla",
          "auto": None, "probe": {"file": "demo/vla_server.py", "contains": "prompt_sha256"},
          "needs": ["gpu"], "effort": "M",
          "i18n": {"en": {"title": "The v2 server"}, "id": {"title": "Server v2"}, "zh": {"title": "v2 伺服器"}}}
    it.update(kw)
    return it


def valid_doc():
    return {"schema": 1, "items": [
        item(),
        item(id="VLA-05", owner="user", auto="first_vla_flight", probe=None, needs=["flight"],
             cards=["ARCH-21"],
             i18n={"en": {"title": "First VLA flight"}, "id": {"title": "Penerbangan VLA pertama"},
                   "zh": {"title": "首次 VLA 飛行"}}),
        item(id="A-03", kind="kpi", wp="WP3", status="blocked", cards=["WP3-15"], milestone=None,
             probe=None, due="2026-11-30",
             i18n={"en": {"title": "Fail-safe correctness"}, "id": {"title": "Ketepatan fail-safe"},
                   "zh": {"title": "失效保護正確率"}}),
    ]}


AERO = "aerialvla-lora/aero_vla@196f2f32"
DIGEST = "3f" * 32                       # a weights_sha256 as the server reports it


def step_rows(n=12, *, kind="step", accepted=True, parse_ok=True, digest=DIGEST, model_id=AERO,
              vla_ticks=8, source=None, t0=1000.0):
    """n vla_steps.jsonl rows as guardrail/vla_link.py writes them (schema
    vla_steps/1: the request with its capture time, the server's answer with
    its model identity, and what the tracker made of it). One decision every
    0.9 s, flown for 9 ticks of the 10 Hz loop until the next supersedes it."""
    rows = []
    for i in range(n):
        tc = t0 + 0.9 * i
        r = {"schema": "vla_steps/1", "seq": i, "kind": kind, "ok": kind != "server_error",
             "parse_ok": parse_ok, "accepted": accepted, "refused": None if accepted else kind,
             "t_capture": tc, "t_sent": tc + 0.01, "t_received": tc + 0.36,
             "raw_text": "20 49 49", "step": {"fwd_m": 1.0, "down_m": 0.0, "dpsi_rad": 0.0},
             "model": {"id": model_id, "base": "openvla-7b@47a0ec7f", "weights_sha256": digest,
                       "precision": "nf4", "attn": "eager", "merged": False},
             "outcome": {"end": "superseded", "t_start": tc + 0.36, "t_end": tc + 1.26,
                         "ticks": {"vla_step": vla_ticks, "hold": 1}}}
        if source:
            r = {"schema": "vla_steps/1", "seq": i, "kind": kind, "source": source, "accepted": accepted,
                 "outcome": {"ticks": {"vla_step": vla_ticks}}}
        rows.append(r)
    return rows


class Repo:
    """A temporary repository: a CHANGELOG and a demo/out with runs. A run
    is a vla_model_hash, or {"hash": ..., "steps": [vla_steps rows],
    "metrics": {...}, "kpi": {...}} for one with the run's own records."""

    def __init__(self, runs=None):
        self.runs = runs or {}

    def __enter__(self):
        self.root = Path(tempfile.mkdtemp(prefix="todo_test_"))
        (self.root / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
        for tag, h in self.runs.items():
            d = self.root / "demo" / "out" / tag
            d.mkdir(parents=True)
            spec = h if isinstance(h, dict) else {"hash": h}
            (d / "manifest.json").write_text(json.dumps({"vla_model_hash": spec["hash"]}), encoding="utf-8")
            if spec.get("steps") is not None:
                (d / "vla_steps.jsonl").write_text(
                    "".join(json.dumps(r) + "\n" for r in spec["steps"]), encoding="utf-8")
            if spec.get("metrics") is not None:
                (d / "metrics.json").write_text(json.dumps(spec["metrics"]), encoding="utf-8")
            if spec.get("kpi") is not None:
                (d / "kpi.json").write_text(json.dumps(spec["kpi"]), encoding="utf-8")
        return self.root

    def __exit__(self, *a):
        shutil.rmtree(self.root, ignore_errors=True)


def errs(mutate, runs=None, git=lambda root, sha: sha == "abc1234"):
    with Repo(runs) as root:
        d = valid_doc()
        mutate(d)
        return T.check(d, root, cards=CARDS, git=git)


def expect(mutate, needle, **kw):
    e = errs(mutate, **kw)
    assert any(needle in m for m in e), f"expected an error containing {needle!r}, got {e}"


# ------------------------------------------------------------------ the good case
def test_the_valid_fixture_passes():
    assert errs(lambda d: None) == []


def test_done_with_a_known_commit_or_changelog_heading_passes():
    def m(d):
        d["items"][0].update(status="done", closed_by={"commit": "abc1234"})
        d["items"][2].update(status="done", closed_by={"changelog": "### 2026-10-21 — a wave"})
    assert errs(m) == []


# ------------------------------------------------------------------ each rule, broken on purpose
def test_ids_must_be_well_formed_and_unique():
    expect(lambda d: d["items"][1].update(id="VLA-01"), "duplicate id")
    expect(lambda d: d["items"][0].update(id="vla 1"), "id must look like")


def test_a_missing_or_placeholder_language_is_refused():
    expect(lambda d: d["items"][0]["i18n"].pop("id"), "id title missing")
    expect(lambda d: d["items"][0]["i18n"]["zh"].update(title=""), "zh title missing")
    expect(lambda d: d["items"][0]["i18n"]["id"].update(title="TODO"), "still says TODO")


def test_simplified_chinese_is_refused():
    expect(lambda d: d["items"][0]["i18n"]["zh"].update(title="v2 服务器与飞机"), "Simplified")
    assert errs(lambda d: d["items"][0]["i18n"]["zh"].update(title="v2 伺服器與飛機")) == []


def test_the_owner_is_a_role_not_a_name():
    expect(lambda d: d["items"][0].update(owner="Nathan"), "owner must be a role")
    expect(lambda d: d["items"][0].update(owner="Prof Lai"), "owner must be a role")


def test_status_kind_wp_due_needs_effort_auto_are_checked():
    expect(lambda d: d["items"][0].update(status="finished"), "status must be one of")
    expect(lambda d: d["items"][0].update(kind="chore"), "kind must be one of")
    expect(lambda d: d["items"][0].update(wp="WP9"), "wp must be one of")
    expect(lambda d: d["items"][0].update(due="30 Nov"), "due must be YYYY-MM-DD")
    expect(lambda d: d["items"][0].update(needs=["coffee"]), "needs value")
    expect(lambda d: d["items"][0].update(effort="XL"), "effort must be one of")
    expect(lambda d: d["items"][0].update(auto="whenever"), "auto must be one of")


def test_an_unknown_card_is_refused():
    expect(lambda d: d["items"][0].update(cards=["ARCH-99"]), "unknown card ARCH-99")


def test_done_needs_something_that_closed_it():
    expect(lambda d: d["items"][0].update(status="done"), "closed_by names no commit")
    expect(lambda d: d["items"][0].update(status="done", closed_by={"commit": "deadbee"}),
           "not a commit git knows")
    expect(lambda d: d["items"][0].update(status="done", closed_by={"changelog": "### 2026-10-22 — later"}),
           "heading not found")
    # a run closes only the auto milestone, and only a run the VLA flew
    flown = {"hash": "aerialvla:openvla-7b+aero_vla@sha256:ab", "steps": step_rows()}
    expect(lambda d: d["items"][0].update(status="done", closed_by={"run": "r1"}),
           "not a demo/out run", runs={"r1": flown})
    expect(lambda d: d["items"][1].update(status="done", closed_by={"run": "r_owl"}),
           "not a demo/out run", runs={"r_owl": "google/owlvit-base-patch32@cbc355fb36458835"})
    assert errs(lambda d: d["items"][1].update(status="done", closed_by={"run": "r1"}),
                runs={"r1": flown}) == []
    # a manifest that only NAMES the VLA closes nothing (review of 8035d0a, P2)
    expect(lambda d: d["items"][1].update(status="done", closed_by={"run": "r1"}),
           "not a demo/out run", runs={"r1": "aerialvla:openvla-7b+aero_vla@sha256:ab"})


def test_a_probe_stays_inside_the_repository():
    for bad in ("../outside.py", "C:/Windows/x.py", "/etc/passwd", ""):
        expect(lambda d, b=bad: d["items"][0].update(probe={"file": b, "contains": "x"}), "probe must be")


# ------------------------------------------------------------------ the auto milestone
def test_only_an_allow_listed_vla_counts_as_a_vla_flight():
    assert T.classify_pilot("google/owlvit-base-patch32@cbc355fb36458835") == "detector"
    assert T.classify_pilot("guardrail.vla_stub.StubVLA@src:0123") == "stub"
    assert T.classify_pilot("none:hand-written-controller") == "none"
    assert T.classify_pilot("aerialvla:openvla-7b+aero_vla@sha256:ab") == "vla"
    assert T.classify_pilot("aerialvla-lora/aero_vla@196f2f32") == "vla"
    assert T.classify_pilot("openvla/openvla-7b@47a0ec7f") == "vla"
    assert T.classify_pilot("my-own-net@sha256:1") == "unknown"
    assert T.classify_pilot(None) == "missing" and T.classify_pilot("") == "missing"


def test_the_first_vla_flight_milestone_turns_done_on_a_real_vla_run_only():
    auto = valid_doc()["items"][1]
    with Repo({"r_owl": "google/owlvit-base-patch32@x", "r_stub": "guardrail.vla_stub.StubVLA@src:1",
               "r_none": "none:hand-written-controller",
               "_aside": {"hash": "openvla/openvla-7b@1", "steps": step_rows()}}) as root:
        vla = T.vla_runs(root / "demo" / "out")
        assert vla == [], f"a non-VLA (or a set-aside folder) counted: {vla}"
        assert T.effective(auto, vla) == ("open", None)
    with Repo({"r_vla": {"hash": "aerialvla:openvla-7b+aero_vla@sha256:ab", "steps": step_rows()}}) as root:
        vla = T.vla_runs(root / "demo" / "out")
        st, ev = T.effective(auto, vla)
        assert st == "done" and ev["run"] == "r_vla"
        assert T.effective(valid_doc()["items"][0], vla)[0] == "open", "a non-auto item moved"


def test_a_configured_vla_is_a_vla_flight_only_on_the_servers_own_evidence():
    """Review of 8035d0a (P2). classify_pilot judged the manifest's
    vla_model_hash alone, and that hash is computed on the PILOT side from
    the configured model id and the local weight files - nothing the server
    returned. A flight configured with AerialVLA whose v2 server never
    answered, or answered only parse_ok=false, holds zero or hovers on every
    tick, yet counted as a VLA flight and closed the first-VLA-flight
    milestone (VLA-05). A run is a VLA flight only when its own records hold
    at least MIN_VLA_DECISIONS answers the model parsed (a step or LAND)
    that the tracker accepted, each carrying the server's weights_sha256
    (one digest, not a placeholder) for an allow-listed model, and at least
    MIN_VLA_STEP_TICKS loop ticks flown toward those steps. Anything else
    configured as a VLA is "vla_unverified", with the reason."""
    H = "aerialvla:openvla-7b+aero_vla@sha256:ab"
    hold = step_rows(vla_ticks=0)                              # every tick a hold
    for r in hold:
        r["outcome"]["ticks"] = {"hold": 10}
    cases = {
        "no_records": {"hash": H},
        "no_accepted_decisions": {"hash": H, "steps": step_rows(kind="parse_fail", parse_ok=False)},
        "server_refused": {"hash": H, "steps": step_rows(kind="server_error", accepted=False, parse_ok=False)},
        "fake_digest": {"hash": H, "steps": step_rows(digest="0" * 64)},      # tests/vla_fake_server.py
        "no_digest": {"hash": H, "steps": step_rows(digest=None)},
        "scripted": {"hash": H, "steps": step_rows(source="scripted")},       # Decision.of(): no reply
        "too_few": {"hash": H, "steps": step_rows(n=T.MIN_VLA_DECISIONS - 1)},
        "two_digests": {"hash": H, "steps": step_rows(n=6) + step_rows(n=6, digest="4e" * 32)},
        "not_a_vla_model": {"hash": H, "steps": step_rows(model_id="my-own-net@1")},
        "zero_vla_step_ticks": {"hash": H, "steps": hold},
    }
    want = {"no_records": "no_records", "no_accepted_decisions": "no_accepted_decisions",
            "server_refused": "no_accepted_decisions", "fake_digest": "no_weights_digest",
            "no_digest": "no_weights_digest", "scripted": "no_accepted_decisions",
            "too_few": "too_few_decisions", "two_digests": "several_weights_digests",
            "not_a_vla_model": "model_not_a_vla", "zero_vla_step_ticks": "too_few_vla_step_ticks"}
    with Repo(cases) as root:
        demo = root / "demo" / "out"
        for tag, why in want.items():
            ev = T.vla_evidence(demo / tag)
            cls, reason = T.classify_run(H, ev)
            assert cls == "vla_unverified" and reason.split(":")[0] == why, (tag, cls, reason, ev)
        assert T.vla_runs(demo) == [], "a run without the server's evidence counted as a VLA flight"
        auto = valid_doc()["items"][1]
        assert T.effective(auto, T.vla_runs(demo)) == ("open", None), "VLA-05 closed on a dead model"
    flown = step_rows()
    with Repo({"r_ok": {"hash": H, "steps": flown},
               # the same records under a manifest that says something else flew
               # (a VLA run in the shadow of the hand-written controller) is not a VLA flight
               "r_shadow": {"hash": "none:hand-written-controller", "steps": flown},
               # the pilot's own summary in metrics.json, RemoteVLABackend.stats() shape
               "r_metrics": {"hash": H, "metrics": {"pilot": {"vla": {
                   "decisions_accepted": 40, "ticks_by_author": {"vla_step": 300, "hold": 20},
                   "identity": {"id": AERO, "weights_sha256": DIGEST}}}}},
               "r_metrics_dead": {"hash": H, "metrics": {"pilot": {"vla": {
                   "decisions_accepted": 0, "ticks_by_author": {"none": 600},
                   "identity": {"id": AERO, "weights_sha256": DIGEST}}}}}}) as root:
        demo = root / "demo" / "out"
        ev = T.vla_evidence(demo / "r_ok")
        assert T.classify_run(H, ev) == ("vla", None), ev
        assert ev["decisions"] == 12 and ev["vla_step_ticks"] == 96 and ev["weights_sha256"] == DIGEST
        assert ev["source"] == "vla_steps.jsonl" and ev["model_id"] == AERO
        assert T.classify_run("none:hand-written-controller", T.vla_evidence(demo / "r_shadow"))[0] == "none"
        mev = T.vla_evidence(demo / "r_metrics")
        assert T.classify_run(H, mev) == ("vla", None) and mev["source"] == "metrics.json#/pilot/vla", mev
        assert T.classify_run(H, T.vla_evidence(demo / "r_metrics_dead"))[0] == "vla_unverified"
        assert sorted(t for t, _ in T.vla_runs(demo)) == ["r_metrics", "r_ok"], T.vla_runs(demo)
    # a broken or foreign line is skipped, never fatal
    with Repo({"r_mixed": {"hash": H, "steps": flown}}) as root:
        p = root / "demo" / "out" / "r_mixed" / "vla_steps.jsonl"
        p.write_text("{not json\n" + json.dumps({"schema": "other"}) + "\n" + p.read_text(encoding="utf-8"),
                     encoding="utf-8")
        ev = T.vla_evidence(p.parent)
        assert T.classify_run(H, ev) == ("vla", None) and ev["bad_lines"] == 1, ev


def _dying(n_ok=10, n_fail=1150):
    """The review's probe (review of d180e01, P3): ten answers the model
    parsed and the tracker took, one vla_step tick each (one second of VLA
    flight), then the server gone: 1150 link_failure rows, one per 0.1 s
    (about 115 s of hover)."""
    rows = step_rows(n=n_ok)
    for r in rows:
        r["outcome"]["ticks"] = {"vla_step": 1}
    t = rows[-1]["t_received"]
    for i in range(n_fail):
        tc = t + 0.1 * (i + 1)
        rows.append({"schema": "vla_steps/1", "seq": n_ok + i, "kind": "link_failure",
                     "t_capture": tc, "t_sent": tc, "t_received": tc + 0.1,
                     "failure": {"kind": "connect_refused", "detail": "[WinError 10061]"}})
    return rows


def test_a_vla_that_dies_early_is_not_a_vla_flight():
    """Review of d180e01 (P3). MIN_VLA_DECISIONS and MIN_VLA_STEP_TICKS are
    absolute counts over the whole run: a manifest naming AerialVLA with ten
    accepted decisions (one vla_step tick each) and then 1150 link failures
    (~115 s of hover) classified as ('vla', None), became last_vla_flight
    and would have closed VLA-05, and the page showed only the decision
    count and the digest. A VLA flight now also needs the VLA to have
    answered at least MIN_DECISION_SHARE of the requests it was sent and to
    have authored (vla_step or hold) at least MIN_VLA_TICK_SHARE of the
    mission's loop ticks; a run with no way to count its mission ticks is
    "mission_ticks_unknown", never a pass. The evidence carries the link
    failures, the server errors and both shares."""
    H = "aerialvla:openvla-7b+aero_vla@sha256:ab"
    no_time = step_rows()
    for r in no_time:
        for k in ("t_capture", "t_sent", "t_received"):
            r.pop(k)
        r["outcome"] = {"ticks": r["outcome"]["ticks"]}
    cases = {
        # the probe, from the steps file alone (its own rows span ~124 s)
        "dying_server": ({"hash": H, "steps": _dying()}, "decision_share_low"),
        # a server that refused nearly everything it was sent
        "erroring_server": ({"hash": H, "steps": step_rows() + step_rows(
            n=40, kind="server_error", accepted=False, parse_ok=False, t0=1011.0)}, "decision_share_low"),
        # the pilot's own summary: 40 good answers, then 1100 ticks of hover
        "mostly_hover": ({"hash": H, "metrics": {"pilot": {"vla": {
            "decisions_accepted": 40, "parse_fail": 0, "server_error": 0, "link_failure": 0,
            "ticks_by_author": {"vla_step": 30, "hold": 10, "hover": 1100, "none": 10},
            "identity": {"id": AERO, "weights_sha256": DIGEST}}}}}, "vla_tick_share_low"),
        # the pilot's summary, half of the accepted answers malformed: the
        # tracker holds zero under a parse failure and authors it "hold", so
        # the totals' 200 hold ticks are not all the VLA's finished steps; a
        # parsed decision's share of them (100) with 100 vla_step ticks is
        # 200 of 500, under half (counting every hold would have made it 300)
        "parse_fail_holds": ({"hash": H, "metrics": {"pilot": {"vla": {
            "decisions_accepted": 20, "parse_fail": 20, "server_error": 0, "link_failure": 0,
            "tracker": {"kind_step": 20, "kind_parse_fail": 20},
            "ticks_by_author": {"vla_step": 100, "hold": 200, "hover": 200},
            "identity": {"id": AERO, "weights_sha256": DIGEST}}}}}, "vla_tick_share_low"),
        # twelve good decisions (108 ticks) in a 1200-tick mission (kpi.json ticks)
        "short_vla_long_mission": ({"hash": H, "steps": step_rows(), "kpi": {"ticks": 1200}},
                                   "vla_tick_share_low"),
        # rows with no times, no metrics, no kpi: the share cannot be judged
        "no_mission_length": ({"hash": H, "steps": no_time}, "mission_ticks_unknown"),
    }
    with Repo({k: v[0] for k, v in cases.items()}) as root:
        demo = root / "demo" / "out"
        for tag, (_, why) in cases.items():
            ev = T.vla_evidence(demo / tag)
            cls, reason = T.classify_run(H, ev)
            assert cls == "vla_unverified" and reason.split(":")[0] == why, (tag, cls, reason)
        assert T.vla_runs(demo) == [], "a VLA that died early counted as a VLA flight"
        assert T.effective(valid_doc()["items"][1], T.vla_runs(demo)) == ("open", None)
        ev = T.vla_evidence(demo / "dying_server")
        assert ev["link_failure"] == 1150 and ev["requests"] == 1160 and ev["decisions"] == 10, ev
        assert ev["decision_share"] < 0.01 and ev["vla_tick_share"] < 0.01, ev
        assert ev["mission_ticks_source"].startswith("vla_steps.jsonl"), ev["mission_ticks_source"]
        assert T.vla_evidence(demo / "erroring_server")["server_error"] == 40
        assert T.vla_evidence(demo / "short_vla_long_mission")["mission_ticks"] == 1200
    # a run the VLA flew passes, whichever record gives the mission length
    with Repo({"rows": {"hash": H, "steps": step_rows()},
               "kpi": {"hash": H, "steps": step_rows(), "kpi": {"ticks": 112}},
               "pilot": {"hash": H, "steps": step_rows(), "metrics": {"ticks": 115, "pilot": {"vla": {
                   "decisions_accepted": 12, "ticks_by_author": {"vla_step": 96, "hold": 12, "hover": 4},
                   "identity": {"id": AERO, "weights_sha256": DIGEST}}}}, "kpi": {"ticks": 9999}}}) as root:
        demo = root / "demo" / "out"
        src = {}
        for tag in ("rows", "kpi", "pilot"):
            ev = T.vla_evidence(demo / tag)
            assert T.classify_run(H, ev) == ("vla", None), (tag, ev)
            assert ev["decision_share"] == 1.0 and ev["vla_tick_share"] > 0.9, (tag, ev)
            src[tag] = ev["mission_ticks_source"]
        assert src["kpi"] == "kpi.json#/ticks" and src["pilot"] == "metrics.json#/pilot/vla/ticks_by_author", src
        assert T.vla_evidence(demo / "pilot")["mission_ticks"] == 112


def test_the_server_must_have_served_the_configured_model():
    """Review of d180e01 (P7). classify_run took any allow-listed model id
    the server reported, never comparing it with what the manifest
    configured: a run configured as AerialVLA and served by the bare
    OpenVLA base (or by a run1-3 fine-tune, the velocity-label weights the
    brief rules out) counted as that VLA flight. Now the server's model must
    be the configured family (AerialVLA vs OpenVLA), and when the manifest's
    vla_model_hash carries a sha256 digest the server's weights_sha256 must
    be that digest - or the pilot side must have recorded that its local
    digest agreed with the server's report (guardrail/manifest.py
    resolve_vla_hash "local; reported agrees", the hash of an id that mixes
    in a HuggingFace revision). A pilot side that itself saw the server run
    other bytes ("@unresolved:reported-digest-mismatch") never counts."""
    other = "5a" * 32
    agreed = {"pilot": {"kind": "vla", "hash_source": "local; reported agrees", "reported_digest": DIGEST}}
    runs = {
        "base_served": ("aerialvla:openvla-7b+aero_vla@sha256:ab",
                        {"steps": step_rows(model_id="openvla-7b@47a0ec7f")}, "server_model_differs"),
        "other_weights": ("aerialvla-lora/aero_vla@sha256:" + other, {"steps": step_rows()},
                          "server_model_differs"),
        "pilot_saw_mismatch": ("aerialvla-lora/aero_vla@unresolved:reported-digest-mismatch",
                               {"steps": step_rows()}, "server_model_differs"),
        "same_weights": ("aerialvla-lora/aero_vla@sha256:" + DIGEST, {"steps": step_rows()}, None),
        "rev_mixed_agreed": ("aerialvla-lora/aero_vla@sha256:" + other,
                             {"steps": step_rows(), "metrics": agreed}, None),
        "openvla_both": ("openvla/openvla-7b@47a0ec7f",
                         {"steps": step_rows(model_id="openvla-7b@47a0ec7f")}, None),
    }
    with Repo({tag: dict(spec, hash=h) for tag, (h, spec, _) in runs.items()}) as root:
        demo = root / "demo" / "out"
        for tag, (h, _, why) in runs.items():
            cls, reason = T.classify_run(h, T.vla_evidence(demo / tag))
            if why is None:
                assert (cls, reason) == ("vla", None), (tag, cls, reason)
            else:
                assert cls == "vla_unverified" and reason.split(":")[0] == why, (tag, cls, reason)
        assert sorted(t for t, _ in T.vla_runs(demo)) == ["openvla_both", "rev_mixed_agreed", "same_weights"]


def test_the_simplified_check_can_fail_and_pass():
    assert {"这", "飞"} <= T.SIMPLIFIED
    assert not ({"這", "飛", "伺", "服", "器"} & T.SIMPLIFIED)


# ------------------------------------------------------------------ the real file
def test_the_real_todo_passes_check():
    p = ROOT / "tracker" / "data" / "todo.json"
    if not p.is_file():
        print("      SKIP: tracker/data/todo.json is not here (local-only data)")
        return SKIP
    doc = json.loads(p.read_text(encoding="utf-8"))
    e = T.check(doc, ROOT)
    assert e == [], e[:10]
    ids = {i["id"] for i in doc["items"]}
    vla = [i for i in doc["items"] if i.get("milestone") == "vla"]
    assert len(vla) >= 9, f"the VLA-pilot milestones are not all seeded ({len(vla)})"
    assert sum(1 for i in vla if i.get("auto") == "first_vla_flight") == 1, "exactly one auto milestone"
    assert len(ids) == len(doc["items"])


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
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
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
