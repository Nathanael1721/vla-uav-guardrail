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
VLA (never the OWL-ViT detector, the StubVLA or a hand-written controller).
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


class Repo:
    """A temporary repository: a CHANGELOG and a demo/out with runs."""

    def __init__(self, runs=None):
        self.runs = runs or {}

    def __enter__(self):
        self.root = Path(tempfile.mkdtemp(prefix="todo_test_"))
        (self.root / "CHANGELOG.md").write_text(CHANGELOG, encoding="utf-8")
        for tag, h in self.runs.items():
            d = self.root / "demo" / "out" / tag
            d.mkdir(parents=True)
            (d / "manifest.json").write_text(json.dumps({"vla_model_hash": h}), encoding="utf-8")
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
    # a run closes only the auto milestone, and only a run that names a real VLA
    expect(lambda d: d["items"][0].update(status="done", closed_by={"run": "r1"}),
           "not a demo/out run", runs={"r1": "aerialvla:openvla-7b+aero_vla@sha256:ab"})
    expect(lambda d: d["items"][1].update(status="done", closed_by={"run": "r_owl"}),
           "not a demo/out run", runs={"r_owl": "google/owlvit-base-patch32@cbc355fb36458835"})
    assert errs(lambda d: d["items"][1].update(status="done", closed_by={"run": "r1"}),
                runs={"r1": "aerialvla:openvla-7b+aero_vla@sha256:ab"}) == []


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
               "r_none": "none:hand-written-controller", "_aside": "openvla/openvla-7b@1"}) as root:
        vla = T.vla_runs(root / "demo" / "out")
        assert vla == [], f"a non-VLA (or a set-aside folder) counted: {vla}"
        assert T.effective(auto, vla) == ("open", None)
    with Repo({"r_vla": "aerialvla:openvla-7b+aero_vla@sha256:ab"}) as root:
        vla = T.vla_runs(root / "demo" / "out")
        st, ev = T.effective(auto, vla)
        assert st == "done" and ev["run"] == "r_vla"
        assert T.effective(valid_doc()["items"][0], vla)[0] == "open", "a non-auto item moved"


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
