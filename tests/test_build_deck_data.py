"""tools/build_deck_data.py: the numbers the September deck's new slides read.

Run either way:
    pytest tests/test_build_deck_data.py -v
    python tests/test_build_deck_data.py

WHY THIS FILE EXISTS

The generator had no test, and two of its four sections broke silently or
loudly on 6 Oct:

* `bundle_facts()` read its own bundle back with `load_bundle(tmp)`. Since
  2026-10-06 that default refuses any bundle not signed by a trusted key, so on
  a machine without the lab key (or without `cryptography`: the 3.11 env) the
  whole deck build died with BundleSignatureError. It also wrote the two
  bundles it compares without a fixed issue time, so "byte-identical under a
  different name" turned False whenever the two writes straddled a second.
* `loop_rates()` read metrics.json's `det_hz`, the rate retracted on 29 Sept
  (it counted the start-gate wait), so the deck's rate-gate slide carried the
  inflated 3.76-5.15 Hz.

Each test below fails on the pre-6-Oct generator and passes on the fixed one.
Tests that need local flight artefacts (gitignored) print SKIP, never PASS.
"""
import json
import os
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import build_deck_data as D                                        # noqa: E402
import guardrail.bundle as B                                       # noqa: E402
from guardrail import load_policy                                  # noqa: E402

SKIP = "SKIP"


@contextmanager
def _env(**kv):
    old = {k: os.environ.get(k) for k in kv}
    try:
        for k, v in kv.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        yield
    finally:
        for k, v in old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


@contextmanager
def _clock_that_ticks_a_second_per_read():
    """Every datetime.now() inside guardrail.bundle is one second later than
    the last: the worst case of two writes straddling a second boundary."""
    real = B.datetime
    state = {"t": datetime(2026, 10, 6, 12, 0, 0, tzinfo=timezone.utc)}

    class Ticking(real):
        @classmethod
        def now(cls, tz=None):
            state["t"] += timedelta(seconds=1)
            return state["t"] if tz else state["t"].replace(tzinfo=None)

    B.datetime = Ticking
    try:
        yield
    finally:
        B.datetime = real


def _no_key():
    """No lab key on this 'machine': default_signer() must return None."""
    return _env(VLAGUARD_SIGNING_KEY=str(Path(tempfile.mkdtemp()) / "absent.pem"),
                SOURCE_DATE_EPOCH=None)


def test_bundle_facts_survive_a_machine_without_the_lab_key():
    with _no_key():
        assert B.default_signer() is None, "the no-key setup still found a key"
        facts = D.bundle_facts()        # the old code raised BundleSignatureError
    assert facts, "no policy produced any bundle facts"
    for name, f in facts.items():
        assert f["signature"] == B.SIG_UNSIGNED, (name, f["signature"])
        assert f["signed_by"] == B.UNSIGNED, (name, f["signed_by"])
        assert f["policy_hash"].startswith("sha256:") and len(f["policy_hash"]) == 71
        assert f["n_rules"] > 0 and f["bytes"] > 0


def test_signature_status_is_recorded_not_assumed():
    """Where this machine has the lab key and can verify it, the facts say
    "verified"; where it does not, "unsigned". Never a third story."""
    with _env(SOURCE_DATE_EPOCH=None):
        facts = D.bundle_facts()
    want = B.SIG_VERIFIED if B.default_signer() is not None else B.SIG_UNSIGNED
    for name, f in facts.items():
        assert f["signature"] == want, (name, f["signature"], want)
        assert "signature" in f and "signed_by" in f, "status missing from the facts"


def test_byte_identity_does_not_depend_on_when_the_second_ticks():
    with _no_key(), _clock_that_ticks_a_second_per_read():
        facts = D.bundle_facts()
    for name, f in facts.items():
        assert f["byte_identical_under_a_different_name"] is True, (
            f"{name}: two writes one second apart were not byte-identical; the "
            f"comparison is measuring the clock, not the archive")
    # One clock read per policy, and a true time: the ticking clock starts at
    # 2026-10-06 12:00:00, so every issue time is after it. Until the review
    # of 6 Oct every bundle said 2026-01-01T00:00:00Z, a date nothing happened on.
    stamps = [f["issued_at"] for f in facts.values()]
    assert all(s > "2026-10-06T12:00:00Z" for s in stamps), stamps
    assert "2026-01-01T00:00:00Z" not in stamps, stamps


def test_a_policy_that_writes_its_own_issue_time_keeps_it():
    """write_bundle refuses an issue time that contradicts the policy's own
    `issued_at` (the grant form). The fixed-date version of this generator
    therefore raised ValueError as soon as a deck policy carried one."""
    src = (ROOT / "policies" / "corridor_survey.yaml").read_text(encoding="utf-8")
    d = Path(tempfile.mkdtemp())
    p = d / "stamped.yaml"
    p.write_text("issued_at: '2026-10-01T08:30:00Z'\n" + src, encoding="utf-8")
    assert load_policy(p).issued_at, "the policy did not keep its issued_at"
    with _no_key(), _clock_that_ticks_a_second_per_read():
        facts = D.bundle_facts([p])
    f = facts["stamped"]
    assert f["issued_at"] == "2026-10-01T08:30:00Z", f["issued_at"]
    assert f["byte_identical_under_a_different_name"] is True


def test_the_clock_patch_really_moves_issued_at():
    """Guard for the test above: if the patched clock did not reach the
    bundle writer, that test would pass on the old code too."""
    with _no_key(), _clock_that_ticks_a_second_per_read():
        a = B._issued_at()
        b = B._issued_at()
    assert a != b, (a, b)


def _fake_run(root: Path, tag: str, det_hz_stored: float, n_ticks=101,
              dt=0.125, seq_step=0.4):
    """A flight folder whose stored det_hz is the old inflated kind (no
    `det_hz_all_inferences_over_mission_s_legacy` key) and whose log says the
    detector ran at seq_step / dt per second during the mission."""
    run = root / tag
    run.mkdir(parents=True)
    rows = [{"t": i * dt, "tick": i, "det_seq": int(i * seq_step)}
            for i in range(n_ticks)]
    (run / "flight_log.jsonl").write_text(
        "\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({"det_hz": det_hz_stored}),
                                      encoding="utf-8")
    span = rows[-1]["t"] - rows[0]["t"]
    return round((rows[-1]["det_seq"] - rows[0]["det_seq"]) / span, 2)


def test_rate_gate_uses_the_mission_rate_not_the_stored_one():
    root = Path(tempfile.mkdtemp())
    want = _fake_run(root, "city_locked", det_hz_stored=7.5)
    rg = D.loop_rates(out_root=root)
    assert rg["n_runs"] == 1, rg
    row = rg["runs"][0]
    assert row["det_hz"] == want, (row, want)
    assert row["det_hz_reported"] == 7.5, row
    assert row["det_hz"] < rg["gate"]["det_hz_min"], (
        "fixture broken: the mission rate should be below the 4.0 Hz gate")
    assert rg["n_meeting_gate"] == 0, rg


def test_rate_gate_on_the_real_flights_matches_the_correction_note():
    """The five published rates were recomputed on 6 Oct
    (docs/CORRECTION-2026-10-06-mid-evaluation-and-deck.md, row A14):
    city_locked 3.76 -> 2.77, city_kpi 5.15 -> 4.06, demo_traffic 4.03 -> 3.11.
    Needs the local flight logs, which are gitignored."""
    need = ("city_locked", "city_kpi", "demo_traffic")
    if not all((ROOT / "demo/out" / t / "flight_log.jsonl").is_file() for t in need):
        return SKIP
    rg = {r["tag"]: r for r in D.loop_rates()["runs"]}
    for tag, reported, mission in (("city_locked", 3.76, 2.77),
                                   ("city_kpi", 5.15, 4.06),
                                   ("demo_traffic", 4.03, 3.11)):
        assert rg[tag]["det_hz_reported"] == reported, rg[tag]
        assert rg[tag]["det_hz"] == mission, rg[tag]


def test_main_writes_where_it_is_told_and_on_the_no_crypto_env():
    """The whole build, end to end, on the 3.11 env when it exists (no
    cryptography, no key): the case that crashed."""
    py311 = Path(r"C:/Users/natha/.conda/envs/vla-drone/python.exe")
    exe = str(py311) if py311.is_file() else sys.executable
    out = Path(tempfile.mkdtemp()) / "deck.json"
    env = dict(os.environ, PYTHONIOENCODING="utf-8",
               VLAGUARD_SIGNING_KEY=str(out.parent / "absent.pem"))
    r = subprocess.run([exe, str(ROOT / "tools" / "build_deck_data.py"),
                        "--out", str(out)], capture_output=True, text=True,
                       encoding="utf-8", env=env, cwd=str(ROOT), timeout=300)
    assert r.returncode == 0, r.stdout[-800:] + r.stderr[-1500:]
    d = json.loads(out.read_text(encoding="utf-8"))
    for f in d["bundles"].values():
        assert f["signature"] == B.SIG_UNSIGNED, f
        assert f["byte_identical_under_a_different_name"] is True, f
    for row in d["rate_gate"]["runs"]:
        assert "det_hz_reported" in row, row
    assert "sweep counts:" in r.stdout, r.stdout[-500:]


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs local flight logs)")
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
