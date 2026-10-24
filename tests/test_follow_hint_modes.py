"""
The follow dataset's memory and search samples (tools/build_follow_dataset.py
--hint-modes, --close-bottom-frac / --close-width-frac) and the ONE hint rule
the flight and the builder share (demo/vla_mission.follow_hint_step).

Run either way:
    pytest tests/test_follow_hint_modes.py -v
    python tests/test_follow_hint_modes.py

WHAT IS AT STAKE (2026-10-24)

The builder kept only captures with a FRESH box (age <= 1 s) and dropped
close boxes with DetectorHint's default rule (bottom edge at 0.92 of the
image). So the fine-tuned model had never seen a frame without the car, nor
a box near the bottom edge - and a VLA follow sends both: no fresh box ->
DetectorHint's MEMORY phrase (the last sighting's bearing, held in the world
frame and re-read against the current heading) for memory_s, then its
SEARCH phrase (a quarter turn toward the side it was last seen on) for
search_s, then nothing; and the flight runs --close-bottom-frac 1.0. Flight
vla_follow_ft_03_pf_20261024 overshot a car stopped ahead, the car left the
bottom of the frame, the hint went to memory then search, and the model
kept answering "forward 4.85 m" for 30 s, away from the car behind it. The
expert in those states - its own recovery, and DAgger's after a VLA window
- is the missing label.

So, pinned here:

  A. ONE RULE. vla_mission.follow_hint_step is DetectorHint's decision
     (fresh / memory / search / close / none, the phrase, the remembered
     sighting); DetectorHint calls it and so does the builder, so the
     phrases and kinds are identical on a lost-and-found sequence, the
     memory phrase rotates with the heading exactly as in flight, and the
     search side follows the current heading too.
  B. THE BUILDER'S EMULATION: --hint-modes fresh,memory,search runs the rule
     over each run's captures in order; a capture the flight would send
     nothing for is dropped "no_hint", a kind left out of --hint-modes
     "hint_kind"; every sample records hint_kind and the emulated state;
     stats.json counts per kind, in all and per run; the DART/DAgger label
     rule applies to memory and search samples unchanged.
  C. THE CLOSE RULE: the builder takes the flight's (--close-bottom-frac
     1.0) - a close box is no sample, as it is no request (and it still
     seeds the memory, as in flight), a box the flight would send keeps its
     phrase - and records it in stats.json and every sample.
  D. THE MIRROR handles memory and search phrases (the search phrase is
     directional) and the hint's bearings.
  E. THE DEFAULT IS UNCHANGED: fresh only, the 0.92 rule, the same samples
     and drop reasons; the trainer reads the new samples and the evaluation
     tool splits its held-out metrics by hint_kind.

No simulator, GPU, model or Pillow (images=False everywhere).

Shown failing first: at d9cdf43 vla_mission.follow_hint_step and
HINT_KINDS do not exist, build_dataset takes no hint_modes /
close_bottom_frac and the CLI no --hint-modes (all 12 tests below fail: an
AttributeError, a TypeError, argparse's exit on an unknown flag, or the
source check finding no follow_hint_step call).
"""
from __future__ import annotations

import inspect
import json
import math
import sys
import tempfile
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools", ROOT / "training", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.vla_link import Capture, Pose, wrap_pi                 # noqa: E402

SKIP = "SKIP"
ALL = ("fresh", "memory", "search")
DT = 0.1


def _B():
    import build_follow_dataset as B
    return B


def _VM():
    import vla_mission as VM
    return VM


def _TF():
    """tests/test_follow_dataset.py's synthetic recordings (the recorder's
    own file format) and its constants."""
    import test_follow_dataset as TF
    return TF


def _rows(path: Path) -> list[dict]:
    return [json.loads(ln) for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]


def _bridge():
    from vla_bridge import VLABridge
    return VLABridge("a red car", hfov_deg=90.0)


def _box(det, t_box, yaw_at, seq=0):
    return {"det": det, "t_box": t_box, "yaw_box": yaw_at(t_box), "t_published": t_box + 0.1,
            "seq": seq, "tier": "ok"}


def _fly(run_dir: Path, close=None, memory_s=6.0, search_s=20.0, fresh_s=1.0) -> dict:
    """The run's captures fed to the FLIGHT's DetectorHint (vla_fly's follow
    construction: memory, search, close, kinds recorded), one call per
    capture as if each were a request: {i: (kind, phrase, last_kind)}."""
    VM = _VM()
    now = {"v": (None, None, None, None)}
    h = VM.DetectorHint(lambda: now["v"], _bridge(), fresh_s=fresh_s, memory_s=memory_s,
                        search_s=search_s, close=close, record_kind=True)
    out = {}
    for c in _rows(run_dir / "captures.jsonl"):
        b = c.get("box") or {}
        now["v"] = (b.get("det"), b.get("t_box"), b.get("yaw_box"), b.get("t_published"))
        p = c["pose"]
        got = h(Capture(c["t_capture"], Pose(p["x"], p["y"], p["up"], p["yaw_rad"]),
                        b"\xff\xd8", None))
        out[c["i"]] = (None if got is None else got.kind, None if got is None else got.phrase,
                       h.last_kind)
    return out


# --------------------------------------------------------------------------- #
# The lost-and-found sequence (A, B): 40 s at 10 Hz, turning 0.05 rad/s right
# --------------------------------------------------------------------------- #
T_LAST = 2.75          # the last box's frame (tau): fresh to 3.7, memory to 8.7, search to 28.7


def _lost_and_found(k, t, yaw_at):
    """Fresh 30 deg right to tau 2.9; the same detection (frame at 2.75)
    published on to 4.9, so fresh to 3.7 and stale after; no box 5.0-33.9;
    a fresh box 20 deg LEFT from 34.0."""
    T0 = _TF().T0
    tau = k * DT
    det = _TF()._det_at_bearing
    if tau < 3.0 - 1e-9:
        return _box(det(30.0), t - 0.2, yaw_at, k)
    if tau < 5.0 - 1e-9:
        return _box(det(30.0), T0 + T_LAST, yaw_at, 28)
    if tau < 34.0 - 1e-9:
        return None
    return _box(det(-20.0), t - 0.2, yaw_at, k)


def _expected_kind(k):
    tau = round(k * DT, 6)
    if tau <= 3.7 + 1e-9 or tau >= 34.0 - 1e-9:
        return "fresh"
    if tau <= 8.7 + 1e-9:
        return "memory"
    if tau <= 28.7 + 1e-9:
        return "search"
    return None


def _lf_run(td: Path, tag="runLF", **kw) -> Path:
    TF = _TF()
    return TF._write_run(td, tag, TF._turning(speed=3.0, rate=0.05), seconds=40.0,
                         box=_lost_and_found, **kw)


def test_the_flight_and_the_builder_share_one_hint_rule_on_a_lost_and_found_sequence():
    B, VM, TF = _B(), _VM(), _TF()
    assert tuple(VM.HINT_KINDS) == ALL
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _lf_run(td)
        flight = _fly(run, close=VM.BoxClose(1.0, 0.25))
        kinds = [flight[k][0] for k in sorted(flight)]
        assert kinds == [_expected_kind(k) for k in sorted(flight)], \
            [(k, kinds[k], _expected_kind(k)) for k in range(len(kinds))
             if kinds[k] != _expected_kind(k)][:5]
        # fresh -> memory -> search -> none -> fresh again, in that order
        runs = [kinds[0]] + [b for a, b in zip(kinds, kinds[1:]) if a != b]
        assert runs == ["fresh", "memory", "search", None, "fresh"], runs
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                             hint_modes=ALL, close_bottom_frac=1.0, close_width_frac=0.25)
        rows = {r["i"]: r for r in _rows(td / "ds" / "samples.jsonl")}
        n_none = sum(1 for v in flight.values() if v[0] is None)
        assert n_none == 52 and st["dropped"]["no_hint"] == n_none, st["dropped"]
        assert st["dropped"]["no_future_pose"] == 10, st["dropped"]     # 39.1 .. 40.0 s
        assert set(st["dropped"]) == {"no_hint", "no_future_pose"}, st["dropped"]
        for k, (kind, phrase, _last) in flight.items():
            if kind is None or k * DT > 39.0 + 1e-9:
                assert k not in rows, k
                continue
            r = rows[k]
            assert (r["hint_kind"], r["phrase"]) == (kind, phrase), (k, r["hint_kind"],
                                                                     r["phrase"], kind, phrase)
            assert r["hint"]["kind"] == kind and r["hint_source"] == "detector"
            assert r["prompt"] == C.build_prompt(phrase, "a red car")
        # The pure rule alone, carrying the sighting, says the same.
        seen = None
        br = _bridge()
        for c in _rows(run / "captures.jsonl"):
            b = c.get("box") or {}
            step = VM.follow_hint_step((b.get("det"), b.get("t_box"), b.get("yaw_box")),
                                       c["t_capture"], c["pose"]["yaw_rad"], seen, br,
                                       fresh_s=1.0, memory_s=6.0, search_s=20.0,
                                       close=VM.BoxClose(1.0, 0.25))
            seen = step.seen
            want_kind, want_phrase, _ = flight[c["i"]]
            assert (step.kind, step.phrase) == (want_kind, want_phrase), c["i"]
        # the emulated state: the box, and the time since the sighting
        for k, r in rows.items():
            tau = k * DT
            if r["hint_kind"] == "fresh":
                assert r["hint"]["box"] == "fresh" and r["hint"]["since_seen_s"] is None
            else:
                assert r["hint"]["box"] == ("stale" if tau < 5.0 - 1e-9 else "none"), k
                assert math.isclose(r["hint"]["since_seen_s"], tau - T_LAST, abs_tol=1e-3), k


def test_the_memory_phrase_is_the_sighting_re_read_against_the_current_heading():
    """vla_mission's memory: the sighting's WORLD bearing (heading at its
    frame + its bearing there) against each capture's heading; the search
    turns toward the side that re-read gives NOW. Turning 0.15 rad/s right
    after losing a car 40 deg right: forward-right, then straight ahead as
    the nose comes round, then the search turns LEFT (it is now behind the
    left of the nose) - the builder's samples and the flight agree."""
    B, VM, TF = _B(), _VM(), _TF()
    from vla_bridge import bearing_to_phrase

    def box(k, t, yaw_at):
        return _box(TF._det_at_bearing(40.0), t - 0.05, yaw_at, k) if k * DT < 2.0 - 1e-9 \
            else None

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        motion = TF._turning(speed=2.0, rate=0.15)
        run = TF._write_run(td, "runRot", motion, seconds=20.0, box=box)
        flight = _fly(run)
        B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                        hint_modes=ALL)
        rows = _rows(td / "ds" / "samples.jsonl")
        t_seen = 1.85
        world = motion(t_seen)[3] + math.radians(40.0)
        mem = [r for r in rows if r["hint_kind"] == "memory"]
        sea = [r for r in rows if r["hint_kind"] == "search"]
        assert mem and sea
        for r in mem + sea:
            rel = wrap_pi(world - r["pose"]["yaw_rad"])
            assert math.isclose(r["hint"]["seen_rel_deg"], math.degrees(rel), abs_tol=1e-3), r["i"]
            if r["hint_kind"] == "memory":
                assert r["phrase"] == bearing_to_phrase(rel), r["i"]
                assert math.isclose(r["hint"]["bearing_deg"], math.degrees(rel), abs_tol=1e-3)
            else:
                assert r["phrase"] == (C.RIGHT if rel >= 0 else C.LEFT), r["i"]
                assert r["hint"]["bearing_deg"] == (90.0 if rel >= 0 else -90.0)
            assert flight[r["i"]][:2] == (r["hint_kind"], r["phrase"]), r["i"]
        assert [p for p in dict.fromkeys(r["phrase"] for r in mem)] == \
            [C.FORWARD_RIGHT, C.STRAIGHT_AHEAD], Counter(r["phrase"] for r in mem)
        assert {r["phrase"] for r in sea} == {C.LEFT}, Counter(r["phrase"] for r in sea)
        assert round(max(r["i"] for r in mem) * DT, 3) == 7.8          # since 1.85: 6 s
        assert round(min(r["i"] for r in sea) * DT, 3) == 7.9


def test_a_capture_the_flight_would_not_send_is_no_hint_and_the_kinds_are_selectable():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _lf_run(td)
        st = B.build_dataset([run], td / "fm", horizon_s=1.0, stride_s=0.0, images=False,
                             hint_modes=("fresh", "memory"))
        assert st["dropped"]["hint_kind"] == 200 and st["dropped"]["no_hint"] == 52, st["dropped"]
        assert st["hints"]["by_kind_real"] == {"fresh": 89, "memory": 50}, st["hints"]
        st = B.build_dataset([run], td / "s", horizon_s=1.0, stride_s=0.0, images=False,
                             hint_modes=("search",))
        assert st["hints"]["by_kind_real"] == {"search": 200}, st["hints"]
        # every fresh capture (99, the last 10 included: the hint is decided
        # before the label) and every memory one (50) is of a kind left out
        assert st["dropped"] == {"hint_kind": 149, "no_hint": 52}, st["dropped"]
        # fresh only (the default): today's reasons, no emulation
        st = B.build_dataset([run], td / "f", horizon_s=1.0, stride_s=0.0, images=False)
        assert st["dropped"] == {"no_box": 290, "stale_box": 12, "no_future_pose": 10}, \
            st["dropped"]
        assert st["hints"]["by_kind_real"] == {"fresh": 89}
        # refused: what the build could not do as it would record it
        for kw, word in ((dict(hint_modes=("fresh", "bogus")), "hint-modes"),
                         (dict(hint_modes=()), "hint-modes"),
                         (dict(memory_s=6.0), "memory"),
                         (dict(search_s=20.0), "search"),
                         (dict(hint_modes=ALL, memory_s=-1.0), "memory"),
                         (dict(keep_close=True, close_bottom_frac=1.0), "close"),
                         (dict(close_bottom_frac=0.0), "close"),
                         (dict(close_width_frac=1.5), "close")):
            try:
                B.build_dataset([run], td / "bad", horizon_s=1.0, images=False, force=True, **kw)
            except B.DatasetError as exc:
                assert word in str(exc), (kw, str(exc))
            else:
                raise AssertionError(f"{kw} was accepted")


# --------------------------------------------------------------------------- #
# C. The close rule
# --------------------------------------------------------------------------- #
def _det_low(deg, bottom_px, W=768, H=432, w_frac=0.10, h=40.0):
    """A box at bearing `deg` whose bottom edge is at `bottom_px` (of H)."""
    det = _TF()._det_at_bearing(deg, w_frac=w_frac, W=W, H=H)
    det[1], det[3] = bottom_px - h / 2.0, h
    return det


def test_the_close_rule_is_the_flights_at_0_92_and_at_1_0():
    B, VM, TF = _B(), _VM(), _TF()
    near_bottom = _det_low(8.0, 0.95 * 432)          # bottom at 0.95: close only at 0.92
    at_edge = _det_low(25.0, 432.0)                  # bottom at the edge: close at both
    assert VM.BoxClose(0.92, 0.25)(near_bottom) and not VM.BoxClose(1.0, 0.25)(near_bottom)
    assert VM.BoxClose(0.92, 0.25)(at_edge) and VM.BoxClose(1.0, 0.25)(at_edge)

    def box(k, t, yaw_at):
        tau = k * DT
        if tau < 3.0 - 1e-9:
            return _box(near_bottom, t - 0.1, yaw_at, k)
        if tau < 5.0 - 1e-9:
            return _box(at_edge, t - 0.1, yaw_at, k)
        return None

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runClose", TF._straight(speed=2.0), seconds=10.0, box=box)
        for bottom, modes in ((0.92, ALL), (1.0, ALL), (1.0, ("fresh",)), (0.92, ("fresh",))):
            out = td / f"ds_{bottom}_{len(modes)}"
            st = B.build_dataset([run], out, horizon_s=1.0, stride_s=0.0, images=False,
                                 hint_modes=modes, close_bottom_frac=bottom,
                                 close_width_frac=0.25)
            rows = {r["i"]: r for r in _rows(out / "samples.jsonl")}
            flight = _fly(run, close=VM.BoxClose(bottom, 0.25),
                          memory_s=6.0 if len(modes) > 1 else 0.0,
                          search_s=20.0 if len(modes) > 1 else 0.0)
            for k, (kind, phrase, last) in flight.items():
                if kind is None or k * DT > 9.0 + 1e-9:
                    assert k not in rows, (bottom, k, last)
                    continue
                assert (rows[k]["hint_kind"], rows[k]["phrase"]) == (kind, phrase), (bottom, k)
            n_close = sum(1 for v in flight.values() if v[2] == "close")
            assert st["dropped"]["close_box"] == n_close == (50 if bottom == 0.92 else 20), \
                (bottom, st["dropped"], n_close)
            if bottom == 1.0:
                # the box the flight sends is a sample, with its phrase
                assert all(rows[k]["phrase"] == C.STRAIGHT_AHEAD
                           and rows[k]["hint_kind"] == "fresh" for k in range(30)), bottom
            if len(modes) > 1:
                # the close box still seeds the memory (vla_mission: the sighting
                # is taken before the close check): forward-right, from it
                mem = [r for r in rows.values() if r["hint_kind"] == "memory"]
                assert mem and {r["phrase"] for r in mem} == {C.FORWARD_RIGHT}, mem[:1]
            rule = {"bottom_frac": bottom, "width_frac": 0.25}
            assert st["hints"]["close_rule"] == rule and st["args"]["close_bottom_frac"] == bottom
            assert st["args"]["close_width_frac"] == 0.25
            assert all(r["close_rule"] == rule for r in rows.values())
        st = B.build_dataset([run], td / "keep", horizon_s=1.0, stride_s=0.0, images=False,
                             keep_close=True)
        assert st["hints"]["close_rule"] is None and "close_box" not in st["dropped"]
        assert all(r["close_rule"] is None for r in _rows(td / "keep" / "samples.jsonl"))


# --------------------------------------------------------------------------- #
# D. The mirror
# --------------------------------------------------------------------------- #
def test_memory_and_search_samples_mirror_their_phrase_and_their_bearings():
    B, TF = _B(), _TF()

    def box(k, t, yaw_at):
        return _box(TF._det_at_bearing(35.0), t - 0.1, yaw_at, k) if k * DT < 2.0 - 1e-9 \
            else None

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runM", TF._turning(speed=3.0, rate=0.02), seconds=14.0, box=box)
        B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                        hint_modes=ALL, mirror=True)
        rows = _rows(td / "ds" / "samples.jsonl")
        src = {r["id"]: r for r in rows if not r["mirrored"]}
        mir = [r for r in rows if r["mirrored"]]
        assert len(mir) == len(src) and {r["hint_kind"] for r in mir} == set(ALL)
        want = {C.FORWARD_RIGHT: C.FORWARD_LEFT, C.STRAIGHT_AHEAD: C.STRAIGHT_AHEAD,
                C.RIGHT: C.LEFT}
        assert {s["phrase"] for s in src.values() if s["hint_kind"] == "search"} == {C.RIGHT}
        for r in mir:
            s = src[r["mirror_of"]]
            assert r["hint_kind"] == s["hint_kind"] and r["hint"]["kind"] == s["hint"]["kind"]
            assert r["phrase"] == want[s["phrase"]], (s["phrase"], r["phrase"])
            assert r["prompt"] == C.build_prompt(r["phrase"], r["object"])
            assert r["label"]["dpsi_rad"] == -s["label"]["dpsi_rad"]
            for k in ("bearing_deg", "seen_rel_deg"):
                if s["hint"][k] is None:
                    assert r["hint"][k] is None
                else:
                    assert r["hint"][k] == -s["hint"][k], (k, s["hint"], r["hint"])
            assert r["hint"]["since_seen_s"] == s["hint"]["since_seen_s"]
            assert r["close_rule"] == s["close_rule"]
        sea = [r for r in mir if r["hint_kind"] == "search"]
        assert sea and {r["phrase"] for r in sea} == {C.LEFT}


# --------------------------------------------------------------------------- #
# B. The stats, and the label rule on perturbed recordings
# --------------------------------------------------------------------------- #
def test_the_stats_count_each_kind_in_all_per_split_and_per_run():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        a = _lf_run(td, "runA")
        h = _lf_run(td, "runH")
        st = B.build_dataset([a, h], td / "ds", horizon_s=1.0, stride_s=0.5, images=False,
                             hint_modes=ALL, holdout=["runH"], mirror=True)
        rows = _rows(td / "ds" / "samples.jsonl")
        real = [r for r in rows if not r["mirrored"]]
        hs = st["hints"]
        assert hs["modes"] == list(ALL) and (hs["fresh_s"], hs["memory_s"], hs["search_s"]) == \
            (1.0, 6.0, 20.0), hs
        assert st["args"]["hint_modes"] == list(ALL) and st["args"]["memory_s"] == 6.0
        assert hs["by_kind_real"] == dict(Counter(r["hint_kind"] for r in real))
        assert hs["by_kind"] == dict(Counter(r["hint_kind"] for r in rows))
        for split in ("train", "holdout"):
            assert hs["by_split"][split] == dict(Counter(r["hint_kind"] for r in rows
                                                         if r["split"] == split)), split
        for tag in ("runA", "runH"):
            assert st["runs"][tag]["hint_kinds"] == dict(Counter(
                r["hint_kind"] for r in real if r["run"] == tag)), tag
        for kind in ALL:
            sel = [r for r in real if r["hint_kind"] == kind]
            lab = hs["label_by_kind"][kind]
            assert lab["n"] == len(sel) > 0
            assert lab["fwd_m"]["n"] == len(sel) and lab["dpsi_rad"]["n"] == len(sel)
            assert sum(lab["abs_dpsi_hist"].values()) == len(sel)
            assert hs["phrases_by_kind"][kind] == dict(Counter(r["phrase"] for r in sel))
        assert hs["box_state"]["memory"] == dict(Counter(
            r["hint"]["box"] for r in real if r["hint_kind"] == "memory"))
        assert set(hs["box_state"]["memory"]) == {"stale", "none"}
        assert st["dropped"]["no_hint"] > 0
        assert not any("memory and search hints are not represented" in n for n in st["notes"])


def test_the_label_rule_drops_a_memory_sample_inside_a_vla_window_and_keeps_the_recovery():
    """DAgger (vla_dagger): a VLA-flown window while the car is out of view;
    memory samples whose horizon overlaps it are dropped "perturbed", those
    after it - the expert's recovery from where the model led it - kept and
    counted per kind."""
    B, TF = _B(), _TF()

    def box(k, t, yaw_at):
        return _box(TF._det_at_bearing(30.0), t - 0.05, yaw_at, k) if k * DT < 2.0 - 1e-9 \
            else None

    def vla(k):
        return {"source": "vla", "active": 40 <= k < 60, "window": 40 <= k < 60}

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = TF._write_run(td, "runD", TF._straight(speed=2.0), seconds=14.0, box=box,
                            perturb=vla)
        st = B.build_dataset([run], td / "ds", horizon_s=1.0, stride_s=0.0, images=False,
                             hint_modes=ALL)
        rows = {r["i"]: r for r in _rows(td / "ds" / "samples.jsonl")}
        # active 4.0 .. 5.9, clean from 6.0, margin 0.3: t = 3.1 .. 6.2 dropped (32)
        assert st["dropped"]["perturbed"] == 32, st["dropped"]
        assert not any(31 <= k <= 62 for k in rows)
        after = [r for k, r in rows.items() if 63 <= k <= 78]
        assert len(after) == 16 and all(r["hint_kind"] == "memory" for r in after)
        for r in after:
            assert math.isclose(r["perturb"]["since_end_s"], r["i"] * DT - 6.0, abs_tol=1e-6)
        pp = st["hints"]["post_perturb_3s_by_kind"]
        assert pp == {"memory": 16, "search": 12}, pp
        assert sum(pp.values()) == st["perturb"]["post_perturb_3s"]


# --------------------------------------------------------------------------- #
# E. The default, the CLI, the trainer and the evaluation tool
# --------------------------------------------------------------------------- #
OLD_SAMPLE_KEYS = {"id", "run", "split", "i", "t_capture", "t_mission", "pose", "image",
                   "mosaic_sha256", "front_jpg_sha256", "down_jpg_sha256", "bottom_tile",
                   "phrase", "hint_source", "object", "prompt", "prompt_sha256", "bins",
                   "action_text", "label", "label_raw", "clipped", "step_decoded",
                   "rotate_only", "horizon_s", "box", "expert", "analysis", "perturb",
                   "mirrored"}
NEW_SAMPLE_KEYS = {"hint_kind", "hint", "close_rule"}


def test_the_default_build_is_fresh_only_with_the_old_close_rule_and_the_same_samples():
    B, TF = _B(), _TF()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _lf_run(td)
        st = B.build_dataset([run], td / "d", horizon_s=1.0, stride_s=0.0, images=False)
        st2 = B.build_dataset([run], td / "e", horizon_s=1.0, stride_s=0.0, images=False,
                              hint_modes=("fresh",), close_bottom_frac=0.92,
                              close_width_frac=0.25)
        a, b = _rows(td / "d" / "samples.jsonl"), _rows(td / "e" / "samples.jsonl")
        assert a == b and st["dropped"] == st2["dropped"]
        assert st["args"]["hint_modes"] == ["fresh"] and st["args"]["memory_s"] is None
        assert st["args"]["search_s"] is None and st["hints"]["memory_s"] is None
        assert st["args"]["close_bottom_frac"] == 0.92 and st["args"]["close_width_frac"] == 0.25
        assert any("memory and search hints are not represented" in n for n in st["notes"])
        # exactly the captures with a fresh, non-close box: detector_phrase's
        br, close = B.make_bridge(), B.default_close_rule()
        want = []
        for c in _rows(run / "captures.jsonl"):
            phrase, why, _ = B.detector_phrase(c, br, close=close)
            if why is None and c["i"] * DT <= 39.0 + 1e-9:
                want.append((c["i"], phrase))
        assert [(r["i"], r["phrase"]) for r in a] == want
        for r in a:
            assert set(r) == OLD_SAMPLE_KEYS | NEW_SAMPLE_KEYS, set(r) ^ (OLD_SAMPLE_KEYS
                                                                          | NEW_SAMPLE_KEYS)
            assert r["hint_kind"] == "fresh" and r["hint"]["box"] == "fresh"
            assert set(r["box"]) == {"det", "age_s", "bearing_deg", "yaw_correction_deg",
                                     "seq", "tier"}
            assert r["close_rule"] == {"bottom_frac": 0.92, "width_frac": 0.25}
        # the drop reasons are today's
        assert set(st["dropped"]) <= set(B.DROP_ORDER) - {"no_hint", "hint_kind"}


def test_the_builder_defaults_are_vla_flys_follow_defaults():
    B = _B()
    import vla_fly
    a = vla_fly.parse_cli(vla_fly.build_arg_parser(), ["--mission", "follow"])
    assert (B.DEFAULT_MEMORY_S, B.DEFAULT_SEARCH_S) == (a.follow_memory_s, a.follow_search_s)
    assert B.DEFAULT_FRESH_S == a.det_fresh_s
    assert (B.DEFAULT_CLOSE_BOTTOM_FRAC, B.DEFAULT_CLOSE_WIDTH_FRAC) == \
        (a.close_bottom_frac, a.close_width_frac)
    assert B.default_close_rule() == _VM().BoxClose(0.92, 0.25)
    assert tuple(B.HINT_MODES) == ALL and tuple(B.DEFAULT_HINT_MODES) == ("fresh",)


def test_one_rule_detectorhint_and_the_builder_both_call_follow_hint_step():
    """Against drift: the decision lives in follow_hint_step only - the
    flight's DetectorHint and the builder call it, and neither carries the
    memory or search arithmetic itself."""
    B, VM = _B(), _VM()
    call = inspect.getsource(VM.DetectorHint.__call__)
    assert "follow_hint_step(" in call and "math.pi" not in call
    assert not hasattr(VM.DetectorHint, "_remembered")
    src = inspect.getsource(B)
    assert "VM.follow_hint_step(" in src and "math.pi / 2" not in src
    rule = inspect.getsource(VM.follow_hint_step)
    assert "math.pi / 2" in rule and "memory_s + search_s" in rule


def test_the_cli_takes_the_hint_and_close_flags_and_refuses_a_bad_mode():
    B = _B()
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        run = _lf_run(td)
        rc = B.main([str(run), "--out", str(td / "ds"), "--no-images", "--stride-s", "0.25",
                     "--hint-modes", "fresh,memory,search", "--memory-s", "6", "--search-s", "20",
                     "--close-bottom-frac", "1.0", "--close-width-frac", "0.25"])
        assert rc == 0
        st = json.loads((td / "ds" / "stats.json").read_text(encoding="utf-8"))
        assert st["args"]["hint_modes"] == list(ALL) and st["args"]["close_bottom_frac"] == 1.0
        assert set(st["hints"]["by_kind_real"]) == set(ALL)
        assert B.main([str(run), "--out", str(td / "bad"), "--no-images",
                       "--hint-modes", "fresh,lost"]) == 2
        assert B.main([str(run), "--out", str(td / "bad2"), "--no-images",
                       "--memory-s", "6"]) == 2


def test_the_trainer_reads_memory_and_search_samples_and_the_eval_splits_by_kind():
    B = _B()
    import finetune_aerialvla_follow as FT
    import eval_vla_adapter as EV
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        a = _lf_run(td, "runA")
        h = _lf_run(td, "runH")
        B.build_dataset([a, h], td / "ds", horizon_s=1.0, stride_s=0.5, images=False,
                        hint_modes=ALL, holdout=["runH"], mirror=True)
        rows = _rows(td / "ds" / "samples.jsonl")
        smps = []
        for i, r in enumerate(rows):
            r = dict(r, image="images/placeholder.png")        # images=False leaves it None
            s = FT.check_sample(r, td / "ds", i)
            assert s.prompt == r["prompt"] and s.target == r["action_text"]
            assert s.extra["hint_kind"] == r["hint_kind"], r["id"]
            smps.append(s)
        held = [s for s in smps if s.split == "holdout"]
        assert {s.extra["hint_kind"] for s in held} == set(ALL)
        # a scripted model right on every memory sample and wrong elsewhere
        outs = [{"idx": s.idx, "run": s.run, "phrase": s.phrase, "target": s.target,
                 "hint_kind": s.extra["hint_kind"],
                 "raw_text": (s.target if s.extra["hint_kind"] == "memory" else "00 00 00")
                 + C.EOS_TEXT} for s in held]
        by = EV.metrics_by_hint_kind(outs)
        assert set(by) == set(ALL)
        assert by["memory"]["exact_bin_rate"]["all"] == 1.0
        assert by["memory"]["n"] == sum(1 for s in held if s.extra["hint_kind"] == "memory")
        assert sum(m["n"] for m in by.values()) == len(held)
        # a dataset from before hint_kind holds fresh hints only
        old = [dict(o) for o in outs]
        for o in old:
            o.pop("hint_kind")
        assert set(EV.metrics_by_hint_kind(old)) == {"fresh"}


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
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
