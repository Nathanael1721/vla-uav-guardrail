"""tools/prefix_eval.py: the WP2-09 offline replay, driven by stand-in models.

Run either way:
    pytest tests/test_prefix_eval.py -v
    python tests/test_prefix_eval.py

WHY THIS FILE EXISTS

The replay answers one question: does the CSP in the prompt change how often
the Shield has to repair what the model proposes, on the same frames. A tool
like that can print a confident number in quiet ways, and each test below is
aimed at one of them:

  * a model that ignores its prompt must give csp - off = 0 exactly (a tool that
    "finds" an effect there is measuring itself);
  * a model that reacts to ANY prefix must show the same effect on the neutral
    arm as on the csp arm, so the report's null exposes it instead of crediting
    the rules;
  * a model that does read the rules must show csp < off while neutral = off;
  * the replay must notice when it is not the flight's model (the logged
    prompt's arm must reproduce the logged raw action);
  * refused ticks and frames that were not kept are counted, not dropped;
  * a run flown under another policy is refused, not scored;
  * the neutral prefix is rule-free and length-matched to the CSP;
  * `--dry-run` never loads a model;
  * the replay takes from the log what the flight flew with (review,
    6 Oct, three mutants that survived the first version of this file): the
    heading each frame was captured at (forward is East at yaw 90 - the
    defect of docs/FINDING-forward-flew-north.md), the step's action scale,
    and a fresh Shield for every frame and arm;
  * the CSP is compiled at the flight's own token budget, and the report
    says whether it is the CSP the flight logged.

No test loads OpenVLA, torch or AirSim. The real model needs a GPU run on
recorded frames, which has not been made yet.
"""
import contextlib
import io
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "demo"))

import prefix_eval as PE                                           # noqa: E402
import real_vla_demo as rv                                         # noqa: E402
from guardrail import load_policy                                  # noqa: E402
from guardrail.compiler import ConstraintCompiler                  # noqa: E402

URBAN = ROOT / "policies" / "urban_demo_policy.yaml"   # NFZ x,y 7..23, band 15-25 m
PED = ROOT / "policies" / "sitl_pedestrian.yaml"
INSTR = "fly forward and avoid restricted areas"
FAST = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]     # 3 m/s forward at scale 150
AWAY = [0.0, -0.02, 0.0, 0.0, 0.0, 0.0, 1.0]    # 3 m/s left (west at yaw 0)
# Four frames facing North just south of the zone (forward runs into it within
# the 3 s lookahead) and two far away (forward is harmless there).
STATES = [dict(x=4.0, y=15.0, up=20.0, yaw_deg=0.0)] * 4 + \
         [dict(x=-60.0, y=-60.0, up=20.0, yaw_deg=0.0)] * 2


class Deaf:
    """Ignores the prompt entirely."""
    name = "stand-in: deaf"

    def __call__(self, frame, prompt):
        return list(FAST)


class PrefixSensitive:
    """Reacts to any text ahead of the question, rules or not."""
    name = "stand-in: any prefix"

    def __call__(self, frame, prompt):
        # The off prompt opens with the question; any prefix pushes it back.
        return list(FAST) if prompt.startswith("In: What action") else list(AWAY)


class RuleReader:
    """Steers away only when told a zone is forbidden."""
    name = "stand-in: reads rules"

    def __call__(self, frame, prompt):
        return list(AWAY) if "Never enter zone" in prompt else list(FAST)


class NotTheFlightModel(RuleReader):
    """Reads rules, but its off-prompt output is not what the flight logged."""
    name = "stand-in: different weights"

    def __call__(self, frame, prompt):
        out = super().__call__(frame, prompt)
        return [v + 0.001 for v in out]


def _run(policy=URBAN, logged_raw=FAST, extra=(), states=None, scale=150.0,
         meta_extra=None, prompt=None):
    """A run folder as real_vla_demo.py writes it (default: --csp off)."""
    d = Path(tempfile.mkdtemp()) / "real_vla_fake"
    (d / "frames" / "r1").mkdir(parents=True)
    pol = load_policy(policy)
    mission = ConstraintCompiler(pol).parse_command(rv.DEFAULT_MISSION)
    meta = {"run_id": "r1", "instruction": INSTR, "policy_path": str(policy),
            "mission": mission.model_dump(), "scale": scale}
    meta.update(meta_extra or {})
    (d / "vla_run.json").write_text(json.dumps(meta), encoding="utf-8")
    prompt = prompt or rv.build_openvla_prompt(INSTR)
    lines = []
    for i, st in enumerate(states or STATES, start=1):
        ff = f"frames/r1/{i:06d}.png"
        (d / ff).write_bytes(b"\x89PNG fake frame %d" % i)
        lines.append({"inference": i, "frame_index": i, "frame_file": ff, "state": st,
                      "raw_action": list(logged_raw), "prompt": prompt, "scale": scale,
                      "policy_hash": pol.policy_hash})
    lines.extend(extra)
    (d / "vla_steps.jsonl").write_text("\n".join(json.dumps(x) for x in lines) + "\n",
                                       encoding="utf-8")
    return d


def _eval(model, **kw):
    return PE.evaluate(PE.load_run(_run(**kw)), model)


def test_a_model_that_ignores_the_prompt_shows_no_effect():
    rep = _eval(Deaf())
    a = rep["arms"]
    assert a["off"]["repairs"] == 4 and a["off"]["frames"] == 6, a["off"]
    assert rep["repair_rate_csp_minus_off"] == 0.0, rep["repair_rate_csp_minus_off"]
    assert rep["repair_rate_neutral_minus_off"] == 0.0
    assert a["csp"]["mean_action_change_vs_off_mps"] == 0.0
    assert a["off"]["repair_mps_mean"] > 0, "a repair of size 0 is not a repair"


def test_a_rule_reading_model_shows_the_effect_and_the_null_stays_flat():
    rep = _eval(RuleReader())
    a = rep["arms"]
    assert (a["off"]["repairs"], a["csp"]["repairs"], a["neutral"]["repairs"]) == (4, 0, 4), a
    assert rep["repair_rate_csp_minus_off"] < 0
    assert rep["repair_rate_neutral_minus_off"] == 0.0
    assert a["csp"]["mean_action_change_vs_off_mps"] > 0
    assert a["neutral"]["mean_action_change_vs_off_mps"] == 0.0
    # the CSP's own action sets: forward into the zone breaks them, away does not
    assert a["off"]["csp_action_set_violation_rate"] > a["csp"]["csp_action_set_violation_rate"]


def test_a_model_moved_by_any_prefix_is_exposed_by_the_neutral_arm():
    rep = _eval(PrefixSensitive())
    assert rep["repair_rate_csp_minus_off"] < 0, rep["arms"]
    assert rep["repair_rate_neutral_minus_off"] == rep["repair_rate_csp_minus_off"], \
        "the null arm must show the same 'effect', so the rules get no credit"


def test_the_replay_must_reproduce_the_flight_or_say_it_does_not():
    ok = _eval(RuleReader())["reproduces_flight"]
    assert ok["arm"] == "off" and (ok["frames_reproduced"], ok["frames_compared"]) == (6, 6), ok
    bad = _eval(NotTheFlightModel())["reproduces_flight"]
    assert bad["frames_compared"] == 6 and bad["frames_reproduced"] == 0, bad
    assert bad["fraction"] == 0.0


def test_unreplayable_steps_are_counted_with_their_reason():
    extra = [{"inference": None, "frame_index": 7, "frame_file": None,
              "state": STATES[0], "refused": {"kind": "csp_over_budget"}},
             {"inference": 7, "frame_index": 8, "frame_file": None, "state": STATES[0],
              "raw_action": FAST},
             {"inference": 8, "frame_index": 9, "frame_file": "frames/r1/gone.png",
              "state": STATES[0], "raw_action": FAST}]
    run = PE.load_run(_run(extra=extra))
    assert len(run["steps"]) == 6 and len(run["skipped"]) == 3, run["skipped"]
    why = " | ".join(s["why"] for s in run["skipped"])
    assert "refused" in why and "--no-frames" in why and "missing" in why, why
    rep = PE.evaluate(run, Deaf())
    assert (rep["frames_replayed"], rep["frames_skipped"]) == (6, 3)


def test_a_run_flown_under_another_policy_is_refused():
    run = PE.load_run(_run())
    try:
        PE.evaluate(run, Deaf(), policy=load_policy(PED))
    except ValueError as e:
        assert "flew policy" in str(e), e
    else:
        raise AssertionError("a replay was scored against rules the flight never had")


def test_the_neutral_prefix_is_rule_free_and_length_matched():
    pol = load_policy(URBAN)
    meta = json.loads((_run() / "vla_run.json").read_text(encoding="utf-8"))
    pr = PE.build_prompts(meta, pol)
    n_csp, n_neu = pr["csp"]["prefix_tokens"], pr["neutral"]["prefix_tokens"]
    sent = rv.count_tokens(PE.NEUTRAL_SENTENCE, None, special_tokens=False)["n"]
    assert n_csp <= n_neu < n_csp + sent + 1, (n_csp, n_neu, sent)
    neu = pr["neutral"]["prompt"]
    for word in ("never", "zone", "stay", "speed", "altitude", "m/s", "keep"):
        assert word not in neu.lower(), f"neutral prefix carries {word!r}"
    assert pr["off"]["prompt"] == rv.build_openvla_prompt(INSTR)
    # The CSP is the one a --csp on flight would log, by content.
    csp = rv.compile_flight_csp(pol, ConstraintCompiler(pol).parse_command(rv.DEFAULT_MISSION))
    assert pr["_csp"]["csp_content_hash"] == rv.prompt_record(INSTR, pol, csp)["csp_content_hash"]
    assert pr["csp"]["prompt"] == rv.build_openvla_prompt(INSTR, csp.natural_language_prompt)


def test_dry_run_builds_prompts_without_a_model():
    class Boom:
        def __init__(self):
            raise AssertionError("--dry-run loaded the model")
    real = PE.OpenVLAReplayModel
    PE.OpenVLAReplayModel = Boom
    try:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = PE.main([str(_run()), "--dry-run"])
    finally:
        PE.OpenVLAReplayModel = real
    shown = json.loads(out.getvalue())
    assert code == 0 and shown["frames_replayable"] == 6, shown
    assert set(shown["prompts"]) >= {"off", "csp", "neutral", "_csp"}


def test_the_report_names_what_produced_it():
    rep = _eval(RuleReader())
    for k in ("run_dir", "run_id", "model", "policy_hash", "instruction", "csp", "generated_utc"):
        assert rep.get(k), k
    lines = PE.summary_lines(rep)
    assert any("neutral - off (the null)" in ln for ln in lines), lines
    json.dumps(rep)      # serialisable as written

def test_forward_is_rotated_by_the_heading_the_frame_was_captured_at():
    """Facing East (yaw 90) at (15, 3), OpenVLA's 'forward' runs East into the
    zone (y 7..23, margin 1 m); read as North it passes beside it. A replay
    that drops the heading scores the frame as clean - the defect of
    docs/FINDING-forward-flew-north.md, now in the replay."""
    east = [dict(x=15.0, y=3.0, up=20.0, yaw_deg=90.0)]
    rep = _eval(Deaf(), states=east)
    assert rep["arms"]["off"]["repairs"] == 1, rep["arms"]["off"]
    north = [dict(x=15.0, y=3.0, up=20.0, yaw_deg=0.0)]
    assert _eval(Deaf(), states=north)["arms"]["off"]["repairs"] == 0, \
        "the same frame facing North must be clean, or the probe proves nothing"


def test_the_action_scale_is_the_one_the_step_flew_with():
    """At scale 30 the 'fast' output is 0.6 m/s and stays short of the zone in
    the 3 s lookahead; at the default 150 it is 3 m/s and enters it."""
    slow = _eval(Deaf(), scale=30.0)
    assert slow["arms"]["off"]["repairs"] == 0, slow["arms"]["off"]
    assert _eval(Deaf(), scale=150.0)["arms"]["off"]["repairs"] == 4
    # The step's own record wins over the run's: steps flown at 30 under a
    # vla_run.json that says 150 (or a run file written before the field
    # existed, which falls back to 150) are replayed at 30.
    mixed = _eval(Deaf(), scale=30.0, meta_extra={"scale": 150.0})
    assert mixed["arms"]["off"]["repairs"] == 0, mixed["arms"]["off"]


def test_every_frame_and_arm_gets_a_fresh_shield():
    """The docstring promises open-loop judging: no Shield state carries from
    one frame to the next. A Shield that judged two frames fails here."""
    made = []

    class OneShotShield(PE.Shield):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self._judged = 0
            made.append(self)

        def filter(self, *a, **k):
            self._judged += 1
            assert self._judged == 1, "one Shield judged more than one frame"
            return super().filter(*a, **k)

    real = PE.Shield
    PE.Shield = OneShotShield
    try:
        rep = _eval(Deaf())
    finally:
        PE.Shield = real
    assert len(made) == rep["frames_replayed"] * len(PE.ARMS), (len(made), rep["frames_replayed"])


def test_the_csp_is_compiled_at_the_flights_budget_and_checked_against_it():
    """Review, 6 Oct: the replay always used the default budget, so a --csp on
    flight with another budget got a csp arm that was not the flown prompt,
    and nothing said so."""
    pol = load_policy(URBAN)
    mission = ConstraintCompiler(pol).parse_command(rv.DEFAULT_MISSION)
    budget = 200
    csp = rv.compile_flight_csp(pol, mission, budget_tokens=budget)
    rec = rv.prompt_record(INSTR, pol, csp)
    flown = rv.build_openvla_prompt(INSTR, csp.natural_language_prompt)
    run = _run(meta_extra={"csp_budget_tokens": budget, "prompt": rec}, prompt=flown)
    rep = PE.evaluate(PE.load_run(run), RuleReader())
    assert rep["csp"]["budget_tokens"] == budget, rep["csp"]
    assert "vla_run.json" in rep["csp"]["budget_source"], rep["csp"]
    assert rep["csp_matches_flight"] is True, (rep["flight_csp_content_hash"], rep["csp"])
    assert rep["reproduces_flight"]["arm"] == "csp", rep["reproduces_flight"]
    # an explicit budget is recorded as such; a CSP the flight did not log
    # cannot be matched, and says None rather than True
    rep = PE.evaluate(PE.load_run(run), RuleReader(), budget=256)
    assert rep["csp"]["budget_source"] == "--budget"
    off = PE.evaluate(PE.load_run(_run()), RuleReader())
    assert off["csp_matches_flight"] is None and off["flight_csp_content_hash"] is None
    # a logged hash that differs is reported as a mismatch
    bad = dict(rec, csp_content_hash="sha256:" + "0" * 64)
    rep = PE.evaluate(PE.load_run(_run(meta_extra={"prompt": bad}, prompt=flown)), RuleReader())
    assert rep["csp_matches_flight"] is False


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
