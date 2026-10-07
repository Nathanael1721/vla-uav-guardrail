"""demo/real_vla_demo.py — the CSP in OpenVLA's prompt, and the missing rotation.

Run either way:
    pytest tests/test_real_vla_prompt.py -v
    python tests/test_real_vla_prompt.py

WHY THIS FILE EXISTS

Audit card WP2-01: "The CSP never reaches any VLA". The compiler built a
Constraint Summary Pack, and the only real VLA in the repo read the typed
instruction and nothing else. `--csp on` now puts the CSP in the prompt. A flag
like that can be wrong in quiet ways, and each test below is aimed at one:

  - "off" must be the OLD prompt byte for byte, AND the default, so
    scripts/demo_real_vla.ps1 (which passes no flag) keeps sending what it
    sent. That does not make old runs the control arm of an A/B: they flew the
    yaw-ignored mapping, so the off arm is flown again with this code;
  - "on" must carry the grant's CSP - `compile_csp`'s filtered, risk-graded,
    budgeted text - not the unfiltered by-type sentence list `build_prompt`
    renders, which reads the same rules in a different order;
  - the logged CSP hash must be the compiler's own `csp_hash`, and must match
    the CSP file written beside the run, or no artefact can be checked;
  - one CSP per policy generation: a hot-applied rule reaches the next
    inference, and a rebuild that fails never serves the old prompt;
  - every refusal (P0 over the CSP budget, prompt over OpenVLA's text budget,
    no sentences) happens BEFORE the 7B model loads, and mid-flight the worker
    holds zero instead of feeding a prompt that failed;
  - a token count without OpenVLA's tokenizer must SAY it is an estimate, and
    a budget it cannot prove must read "unknown", never "fits"; an exact count
    must name the tokenizer.json it came from;
  - with --csp off the CSP fields are None, never 0 (silence reads as success).

The rotation tests are the regression for the body-to-world bug. On the old
mapping (yaw ignored) `test_yaw_90_forward_flies_east` and
`test_yaw_90_right_flies_south` FAILED: facing East, "forward" flew North
(vx=1.5 vy=0.0). That run is recorded in the changelog entry for this unit.

No test here loads OpenVLA's weights, torch, or AirSim. The real-tokenizer test
reads tokenizer.json through the `tokenizers` package and checks that torch
stayed out. The worker tests run the real `_worker` loop, and the flight tests
run the real `main()` - preflight, backend `__init__`, worker thread, control
loop, Shield, audit, run record - against stand-in airsim/cv2/PIL modules, with
only `OpenVLABackend._load` (the GPU part) replaced. So "the control loop's
State carries the heading", "the CSP file and the frames are written" and "a
prompt that fails after the model loads stops the flight" are checked on the
code that flies, not on a copy or a helper.
"""
import contextlib
import hashlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import threading
import time as _time
import types
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

_HEAVY = ("torch", "transformers", "airsim", "cv2", "bitsandbytes")
_preloaded = {m for m in _HEAVY if m in sys.modules}
import real_vla_demo as rv                                         # noqa: E402
_loaded_by_import = {m for m in _HEAVY if m in sys.modules} - _preloaded

from guardrail import Shield, load_policy                          # noqa: E402
from guardrail.compiler import ConstraintCompiler, Mission         # noqa: E402
from guardrail.csp import CSP, CSPBudgetExceeded                   # noqa: E402
from guardrail.csp import csp_hash as compiler_csp_hash            # noqa: E402
from guardrail.models import (AltitudeEnvelope, Corridor,          # noqa: E402
                              KinematicEnvelope, Policy, PolygonFence,
                              State, SubjectStandoff)

URBAN = ROOT / "policies" / "urban_demo_policy.yaml"       # the demo's default
PED = ROOT / "policies" / "sitl_pedestrian.yaml"
CORRIDOR = ROOT / "policies" / "corridor_survey.yaml"
INSTR = "fly forward and avoid restricted areas"
QUESTION = "What action should the robot take to"
OLD_PROMPT = f"In: What action should the robot take to {INSTR}?\nOut:"   # 2026-07-14

FWD = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]     # 1.5 m/s forward at scale 150
RIGHT = [0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 1.0]   # 1.5 m/s right at scale 150
HUGE = 10 ** 6                                 # a CSP budget nothing reaches


class Skipped(Exception):
    """Raised (standalone) when a test cannot run here; counted, never a pass."""


def _skip(msg):
    if "pytest" in sys.modules:
        import pytest
        pytest.skip(msg)
    raise Skipped(msg)


def _mission():
    return Mission(task_text="fly to (40, 40)", target_x=40.0, target_y=40.0,
                   cruise_alt_m=20.0, speed_pref_mps=6.0)


def _fp(policy, csp_on=True, tokenizer=None, **kw):
    return rv.FlightPrompt(INSTR, policy, _mission(), csp_on, tokenizer, **kw)


def _rec(policy, csp_on=True, tokenizer=None, **kw):
    return _fp(policy, csp_on, tokenizer, **kw).current()


class CharTokenizer:
    """One token per character (+1 BOS): a count no word splitter produces, so
    a test can tell the exact path from the estimate, and it records whether
    special tokens were asked for."""

    def __init__(self):
        self.special = []

    def __call__(self, text, add_special_tokens=True):
        self.special.append(add_special_tokens)
        return {"input_ids": [0] * (len(text) + (1 if add_special_tokens else 0))}


class WordTokenizer:
    """One token per whitespace word (+1 BOS): an exact count small enough to
    fit the text budget where the estimate's byte bound cannot prove a fit."""

    def __call__(self, text, add_special_tokens=True):
        return {"input_ids": [0] * (len(text.split()) + (1 if add_special_tokens else 0))}


def _many_fences(n):
    raw = yaml.safe_load(URBAN.read_text(encoding="utf-8"))
    fence = next(c for c in raw["constraints"] if c["type"] == "polygon_fence")
    raw["constraints"] = [dict(fence, id=f"nfz-{i:03d}") for i in range(n)]
    return Policy.model_validate(raw)


def _nothing_in_region():
    """A policy whose only rule is a keep-out zone 9 km away: the CSP filters it
    out of the mission region, so the CSP has no sentences. load_policy refuses
    a policy with NO rules since 2026-10-06 (the DSL lint), so a rule-less
    policy file can no longer reach the prompt builder; this one can, and it
    still exercises the "no sentences" refusal."""
    return Policy.model_validate({"policy_id": "nothing-in-region", "constraints": [
        {"id": "nfz-far", "type": "polygon_fence", "vertices": [
            {"x": 9000, "y": 9000}, {"x": 9010, "y": 9000},
            {"x": 9010, "y": 9010}, {"x": 9000, "y": 9010}]}]})


def _policy_file(pol):
    p = Path(tempfile.mkdtemp()) / f"{pol.policy_id}.yaml"
    p.write_text(yaml.safe_dump(pol.model_dump(mode="json")), encoding="utf-8")
    return p


def _expect(exc, fn, *a, **k):
    try:
        fn(*a, **k)
    except exc as e:
        return e
    raise AssertionError(f"{getattr(fn, '__name__', fn)} did not raise {exc.__name__}")


# --------------------------------------------------------------------------- #
# The prompt
# --------------------------------------------------------------------------- #

def test_csp_off_is_the_old_prompt_byte_for_byte():
    """The literal the 2026-07-14 runs sent, spelled out here rather than read
    from the module, so a template edit cannot pass itself."""
    assert rv.build_openvla_prompt(INSTR) == OLD_PROMPT
    rec = _rec(load_policy(URBAN), csp_on=False)
    assert rec["prompt"] == OLD_PROMPT, rec["prompt"]


def test_csp_off_fields_are_none_not_zero():
    fp = _fp(load_policy(URBAN), csp_on=False)
    rec = fp.current()
    assert rec["csp"] == "off" and fp.csp is None and fp.n_compiles == 0
    for k in ("csp_text", "csp_tokens", "csp_overhead_tokens", "csp_hash",
              "csp_content_hash",
              "csp_issued_at", "csp_file", "csp_tokens_used", "csp_budget_tokens",
              "csp_token_counter", "csp_time_filter", "csp_decisions"):
        assert rec[k] is None, f"{k}={rec[k]!r}: 'no CSP' must not look like a zero"
    assert rec["policy_hash"] == load_policy(URBAN).policy_hash


def test_csp_on_injects_compile_csp_not_the_unfiltered_sentence_list():
    """REVIEW FIX. The first version injected build_prompt's
    natural_language_prompt: every rule, by type, no filter, no risk order, no
    budget. The grant's CSP is compile_csp's. On all 29 shipped policies the two
    texts differ (P0 first, by risk), so reverting fails here."""
    for path in (URBAN, PED, CORRIDOR):
        pol = load_policy(path)
        want = ConstraintCompiler(pol).compile_csp(_mission(), now=None)
        old = yaml.safe_load(ConstraintCompiler(pol).build_prompt(_mission()))
        assert want.natural_language_prompt != old["natural_language_prompt"], path.name
        fp = _fp(pol)
        rec = fp.current()
        assert rec["csp_text"] == want.natural_language_prompt, (path.name, rec["csp_text"])
        assert fp.csp.selection.token_counter == want.selection.token_counter
        assert rec["csp_tokens_used"] == want.selection.tokens_used
        assert rec["csp_budget_tokens"] == rv.DEFAULT_BUDGET_TOKENS
        assert rec["csp_time_filter"] is False, "the demo's Shield has no clock"
        assert rec["csp_decisions"] == {r.id: r.decision for r in want.selection.rules}


def test_csp_on_puts_the_rules_before_the_question():
    rec = _rec(load_policy(URBAN))
    p = rec["prompt"]
    assert p.startswith("In: ") and p.endswith("?\nOut:"), repr(p)
    assert p.count(QUESTION) == 1 and p.count(INSTR) == 1, repr(p)
    q = p.index(QUESTION)
    # The CSP sits exactly between "In: " and the question, nothing else.
    assert p[len("In: "):q].strip() == rec["csp_text"], repr(p)
    assert rec["csp_text"] and q > len("In: ") + 20, repr(p)


def test_the_rule_text_is_the_loaded_policy():
    """Every rule of three different policies appears in words, checked against
    the policy objects' own parameters rather than a hard-coded sentence."""
    for path in (URBAN, PED, CORRIDOR):
        pol = load_policy(path)
        rec = _rec(pol)
        assert set(rec["csp_decisions"].values()) == {"kept"}, rec["csp_decisions"]
        csp, q = rec["csp_text"], rec["prompt"].index(QUESTION)
        need = []
        need += [f"Never enter zone '{f.id}'" for f in pol.by_type(PolygonFence)]
        need += [f"corridor '{c.id}'" for c in pol.by_type(Corridor)]
        for e in pol.by_type(AltitudeEnvelope):
            need.append(f"between {e.alt_min_m:g} m and {e.alt_max_m:g} m altitude")
        need += [f"{k.speed_max_mps:g} m/s" for k in pol.by_type(KinematicEnvelope)]
        need += [f"{s.min_range_m:g} m away" for s in pol.by_type(SubjectStandoff)]
        assert len(need) >= len(pol.constraints), (path.name, need)
        for phrase in need:
            assert phrase in csp, f"{path.name}: {phrase!r} missing from {csp!r}"
            assert rec["prompt"].index(phrase) < q, f"{phrase!r} after the question"
        assert rec["n_rules"] == len(pol.constraints)
    # One literal, so a template change that drops the stand-off is loud.
    assert "10 m away from any pedestrian" in _rec(load_policy(PED))["csp_text"]


def test_the_logged_csp_hash_is_the_compilers_and_matches_the_written_file():
    """REVIEW FIX. The first version defined its own `csp_hash` over the
    mission-free summary_pack, so no logged hash could ever match a CSP
    artefact. Now there is one definition, and the file beside the run hashes
    to exactly what every step logged."""
    assert rv.csp_hash is compiler_csp_hash, "a second csp_hash definition is back"
    assert not hasattr(rv, "_canonical")
    out = Path(tempfile.mkdtemp())
    pol = load_policy(URBAN)
    fp = _fp(pol)
    rec = fp.current()
    assert rec["csp_file"] is None, "nothing written before attach()"
    fp.attach(out)
    assert rec["csp_file"] == f"csp/csp-g0-{rec['csp_hash'][7:19]}.json", rec["csp_file"]
    on_disk = CSP.model_validate_json((out / rec["csp_file"]).read_text(encoding="utf-8"))
    assert compiler_csp_hash(on_disk) == rec["csp_hash"] == compiler_csp_hash(fp.csp)
    assert on_disk.natural_language_prompt == rec["csp_text"]
    assert on_disk.policy_hash == pol.policy_hash and on_disk.issued_at == rec["csp_issued_at"]
    assert rec["csp_hash"].startswith("sha256:") and len(rec["csp_hash"]) == 7 + 64
    other = _rec(load_policy(PED))
    assert other["csp_hash"] != rec["csp_hash"]
    assert other["prompt_sha256"] != rec["prompt_sha256"]


def test_the_csp_file_is_the_object_the_prompt_was_built_from():
    """REVIEW FIX. `_write_csp` must write the CSP the prompt came from, not a
    recompile. The test above cannot tell: issued_at has one-second resolution,
    so two compiles in the same second hash the same. Here every compile gets
    its own issued_at, so a file written from a recompile hashes to something
    no step logged - and the first assert proves the test can see that."""
    real = rv.compile_flight_csp
    n = [0]

    def stamping(*a, **k):
        n[0] += 1
        return real(*a, **k).model_copy(
            update={"issued_at": f"2026-10-06T00:{n[0] // 60:02d}:{n[0] % 60:02d}+00:00"})

    pol, out = load_policy(URBAN), Path(tempfile.mkdtemp())
    with _patched(compile_flight_csp=stamping):
        assert compiler_csp_hash(stamping(pol, _mission())) != \
            compiler_csp_hash(stamping(pol, _mission())), "the stamp does not change the hash"
        fp = _fp(pol)
        fp.attach(out, "csp/run-a")
        recs = [fp.current()]
        pol.generation += 1
        recs.append(fp.current())
    for rec in recs:
        on_disk = CSP.model_validate_json((out / rec["csp_file"]).read_text(encoding="utf-8"))
        assert compiler_csp_hash(on_disk) == rec["csp_hash"], \
            f"{rec['csp_file']} is not the CSP the prompt was built from"
        assert on_disk.issued_at == rec["csp_issued_at"]
        assert rec["csp_file"].startswith("csp/run-a/csp-g"), rec["csp_file"]
    assert fp.n_compiles == 2


def test_set_tokenizer_recounts_without_reissuing_the_csp():
    """REVIEW FIX. When the preflight had no tokenizer file, the processor's
    tokenizer arrives after the model loads. The recount must keep the CSP as
    issued (same hash, same issued_at, no recompile) and keep naming the file
    already written: the run record and every step take csp_file from here."""
    out = Path(tempfile.mkdtemp())
    fp = _fp(load_policy(URBAN), tokenizer=None)
    before = dict(fp.current())
    fp.attach(out, "csp/run-b")
    written = fp.current()["csp_file"]
    assert written == f"csp/run-b/csp-g0-{before['csp_hash'][7:19]}.json", written
    assert before["token_method"] == rv.TOKENS_ESTIMATE and before["tokenizer"] is None
    fp.set_tokenizer(CharTokenizer())
    after = fp.current()
    assert after["token_method"] == rv.TOKENS_EXACT
    assert after["prompt_tokens"] == len(after["prompt"]) + 1
    assert after["tokenizer"] == "CharTokenizer: source unidentified"
    assert after["csp_hash"] == before["csp_hash"]
    assert after["csp_issued_at"] == before["csp_issued_at"]
    assert after["csp_file"] == written and (out / written).exists(), after["csp_file"]
    assert fp.n_compiles == 1, "recounting recompiled the CSP"


def test_one_csp_per_generation():
    """compile_csp with no clock stamps issued_at from the wall clock, and the
    hash covers it: recompiling per inference would give each step a new hash
    for the same text. So the CSP is pinned per (policy_hash, generation)."""
    out = Path(tempfile.mkdtemp())
    pol = load_policy(URBAN)
    fp = _fp(pol)
    fp.attach(out)
    first = fp.current()
    assert fp.current() is first and fp.n_compiles == 1, "rebuilt with nothing changed"
    pol.generation += 1                          # what Shield.hot_apply also does
    second = fp.current()
    assert fp.n_compiles == 2 and second["policy_generation"] == 1 == fp.csp.generation
    assert second["csp_hash"] != first["csp_hash"]
    assert second["csp_file"].startswith("csp/csp-g1-") and (out / second["csp_file"]).exists()
    assert (out / first["csp_file"]).exists(), "the generation-0 CSP must stay on disk"


def test_a_hot_applied_rule_reaches_the_next_inference():
    """The real Shield.hot_apply (appends the fence, bumps the generation) on
    the policy the backend reads; the next prompt carries the new zone."""
    pol = load_policy(URBAN)
    b = _StandInBackend(_fp(pol))                         # no model load
    first = b.prompt()
    assert b.prompt() is first
    Shield(pol).hot_apply(pol.constraints[0].model_copy(update={"id": "nfz-hot"}))
    second = b.prompt()
    assert "Never enter zone 'nfz-hot'" in second["prompt"], second["prompt"]
    assert "nfz-hot" not in first["prompt"]
    assert second["csp_hash"] != first["csp_hash"]
    assert second["policy_hash"] == pol.policy_hash != first["policy_hash"]
    assert second["policy_generation"] == 1


def test_a_failed_rebuild_never_serves_the_old_prompt():
    pol = load_policy(URBAN)
    fp = _fp(pol)
    good = fp.current()
    pol.constraints[:] = _many_fences(300).constraints     # P0 now far over budget
    e = _expect(CSPBudgetExceeded, fp.current)
    assert len(e.p0_ids) == 300, "every P0 rule named, none dropped to fit"
    _expect(CSPBudgetExceeded, fp.current)                 # and again: no stale prompt
    rec, refusal = rv.check_prompt(fp)
    assert rec is None and refusal["kind"] == "csp_p0_over_budget", refusal
    assert fp.csp is None and good["csp_hash"]


def test_an_empty_csp_is_refused():
    for blank in ("", "   "):
        _expect(ValueError, rv.build_openvla_prompt, INSTR, blank)
    empty = Policy(policy_id="empty", constraints=[])
    _expect(ValueError, _rec, empty)
    assert rv.check_prompt(_fp(empty))[1]["kind"] == "no_prompt"
    # ...while --csp off with that policy is fine: there is nothing to inject.
    assert _rec(empty, csp_on=False)["csp"] == "off"


def test_a_csp_from_another_generation_is_refused():
    pol = load_policy(URBAN)
    csp = rv.compile_flight_csp(pol, _mission())
    pol.generation += 1
    _expect(ValueError, rv.prompt_record, INSTR, pol, csp)
    _expect(ValueError, rv.prompt_record, INSTR, load_policy(PED), csp)


# --------------------------------------------------------------------------- #
# Token counts and the budgets
# --------------------------------------------------------------------------- #

def test_the_tokenizer_is_used_when_there_is_one():
    tok = CharTokenizer()
    rec = _rec(load_policy(URBAN), tokenizer=tok)
    assert rec["token_method"] == rv.TOKENS_EXACT
    assert rec["prompt_tokens"] == len(rec["prompt"]) + 1          # + BOS
    assert rec["csp_tokens"] == len(rec["csp_text"])               # alone, no BOS
    off = len(rv.build_openvla_prompt(INSTR)) + 1
    assert rec["csp_overhead_tokens"] == rec["prompt_tokens"] - off
    assert True in tok.special and False in tok.special, tok.special
    assert rec["budget_used_frac"] == round(rec["prompt_tokens"] / rv.OPENVLA_TEXT_BUDGET, 4)
    assert rec["within_budget"] is True


def test_an_exact_count_names_the_tokenizer_file_it_came_from():
    """REVIEW FIX (CONTRIBUTING item 5). "openvla_tokenizer" alone did not say
    which tokenizer.json produced a count. The processor's HF tokenizer is
    named by the sha256 of the tokenizer.json in the folder it loaded from, in
    the compiler's own label format; a tokenizer with no traceable file says
    so; an estimate has no tokenizer (None), and the step lines carry the
    field so each one can be traced on its own."""
    d = Path(tempfile.mkdtemp())
    (d / "tokenizer.json").write_bytes(b'{"stand-in": 1}')

    class HFLike(CharTokenizer):
        name_or_path = str(d)

    want = "openvla-llama2-sp@" + hashlib.sha256(b'{"stand-in": 1}').hexdigest()[:12]
    assert rv.tokenizer_id(HFLike()) == f"{want} via HFLike"
    pol = load_policy(URBAN)
    rec = _rec(pol, tokenizer=HFLike())
    assert rec["tokenizer"] == f"{want} via HFLike" and rec["token_method"] == rv.TOKENS_EXACT
    assert rv.tokenizer_id(CharTokenizer()) == "CharTokenizer: source unidentified"
    assert rv.tokenizer_id(None) is None and _rec(pol)["tokenizer"] is None
    step = rv.step_record(1, 1, None, State(x=0.0, y=0.0, up=20.0), FWD, 150.0, rec)
    assert step["tokenizer"] == rec["tokenizer"], "the step line lost the tokenizer"


def test_without_a_tokenizer_the_count_says_estimate():
    rec = _rec(load_policy(URBAN))
    assert rec["token_method"] == rv.TOKENS_ESTIMATE
    assert "ESTIMATE" in rec["token_method"], "the label must be visible in the log"
    assert rec["prompt_tokens"] == len(rec["prompt"].split())
    assert rec["budget_used_frac"] is None, "a fraction of an estimate reads as measured"
    ub = rec["prompt_tokens_upper_bound"]
    assert ub == len(rec["prompt"].encode("utf-8")) + 2 and ub >= rec["prompt_tokens"]


def test_the_text_budget_arithmetic():
    """1784 = 2048 - 256 - 7 - 1. Only 2048 (llm_max_length) is in config.json;
    pin it to the checkpoint when the checkpoint is here."""
    assert rv.OPENVLA_TEXT_BUDGET == 2048 - 256 - 7 - 1 == 1784
    cfg = Path(rv.MODEL_ID) / "config.json"
    if cfg.exists():
        got = json.loads(cfg.read_text(encoding="utf-8"))
        assert got["llm_max_length"] == rv.OPENVLA_LLM_MAX_LENGTH, got["llm_max_length"]
        assert got["image_sizes"] == [224, 224] == list(rv.IMAGE_SIZE)


def test_the_run_record_names_the_resize_filter():
    """The worker resizes with Pillow's default filter; an offline replay that
    resizes differently feeds other pixels, so vla_run.json names it."""
    assert rv.preprocess_note("12.2.0")["resample"] == "BICUBIC"
    assert rv.preprocess_note("7.0.0")["resample"] == "BICUBIC"
    assert rv.preprocess_note("6.2.1")["resample"] == "NEAREST"
    assert rv.preprocess_note(None)["resample"] is None, "unknown must not read as a filter"
    assert rv.preprocess_note("12.2.0")["resize"] == [224, 224]


def test_the_text_budget_boundary_is_inclusive():
    pol = load_policy(URBAN)
    csp = rv.compile_flight_csp(pol, _mission())
    n = rv.prompt_record(INSTR, pol, csp, CharTokenizer())["prompt_tokens"]
    assert rv.prompt_record(INSTR, pol, csp, CharTokenizer(), budget=n)["within_budget"] is True
    assert rv.prompt_record(INSTR, pol, csp, CharTokenizer(), budget=n - 1)["within_budget"] is False


def test_an_overflow_of_the_text_budget_is_reported_and_nothing_is_truncated():
    """R7. With the CSP budget out of the way, 300 fences at one token per
    character are far past 1784; the record must say so and keep them all."""
    fp = _fp(_many_fences(300), tokenizer=CharTokenizer(), csp_budget=HUGE)
    rec, refusal = rv.check_prompt(fp)
    assert rec["within_budget"] is False and refusal["kind"] == "prompt_over_text_budget"
    assert rec["budget_used_frac"] > 1.0
    for i in range(300):
        assert f"'nfz-{i:03d}'" in rec["prompt"], f"nfz-{i:03d} was truncated away"


def test_an_estimate_never_claims_a_fit_it_cannot_prove():
    pol = load_policy(URBAN)
    fits = _rec(pol)
    assert fits["within_budget"] is True and fits["prompt_tokens_upper_bound"] < 1784
    tight = _rec(pol, text_budget=100)
    # ~40 words "fit" in 100, but the byte bound does not: unknown, not True.
    assert tight["prompt_tokens"] <= 100 < tight["prompt_tokens_upper_bound"], tight
    assert tight["within_budget"] is None, tight["within_budget"]
    assert rv.check_prompt(_fp(pol, text_budget=100))[1]["kind"] == "fit_unproven"


# --------------------------------------------------------------------------- #
# main(): --print-prompt, and refusals before the model loads
# --------------------------------------------------------------------------- #

@contextlib.contextmanager
def _patched(**attrs):
    saved = {k: getattr(rv, k) for k in attrs}
    for k, v in attrs.items():
        setattr(rv, k, v)
    try:
        yield
    finally:
        for k, v in saved.items():
            setattr(rv, k, v)


def _main_json(argv, tokenizer=CharTokenizer):
    buf = io.StringIO()
    with _patched(load_openvla_tokenizer=lambda *a, **k: tokenizer() if tokenizer else None):
        with contextlib.redirect_stdout(buf):
            code = rv.main(argv)
    return code, json.loads(buf.getvalue())


def test_print_prompt_default_is_csp_off_and_the_old_prompt():
    """REVIEW FIX. `--csp off` being the DEFAULT is what keeps the launcher
    (which passes no --csp) sending the 2026-07-14 prompt; nothing pinned it
    before. Same prompt bytes only: the action mapping changed since."""
    code, rec = _main_json(["--print-prompt"])
    assert code == rv.EXIT_OK and rec["refused"] is None
    assert rec["csp"] == "off" and rec["prompt"] == OLD_PROMPT, rec["prompt"]
    assert rec["csp_hash"] is None and rec["csp_text"] is None


def test_print_prompt_exit_codes_need_no_model_and_no_sim():
    """0 fits, 1 over a budget, 2 cannot tell, 3 no prompt: an unknown or an
    overflow must never exit like a pass."""
    code, rec = _main_json(["--csp", "on", "--print-prompt"])
    assert code == 0 and rec["csp"] == "on" and rec["token_method"] == rv.TOKENS_EXACT
    assert "Never enter zone 'nfz-residential-block'" in rec["prompt"]

    big = str(_policy_file(_many_fences(300)))
    code, rec = _main_json(["--csp", "on", "--print-prompt", "--policy", big])
    assert code == rv.EXIT_OVERFLOW and rec["refused"]["kind"] == "csp_p0_over_budget", rec
    code, rec = _main_json(["--csp", "on", "--print-prompt", "--policy", big,
                            "--csp-budget", str(HUGE)])
    assert code == rv.EXIT_OVERFLOW and rec["refused"]["kind"] == "prompt_over_text_budget"
    code, rec = _main_json(["--csp", "on", "--print-prompt", "--policy", big,
                            "--csp-budget", str(HUGE)], tokenizer=None)
    assert code == rv.EXIT_UNKNOWN and rec["within_budget"] is None, (code, rec["refused"])

    empty = str(_policy_file(_nothing_in_region()))
    code, rec = _main_json(["--csp", "on", "--print-prompt", "--policy", empty])
    assert code == rv.EXIT_REFUSED and rec["refused"]["kind"] == "no_prompt"
    assert len({rv.EXIT_OK, rv.EXIT_OVERFLOW, rv.EXIT_UNKNOWN, rv.EXIT_REFUSED}) == 4
    assert not ({"torch", "airsim"} & (set(sys.modules) - _preloaded)), "loaded a model or sim"


def test_a_refused_flight_never_loads_the_model():
    """REVIEW FIX. The R7 refusal used to run only AFTER the multi-minute 4-bit
    load, and nothing tested it. Now it is a preflight: the backend is never
    constructed, nothing is written, AirSim is never imported."""
    built = []

    class NoBackend:
        def __init__(self, *a, **k):
            built.append(a)
            raise AssertionError("OpenVLABackend constructed for a refused run")

    tmp = Path(tempfile.mkdtemp())
    big = str(_policy_file(_many_fences(300)))
    empty = str(_policy_file(_nothing_in_region()))
    cases = {"csp_p0_over_budget": ["--policy", big],
             "prompt_over_text_budget": ["--policy", big, "--csp-budget", str(HUGE)],
             "no_prompt": ["--policy", empty]}
    for kind, extra in cases.items():
        buf = io.StringIO()
        with _patched(OpenVLABackend=NoBackend, ROOT=tmp,
                      load_openvla_tokenizer=lambda *a, **k: CharTokenizer()):
            with contextlib.redirect_stdout(buf):
                code = rv.main(["--csp", "on", "--tag", "refused"] + extra)
        assert code == rv.EXIT_REFUSED, (kind, code, buf.getvalue())
        assert kind in buf.getvalue() and "REFUSING TO FLY" in buf.getvalue()
    assert not built, built
    assert not (tmp / "demo").exists(), "a refused run wrote output"
    assert "airsim" not in (set(sys.modules) - _preloaded)


# --------------------------------------------------------------------------- #
# The rotation (regression: yaw was ignored)
# --------------------------------------------------------------------------- #

def _close(a, b, tol=1e-9):
    return abs(a - b) < tol


def test_yaw_0_is_unchanged():
    a = rv.openvla_raw_to_action(FWD, 150.0, 0.0)
    assert _close(a.vx, 1.5) and _close(a.vy, 0.0), a
    a = rv.openvla_raw_to_action(RIGHT, 150.0, 0.0)
    assert _close(a.vx, 0.0) and _close(a.vy, 1.5), a


def test_yaw_90_forward_flies_east():
    """REGRESSION. Old code: vx=1.5 vy=0.0 — facing East, it flew North."""
    a = rv.openvla_raw_to_action(FWD, 150.0, 90.0)
    assert _close(a.vx, 0.0) and _close(a.vy, 1.5), f"facing East, forward flew {a}"


def test_yaw_90_right_flies_south():
    """REGRESSION. Old code: vx=0.0 vy=1.5 — facing East, "right" flew East."""
    a = rv.openvla_raw_to_action(RIGHT, 150.0, 90.0)
    assert _close(a.vx, -1.5) and _close(a.vy, 0.0), f"facing East, right flew {a}"


def test_rotation_keeps_speed_height_and_yaw_rate():
    raw = [0.013, -0.007, 0.004, 0.2, 0.1, 0.3, 1.0]
    f, r, u = rv.openvla_raw_to_body(raw, 150.0)
    from guardrail.frames import to_body
    for yaw in range(-180, 181, 15):
        a = rv.openvla_raw_to_action(raw, 150.0, float(yaw))
        assert _close(math.hypot(a.vx, a.vy), math.hypot(f, r), 1e-9), yaw
        assert _close(a.vz_up, u) and a.yaw_rate == 0.0, (yaw, a)
        bf, br, _, _ = to_body(a, float(yaw))     # and back, at the same heading
        assert _close(bf, f, 1e-9) and _close(br, r, 1e-9), yaw


def _quat_ned(yaw_deg, pitch_deg=0.0, roll_deg=0.0):
    """ZYX (yaw, pitch, roll) -> quaternion (w, x, y, z), AirSim's convention."""
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _pose(yaw_deg, x=5.0, y=6.0, z=-20.0, pitch_deg=0.0, roll_deg=0.0):
    q = _quat_ned(yaw_deg, pitch_deg, roll_deg)
    return _Obj(position=_Obj(x_val=x, y_val=y, z_val=z),
                orientation=_Obj(w_val=q[0], x_val=q[1], y_val=q[2], z_val=q[3]))


def test_yaw_from_an_airsim_quaternion():
    assert _close(rv.yaw_deg_from_quaternion(1.0, 0.0, 0.0, 0.0), 0.0)
    for yaw in (90.0, -90.0, 45.0, 179.0, -135.0):
        assert _close(rv.yaw_deg_from_quaternion(*_quat_ned(yaw)), yaw, 1e-9), yaw
    # a banked, pitched airframe still reports its heading
    assert _close(rv.yaw_deg_from_quaternion(*_quat_ned(60.0, 10.0, 5.0)), 60.0, 1e-9)


_AIRSIM_CHECK = r"""
import math, random, sys
sys.path.insert(0, sys.argv[1])
import real_vla_demo as rv
from airsim.utils import to_eularian_angles, to_quaternion
random.seed(0)
worst = 0.0
for _ in range(500):
    pitch, roll = (math.radians(random.uniform(-30, 30)) for _ in range(2))
    yaw = math.radians(random.uniform(-179, 179))
    q = to_quaternion(pitch, roll, yaw)
    ours = rv.yaw_deg_from_quaternion(q.w_val, q.x_val, q.y_val, q.z_val)
    worst = max(worst, abs(ours - math.degrees(to_eularian_angles(q)[2])))
print(worst)
"""


def test_the_heading_matches_airsims_own_conversion():
    """The test above checks the formula against THIS file's quaternion
    helper, which could share a sign mistake with it. Here AirSim's own
    to_quaternion / to_eularian_angles are the reference, over 500 random
    banked poses. In a subprocess, so importing airsim cannot leak into the
    "no sim was loaded" checks; skipped (and counted) where airsim is absent."""
    import importlib.util
    import subprocess
    if importlib.util.find_spec("airsim") is None:
        _skip("the `airsim` package is not installed in this env")
    r = subprocess.run([sys.executable, "-c", _AIRSIM_CHECK, str(ROOT / "demo")],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr[-800:]
    worst = float(r.stdout.strip().splitlines()[-1])
    assert worst < 1e-9, f"yaw differs from airsim.to_eularian_angles by {worst} deg"


def test_the_control_loop_state_carries_the_heading():
    """REVIEW FIX. The control loop and the worker build State through one
    helper; before 2026-10-06 the control loop's State had no heading (0)."""
    st = rv.state_from_pose(_pose(-120.0, x=1.0, y=-2.0, z=-18.5, pitch_deg=4.0))
    assert _close(st.yaw_deg, -120.0, 1e-9), st
    assert (st.x, st.y, st.up) == (1.0, -2.0, 18.5), st


# --------------------------------------------------------------------------- #
# Offline replay, and the worker loop that flies
# --------------------------------------------------------------------------- #

def test_a_logged_step_replays_offline():
    fp = _fp(load_policy(URBAN))
    fp.attach(Path(tempfile.mkdtemp()))
    prec = fp.current()
    st = State(x=3.0, y=-2.0, up=20.0, yaw_deg=90.0)
    raw = [0.01, 0.004, -0.002, 0.1, 0.0, 0.2, 1.0]
    rec = json.loads(json.dumps(rv.step_record(7, 9, "frames/r/000009.png", st, raw,
                                               150.0, prec, 0.91)))
    for k in ("inference", "frame_index", "frame_file", "state", "raw_action",
              "scale", "prompt", "prompt_sha256", "prompt_tokens", "token_method",
              "csp_hash", "csp_file", "policy_hash", "policy_generation",
              "world_action", "within_budget"):
        assert rec.get(k) is not None, f"{k} missing: the step cannot be replayed"
    assert rec["raw_action"] == raw and rec["frame_index"] == 9
    assert rec["prompt"] == prec["prompt"] and rec["policy_generation"] == 0
    assert rec["body_action"]["lateral_sign"].startswith("unverified"), \
        "the right/left assumption must travel with the data"
    again = rv.replay_step(rec)
    assert again.model_dump() == rec["world_action"], (again, rec["world_action"])
    assert rec["world_action"]["vy"] > 1.4, "logged at yaw 90, forward must be East"
    # --no-frames: the record says the frame is gone rather than naming a file
    nf = rv.step_record(1, 1, None, st, raw, 150.0, prec)
    assert nf["frame_file"] is None and nf["infer_s"] is None


PNG = b"\x89PNG-stand-in-bytes"


@contextlib.contextmanager
def _stand_in_modules(mods):
    saved = {k: sys.modules.get(k) for k in mods}
    sys.modules.update(mods)
    try:
        yield
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


class _Sim:
    """A stand-in AirSim world, shared by every client that main() and the
    worker open (they open two, as in the real demo).

    The pose is x=5, y=6, z=`self.z` (NED: -20 is 20 m up), heading
    `self.yaw`; a test changes either at any moment and the next
    simGetVehiclePose sees it. simGetImage serves `frames` PNGs, calling
    `before_frame(i)` first; after that it calls `exhausted()` (the worker
    tests stop the loop there) and returns None. Takeoffs, landings and every
    velocity command are recorded."""

    def __init__(self, yaw=0.0, frames=1, before_frame=None, exhausted=None):
        self.yaw, self.z = yaw, -20.0
        self.frames, self.before_frame, self.exhausted = frames, before_frame, exhausted
        self.calls = {"img": 0, "pose": 0, "takeoff": 0, "land": 0, "move": []}

    def modules(self, stand_in_pil=True):
        """airsim and cv2 stand-ins, and PIL's unless `stand_in_pil` is False
        (then the worker's resize runs on the real Pillow, and matplotlib,
        which imports PIL, can plot)."""
        import numpy as np
        sim = self

        class Fut:
            def join(self):
                pass

        class Client:
            def confirmConnection(self):
                pass

            def enableApiControl(self, on):
                pass

            def armDisarm(self, on):
                pass

            def takeoffAsync(self):
                sim.calls["takeoff"] += 1
                return Fut()

            def landAsync(self):
                sim.calls["land"] += 1
                return Fut()

            def moveByVelocityAsync(self, vx, vy, vz, duration=None):
                sim.calls["move"].append((vx, vy, vz))
                return Fut()

            def simGetImage(self, cam, kind):
                sim.calls["img"] += 1
                if sim.calls["img"] > sim.frames:
                    if sim.exhausted is not None:
                        sim.exhausted()
                    return None
                if sim.before_frame is not None:
                    sim.before_frame(sim.calls["img"])
                return PNG

            def simGetVehiclePose(self):
                sim.calls["pose"] += 1
                return _pose(sim.yaw, z=sim.z)

        airsim = types.ModuleType("airsim")
        airsim.MultirotorClient = Client
        airsim.ImageType = _Obj(Scene=0)
        cv2 = types.ModuleType("cv2")
        cv2.IMREAD_COLOR, cv2.COLOR_BGR2RGB = 1, 4
        cv2.imdecode = lambda buf, flag: np.zeros((4, 4, 3), dtype=np.uint8)
        cv2.cvtColor = lambda img, code: img
        if not stand_in_pil:
            return {"airsim": airsim, "cv2": cv2}
        pil = types.ModuleType("PIL")
        pil.Image = _Obj(fromarray=lambda a: _Obj(
            resize=lambda size: "img224" if tuple(size) == (224, 224) else f"img{size}"))
        return {"airsim": airsim, "cv2": cv2, "PIL": pil}


def _is_224(img):
    return img == "img224" or getattr(img, "size", None) == (224, 224)


class _StandInBackend(rv.OpenVLABackend):
    """OpenVLABackend with ONLY `_load` (the GPU part) replaced: the real
    `__init__`, worker, start and stop run. The processor records every prompt
    it receives and carries `proc_tokenizer` - the processor's own tokenizer,
    which the real `__init__` hands to the FlightPrompt when the preflight had
    none. predict_action returns `next_raw`, then calls `after_predict`.

    For main(): `start()` waits (real time) for the first inference or
    refusal, so every control tick after it has a step to trace to, and
    `stop()` puts the drone on the ground so main()'s descent loop ends."""
    proc_tokenizer = None
    sim = None

    def _load(self):
        import numpy as np
        self.seen, self.next_raw, self.after_predict = [], np.array(FWD), None
        b = self

        class Inputs(dict):
            def to(self, *a, **k):
                return self

        class Proc:
            tokenizer = b.proc_tokenizer

            def __call__(self, prompt, img):
                b.seen.append((prompt, img))
                return Inputs(input_ids=[1, 2, 3], pixel_values="px")

        def predict_action(**kw):
            assert kw["unnorm_key"] == rv.UNNORM_KEY
            raw = b.next_raw
            if b.after_predict is not None:
                b.after_predict()
            return raw

        return (Proc(), _Obj(predict_action=predict_action),
                _Obj(no_grad=contextlib.nullcontext, bfloat16="bf16"))

    def start(self):
        super().start()
        t = _time.time()
        while self._n_inf + self._n_refused == 0 and _time.time() - t < 10.0:
            _time.sleep(0.001)

    def stop(self):
        super().stop()
        if self.sim is not None:
            self.sim.z = -0.5


def _backend(prompts, out, frames_dir="frames/run-test"):
    return _StandInBackend(prompts, scale=150.0, out=out, frames_dir=frames_dir)


def _run_worker(b, sim):
    sim.exhausted = lambda: setattr(b, "_stop", True)
    with _stand_in_modules(sim.modules()), contextlib.redirect_stdout(io.StringIO()):
        b._worker()
    lines = (b.out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()
    return [json.loads(x) for x in lines]


def test_the_worker_feeds_the_csp_prompt_and_logs_a_replayable_step():
    """The real `_worker` loop against stand-ins. Checks the prompt the
    PROCESSOR received (not the one we meant to send), the heading of the pose
    read at capture, and that the frame, the CSP file and the step line land
    on disk. REVIEW FIX: the stand-in turns the drone to North the moment the
    model answers, so a pose read moved past inference logs (and flies) yaw 0,
    and a second pose read shows in the call count."""
    out = Path(tempfile.mkdtemp())
    fp = _fp(load_policy(URBAN), tokenizer=CharTokenizer())
    fp.attach(out)
    b = _backend(fp, out)
    sim = _Sim(yaw=90.0, frames=1)
    b.after_predict = lambda: setattr(sim, "yaw", 0.0)
    recs = _run_worker(b, sim)

    assert len(b.seen) == 1 and b.seen[0][1] == "img224", b.seen
    assert b.seen[0][0] == b.prompt()["prompt"]
    assert "Never enter zone 'nfz-residential-block'" in b.seen[0][0], b.seen[0][0]
    assert len(recs) == 1 and sim.calls["pose"] == 1, (recs, sim.calls)
    rec = recs[0]
    assert rec["prompt"] == b.seen[0][0] and rec["csp"] == "on"
    assert rec["token_method"] == rv.TOKENS_EXACT
    csp_file = CSP.model_validate_json((out / rec["csp_file"]).read_text(encoding="utf-8"))
    assert compiler_csp_hash(csp_file) == rec["csp_hash"]
    assert _close(rec["state"]["yaw_deg"], 90.0, 1e-9), \
        f"logged yaw {rec['state']['yaw_deg']}: the pose was read after inference"
    assert rec["state"]["up"] == 20.0
    assert rec["frame_index"] == 1 and rec["frame_file"] == "frames/run-test/000001.png"
    assert (out / rec["frame_file"]).read_bytes() == PNG
    assert _close(rec["world_action"]["vy"], 1.5) and _close(rec["world_action"]["vx"], 0.0)
    act, n = b.latest()
    assert n == 1 and act.model_dump() == rec["world_action"]
    assert rv.replay_step(rec).model_dump() == rec["world_action"]


def test_the_worker_rebuilds_the_prompt_after_a_hot_apply():
    out = Path(tempfile.mkdtemp())
    pol = load_policy(URBAN)
    shield = Shield(pol)
    fp = _fp(pol, tokenizer=CharTokenizer())
    fp.attach(out)
    b = _backend(fp, out)

    def before_frame(i):
        if i == 2:
            shield.hot_apply(pol.constraints[0].model_copy(update={"id": "nfz-hot"}))

    recs = _run_worker(b, _Sim(yaw=0.0, frames=2, before_frame=before_frame))
    assert len(b.seen) == 2 and len(recs) == 2
    assert "nfz-hot" not in b.seen[0][0] and "Never enter zone 'nfz-hot'" in b.seen[1][0]
    assert [r["policy_generation"] for r in recs] == [0, 1]
    assert recs[0]["csp_hash"] != recs[1]["csp_hash"]
    assert recs[0]["csp_file"] != recs[1]["csp_file"]
    assert all((out / r["csp_file"]).exists() for r in recs)


def test_the_worker_refuses_a_prompt_that_failed_and_holds_zero():
    """REVIEW FIX. A prompt rebuilt mid-flight was never budget-checked again:
    an over-budget prompt would have been fed to the model. Now the frame is
    refused, the action held at zero (not the stale one), and the refusal
    logged; once the policy is valid again, inference resumes, and the frame
    file is named by FRAME index, not by inference number. SECOND REVIEW FIX:
    the held zero used to keep the previous inference's number, so
    trajectory.json credited the hover ticks to an inference that did not
    produce them; during the hold latest() now names no inference (None)."""
    out = Path(tempfile.mkdtemp())
    pol = load_policy(URBAN)
    good = list(pol.constraints)
    pol.constraints[:] = _many_fences(300).constraints
    fp = _fp(pol, tokenizer=CharTokenizer(), csp_budget=HUGE)
    b = _backend(fp, out)
    b._last, b._last_inf = rv.Action4D(vx=3.0), 7      # a stale action from before
    held = []

    def before_frame(i):
        if i == 2:
            act, n = b.latest()
            held.append((act.model_dump(), n))
            pol.constraints[:] = good            # the policy becomes valid again

    recs = _run_worker(b, _Sim(yaw=0.0, frames=2, before_frame=before_frame))
    assert held == [(rv.Action4D().model_dump(), None)], \
        f"held {held}: not zero, or credited to an inference"
    assert len(recs) == 2 and b._n_refused == 1, recs
    ref, ok = recs
    assert ref["refused"]["kind"] == "prompt_over_text_budget", ref["refused"]
    assert ref["inference"] is None and ref["frame_file"] is None and ref["frame_index"] == 1
    assert ref["within_budget"] is False
    assert len(b.seen) == 1 and "nfz-299" not in b.seen[0][0], "the failed prompt was fed"
    assert ok["inference"] == 1 and ok["frame_index"] == 2
    assert ok["frame_file"] == "frames/run-test/000002.png" and (out / ok["frame_file"]).exists()
    assert not (out / "frames/run-test/000001.png").exists(), "a refused frame was kept"
    assert b.latest()[1] == 1, "inference resumed but the action is not credited to it"


# --------------------------------------------------------------------------- #
# main()'s flight path, end to end against the stand-in sim
# --------------------------------------------------------------------------- #

class _Clock:
    """main()'s `time` module. sleep() on the main thread advances a fake
    clock by the full amount, so the control loop runs MAX_S / TICK ticks on
    any machine; on other threads (the worker) it only yields."""

    def __init__(self):
        self.t = 1.0e6
        self._main = threading.main_thread()

    def time(self):
        return self.t

    def sleep(self, s):
        if threading.current_thread() is self._main:
            self.t += s
        _time.sleep(0.0005)


def _fly(argv, *, yaw=90.0, frames=3, preflight_tokenizer=CharTokenizer,
         proc_tokenizer=None, plot=False):
    """rv.main(argv) end to end: the real preflight, FlightPrompt, backend
    __init__, worker thread, control loop, Shield, audit, run record and
    report, against `_Sim`. Only `_load` is a stand-in. Output goes to a temp
    ROOT. Returns (exit code, out dir, stdout, sim, backend or None)."""
    tmp = Path(tempfile.mkdtemp())
    sim, built = _Sim(yaw=yaw, frames=frames), []

    class Backend(_StandInBackend):
        def __init__(self, *a, **k):
            built.append(self)
            super().__init__(*a, **k)

    Backend.sim, Backend.proc_tokenizer = sim, proc_tokenizer
    mods = sim.modules(stand_in_pil=importlib.util.find_spec("PIL") is None)
    if not plot:
        mods["matplotlib"] = None          # `import matplotlib` raises: plot skipped
    buf = io.StringIO()
    with _stand_in_modules(mods), \
            _patched(OpenVLABackend=Backend, ROOT=tmp, MAX_S=1.0, time=_Clock(),
                     load_openvla_tokenizer=lambda *a, **k: (
                         preflight_tokenizer() if preflight_tokenizer else None)):
        with contextlib.redirect_stdout(buf):
            code = rv.main(argv)
    out = tmp / "demo" / "out" / argv[argv.index("--tag") + 1]
    return code, out, buf.getvalue(), sim, (built[0] if built else None)


def _steps(out):
    return [json.loads(x) for x in
            (out / "vla_steps.jsonl").read_text(encoding="utf-8").splitlines()]


def test_main_flies_the_csp_prompt_with_the_heading_and_writes_every_artefact():
    """REVIEW FIX (blocker). No test ran main()'s flight path: putting the
    control loop's old `State(x, y, up)` (no heading) back into main() passed
    every test, and so did deleting `prompts.attach(out)` or forcing
    `frames_dir = None`. Here main() flies 1 s at yaw 90 with --csp on.

    The control loop's trajectory carries the pose's heading; the first
    command sent to the sim is East (forward, facing East, after the rate
    limiter); the processor received the CSP prompt the steps logged; every
    step names a CSP file under csp/<run_id>/ that hashes to its csp_hash;
    every frame is under frames/<run_id>/, named by frame index; vla_run.json
    holds the same prompt, the per-run folders, the resize filter and the
    library versions."""
    code, out, stdout, sim, b = _fly(
        ["--csp", "on", "--tag", "fly", "--policy", str(URBAN)], yaw=90.0, plot=True)
    assert code == rv.EXIT_OK, f"exit {code}\n{stdout[-1500:]}"
    run = json.loads((out / "vla_run.json").read_text(encoding="utf-8"))
    run_id = run["run_id"]
    assert run["frames_dir"] == f"frames/{run_id}" and run["csp_dir"] == f"csp/{run_id}", \
        f"frames_dir {run['frames_dir']!r}, csp_dir {run['csp_dir']!r}: not per run"
    assert run["prompt"]["csp"] == "on" and run["prompt"]["token_method"] == rv.TOKENS_EXACT
    assert run["preprocess"]["resize"] == [224, 224] and "Pillow" in run["libraries"]

    steps = _steps(out)
    assert len(steps) == 3 and [s["inference"] for s in steps] == [1, 2, 3], steps
    assert [p for p, _ in b.seen] == [s["prompt"] for s in steps] == [run["prompt"]["prompt"]] * 3
    assert all(_is_224(img) for _, img in b.seen), [img for _, img in b.seen]
    assert "Never enter zone 'nfz-residential-block'" in steps[0]["prompt"]
    for s in steps:
        assert s["csp_file"] and s["csp_file"].startswith(f"csp/{run_id}/"), \
            f"step {s['inference']} names no CSP file under csp/{run_id}/: {s['csp_file']!r}"
        on_disk = CSP.model_validate_json((out / s["csp_file"]).read_text(encoding="utf-8"))
        assert compiler_csp_hash(on_disk) == s["csp_hash"] == run["prompt"]["csp_hash"]
        assert s["frame_file"] == f"frames/{run_id}/{s['frame_index']:06d}.png", s["frame_file"]
        assert (out / s["frame_file"]).read_bytes() == PNG
        assert _close(s["state"]["yaw_deg"], 90.0, 1e-9)
        assert _close(s["world_action"]["vy"], 1.5) and _close(s["world_action"]["vx"], 0.0)

    traj = json.loads((out / "trajectory.json").read_text(encoding="utf-8"))
    assert 10 <= len(traj) <= 11, len(traj)                   # MAX_S 1.0 / TICK 0.1
    bad = [r["yaw_deg"] for r in traj if not _close(r["yaw_deg"], 90.0, 1e-6)]
    assert not bad, f"the control loop's State lost the heading: yaw {bad[:3]}"
    assert all(r["inference"] in (1, 2, 3) for r in traj), [r["inference"] for r in traj]
    vx, vy, vz = sim.calls["move"][0]
    assert _close(vx, 0.0, 1e-9) and _close(vy, 0.25, 1e-9), \
        f"first command {sim.calls['move'][0]}: facing East, forward must fly East"
    assert len(sim.calls["move"]) == len(traj) and sim.calls["takeoff"] == 1 == sim.calls["land"]
    if importlib.util.find_spec("matplotlib") is not None:
        assert (out / "trajectory.png").exists(), stdout[-800:]
    assert "inferences 3 | refused 0" in stdout, stdout[-800:]


def test_main_without_frames_says_no_frame_and_off_sends_the_old_prompt():
    """--no-frames: no frames folder, and every step says frame_file null
    rather than naming a file. --csp off (the default): the old prompt, no CSP
    file, the CSP fields null. Facing North the rotation is the identity."""
    code, out, stdout, sim, b = _fly(
        ["--no-frames", "--tag", "nf", "--policy", str(URBAN)], yaw=0.0)
    assert code == rv.EXIT_OK, f"exit {code}\n{stdout[-1500:]}"
    run = json.loads((out / "vla_run.json").read_text(encoding="utf-8"))
    assert run["frames_dir"] is None and run["prompt"]["prompt"] == OLD_PROMPT
    steps = _steps(out)
    assert len(steps) == 3
    for s in steps:
        assert s["frame_file"] is None and s["csp"] == "off" and s["prompt"] == OLD_PROMPT
        assert s["csp_file"] is None and s["csp_hash"] is None
        assert _close(s["world_action"]["vx"], 1.5) and _close(s["world_action"]["vy"], 0.0)
    assert not (out / "frames").exists(), "frames kept under --no-frames"
    csp_files = list((out / "csp").rglob("*.json")) if (out / "csp").exists() else []
    assert csp_files == [], f"--csp off wrote {csp_files}"
    assert all(_close(r["yaw_deg"], 0.0, 1e-6) for r in json.loads(
        (out / "trajectory.json").read_text(encoding="utf-8")))


def _big_policy_file():
    """100 fences: the estimate cannot prove a fit (byte bound past 1784) while
    a word count can - the case the post-load recount decides."""
    return str(_policy_file(_many_fences(100)))


def test_main_flies_when_the_preflight_cannot_tell_and_the_exact_count_fits():
    """Without tokenizer.json the preflight can only say "fit unproven"; that
    must NOT refuse the flight, because the processor's exact tokenizer
    decides after the load. Here it fits, so the run flies and logs exact
    counts."""
    code, out, stdout, sim, b = _fly(
        ["--csp", "on", "--csp-budget", str(HUGE), "--tag", "fu", "--policy", _big_policy_file()],
        preflight_tokenizer=None, proc_tokenizer=WordTokenizer(), frames=1)
    assert code == rv.EXIT_OK, (f"exit {code}: an unproven preflight refused a flight "
                                f"the exact count allows\n{stdout[-600:]}")
    assert "REFUSING" not in stdout
    steps = _steps(out)
    assert len(steps) == 1 and steps[0]["token_method"] == rv.TOKENS_EXACT
    assert steps[0]["tokenizer"] == "WordTokenizer: source unidentified"
    assert steps[0]["within_budget"] is True and "'nfz-099'" in steps[0]["prompt"]
    run = json.loads((out / "vla_run.json").read_text(encoding="utf-8"))
    assert run["prompt"]["token_method"] == rv.TOKENS_EXACT and sim.calls["takeoff"] == 1


def test_main_refuses_after_the_load_when_the_exact_count_overflows():
    """The same unproven preflight, but the exact count is over OpenVLA's text
    budget: main() must refuse BEFORE takeoff - no command sent, no step, no
    run record - and never start the worker."""
    code, out, stdout, sim, b = _fly(
        ["--csp", "on", "--csp-budget", str(HUGE), "--tag", "ov", "--policy", _big_policy_file()],
        preflight_tokenizer=None, proc_tokenizer=CharTokenizer(), frames=1)
    assert code == rv.EXIT_REFUSED, \
        f"exit {code}: flew a prompt the exact count put over the text budget"
    assert "REFUSING TO FLY (prompt_over_text_budget)" in stdout, stdout[-800:]
    assert b is not None and b._thread.ident is None, "the worker was started"
    assert sim.calls["takeoff"] == 0 and sim.calls["move"] == [] and sim.calls["img"] == 0
    assert _steps(out) == [] and not (out / "vla_run.json").exists()


# --------------------------------------------------------------------------- #
# The tokenizer
# --------------------------------------------------------------------------- #

def test_a_missing_tokenizer_is_none_not_a_crash():
    """The note goes to stderr: on stdout it would sit ahead of --print-prompt's
    JSON, and that output would not parse on any machine without the file."""
    with contextlib.redirect_stdout(io.StringIO()) as out, \
            contextlib.redirect_stderr(io.StringIO()) as err:
        assert rv.load_openvla_tokenizer(tempfile.mkdtemp()) is None
    assert "ESTIMATES" in err.getvalue(), "the fallback must say what it falls back to"
    assert out.getvalue() == "", f"stdout polluted: {out.getvalue()!r}"


def test_print_prompt_stdout_is_json_without_the_checkpoint():
    """The real loader (not a stand-in) pointed at a folder with no
    tokenizer.json: stdout must still be exactly one JSON document."""
    out, err = io.StringIO(), io.StringIO()
    with _patched(MODEL_ID=tempfile.mkdtemp()):
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = rv.main(["--csp", "on", "--print-prompt"])
    rec = json.loads(out.getvalue())                     # raises if polluted
    assert code == rv.EXIT_OK and rec["token_method"] == rv.TOKENS_ESTIMATE, (code, rec)
    assert "ESTIMATES" in err.getvalue()


def test_the_real_openvla_tokenizer_if_present():
    """With the checkpoint's tokenizer.json (CPU, no weights, no torch): the
    whitespace estimate really does under-count, the byte bound really is an
    upper bound, and so is the compiler's table counter. The loader is the
    compiler's (`guardrail.csp.openvla_token_counter`), named by the file's
    sha256; and the prompt count's "+1 for BOS" equals what the file's own
    post-processor adds, checked against `tokenizers` directly. Skipped, and
    counted as skipped, where the package or the file is absent."""
    try:
        from tokenizers import Tokenizer
    except ImportError:
        _skip("the `tokenizers` package is not installed in this env")
    tj = Path(rv.MODEL_ID) / "tokenizer.json"
    if not tj.exists():
        _skip(f"no tokenizer.json at {rv.MODEL_ID}")
    torch_before = "torch" in sys.modules
    with contextlib.redirect_stdout(io.StringIO()):
        tok = rv.load_openvla_tokenizer()
    assert tok is not None, "tokenizers and the file are here, yet it failed to load"
    assert torch_before or "torch" not in sys.modules, "counting tokens imported torch"
    from guardrail.csp import OpenVLATokenCounter
    assert isinstance(tok, OpenVLATokenCounter), type(tok)
    sha12 = hashlib.sha256(tj.read_bytes()).hexdigest()[:12]
    assert tok.name == f"openvla-llama2-sp@{sha12}" == rv.tokenizer_id(tok), tok.name
    raw = Tokenizer.from_file(str(tj))
    for path in (URBAN, PED, CORRIDOR):
        pol = load_policy(path)
        exact = _rec(pol, tokenizer=tok)
        est = _rec(pol)
        assert exact["token_method"] == rv.TOKENS_EXACT
        assert exact["tokenizer"] == tok.name
        assert exact["prompt_tokens"] == len(raw.encode(exact["prompt"]).ids), "BOS miscounted"
        assert exact["csp_tokens"] == len(raw.encode(exact["csp_text"],
                                                     add_special_tokens=False).ids)
        assert exact["within_budget"] is True
        assert exact["csp_tokens"] > est["csp_tokens"], (path.name, exact["csp_tokens"])
        assert est["prompt_tokens_upper_bound"] >= exact["prompt_tokens"], path.name
        assert exact["csp_tokens_used"] >= exact["csp_tokens"], "table counter under-counted"
        print(f"      {path.name}: CSP {exact['csp_tokens']} tokens "
              f"(table {exact['csp_tokens_used']}, whitespace words {est['csp_tokens']}), "
              f"prompt {exact['prompt_tokens']}/{rv.OPENVLA_TEXT_BUDGET}")


def test_two_unclocked_compiles_match_by_content_hash_not_by_csp_hash():
    """The demo compiles without a clock, so `issued_at` comes from the wall
    clock and csp_hash differs between two runs of the same flight. The record
    now carries csp_content_hash (every field but the stamp), which matches."""
    from guardrail.csp import csp_content_hash
    pol = load_policy(PED)
    a = rv.compile_flight_csp(pol, _mission())
    b = a.model_copy(update={"issued_at": "2099-01-01T00:00:00"})
    ra, rb = rv.prompt_record(INSTR, pol, a), rv.prompt_record(INSTR, pol, b)
    assert ra["csp_hash"] != rb["csp_hash"], "fixture broken: the stamps were equal"
    assert ra["csp_content_hash"] == rb["csp_content_hash"] == csp_content_hash(a)
    assert ra["csp_content_hash"].startswith("sha256:"), ra["csp_content_hash"]
    # and a different policy's CSP is a different content hash
    other = load_policy(CORRIDOR)
    rc = rv.prompt_record(INSTR, other, rv.compile_flight_csp(other, _mission()))
    assert rc["csp_content_hash"] != ra["csp_content_hash"]


def test_a_stored_paraphrase_is_flown_traceably_and_only_as_a_verb_phrase():
    """--paraphrase-index / --paraphrase-seed (WP4 per-paraphrase robustness):
    the flown text comes from the stored set, its record names the item, and an
    `utterance` item is refused in OpenVLA's question frame unless asked for."""
    text, rec = rv.paraphrased_instruction(INSTR)
    assert (text, rec) == (INSTR, None), "no flag must fly the instruction as typed"
    t0, r0 = rv.paraphrased_instruction(INSTR, index=0)
    assert t0 != INSTR and r0["index"] == 0 and r0["form"] == "verb_phrase", r0
    assert r0["text"] == t0 and r0["paraphrase_id"].startswith("pp-"), r0
    assert r0["grammar_confound"] is False
    assert rv.build_openvla_prompt(t0) == f"In: {QUESTION} {t0}?\nOut:"
    # index 4 is an utterance ("Please fly forward, ..."): refused by default
    try:
        rv.paraphrased_instruction(INSTR, index=4)
    except ValueError as e:
        assert "verb_phrase" in str(e), e
    else:
        raise AssertionError("an utterance was accepted into the question frame")
    t4, r4 = rv.paraphrased_instruction(INSTR, index=4, allow_utterance=True)
    assert r4["form"] == "utterance" and r4["grammar_confound"] is True, r4
    # a seed draw is deterministic and lands on a verb phrase
    ta, ra = rv.paraphrased_instruction(INSTR, seed=7)
    tb, rb = rv.paraphrased_instruction(INSTR, seed=7)
    assert (ta, ra) == (tb, rb) and ra["form"] == "verb_phrase", ra
    seen = {rv.paraphrased_instruction(INSTR, seed=s)[1]["index"] for s in range(12)}
    assert len(seen) > 1, f"every seed drew the same item {seen}: the seed does nothing"
    # an instruction with no stored set refuses; it never falls back silently
    from guardrail import paraphraser as P
    try:
        rv.paraphrased_instruction("hover over the moon", index=0)
    except P.UnknownSource:
        pass
    else:
        raise AssertionError("a paraphrase run fell back to the canonical text")


def test_print_prompt_names_the_paraphrase_it_would_fly():
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = rv.main(["--print-prompt", "--paraphrase-index", "1"])
    shown = json.loads(out.getvalue())
    assert shown["paraphrase"]["index"] == 1, shown["paraphrase"]
    assert shown["paraphrase"]["text"] in shown["prompt"], shown["prompt"]
    assert INSTR not in shown["prompt"], "the canonical text was flown instead"
    assert code == rv._PRINT_EXIT[None if shown["refused"] is None
                                  else shown["refused"]["kind"]]


def test_importing_the_demo_loads_no_model():
    assert not _loaded_by_import, f"import pulled in {_loaded_by_import}"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Skipped as e:
            skipped += 1
            print(f"SKIP  {fn.__name__}\n      {e}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    ran = len(fns) - skipped
    print(f"\n{ran - failed}/{ran} passed" + (f", {skipped} skipped" if skipped else ""))
    sys.exit(1 if failed else 0)
