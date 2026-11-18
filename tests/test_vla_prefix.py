"""The CSP in a text-reading VLA's prompt for the PAS follow's prefix A/B
(guardrail/vla_prefix.py; WP4-15, WP4-01).

Run either way:
    pytest tests/test_vla_prefix.py -v
    python tests/test_vla_prefix.py

What is pinned here:
  * Each model's prompt slot and budget: AerialVLA's instruction line at the
    compiler's default budget; AIoT-VLA's object words inside its backbone's
    text window, the budget the window minus the special tokens minus the
    object words; every other family refused.
  * The grant's budget rule on that slot: P0 rules that do not fit raise
    (CSPBudgetExceeded -> PrefixRefused "csp_budget_exceeded"), never a
    truncation; with OWL-ViT's own CLIP tokenizer (SKIPPED where it is not
    on the machine) the follow policy's four P0 sentences need 45 tokens
    against 11 beside "a red car".
  * The neutral arm: rule-free, length-matched in the model's OWN tokens
    (with OpenVLA's tokenizer.json where it is here, SKIPPED elsewhere: 85
    tokens of CSP, the neutral text within one word above), inside the
    budget, and refused where that tokenizer is not on the machine.
  * A refusal names each P0 sentence's own count beside the budget.
  * The AIoT-VLA server's CSP slot: the query "<csp> <object words>", the
    contrast phrases still the object words', the window refused not cut,
    every other family still refusing a CSP arm.
  * The warm-up asks the flight's prompt.
  * tools/kpi_report.py pairs the arms by --ab-pair and shows the delta,
    keeps them out of every other A/B group, and names a refused arm.
"""
from __future__ import annotations

import base64
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import aerialvla_codec as C                          # noqa: E402
from guardrail import vla_prefix as VP                              # noqa: E402
from guardrail.compiler import DEFAULT_BUDGET_TOKENS, Mission       # noqa: E402
from guardrail.csp import DEFAULT_TOKEN_COUNTER, TokenCounter       # noqa: E402
from guardrail.models import load_policy                            # noqa: E402

SKIP = "SKIP"
FOLLOW_POLICY = ROOT / "policies" / "follow_car_citylife.yaml"
OWL = "google/owlvit-base-patch32"
AERIAL = {"id": "aerialvla-lora/aero_vla@196f2f32"}


class Words(TokenCounter):
    """One token per space-separated word: a stand-in for CLIP's BPE whose
    counts a test can do by eye."""
    name = "words-test"

    def count(self, text: str) -> int:
        return len(text.split()) if text else 0


def _aiot_identity(window=16, specials=2, tokenizer=OWL):
    return {"id": "aiotvla/v10b@12345678", "variant": "foreign",
            "foreign": {"family": "aiotvla", "config": {"csp_slot": {
                "placement": "in front of the object words", "text_window_tokens": window,
                "special_tokens": specials, "tokenizer": tokenizer}}}}


def _mission():
    return Mission(task_text="follow a red car", target_x=35.0, target_y=-20.0,
                   cruise_alt_m=8.0, speed_pref_mps=1.0, start_x=35.0, start_y=-20.0)


def _refused(fn, code):
    try:
        fn()
    except VP.PrefixRefused as exc:
        assert exc.code == code, (exc.code, str(exc))
        return exc
    raise AssertionError(f"not refused ({code})")


# ------------------------------------------------------------------ the slot
def test_aerialvla_and_its_stand_in_get_the_instruction_line_at_the_default_budget():
    for ident in (AERIAL, {"id": "fake:scripted"}, None):
        s = VP.slot_for(ident, "a red car")
        assert (s.model, s.budget_tokens, s.counter) == ("aerialvla", DEFAULT_BUDGET_TOKENS,
                                                          DEFAULT_TOKEN_COUNTER), s
        assert s.window_tokens is None and "build_prompt" in s.placement


def test_aiotvlas_budget_is_its_window_less_the_special_tokens_and_the_object_words():
    s = VP.slot_for(_aiot_identity(), "a red car", counter_factory=lambda tok: Words())
    assert (s.model, s.window_tokens, s.special_tokens, s.object_tokens, s.budget_tokens) == \
        ("aiotvla", 16, 2, 3, 11), s
    rec = s.record()
    assert rec["token_counter"] == "words-test" and "16" in rec["budget_basis"]
    # the tokenizer the hello names is the one asked for
    asked = []
    VP.slot_for(_aiot_identity(tokenizer="x/y"), "a red car",
                counter_factory=lambda tok: asked.append(tok) or Words())
    assert asked == ["x/y"]


def test_a_slot_is_refused_without_a_declared_window_a_tokenizer_or_room():
    _refused(lambda: VP.slot_for({"id": "dronecats/DroneCATS-Cosmos3-Edge-2B@1a2b3c4d",
                                  "foreign": {"config": {}}}, "a red car"), "no_slot")
    bad = _aiot_identity()
    del bad["foreign"]["config"]["csp_slot"]["text_window_tokens"]
    _refused(lambda: VP.slot_for(bad, "a red car", counter_factory=lambda t: Words()),
             "no_slot")
    _refused(lambda: VP.slot_for(_aiot_identity(), "a red car",
                                 counter_factory=lambda t: None), "no_tokenizer")
    _refused(lambda: VP.slot_for(_aiot_identity(window=5), "a red car",
                                 counter_factory=lambda t: Words()), "no_room")


# ---------------------------------------------------------------- the prefix
def test_the_three_arms_on_aerialvla_carry_the_csp_its_null_and_nothing():
    pol = load_policy(FOLLOW_POLICY)
    # the model's own tokenizer stood in by Words (the real one: the test
    # with OpenVLA's tokenizer below)
    s = VP.slot_for(AERIAL, "a red car", exact_counter_factory=Words)
    off, csp0 = VP.compile_prompt_prefix(pol, _mission(), s, "off", "a red car")
    assert csp0 is None and off["prefix"] == "off" and off["prefix_text"] is None
    assert off["prefix_sha256"] is None and off["model_trained_on_prefix"] is False
    on, csp = VP.compile_prompt_prefix(pol, _mission(), s, "csp", "a red car")
    assert on["prefix"] == "on" and on["csp_mode"] == "csp"
    assert on["prefix_text"] == csp.natural_language_prompt.strip()
    assert on["prefix_sha256"] == VP.sha256_text(on["prefix_text"])
    assert sorted(on["p0_ids"]) == ["alt-band", "bld-clearance", "standoff-any",
                                    "standoff-pedestrian"], on["p0_ids"]
    assert 0 < on["prefix_tokens"] <= DEFAULT_BUDGET_TOKENS
    assert on["csp_hash"].startswith("sha256:") and on["csp_content_hash"].startswith("sha256:")
    # the codec places it on the instruction line, ahead of the task sentence
    prompt = C.build_prompt("straight ahead ", "a red car", on["prefix_text"], "csp")
    assert prompt.startswith("<image>\n" + on["prefix_text"] + " Fly straight ahead "), prompt
    assert on["prefix_token_counter"] == DEFAULT_TOKEN_COUNTER.name
    w = Words()
    assert on["prefix_tokens_exact"] == w.count(on["prefix_text"])
    nt, _ = VP.compile_prompt_prefix(pol, _mission(), s, "neutral", "a red car")
    assert nt["prefix"] == "neutral" and nt["prefix_text"] != on["prefix_text"]
    # matched in the model's own tokens, not under the budget's upper bound:
    # the same count, word for word (Words: one token per word)
    lm = nt["length_matched_to"]
    assert lm["csp_sha256"] == on["prefix_sha256"] and lm["token_counter"] == "words-test"
    assert lm["csp_tokens"] == on["prefix_tokens_exact"] == w.count(on["prefix_text"])
    assert lm["neutral_tokens"] == nt["prefix_tokens"] == nt["prefix_tokens_exact"] \
        == w.count(nt["prefix_text"]) == lm["csp_tokens"], (lm, nt["prefix_tokens"])
    assert nt["prefix_token_counter"] == "words-test"
    assert lm["csp_tokens_budget_counter"] == on["prefix_tokens"]
    for word in ("m/s", "altitude", "Keep", "Never", "zone"):
        assert word not in nt["prefix_text"], word
    # same policy, mission and clock: the same content
    on2, _ = VP.compile_prompt_prefix(pol, _mission(), s, "csp", "a red car")
    assert on2["csp_content_hash"] == on["csp_content_hash"]


def test_aerialvlas_neutral_arm_is_refused_without_its_own_tokenizer():
    """Review of 94a7d7b: matched under the budget's upper-bound counter the
    neutral text was 18 real OpenVLA tokens against the CSP's 85. Without
    OpenVLA's tokenizer on the machine the neutral arm is refused, never
    mis-matched; the csp and off arms need no exact count and still compile."""
    pol = load_policy(FOLLOW_POLICY)
    s = VP.slot_for(AERIAL, "a red car", exact_counter_factory=lambda: None)
    assert s.exact_counter is None and s.record()["exact_token_counter"] is None
    exc = _refused(lambda: VP.compile_prompt_prefix(pol, _mission(), s, "neutral", "a red car"),
                   "no_tokenizer")
    assert "refused rather than mis-matched" in str(exc), exc
    on, _ = VP.compile_prompt_prefix(pol, _mission(), s, "csp", "a red car")
    assert on["prefix_tokens_exact"] is None and on["prefix_tokens"] > 0
    off, _ = VP.compile_prompt_prefix(pol, _mission(), s, "off", "a red car")
    assert off["prefix_text"] is None


def test_with_openvlas_own_tokenizer_the_neutral_arm_is_the_csps_length():
    """Both arms counted with OpenVLA's tokenizer.json (SKIPPED where it is
    not on this machine: guardrail.csp.openvla_token_counter, OPENVLA_DIR).
    The CSP of the follow policy is 85 OpenVLA tokens; the neutral text is
    padded word by word until it is at least that, so it overshoots by less
    than one word's tokens."""
    from guardrail.csp import openvla_token_counter
    exact = openvla_token_counter()
    if exact is None:
        return SKIP
    pol = load_policy(FOLLOW_POLICY)
    s = VP.slot_for(AERIAL, "a red car")
    assert s.exact_counter is not None and s.exact_counter.name == exact.name
    on, _ = VP.compile_prompt_prefix(pol, _mission(), s, "csp", "a red car")
    nt, _ = VP.compile_prompt_prefix(pol, _mission(), s, "neutral", "a red car")
    csp_n, nt_n = exact.count(on["prefix_text"]), exact.count(nt["prefix_text"])
    assert csp_n == on["prefix_tokens_exact"] == 85, (csp_n, on["prefix_tokens_exact"])
    longest_word = max(exact.count(x) for x in VP.NEUTRAL_SENTENCE.split())
    assert csp_n <= nt_n < csp_n + longest_word, (csp_n, nt_n, nt["prefix_text"])
    lm = nt["length_matched_to"]
    assert (lm["csp_tokens"], lm["neutral_tokens"], lm["token_counter"]) == \
        (csp_n, nt_n, exact.name), lm
    assert nt["prefix_tokens"] == nt_n <= DEFAULT_BUDGET_TOKENS
    assert nt["prefix_token_counter"].startswith("openvla-llama2-sp@")


def test_p0_rules_over_aiotvlas_window_are_refused_never_truncated():
    pol = load_policy(FOLLOW_POLICY)
    s = VP.slot_for(_aiot_identity(), "a red car", counter_factory=lambda t: Words())
    for arm in ("csp", "neutral"):
        exc = _refused(lambda: VP.compile_prompt_prefix(pol, _mission(), s, arm, "a red car"),
                       "csp_budget_exceeded")
        assert exc.facts["budget_tokens"] == 11 and exc.facts["p0_tokens"] > 11, exc.facts
        assert len(exc.facts["p0_ids"]) == 4 and "never truncated" in str(exc)
        # each P0 sentence's own count beside the budget, in the same counter
        each = exc.facts["p0_rules"]
        assert sorted(r["id"] for r in each) == sorted(exc.facts["p0_ids"]), each
        assert sum(r["tokens"] for r in each) == exc.facts["p0_tokens"], each
        assert all(r["fits_alone"] == (r["tokens"] <= 11) for r in each)
        assert "each P0 sentence alone" in str(exc)
    # the off arm needs no room and records the window it was flown in
    off, _ = VP.compile_prompt_prefix(pol, _mission(), s, "off", "a red car")
    assert off["instruction_tokens"] == 3 + 2 and off["slot"]["window_tokens"] == 16


def test_a_policy_the_compiler_refuses_is_a_refused_prefix_not_a_crash():
    pol = load_policy(FOLLOW_POLICY)
    s = VP.slot_for(AERIAL, "a red car")
    exc = _refused(lambda: VP.compile_prompt_prefix(pol, None, s, "csp", "a red car"),
                   "compile_failed")
    assert "Prefix Compiler refused" in str(exc)


def test_a_prefix_that_fits_the_budget_but_not_the_window_is_refused():
    pol = load_policy(FOLLOW_POLICY)
    roomy = VP.PromptSlot(model="aiotvla", placement="t", counter=Words(), budget_tokens=200,
                          window_tokens=60, special_tokens=2, object_tokens=3)
    _refused(lambda: VP.compile_prompt_prefix(pol, _mission(), roomy, "csp", "a red car"),
             "window_exceeded")
    fits = VP.PromptSlot(model="aiotvla", placement="t", counter=Words(), budget_tokens=200,
                         window_tokens=200, special_tokens=2, object_tokens=3)
    rec, _ = VP.compile_prompt_prefix(pol, _mission(), fits, "csp", "a red car")
    assert rec["instruction_tokens"] == rec["prefix_tokens"] + 3 + 2, rec


def test_the_neutral_text_is_rule_free_length_matched_and_bounded():
    w = Words()
    t = VP.neutral_text(14, w)
    assert w.count(t) == 14 and t.startswith(VP.NEUTRAL_SENTENCE)
    assert w.count(VP.neutral_text(14, w, max_tokens=5)) == 5
    try:
        VP.neutral_text(0, w)
    except ValueError:
        pass
    else:
        raise AssertionError("an empty CSP was length-matched")
    import prefix_eval as PE
    assert PE.NEUTRAL_SENTENCE == VP.NEUTRAL_SENTENCE, "one neutral sentence for every arm"


def test_owlvits_own_tokenizer_leaves_11_tokens_and_the_follow_policys_p0_needs_45():
    """With the CLIP tokenizer AIoT-VLA's backbone uses (SKIPPED where it is
    not on this machine: the counter never downloads)."""
    counter = VP.clip_token_counter(OWL)
    if counter is None:
        return SKIP
    s = VP.slot_for(_aiot_identity(), "a red car", counter_factory=lambda t: counter)
    assert (s.object_tokens, s.budget_tokens) == (3, 11), s
    pol = load_policy(FOLLOW_POLICY)
    exc = _refused(lambda: VP.compile_prompt_prefix(pol, _mission(), s, "csp", "a red car"),
                   "csp_budget_exceeded")
    assert exc.facts["p0_tokens"] == 45 and exc.facts["token_counter"].startswith("clip-bpe:")
    # one at a time: alt-band or bld-clearance alone would fit, no two do
    each = {r["id"]: (r["tokens"], r["fits_alone"]) for r in exc.facts["p0_rules"]}
    assert each == {"alt-band": (10, True), "bld-clearance": (9, True),
                    "standoff-any": (12, False), "standoff-pedestrian": (14, False)}, each
    # the server's own window check and slot block, on the same tokenizer
    import vla_server_foreign as F
    tok = counter._tok
    slot = F.aiotvla_csp_slot(tok, OWL, 16)
    assert (slot["text_window_tokens"], slot["special_tokens"], slot["tokenizer"]) == (16, 2, OWL)
    assert F.check_text_window(tok, "a red car", 16) == 5
    try:
        # 14 tokens of the sentence, 3 of the words, 2 special: 19
        F.check_text_window(tok, "Keep at least 10 m away from the pedestrian you are "
                                 "following. a red car", 16)
    except C.CodecError as exc2:
        assert exc2.code == C.ERR_BAD_CSP and "never truncated" in exc2.detail
    else:
        raise AssertionError("an over-long query was not refused")


# ------------------------------------------------------- the AIoT-VLA server
class FakeTok:
    """A tokenizer stand-in: a start and an end token around one per word."""

    def __call__(self, text, add_special_tokens=True):
        ids = list(range(len(text.split())))
        return {"input_ids": ([0] + ids + [1]) if add_special_tokens else ids}


def _server(model):
    import vla_server_foreign as F
    ident = F.foreign_identity("aiotvla", name="v10b", folder="v10b", revision=None,
                               digest="cd" * 32, precision="fp16", attn="sdpa",
                               front_source="vla", facts={"horizon_s": 0.5})
    return F, F.ForeignServer(F.AIOTVLAFamily(horizon_s=0.5), model, ident, None,
                              decode_front=lambda req: type("I", (), {"size": (256, 256)})(),
                              hash_fn=lambda img: "0" * 64, prepare=lambda img: (img, {}))


def _infer(**over):
    r = {"v": 2, "op": "infer", "seq": 3, "t_capture": 50.0,
         "pose": {"x": 0.0, "y": 0.0, "up": 8.0, "yaw_rad": 0.0},
         "front_jpg_b64": base64.b64encode(b"x").decode(), "down_jpg_b64": None,
         "phrase": "straight ahead ", "hint_source": "detector", "object": "a red car",
         "csp_text": None, "csp_mode": "off"}
    r.update(over)
    return r


class Scripted:
    precision, attn, merged = "fp16", "sdpa", False

    def __init__(self, window=None):
        self.calls = []
        self.window = window

    def check_inputs(self, inputs):
        if self.window is not None:
            import vla_server_foreign as F
            F.check_text_window(FakeTok(), inputs["instruction"], self.window)

    def generate(self, image, inputs):
        self.calls.append(dict(inputs))
        return {"raw": {"fwd_m": 0.8, "down_m": 0.0, "dpsi_rad": 0.1, "visible_p": 0.7,
                        "bearing_rad": 0.1, "slots_valid": 1, "history_reset": None},
                "model_s": 0.02, "pre_s": 0.01, "gen_s": 0.01}


def test_aiotvla_reads_the_csp_in_front_of_its_object_words():
    m = Scripted(window=16)
    F, srv = _server(m)
    resp, _, _ = srv.handle(_infer())
    assert resp["ok"], resp.get("error")
    assert m.calls[-1]["instruction"] == "a red car" and "csp_mode" not in m.calls[-1]
    resp, _, _ = srv.handle(_infer(seq=4, t_capture=50.1, csp_mode="csp",
                                   csp_text="Stay below 14 m."))
    assert resp["ok"], resp.get("error")
    got = m.calls[-1]
    assert got["instruction"] == "Stay below 14 m. a red car" and got["object"] == "a red car"
    assert got["csp_mode"] == "csp" and got["csp_sha256"] == VP.sha256_text("Stay below 14 m.")
    assert VP.instruction_text("Stay below 14 m.", "a red car") == got["instruction"]


def test_a_query_over_the_window_is_refused_and_the_model_never_runs():
    m = Scripted(window=16)
    F, srv = _server(m)
    long = "Keep at least 10 m away from the pedestrian you are following."
    resp, _, _ = srv.handle(_infer(csp_mode="csp", csp_text=long))
    assert resp["ok"] is False and resp["error"].startswith(C.ERR_BAD_CSP), resp
    assert "never truncated" in resp["error"] and not m.calls
    # the slot block the hello carries, from the backbone's tokenizer
    slot = F.aiotvla_csp_slot(FakeTok(), OWL, 16)
    assert slot["special_tokens"] == 2 and "never" in slot["truncation"]
    assert F.AIOTVLAFamily.csp_slot is True
    for fam in (F.DroneCATSFamily, F.RaceVLAFamily):
        assert getattr(fam, "csp_slot", False) is False, fam


def test_the_contrast_phrases_stay_the_object_words():
    """AIOTVLAModel.generate looks up the contrast phrases by the object
    words, so a prefix does not switch the memory's contrast off: read from
    its source (the model needs the backbone to run)."""
    import inspect
    import vla_server_foreign as F
    src = inspect.getsource(F.AIOTVLAModel.generate)
    assert 'self.contrast.get(inputs.get("object", phrase)' in src, src[:400]


# -------------------------------------------------------------- the warm-up
def test_the_warm_up_asks_the_flights_prompt():
    import vla_mission as VM
    from guardrail.vla_link import Capture, Hint, Pose

    class Client:
        response_timeout_s = 1.0

        def __init__(self):
            self.sent = []

        def submit(self, fields, response_timeout_s=None):
            self.sent.append(fields)
            return None

    cap = Capture(1.0, Pose(0.0, 0.0, 8.0, 0.0), b"\xff\xd8x\xff\xd9", None)
    c = Client()
    VM.warmup_infer(c, cap, Hint("straight ahead ", "detector"), "a red car", 5.0,
                    csp_mode="csp", csp_text="Stay below 14 m.")
    VM.warmup_infer(c, cap, Hint("straight ahead ", "detector"), "a red car", 5.0)
    assert (c.sent[0]["csp_mode"], c.sent[0]["csp_text"]) == ("csp", "Stay below 14 m.")
    assert (c.sent[1]["csp_mode"], c.sent[1]["csp_text"]) == ("off", None)


# ------------------------------------------------------------ the KPI report
def _ep(i, pair, prefix, *, rail="hil", seed=20261006, model="aerialvla:x", success=True,
        repairs=3, within=0.5, grade=True, policy_hash="sha256:" + "p" * 64,
        scene="CityLife_follow", after=None, run_id=None):
    vp = {"in_ab": True, "arm": {"on": "csp", "off": "off"}.get(prefix, prefix),
          "prefix": prefix, "ab_pair": pair, "object": "a red car", "seed": seed,
          "model": model, "slot": {"model": "aerialvla"},
          "prefix_sha256": None if prefix == "off" else "ab" * 32}
    import kpi_report as KR
    return {"id": run_id or f"run{i}", "kind": "flight", "rail": rail,
            "arms": KR.vla_prefix_arms(vp, policy_hash=policy_hash, scene=scene),
            "prefix": prefix, "mission_success": success, "mission_goal_status": "undeclared",
            "p0_escapes": 0, "repair_count": repairs, "repaired_ticks": repairs,
            "mean_repair_magnitude_mps": 0.5 if repairs else None,
            "follow_within_30m": within,
            "follow_within_30m_after_acquire": within if after is None else after,
            "kpi_grade": grade, "mission_outcome": "success" if success else "target_lost",
            "ticks": 100}


def test_the_report_pairs_the_vla_arms_by_label_and_shows_the_delta():
    import kpi_report as KR
    eps = [_ep(1, "p1", "on", repairs=2, within=0.4), _ep(2, "p1", "off", repairs=6, within=0.6),
           _ep(3, "p2", "off", repairs=4, within=0.7), _ep(4, "p2", "on", repairs=1, within=0.5),
           _ep(5, "p2", "neutral", repairs=5, within=0.6),
           _ep(6, "p3", "on"),                                  # no off arm
           _ep(7, "p4", "on"), _ep(8, "p4", "off", seed=7)]     # arms on two seeds
    ab, para = KR.arms_sections(eps)
    assert ab["status"] == "measured" and ab["pairs"] == 2 and ab["headless_pairs"] == 0, ab
    v = ab["vla_text"]
    assert v["pairs"] == 2 and v["models"] == ["aerialvla:x"] and v["rails"] == ["hil"]
    rep = v["figures"]["measures"]["mean_repair_count_per_episode"]
    assert (rep["on"], rep["off"], rep["delta_on_minus_off"]) == (1.5, 5.0, -3.5), rep
    assert v["follow_within_30m"] == {"on": 0.45, "off": 0.65, "delta_on_minus_off": -0.2}
    assert v["follow_within_30m_after_acquire"] == v["follow_within_30m"]
    assert v["figures"]["aa"]["status"] == "n/m" and "aa_follow" not in v
    # every flight succeeded: mission success cannot separate the arms
    assert v["mission_success"]["separates"] is False and v["mission_success"]["all"] is True
    assert v["neutral"]["pairs"] == 1
    why = {u["id"]: u["why"] for u in v["unpaired"]}
    assert "no off arm" in why["run6"] and "seed" in why["run7"] and "seed" in why["run8"], why
    assert "consumer" not in ab and para["status"] == "n/m"
    md = "\n".join(KR._render_arms(ab, para))
    assert "Flown by a VLA that reads the prefix" in md and "`p1`" in md and "`p2`" in md
    assert "never trained" not in md or "not an expected effect" in md
    assert "Unpaired: `run6`" in md
    # none of it is a headless pair or a text pilot's of the stress harness
    assert all(e["arms"].get("pilot_reads_text") is None for e in eps)


def test_unpaired_vla_flights_are_listed_and_two_ab_pilots_are_never_pooled():
    import kpi_report as KR
    ab, _ = KR.arms_sections([_ep(1, "p1", "on")])
    assert ab["status"] == "n/m" and ab["vla_text_unpaired"][0]["id"] == "run1", ab
    assert "Unpaired VLA prefix-A/B flight `run1`" in "\n".join(KR._render_arms(ab, None))
    consumer = {"id": "c1", "kind": "flight", "scenario_id": "s", "rail": "hil",
                "arms": {"pilot_reads_prefix": True, "prefix": "on"}}
    why = KR.mixed_pilots_refusal([_ep(1, "p1", "on"), consumer])
    assert why and "two prefix A/Bs" in why, why
    assert KR.mixed_pilots_refusal([_ep(1, "p1", "on"), _ep(2, "p1", "off")]) is None
    # nor with the stress harness's text-reading pilot (review of 94a7d7b:
    # the A/B headline would have pooled its pairs with the VLA's), whether
    # its episodes are rail flights of the library or not
    for kind, sid in (("flight", "s"), ("episode", None)):
        reader = {"id": "t1", "kind": kind, "scenario_id": sid, "rail": "hil",
                  "arms": {"pilot_reads_text": True, "prefix": "on", "cell": "c", "seed": 1}}
        why = KR.mixed_pilots_refusal([_ep(1, "p1", "on"), _ep(2, "p1", "off"), reader])
        assert why and "text-reading pilot" in why and "run1" in why, why
    # a stress-harness report without any VLA flight is the one it always was
    assert KR.mixed_pilots_refusal([reader]) is None


def test_the_pair_shares_the_policy_bundle_and_the_scene():
    """Review of 94a7d7b: two arms flown on different bundles, or in
    different scenes, under one --ab-pair label are not a pair."""
    import kpi_report as KR
    eps = [_ep(1, "p1", "on"), _ep(2, "p1", "off", policy_hash="sha256:" + "q" * 64),
           _ep(3, "p2", "on"), _ep(4, "p2", "off", scene="CityLife_other"),
           _ep(5, "p3", "on"), _ep(6, "p3", "off")]
    ab, _ = KR.arms_sections(eps)
    v = ab["vla_text"]
    assert v["pairs"] == 1 and [p["pair"] for p in v["per_pair"]] == ["p3"], v["per_pair"]
    why = {u["id"]: u["why"] for u in v["unpaired"]}
    assert "policy_hash" in why["run1"] and "policy_hash" in why["run2"], why
    assert "scene" in why["run3"] and "scene" in why["run4"], why
    assert "seed fixes the node's own random generators only" in v["note"]
    assert v["per_pair"][0]["scene"] == "CityLife_follow"


def test_a_second_prefix_on_flight_is_the_aa_floor_and_a_failed_follow_is_named():
    """The A/A repeat (run_pas_prefix_ab.sh AA=1: the csp arm flown again in
    its pair) is the noise floor the delta is read against; when every
    flight lost the car, mission success is 0 against 0 and the report says
    to read the repairs, their magnitude and the follow measures instead."""
    import kpi_report as KR
    eps = [_ep(1, "p1", "on", success=False, repairs=2, within=0.3, after=0.4,
               run_id="pab_p1_csp"),
           _ep(2, "p1", "on", success=False, repairs=4, within=0.2, after=0.5,
               run_id="pab_p1_csp_aa"),
           _ep(3, "p1", "off", success=False, repairs=6, within=0.1, after=0.2,
               run_id="pab_p1_off"),
           _ep(4, "p2", "on", success=False, repairs=1, within=0.3, after=0.3,
               run_id="pab_p2_csp"),
           _ep(5, "p2", "off", success=False, repairs=3, within=0.3, after=0.3,
               run_id="pab_p2_off")]
    ab, _ = KR.arms_sections(eps)
    v = ab["vla_text"]
    assert v["pairs"] == 2 and v["repeats_not_counted"] == 0, v
    aa = v["figures"]["aa"]
    assert aa["status"] == "measured" and aa["pairs"] == 1, aa
    fl = aa["measures"]["mean_repair_count_per_episode"]
    assert (fl["first"], fl["repeat"], fl["max_pair_abs_diff"]) == (2.0, 4.0, 2.0), fl
    assert v["aa_follow"]["follow_within_30m_after_acquire"] == {
        "first": 0.4, "repeat": 0.5, "max_pair_abs_diff": 0.1}, v["aa_follow"]
    # the on arm's first flight by id is the counted one, the repeat is not
    rp = v["figures"]["measures"]["mean_repair_count_per_episode"]
    assert (rp["on"], rp["off"]) == (1.5, 4.5), rp
    assert v["follow_within_30m_after_acquire"] == {"on": 0.35, "off": 0.25,
                                                    "delta_on_minus_off": 0.1}
    ms = v["mission_success"]
    assert ms["separates"] is False and ms["all"] is False and "repairs" in ms["why"], ms
    assert [p["aa_flown"] for p in v["per_pair"]] == [True, False]
    md = "\n".join(KR._render_arms(ab, None))
    assert "MISSION SUCCESS DOES NOT SEPARATE THE ARMS" in md, md
    assert "from acquisition: on 0.35" in md and "largest gap in one A/A pair 0.1" in md, md
    # arms that differ in outcome do separate
    ab2, _ = KR.arms_sections([_ep(1, "p1", "on"), _ep(2, "p1", "off", success=False)])
    assert ab2["vla_text"]["mission_success"]["separates"] is True


def test_the_report_reads_the_bundle_scene_and_after_acquire_share_from_the_run():
    """load_run: the manifest's policy hash, sim_config.scene_id and the
    follow's two shares (from take-off, from acquisition) of a VLA prefix
    flight's folder."""
    import kpi_report as KR
    vp = {"in_ab": True, "arm": "off", "prefix": "off", "ab_pair": "pab_p1",
          "object": "a red car", "seed": 5, "model": "aerialvla:x",
          "slot": {"model": "aerialvla"}, "prefix_sha256": None}
    rows = [{"t": round(0.1 * (i + 1), 3), "tick": i + 1, "x": 35.0 + i, "y": -20.0,
             "up": 8.0, "raw": {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
             "emitted": {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
             "violations": [], "emitted_violations": [], "repairs": [], "braked": False}
            for i in range(10)]
    with tempfile.TemporaryDirectory() as d:
        run = Path(d) / "pab_p1_off"
        run.mkdir()
        (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                              encoding="utf-8")
        (run / "metrics.json").write_text(json.dumps({
            "vla_prefix": vp, "frac_within_30m": 0.6, "frac_within_30m_after_acquire": 0.75,
            "sim_config": {"scene_id": "CityLife_follow"}, "shield": "on"}), encoding="utf-8")
        (run / "manifest.json").write_text(json.dumps({
            "code_revision": "abc", "vla_model_hash": "x", "policy_hash": "sha256:" + "e" * 64,
            "random_seed": 5, "sim_speedup": 1.0, "topology": "canonical-hil"}),
            encoding="utf-8")
        e = KR.load_run(run, {}, {})
    a = e["arms"]
    assert (a["policy_hash"], a["scene"], a["vla_reads_prefix"]) == \
        ("sha256:" + "e" * 64, "CityLife_follow", True), a
    assert (e["follow_within_30m"], e["follow_within_30m_after_acquire"]) == (0.6, 0.75), e


def test_a_refused_arm_is_named_with_why_not_as_a_missing_log():
    import kpi_report as KR
    with tempfile.TemporaryDirectory() as d:
        run = Path(d) / "pab_p1_csp"
        run.mkdir()
        (run / KR.VLA_PREFIX_REFUSED).write_text(json.dumps(
            {"arm": "csp", "ab_pair": "pab_p1", "code": "csp_budget_exceeded",
             "why": "csp_budget_exceeded: the 4 P0 rule(s) in scope need 45 tokens"}),
            encoding="utf-8")
        e = KR.load_run(run, {}, {})
        assert "refused before take-off" in e["skipped"] and "45 tokens" in e["skipped"], e
        assert e["prefix_refused"]["code"] == "csp_budget_exceeded"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs the model's tokenizer on this machine)")
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
