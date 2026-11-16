"""The VLA backend table: one protocol, sources cited, and no cell that lies.

Run either way:
    pytest tests/test_vla_backend_table.py -v
    python tests/test_vla_backend_table.py

WHAT IS BEING CLAIMED

tools/vla_backend_table.py fixes one protocol (R1 rate, C1 closed loop),
fills the table only from files that exist, says "source missing" rather than
dropping a row when a file is absent, keeps assisted runs apart from pure ones,
puts an Orin measurement in the Orin column only when the host passed
guardrail.manifest's Orin rule, keeps a CSP-prompt rate apart from the plain
prompt's, chooses no default backend, flags every place that still states the
assisted 0.996 result without its condition, and refuses to load a 7 B model
unless told to.

Protocol 2 (2026-11-16) adds, and the tests at the end hold it to:
  * C2, the Project AirSim follow on hil, with the sentence that it was added
    after the choice it bears on; R1 and C1 are word for word what they were,
    and every row states why C1 was not flown;
  * the AGX Orin benches that are not R1 reach the Orin column each with its
    label (own inputs, count, server in the path), never as "R1", and only
    when the file's own host block passes the Orin rule;
  * a hil follow is scored as C2 only when it meets every condition of C2; a
    stressor, a hot-applied policy, a paraphrase, another feed, a flight not
    flown alone or one that never started is listed with its reason;
  * the 28 October and 4 November matrices are records "not C1, not C2";
  * the default line is "none" until `--default` names a backend the set
    holds an Orin rate and three C2 flights for, with a date after the set;
  * OpenVLA-7B bf16 is timed through deploy/orin/vla_bf16_check.py's loader
    and refused off the GPU or out of bf16; the int8 weight-only key records
    a refusal with its stage and cause instead of a timing.

No test here loads a GPU model. The GPU paths are exercised with fakes that
stand in for exactly the loader and the backend class the flight scripts use,
and the OpenVLA fake refuses an attention mask: passing one crashes OpenVLA's
predict_action (see demo/real_vla_demo.py), and the measurement must call it
the way the flight does.
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import vla_backend_table as V                                       # noqa: E402

SKIP = "SKIP"
PACK = ROOT / "deploy" / "evidence" / "shield_tick_pack.json"
COMMITTED_TABLE = ROOT / "deploy" / "evidence" / "vla_backend_table.json"


def _tmp() -> Path:
    return Path(tempfile.mkdtemp())


# --------------------------------------------------------------------------- #
# the protocol
# --------------------------------------------------------------------------- #
def test_the_protocol_states_inputs_counts_reference_and_nulls():
    p = V.PROTOCOL
    assert p["id"] == V.PROTOCOL_ID
    r1, c1 = p["R1_rate"], p["C1_closed_loop"]
    assert r1["warmup"] == 5 and r1["timed"] == 50
    assert "10 Hz" in r1["reference"] and "stub" in r1["floor"]
    assert "no simulator" in r1["isolation"]
    assert c1["seeds"] >= 5 and c1["reached_radius_m"] > 0
    assert "goal blend 0" in c1["assist"]
    assert any("hover" in n for n in c1["nulls"]) and any("stub" in n for n in c1["nulls"])
    assert (ROOT / "policies" / "urban_demo_policy.yaml").is_file(), \
        "C1 names a policy file that does not exist"


def test_every_named_backend_has_a_row_and_cognitivedrone_says_why_it_is_empty():
    # The grant's list (Architecture constraints p.2-3): "CognitiveDrone,
    # OpenVLA generic, BitVLA, in-house stubs", plus what this project flew.
    assert set(V.BACKENDS) >= {"stub", "bc_v3", "openvla_7b_4bit", "aerialvla_lora",
                               "aerialvla_ft_run2", "cognitivedrone", "bitvla"}
    assert "no checkpoint" in V.BACKENDS["cognitivedrone"]["unavailable"]
    assert "no checkpoint" in V.BACKENDS["bitvla"]["unavailable"]
    for bid in ("cognitivedrone", "bitvla"):
        try:
            V.measure(bid, "dev")
        except ValueError as exc:
            assert "no checkpoint" in str(exc), exc
        else:
            raise AssertionError(f"{bid} produced a measurement from nothing")
    for bid, spec in V.BACKENDS.items():
        code = spec.get("code")
        if code:
            path = code.split(" ")[0]
            assert (ROOT / path).is_file(), f"{bid}: code {path} does not exist"


def test_the_protocol_frames_are_committed_with_their_poses():
    meta = json.loads((V.FRAMES_DIR / "frames.json").read_text(encoding="utf-8"))
    assert len(meta["frames"]) >= 2 and len(meta["target"]) == 2
    for f in meta["frames"]:
        assert (V.FRAMES_DIR / f["file"]).is_file(), f["file"]
        for k in ("x", "y", "psi"):
            assert isinstance(f[k], float), (f["file"], k)


# --------------------------------------------------------------------------- #
# statistics
# --------------------------------------------------------------------------- #
def test_latency_stats_and_the_contract_line():
    s = V.latency_stats([0.05] * 94 + [0.2] * 6)
    assert s["n"] == 100 and math.isclose(s["p50_ms"], 50.0)
    assert math.isclose(s["p95_ms"], 200.0), s
    assert s["meets_contract_p95"] is False
    assert math.isclose(s["hz_at_p50"], 20.0)
    assert V.latency_stats([0.05] * 100)["meets_contract_p95"] is True
    assert V.latency_stats([]) == {"n": 0}


# --------------------------------------------------------------------------- #
# extractors
# --------------------------------------------------------------------------- #
def test_openvla_probe_logs_are_parsed_and_a_missing_one_is_reported():
    d = _tmp()
    (d / "_vla4bit.txt").write_text(
        "Loading checkpoint shards: 100%\nMODEL_LOADED_4BIT 97.6 s VRAM 4.7 GB\n"
        "ACTION [0 0 0 0 0 0 1] infer 1.96 s\nACTION [0 0 0 0 0 0 1] infer 0.86 s\n",
        encoding="utf-8")
    recs = V.extract_openvla_probes(d)
    four = next(r for r in recs if r.get("quantisation") == "4-bit NF4")
    assert four["values_s"] == [1.96, 0.86] and four["load_s"] == 97.6
    assert four["vram_gb"] == 4.7 and math.isclose(four["hz_warm"], 1 / 0.86)
    assert four["protocol"] == "not R1"
    bf16 = next(r for r in recs if "bf16" in r.get("what", ""))
    assert bf16["status"] == "source missing on this host", bf16


def test_flight_rates_are_grouped_by_adapter_and_carry_no_success_figure():
    d = _tmp()
    rows = [{"adapter": "D:/models/aerialvla-lora/aero_vla", "inference_hz": 0.12,
             "n_inference": 15, "terminated_by": "timeout"},
            {"adapter": "D:/models/aerialvla-lora/aero_vla", "inference_hz": 0.15,
             "n_inference": 18, "terminated_by": "timeout"},
            {"adapter": "D:/models/aerialvla-ft/run2/epoch1", "inference_hz": 0.11,
             "n_inference": 13, "terminated_by": "timeout"}]
    (d / "results.jsonl").write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    recs = V.extract_aerialvla_flight_rates(d / "results.jsonl")
    by = {r["backend"]: r for r in recs}
    assert by["aerialvla_lora"]["flights"] == 2 and by["aerialvla_lora"]["hz_max"] == 0.15
    assert by["aerialvla_ft_run2"]["inferences"] == 13
    for r in recs:
        assert "reached" not in r and "no success figure" in r["conditions"]


def test_assisted_evaluations_are_never_reported_as_pure():
    d = _tmp()

    def ev(name, adapter, blend, reached):
        res = [{"reached": i < reached, "params": {"goal_blend": blend, "yaw_gain": 0.4}}
               for i in range(5)]
        (d / f"eval_{name}.json").write_text(json.dumps(
            {"adapter": adapter, "results": res, "mean_interventions": 1.0}), encoding="utf-8")

    ev("assist", "D:/models/aerialvla-ft/run2/epoch1", 0.55, 5)
    ev("pure", "D:/models/aerialvla-ft/run2/epoch1", 0.0, 1)
    recs = {Path(r["source"]["file"]).name: r for r in V.extract_aerialvla_evals(d)}
    assert recs["eval_assist.json"]["assisted"] is True
    assert "ASSISTED" in recs["eval_assist.json"]["conditions"]
    assert recs["eval_pure.json"]["assisted"] is False
    assert recs["eval_pure.json"]["reached"] == 1


def test_a_missing_source_is_a_record_not_a_vanished_row():
    """Silence reads as success: an empty demo/out must leave every row in
    place and say 'source missing', not shrink the table."""
    empty = _tmp()
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty,
                      evidence=empty)
    assert [r["backend"] for r in t["rows"]] == list(V.BACKENDS)
    ov = next(r for r in t["rows"] if r["backend"] == "openvla_7b_4bit")
    assert any(rec.get("status") == "source missing on this host" for rec in ov["records"])
    stub = next(r for r in t["rows"] if r["backend"] == "stub")
    assert all(rec.get("status") for rec in stub["records"]), stub["records"]
    assert all(w["present"] is False for w in ov["weights_status"])


def test_the_claim_check_finds_the_contradiction_and_drops_it_once_corrected():
    root = _tmp()
    (root / "docs").mkdir()
    (root / "training").mkdir()
    (root / "training" / "e.json").write_text(json.dumps(
        {"results": [{"params": {"goal_blend": 0.55}}]}), encoding="utf-8")
    check = [{"doc": "docs/r.md", "must_contain": ["0.996", "goal assist off"],
              "file": "training/e.json", "field": "goal_blend", "why": "assist was on"}]
    (root / "docs" / "r.md").write_text("| x | 100 % reached, 0.996, goal assist off |\n",
                                        encoding="utf-8")
    found = V.check_claims(check, root=root)
    assert len(found) == 1 and found[0]["doc"] == "docs/r.md:1", found
    (root / "docs" / "r.md").write_text("| x | 100 % reached, 0.996, goal blend 0.55 |\n",
                                        encoding="utf-8")
    assert V.check_claims(check, root=root) == []


def test_an_unqualified_0996_is_flagged_and_a_qualified_one_is_not():
    """The mid-term deck says '100 % reached, efficiency 0.996' with no
    condition at all; that hides the assist as surely as 'goal assist off'."""
    root = _tmp()
    (root / "tools").mkdir()
    (root / "training").mkdir()
    (root / "training" / "e.json").write_text(json.dumps(
        {"results": [{"params": {"goal_blend": 0.55}}]}), encoding="utf-8")
    check = [c for c in V.CLAIM_CHECKS if c["doc"].endswith("build_midterm_deck.js")]
    assert len(check) == 1
    check = [dict(check[0], doc="tools/deck.js", file="training/e.json")]
    deck = root / "tools" / "deck.js"
    deck.write_text('["Ours", "Expert flights", "100 % reached, efficiency 0.996"],\n',
                    encoding="utf-8")
    assert len(V.check_claims(check, root=root)) == 1
    deck.write_text('["Ours", "Expert flights", "5/5 reached, efficiency 0.996, with '
                    'mission-direction assist (goal_blend 0.55)"],\n', encoding="utf-8")
    assert V.check_claims(check, root=root) == []


def test_the_claim_checks_cover_the_report_its_generator_and_the_deck():
    docs = {c["doc"] for c in V.CLAIM_CHECKS}
    assert {"docs/MIDTERM-REPORT-Aug2026.md", "tools/build_report_results.py",
            "tools/deck/build_midterm_deck.js"} <= docs, docs


def test_the_correction_carries_its_null_from_the_files():
    """5/5 with the assist does not show the fine-tune helped: the adapter
    before fine-tuning also reached 5/5 under the same assist. The correction
    must say so, with the numbers read from the evaluation files."""
    files = {Path(f).name: V.eval_summary(ROOT / f) for f in V.ASSIST_EVIDENCE}
    if any(v is None for v in files.values()):
        return SKIP
    ft2, base = files["eval_ft2_deploy.json"], files["eval_base_deploy.json"]
    assert ft2["assisted"] and base["assisted"] and ft2["goal_blend"] == base["goal_blend"]
    assert (ft2["reached"], ft2["episodes"]) == (5, 5) and ft2["mean_efficiency"] == 0.996
    assert (base["reached"], base["episodes"]) == (5, 5) and base["mean_efficiency"] == 0.942
    pure = files["eval_ft2_e1.json"], files["eval_ft2_yg04.json"], files["eval_baseline.json"]
    assert [(p["reached"], p["assisted"]) for p in pure] == [(0, False), (1, False), (1, False)]
    assert files["eval_ft3_deploy.json"]["reached"] == 5
    # Each figure must sit next to the file it comes from, so dropping the
    # null sentence (or quoting a number without its source) fails here.
    why = V.CLAIM_CHECKS[0]["why"]
    want = [
        f"5/5 reached and {ft2['mean_efficiency']} efficiency ({ft2['file']})",
        f"also reached {base['reached']}/{base['episodes']} "
        f"(efficiency {base['mean_efficiency']}, {base['file']})",
        f"also reached {files['eval_ft3_deploy.json']['reached']}/5 "
        f"({files['eval_ft3_deploy.json']['file']})",
        f"reached {pure[0]['reached']}/5 at yaw gain 1.0 ({pure[0]['file']})",
        f"{pure[1]['reached']}/5 at yaw gain 0.4 ({pure[1]['file']})",
        f"original adapter {pure[2]['reached']}/5 ({pure[2]['file']})",
        "goal_blend 0.55",
    ]
    for s in want:
        assert s in why, f"the correction no longer says: {s}"
    assert all(c["why"] == why for c in V.CLAIM_CHECKS), "the three checks disagree"


# --------------------------------------------------------------------------- #
# measurement routing and refusals
# --------------------------------------------------------------------------- #
def _rate(is_orin: bool, backend: str = "stub", csp: str | None = None) -> dict:
    out = {"schema": V.RATE_SCHEMA, "backend": backend, "protocol": V.PROTOCOL_ID,
           "host": {"jetson": {"is_orin": is_orin}},
           "stats": V.latency_stats([0.002] * 50)}
    if csp:
        out["prompt_arm"] = {"csp": csp}
    return out


def test_an_orin_measurement_lands_in_the_orin_column_only():
    ev = _tmp()
    (ev / "hil").mkdir()
    (ev / "hil" / "vla_rate_stub.json").write_text(json.dumps(_rate(True)), encoding="utf-8")
    empty = _tmp()
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty, evidence=ev)
    stub = next(r for r in t["rows"] if r["backend"] == "stub")
    assert stub["cells"]["rate_orin"].startswith("R1:"), stub["cells"]
    assert not stub["cells"]["rate_desktop"].startswith("R1:"), stub["cells"]
    (ev / "hil" / "vla_rate_stub.json").write_text(json.dumps(_rate(False)), encoding="utf-8")
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty, evidence=ev)
    stub = next(r for r in t["rows"] if r["backend"] == "stub")
    assert stub["cells"]["rate_orin"] == "to measure on the Orin (R1)", stub["cells"]
    assert stub["cells"]["rate_desktop"].startswith("R1:")


def test_a_gpu_backend_is_refused_without_allow_gpu():
    for b in V.GPU_BACKENDS:
        try:
            V.measure(b, "dev", model_root=Path("/models"))
        except PermissionError as exc:
            assert "--allow-gpu" in str(exc)
            continue
        raise AssertionError(f"{b} would have loaded a 7 B model without --allow-gpu")


def test_a_hil_label_is_refused_off_an_orin():
    import profile_shield_tick as P
    if P.host_facts()["jetson"]["is_orin"]:
        return SKIP
    try:
        V.measure("stub", "hil")
    except PermissionError as exc:
        assert "Jetson Orin" in str(exc)
    else:
        raise AssertionError("a desktop rate was labelled hil")


def test_no_default_backend_is_chosen_before_the_measurement_set():
    """The grant: the project "does not commit to any one of them as a
    'default' until the Q2 measurement set lands". The builder itself, not
    only the committed JSON, must say none."""
    empty = _tmp()
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty,
                      evidence=empty)
    assert t["default_backend"] is None, t["default_backend"]
    assert "Orin columns are empty" in t["decision"], t["decision"]
    assert "KPI runs" not in t["decision"] and "not KPI-bearing" in t["decision"]
    assert "not KPI-bearing" in V.BACKENDS["stub"]["role"]


def test_a_csp_prompt_rate_is_never_shown_as_the_plain_prompts():
    ev = _tmp()
    (ev / "dev").mkdir()
    (ev / "dev" / "vla_rate_openvla_7b_4bit-csp.json").write_text(
        json.dumps(_rate(False, "openvla_7b_4bit", csp="on")), encoding="utf-8")
    empty = _tmp()
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty, evidence=ev)
    ov = next(r for r in t["rows"] if r["backend"] == "openvla_7b_4bit")
    assert ov["cells"]["rate_desktop"].startswith("R1, CSP prompt:"), ov["cells"]
    (ev / "dev" / "vla_rate_openvla_7b_4bit.json").write_text(
        json.dumps(_rate(False, "openvla_7b_4bit", csp="off")), encoding="utf-8")
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty, evidence=ev)
    ov = next(r for r in t["rows"] if r["backend"] == "openvla_7b_4bit")
    cell = ov["cells"]["rate_desktop"]
    assert cell.startswith("R1: ") and "R1, CSP prompt:" in cell, cell


def test_a_bc_policy_that_lands_on_the_gpu_is_refused_not_timed():
    class Param:
        def __init__(self, device):
            self.device = device

        def numel(self):
            return 10

    class Model:
        def __init__(self, devs):
            self.devs = devs

        def parameters(self):
            return [Param(d) for d in self.devs]

    class FakeBC:
        def __init__(self, devs):
            self.model = Model(devs)

        def act(self, state):
            return None

    try:
        V.make_cpu_backend("bc_v3", bc_loader=lambda m, p: FakeBC(["cuda:0"]))
    except RuntimeError as exc:
        assert "refusing to time it as a CPU run" in str(exc), exc
    else:
        raise AssertionError("a bc_v3 model on cuda:0 was timed as a CPU run")
    b, extra = V.make_cpu_backend("bc_v3", bc_loader=lambda m, p: FakeBC(["cpu"]))
    assert extra["param_devices"] == ["cpu"] and extra["params"] == 10


class _FakeImg:
    def crop(self, box):
        return self

    def resize(self, size):
        return self


class _FakeInputs(dict):
    def to(self, *a, **kw):
        return self


def test_openvla_measurement_calls_predict_action_the_way_the_flight_does():
    calls = []

    class FakeModel:
        def predict_action(self, **kw):
            assert "attention_mask" not in kw, "attention_mask crashes OpenVLA generate"
            assert set(kw) == {"input_ids", "pixel_values", "unnorm_key", "do_sample"}, kw
            calls.append(kw)
            n = len(calls)
            return [float("nan")] * 7 if n == 9 else [0.0] * 6 + [1.0]

    class FakeTorch:
        bfloat16 = "bf16"

        class _NG:
            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

        def no_grad(self):
            return self._NG()

    seen_prompts = []

    def proc(prompt, img):
        assert "What action should the robot take" in prompt, prompt
        seen_prompts.append(prompt)
        return _FakeInputs(input_ids=[1, 2], pixel_values=[0], attention_mask=[1, 1])

    t = [0.0]

    def clock():
        t[0] += 0.25
        return t[0]

    meta = {"frames": [{"file": "a.png"}, {"file": "b.png"}]}
    res = V.measure_openvla(Path("/models"), warmup=2, timed=10,
                            loader=lambda: (proc, FakeModel(), FakeTorch()),
                            frames=([_FakeImg(), _FakeImg()], meta), clock=clock)
    import real_vla_demo as RV
    assert RV.MODEL_ID.replace("\\", "/").endswith("/models/openvla-7b"), RV.MODEL_ID
    assert len(calls) == 12 and len(res["latency_s"]) == 10
    assert all(math.isclose(v, 0.25) for v in res["latency_s"]), res["latency_s"]
    assert res["undecodable_outputs"] == 1, "a NaN action was not counted"
    # The flight script's default prompt (--csp off), recorded with its length.
    assert res["prompt_arm"] == {"csp": "off"} and res["prompt"] == RV.build_openvla_prompt(
        "fly forward and avoid restricted areas")
    assert set(seen_prompts) == {res["prompt"]}
    assert res["prompt_tokens"]["n"] > 0 and res["prompt_tokens"]["method"], res["prompt_tokens"]


def test_the_csp_arm_times_the_longer_prompt_the_flight_script_builds():
    import real_vla_demo as RV

    class FakeModel:
        def predict_action(self, **kw):
            return [0.0] * 6 + [1.0]

    class FakeTorch:
        bfloat16 = "bf16"

        class _NG:
            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

        def no_grad(self):
            return self._NG()

    seen = []

    def proc(prompt, img):
        seen.append(prompt)
        return _FakeInputs(input_ids=[1], pixel_values=[0])

    meta = {"frames": [{"file": "a.png"}]}
    kw = dict(warmup=1, timed=3, loader=lambda: (proc, FakeModel(), FakeTorch()),
              frames=([_FakeImg()], meta))
    off = V.measure_openvla(Path("/models"), **kw)
    on = V.measure_openvla(Path("/models"), csp="on", **kw)
    assert on["prompt_arm"]["csp"] == "on" and on["prompt"] != off["prompt"]
    assert len(on["prompt"]) > len(off["prompt"])
    assert on["prompt"].endswith("Out:") and off["prompt"].endswith("Out:")
    assert on["prompt_tokens"]["n"] > off["prompt_tokens"]["n"], (on["prompt_tokens"],
                                                                 off["prompt_tokens"])
    assert on["prompt_arm"]["csp_mission"] == RV.DEFAULT_MISSION
    try:
        V.measure("stub", "dev", csp="on")
    except ValueError as exc:
        # since 2026-11-16 the prompt arm is every OpenVLA-7B key's (4-bit,
        # bf16, the int8 attempt): the refusal names them
        assert "the OpenVLA-7B keys only" in str(exc) and "openvla_7b_4bit" in str(exc), exc
    else:
        raise AssertionError("--csp on was accepted for a backend that reads no prompt")


def test_aerialvla_measurement_reads_the_backends_own_inference_log():
    seen = {}

    class FakeAerial:
        def __init__(self, obj, target, obs_factory=None, lora_id=None, inference_log=None):
            seen.update(obj=obj, target=target, lora=lora_id)
            self.get = obs_factory()
            self.log = Path(inference_log)
            self._stop = False

        def start(self):
            def run():
                k = 0
                while not self._stop and k < 30:
                    front, down, pose = self.get()
                    seen.setdefault("poses", []).append(pose)
                    with self.log.open("a", encoding="utf-8") as fh:
                        fh.write(json.dumps({"seq": k + 1, "cycle_s": 0.5, "gen_s": 0.4,
                                             "new_tokens": 11}) + "\n")
                    k += 1
                    time.sleep(0.001)
            self.th = threading.Thread(target=run, daemon=True)
            self.th.start()

        def stop(self):
            self._stop = True
            self.th.join(timeout=2)

    meta = {"target": [38.0, 26.0],
            "frames": [{"file": "a.png", "x": 1.0, "y": 2.0, "psi": 0.5},
                       {"file": "b.png", "x": 3.0, "y": 4.0, "psi": 1.5}]}
    res = V.measure_aerialvla(Path("/models"), "aerialvla-ft/run2/epoch1", warmup=2, timed=6,
                              backend_cls=FakeAerial,
                              frames=([_FakeImg(), _FakeImg()], meta), timeout_s=10)
    assert seen["target"] == (38.0, 26.0) and seen["obj"] == "orange barrier"
    assert seen["lora"].replace("\\", "/").endswith("aerialvla-ft/run2/epoch1")
    assert seen["poses"][:2] == [(1.0, 2.0, 0.5), (3.0, 4.0, 1.5)]
    assert res["complete"] and len(res["latency_s"]) == 6
    assert res["undecodable_outputs"] is None and "undecodable_outputs_upper_bound" in res


def test_the_stub_is_measured_on_the_protocol_states():
    if not PACK.is_file():
        return SKIP
    res = V.measure("stub", "dev")
    assert res["schema"] == V.RATE_SCHEMA and res["test"] == "R1"
    assert res["stats"]["n"] == 50 and res["inputs"]["flight"] == "ros2_shield_on"
    assert res["stats"]["meets_contract_p95"] is True


def test_bc_v3_is_measured_on_the_cpu_only():
    if not (PACK.is_file() and (ROOT / "models" / "vla_policy_v2.pt").is_file()):
        return SKIP
    try:
        import torch  # noqa: F401
    except ImportError:
        return SKIP
    res = V.measure("bc_v3", "dev")
    assert res["param_devices"] == ["cpu"], res["param_devices"]
    assert res["stats"]["n"] == 50


# --------------------------------------------------------------------------- #
# the AerialVLA serving configurations (wave V1, 2026-10-21)
# --------------------------------------------------------------------------- #
def _bench(precision, attn, merged, p50, vram, parse_fail=0, bins=(96, 48, 49),
           smi_after=9000, smi_before=None, reserved=None, load=None):
    outs = [{"frame": f, "phrase": p, "raw_text": "x", "parse_ok": True, "bins": list(bins),
             "land": False} for f in ("a.png", "b.png") for p in ("", "straight ahead ")]
    v = {"inference_peak_allocated_gb": vram, "load_peak_allocated_gb": vram + 1,
         "nvidia_smi_used_mib_after_timed": smi_after}
    if smi_before is not None:
        v.update(nvidia_smi_used_mib_before_load=smi_before,
                 process_delta_mib=smi_after - smi_before)
    if reserved is not None:
        v["inference_peak_reserved_gb"] = reserved
    return {"config": {"precision": precision, "attn": attn, "merged": merged},
            "identity": {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
                         "weights_sha256": "ab" * 32},
            "created_utc": "2026-10-21T13:40:47Z",
            "facts": {"load": {"load_total_s": 60.0} if load is None else load,
                      "libraries": {"torch": "2.6.0"},
                      "host": {"host_arch": "AMD64", "hostname": "lab-desk-42"}},
            "warmup": 5, "timed": 50,
            "stats_latency": {"n": 50, "p50_ms": p50, "p95_ms": p50 * 1.1, "max_ms": p50 * 1.2,
                              "mean_ms": p50, "hz_at_p50": 1000 / p50},
            "stats_gen": {"p50_ms": p50 * 0.9}, "stats_server": {"p50_ms": p50 * 1.05},
            "stats_probe_latency": {"p50_ms": p50, "p95_ms": p50 * 1.2},
            "new_tokens": [9], "parse_fail_timed": parse_fail, "parse_fail_probe": 0,
            "land_probe": 0, "probe_outputs": outs, "timed_outputs": outs[:2],
            "vram": v}


def test_the_low_vram_option_follows_the_fixed_rule_and_nothing_is_a_default():
    """Decided 2026-10-22 (review of V1a): bf16 reproduces the reference,
    nf4 agrees on only 25-33 %, so the summary names OPTIONS - the low-VRAM
    row by a rule fixed here, the row closest to the reference, the fastest -
    and records that there is no default (the server refuses to load without
    an explicit configuration); the CityLife VRAM measurement decides."""
    benches = [_bench("bf16", "eager", False, 400.0, 15.5, smi_before=3000, smi_after=19400),
               _bench("bf16", "sdpa", False, 380.0, 15.5, smi_before=3000, smi_after=19400),
               _bench("bf16", "sdpa", True, 300.0, 15.6, bins=(96, 48, 50),  # fastest, too big
                      smi_before=3000, smi_after=19000),
               _bench("nf4", "sdpa", True, 520.0, 5.0, bins=(95, 48, 49),
                      smi_before=3000, smi_after=10500),
               _bench("nf4", "eager", False, 450.0, 5.5, parse_fail=1,       # a parse failure
                      smi_before=3000, smi_after=10000),
               _bench("nf4", "eager", True, 1300.0, 5.0,                     # misses the gate
                      smi_before=3000, smi_after=10500)]
    s = V.summarise_serving(benches, vram_budget_gb=10.0)
    assert s["schema"] == V.SERVING_SCHEMA and s["gate_p50_ms"] == 1000.0
    # The committed summary describes the host by hardware, not by its name.
    assert s["host"]["host"] == {"host_arch": "AMD64"} and "lab-desk-42" not in json.dumps(s)
    assert s["fastest"] == "bf16/sdpa/merged" and s["low_vram"] == "nf4/sdpa/merged", s
    assert s["low_vram_config"] == {"precision": "nf4", "attn": "sdpa", "merged": True}
    assert s["fidelity"] == "bf16/sdpa/unmerged", s      # the reference itself is not an option
    assert s["default"] is None and "recommended" not in s and "recommended_config" not in s
    assert "CityLife" in s["choice"] and "--precision" in s["choice"]
    assert s["measured_utc"] == ["2026-10-21T13:40:47Z", "2026-10-21T13:40:47Z"]
    rows = {V._cfg_name(c): c for c in s["configs"]}
    assert rows["nf4/eager/merged"]["meets_gate"] is False
    assert rows["bf16/sdpa/unmerged"]["agreement_probe"] == {"n": 4, "exact": 1.0, "close": 1.0}
    nf4 = rows["nf4/sdpa/merged"]["agreement_probe"]
    assert nf4["exact"] == 0.0 and nf4["close"] == 1.0, nf4        # one forward bin off
    assert V.summarise_serving(benches, vram_budget_gb=4.0)["low_vram"] is None
    md = "\n".join(V.render_serving(s))
    assert "| nf4 | sdpa | merged | 520 |" in md and "No serving default." in md, md
    assert "Low-VRAM option:** nf4/sdpa/merged" in md and "allocated by torch" in md
    assert "Recommended" not in md and "not the project's default backend" in md
    cell = V.serving_r1_cell(s)
    assert cell.startswith("R1 via demo/vla_server_v2.py, nf4/sdpa/merged") and "p50 520 ms" in cell
    assert "low-VRAM option" in cell and "default" in cell and "0% of" in cell, cell


def test_the_vram_budget_is_applied_to_what_the_process_holds_not_to_torch_allocated():
    """Review of V1a (P5): the budget rule compared torch's
    max_memory_allocated, which leaves out the allocator cache and the CUDA
    context. On the RTX 4090 the nf4 merged process took about 7.9 GB by
    nvidia-smi (desktop subtracted) against the 5.4 GB reported. The rule
    now reads the process: nvidia-smi after minus before the load when the
    bench recorded both, else the card minus a declared desktop baseline,
    else torch's reserved peak; with none of them the row is not eligible.
    And a missing load time is None, not 0.0."""
    MIB = 1048576 / 1e9
    tight = [_bench("bf16", "eager", False, 400.0, 15.5, smi_before=3000, smi_after=19400),
             # 5.0 GB allocated, but the process holds (9900 - 3000) MiB = 7.24 GB
             _bench("nf4", "sdpa", True, 520.0, 5.0, smi_before=3000, smi_after=9900)]
    s = V.summarise_serving(tight, vram_budget_gb=6.0)
    row = next(c for c in s["configs"] if V._cfg_name(c) == "nf4/sdpa/merged")
    assert row["vram_process_gb"] == round(6900 * MIB, 2) and "before the load" in \
        row["vram_process_basis"], row
    assert s["low_vram"] is None, "the budget was applied to torch's allocated peak"
    assert V.summarise_serving(tight, vram_budget_gb=8.0)["low_vram"] == "nf4/sdpa/merged"
    # a bench without a per-run baseline: the declared desktop baseline (GB)
    legacy = [_bench("nf4", "sdpa", True, 520.0, 5.0, smi_after=10760)]
    s2 = V.summarise_serving(legacy, vram_budget_gb=10.0, desktop_baseline_gb=3.4)
    r2 = s2["configs"][0]
    assert r2["vram_process_gb"] == round(10760 * MIB - 3.4, 2) and "3.4 GB" in \
        r2["vram_process_basis"], r2
    assert s2["desktop_baseline_gb"] == 3.4
    # torch's reserved peak, a lower bound, when nvidia-smi is missing
    r3 = V.summarise_serving([_bench("nf4", "sdpa", True, 520.0, 5.0, smi_after=None,
                                     reserved=5.9)], vram_budget_gb=10.0)["configs"][0]
    assert r3["vram_process_gb"] == 5.9 and "reserved" in r3["vram_process_basis"], r3
    # nothing but the allocated peak: not measured, not eligible
    s4 = V.summarise_serving([_bench("nf4", "sdpa", True, 520.0, 5.0, smi_after=None)],
                             vram_budget_gb=10.0)
    assert s4["configs"][0]["vram_process_gb"] is None and s4["low_vram"] is None
    assert s4["configs"][0]["vram_inference_peak_reserved_gb"] is None
    # a missing load time is unknown, not instantaneous
    s5 = V.summarise_serving([_bench("nf4", "sdpa", True, 520.0, 5.0, load={"merge_s": 6.0})],
                             vram_budget_gb=10.0)
    assert s5["configs"][0]["load_s"] is None, s5["configs"][0]


def test_the_serving_section_reaches_the_table_only_when_it_exists():
    empty = _tmp()
    side = empty / "serving.json"
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty, evidence=empty,
                      serving_json=side)
    assert t["aerialvla_serving"] is None and "AerialVLA serving" not in V.render_markdown(t)
    s = V.summarise_serving([_bench("bf16", "eager", False, 400.0, 15.5, smi_before=3000),
                             _bench("nf4", "sdpa", True, 520.0, 5.0, smi_before=3000)],
                            vram_budget_gb=10.0)
    side.write_text(json.dumps(s), encoding="utf-8")
    t = V.build_table(demo_out=empty, experiments_out=empty, model_root=empty, evidence=empty,
                      serving_json=side)
    row = next(r for r in t["rows"] if r["backend"] == "aerialvla_lora")
    assert row["cells"]["rate_desktop"].startswith("R1 via demo/vla_server_v2.py")
    assert t["default_backend"] is None, "a serving default is not the backend default"
    assert "## AerialVLA serving configurations" in V.render_markdown(t)


def test_the_committed_serving_summary_is_in_the_committed_table():
    if not (V.SERVING_JSON.is_file() and COMMITTED_TABLE.is_file()):
        return SKIP
    s = json.loads(V.SERVING_JSON.read_text(encoding="utf-8"))
    t = json.loads(COMMITTED_TABLE.read_text(encoding="utf-8"))
    assert s["schema"] == V.SERVING_SCHEMA and t["aerialvla_serving"] == V.serving_for_table(s)
    assert all("probe_outputs" not in c for c in t["aerialvla_serving"]["configs"])
    assert all(len(c["probe_outputs"]) == c["probe_n"] > 0 for c in s["configs"]), \
        "the probe texts behind the agreement column are not in the serving file"
    md = (ROOT / "deploy" / "evidence" / "vla_backend_table.md").read_text(encoding="utf-8")
    assert md == V.render_markdown(t) + "\n", "the committed md is not its JSON re-rendered"
    for c in s["configs"]:
        assert c["n"] >= 30 and c["meets_gate"] == (c["p50_ms"] <= s["gate_p50_ms"]), c


# --------------------------------------------------------------------------- #
# the committed table
# --------------------------------------------------------------------------- #
def test_the_committed_table_still_matches_the_files_it_cites():
    if not COMMITTED_TABLE.is_file():
        return SKIP
    t = json.loads(COMMITTED_TABLE.read_text(encoding="utf-8"))
    assert t["schema"] == V.TABLE_SCHEMA and t["default_backend"] is None
    assert "KPI runs" not in json.dumps(t), "the dev-rail runs are not KPI-bearing"
    assert [r["backend"] for r in t["rows"]] == list(V.BACKENDS)
    checked = 0
    for r in t["rows"]:
        for rec in r["records"]:
            src = rec.get("source") or {}
            f = src.get("file", "").split(":")[0]
            if not (src.get("sha256") and (ROOT / f).is_file()):
                continue
            # Run folders are gitignored and other tools rewrite their summaries
            # (rescore_kpis.py); only tracked sources must still match. A stale
            # untracked one is printed, so it is seen, and fixed by re-running
            # `python tools/vla_backend_table.py table`.
            if f.startswith(("demo/out/", "experiments/out/")):
                if V._sha256(ROOT / f) != src["sha256"]:
                    print(f"      note: {f} changed since the table was built")
                continue
            assert V._sha256(ROOT / f) == src["sha256"], \
                f"{f} changed since the table: re-run `python tools/vla_backend_table.py table`"
            checked += 1
        for res in r.get("r1_results") or []:
            assert res["schema"] == V.RATE_SCHEMA, (r["backend"], res["schema"])
            col = "rate_orin" if res["host"]["jetson"]["is_orin"] else "rate_desktop"
            if res.get("status") == "refused":
                assert r["cells"][col].startswith("attempted"), (r["backend"], col, r["cells"])
                continue
            assert r["cells"][col].startswith("R1"), (r["backend"], col, r["cells"])
    assert checked > 0 or not (ROOT / "training" / "eval_ft2_deploy.json").is_file()


# --------------------------------------------------------------------------- #
# protocol 2 (2026-11-16): C2, the Orin benches, the matrices, the default line
# --------------------------------------------------------------------------- #
HIL_EVIDENCE = V.EVIDENCE / "hil"
T0_15_NOV = 1794750528.0                 # pas_hil_feed_07's own mission start (UTC 2026-11-15)
T0_UTC_16_NOV_2330 = 1794871800.0        # 2026-11-16 23:30 UTC = 2026-11-17 07:30 in the lab


def _build(**kw):
    """build_table with every folder empty unless named."""
    empty = _tmp()
    args = dict(demo_out=empty, experiments_out=empty, model_root=empty, evidence=empty,
                docs_data=empty, bundles=empty)
    args.update(kw)
    return V.build_table(**args)


def _row(t: dict, bid: str) -> dict:
    return next(r for r in t["rows"] if r["backend"] == bid)


def _pas_run(root: Path, tag: str, *, model="aiotvla/v10b@35259bd1", feed="stream", cap=0.0,
             kpi_grade=True, topology="hil", stress=None, hot=False, identity=True,
             estimator=True, reached=True, ticks=4198, end="max_s reached", w30=0.3523,
             dec_hz=10.93, interventions=284, p0=0, pilot_kind="vla",
             rail="projectairsim-ardupilot", mission_s=420.17,
             negq=("a traffic light",), capture=0.05,
             bundle="follow-car-citylife-v0.1.3.tar.gz", max_s=420.0, outcome="fail",
             fail_reasons=("unsafe_position_entered",), unsafe=3, t0=T0_15_NOV) -> Path:
    """A perception-rail follow's run folder with the fields the C2 reader
    reads, shaped as the node writes them (metrics.json, kpi.json; the
    defaults are pas_hil_feed_07's). `max_s=None` writes no mission-monitor
    limit, as a run with no VLA mission block has none; `negq`, `capture`
    or `bundle` None leaves that record out."""
    d = root / tag
    d.mkdir(parents=True)
    feed_params = {} if feed is None else {"vla_feed": feed, "vla_feed_max_hz": cap}
    if negq is not None:
        feed_params["negative_query"] = list(negq)
    if capture is not None:
        feed_params["front_capture_interval_s"] = capture
    m = {"tag": tag, "rail": rail, "topology": topology, "grant_topology": "hil",
         "ticks": ticks, "end_reason": end, "mission_s": mission_s if ticks else 0.0,
         "frac_within_30m": w30 if ticks else None,
         "interventions": interventions if ticks else None, "det_hz": 2.49,
         "perception_stress": {"name": stress} if stress else None,
         "hot_apply": {"n_applied": 2} if hot else None,
         "mission": {"task": "follow a red car"}, "start_gate": {"reached": reached},
         "params": {"pilot": "vla", "mission": "follow", "identity": identity,
                    "target_estimator": estimator, "stress": stress, **feed_params},
         "pilot": {"kind": pilot_kind,
                   "server_identity": {"id": model, "precision": "bf16"},
                   "summary": {"decision_hz": dec_hz, "answers_by_kind": {"step": 4648},
                               "parse_fail": 0}},
         "vla_observation": {"asking_s": 353.393, "t0_wall": t0}}
    if bundle is not None:
        m["policy_source"] = {"kind": "bundle", "default_bundle": True,
                              "path": f"/home/nathan-agxiot/vla-drone/bundles/{bundle}"}
    if max_s is not None and ticks:
        m["vla"] = {"mission": {"monitor": {"config": {"mission": "follow", "max_s": max_s}}}}
    k = {"kpi_grade": kpi_grade, "p0_escapes": p0, "p0_violation_escape_rate": 0.0,
         "outcome": outcome, "mission_fail_reasons": list(fail_reasons or ()),
         "unsafe_p0_ticks": unsafe if ticks else 0, "p0_ticks_not_measurable": 0,
         "p0_ticks_blind": None,
         "manifest": {"code_revision": "deac4b3a5b16", "topology": topology}}
    (d / "metrics.json").write_text(json.dumps(m), encoding="utf-8")
    (d / "kpi.json").write_text(json.dumps(k), encoding="utf-8")
    return d


def test_protocol_2_keeps_r1_and_c1_and_says_c2_came_after_the_choice():
    p = V.PROTOCOL
    assert V.PROTOCOL_ID == p["id"] == "vla-backend-protocol/2" and p["date"] == V.PROTOCOL_DATE
    # R1 and C1 are the tests of protocol 1
    assert (p["R1_rate"]["warmup"], p["R1_rate"]["timed"]) == (5, 50)
    c1 = p["C1_closed_loop"]
    assert (c1["seeds"], c1["time_limit_s"], c1["reached_radius_m"]) == (5, 120, 3.0)
    assert "fly to (40, 40) at 6 m/s altitude 20" in c1["scenario"]
    assert "unchanged" in p["changes_from_1"]
    c2 = p["C2_follow_closed_loop"]
    said = c2["added_after_the_choice"]
    assert c2["added"] == V.PROTOCOL_DATE == "2026-11-16"
    assert "after the lab named AIoT-VLA v10b" in said and "15 November 2026" in said
    assert "follow-only model cannot fly" in said and "not a prediction" in said
    # never the PI's: the choice is the lab's, in this tool's words
    assert "PI" not in said and "accepted" not in said and "approved" not in said
    outs = " ".join(c2["outputs"])
    for needed in ("within", "30 m", "decision_hz_mission", "interventions_per_100_ticks",
                   "p0_escapes"):
        assert needed in outs, needed
    assert "hil" in c2["rail"] and "MAVROS 2" in c2["rail"] and "Jetson Orin" in c2["rail"]
    assert c2["flights_per_backend"] == 3
    assert (ROOT / V.PAS_LAUNCHER).is_file() and V.PAS_LAUNCHER in c2["launch"]["script"]
    assert c2["launch"]["node_args"] == ["--identity", "--negative-query", "a traffic light",
                                         "--target-estimator"]
    assert any("servo" in n for n in c2["nulls"])
    md = V.render_markdown(_build())
    assert said in md and "## C2 on hil" in md and "## What R1, C1 and C2 are" in md
    assert "No C2 flight is on disk." in md


def test_every_row_states_why_c1_was_not_flown_and_the_new_backends_are_rows():
    assert set(V.BACKENDS) >= {"aiotvla_v10b", "dronecats_2b", "racevla_7b", "aerialvla_bf16",
                               "openvla_7b_bf16", "openvla_7b_int8wo"}
    t = _build()
    for r in t["rows"]:
        c1 = r["cells"]["closed_loop_hil_c1"]
        assert c1 and c1 != V.C1_TO_FLY == "to measure on hil (C1)", \
            (r["backend"], "no reason for C1")
        assert c1.startswith(("not flown", "cannot fly", "weights not obtained")), (r["backend"], c1)
        assert "closed_loop_hil" not in r["cells"], "the hil cell is two cells since /3"
    assert "follow-only" in _row(t, "aiotvla_v10b")["cells"]["closed_loop_hil_c1"]
    assert "C2" in _row(t, "aiotvla_v10b")["cells"]["closed_loop_hil_c1"]
    assert t["c1_bundles"]["found"] == [] and "urban_demo_policy" in t["c1_bundles"]["policy"]
    md = V.render_markdown(t)
    assert "## C1 on hil: flown by no backend, and why" in md
    # a signed urban_demo bundle, once there, is named
    b = _tmp()
    (b / "urban-demo-v0.1.0.tar.gz").write_bytes(b"x")
    assert _build(bundles=b)["c1_bundles"]["found"] == ["urban-demo-v0.1.0.tar.gz"]
    # the served families measure through their servers' benches, and say which
    for bid in V.SERVED_BACKENDS:
        try:
            V.measure(bid, "dev")
        except ValueError as exc:
            assert "bench" in str(exc), exc
        else:
            raise AssertionError(f"{bid} was timed by `measure`")


def test_the_agx_benches_reach_the_orin_column_each_with_its_label():
    if not (HIL_EVIDENCE / "aiotvla_v10b_bench_agx.json").is_file():
        return SKIP
    recs = V.extract_orin_benches(V.EVIDENCE)
    by = {}
    for r in recs:
        by.setdefault(r["backend"], []).append(r)
    assert set(by) >= {"aerialvla_bf16", "aiotvla_v10b", "openvla_7b_bf16"}, set(by)
    assert all(r["protocol"] == "not R1" and r["why_not_r1"] for r in recs)
    assert all(r["host_is_orin"] for r in recs), [r["host_basis"] for r in recs]
    # AIoT-VLA: its own frames, said so, and both the round trip and the model
    aiot = by["aiotvla_v10b"][0]
    raw = json.loads((HIL_EVIDENCE / "aiotvla_v10b_bench_agx.json").read_text(encoding="utf-8"))
    assert aiot["own_inputs"] is True and "aiot_frames" in aiot["inputs"]
    assert aiot["stats"]["p95_ms"] == raw["roundtrip"]["p95_ms"] == 41.7
    assert aiot["server_latency"]["p95_ms"] == raw["server_latency"]["p95_ms"]
    assert aiot["timed"] == raw["n"] == 300
    cell = V.bench_cell(aiot)
    assert cell.startswith("not R1 - own inputs (aiot_frames)") and "p95 41.7 ms" in cell
    # AerialVLA: the R1 frames at 5 + 50, still not R1 (the server, the power mode)
    cfgs = {r["config"]: r for r in by["aerialvla_bf16"]}
    assert set(cfgs) == {"bf16/sdpa/unmerged", "bf16/sdpa/merged"}
    assert all(not r["own_inputs"] and (r["warmup"], r["timed"]) == (5, 50)
               for r in cfgs.values())
    assert cfgs["bf16/sdpa/merged"]["stats"]["p50_ms"] == 1073.9
    assert "own inputs" not in V.bench_cell(cfgs["bf16/sdpa/merged"])
    assert any("power mode" in w for w in cfgs["bf16/sdpa/merged"]["why_not_r1"])
    # the bf16 gate check: eight answers, the first cold, no host block
    gate = by["openvla_7b_bf16"][0]
    assert gate["timed"] == 8 and gate["first_s"] > gate["rest_max_s"] > 0.9
    assert "no host block" in gate["host_basis"]
    assert V.bench_cell(gate).startswith("not R1 - gate check, 8 answers")
    # in the table: labelled, never counted as R1; R1 still owed where `measure`
    # can time the model, and said not to cover a served model, which it cannot
    t = _build(evidence=V.EVIDENCE)
    cell = _row(t, "openvla_7b_bf16")["cells"]["rate_orin"]
    assert cell.startswith("R1: ") and "vla_rate_openvla_7b_bf16.json" in cell, cell
    for bid in ("aiotvla_v10b", "aerialvla_bf16"):
        cell = _row(t, bid)["cells"]["rate_orin"]
        assert cell.startswith("not R1 - ") and "to measure" not in cell, cell
        assert "R1 does not cover a model served over protocol v2 (`measure` refuses it): " \
               "its Orin timing is its server's bench (" in cell, cell
    assert "bf16/sdpa/unmerged" in _row(t, "aerialvla_bf16")["cells"]["rate_orin"]
    assert "bf16/sdpa/merged" in _row(t, "aerialvla_bf16")["cells"]["rate_orin"]
    # 16 Nov on the AGX: R1 for stub, bc_v3 and OpenVLA bf16; the int8 attempt
    # refused; the DroneCATS and RaceVLA server benches
    assert "holds R1 for 3 of the 7 backends `measure` can time" in t["decision"], t["decision"]
    assert "not R1, labelled as such, for 4 backends" in t["decision"], t["decision"]
    assert "a recorded attempt with no timing for 1 more" in t["decision"], t["decision"]
    assert "R1 does not cover the 4 backends served over protocol v2" in t["decision"]
    for bid in ("dronecats_2b", "racevla_7b"):
        cell = _row(t, bid)["cells"]["rate_orin"]
        assert cell.startswith("not R1 - round trip through its server") and             f"{bid}_bench_agx.json" in cell, cell
    for bid in ("stub", "bc_v3"):
        assert _row(t, bid)["cells"]["rate_orin"].startswith("R1: "), bid
    assert _row(t, "openvla_7b_4bit")["cells"]["rate_orin"].startswith(
        "to measure on the Orin (R1); needs bitsandbytes")
    assert "torchao" in _row(t, "openvla_7b_int8wo")["cells"]["rate_orin"]


def test_a_bench_from_another_host_or_on_the_r1_frames_is_labelled_as_what_it_is():
    src = HIL_EVIDENCE / "aiotvla_v10b_bench_agx.json"
    if not src.is_file():
        return SKIP
    raw = json.loads(src.read_text(encoding="utf-8"))
    raw.pop("rows", None)
    ev = _tmp()
    (ev / "hil").mkdir()
    # the same bench made on a desktop, lying in the hil folder: not an Orin's
    desk = json.loads(json.dumps(raw))
    desk["identity"]["host"] = {"host_arch": "AMD64", "device_model": None,
                                "l4t_release": None, "jetpack": None}
    (ev / "hil" / "desk.json").write_text(json.dumps(desk), encoding="utf-8")
    rec = V.extract_orin_benches(ev)[0]
    assert rec["host_is_orin"] is False and "aarch64" in rec["host_basis"]
    row = _row(_build(evidence=ev), "aiotvla_v10b")
    assert row["cells"]["rate_orin"].startswith("no Orin bench yet; ") and not row["orin_timed"]
    # no host block at all is not an Orin either
    desk["identity"].pop("host")
    (ev / "hil" / "desk.json").write_text(json.dumps(desk), encoding="utf-8")
    assert V.extract_orin_benches(ev)[0]["host_is_orin"] is False
    # the R1 frames through the foreign bench: no "own inputs", still not R1
    r1f = json.loads(json.dumps(raw))
    r1f["frames"] = "/home/x/vla-drone/deploy/evidence/vla_protocol_frames"
    (ev / "hil" / "desk.json").write_text(json.dumps(r1f), encoding="utf-8")
    rec = V.extract_orin_benches(ev)[0]
    assert rec["own_inputs"] is False and rec["protocol"] == "not R1"
    assert "own inputs" not in V.bench_cell(rec) and V.bench_cell(rec).startswith("not R1 - ")
    assert any("3 warm-up + 300 timed, not R1's 5 + 50" in w for w in rec["why_not_r1"])
    # ... and at R1's own counts it is still not R1 (the server is in the path),
    # but it is no longer said to differ in its counts
    r1f["warmup"], r1f["n"] = 5, 50
    (ev / "hil" / "desk.json").write_text(json.dumps(r1f), encoding="utf-8")
    rec = V.extract_orin_benches(ev)[0]
    assert rec["protocol"] == "not R1" and not any("5 + 50" in w for w in rec["why_not_r1"])
    assert any("round trip over TCP" in w for w in rec["why_not_r1"]), rec["why_not_r1"]
    # a file this tool does not know says only that it records no power mode;
    # a file that records one is quoted (never the README's word for another session)
    assert any(w.startswith("the file records no power mode; ") for w in rec["why_not_r1"])
    assert not any("README" in w for w in rec["why_not_r1"]), rec["why_not_r1"]
    r1f["power_mode"] = "MODE_30W (ID 2), jetson_clocks on"
    (ev / "hil" / "desk.json").write_text(json.dumps(r1f), encoding="utf-8")
    rec = V.extract_orin_benches(ev)[0]
    assert "power mode as the file records it: MODE_30W (ID 2), jetson_clocks on" in \
        rec["why_not_r1"]
    known = V.extract_orin_benches(V.EVIDENCE)
    aiot_known = next(r for r in known if r["backend"] == "aiotvla_v10b")
    assert any("states none for that session" in w for w in aiot_known["why_not_r1"])
    # an R1 result from an Orin takes the cell; the bench stays in the records
    (ev / "hil" / "vla_rate_aiotvla_v10b.json").write_text(
        json.dumps(_rate(True, "aiotvla_v10b")), encoding="utf-8")
    row = _row(_build(evidence=ev), "aiotvla_v10b")
    assert row["cells"]["rate_orin"].startswith("R1: "), row["cells"]
    assert any(m["metric"] == "bench" for m in row["records"])


def test_c2_scores_only_the_flights_that_meet_it_and_lists_the_rest_with_the_reason():
    out = _tmp()
    docs = _tmp()
    _pas_run(out, "pas_hil_c2_01")
    _pas_run(out, "pas_hil_c2_02", w30=0.2694, dec_hz=10.62, interventions=332)
    _pas_run(out, "pas_hil_lost_01", ticks=980, end="target_lost", w30=0.1378, dec_hz=3.45,
             interventions=39, mission_s=98.1, outcome="success", fail_reasons=(),
             unsafe=0)                                          # lost the car: still C2
    # given C2's 420 s, it is C2 however it ended: a takeover is a result
    _pas_run(out, "pas_hil_takeover_01", ticks=2000, end="autopilot took over (RTL)",
             mission_s=200.3, w30=0.4)
    # a shakedown: one that ran to its 60 s limit (no record of the limit, then
    # one that records it), one that the tag marks, and one that ended early
    # for another reason with no record of its limit
    _pas_run(out, "pas_hil_short_01", ticks=600, mission_s=60.1, w30=0.9, max_s=None)
    _pas_run(out, "pas_hil_short_02", ticks=600, mission_s=60.1, w30=0.9, max_s=60.0)
    _pas_run(out, "pas_hil_shake_01", ticks=300, end="target_lost", mission_s=30.2, w30=0.9)
    _pas_run(out, "pas_hil_early_01", ticks=200, end="autopilot took over (RTL)",
             mission_s=20.1, w30=0.9, max_s=None)
    # what C2 fixes beside the option set, read back from the run's own record
    _pas_run(out, "pas_hil_nonq_01", negq=None)
    _pas_run(out, "pas_hil_otherq_01", negq=("a bus",))
    _pas_run(out, "pas_hil_nfz_01", bundle="follow-car-citylife-nfz3b-v0.1.1.tar.gz")
    _pas_run(out, "pas_hil_nobundle_01", bundle=None)
    _pas_run(out, "pas_hil_nocam_01", capture=None)
    _pas_run(out, "pas_hil_cam66_01", capture=0.066)
    _pas_run(out, "pas_hil_rain_01", stress="rain", kpi_grade=False)
    _pas_run(out, "pas_hil_hot_01", hot=True)
    _pas_run(out, "pas_hil_tick_01", feed="tick")
    _pas_run(out, "pas_hil_old_01", feed=None)                 # before the feed existed
    _pas_run(out, "pas_hil_cap_01", cap=12.0)
    _pas_run(out, "pas_hil_noest_01", identity=False, estimator=False)
    _pas_run(out, "pas_hil_gate_01", reached=False, ticks=0, kpi_grade=False,
             topology="projectairsim-ardupilot-mavros", end="no mission: start gate timed out")
    _pas_run(out, "pas_hil_para_01")
    _pas_run(out, "pas_hil_feed_05")                           # C2_NOT_ALONE
    _pas_run(out, "pas_hil_dirty_01", kpi_grade=False)
    _pas_run(out, "pas_hil_dc_01", model="dronecats/DroneCATS-Cosmos3-Edge-2B@5080e727",
             feed="tick", w30=0.5, dec_hz=0.4)
    _pas_run(out, "pas_hil_servo_01", pilot_kind="servo", model=None, feed=None, w30=0.9)
    # not hil at all, and not this rail: neither is a hil follow
    d = _pas_run(out, "pas_dev_mavros_09", topology="dev")
    m = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
    m["grant_topology"] = "dev"
    (d / "metrics.json").write_text(json.dumps(m), encoding="utf-8")
    _pas_run(out, "pas_other_rail_01", rail="projectairsim-simple_flight")
    (docs / "pas_hil_paraphrase_x").mkdir()
    (docs / "pas_hil_paraphrase_x" / "flights.json").write_text(json.dumps({"runs": [
        {"run": "pas_hil_c2_01", "paraphrase_id": "source", "text": "follow a red car"},
        # the source instruction flown again under another id is not a paraphrase
        {"run": "pas_hil_c2_02", "paraphrase_id": "source-repeat", "text": "follow a red car"},
        {"run": "pas_hil_para_01", "paraphrase_id": "pp-1", "text": "track a red car"}]}),
        encoding="utf-8")
    assert V.paraphrase_flights(docs) == {"pas_hil_para_01": "track a red car"}
    # a flight known only by its committed summary: listed, never scored
    (docs / "pas_hil_feed_x").mkdir()
    (docs / "pas_hil_feed_x" / "flights.json").write_text(json.dumps({
        "what": "AIoT-VLA v10b on the AGX Orin", "flights": [
            {"tag": "pas_hil_summary_01", "kpi_grade": True, "topology": "hil", "ticks": 970,
             "interventions": 14, "p0_escapes": 0, "decision_hz_mission": 2.997,
             "end_reason": "target_lost", "vla_feed": "stream"},
            {"tag": "pas_hil_c2_01", "kpi_grade": True, "ticks": 1}]}), encoding="utf-8")
    t = _build(demo_out=out, docs_data=docs)
    aiot = _row(t, "aiotvla_v10b")
    flights = {f["tag"]: f for f in aiot["records"] if f["metric"] == "hil_follow"}
    c2 = {tag for tag, f in flights.items() if not f["not_c2"]}
    assert c2 == {"pas_hil_c2_01", "pas_hil_c2_02", "pas_hil_lost_01", "pas_hil_takeover_01"}, c2
    assert all(flights[tag]["protocol"] == "C2" for tag in c2)
    why = {tag: " | ".join(f["not_c2"]) for tag, f in flights.items() if f["not_c2"]}
    for tag, said in (("pas_hil_rain_01", "stressor 'rain'"),
                      ("pas_hil_hot_01", "hot-applied"),
                      ("pas_hil_tick_01", "feed tick, not the family's C2 feed (stream"),
                      ("pas_hil_old_01", "feed tick"),
                      ("pas_hil_cap_01", "capped at 12 Hz"),
                      ("pas_hil_noest_01", "--identity and --target-estimator"),
                      ("pas_hil_gate_01", "start gate never passed"),
                      ("pas_hil_para_01", "paraphrased instruction ('track a red car')"),
                      ("pas_hil_feed_05", "not flown alone"),
                      ("pas_hil_dirty_01", "not KPI-grade"),
                      ("pas_hil_short_01", "ran to a time limit of about 60 s, under C2's 420 s"),
                      ("pas_hil_short_02", "given a time limit of 60 s, not C2's 420 s"),
                      ("pas_hil_shake_01", "a shakedown by its tag ('_shake')"),
                      ("pas_hil_early_01", "ended after 20 s (autopilot took over (RTL)) "
                                           "without losing the car, and the run records no "
                                           "time limit"),
                      ("pas_hil_nonq_01", "negative query None, not C2's ['a traffic light']"),
                      ("pas_hil_otherq_01", "negative query ['a bus']"),
                      ("pas_hil_nfz_01", "policy bundle follow-car-citylife-nfz3b-v0.1.1.tar.gz, "
                                         "not the node's default"),
                      ("pas_hil_nobundle_01", "records no policy bundle"),
                      ("pas_hil_nocam_01", "FrontCamera capture interval - s, not the 0.05 s"),
                      ("pas_hil_cam66_01", "FrontCamera capture interval 0.066 s"),
                      ("pas_hil_summary_01", "only its committed summary")):
        assert said in why[tag], (tag, why[tag])
    # each of those is not C2 for that reason alone
    for tag in ("pas_hil_short_02", "pas_hil_early_01", "pas_hil_nonq_01", "pas_hil_otherq_01",
                "pas_hil_nfz_01", "pas_hil_nobundle_01", "pas_hil_nocam_01",
                "pas_hil_cam66_01"):
        assert len(flights[tag]["not_c2"]) == 1, (tag, flights[tag]["not_c2"])
    # the tick feed asks no 15 Hz camera: another family's flight is not held to it
    assert flights["pas_hil_c2_01"]["max_s_given"] == 420.0
    assert flights["pas_hil_c2_01"]["bundle"] == "follow-car-citylife-v0.1.3.tar.gz"
    assert "not KPI-grade" in why["pas_hil_rain_01"] and "not hil" in why["pas_hil_gate_01"]
    assert flights["pas_hil_summary_01"]["frac_within_30m"] is None
    assert flights["pas_hil_summary_01"]["summary_only"] is True
    assert sum(1 for f in flights.values() if f["tag"] == "pas_hil_c2_01") == 1
    assert "pas_dev_mavros_09" not in flights and "pas_other_rail_01" not in flights
    # what C2 reports, per flight
    f = flights["pas_hil_c2_01"]
    assert f["frac_within_30m"] == 0.3523 and f["decision_hz_mission"] == 10.93
    assert math.isclose(f["decision_hz_asked"], 4648 / 353.393)
    assert math.isclose(f["interventions_per_100_ticks"], 100 * 284 / 4198)
    assert f["p0_escapes"] == 0 and f["date"] == "2026-11-15" and f["kpi_grade"] is True
    # the flight's own KPI verdict is read beside the escape count
    assert (f["kpi_outcome"], f["mission_fail_reasons"], f["unsafe_p0_ticks"]) == \
        ("fail", ["unsafe_position_entered"], 3)
    assert f["p0_ticks_not_measurable"] == 0 and f["p0_ticks_blind"] is None
    # cited by its name in a checkout, not by this machine's path to it
    assert f["source"]["file"] == "demo/out/pas_hil_c2_01/metrics.json", f["source"]
    cell = aiot["cells"]["closed_loop_hil_c2"]
    assert cell.startswith("C2, 4 flights (1 autopilot took over (RTL), 2 max_s reached, "
                           "1 target_lost): within 30 m 0.138-0.400 (median 0.311)"), cell
    assert "P0 escapes 0" in cell and "more to fly" not in cell
    assert "KPI outcome 3 fail, 1 success (unsafe_position_entered 3)" in cell, cell
    assert "ticks in a P0-unsafe position 3, 3, 0, 3" in cell, cell
    assert "P0 ticks not measurable / blind 0/-, 0/-, 0/-, 0/-" in cell, cell
    assert f"{len(flights) - 4} more hil follows outside C2" in cell, cell
    # another family's flight is its own row's, with its own feed
    dc = _row(t, "dronecats_2b")
    assert dc["cells"]["closed_loop_hil_c2"].startswith("C2, 1 flight (1 max_s reached)")
    assert "the protocol asks for 3: 2 more to fly" in dc["cells"]["closed_loop_hil_c2"]
    # the hand-written follow is the null: kept, not a row, never a C2 flight
    servo = [r for r in t["other_records"] if r.get("tag") == "pas_hil_servo_01"]
    assert servo and servo[0]["backend"] == V.HAND_WRITTEN and "null" in servo[0]["not_c2"][0]
    assert "C2 by 2 " in t["decision"], t["decision"]
    md = V.render_markdown(t)
    assert "| pas_hil_lost_01 | AIoT-VLA v10b |" in md and "| pas_hil_rain_01 |" in md
    assert "hil follow flights that are not C2" in md
    row = next(ln for ln in md.splitlines() if ln.startswith("| pas_hil_c2_01 |"))
    assert "| 0 | fail (unsafe_position_entered) | 3 | 0 / - |" in row, row
    assert "0 escapes and still fail its KPI on position" in md
    # a flight that never flew a tick has no P0 count to show: '-', never 0
    gate = next(ln for ln in md.splitlines() if ln.startswith("| pas_hil_gate_01 |"))
    assert gate.endswith("| - | - (no mission) |"), gate
    assert "C2's null (the hand-written follow on the same rail (--pilot servo): the " \
           "no-learning reference): 1 hil follow of it on disk, listed below." in md
    assert "no flight of it on disk" in V.render_markdown(_build())
    # a count a run did not record is said, never summed as 0
    assert V.p0_escapes_text([{"p0_escapes": 0}, {"p0_escapes": None}]) == \
        "P0 escapes 0 (not recorded for 1 of 2)"
    assert V.p0_escapes_text([{"p0_escapes": None}]) == "P0 escapes not recorded"
    assert "KPI outcome 1 not recorded" in V.kpi_verdict_text([{"kpi_outcome": None}])
    # a stored tag is not a path component the reader trusts: a second root
    # that holds the same tag does not replace the first
    other = _tmp()
    _pas_run(other, "pas_hil_c2_01", w30=0.99)
    _pas_run(other, "pas_hil_c2_03", w30=0.31)
    t2 = _build(demo_out=out, docs_data=docs, runs=[other])
    fl = {f["tag"]: f for f in _row(t2, "aiotvla_v10b")["records"] if f["metric"] == "hil_follow"}
    assert fl["pas_hil_c2_01"]["frac_within_30m"] == 0.3523
    assert fl["pas_hil_c2_03"]["source"]["file"] == \
        f"{other.as_posix()}/pas_hil_c2_03/metrics.json"


def test_the_committed_note_still_says_which_flights_did_not_fly_alone():
    """C2_NOT_ALONE is kept by hand; the note it cites must still say it."""
    for tag, why in V.C2_NOT_ALONE.items():
        doc = ROOT / why.split("(")[1].rstrip(")")
        assert doc.is_file(), doc
        text = " ".join(doc.read_text(encoding="utf-8").split())
        assert "Desktop tests were run beside pas_hil_feed_05 / _06" in text, doc
        assert tag in ("pas_hil_feed_05", "pas_hil_feed_06")


def test_the_follow_matrices_are_records_outside_c1_and_c2():
    # 27-28 October: the committed file
    recs = V.extract_matrix_1028(V.DOCS_DATA)
    by = {r["backend"]: r for r in recs}
    assert {"aerialvla_bf16", "dronecats_2b", "racevla_7b", V.HAND_WRITTEN, V.STAND_IN} <= set(by)
    assert all(r["protocol"] == "not C1, not C2" and "not ArduPilot" in r["conditions"]
               for r in recs)
    aero = {f["tag"]: f for f in by["aerialvla_bf16"]["flights"]}
    assert aero["cmp2_hier_aerovla_20261028"]["within_30m"] == 0.309
    assert aero["orin_hier_aerovla_20261028"]["within_30m"] == 0.989
    assert aero["orin_hier_aerovla_20261028"]["platform"].startswith("Jetson Orin Nano")
    raw = json.loads((V.DOCS_DATA / "progress_1028.json").read_text(encoding="utf-8"))
    assert sum(len(r["flights"]) for r in recs) == len(raw["matrix_rows"]) == 42
    # A row the file labels hand-written but which records a server latency:
    # the label is contradicted by its own row. Without the run folder it
    # keeps the label and says so; with it, the folder's pilot record decides.
    odd = [r for r in raw["matrix_rows"]
           if r["model"] == "manual" and r.get("latency_p50_s") is not None]
    assert [r["tag"] for r in odd] == ["cmp_hier_fixed12_20261027"], odd
    hand = {f["tag"]: f for f in by[V.HAND_WRITTEN]["flights"]}
    assert "label stands unchecked" in hand["cmp_hier_fixed12_20261027"]["note"]
    assert hand["cmp_manual_20261027"]["note"] is None
    folders = _tmp()
    (folders / "cmp_hier_fixed12_20261027").mkdir()
    (folders / "cmp_hier_fixed12_20261027" / "metrics.json").write_text(json.dumps({
        "tag": "cmp_hier_fixed12_20261027", "frac_within_30m": 0.98,
        "vla_hierarchy": {"latency_s": {"p50": 0.08}},
        "pilot": {"kind": "stub", "server_identity": {"id": "fake:scripted"}}}), encoding="utf-8")
    checked = {r["backend"]: r for r in V.extract_matrix_1028(V.DOCS_DATA, [folders])}
    moved = {f["tag"]: f for f in checked[V.STAND_IN]["flights"]}["cmp_hier_fixed12_20261027"]
    assert "fake:scripted" in moved["note"] and "scripted 12 m stand-in" in moved["kind"]
    assert moved["within_30m"] == 0.98, "the score is still the file's"
    assert "cmp_hier_fixed12_20261027" not in {
        f["tag"] for f in checked[V.HAND_WRITTEN]["flights"]}
    assert sum(len(r["flights"]) for r in checked.values()) == 42
    # 3-4 November: run folders, every one found, a tag outside the matrix not
    out = _tmp()

    def run(tag, metrics, rate=0.0, bundle="follow-car-citylife-nfz-v0.1.1.tar.gz",
            commit="6da813de8ad7"):
        (out / tag).mkdir()
        (out / tag / "metrics.json").write_text(json.dumps(dict(
            metrics, tag=tag, policy_source={"path": f"/home/x/vla-drone/bundles/{bundle}"})),
            encoding="utf-8")
        (out / tag / "kpi.json").write_text(json.dumps(
            {"p0_violation_escape_rate": rate,
             "manifest": {"topology": "projectairsim-remote-sim",
                          "code_revision": commit}}), encoding="utf-8")

    aero_id = {"id": "aerialvla-lora/aero_vla@unknown", "precision": "bf16", "attn": "sdpa",
               "merged": True}
    run("agx_e2e_aerovla_20261103", {
        "rail": "projectairsim-simple_flight", "frac_within_30m": 0.213,
        "frac_within_30m_after_acquire": 0.048,
        "pilot": {"kind": "vla", "server_identity": aero_id,
                  "summary": {"latency_server_s": {"median": 1.412}}}})
    run("agx_hier_aerovla_20261103", {
        "frac_within_30m": 1.0, "vla_hierarchy": {"latency_s": {"p50": 1.44}},
        "pilot": {"kind": "vla", "server_identity": aero_id}})
    run("agx_hier_fixed12_20261103", {
        "frac_within_30m": 0.374, "vla_hierarchy": {"latency_s": {"p50": 0.09}},
        "pilot": {"kind": "stub", "server_identity": {"id": "fake:scripted"}}})
    run("agx_manual_20261103", {"frac_within_30m": 0.952, "pilot": {"kind": "none"}})
    # the first AGX follow of 3 November, at an earlier commit: listed, with it
    run("agx_e2e_aerovla_01", {"frac_within_30m_after_acquire": 0.184,
                               "pilot": {"kind": "vla", "server_identity": aero_id}},
        commit="07cee03f0049")
    # the same configuration under another bundle: listed, with the bundle
    run("nfz3b_agx_hier_aerovla_20261104", {
        "frac_within_30m": 0.494, "vla_hierarchy": {"latency_s": {"p50": 1.4}},
        "pilot": {"kind": "vla", "server_identity": aero_id}},
        bundle="follow-car-citylife-nfz3b-v0.1.1.tar.gz", commit="989f96724b74")
    # a tag outside the matrix: not listed
    run("agx_scratch_e2e_aerovla", {"frac_within_30m_after_acquire": 0.5,
                                    "pilot": {"kind": "vla", "server_identity": aero_id}})
    recs = V.extract_matrix_agx([out])
    by = {r["backend"]: r for r in recs}
    flights = {f["tag"]: f for f in by["aerialvla_bf16"]["flights"]}
    assert set(flights) == {"agx_e2e_aerovla_20261103", "agx_hier_aerovla_20261103",
                            "agx_e2e_aerovla_01", "nfz3b_agx_hier_aerovla_20261104"}, set(flights)
    assert flights["agx_e2e_aerovla_01"]["within_30m"] == 0.184
    assert flights["agx_e2e_aerovla_01"]["code_revision"] == "07cee03f0049"
    assert flights["nfz3b_agx_hier_aerovla_20261104"]["bundle"] == \
        "follow-car-citylife-nfz3b-v0.1.1.tar.gz"
    assert flights["agx_hier_aerovla_20261103"]["bundle"] == \
        "follow-car-citylife-nfz-v0.1.1.tar.gz"
    # a pure flight is scored after acquisition, a hierarchical one from the gate
    assert flights["agx_e2e_aerovla_20261103"]["within_30m"] == 0.048
    assert flights["agx_e2e_aerovla_20261103"]["latency_p50_s"] == 1.412
    assert flights["agx_hier_aerovla_20261103"]["within_30m"] == 1.0
    assert flights["agx_hier_aerovla_20261103"]["config"] == "bf16/sdpa/merged"
    assert by[V.STAND_IN]["flights"][0]["tag"] == "agx_hier_fixed12_20261103"
    assert by[V.HAND_WRITTEN]["flights"][0]["within_30m"] == 0.952
    assert all(r["protocol"] == "not C1, not C2" for r in recs)
    # no folder here: a record that says so, not silence
    gone = V.extract_matrix_agx([_tmp()])
    assert gone[0]["status"] == "source missing on this host", gone
    # in the table they are neither C1 nor C2, and they are not ArduPilot
    t = _build(demo_out=out, docs_data=V.DOCS_DATA)
    row = _row(t, "aerialvla_bf16")
    assert row["cells"]["closed_loop_hil_c2"] == "to fly on hil (C2)"
    assert row["cells"]["closed_loop_hil_c1"].startswith("not flown")
    assert row["cells"]["closed_loop_desktop"] == "not C1 - see records"
    assert not any(r["cells"]["closed_loop_hil_c2"].startswith("C2,") for r in t["rows"])
    md = V.render_markdown(t)
    assert "## Follow flights on Project AirSim simple_flight (not ArduPilot: not C1, not C2)" \
        in md
    assert "**27-28 October 2026.**" in md and "**3-4 November 2026.**" in md
    assert "| Hand-written follow (no VLA) | agx_manual_20261103 |" in md
    # each AGX flight shows its commit and bundle, so another scenario is not
    # read as run-to-run spread of one configuration
    assert "| Commit | Policy bundle |" in md
    nfz = next(ln for ln in md.splitlines() if "| nfz3b_agx_hier_aerovla_20261104 |" in ln)
    assert "| 989f967 | follow-car-citylife-nfz3b-v0.1.1.tar.gz |" in nfz, nfz
    assert "another scenario, not a repeat" in md


# Every backend that can fly C2: (identity id, feed), as its server reports it.
_C2_FAMILIES = {"aiotvla_v10b": ("aiotvla/v10b@35259bd1", "stream"),
                "aerialvla_bf16": ("aerialvla-lora/aero_vla@unknown", "tick"),
                "dronecats_2b": ("dronecats/DroneCATS-Cosmos3-Edge-2B@5080e727", "tick"),
                "racevla_7b": ("racevla/drone6_all@03f3bb48", "tick")}


def _default_set(n_c2: int = 3, others: bool = True, stub_r1: bool = True):
    """A set that holds AIoT-VLA's AGX bench and `n_c2` C2 flights of it;
    with `others`, an Orin bench and three C2 flights of every other backend
    that can fly C2 (the AerialVLA benches as committed, a foreign bench for
    DroneCATS and RaceVLA made from AIoT-VLA's file with the identity
    changed); with `stub_r1`, the stub's R1 on an Orin."""
    ev, out = _tmp(), _tmp()
    (ev / "hil").mkdir()
    raw = json.loads((HIL_EVIDENCE / "aiotvla_v10b_bench_agx.json").read_text(encoding="utf-8"))
    raw.pop("rows", None)
    (ev / "hil" / "aiotvla_v10b_bench_agx.json").write_text(json.dumps(raw), encoding="utf-8")
    for k in range(n_c2):
        _pas_run(out, f"pas_hil_c2_0{k + 1}", w30=0.2 + 0.05 * k)
    if others:
        for name in ("aerialvla_serving_bench_agx.json", "aerialvla_serving_bench_agx_merged.json"):
            (ev / "hil" / name).write_bytes((HIL_EVIDENCE / name).read_bytes())
        for bid in ("dronecats_2b", "racevla_7b"):
            other = json.loads(json.dumps(raw))
            other["identity"]["id"] = _C2_FAMILIES[bid][0]
            (ev / "hil" / f"{bid}_bench_agx.json").write_text(json.dumps(other),
                                                              encoding="utf-8")
        for bid in ("aerialvla_bf16", "dronecats_2b", "racevla_7b"):
            model, feed = _C2_FAMILIES[bid]
            for k in range(3):
                _pas_run(out, f"pas_hil_c2_{bid}_0{k + 1}", model=model, feed=feed,
                         w30=0.1 + 0.1 * k)
    if stub_r1:
        (ev / "hil" / "vla_rate_stub.json").write_text(json.dumps(_rate(True)),
                                                       encoding="utf-8")
    return dict(evidence=ev, demo_out=out)


def test_the_default_line_is_none_until_a_default_is_named_after_the_set():
    if not (HIL_EVIDENCE / "aiotvla_v10b_bench_agx.json").is_file():
        return SKIP
    kw = _default_set()
    t = _build(**kw)
    assert t["default_backend"] is None
    assert "**Default backend:** none." in V.render_markdown(t)
    assert V.newest_measurement(t) == "2026-11-15"

    def named(backend="aiotvla_v10b", scope="the follow task", date="2026-11-17",
              today="2026-11-18", **over):
        return _build(default={"backend": backend, "scope": scope, "date": date},
                      today=today, **{**kw, **over})

    def refused(said, **a):
        try:
            named(**a)
        except V.DefaultRefused as exc:
            assert said in str(exc), (said, str(exc))
        else:
            raise AssertionError(f"a default was named although: {said} ({a})")

    # not before the set: its newest flight is of 15 November, the protocol of the 16th
    refused("before vla-backend-protocol/2", date="2026-11-15")
    refused("before vla-backend-protocol/2", date="2026-11-14")
    refused("in the future", date="2026-11-19")
    refused("YYYY-MM-DD", date="17 Nov")
    refused("--default-scope", scope=" ")
    refused("not a row", backend="gpt")
    refused("no checkpoint", backend="bitvla")
    refused("no Orin rate", backend="dronecats_2b", **_default_set(others=False))
    refused("2 C2 flight(s)", **_default_set(2))
    # the default's own numbers are not enough: the set must compare it with
    # every other backend that can fly C2 on the same tests, and hold R1's floor
    refused("does not yet compare it with the other backends on the same tests - "
            "stub: no R1 on an Orin (R1's floor on the same host); aerialvla_bf16: no Orin "
            "timing, 0 of 3 C2 flights; dronecats_2b: no Orin timing, 0 of 3 C2 flights; "
            "racevla_7b: no Orin timing, 0 of 3 C2 flights",
            **_default_set(others=False, stub_r1=False))
    refused("stub: no R1 on an Orin", **_default_set(stub_r1=False))
    short = _default_set()
    import shutil
    shutil.rmtree(short["demo_out"] / "pas_hil_c2_racevla_7b_03")
    refused("racevla_7b: 2 of 3 C2 flights", **short)
    partial = _build(**_default_set(others=False))
    assert "racevla_7b: no Orin timing, 0 of 3 C2 flights" in partial["comparison_gaps"]
    assert "What the set lacks before `table --default` names one" in \
        V.render_markdown(partial)
    # dates are the lab's (UTC+8): a flight at 07:30 on the 17th in the lab is
    # 23:30 UTC on the 16th, and the set's newest measurement is of the 17th
    assert V.lab_date(T0_UTC_16_NOV_2330) == "2026-11-17"
    assert V.lab_date("2026-11-16T23:30:00Z") == "2026-11-17"
    assert V.lab_date("2026-11-16T15:59:59Z") == "2026-11-16" and V.lab_date(None) is None
    early = _default_set()
    _pas_run(early["demo_out"], "pas_hil_c2_04", t0=T0_UTC_16_NOV_2330)
    assert V.newest_measurement(_build(**early)) == "2026-11-17"
    refused("not after the set's newest measurement (2026-11-17)", date="2026-11-17",
            today="2026-11-18", **early)
    # a flight newer than the date: the default would be before the set
    late = _default_set()
    d = late["demo_out"] / "pas_hil_c2_01" / "metrics.json"
    m = json.loads(d.read_text(encoding="utf-8"))
    m["vla_observation"]["t0_wall"] = T0_15_NOV + 3 * 86400          # 18 November
    d.write_text(json.dumps(m), encoding="utf-8")
    refused("not after the set's newest measurement (2026-11-18)", date="2026-11-18",
            today="2026-11-20", **late)
    # named after the set: the line carries the backend's own numbers and no one's approval
    t = named()
    d = t["default_backend"]
    assert (d["backend"], d["scope"], d["date"]) == ("aiotvla_v10b", "the follow task",
                                                     "2026-11-17")
    assert d["set_newest_measurement"] == "2026-11-15" and d["named_by"] == "the lab"
    assert d["closed_loop_hil_c2"].startswith("C2, 3 flights")
    assert d["rate_orin"].startswith("not R1 - own inputs")
    md = V.render_markdown(t)
    line = next(ln for ln in md.splitlines() if ln.startswith("**Default backend:**"))
    assert "AIoT-VLA v10b" in line and "for the follow task" in line and "2026-11-17" in line
    assert "named by the lab on 2026-11-17" in line and "within 30 m 0.200-0.300" in line, line
    assert "records no confirmation of the choice by anyone" in line
    for word in ("the PI", " PI ", "approved", "accepted", "confirmed by"):
        assert word not in line, (word, line)
    # the command line: a refusal writes nothing and exits 2
    oj, om = _tmp() / "t.json", _tmp() / "t.md"
    rc = V.main(["table", "--demo-out", str(kw["demo_out"]), "--experiments-out", str(_tmp()),
                 "--out-json", str(oj), "--out-md", str(om), "--default", "aiotvla_v10b",
                 "--default-scope", "the follow task", "--default-date", "2026-11-10"])
    assert rc == 2 and not oj.exists() and not om.exists()
    rc = V.main(["table", "--demo-out", str(kw["demo_out"]), "--experiments-out", str(_tmp()),
                 "--out-json", str(oj), "--out-md", str(om), "--default-date", "2026-11-17"])
    assert rc == 2 and not oj.exists(), "a date with no backend named one"


def test_g3_is_reported_as_measurements_and_never_as_a_decision():
    if not (HIL_EVIDENCE / "aerialvla_serving_bench_agx.json").is_file():
        return SKIP
    t = _build(evidence=V.EVIDENCE)
    g = t["g3"]
    assert "100 ms p95 by 2026-08-01" in g["gate"] and "could be added" in g["gate"]
    assert g["threshold_ms"] == 100.0 and g["decision"] == "not recorded by this tool"
    seven = {b["backend"]: b["orin"] for b in g["seven_b"]}
    benches = [o for o in seven["aerialvla_bf16"] if o["p95_ms"] is not None]
    assert {o["p95_ms"] for o in benches} == {1537.8, 1099.6}
    assert all(o["what"].startswith("not R1") for o in seven["aerialvla_bf16"])
    assert "aiotvla_v10b" not in seven
    small = {b["backend"]: b["orin"] for b in g["smaller"]}
    assert small["aiotvla_v10b"][0]["p95_ms"] == 41.7
    # what the board measured on 16 Nov, each from its own committed file
    r1 = json.loads((HIL_EVIDENCE / "vla_rate_openvla_7b_bf16.json").read_text(encoding="utf-8"))
    bf16 = seven["openvla_7b_bf16"]
    assert bf16[0]["what"] == "R1" and bf16[0]["p95_ms"] > g["threshold_ms"]
    assert abs(bf16[0]["p95_ms"] - r1["stats"]["p95_ms"]) < 1e-6, (bf16[0], r1["stats"])
    assert any(o["what"].startswith("not R1") and o["p95_ms"] is None for o in bf16)  # the 8-answer gate check
    rv = [o for o in seven["racevla_7b"] if o["p95_ms"] is not None]
    assert len(rv) == 1 and rv[0]["what"].startswith("not R1") and rv[0]["p95_ms"] > g["threshold_ms"]
    i8 = json.loads((HIL_EVIDENCE / "vla_rate_openvla_7b_int8wo.json").read_text(encoding="utf-8"))
    assert i8["status"] == "refused" and g["int8_on_orin"] == "refused"
    md = V.render_markdown(t)
    assert "Whether the gate is added and applied is not recorded by this tool." in md
    for claim in ("gate applied", "gate is applied", "the PI"):
        assert claim not in md, claim


# --------------------------------------------------------------------------- #
# the measure keys: OpenVLA-7B bf16 and the int8 weight-only attempt
# --------------------------------------------------------------------------- #
class _Par:
    def __init__(self, dtype, device):
        self.dtype, self.device = dtype, device


class _FakeTorch:
    bfloat16 = "bf16"

    class _NG:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    def no_grad(self):
        return self._NG()


class _FakeVLA:
    """A model with parameters, answering seven finite numbers."""

    def __init__(self, dtype="torch.bfloat16", device="cuda:0", fail_at=None):
        self.par, self.calls, self.fail_at = _Par(dtype, device), 0, fail_at

    def parameters(self):
        return iter([self.par])

    def predict_action(self, **kw):
        assert set(kw) == {"input_ids", "pixel_values", "unnorm_key", "do_sample"}, kw
        self.calls += 1
        if self.fail_at == self.calls:
            raise RuntimeError("quantized linear has no kernel for this device")
        return [0.0] * 6 + [1.0]


def _fake_proc(prompt, img):
    return _FakeInputs(input_ids=[1], pixel_values=[0])


_FRAMES = ([_FakeImg()], {"frames": [{"file": "a.png"}]})


def test_openvla_bf16_is_timed_through_the_orin_checks_loader_and_refused_out_of_bf16():
    import real_vla_demo as RV
    check = V._bf16_check_module()
    # one bf16 loader: the Orin gate check's own, which that check itself uses
    src = (ROOT / "deploy" / "orin" / "vla_bf16_check.py").read_text(encoding="utf-8")
    assert callable(check.load_bf16) and "proc, model, _ = load_bf16(model_dir)" in src
    assert "bitsandbytes" not in src.split("def load_bf16")[1].split("def collect")[0].replace(
        "It needs no bitsandbytes", "")
    assert src.count("AutoModelForVision2Seq.from_pretrained(") == 1, "a second loader"
    # R1 for this key asks what the gate check asked: its prompt and its unnorm key
    assert RV.build_openvla_prompt("fly forward and avoid restricted areas") == check.PROMPT
    assert RV.UNNORM_KEY == check.UNNORM_KEY
    seen = []

    def load(model_dir):
        seen.append(model_dir)
        return _fake_proc, _FakeVLA(), _FakeTorch()

    proc, model, torch = V.load_openvla_bf16(Path("/models/openvla-7b"), load=load)
    assert seen and seen[0].replace("\\", "/").endswith("/models/openvla-7b")
    for dtype, device in (("torch.bfloat16", "cpu"), ("torch.float32", "cuda:0"),
                          ("torch.uint8", "cuda:0")):
        try:
            V.load_openvla_bf16("/m", load=lambda d: (_fake_proc, _FakeVLA(dtype, device),
                                                      _FakeTorch()))
        except RuntimeError as exc:
            assert "refusing to time it as bf16" in str(exc), exc
        else:
            raise AssertionError(f"{dtype} on {device} was accepted as the bf16 row")
    # through `measure`: a GPU key (refused without --allow-gpu), recorded as bf16
    assert V.OPENVLA_PRECISION["openvla_7b_bf16"] == "bf16"
    assert "openvla_7b_bf16" in V.GPU_BACKENDS
    res = V.measure("openvla_7b_bf16", "dev", model_root=Path("/models"), allow_gpu=True,
                    warmup=5, timed=50, frames=_FRAMES,
                    loader=lambda: (_fake_proc, _FakeVLA(), _FakeTorch()))
    assert res["status"] == "measured" and res["precision"] == "bf16"
    assert "vla_bf16_check.py" in res["loader"] and res["stats"]["n"] == 50
    assert res["param"] == {"param_dtype": "torch.bfloat16", "param_device": "cuda:0"}
    assert res["protocol"] == V.PROTOCOL_ID and res["test"] == "R1"
    # the 4-bit key still loads through the flight script's loader
    assert "real_vla_demo.py" in V._OPENVLA_LOADER_TEXT["nf4"]
    try:
        V.measure_openvla(Path("/models"), precision="int8", frames=_FRAMES,
                          loader=lambda: (_fake_proc, _FakeVLA(), _FakeTorch()))
    except ValueError as exc:
        assert "not one of" in str(exc)
    else:
        raise AssertionError("an unknown precision was timed")


def test_the_int8_attempt_records_its_refusal_with_stage_and_cause():
    loads = []

    def load(model_dir):
        loads.append(model_dir)
        return _fake_proc, _FakeVLA(), _FakeTorch()

    # no torchao: refused before the 15 GB load is even started
    try:
        V.load_openvla_int8wo("/m", load=load, find_spec=lambda name: None)
    except V.MeasureRefused as exc:
        assert exc.stage == "import" and "torchao is not installed" in exc.cause
    else:
        raise AssertionError("int8 weight-only without torchao was not refused")
    assert loads == [], "the model was loaded although torchao is absent"

    def no_kernel(model):
        raise NotImplementedError("no int8 kernel for sm_87")

    try:
        V.load_openvla_int8wo("/m", load=load, quantizer=no_kernel)
    except V.MeasureRefused as exc:
        assert exc.stage == "quantize" and "NotImplementedError: no int8 kernel" in exc.cause
    else:
        raise AssertionError("a failed quantisation was not refused")
    # a bf16 load that fails is not an int8 result: a failed run, not a refusal
    def no_weights(model_dir):
        raise OSError(f"{model_dir} does not appear to have a file named config.json")
    for bad in (lambda d: (_fake_proc, _FakeVLA(device="cpu"), _FakeTorch()), no_weights):
        try:
            V.load_openvla_int8wo("/m", quantizer=lambda m: None, load=bad)
        except V.MeasureRefused as exc:
            raise AssertionError(f"a failed bf16 load was recorded as an int8 refusal at "
                                 f"{exc.stage}") from None
        except RuntimeError as exc:
            assert "not an int8 result" in str(exc) and "--model-root" in str(exc), exc
        else:
            raise AssertionError("a model off the GPU was quantised and timed")
    # ... and the command fails (exit 1) and writes no result file
    out = _tmp() / "int8.json"
    real = V.load_openvla_int8wo
    V.load_openvla_int8wo = lambda d, **k: real(d, quantizer=lambda m: None, load=no_weights)
    try:
        rc = V.main(["measure", "--backend", "openvla_7b_int8wo", "--topology", "dev",
                     "--allow-gpu", "--model-root", "/nowhere", "--out", str(out)])
    finally:
        V.load_openvla_int8wo = real
    assert rc == 1 and not out.exists(), rc
    # a quantised model that cannot answer: stage inference; a bf16 one just raises
    def broken():
        return _fake_proc, _FakeVLA(fail_at=2), _FakeTorch()
    try:
        V.measure_openvla(Path("/models"), warmup=1, timed=3, precision="int8wo",
                          frames=_FRAMES, loader=broken)
    except V.MeasureRefused as exc:
        assert exc.stage == "inference" and "inference 2" in exc.cause
    else:
        raise AssertionError("a failing int8 inference was not refused")
    try:
        V.measure_openvla(Path("/models"), warmup=1, timed=3, precision="bf16",
                          frames=_FRAMES, loader=broken)
    except V.MeasureRefused:
        raise AssertionError("a bf16 failure was recorded as an int8 refusal") from None
    except RuntimeError:
        pass
    # `measure` turns the refusal into the result: no timing, the cause, what is installed
    def refuse():
        raise V.MeasureRefused("import", "torchao is not installed in this environment")
    res = V.measure("openvla_7b_int8wo", "dev", model_root=Path("/models"), allow_gpu=True,
                    frames=_FRAMES, loader=refuse)
    assert res["status"] == "refused" and res["stats"] == {"n": 0} and res["latency_s"] == []
    assert res["refusal"]["stage"] == "import" and "torchao" in res["refusal"]["cause"]
    assert set(res["refusal"]["packages"]) >= {"torch", "torchao", "bitsandbytes"}
    assert res["precision"] == "int8wo" and res["undecodable_outputs"] is None
    # and when it does quantise, it is timed like any other key
    done = []
    res = V.measure("openvla_7b_int8wo", "dev", model_root=Path("/models"), allow_gpu=True,
                    warmup=2, timed=4, frames=_FRAMES,
                    loader=lambda: V.load_openvla_int8wo("/m", load=load,
                                                         quantizer=done.append))
    assert res["status"] == "measured" and res["stats"]["n"] == 4 and len(done) == 1
    try:
        V.measure("openvla_7b_int8wo", "dev", model_root=Path("/models"))
    except PermissionError as exc:
        assert "--allow-gpu" in str(exc)
    else:
        raise AssertionError("the int8 attempt would have loaded a 7 B model unasked")


def test_a_recorded_refusal_shows_in_the_table_as_an_attempt_not_as_a_rate():
    def refused(is_orin):
        r = _rate(is_orin, "openvla_7b_int8wo")
        r.update({"status": "refused", "stats": {"n": 0}, "created_utc": "2026-11-17T03:00:00Z",
                  "refusal": {"stage": "import",
                              "cause": "torchao is not installed in this environment",
                              "packages": {"torchao": None}}})
        return r
    ev = _tmp()
    (ev / "hil").mkdir()
    (ev / "hil" / "vla_rate_openvla_7b_int8wo.json").write_text(json.dumps(refused(True)),
                                                               encoding="utf-8")
    t = _build(evidence=ev)
    cell = _row(t, "openvla_7b_int8wo")["cells"]["rate_orin"]
    assert cell.startswith("attempted 2026-11-17, refused at import: torchao is not installed"), cell
    assert cell.endswith("R1: to measure on the Orin; needs torchao, which the AGX Orin's "
                         "native environment does not install (deploy/orin/agx_vla_env.sh)")
    assert "holds R1 for 0 of" in t["decision"], t["decision"]
    # an attempt with no timing is counted apart, never as a bench
    assert "labelled as such, for 0 backends; a recorded attempt with no timing for 1 more" \
        in t["decision"], t["decision"]
    assert not _row(t, "openvla_7b_int8wo")["orin_timed"]
    assert t["g3"]["int8_on_orin"] == "refused"
    line = next(b for b in t["g3"]["seven_b"] if b["backend"] == "openvla_7b_int8wo")["orin"][0]
    assert line["what"] == "attempt refused" and line["stage"] == "import"
    md = V.render_markdown(t)
    assert "refused at import: torchao is not installed" in md and "int8 on an Orin: refused" in md
    # the command writes the refusal and ends cleanly
    out = _tmp() / "r.json"
    real = V.measure
    V.measure = lambda *a, **k: dict(refused(False), protocol=V.PROTOCOL_ID)
    try:
        rc = V.main(["measure", "--backend", "openvla_7b_int8wo", "--topology", "dev",
                     "--allow-gpu", "--model-root", "/models", "--out", str(out)])
    finally:
        V.measure = real
    assert rc == 0 and json.loads(out.read_text(encoding="utf-8"))["status"] == "refused"


def test_the_committed_table_is_protocol_2_with_real_cells_only():
    if not COMMITTED_TABLE.is_file():
        return SKIP
    t = json.loads(COMMITTED_TABLE.read_text(encoding="utf-8"))
    assert t["protocol"]["id"] == "vla-backend-protocol/2" and t["default_backend"] is None
    md = (ROOT / "deploy" / "evidence" / "vla_backend_table.md").read_text(encoding="utf-8")
    assert "**Default backend:** none." in md
    assert t["protocol"]["C2_follow_closed_loop"]["added_after_the_choice"] in md
    benches = {r["source"]["file"]: r for r in V.extract_orin_benches(V.EVIDENCE)}
    for r in t["rows"]:
        cell = r["cells"]["rate_orin"]
        # an Orin cell is an R1 result made on an Orin, a labelled bench whose
        # file is committed and still says the same, an attempt, or "to measure"
        has_r1 = any(x["host"]["jetson"]["is_orin"] and x.get("status") != "refused"
                     for x in r["r1_results"])
        assert cell.startswith("R1") == has_r1, (r["backend"], cell)
        for rec in r["records"]:
            if rec.get("metric") == "bench" and rec.get("host_is_orin"):
                now = benches[rec["source"]["file"]]
                if not has_r1:      # an R1 on an Orin is the cell; the bench stays a record
                    assert V.bench_cell(now) in cell, (r["backend"], V.bench_cell(now))
                assert now["source"]["sha256"] == rec["source"]["sha256"], rec["source"]["file"]
        if not has_r1 and not r.get("unavailable"):
            if r["backend"] in V.SERVED_BACKENDS:
                assert "R1 does not cover a model served over protocol v2" in cell, cell
                assert "to measure" not in cell, (r["backend"], cell)
            else:
                assert "to measure on the Orin" in cell, (r["backend"], cell)
        # C2: only flights that meet it are in the cell's count, each with
        # what C2 fixes as its own record states it, and its own KPI verdict
        c2 = [f for f in r["records"] if f.get("metric") == "hil_follow" and not f["not_c2"]]
        for f in c2:
            assert f["kpi_grade"] is True and f["topology"] == "hil" and not f["stress"], f["tag"]
            assert f["identity"] and f["target_estimator"] and f["tag"] not in V.C2_NOT_ALONE
            assert f["frac_within_30m"] is not None and f["p0_escapes"] is not None, f["tag"]
            assert f["negative_query"] == V.C2_NEGATIVE_QUERY and f["max_s_given"] == V.C2_MAX_S
            assert V.C2_BUNDLE.match(f["bundle"]), f["bundle"]
            if f["vla_feed"] == "stream":
                assert f["front_capture_interval_s"] == V.C2_STREAM_CAPTURE_INTERVAL_S
            assert f["kpi_outcome"] in ("success", "fail") and \
                f["unsafe_p0_ticks"] is not None, f["tag"]
            line = next(ln for ln in md.splitlines() if ln.startswith(f"| {f['tag']} |"))
            assert f"| {V.kpi_outcome_text(f)} | {f['unsafe_p0_ticks']} |" in line, line
        c2_cell = r["cells"]["closed_loop_hil_c2"]
        assert c2_cell.startswith("C2, ") == bool(c2), (r["backend"], c2_cell)
        if c2:
            assert c2_cell.startswith(f"C2, {len(c2)} flight"), c2_cell
            assert V.kpi_verdict_text(c2) in c2_cell, c2_cell
        # a flight that flew no tick shows no P0 count
        for f in r["records"]:
            if f.get("metric") == "hil_follow" and f["not_c2"] and not f["ticks"]:
                line = next(ln for ln in md.splitlines() if ln.startswith(f"| {f['tag']} |"))
                assert line.endswith("| - | - (no mission) |"), line
        assert not any(f.get("protocol") == "C1" for f in r["records"]), "no C1 has been flown"
    assert all(rec["protocol"] == "not C1, not C2"
               for rec in sum((r["records"] for r in t["rows"]), []) + t["other_records"]
               if rec.get("metric") == "follow_matrix" and not rec.get("status"))
    # no default is named while the set does not compare the backends
    assert t["comparison_gaps"] and "What the set lacks before" in md


def test_a_rate_result_under_another_protocol_is_listed_not_shown_as_r1():
    ev = _tmp()
    (ev / "hil").mkdir()
    r = _rate(True)
    r["protocol"] = "vla-backend-protocol/9"
    (ev / "hil" / "vla_rate_stub.json").write_text(json.dumps(r), encoding="utf-8")
    stub = _row(_build(evidence=ev), "stub")
    assert stub["cells"]["rate_orin"] == "to measure on the Orin (R1)", stub["cells"]
    other = [m for m in stub["records"] if m["metric"] == "rate_other_protocol"]
    assert other and "'vla-backend-protocol/9'" in other[0]["status"], stub["records"]
    # a result made under /1 is R1 of /2 as well (R1 is word for word /1's)
    r["protocol"] = "vla-backend-protocol/1"
    (ev / "hil" / "vla_rate_stub.json").write_text(json.dumps(r), encoding="utf-8")
    assert _row(_build(evidence=ev), "stub")["cells"]["rate_orin"].startswith("R1: ")
    assert set(V.R1_PROTOCOLS) == {"vla-backend-protocol/1", V.PROTOCOL_ID}


def test_the_c1_sentences_are_counted_from_the_records_not_written_in():
    t = _build()
    assert "C1 has been flown by no backend" in t["decision"]
    assert "## C1 on hil: flown by no backend, and why" in V.render_markdown(t)
    # a C1 record, once a reader yields one, is counted and named
    row = _row(t, "stub")
    row["records"].append({"backend": "stub", "metric": "closed_loop", "protocol": "C1",
                           "source": {"file": "x"}, "status": "test"})
    assert "## C1 on hil: flown by " + row["label"] in V.render_markdown(t)
    assert "not flown on hil by any backend" not in json.dumps(V.PROTOCOL)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs a file outside git, torch, or a non-Jetson host)")
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
