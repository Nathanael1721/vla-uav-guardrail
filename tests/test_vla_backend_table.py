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
        assert "openvla_7b_4bit only" in str(exc)
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
            assert r["cells"][col].startswith("R1"), (r["backend"], col, r["cells"])
    assert checked > 0 or not (ROOT / "training" / "eval_ft2_deploy.json").is_file()


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
