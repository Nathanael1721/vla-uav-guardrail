"""One protocol for comparing VLA backends, and the table of what is measured where.

    python tools/vla_backend_table.py protocol                    # the protocol, as JSON
    python tools/vla_backend_table.py table                       # assemble from files on disk
    python tools/vla_backend_table.py measure --backend stub  --topology dev
    python tools/vla_backend_table.py measure --backend bc_v3 --topology dev
    python tools/vla_backend_table.py measure --backend aerialvla_ft_run2 \
        --allow-gpu --model-root /models --topology hil           # on the Jetson Orin

WHY THIS EXISTS

The grant (Architecture constraints p.2) names the backends the slot must take
- "CognitiveDrone, OpenVLA generic, BitVLA, in-house stubs" - and says the
project "does not commit to any one of them as a 'default' until the Q2
measurement set lands". Audit cards ARCH-30 and X-04 found no such set: a few
desktop numbers from July and August, each taken under different conditions,
and a mid-term table (docs/MIDTERM-REPORT-Aug2026.md section 6.4) with no rate
column and no common scenario. ARCH-03 adds that no VLA has been timed on the
Jetson Orin the grant measures latency on.

This tool fixes the protocol first, then fills the table from what is already
on disk - citing each file and the conditions it was measured under - and marks
every cell that has to be measured on the Orin.

THE PROTOCOL (vla-backend-protocol/1; `protocol` prints it in full)

  R1  inference rate, open loop. Fixed inputs committed with this tool
      (deploy/evidence/vla_protocol_frames/: two 224x448 front-over-down
      mosaics captured in Project AirSim at recorded poses; OpenVLA gets the
      front half), or for state policies the packed states of the dev-rail
      flight ros2_shield_on. Load once, 5 untimed warm-up inferences, 50 timed,
      greedy decoding, nothing else on the GPU. Per-inference wall time from
      input ready to action decoded: p50, p95, max, Hz at p50, peak VRAM, and
      the number of outputs that did not decode to an action (never dropped).
      Reference line: the grant's 10 Hz (100 ms per action). Floor: the stub on
      the same host.
  C1  closed-loop success. policies/urban_demo_policy.yaml, "fly to (40, 40)
      at 6 m/s altitude 20", 5 seeds, Shield on, 120 s limit, reached = within
      3 m of the target. No assist: goal blend 0, no planner, no path
      follower; a run with any assist is recorded as assisted and is not C1.
      Reported: reached n/N, time to reach, Shield interventions per 100 ticks,
      P0 escapes (must be 0), achieved action rate in the loop. Nulls: hover
      (zero action) and the stub, which is handed the target coordinates - a
      learned backend that does not beat the stub has shown nothing on C1.

WHAT `measure` WILL AND WILL NOT DO

`stub` and `bc_v3` run on the CPU (bc_v3 is a 0.54 M-parameter TorchScript MLP
and is loaded with CUDA hidden; a model that lands anywhere but the CPU is
refused, not timed). The three OpenVLA-based backends load a 7 B model onto
the GPU, so they refuse to run without `--allow-gpu`; they reuse the loaders
and the predict call of the flight scripts (demo/real_vla_demo.py,
demo/aerialvla_demo.py) rather than a second copy. The OpenVLA prompt is the
flight script's default (`--csp off`); `--csp on` times the longer prompt
with the Constraint Summary Pack ahead of the question, as `real_vla_demo.py
--csp on` flies it. Either way the prompt and its token count are recorded,
because the rate depends on the prompt length. CognitiveDrone and BitVLA have
no weights and no loader on this machine (see the table).

Results are written to <evidence>/<topology>/vla_rate_<backend>.json, where
<evidence> is $VLAGUARD_EVIDENCE_OUT or deploy/evidence (the Orin compose file
sets the git-ignored deploy/evidence/incoming; see tools/profile_shield_tick.py).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

TABLE_SCHEMA = "vlaguard.vla_backend_table/2"
# /2: the host is judged by guardrail.manifest's Orin rule (host.jetson.is_orin).
RATE_SCHEMA = "vlaguard.vla_rate/2"
PROTOCOL_ID = "vla-backend-protocol/1"

EVIDENCE = ROOT / "deploy" / "evidence"
FRAMES_DIR = EVIDENCE / "vla_protocol_frames"
DEFAULT_TABLE_JSON = EVIDENCE / "vla_backend_table.json"
DEFAULT_TABLE_MD = EVIDENCE / "vla_backend_table.md"
CONTRACT_HZ = 10.0
CONTRACT_MS = 1000.0 / CONTRACT_HZ

PROTOCOL = {
    "id": PROTOCOL_ID,
    "R1_rate": {
        "kind": "open loop",
        "inputs": {
            "camera_vla": "deploy/evidence/vla_protocol_frames/*.png (224x448 front-over-down "
                          "mosaics, Project AirSim, poses in frames.json); OpenVLA-7B is given "
                          "the front (upper) 224x224 half",
            "state_policy": "rows of flight ros2_shield_on in deploy/evidence/shield_tick_pack.json, "
                            "mission 'fly to the northeast pad at 6 m/s' under "
                            "policies/sim_demo_policy.yaml (the dev rail's own mission)",
        },
        "warmup": 5, "timed": 50,
        "decoding": "greedy; the flight scripts' own max_new_tokens",
        "isolation": "the model's process alone on the GPU: no simulator, no second model",
        "timer": "wall time from input ready to action decoded, per inference",
        "outputs": ["p50_ms", "p95_ms", "max_ms", "hz_at_p50", "peak_vram_gb", "load_s",
                    "undecodable_outputs"],
        "reference": f"the grant's {CONTRACT_HZ:g} Hz VLA -> Shield rate ({CONTRACT_MS:g} ms)",
        "floor": "the stub on the same host",
        "hosts": ["desktop (dev)", "Jetson Orin (hil), same container image definition"],
    },
    "C1_closed_loop": {
        "scenario": "policies/urban_demo_policy.yaml; instruction 'fly to (40, 40) at 6 m/s "
                    "altitude 20'; spawn (0, 0); the policy's no-fly zone lies on the "
                    "straight line",
        "rail": "hil: ArduPilot SITL + MAVROS 2, Shield and VLA on the Orin; camera VLAs need "
                "Project AirSim for physics and camera (ardupilot-api controller)",
        "seeds": 5, "time_limit_s": 120, "reached_radius_m": 3.0,
        "assist": "none: goal blend 0, no planner, no path follower. Assisted runs are "
                  "recorded as assisted and are not C1.",
        "outputs": ["reached_n_of_N", "time_to_reach_s", "interventions_per_100_ticks",
                    "p0_escapes", "action_rate_hz_in_loop"],
        "nulls": ["hover (zero action): reached 0/N by construction",
                  "stub P-controller, given the target coordinates: the no-learning "
                  "reference a learned backend has to beat"],
    },
}

# What each backend is, where its weights live, and the code that flies it.
# `weights_root` paths are relative to --model-root (D:/models on the lab
# desktop, /models in the containers); `weights_repo` paths to the repository.
BACKENDS = {
    "stub": {
        "label": "Stub (hand-written P-controller, no model)",
        "kind": "code", "inputs": "state + target coordinates",
        "weights_repo": [], "weights_root": [], "code": "guardrail/vla_stub.py",
        "role": "the null; pilot of the stored dev-rail runs (demo/out/ros2_*, not "
                "KPI-bearing: the grant takes KPI figures from hil)",
    },
    "bc_v3": {
        "label": "Behaviour-cloned state policy (vla_policy_v2)",
        "kind": "state + fence-geometry policy, MLP, 26 features",
        "inputs": "state + policy geometry (no camera, no language)",
        "weights_repo": ["models/vla_policy_v2.pt", "models/vla_policy_v2.onnx"],
        "weights_root": [], "code": "guardrail/vla_bc.py (BCVLAv3)",
    },
    "openvla_7b_4bit": {
        "label": "OpenVLA-7B, 4-bit NF4",
        "kind": "camera + language VLA (arm-trained; unnorm_key bridge_orig)",
        "inputs": "224x224 image + instruction",
        "weights_repo": [], "weights_root": ["openvla-7b"], "hf_id": "openvla/openvla-7b",
        "code": "demo/real_vla_demo.py (OpenVLABackend)",
    },
    "aerialvla_lora": {
        "label": "AerialVLA LoRA on OpenVLA-7B, 4-bit",
        "kind": "camera + language VLA, UAV-trained adapter",
        "inputs": "224x448 front+down mosaic + direction phrase + object phrase",
        "weights_repo": [], "weights_root": ["openvla-7b", "aerialvla-lora/aero_vla"],
        "hf_id": "XuPeng23/AerialVLA", "code": "demo/aerialvla_demo.py (AerialVLABackend)",
    },
    "aerialvla_ft_run2": {
        "label": "AerialVLA + our QLoRA fine-tune (run2/epoch1), 4-bit",
        "kind": "camera + language VLA, fine-tuned on 1,961 self-collected samples",
        "inputs": "as AerialVLA",
        "weights_repo": ["models/aerialvla_deploy_manifest.json"],
        "weights_root": ["openvla-7b", "aerialvla-ft/run2/epoch1"],
        "code": "demo/aerialvla_demo.py (AerialVLABackend, --adapter)",
    },
    "cognitivedrone": {
        "label": "CognitiveDrone (the grant's action-space reference)",
        "kind": "camera + language VLA (OpenVLA-7B fine-tune, 4-D actions)",
        "inputs": "FPV image + instruction",
        "weights_repo": [], "weights_root": [], "code": None,
        "unavailable": ("no checkpoint on this machine. The saved project page "
                        "(source/pages/cognitivedrone.html) links a dataset "
                        "(huggingface ArtemLykov/cognitiveDrone_dataset) and a data-collector "
                        "repository, no model weights; the paper is "
                        "source/papers/2503.02572-CognitiveDrone.pdf"),
    },
    "bitvla": {
        "label": "BitVLA (named by the grant among the switchable backends)",
        "kind": "quantised / bit-efficient camera + language VLA",
        "inputs": "image + instruction",
        "weights_repo": [], "weights_root": [], "code": None,
        "unavailable": ("no checkpoint and no loader in this repository. The grant lists it "
                        "with CognitiveDrone, OpenVLA and in-house stubs as backends the "
                        "slot must accept (Architecture constraints p.2-3); the reference "
                        "design names it as a smaller backend to swap in if a 7 B model "
                        "misses 10 Hz on the Orin (docs/04-risks-and-fallbacks/"
                        "fallback-gates.md), with its citation still to be added"),
    },
}
GPU_BACKENDS = ("openvla_7b_4bit", "aerialvla_lora", "aerialvla_ft_run2")
CPU_BACKENDS = ("stub", "bc_v3")

# The assisted evaluation behind the retracted row, its null (the adapter
# before fine-tuning, same assist), and the unassisted runs of both. Read from
# the files every time, so the correction carries the numbers on disk.
ASSIST_EVIDENCE = ("training/eval_ft2_deploy.json", "training/eval_base_deploy.json",
                   "training/eval_ft3_deploy.json", "training/eval_ft2_e1.json",
                   "training/eval_ft2_yg04.json", "training/eval_baseline.json")
_ASSIST_WHY = (
    "the fine-tune's 5/5 reached and 0.996 efficiency (training/eval_ft2_deploy.json) "
    "were flown with goal_blend 0.55, i.e. with the mission-direction assist ON. Its null "
    "does not separate it from the adapter before fine-tuning: under the same assist the "
    "original AerialVLA adapter also reached 5/5 (efficiency 0.942, "
    "training/eval_base_deploy.json), and the run3 fine-tune also reached 5/5 "
    "(training/eval_ft3_deploy.json). Without the assist (goal_blend 0) the run2 "
    "fine-tune reached 0/5 at yaw gain 1.0 (training/eval_ft2_e1.json) and 1/5 at yaw "
    "gain 0.4 (training/eval_ft2_yg04.json), and the original adapter 1/5 "
    "(training/eval_baseline.json). Correct statement: 5/5 reached (efficiency 0.996) "
    "with mission-direction assist (goal_blend 0.55); the original adapter under the "
    "same assist also reached 5/5 (0.942); without assist 0/5 (yaw gain 1.0) and 1/5 "
    "(yaw gain 0.4)")
_NO_ASSIST_WORDS = ("assist", "goal_blend", "goal blend")

# Statements that a file on disk contradicts. Each is checked every time the
# table is built, so a correction that is never made keeps showing up. A line
# is flagged when it holds every `must_contain` string and none of the
# `must_not_contain` ones, and the `file` it is about shows the assist was on.
CLAIM_CHECKS = [
    {
        "doc": "docs/MIDTERM-REPORT-Aug2026.md",
        "must_contain": ["0.996", "goal assist off"], "must_not_contain": [],
        "file": "training/eval_ft2_deploy.json",
        "field": "results[*].params.goal_blend",
        "note": "states the assisted result as 'goal assist off'",
        "why": _ASSIST_WHY,
    },
    {
        # The generator of the line above: a hand fix to the .md would be
        # overwritten the next time the report is built.
        "doc": "tools/build_report_results.py",
        "must_contain": ["0.996", "goal assist off"], "must_not_contain": [],
        "file": "training/eval_ft2_deploy.json",
        "field": "results[*].params.goal_blend",
        "note": "generates docs/MIDTERM-REPORT-Aug2026.md section 6.4, so a hand fix to "
                "the report would be overwritten",
        "why": _ASSIST_WHY,
    },
    {
        # The mid-term deck states the number with no condition at all.
        "doc": "tools/deck/build_midterm_deck.js",
        "must_contain": ["0.996", "reached"], "must_not_contain": list(_NO_ASSIST_WORDS),
        "file": "training/eval_ft2_deploy.json",
        "field": "results[*].params.goal_blend",
        "note": "states 0.996 with no assist condition",
        "why": _ASSIST_WHY,
    },
]


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
from profile_shield_tick import write_lf  # noqa: E402  (LF on every OS)


def _rel(p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return Path(p).as_posix()


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with Path(p).open("rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _mtime_date(p: Path) -> str:
    return datetime.fromtimestamp(Path(p).stat().st_mtime).strftime("%Y-%m-%d")


def _src(p: Path) -> dict:
    return {"file": _rel(p), "sha256": _sha256(p), "file_date": _mtime_date(p)}


def missing(backend: str, metric: str, path: Path, what: str) -> dict:
    """A record that says a source is absent here, rather than a zero."""
    return {"backend": backend, "metric": metric, "status": "source missing on this host",
            "source": {"file": _rel(path)}, "what": what}


def latency_stats(seconds: list[float]) -> dict:
    """Latency list in seconds -> ms percentiles (nearest rank) and Hz at p50."""
    if not seconds:
        return {"n": 0}
    ms = sorted(s * 1e3 for s in seconds)

    def rank(q):
        return ms[min(len(ms) - 1, max(0, int(math.ceil(q * len(ms))) - 1))]

    p50 = statistics.median(ms)
    return {"n": len(ms), "p50_ms": p50, "p95_ms": rank(0.95), "max_ms": ms[-1],
            "mean_ms": statistics.fmean(ms), "hz_at_p50": (1000.0 / p50) if p50 > 0 else None,
            "meets_contract_p95": rank(0.95) <= CONTRACT_MS}


def weights_status(backend: str, model_root: Path) -> list[dict]:
    """Where each weight file or directory should be, whether it is there, and
    its size. Never hashes a 15 GB directory: presence and bytes only."""
    spec = BACKENDS[backend]
    out = []
    for rel in spec.get("weights_repo") or []:
        p = ROOT / rel
        out.append({"path": rel, "where": "repository", "present": p.exists(),
                    "bytes": p.stat().st_size if p.is_file() else None})
    for rel in spec.get("weights_root") or []:
        p = Path(model_root) / rel
        size = None
        if p.is_dir():
            size = sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
        elif p.is_file():
            size = p.stat().st_size
        out.append({"path": f"<model-root>/{rel}", "where": str(Path(model_root).as_posix()),
                    "present": p.exists(), "bytes": size})
    return out


# --------------------------------------------------------------------------- #
# Extractors: what is already on disk
# --------------------------------------------------------------------------- #
_INFER = re.compile(r"infer\s+([0-9.]+)\s*s")
_LOADED = re.compile(r"MODEL_LOADED(?:_4BIT)?\s+(?:([0-9.]+)\s*s\s+)?VRAM\s+([0-9.]+)\s*GB")


def extract_openvla_probes(demo_out: Path) -> list[dict]:
    """demo/out/_vla4bit.txt (4-bit) and _vlaload.txt (bf16): two standalone
    load probes, two inferences each, no simulator."""
    recs = []
    for name, quant in (("_vla4bit.txt", "4-bit NF4"), ("_vlaload.txt", "bf16 (unquantised)")):
        p = Path(demo_out) / name
        if not p.is_file():
            recs.append(missing("openvla_7b_4bit", "latency", p,
                                f"OpenVLA-7B {quant} load probe"))
            continue
        text = p.read_text(encoding="utf-8", errors="replace")
        lat = [float(v) for v in _INFER.findall(text)]
        m = _LOADED.search(text)
        recs.append({
            "backend": "openvla_7b_4bit", "metric": "latency", "protocol": "not R1",
            "quantisation": quant, "values_s": lat,
            "load_s": float(m.group(1)) if m and m.group(1) else None,
            "vram_gb": float(m.group(2)) if m else None,
            "warm_latency_s": lat[-1] if len(lat) > 1 else None,
            "hz_warm": (1.0 / lat[-1]) if len(lat) > 1 and lat[-1] > 0 else None,
            "n": len(lat),
            "conditions": "standalone load probe on the lab desktop, no simulator; the first "
                          "call includes warm-up; n is far below R1's 50",
            "source": _src(p)})
    return recs


def extract_aerialvla_flight_rates(results: Path) -> list[dict]:
    """experiments/out/results.jsonl: the eight simultaneity flights. Their
    inference ran inside the flight process while Project AirSim rendered on
    the same GPU. The flights are fixed-length (the script ends on timeout or
    error only), so they carry no success figure."""
    if not Path(results).is_file():
        return [missing("aerialvla_lora", "inference_hz_in_flight", results,
                        "AerialVLA simultaneity flights")]
    by_adapter: dict[str, list[dict]] = {}
    for line in Path(results).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        by_adapter.setdefault(str(r.get("adapter")), []).append(r)
    recs = []
    for adapter, rows in sorted(by_adapter.items()):
        backend = "aerialvla_ft_run2" if "run2" in adapter else (
            "aerialvla_lora" if "aero_vla" in adapter else "unknown")
        hz = [r["inference_hz"] for r in rows if r.get("inference_hz") is not None]
        recs.append({
            "backend": backend, "metric": "inference_hz_in_flight", "protocol": "not R1",
            "adapter": adapter, "flights": len(rows),
            "hz_min": min(hz) if hz else None, "hz_max": max(hz) if hz else None,
            "inferences": sum(int(r.get("n_inference") or 0) for r in rows),
            "terminated_by": sorted({str(r.get("terminated_by")) for r in rows}),
            "conditions": "in flight, Project AirSim rendering on the same GPU, inference in a "
                          "background thread of the flight process; fixed-length flights "
                          "(no reached-target ending), so no success figure",
            "source": _src(results)})
    return recs


def extract_stated_rates() -> list[dict]:
    """Rates that are stated in a text but have no per-inference file behind
    them. Recorded as stated, so the table never shows them as measured."""
    recs = []
    log = ROOT / "training" / "research_log.md"
    if log.is_file():
        text = log.read_text(encoding="utf-8", errors="replace")
        for i, line in enumerate(text.splitlines(), 1):
            if "0.8 Hz inference" in line:
                recs.append({"backend": "aerialvla_ft_run2", "metric": "inference_hz",
                             "protocol": "stated only", "value_hz": 0.8,
                             "conditions": "fine-tuning campaign evaluation, AirSimNH; no "
                                           "per-inference timing file was kept",
                             "source": {"file": f"{_rel(log)}:{i}"}})
                break
    srv = ROOT / "demo" / "vla_server.py"
    if srv.is_file():
        text = srv.read_text(encoding="utf-8", errors="replace")
        if "separate process" in text and "225 ms/token" in text:
            recs.append({"backend": "aerialvla_lora", "metric": "seconds_per_action",
                         "protocol": "stated only",
                         "value": "about 2.5 s per action in a separate process vs about 12 s "
                                  "inside the 10 Hz flight process",
                         "conditions": "one GPU, a flight airborne and two 7 B models resident; "
                                       "measured while writing demo/vla_server.py, no file kept",
                         "source": {"file": _rel(srv)}})
    return recs


def extract_aerialvla_evals(training: Path) -> list[dict]:
    """training/eval_*.json: closed-loop episodes in AirSimNH (classic AirSim
    1.8), 5 random targets each, with the assist recorded per run."""
    recs = []
    files = sorted(Path(training).glob("eval_*.json"))
    if not files:
        return [missing("aerialvla_ft_run2", "closed_loop", Path(training) / "eval_*.json",
                        "AerialVLA closed-loop evaluations")]
    for p in files:
        d = json.loads(p.read_text(encoding="utf-8"))
        res = d.get("results") or []
        adapter = str(d.get("adapter") or "")
        if "aero_vla" in adapter:
            backend = "aerialvla_lora"
        elif "run2/epoch1" in adapter:
            backend = "aerialvla_ft_run2"
        else:
            backend = "aerialvla_ft_other"
        blends = sorted({r.get("params", {}).get("goal_blend") for r in res})
        yaws = sorted({r.get("params", {}).get("yaw_gain") for r in res})
        reached = sum(1 for r in res if r.get("reached"))
        assisted = any((b or 0) > 0 for b in blends)
        recs.append({
            "backend": backend, "metric": "closed_loop", "protocol": "not C1",
            "adapter": adapter, "episodes": len(res), "reached": reached,
            "goal_blend": blends, "yaw_gain": yaws, "assisted": assisted,
            "mean_efficiency": d.get("mean_efficiency"),
            "mean_interventions": d.get("mean_interventions"),
            "nfz_total_s": d.get("nfz_total_s"),
            "conditions": "AirSimNH (classic AirSim 1.8), 5 random targets, Shield on, "
                          "training/eval_aerialvla.py; "
                          + ("ASSISTED: goal_blend > 0 mixes a mission-direction term into "
                             "the action" if assisted else "pure VLA (goal_blend 0)"),
            "source": _src(p)})
    return recs


def extract_bc_offline(models: Path) -> list[dict]:
    p = Path(models) / "vla_policy_v2.metrics.json"
    if not p.is_file():
        return [missing("bc_v3", "closed_loop", p, "BC policy offline evaluation")]
    d = json.loads(p.read_text(encoding="utf-8"))
    fin = d.get("final_with_altitude_aware_decoder") or {}
    return [{
        "backend": "bc_v3", "metric": "closed_loop", "protocol": "not C1",
        "episodes": fin.get("eval_episodes"), "reach_rate_unshielded": fin.get("reach_rate"),
        "reach_rate_shielded": fin.get("shielded_reach_rate"),
        "nfz_entry_rate_unshielded": fin.get("nfz_entry_rate"),
        "intervention_rate": fin.get("intervention_rate"),
        "conditions": "offline kinematic rollouts of held-out scenarios "
                      "(training/evaluate_policy.py), not a simulator and not ArduPilot",
        "source": _src(p)}]


def extract_stub_dev_rail(demo_out: Path) -> list[dict]:
    """The stored dev-rail runs: ArduPilot SITL + MAVROS 2, flown by the stub."""
    recs = []
    for tag in ("ros2_shield_on", "ros2_shield_on_dynamic", "ros2_ped_on"):
        d = Path(demo_out) / tag
        mp, kp = d / "metrics.json", d / "kpi.json"
        if not mp.is_file():
            recs.append(missing("stub", "closed_loop", mp, f"dev-rail run {tag}"))
            continue
        m = json.loads(mp.read_text(encoding="utf-8"))
        k = json.loads(kp.read_text(encoding="utf-8")) if kp.is_file() else {}
        recs.append({
            "backend": "stub", "metric": "closed_loop", "protocol": "not C1", "run": tag,
            "reached": bool(m.get("reached")), "ticks": m.get("ticks"),
            "interventions": m.get("interventions"),
            "p0_violation_escape_rate": k.get("p0_violation_escape_rate"),
            "conditions": "dev topology: ArduPilot SITL + MAVROS 2 on one desktop, "
                          "policies of the sim demo, one flight",
            "source": _src(mp)})
    return recs


def extract_openvla_closed_loop_statement() -> list[dict]:
    doc = ROOT / "docs" / "MIDTERM-REPORT-Aug2026.md"
    if not doc.is_file():
        return []
    for i, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), 1):
        if line.startswith("| OpenVLA-7B, 4-bit") and "553 ticks" in line:
            return [{"backend": "openvla_7b_4bit", "metric": "closed_loop",
                     "protocol": "stated only",
                     "value": "553 ticks of a 10 Hz control loop, 10 Shield interventions, "
                              "NFZ 0.0 s",
                     "conditions": "the 10 Hz is the control loop, which held the last action "
                                   "between inferences; base OpenVLA was not given a target, "
                                   "so there is no success criterion; no run directory of "
                                   "this flight is on this machine",
                     "source": {"file": f"{_rel(doc)}:{i}"}}]
    return []


def eval_summary(path: Path) -> dict | None:
    """One closed-loop evaluation file as the numbers a correction quotes:
    reached n/N, the assist (goal_blend), the yaw gain and the efficiency."""
    p = Path(path)
    if not p.is_file():
        return None
    d = json.loads(p.read_text(encoding="utf-8"))
    res = d.get("results") or []
    blends = sorted({(r.get("params") or {}).get("goal_blend") for r in res},
                    key=lambda v: (v is None, v))
    return {"file": _rel(p), "adapter": d.get("adapter"),
            "reached": sum(1 for r in res if r.get("reached")), "episodes": len(res),
            "goal_blend": blends,
            "yaw_gain": sorted({(r.get("params") or {}).get("yaw_gain") for r in res},
                               key=lambda v: (v is None, v)),
            "assisted": any((b or 0) > 0 for b in blends),
            "mean_efficiency": d.get("mean_efficiency")}


def check_claims(checks=None, root: Path = ROOT) -> list[dict]:
    """Each statement in CLAIM_CHECKS that its file contradicts, today, with
    the evidence a correction has to carry (ASSIST_EVIDENCE, read now)."""
    out = []
    for c in (CLAIM_CHECKS if checks is None else checks):
        doc = Path(root) / c["doc"]
        src = Path(root) / c["file"]
        if not (doc.is_file() and src.is_file()):
            continue
        hit = None
        for i, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), 1):
            low = line.lower()
            if all(s in line for s in c["must_contain"]) and \
                    not any(w.lower() in low for w in c.get("must_not_contain") or []):
                hit = (i, line.strip())
                break
        if hit is None:
            continue
        data = json.loads(src.read_text(encoding="utf-8"))
        blends = sorted({(r.get("params") or {}).get("goal_blend")
                         for r in data.get("results") or []},
                        key=lambda v: (v is None, v))
        if any((b or 0) > 0 for b in blends):
            evidence = [s for s in (eval_summary(Path(root) / f) for f in ASSIST_EVIDENCE) if s]
            out.append({"doc": f"{c['doc']}:{hit[0]}", "claim": hit[1],
                        "contradicted_by": c["file"], "field": c["field"],
                        "value": blends, "note": c.get("note"), "why": c["why"],
                        "evidence": evidence})
    return out


# --------------------------------------------------------------------------- #
# Measurements made with the protocol
# --------------------------------------------------------------------------- #
def load_rate_results(root: Path = EVIDENCE) -> list[dict]:
    out = []
    for p in sorted(Path(root).glob("*/vla_rate_*.json")):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if d.get("schema") == RATE_SCHEMA:
            d["_file"] = _rel(p)
            out.append(d)
    return out


def _protocol_states(n: int):
    """The R1 state inputs: rows of ros2_shield_on from the committed pack."""
    from guardrail.models import State
    from profile_shield_tick import DEFAULT_PACK
    pack = json.loads(Path(DEFAULT_PACK).read_text(encoding="utf-8"))
    flight = next(f for f in pack["flights"] if f["tag"] == "ros2_shield_on")
    rows = flight["rows"]
    return [State(x=r[1], y=r[2], up=r[3], yaw_deg=r[4]) for r in rows[:n]], \
        {"file": _rel(Path(DEFAULT_PACK)), "flight": "ros2_shield_on", "rows": min(n, len(rows))}


def _load_bc_v3(mission, policy):
    from guardrail.vla_bc import BCVLAv3
    return BCVLAv3(mission, policy)


def make_cpu_backend(name: str, bc_loader=None):
    """The backend under R1 on the CPU. `bc_loader(mission, policy)` stands in
    for guardrail.vla_bc.BCVLAv3 in tests."""
    from guardrail import load_policy
    from guardrail.compiler import ConstraintCompiler
    policy = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    mission = ConstraintCompiler(policy).parse_command("fly to the northeast pad at 6 m/s")
    if name == "stub":
        from guardrail.vla_stub import StubVLA
        return StubVLA(mission), {}
    if name == "bc_v3":
        # Hide every GPU before torch is imported: this backend is measured on
        # the CPU, and nothing here may put a model on the card.
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        b = (bc_loader or _load_bc_v3)(mission, policy)
        devs = sorted({str(p.device) for p in b.model.parameters()})
        if devs != ["cpu"]:
            raise RuntimeError(f"bc_v3 loaded onto {devs}, refusing to time it as a CPU run")
        return b, {"param_devices": devs,
                   "params": sum(p.numel() for p in b.model.parameters())}
    raise ValueError(name)


def measure_cpu(name: str, warmup: int = 5, timed: int = 50, clock=time.perf_counter,
                bc_loader=None) -> dict:
    t0 = clock()
    backend, extra = make_cpu_backend(name, bc_loader=bc_loader)
    load_s = clock() - t0
    states, inputs = _protocol_states(warmup + timed)
    if len(states) < warmup + timed:
        raise RuntimeError(f"only {len(states)} protocol states, {warmup + timed} needed")
    for st in states[:warmup]:
        backend.act(st)
    lat = []
    for st in states[warmup:warmup + timed]:
        a = clock()
        backend.act(st)
        lat.append(clock() - a)
    return {"load_s": load_s, "latency_s": lat, "inputs": inputs,
            "undecodable_outputs": 0, "device": "cpu", **extra}


def _protocol_frames():
    """(front-over-down mosaics, metadata) from the committed frame set."""
    from PIL import Image
    meta = json.loads((FRAMES_DIR / "frames.json").read_text(encoding="utf-8"))
    imgs = [Image.open(FRAMES_DIR / f["file"]).convert("RGB") for f in meta["frames"]]
    return imgs, meta


def openvla_r1_prompt(RV, instruction: str, csp: str = "off") -> tuple[str, dict]:
    """The prompt R1 times, built by demo/real_vla_demo.py itself.

    "off" is that script's default (`--csp off`, the 2026-07-14 prompt byte for
    byte). "on" is `--csp on`: the Constraint Summary Pack of the script's
    default policy and mission (policies/urban_demo_policy.yaml,
    RV.DEFAULT_MISSION) ahead of the question, compiled by the script's own
    compile_flight_csp. The CSP prompt is longer, and a 7 B model's latency
    grows with prompt length, so the arm is part of the record."""
    if csp == "off":
        return RV.build_openvla_prompt(instruction), {"csp": "off"}
    if csp != "on":
        raise ValueError(f"csp {csp!r} is not 'on' or 'off'")
    from guardrail import load_policy
    from guardrail.compiler import ConstraintCompiler
    policy_path = ROOT / "policies" / "urban_demo_policy.yaml"
    policy = load_policy(policy_path)
    mission = ConstraintCompiler(policy).parse_command(RV.DEFAULT_MISSION)
    pack = RV.compile_flight_csp(policy, mission)
    return (RV.build_openvla_prompt(instruction, pack.natural_language_prompt),
            {"csp": "on", "csp_policy": _rel(policy_path), "csp_mission": RV.DEFAULT_MISSION})


def measure_openvla(model_root: Path, warmup: int = 5, timed: int = 50,
                    loader=None, frames=None, clock=time.perf_counter,
                    instruction: str = "fly forward and avoid restricted areas",
                    csp: str = "off") -> dict:
    """R1 for OpenVLA-7B 4-bit, through demo/real_vla_demo.py's own loader and
    its own predict_action call (input_ids + pixel_values ONLY - passing the
    attention mask crashes generate, see that file's worker), with the
    script's own prompt (`csp`, see openvla_r1_prompt). The prompt and its
    token count are recorded: exact with the processor's tokenizer, else a
    labelled estimate (real_vla_demo.count_tokens)."""
    sys.path.insert(0, str(ROOT / "demo"))
    import real_vla_demo as RV
    RV.MODEL_ID = str(Path(model_root) / "openvla-7b")
    prompt, prompt_arm = openvla_r1_prompt(RV, instruction, csp)
    imgs, meta = frames if frames is not None else _protocol_frames()
    fronts = [im.crop((0, 0, 224, 224)).resize(RV.IMAGE_SIZE) for im in imgs]
    t0 = clock()
    proc, model, torch = (loader or (lambda: RV.OpenVLABackend._load(None)))()
    load_s = clock() - t0
    tokens = RV.count_tokens(prompt, tokenizer=getattr(proc, "tokenizer", None))
    lat, bad = [], 0
    for k in range(warmup + timed):
        img = fronts[k % len(fronts)]
        a = clock()
        inputs = proc(prompt, img).to("cuda", dtype=torch.bfloat16)
        with torch.no_grad():
            raw = model.predict_action(input_ids=inputs["input_ids"],
                                       pixel_values=inputs["pixel_values"],
                                       unnorm_key=RV.UNNORM_KEY, do_sample=False)
        dt = clock() - a
        try:
            ok = len(list(raw)) >= 7 and all(math.isfinite(float(v)) for v in raw)
        except (TypeError, ValueError):
            ok = False
        if k >= warmup:
            lat.append(dt)
            bad += not ok
    vram = None
    try:
        vram = torch.cuda.max_memory_allocated() / 1e9
    except Exception:                                     # noqa: BLE001
        vram = None
    return {"load_s": load_s, "latency_s": lat, "undecodable_outputs": bad,
            "peak_vram_gb": vram, "prompt": prompt, "prompt_arm": prompt_arm,
            "prompt_tokens": tokens,
            "inputs": {"frames": [f["file"] for f in meta["frames"]], "crop": "front 224x224"}}


def measure_aerialvla(model_root: Path, adapter_rel: str, warmup: int = 5, timed: int = 50,
                      backend_cls=None, frames=None, timeout_s: float = 1800.0,
                      object_phrase: str = "orange barrier") -> dict:
    """R1 for an AerialVLA adapter, through demo/aerialvla_demo.AerialVLABackend
    unchanged: the protocol frames are served by its obs_factory hook and the
    per-inference timing comes from its own inference log (gen_s, cycle_s)."""
    import tempfile
    sys.path.insert(0, str(ROOT / "demo"))
    if backend_cls is None:
        import aerialvla_demo as AV
        AV.BASE_ID = str(Path(model_root) / "openvla-7b")
        backend_cls = AV.AerialVLABackend
    imgs, meta = frames if frames is not None else _protocol_frames()
    poses = [(f["x"], f["y"], f["psi"]) for f in meta["frames"]]
    tiles = [(im.crop((0, 0, 224, 224)), im.crop((0, 224, 224, 448))) for im in imgs]
    counter = {"k": 0}

    def obs_factory():
        def get():
            k = counter["k"]
            counter["k"] += 1
            front, down = tiles[k % len(tiles)]
            return front, down, poses[k % len(poses)]
        return get

    log = Path(tempfile.mkdtemp()) / "inference.jsonl"
    t0 = time.perf_counter()
    b = backend_cls(object_phrase, tuple(meta["target"]), obs_factory=obs_factory,
                    lora_id=str(Path(model_root) / adapter_rel), inference_log=log)
    load_s = time.perf_counter() - t0
    b.start()
    deadline = time.perf_counter() + timeout_s
    want = warmup + timed
    try:
        while time.perf_counter() < deadline:
            n = len(log.read_text(encoding="utf-8").splitlines()) if log.is_file() else 0
            if n >= want:
                break
            time.sleep(0.2)
    finally:
        b.stop()
    recs = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()] \
        if log.is_file() else []
    timed_recs = recs[warmup:want]
    # An output that does not decode to three action bins writes no log line,
    # so it shows only as a frame served without a line. One frame may also
    # have been in flight when the worker was stopped, which is why this is an
    # upper bound and both counts are kept.
    served = counter["k"]
    vram = None
    try:
        import torch
        vram = torch.cuda.max_memory_allocated() / 1e9
    except Exception:                                     # noqa: BLE001
        vram = None
    return {"load_s": load_s, "latency_s": [r.get("cycle_s") for r in timed_recs
                                            if r.get("cycle_s") is not None],
            "generate_s": [r.get("gen_s") for r in timed_recs],
            "new_tokens": [r.get("new_tokens") for r in timed_recs],
            "frames_served": served, "log_lines": len(recs),
            "undecodable_outputs": None,
            "undecodable_outputs_upper_bound": max(0, served - len(recs)),
            "peak_vram_gb": vram,
            "inputs": {"frames": [f["file"] for f in meta["frames"]],
                       "object_phrase": object_phrase, "hint": "from the frames' poses "
                       "to the recorded target (as trained)"},
            "complete": len(timed_recs) == timed}


def measure(name: str, topology: str, model_root: Path | None = None,
            allow_gpu: bool = False, warmup: int = 5, timed: int = 50,
            facts: dict | None = None, **kw) -> dict:
    from profile_shield_tick import check_topology_claim, host_facts
    facts = facts if facts is not None else host_facts()
    refused = check_topology_claim(topology, facts)
    if refused:
        raise PermissionError("; ".join(refused))
    if name in CPU_BACKENDS:
        if kw.pop("csp", "off") != "off":
            raise ValueError(f"{name} reads no prompt; --csp applies to openvla_7b_4bit only")
        res = measure_cpu(name, warmup, timed, **kw)
    elif name in GPU_BACKENDS:
        if not allow_gpu:
            raise PermissionError(f"{name} loads a 7 B model onto the GPU; pass --allow-gpu "
                                  f"on a host where that is intended")
        if model_root is None:
            raise ValueError("--model-root is required for a GPU backend")
        if name == "openvla_7b_4bit":
            res = measure_openvla(model_root, warmup, timed, **kw)
        else:
            rel = BACKENDS[name]["weights_root"][1]
            if kw.pop("csp", "off") != "off":
                raise ValueError(f"{name} builds its own direction + object prompt; "
                                 f"--csp applies to openvla_7b_4bit only")
            res = measure_aerialvla(model_root, rel, warmup, timed, **kw)
    else:
        raise ValueError(f"{name}: no measurement path ({BACKENDS.get(name, {}).get('unavailable', 'unknown backend')})")
    lat = latency_stats([s for s in res["latency_s"] if s is not None])
    return {"schema": RATE_SCHEMA, "protocol": PROTOCOL_ID, "test": "R1",
            "backend": name, "topology": topology,
            "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "host": facts, "warmup": warmup, "timed": timed, "stats": lat,
            "weights": weights_status(name, model_root or Path("/models")),
            **{k: v for k, v in res.items() if k != "latency_s"},
            "latency_s": res["latency_s"]}


# --------------------------------------------------------------------------- #
# The table
# --------------------------------------------------------------------------- #
def _fmt_ms(s: dict) -> str:
    if not s.get("n"):
        return "no samples"
    hz = s.get("hz_at_p50")
    rate = f"{hz:.3g} Hz" if hz is not None else "rate n/a"
    return f"p50 {s['p50_ms']:.3g} ms ({rate}), p95 {s['p95_ms']:.3g} ms, n={s['n']}"


def _r1_cell(results: list[dict]) -> str:
    """R1 results for one backend on one kind of host, one per prompt arm, so a
    CSP-prompt rate is never shown as the plain prompt's."""
    def arm(r):
        return (r.get("prompt_arm") or {}).get("csp", "off")
    parts = []
    off = [r for r in results if arm(r) == "off"]
    on = [r for r in results if arm(r) == "on"]
    if off:
        parts.append(f"R1: {_fmt_ms(off[-1]['stats'])} [{off[-1]['_file']}]")
    if on:
        parts.append(f"R1, CSP prompt: {_fmt_ms(on[-1]['stats'])} [{on[-1]['_file']}]")
    return "; ".join(parts)


def build_table(demo_out: Path = ROOT / "demo" / "out",
                experiments_out: Path = ROOT / "experiments" / "out",
                model_root: Path = Path(os.environ.get("VLAGUARD_MODEL_ROOT", "D:/models")),
                evidence: Path = EVIDENCE) -> dict:
    records = (extract_openvla_probes(demo_out)
               + extract_aerialvla_flight_rates(Path(experiments_out) / "results.jsonl")
               + extract_stated_rates()
               + extract_aerialvla_evals(ROOT / "training")
               + extract_bc_offline(ROOT / "models")
               + extract_stub_dev_rail(demo_out)
               + extract_openvla_closed_loop_statement())
    rates = load_rate_results(evidence)
    rows = []
    for bid, spec in BACKENDS.items():
        mine = [r for r in records if r.get("backend") == bid]
        r1 = [r for r in rates if r["backend"] == bid]
        # The Orin column takes only a result whose host passed
        # guardrail.manifest's Orin rule, whatever folder it was found in.
        desk = [r for r in r1 if not r["host"]["jetson"]["is_orin"]]
        orin = [r for r in r1 if r["host"]["jetson"]["is_orin"]]
        cells = {}
        if desk:
            cells["rate_desktop"] = _r1_cell(desk)
        else:
            other = [m for m in mine if m["metric"] in ("latency", "inference_hz_in_flight",
                                                        "inference_hz", "seconds_per_action")]
            cells["rate_desktop"] = ("not R1 - see records" if other else
                                     ("n/a (no weights here)" if spec.get("unavailable")
                                      else "not measured"))
        if orin:
            cells["rate_orin"] = _r1_cell(orin)
        else:
            cells["rate_orin"] = ("weights not obtained" if spec.get("unavailable")
                                  else "to measure on the Orin (R1)")
        cl = [m for m in mine if m["metric"] == "closed_loop"]
        cells["closed_loop_desktop"] = "not C1 - see records" if cl else (
            "n/a (no weights here)" if spec.get("unavailable") else "not measured")
        cells["closed_loop_hil"] = ("weights not obtained" if spec.get("unavailable")
                                    else "to measure on hil (C1)")
        rows.append({"backend": bid, **{k: v for k, v in spec.items()},
                     "weights_status": weights_status(bid, model_root),
                     "cells": cells, "records": mine,
                     "r1_results": [{k: v for k, v in r.items() if k not in ("latency_s",)}
                                    for r in r1]})
    listed = set(BACKENDS)
    n_orin = sum(1 for r in rows if r["cells"]["rate_orin"].startswith("R1:"))
    return {
        "schema": TABLE_SCHEMA, "protocol": PROTOCOL,
        "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "model_root": str(Path(model_root).as_posix()),
        "rows": rows,
        # Records for backends that are not table rows (the run1 and run3
        # fine-tunes): kept, so nothing read from disk disappears silently.
        "other_records": [r for r in records if r.get("backend") not in listed],
        "contradictions": check_claims(),
        "default_backend": None,
        "decision": (
            "The grant defers the choice to this measurement set, and "
            + (f"the Orin rate column holds {n_orin} of {len(BACKENDS)} backends"
               if n_orin else "the Orin columns are empty")
            + ". The stored dev-rail runs (demo/out/ros2_*, not KPI-bearing: the grant "
              "takes KPI figures from hil) were flown by the stub."),
    }


def _summary_line(rec: dict) -> str:
    src = rec.get("source", {}).get("file", "?")
    if rec.get("status"):
        return f"{rec['metric']}: {rec['status']} ({src})"
    m = rec["metric"]
    if m == "latency":
        vals = ", ".join(f"{v:.2f} s" for v in rec["values_s"])
        warm = (f"; warm {rec['warm_latency_s']:.2f} s = {rec['hz_warm']:.2f} Hz"
                if rec.get("warm_latency_s") else "")
        return (f"{rec['quantisation']}: per-inference {vals}{warm}; VRAM {rec['vram_gb']} GB "
                f"[{src}, {rec['source']['file_date']}]")
    if m == "inference_hz_in_flight":
        return (f"in flight: {rec['hz_min']:.2f}-{rec['hz_max']:.2f} Hz over {rec['flights']} "
                f"flights ({rec['inferences']} inferences) [{src}]")
    if m == "inference_hz":
        return f"stated {rec['value_hz']} Hz, no timing file [{src}]"
    if m == "seconds_per_action":
        return f"stated: {rec['value']} [{src}]"
    if m == "closed_loop":
        if rec.get("protocol") == "stated only":
            return f"stated: {rec['value']} [{src}]"
        if rec["backend"] == "bc_v3":
            return (f"offline: reach {rec['reach_rate_unshielded']} unshielded, "
                    f"{rec['reach_rate_shielded']} shielded over {rec['episodes']} episodes "
                    f"(kinematic rollouts, not a simulator) [{src}]")
        if rec["backend"] == "stub":
            return (f"{rec['run']}: reached={rec['reached']}, {rec['interventions']} "
                    f"interventions in {rec['ticks']} ticks, P0 escape rate "
                    f"{rec['p0_violation_escape_rate']} [{src}]")
        mode = (f"ASSISTED goal_blend {rec['goal_blend']}" if rec["assisted"]
                else f"pure (goal_blend 0), yaw gain {rec['yaw_gain']}")
        return (f"{rec['reached']}/{rec['episodes']} reached, {mode}, mean interventions "
                f"{rec['mean_interventions']} [{src}]")
    return json.dumps(rec)


def render_markdown(t: dict) -> str:
    out = ["# VLA backend comparison - measurement table",
           "",
           f"Generated by `python tools/vla_backend_table.py table` on {t['created_utc']}. "
           f"Protocol `{t['protocol']['id']}` (the protocol itself: "
           f"`python tools/vla_backend_table.py protocol`). Do not edit by hand; "
           f"re-run the tool.",
           "",
           f"**Default backend:** none. {t['decision']}",
           "",
           "| Backend | Weights on this host | Rate, desktop | Rate, Jetson Orin | "
           "Closed loop, desktop | Closed loop, hil |",
           "|---|---|---|---|---|---|"]
    for r in t["rows"]:
        ws = r["weights_status"]
        if not ws:
            w = "none needed" if not r.get("unavailable") else "not obtained"
        else:
            w = "; ".join(f"{x['path']}: {'yes' if x['present'] else 'MISSING'}" for x in ws)
        c = r["cells"]
        out.append(f"| {r['label']} | {w} | {c['rate_desktop']} | {c['rate_orin']} | "
                   f"{c['closed_loop_desktop']} | {c['closed_loop_hil']} |")
    out += ["", "## Records already on disk (none of them is R1 or C1)", ""]
    for r in t["rows"]:
        if not r["records"] and not r.get("unavailable"):
            continue
        out.append(f"**{r['label']}**")
        out.append("")
        if r.get("unavailable"):
            out.append(f"- {r['unavailable']}")
        for rec in r["records"]:
            out.append(f"- {_summary_line(rec)}")
            if rec.get("conditions"):
                out.append(f"  - conditions: {rec['conditions']}")
        out.append("")
    if t["contradictions"]:
        out += ["## Statements a file on disk contradicts", ""]
        whys: list[str] = []
        for c in t["contradictions"]:
            note = f" ({c['note']})" if c.get("note") else ""
            out.append(f"- `{c['doc']}`{note}: \"{c['claim'][:160]}\" - contradicted by "
                       f"`{c['contradicted_by']}` ({c['field']} = {c['value']})")
            if c["why"] not in whys:
                whys.append(c["why"])
        for w in whys:
            out += ["", f"Why, and the correct statement: {w}."]
        ev = t["contradictions"][0].get("evidence") or []
        if ev:
            out += ["", "The evaluations behind the correction, read from disk:", "",
                    "| File | Adapter | Reached | goal_blend (assist) | yaw gain | "
                    "Mean efficiency |", "|---|---|---|---|---|---|"]
            for e in ev:
                out.append(f"| `{e['file']}` | `{e['adapter']}` | {e['reached']}/{e['episodes']} "
                           f"| {e['goal_blend']} ({'on' if e['assisted'] else 'off'}) "
                           f"| {e['yaw_gain']} | {e['mean_efficiency']} |")
        out.append("")
    out += ["## What R1 and C1 are", "",
            f"- **R1 (rate, open loop):** {t['protocol']['R1_rate']['inputs']['camera_vla']}; "
            f"{t['protocol']['R1_rate']['warmup']} warm-up, {t['protocol']['R1_rate']['timed']} "
            f"timed inferences, {t['protocol']['R1_rate']['isolation']}. Reference: "
            f"{t['protocol']['R1_rate']['reference']}. Floor: {t['protocol']['R1_rate']['floor']}.",
            f"- **C1 (closed loop):** {t['protocol']['C1_closed_loop']['scenario']}; "
            f"{t['protocol']['C1_closed_loop']['seeds']} seeds, "
            f"{t['protocol']['C1_closed_loop']['time_limit_s']} s limit, reached within "
            f"{t['protocol']['C1_closed_loop']['reached_radius_m']} m. "
            f"{t['protocol']['C1_closed_loop']['assist']} Nulls: "
            + "; ".join(t["protocol"]["C1_closed_loop"]["nulls"]) + ".",
            ""]
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("protocol", help="print the protocol as JSON")
    p_t = sub.add_parser("table", help="assemble the table from files on disk")
    p_t.add_argument("--out-json", type=Path, default=DEFAULT_TABLE_JSON)
    p_t.add_argument("--out-md", type=Path, default=DEFAULT_TABLE_MD)
    p_t.add_argument("--model-root", type=Path,
                     default=Path(os.environ.get("VLAGUARD_MODEL_ROOT", "D:/models")))
    p_m = sub.add_parser("measure", help="run R1 for one backend on this host")
    p_m.add_argument("--backend", required=True, choices=list(BACKENDS))
    p_m.add_argument("--topology", default=os.environ.get("VLAGUARD_TOPOLOGY"))
    p_m.add_argument("--allow-gpu", action="store_true")
    p_m.add_argument("--model-root", type=Path, default=None)
    p_m.add_argument("--warmup", type=int, default=PROTOCOL["R1_rate"]["warmup"])
    p_m.add_argument("--timed", type=int, default=PROTOCOL["R1_rate"]["timed"])
    p_m.add_argument("--csp", choices=("off", "on"), default="off",
                     help="openvla_7b_4bit only: the prompt arm, as real_vla_demo.py --csp "
                          "(default off, that script's default)")
    p_m.add_argument("--out", type=Path, default=None,
                     help="default: <evidence>/<topology>/vla_rate_<backend>.json, "
                          "<evidence> = $VLAGUARD_EVIDENCE_OUT or deploy/evidence")
    p_m.add_argument("--cpus", default=None,
                     help="pin to these logical CPUs, e.g. 0-15 (recorded in the result)")
    p_m.add_argument("--note", default=None,
                     help="free text stored with the result, e.g. the Orin's power mode "
                          "(nvpmodel is not visible inside a container)")
    args = ap.parse_args(argv)

    if args.cmd == "protocol":
        print(json.dumps(PROTOCOL, indent=2))
        return 0
    if args.cmd == "table":
        t = build_table(model_root=args.model_root)
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        write_lf(args.out_json, json.dumps(t, indent=1) + "\n")
        write_lf(args.out_md, render_markdown(t) + "\n")
        for r in t["rows"]:
            print(f"{r['backend']:<20} rate desktop: {r['cells']['rate_desktop']}")
            print(f"{'':<20} rate Orin   : {r['cells']['rate_orin']}")
        for c in t["contradictions"]:
            print(f"CONTRADICTION  {c['doc']}: {c.get('note') or ''}")
        if t["contradictions"]:
            print(f"               why: {t['contradictions'][0]['why']}")
        print(f"wrote {_rel(args.out_json)} and {_rel(args.out_md)}")
        return 0
    pinned = None
    if args.cpus:
        from profile_shield_tick import parse_cpus, pin_cpus
        try:
            pinned = pin_cpus(parse_cpus(args.cpus))
        except (RuntimeError, ValueError, OSError) as exc:
            print(f"REFUSED  --cpus {args.cpus}: {exc}")
            return 2
    try:
        res = measure(args.backend, args.topology, model_root=args.model_root,
                      allow_gpu=args.allow_gpu, warmup=args.warmup, timed=args.timed,
                      csp=args.csp)
        res["cpu_affinity"] = pinned
        res["note"] = args.note
    except PermissionError as exc:
        print(f"REFUSED  {exc}")
        return 2
    except (ValueError, RuntimeError) as exc:
        print(f"FAILED   {exc}")
        return 1
    res["command"] = "python tools/vla_backend_table.py " + shlex.join(
        argv if argv is not None else sys.argv[1:])
    from profile_shield_tick import evidence_root
    suffix = "-csp" if args.csp == "on" else ""
    out = args.out or (evidence_root() / args.topology / f"vla_rate_{args.backend}{suffix}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    write_lf(out, json.dumps(res, indent=1) + "\n")
    s = res["stats"]
    print(f"{args.backend}: {_fmt_ms(s)}; meets {CONTRACT_HZ:g} Hz at p95: "
          f"{s.get('meets_contract_p95')}; undecodable {res.get('undecodable_outputs')}")
    print(f"wrote {_rel(out)}")
    return 0 if s.get("n") else 1


if __name__ == "__main__":
    sys.exit(main())
