"""Gate G3 (Orin VLA inference latency), judged from the stored hil flights.

    python tools/g3_gate.py --runs F:/vla_drone_offload/jetson_runs \
        --write docs/data/g3_gate_20261117      # read the run folders, write the table
    python tools/g3_gate.py --check docs/data/g3_gate_20261117/g3_gate.json
                                                # re-judge the committed table, no run folders

Exit 0 when at least one learned backend passes the gate on the AGX Orin in
flight, 1 when none does, 2 when the input is missing.

THE GATE (reference design, docs/04-risks-and-fallbacks/fallback-gates.md)

  "Gate G3 candidate - Orin VLA inference latency: If the chosen 7B-class VLA
  backend cannot produce 4-D actions at 10 Hz on the Orin in int8
  quantization, a gate could be added: 'if Orin inference latency > 100 ms p95
  by 2026-08-01, swap to a smaller / quantized backend (e.g. BitVLA, TinyVLA).'"

WHAT IS JUDGED, AND ON WHICH HOST

A backend passes when every one of its C2 flights (the hil follow, the set
tools/vla_backend_table.py selects; at least C2_FLIGHTS of them) meets both:

  * decision latency p95 <= 100 ms, where the latency is the round trip of one
    request in flight: the perception node sends the frame, the VLA server
    answers (vla_steps.jsonl `rtt_s`; both processes on the AGX Orin). The
    server's own model time (`latency_server_s`) and the time from camera
    capture to the parsed answer (`latency_client_s`) are shown beside it;
  * >= 10 answers parsed per second while the model was being asked
    (decision_hz_asked of the backend table: not held by the rail, not
    withheld because the detector had no box). The rate over the whole
    mission is shown beside it.

and its vla_steps.jsonl was written in a run whose metrics.json hil_evidence
names an AGX Orin as the VLA's host (`vla_host.device_model`; the top-level
`device_model` is the Shield's host and is never used for this), with no
failed request (ok false or no rtt_s). A flight whose folder is not found,
whose VLA host is null/absent ("not shown") or not an AGX Orin (an Orin Nano
does not count), is listed and does not count. A backend with no flight read
and no timed Orin bench is "not measured", not "fails".

The open-loop benches the backend table holds for the Orin (its `g3`
section) are carried into the table as the second column set: they are not
in flight, and not the gate's measure here.

Nothing here loads a model or touches the rig; it reads files.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TABLE = ROOT / "deploy" / "evidence" / "vla_backend_table.json"

SCHEMA = "g3_gate/1"
THRESHOLD_P95_MS = 100.0
MIN_HZ = 10.0
C2_FLIGHTS = 3
GATE_TEXT = (
    "If the chosen 7B-class VLA backend cannot produce 4-D actions at 10 Hz on the "
    "Orin in `int8` quantization, a gate could be added: \"if Orin inference latency "
    "> 100 ms p95 by 2026-08-01, swap to a smaller / quantized backend (e.g. BitVLA, "
    "TinyVLA).\" Whether to add this gate depends on Q2 measurement.")
GATE_SOURCE = "kuanting-vla-uav-guardrail docs/04-risks-and-fallbacks/fallback-gates.md, Gate G3 candidate"

# 7B-class: OpenVLA-7B and its fine-tunes. "smaller": what the gate swaps to.
SEVEN_B = {"openvla_7b_4bit", "openvla_7b_bf16", "openvla_7b_int8wo", "aerialvla_lora",
           "aerialvla_ft_run2", "aerialvla_bf16", "racevla_7b"}
NOT_LEARNED_VLA = {"stub", "bc_v3"}   # no model / no camera: not a VLA backend


def pct(xs: list[float], q: float) -> float | None:
    """Linear-interpolated percentile (numpy's default), q in [0, 1]."""
    if not xs:
        return None
    s = sorted(xs)
    k = (len(s) - 1) * q
    f = int(k)
    c = min(f + 1, len(s) - 1)
    return s[f] + (s[c] - s[f]) * (k - f)


def spread(xs: list[float]) -> dict:
    return {"n": len(xs),
            "p50": _r(pct(xs, 0.50)), "p95": _r(pct(xs, 0.95)), "p99": _r(pct(xs, 0.99)),
            "max": _r(max(xs) if xs else None)}


def _r(x, nd=1):
    return None if x is None else round(float(x), nd)


def c2_flights(table: dict) -> dict[str, list[dict]]:
    """backend -> its C2 flight records, as the backend table scored them."""
    out: dict[str, list[dict]] = {}
    for row in table.get("rows") or []:
        for f in row.get("records") or []:
            if f.get("protocol") == "C2" and not f.get("not_c2") and not f.get("c2_censored"):
                out.setdefault(row["backend"], []).append(f)
    return out


def find_run(tag: str, roots: list[Path]) -> Path | None:
    for r in roots:
        p = Path(r) / tag
        if (p / "vla_steps.jsonl").is_file():
            return p
    return None


def flight_latency(run: Path) -> dict:
    """Per-decision latencies of one flight, from its vla_steps.jsonl.

    A request that failed (ok false, or no rtt_s: timed out, errored) has no
    latency to pool; it is counted in `failed_requests` and fails the flight
    (judge_flight), so a timeout can never lower the p95."""
    rtt, srv, cap, wait = [], [], [], []
    rows = parsed = failed = 0
    with open(run / "vla_steps.jsonl", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            s = json.loads(line)
            rows += 1
            if not s.get("ok") or s.get("rtt_s") is None:
                failed += 1
                continue
            if s.get("parse_ok"):
                parsed += 1
            rtt.append(1000.0 * float(s["rtt_s"]))
            if s.get("latency_server_s") is not None:
                srv.append(1000.0 * float(s["latency_server_s"]))
            if s.get("latency_client_s") is not None:
                cap.append(1000.0 * float(s["latency_client_s"]))
            if s.get("t_sent") is not None and s.get("t_capture") is not None:
                wait.append(1000.0 * (float(s["t_sent"]) - float(s["t_capture"])))
    metrics = json.loads((run / "metrics.json").read_text(encoding="utf-8")) \
        if (run / "metrics.json").is_file() else {}
    ev = metrics.get("hil_evidence") or {}
    # Only the VLA server's own report counts. hil_evidence.device_model is the
    # Shield/perception host; a run with vla_host null or absent cannot be shown
    # to have run the VLA on the Orin (guardrail/manifest._vla_host_missing).
    vla_host = (ev.get("vla_host") or {}).get("device_model")
    return {"answers": rows, "parsed": parsed, "failed_requests": failed,
            "round_trip_ms": spread(rtt), "server_ms": spread(srv),
            "capture_to_answer_ms": spread(cap), "capture_to_send_ms": spread(wait),
            "vla_host": vla_host,
            "_rtt": rtt, "_srv": srv, "_cap": cap}


def is_orin(model: str | None) -> bool:
    """The grant's hil topology names the Jetson AGX Orin; an Orin Nano does not count."""
    return bool(model) and "agx orin" in model.lower()


def judge_flight(f: dict) -> list[str]:
    """Why one flight misses the gate (empty: it meets it)."""
    why = []
    if f.get("missing"):
        return [f["missing"]]
    if not f.get("vla_host"):
        why.append("VLA host not shown (metrics.json hil_evidence.vla_host missing or null)")
    elif not is_orin(f.get("vla_host")):
        why.append(f"VLA host {f.get('vla_host')!r} is not an AGX Orin")
    if f.get("failed_requests"):
        why.append(f"{f['failed_requests']} requests failed (no answer, no round trip)")
    p95 = (f.get("round_trip_ms") or {}).get("p95")
    if p95 is None or p95 > THRESHOLD_P95_MS:
        why.append(f"round trip p95 {p95} ms > {THRESHOLD_P95_MS:g} ms")
    hz = f.get("decision_hz_asked")
    if hz is None or hz < MIN_HZ:
        why.append(f"{_r(hz, 2)} decisions/s while asked < {MIN_HZ:g}")
    return why


def judge_backend(b: dict) -> dict:
    flights = b["in_flight"]["flights"]
    for f in flights:
        f["misses"] = judge_flight(f)
    counted = [f for f in flights if not f.get("missing") and is_orin(f.get("vla_host"))]
    why = []
    if len(counted) < C2_FLIGHTS:
        why.append(f"{len(counted)} C2 flights on the Orin read, the gate asks for {C2_FLIGHTS}")
    for f in flights:
        if f["misses"]:
            why.append(f"{f['tag']}: " + "; ".join(f["misses"]))
    b["passes"] = not why
    b["why_not"] = why
    # "fails" always means measured and failed: a backend with no flight read on
    # the Orin and no timed bench there is "not measured" instead.
    timed = any(o.get("p95_ms") is not None or o.get("slowest_ms") is not None
                or o.get("in_flight_p50_ms") for o in b.get("orin_bench_open_loop") or [])
    read = any(not f.get("missing") for f in flights)
    b["status"] = "passes" if b["passes"] else ("fails" if (read or timed) else "not measured")
    return b


def power_mode(file: str | None) -> str:
    """The AGX Orin's nvpmodel mode a bench file recorded, or 'not recorded'."""
    p = ROOT / file if file else None
    if p is None or p.suffix != ".json" or not p.is_file():
        return "not recorded"
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return "not recorded"
    for h in (d.get("host") or {}, d):           # R1 benches nest it under "host"
        name = ((h.get("power") or {}).get("nvpmodel") or {}).get("mode_name") \
            or (h.get("jetson") or {}).get("nvpmodel")
        if name:
            return name
    return "not recorded"


def build(table: dict, roots: list[Path]) -> dict:
    flights_by = c2_flights(table)
    labels = {r["backend"]: r.get("label") for r in table.get("rows") or []}
    benches = {}
    for grp in ("seven_b", "smaller"):
        for b in (table.get("g3") or {}).get(grp) or []:
            benches[b["backend"]] = [dict(o, power_mode=power_mode(o.get("file")))
                                     for o in b.get("orin") or []]
    order = [r["backend"] for r in table.get("rows") or []
             if r["backend"] not in NOT_LEARNED_VLA]
    backends = []
    for name in order:
        flights, pooled = [], {"_rtt": [], "_srv": [], "_cap": []}
        for rec in flights_by.get(name, []):
            run = find_run(rec["tag"], roots)
            f = {"tag": rec["tag"], "commit": (rec.get("code_revision") or "")[:7],
                 "end_reason": rec.get("end_reason"),
                 "decision_hz_asked": _r(rec.get("decision_hz_asked"), 2),
                 "decision_hz_mission": _r(rec.get("decision_hz_mission"), 2)}
            if run is None:
                f["missing"] = "run folder (vla_steps.jsonl) not found under --runs"
            else:
                lat = flight_latency(run)
                for k in pooled:
                    pooled[k] += lat.pop(k)
                f.update(lat)
            flights.append(f)
        if not flights and name not in benches:
            continue
        b = {"backend": name, "label": labels.get(name),
             "class": "7B" if name in SEVEN_B else "smaller",
             "in_flight": {"flights": flights,
                           "pooled": {"round_trip_ms": spread(pooled["_rtt"]),
                                      "server_ms": spread(pooled["_srv"]),
                                      "capture_to_answer_ms": spread(pooled["_cap"])}},
             "orin_bench_open_loop": benches.get(name, [])}
        backends.append(judge_backend(b))
    return finish({"schema": SCHEMA, "backends": backends,
                   "int8_on_orin": (table.get("g3") or {}).get("int8_on_orin"),
                   "int8_note": (table.get("g3") or {}).get("int8_note"),
                   "table_source": {"file": "deploy/evidence/vla_backend_table.json",
                                    "created_utc": table.get("created_utc")}})


def finish(out: dict) -> dict:
    out["gate"] = {"text": GATE_TEXT, "source": GATE_SOURCE,
                   "threshold_p95_ms": THRESHOLD_P95_MS, "min_decisions_hz": MIN_HZ,
                   "c2_flights": C2_FLIGHTS,
                   "latency_measure": "round trip of one request in flight (vla_steps.jsonl "
                                      "rtt_s: perception node to VLA server and back, both on "
                                      "the AGX Orin)",
                   "rate_measure": "answers parsed per second while the model was asked "
                                   "(the backend table's decision_hz_asked)"}
    for b in out["backends"]:
        judge_backend(b)
    passing = [b["backend"] for b in out["backends"] if b["passes"]]
    seven = [b for b in out["backends"] if b["class"] == "7B"]
    out["verdict"] = {
        "passing": passing,
        "seven_b_passing": [b["backend"] for b in seven if b["passes"]],
        "seven_b_failing": [b["backend"] for b in seven if b["status"] == "fails"],
        "seven_b_not_measured": [b["backend"] for b in seven if b["status"] == "not measured"],
        "gate_met_by_a_learned_backend": bool(passing),
    }
    return out


def _cell(s: dict | None) -> str:
    if not s or not s.get("n"):
        return "-"
    return f"{s['p50']:g} / {s['p95']:g} / {s['p99']:g}"


def _bench(rows: list[dict]) -> str:
    parts = []
    for o in rows:
        pm = f", power mode {o.get('power_mode') or 'not recorded'}"
        if o.get("p95_ms") is not None:
            parts.append(f"p95 {o['p95_ms']:.1f} ms, n={o.get('n')} ({o['what']}{pm})")
        elif o.get("slowest_ms") is not None:
            parts.append(f"slowest {o['slowest_ms']:g} ms, n={o.get('n')} ({o['what']}{pm})")
        elif o.get("in_flight_p50_ms"):
            parts.append("in flight 3-4 Nov, server p50 "
                         + "-".join(f"{x:.0f}" for x in o["in_flight_p50_ms"]) + " ms")
        else:
            parts.append(o.get("what") + (f": {o['cause']}" if o.get("cause") else ""))
    return "; ".join(parts) or "none on the Orin"


def render(out: dict) -> str:
    g = out["gate"]
    lines = [
        "# Gate G3: VLA decision latency and rate on the AGX Orin",
        "",
        f"Generated by `python tools/g3_gate.py`; schema `{out['schema']}`. Do not edit by hand.",
        "",
        f"Gate: p95 <= {g['threshold_p95_ms']:g} ms and >= {g['min_decisions_hz']:g} decisions/s, "
        f"on every one of >= {g['c2_flights']} C2 hil flights whose metrics.json names an AGX Orin "
        f"as the VLA server's host (hil_evidence.vla_host), with no failed request. "
        f"Latency: {g['latency_measure']}. Rate: {g['rate_measure']}.",
        "",
        "## Per flight (in flight, hil; VLA host as each run recorded it)",
        "",
        "| Backend | Class | Flight | VLA host | Answers | Failed requests "
        "| Round trip p50 / p95 / p99 ms "
        "| Server (model) p50 / p95 / p99 ms | Capture to answer p50 / p95 / p99 ms "
        "| Decisions/s asked | Decisions/s mission | Meets gate |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for b in out["backends"]:
        for f in b["in_flight"]["flights"]:
            if f.get("missing"):
                lines.append(f"| {b['label']} | {b['class']} | {f['tag']} | - | - | - | - | - | - "
                             f"| {f['decision_hz_asked']} | {f['decision_hz_mission']} "
                             f"| no: {f['missing']} |")
                continue
            ok = "yes" if not f["misses"] else "no: " + "; ".join(f["misses"])
            lines.append(
                f"| {b['label']} | {b['class']} | {f['tag']} | {f.get('vla_host') or 'not shown'} "
                f"| {f['answers']} | {f.get('failed_requests', '-')} "
                f"| {_cell(f['round_trip_ms'])} | {_cell(f['server_ms'])} "
                f"| {_cell(f['capture_to_answer_ms'])} | {f['decision_hz_asked']} "
                f"| {f['decision_hz_mission']} | {ok} |")
    lines += ["", "## Per backend", "",
              "| Backend | Class | C2 flights | Pooled round trip p50 / p95 / p99 ms "
              "| Pooled server p50 / p95 / p99 ms | Decisions/s asked (range) "
              "| Open-loop bench on the AGX Orin (not in flight) | G3 |",
              "|---|---|---|---|---|---|---|---|"]
    for b in out["backends"]:
        fl = b["in_flight"]["flights"]
        hz = [f["decision_hz_asked"] for f in fl if f.get("decision_hz_asked") is not None]
        rng = f"{min(hz):g}-{max(hz):g}" if hz else "-"
        p = b["in_flight"]["pooled"]
        lines.append(f"| {b['label']} | {b['class']} | {len(fl)} | {_cell(p['round_trip_ms'])} "
                     f"| {_cell(p['server_ms'])} | {rng} | {_bench(b['orin_bench_open_loop'])} "
                     f"| {b['status']} |")
    v = out["verdict"]
    lines += ["", "## Verdict", "",
              f"- Learned backends that pass G3 on the AGX Orin in flight: "
              f"{', '.join(v['passing']) or 'none'}.",
              f"- 7B-class backends that pass: {', '.join(v['seven_b_passing']) or 'none'}.",
              f"- 7B-class backends measured on the AGX Orin that fail: "
              f"{', '.join(v['seven_b_failing']) or 'none'}.",
              f"- 7B-class backends not measured on the AGX Orin (no flight, no timed bench): "
              f"{', '.join(v['seven_b_not_measured']) or 'none'}.",
              f"- int8 on the Orin: {out.get('int8_on_orin')} ({out.get('int8_note') or 'no note'}).",
              ""]
    return "\n".join(lines)


def strip_private(out: dict) -> dict:
    for b in out["backends"]:
        for f in b["in_flight"]["flights"]:
            for k in [k for k in f if k.startswith("_")]:
                f.pop(k)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--table", type=Path, default=TABLE,
                    help="the backend table JSON (its C2 flights and Orin benches)")
    ap.add_argument("--runs", type=Path, nargs="*", default=[],
                    help="folders holding the hil run folders (default: this checkout's demo/out)")
    ap.add_argument("--write", type=Path, default=None,
                    help="folder to write g3_gate.json and g3_gate.md into")
    ap.add_argument("--check", type=Path, default=None,
                    help="re-judge a written g3_gate.json instead of reading run folders")
    a = ap.parse_args(argv)
    if a.check is not None:
        if not a.check.is_file():
            print(f"no such file: {a.check}", file=sys.stderr)
            return 2
        out = finish(json.loads(a.check.read_text(encoding="utf-8")))
    else:
        if not a.table.is_file():
            print(f"no backend table: {a.table}", file=sys.stderr)
            return 2
        roots = list(a.runs) or [ROOT / "demo" / "out"]
        out = strip_private(build(json.loads(a.table.read_text(encoding="utf-8")), roots))
        if not any(f for b in out["backends"] for f in b["in_flight"]["flights"]
                   if not f.get("missing")):
            print("no C2 run folder found under " + ", ".join(map(str, roots)), file=sys.stderr)
            return 2
    md = render(out)
    print(md)
    if a.write is not None:
        a.write.mkdir(parents=True, exist_ok=True)
        with open(a.write / "g3_gate.json", "w", encoding="utf-8", newline="\n") as fh:
            json.dump(out, fh, indent=1)
            fh.write("\n")
        with open(a.write / "g3_gate.md", "w", encoding="utf-8", newline="\n") as fh:
            fh.write(md)
    return 0 if out["verdict"]["gate_met_by_a_learned_backend"] else 1


if __name__ == "__main__":
    sys.exit(main())
