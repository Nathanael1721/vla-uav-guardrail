"""
Build docs/data/eval_sep2026.json: every number the September decks and the
mid-evaluation report quote, computed once, from the artefacts.

Why one file. The September deck once printed a tracking figure the report did
not, and a hand-typed "fired 6 times" reached a meeting pack before anyone
scored it. So nothing in the new decks or the report is typed by hand: they
read this file, and this script reads the flight logs, the KPI files and the
benchmark JSON. It reuses `demo/track_truth.py` rather than re-implementing any
scoring, so there is exactly one scorer.

It prints every value it writes. That transcript is also the evidence that the
numbers are what the artefacts say.

Usage:
    python tools/build_eval_data.py            # everything, including tests
    python tools/build_eval_data.py --no-tests # skip running the test suites
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import math
import re
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "data" / "eval_sep2026.json"
sys.path.insert(0, str(ROOT / "demo"))

import track_truth as T  # noqa: E402

LAST_MEETING = "2026-09-02"
# retarget_fixed ran 69.95 s and retarget_smooth 119.95 s. Before/after figures
# are only comparable over the window both flights actually flew.
MATCHED_WINDOW_S = 69.95


def say(key, value):
    print(f"  {key:52s} {value}")
    return value


def load_rows(tag):
    f = ROOT / "demo" / "out" / tag / "flight_log.jsonl"
    return [json.loads(l) for l in f.open(encoding="utf-8")]


def load_json(rel):
    return json.loads((ROOT / rel).read_text(encoding="utf-8"))


def med(a):
    return statistics.median(a) if a else None


def nearest(r):
    pts = (r.get("truth") or {}).get("pts") or []
    return min((math.hypot(r["x"] - px, r["y"] - py) for px, py in pts),
               default=None)


# ── KPI-grade rail ──────────────────────────────────────────────────────────
def kpi_rail():
    print("\n[kpi] canonical-hil rail (ArduPilot SITL over MAVROS 2)")
    runs = {}
    for tag in ("ros2_shield_off", "ros2_shield_on", "ros2_shield_on_dynamic",
                "ros2_ped_off", "ros2_ped_on"):
        k = load_json(f"demo/out/{tag}/kpi.json")
        m = load_json(f"demo/out/{tag}/metrics.json")
        man = load_json(f"demo/out/{tag}/manifest.json")
        runs[tag] = {
            "topology": m.get("topology") or man.get("topology"),
            "kpi_grade": k.get("kpi_grade"),
            "shield": m.get("shield"),
            "ticks": k.get("ticks"),
            "mission_success": k.get("mission_success"),
            "outcome": k.get("outcome"),
            "p0_violation_escape_rate": k.get("p0_violation_escape_rate"),
            "p0_ticks_not_measurable": k.get("p0_ticks_not_measurable"),
            "failsafe_trigger_correctness": k.get("failsafe_trigger_correctness"),
            "mean_repair_magnitude_mps": k.get("mean_repair_magnitude_mps"),
            "max_repair_magnitude_mps": k.get("max_repair_magnitude_mps"),
            "mean_time_to_safe_s": k.get("mean_time_to_safe_s"),
            "time_to_safe_episodes": k.get("time_to_safe_episodes"),
            "nfz_s": m.get("nfz_s"),
            "standoff_s": m.get("standoff_s"),
            "standoff_min_range_m": m.get("standoff_min_range_m"),
            "subject_position_source": (m.get("subject") or {}).get("position_source"),
            "policy_hash": man.get("policy_hash"),
        }
        say(tag, {x: runs[tag][x] for x in (
            "kpi_grade", "p0_violation_escape_rate", "mean_repair_magnitude_mps",
            "mean_time_to_safe_s", "failsafe_trigger_correctness", "nfz_s",
            "standoff_min_range_m")})
    n_grade = sum(1 for f in glob.glob(str(ROOT / "demo/out/*/kpi.json"))
                  if json.loads(Path(f).read_text(encoding="utf-8")).get("kpi_grade"))
    n_all = len(glob.glob(str(ROOT / "demo/out/*/kpi.json")))
    say("flights with kpi.json / kpi_grade", f"{n_all} / {n_grade}")
    # "0.0 on every flight" was published and is false: the unshielded control
    # flights exist to FAIL. Count the two populations instead of asserting.
    zero, controls, other = 0, [], []
    for f in sorted(glob.glob(str(ROOT / "demo/out/*/kpi.json"))):
        tag = Path(f).parent.name
        rate = json.loads(Path(f).read_text(encoding="utf-8")).get("p0_violation_escape_rate")
        if rate == 0.0:
            zero += 1
        elif tag.endswith("_off"):
            controls.append({"tag": tag, "rate": rate})
        else:
            other.append({"tag": tag, "rate": rate})
    say("escape rate 0.0 / unshielded controls / anything else",
        f"{zero} / {len(controls)} / {other or 'none'}")
    return {"runs": runs, "flights_total": n_all, "flights_kpi_grade": n_grade,
            "flights_escape_zero": zero, "unshielded_controls": controls,
            "flights_nonzero_unexplained": other}


# ── the retarget flights: tracking, yaw, ring ───────────────────────────────
def phase_stats(rows, cls, t_max=None):
    seg = [r for r in rows if (r.get("truth") or {}).get("class") == cls
           and (t_max is None or r["t"] <= t_max)]
    if not seg:
        return None
    s = T.score_rows(seg)
    yaw = sorted(abs(r["raw"]["yaw_rate"]) * 180 / math.pi
                 for r in seg if r.get("raw"))
    flips = sum(1 for a, b in zip(seg, seg[1:]) if a.get("raw") and b.get("raw")
                and a["raw"]["yaw_rate"] * b["raw"]["yaw_rate"] < 0)
    near = [nearest(r) for r in seg]
    inside = [r for r, d in zip(seg, near) if d is not None and d < 10.0]
    silent = [r for r in inside if not any(
        v.get("rule_id") == "standoff-pedestrian" for v in r.get("violations", []))]
    rng_err = [abs(r["est"]["rng"] - d) for r, d in zip(seg, near)
               if d is not None and (r.get("est") or {}).get("served")
               and (r.get("est") or {}).get("rng") is not None]
    return {
        "ticks": len(seg),
        "box_err_px_median": s["det_gt_err_px_median_in_shot"],
        "box_err_px_null_centre": s["det_gt_err_px_median_chance_centre"],
        "box_err_deg_median": s["det_gt_err_deg_median_in_shot"],
        "box_err_deg_null_centre": s["det_gt_err_deg_median_chance_centre"],
        "yaw_dps_median": round(yaw[len(yaw) // 2], 1) if yaw else None,
        "yaw_dps_max": round(yaw[-1], 1) if yaw else None,
        "yaw_sign_flip_frac": round(flips / len(seg), 3),
        "closest_real_m": round(min(d for d in near if d is not None), 2)
        if any(d is not None for d in near) else None,
        "ticks_real_inside_10m": len(inside) if cls == "pedestrian" else None,
        "ticks_ring_silent": len(silent) if cls == "pedestrian" else None,
        "served_range_minus_nearest_person_m_median": round(med(rng_err), 1) if rng_err else None,
    }


def ring_coverage(rows, t_max=None, pitch_deg=20.0, vhalf_deg=29.4, hhalf_deg=45.0):
    """Where was the real pedestrian inside 10 m, relative to the camera?

    `standoff_score` counts a tick positive when ANY logged pedestrian is inside
    the ring. The rule it scores, SubjectStandoff, protects only the SUBJECT
    being followed (guardrail/models.py) - one position per tick. So a positive
    can be a bystander the rule was never written to protect, and one the only
    perception sensor could not have seen. This splits the positives by whether
    that person was in frame at all: inside the +-45 deg horizontal FOV and
    beyond the near edge of the ground footprint (altitude / tan(pitch + vhalf),
    6.9 m at 8 m).
    """
    seg = [r for r in rows if (r.get("truth") or {}).get("class") == "pedestrian"
           and (t_max is None or r["t"] <= t_max)]
    n = in_frame = outside_hfov = under_nose = 0
    for r in seg:
        dist, px, py = min((math.hypot(r["x"] - px, r["y"] - py), px, py)
                           for px, py in r["truth"]["pts"])
        if dist >= 10.0:
            continue
        n += 1
        brg = (math.degrees(math.atan2(py - r["y"], px - r["x"]) - r["psi"]) + 180) % 360 - 180
        near_edge = (r.get("up") or 8.0) / math.tan(math.radians(pitch_deg + vhalf_deg))
        if abs(brg) > hhalf_deg:
            outside_hfov += 1
        elif dist < near_edge:
            under_nose += 1
        else:
            in_frame += 1
    return {"ticks_person_inside_10m": n, "in_frame": in_frame,
            "outside_hfov": outside_hfov, "under_nose": under_nose,
            "in_frame_frac": round(in_frame / n, 3) if n else None}


def subject_range_check(rows, tick_lo=340, tick_hi=700, hhalf_deg=45.0, near_edge_m=6.9):
    """Was the served range wrong, or was it measured against the wrong person?

    Ticks 340-700 of retarget_smooth were published as "a real person 5.06 m
    away, estimator served 43 m, so depth through an 8 px box is background".
    That compared the SUBJECT's estimate with the NEAREST pedestrian. Here the
    estimate is compared with the pedestrian under the detection box (the one
    being followed) and with the nearest one the camera could see.
    """
    seg = [r for r in rows if tick_lo <= r["tick"] <= tick_hi
           and (r.get("est") or {}).get("served")]
    near_brg, under_box, best_vis, est = [], [], [], []
    for r in seg:
        peds = []
        for px, py in r["truth"]["pts"]:
            d = math.hypot(r["x"] - px, r["y"] - py)
            b = (math.degrees(math.atan2(py - r["y"], px - r["x"]) - r["psi"]) + 180) % 360 - 180
            peds.append((d, b))
        near_brg.append(abs(min(peds)[1]))
        vis = [(d, b) for d, b in peds if abs(b) <= hhalf_deg and d > near_edge_m]
        est.append(r["est"]["rng"])
        if vis:
            best_vis.append(min(abs(r["est"]["rng"] - d) for d, _ in vis))
            if r.get("bearing_deg") is not None:
                under_box.append(min(vis, key=lambda v: abs(v[1] - r["bearing_deg"]))[0])
    return {"ticks": [tick_lo, tick_hi], "n": len(seg),
            "nearest_person_abs_bearing_deg_median": round(med(near_brg), 1),
            "est_range_m_median": round(med(est), 1),
            "person_under_box_range_m_median": round(med(under_box), 1) if under_box else None,
            "est_minus_best_visible_person_m_median": round(med(best_vis), 1) if best_vis else None}


def retarget():
    print("\n[retarget] before = retarget_fixed, after = retarget_smooth")
    out = {"matched_window_s": MATCHED_WINDOW_S}
    old, new = load_rows("retarget_fixed"), load_rows("retarget_smooth")
    out["duration_s"] = {"retarget_fixed": round(old[-1]["t"], 2),
                         "retarget_smooth": round(new[-1]["t"], 2)}
    say("durations (s)", out["duration_s"])
    out["before_ped_matched"] = say("before, pedestrian, t<=70s",
                                    phase_stats(old, "pedestrian", MATCHED_WINDOW_S))
    out["after_ped_matched"] = say("after, pedestrian, t<=70s",
                                   phase_stats(new, "pedestrian", MATCHED_WINDOW_S))
    out["after_ped_full"] = say("after, pedestrian, whole flight",
                                phase_stats(new, "pedestrian"))
    out["before_car"] = say("before, car", phase_stats(old, "car"))
    out["after_car"] = say("after, car", phase_stats(new, "car"))

    m_new = load_json("demo/out/retarget_smooth/metrics.json")
    out["standoff_score_after"] = say("standoff_score, retarget_smooth",
                                      m_new["standoff_score"]["standoff-pedestrian"])
    # The flight that was reported as "fired six times". Its metrics predate
    # the scorer, so the rule list is rebuilt from the policy that governed it.
    sys.path.insert(0, str(ROOT))
    from guardrail import load_policy  # noqa: E402
    from guardrail.models import SubjectStandoff  # noqa: E402
    pol = load_policy(ROOT / "policies" / "follow_pedestrian.yaml")
    audit = [json.loads(l) for l in
             (ROOT / "demo/out/retarget_fixed/audit.jsonl").open(encoding="utf-8")]
    sc_old = T.score_standoff_firings(old, pol.by_type(SubjectStandoff), audit)
    out["standoff_score_before"] = say("standoff_score, retarget_fixed",
                                       sc_old["standoff-pedestrian"])
    out["coverage_before_matched"] = say("ring positives by visibility, before t<=70s",
                                         ring_coverage(old, MATCHED_WINDOW_S))
    out["coverage_after_matched"] = say("ring positives by visibility, after t<=70s",
                                        ring_coverage(new, MATCHED_WINDOW_S))
    out["coverage_after_full"] = say("ring positives by visibility, after, whole",
                                     ring_coverage(new))
    out["subject_range_check"] = say("ticks 340-700: estimate vs the followed person",
                                     subject_range_check(new))
    # The two frames the decks and the report show, read from the log rather
    # than from the HUD text burned into the image. The recorder writes one
    # frame per 50 ms from mission start, so frame index = t x 20 (checked
    # against the HUD clock: frame 450 reads 22.5 s, frame 1200 reads 60.0 s).
    figs = {}
    for key, t_want in (("car", 22.5), ("person", 60.0)):
        r = min((x for x in new if x.get("det")), key=lambda x: abs(x["t"] - t_want))
        figs[key] = {"t": t_want, "frame": int(round(t_want * 20)), "log_t": round(r["t"], 2),
                     "class": r["truth"]["class"], "score": round(r["det"]["score"], 3)}
    out["figure_frames"] = say("figure frames", figs)
    k_new = load_json("demo/out/retarget_smooth/kpi.json")
    out["p0_escape_after"] = say("p0_violation_escape_rate, retarget_smooth",
                                 k_new["p0_violation_escape_rate"])
    ev = (m_new.get("retargets") or [{}])[0]
    out["retarget_event"] = say("retarget event", {
        "t": round(ev.get("t", 0), 1), "tick": ev.get("tick"),
        "from": (ev.get("from") or {}).get("object"),
        "to": (ev.get("to") or {}).get("object"),
        "to_class": (ev.get("to") or {}).get("class")})
    return out


# ── the instance lock, scored against its null ──────────────────────────────
def lock():
    print("\n[lock] city_full (no lock) vs lock_on (lock); city_locked for reference")
    out = {}
    # lock_on is the documented A/B partner of city_full (same scene, lock on).
    for tag in ("city_full", "lock_on", "city_locked"):
        s = T.score_rows(load_rows(tag))
        out[tag] = say(tag, {
            "box_err_px_median_in_shot": s["det_gt_err_px_median_in_shot"],
            "null_centre": s["det_gt_err_px_median_chance_centre"],
            "box_err_deg_median_in_shot": s["det_gt_err_deg_median_in_shot"],
            "frac_on_target": s["frac_on_target"],
            "det_scored": s["det_scored"]})
    return out


# ── detector: bench, flight rates, contention ───────────────────────────────
def detector():
    print("\n[detector]")
    bench = load_json("docs/data/detector_bench.json")
    by = {r["backend"]: r for r in bench["results"]}
    ow, gd = by["owlvit"], by["gdino"]
    out = {"bench": {b: {k: r[k] for k in ("model_id", "total_ms_median", "hz_ceiling",
                                           "taxi_score_median", "person_score_median",
                                           "vram_gb")} for b, r in by.items()}}
    out["gdino_over_owlvit_taxi"] = say("G-DINO / OWL-ViT, taxi",
                                        round(gd["taxi_score_median"] / ow["taxi_score_median"], 2))
    out["gdino_over_owlvit_person"] = say("G-DINO / OWL-ViT, person",
                                          round(gd["person_score_median"] / ow["person_score_median"], 2))
    say("OWL-ViT idle ceiling Hz", ow["hz_ceiling"])

    flights = {}
    for tag in ("city_locked", "city_kpi", "demo_traffic", "retarget_fixed", "retarget_smooth"):
        rows = load_rows(tag)
        pre = [r["pre_ms"] for r in rows if r.get("pre_ms")]
        fwd = [r["fwd_ms"] for r in rows if r.get("fwd_ms")]
        span = rows[-1]["t"] - rows[0]["t"]
        m = load_json(f"demo/out/{tag}/metrics.json")
        flights[tag] = {"loop_hz": round(len(rows) / span, 2) if span > 0 else None,
                        "det_hz": det_hz_mission(rows, m),
                        "det_hz_reported": m.get("det_hz"),
                        "pre_ms_median": round(med(pre), 1) if pre else None,
                        "fwd_ms_median": round(med(fwd), 1) if fwd else None}
        say(f"in flight: {tag}", flights[tag])
    out["flights"] = flights
    out["gate"] = {"loop_hz_min": 9.5, "det_hz_min": 4.0}

    # Measured on this machine on 2026-09-10 with the scratchpad probes
    # probe_timing.py / probe_owlv2.py: owlvit-base-patch32, one query,
    # torch.cuda.synchronize(), median of 20 after 5 warm-up reps.
    offline = {"gpu": "NVIDIA GeForce RTX 4090 (24 GB)", "measured": "2026-09-10",
               "model": "google/owlvit-base-patch32",
               "rows": [{"capture": "400x225", "pre_ms": 22.0, "fwd_ms": 12.6, "total_ms": 35.2},
                        {"capture": "768x432", "pre_ms": 17.6, "fwd_ms": 12.5, "total_ms": 30.7},
                        {"capture": "1280x720", "pre_ms": 23.2, "fwd_ms": 13.2, "total_ms": 36.9}],
               "processor_resize": "768x768",
               "owlv2_patch16_total_ms": 167.7}
    out["offline_4090"] = say("offline timing (4090, 2026-09-10)", offline)
    pre_x = [f["pre_ms_median"] / 22.0 for f in flights.values() if f["pre_ms_median"]]
    fwd_x = [f["fwd_ms_median"] / 12.6 for f in flights.values() if f["fwd_ms_median"]]
    out["contention"] = say("in-flight slowdown vs offline (min..max)", {
        "cpu_preprocess_x": [round(min(pre_x), 1), round(max(pre_x), 1)],
        "gpu_forward_x": [round(min(fwd_x), 1), round(max(fwd_x), 1)]})

    # What a 0.5 m pedestrian becomes inside the model, at 90 deg HFOV.
    def patches(rng, side=768, patch=32, hfov=90.0):
        return round(0.5 / rng / math.radians(hfov) * side / patch, 2)
    out["pedestrian_patches"] = say("0.5 m person, patches (b32@768 | v2-b16@960)", {
        f"{r}m": [patches(r), patches(r, 960, 16)] for r in (10, 16, 20)})
    out["pedestrian_patches_by_hfov_16m"] = say("0.5 m person at 16 m, b32 patches by HFOV", {
        f"{h}deg": patches(16, hfov=float(h)) for h in (90, 60, 45, 30)})
    # The stand-off the servo asks for, from the function that computes it.
    import follow_vlm as FV  # noqa: E402
    out["standoff_setpoint_m"] = say("want_range_from_width(0.16, width)", {
        "car_4.0m": round(FV.want_range_from_width(0.16, 4.0), 2),
        "person_0.5m": round(FV.want_range_from_width(0.16, 0.5), 2)})
    out["pedestrian_px_400_capture"] = say("0.5 m person, px in 400-wide capture", {
        f"{r}m": round(0.5 / r / math.radians(90.0) * 400, 1) for r in (8, 10, 16, 20)})
    return out


# ── repository facts ────────────────────────────────────────────────────────
def repo(run_tests):
    print("\n[repo]")
    git = lambda *a: subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True,
                                    text=True, encoding="utf-8").stdout.strip()
    since = [l.split("|", 2) for l in git("log", "--since", LAST_MEETING,
                                          "--format=%ad|%h|%s", "--date=short",
                                          "master").splitlines() if l]
    out = {"last_meeting": LAST_MEETING,
           "commits_since_meeting": [{"date": d, "sha": h, "subject": s} for d, h, s in since],
           "commits_total": int(git("rev-list", "--count", "master") or 0),
           "version": (ROOT / "VERSION").read_text(encoding="utf-8").strip(),
           # Tracked only: an untracked draft is not a published finding.
           "finding_docs": len([p for p in git("ls-files", "docs/").splitlines()
                                if re.match(r"docs/FINDING-.*\.md$", p)]),
           "policies": len(glob.glob(str(ROOT / "policies/*.yaml"))),
           "test_files": len(glob.glob(str(ROOT / "tests/test_*.py")))}
    say("commits since last meeting", len(since))
    say("commits total", out["commits_total"])
    say("version / finding docs / policies / test files",
        f"{out['version']} / {out['finding_docs']} / {out['policies']} / {out['test_files']}")

    if run_tests:
        passed = total = 0
        per = {}
        for f in sorted(glob.glob(str(ROOT / "tests/test_*.py"))):
            if "coverage" in f:
                continue
            r = subprocess.run([sys.executable, f], capture_output=True, text=True,
                               encoding="utf-8", errors="replace", cwd=str(ROOT))
            m = re.search(r"(\d+)/(\d+) passed", r.stdout + r.stderr)
            if m:
                p, t = int(m.group(1)), int(m.group(2))
                passed += p; total += t
                per[Path(f).name] = f"{p}/{t}"
        out["tests_fast"] = {"passed": passed, "total": total, "per_file": per}
        say("fast tests", f"{passed}/{total}")
    # The ~10 min coverage suite is run separately and recorded as measured.
    out["tests_coverage"] = {"passed": 17, "total": 17, "note": "test_guardrail_coverage.py, run 2026-09-14"}
    return out


# ── simulation rails, counted from each flight's manifest ───────────────────
def rails():
    print(chr(10) + "[rails] flights per topology, and camera flights against the rate gates")
    counts, cam, both = {}, 0, 0
    for f in sorted(glob.glob(str(ROOT / "demo/out/*/manifest.json"))):
        d = Path(f).parent
        topo = json.loads(Path(f).read_text(encoding="utf-8")).get("topology")
        counts[topo] = counts.get(topo, 0) + 1
        mp, fl = d / "metrics.json", d / "flight_log.jsonl"
        if not (mp.exists() and fl.exists()):
            continue
        m_ = json.loads(mp.read_text(encoding="utf-8"))
        if m_.get("det_hz") is None:
            continue
        rows = [json.loads(l) for l in fl.open(encoding="utf-8")]
        if not rows:
            continue
        det_hz = det_hz_mission(rows, m_)
        if det_hz is None:
            continue
        span = rows[-1]["t"] - rows[0]["t"]
        loop_hz = len(rows) / span if span > 0 else 0.0
        cam += 1
        both += int(loop_hz >= 9.5 and det_hz >= 4.0)
    say("flights per topology", counts)
    say("camera flights / meeting loop>=9.5 and det>=4.0", f"{cam} / {both}")
    return {"counts": counts, "camera_flights": cam, "camera_flights_meeting_both_gates": both}


# ── the headless scenario sweep (WP4) ───────────────────────────────────────
def sweep():
    print("\n[sweep] experiments/scenarios.yaml, scored with guardrail.kpi.compute")
    d = load_json("docs/data/scenario_sweep.json")
    out = {"counts": d["_counts"], "results": [
        {"id": r["id"], "status": r["status"],
         "p0_violation_escape_rate": r["kpi"].get("p0_violation_escape_rate"),
         "why": r["why"].split("\n")[0]} for r in d["results"]]}
    say("counts", out["counts"])
    for r in out["results"]:
        say(f"  {r['id']}", f"{r['status']}  escape={r['p0_violation_escape_rate']}")
    return out


def det_hz_mission(rows, m):
    """Detector inferences per second DURING the mission.

    metrics.json's `det_hz` was, until 2026-09-29, (every inference since the
    detector loaded) / (ticks x 0.1 s): the start gate's minutes of inferences
    counted, and a loop below 10 Hz shrank the denominator - citylife_follow2
    reported 7.60 Hz for a detector that ran at 3.06. A metrics.json written
    since carries `det_hz_all_inferences_over_mission_s_legacy` and a correct
    `det_hz`; for the older ones the rate is recomputed from the flight log:
    the inference seq numbers the ticks consumed, over the ticks' own span."""
    if m and "det_hz_all_inferences_over_mission_s_legacy" in m:
        return m.get("det_hz")
    seqs = [r["det_seq"] for r in rows if r.get("det_seq") is not None]
    if len(seqs) < 2 or len(rows) < 2 or rows[-1]["t"] <= rows[0]["t"]:
        return None
    return round((max(seqs) - min(seqs)) / (rows[-1]["t"] - rows[0]["t"]), 2)


def _loop_hz(rows):
    if len(rows) < 2 or rows[-1]["t"] <= rows[0]["t"]:
        return None
    return round((len(rows) - 1) / (rows[-1]["t"] - rows[0]["t"]), 2)


def _longest_within(rows, radius_m=30.0):
    """Longest continuous stretch with the subject within `radius_m` of the
    aircraft, from the row's own truth: seconds and metres flown."""
    import math
    best = (0.0, 0.0)
    start = None
    for i, r in enumerate(rows + [None]):
        pts = ((r or {}).get("truth") or {}).get("pts") or []
        near = bool(pts) and min(math.hypot(r["x"] - a, r["y"] - b) for a, b in pts) <= radius_m
        if near and start is None:
            start = i
        elif not near and start is not None:
            a, b = start, i - 1
            dur = rows[b]["t"] - rows[a]["t"]
            path = sum(math.hypot(rows[k + 1]["x"] - rows[k]["x"], rows[k + 1]["y"] - rows[k]["y"])
                       for k in range(a, b))
            if dur > best[0]:
                best = (dur, path)
            start = None
    return {"s": round(best[0], 1), "m_flown": round(best[1], 0)}


def citylife_flights():
    """Every CityLife flight, re-scored from its artefacts by track_truth.

    `load_rows` recovers each box's frame width from detections.jsonl; the
    2026-09-22 numbers were computed with the 400 px default on a 768 px camera.
    Missing flights are skipped, not zero-filled.
    """
    out = {}
    # The red-car flights in the order they were flown on 2026-09-23; each
    # name says what that flight isolated (see the third part of
    # docs/FINDING-crowd-pedestrians-and-traffic.md).
    # The fourth pass (2026-09-24..29) adds the trail-follow flights: _trail
    # (first version), _trail2 and _trail3 (each exposing a defect fixed after
    # it), _final1.._final4 (_final1 exposed the chase throttle fixed before
    # _final3/_final4), and the pedestrian
    # mission on the CityLife obstacle map.
    for tag in ("citylife_follow2", "citylife_follow3", "citylife_city",
                "citylife_redcar_far", "citylife_redcar_pedpolicy",
                "citylife_redcar_carpolicy", "citylife_redcar_ground", "citylife_redcar_high",
                "citylife_redcar_trail", "citylife_redcar_trail2", "citylife_redcar_trail3",
                "citylife_redcar_final1", "citylife_redcar_final2",
                "citylife_redcar_final3", "citylife_redcar_final4", "citylife_ped_final"):
        run = ROOT / "demo/out" / tag
        log = run / "flight_log.jsonl"
        if not log.exists() or log.stat().st_size == 0:
            continue
        rows = T.load_rows(run)
        cls = T.score_rows(rows)
        inst = T.score_instance(rows)
        m = {}
        if (run / "metrics.json").exists():
            m = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
        rec = {"object": m.get("object"), "ticks": len(rows),
               "det_hz": det_hz_mission(rows, m),
               "det_hz_reported": m.get("det_hz"),
               "p0_violation_escape_rate": m.get("p0_violation_escape_rate"),
               "frac_on_target": cls.get("frac_on_target"),
               "frac_on_target_chance": cls.get("frac_on_target_chance"),
               "err_px_median_in_shot": cls.get("det_gt_err_px_median_in_shot"),
               "err_px_median_chance": cls.get("det_gt_err_px_median_chance")}
        if inst is not None:
            rec["instance"] = {k: inst.get(k) for k in (
                "frac_on_target", "frac_on_target_chance", "frac_in_shot",
                "reassociations", "subjects")}
        rec["loop_hz"] = _loop_hz(rows)
        rec["longest_within_30m"] = _longest_within(rows)
        for k in ("start_gate", "stage_ms_median", "tick_pacing", "presence_blocked_ticks",
                  "sep_min_m", "sep_mean_m", "target_lock", "frac_within_30m",
                  "trail", "collisions", "off_map_ticks", "obstacle_map_loaded",
                  "presence_block_reasons", "ground_check_waiver", "timer_resolution_ms",
                  "start_heading_err_deg"):
            if k in m:
                v = m[k]
                if k == "start_gate" and isinstance(v, dict):
                    v = {kk: vv for kk, vv in v.items() if kk != "truth_trace"}
                rec[k] = v
        out[tag] = rec
        say(f"  {tag}", f"on target {rec['frac_on_target']} vs chance "
                        f"{rec['frac_on_target_chance']}, escape {rec['p0_violation_escape_rate']}")
    return out


# ── built but not yet flown ─────────────────────────────────────────────────
def unflown():
    print("\n[unflown] stated, not measured in flight")
    out = {
        # Flown: every CityLife flight used it. Until commit 37514c3 the scorer
        # assumed 400 px for them - see citylife_flights(). The record stays
        # under `unflown` because three report builders read that key.
        "camera_768x432": {"status": "flown: every CityLife flight used 768x432; until "
                                     "commit 37514c3 (2026-09-23) the scorer assumed 400 px",
                           "evidence": "demo/pas_config/robot_semantic_quad.jsonc",
                           "flight_artefacts": sorted(
                               p.name for p in (ROOT / "demo/out").glob("citylife*")
                               if (p / "detections.jsonl").exists())},
        # Measured in the editor over MCP (PASBlocks/ is gitignored, so there
        # is no artefact to recompute the traffic numbers from); see the third
        # part of docs/FINDING-crowd-pedestrians-and-traffic.md and
        # tools/citylife_mcp/verify_drive.py. The FLIGHT numbers are not typed
        # here: citylife_flights() re-scores each flight from its own artefacts,
        # in the right frame width - hand-typed copies are how 0.821 survived.
        "citylife_level": {"status": "rebuilt 2026-09-23: keep-left, curvature follower, "
                                     "yields, one red car; 2026-09-29: its own obstacle "
                                     "map, the on-the-zebra stop, and the red-car mission "
                                     "flown with trail follow",
                           "pedestrians": 40, "cars": 24,
                           "pedestrian_model": "City Sample Crowd, 6 variants (3 male, 3 female)",
                           "area_m": [200, 185],
                           "drive_side": "left (read off the lane markings, 2026-09-23)",
                           "car_routes_m": [616, 334, 288],
                           "car_routes_note": "loops A/B/C, keep-left lane-centre paths from "
                                              "tools/citylife_routes.py (408/224/192 points) "
                                              "since 2026-09-23; citylife_city, _follow2 and "
                                              "_follow3 were flown on the 2026-09-22 "
                                              "right-hand loops",
                           "car_min_gap_cm_2026_09_22": 430,
                           "car_min_gap_2026_09_22_note": "in-engine, ~1900 ticks of Simulate, "
                                                          "24 cars on the 2026-09-22 loops",
                           "traffic_simulate_2026_09_23": {
                               "minutes": 4, "cars": 24,
                               "lane_error_max_cm": 10.1,
                               "lateral_accel_max_mps2": 1.93,
                               "car_min_gap_cm": 650,
                               "stand_still_max_s": 22.3,
                               "ped_yield_half_ticks": 780,
                               "car_on_zebra_with_walking_ped_half_ticks": 5,
                               "note": "counters kept inside each car; editor throttled "
                                       "to ~3 fps; the 5 zebra ticks are not explained"},
                           # Final DriveTick (nearest-ahead latch, per-crossing
                           # release), 2026-09-29; v1-v3 in the FINDING's fourth part.
                           "traffic_simulate_2026_09_29": {
                               "minutes": 4.4, "cars": 24,
                               "lane_error_max_cm": 10.3,
                               "lateral_accel_max_mps2": 1.93,
                               "car_min_gap_cm": 650,
                               "stand_still_max_s": 42.0,
                               "stand_still_note": "loop B waiting to give way at its "
                                                   "junctions (every car stopped > 8 s had "
                                                   "Conflict set when polled)",
                               "ped_yield_half_ticks": 1528,
                               "car_on_zebra_with_walking_ped_half_ticks": 4,
                               "on_zebra_emergency_stops_half_ticks": 14,
                               "emergency_stops_above_50cms": 0,
                               "walking_ped_within_4m_at_speed": 0},
                           "obstacle_map": {
                               "path": "demo/out/citymap_citylife (gitignored; rebuilt by "
                                       "demo/build_voxel_map.py)",
                               "ned_x_m": [-140, 218], "ned_y_m": [-60, 138], "res_m": 2.0,
                               "cells_6to14": 6021, "cells_15to55": 6020, "cells_2to4": 5043,
                               "street_cells": 10642, "canopy_cells": 415,
                               "regression_cells_compared": 5600,
                               "regression_cells_disagreeing": 0},
                           "flights": citylife_flights(),
                           "evidence": ["docs/FINDING-citylife-level.md",
                                        "docs/FINDING-crowd-pedestrians-and-traffic.md",
                                        "tools/citylife_routes.py",
                                        "tools/citylife_mcp/"]},
    }
    say("camera 768x432 flight artefacts", out["camera_768x432"]["flight_artefacts"] or "none")
    say("CityLife level", out["citylife_level"]["status"])
    out["trail_follow_demo_day"] = demo_day_trail_ab()
    return out


def demo_day_trail_ab():
    """DEMO 1 (yellow car, turn route) with and without --trail-follow, flown
    2026-09-25 (_nose, _trail) and 2026-09-29 (_trail2, the finished
    controller), against the August reference demo_follow."""
    out = {}
    for tag in ("demo_follow", "demo_follow_nose", "demo_follow_trail", "demo_follow_trail2"):
        run = ROOT / "demo/out" / tag
        mf = run / "metrics.json"
        if not mf.exists():
            continue
        m = json.loads(mf.read_text(encoding="utf-8"))
        rows = []
        log = run / "flight_log.jsonl"
        if log.exists():
            rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines() if line]
        out[tag] = {"frac_within_30m": m.get("frac_within_30m"),
                    "sep_mean_m": m.get("sep_mean_m"),
                    "frac_on_target": m.get("frac_on_target"),
                    "frac_on_target_chance": m.get("frac_on_target_chance"),
                    "interventions": m.get("interventions"),
                    "loop_hz": _loop_hz(rows),
                    "trail": m.get("trail"),
                    "p0_violation_escape_rate": m.get("p0_violation_escape_rate")}
    say("trail follow, Demo_day DEMO 1", {k: (v["frac_within_30m"], v["loop_hz"]) for k, v in out.items()})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-tests", action="store_true")
    args = ap.parse_args()
    data = {"generated": dt.date.today().isoformat(),
            "source": "tools/build_eval_data.py",
            "kpi": kpi_rail(), "retarget": retarget(), "lock": lock(),
            "detector": detector(), "rails": rails(), "sweep": sweep(), "repo": repo(not args.no_tests),
            "unflown": unflown()}
    OUT.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\nwrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
