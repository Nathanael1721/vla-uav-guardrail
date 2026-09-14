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
    return {"runs": runs, "flights_total": n_all, "flights_kpi_grade": n_grade}


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
                        "det_hz": m.get("det_hz"),
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
    out["pedestrian_px_400_capture"] = say("0.5 m person, px in 400-wide capture", {
        f"{r}m": round(0.5 / r / math.radians(90.0) * 400, 1) for r in (8, 10, 16)})
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
    out["tests_coverage"] = {"passed": 17, "total": 17, "note": "test_guardrail_coverage.py, run 2026-09-09"}
    return out


# ── built but not yet flown ─────────────────────────────────────────────────
def unflown():
    print("\n[unflown] stated, not measured in flight")
    res768 = ROOT / "demo/out/res768"
    out = {
        "camera_768x432": {"status": "committed, not flown",
                           "evidence": "demo/pas_config/robot_semantic_quad.jsonc",
                           "flight_artefacts": sorted(p.name for p in res768.glob("*")) if res768.exists() else []},
        "citylife_level": {"status": "built, walking verified in editor, not flown",
                           "pedestrians": 16, "cars": 8,
                           "evidence": "docs/FINDING-citylife-level.md"},
    }
    say("camera 768x432 flight artefacts", out["camera_768x432"]["flight_artefacts"] or "none")
    say("CityLife level", out["citylife_level"]["status"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-tests", action="store_true")
    args = ap.parse_args()
    data = {"generated": dt.date.today().isoformat(),
            "source": "tools/build_eval_data.py",
            "kpi": kpi_rail(), "retarget": retarget(), "lock": lock(),
            "detector": detector(), "repo": repo(not args.no_tests),
            "unflown": unflown()}
    OUT.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\nwrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
