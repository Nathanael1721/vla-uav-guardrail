"""Numbers and figures for the 28 Oct 2026 progress deck
(tools/deck/build_progress_1028_deck.js): every model flown end to end ("pure
VLA") and hierarchically, on the PC and with the flight client on the Jetson
Orin Nano, and the three-zone guardrail test (policies/follow_car_citylife_nfz3b.yaml;
its first placement, follow_car_citylife_nfz3.yaml, is reported as the first attempt).

Everything is read from the flights' own files at build time - nothing is
typed in:
  demo/out/{cmp,cmp2,orin}_*_{20261027,20261028}/metrics.json, kpi.json,
  flight_log.jsonl and (Jetson) console.log; demo/out/nfz3b_*_20261028/ and
  demo/out/nfz3_pc_hier_dronecats_20261028/ (the first attempt);
  C:/vla_drone_offload/jetson_runs/jetson_facts_20261028.json (the board's
  own answers over SSH: model, L4T, power mode, deployed commit).

Writes docs/data/progress_1028.json and, under
docs/share/2026-10-28-Progress/_build/, the zone map and the frames the deck
shows. Run with the vla-real env (matplotlib, shapely, yaml, PIL):

  C:/Users/natha/.conda/envs/vla-real/python.exe tools/deck/build_progress_1028_data.py
"""
from __future__ import annotations

import json
import re
import statistics
import sys
from pathlib import Path

import yaml
from shapely.geometry import Polygon

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:          # guardrail.projection when run as a script
    sys.path.insert(0, str(REPO))
from guardrail.geometry import edge_distance, signed_edge_distance   # noqa: E402
from guardrail.projection import raw_frame_origin, raw_point_metres   # noqa: E402
OUT = REPO / "demo" / "out"
DATA = REPO / "docs" / "data" / "progress_1028.json"
BUILD = REPO / "docs" / "share" / "2026-10-28-Progress" / "_build"
FACTS = Path("C:/vla_drone_offload/jetson_runs/jetson_facts_20261028.json")
NFZ3B = REPO / "policies" / "follow_car_citylife_nfz3b.yaml"
NFZ3 = REPO / "policies" / "follow_car_citylife_nfz3.yaml"   # the first attempt's placement
CITYMAP = REPO / "demo" / "out" / "citymap_citylife"

MODELS = {"aerovla": "AerialVLA (original)", "r1": "AerialVLA FT r1", "r2": "AerialVLA FT r2",
          "r3": "AerialVLA FT r3", "r4": "AerialVLA FT r4", "dronecats": "DroneCATS-2B",
          "racevla": "RaceVLA-7B"}
NFZ_RUNS = {"PC": ("nfz3b_pc_hier_dronecats_20261028", "cmp2_hier_dronecats_20261028", "dronecats"),
            "Jetson": ("nfz3b_orin_hier_aerovla_20261028", "orin_hier_aerovla_20261028", "aerovla")}
FIRST_ATTEMPT = "nfz3_pc_hier_dronecats_20261028"


def load(run: str) -> tuple[dict, dict]:
    d = OUT / run
    m = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
    k = json.loads((d / "kpi.json").read_text(encoding="utf-8")) if (d / "kpi.json").exists() else {}
    return m, k


def is_e2e(m: dict, run: str) -> bool:
    return "vla_hierarchy" not in m and (OUT / run / "vla_steps.jsonl").exists()


def score(m: dict, run: str) -> float | None:
    """The comparison's score: an end-to-end flight from its first
    acquisition (frac_within_30m_after_acquire), the controller's flights from
    their start gate's acquisition (follow_vlm's frac_within_30m)."""
    return m.get("frac_within_30m_after_acquire") if is_e2e(m, run) else m.get("frac_within_30m")


def hier_latency(run: str) -> float | None:
    for p in (OUT / run / "console.log", Path(f"D:/vla_drone_offload/matrix_20261028/{run}.log"),
              Path(f"D:/vla_drone_offload/matrix_20261027/{run}.log")):
        if p.exists():
            for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
                mm = re.search(r"ticks: follow .* latency p50 ([0-9.]+) s", line)
                if mm:
                    return round(float(mm.group(1)), 2)
    return None


def matrix() -> dict:
    rows = []
    for d in sorted(OUT.iterdir()):
        mt = re.fullmatch(r"(cmp|cmp2|orin)_(?:(e2e|hier)_)?([a-z0-9]+)_(2026102[78])", d.name)
        if not mt or not (d / "metrics.json").exists():
            continue
        pre, mode, model, date = mt.groups()
        if model == "follow":
            continue
        m, k = load(d.name)
        plat = "Jetson" if pre == "orin" else "PC"
        if model == "fixed12" and pre == "cmp":
            # 27 Oct: the scripted answers never parsed (Start-Process split the
            # text at its spaces); the controller flew it alone as the fallback.
            kind = "manual"
        elif model == "manual":
            kind = "manual"
        elif model == "fixed12":
            kind = "fixed12"
        else:
            kind = "hier" if mode == "hier" else "pure"
        h = m.get("vla_hierarchy") or {}
        s = (m.get("pilot") or {}).get("summary") or {}
        rows.append({
            "tag": d.name, "platform": plat, "kind": kind, "model": model if kind in ("hier", "pure") else kind,
            "score": score(m, d.name), "collisions_mission": (m.get("collisions") or {}).get("mission"),
            "p0_escape_rate": k.get("p0_violation_escape_rate", m.get("p0_violation_escape_rate")),
            "p0_violation_ticks": k.get("p0_violation_ticks"),
            "det_hz": m.get("det_hz"), "car_out_of_view": m.get("frac_absent"),
            "stand_off_p50": round((h.get("s_follow_m") or {}).get("p50"), 1) if (h.get("s_follow_m") or {}).get("p50") else None,
            "latency_p50_s": (round((s.get("latency_client_s") or {}).get("median"), 2)
                              if is_e2e(m, d.name) and (s.get("latency_client_s") or {}).get("median") else hier_latency(d.name)),
        })
    return rows


def mean(v):
    v = [x for x in v if x is not None]
    return round(statistics.mean(v), 3) if v else None


def zone_report(run: str, policy: Path = NFZ3B) -> dict:
    # The zones in the Shield's metres (2026-11-04): the policy files are WGS84
    # since 3 Nov, and v["x"] alone stopped with KeyError on them.
    pol = yaml.safe_load(policy.read_text(encoding="utf-8"))
    org = raw_frame_origin(pol)
    zones = [(c["id"], Polygon([raw_point_metres(v, org) for v in c["vertices"]]),
              float(c.get("margin_m", 0)))
             for c in pol["constraints"] if c["type"] == "polygon_fence"]
    rows = [json.loads(l) for l in (OUT / run / "flight_log.jsonl").open(encoding="utf-8")]
    out = {}
    for zid, poly, margin in zones:
        def sd(x, y, poly=poly):
            return signed_edge_distance(poly, x, y)
        d = [sd(r["x"], r["y"]) for r in rows]
        car = [r["truth"]["pts"][0] for r in rows if r.get("truth") and r["truth"].get("pts")]
        cd = [sd(*p) for p in car]
        out[zid] = {"drone_min_dist_m": round(min(d), 2), "drone_ticks_inside": sum(v < 0 for v in d),
                    "drone_ticks_in_margin": sum(0 <= v < margin for v in d),
                    "drone_ticks_within_12m": sum(v < 12 for v in d),
                    "car_ticks_inside": sum(v < 0 for v in cd), "car_ticks_in_margin": sum(0 <= v < margin for v in cd),
                    "ticks": len(rows)}
    return out


def nfz() -> dict:
    res = {}
    for plat, (run, base, model) in NFZ_RUNS.items():
        if not (OUT / run / "metrics.json").exists():
            res[plat] = {"tag": run, "missing": True}
            continue
        m, k = load(run)
        hud = m.get("policy_hud") or {}
        h = m.get("vla_hierarchy") or {}
        res[plat] = {
            "tag": run, "model": model, "name": MODELS[model], "within30": m.get("frac_within_30m"),
            "sep_mean_m": m.get("sep_mean_m"), "car_out_of_view": m.get("frac_absent"),
            "collisions_mission": (m.get("collisions") or {}).get("mission"),
            "nfz_entered": m.get("nfz_entered"), "nfz_s": m.get("nfz_s"),
            "fence_aim_ticks": m.get("fence_aim_ticks"), "nfz_hold_ticks": m.get("nfz_hold_ticks"),
            "hud_counts": hud.get("counts"), "hud_banner_ticks": hud.get("banner_ticks"),
            "p0_escape_rate": k.get("p0_violation_escape_rate", m.get("p0_violation_escape_rate")),
            "p0_violation_ticks": k.get("p0_violation_ticks"), "mission_success": k.get("mission_success"),
            "stand_off_p50": round((h.get("s_follow_m") or {}).get("p50") or 0, 1) or None,
            "authority": h.get("authority_share"),
            "zones": zone_report(run),
            "baseline": {"tag": base, "within30": load(base)[0].get("frac_within_30m"), "zones": zone_report(base)},
        }
    if (OUT / FIRST_ATTEMPT / "metrics.json").exists():
        m, k = load(FIRST_ATTEMPT)
        res["first_attempt"] = {
            "tag": FIRST_ATTEMPT, "policy": "follow_car_citylife_nfz3.yaml", "within30": m.get("frac_within_30m"),
            "nfz_entered": m.get("nfz_entered"), "nfz_hold_ticks": m.get("nfz_hold_ticks"),
            "hud_counts": (m.get("policy_hud") or {}).get("counts"),
            "p0_escape_rate": k.get("p0_violation_escape_rate", m.get("p0_violation_escape_rate")),
            "collisions_mission": (m.get("collisions") or {}).get("mission"),
            "zones": zone_report(FIRST_ATTEMPT, NFZ3)}
    return res


def jetson() -> dict:
    facts = json.loads(FACTS.read_text(encoding="utf-8")) if FACTS.exists() else {}
    lines = []
    log = OUT / "orin_hier_aerovla_20261028" / "console.log"
    if log.exists():
        for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
            if re.search(r"simulator reachable at|Connecting to simulation server at|\[grounder\].*loaded|"
                         r"\[vla-hierarchy\] AerialVLA .* at 127\.0\.0\.1|\[start\] subject acquired after", line):
                lines.append(re.sub(r"(\[start\] subject acquired after [0-9]+ s).*", r"\1 ...", line)[:150])
    return {"facts": facts, "evidence_log": "demo/out/orin_hier_aerovla_20261028/console.log",
            "evidence_lines": lines[:6]}


def figures(nfz_res: dict) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    BUILD.mkdir(parents=True, exist_ok=True)
    occ = np.load(CITYMAP / "occ_day_flightband_6to14.npz")
    st = np.load(CITYMAP / "street.npz")
    res, ox, oy = float(occ["res"]), float(occ["origin_x"]), float(occ["origin_y"])
    O, S = occ["occ"], st["street"]
    nx, ny = O.shape
    img = np.ones((ny, nx, 3))
    img[S.T > 0] = (0.90, 0.92, 0.93)
    img[O.T > 0] = (0.27, 0.27, 0.26)
    fig, ax = plt.subplots(figsize=(9.2, 6.0))
    ax.imshow(img, origin="lower", extent=[ox, ox + nx * res, oy, oy + ny * res])
    pol = yaml.safe_load(NFZ3B.read_text(encoding="utf-8"))
    org = raw_frame_origin(pol)                 # WGS84 since 3 Nov: the Shield's metres
    for c in pol["constraints"]:
        if c["type"] == "polygon_fence":
            ring = [raw_point_metres(p, org) for p in c["vertices"]]
            v = ring + [ring[0]]
            ax.fill([p[0] for p in v], [p[1] for p in v], color="#D9534F", alpha=0.45, lw=0)
            ax.plot([p[0] for p in v], [p[1] for p in v], color="#B02A26", lw=1.2)
            cx = sum(p[0] for p in ring) / 4
            top = max(p[1] for p in ring)
            # labels inside the map, on a white box so no track runs through them
            ty, va = (top - 18, "top") if top > 125 else (top + 3, "bottom")
            ax.text(cx + (14 if c["id"] == "nfz-frontage" else 0), ty, c["id"], color="#B02A26", fontsize=8.5,
                    ha="center", va=va, fontweight="bold",
                    bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": "#B02A26", "lw": 0.6, "alpha": 0.92})
    colors = {"PC": "#249DB2", "Jetson": "#E08A1E"}
    car_drawn = False
    for plat, r in ((p, nfz_res[p]) for p in ("PC", "Jetson") if p in nfz_res):
        if r.get("missing"):
            continue
        rows = [json.loads(l) for l in (OUT / r["tag"] / "flight_log.jsonl").open(encoding="utf-8")]
        if not car_drawn:
            car = [q["truth"]["pts"][0] for q in rows if q.get("truth") and q["truth"].get("pts")]
            ax.plot([p[0] for p in car], [p[1] for p in car], color="#C0392B", lw=0.8, ls="--", label="red car (truth)")
            car_drawn = True
        ax.plot([q["x"] for q in rows], [q["y"] for q in rows], color=colors[plat], lw=1.6,
                label=f"drone, {plat}: {r['name']}")
    ax.set_xlim(20, 225); ax.set_ylim(-55, 146); ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(loc="center", fontsize=9, frameon=True)
    fig.savefig(BUILD / "nfz3_map.png", dpi=170, bbox_inches="tight")
    plt.close(fig)
    out = {"nfz3_map": str((BUILD / "nfz3_map.png").relative_to(REPO)).replace("\\", "/")}
    # one view frame per NFZ flight, at the tick nearest a new zone (the HUD is
    # burned into the frame: banner, rule panel, map)
    from PIL import Image
    for plat, r in ((p, nfz_res[p]) for p in ("PC", "Jetson") if p in nfz_res):
        if r.get("missing"):
            continue
        run = OUT / r["tag"]
        rec = run / "view" / "recorder.json"
        if not rec.exists():
            continue
        ft = json.loads(rec.read_text(encoding="utf-8")).get("frame_t") or []
        rows = [json.loads(l) for l in (run / "flight_log.jsonl").open(encoding="utf-8")]
        poly = Polygon([(100, 122), (124, 122), (124, 134), (100, 134)])
        near = min(rows, key=lambda q: edge_distance(poly, q["x"], q["y"]) + (0 if 98 <= q["x"] <= 126 else 99))
        idx = min(range(len(ft)), key=lambda i: abs(ft[i] - near["t"])) if ft else None
        fpv = run / "view" / "fpv" / f"{idx:05d}.jpg" if idx is not None else None
        if fpv and fpv.exists():
            dst = BUILD / f"nfz_frame_{plat.lower()}.jpg"
            Image.open(fpv).convert("RGB").save(dst, quality=90)
            out[f"nfz_frame_{plat.lower()}"] = str(dst.relative_to(REPO)).replace("\\", "/")
            out[f"nfz_frame_{plat.lower()}_t"] = round(near["t"], 1)
    return out


def main() -> None:
    rows = matrix()
    table = {}
    for r in rows:
        table.setdefault(r["platform"], {}).setdefault(r["kind"], {}).setdefault(r["model"], []).append(r)
    top3 = {}
    for plat in ("PC", "Jetson"):
        for kind in ("pure", "hier"):
            ranked = sorted(((MODELS[m], mean([x["score"] for x in v]), [x["score"] for x in v])
                             for m, v in table.get(plat, {}).get(kind, {}).items()), key=lambda t: -(t[1] or 0))
            top3[f"{plat} {kind}"] = ranked
    nfz_res = nfz()
    safety_rows = rows + [{"collisions_mission": r.get("collisions_mission"), "p0_escape_rate": r.get("p0_escape_rate"),
                           "p0_violation_ticks": r.get("p0_violation_ticks")}
                          for r in nfz_res.values() if not r.get("missing") and "zones" in r]
    data = {
        "matrix_rows": rows,
        "ranking": top3,
        "baseline": {p: [x["score"] for x in table.get(p, {}).get("manual", {}).get("manual", [])] for p in ("PC", "Jetson")},
        "fixed12": {p: [x["score"] for x in table.get(p, {}).get("fixed12", {}).get("fixed12", [])] for p in ("PC", "Jetson")},
        "det_hz_manual": {p: [x["det_hz"] for x in table.get(p, {}).get("manual", {}).get("manual", [])] for p in ("PC", "Jetson")},
        "safety": {
            "flights": len(safety_rows),
            "mission_collisions": sum(int(x["collisions_mission"] or 0) for x in safety_rows),
            "p0_escape_rates": sorted({str(x["p0_escape_rate"]) for x in safety_rows}),
            "p0_violation_ticks": sum(int(x["p0_violation_ticks"] or 0) for x in safety_rows),
        },
        "nfz": nfz_res,
        "jetson": jetson(),
    }
    data["figures"] = figures(nfz_res)
    DATA.parent.mkdir(parents=True, exist_ok=True)
    DATA.write_text(json.dumps(data, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {DATA.relative_to(REPO)}: {len(rows)} matrix flights, NFZ "
          f"{[p for p, r in nfz_res.items() if not r.get('missing')]}, figures {list(data['figures'])}")


if __name__ == "__main__":
    main()
