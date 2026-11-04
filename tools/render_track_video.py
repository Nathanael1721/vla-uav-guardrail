"""Render a flight's video from its flight log, for rails that record no camera.

    python tools/render_track_video.py --tag gazebo_shield_on_20261102
    python tools/render_track_video.py --tag ros2_shield_on --fps 10 --out x.mp4

The SITL rails (pymavlink, ROS 2 + MAVROS 2, Gazebo Harmonic) fly ArduPilot
without a camera the Shield sees, so a run leaves numbers and a trajectory.png
and no video. This draws the run as it was logged, tick by tick: the policy's
zones (and their margin rings) and altitude band, the mission target, the
aircraft's track, the pilot's command and what the Shield emitted, the
escalation FSM's state and the edge that changed it, and the rules the Shield
saw violated. Every frame says, in its corner, that it is a replay of the
flight log and not a camera recording: it shows what the Shield logged, which
is the claim, and nothing a camera would add.

Inputs, all from demo/out/<tag>/: flight_log.jsonl (the per-tick record),
policy_g0.json (the policy in force at generation 0) and prompt.yaml (the
mission target, when there is one). Later generations' zones (hot-apply) are
drawn from the per-tick `generation` with the run's policy_g<N>.json files.

Since 2026-11-03 it also draws what a flight of that date needs to be read:
a corridor (keep-in, the tube around its centerline) as the swaps of its
generation leave it (corridor_swap), a zone a time_window_switch has turned
off as switched off (dashed, "off"), the autopilot's mode on every frame
(the rows' `mode`), and, for a mission-style flight (sitl/ros2_mission_node.py),
the planned waypoints and the waypoint the autopilot holds for the next leg.
A run with no escalation FSM in its rows (Shield off, mission style) says so
in the title instead of showing an FSM state it never had. The axes are the
rows' own: x North (horizontal), y East (vertical); until this date they
were labelled the other way round.

Writes demo/out/<tag>/<tag>_track.mp4 by default, through a .partial file
that is checked (non-empty) before it replaces the target.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

STATE_COLOURS = {"Normal": "#2e7d32", "Brake": "#f9a825", "Loiter": "#ef6c00",
                 "RTL": "#c62828", "Land": "#6a1b9a"}


def load_rows(run: Path) -> list[dict]:
    rows = []
    with open(run / "flight_log.jsonl", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    if not rows:
        raise SystemExit(f"{run / 'flight_log.jsonl'} has no rows")
    return rows


def load_policies(run: Path) -> dict[int, dict]:
    out = {}
    for p in sorted(run.glob("policy_g*.json")):
        try:
            g = int(p.stem.split("_g", 1)[1])
        except ValueError:
            continue
        out[g] = json.loads(p.read_text(encoding="utf-8"))
    return out


def policy_origin(policy: dict):
    """The policy's frame origin (lat, lon): stated, or derived as the model
    derives it (projection.derive_frame_origin); None for a metre policy."""
    o = policy.get("origin")
    if isinstance(o, dict) and o.get("lat") is not None and o.get("lon") is not None:
        return (float(o["lat"]), float(o["lon"]))
    from guardrail.projection import derive_frame_origin
    return derive_frame_origin(policy.get("constraints", []))


def point_xy(p: dict, origin) -> tuple[float, float]:
    """A point in local metres (x North, y East): as written, or - a WGS84
    point, which a policy dump has carried as lat/lon only since IR schema
    3.1.0 (2026-11-03) - projected about the policy's frame origin exactly as
    the Shield projects it (projection.local_metres). Until 2026-11-04 this
    file read p["x"] alone and stopped with KeyError: 'x' on every flight of
    a WGS84 policy."""
    if p.get("x") is not None and p.get("y") is not None:
        return float(p["x"]), float(p["y"])
    if origin is None or p.get("lat") is None or p.get("lon") is None:
        raise ValueError(f"a point with neither x/y nor a placeable lat/lon: {p}")
    from guardrail.projection import local_metres
    return local_metres(origin, float(p["lat"]), float(p["lon"]))


def zones(policy: dict) -> list[dict]:
    """Keep-out shapes in local metres: polygons and circles, with margins."""
    out = []
    org = policy_origin(policy)
    for c in policy.get("constraints", []):
        t = c.get("type")
        if t in ("polygon_fence", "dynamic_nfz") and c.get("vertices"):
            out.append({"id": c.get("id"), "kind": "poly",
                        "xy": [point_xy(v, org) for v in c["vertices"]],
                        "margin": float(c.get("margin_m") or 0.0),
                        "hard": c.get("constraint_type", "hard") == "hard",
                        "scheduled": bool(c.get("valid_time"))})
        elif t == "circle_fence" and c.get("center"):
            out.append({"id": c.get("id"), "kind": "circle",
                        "c": point_xy(c["center"], org),
                        "r": float(c.get("radius_m", 0.0)),
                        "margin": float(c.get("margin_m") or 0.0),
                        "hard": c.get("constraint_type", "hard") == "hard",
                        "scheduled": bool(c.get("valid_time"))})
    return out


def switched_off(policy: dict) -> set[str]:
    """Ids a time_window_switch with no schedule holds OFF (active false)."""
    off, on = set(), set()
    for c in policy.get("constraints", []):
        if c.get("type") == "time_window_switch" and not c.get("valid_time"):
            tgt = c.get("target_id")
            if c.get("active") is False:
                off.add(tgt)
                on.discard(tgt)
            elif c.get("active") is True:
                on.add(tgt)
                off.discard(tgt)
    return off


def corridors(policy: dict) -> list[dict]:
    """Each corridor as its generation flies it: the last corridor_swap that
    targets it (with no schedule) replaces its centerline, width and band.
    {"id", "swapped_by", "centerline": [(x, y)...], "width_m", "band"}."""
    cons = policy.get("constraints", [])
    org = policy_origin(policy)
    out = []
    for c in cons:
        if c.get("type") != "corridor":
            continue
        cur, by = c, None
        for s in cons:
            if s.get("type") == "corridor_swap" and s.get("target_id") == c.get("id") \
                    and not s.get("valid_time"):
                cur, by = s, s.get("id")
        out.append({"id": c.get("id"), "swapped_by": by,
                    "centerline": [point_xy(p, org) for p in cur.get("centerline", [])],
                    "width_m": float(cur.get("width_m", 0.0)),
                    "band": (float(cur.get("altitude_floor_m", 0.0)),
                             float(cur.get("altitude_ceiling_m", 1000.0)))})
    return out


def corridor_outline(cor: dict) -> list[tuple[float, float]]:
    """The tube's outline: points within width/2 of the centerline segments
    (round ends), as the Shield measures a corridor."""
    from shapely.geometry import LineString
    ring = LineString(cor["centerline"]).buffer(cor["width_m"] / 2.0, quad_segs=16)
    return list(ring.exterior.coords)


def mission_waypoints(run: Path) -> list[tuple[float, float]]:
    """A mission-style flight's planned waypoints (mission.json), or []."""
    p = run / "mission.json"
    if not p.is_file():
        return []
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
    except ValueError:
        return []
    return [(float(w[0]), float(w[1])) for w in (doc.get("plan") or [])]


def altitude_band(policy: dict) -> tuple[float, float] | None:
    for c in policy.get("constraints", []):
        if c.get("type") == "altitude_envelope":
            return float(c.get("alt_min_m", 0.0)), float(c.get("alt_max_m", 0.0))
    return None


def mission_target(run: Path) -> tuple[float, float] | None:
    p = run / "prompt.yaml"
    if not p.is_file():
        return None
    try:
        import yaml
        doc = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    except Exception:                                   # noqa: BLE001
        return None
    tgt = ((doc.get("mission") or {}).get("target")) or {}
    if "x" in tgt and "y" in tgt:
        return float(tgt["x"]), float(tgt["y"])
    return None


def state_at(rows: list[dict], i: int) -> tuple[str, str | None]:
    """The FSM state after tick i, and the last edge taken up to it."""
    edge = None
    state = "Normal"
    for r in rows[: i + 1]:
        if r.get("fsm_state_after"):
            state = r["fsm_state_after"]
        if r.get("fsm_edge"):
            edge = f"{r.get('fsm_state_before')}->{r.get('fsm_state_after')} [{r['fsm_edge']}] t={r['t']:.1f}s"
    return state, edge


def bounds(rows, zs, target, pad=6.0):
    xs = [r["x"] for r in rows] + [0.0]
    ys = [r["y"] for r in rows] + [0.0]
    for z in zs:
        if z["kind"] == "poly":
            xs += [p[0] for p in z["xy"]]
            ys += [p[1] for p in z["xy"]]
        else:
            xs += [z["c"][0] - z["r"], z["c"][0] + z["r"]]
            ys += [z["c"][1] - z["r"], z["c"][1] + z["r"]]
    if target:
        xs.append(target[0])
        ys.append(target[1])
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    span = max(x1 - x0, y1 - y0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2


def render(run: Path, out: Path, fps: int = 10, step: int = 1) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Circle, Polygon
    from tools.encode_deliverable import find_ffmpeg

    rows = load_rows(run)
    pols = load_policies(run)
    if not pols:
        raise SystemExit(f"no policy_g*.json in {run}")
    g0 = pols[min(pols)]
    band = altitude_band(g0)
    target = mission_target(run)
    allz = [z for p in pols.values() for z in zones(p)]
    allc = [{"id": c["id"], "kind": "poly", "xy": corridor_outline(c)}
            for p in pols.values() for c in corridors(p) if len(c["centerline"]) >= 2]
    wps = mission_waypoints(run)
    x0, x1, y0, y1 = bounds(rows, allz + allc, target)
    has_fsm = any(r.get("fsm_state_after") for r in rows)

    plt.rcParams["animation.ffmpeg_path"] = find_ffmpeg()
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.04, 0.08, 0.52, 0.84])
    az = fig.add_axes([0.62, 0.55, 0.35, 0.37])
    info = fig.add_axes([0.62, 0.06, 0.35, 0.42])
    info.axis("off")

    ts = [r["t"] for r in rows]
    ups = [r["up"] for r in rows]
    az.plot(ts, ups, color="#90a4ae", lw=1)
    if band:
        az.axhspan(band[0], band[1], color="#c8e6c9", alpha=0.6, label="altitude band")
    alt_dot, = az.plot([], [], "o", color="#1565c0")
    az.set_xlabel("t (s)")
    az.set_ylabel("altitude up (m)")
    az.set_title("altitude")

    zone_patches: dict[int, list] = {}

    def draw_zones(gen: int):
        for g, patches in zone_patches.items():
            for p in patches:
                p.set_visible(g == gen)
        if gen in zone_patches:
            return
        patches = []
        pol = pols.get(gen, g0)
        off = switched_off(pol)
        for cor in corridors(pol):
            if len(cor["centerline"]) < 2:
                continue
            patches.append(ax.add_patch(Polygon(corridor_outline(cor), closed=True, fill=False,
                                                ec="#2e7d32", lw=1.6, ls="-")))
            cx, cy = cor["centerline"][-1]
            label = cor["id"] + (f" (swap {cor['swapped_by']})" if cor["swapped_by"] else "")
            patches.append(ax.text(cx, cy, f"corridor {label}", fontsize=7, color="#2e7d32"))
        for z in zones(pol):
            col = "#c62828" if z["hard"] else "#ef6c00"
            is_off = z["id"] in off
            # A zone with a schedule (valid_time) is in force only inside its
            # window, which this drawing does not evaluate: outlined, labelled.
            sched = z.get("scheduled") and not is_off
            kw = (dict(fill=False, ec=col, ls=":", lw=1.0) if is_off
                  else dict(fill=False, ec=col, ls="--", lw=1.0, hatch="//") if sched
                  else dict(fc=col, alpha=0.25, ec=col))
            if z["kind"] == "poly":
                patches.append(ax.add_patch(Polygon(z["xy"], closed=True, **kw)))
            else:
                patches.append(ax.add_patch(Circle(z["c"], z["r"], **kw)))
                if z["margin"]:
                    patches.append(ax.add_patch(Circle(z["c"], z["r"] + z["margin"], fill=False,
                                                       ec=col, ls="--", lw=0.8)))
            patches.append(ax.text(*(z["xy"][0] if z["kind"] == "poly" else z["c"]),
                                   z["id"] + (" (off)" if is_off else "")
                                   + (" (valid_time schedule)" if sched else ""),
                                   fontsize=8, color=col))
        zone_patches[gen] = patches

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("x north (m)")
    ax.set_ylabel("y east (m)")
    ax.grid(alpha=0.3)
    ax.plot([0], [0], "s", color="#455a64", ms=8)
    ax.text(0.5, -1.5, "home", fontsize=8, color="#455a64")
    if target:
        ax.plot([target[0]], [target[1]], "*", color="#1565c0", ms=14)
        ax.text(target[0] + 0.6, target[1] + 0.6, "target", fontsize=8, color="#1565c0")
    if wps:
        ax.plot([0.0] + [w[0] for w in wps], [0.0] + [w[1] for w in wps], ":", color="#5c6bc0",
                lw=1.0)
        ax.plot([w[0] for w in wps], [w[1] for w in wps], "x", color="#5c6bc0", ms=9,
                label="planned waypoints")
    wp_hold, = ax.plot([], [], "D", color="#ef6c00", ms=8,
                       label="next waypoint (autopilot)" if wps else None)
    wp_line, = ax.plot([], [], "--", color="#ef6c00", lw=1.0)
    track, = ax.plot([], [], "-", color="#1565c0", lw=1.5)
    dot, = ax.plot([], [], "o", color="#1565c0", ms=9)
    raw_q = ax.quiver([0], [0], [0], [0], color="#9e9e9e", scale=1, scale_units="xy", angles="xy",
                      width=0.006, label="pilot command (x2 s)")
    emi_q = ax.quiver([0], [0], [0], [0], color="#2e7d32", scale=1, scale_units="xy", angles="xy",
                      width=0.006, label="Shield emitted (x2 s)")
    ax.legend(loc="lower right", fontsize=8)
    title = fig.text(0.04, 0.95, "", fontsize=13, weight="bold")
    fig.text(0.04, 0.015, "Replay of the flight log (demo/out/%s/flight_log.jsonl), not a camera "
             "recording." % run.name, fontsize=9, color="#616161")
    body = info.text(0.0, 1.0, "", va="top", family="monospace", fontsize=9)

    idx = list(range(0, len(rows), max(1, step)))
    if idx[-1] != len(rows) - 1:
        idx.append(len(rows) - 1)

    def frame(k):
        i = idx[k]
        r = rows[i]
        draw_zones(int(r.get("generation") or 0))
        track.set_data([q["x"] for q in rows[: i + 1]], [q["y"] for q in rows[: i + 1]])
        dot.set_data([r["x"]], [r["y"]])
        alt_dot.set_data([r["t"]], [r["up"]])
        raw, emi = r.get("raw") or {}, r.get("emitted") or {}
        raw_q.set_offsets([[r["x"], r["y"]]])
        raw_q.set_UVC([2 * raw.get("vx", 0.0)], [2 * raw.get("vy", 0.0)])
        emi_q.set_offsets([[r["x"], r["y"]]])
        emi_q.set_UVC([2 * emi.get("vx", 0.0)], [2 * emi.get("vy", 0.0)])
        state, edge = state_at(rows, i)
        nwp = r.get("next_wp_autopilot")
        if nwp:
            wp_hold.set_data([nwp[0]], [nwp[1]])
            wp_line.set_data([r["x"], nwp[0]], [r["y"], nwp[1]])
        else:
            wp_hold.set_data([], [])
            wp_line.set_data([], [])
        if has_fsm:
            dot.set_color(STATE_COLOURS.get(state, "#1565c0"))
            title.set_text(f"{run.name}   t = {r['t']:5.1f} s   FSM: {state}")
            title.set_color(STATE_COLOURS.get(state, "black"))
        else:
            title.set_text(f"{run.name}   t = {r['t']:5.1f} s   no escalation FSM   "
                           f"mode {r.get('mode') or '-'}")
            title.set_color("black")
        viol = ", ".join(sorted({v.get("rule_id", "?") for v in r.get("violations") or []})) or "-"
        reps = "; ".join(f"{p.get('operator')}" + (f" {p['magnitude_m']:.2f} m"
                                                    if isinstance(p.get("magnitude_m"), (int, float))
                                                    else "")
                         for p in r.get("repairs") or []) or "-"
        hz = math.hypot(raw.get("vx", 0.0), raw.get("vy", 0.0))
        ez = math.hypot(emi.get("vx", 0.0), emi.get("vy", 0.0))
        body.set_text(
            f"tick {r.get('tick')}   generation {r.get('generation')}\n"
            f"position  x {r['x']:7.2f}  y {r['y']:7.2f}  up {r['up']:6.2f} m\n"
            f"pilot     |v_h| {hz:5.2f} m/s  vz {raw.get('vz_up', 0.0):5.2f}\n"
            f"emitted   |v_h| {ez:5.2f} m/s  vz {emi.get('vz_up', 0.0):5.2f}\n"
            f"flown     {r.get('flown')}   setpoint {r.get('setpoint')}\n"
            f"forecast violations: {viol[:60]}\n"
            f"repairs: {reps[:60]}\n"
            f"last FSM edge: {edge or '-'}\n"
            f"autopilot mode: {r.get('mode') or r.get('autopilot_set_mode') or '-'}"
            + (f"\nnext waypoint {r.get('next_wp_index')}: planned {r.get('next_wp_planned')}\n"
               f"  autopilot holds {nwp}  replaced {r.get('wp_replaced')}"
               if r.get("next_wp_planned") is not None else ""))
        return track, dot, alt_dot, raw_q, emi_q, title, body, wp_hold, wp_line

    anim = animation.FuncAnimation(fig, frame, frames=len(idx), blit=False)
    out.parent.mkdir(parents=True, exist_ok=True)
    partial = out.with_name(out.stem + ".partial.mp4")
    anim.save(str(partial), writer=animation.FFMpegWriter(fps=fps, bitrate=2400,
                                                          extra_args=["-pix_fmt", "yuv420p"]))
    plt.close(fig)
    if not partial.is_file() or partial.stat().st_size == 0:
        raise SystemExit(f"nothing written to {partial}")
    os.replace(partial, out)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--tag", required=True, help="run folder under demo/out/")
    ap.add_argument("--demo-out", type=Path, default=ROOT / "demo" / "out")
    ap.add_argument("--fps", type=int, default=10,
                    help="frames per second; 10 with --step 1 is real time at the 10 Hz monitor")
    ap.add_argument("--step", type=int, default=1, help="draw every Nth tick")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    run = a.demo_out / a.tag
    if not run.is_dir():
        print(f"no run folder {run}", file=sys.stderr)
        return 2
    out = a.out or run / f"{a.tag}_track.mp4"
    p = render(run, out, fps=a.fps, step=a.step)
    print(f"[video] {p} ({p.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
