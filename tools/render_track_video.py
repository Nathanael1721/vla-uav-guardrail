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

Since 2026-11-11 a MOVING zone (a dynamic_nfz with a `motion`: yaw_rate_dps,
vx_mps, vy_mps) is drawn where its motion has taken it at each frame's time,
by the model's own geometry (guardrail.models.DynamicNFZ.ring_at), from the
time it was last spawned or edited: the run's events.jsonl
`policy_generation` record "hot_apply <id>" (zone_anchors); a zone the
policy declared, or one no event names, moves from the first row of the
generation that brought it. Until then every zone was drawn at its authored
vertices and a spinning bar stood still. The age is read on the rows' clock
(mission time); the Shield places the zone on its own tick clock (counted
0.1 s ticks on the hil node, anchored at its last tick), so the drawing may
lead the Shield's placement by about one tick: on hs06b_004_dyn-nfz-rotate-
spin (a 40 m bar at 6 deg/s) the drawn bar's signed distance to the
aircraft is within 0.21 m of the one the Shield logged (rows' `zone_d_m`)
on every row, under a pixel at the drawing's scale. The footer names the log the
frames replay by its real path (it said demo/out/<tag>/flight_log.jsonl for
every run, wherever it was), and --stills draws chosen frames as PNGs
instead of a video. `--tag` may name a nested run (hil_smoke/<run>).

Since 2026-11-18 a run whose rows carry up_ref "terrain" (up = height above
a ground grid, up_home = height above home, msl_m = MSL) has its altitude
panel drawn in MSL (terrain_panel). Only the ROS 2 Shield node with --ground
over a DEM (terrain.TerrainReference) writes such rows; the headless
harness's rows carry msl_m but no up_ref (and its episodes no policy_g*.json,
so this tool refuses them), and the node over a FLAT --ground writes neither
(metrics.json ground.up_ref "home"). The panel shows the aircraft's msl_m,
the ground under it (msl_m - up), home's elevation, each AGL
altitude_envelope of policy_g0 as a band that follows the ground (ground +
min .. ground + max) and each MSL one (altitude_ref MSL) as a fixed band,
labelled by rule id. Every interval the aircraft is outside a band is shaded
red on the time axis (breach_intervals), so every frame shows it, and the
dot turns red inside one; the text block shows up, up_home, msl_m and the
ground. The y axis covers the ground, the aircraft, the MSL bands and the
AGL floors; an AGL ceiling above that is named at the top of the panel, not
drawn (at 581-653 m the 0.49 m floor breach of msl_dem_sitl_01 was under two
pixels). Until then the panel drew `up` on the same axis as the MSL band,
which on msl_dem_sitl_01 put a 590-624 m band over a 7-20 m trace.

On any other run the panel draws `up` (above home) as before. An MSL
envelope there is drawn on that axis only when the run says where home is in
MSL (metrics.json ground: a flat --ground's elevation, flat_ground_msl),
offset by it and labelled; otherwise it is named as not drawn. Until then the
first altitude_envelope was drawn on the up axis whatever its reference. A run
with no MSL envelope is drawn exactly as before (stills of dev_compose_01
and gcs_fence_off_20261104 byte-identical before and after).

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
    derives it; None for a metre policy (projection.raw_frame_origin)."""
    from guardrail.projection import raw_frame_origin
    return raw_frame_origin(policy)


def point_xy(p: dict, origin) -> tuple[float, float]:
    """A point in local metres (x North, y East): as written, or - a WGS84
    point, which a policy dump has carried as lat/lon only since IR schema
    3.1.0 (2026-11-03) - projected about the policy's frame origin exactly as
    the Shield projects it (projection.raw_point_metres). Until 2026-11-04
    this file read p["x"] alone and stopped with KeyError: 'x' on every
    flight of a WGS84 policy."""
    from guardrail.projection import raw_point_metres
    return raw_point_metres(p, origin)


def zones(policy: dict) -> list[dict]:
    """Keep-out shapes in local metres: polygons and circles, with margins."""
    out = []
    org = policy_origin(policy)
    for c in policy.get("constraints", []):
        t = c.get("type")
        if t in ("polygon_fence", "dynamic_nfz") and c.get("vertices"):
            mo = c.get("motion") if t == "dynamic_nfz" else None
            moving = isinstance(mo, dict) and any(
                float(mo.get(k) or 0.0) for k in ("yaw_rate_dps", "vx_mps", "vy_mps"))
            out.append({"id": c.get("id"), "kind": "poly",
                        "xy": [point_xy(v, org) for v in c["vertices"]],
                        "margin": float(c.get("margin_m") or 0.0),
                        "hard": c.get("constraint_type", "hard") == "hard",
                        "scheduled": bool(c.get("valid_time")),
                        # A dynamic_nfz's declared motion (None: it stands
                        # still once spawned): drawn by moving_ring.
                        "motion": dict(mo) if moving else None,
                        "rule": c if moving else None})
        elif t == "circle_fence" and c.get("center"):
            out.append({"id": c.get("id"), "kind": "circle",
                        "c": point_xy(c["center"], org),
                        "r": float(c.get("radius_m", 0.0)),
                        "margin": float(c.get("margin_m") or 0.0),
                        "hard": c.get("constraint_type", "hard") == "hard",
                        "scheduled": bool(c.get("valid_time"))})
    return out


def load_events(run: Path) -> list[dict]:
    """The run's events.jsonl records (EpisodeRecord.event), [] without one."""
    p = run / "events.jsonl"
    if not p.is_file():
        return []
    out = []
    for line in p.read_text(encoding="utf-8").splitlines():
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if isinstance(e, dict):
            out.append(e)
    return out


def zone_anchors(events: list[dict]) -> dict[int, dict[str, float]]:
    """generation -> {zone id: the time its geometry was last set}: every
    `policy_generation` record whose reason is "hot_apply <id>" (a spawn, a
    move, a translate, a rotate, a scale - each re-anchors the zone where it
    then was, guardrail.shield.Shield._edit_zone) on the rows' clock,
    carried into every later generation until the next such record. A
    record with no time (the take-off's) anchors nothing."""
    gens = sorted((e for e in events if e.get("kind") == "policy_generation"
                   and isinstance(e.get("generation"), int)),
                  key=lambda e: e["generation"])
    out: dict[int, dict[str, float]] = {}
    cur: dict[str, float] = {}
    for e in gens:
        why = str(e.get("reason") or "")
        if why.startswith("hot_apply ") and isinstance(e.get("t"), (int, float)):
            cur = dict(cur, **{why[len("hot_apply "):].strip(): float(e["t"])})
        out[e["generation"]] = dict(cur)
    return out


def zone_anchor(z: dict, gen: int, anchors: dict[int, dict[str, float]],
                gen_start: dict[int, float]) -> tuple[float, str]:
    """(the time zone `z` of generation `gen` started moving from, where
    that time comes from): its last hot-apply (events.jsonl), else the first
    row of the earliest generation that holds it unchanged up to `gen` -
    for a zone the policy declared, the flight's first row, which is where
    the Shield anchors one (its first tick)."""
    a = (anchors.get(gen) or {}).get(z["id"])
    if a is not None:
        return a, "events.jsonl"
    since = z.get("_since_gen", gen)
    return gen_start.get(since, 0.0), f"first row of generation {since}"


def generation_starts(rows: list[dict], gens) -> dict[int, float]:
    """generation -> the time of the first row flown under it or a later one
    (two generations applied on one tick leave rows of the later only)."""
    out = {}
    for g in sorted(gens):
        t = next((r["t"] for r in rows if int(r.get("generation") or 0) >= g), None)
        out[g] = float(rows[0]["t"] if t is None and g == min(gens) else
                       t if t is not None else rows[-1]["t"])
    return out


def zones_by_generation(pols: dict[int, dict]) -> dict[int, list[dict]]:
    """Each generation's zones (zones()), a moving one marked with the
    earliest generation that holds it unchanged (`_since_gen`): the one
    whose start it moves from when no event anchors it."""
    out: dict[int, list[dict]] = {}
    prev: dict[str, dict] = {}
    for g in sorted(pols):
        zs = zones(pols[g])
        for z in zs:
            p = prev.get(z["id"])
            same = (p is not None and p.get("xy") == z.get("xy")
                    and p.get("motion") == z.get("motion"))
            z["_since_gen"] = p["_since_gen"] if same else g
        prev = {z["id"]: z for z in zs}
        out[g] = zs
    return out


def moving_ring(z: dict, age_s: float) -> list[tuple[float, float]]:
    """A moving zone's polygon `age_s` after its anchor, by the model's own
    geometry (guardrail.models.DynamicNFZ.ring_at: about the vertex mean,
    rotated and translated), in the drawing's metres. The zone is built once
    and kept on `z` (`_model`)."""
    m = z.get("_model")
    if m is None:
        from guardrail.models import DynamicNFZ
        r = z.get("rule") or {}
        m = DynamicNFZ.model_validate({
            "id": z["id"], "type": "dynamic_nfz",
            "vertices": [{"x": float(x), "y": float(y)} for x, y in z["xy"]],
            "motion": z["motion"],
            "constraint_type": r.get("constraint_type", "hard"),
            "priority": r.get("priority", "P0"), "scope": r.get("scope", "mission"),
            "violation_action": r.get("violation_action", "project_fix"),
            "margin_m": z.get("margin", 1.0)})
        z["_model"] = m
    return [(float(x), float(y)) for x, y in m.ring_at(max(0.0, float(age_s)))]


def log_label(run: Path) -> str:
    """The flight log the frames replay, by its real path: relative to this
    repository when it is in it, else from the `demo/out` of the checkout
    that holds it, else as given."""
    p = (run / "flight_log.jsonl").resolve()
    try:
        return p.relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        pass
    parts = p.parts
    for i in range(len(parts) - 2, -1, -1):
        if parts[i] == "demo" and parts[i + 1] == "out":
            return "/".join(parts[i:])
    return p.as_posix()


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
    """The first above-ground altitude_envelope's (min, max): the band drawn
    on an `up` axis. An MSL envelope (altitude_ref MSL) is not one: its
    numbers are not heights above home (home_axis_msl_bands)."""
    for c in policy.get("constraints", []):
        if c.get("type") == "altitude_envelope" and c.get("altitude_ref") != "MSL":
            return float(c.get("alt_min_m", 0.0)), float(c.get("alt_max_m", 0.0))
    return None


def flat_ground_msl(run: Path) -> float | None:
    """Home's elevation in MSL for a run flown over a FLAT --ground (the ROS
    2 Shield node's metrics.json ground: {"ground": {"kind": "flat",
    "elevation_msl_m": ...}, "up_ref": "home"}); None for any other run,
    whose `up` has no stated MSL reference."""
    p = run / "metrics.json"
    if not p.is_file():
        return None
    try:
        g = json.loads(p.read_text(encoding="utf-8")).get("ground")
    except (ValueError, AttributeError):
        return None
    if not isinstance(g, dict) or g.get("up_ref") != "home":
        return None
    flat = g.get("ground")
    if not isinstance(flat, dict) or flat.get("kind") != "flat":
        return None
    e = flat.get("elevation_msl_m")
    return float(e) if isinstance(e, (int, float)) and math.isfinite(e) else None


def home_axis_msl_bands(policy: dict, home_msl: float | None) -> list[dict]:
    """Each MSL altitude_envelope as drawn on an `up` (above home) axis:
    {"id", "lo", "hi", "up_lo", "up_hi"}, up_lo / up_hi the band less home's
    elevation, None when `home_msl` is unknown (the band is then named, not
    drawn)."""
    out = []
    for e in altitude_envelopes(policy):
        if e["ref"] != "MSL":
            continue
        k = home_msl is not None
        out.append(dict(e, up_lo=e["lo"] - home_msl if k else None,
                        up_hi=e["hi"] - home_msl if k else None))
    return out


def altitude_envelopes(policy: dict) -> list[dict]:
    """Every altitude_envelope with the height it is measured from:
    {"id", "lo", "hi", "ref"}, ref "MSL" for `altitude_ref: MSL`, else
    "AGL" (guardrail.models.AltitudeEnvelope.altitude_ref: absent means
    above ground, models.AltitudeRef)."""
    out = []
    for c in policy.get("constraints", []):
        if c.get("type") == "altitude_envelope":
            out.append({"id": c.get("id"), "lo": float(c.get("alt_min_m", 0.0)),
                        "hi": float(c.get("alt_max_m", 0.0)),
                        "ref": "MSL" if c.get("altitude_ref") == "MSL" else "AGL"})
    return out


def terrain_referenced(rows: list[dict]) -> bool:
    """True for a run whose rows measure `up` above a ground grid (up_ref
    "terrain": the ROS 2 Shield node with --ground over a DEM,
    terrain.TerrainReference); its altitude panel is drawn in MSL
    (terrain_panel). The node over a flat --ground and the headless harness
    write no up_ref on their rows."""
    return any(r.get("up_ref") == "terrain" for r in rows)


def _num(v) -> float:
    return float(v) if isinstance(v, (int, float)) else float("nan")


def terrain_panel(rows: list[dict], envs: list[dict]) -> dict:
    """The altitude panel of an up_ref "terrain" run, in metres MSL, from
    the rows alone: the aircraft (msl_m), the ground under it (msl_m - up:
    up is the height above the grid), home's elevation (the median of
    msl_m - up_home, None without up_home), each AGL envelope as a band
    that follows the ground (ground + lo .. ground + hi) and each MSL
    envelope as a fixed band. A row missing a value gives NaN there."""
    ts = [float(r["t"]) for r in rows]
    msl = [_num(r.get("msl_m")) for r in rows]
    ground = [m - _num(r.get("up")) for m, r in zip(msl, rows)]
    homes = sorted(m - _num(r.get("up_home")) for m, r in zip(msl, rows)
                   if not math.isnan(m - _num(r.get("up_home"))))
    home = homes[len(homes) // 2] if homes else None
    bands = []
    for e in envs:
        if e["ref"] == "MSL":
            bands.append(dict(e, lo_s=[e["lo"]] * len(ts), hi_s=[e["hi"]] * len(ts)))
        else:
            bands.append(dict(e, lo_s=[g + e["lo"] for g in ground],
                              hi_s=[g + e["hi"] for g in ground]))
    return {"t": ts, "msl": msl, "ground": ground, "home": home, "bands": bands}


def outside_bands(panel: dict, i: int) -> list[str]:
    """Ids of the bands row `i`'s aircraft is outside of (MSL comparison;
    for an AGL band the same as up against its limits). A row with no
    msl_m, or a band with no limits there (an AGL band on a row with no
    `up`: no ground), is outside nothing: a gap, not a breach."""
    m = panel["msl"][i]
    if math.isnan(m):
        return []
    return [b["id"] for b in panel["bands"]
            if not (math.isnan(b["lo_s"][i]) or math.isnan(b["hi_s"][i]))
            and not (b["lo_s"][i] <= m <= b["hi_s"][i])]


def breach_intervals(panel: dict) -> list[tuple[float, float, list[str]]]:
    """The runs of consecutive rows outside at least one band: (first t,
    last t, the band ids outside anywhere in the run), in time order."""
    out: list[tuple[float, float, list[str]]] = []
    cur = None
    for i, t in enumerate(panel["t"]):
        ids = outside_bands(panel, i)
        if ids and cur is not None:
            cur[1] = t
            cur[2].update(ids)
        elif ids:
            cur = [t, t, set(ids)]
        elif cur is not None:
            out.append((cur[0], cur[1], sorted(cur[2])))
            cur = None
    if cur is not None:
        out.append((cur[0], cur[1], sorted(cur[2])))
    return out


def terrain_ylim(panel: dict, margin: float = 3.0) -> tuple[float, float] | None:
    """The panel's y range: the ground less `margin` up to the highest of the
    aircraft, the MSL bands and the AGL floors plus `margin`. An AGL
    ceiling far above (60 m over a ridge) is left off it: on the axis it
    would shrink a sub-metre floor breach under two pixels. None without a
    finite value."""
    fin = lambda s: [v for v in s if not math.isnan(v)]   # noqa: E731
    lows = fin(panel["ground"]) + fin(panel["msl"])
    highs = fin(panel["msl"])
    for b in panel["bands"]:
        if b["ref"] == "MSL":
            lows.append(b["lo"])
            highs.append(b["hi"])
        else:
            highs += fin(b["lo_s"])
    if not lows or not highs:
        return None
    return min(lows) - margin, max(highs) + margin


def altitude_readout(r: dict, panel: dict | None, i: int) -> tuple[float, bool, str]:
    """What frame `i` (row `r`) shows in the altitude panel: the dot's y,
    whether it is outside a band (a terrain run's dot turns red) and the
    text block's position lines. Without a terrain panel the dot is `up`
    and nothing is judged; with one the dot is msl_m."""
    if panel is None:
        return (r["up"], False,
                f"position  x {r['x']:7.2f}  y {r['y']:7.2f}  up {r['up']:6.2f} m\n")
    out_of = outside_bands(panel, i)
    return (panel["msl"][i], bool(out_of),
            f"position  x {r['x']:7.2f}  y {r['y']:7.2f}\n"
            f"height    up {_num(r.get('up')):6.2f} m above grid"
            f"  up_home {_num(r.get('up_home')):6.2f} m\n"
            f"          msl_m {panel['msl'][i]:7.2f}"
            f"  ground {panel['ground'][i]:7.2f} m MSL\n"
            f"outside bands: {', '.join(out_of) or '-'}\n")


# Fill colours: AGL bands (following the ground) blue, MSL bands green.
_BAND_COLOURS = {"AGL": ("#90caf9", "#1565c0"), "MSL": ("#c8e6c9", "#2e7d32")}


def draw_terrain_altitude(az, panel: dict) -> None:
    """Draw `panel` (terrain_panel) on the altitude axes `az`: altitude MSL
    on the y axis (terrain_ylim), each band labelled on the plot by its rule
    id (a legend would sit on the bands' edges), every breach interval
    (breach_intervals) shaded red with the aircraft line red inside it, and
    an AGL ceiling above the axis named at its top."""
    ts, ground = panel["t"], panel["ground"]
    fin = lambda s: [(t, v) for t, v in zip(ts, s) if not math.isnan(v)]   # noqa: E731
    lim = terrain_ylim(panel)
    # Half a tick each side, so a one-tick breach is a visible span.
    dts = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
    half = dts[len(dts) // 2] / 2.0 if dts else 0.05
    for t0, t1, _ in breach_intervals(panel):
        # Over the band fills (zorder 1), under the lines (2): red, not grey.
        az.axvspan(t0 - half, t1 + half, color="#ef9a9a", alpha=0.45, lw=0, zorder=1.5)
    g = fin(ground)
    if g:
        bottom = (lim[0] if lim else min(v for _, v in g) - 3.0)
        az.fill_between(ts, [bottom] * len(ts), ground, color="#d7ccc8", alpha=0.9, lw=0)
        az.plot(ts, ground, color="#6d4c41", lw=1.2)
        tg = max(g, key=lambda p: p[1])[0]
        az.text(tg, bottom + 0.5, "ground (msl_m - up)", fontsize=7, color="#6d4c41",
                ha="center", va="bottom")
    off_axis = []
    for b in panel["bands"]:
        fill, edge = _BAND_COLOURS[b["ref"]]
        if b["ref"] == "MSL":
            az.axhspan(b["lo"], b["hi"], color=fill, alpha=0.6)
            az.text(ts[-1], b["hi"], f"{b['id']}: MSL {b['lo']:g}-{b['hi']:g} m ",
                    fontsize=7, color=edge, ha="right", va="top")
            continue
        az.fill_between(ts, b["lo_s"], b["hi_s"], color=fill, alpha=0.35, lw=0)
        az.plot(ts, b["lo_s"], "--", color=edge, lw=0.9)
        az.plot(ts, b["hi_s"], "--", color=edge, lw=0.9)
        lo_f, hi_f = fin(b["lo_s"]), fin(b["hi_s"])
        name = f"{b['id']}: AGL {b['lo']:g}-{b['hi']:g} m"
        if lim and hi_f and min(v for _, v in hi_f) > lim[1]:
            off_axis.append(f"{b['id']} ceiling {min(v for _, v in hi_f):.1f}-"
                            f"{max(v for _, v in hi_f):.1f} m MSL above the axis")
            if lo_f:
                # Under the floor where it is highest (the ridge's plateau):
                # the widest gap between the floor and the ground below it.
                top = max(v for _, v in lo_f)
                near = [t for t, v in lo_f if v >= top - 1.0]
                az.text(near[len(near) // 2], top, name, fontsize=7, color=edge,
                        ha="center", va="top")
        elif hi_f:
            az.text(hi_f[0][0], hi_f[0][1], f" {name}", fontsize=7, color=edge,
                    ha="left", va="top")
    if panel["home"] is not None:
        az.axhline(panel["home"], color="#455a64", ls=":", lw=1.0)
        az.text(ts[-1], panel["home"], f"home {panel['home']:.1f} m ", fontsize=7,
                color="#455a64", ha="right", va="top")
    az.plot(ts, panel["msl"], color="#546e7a", lw=1)
    # The aircraft line red where it is outside a band (NaN elsewhere).
    flags = [bool(outside_bands(panel, i)) for i in range(len(ts))]
    if any(flags):
        az.plot(ts, [m if f else float("nan") for m, f in zip(panel["msl"], flags)],
                color="#c62828", lw=2.0)
    a = fin(panel["msl"])
    if a:
        # At the end of the trace, clear of the ridge where a breach is.
        az.text(a[-1][0], a[-1][1], "aircraft msl_m ", fontsize=7, color="#546e7a",
                ha="right", va="bottom")
    note = ["red: outside a band"] if any(flags) else []
    if off_axis or note:
        az.text(0.01, 0.98, "\n".join(note + off_axis), transform=az.transAxes, fontsize=7,
                color="#37474f", ha="left", va="top")
    if lim:
        az.set_ylim(*lim)
    az.set_ylabel("altitude MSL (m)")


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
    """The run's video at `out` (an .mp4 through ffmpeg)."""
    return _render(run, out, fps, step, None)


def render_stills(run: Path, out_dir: Path, times: list[float], step: int = 1,
                  probe: list | None = None) -> list[Path]:
    """The frames nearest each of `times` (mission seconds) as PNGs in
    `out_dir`, drawn exactly as the video draws them; no ffmpeg needed.
    With `probe` (a list), each still also appends what it drew in the
    altitude panel and the text block: {"t", "alt_dot" (x, y),
    "alt_dot_colour", "alt_ylabel", "alt_texts", "body"}."""
    return _render(run, out_dir, 10, step, [float(t) for t in times], probe)


def _render(run: Path, out: Path, fps: int, step: int, stills: list[float] | None,
            probe: list | None = None):
    # The run's own inputs are judged before the drawing stack is imported: a
    # run with nothing to draw is refused the same way on a host without
    # matplotlib (found by the Python 3.12 CI env, 2026-11-06).
    rows = load_rows(run)
    pols = load_policies(run)
    if not pols:
        raise SystemExit(f"no policy_g*.json in {run}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Circle, Polygon
    g0 = pols[min(pols)]
    band = altitude_band(g0)
    target = mission_target(run)
    # Each generation's zones, a moving one with where its motion starts
    # (zone_anchor: events.jsonl, else its generation's first row).
    zbg = zones_by_generation(pols)
    anchors = zone_anchors(load_events(run))
    gstart = generation_starts(rows, pols)
    allz = [z for zs in zbg.values() for z in zs]
    # A moving zone's extent is everywhere it went while its generation flew.
    swept = [{"kind": "poly", "xy": moving_ring(z, float(r["t"]) - zone_anchor(
                 z, g, anchors, gstart)[0])}
             for g, zs in zbg.items() for z in zs if z.get("motion")
             for r in rows if int(r.get("generation") or 0) == g]
    allc = [{"id": c["id"], "kind": "poly", "xy": corridor_outline(c)}
            for p in pols.values() for c in corridors(p) if len(c["centerline"]) >= 2]
    wps = mission_waypoints(run)
    x0, x1, y0, y1 = bounds(rows, allz + swept + allc, target)
    has_fsm = any(r.get("fsm_state_after") for r in rows)

    if stills is None:
        from tools.encode_deliverable import find_ffmpeg
        plt.rcParams["animation.ffmpeg_path"] = find_ffmpeg()
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.04, 0.08, 0.52, 0.84])
    az = fig.add_axes([0.62, 0.55, 0.35, 0.37])
    info = fig.add_axes([0.62, 0.06, 0.35, 0.42])
    info.axis("off")

    ts = [r["t"] for r in rows]
    # A run whose `up` is above a ground grid (up_ref "terrain") is drawn in
    # MSL: up, up_home and msl_m are three different heights there, and the
    # MSL bands and the AGL bands share no axis but MSL.
    panel = terrain_panel(rows, altitude_envelopes(g0)) if terrain_referenced(rows) else None
    if panel is not None:
        draw_terrain_altitude(az, panel)
        alt_dot, = az.plot([], [], "o", color="#1565c0", zorder=5)
    else:
        ups = [r["up"] for r in rows]
        az.plot(ts, ups, color="#90a4ae", lw=1)
        if band:
            az.axhspan(band[0], band[1], color="#c8e6c9", alpha=0.6, label="altitude band")
        # An MSL envelope on this above-home axis: offset by home's MSL
        # elevation where the run states it (a flat --ground), else named
        # as not drawn. A policy with none draws nothing here.
        msl_bands = home_axis_msl_bands(g0, flat_ground_msl(run))
        for k, b in enumerate(msl_bands):
            if b["up_lo"] is not None:
                az.axhspan(b["up_lo"], b["up_hi"], color="#a5d6a7", alpha=0.5)
                az.text(ts[-1], b["up_hi"], f"{b['id']}: MSL {b['lo']:g}-{b['hi']:g} m"
                        f" (home {b['lo'] - b['up_lo']:g} m MSL) ", fontsize=7,
                        color="#2e7d32", ha="right", va="top")
            else:
                az.text(0.01, 0.98 - 0.07 * k, f"{b['id']}: MSL {b['lo']:g}-{b['hi']:g} m"
                        " not drawn (no MSL reference for home)", transform=az.transAxes,
                        fontsize=7, color="#2e7d32", ha="left", va="top")
        alt_dot, = az.plot([], [], "o", color="#1565c0")
    az.set_xlabel("t (s)")
    if panel is None:
        az.set_ylabel("altitude up (m)")
    az.set_title("altitude")

    zone_patches: dict[int, list] = {}
    # generation -> [(zone, its polygon, its label)] of the zones that move
    moving_patches: dict[int, list] = {}

    def place_moving(gen: int, t: float) -> list[str]:
        """Each moving zone of `gen` where its motion has taken it at `t`
        (DynamicNFZ.ring_at of its age); one info line per zone."""
        lines = []
        for z, poly, txt in moving_patches.get(gen, []):
            a, src = zone_anchor(z, gen, anchors, gstart)
            ring = moving_ring(z, t - a)
            poly.set_xy(ring)
            txt.set_position(ring[0])
            mo = z["motion"]
            lines.append(f"moving {z['id']}: {float(mo.get('yaw_rate_dps') or 0):g} deg/s, "
                         f"v ({float(mo.get('vx_mps') or 0):g}, {float(mo.get('vy_mps') or 0):g})"
                         f" m/s, age {max(0.0, t - a):.1f} s ({src})")
        return lines

    def draw_zones(gen: int, t: float) -> list[str]:
        for g, patches in zone_patches.items():
            for p in patches:
                p.set_visible(g == gen)
        if gen in zone_patches:
            return place_moving(gen, t)
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
        for z in zbg.get(gen) or zones(pol):
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
                poly = patches[-1]
            else:
                patches.append(ax.add_patch(Circle(z["c"], z["r"], **kw)))
                if z["margin"]:
                    patches.append(ax.add_patch(Circle(z["c"], z["r"] + z["margin"], fill=False,
                                                       ec=col, ls="--", lw=0.8)))
            patches.append(ax.text(*(z["xy"][0] if z["kind"] == "poly" else z["c"]),
                                   z["id"] + (" (off)" if is_off else "")
                                   + (" (valid_time schedule)" if sched else "")
                                   + (" (moving)" if z.get("motion") else ""),
                                   fontsize=8, color=col))
            if z["kind"] == "poly" and z.get("motion"):
                moving_patches.setdefault(gen, []).append((z, poly, patches[-1]))
        zone_patches[gen] = patches
        return place_moving(gen, t)

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
    fig.text(0.04, 0.015, "Replay of the flight log (%s), not a camera "
             "recording." % log_label(run), fontsize=9, color="#616161")
    body = info.text(0.0, 1.0, "", va="top", family="monospace", fontsize=9)

    idx = list(range(0, len(rows), max(1, step)))
    if idx[-1] != len(rows) - 1:
        idx.append(len(rows) - 1)

    def frame(k):
        i = idx[k]
        r = rows[i]
        moving = draw_zones(int(r.get("generation") or 0), float(r["t"]))
        track.set_data([q["x"] for q in rows[: i + 1]], [q["y"] for q in rows[: i + 1]])
        dot.set_data([r["x"]], [r["y"]])
        alt_y, outside, position = altitude_readout(r, panel, i)
        alt_dot.set_data([r["t"]], [alt_y])
        if panel is not None:
            alt_dot.set_color("#c62828" if outside else "#1565c0")
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
            + position +
            f"pilot     |v_h| {hz:5.2f} m/s  vz {raw.get('vz_up', 0.0):5.2f}\n"
            f"emitted   |v_h| {ez:5.2f} m/s  vz {emi.get('vz_up', 0.0):5.2f}\n"
            f"flown     {r.get('flown')}   setpoint {r.get('setpoint')}\n"
            f"forecast violations: {viol[:60]}\n"
            f"repairs: {reps[:60]}\n"
            f"last FSM edge: {edge or '-'}\n"
            f"autopilot mode: {r.get('mode') or r.get('autopilot_set_mode') or '-'}"
            + (f"\nnext waypoint {r.get('next_wp_index')}: planned {r.get('next_wp_planned')}\n"
               f"  autopilot holds {nwp}  replaced {r.get('wp_replaced')}"
               if r.get("next_wp_planned") is not None else "")
            + "".join(f"\n{m[:64]}" for m in moving))
        return track, dot, alt_dot, raw_q, emi_q, title, body, wp_hold, wp_line

    if stills is not None:
        # The frames nearest the times asked for, drawn by the same frame().
        out.mkdir(parents=True, exist_ok=True)
        written = []
        for t in stills:
            k = min(range(len(idx)), key=lambda j: abs(float(rows[idx[j]]["t"]) - t))
            frame(k)
            p = out / f"{run.name}_t{float(rows[idx[k]]['t']):07.2f}.png"
            fig.savefig(p, dpi=100)
            written.append(p)
            if probe is not None:
                xd, yd = alt_dot.get_data()
                from matplotlib.colors import to_hex
                probe.append({"t": float(rows[idx[k]]["t"]),
                              "alt_dot": (float(xd[0]), float(yd[0])),
                              "alt_dot_colour": to_hex(alt_dot.get_color()),
                              "alt_ylabel": az.get_ylabel(),
                              "alt_texts": [x.get_text() for x in az.texts],
                              "body": body.get_text()})
        plt.close(fig)
        return written

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
    ap.add_argument("--tag", required=True,
                    help="run folder under demo/out/ (a nested one too: hil_smoke/<run>)")
    ap.add_argument("--demo-out", type=Path, default=ROOT / "demo" / "out")
    ap.add_argument("--fps", type=int, default=10,
                    help="frames per second; 10 with --step 1 is real time at the 10 Hz monitor")
    ap.add_argument("--step", type=int, default=1, help="draw every Nth tick")
    ap.add_argument("--out", type=Path, default=None,
                    help="the video (default <run>/<run>_track.mp4); with --stills, the "
                         "folder for the PNGs (default <run>/stills)")
    ap.add_argument("--stills", default=None, metavar="T1,T2,...",
                    help="draw only the frames nearest these mission times (s) as PNGs")
    a = ap.parse_args(argv)
    run = a.demo_out / a.tag
    if not run.is_dir():
        print(f"no run folder {run}", file=sys.stderr)
        return 2
    if a.stills:
        times = [float(x) for x in a.stills.split(",") if x.strip()]
        for p in render_stills(run, a.out or run / "stills", times, step=a.step):
            print(f"[still] {p}")
        return 0
    out = a.out or run / f"{run.name}_track.mp4"
    p = render(run, out, fps=a.fps, step=a.step)
    print(f"[video] {p} ({p.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
