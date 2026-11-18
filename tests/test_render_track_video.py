"""tools/render_track_video.py: a flight-log replay video for rails with no camera.

Run either way:
    pytest tests/test_render_track_video.py -v
    python tests/test_render_track_video.py

What these guard against: a video that does not say it is a log replay (it
could be taken for a camera recording), a run folder with no policy drawn as
an empty map, and a failed writer leaving nothing while reporting success.

Since 2026-11-11: a moving zone (a dynamic_nfz's `motion`) drawn where it is
at each frame's time by DynamicNFZ.ring_at, from its last hot-apply in
events.jsonl (else its generation's first row), not at its authored
vertices; the footer naming the log by its real path, not
demo/out/<tag>/flight_log.jsonl; and --stills drawing chosen frames as PNGs.
Shown failing first: against 0df7ee8 zones() carried no motion, and
zone_anchors, moving_ring, log_label and render_stills did not exist.

Since 2026-11-18: an up_ref "terrain" run's altitude panel drawn in MSL
(aircraft msl_m, ground msl_m - up, home, AGL bands following the ground,
MSL bands fixed, labelled by rule id), and a run without up_ref terrain not
taking that path. Shown failing first: against 5966c5c the four terrain
tests stopped with AttributeError (no altitude_envelopes, terrain_panel,
draw_terrain_altitude).

After that change's review (2026-11-18): a row with msl_m but no `up` is a
gap, not an AGL breach; breach intervals shaded on the time axis with the AGL
ceiling off the y axis; each frame's dot and text block read back from the
rendered still (render_stills probe); and an MSL envelope on a run without
up_ref terrain offset by a flat ground's elevation or named as not drawn.
Shown failing first: against 456393f, 14/19 passed (no breach_intervals, no
probe, NaN limits read as outside, ylim 92.9-183.6 with the ceiling on it, the
MSL band drawn as the up-axis band).
"""
import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import render_track_video as R                     # noqa: E402

SKIP = "SKIP"

POLICY = {"constraints": [
    {"id": "nfz", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard", "priority": "P0",
     "margin_m": 1.0, "vertices": [{"x": 5, "y": 5}, {"x": 9, "y": 5}, {"x": 9, "y": 9},
                                   {"x": 5, "y": 9}]},
    {"id": "ring", "type": "circle_fence", "scope": "mission", "constraint_type": "soft", "priority": "P1",
     "center": {"x": -4, "y": 2}, "radius_m": 2.0, "margin_m": 0.5},
    {"id": "band", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20}]}


def _row(i, state_before="Normal", state_after="Normal", edge=None):
    return {"t": i * 0.1, "tick": i + 1, "x": 0.5 * i, "y": 0.2 * i, "up": 15.0,
            "raw": {"vx": 4.0, "vy": 4.0, "vz_up": 0.0}, "emitted": {"vx": 2.0, "vy": -1.0, "vz_up": 0.0},
            "violations": [{"rule_id": "nfz"}], "repairs": [{"operator": "GeofenceSlide", "magnitude_m": 3.14159}],
            "flown": True, "setpoint": "velocity", "generation": 0,
            "fsm_state_before": state_before, "fsm_state_after": state_after, "fsm_edge": edge}


def _run(d: Path, rows, policy=POLICY) -> Path:
    run = d / "demo" / "out" / "fake_run"
    run.mkdir(parents=True)
    (run / "flight_log.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n",
                                          encoding="utf-8")
    if policy is not None:
        (run / "policy_g0.json").write_text(json.dumps(policy), encoding="utf-8")
    (run / "prompt.yaml").write_text("mission:\n  target:\n    x: 12\n    y: 12\n", encoding="utf-8")
    return run


def test_zones_read_polygons_and_circles_with_margins_and_hardness():
    z = R.zones(POLICY)
    assert [(q["id"], q["kind"], q["hard"]) for q in z] == [("nfz", "poly", True),
                                                            ("ring", "circle", False)]
    assert z[0]["margin"] == 1.0 and z[1]["r"] == 2.0
    assert R.altitude_band(POLICY) == (10.0, 20.0)


def test_a_wgs84_policy_dump_is_drawn_at_the_shields_metres():
    """Since IR schema 3.1.0 a policy dump carries a WGS84 point as lat/lon
    only; the drawing must place it where the Shield does (its frame origin,
    projection.local_metres), and keep a metre zone hot-applied into it as
    written. Shown failing before 2026-11-04: KeyError: 'x' on the first
    vertex, as on every ROS 2 rail flight of 4 Nov."""
    from guardrail.projection import LocalProjection, local_metres
    org = (-35.363261, 149.16523)
    proj = LocalProjection(*org)
    ll = lambda x, y: dict(zip(("lat", "lon"), proj.to_latlon(x, y)))   # noqa: E731
    pol = {"origin": {"lat": org[0], "lon": org[1]}, "constraints": [
        {"id": "nfz-square", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "margin_m": 1.0, "vertices": [ll(16, 14), ll(32, 14), ll(32, 30), ll(16, 30)]},
        {"id": "ring", "type": "circle_fence", "center": ll(-4, 2), "radius_m": 2.0},
        {"id": "nfz-hot-1", "type": "dynamic_nfz", "scope": "mission", "constraint_type": "hard",
         "vertices": [{"x": 24, "y": -6}, {"x": 30, "y": -6}, {"x": 30, "y": 6}]},
        {"id": "route", "type": "corridor", "width_m": 40,
         "centerline": [ll(0, 0), ll(60, 0)]}]}
    z = {q["id"]: q for q in R.zones(pol)}
    want = [local_metres(org, v["lat"], v["lon"]) for v in pol["constraints"][0]["vertices"]]
    assert z["nfz-square"]["xy"] == want
    assert all(abs(a - b) < 1e-5 for p, q in zip(want, [(16, 14), (32, 14), (32, 30), (16, 30)])
               for a, b in zip(p, q)), want
    assert all(abs(a - b) < 1e-5 for a, b in zip(z["ring"]["c"], (-4, 2)))
    assert z["nfz-hot-1"]["xy"] == [(24.0, -6.0), (30.0, -6.0), (30.0, 6.0)]
    (cor,) = R.corridors(pol)
    assert all(abs(a - b) < 1e-5 for p, q in zip(cor["centerline"], [(0, 0), (60, 0)])
               for a, b in zip(p, q)), cor["centerline"]
    # A policy with no stated origin derives it as the model does.
    del pol["origin"]
    assert R.policy_origin(pol) is not None


def test_the_fsm_state_and_last_edge_follow_the_log():
    rows = [_row(0), _row(1, "Normal", "Brake", "X1"), _row(2, "Brake", "Brake"),
            _row(3, "Brake", "Loiter", "G3")]
    assert R.state_at(rows, 0) == ("Normal", None)
    s, e = R.state_at(rows, 2)
    assert s == "Brake" and "[X1]" in e
    s, e = R.state_at(rows, 3)
    assert s == "Loiter" and "[G3]" in e


def test_a_run_without_a_policy_is_refused_not_drawn_empty():
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_row(0)], policy=None)
        try:
            R.render(run, run / "x.mp4")
        except SystemExit as e:
            assert "policy" in str(e)
        else:
            raise AssertionError("rendered a run with no policy")


def test_the_mission_target_is_read_from_the_prompt():
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_row(0)])
        assert R.mission_target(run) == (12.0, 12.0)


# A 40 m bar along y, spawned at t = 3 s and set spinning at 6 deg/s (the
# hil rail's dyn-nfz-rotate-spin): generation 1 the spawn, 2 the spin.
BAR = [{"x": -170.0, "y": 20.0}, {"x": -170.0, "y": -20.0}, {"x": -166.0, "y": -20.0},
       {"x": -166.0, "y": 20.0}]


def _bar(motion=None):
    c = {"id": "bar", "type": "dynamic_nfz", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "project_fix", "margin_m": 1.0,
         "vertices": BAR}
    if motion is not None:
        c["motion"] = motion
    return c


SPIN = {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 6.0}
EVENTS = [{"kind": "policy_generation", "t": None, "generation": 0, "reason": "take-off"},
          {"kind": "policy_generation", "t": 3.0, "generation": 1, "reason": "hot_apply bar"},
          {"kind": "policy_generation", "t": 3.0, "generation": 2, "reason": "hot_apply bar"},
          {"kind": "policy_generation", "t": 9.0, "generation": 3,
           "reason": "hot_apply other-zone"}]


def test_a_moving_zone_carries_its_motion_and_a_still_one_does_not():
    z = R.zones({"constraints": [_bar(SPIN), dict(_bar(), id="still"),
                                 dict(_bar({"vx_mps": 0, "vy_mps": 0, "yaw_rate_dps": 0}),
                                      id="zero")]})
    assert z[0]["motion"] == SPIN and z[0]["rule"]["id"] == "bar"
    assert z[1]["motion"] is None and z[2]["motion"] is None, "no motion, or a zero one"
    assert all(q["motion"] is None for q in R.zones(POLICY) if q["kind"] == "poly")


def test_a_zone_is_anchored_at_its_last_hot_apply_and_carried_forward():
    a = R.zone_anchors(EVENTS)
    assert a == {0: {}, 1: {"bar": 3.0}, 2: {"bar": 3.0},
                 3: {"bar": 3.0, "other-zone": 9.0}}, a
    # A later edit of the zone re-anchors it.
    a = R.zone_anchors(EVENTS + [{"kind": "policy_generation", "t": 12.5, "generation": 4,
                                  "reason": "hot_apply bar"}])
    assert a[4]["bar"] == 12.5 and a[3]["bar"] == 3.0
    assert R.zone_anchors([]) == {}


def test_the_drawn_ring_is_the_models_ring_at_the_zones_age():
    from guardrail.models import DynamicNFZ
    (z,) = R.zones({"constraints": [_bar(SPIN)]})
    # 15 s at 6 deg/s is a quarter turn about the vertex mean (-168, 0): the
    # bar along y now lies along x.
    ring = R.moving_ring(z, 15.0)
    xs, ys = [p[0] for p in ring], [p[1] for p in ring]
    assert abs((max(xs) - min(xs)) - 40.0) < 1e-6 and abs((max(ys) - min(ys)) - 4.0) < 1e-6
    assert abs(sum(xs) / 4 + 168.0) < 1e-6 and abs(sum(ys) / 4) < 1e-6
    want = DynamicNFZ.model_validate(_bar(SPIN)).ring_at(15.0)
    assert ring == [(float(x), float(y)) for x, y in want]
    assert R.moving_ring(z, -2.0) == R.moving_ring(z, 0.0), "no age before the anchor"
    (t,) = R.zones({"constraints": [_bar({"vx_mps": 2.0, "vy_mps": -1.0, "yaw_rate_dps": 0.0})]})
    moved = R.moving_ring(t, 5.0)
    assert all(abs(mx - (x + 10.0)) < 1e-9 and abs(my - (y - 5.0)) < 1e-9
               for (mx, my), (x, y) in zip(moved, R.moving_ring(t, 0.0)))


def test_without_events_a_zone_moves_from_its_generations_first_row():
    pols = {0: {"constraints": [_bar(SPIN)]}, 1: {"constraints": [_bar(SPIN)]},
            2: {"constraints": [dict(_bar(SPIN), vertices=BAR[1:] + BAR[:1])]}}
    zbg = R.zones_by_generation(pols)
    assert [zbg[g][0]["_since_gen"] for g in (0, 1, 2)] == [0, 0, 2]
    rows = [{"t": 0.1}, {"t": 0.2, "generation": 1}, {"t": 4.0, "generation": 2}]
    gs = R.generation_starts(rows, pols)
    assert gs == {0: 0.1, 1: 0.2, 2: 4.0}, gs
    # A zone the policy declared moves from the flight's first row (the
    # Shield anchors one at its first tick), an edited one from its edit.
    assert R.zone_anchor(zbg[1][0], 1, {}, gs) == (0.1, "first row of generation 0")
    assert R.zone_anchor(zbg[2][0], 2, {}, gs) == (4.0, "first row of generation 2")
    assert R.zone_anchor(zbg[2][0], 2, {2: {"bar": 3.9}}, gs) == (3.9, "events.jsonl")


def test_the_footer_names_the_log_by_its_real_path():
    assert R.log_label(ROOT / "demo" / "out" / "x") == "demo/out/x/flight_log.jsonl"
    with tempfile.TemporaryDirectory() as d:
        other = Path(d) / "lab" / "demo" / "out" / "hil_smoke" / "hs06b_004"
        assert R.log_label(other) == "demo/out/hil_smoke/hs06b_004/flight_log.jsonl"
        loose = Path(d) / "runs" / "r1"
        assert R.log_label(loose) == (loose / "flight_log.jsonl").resolve().as_posix()
    src = (ROOT / "tools" / "render_track_video.py").read_text(encoding="utf-8")
    assert "demo/out/%s/flight_log.jsonl" not in src


def test_stills_draw_the_spinning_bar_where_it_is_at_each_time():
    try:
        import matplotlib                                      # noqa: F401
    except ImportError as e:
        print(f"      SKIP: no matplotlib ({e})")
        return SKIP
    with tempfile.TemporaryDirectory() as d:
        rows = [dict(_row(i), generation=0 if i < 30 else 2, x=-200.0 + 0.4 * i, y=0.0)
                for i in range(200)]
        run = _run(Path(d), rows, policy={"constraints": [POLICY["constraints"][2]]})
        (run / "policy_g1.json").write_text(json.dumps({"constraints": [_bar()]}),
                                            encoding="utf-8")
        (run / "policy_g2.json").write_text(json.dumps({"constraints": [_bar(SPIN)]}),
                                            encoding="utf-8")
        (run / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in EVENTS),
                                          encoding="utf-8")
        out = Path(d) / "stills"
        got = R.render_stills(run, out, [5.0, 18.0])
        assert [p.name for p in got] == ["fake_run_t0005.00.png", "fake_run_t0018.00.png"]
        assert all(p.is_file() and p.stat().st_size > 0 for p in got)
        assert got[0].read_bytes() != got[1].read_bytes()
        rc = R.main(["--tag", "fake_run", "--demo-out", str(run.parent), "--stills", "6",
                     "--out", str(out / "cli")])
        assert rc == 0 and (out / "cli" / "fake_run_t0006.00.png").is_file()


# An up_ref "terrain" run (2026-11-18, msl_dem_sitl_01): home at 100 m MSL,
# ground rising east at 0.5 m per metre, the aircraft holding 115 m MSL; up
# is the height above the grid, up_home the height above home.
TERRAIN_POLICY = {"constraints": [
    {"id": "msl-ceiling", "type": "altitude_envelope", "alt_min_m": 105, "alt_max_m": 130,
     "altitude_ref": "MSL", "constraint_type": "hard", "priority": "P0"},
    {"id": "agl-band", "type": "altitude_envelope", "alt_min_m": 8, "alt_max_m": 60,
     "constraint_type": "hard", "priority": "P0"}]}


def _terrain_row(i):
    ground, msl = 100.0 + 0.5 * i, 115.0
    return dict(_row(i), x=0.0, y=float(i), up=msl - ground, up_home=msl - 100.0, msl_m=msl,
                up_ref="terrain")


def test_altitude_envelopes_carry_their_reference_absent_is_agl():
    envs = R.altitude_envelopes(TERRAIN_POLICY)
    assert envs == [{"id": "msl-ceiling", "lo": 105.0, "hi": 130.0, "ref": "MSL"},
                    {"id": "agl-band", "lo": 8.0, "hi": 60.0, "ref": "AGL"}], envs
    assert R.altitude_envelopes(POLICY) == [{"id": "band", "lo": 10.0, "hi": 20.0, "ref": "AGL"}]
    assert R.terrain_referenced([_terrain_row(0)])
    assert not R.terrain_referenced([_row(0)]), "a row with no up_ref is not terrain"
    assert not R.terrain_referenced([dict(_row(0), up_ref="home")])


def test_the_terrain_panel_is_in_msl_with_agl_bands_following_the_ground():
    """Shown failing first: against 5966c5c the panel drew `up` (height
    above the grid) on an axis that also held the MSL band, and none of
    altitude_envelopes / terrain_panel / outside_bands existed."""
    rows = [_terrain_row(i) for i in range(40)]
    p = R.terrain_panel(rows, R.altitude_envelopes(TERRAIN_POLICY))
    assert p["msl"] == [115.0] * 40
    assert all(abs(g - (100.0 + 0.5 * i)) < 1e-9 for i, g in enumerate(p["ground"]))
    assert p["home"] == 100.0
    msl_b, agl_b = p["bands"]
    assert msl_b["id"] == "msl-ceiling" and set(msl_b["lo_s"]) == {105.0} \
        and set(msl_b["hi_s"]) == {130.0}, "an MSL band is fixed"
    assert all(abs(lo - (108.0 + 0.5 * i)) < 1e-9 and abs(hi - (160.0 + 0.5 * i)) < 1e-9
               for i, (lo, hi) in enumerate(zip(agl_b["lo_s"], agl_b["hi_s"]))), \
        "an AGL band is ground + min .. ground + max"
    # up = 15 - 0.5 i: under the 8 m floor from i = 15 (7.5 m above the grid).
    assert R.outside_bands(p, 14) == [] and R.outside_bands(p, 15) == ["agl-band"]
    # A row with no msl_m is a gap, not a zero, and is outside nothing.
    gap = R.terrain_panel([dict(_terrain_row(0), msl_m=None)], R.altitude_envelopes(TERRAIN_POLICY))
    assert math.isnan(gap["msl"][0]) and math.isnan(gap["ground"][0]) and gap["home"] is None
    assert R.outside_bands(gap, 0) == []
    # A row with msl_m but no `up` has no ground under it: its AGL band has
    # no limits there and it is outside nothing, not outside the AGL band
    # (review 2026-11-18: NaN limits read as a breach).
    noup = R.terrain_panel([dict(_terrain_row(0), up=None)], R.altitude_envelopes(TERRAIN_POLICY))
    assert noup["msl"] == [115.0] and math.isnan(noup["ground"][0])
    assert R.outside_bands(noup, 0) == []
    # ... while its MSL band is still judged.
    high = R.terrain_panel([dict(_terrain_row(0), up=None, msl_m=140.0)],
                           R.altitude_envelopes(TERRAIN_POLICY))
    assert R.outside_bands(high, 0) == ["msl-ceiling"]


def test_breach_intervals_are_the_runs_of_rows_outside_a_band():
    rows = [_terrain_row(i) for i in range(40)]
    p = R.terrain_panel(rows, R.altitude_envelopes(TERRAIN_POLICY))
    (iv,) = R.breach_intervals(p)
    assert abs(iv[0] - 1.5) < 1e-9 and abs(iv[1] - 3.9) < 1e-9 and iv[2] == ["agl-band"], iv
    # Two separate runs, the second outside both bands.
    rows = [dict(_terrain_row(0), t=0.0, up=5.0), dict(_terrain_row(0), t=0.1),
            dict(_terrain_row(0), t=0.2, up=5.0, msl_m=140.0),
            dict(_terrain_row(0), t=0.3, up=5.0)]
    p = R.terrain_panel(rows, R.altitude_envelopes(TERRAIN_POLICY))
    assert R.breach_intervals(p) == [(0.0, 0.0, ["agl-band"]),
                                     (0.2, 0.3, ["agl-band", "msl-ceiling"])]
    assert R.breach_intervals(R.terrain_panel([_terrain_row(0)],
                                              R.altitude_envelopes(TERRAIN_POLICY))) == []


def test_the_terrain_panel_draws_msl_axis_and_labels_bands_by_rule_id():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"      SKIP: no matplotlib ({e})")
        return SKIP
    rows = [_terrain_row(i) for i in range(40)]
    fig, az = plt.subplots()
    R.draw_terrain_altitude(az, R.terrain_panel(rows, R.altitude_envelopes(TERRAIN_POLICY)))
    assert az.get_ylabel() == "altitude MSL (m)"
    texts = [t.get_text() for t in az.texts]
    assert any("msl-ceiling: MSL 105-130 m" in t for t in texts), texts
    assert any("agl-band: AGL 8-60 m" in t for t in texts), texts
    assert any("home 100.0 m" in t for t in texts), texts
    lo, hi = az.get_ylim()
    # The ground (100..119.5 m), the aircraft (115), the MSL band (to 130)
    # and the AGL floor (108..127.5) are on the axis; the AGL ceiling
    # (160..179.5) is not, and is named instead (review 2026-11-18: with it
    # on the axis a 0.49 m floor breach was under two pixels).
    assert lo <= 100.0 and 130.0 <= hi < 160.0, (lo, hi)
    assert any("agl-band ceiling 160.0-179.5 m MSL above the axis" in t for t in texts), texts
    # The aircraft line is msl_m, not up.
    assert any(list(ln.get_ydata()) == [115.0] * 40 for ln in az.lines)
    # The breach (rows 15..39, t 1.5..3.9 s) is shaded on the time axis and
    # the aircraft line drawn red there, so every frame shows it.
    spans = [p for p in az.patches if p.get_facecolor()[:3] == matplotlib.colors.to_rgb("#ef9a9a")]
    assert len(spans) == 1, az.patches
    s = spans[0]
    # axvspan is a Rectangle from matplotlib 3.9, a Polygon before it.
    xs = ([s.get_x(), s.get_x() + s.get_width()] if hasattr(s, "get_width")
          else [v[0] for v in s.get_xy()])
    assert min(xs) < 1.5 < 3.9 < max(xs), xs
    red = [ln for ln in az.lines if ln.get_color() == "#c62828"]
    assert len(red) == 1
    ys = list(red[0].get_ydata())
    assert all(math.isnan(v) for v in ys[:15]) and ys[15:] == [115.0] * 25, ys
    assert any("red: outside a band" in t for t in texts), texts
    # The aircraft label sits at the end of the trace, not over the ridge.
    (lab,) = [t for t in az.texts if "aircraft msl_m" in t.get_text()]
    assert abs(lab.get_position()[0] - 3.9) < 1e-9, lab.get_position()
    plt.close(fig)


def test_only_a_terrain_run_takes_the_terrain_panel_and_both_render():
    try:
        import matplotlib                                      # noqa: F401
    except ImportError as e:
        print(f"      SKIP: no matplotlib ({e})")
        return SKIP
    calls = []
    orig = R.draw_terrain_altitude
    R.draw_terrain_altitude = lambda az, panel: (calls.append(panel), orig(az, panel))
    try:
        with tempfile.TemporaryDirectory() as d:
            run = _run(Path(d), [_row(i) for i in range(20)])
            assert R.render_stills(run, Path(d) / "plain", [1.0])[0].is_file()
            assert calls == [], "a run without up_ref terrain draws as before"
        with tempfile.TemporaryDirectory() as d:
            run = _run(Path(d), [_terrain_row(i) for i in range(40)], policy=TERRAIN_POLICY)
            (p,) = R.render_stills(run, Path(d) / "terrain", [2.0])
            assert p.is_file() and p.stat().st_size > 0
            assert len(calls) == 1 and calls[0]["home"] == 100.0
    finally:
        R.draw_terrain_altitude = orig


def test_each_frame_plots_and_prints_the_rows_own_heights():
    """What a frame draws, read back from the rendered still (render_stills
    probe): on a terrain run the dot at msl_m, red outside a band, and the
    text block's up / up_home / msl_m / ground and outside-bands lines; on
    any other run the dot at `up` on the "altitude up" axis. Review
    2026-11-18: only the PNG's existence was checked, so a dot plotted at
    `up` or swapped text values would have passed."""
    try:
        import matplotlib                                      # noqa: F401
    except ImportError as e:
        print(f"      SKIP: no matplotlib ({e})")
        return SKIP
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_terrain_row(i) for i in range(40)], policy=TERRAIN_POLICY)
        got = []
        R.render_stills(run, Path(d) / "terrain", [1.0, 2.0], probe=got)
        inside, outside = got
        # Row 10: ground 105, up 10 above it, 15 above home, msl 115.
        assert inside["alt_dot"] == (1.0, 115.0) and inside["alt_dot_colour"] == "#1565c0", inside
        assert inside["alt_ylabel"] == "altitude MSL (m)"
        b = inside["body"]
        assert "height    up  10.00 m above grid  up_home  15.00 m" in b, b
        assert "msl_m  115.00  ground  105.00 m MSL" in b, b
        assert "outside bands: -" in b, b
        # Row 20: ground 110, up 5 (under the 8 m floor), msl 115.
        assert outside["alt_dot"] == (2.0, 115.0) and outside["alt_dot_colour"] == "#c62828"
        b = outside["body"]
        assert "up   5.00 m above grid  up_home  15.00 m" in b, b
        assert "msl_m  115.00  ground  110.00 m MSL" in b, b
        assert "outside bands: agl-band" in b, b
    with tempfile.TemporaryDirectory() as d:
        rows = [dict(_row(i), up=12.0 + 0.1 * i) for i in range(20)]
        run = _run(Path(d), rows)
        got = []
        R.render_stills(run, Path(d) / "plain", [1.0], probe=got)
        (f,) = got
        assert f["alt_dot"] == (1.0, 13.0) and f["alt_ylabel"] == "altitude up (m)", f
        assert "position  x    5.00  y    2.00  up  13.00 m" in f["body"], f["body"]
        assert "msl_m" not in f["body"] and "outside bands" not in f["body"]


# An MSL envelope over a run with no up_ref terrain (the node over a FLAT
# --ground: rows above home, metrics.json ground.up_ref "home").
FLAT_POLICY = {"constraints": [
    {"id": "msl-ceiling", "type": "altitude_envelope", "alt_min_m": 590, "alt_max_m": 624,
     "altitude_ref": "MSL"},
    {"id": "band", "type": "altitude_envelope", "alt_min_m": 5, "alt_max_m": 60}]}


def test_an_msl_envelope_off_a_terrain_run_is_offset_by_home_or_not_drawn():
    """Review 2026-11-18: a run without up_ref terrain drew the FIRST
    altitude_envelope on its above-home axis whatever its reference, so an
    MSL 590-624 m band sat over a 15 m trace. Now the up axis takes the first
    AGL envelope, and an MSL one is offset by home's elevation where the run
    states it (a flat --ground in metrics.json) or named as not drawn."""
    assert R.altitude_band(FLAT_POLICY) == (5.0, 60.0)
    assert R.altitude_band(TERRAIN_POLICY) == (8.0, 60.0)
    assert R.home_axis_msl_bands(FLAT_POLICY, 584.0) == [
        {"id": "msl-ceiling", "lo": 590.0, "hi": 624.0, "ref": "MSL", "up_lo": 6.0, "up_hi": 40.0}]
    assert R.home_axis_msl_bands(FLAT_POLICY, None)[0]["up_lo"] is None
    assert R.home_axis_msl_bands(POLICY, 584.0) == []
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [dict(_row(i), up=15.0) for i in range(20)], policy=FLAT_POLICY)
        assert R.flat_ground_msl(run) is None, "no metrics.json"
        for g in ({"ground": {"kind": "grid", "min_msl_m": 580}, "up_ref": "terrain"},
                  {"ground": {"kind": "flat", "elevation_msl_m": 584.0}, "up_ref": "terrain"},
                  "junk"):
            (run / "metrics.json").write_text(json.dumps({"ground": g}), encoding="utf-8")
            assert R.flat_ground_msl(run) is None, g
        try:
            import matplotlib                                  # noqa: F401
        except ImportError:
            return None
        got = []
        R.render_stills(run, Path(d) / "none", [1.0], probe=got)
        assert any("msl-ceiling: MSL 590-624 m not drawn" in t for t in got[0]["alt_texts"]), got
        (run / "metrics.json").write_text(json.dumps({"ground": {
            "ground": {"kind": "flat", "elevation_msl_m": 584.0, "source": "x"},
            "up_ref": "home"}}), encoding="utf-8")
        assert R.flat_ground_msl(run) == 584.0
        got = []
        R.render_stills(run, Path(d) / "flat", [1.0], probe=got)
        assert got[0]["alt_dot"] == (1.0, 15.0) and got[0]["alt_ylabel"] == "altitude up (m)"
        assert any("msl-ceiling: MSL 590-624 m (home 584 m MSL)" in t
                   for t in got[0]["alt_texts"]), got


def test_a_short_run_renders_a_non_empty_video_and_says_it_is_a_log_replay():
    try:
        from tools.encode_deliverable import find_ffmpeg
        find_ffmpeg()
        import matplotlib                                      # noqa: F401
    except (Exception, SystemExit) as e:                       # noqa: BLE001
        print(f"      SKIP: no ffmpeg or matplotlib ({e})")
        return SKIP
    src = (ROOT / "tools" / "render_track_video.py").read_text(encoding="utf-8")
    assert "not a camera" in src and "recording." in src
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_row(i, "Normal", "Brake" if i else "Normal", "X1" if i == 1 else None)
                             for i in range(4)])
        out = R.render(run, run / "v.mp4", fps=4)
        assert out.is_file() and out.stat().st_size > 0
        assert not (run / "v.partial.mp4").exists()


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
            else:
                print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed, {skipped} skipped")
    sys.exit(1 if failed else 0)
