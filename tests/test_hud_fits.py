"""
demo/policy_hud.py and demo/vla_mission.py - what the video panels say, and
that all of it fits on the frame.

Run either way:
    pytest tests/test_hud_fits.py -v
    python tests/test_hud_fits.py

Found on 2026-11-04 by looking at frames of the AGX Orin matrix videos:

  * the rule panel's last line, "Shield corrections N  brakes N  holds N  P0
    escapes N", ran past the frame's right edge on every Jetson-flown video:
    the Jetsons have no Consolas, the fallback DejaVu Sans Mono is wider, and
    the panel was laid out at fixed pixel columns (478.7 px of text for 460 px
    of room with one-digit counts; on the PC, 478.5 px once the counts read
    2784 and 454). A ten-character value ("141 m away") ran into its status.
  * the VLA panel named every model "AerialVLA" (DroneCATS and RaceVLA too).
  * a long Shield banner covered the status box's speed.

No simulator, no GPU: everything is drawn on blank frames.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import policy_hud as H                                              # noqa: E402

# The rows and counts of nfz3b_agx_hier_aerovla_20261103 at t 418.7 s.
ROWS = [
    {"label": "NFZ nfz-frontage", "limit": "keep out", "value": "141 m away", "status": "ok"},
    {"label": "NFZ nfz-north-street", "limit": "keep out", "value": "37 m away", "status": "ok"},
    {"label": "NFZ nfz-south-block", "limit": "keep out", "value": "189 m away", "status": "ok"},
    {"label": "Stand-off from person", "limit": ">= 10 m", "value": "no target", "status": "idle"},
    {"label": "Stand-off from target", "limit": ">= 5 m", "value": "no target", "status": "idle"},
    {"label": "Obstacle clearance", "limit": ">= 3 m", "value": "off map", "status": "near"},
    {"label": "Altitude band", "limit": "6-14 m", "value": "8.1 m", "status": "ok"},
    {"label": "Speed limit", "limit": "<= 5 m/s", "value": "1.5 m/s", "status": "act"},
]
LONG_BANNER = {"level": "act",
               "text": "SHIELD CORRECTED COMMAND - pushed away from obstacle (bld-clearance)"}
LINES = ["t 418.7s   alt 8.1 m", "bearing -46.6deg   fwd +1.5 m/s",
         "separation 225.8 m", "HOLDING OVER JUNCTION"]


def _snap(counts=None, banner=None, rows=ROWS):
    return {"t": 1.0, "banner": banner or LONG_BANNER, "rows": rows,
            "counts": counts or {"repaired": 2784, "braked": 0, "held": 454, "p0_escapes": 0},
            "policy": "follow-car-citylife-nfz3b v0.1.1", "map": None}


def _layer(size=(1280, 720)):
    from PIL import Image
    return Image.new("RGBA", size, (0, 0, 0, 0))


def _ink_outside(layer, boxes) -> int:
    """Pixels drawn outside every box drawn (alpha > 0)."""
    import numpy as np
    a = np.asarray(layer.getchannel("A")) > 0
    keep = np.zeros_like(a)
    for bb in boxes:
        x0, y0, x1, y1 = H._pixel_box(bb, layer.size)
        keep[y0:y1, x0:x1] = True
    return int((a & ~keep).sum())


def _intersects(a, b) -> bool:
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def test_dejavu_sans_mono_is_the_face_on_every_machine():
    """The Jetson's face, asked for first, so the PC lays out what the
    Jetson draws. Found by name or inside matplotlib; if neither exists on a
    machine, the fallbacks below still have to fit (the other tests)."""
    paths = [p for p in H._face_paths(False) if "DejaVu" in p]
    assert paths and paths[0] == "DejaVuSansMono.ttf", paths
    f = H.font(15)
    found = any(Path(p).is_file() for p in paths[1:])
    if found:
        assert str(getattr(f, "path", "")).endswith("DejaVuSansMono.ttf"), f.path


def test_the_rule_panel_and_banner_fit_the_frame_with_the_nfz3b_counts():
    for size in ((1280, 720), (960, 540), (1920, 1080)):
        layer = _layer(size)
        box = H.draw_status_box(layer, LINES)
        drawn = H.draw_policy_overlay(layer, _snap(), avoid=[box])
        assert _ink_outside(layer, [box] + drawn) == 0, size
        W, Hh = size
        for bb in [box] + drawn:
            assert 0 <= bb[0] and bb[2] <= W and 0 <= bb[1] and bb[3] <= Hh, (size, bb)


def test_the_p0_escapes_are_drawn_inside_the_panel_and_red_when_not_zero():
    import numpy as np
    for esc, red in ((0, False), (3, True)):
        layer = _layer()
        banner, panel = H.draw_policy_overlay(layer, _snap(
            counts={"repaired": 2784, "braked": 0, "held": 454, "p0_escapes": esc}))
        x0, y0, x1, y1 = H._pixel_box(panel, layer.size)
        px = np.asarray(layer.convert("RGB"))[y0:y1, x0:x1].reshape(-1, 3)
        hit = int(((px == np.array(H.COLOURS["brake"])).all(axis=1)).sum())
        assert (hit > 0) == red, (esc, hit)


def test_value_and_status_columns_never_touch():
    """The status column starts after the widest value the panel can hold
    (12 characters): in the pixels drawn, each row's value - its rightmost
    light ink - ends a visible gap (half a character) before its status's
    orange begins. "141 m awayOK" had none."""
    import numpy as np
    rows = [dict(r, value="W" * 12, status="act") for r in ROWS]
    layer = _layer()
    _, panel = H.draw_policy_overlay(layer, _snap(rows=rows))
    a = np.asarray(layer.convert("RGB")).astype(int)
    light = (np.abs(a - 235) < 25).all(axis=2)
    orange = np.abs(a - np.array(H.COLOURS["act"])).sum(axis=2) < 30
    x_text = int(panel[0] + 30)                       # right of the status dots
    for i in range(len(rows)):
        y0 = int(panel[1] + 28 + 21 * i)
        band_l, band_o = light[y0:y0 + 19, x_text:], orange[y0:y0 + 19, x_text:]
        value_end = np.where(band_l.any(axis=0))[0].max()
        status_start = np.where(band_o.any(axis=0))[0].min()
        assert status_start - value_end >= 0.5 * H._tw(H.font(15), "W"), (
            i, status_start, value_end)


def test_a_long_banner_stays_off_the_status_box():
    """Drawn on a layer of its own, no ink of the overlay lands in the box."""
    import numpy as np
    box = H.draw_status_box(_layer(), LINES)
    over = _layer()
    banner, panel = H.draw_policy_overlay(over, _snap(), avoid=[box])[:2]
    x0, y0, x1, y1 = H._pixel_box(box, over.size)
    assert not (np.asarray(over.getchannel("A"))[y0:y1, x0:x1] > 0).any(), (banner, box)
    assert not _intersects(banner, panel), (banner, panel)
    # Without anything to avoid the same banner is centred on the frame.
    alone = H.draw_policy_overlay(_layer(), _snap())[0]
    assert abs((alone[0] + alone[2]) / 2 - 640) < 1.0, alone


def test_a_banner_too_long_for_its_room_is_shrunk_then_cut_never_overflows():
    layer = _layer()
    box = H.draw_status_box(layer, ["x" * 60] * 4)          # the widest box there is
    ban = {"level": "brake", "text": "P0 RULE STILL BROKEN AFTER SHIELD (" + ", ".join(
        f"nfz-zone-{i}" for i in range(12)) + ")"}
    drawn = H.draw_policy_overlay(layer, _snap(banner=ban), avoid=[box])
    assert drawn[0][0] >= box[2] and drawn[0][2] <= 1280, drawn[0]
    assert _ink_outside(layer, [box] + drawn) == 0


def test_the_status_box_grows_to_its_line_cuts_past_45_percent_and_never_shrinks():
    layer = _layer()
    short = H.draw_status_box(layer, ["t 1.0s"])
    assert abs((short[2] - short[0]) - 322) < 1e-6, short          # the box it always was
    vla = "AerialVLA  no decision flying  pipelined"
    assert H.draw_status_box(_layer(), [vla])[2] > 8 + H._tw(H.font(15), vla)
    huge = H.draw_status_box(layer := _layer(), ["y" * 200])
    assert huge[2] <= 0.45 * 1280 + 1e-6 and _ink_outside(layer, [huge]) == 0
    cache = H.OverlayCache()
    frame = _layer().convert("RGB")
    cache.draw(frame, _snap(), [vla])
    wide = cache._box_w
    cache.draw(frame, dict(_snap(), t=2.0), ["t"])
    assert cache._box_w == wide, (cache._box_w, wide)


def test_the_cache_pastes_what_it_drew():
    """One tile per drawn box; the frame outside them is untouched."""
    import numpy as np
    from PIL import Image
    frame = Image.new("RGB", (1280, 720), (0, 120, 0))
    H.OverlayCache().draw(frame, _snap(), LINES)
    px = np.asarray(frame)
    assert (px[400:700, 300:1000] == (0, 120, 0)).all()


# ------------------------------------------------------- the model on the panel

def _vm():
    import vla_mission
    return vla_mission


# The ids the AGX matrix's servers sent (metrics.json pilot.vla.identity.id).
IDS = {"aerialvla-lora/aero_vla@unknown": "AerialVLA",
       "aerialvla-ft-follow/follow_v2_r1/epoch4@72cd9ae4": "AerialVLA FT v2_r1",
       "aerialvla-ft-follow/follow_v3_pf_r1/epoch1@3309cdfb": "AerialVLA FT v3_pf_r1",
       "dronecats/DroneCATS-Cosmos3-Edge-2B@5080e727": "DroneCATS",
       "racevla/drone6_all@03f3bb48": "RaceVLA",
       "fake:scripted": "stand-in, not a VLA"}


def test_the_panel_names_the_model_the_server_said_it_is():
    VM = _vm()
    for sid, want in IDS.items():
        assert VM.model_label({"id": sid}) == want, (sid, VM.model_label({"id": sid}))
    assert VM.model_label(None) == "VLA" and VM.model_label({"id": ""}) == "VLA"


def test_hud_lines_say_the_model_and_never_default_to_aerialvla():
    VM = _vm()

    class Tk:
        author, reason, seq, age_s, land_requested = "hover", "no_new_decision", None, None, False

    for model, want in (("DroneCATS", "DroneCATS  no decision flying"), (None, "VLA  no decision flying")):
        hud = VM.vla_hud(1.0, 8.0, Tk(), {}, None, touched=False, shield_on=True,
                         mission="follow", cadence=None, goal_dist_m=None, box_frac=None,
                         end_reason=None, model=model)
        assert VM.hud_lines(hud)[1] == want, VM.hud_lines(hud)


def test_every_vla_rail_hands_the_panel_the_servers_model():
    """vla_fly (the city rail) and the ArduPilot rail's VLA pilot pass
    model_label of the server's hello; neither may fall back to the old
    hard-coded name."""
    fly = (ROOT / "demo" / "vla_fly.py").read_text(encoding="utf-8")
    node = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    mission = (ROOT / "demo" / "vla_mission.py").read_text(encoding="utf-8")
    assert "model_label=VM.model_label(identity)" in fly
    assert "model_label=VM.model_label(vla_identity)" in node
    assert 'f"AerialVLA #' not in mission and 'f"AerialVLA  no decision' not in mission


def test_the_hierarchy_rule_names_no_model():
    import vla_hierarchy
    assert "AerialVLA" not in vla_hierarchy.RULE, vla_hierarchy.RULE[:60]


# --------------------------------------------------------------------- runner

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
