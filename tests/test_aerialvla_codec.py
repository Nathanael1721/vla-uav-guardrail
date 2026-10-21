"""
guardrail/aerialvla_codec.py - AerialVLA's prompt, vocabulary, mosaic and a
strict reader for its output.

Run either way:
    pytest tests/test_aerialvla_codec.py -v
    python tests/test_aerialvla_codec.py

What is at stake, each with a test that fails if it breaks silently:

  1. THE READER. Until 2026-10-21 every caller took the last three integers
     of the text (`re.findall(r"\\d+", tail)[-3:]`, upstream's parser), which
     reads malformed landing outputs as motion: '00 49 494 49' became forward
     2.5 m and a clamped +5 m "down". Every malformed output the 4 Aug
     ablation actually produced (experiments/out/ablation_*.json) is pinned
     here as parse_ok=False, and the old reading of the worst one is pinned
     as the dive it was.
  2. THE PROMPT must be upstream's byte for byte, for all 8 phrases, and the
     vocabulary must be the one demo/vla_bridge.py and
     demo/aerialvla_demo.semantic_direction use.
  3. THE UNITS: bins decode to metres / radians PER STEP at the lower edge
     of the truncation cell, upstream's decoder.
  4. No torch and no Pillow at import: the Shield's import path and the CI
     jobs import this module. The image tests need Pillow and SKIP without it
     (no CI job installs it).
"""
from __future__ import annotations

import ast
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from guardrail import aerialvla_codec as C                        # noqa: E402

SKIP = "SKIP"
CODEC = ROOT / "guardrail" / "aerialvla_codec.py"
RESCORE = ROOT / "docs" / "data" / "aerialvla_ablation_rescore.json"

# Every malformed output the 4 Aug ablation produced (both adapters), as the
# script stored it (`tail`): the strict reader must refuse each one.
ABLATION_MALFORMED = [
    "00 49 4949</s>",              # orig + ft, no hint: yaw bin 4949 -> clamped +1.1
    "00 49 494 49</s>",            # orig, no hint: (49, 494, 49) -> fwd 2.5, down +5
    "00 49 4944949</s>",           # orig, no hint
    "00 49 49 49 49 48</s>",       # orig, no hint: six bins
    "00 49 49 L 49</s>",           # orig + ft, no hint: a stray "L"
    "9运 48 60</s>",           # orig, forward-right: a CJK character inside a bin
    "49 49 49 49 49 49 49",        # ft, no hint: no EOS (ran out of max_new_tokens)
    "00 49 49 49 49 49 49",        # ft, no hint: no EOS
    "   70 48</s>",                # ft, to your right: two bins, leading spaces
    "                </s>",        # ft, forward-left: nothing but spaces
    "15 51 0</s>",                 # ft, to your left: a one-digit bin (read as -1.1)
    "15 49 0</s>",                 # ft, to your left
    "25 49 0</s>",                 # ft, to your left
]
# Shapes the model has not produced (yet) and the reader must refuse anyway.
OTHER_MALFORMED = [
    "", " 00 49 49</s>", "00 49 49 </s>", "00  49 49</s>", "00 49 49 LAND LAND</s>",
    "00 49 49 land</s>", "00 49 49 LAN</s>", "00 49 49 <LAND></s>", "00 49 49</s></s>",
    "00 49 49\n</s>", "00\t49 49</s>", "99 49 49</s>", "00 99 49</s>", "00 49 99</s>",
    "0 49 49</s>", "000 49 49</s>", "00 49</s>", "00 49 49 49</s>", "-1 49 49</s>",
    "００ 49 49</s>",      # fullwidth digits: \d matches them, [0-9] does not
    "LAND</s>", "</s>", "00 49 49<pad>", "00,49,49</s>",
]


# --------------------------------------------------------------------- reader

def test_the_trained_shapes_are_accepted_and_decoded_as_a_step():
    p = C.parse_action("96 48 49</s>")
    assert p.parse_ok and p.bins == (96, 48, 49) and not p.land and p.eos
    assert math.isclose(p.step.fwd_m, 96 / 98 * 5.0)
    assert math.isclose(p.step.down_m, 48 / 98 * 10.0 - 5.0)
    assert p.step.dpsi_rad == 0.0
    q = C.parse_action("00 49 49 LAND</s>")
    assert q.parse_ok and q.land and q.bins == (0, 49, 49)
    assert q.step.to_dict() == {"fwd_m": 0.0, "down_m": 0.0, "dpsi_rad": 0.0}
    # EOS is optional unless required (a text decoded without special tokens)
    assert C.parse_action("00 49 49").parse_ok
    assert not C.parse_action("00 49 49", require_eos=True).parse_ok
    assert C.parse_action("00 49 49 LAND</s>", require_eos=True).parse_ok


def test_every_malformed_ablation_output_is_refused_and_holds_nothing():
    for text in ABLATION_MALFORMED + OTHER_MALFORMED:
        p = C.parse_action(text, require_eos=True)
        assert not p.parse_ok, f"accepted {text!r}"
        assert p.bins is None and p.step is None and p.land is False, (text, p)
        assert p.reason, f"no reason for {text!r}"
        # require_eos=False must not rescue any of them either
        assert not C.parse_action(text).parse_ok, text


def test_the_old_reader_turned_a_landing_output_into_a_dive():
    """The defect itself, pinned: the last-three-integers reading of
    '00 49 494 49' is forward 2.5 m and down +5.0 m (a 5 m descent), and of
    '00 49 4949' a full +1.1 rad turn - both out of outputs that begin with
    the stop triple. The strict reader refuses both."""
    dive = C.legacy_last_three_ints("00 49 494 49</s>")
    assert dive["ok"] and dive["bins"] == [49, 494, 49]
    assert dive["step"] == {"fwd_m": 2.5, "down_m": 5.0, "dpsi_rad": 0.0}, dive
    assert dive["land"] is False
    spin = C.legacy_last_three_ints("00 49 4949</s>")
    assert spin["step"]["dpsi_rad"] == 1.1 and spin["land"] is False
    for text in ("00 49 494 49</s>", "00 49 4949</s>"):
        assert not C.parse_action(text, require_eos=True).parse_ok


def test_land_reports_the_decoded_step_it_came_with():
    """LAND => land=True and the step as decoded (normally zero), never
    zeroed by the reader: the rail decides what LAND means."""
    p = C.parse_action("50 49 49 LAND</s>")
    assert p.parse_ok and p.land and math.isclose(p.step.fwd_m, 50 / 98 * 5.0)
    z = C.parse_action("00 49 49</s>")
    assert z.parse_ok and not z.land and z.step.is_zero, "a zero step is not LAND"


def test_a_decode_of_prompt_and_output_loses_exactly_the_prompts_space():
    assert C.generated_from_decoded("x\nAction: 00 49 49</s>") == "00 49 49</s>"
    assert C.generated_from_decoded("Action:  00 49 49</s>") == " 00 49 49</s>"
    assert not C.parse_action(C.generated_from_decoded("Action:  00 49 49</s>")).parse_ok


# ---------------------------------------------------------------------- units

def test_bins_decode_to_the_lower_edge_of_their_truncation_cell():
    for axis, (lo, hi) in C.RANGES.items():
        assert C.dequantize(0, axis) == lo and C.dequantize(98, axis) == hi
        w = (hi - lo) / 98
        for b in range(99):
            v = C.dequantize(b, axis)
            assert C.quantize(v + w / 2, axis) == b, (axis, b)       # the cell's centre
            a, z = C.bin_interval(b, axis)
            assert a == v and (z == a + w if b < 98 else z == hi)
    assert C.dequantize(49, "down_m") == 0.0 and C.dequantize(49, "dpsi_rad") == 0.0
    # A level-flight training label (down = -0.0030 m) truncates to bin 48,
    # whose lower edge is a 0.102 m climb: the model's frequent '96 48 49'.
    assert C.quantize(-0.0029972265637362074, "down_m") == 48
    assert math.isclose(C.dequantize(48, "down_m"), -10 / 98)
    lo48, hi48 = C.bin_interval(48, "down_m")
    assert lo48 <= -0.003 < hi48
    for bad in (-1, 99, 4949, 1.0, True):
        try:
            C.dequantize(bad, "fwd_m")
        except ValueError:
            continue
        raise AssertionError(f"dequantize accepted {bad!r}")


def test_a_step_rotates_only_at_upstreams_threshold():
    assert C.Step(4.0, 0.0, 0.25).rotate_only and C.Step(4.0, 0.0, -0.25).rotate_only
    assert not C.Step(4.0, 0.0, 0.2449).rotate_only
    s = C.bins_to_step((96, 48, 60))
    assert math.isclose(s.dpsi_rad, 60 / 98 * 2.2 - 1.1) and s.rotate_only is (abs(s.dpsi_rad) >= 0.25)
    assert C.YAW_ONLY_THRESHOLD_RAD == 0.25 and C.CRUISE_SPEED_MPS == 1.0
    assert C.HOVER_AFTER_STEP_S == 0.5 and C.NUM_BINS == 99


# --------------------------------------------------------------------- prompt

def test_the_prompt_is_upstreams_template_byte_for_byte_for_all_eight_phrases():
    for phrase in C.PHRASES:
        got = C.build_prompt(phrase, "orange car")
        want = "<image>\nFly " + phrase + "and find the target. orange car\nAction: "
        assert got == want, (phrase, got)
        assert got.endswith("\nAction: ")
    assert C.build_prompt("straight ahead ", "a red car") == \
        "<image>\nFly straight ahead and find the target. a red car\nAction: "
    assert C.build_prompt("", "a red car") == \
        "<image>\nFly and find the target. a red car\nAction: "
    assert C.PROMPT_TEMPLATE == "<image>\nFly {direction}and find the target. {object}\nAction: "


def test_the_legacy_prompt_builder_agrees_except_for_an_empty_object():
    """demo/aerialvla_demo.build_prompt normalises whitespace; it is kept for
    the 4 Aug ablation. The two agree byte for byte for every phrase and
    every non-empty object, and differ only by the template's space before
    the newline when the object is empty."""
    from aerialvla_demo import build_prompt as legacy
    for phrase in C.PHRASES:
        for obj in ("orange barrier", "The man is walking on a road near some bushes."):
            assert C.build_prompt(phrase, obj) == legacy(phrase, obj), (phrase, obj)
        assert C.build_prompt(phrase, "") == legacy(phrase, "").replace(
            "target.\nAction: ", "target. \nAction: ")


def test_the_vocabulary_is_the_one_the_bridge_and_semantic_direction_use():
    import vla_bridge
    from aerialvla_demo import semantic_direction
    assert len(C.PHRASES) == 8 and len(set(C.PHRASES)) == 8 and C.NO_HINT in C.PHRASES
    assert set(C.DIRECTION_VOCABULARY) == set(vla_bridge.DIRECTION_VOCABULARY)
    seen = set()
    a = -180.0
    while a < 180.0:
        r = math.radians(a)
        seen.add(semantic_direction((0.0, 0.0), 0.0, (10 * math.cos(r), 10 * math.sin(r))))
        a += 0.25
    assert seen == set(C.DIRECTION_VOCABULARY), seen ^ set(C.DIRECTION_VOCABULARY)
    assert semantic_direction((0.0, 0.0), 0.0, None) == C.NO_HINT


def test_phrase_object_and_hint_validation_refuse_near_misses():
    for bad in ("straight ahead", "Straight ahead ", " straight ahead ", "forward right ",
                None, 3, "to your left  "):
        try:
            C.validate_phrase(bad)
        except C.CodecError as e:
            assert e.code == C.ERR_BAD_PHRASE
            continue
        raise AssertionError(f"accepted phrase {bad!r}")
    for bad in ("a\ncar", "a car\n", " a car", "a car ", "x" * (C.MAX_OBJECT_CHARS + 1), 5):
        try:
            C.validate_object(bad)
        except C.CodecError as e:
            assert e.code == C.ERR_BAD_OBJECT
            continue
        raise AssertionError(f"accepted object {bad!r}")
    assert C.validate_object("") == ""
    C.validate_hint("", "none")
    C.validate_hint("", "goal")                           # "you have arrived"
    C.validate_hint("forward-left ", "detector")
    for phrase, src in (("forward-left ", "none"), ("", "detector"), ("", "language"),
                        ("straight ahead ", None)):
        try:
            C.validate_hint(phrase, src)
        except C.CodecError as e:
            assert e.code == C.ERR_BAD_HINT
            continue
        raise AssertionError(f"accepted {phrase!r} from {src!r}")


def test_the_csp_goes_on_the_instruction_line_before_fly_and_off_is_untouched():
    off = C.build_prompt("forward-left ", "orange car")
    csp = C.build_prompt("forward-left ", "orange car", "Stay below 14 m.", "csp")
    neu = C.build_prompt("forward-left ", "orange car", "The scene is ordinary.", "neutral")
    assert csp == "<image>\nStay below 14 m. Fly forward-left and find the target. orange car\nAction: "
    assert neu == "<image>\nThe scene is ordinary. Fly forward-left and find the target. orange car\nAction: "
    assert csp.endswith("\nAction: ") and csp.count("\n") == off.count("\n") == 2
    assert C.build_prompt("forward-left ", "orange car", "  Stay low.  ", "csp") == \
        "<image>\nStay low. Fly forward-left and find the target. orange car\nAction: "
    assert len({C.prompt_sha256(x) for x in (off, csp, neu)}) == 3
    assert C.prompt_sha256(off) == hashlib.sha256(off.encode("utf-8")).hexdigest()
    for mode, text in (("off", "Stay low."), ("csp", ""), ("csp", "   "), ("csp", None),
                       ("neutral", "line one\nline two"), ("on", "x"), ("csp", 5)):
        try:
            C.build_prompt("forward-left ", "orange car", text, mode)
        except C.CodecError as e:
            assert e.code == C.ERR_BAD_CSP
            continue
        raise AssertionError(f"accepted csp_mode {mode!r} with {text!r}")


# --------------------------------------------------------------------- import

def test_a_position_written_into_the_object_or_the_csp_is_refused():
    """Review of V1a (P10), probe-verified at 6519b20: coordinates were
    refused by KEY only, so object='target at x=38.0 y=26.0' reached the
    prompt - the one way left to tell the VLA where the target is. Text that
    looks like a position is now refused with its own code. The accepted
    texts are upstream's own kind (verbatim from the training set, which
    has 7,611 distinct descriptions and none refused: a lone number, a year,
    the word "target" in prose) and the CSP's rule sentences."""
    leaks = ["target at x=38.0 y=26.0", "the car at (38, 26)", "an orange car, goal: 38",
             "a car at lat 35.6895, lon 139.6917", "the barrier near 38.0, 26.0",
             "coordinates [38,26]", "a red car, x=38", "the target is 38 m east",
             "a van at y: -12", "destination = 4"]
    for obj in leaks:
        try:
            C.validate_object(obj)
        except C.CodecError as e:
            assert e.code == C.ERR_COORDINATE_TEXT, (obj, e.code)
            continue
        raise AssertionError(f"object {obj!r} was accepted")
    fine = ["a parked car", "orange barrier", "",
            'The man is standing alone on an empty asphalt parking lot with white lines and a '
            'number "38" marked on the ground near him',
            "The blue car is located on a road with a speed limit of 30, surrounded by foliage",
            "The area surrounding the target, a man, consists of dense greenery",
            'banners advertising messages like "Live Your Dream in 2021,"',
            "a restaurant offering 24-hour service", "a 3-story building on 5th avenue"]
    for obj in fine:
        assert C.validate_object(obj) == obj
        assert C.build_prompt(C.STRAIGHT_AHEAD, obj)
    csp = ("Stay between 5 m and 60 m altitude. Never enter zone 'nfz_1', a circle of radius "
           "12.5 m. Keep speed at or below 4 m/s.")
    assert C.validate_csp("csp", csp) == csp
    for leak in ("Never enter zone 'z1': the straight path from (0, 0) to the target (38, 26) "
                 "crosses this zone.", "Stay low. Goal at 38, 26."):
        try:
            C.validate_csp("csp", leak)
        except C.CodecError as e:
            assert e.code == C.ERR_COORDINATE_TEXT, (leak, e.code)
            continue
        raise AssertionError(f"csp_text {leak!r} was accepted")


def test_the_codec_imports_without_torch_or_pillow():
    code = ("import sys\n"
            f"sys.path.insert(0, {str(ROOT)!r})\n"
            "for b in ('torch', 'transformers', 'PIL'): sys.modules[b] = None\n"
            "from guardrail import aerialvla_codec as C\n"
            "assert C.parse_action('00 49 49 LAND</s>').land\n"
            "assert C.build_prompt('', 'x').startswith('<image>')\n"
            "C.validate_request({'v': 2, 'op': 'hello'})\n"
            "print('ok')\n")
    r = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True, text=True,
                       timeout=120)
    assert r.returncode == 0 and "ok" in r.stdout, (r.stdout + r.stderr)[-800:]
    tree = ast.parse(CODEC.read_text(encoding="utf-8"))
    top = {(a.name if isinstance(n, ast.Import) else n.module).split(".")[0]
           for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))
           for a in (n.names if isinstance(n, ast.Import) else [n])}
    assert not top & {"torch", "transformers", "PIL", "numpy"}, top


def test_the_demo_and_the_ablation_read_through_the_codec():
    """Neither file may take integers out of the model's text any more."""
    for rel in ("demo/aerialvla_demo.py", "experiments/ablate_image_vs_hint.py"):
        src = (ROOT / rel).read_text(encoding="utf-8")
        tree = ast.parse(src)
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == "findall"]
        assert not calls, f"{rel} still calls findall"
        imported = {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)
                    and n.module == "guardrail.aerialvla_codec" for a in n.names}
        assert "parse_action" in imported, rel


def test_the_ablation_never_overwrites_the_files_the_rescore_pins():
    """Review of V1a (P7): re-running experiments/ablate_image_vs_hint.py
    overwrote experiments/out/ablation_{label}.json - the gitignored 4 Aug
    source that docs/data/aerialvla_ablation_rescore.json pins by SHA-256 -
    after which `rescore_ablation.py --check` said DIFFERS and the original
    outputs survived only inside the rescore. A run now writes a new
    timestamped file by default, and --out onto an existing file is refused
    (before the model loads) unless --force."""
    import contextlib
    import importlib.util
    import io
    import tempfile
    from datetime import datetime, timezone
    spec = importlib.util.spec_from_file_location(
        "ablate_image_vs_hint", ROOT / "experiments" / "ablate_image_vs_hint.py")
    ab = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ab)
    now = datetime(2026, 10, 22, 1, 2, 3, tzinfo=timezone.utc)
    p = ab.output_path("orig", None, False, now)
    assert p == ab.OUT / "ablation_orig_20261022T010203Z.json", p
    assert p.name != "ablation_orig.json"
    d = Path(tempfile.mkdtemp(prefix="ablate_"))
    taken = d / "ablation_orig.json"
    taken.write_text("[]", encoding="utf-8")
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        try:
            ab.main(["--label", "orig", "--out", str(taken)])
        except SystemExit as e:
            assert e.code not in (0, None) and "--force" in str(e.code), e.code
        else:
            raise AssertionError("an existing ablation output was overwritten")
    assert taken.read_text(encoding="utf-8") == "[]"
    assert ab.output_path("orig", str(taken), True, now) == taken       # --force: on purpose
    assert ab.output_path("orig", str(d / "new.json"), False, now) == d / "new.json"


# ---------------------------------------------------------------------- images

def _pil():
    """Pillow's Image module, or None (the test SKIPs). With
    GUARDRAIL_REQUIRE_PILLOW=1 - set by the CI step that installs Pillow for
    these files (review of V1a, P9) - a missing Pillow is a broken install,
    and the test fails instead of skipping."""
    import os
    try:
        from PIL import Image
        return Image
    except ImportError:
        if os.environ.get("GUARDRAIL_REQUIRE_PILLOW") == "1":
            raise AssertionError("GUARDRAIL_REQUIRE_PILLOW=1 and Pillow does not import")
        return None


def test_the_mosaic_is_upstreams_construction_and_a_missing_down_is_recorded():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    front = Image.new("RGB", (768, 432), (200, 30, 30))
    for x in range(0, 768, 7):
        front.putpixel((x, x % 432), (10, 250, 10))
    down = Image.new("RGB", (400, 225), (20, 20, 220))
    m, info = C.build_mosaic(front, down)
    assert m.size == (224, 448) and m.mode == "RGB"
    assert info["down_missing"] is False and info["down_fill"] is None
    assert info["front_src_size"] == [768, 432] and info["down_src_size"] == [400, 225]
    # upstream's own code (aerialvla_wrapper_ui.py:135-141), byte for byte
    rs = getattr(Image, "Resampling", Image).BICUBIC
    ref = Image.new("RGB", (224, 448), (0, 0, 0))
    ref.paste(front.resize((224, 224), resample=rs), (0, 0))
    ref.paste(down.resize((224, 224), resample=rs), (0, 224))
    assert m.tobytes() == ref.tobytes()
    assert C.mosaic_sha256(m) == hashlib.sha256(ref.tobytes()).hexdigest()
    # a missing down frame: black lower tile, and it says so
    m2, info2 = C.build_mosaic(front, None)
    assert info2["down_missing"] is True and info2["down_fill"] == "black"
    assert m2.getpixel((100, 300)) == (0, 0, 0) and m2.crop((0, 0, 224, 224)).tobytes() == \
        ref.crop((0, 0, 224, 224)).tobytes()
    assert C.mosaic_sha256(m2) != C.mosaic_sha256(m)
    try:
        C.build_mosaic(front, None, missing_down="refuse")
    except C.CodecError as e:
        assert e.code == C.ERR_BAD_IMAGE
    else:
        raise AssertionError("a refused missing down frame was accepted")
    try:
        C.mosaic_sha256(front)
    except ValueError:
        pass
    else:
        raise AssertionError("hashed something that is not a 224x448 mosaic")


def test_frames_round_trip_through_base64_jpeg_and_bad_ones_are_refused():
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    img = Image.new("RGB", (64, 48), (120, 60, 30))
    back = C.decode_image_b64(C.encode_jpeg_b64(img))
    assert back.size == (64, 48) and back.mode == "RGB"
    for bad in ("", "not base64!!", "aGVsbG8=", None):
        try:
            C.decode_image_b64(bad, "front_jpg_b64")
        except C.CodecError as e:
            assert e.code == C.ERR_BAD_IMAGE
            continue
        raise AssertionError(f"decoded {bad!r}")


def png_claiming(w: int, h: int) -> bytes:
    """A PNG whose header claims w x h RGB pixels and which carries no pixel
    data: Image.open reads the size from IHDR alone, so a few dozen bytes
    claim a frame of any size."""
    import struct
    import zlib

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(b"")) + chunk(b"IEND", b""))


def test_a_frame_claiming_too_many_pixels_is_a_bad_image_not_a_crash():
    """Review of V1a (P1): a PNG claiming 20000x20000 pixels made Image.open
    raise DecompressionBombError, which subclasses Exception, not OSError, so
    it escaped decode_image_b64 and the server dropped the connection. Now
    every decoder exception is a bad image, and a frame claiming more than
    MAX_FRAME_PIXELS is refused from its header before any pixel is
    decoded (5000x5000 is below Pillow's own limit and was decoded)."""
    Image = _pil()
    if Image is None:
        print("      SKIP: Pillow is not installed (the VLA environment has it)")
        return SKIP
    import base64
    for (w, h), says in (((20000, 20000), "DecompressionBombError"),
                         ((5000, 5000), "5000x5000 pixels"), ((4097, 4096), "4097x4096")):
        b64 = base64.b64encode(png_claiming(w, h)).decode("ascii")
        try:
            C.decode_image_b64(b64, "front_jpg_b64")
        except C.CodecError as e:
            assert e.code == C.ERR_BAD_IMAGE and says in e.detail, (w, h, e.detail)
            continue
        raise AssertionError(f"decoded a frame claiming {w}x{h}")
    assert C.MAX_FRAME_PIXELS == 4096 * 4096 < Image.MAX_IMAGE_PIXELS


# ------------------------------------------------------------ committed rescore

def test_the_committed_rescore_is_what_the_strict_reader_says_about_each_row():
    """docs/data/aerialvla_ablation_rescore.json carries every row's stored
    text; re-reading each with the codec must give the committed verdict, and
    the headline numbers of the 2026-10-21 correction must be the file's."""
    if not RESCORE.is_file():
        print("      SKIP: docs/data/aerialvla_ablation_rescore.json is absent")
        return SKIP
    d = json.loads(RESCORE.read_text(encoding="utf-8"))
    assert d["schema"] == "vlaguard.aerialvla_ablation_rescore/1"
    for label, a in d["adapters"].items():
        assert len(a["rows"]) == 108, label
        for r in a["rows"]:
            p = C.parse_action(C.generated_from_decoded("Action:" + r["tail"]), require_eos=True)
            assert p.parse_ok == r["strict"]["parse_ok"] and p.land == r["strict"]["land"], r
            assert (list(p.bins) if p.bins else None) == r["strict"]["bins"], r
    o = d["adapters"]["orig"]["summary"]
    nh = o["by_hint"]["(none)"]
    assert (nh["stop"], nh["land"], nh["hold"], nh["move"]) == (17, 11, 6, 1), nh
    pr = o["no_hint_present_vs_absent"]
    assert (pr["present"]["move"], pr["absent"]["move"]) == (1, 0)
    assert o["hinted_land"] == 0 and o["hinted_rows"] == 90 and o["parse_ok"] == 101
    ft = d["adapters"]["ft"]["summary"]
    assert ft["parse_ok"] == 96 and ft["by_hint"]["to your left"]["mean_dpsi_rad"] == -0.428
    malformed = {r["tail"] for a in d["adapters"].values() for r in a["rows"]
                 if not r["strict"]["parse_ok"]}
    assert malformed <= set(ABLATION_MALFORMED), malformed - set(ABLATION_MALFORMED)
    assert set(ABLATION_MALFORMED) <= malformed, set(ABLATION_MALFORMED) - malformed


# --------------------------------------------------------------------- runner

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
