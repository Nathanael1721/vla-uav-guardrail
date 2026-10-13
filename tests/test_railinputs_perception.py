"""The perception rail's inputs (demo/pas_ardupilot, part 2 of the rail-inputs
unit): the rule clock, the follow mission's CSP, --dev-yaml, the GeoFence
zones, the subject's ground truth, the literal LOITER hold and the MAVROS
path's parity with the pymavlink one.

    python tests/test_railinputs_perception.py      (no pytest needed)

Every flight here is the stand-in of tests/test_pas_ardupilot.py (a point-mass
ArduPilot behind a fake link, a fake detector and camera): nothing was flown
in Project AirSim, SITL or MAVROS.

Shown failing first against a pristine HEAD 4418ae0 copy (git archive) with
this file copied in: the node's parser refuses --clock-start, --subject-actor
and --dev-yaml (argparse exit 2, counted as a failure by the runner below);
every run records csp_failed "no mission given"; a YAML policy flies without
--dev-yaml; no link has upload_fence / override_throttle / rc_override_plan;
the param file has FENCE_TYPE 3 and no AVOID_ENABLE; MavrosLink never sets
the pitch, has no geofence clients and records no local_processes.

The review fixes of 2026-10-13 are guarded here too, each test shown failing
on the builder's version of the same files (a scratch copy with the fix
reverted) as well as on HEAD: a re-upload kept an enforced zone the aircraft
was next to OFF the fence; three failed pose reads stopped the truth for good
and the read ticks scored the flight; the pose RPC ran before the setpoint
was sent; the RC3 override went on after the autopilot left LOITER for a
mode nobody asked for; with no zone nothing was sent, so a stale fence list
stayed enforced; an in-flight upload had a 10 s timeout per phase; the
launcher flew a stale bundle by name; MAVROS's fcu_url was read from a
parameter MAVROS 2 does not declare where it was asked for.
"""
from __future__ import annotations

import collections
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "demo", PAS, ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import test_pas_ardupilot as T                                       # noqa: E402
import autopilot as ap                                               # noqa: E402
import mavros_link as ml                                             # noqa: E402
import perception_node as pn                                         # noqa: E402
import rail                                                          # noqa: E402
from guardrail.manifest import TOPOLOGY_DEV, build_manifest          # noqa: E402

SKIP = "SKIP"
ANCHOR = (35.6895, 139.6917, 40.0)        # the CityLife scene's home-geo-point


def _tmp() -> Path:
    return Path(tempfile.mkdtemp(prefix="pas_ri_"))


def _fly(policy: str, *, shield="on", max_s="3", extra=(), dev_yaml=True,
         link_cls=None, link_kw=None, camera_cls=None, camera_kw=None,
         grounder_kw=None):
    """tests/test_pas_ardupilot.py's _fly, with the link and camera chosen."""
    out = _tmp()
    pol = out / "policy.yaml"
    pol.write_text(policy, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", max_s,
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", T.WIN, "--wsl-ip", T.WSL, "--shield", shield,
        "--speed-max", "4", *(["--dev-yaml"] if dev_yaml else []), *extra])
    clk = T.Clock()
    anchor = ap.FrameAnchor(*ANCHOR, "scene.home-geo-point")
    link = (link_cls or T.FakeLink)(clk, anchor, **(link_kw or {}))
    cam = (camera_cls or T.FakeCamera)(link, **(camera_kw or {}))
    rc = pn.run(args, link=link, camera=cam,
                grounder=T.FakeDetections(clk, **(grounder_kw or {})),
                obs=T.FakeObs(), clock=clk, sleep=clk.sleep)
    return rc, out / "run", link, cam


def _rows(run):
    return [json.loads(x) for x in (run / "flight_log.jsonl").read_text().splitlines()]


def _json(run, name):
    return json.loads((run / name).read_text())


def _events(run):
    return [json.loads(x) for x in (run / "events.jsonl").read_text().splitlines()]


NFZ_AHEAD = """policy_id: pas-ri-nfz
version: 0.1.0
constraints:
  - id: nfz-ahead
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: 48, y: -30 }
      - { x: 60, y: -30 }
      - { x: 60, y: -10 }
      - { x: 48, y: -10 }
    altitude_floor_m: 0
    altitude_ceiling_m: 100
    margin_m: 1.0
"""

WINDOWED = NFZ_AHEAD.replace("pas-ri-nfz", "pas-ri-windowed") + """    valid_time:
      recurrence:
        start_time: "08:00"
        end_time: "09:00"
"""


# =========================================================================== #
# (1) the rule clock
# =========================================================================== #

def test_the_shield_and_the_csp_read_one_scripted_rule_clock():
    """A zone in force 08:00-09:00, flown into with the Shield off. At a
    scripted 08:30 the zone binds and its dwell counts; at 12:00 it is off,
    nothing violates it, and the time inside is nfz_off_by_clock_s. Before,
    the node gave the Shield no clock: every window was always in force."""
    rc, run, _, _ = _fly(WINDOWED, shield="off", max_s="6",
                         extra=("--clock-start", "2026-10-12T08:30:00"))
    assert rc == 0
    rows = _rows(run)
    hit = [r for r in rows if any(v["rule_id"] == "nfz-ahead" for v in r["violations"])]
    assert hit, "inside its window the zone bound nothing"
    m = _json(run, "metrics.json")
    assert m["nfz_s"] > 0 and m["nfz_off_by_clock_s"] == {}, (m["nfz_s"], m["nfz_off_by_clock_s"])
    assert m["rule_clock"]["kind"] == "scripted"
    assert m["rule_clock"]["start"] == "2026-10-12T08:30:00"
    assert m["generations"][0]["rule_clock_reading"] == "2026-10-12T08:30:00"
    csp = _json(run, "csp_g0.json")
    assert csp["issued_at"] == "2026-10-12T08:30:00", csp["issued_at"]
    start = next(e for e in _events(run) if e["kind"] == "episode_start")
    assert start["rule_clock"]["kind"] == "scripted"
    traj = _json(run, "trajectory.json")
    assert traj[0]["clock"].startswith("2026-10-12T08:30") and \
        traj[-1]["clock"] > traj[0]["clock"], "the clock does not advance with the mission"
    # One reading per tick, taken with the Shield's, on the row and the point.
    assert rows[0]["rule_clock"].startswith("2026-10-12T08:30:00")
    assert all(r["rule_clock"][:19] == p["clock"][:19] for r, p in zip(rows, traj))

    rc, run, _, _ = _fly(WINDOWED, shield="off", max_s="6",
                         extra=("--clock-start", "2026-10-12T12:00:00"))
    rows = _rows(run)
    assert not any(v["rule_id"] == "nfz-ahead" for r in rows for v in r["violations"]), \
        "at 12:00 a zone in force 08:00-09:00 bound the Shield"
    m = _json(run, "metrics.json")
    assert m["nfz_s"] == 0 and m["nfz_off_by_clock_s"].get("nfz-ahead", 0) > 0, m
    # Without --clock-start: the wall clock in the mission's zone, recorded.
    rc, run, _, _ = _fly(NFZ_AHEAD, max_s="1")
    rcl = _json(run, "metrics.json")["rule_clock"]
    assert rcl["kind"] == "wall" and rcl["tz"].startswith("+08:00"), rcl
    # An unreadable start is refused before anything flies.
    try:
        _fly(NFZ_AHEAD, max_s="1", extra=("--clock-start", "12 Oct 2026"))
    except ValueError as exc:
        assert "ISO 8601" in str(exc)
    else:
        raise AssertionError("an unreadable --clock-start was flown")


# =========================================================================== #
# (2) the follow mission and its CSP
# =========================================================================== #

def test_a_follow_mission_compiles_the_csp_of_generation_zero():
    """Every run used to record csp_failed 'no mission given'. The mission is
    the follow (task text from --object), from the scene's spawn point, with
    the start itself as the target proxy - never the subject's truth."""
    rc, run, link, _ = _fly(NFZ_AHEAD, max_s="1", extra=("--object", "a red car"))
    assert rc == 0
    ev = _events(run)
    assert not [e for e in ev if e["kind"] == "csp_failed"], \
        [e for e in ev if e["kind"] == "csp_failed"]
    assert (run / "csp_g0.json").is_file() and (run / "prompt.yaml").is_file()
    assert "follow a red car" in (run / "prompt.yaml").read_text(encoding="utf-8")
    m = _json(run, "metrics.json")
    mis = m["mission"]
    assert mis["task"] == "follow a red car"
    # robot origin "35.0 -20.0 -2.0" in scene_citylife_ardupilot.jsonc
    assert mis["start"] == [35.0, -20.0] and mis["target"] == mis["start"], mis
    assert "proxy" in mis["target_source"] and "spawn" in mis["start_source"]
    assert m["generations"][0]["csp"] is True
    check = next(e for e in ev if e["kind"] == "mission_start_check")
    assert check["gap_m"] < 0.05, check
    # The CSP ranks the zone by its distance from the start: 12 m to its margin.
    csp = _json(run, "csp_g0.json")
    assert any("nfz-ahead" in json.dumps(c) for c in csp["P0_constraints"]), csp["P0_constraints"]


def test_the_spawn_point_is_moved_into_a_policy_origin_frame():
    """With a policy origin the Shield's frame is not the scene's: the spawn
    point (scene NED about the home-geo-point) is projected into it."""
    scene = rail.load_jsonc(rail.SCENE_TEMPLATE)
    home = rail.home_geo_point(scene)
    same = ap.FrameAnchor(home[0], home[1], home[2], "scene.home-geo-point")
    assert pn.scene_spawn_xy(scene, same)[0] == (35.0, -20.0)
    # An origin 100 m north and 50 m west of the scene's home.
    lat, lon = same.to_latlon(100.0, -50.0)
    other = ap.FrameAnchor(lat, lon, None, "policy.origin")
    (x, y), src = pn.scene_spawn_xy(scene, other)
    assert abs(x - (35.0 - 100.0)) < 0.05 and abs(y - (-20.0 + 50.0)) < 0.05, (x, y)
    assert "policy.origin" in src
    assert pn.scene_spawn_xy(None, None)[0] == (0.0, 0.0)
    m = pn.follow_mission("a person", (1.0, 2.0), 8.0, 3.0)
    assert (m.start_x, m.start_y, m.target_x, m.target_y) == (1.0, 2.0, 1.0, 2.0)
    assert m.task_text == "follow a person" and m.cruise_alt_m == 8.0


# =========================================================================== #
# (3) --dev-yaml
# =========================================================================== #

def test_a_yaml_policy_flies_only_under_dev_yaml():
    try:
        _fly(NFZ_AHEAD, max_s="1", dev_yaml=False)
    except SystemExit as exc:
        assert "--dev-yaml" in str(exc) and "build_flight_bundles" in str(exc), exc
    else:
        raise AssertionError("a YAML policy flew without --dev-yaml")
    rc, run, _, _ = _fly(NFZ_AHEAD, max_s="1")
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    assert m["policy_source"]["dev_yaml"] is True and m["policy_source"]["signature"] == "unsigned"
    assert not kpi["kpi_grade"] and any("--dev-yaml" in r for r in kpi["kpi_grade_reasons"])
    # No policy option: the default policy's signed bundle, or a refusal.
    args = pn.build_parser().parse_args(["--network", "mirrored"])
    missing = _tmp() / "follow-car-citylife-v0.1.0.tar.gz"
    try:
        pn.resolve_policy(args, default_bundle=missing)
    except ValueError as exc:
        assert "--dev-yaml" in str(exc) and "build_flight_bundles" in str(exc)
    else:
        raise AssertionError("no policy and no bundle was accepted")
    missing.write_bytes(b"stand-in")
    b, y, rec = pn.resolve_policy(args, default_bundle=missing)
    assert (b, y, rec) == (str(missing), None, {"default_bundle": True})
    args = pn.build_parser().parse_args(["--network", "mirrored", "--dev-yaml"])
    b, y, rec = pn.resolve_policy(args, default_bundle=_tmp() / "none.tar.gz")
    assert b is None and Path(y) == pn.DEFAULT_POLICY and rec["dev_yaml"] is True
    # The dry run names the refusal the run would make.
    old = pn.flight_bundle_path
    pn.flight_bundle_path = lambda p, bundles_dir=None: missing.with_name("absent.tar.gz")
    try:
        rep = pn.dry_run(pn.build_parser().parse_args(["--dry-run", "--network",
                                                       "mirrored"]), _tmp())
    finally:
        pn.flight_bundle_path = old
    assert not rep["ok"] and any("--dev-yaml" in p for p in rep["problems"]), rep["problems"]


def _bundle_of(yaml_text: str, bundles: Path) -> tuple[Path, Path]:
    """(yaml, bundle): a policy file and its UNSIGNED bundle, named as
    tools/build_flight_bundles.py names the signed one. Only the policy hash
    inside is read by the staleness check, so no signing key is needed."""
    from guardrail import load_policy
    from guardrail.bundle import write_bundle
    from guardrail.manifest import flight_bundle_path
    y = _tmp() / "policy.yaml"
    y.write_text(yaml_text, encoding="utf-8")
    pol = load_policy(y)
    b = flight_bundle_path(pol, bundles)
    write_bundle(pol, b, signer=None)
    return y, b


EDITED = NFZ_AHEAD.replace("margin_m: 1.0", "margin_m: 1.5")   # same id and version


def test_a_stale_default_bundle_is_refused_naming_both_hashes():
    """Bundles are named by policy id and version only, and bundles/ is a
    gitignored build product: a YAML edited without a version bump kept
    its stale bundle's name, and the node's no-option default flew the old
    rules (review 2026-10-13: the hashes differed, nothing was said)."""
    from guardrail.manifest import flight_bundle_status, flight_policy_choice
    bundles = _tmp()
    y, b = _bundle_of(NFZ_AHEAD, bundles)
    st = flight_bundle_status(y, bundles_dir=bundles)
    assert st["present"] and st["matches"] is True and st["bundle_hash"] == st["yaml_hash"]
    assert flight_policy_choice(None, None, dev_yaml=False, default_yaml=y,
                                default_bundle=b) == (str(b), None, {"default_bundle": True})
    y.write_text(EDITED, encoding="utf-8")
    st = flight_bundle_status(y, bundles_dir=bundles)
    assert st["matches"] is False and st["bundle_hash"] != st["yaml_hash"], st
    try:
        flight_policy_choice(None, None, dev_yaml=False, default_yaml=y, default_bundle=b)
    except ValueError as exc:
        assert "stale" in str(exc) and st["bundle_hash"] in str(exc) \
            and st["yaml_hash"] in str(exc), exc
    else:
        raise AssertionError("a stale default bundle was flown")
    # The perception node's own no-option default goes through the same check.
    old = pn.DEFAULT_POLICY
    pn.DEFAULT_POLICY = y
    try:
        args = pn.build_parser().parse_args(["--network", "mirrored"])
        T._raises(pn.resolve_policy, args, default_bundle=b, contains="stale")
    finally:
        pn.DEFAULT_POLICY = old
    # An unreadable bundle is not judged here: the flight loader refuses it.
    junk = _tmp() / "pas-ri-nfz-v0.1.0.tar.gz"
    junk.write_bytes(b"stand-in")
    assert flight_bundle_status(y, bundle=junk)["matches"] is None


def _pwsh():
    return shutil.which("pwsh") or shutil.which("powershell")


def _policy_args(yaml: Path, bundles: Path) -> tuple[str, str]:
    """Run the launcher's own Get-PolicyArgs (cut out of the .ps1) under
    PowerShell: (the argument list joined by '|', what it said)."""
    src = (ROOT / "scripts" / "run_pas_ardupilot.ps1").read_text(encoding="utf-8")
    fn = src[src.index("function Get-PolicyArgs"):src.index("# ---- WSL2 networking")]
    ps = "\n".join([
        '$ErrorActionPreference = "Stop"',
        f"$Root = '{ROOT}'", f"$Py = '{sys.executable}'", f"$PolicyFile = '{yaml}'",
        'function Say($m) { [Console]::Error.WriteLine("SAY $m") }',
        'function Warn($m) { [Console]::Error.WriteLine("WARN $m") }',
        fn, f"(Get-PolicyArgs -BundlesDir '{bundles}') -join '|'", ""])
    f = _tmp() / "policy_args.ps1"
    f.write_text(ps, encoding="utf-8")
    r = subprocess.run([_pwsh(), "-NoProfile", "-NonInteractive", "-File", str(f)],
                       capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stderr
    return r.stdout.strip().splitlines()[-1], r.stderr


def test_the_launcher_flies_the_signed_bundle_or_says_dev_yaml():
    """The launcher's Get-PolicyArgs, RUN (the old test only read its text,
    so an inverted Test-Path passed): the bundle when it is there and still
    the YAML's policy; otherwise the YAML under --dev-yaml, saying why - a
    stale bundle by both hashes."""
    src = (ROOT / "scripts" / "run_pas_ardupilot.ps1").read_text(encoding="utf-8")
    node = src[src.index('"node" {'):src.index('"g1" {')]
    assert "$a += Get-PolicyArgs" in node and '@("--policy", $PolicyFile)' not in node
    check = src[src.index('if ($Step -eq "check")'):src.index('if ($Step -eq "g1")')]
    assert "Get-PolicyArgs" in check
    if _pwsh() is None:
        return SKIP
    bundles = _tmp()
    y, b = _bundle_of(NFZ_AHEAD, bundles)
    got, said = _policy_args(y, bundles)
    assert got.split("|")[0] == "--bundle" and Path(got.split("|")[1]) == b, (got, said)
    y.write_text(EDITED, encoding="utf-8")
    got, said = _policy_args(y, bundles)
    assert got.split("|") == ["--policy", str(y), "--dev-yaml"], got
    assert "STALE" in said and "--dev-yaml" in said, said
    got, said = _policy_args(y, _tmp())
    assert got.split("|") == ["--policy", str(y), "--dev-yaml"], got
    assert "no signed bundle" in said, said


# =========================================================================== #
# (4) the GeoFence zones
# =========================================================================== #

FENCED = """policy_id: pas-ri-fenced
version: 0.1.0
constraints:
  - id: nfz-ahead
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: 48, y: -30 }
      - { x: 60, y: -30 }
      - { x: 60, y: -10 }
      - { x: 48, y: -10 }
    altitude_floor_m: 0
    altitude_ceiling_m: 100
    margin_m: 1.0
  - id: nfz-pad
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: 33, y: -22 }
      - { x: 37, y: -22 }
      - { x: 37, y: -18 }
      - { x: 33, y: -18 }
    altitude_floor_m: 0
    altitude_ceiling_m: 100
    margin_m: 0.5
  - id: nfz-window
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: -60, y: 60 }
      - { x: -50, y: 60 }
      - { x: -50, y: 70 }
      - { x: -60, y: 70 }
    altitude_floor_m: 0
    altitude_ceiling_m: 100
    margin_m: 1.0
    valid_time:
      recurrence:
        start_time: "08:00"
        end_time: "09:00"
  - id: nfz-low
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: repair
    vertices:
      - { x: -60, y: -60 }
      - { x: -50, y: -60 }
      - { x: -50, y: -50 }
      - { x: -60, y: -50 }
    altitude_floor_m: 0
    altitude_ceiling_m: 10
    margin_m: 1.0
"""

# FENCE_TYPE 5 = altitude (1) + polygon (4), circle bit (2) off: only the
# polygon bit can make zones_enforced True here.
FENCE_VALS = {"FENCE_ENABLE": 1.0, "FENCE_TYPE": 5.0, "FENCE_ACTION": 1.0,
              "FENCE_ALT_MAX": 20.0, "FENCE_RADIUS": 400.0, "FENCE_MARGIN": 2.0,
              "AVOID_ENABLE": 0.0}


class FenceLink(T.FakeLink):
    """A link that reads the fence parameters, uploads zones and runs the
    pre-arm step before it arms, logging the order. With `previous` it can
    also read the fence list it held before (a count)."""

    def __init__(self, *a, fence_vals=None, ok=True, no_fix_before_arm=False, **k):
        super().__init__(*a, **k)
        self.fence_vals = dict(FENCE_VALS if fence_vals is None else fence_vals)
        self.ok, self.log, self.uploads, self.timeouts = ok, [], [], []
        self.no_fix_before_arm, self.armed_yet = no_fix_before_arm, False

    def autopilot_fence(self):
        return {**{n: self.fence_vals.get(n) for n in ap.FENCE_PARAMS}, "source": "fake"}

    def state(self, anchor):
        if self.no_fix_before_arm and not self.armed_yet:
            return None
        return super().state(anchor)

    def bring_up(self, alt, before_arm=None):
        self.pump()
        if before_arm is not None:
            before_arm()
        self.log.append("arm")
        self.armed_yet = True
        super().bring_up(alt)

    def upload_fence(self, items, timeout=10.0):
        self.log.append("upload")
        self.uploads.append(items)
        self.timeouts.append(timeout)
        n = len(items)
        return {"via": "fake", "items": n, "transferred": n if self.ok else 0,
                "ack": 0 if self.ok else 1, "read_back": n if self.ok else None,
                "read_back_match": self.ok, "success": self.ok,
                "why": None if self.ok else "ACK type 1", "upload_ms": 1.0}


class FenceReadLink(FenceLink):
    def __init__(self, *a, previous=0, **k):
        super().__init__(*a, **k)
        self.previous = previous

    def read_fence(self, timeout=5.0):
        self.log.append("read")
        return {"via": "fake", "count": self.previous, "items": None, "why": None}


def test_the_clock_proof_zones_go_up_before_arming_and_a_pad_zone_once_clear():
    """The rail only read the fence parameters back: the backstop held an
    altitude and a 400 m circle, never one of the policy's zones."""
    rc, run, link, _ = _fly(FENCED, shield="off", max_s="4", link_cls=FenceLink)
    assert rc == 0
    assert link.log[:2] == ["upload", "arm"], link.log
    first, second = link.uploads[0], link.uploads[1]
    assert {it["id"] for it in first} == {"nfz-ahead"} and len(first) == 4, first
    assert {it["id"] for it in second} == {"nfz-ahead", "nfz-pad"} and len(second) == 8
    # Projected about the frame's anchor, the projection the EKF is read with.
    a = ap.FrameAnchor(*ANCHOR, "scene.home-geo-point")
    lat, lon = a.to_latlon(48.0, -30.0)
    v0 = first[0]
    assert abs(v0["x_lat"] - lat) < 1e-9 and abs(v0["y_long"] - lon) < 1e-9
    assert v0["command"] == 5002 and v0["param1"] == 4.0
    m = _json(run, "metrics.json")
    ups = m["fence_uploads"]
    assert m["fence_upload_phase"] == "before arming" and len(ups) == 2
    skipped = {s["id"]: s for s in ups[0]["skipped"]}
    assert skipped["nfz-window"].get("time_controlled") is True
    assert "does not cover" in skipped["nfz-low"]["why"]
    assert skipped["nfz-pad"]["deferred"] is True
    assert ups[0]["zones_over_home"] == ["nfz-pad"]
    assert "now clear" in ups[1]["reason"] and ups[1]["success"] is True
    # The second upload was made in flight, with the short timeout.
    assert link.timeouts == [10.0, pn.MIDFLIGHT_FENCE_TIMEOUT_S], link.timeouts
    assert ups[1]["in_flight"] is True and ups[1]["ticks_missed"] == 0
    assert m["fences_still_deferred"] == []
    assert m["autopilot_fence"]["zones_enforced"] is True
    kinds = [e["kind"] for e in _events(run)]
    assert kinds.count("fence_upload") == 2


def test_zones_enforced_is_fence_type_s_polygon_bit_alone():
    """FENCE_TYPE 3 (altitude + circle) holds no uploaded zone; 4 and 5 do.
    A fixture of 7 (every bit) could not tell the polygon bit from the
    circle bit."""
    from guardrail import load_policy
    f = _tmp() / "p.yaml"
    f.write_text(FENCED, encoding="utf-8")
    pol = load_policy(f)

    class L:
        def __init__(self, t):
            self.t = t

        def autopilot_fence(self):
            return {**FENCE_VALS, "FENCE_TYPE": self.t, "source": "fake"}
    got = {t: pn.autopilot_fence_record(L(t), pol, 8.0)["zones_enforced"]
           for t in (3.0, 4.0, 5.0, 2.0)}
    assert got == {3.0: False, 4.0: True, 5.0: True, 2.0: False}, got
    assert pn.autopilot_fence_record(L(None), pol, 8.0)["zones_enforced"] is None


class StubRec:
    def __init__(self):
        self.events = []

    def event(self, kind, **k):
        self.events.append((kind, k))


def _backstop(policy_text=None, ap_fence=None, link=None):
    from guardrail import load_policy
    f = _tmp() / "p.yaml"
    f.write_text(policy_text or FENCED, encoding="utf-8")
    link = link or FenceLink(T.Clock(), ap.FrameAnchor(*ANCHOR, "scene"))
    bs = pn.FenceBackstop(load_policy(f), link, StubRec(),
                          ap.FrameAnchor(*ANCHOR, "scene.home-geo-point"),
                          ap_fence or {"FENCE_ALT_MAX": 20.0, "FENCE_MARGIN": 2.0,
                                       "zones_enforced": True})
    return bs, link


def _ids(items):
    return sorted({it["id"] for it in items})


def test_a_re_upload_keeps_an_enforced_zone_the_aircraft_is_next_to():
    """REVIEW (2026-10-13, confirmed by a probe). Each upload replaces the
    whole fence list. When the held-back pad zone cleared with the aircraft
    1 m outside nfz-ahead, the re-upload held nfz-ahead back - it was near -
    and so took it OFF the fence exactly where the backstop is for."""
    bs, link = _backstop()
    bs.publish((35.0, -20.0), "before arming")
    assert _ids(link.uploads[0]) == ["nfz-ahead"] and bs.deferred == ["nfz-pad"]
    out = bs.recheck((47.0, -20.0), [], t=1.0)
    assert _ids(link.uploads[1]) == ["nfz-ahead", "nfz-pad"], _ids(link.uploads[1])
    assert out["kept_near"] == ["nfz-ahead"] and out["deferred"] == []
    assert sorted(bs.enforced) == ["nfz-ahead", "nfz-pad"]
    # Even with the aircraft INSIDE an enforced zone (the Shield failed), a
    # later upload keeps it: dropping it would hide the breach.
    assert pn.defer_near(bs.zones, (50.0, -20.0), 2.0, enforced=bs.enforced)[1] == []
    # A zone not yet on the fence, or reshaped, is still held back near it.
    moved = dict(bs.enforced["nfz-ahead"], vertices=[[48, -31], [60, -31], [60, -10], [48, -10]])
    assert pn.defer_near([moved], (47.0, -20.0), 2.0, enforced=bs.enforced)[0] == []
    # The ROS 2 adapter's selection takes the same set (ros2_shield_node).
    import mavlink_adapter_node as mad
    fences, skipped = mad.policy_fences(bs.policy, column_m=20.0, position=(47.0, -20.0),
                                        enforced={"nfz-ahead": bs.enforced["nfz-ahead"]})
    assert [f["id"] for f in fences] == ["nfz-ahead", "nfz-pad"], (fences, skipped)
    fences, _ = mad.policy_fences(bs.policy, column_m=20.0, position=(47.0, -20.0))
    assert [f["id"] for f in fences] == ["nfz-pad"], \
        "without the enforced set the adapter holds nfz-ahead back"
    src = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "enforced=self.fences_enforced" in src and "self.fences_enforced = {" in src


def test_a_held_back_zone_waits_for_the_shield_the_margin_and_a_known_position():
    # Clear of the pad by geometry, but the Shield still finds the position
    # unsafe under it: not yet.
    bs, link = _backstop()
    bs.publish((35.0, -20.0), "before arming")
    assert bs.recheck((45.0, -20.0), ["nfz-pad"]) is None and len(link.uploads) == 1
    assert bs.recheck((45.0, -20.0), []) is not None and len(link.uploads) == 2
    # The autopilot's own FENCE_MARGIN, not the default 2 m: 3 m from the pad
    # is clear of a 2 m margin and not of a 5 m one.
    for margin, uploaded in ((5.0, False), (2.0, True)):
        bs, link = _backstop(ap_fence={"FENCE_ALT_MAX": 20.0, "FENCE_MARGIN": margin})
        bs.publish((35.0, -20.0), "before arming")
        bs.recheck((40.0, -20.0), [])
        assert (len(link.uploads) == 2) is uploaded, (margin, link.uploads)
    # An unknown position holds back EVERY zone; the list is cleared, and the
    # zones go up once the aircraft is placed.
    bs, link = _backstop()
    out = bs.publish(None, "before arming")
    assert link.uploads == [[]] and bs.deferred == ["nfz-ahead", "nfz-pad"], (link.uploads, out)
    bs.recheck((35.0, -20.0), [])
    assert _ids(link.uploads[1]) == ["nfz-ahead"] and bs.deferred == ["nfz-pad"]


def test_with_no_zone_the_list_is_cleared_unless_known_empty_and_a_failure_is_final():
    """REVIEW (2026-10-13). With no zone to upload nothing was sent, so a
    list left by an earlier run or another policy stayed enforced under
    FENCE_TYPE's polygon bit. The ROS 2 adapter pushes the empty list; so
    does this node now - unless the list is known empty - and the list it
    replaces (a ground station's) is recorded first."""
    rc, run, link, _ = _fly(T.CAP_POLICY, max_s="1", link_cls=FenceLink)
    assert link.uploads == [[]], link.uploads
    up = _json(run, "metrics.json")["fence_upload"]
    assert up["sent"] is True and up["success"] is True and "CLEARED" in up["why"], up
    assert "cannot read" in up["previous"]["why"]
    rc, run, link, _ = _fly(T.CAP_POLICY, max_s="1", link_cls=FenceReadLink)
    assert link.uploads == [] and link.log[0] == "read"
    up = _json(run, "metrics.json")["fence_upload"]
    assert up["sent"] is False and "already empty" in up["why"], up
    rc, run, link, _ = _fly(FENCED, max_s="1", link_cls=FenceReadLink,
                            link_kw={"previous": 4})
    assert link.log[:3] == ["read", "upload", "arm"], link.log
    first = _json(run, "metrics.json")["fence_uploads"][0]
    assert first["previous"]["count"] == 4 and _ids(link.uploads[0]) == ["nfz-ahead"]
    rc, run, link, _ = _fly(FENCED, shield="off", max_s="4", link_cls=FenceLink,
                            link_kw={"ok": False})
    assert len(link.uploads) == 1, "a refused upload was retried on later ticks"
    m = _json(run, "metrics.json")
    assert m["fence_upload"]["success"] is False and m["fences_still_deferred"] == ["nfz-pad"]
    # Unread FENCE_ALT_MAX: no zone can be judged, so none is exported, and
    # the list is cleared rather than left to another policy's zones.
    rc, run, link, _ = _fly(FENCED, max_s="1", link_cls=FenceLink,
                            link_kw={"fence_vals": {**FENCE_VALS, "FENCE_ALT_MAX": None}})
    assert link.uploads == [[]]
    rc, run, link, _ = _fly(FENCED, max_s="1", link_cls=FenceReadLink,
                            extra=("--no-fence-upload",))
    assert link.uploads == [] and "read" not in link.log and "--no-fence-upload" in \
        _json(run, "metrics.json")["fence_upload"]["why"]


class SlowFenceLink(FenceLink):
    """An autopilot that never answers an upload made in flight: it costs
    the caller both phases of the timeout it passed."""

    def upload_fence(self, items, timeout=10.0):
        if not self.armed_yet:
            return super().upload_fence(items, timeout)
        self.uploads.append(items)
        self.timeouts.append(timeout)
        self.clock.t += 2 * timeout
        return {"via": "fake", "items": len(items), "transferred": 0, "ack": None,
                "read_back": None, "read_back_match": None, "success": False,
                "why": "timed out after 0/8 items", "upload_ms": 2000.0 * timeout}


def test_an_in_flight_upload_is_bounded_and_the_loop_keeps_ticking():
    """REVIEW (2026-10-13). The deferred zone's upload runs inside the 10 Hz
    loop; with the links' 10 s per phase a silent autopilot stopped the
    Shield and the setpoints for up to 20 s. In flight each phase now gets
    MIDFLIGHT_FENCE_TIMEOUT_S, and the ticks it cost are recorded."""
    rc, run, link, _ = _fly(FENCED, shield="off", max_s="4", link_cls=SlowFenceLink)
    assert rc == 0
    assert link.timeouts == [10.0, pn.MIDFLIGHT_FENCE_TIMEOUT_S], link.timeouts
    m = _json(run, "metrics.json")
    up = m["fence_uploads"][1]
    assert up["in_flight"] and up["success"] is False and up["timeout_s"] == 0.3
    assert up["ticks_missed"] == 6 and up["tick_budget_ms"] == 100, up
    rows = _rows(run)
    gaps = [b["t"] - a["t"] for a, b in zip(rows, rows[1:])]
    assert max(gaps) < 0.75 and sum(1 for g in gaps if g > 0.15) == 1, gaps
    assert rows[-1]["t"] > 3.5, "the loop stopped after the upload"
    assert len(link.uploads) == 2, "a failed in-flight upload was retried"


class DriftLink(FenceLink):
    """After the autopilot takes the aircraft (RTL) it flies it east, clear
    of the pad, on its own."""

    def pump(self):
        if self.snap.mode == "RTL" and self.t_last is not None:
            self.x += 4.0 * (self.clock.t - self.t_last)
        super().pump()


def test_no_zone_goes_up_once_the_autopilot_has_the_aircraft():
    rc, run, link, _ = _fly(FENCED, shield="off", max_s="4", link_cls=DriftLink,
                            link_kw={"takeover_at": (0.2, "RTL")})
    rows = _rows(run)
    assert rows[-1]["x"] > 40.0, "the aircraft never cleared the pad: the test proves nothing"
    assert _json(run, "metrics.json")["autopilot_took_over"]["mode"] == "RTL"
    assert len(link.uploads) == 1, "a zone was uploaded while the autopilot had the aircraft"


def test_both_links_run_the_pre_arm_step_after_the_fix_and_before_guided():
    """The zones must be on the fence before the aircraft arms, and the
    position known when they are chosen (a zone over the pad is held back):
    after the EKF / global-position wait, before GUIDED."""
    link, fm = _pml(None)
    log = []
    link.identify = lambda timeout=5.0: log.append("identify")
    link.wait = lambda pred, timeout=0.0, poll=0.05: log.append("ekf") or True
    link.set_mode = lambda name, timeout=10.0: log.append(name) or True
    link.arm = lambda timeout=60.0: log.append("arm") or True
    link.takeoff = lambda alt, timeout=60.0: log.append("takeoff") or True
    link.bring_up(8.0, before_arm=lambda: log.append("before_arm"))
    assert log[:6] == ["identify", "ekf", "before_arm", "GUIDED", "arm", "takeoff"], log
    mv, node = _mavros()
    log = []
    mv.identify = lambda timeout=20.0: log.append("identify")
    mv.wait = lambda pred, timeout=0.0, poll=0.05: log.append("wait") or True
    mv._call = lambda client, req, timeout=5.0: log.append(
        getattr(req, "custom_mode", None) or ("arm" if getattr(req, "value", None)
                                              else type(req).__name__))
    mv.bring_up(8.0, timeout=1.0, before_arm=lambda: log.append("before_arm"))
    # connected, identify, stream rate, global position, THEN the pre-arm step.
    assert log[:6] == ["wait", "identify", "SimpleNamespace", "wait", "before_arm",
                       "GUIDED"], log


def test_a_monitor_only_zone_is_not_put_on_the_autopilot_fence():
    """The Shield records an entry into a monitor_only zone and lets it
    through; an autopilot fence around it would RTL that flight."""
    from guardrail import load_policy
    from guardrail.fence_export import policy_zones
    f = _tmp() / "p.yaml"
    f.write_text(NFZ_AHEAD.replace("violation_action: repair",
                                   "violation_action: monitor_only"), encoding="utf-8")
    zones, skipped = policy_zones(load_policy(f), column_m=20.0)
    assert zones == [] and "monitor_only" in skipped[0]["why"], (zones, skipped)
    f.write_text(NFZ_AHEAD, encoding="utf-8")
    assert [z["id"] for z in policy_zones(load_policy(f), column_m=20.0)[0]] == ["nfz-ahead"]


class FenceMaster:
    """ArduPilot's side of the mission protocol for the FENCE list, with the
    ways it can go wrong: a read-back item that differs (`corrupt` names the
    field), a read-back count one short, an ACCEPTED ack before every item
    went (`early_ack` after that many), a request for an item that does not
    exist, and decoys that must be ignored - a MISSION_REQUEST_INT for the
    AUTO mission (mission_type 0) and one from another system (Mission
    Planner, 255) - before every real request. `stored` is the list it holds
    before an upload."""
    WIRE_PROTOCOL_VERSION = "2.0"

    def __init__(self, corrupt=None, ack_type=0, short=False, early_ack=None,
                 seq_oob=False, decoys=False, stored=None):
        self.mav = T.FakeMav(self._respond)
        self.inbox, self.stored, self.n = [], list(stored or []), None
        self.corrupt, self.ack_type, self.short = corrupt, ack_type, short
        self.early_ack, self.seq_oob, self.decoys = early_ack, seq_oob, decoys
        self.target_system = self.target_component = 1

    def _request(self, seq):
        if self.decoys:
            self.inbox.append(T.Msg("MISSION_REQUEST_INT", seq=self.n + 5, mission_type=0))
            self.inbox.append(T.Msg("MISSION_REQUEST_INT", src=255, seq=self.n + 6,
                                    mission_type=1))
        self.inbox.append(T.Msg("MISSION_REQUEST_INT", seq=seq, mission_type=1))

    def _respond(self, name, a):
        if name == "mission_count_send":
            self.n, self.stored = a[2], [None] * a[2]
            if not self.n:
                self.inbox.append(T.Msg("MISSION_ACK", type=0, mission_type=1))
            else:
                self._request(self.n if self.seq_oob else 0)
        elif name == "mission_item_int_send":
            seq = a[2]
            self.stored[seq] = (a[4], a[11], a[12], a[7])
            if self.early_ack is not None and seq + 1 >= self.early_ack:
                self.inbox.append(T.Msg("MISSION_ACK", type=0, mission_type=1))
            elif seq + 1 < self.n:
                self._request(seq + 1)
            else:
                self.inbox.append(T.Msg("MISSION_ACK", type=self.ack_type, mission_type=1))
        elif name == "mission_request_list_send":
            self.inbox.append(T.Msg("MISSION_COUNT", mission_type=1,
                                    count=len(self.stored) - (1 if self.short else 0)))
        elif name == "mission_request_int_send":
            cmd, x, y, p1 = self.stored[a[2]]
            c = self.corrupt
            self.inbox.append(T.Msg("MISSION_ITEM_INT", seq=a[2], command=cmd,
                                    x=x + (1 if c == "x" else 0), y=y + (1 if c == "y" else 0),
                                    param1=p1 + (0.5 if c == "param1" else 0.0),
                                    mission_type=1))

    def recv_match(self, type=None, blocking=False, timeout=None):
        return self.inbox.pop(0) if self.inbox else None


def _pml(master, params=None):
    fm = T.FakeMavutil([T.AP_HB], params=params or {})
    link = T._link(fm)
    link.connect(timeout=5)
    if master is not None:
        link.m = master
    return link, fm


def test_the_pymavlink_link_uploads_the_fence_list_and_reads_it_back():
    import mavlink_adapter_node as mad
    a = ap.FrameAnchor(*ANCHOR, "scene")
    items = mad.fence_items([{"id": "z", "kind": "polygon",
                              "vertices": [[48, -30], [60, -30], [60, -10], [48, -10]]},
                             {"id": "c", "kind": "circle", "center": [0, 0],
                              "radius_m": 7.5}], a.lat, a.lon)
    fmast = FenceMaster(decoys=True)
    link, _ = _pml(fmast)
    res = link.upload_fence(items, timeout=2)
    assert res["success"] and res["read_back"] == 5 and res["read_back_match"], res
    sent = fmast.mav.sent
    assert all(k.get("mission_type") == ap.MAV_MISSION_TYPE_FENCE for n, _, k in sent
               if n.startswith("mission_")), sent
    assert [s[0] for s in sent if s[0] == "mission_item_int_send"].__len__() == 5
    # What the autopilot STORED is where the zones are: every vertex and the
    # circle's centre as degrees x 1e7, with the command and param1.
    for (cmd, x, y, p1), it in zip(fmast.stored, items):
        assert (cmd, x, y) == (it["command"], round(it["x_lat"] * 1e7),
                               round(it["y_long"] * 1e7)), (cmd, x, y, it)
        assert abs(p1 - it["param1"]) < 1e-9
    lat, lon = a.to_latlon(48.0, -30.0)
    assert abs(fmast.stored[0][1] / 1e7 - lat) < 1e-7 and abs(fmast.stored[0][2] / 1e7 - lon) < 1e-7
    assert fmast.stored[4][0] == 5004 and abs(fmast.stored[4][3] - 7.5) < 1e-9
    # Every way the autopilot can disagree is a refusal, said.
    for kw, why in (({"corrupt": "x"}, "differ"), ({"corrupt": "y"}, "differ"),
                    ({"corrupt": "param1"}, "differ"), ({"short": True}, "read back 4"),
                    ({"ack_type": 1}, "ACK type 1"), ({"early_ack": 3}, "after 3/5"),
                    ({"seq_oob": True}, "asked for item 5 of 5")):
        link, _ = _pml(FenceMaster(**kw))
        res = link.upload_fence(items, timeout=2)
        assert not res["success"] and why in res["why"], (kw, res)
    # An empty list clears the autopilot's (an ACK straight after the count).
    fm0 = FenceMaster()
    link, _ = _pml(fm0)
    res = link.upload_fence([], timeout=2)
    assert res["success"] and res["read_back"] == 0 and res["items"] == 0, res
    # The list held BEFORE an upload, read back for the record.
    held = [(5002, 356895000, 1396917000, 4.0)] * 4
    link, _ = _pml(FenceMaster(stored=held))
    prev = link.read_fence(timeout=2)
    assert prev["count"] == 4 and len(prev["items"]) == 4 and prev["why"] is None, prev
    assert abs(prev["items"][0]["lat"] - 35.6895) < 1e-9 and prev["items"][0]["command"] == 5002
    v1 = FenceMaster()
    v1.WIRE_PROTOCOL_VERSION = "1.0"
    link, _ = _pml(v1)
    res = link.upload_fence(items, timeout=2)
    assert not res["success"] and "MAVLink 1" in res["why"] and v1.mav.sent == [], res
    assert link.read_fence(timeout=1)["count"] is None and v1.mav.sent == []


FAKE_MSGS = SimpleNamespace(**{**vars(T.FAKE_MSGS),
                               "RCIn": object, "OverrideRCIn": SimpleNamespace,
                               "Waypoint": SimpleNamespace, "WaypointPush": T._SRV,
                               "WaypointPull": T._SRV})


def _mavros():
    node = T.FakeNode()
    return ml.MavrosLink(node=node, rclpy=T.FAKE_RCLPY, msgs=FAKE_MSGS,
                         clock=T.Clock(0.01)), node


def test_the_mavros_link_pushes_the_fence_and_reads_it_back():
    link, node = _mavros()
    items = [{"frame": 3, "command": 5002, "param1": 3.0, "x_lat": 35.0 + i * 1e-4,
              "y_long": 139.0, "id": "z"} for i in range(3)]
    res = link.upload_fence(items, timeout=0.1)
    assert not res["success"] and "not available" in res["why"], res
    node.clients["/mavros/geofence/push"].response = SimpleNamespace(success=True,
                                                                     wp_transfered=3)
    node.clients["/mavros/geofence/pull"].response = SimpleNamespace(success=True,
                                                                     wp_received=3)
    res = link.upload_fence(items, timeout=0.1)
    assert res["success"] and res["read_back"] == 3, res
    req = node.clients["/mavros/geofence/push"].requests[-1]
    assert req.start_index == 0 and len(req.waypoints) == 3
    wp = req.waypoints[0]
    assert (wp.frame, wp.command, wp.param1, wp.x_lat) == (3, 5002, 3.0, 35.0)
    node.clients["/mavros/geofence/pull"].response = SimpleNamespace(success=True,
                                                                     wp_received=2)
    assert not link.upload_fence(items, timeout=0.1)["success"]
    # MAVROS refusing the push, or the pull, is a failure whatever it counted.
    node.clients["/mavros/geofence/push"].response = SimpleNamespace(success=False,
                                                                     wp_transfered=3)
    node.clients["/mavros/geofence/pull"].response = SimpleNamespace(success=True,
                                                                     wp_received=3)
    res = link.upload_fence(items, timeout=0.1)
    assert not res["success"] and "refused the push" in res["why"], res
    node.clients["/mavros/geofence/push"].response = SimpleNamespace(success=True,
                                                                     wp_transfered=3)
    node.clients["/mavros/geofence/pull"].response = SimpleNamespace(success=False,
                                                                     wp_received=3)
    res = link.upload_fence(items, timeout=0.1)
    assert not res["success"] and "did not read it back" in res["why"], res
    assert link.read_fence(timeout=0.1)["count"] is None
    # The list held before an upload: MAVROS's count of what it pulled.
    node.clients["/mavros/geofence/pull"].response = SimpleNamespace(success=True,
                                                                     wp_received=4)
    assert link.read_fence(timeout=0.1)["count"] == 4
    # An empty push clears the list, as the ROS 2 adapter's does.
    node.clients["/mavros/geofence/pull"].response = SimpleNamespace(success=True,
                                                                     wp_received=0)
    res = link.upload_fence([], timeout=0.1)
    assert res["success"] and node.clients["/mavros/geofence/push"].requests[-1].waypoints == []


def test_the_param_file_enforces_uploaded_zones_and_keeps_g1_s_values():
    """citylife-quad.param against sitl/fence/guardrail_fence.parm: the
    polygon bit on (zones enforced), simple avoidance off (the fence acts on
    a breach only), and every value gate G1 flew unchanged."""
    import mavlink_adapter_node as mad
    ours = mad.read_parm(rail.PARAM_FILE)
    sitl = mad.read_parm(ROOT / "sitl" / "fence" / "guardrail_fence.parm")
    assert int(ours["FENCE_TYPE"]) & 0b111 == 0b111, ours["FENCE_TYPE"]
    assert int(sitl["FENCE_TYPE"]) & 0b100 and int(ours["FENCE_TYPE"]) & 0b100
    for k in ("AVOID_ENABLE", "FENCE_ACTION", "FENCE_MARGIN", "FENCE_ENABLE"):
        assert ours.get(k) == sitl.get(k), (k, ours.get(k), sitl.get(k))
    assert ours["AVOID_ENABLE"] == 0.0
    g1 = {"FENCE_ALT_MAX": 20.0, "FENCE_RADIUS": 400.0, "FENCE_ACTION": 1.0,
          "SCHED_LOOP_RATE": 200.0, "WPNAV_SPEED": 500.0, "RTL_ALT": 1200.0,
          "FRAME_CLASS": 1.0, "FRAME_TYPE": 1.0}
    for k, v in g1.items():
        assert ours.get(k) == v, (k, ours.get(k))


# =========================================================================== #
# (5) the subject's ground truth
# =========================================================================== #

STANDOFF_ANY = """policy_id: pas-ri-standoff
version: 0.1.0
constraints:
  - id: standoff-any
    type: subject_standoff
    constraint_type: hard
    priority: P0
    violation_action: repair
    subject_class: "*"
    min_range_m: 5.0
"""


class TruthCamera(T.FakeCamera):
    """A simulator that knows where the car is: `value` (fixed), a NaN, an
    error, or 2 m east of the aircraft (the default). Asked for the aircraft
    itself (pn.AIRCRAFT_ACTOR: the truth's frame check), it answers where
    the link has it; those reads are counted apart (`aircraft_reads`), so
    `names`, `script` and `sent_at_read` stay the subject's."""

    def __init__(self, link, log=None, value=None, raises=None, script=()):
        super().__init__(link, log)
        self.value, self.raises, self.names = value, raises, []
        # Per call, before the default: a metres-east offset, or an error.
        self.script = list(script)
        # How many setpoints had gone out when each pose was read.
        self.sent_at_read = []
        self.aircraft_reads = 0

    def object_xy(self, name):
        if name == getattr(pn, "AIRCRAFT_ACTOR", None):
            self.aircraft_reads += 1
            return (self.link.x, self.link.y)
        self.names.append(name)
        self.sent_at_read.append(len(self.link.sent))
        step = self.script.pop(0) if self.script else None
        if isinstance(step, BaseException):
            raise step
        if step is not None:
            return (self.link.x, self.link.y + step)
        if self.raises is not None:
            raise self.raises
        if self.value is not None:
            return self.value
        return (self.link.x, self.link.y + 2.0)


def test_the_subject_truth_is_logged_and_never_given_to_the_shield():
    """The car 2 m from the aircraft, inside the 5 m P0 ring, while the
    width estimate puts it about 96 m away. The record shows the truth (2 m,
    every read tick in the ring); the Shield, given the estimate, saw no
    stand-off violation."""
    rc, run, link, cam = _fly(STANDOFF_ANY, max_s="2", camera_cls=TruthCamera,
                              extra=("--object", "a red car"))
    assert rc == 0 and set(cam.names) == {"Car_10"}, cam.names
    rows = _rows(run)
    assert rows and all(r["subject_truth"] is not None for r in rows)
    assert all(abs(r["subject_range_m"] - 2.0) < 0.05 for r in rows)
    est = [r for r in rows if r["subject"] is not None]
    assert est and all(r["subject_gap_m"] > 50.0 for r in est), \
        "the estimate sat on the truth: the test cannot tell them apart"
    assert not any(v["rule_id"] == "standoff-any" for r in rows for v in r["violations"]), \
        "the Shield bound the truth's ring: ground truth reached it"
    m = _json(run, "metrics.json")
    st = m["subject_truth"]
    assert st["actor"] == "Car_10" and st["ticks_read"] == len(rows)
    assert st["standoff_ring_m"] == 5.0 and st["frac_in_ring"] == 1.0
    assert abs(st["min_range_m"] - 2.0) < 0.05 and st["estimate_gap_m"]["n"] == len(est)
    assert m["follow_scored"] is True and m["frac_within_30m"] == 1.0
    assert m["standoff_s"] == round(len(rows) * pn.TICK, 2) and m["standoff_min_range_m"] == st["min_range_m"]
    assert st["coverage"] == 1.0 and m["standoff_basis"].startswith("measured")
    # The repo's subject convention (tgt_x / tgt_y) carries the truth, so the
    # KPI's hover null for the follow proxy can be computed beside it.
    assert all(abs(r["tgt_x"] - r["subject_truth"][0]) < 0.01
               and abs(r["tgt_y"] - r["subject_truth"][1]) < 0.01 for r in rows)
    hover = m["null_hover_mission"]
    assert hover["success"] is not None and hover["basis"].startswith("follow proxy"), hover
    # Read AFTER the tick's setpoint went out: never between the action and
    # the autopilot (review 2026-10-13: the RPC sat before Shield.filter).
    assert cam.sent_at_read == list(range(1, len(rows) + 1)), cam.sent_at_read[:5]
    assert st["reader"]["attempts"] == len(rows) and st["reader"]["read_ms"]["n"] == len(rows)
    assert all(r["rule_clock"] for r in rows)
    kpi = _json(run, "kpi.json")
    assert not any("not scored" in w for w in kpi["kpi_grade_reasons"])


def test_a_transient_pose_failure_backs_off_and_never_scores_the_read_ticks_alone():
    """REVIEW (2026-10-13). Three failed reads (0.3 s) stopped the truth for
    the rest of the flight, and the five ticks read before were scored as
    the flight: standoff_s 0.0, follow_scored True, mission success - while
    the car then sat 2 m away. Now the reader backs off and retries, the
    dwell it then sees is reported (a lower bound: the unread ticks may add
    to it), and a partial truth scores nothing else."""
    script = [10.0] * 5 + [TimeoutError("rpc")] * 3
    rc, run, link, cam = _fly(STANDOFF_ANY, max_s="4", camera_cls=TruthCamera,
                              camera_kw={"script": script}, extra=("--object", "a red car"))
    assert rc == 0
    rows = _rows(run)
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    st = m["subject_truth"]
    assert st["reader"]["backoffs"] == 1 and st["reader"]["failures"] == 3, st["reader"]
    # 5 read, 3 failed, 10 skipped, then read again: in the ring from there.
    assert st["ticks_read"] == len(rows) - 13 and st["ticks_in_ring"] == len(rows) - 18, st
    assert any("backing off" in r.get("subject_truth_why", "") for r in rows)
    assert st["coverage"] < pn.TRUTH_MIN_COVERAGE and st["scored"] is False
    assert m["follow_scored"] is False and m["frac_within_30m"] is None
    assert m["standoff_s"] == round((len(rows) - 18) * pn.TICK, 2), m["standoff_s"]
    assert "LOWER BOUND" in m["standoff_basis"]
    assert kpi["mission_success"] is False and "dwell:standoff_s" in kpi["mission_fail_reasons"]
    assert any("not scored" in w for w in kpi["kpi_grade_reasons"]), kpi["kpi_grade_reasons"]


def test_the_truth_reader_counts_failures_in_a_row_and_nan_answers():
    """`errors` is CONSECUTIVE: fail, ok, fail, fail keeps reading. A NaN
    (no actor with that tag) is a failure too: it used to cost an RPC every
    tick for the whole flight."""
    home = ANCHOR
    anchor = ap.FrameAnchor(home[0], home[1], home[2], "scene.home-geo-point")
    cam = SimpleNamespace(calls=0)
    seq = [RuntimeError("x"), (1.0, 2.0), RuntimeError("y"), RuntimeError("z"), (3.0, 4.0)]

    def object_xy(name):
        cam.calls += 1
        step = seq.pop(0)
        if isinstance(step, BaseException):
            raise step
        return step
    cam.object_xy = object_xy
    tr = pn.SubjectTruth(cam, "Car_10", None, home, anchor)
    got = [tr.read() for _ in range(5)]
    assert cam.calls == 5 and got[4]["xy"] == (3.0, 4.0), (cam.calls, got)
    rc, run, link, cam = _fly(STANDOFF_ANY, max_s="3", camera_cls=TruthCamera,
                              camera_kw={"value": (float("nan"), float("nan"))},
                              extra=("--object", "a red car"))
    # Ticks 1-3 fail, 4-13 wait, 14 fails, then a 20-tick wait: 4 reads in 30.
    assert len(cam.names) == 4, len(cam.names)
    assert _json(run, "metrics.json")["subject_truth"]["reader"]["backoffs"] == 2


def test_the_ring_is_strict_and_coverage_decides_what_scores():
    truth = SimpleNamespace(actor="Car_10", source="s", summary=lambda: None)

    def rows(ranges, n):
        out = [{"subject_truth": [0.0, 0.0], "subject_range_m": d} for d in ranges]
        return out + [{"subject_truth": None, "subject_truth_why": "x"}] * (n - len(ranges))
    s = pn.truth_summary(rows([5.0] * 20, 20), truth, 5.0, 0.1)
    assert s["ticks_in_ring"] == 0 and s["standoff_s"] == 0.0 and s["scored"], s
    s = pn.truth_summary(rows([4.9] * 19, 20), truth, 5.0, 0.1)
    assert s["scored"] and s["standoff_s"] == 1.9 and s["frac_within_30m"] == 1.0
    s = pn.truth_summary(rows([9.0] * 18, 20), truth, 5.0, 0.1)
    assert not s["scored"] and s["standoff_s"] is None and s["frac_within_30m"] is None
    assert s["frac_within_30m_of_read"] == 1.0 and "not measured" in s["standoff_basis"]
    s = pn.truth_summary(rows([9.0] * 9 + [3.0] * 9, 20), truth, 5.0, 0.1)
    assert not s["scored"] and s["standoff_s"] == 0.9 and "LOWER BOUND" in s["standoff_basis"]


def test_an_unread_subject_pose_is_none_with_its_reason_never_zero():
    cases = (
        ({"camera_cls": TruthCamera, "camera_kw": {"value": (float("nan"), float("nan"))}},
         "no actor tagged 'Car_10'"),
        ({"camera_cls": TruthCamera, "camera_kw": {"raises": ConnectionError("rpc down")}},
         "GetObjectPoses failed"),
        ({}, "reads no simulator poses"),
        ({"camera_cls": TruthCamera, "extra": ("--subject-actor", "none")},
         "--subject-actor none"),
        ({"camera_cls": TruthCamera, "extra": ("--object", "a person")},
         "no level actor is known"),
    )
    for kw, why in cases:
        extra = kw.pop("extra", ("--object", "a red car"))
        rc, run, _, cam = _fly(STANDOFF_ANY, max_s="1", extra=extra, **kw)
        rows = _rows(run)
        assert all(r["subject_truth"] is None and why in r["subject_truth_why"]
                   for r in rows[:3]), (why, rows[0].get("subject_truth_why"))
        assert "subject_range_m" not in rows[0]
        m = _json(run, "metrics.json")
        assert m["follow_scored"] is False and m["standoff_s"] is None
        assert m["frac_within_30m"] is None and m["standoff_min_range_m"] is None
        assert m["subject_truth"]["min_range_m"] is None
        assert any(why in k for k in m["subject_truth"]["unread_why"]), m["subject_truth"]
    # A simulator whose pose service fails is asked again only after a
    # back-off (TRUTH_RETRY_TICKS, then doubling), not every tick.
    rc, run, _, cam = _fly(STANDOFF_ANY, max_s="1", camera_cls=TruthCamera,
                           camera_kw={"raises": ConnectionError("rpc down")},
                           extra=("--object", "a red car"))
    assert len(cam.names) == pn.TRUTH_MAX_ERRORS
    assert "backing off after 3 failed reads" in _rows(run)[-1]["subject_truth_why"]
    assert "not scored" in " ".join(_json(run, "kpi.json")["kpi_grade_reasons"])


def test_the_simulator_pose_is_read_by_tag_and_moved_into_the_shield_frame():
    class World:
        def __init__(self):
            self.asked = []

        def get_object_poses(self, names):
            self.asked.append(list(names))
            return [{"translation": {"x": 37.5, "y": -20.0, "z": 0.0},
                     "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}]
    cam = pn.ProjectAirSimCamera(T.FakeObs(), {"dir": ".", "scene": "x"})
    cam.world = World()
    assert cam.object_xy("Car_10") == (37.5, -20.0) and cam.world.asked == [["Car_10"]]
    home = ANCHOR
    same = ap.FrameAnchor(home[0], home[1], home[2], "scene.home-geo-point")
    assert pn.scene_to_anchor(37.5, -20.0, home, same) == (37.5, -20.0)
    lat, lon = same.to_latlon(10.0, 10.0)
    other = ap.FrameAnchor(lat, lon, None, "policy.origin")
    x, y = pn.scene_to_anchor(37.5, -20.0, home, other)
    assert abs(x - 27.5) < 0.02 and abs(y + 30.0) < 0.02, (x, y)
    assert pn.scene_to_anchor(1.0, 2.0, None, same) is None
    assert pn.subject_actor_for("auto", "a red car", "car") == ("Car_10", None)
    assert pn.subject_actor_for("Car_04", "a blue car", "car") == ("Car_04", None)
    assert pn.subject_actor_for("auto", "a blue car", "car")[0] is None


# =========================================================================== #
# (6) the literal LOITER hold
# =========================================================================== #

class ThrottleLink(T.FakeLink):
    """An autopilot that reports the RC override it was sent (RC_CHANNELS)."""

    def __init__(self, *a, kind="sitl", accepted=True, **k):
        super().__init__(*a, **k)
        self.autopilot_kind, self.overrides, self.log = kind, [], []
        self.accepted = accepted

    def evidence(self):
        e = super().evidence()
        tev = {k: v for k, v in e["topology_evidence"].items() if k != "autopilot_kind"}
        if self.autopilot_kind is not None:
            tev["autopilot_kind"] = self.autopilot_kind
        return {**e, "topology_evidence": tev}

    def rc_override_plan(self):
        return {"via": "fake", "sysid": 255, "accepted_expected": self.accepted,
                "why": None if self.accepted else
                "ArduPilot takes overrides from SYSID_MYGCS 255, and MAVROS sends as 1"}

    def override_throttle(self, pwm):
        self.overrides.append(pwm)
        self.log.append(("rc3", pwm))
        self.snap.rc3, self.snap.t_rc = (1100 if pwm is None else pwm), self.clock.t
        return True

    def request_mode(self, name):
        self.log.append(("mode", name))
        return super().request_mode(name)

    def land(self):
        self.log.append(("mode", "LAND"))
        return super().land()


def test_the_literal_loiter_hold_centres_the_throttle_on_sitl():
    """ArduCopter's LOITER climbs at the throttle stick's rate; on 7 and 8
    Oct it sank with no pilot. With --hold-mode LOITER on SITL the node
    holds channel 3 at 1500 every tick the FSM holds, and hands it back
    before the mission-end LAND."""
    rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="3", link_cls=ThrottleLink,
                            extra=("--hold-mode", "LOITER"))
    assert rc == 0
    rows = _rows(run)
    hold = [r for r in rows if r.get("fsm_state_after") == "Loiter"]
    assert hold, "the FSM never held: the test proves nothing"
    assert all(r["rc3_override"] == 1500 for r in hold)
    assert link.requested == ["LOITER", "LAND"], link.requested
    assert link.overrides[:-1] == [1500] * len(hold) and link.overrides[-1] is None
    i_rel = link.log.index(("rc3", None))
    assert link.log[i_rel + 1] == ("mode", "LAND"), link.log[-3:]
    m = _json(run, "metrics.json")
    ov = m["hold"]["rc_override"]
    assert m["hold"]["autopilot_mode"] == "LOITER" and ov["enabled"] is True
    assert ov["ticks_sent"] == len(hold) and ov["pwm"] == 1500 and ov["channel"] == 3
    assert ov["spells"][0]["t_end"] is not None and ov["rc3_reported"]["at_pwm"] >= len(hold) - 1
    acts = [e["action"] for e in _events(run) if e["kind"] == "rc_override"]
    assert acts == ["hold", "release"], acts


def test_no_override_with_brake_or_on_a_hardware_autopilot():
    rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="2", link_cls=ThrottleLink)
    assert link.overrides == [] and link.requested[0] == "BRAKE"
    ov = _json(run, "metrics.json")["hold"]["rc_override"]
    assert ov["enabled"] is False and "BRAKE" in ov["why"]
    assert all(r["rc3_override"] is None for r in _rows(run))
    rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="2", link_cls=ThrottleLink,
                            link_kw={"kind": "hardware"}, extra=("--hold-mode", "LOITER"))
    assert link.overrides == [] and link.requested[0] == "LOITER"
    ov = _json(run, "metrics.json")["hold"]["rc_override"]
    assert ov["enabled"] is False and "safety pilot" in ov["why"]
    # A kind nobody read may be an airframe: no override either.
    rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="2", link_cls=ThrottleLink,
                            link_kw={"kind": None}, extra=("--hold-mode", "LOITER"))
    assert pn.autopilot_kind_of(link) is None and link.overrides == []
    ov = _json(run, "metrics.json")["hold"]["rc_override"]
    assert ov["enabled"] is False and "None" in ov["why"], ov["why"]
    # An override ArduPilot would drop (MAVROS's system_id is not SYSID_MYGCS)
    # is not sent and counted as a hold that held nothing.
    rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="2", link_cls=ThrottleLink,
                            link_kw={"accepted": False}, extra=("--hold-mode", "LOITER"))
    assert link.overrides == []
    ov = _json(run, "metrics.json")["hold"]["rc_override"]
    assert ov["enabled"] is False and "ignore the override" in ov["why"], ov
    assert ov["plan"]["accepted_expected"] is False and ov["ticks_sent"] == 0


class OperatorLink(ThrottleLink):
    """A pilot who switches the autopilot out of LOITER (to `to`) right after
    the `after`-th 1500 override: no request of this node explains it."""

    def __init__(self, *a, after=3, to="STABILIZE", **k):
        super().__init__(*a, **k)
        self.after, self.to = after, to

    def override_throttle(self, pwm):
        r = super().override_throttle(pwm)
        if pwm is not None and self.overrides.count(1500) == self.after:
            self._mode(self.to)
        return r


def test_the_override_ends_the_tick_the_autopilot_leaves_loiter_on_its_own():
    """REVIEW (2026-10-13). A takeover was seen only out of GUIDED. In the
    FSM's hold the autopilot is in LOITER, so a pilot's STABILIZE (or a
    fence's RTL) from there was never a takeover, and the node went on
    pinning channel 3 at 1500 under the pilot: 27 more overrides in the
    reviewer's run."""
    for to in ("STABILIZE", "RTL"):
        rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="3", link_cls=OperatorLink,
                                link_kw={"to": to}, extra=("--hold-mode", "LOITER"))
        assert rc == 0
        assert link.overrides == [1500, 1500, 1500, None], (to, link.overrides)
        m = _json(run, "metrics.json")
        took = m["autopilot_took_over"]
        assert took and took["mode"] == to and took["previous"] == "LOITER", took
        assert link.requested == ["LOITER"], "the autopilot's own mode must not be overridden"
        rel = [e for e in _events(run) if e["kind"] == "rc_override" and e["action"] == "release"]
        assert len(rel) == 1 and "took the aircraft" in rel[0]["why"], rel
        rows = _rows(run)
        assert all(r["rc3_override"] is None for r in rows if r["ap_mode"] == to)


def test_the_override_follows_the_hold_and_an_fsm_fault():
    """The release when the hold ends (unit), and the hold an FSM fault
    asks for (in flight): channel 3 centred on every tick of it."""
    link = ThrottleLink(T.Clock(), ap.FrameAnchor(*ANCHOR, "scene"))
    rec = StubRec()
    th = pn.ThrottleHold(link, "LOITER", "sitl", rec)
    for i in range(3):
        assert th.tick(0.1 * i, True, link.snap) == 1500
    assert th.tick(0.3, False, link.snap) is None
    th.tick(0.4, False, link.snap)
    assert link.overrides == [1500, 1500, 1500, None], link.overrides
    assert [k["action"] for _, k in rec.events] == ["hold", "release"]
    assert th.summary()["spells"] == [{"t_start": 0.0, "t_end": 0.3}]
    real = pn.tick_input_from_decision

    def refuse(*a, **k):
        raise ValueError("forced: a decision the FSM refuses")
    pn.tick_input_from_decision = refuse
    try:
        rc, run, link, _ = _fly(T.LOITER_POLICY, max_s="2", link_cls=ThrottleLink,
                                extra=("--hold-mode", "LOITER"))
    finally:
        pn.tick_input_from_decision = real
    rows = _rows(run)
    assert "forced" in _json(run, "metrics.json")["fsm"]["fault"]
    assert link.requested[0] == "LOITER" and all(r["rc3_override"] == 1500 for r in rows)
    assert link.overrides == [1500] * len(rows) + [None], link.overrides


def test_the_pymavlink_override_is_sent_as_the_ground_station_and_reported_back():
    # SYSID_MYGCS 250, not ArduPilot's default 255: the id is READ.
    link, fm = _pml(None, params={"SYSID_MYGCS": 250.0})
    plan = link.rc_override_plan()
    assert plan["sysid"] == 250 and plan["accepted_expected"] is True
    seen = []
    orig = fm.master.mav.respond

    def respond(name, a):
        if name == "rc_channels_override_send":
            seen.append(fm.master.mav.srcSystem)
        orig(name, a)
    fm.master.mav.respond = respond
    link.override_throttle(1500)
    name, a, _ = fm.master.mav.sent[-1]
    assert name == "rc_channels_override_send"
    assert a == (1, 1, 65535, 65535, 1500, 65535, 65535, 65535, 65535, 65535), a
    assert seen == [250], "ArduPilot ignores an override from any id but SYSID_MYGCS"
    assert fm.master.mav.srcSystem == 1, "the node's own system id was not restored"
    link.override_throttle(None)
    assert fm.master.mav.sent[-1][1][4] == 0, "None must hand channel 3 back to the radio"
    fm.master.inbox = [T.Msg("RC_CHANNELS", chan3_raw=1500)]
    link.pump()
    assert link.snap.rc3 == 1500
    mute, _ = _pml(None)
    p = mute.rc_override_plan()
    assert p["sysid"] == 255 and p["accepted_expected"] is None and "not read" in p["sysid_source"]


def test_the_mavros_override_and_the_system_id_it_needs():
    link, node = _mavros()
    link.override_throttle(1500)
    msg = node.pubs[ml.TOPIC_RC_OVERRIDE].msgs[-1]
    assert len(msg.channels) == 18 and msg.channels[2] == 1500
    assert all(c == 65535 for i, c in enumerate(msg.channels) if i != 2)
    link.override_throttle(None)
    assert node.pubs[ml.TOPIC_RC_OVERRIDE].msgs[-1].channels[2] == 0
    node.clients["/mavros/param/get_parameters"].response = T._ros_params({"SYSID_MYGCS": 255})
    node.clients["/mavros/get_parameters"].response = T._ros_params({"system_id": 1})
    p = link.rc_override_plan()
    assert p["accepted_expected"] is False and "255" in p["why"] and "1" in p["why"], p
    node.clients["/mavros/get_parameters"].response = T._ros_params({"system_id": 255})
    assert link.rc_override_plan()["accepted_expected"] is True
    node.subs["/mavros/rc/in"](SimpleNamespace(channels=[1100, 1200, 1500, 1900]))
    assert link.snap.rc3 == 1500, "channel 3 is the third entry (index 2)"


# =========================================================================== #
# (7) the MAVROS path carries what the pymavlink path does
# =========================================================================== #

def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _rotate(q, v):
    w, x, y, z = q
    p = _quat_mul(_quat_mul(q, (0.0, *v)), (w, -x, -y, -z))
    return p[1:]


def test_the_mavros_link_reads_the_body_pitch_and_roll():
    """The depth rules read the body pitch; over MAVROS it was always None
    (judged at level). Built from the physical facts: nose 10 deg up puts
    the forward axis above the horizon, right wing 5 deg down puts the
    right axis below it, heading 30 deg."""
    h = math.radians(30.0)

    def axis(ang, ax):
        s = math.sin(ang / 2.0)
        return (math.cos(ang / 2.0), *(s * c for c in ax))
    # ENU world, FLU body: yaw about up, then a nose-up turn about FLU's
    # left axis is NEGATIVE, then roll about forward.
    q = _quat_mul(_quat_mul(axis(math.pi / 2 - h, (0, 0, 1)),
                            axis(-math.radians(10.0), (0, 1, 0))),
                  axis(math.radians(5.0), (1, 0, 0)))
    fwd, right = _rotate(q, (1.0, 0.0, 0.0)), _rotate(q, (0.0, -1.0, 0.0))
    assert fwd[2] > 0.17 and right[2] < -0.08, (fwd, right)     # the physics
    qs = SimpleNamespace(w=q[0], x=q[1], y=q[2], z=q[3])
    pitch, roll = ml.enu_quat_to_ned_pitch_roll(qs)
    assert abs(math.degrees(pitch) - 10.0) < 1e-6 and abs(math.degrees(roll) - 5.0) < 1e-6
    assert abs(math.degrees(ml.enu_quat_to_ned_yaw(qs)) - 30.0) < 1e-6
    link, node = _mavros()
    node.subs["/mavros/local_position/pose"](SimpleNamespace(
        pose=SimpleNamespace(orientation=qs)))
    assert abs(link.snap.pitch - math.radians(10.0)) < 1e-9
    assert abs(link.snap.roll - math.radians(5.0)) < 1e-9


def _identify(link, node, mavros_params):
    node.clients["/mavros/vehicle_info_get"].response = SimpleNamespace(
        success=True, vehicles=[SimpleNamespace(
            available_info=2, flight_sw_version=T.AP_VERSION.flight_sw_version,
            flight_custom_version=T.AP_VERSION.flight_custom_version,
            type=2, autopilot=3)])
    node.clients["/mavros/param/get_parameters"].response = T._ros_params(
        {"SIM_SPEEDUP": 1.0, "SYSID_THISMAV": 1.0})
    node.clients["/mavros/get_parameters"].response = T._ros_params(mavros_params)
    link.identify(timeout=1)


def test_the_mavros_evidence_carries_the_link_and_this_host_s_processes():
    """sitl/ros2_shield_node.py records the launch script's MAVROS_FCU_URL
    when the parameter is unreadable, and which MAVLink actors run on its
    host - without which an Orin node behind mavlink-router can never show
    the autopilot runs elsewhere. The perception node's MAVROS link did not."""
    procs = {"local_sitl": [], "local_mavros": ["mavros_node"], "local_router": []}
    old_scan, old_env = ml.scan_local_processes, os.environ.get("MAVROS_FCU_URL")
    ml.scan_local_processes = lambda: dict(procs)
    os.environ["MAVROS_FCU_URL"] = "udp://:14555@"
    try:
        link, node = _mavros()
        node.subs["/mavros/state"](SimpleNamespace(connected=True, armed=False, mode="GUIDED"))
        _identify(link, node, {})
        tev = link.evidence()["topology_evidence"]
    finally:
        ml.scan_local_processes = old_scan
        if old_env is None:
            os.environ.pop("MAVROS_FCU_URL", None)
        else:
            os.environ["MAVROS_FCU_URL"] = old_env
    assert tev["network_link"]["fcu_url"] == "udp://:14555@"
    assert tev["network_link"]["source"] == "MAVROS_FCU_URL (launch script)"
    assert tev["local_processes"] == procs and tev["mavros_on"] == "shield_host"
    # MAVROS 2's mavros_node declares fcu_url on /mavros_node and hands the
    # router a string ARRAY, fcu_urls (review 2026-10-13: the builder asked
    # the router for a string 'fcu_url', a name it does not declare). Read
    # before any launch-script variable (start_ardupilot.sh exports none).
    old_env = os.environ.pop("MAVROS_FCU_URL", None)
    try:
        link, node = _mavros()
        node.clients["/mavros_node/get_parameters"].response = T._ros_params(
            {"fcu_url": "udp://:14555@"})
        node.clients["/mavros_router/get_parameters"].response = _ros_array("fcu_urls", [
            "udp://:9999@"])
        _identify(link, node, {})
        nl = link.evidence()["topology_evidence"]["network_link"]
        assert nl["fcu_url"] == "udp://:14555@" and nl["source"] == \
            "/mavros_node fcu_url parameter", nl
        link, node = _mavros()
        node.clients["/mavros_router/get_parameters"].response = _ros_array("fcu_urls", [
            "", "udp://:14555@"])
        _identify(link, node, {})
        nl = link.evidence()["topology_evidence"]["network_link"]
        assert nl["fcu_url"] == "udp://:14555@" and nl["source"] == \
            "/mavros_router fcu_urls parameter", nl
        # A string 'fcu_url' on the router is not what MAVROS 2 declares there.
        link, node = _mavros()
        node.clients["/mavros_router/get_parameters"].response = T._ros_params(
            {"fcu_url": "udp://:14555@"})
        _identify(link, node, {})
        assert link.evidence()["topology_evidence"]["network_link"]["fcu_url"] is None
    finally:
        if old_env is not None:
            os.environ["MAVROS_FCU_URL"] = old_env


def _ros_array(name, values):
    """A GetParameters answer carrying one string array (type 9)."""
    def answer(req):
        return SimpleNamespace(values=[
            SimpleNamespace(type=9, string_array_value=list(values), double_value=0.0,
                            integer_value=0, string_value="") if n == name else
            SimpleNamespace(type=0, double_value=0.0, integer_value=0, string_value="")
            for n in req.names])
    return answer


def test_a_mavros_run_beside_the_ardupilot_scene_earns_dev_and_pymavlink_does_not():
    """The manifest's dev label beside the rail's own scene needs the live
    MAVROS chain and MAVROS's fcu_url (guardrail.manifest, 2026-10-12): the
    MavrosLink's evidence must carry both, and the pymavlink link's never
    passes."""
    cfg = rail.write_sim_configs(_tmp(), rail.plan_network("mirrored"))
    scene = Path(cfg["dir"]) / cfg["scene"]
    old = os.environ.get("ROS_DISTRO")
    os.environ["ROS_DISTRO"] = "jazzy"
    old_url = os.environ.pop("MAVROS_FCU_URL", None)
    try:
        link, node = _mavros()
        node.subs["/mavros/state"](SimpleNamespace(connected=True, armed=False, mode="GUIDED"))
        _identify(link, node, {"fcu_url": "udp://:14555@"})
        man = build_manifest("sha256:x", pn.NO_VLA, 1, scene_path=scene,
                             topology=TOPOLOGY_DEV,
                             hil_evidence=link.evidence()["topology_evidence"])
        assert man["topology"] == TOPOLOGY_DEV
        bare, bnode = _mavros()
        bnode.subs["/mavros/state"](SimpleNamespace(connected=True, armed=False, mode="GUIDED"))
        _identify(bare, bnode, {})
        T._raises(build_manifest, "sha256:x", pn.NO_VLA, 1, scene_path=scene,
                  topology=TOPOLOGY_DEV,
                  hil_evidence=bare.evidence()["topology_evidence"], contains="fcu_url")
    finally:
        if old is None:
            os.environ.pop("ROS_DISTRO", None)
        else:
            os.environ["ROS_DISTRO"] = old
        if old_url is not None:
            os.environ["MAVROS_FCU_URL"] = old_url
    pm, _ = _pml(None, params={"SIM_SPEEDUP": 1.0})
    pm.identify(timeout=0.2)
    T._raises(build_manifest, "sha256:x", pn.NO_VLA, 1, scene_path=scene,
              topology=TOPOLOGY_DEV, hil_evidence=pm.evidence()["topology_evidence"])


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v) and v.__module__ == __name__]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except (AssertionError, SystemExit) as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {type(e).__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
