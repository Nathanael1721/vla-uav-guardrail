"""The ArduPilot rails, checked without a simulator.

Run either way:
    pytest tests/test_sitl_rails.py -v
    python tests/test_sitl_rails.py

WHY THIS FILE EXISTS

Nothing imported `sitl/` until 2026-08-28, and it cost exactly what you would
expect. Commit `01e22ca` made `AuditLogger.policy_hash` a read-only property so
that a hot-applied rule restamps the hash. Seven `demo/*.py` call sites were
updated. The two in `sitl/` were not, and both assign to that property right
after `hot_apply`:

    sitl/run_sitl_demo.py:198      audit.policy_hash = policy.policy_hash
    sitl/ros2_shield_node.py:206   self.audit.policy_hash = ...

So every `--dynamic` run died at t = 8 s with `AttributeError`, on the rail that
carries the grant's contractual KPI figures, and nothing noticed. The change was
verified against 242 tests; none of them touched this code.

These tests need no autopilot, no ROS, no MAVLink. They exercise the two things
the rails do that the AirSim demos do not: hot-apply a fence mid-flight, and
record the flown action's violations for a shield-OFF control run.

Since 2026-10-06 `ros2_shield_node.py` keeps its per-tick logic in plain
functions (`shield_step`, `build_row`) and imports `rclpy` only inside a guard,
so those are imported and run here; the ROS wiring itself is still checked by
reading the source, and was flown in WSL (docs/DESIGN-ros2-interface.md).
"""
import ast
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import load_policy                                    # noqa: E402
from guardrail.audit import AuditLogger                              # noqa: E402
from guardrail.models import Action4D, PolygonFence, State, XY       # noqa: E402
from guardrail.shield import Shield                                  # noqa: E402

SITL = ROOT / "sitl"
POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _fresh_audit():
    """Construct the logger the way the SITL rails construct it."""
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    path = Path(tempfile.mkdtemp()) / "audit.jsonl"
    return policy, shield, AuditLogger(path, policy), path


def test_a_hot_applied_fence_restamps_the_audit_hash():
    """The exact sequence the --dynamic runs perform at t = 8 s: a tick, the
    zone through EpisodeRecord.apply_zone (a dynamic_nfz where the Shield
    enforces the grant's mid-flight update model), another tick."""
    from guardrail.replay import EpisodeRecord
    policy, shield, audit, path = _fresh_audit()
    rec = EpisodeRecord(path.parent / "ep", policy, None)
    before = policy.policy_hash

    audit.log(1, shield.filter(State(x=-20, y=-20, up=4), Action4D(vx=9)))
    applied = rec.apply_zone(shield, PolygonFence(
        id="nfz-dynamic", type="polygon_fence", **HARD_P0,
        vertices=[XY(x=-25, y=-25), XY(x=-15, y=-25),
                  XY(x=-15, y=-15), XY(x=-25, y=-15)], margin_m=1.0))
    assert applied.id == "nfz-dynamic" and policy.generation == 1
    assert [g["generation"] for g in rec.generations] == [0, 1]
    audit.log(2, shield.filter(State(x=-20, y=-20, up=4), Action4D(vx=1)))

    recs = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines()]
    assert len(recs) == 2, recs
    assert recs[0]["policy_hash"] == before
    assert recs[1]["policy_hash"] != before, (
        "a hot-applied rule must restamp the hash; the audit trail is supposed "
        "to show which rules were in force for each record")


def test_neither_rail_assigns_to_the_read_only_policy_hash():
    """The regression itself, caught by reading the source.

    `policy_hash` is a property with no setter, so an assignment is an
    AttributeError at run time and completely invisible to import-time checks.
    Every demo and sitl script is read since 2026-10-11: demo/run_demo.py
    still assigned it twice, so every shielded run died on its first tick
    (since 2026-08-26), unseen because nothing here read demo/.
    """
    scripts = (sorted(SITL.glob("*.py")) + sorted((ROOT / "demo").glob("*.py"))
               + sorted((ROOT / "demo" / "pas_ardupilot").glob("*.py")))
    assert len(scripts) > 20, scripts
    for path in scripts:
        src = path.read_text(encoding="utf-8")
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
                continue
            for tgt in (node.targets if isinstance(node, ast.Assign) else [node.target]):
                if isinstance(tgt, ast.Attribute) and tgt.attr == "policy_hash" \
                        and "audit" in ast.unparse(tgt.value).lower():
                    raise AssertionError(
                        f"{path.relative_to(ROOT)}:{node.lineno} assigns to "
                        f"AuditLogger.policy_hash, which is read-only. Pass the "
                        f"policy object to AuditLogger instead and delete the "
                        f"assignment.")


def test_both_rails_construct_the_logger_with_the_policy_not_the_hash():
    """Passing the hash string is legal but silently loses hot-apply tracking."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        tree = ast.parse((SITL / name).read_text(encoding="utf-8"))
        made = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == "AuditLogger"]
        assert made, f"{name} builds no AuditLogger"
        for call in made:
            assert ast.unparse(call.args[1]) in ("policy", "self.policy"), (
                f"{name} must hand AuditLogger the POLICY, so the hash is read "
                f"live and a mid-flight rule change is visible in the audit "
                f"log; it passes {ast.unparse(call.args[1])!r}")


def test_both_rails_record_the_flown_actions_violations():
    """Without this the KPI falls back to an inference that cannot fail.

    Recomputing the 2026-08-24 artefacts with the current kpi.py reported
    `p0_ticks_not_measurable` equal to the FULL P0 tick count on every SITL run
    - 132 of 132 on `sitl_shield_on`. The headline "escape rate 0.0 on the real
    ArduPilot rail" was inferred, never measured.
    """
    # Since 2026-10-07 the choice lives in one place, guardrail.replay.
    # flown_fields (tested in tests/test_replay.py), and both rails call it.
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "flown_fields(" in src, (
            f"{name} does not log emitted_violations through flown_fields, so "
            f"guardrail/kpi.py cannot MEASURE a P0 escape on its rows")
    from guardrail import replay
    rsrc = Path(replay.__file__).read_text(encoding="utf-8")
    assert "decision.emitted_violations if shield_on else decision.violations" \
        in rsrc, ("flown_fields must pick the list by which action actually "
                  "flew: the re-check when the shield is ON, the violations on "
                  "`raw` when it is OFF - otherwise the control run scores clean")


def test_the_control_arm_still_earns_a_nonzero_escape_rate():
    """The A/B only means something if shield-OFF can fail.

    Simulates the row-building both rails do, with the shield off: `raw` flies
    unmodified, so the violations found on `raw` are the flown action's.
    """
    from guardrail import kpi as K
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    st = State(x=27.0, y=10.0, up=6.0)         # inside sim_demo_policy's zone
    rows = []
    for tick in range(40):
        raw = Action4D(vx=4.0)
        d = shield.filter(st, raw)
        rows.append({
            "t": tick * 0.1, "tick": tick,
            "raw": raw.model_dump(), "emitted": raw.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            # shield OFF: `raw` flew, so its violations are the flown ones
            "emitted_violations": [v.model_dump() for v in d.violations],
            "repairs": [], "braked": False,
        })
    res = K.compute(rows, K.rule_priorities(policy), {})
    assert res["p0_ticks_not_measurable"] == 0, (
        "the control arm must be MEASURED, not inferred")
    assert res["p0_violation_escape_rate"] > 0.0, (
        "an unguarded flight straight into a no-fly zone must score escapes; "
        "if this is zero the A/B proves nothing")


def test_the_shielded_arm_is_measured_and_clean():
    from guardrail import kpi as K
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    st = State(x=27.0, y=10.0, up=6.0)
    rows = []
    for tick in range(40):
        raw = Action4D(vx=4.0)
        d = shield.filter(st, raw)
        rows.append({
            "t": tick * 0.1, "tick": tick,
            "raw": raw.model_dump(), "emitted": d.emitted.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            "emitted_violations": [v.model_dump() for v in d.emitted_violations],
            "repairs": [r.model_dump() for r in d.repairs],
            "braked": d.braked,
        })
    res = K.compute(rows, K.rule_priorities(policy), {})
    assert res["p0_ticks_not_measurable"] == 0
    assert res["p0_violation_escape_rate"] == 0.0, (
        f"the Shield let a P0 violation through: {res}")


def test_the_ros2_rail_reads_sim_speedup_instead_of_asserting_it():
    """build_manifest's docstring: "a hand-written 1.0 is exactly the number a
    broken run would also carry."

    The perverse part of the old code: the three runs that PASSED is_kpi_grade()
    asserted this value, while the pymavlink rail that reads it was refused for
    its topology.
    """
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "sim_speedup=1.0" not in src, (
        "ros2_shield_node.py still asserts sim_speedup=1.0")
    assert "def read_sim_speedup" in src and "sim_speedup=speedup" in src
    # Since 2026-11-06 the mission-style node reads it the same way, through
    # the same reader (sitl/mavros_evidence.py poll_sim_speedup).
    src = (SITL / "ros2_mission_node.py").read_text(encoding="utf-8")
    assert "sim_speedup=1.0" not in src, (
        "ros2_mission_node.py asserts sim_speedup=1.0")
    assert "def read_sim_speedup" in src and "sim_speedup=self.sim_speedup" in src


def test_both_rails_write_the_kpi_grade_into_the_artefact():
    """A reader holding kpi.json must be able to tell whether its numbers are
    quotable as contractual figures."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert '"kpi_grade"' in src, f"{name} does not record the grade in kpi.json"


def test_no_shell_script_has_windows_line_endings():
    """A .sh with CRLF is not a runnable script.

    bash reads the carriage return as part of the token, so `set -e` becomes an
    invalid option and the first function definition is a syntax error. This is
    easy to reintroduce from Windows - pathlib.write_text translates newlines on
    write - and the failure appears in WSL, far from the edit.
    """
    crlf = bytes([13, 10])          # CR LF, written this way so
    # the literal cannot itself be mangled by a newline-translating write
    for sh in sorted((ROOT / "sitl").rglob("*.sh")):
        assert crlf not in sh.read_bytes(), (
            f"{sh.name} has CRLF line endings and will not run under bash")


def test_no_sitl_script_hard_codes_this_pcs_repository_path():
    """X-08. A script that cds into /mnt/d/OneDrive/... runs only on this PC,
    and the stack has to move to the Orin and later the drone with no code
    change. Every sitl/*.sh finds the repository from its own location
    (run_gazebo_demo.sh hard-coded it until 2026-10-07)."""
    for sh in sorted((ROOT / "sitl").rglob("*.sh")):
        for n, line in enumerate(sh.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            assert "/mnt/d/" not in code and "OneDrive" not in code, (
                f"{sh.name}:{n} hard-codes a path on this PC: {line.strip()}")


# --------------------------------------------------------------------------- #
# The grant's ROS 2 interface (2026-10-06): body frame, records, adapter
# --------------------------------------------------------------------------- #

sys.path.insert(0, str(SITL))


def _node():
    import ros2_shield_node as N
    return N


def test_the_shield_node_reads_the_vla_action_in_body_frame():
    """WP3-04 / ARCH-04. Heading East, a body-forward 4 m/s points at the
    no-fly zone east of the aircraft; read as world frame (the old node) it
    points North, past the zone, and the Shield would see nothing."""
    N = _node()
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    st = State(x=15.0, y=0.0, up=15.0, yaw_deg=90.0)
    body = Action4D(vx=4.0)
    decision, raw_w, em_w, em_b = N.shield_step(shield, st, body, True)
    assert abs(raw_w.vy - 4.0) < 1e-9 and abs(raw_w.vx) < 1e-9, raw_w
    assert any(v.rule_id == "nfz-square" for v in decision.violations), (
        "the body-forward action at heading East must be checked against the "
        "zone to the East")
    # The repaired action goes back out in body frame, exactly.
    from guardrail.frames import from_body
    back = from_body(em_b.vx, em_b.vy, em_b.vz_up, em_b.yaw_rate, st.yaw_deg)
    assert abs(back.vx - em_w.vx) < 1e-9 and abs(back.vy - em_w.vy) < 1e-9
    assert em_b.model_dump() != body.model_dump(), "the Shield did not repair"
    # The same numbers read as WORLD frame: no violation at all.
    world_reading = Shield(load_policy(POLICY)).filter(st, body)
    assert not any(v.rule_id == "nfz-square" for v in world_reading.violations)


def test_with_the_shield_off_the_body_action_passes_untouched():
    N = _node()
    shield = Shield(load_policy(POLICY), lookahead_s=3.0, dt=0.5)
    st = State(x=15.0, y=0.0, up=15.0, yaw_deg=37.0)
    body = Action4D(vx=4.0, vy=-1.0, vz_up=0.3, yaw_rate=0.2)
    decision, _, _, em_b = N.shield_step(shield, st, body, False)
    assert em_b is body, "the control arm must fly the VLA's bytes, not a rotation"
    assert decision.violations, "the control arm still records what it broke"


def test_every_row_carries_what_the_kpis_need():
    """WP3-17, WP4-13, WP1-24, WP3-13: unsafe, policy_hash + generation, the
    Shield's own time, the declared subject, both frames, the FSM state."""
    from guardrail.fsm import EscalationFSM, tick_input_from_decision
    from guardrail.replay import episode_row_fields
    N = _node()
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    st = State(x=15.0, y=15.0, up=15.0, yaw_deg=0.0)      # inside the zone
    body = Action4D(vx=1.0)
    decision, raw_w, em_w, em_b = N.shield_step(shield, st, body, True)
    stop = shield.state_is_unsafe(st)
    out = EscalationFSM().step(tick_input_from_decision(
        0.1, decision, policy, horizon_s=0.1, stop_illegal=stop))
    row = N.build_row(now=0.1, tick=1, state=st, body=body, decision=decision,
                      raw_world=raw_w, emitted_world=em_w, emitted_body=em_b,
                      shield_on=True,
                      extra=episode_row_fields(shield, st, policy),
                      subject=(30.0, 30.0), fsm_out=out, stop_illegal=stop,
                      mode="GUIDED")
    for key in ("unsafe", "unsafe_rules", "policy_hash", "generation",
                "shield_ms", "tgt_x", "tgt_y", "raw_body", "emitted_body",
                "fsm_state_before", "fsm_state_after", "failsafe", "mode"):
        assert key in row, f"row lacks {key}"
    assert row["unsafe"] and "nfz-square" in row["unsafe_rules"]
    assert row["policy_hash"] == policy.policy_hash and row["generation"] == 0
    json.dumps(row)                                  # it must serialise


def test_the_ros2_node_publishes_the_grants_setpoint_not_a_twist():
    """WP3-03 / ARCH-05: /shield/setpoint to the adapter, never a Twist on
    cmd_vel_unstamped from the Shield node itself."""
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    imported = {a.asname or a.name for n in ast.walk(tree)
                if isinstance(n, ast.ImportFrom) for a in n.names}
    strings = {n.value for n in ast.walk(tree)
               if isinstance(n, ast.Constant) and isinstance(n.value, str)
               and n.value != ast.get_docstring(tree, clean=False)}
    assert "Twist" not in imported, "the Shield node imports Twist again"
    assert not any("cmd_vel" in s for s in strings), "it publishes on cmd_vel"
    assert "TOPIC_SETPOINT" in src and "TOPIC_MODE_REQUEST" in src
    import mavlink_adapter_node as A
    assert A.TOPIC_SETPOINT == "/shield/setpoint"
    assert A.TOPIC_RAW_LOCAL == "/mavros/setpoint_raw/local"


def test_the_node_names_the_pilot_it_was_told_not_a_constant():
    """ARCH-06: a real VLA node would have been filed as StubVLA, because the
    Shield node wrote that string into every manifest."""
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert '"guardrail.vla_stub.StubVLA"' not in src
    assert "/vla/identity" in src
    stub = (SITL / "ros2_vla_stub_node.py").read_text(encoding="utf-8")
    assert "/vla/identity" in stub


def test_both_rails_keep_every_generation_and_take_a_seed():
    """WP2-11 / WP2-15 / WP4-12: the episode record writes policy_g<N> and
    csp_g<N> at take-off and after every hot-apply; the seed is an argument,
    not a literal 0 in the manifest call."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "EpisodeRecord(" in src and (".apply_zone(" in src
                                            or ".hot_applied(" in src), name
        assert "episode_row_fields(" in src, name
        assert '"--seed"' in src and "seed=0," not in src, name
        # The episode record moves the stale audit aside BEFORE the logger
        # opens it, or the first record lands in the old file.
        assert src.index("EpisodeRecord(") < src.index("AuditLogger(out"), name


def test_both_rails_record_the_autopilots_own_version():
    """ARCH-31: what flew is named by the autopilot (AUTOPILOT_VERSION), not
    by the setup script's pin."""
    assert "ardupilot_version_from_mavlink(" in (
        SITL / "run_sitl_demo.py").read_text(encoding="utf-8")
    # The ROS 2 nodes ask MAVROS's vehicle_info_get and decode the answer
    # with sitl/mavros_evidence.py (2026-11-06; until then inside the Shield
    # node, and the mission-style node did not ask at all).
    assert "describe_autopilot_version(" in (
        SITL / "mavros_evidence.py").read_text(encoding="utf-8")
    for name in ("ros2_shield_node.py", "ros2_mission_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "VehicleInfoGet" in src and "poll_autopilot_version(" in src, name


def test_the_vla_stub_publishes_body_frame():
    """Heading East, a stub flying North sees its target on its LEFT: the
    body-frame action is (0, -v), not the world (v, 0)."""
    import ros2_vla_stub_node as V
    from guardrail.compiler import ConstraintCompiler
    from guardrail.frames import from_body
    from guardrail.vla_stub import StubVLA
    policy = load_policy(POLICY)
    m = ConstraintCompiler(policy).parse_command("fly to (40, 0) at 4 m/s")
    st = State(x=0.0, y=0.0, up=15.0, yaw_deg=90.0)
    body = V.body_action(StubVLA(m), st, yaw_rate=0.2)
    assert abs(body[0]) < 1e-9 and body[1] < -3.9, body
    assert body[3] == 0.2
    w = from_body(*body, st.yaw_deg)
    assert abs(w.vx - 4.0) < 1e-9 and abs(w.vy) < 1e-9


def test_the_kpi_grade_reasons_are_not_overwritten_before_the_report():
    """run_sitl_demo.py bound the grade's reasons to `why`, then rebound `why`
    to verify_replay's notes before writing report.md and the [kpi] line, so
    the report gave replay notes ("signature UNSIGNED", "no CSP on file") as
    the reasons a run was not KPI-grade while kpi.json said "topology",
    "dirty tree". Found on the 2026-10-06 flight of that rail."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        tree = ast.parse((SITL / name).read_text(encoding="utf-8"))
        for fn in ast.walk(tree):
            if not isinstance(fn, ast.FunctionDef):
                continue
            reason_vars, lines = set(), []
            for node in ast.walk(fn):
                if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call) \
                        and getattr(node.value.func, "id", "") == "is_kpi_grade" \
                        and isinstance(node.targets[0], ast.Tuple):
                    reason_vars.add(node.targets[0].elts[1].id)
                    lines.append(node.lineno)
            if not reason_vars:
                continue
            for node in ast.walk(fn):
                if not (isinstance(node, ast.Assign) and node.lineno > min(lines)
                        and isinstance(node.value, ast.Call)
                        and getattr(node.value.func, "id", "") == "verify_replay"):
                    continue
                for tgt in node.targets:
                    names = {e.id for e in getattr(tgt, "elts", [tgt])
                             if isinstance(e, ast.Name)}
                    assert not names & reason_vars, (
                        f"{name}:{node.lineno} rebinds {names & reason_vars} "
                        f"(the KPI-grade reasons) to verify_replay's notes")


def _row(N, st, body, shield_on, setpoint, decision=None, shield=None):
    from guardrail.replay import episode_row_fields
    policy = load_policy(POLICY)
    shield = shield or Shield(policy, lookahead_s=3.0, dt=0.5)
    if decision is None:
        decision, raw_w, em_w, em_b = N.shield_step(shield, st, body, shield_on)
    else:
        raw_w, em_w, em_b = decision.raw, decision.emitted, decision.emitted
    return N.build_row(now=1.0, tick=10, state=st, body=body, decision=decision,
                       raw_world=raw_w, emitted_world=em_w,
                       emitted_body=None if setpoint == "none" else em_b,
                       shield_on=shield_on,
                       extra=episode_row_fields(shield, st, policy, 123),
                       subject=None, mode="RTL" if setpoint == "none" else "GUIDED",
                       setpoint=setpoint, shield=shield)


def test_a_tick_the_autopilot_flew_is_not_an_escape():
    """Review 2026-10-07: in the shield-off GeoFence run, 250 of the 283
    counted P0 escapes were ticks on which ArduPilot's RTL flew the aircraft
    and the node streamed nothing (setpoint 'none'). build_row wrote the raw
    action's violations as `emitted_violations` regardless."""
    from guardrail import kpi as K
    N = _node()
    st = State(x=4.0, y=15.0, up=15.0, yaw_deg=0.0)    # heading at the zone
    body = Action4D(vx=6.0)
    flown = _row(N, st, body, False, "pass")
    held = _row(N, st, body, False, "none")
    assert flown["flown"] is True and flown["emitted_violations"], flown
    assert held["flown"] is False and held["emitted_violations"] == [], held
    assert held["violations"], "the raw action's own check is still recorded"
    pri = K.rule_priorities(load_policy(POLICY))
    res = K.compute([flown] * 3 + [held] * 7, pri, {"shield": "off"})
    assert res["p0_escapes"] == 3, res["p0_escapes"]


def test_a_brake_tick_is_checked_as_the_zero_action():
    """The FSM's Brake flies Action4D(): its check is standing still here,
    not the repaired action the Shield produced and nobody flew."""
    N = _node()
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    inside = State(x=15.0, y=15.0, up=15.0, yaw_deg=0.0)
    row = _row(N, inside, Action4D(vx=1.0), True, "brake", shield=shield)
    assert row["flown"] and any(v["rule_id"] == "nfz-square"
                                for v in row["emitted_violations"]), row
    try:
        N.build_row(now=1.0, tick=1, state=inside, body=Action4D(),
                    decision=shield.filter(inside, Action4D()),
                    raw_world=Action4D(), emitted_world=Action4D(),
                    emitted_body=Action4D(), shield_on=True, extra={},
                    subject=None, setpoint="brake")
    except ValueError:
        pass
    else:
        raise AssertionError("a Brake row was built without the Shield")


def test_the_control_arm_never_records_a_brake_or_a_repair():
    """With the Shield off nothing it decided was flown, so a decision that
    braked must not appear as `braked` (or repairs) in the control arm's row:
    kpi.py would credit the control arm with acting."""
    from guardrail.shield import Repair, ShieldDecision, Violation
    N = _node()
    st = State(x=4.0, y=15.0, up=15.0)
    v = Violation(rule_id="nfz-square", category="geofence", detail="x",
                  predicted_at_s=0.5)
    d = ShieldDecision(raw=Action4D(vx=6.0), emitted=Action4D(), violations=[v],
                       repairs=[Repair(operator="Brake", detail="x")],
                       braked=True, emitted_violations=[])
    off = _row(N, st, Action4D(vx=6.0), False, "pass", decision=d)
    assert off["braked"] is False and off["repairs"] == [], off
    on = _row(N, st, Action4D(vx=6.0), True, "pass", decision=d)
    assert on["braked"] is True and on["repairs"], on


def test_both_rails_fly_the_grants_lookahead():
    """Safety Shield page: "Lookahead horizon: 5 s of predicted trajectory at
    10 Hz = 50 future poses". Both rails ran 3 s at 0.5 s (6 poses). A zone
    4.3 s ahead is the difference: seen at 5 s, missed at 3 s."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "LOOKAHEAD_S = 5.0" in src and "LOOKAHEAD_DT_S = 0.1" in src, name
        assert "lookahead_s=3.0" not in src and "dt=0.5" not in src, name
        assert "lookahead_s=LOOKAHEAD_S" in src, name
    N = _node()
    st = State(x=-20.0, y=15.0, up=15.0)
    a = Action4D(vx=6.0)
    grant = Shield(load_policy(POLICY), lookahead_s=N.LOOKAHEAD_S,
                   dt=N.LOOKAHEAD_DT_S).filter(st, a)
    old = Shield(load_policy(POLICY), lookahead_s=3.0, dt=0.5).filter(st, a)
    assert any(v.rule_id == "nfz-square" for v in grant.violations), grant
    assert not any(v.rule_id == "nfz-square" for v in old.violations), old


def test_both_rails_bind_every_record_to_their_episode_and_end_it():
    """The review's mixed-episode bundle: rows, metrics and kpi.json carry
    the episode id, and the episode ends with mission_end, without which
    guardrail.replay refuses it as unfinished."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "episode_id)" in src and "rec.episode_id" in src, name
        assert '"episode_id": ' in src or "\"episode_id\"] = " in src, name
        assert 'kpi["episode_id"]' in src or 'kpi_res["episode_id"]' in src, name
        assert "rec.end(" in src, name
        assert "kpi_evidence_grade(" in src, (
            f"{name} never asks for a signed bundle before a run is KPI-grade")


def test_the_vla_stub_says_which_host_it_runs_on():
    """hil needs the VLA on the Orin too; only the VLA node's host can say."""
    import ros2_vla_stub_node as V
    from guardrail.compiler import ConstraintCompiler
    policy = load_policy(POLICY)
    m = ConstraintCompiler(policy).parse_command("fly to (40, 0) at 4 m/s")
    rec = V.identity_record(policy, {"kind": "yaml"}, m, yaw_rate=0.2, seed=1)
    assert {"host_arch", "device_model", "l4t_release"} <= set(rec["host"]), rec
    assert rec["kind"] == "stub" and rec["frame"] == "body"
    json.dumps(rec)


def test_the_shield_node_defers_a_zone_over_the_aircraft_and_reads_the_backstop():
    """Review 2026-10-07: every hot-applied zone went to the autopilot at
    once, so one hot-applied ON TOP of the aircraft would breach the GeoFence
    and RTL while the Shield was still working. And the node never read the
    fence parameters back, so a GUARDRAIL_FENCE=0 run recorded a successful
    upload as if a backstop existed."""
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "position=pos," in src and "pos, pos_src = (st.x, st.y)" in src
    # No pose yet = on the pad: held back as if at home, never "unknown, so
    # upload everything" (a 2026-10-07 flight armed a zone over home that way
    # and the GeoFence kept it on the pad). Home is --start's position in
    # the policy's frame since 2026-11-06 ((0, 0) without one, as before;
    # test_a_scenario_start_puts_every_pose_and_the_home_in_the_policy_frame).
    assert 'pos, pos_src = self.start_xy, "assumed home' in src
    assert "self.start_xy = (start[0], start[1]) if start else (0.0, 0.0)" in src
    assert "self._recheck_deferred(st," in src
    assert "def read_autopilot_fence" in src and '"backstop_active"' in src
    assert "fence_alt_plan(" in src and '"FENCE_ALT_MAX"' in src
    assert "read_fence_alt_max(FENCE_PARM) or 30.0" not in src
    # A 2026-10-07 flight read FENCE_ENABLE before MAVROS had FENCE_ALT_MAX,
    # so no zone was exported: the read waits for all of them, and a column
    # still unknown at altitude is read again before the mission.
    assert "all(vals.get(k) is not None for k in FENCE_PARAMS)" in src
    assert '"fence parameters read late"' in src
    # A hardware autopilot's GeoFence is GCS-set: only SITL's is changed, and
    # never one the ground station set on this run (gcs_backstop.json, WP3-10,
    # 2026-11-03): the GCS owns the parameters, the node records its read-back.
    assert "raise_ok = is_sitl and gcs is None" in src and "and raise_ok:" in src
    assert "gcs = gcs_backstop_record(self.out, vals)" in src and '"gcs": gcs,' in src
    # The take-off publish waits for the autopilot's values and a position.
    i = src.index("def bring_up")
    body = src[i:src.index("def start_mission")]
    assert body.index("self.read_autopilot_fence()") < \
        body.index('self._publish_fences("take-off")')


def test_the_shield_node_records_the_gcs_set_fence_beside_its_own_read_back():
    """WP3-10 (2026-11-03): when the ground station's step wrote
    gcs_backstop.json into the run folder, the node records its verdict and
    read-back, and whether the values it read itself through MAVROS are the
    GCS's; no file, no record; an unreadable file is recorded as such."""
    import json as _json
    import ros2_shield_node as N
    d = Path(tempfile.mkdtemp(prefix="gcs_rec_"))
    assert N.gcs_backstop_record(d, {}) is None
    doc = {"ok": True, "link": "udpin:127.0.0.1:14560", "gcs_sysid": 255,
           "params": {"FENCE_ENABLE": {"before": 0.0, "read_back": 1.0},
                      "FENCE_TYPE": {"before": 7.0, "read_back": 5.0}},
           "fence": {"zones": ["nfz-square"], "items_sent": 4, "items_read_back": 4,
                     "match": True}}
    (d / "gcs_backstop.json").write_text(_json.dumps(doc), encoding="utf-8")
    r = N.gcs_backstop_record(d, {"FENCE_ENABLE": 1.0, "FENCE_TYPE": 5.0})
    assert r["ok"] and r["matches_autopilot"] and r["differences"] == {}, r
    assert r["params_before"]["FENCE_ENABLE"] == 0.0 and r["fence_items_read_back"] == 4
    r = N.gcs_backstop_record(d, {"FENCE_ENABLE": 0.0, "FENCE_TYPE": 5.0})
    assert not r["matches_autopilot"] and list(r["differences"]) == ["FENCE_ENABLE"], r
    (d / "gcs_backstop.json").write_text("{not json", encoding="utf-8")
    r = N.gcs_backstop_record(d, {})
    assert r["ok"] is False and "unreadable" in r["error"], r


def test_the_gcs_check_needs_position_and_attitude_through_the_whole_mission():
    """sitl/gcs_listen.py. The 2026-10-06 claim rested on a packet count:
    1651 packets, of which 1393 PARAM_VALUE, 197 FTP, 33 heartbeats, no
    position, and nothing after 32 s - before the mission started. A count
    passes that; this check does not."""
    import gcs_listen as G
    # MAVLink v2 HEARTBEAT (id 0) and v1 ATTITUDE (id 30) frames from sys 1.
    v2 = bytes([0xFD, 9, 0, 0, 7, 1, 1, 0, 0, 0]) + bytes(9) + bytes(2)
    v1 = bytes([0xFE, 28, 7, 1, 1, 30]) + bytes(28) + bytes(2)
    assert G.parse_datagram(v2 + v1) == [(1, 0), (1, 30)]
    assert G.parse_datagram(b"hello") == []
    t0 = 1_000.0
    window = (t0, t0 + 30.0)
    full = {"arrivals": {n: [t0 + 0.1 * k for k in range(301)]
                         for n in G.REQUIRED}}
    ok, why = G.check(full, window)
    assert ok, why
    # The 2026-10-06 shape: parameters and heartbeats, nothing to draw.
    params_only = {"arrivals": {"HEARTBEAT": [t0 - 20 + k for k in range(32)]}}
    ok, why = G.check(params_only, window)
    assert not ok and all("none received" in w for w in why), why
    # The stream stops 10 s into the mission.
    stops = {"arrivals": {n: [t0 + 0.1 * k for k in range(100)]
                          for n in G.REQUIRED}}
    ok, why = G.check(stops, window)
    assert not ok and "largest gap 20.1 s" in why[0], why
    ok, why = G.check(full, None)
    assert not ok and "no mission window" in why[0]
    ev = [{"kind": "mission_start", "wall": "2026-10-07T01:00:00.000+00:00"},
          {"kind": "mission_end", "wall": "2026-10-07T01:00:30.000+00:00"}]
    a, b = G.mission_window(ev)
    assert abs((b - a) - 30.0) < 1e-6


def test_both_rails_run_one_fsm_and_audit_it_after_it_stepped():
    """guardrail.replay.rail_shield / audit_tick (2026-10-07): the rails build
    their Shield, the warm-up one included, without an escalation FSM of its
    own, and write each audit record AFTER their own FSM stepped, with its
    verdict. Before, the Shield's FSM and the node's both ran and the audit
    recorded the Shield's (3 of 83 ticks disagreed on ros2fix_on_dyn_nfz)."""
    for name, step in (("run_sitl_demo.py", "fsm.step("),
                       ("ros2_shield_node.py", "self.fsm.step(")):
        src = (SITL / name).read_text(encoding="utf-8")
        calls = [n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)]
        direct = [n.lineno for n in calls
                  if isinstance(n.func, ast.Name) and n.func.id == "Shield"]
        assert not direct, f"{name} builds Shield(...) at lines {direct}"
        logs = [n.lineno for n in calls
                if isinstance(n.func, ast.Attribute) and n.func.attr == "log"
                and getattr(n.func.value, "attr",
                            getattr(n.func.value, "id", None)) == "audit"]
        assert not logs, f"{name} calls audit.log directly at lines {logs}"
        assert "rail_shield(" in src, f"{name} does not use rail_shield"
        audit_at = [n.lineno for n in calls if isinstance(n.func, ast.Name)
                    and n.func.id == "audit_tick"]
        assert len(audit_at) == 1, (name, audit_at)
        lines = src.splitlines()
        step_at = next(i for i, ln in enumerate(lines, 1) if step in ln)
        assert audit_at[0] > step_at, (
            f"{name}: the audit record (line {audit_at[0]}) is written before "
            f"the FSM steps (line {step_at})")


def test_a_ticks_audit_record_and_log_row_carry_the_same_tick_number():
    """The ROS 2 node numbered a tick's audit record len(traj) BEFORE the row
    was appended and the row len(traj) AFTER it, so audit tick k was log row
    k + 1 (every audit.jsonl it wrote; the 2026-10-07 flight ros2fix2_on_fsm
    agrees with its log on all 82 records only at that offset). Both now take
    one name, bound once per tick; the pymavlink rail always did."""
    for name, row_tick in (("ros2_shield_node.py", "build_row"),
                           ("run_sitl_demo.py", None)):
        src = (SITL / name).read_text(encoding="utf-8")
        tree = ast.parse(src)
        call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == "audit_tick")
        arg = call.args[1]
        assert isinstance(arg, ast.Name), (
            f"{name}: audit_tick's tick is {ast.unparse(arg)!r}, an expression "
            f"evaluated apart from the row's")
        if row_tick:
            row = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and isinstance(n.func, ast.Name) and n.func.id == row_tick)
            kw = next(k.value for k in row.keywords if k.arg == "tick")
            assert isinstance(kw, ast.Name) and kw.id == arg.id, (
                name, ast.unparse(kw), arg.id)
        else:
            assert f'"tick": {arg.id},' in src, name


def test_both_rails_give_the_audit_logger_the_episode_id():
    """Every audit record then names its episode (guardrail.replay refuses a
    record naming another one)."""
    for name in ("run_sitl_demo.py", "ros2_shield_node.py"):
        tree = ast.parse((SITL / name).read_text(encoding="utf-8"))
        made = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == "AuditLogger"]
        assert len(made) == 1, (name, len(made))
        kw = {k.arg: ast.unparse(k.value) for k in made[0].keywords}
        assert "episode_id" in kw and "episode_id" in kw["episode_id"], (name, kw)


def test_fsm_theta_basis_is_stated_and_the_proxy_is_used_until_shield_reports_magnitude():
    N = _node()
    from guardrail.shield import Repair
    h = N.default_theta_horizon()
    if "magnitude_m" in Repair.model_fields:
        assert h is None
    else:
        assert h == N.PROXY_HORIZON_S == 0.1
        basis = N.theta_basis(h)
        assert "cannot fire" in basis["note"], basis


# --------------------------------------------------------------------------- #
# The rails flown headless (2026-10-11): stand-ins for AirSim and pymavlink
# --------------------------------------------------------------------------- #
#
# demo/run_demo.py needs AirSim and sitl/run_sitl_demo.py a MAVLink link to
# ArduPilot; neither has a stub mode. These tests fly their main() against a
# point mass that integrates the last velocity command on a virtual clock, so
# the code between "connect" and "report" runs here, on any machine, without a
# simulator. They prove code paths and records, not flight dynamics.

import contextlib                                                     # noqa: E402
import dataclasses                                                    # noqa: E402
import importlib.util                                                 # noqa: E402
import types                                                          # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

DEMO = ROOT / "demo"


class _Sim:
    """A point mass: position (North, East, up) and the last velocity command,
    integrated by the virtual clock's sleep()."""

    def __init__(self, up: float = 0.0):
        self.t = 0.0
        self.x = self.y = 0.0
        self.up = up
        self.v = (0.0, 0.0, 0.0)          # North, East, up (m/s)
        self.modes: list = []             # set_mode arguments, in order

    def time(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        dt = float(dt)
        self.x += self.v[0] * dt
        self.y += self.v[1] * dt
        self.up = max(0.0, self.up + self.v[2] * dt)
        self.t += dt


@contextlib.contextmanager
def _argv(args: list[str]):
    old = sys.argv
    sys.argv = list(args)
    try:
        yield
    finally:
        sys.argv = old


def _load(path: Path, name: str, fakes: dict | None = None):
    """A FRESH copy of a script as a module (its own globals, so a test's
    patches never leak), with `fakes` standing in for modules this machine
    may not have; the fakes are removed from sys.modules again afterwards."""
    fakes = fakes or {}
    saved = {k: sys.modules.get(k) for k in fakes}
    sys.modules.update(fakes)
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


# ---------------------------------------------------------------- run_demo.py

def _fake_airsim(sim: _Sim):
    """Just enough of the airsim package for demo/run_demo.py."""
    air = types.ModuleType("airsim")
    air.ImageType = types.SimpleNamespace(Scene=0)

    class _Done:
        def __init__(self, fn=None):
            self.fn = fn

        def join(self):
            if self.fn is not None:
                self.fn()

    class MultirotorClient:
        def confirmConnection(self):
            pass

        def enableApiControl(self, on):
            pass

        def armDisarm(self, on):
            pass

        def takeoffAsync(self):
            return _Done(lambda: setattr(sim, "up", 3.0))

        def landAsync(self):
            return _Done(lambda: setattr(sim, "up", 0.0))

        def simGetVehiclePose(self):
            pos = types.SimpleNamespace(x_val=sim.x, y_val=sim.y, z_val=-sim.up)
            return types.SimpleNamespace(position=pos)

        def moveByVelocityAsync(self, vx, vy, vz_ned, duration=0.0):
            sim.v = (float(vx), float(vy), -float(vz_ned))
            return _Done()

        def simGetImage(self, camera, kind):
            return None                   # no camera: run_demo skips the frame

    air.MultirotorClient = MultirotorClient
    return air


def _fly_run_demo(args: list[str], patch=None) -> tuple[Path, str]:
    """demo/run_demo.py main() on the stand-in simulator; returns (out dir,
    report.md text)."""
    sim = _Sim()
    air = _fake_airsim(sim)
    RD = _load(DEMO / "run_demo.py", "run_demo_headless", {"airsim": air})
    RD.airsim = air
    RD.time = types.SimpleNamespace(time=sim.time, sleep=sim.sleep)
    tmp = Path(tempfile.mkdtemp(prefix="run_demo_"))
    RD.ROOT = tmp                         # demo/out/<tag> lands in the temp dir
    if patch is not None:
        patch(RD)
    with _argv(["run_demo.py", "--policy", str(POLICY), "--tag", "t"] + args):
        assert RD.main() == 0
    out = tmp / "demo" / "out" / "t"
    return out, (out / "report.md").read_text(encoding="utf-8")


def _report_value(report: str, label: str) -> str:
    for line in report.splitlines():
        if line.startswith(f"| {label} |") or line.startswith(f"| **{label}** |"):
            return line.split("|")[2].strip().strip("*").strip()
    raise AssertionError(f"no '{label}' row in report.md:\n{report}")


def test_run_demo_sends_its_dynamic_zone_as_a_dynamic_nfz():
    """compiler-conform review: `--dynamic` hot-applied a PolygonFence after
    mission start, which Shield.hot_apply refuses (LockedRuleClass) since
    2026-10-07. Shown failing first, then the fix: the same zone as the
    dynamic_nfz it stands for, counted by the dwell from when it went live."""
    from guardrail.shield import LockedRuleClass
    RD = _load(DEMO / "run_demo.py", "run_demo_unit", {"airsim": _fake_airsim(_Sim())})
    policy = load_policy(POLICY)
    shield = Shield(policy, lookahead_s=3.0, dt=0.5)
    shield.filter(State(x=0.0, y=0.0, up=15.0), Action4D(vx=1.0))   # mission start
    try:
        shield.hot_apply(RD.DYNAMIC_FENCE)            # what main() did until today
    except LockedRuleClass:
        pass
    else:
        raise AssertionError("the Shield took a polygon_fence after mission start")
    zone = RD.apply_dynamic_zone(shield, RD.DYNAMIC_FENCE)
    assert zone.type == "dynamic_nfz" and zone.id == RD.DYNAMIC_FENCE.id
    assert policy.generation == 1 and zone.motion is None
    assert [(v.x, v.y) for v in zone.vertices] == [(v.x, v.y) for v in RD.DYNAMIC_FENCE.vertices]
    pts = [{"t": t, "x": 27.0, "y": 10.0, "up": 15.0} for t in (7.0, 8.0, 9.0)]
    d = RD.nfz_dwell(shield, pts, spawn_t=8.0)
    assert d["nfz_by_zone_s"] == {RD.DYNAMIC_FENCE.id: 0.2}, d
    # The spawn tick logged as t = 8.0 (the trajectory rounds t to 0.01 s)
    # while the clock read 8.004: that tick still counts.
    d = RD.nfz_dwell(shield, pts, spawn_t=8.004)
    assert d["nfz_by_zone_s"] == {RD.DYNAMIC_FENCE.id: 0.2}, d
    # The count it replaced, policy.by_type(PolygonFence), sees no zone here.
    from guardrail.models import PolygonFence
    assert RD.DYNAMIC_FENCE.id not in [f.id for f in policy.by_type(PolygonFence)]


def test_run_demo_flies_dynamic_headless_with_the_shield_on():
    """`python demo/run_demo.py --shield on --dynamic`, end to end. Before
    today this died on its first shielded tick (AuditLogger.policy_hash is
    read-only since 2026-08-26), and with that fixed, at t = 8 s
    (LockedRuleClass)."""
    out, report = _fly_run_demo(["--shield", "on", "--dynamic"])
    assert _report_value(report, "Target reached") == "yes", report
    assert "generation 1" in _report_value(report, "Dynamic NFZ"), report
    assert "nfz-dynamic" in _report_value(report, "Zones counted"), report
    assert _report_value(report, "Time inside NFZ") == "0.0 s", report
    # The grant's file name since 2026-11-02; the old one is not written.
    from guardrail.audit import AUDIT_FILE, LEGACY_AUDIT_FILE
    assert not (out / LEGACY_AUDIT_FILE).exists()
    recs = [json.loads(l) for l in (out / AUDIT_FILE).read_text(encoding="utf-8").splitlines()]
    assert len({r["policy_hash"] for r in recs}) == 2, (
        "the audit records must carry the generation after the hot-apply")


def test_run_demo_counts_time_inside_its_dynamic_zone_with_the_shield_off():
    """The control arm, with the zone put on the straight path and spawned
    early: the dwell names it, and only from when it went live."""
    from guardrail.models import PolygonFence, XY

    def on_the_path(RD):
        RD.DYNAMIC_AT_S = 2.0
        RD.DYNAMIC_FENCE = PolygonFence(
            id="nfz-dynamic", type="polygon_fence", **HARD_P0, margin_m=1.0,
            vertices=[XY(x=24, y=24), XY(x=28, y=24), XY(x=28, y=28), XY(x=24, y=28)])
    out, report = _fly_run_demo(["--shield", "off", "--dynamic"], on_the_path)
    by_zone = _report_value(report, "Time inside NFZ, by zone")
    assert "nfz-dynamic" in by_zone and "nfz-square" in by_zone, report
    assert float(_report_value(report, "Time inside NFZ").split()[0]) > 0.0, report
    assert _report_value(report, "Moving zones NOT counted") == "none", report


# ----------------------------------------------------------- run_sitl_demo.py

MODE_IDS = {"STABILIZE": 0, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9, "BRAKE": 17}


def _fly_pymavlink_rail(args: list[str], patch=None, path: Path | None = None):
    """sitl/run_sitl_demo.py main() against a stand-in autopilot. Returns
    (out dir, metrics, rows, events, the sim, the module)."""
    sim = _Sim()
    fakes = {}
    if importlib.util.find_spec("pymavlink") is None:
        pm = types.ModuleType("pymavlink")
        pm.mavutil = types.SimpleNamespace()
        fakes["pymavlink"] = pm
    R = _load(path or (SITL / "run_sitl_demo.py"), "run_sitl_demo_headless", fakes)

    class _M:
        target_system = target_component = 1

        def mode_mapping(self):
            return dict(MODE_IDS)

        def set_mode(self, mode_id):
            sim.modes.append(mode_id)

        def motors_armed(self):
            return True

    class _Link:
        def __init__(self, url):
            self.m = _M()

        def autopilot_fence(self, policy, cruise_alt_m):
            return {"FENCE_ENABLE": None, "FENCE_ALT_MAX": None,
                    "fence_alt_max_raised_from": None, "AVOID_ENABLE": None,
                    "source": "stand-in (tests/test_sitl_rails.py)"}

        def prepare(self, alt_m):
            sim.up = float(alt_m)

        def state(self, timeout=1.0):
            return State(x=sim.x, y=sim.y, up=sim.up)

        def send_velocity(self, vx, vy, vz_up, yaw_rate):
            sim.v = (float(vx), float(vy), float(vz_up))

        def land_disarm(self):            # the real one sets LAND
            sim.v = (0.0, 0.0, 0.0)
            sim.modes.append(MODE_IDS["LAND"])

    R.MavlinkAdapter = _Link
    R.ardupilot_version_from_mavlink = lambda m: {"ardupilot_version": "stand-in"}
    R.sim_speedup_from_mavlink = lambda m: None
    R.time = types.SimpleNamespace(time=sim.time, sleep=sim.sleep)
    tmp = Path(tempfile.mkdtemp(prefix="sitl_rail_"))
    R.ROOT = tmp
    if patch is not None:
        patch(R)
    # A YAML policy flies only as a development run (--dev-yaml) since
    # 2026-10-12; tests/test_railinputs_policy.py checks the refusal.
    with _argv(["run_sitl_demo.py", "--policy", str(POLICY), "--dev-yaml",
                "--tag", "t"] + args):
        assert R.main() == 0
    out = tmp / "demo" / "out" / "t"
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    rows = [json.loads(l) for l in (out / "flight_log.jsonl").read_text(encoding="utf-8").splitlines()]
    events = [json.loads(l) for l in (out / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    return out, metrics, rows, events, sim, R


def _spy_and_force_a_hold(seen: list, hold_at_s: float = 2.0):
    """Record every tick_input_from_decision call, and make the FSM ask for
    LOITER once (its own state stays as it is): the rail must send BRAKE."""
    def patch(R):
        real_tick = R.tick_input_from_decision

        def spy(*a, **kw):
            seen.append(kw)
            return real_tick(*a, **kw)
        R.tick_input_from_decision = spy

        class OneHold(R.EscalationFSM):
            forced = False

            def step(self, inp):
                out = super().step(inp)
                if not OneHold.forced and inp.t >= hold_at_s:
                    OneHold.forced = True
                    out = dataclasses.replace(out, set_mode="LOITER",
                                              reason="test: the FSM asks for a hold")
                return out
        R.EscalationFSM = OneHold
    return patch


def test_the_pymavlink_rail_wires_the_fsm_as_the_other_two_rails():
    """fsm-standoff-fix review: this rail alone still flew theta on the 0.1 s
    velocity proxy, gave the FSM no illegal depth and no applied caps, and
    sent the FSM's LOITER as ArduCopter's LOITER, which sinks with nobody on
    the throttle (14.2 m to the ground on 7 Oct)."""
    seen: list = []
    out, metrics, rows, events, sim, R = _fly_pymavlink_rail(
        ["--shield", "on"], _spy_and_force_a_hold(seen))
    assert seen, "the FSM never stepped"
    assert all(kw.get("horizon_s") is None for kw in seen), \
        "theta must read each repair's own magnitude_m (DEFAULT_THETA_HORIZON_S)"
    assert all("unsafe_depth_m" in kw for kw in seen), "X6/X7 got no illegal depth"
    assert all(kw.get("caps") is not None for kw in seen), \
        "the FSM must get the caps the Shield clamped against (or False)"
    assert metrics["fsm"]["theta_basis"] == {"source": "per-operator magnitude_m",
                                             "horizon_s": None}, metrics["fsm"]
    assert metrics["fsm"]["hold_mode"] == {"fsm_mode": "LOITER", "autopilot_mode": "BRAKE"}
    # Every request in both words, and what was SENT is ArduCopter's word:
    # the FSM's LOITER as BRAKE, never as LOITER.
    fsm_words, ap_words = metrics["mode_requests"], metrics["autopilot_mode_requests"]
    assert "LOITER" in fsm_words, fsm_words
    assert ap_words == [R.autopilot_mode(m) for m in fsm_words], (fsm_words, ap_words)
    assert "LOITER" not in ap_words and "BRAKE" in ap_words, ap_words
    assert sim.modes[:len(ap_words)] == [MODE_IDS[m] for m in ap_words], (sim.modes, ap_words)
    assert MODE_IDS["LOITER"] not in sim.modes, sim.modes
    req = [e for e in events if e["kind"] == "mode_request"]
    assert [(e["mode"], e["autopilot_mode"]) for e in req] == \
        list(zip(fsm_words, ap_words))[:len(req)], req
    fsm_rows = [r for r in rows if "fsm_state_after" in r]
    assert fsm_rows and all("unsafe_depth_m" in r for r in fsm_rows)
    assert any(isinstance(r["unsafe_depth_m"], float) for r in fsm_rows)
    held = [r for r in fsm_rows if r.get("set_mode") == "LOITER"]
    assert held and all(r["autopilot_set_mode"] == "BRAKE" for r in held), held
    # Zone dwell: the shared count, its fields in metrics.json.
    assert metrics["nfz_moving_not_counted"] == [] and "nfz_by_zone_s" in metrics


def test_the_pymavlink_rail_selects_the_one_period_reading_only_when_asked():
    seen: list = []
    _, metrics, *_ = _fly_pymavlink_rail(
        ["--shield", "on", "--theta-horizon-s", "0.1"], _spy_and_force_a_hold(seen))
    assert {kw.get("horizon_s") for kw in seen} == {0.1}
    assert metrics["fsm"]["theta_basis"]["horizon_s"] == 0.1
    assert "one-period" in metrics["fsm"]["theta_basis"]["source"]


def test_the_pymavlink_rail_holds_a_refused_fsm_input_as_brake():
    """A Shield/FSM contract break mid-flight: the rail holds the aircraft as
    the FSM's Loiter is held, BRAKE, and records both words."""
    calls = {"n": 0}

    def patch(R):
        real_tick = R.tick_input_from_decision

        def refuse_fifth(*a, **kw):
            calls["n"] += 1
            if calls["n"] == 5:
                raise ValueError("test: the FSM refused this input")
            return real_tick(*a, **kw)
        R.tick_input_from_decision = refuse_fifth
    _, metrics, rows, events, sim, _ = _fly_pymavlink_rail(["--shield", "on"], patch)
    assert sim.modes[0] == MODE_IDS["BRAKE"], sim.modes
    assert MODE_IDS["LOITER"] not in sim.modes
    assert metrics["fsm"]["fault"] == "test: the FSM refused this input"
    assert metrics["mode_requests"][0] == "LOITER"
    assert metrics["autopilot_mode_requests"][0] == "BRAKE"
    assert any(e["kind"] == "fsm_fault" for e in events)
    assert all(r["setpoint"] == "none" for r in rows[4:]), "a faulted rail streamed"


def _loiter_at_a_moved_point(east_m: float = 6.0):
    """Patch: the FSM holds its Loiter (T_recover 20 s; the stand-in's flight
    to (30, 30) enters it by G3 at 0.1 s, nfz-square's repair over theta),
    and the hold-point search answers a point `east_m` east of the aircraft -
    the plan guardrail/hold_flight.py must fly."""
    from guardrail import holdpoint as HP
    from guardrail.fsm import FSMConfig, HoldPoint

    def patch(R):
        class LongLoiter(R.EscalationFSM):
            def __init__(self, cfg=None):
                super().__init__(FSMConfig(t_recover_s=20.0, loiter_timeout_s=60.0))
        R.EscalationFSM = LongLoiter

        def plan_for(shield, fsm, state, **kw):
            return HoldPoint("LOITER", "moved", state.x, state.y + east_m, state.up,
                             legal=True, travel_s=east_m / 4.0, speed_mps=4.0,
                             climb_mps=2.0, reason="test: a point east")
        # The rail plans through a HoldPlanner (the per-tick bound): this plan
        # charges no budget, so it is taken on the tick it is asked for.
        R.holdpoint = types.SimpleNamespace(plan_for=plan_for,
                                            hold_point_legal=lambda *a, **k: True,
                                            HoldPlanner=HP.HoldPlanner)
    return patch


def test_the_pymavlink_rail_flies_a_moved_hold_then_holds():
    """2026-11-07 (guardrail/hold_flight.py, the mode mapping): a Loiter whose
    hold the search moved is flown in GUIDED to the point - the FSM's LOITER
    is not sent on entry - and the hold (BRAKE) is sent on arrival. The
    transit rows keep setpoint "none" (no VLA action flown) and carry
    `hold_transit`; `--hold-points in_place` holds where the aircraft is, at
    once, as before."""
    _, metrics, rows, events, sim, R = _fly_pymavlink_rail(
        ["--shield", "on"], _loiter_at_a_moved_point())
    assert metrics["fsm"]["hold_points"]["choice"] == "moved"
    assert metrics["fsm"]["hold_points"]["moved_transits"] >= 1, metrics["fsm"]
    g3 = next(r for r in rows if r.get("fsm_edge") == "G3")
    plans = [e for e in events if e["kind"] == "hold_point"]
    # The plan's own `kind` is recorded as data_kind (EpisodeRecord.event).
    assert plans and plans[0]["data_kind"] == "moved" and abs(plans[0]["t"] - g3["t"]) < 1e-6
    holds = [e for e in events if e["kind"] == "mode_request" and e["mode"] == "LOITER"]
    assert holds and holds[0]["autopilot_mode"] == "BRAKE", holds
    tr = [r for r in rows if "hold_transit" in r and r["t"] < holds[0]["t"]]
    # The entry tick streams the FSM's stop (its hand-over), as the harness
    # flies it; the transit starts on the next one.
    assert g3["setpoint"] == "brake" and "hold_transit" not in g3, g3
    assert tr and abs(tr[0]["t"] - g3["t"] - 0.1) < 1e-6, (tr[0]["t"], g3["t"])
    assert all(r["setpoint"] == "none" for r in tr), tr[:3]
    assert all(r["hold_transit"][1] > 0.0 for r in tr), "the transit flew east"
    flight = [e for e in events if e["kind"] == "hold_flight" and e["t"] <= holds[0]["t"]]
    assert "GUIDED transit" in flight[0]["note"] and "reached" in flight[-1]["note"], flight
    assert holds[0]["t"] > tr[-1]["t"], "the hold is sent after the transit"
    # In place: the old hold, sent on the entry tick, nothing streamed.
    _, m2, rows2, ev2, _, _ = _fly_pymavlink_rail(
        ["--shield", "on", "--hold-points", "in_place"], _loiter_at_a_moved_point())
    assert m2["fsm"]["hold_points"] == {"choice": "in_place", "moved_transits": 0}
    assert not any("hold_transit" in r for r in rows2)
    entry = next(r for r in rows2 if r.get("fsm_edge") == "G3")
    first = next(e for e in ev2 if e["kind"] == "mode_request")
    assert (first["mode"], first["autopilot_mode"]) == ("LOITER", "BRAKE")
    assert abs(first["t"] - entry["t"]) < 1e-6, (first, entry["t"])


def _zone_on_the_path(at_s: float):
    """Patch: the --dynamic zone put on the straight path to (30, 30) and
    applied `at_s` seconds in. The stand-in, shield off, is inside its
    footprint from t = 5.7 s to 6.8 s."""
    from guardrail.models import PolygonFence, XY

    def patch(R):
        R.DYNAMIC_AT_S = at_s
        R.DYNAMIC_FENCE = PolygonFence(
            id="nfz-dynamic", type="polygon_fence", **HARD_P0, margin_m=1.0,
            vertices=[XY(x=24, y=24), XY(x=28, y=24), XY(x=28, y=28), XY(x=24, y=28)])
    return patch


def _in_footprint(traj: list, fence) -> list:
    """Trajectory points inside a zone's AUTHORED polygon and altitude band,
    worked out here, apart from guardrail.replay.zone_dwell."""
    from shapely.geometry import Point
    from guardrail.geometry import fence_polygon
    poly = fence_polygon(fence)
    return [p for p in traj if poly.contains(Point(p["x"], p["y"]))
            and fence.altitude_floor_m <= p["up"] <= fence.altitude_ceiling_m]


def test_the_pymavlink_rail_counts_its_dynamic_zone_from_when_it_went_live():
    """The control arm through a hot-applied dynamic_nfz on the straight path,
    applied while the aircraft is already inside it: counted (the rails' old
    by_type(PolygonFence) count would have missed it), and only from the tick
    it went live, that tick included.

    rails-integration review: this test used to apply the zone before the
    aircraft got there, so it asserted only that no point was in the zone
    before the spawn, and dropping live_from from the rail survived it. Now
    the aircraft is in the footprint on both sides of the spawn, and the
    count must equal the in-footprint ticks from the spawn on.

    The loop here takes 3 ms per tick besides its 0.1 s sleep, as a real one
    does, so the clock does not land on the 0.01 s grid. That showed a second
    defect: the rail compared the trajectory's t, rounded to 0.01 s, with the
    unrounded spawn time, so the spawn tick (t = 6.283 s, logged 6.28) was
    dropped whenever the clock read past the rounded value: on a real clock,
    any spawn reading in the lower half of a 0.01 s step."""
    on_the_path = _zone_on_the_path(6.2)

    def patch(R):
        on_the_path(R)
        sleep = R.time.sleep
        R.time = types.SimpleNamespace(time=R.time.time,
                                       sleep=lambda dt: sleep(dt + 0.003))
    out, metrics, rows, events, _, R = _fly_pymavlink_rail(
        ["--shield", "off", "--dynamic"], patch)
    spawn = next(e["t"] for e in events if e["kind"] == "policy_generation"
                 and e.get("generation") == 1)
    traj = json.loads((out / "trajectory.json").read_text(encoding="utf-8"))
    inside = _in_footprint(traj, R.DYNAMIC_FENCE)
    before = [p for p in inside if p["t"] < round(spawn, 2)]
    after = [p for p in inside if p["t"] >= round(spawn, 2)]
    assert before and after, (
        f"the aircraft must be in the footprint before ({len(before)}) and after "
        f"({len(after)}) the spawn at {spawn}, or the test proves nothing")
    assert after[0]["t"] == round(spawn, 2), (after[0], spawn)
    got = metrics["nfz_by_zone_s"].get("nfz-dynamic")
    assert got == round(len(after) * R.TICK, 3), (
        f"nfz-dynamic {got} s; {len(after)} ticks from the spawn on "
        f"({len(before)} before it must not count)")
    assert metrics["nfz_by_zone_s"].get("nfz-square", 0) > 0, metrics["nfz_by_zone_s"]
    assert abs(metrics["nfz_s"] - sum(metrics["nfz_by_zone_s"].values())) < 0.051


def test_the_pymavlink_rail_lands_at_mission_end_and_records_it():
    """A mission that ends without a fail-safe (shield off: target reached)
    asks for LAND, and that request is on record in both vocabularies."""
    _, metrics, _, _, sim, _ = _fly_pymavlink_rail(["--shield", "off"])
    assert metrics["reached"] is True, metrics.get("reached")
    assert metrics["mode_requests"] == ["LAND"], metrics["mode_requests"]
    assert metrics["autopilot_mode_requests"] == ["LAND"], metrics["autopilot_mode_requests"]
    assert sim.modes == [MODE_IDS["LAND"]], sim.modes


class _PoseStream:
    """pymavlink as MavlinkAdapter.state() meets it: one LOCAL_POSITION_NED
    every 100 ms of a virtual clock (ms), until `last_ms` if given.
    recv_match hands back the OLDEST unread message (pymavlink reads only the
    bytes it needs, so the backlog waits in the socket buffer); None when
    nothing new has arrived and it may not block; when it may, it waits for
    the next arrival, or returns None after `timeout` if none comes."""

    def __init__(self, last_ms: float | None = None):
        self.now_ms = 0.0
        self.read = 0                     # index of the oldest unread message
        self.last_ms = last_ms

    def _arrived(self) -> int:
        t = self.now_ms if self.last_ms is None else min(self.now_ms, self.last_ms)
        return int(t // 100) + 1          # message k arrives at k * 100 ms

    def recv_match(self, type=None, blocking=False, timeout=None):
        assert type == "LOCAL_POSITION_NED", type
        if self.read >= self._arrived():
            due = self.read * 100.0
            if not blocking:
                return None
            if (self.last_ms is not None and due > self.last_ms) \
                    or due - self.now_ms > timeout * 1000.0:
                self.now_ms += timeout * 1000.0
                return None
            self.now_ms = due
        k, self.read = self.read, self.read + 1
        return types.SimpleNamespace(time_boot_ms=k * 100, x=float(k), y=0.0, z=-15.0)


def _pymavlink_rail_module():
    fakes = {}
    if importlib.util.find_spec("pymavlink") is None:
        pm = types.ModuleType("pymavlink")
        pm.mavutil = types.SimpleNamespace()
        fakes["pymavlink"] = pm
    return _load(SITL / "run_sitl_demo.py", "run_sitl_demo_pose", fakes)


def test_the_pymavlink_rail_flies_on_the_newest_pose_not_the_oldest():
    """bh_final sitl-ros2/pymavlink-fifo-stale-pose: state() took ONE
    blocking recv_match per tick, which returns pymavlink's oldest unread
    message. The loop ran at 8.9-9.4 Hz against the 10 Hz stream, so the
    pose the Shield, the FSM and the stub used fell further behind all
    flight (6.6 s by 110 s on sitl_ped_on). Driven here at 9 Hz for 60 s:
    the pose handed back must stay within one stream period of the newest,
    and a stream that stops must read as no pose, not as an old one."""
    R = _pymavlink_rail_module()
    link = object.__new__(R.MavlinkAdapter)           # __init__ would connect
    link.m = stream = _PoseStream(last_ms=60_000.0)
    ages = []
    for i in range(540):                              # 9 Hz, 60 s
        stream.now_ms = max(stream.now_ms, i * 1000.0 / 9.0)
        st = link.state()
        assert st is not None, f"no pose at tick {i}"
        ages.append(stream.now_ms - st.x * 100.0)     # x carries the message index
    assert max(ages) <= 100.0, (
        f"the pose fell {max(ages) / 1000:.1f} s behind the stream (oldest-first "
        f"queue); ages at 10/30/60 s: {ages[89]:.0f} / {ages[269]:.0f} / {ages[-1]:.0f} ms")
    stream.now_ms = 61_000.0                          # the stream stopped at 60 s
    st = link.state(timeout=1.0)
    assert st is not None and st.x * 100.0 == 60_000.0, "the last pose sent was not read"
    st = link.state(timeout=1.0)
    assert st is None, f"a dead stream returned a pose from {st.x * 100:.0f} ms"


def test_the_pymavlink_rail_ticks_at_10_hz_however_long_the_work_takes():
    """The loop slept a fixed TICK AFTER the work, so it ticked every 0.1 s
    plus the work. zone_dwell, alt_seconds and standoff_seconds count ticks x
    TICK, which matched the poses only while each tick consumed one 0.1 s
    message from the queue; with the newest pose read each tick, the ticks
    themselves must be TICK apart. Here every tick's work takes 7 ms of the
    virtual clock (inside sleep, where the stand-in's clock moves)."""
    def patch(R):
        sleep = R.time.sleep
        R.time = types.SimpleNamespace(time=R.time.time,
                                       sleep=lambda dt: sleep(dt + 0.007))
    _, metrics, rows, _, _, R = _fly_pymavlink_rail(["--shield", "off"], patch)
    t = [r["t"] for r in rows]
    assert len(t) > 20, f"only {len(t)} ticks flown"
    dts = sorted(b - a for a, b in zip(t[1:], t[2:]))  # the first tick sets the phase
    mean = (t[-1] - t[1]) / (len(t) - 2)
    assert abs(dts[len(dts) // 2] - R.TICK) < 0.0015 and abs(mean - R.TICK) < 0.0015, (
        f"ticks {mean * 1000:.1f} ms apart (median {dts[len(dts) // 2] * 1000:.1f} ms), "
        f"not {R.TICK * 1000:.0f}")


def test_the_pymavlink_rail_still_lets_go_of_its_lock_after_an_overrun():
    """Review of 2026-10-19 (P3): with the deadline pacing, a tick that
    overran slept 0.0 between lock.release() and lock.acquire(). The lock is
    not fair, so the loop could take it straight back, and under a sustained
    overrun a REST hot-apply waiting on it could wait indefinitely (before,
    every tick slept a full TICK). Here every tick's work takes 150 ms: each
    sleep between ticks must still be a real one (LOCK_YIELD_S, 1 ms), and
    only that - the deadline resyncs, so there is no catch-up and no slack
    left over from a missed deadline."""
    slept = []

    def patch(R):
        sleep = R.time.sleep

        def overrunning(dt):
            slept.append(dt)
            sleep(dt + 0.15)
        R.time = types.SimpleNamespace(time=R.time.time, sleep=overrunning)
    _, _, rows, _, _, R = _fly_pymavlink_rail(["--shield", "off"], patch)
    assert len(rows) > 20 and len(slept) > 20, (len(rows), len(slept))
    assert min(slept) >= 0.001, (
        f"the loop let go of its lock for {min(slept) * 1000:.1f} ms after an "
        f"overrun ({sum(1 for s in slept if s <= 0.0)} of {len(slept)} sleeps were 0)")
    floor = [s for s in slept if s < 0.01]
    assert len(floor) >= len(slept) - 1 and max(floor) <= R.LOCK_YIELD_S + 1e-9, (
        f"overrun ticks slept {sorted(set(round(s, 4) for s in slept))} s, not the floor")


def _parse(parser, argv):
    """parse_known_args, with argparse's exit (status 2) as a test failure
    instead of the end of the run."""
    try:
        return parser.parse_known_args(argv)
    except SystemExit as e:
        raise AssertionError(f"{argv} refused by the parser (exit {e.code})")


def test_every_rail_takes_a_negative_x_for_its_xy_options():
    """bh_final sitl-ros2/xy-args-negative-crash: argparse takes a separate
    value that starts with '-' for an option unless it reads as a negative
    NUMBER, and '-20,15' does not (the comma). So `--target -20,15` - a
    target south of home - exited with status 2 on the Shield node and the
    VLA stub, and `--subject -5.5,3` on the pymavlink rail, on the 3.11 and
    3.12.3 interpreters the rails run; the stress harness passes the value as
    its own argv element (experiments/sweep_scenarios.py build_sitl_cmd)."""
    N = _node()
    import ros2_vla_stub_node as V
    for argv, want in ((["--target", "-20,15", "--subject", "-5.5,3"], ("-20,15", "-5.5,3")),
                       (["--target=-20,15", "--subject=-5.5,3"], ("-20,15", "-5.5,3")),
                       (["--target", "-.5,15", "--subject", "-5,-3"], ("-.5,15", "-5,-3"))):
        a, rest = _parse(N.make_parser(), argv)
        assert (a.target, a.subject, rest) == (*want, []), (argv, a.target, a.subject, rest)
    a, rest = _parse(V.make_parser(), ["--target", "-20.5,15", "--yaw-rate", "0.2"])
    assert (a.target, a.yaw_rate, rest) == ("-20.5,15", 0.2, []), (a, rest)
    a, _ = _parse(N.make_parser(), ["--target", "20,-15"])
    assert a.target == "20,-15"
    # A real option after the flag is never taken for its value.
    from mavlink_adapter_node import join_negative_xy as J
    assert J(["--target", "--tag", "x"]) == ["--target", "--tag", "x"]
    assert J(["--subject", "-5,3", "--target", "1,2"]) == ["--subject=-5,3", "--target", "1,2"]
    # The pymavlink rail, through its own main().
    try:
        _, metrics, *_ = _fly_pymavlink_rail(["--shield", "off", "--subject", "-5.5,3"])
    except SystemExit as e:
        raise AssertionError(f"run_sitl_demo.py refused --subject -5.5,3 (exit {e.code})")
    assert (metrics["subject"]["x"], metrics["subject"]["y"]) == (-5.5, 3.0), metrics["subject"]


def test_the_shield_node_routes_mode_requests_to_the_adapter_by_name():
    """bh_final sitl-ros2/mode-request-fallback-dead-under-bag: rosbag2
    subscribes to /shield/mode_request in every recorded run, so 'any
    subscriber' could never fall back to MAVROS with the adapter down."""
    N = _node()
    assert N.adapter_listening(["mavlink_adapter"])
    assert N.adapter_listening(["rosbag2_recorder", "mavlink_adapter"])
    assert not N.adapter_listening(["rosbag2_recorder"])
    assert not N.adapter_listening([])
    import mavlink_adapter_node as A
    assert A.ADAPTER_NODE_NAME == "mavlink_adapter"
    src = (SITL / "mavlink_adapter_node.py").read_text(encoding="utf-8")
    assert "super().__init__(ADAPTER_NODE_NAME)" in src


def test_the_shield_node_watches_the_pose_stream_with_its_own_timeout():
    """bh_final sitl-ros2/frozen-pose-undetected: a pose that stopped was
    re-checked every tick and nothing said so. The node now runs a second
    watchdog on the pose; its default sits under the Shield's lookahead, so
    the hold comes while the forecast from the last pose still covers the
    path, and above the 0.6-0.7 s simulator stall seen on ros2if_on_dyn_yaw."""
    N = _node()
    assert 0.7 < N.POSE_TIMEOUT_S < N.LOOKAHEAD_S
    a, _ = _parse(N.make_parser(), [])
    assert a.pose_timeout_s == N.POSE_TIMEOUT_S
    w = N.ActionWatchdog(a.pose_timeout_s, what="pose")
    w.start(0.0)
    for t in (0.1, 0.2, 0.3):
        w.received(t)
        assert w.check(t) is None
    assert w.check(1.25) is None and w.check(1.35) == "stale"
    assert abs(w.age(1.35) - 1.05) < 1e-9 and w.stale
    w.received(1.4)
    assert w.check(1.4) == "resumed" and not w.stale
    try:
        N.ActionWatchdog(0.0, what="pose")
    except ValueError as e:
        assert "pose timeout" in str(e), e
    else:
        raise AssertionError("a zero pose timeout was accepted")


def test_the_pymavlink_rail_writes_each_generations_cost_map_when_asked():
    """--cost-map: every generation's CSP names its map, both files are on
    disk and load as that map, and the replay bundle carries them and still
    re-derives its KPIs. Without the flag: no map, no ref."""
    from guardrail.compiler import load_cost_map
    from guardrail.replay import EPISODE, read_replay, verify_replay
    out, metrics, _, events, _, _ = _fly_pymavlink_rail(
        ["--shield", "on", "--dynamic", "--cost-map"])
    gens = [e for e in events if e["kind"] == "policy_generation"]
    assert [e["generation"] for e in gens] == [0, 1], gens
    assert all(e.get("cost_map_ref") for e in gens), gens
    assert not [e for e in events if e["kind"] == "cost_map_failed"]
    for g in (0, 1):
        csp = json.loads((out / f"csp_g{g}.json").read_text(encoding="utf-8"))
        assert csp["cost_map_ref"] == gens[g]["cost_map_ref"], (g, csp["cost_map_ref"])
        cm = load_cost_map(out / f"costmap_csp_g{g}.json", ref=csp["cost_map_ref"])
        assert cm.generation == g and (out / f"costmap_csp_g{g}.npz").is_file()
    bundle = out / f"{out.name}.replay.tar.gz"
    members = read_replay(bundle)["index"]["members"]
    for g in (0, 1):
        for ext in ("npz", "json"):
            assert f"{EPISODE}costmap_csp_g{g}.{ext}" in members, sorted(members)
    ok, why = verify_replay(bundle)
    assert ok, why
    # Off by default.
    out2, _, _, events2, _, _ = _fly_pymavlink_rail(["--shield", "off", "--dynamic"])
    assert not list(out2.glob("costmap_*")), list(out2.glob("costmap_*"))
    assert all("cost_map_ref" not in e for e in events2
               if e["kind"] == "policy_generation")


def test_the_ros2_rail_passes_cost_map_and_live_from():
    """The ROS 2 node cannot fly here (rclpy), so its wiring is read: the
    --cost-map flag reaches EpisodeRecord, and the dynamic zone's spawn time
    reaches zone_dwell as live_from, on the trajectory's 0.01 s grid - since
    2026-10-12 beside every zone spawned over the REST hot-apply API
    (guardrail.api.FlightHotApply.live_from), which zone_dwell counted over
    the whole flight before."""
    N = _node()
    assert N.make_parser().parse_args(["--cost-map"]).cost_map is True
    assert N.make_parser().parse_args([]).cost_map is False
    tree = ast.parse((SITL / "ros2_shield_node.py").read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    recs = [n for n in calls if getattr(n.func, "id", None) == "EpisodeRecord"]
    assert len(recs) == 1, len(recs)
    kw = {k.arg: ast.unparse(k.value) for k in recs[0].keywords}
    assert kw.get("cost_map") in ("args.cost_map", "getattr(args, 'cost_map', False)"), kw
    dwell = [n for n in calls if getattr(n.func, "id", None) == "zone_dwell"]
    assert len(dwell) == 1, len(dwell)
    kw = {k.arg: k.value for k in dwell[0].keywords}
    assert "live_from" in kw, list(kw)
    # The shape, not just the names: {zone: spawn time on the 0.01 s grid}
    # once the zone went live, the REST-spawned zones beside it, None when
    # there is none (a `None and {...}` or a constant condition would still
    # name both).
    assert ast.unparse(kw["live_from"]) == "live or None", ast.unparse(kw["live_from"])
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    body = src[src.index("def _report"):src.index("self.dwell = zone_dwell(")]
    assert "live = dict(self.hot.live_from) if self.hot is not None else {}" in body
    assert "if self.dynamic_fence_id and self.spawn_t is not None:" in body
    assert "live[self.dynamic_fence_id] = round(self.spawn_t, 2)" in body


# ------------------------------------------------------- every rail, one count

DWELL_RAILS = ("demo/run_demo.py", "demo/real_vla_demo.py", "demo/aerialvla_demo.py",
               "demo/aerialvla_pas_demo.py", "demo/semantic_demo.py",
               "demo/semantic_seek.py", "demo/pas_ardupilot/perception_node.py",
               "sitl/run_sitl_demo.py", "sitl/ros2_shield_node.py")


def test_every_rail_counts_zone_dwell_through_the_shield():
    """nfz_s, which guardrail/kpi.py reads for the mission outcome, is one
    count on every rail (guardrail.replay.zone_dwell, demo/follow_vlm.py's
    count). Until 2026-10-11 each rail walked policy.by_type(PolygonFence)
    itself, so a declared dynamic_nfz scored 0 s (models.FLIGHT_HELD_TYPES);
    the two SITL rails counted it, scoring a moving one where it was declared."""
    for rel in DWELL_RAILS:
        src = (ROOT / rel).read_text(encoding="utf-8")
        tree = ast.parse(src)
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and getattr(n.func, "id", getattr(n.func, "attr", "")) == "zone_dwell"]
        assert calls, f"{rel} does not count zone dwell through zone_dwell"
        assert "poly.contains(Point(" not in src, f"{rel} still walks polygons for dwell"
    for rel in DWELL_RAILS[1:]:
        src = (ROOT / rel).read_text(encoding="utf-8")
        if '"nfz_s"' in src:
            assert '"nfz_moving_not_counted"' in src, (
                f"{rel} writes nfz_s without naming the zones it could not count")


def test_the_pas_planner_routes_around_every_static_zone_and_leaves_moving_ones():
    """demo/aerialvla_pas_demo.planning_zones feeds the A* planner and the
    proximity slow-down. Since 2026-10-11 it holds polygon and circle fences
    and every dynamic_nfz without motion (until then: polygon fences only);
    a moving dynamic_nfz is left to the Shield, which places it each tick."""
    from guardrail.models import Policy
    sq = [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}, {"x": 0, "y": 10}]
    pol = Policy.model_validate({"policy_id": "plan", "constraints": [
        {"id": "poly", "type": "polygon_fence", **HARD_P0, "vertices": sq},
        {"id": "circ", "type": "circle_fence", **HARD_P0, "center": {"x": 40, "y": 0},
         "radius_m": 5},
        {"id": "dyn", "type": "dynamic_nfz", **HARD_P0, "vertices": sq},
        {"id": "mov", "type": "dynamic_nfz", **HARD_P0, "motion": {"vx_mps": 1.0}, "vertices": sq},
        {"id": "alt", "type": "altitude_envelope", **HARD_P0, "alt_min_m": 10, "alt_max_m": 20},
    ]})
    A = _load(DEMO / "aerialvla_pas_demo.py", "aerialvla_pas_demo_unit")
    assert [z.id for z in A.planning_zones(pol)] == ["poly", "circ", "dyn"]


def test_sim_stressors_are_only_the_simulators_own_parameters():
    """X-14: a stressor changes the simulated world, never the firmware under
    test, and a value that is not a finite number is refused. Since
    2026-11-07 the rule lives in sitl/sitl_stressors.py, shared with the ROS 2
    rails' simulator side, and the pymavlink rail imports it from there
    rather than keeping a copy."""
    sys.path.insert(0, str(SITL))
    from sitl_stressors import parse_sim_params as parse
    src = (SITL / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert "from sitl_stressors import" in src and "def parse_sim_params" not in src
    assert parse(None) == {}
    assert parse(["sim_wind_spd=6", "SIM_WIND_DIR=225", "SIM_GPS_DISABLE=1"]) == {
        "SIM_WIND_SPD": 6.0, "SIM_WIND_DIR": 225.0, "SIM_GPS_DISABLE": 1.0}
    for bad in ("FENCE_ENABLE=0", "WPNAV_SPEED=2000", "SIM_WIND_SPD", "SIM_=1",
                "SIM_WIND_SPD=fast", "SIM_WIND_SPD=nan", "SIM_WIND_SPD=inf"):
        try:
            parse([bad])
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")


def test_a_stressor_that_does_not_read_back_stops_the_run():
    src = (SITL / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert '"sim_stressors": sim_stressors' in src, "metrics.json must record the stressors"
    tail = src.split("sim_stressors = []", 1)[1][:1500]
    assert "did not read back as" in tail and "raise SystemExit" in tail


# --------------------------------------------------------------------------- #
# The MAVROS evidence and the six-field manifest (WP4-12), one module for the
# GUIDED Shield node and the mission-style node (2026-11-06)
# --------------------------------------------------------------------------- #

GRANT_FIELDS = ["code_revision", "vla_model_hash", "policy_hash", "random_seed",
                "sim_speedup", "topology"]
DESKTOP = {"host_arch": "x86_64", "device_model": None, "l4t_release": None,
           "jetpack": None, "hostname": "desktop"}


class _Param:
    """An rcl_interfaces/ParameterValue as MAVROS returns it."""

    def __init__(self, type, double_value=0.0, integer_value=0, string_value=""):
        self.type, self.double_value = type, double_value
        self.integer_value, self.string_value = integer_value, string_value


def _vehicle_info(available_info=2):
    """MAVROS's /mavros/vehicle_info_get answer for ArduCopter 4.5.7 (official
    build, git 2a3dc4b7); available_info bit 2 = AUTOPILOT_VERSION received."""
    from types import SimpleNamespace
    v = SimpleNamespace(available_info=available_info,
                        flight_sw_version=(4 << 24) | (5 << 16) | (7 << 8) | 255,
                        flight_custom_version=list(b"2a3dc4b7"), type=2, autopilot=3)
    return SimpleNamespace(success=True, vehicles=[v])


def test_the_mavros_evidence_is_read_from_the_autopilot_never_assumed():
    """sitl/mavros_evidence.py, the readers both ROS 2 rail nodes use: a
    parameter as MAVROS returns it (SIM_SPEEDUP sometimes INTEGER with the
    value in double_value), SIM_SPEEDUP polled through MAVROS's refill race
    (0.0, then the value) and None - never 1.0 - when it never answers, a
    hardware autopilot told apart by SYSID_THISMAV without SIM_*, the
    autopilot's own AUTOPILOT_VERSION with the setup banner only as a
    labelled fallback, MAVROS's own fcu_url before the launch script's, and
    vla_host only where a separate VLA node reports one."""
    from types import SimpleNamespace
    sys.path.insert(0, str(SITL))
    import mavros_evidence as E
    assert E.param_value(_Param(0)) is None
    assert E.param_value(_Param(4, string_value="udp://:14555@")) == "udp://:14555@"
    assert E.param_value(_Param(2, integer_value=1)) == 1.0
    assert E.param_value(_Param(2, double_value=1.0)) == 1.0

    def gone(*a):
        raise RuntimeError("service gone")
    assert E.get_params(gone, None, dict, ["SIM_SPEEDUP"]) == {}
    assert E.get_params(lambda *a: None, None, dict, ["SIM_SPEEDUP"]) == {}
    assert E.get_params(lambda c, req, t: SimpleNamespace(values=[_Param(3, 1.0)]),
                        None, dict, ["SIM_SPEEDUP"]) == {"SIM_SPEEDUP": 1.0}

    clk = [0.0]

    def spin():
        clk[0] += 0.5

    def now():
        return clk[0]
    answers = iter([{"SIM_SPEEDUP": 0.0, "SYSID_THISMAV": 1.0},
                    {"SIM_SPEEDUP": 1.0, "SYSID_THISMAV": 1.0}])
    ev, warned = {}, []
    assert E.poll_sim_speedup(lambda names: next(answers), spin, ev, now=now) == 1.0
    assert ev == {"autopilot_kind": "sitl"}
    assert E.poll_sim_speedup(lambda names: {}, spin, ev, now=now, warn=warned.append) is None
    assert ev == {"autopilot_kind": None} and warned == ["[speedup] SIM_SPEEDUP never resolved"]
    assert E.poll_sim_speedup(lambda names: {"SIM_SPEEDUP": None, "SYSID_THISMAV": 1.0},
                              spin, ev, now=now) is None
    assert ev == {"autopilot_kind": "hardware"}

    ev, warned = {}, []
    E.poll_autopilot_version(_vehicle_info, spin, ev, now=now, environ={})
    assert ev["ardupilot_version"] == "ArduCopter V4.5.7 (2a3dc4b7)", ev
    assert ev["ardupilot_version_source"] == "AUTOPILOT_VERSION via /mavros/vehicle_info_get"
    ev = {}
    E.poll_autopilot_version(lambda: _vehicle_info(available_info=0), spin, ev, now=now,
                             environ={"ARDUPILOT_VERSION": "ArduCopter V4.5.7 (2a3dc4b7)",
                                      "ARDUPILOT_PIN_OK": "1"}, warn=warned.append)
    assert "banner" in ev["ardupilot_version_source"] and ev["ardupilot_pin_ok"] is True
    ev = {}
    E.poll_autopilot_version(gone, spin, ev, now=now, environ={}, warn=warned.append)
    assert ev == {} and warned == ["autopilot version not read"], (ev, warned)

    ev = {}
    E.record_link("udp://:14555@127.0.0.1:14550", ev, environ={"MAVLINK_ROUTER": "routerd"},
                  scan=lambda: {"local_sitl": ["arducopter"], "local_mavros": True,
                                "local_router": []})
    assert ev["network_link"]["source"] == "/mavros fcu_url parameter"
    assert ev["network_link"]["fcu_url"] == "udp://:14555@127.0.0.1:14550"
    assert ev["mavlink_router"] == "routerd" and ev["mavros_on"] == "shield_host"
    ev = {}
    E.record_link(None, ev, environ={"MAVROS_FCU_URL": "udp://:14555@"}, scan=lambda: None)
    assert ev["network_link"]["source"] == "MAVROS_FCU_URL (launch script)"
    assert ev["network_link"]["fcu_url"] == "udp://:14555@" and ev["mavros_on"] is None

    shield = E.initial_evidence(vla_node=True, environ={"ROS_DISTRO": "jazzy"}, host=DESKTOP)
    mission = E.initial_evidence(vla_node=False, environ={"ROS_DISTRO": "jazzy"}, host=DESKTOP)
    assert shield["vla_host"] is None and "vla_host" not in mission
    assert mission["ros_distro"] == "jazzy" and mission["fcu_connected"] is None
    assert "ros2_security" not in mission and mission["hostname"] == "desktop"
    E.note_mavros_state(mission, 1)
    assert mission["mavros_node"] == "/mavros" and mission["fcu_connected"] is True


def test_both_ros2_rail_nodes_gather_their_evidence_through_one_module():
    """The mission-style node (2026-11-03) gathered no evidence and wrote no
    manifest. Both nodes now gather it with sitl/mavros_evidence.py and build
    the manifest with its topology_manifest: neither keeps a second copy of
    the readers or the topology rule, and neither asserts a speedup."""
    ev_src = (SITL / "mavros_evidence.py").read_text(encoding="utf-8")
    assert "describe_autopilot_version(" in ev_src and "detect_topology(" in ev_src
    for name in ("ros2_shield_node.py", "ros2_mission_node.py"):
        src = (SITL / name).read_text(encoding="utf-8")
        assert "from mavros_evidence import" in src, name
        for call in ("initial_evidence(", "note_mavros_state(", "poll_sim_speedup(",
                     "poll_autopilot_version(", "record_link(", "topology_manifest("):
            assert call in src, (name, call)
        assert "def read_sim_speedup" in src and "GetParameters" in src, name
        for copy in ("describe_autopilot_version(", "detect_topology(",
                     "network_link_evidence(", "sim_speedup=1.0"):
            assert copy not in src, (name, copy)


def test_the_mission_node_takes_the_shield_nodes_seed():
    """random_seed is the seed the run was given: --seed on both nodes, the
    same default (run_ros2_demo.sh hands its extra args to whichever node
    takes the Shield's place)."""
    sys.path.insert(0, str(SITL))
    import ros2_mission_node as M
    N = _node()
    assert M.make_parser().parse_known_args([])[0].seed \
        == N.make_parser().parse_known_args([])[0].seed == 0
    assert M.make_parser().parse_known_args(["--seed", "7"])[0].seed == 7


def test_a_mission_run_whose_mavros_evidence_does_not_hold_keeps_its_manifest():
    """A mission refused before MAVROS ever connected (main() ends it through
    _finish) still writes the six fields: the grant label is refused on the
    evidence and the rail's own label stamped, the refusal is in metrics
    (`topology_refused`), and an unread speedup is "unresolved", not 1.0."""
    sys.path.insert(0, str(SITL))
    import mavros_evidence as E
    import ros2_mission_node as M
    from guardrail.manifest import NO_VLA, TOPOLOGY_ARDUPILOT_SITL, UNRESOLVED
    pol = load_policy(POLICY)
    out = Path(tempfile.mkdtemp(prefix="mission_refused_"))
    metrics, warned = {"rail": "ros2_mavros_mission"}, []
    man = M.write_run_records(out, metrics, policy_hash=pol.policy_hash, seed=3,
                              sim_speedup=None, warn=warned.append,
                              evidence=E.initial_evidence(vla_node=False, environ={},
                                                          host=DESKTOP))
    assert list(man) == GRANT_FIELDS, man
    assert man["topology"] == TOPOLOGY_ARDUPILOT_SITL and man["sim_speedup"] == UNRESOLVED
    assert man["vla_model_hash"] == NO_VLA and man["random_seed"] == 3
    assert man["policy_hash"] == pol.policy_hash
    disk = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert disk["topology"] == TOPOLOGY_ARDUPILOT_SITL and "fcu_connected" in disk["topology_refused"]
    assert json.loads((out / "manifest.json").read_text(encoding="utf-8")) == man
    assert warned and warned[0].startswith("not the dev topology"), warned


def _mission_standin(path: Path = SITL / "ros2_mission_node.py"):
    """sitl/ros2_mission_node.py on tests/test_railinputs_node.py's stand-in
    ROS stack (no rclpy, no MAVROS, no autopilot), plus the two mission
    message types the mission node adds. The modules it shares with the
    plain imports (the adapter's constants, the evidence readers) are
    imported first, without the stand-in, so they are not cached with it."""
    import importlib.util
    sys.path.insert(0, str(SITL))
    sys.path.insert(0, str(ROOT / "tests"))
    import mavlink_adapter_node                                   # noqa: F401
    import mavros_evidence                                        # noqa: F401
    import test_railinputs_node as RN
    fakes = RN._fake_ros()
    for n in ("WaypointList", "WaypointReached"):
        setattr(fakes["mavros_msgs.msg"], n, type(n, (RN._Msg,), {}))
    saved = {k: sys.modules.get(k) for k in fakes}
    sys.modules.update(fakes)
    try:
        spec = importlib.util.spec_from_file_location("ros2_mission_node_standin", path)
        M = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(M)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    assert M.HAVE_ROS, "the stand-in ROS stack did not import"
    return RN, M


def test_the_mission_node_writes_the_six_field_manifest_beside_its_metrics():
    """WP4-12 on the mission style. The real MissionShieldNode, on the
    stand-in ROS stack, is brought up against a stand-in MAVROS that answers
    as SITL does (connected, AUTOPILOT_VERSION, its fcu_url, SIM_SPEEDUP 0.0
    and then 1.0 - MAVROS's refill race), flies under a zone hot-applied
    after take-off, and ends. manifest.json lies beside metrics.json with the
    grant's six fields: NO_VLA, the --seed it was given, the speedup the
    autopilot reported, topology dev from the evidence, and the FINAL policy
    hash - the last generation, which the replay pack binds the bundle to.
    metrics.json carries the topology, the evidence and the take-off hash.
    Fails on the node as of 0029a43 (mission_wp_20261104 has no manifest)."""
    import os
    from types import SimpleNamespace
    from guardrail.manifest import NO_VLA
    from guardrail.models import DynamicNFZ
    from guardrail.replay import last_generation_policy, read_replay, write_replay
    RN, M = _mission_standin()
    clk = RN._Clock()
    clk.perf_counter = clk.monotonic           # the tick times its own Shield work
    M.time = clk
    M.rclpy.spin_once = lambda *a, **kw: setattr(clk, "t", clk.t + 0.5)
    args, _ = M.make_parser().parse_known_args(
        ["--policy", str(POLICY), "--dev-yaml", "--waypoints", "0,30;30,30", "--seed", "7"])
    out = Path(tempfile.mkdtemp(prefix="mission_standin_"))
    keys = ("ROS_DISTRO", "MAVROS_FCU_URL", "MAVLINK_ROUTER", "ARDUPILOT_VERSION",
            "ROS_SECURITY_ENABLE")
    saved = {k: os.environ.get(k) for k in keys}
    for k in keys:
        os.environ.pop(k, None)
    os.environ["ROS_DISTRO"] = "jazzy"            # the rail sources ROS 2 Jazzy
    try:
        node = M.MissionShieldNode(args, out)
        speedups = iter([0.0, 1.0])
        asked = []
        # getattr: a node that asks MAVROS nothing (0029a43) still flies here,
        # and the test then fails where it should, on the missing manifest.
        info, link, params = (getattr(node, n, object()) for n in
                              ("cli_info", "cli_mavros_param", "cli_param"))

        def call(client, req, timeout=5.0):
            if client is info:
                return _vehicle_info()
            if client is link:
                return SimpleNamespace(values=[_Param(4, string_value="udp://:14555@127.0.0.1:14550")])
            if client is params:
                asked.append(list(req.names))
                return SimpleNamespace(values=[_Param(3, next(speedups, 1.0)),
                                               _Param(2, integer_value=1)])
            return SimpleNamespace(success=True, mode_sent=True)   # rate, mode, arm, take-off
        node._call = call
        node._on_state(M.MavState(connected=True, armed=False, mode="GUIDED"))
        node.state = State(x=0.0, y=0.0, up=node.alt)
        node._on_adapter(M.String(data=json.dumps(
            {"kind": "mission_upload", "success": True, "match": True, "items": 4,
             "read_back": 4})))
        loaded = node.policy.policy_hash
        node.bring_up()
        node.start()
        node._on_state(M.MavState(connected=True, armed=True, mode="AUTO"))
        zone = DynamicNFZ(id="nfz-hot-1", type="dynamic_nfz", **HARD_P0, margin_m=1.0,
                          vertices=[XY(x=-4, y=18), XY(x=4, y=18), XY(x=4, y=22),
                                    XY(x=-4, y=22)])
        with node._lock:                    # as the REST hot-apply thread does
            node.shield.hot_apply(zone)
            node.rec.hot_applied(zone.id, t=node._t())
        for _ in range(3):
            clk.t += 0.1
            node._tick()
        node._finish("last waypoint reached")
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    assert (out / "manifest.json").is_file(), "the mission node wrote no manifest.json"
    man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    met = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert list(man) == GRANT_FIELDS, man
    assert man["vla_model_hash"] == NO_VLA and man["random_seed"] == 7 == met["seed"]
    assert man["sim_speedup"] == 1.0 and len(asked) == 2, (man, asked)
    assert asked[0] == ["SIM_SPEEDUP", "SYSID_THISMAV"]
    assert man["topology"] == "dev" == met["topology"] and "topology_refused" not in met, met
    assert isinstance(man["code_revision"], str) and man["code_revision"]
    ev = met["hil_evidence"]
    assert ev["fcu_connected"] is True and ev["mavros_node"] == "/mavros"
    assert ev["ardupilot_version"] == "ArduCopter V4.5.7 (2a3dc4b7)", ev
    assert ev["autopilot_kind"] == "sitl" and "vla_host" not in ev
    assert ev["network_link"]["fcu_url"] == "udp://:14555@127.0.0.1:14550"
    # The take-off hash and the final one: the manifest carries the final
    # one, the hash of the last generation the episode recorded.
    final = last_generation_policy(out)
    assert final.generation == 1 and man["policy_hash"] == final.policy_hash
    assert met["policy_hash_final"] == man["policy_hash"] != loaded
    assert met["policy_hash_at_takeoff"] == loaded and met["generation_at_takeoff"] == 0
    assert met["generation_final"] == 1
    # ...which is what binds the replay bundle: "manifest", not "unverified".
    bundle = out / "standin.replay.tar.gz"
    write_replay(out, final, bundle, changelog="stand-in", signer=None,
                 policy_source=met["policy_source"])
    idx = read_replay(bundle)["index"]
    assert idx["policy_binding"] == "manifest" and idx["flown_policy_hash"] == final.policy_hash


# --------------------------------------------------------------------------- #
# A stress scenario on the ROS 2 rail (2026-11-06): the node takes the
# scenario's start, target, pilot and events (sitl/ros2_shield_node.py,
# sitl/ros2_vla_stub_node.py), driven on tests/test_railinputs_node.py's
# stand-in ROS stack
# --------------------------------------------------------------------------- #

CMAC = (-35.363261, 149.16523)


def _scenario_node(argv):
    """sitl/ros2_shield_node.py on the stand-in ROS stack, built from its own
    parser with `argv` (the dev YAML sim_demo policy, frame origin CMAC) on a
    virtual clock; nothing ticks until start_mission. (RN, N, node, clk, out)."""
    sys.path.insert(0, str(ROOT / "tests"))
    import test_railinputs_node as RN
    N = RN._node_module()
    clk = RN._Clock()
    N.time = clk
    args, _ = N.make_parser().parse_known_args(
        ["--policy", str(POLICY), "--dev-yaml"] + list(argv))
    out = Path(tempfile.mkdtemp(prefix="scenario_node_"))
    return RN, N, N.ShieldNode(args, out), clk, out


def _ekf_pose(RN, N, node, north=0.0, east=0.0, up=15.0, yaw=0.0):
    """MAVROS's pose: metres about the EKF origin (SITL: home)."""
    node._on_pose(RN._pose_msg(N, State(x=north, y=east, up=up, yaw_deg=yaw)))


def _fly(RN, N, node, clk, n, **pose):
    for _ in range(n):
        _ekf_pose(RN, N, node, **pose)
        RN._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()


def test_a_scenario_start_puts_every_pose_and_the_home_in_the_policy_frame():
    """--start X,Y,UP: SITL boots at the scenario's start, so MAVROS's local
    frame is about (X, Y) of the policy's; every pose is offset by it, the
    pad before the first pose is there, and the fences document names it
    (`home_policy_ne`, which the adapter projects the zones about). The null:
    without --start the pose is the policy frame's, as every run before."""
    RN, N, node, clk, out = _scenario_node(["--start", "-200,0,15", "--target=-80,0,12"])
    assert node.start_xy == (-200.0, 0.0) and node.start_up == 15.0
    assert (node.mission.target_x, node.mission.target_y) == (-80.0, 0.0)
    assert node.mission.cruise_alt_m == 12.0, "the target's altitude is the cruise's"
    node.fence_column_m = 30.0
    node._publish_fences_locked("take-off")
    doc = json.loads(node.pubs[N.TOPIC_FENCES].sent[-1].data)
    assert doc["home_policy_ne"] == [-200.0, 0.0] and doc["position"] == [-200.0, 0.0], doc
    _ekf_pose(RN, N, node, north=3.0, east=4.0, up=15.0, yaw=90.0)
    st = node.state
    assert (st.x, st.y, st.up) == (-197.0, 4.0, 15.0) and abs(st.yaw_deg - 90.0) < 1e-6, st
    # A zone over the start (not over the frame origin) is the one an RTL lands in.
    zone = Shield(load_policy(POLICY)).zones_now(15.0)[0].polygon.centroid
    RN2, N2, over, _, _ = _scenario_node([f"--start={zone.x},{zone.y},15"])
    over.fence_column_m = 30.0
    over._publish_fences_locked("take-off")
    assert over.zones_over_home == ["nfz-square"], over.zones_over_home
    # The null: no --start.
    RN3, N3, plain, _, _ = _scenario_node([])
    _ekf_pose(RN3, N3, plain, north=3.0, east=4.0)
    assert (plain.state.x, plain.state.y) == (3.0, 4.0)
    assert plain.zones_over_home == [] and plain.start_given is False


def test_a_home_that_is_not_the_scenarios_start_is_refused_before_arming():
    """The check that catches a SITL booted at the wrong home: the
    autopilot's home, projected about the policy's frame origin, must lie
    within 5 m of --start. Shown on home_check and on the node's check_home
    (which raises before GUIDED); a home that never arrives is refused after
    the wait, a metre policy has nothing to check, and without --start
    nothing is checked, as before."""
    N = _node()
    from guardrail.projection import LocalProjection
    proj = LocalProjection(*CMAC)
    lat, lon = proj.to_latlon(-200.0, 0.0)
    ok = N.home_check((lat, lon, (0.0, 0.0)), CMAC, (-200.0, 0.0))
    assert ok["checked"] and ok["ok"] and ok["miss_m"] < 0.01, ok
    assert N.home_check(proj.to_latlon(-200.0, 4.9) + ((0.0, 0.0),), CMAC, (-200.0, 0.0))["ok"]
    near = N.home_check(proj.to_latlon(-200.0, 5.1) + ((0.0, 0.0),), CMAC, (-200.0, 0.0))
    assert not near["ok"] and "SITL booted at another home" in near["why"], near
    wrong = N.home_check((CMAC[0], CMAC[1], (0.0, 0.0)), CMAC, (-200.0, 0.0))
    assert not wrong["ok"] and abs(wrong["miss_m"] - 200.0) < 0.01, wrong
    # Home 3 m from the EKF origin (a real autopilot's) is where the frame says.
    off = N.home_check(proj.to_latlon(-197.0, 0.0) + ((3.0, 0.0),), CMAC, (-200.0, 0.0))
    assert off["ok"], off
    assert not N.home_check(None, CMAC, (-200.0, 0.0))["ok"]
    metre = N.home_check(None, None, (-200.0, 0.0))
    assert metre["checked"] is False and metre["ok"] is True
    # On the node.
    RN, N1, node, clk, out = _scenario_node(["--start=-200,0,15"])
    node.home_geo = (CMAC[0], CMAC[1], (0.0, 0.0))            # booted at CMAC
    try:
        node.check_home()
    except RuntimeError as e:
        assert "refusing to arm" in str(e) and "SITL booted" in str(e), e
    else:
        raise AssertionError("a SITL booted 200 m from the start was armed")
    ev = [e for e in RN._events(out) if e["kind"] == "home_check"]
    assert ev and ev[-1]["ok"] is False and abs(ev[-1]["miss_m"] - 200.0) < 0.01, ev
    node.home_geo = (lat, lon, (0.0, 0.0))
    node.check_home()
    assert node.home_check["ok"] and node.home_check["checked"]
    # No home ever arrives: refused once the wait (on the node's clock) is over.
    RN2, N2, quiet, clk2, _ = _scenario_node(["--start=-200,0,15"])
    N2.rclpy.spin_once = lambda *a, **kw: setattr(clk2, "t", clk2.t + 0.5)
    try:
        quiet.check_home()
    except RuntimeError as e:
        assert "no home position" in str(e), e
    else:
        raise AssertionError("armed with no home position to check")
    assert quiet.home_check["waited_s"] >= N2.HOME_WAIT_S
    # Without --start: nothing checked, as before.
    _, _, plain, _, _ = _scenario_node([])
    plain.check_home()
    assert plain.home_check is None


def test_a_mission_with_no_target_flies_to_its_cap_and_declares_no_goal():
    """--no-target (a constant pilot): no reach test, and metrics.json
    `reached` / `frac_within_30m` null - no goal, so none missed. The null:
    the same position WITH a target there ends the mission at once."""
    RN, N, node, clk, out = _scenario_node(["--start=-30,-30,3", "--no-target",
                                            "--max-s", "0.75"])
    assert not node.has_target
    assert (node.mission.target_x, node.mission.target_y) == (-30.0, -30.0)
    _ekf_pose(RN, N, node, up=3.0)
    node.start_mission()
    _fly(RN, N, node, clk, 5, up=3.0)
    assert not node.done and len(node.rows) == 5
    _fly(RN, N, node, clk, 4, up=3.0)
    assert node.done
    m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert m["reached"] is None and m["frac_within_30m"] is None and m["target"] is None, m
    assert m["start"] == {"policy_ne": [-30.0, -30.0], "up": 3.0, "source": "--start"}
    start = [e for e in RN._events(out) if e["kind"] == "mission_start"][0]
    assert start["target"] is None and start["start"] == [-30.0, -30.0, 3.0]
    # The null.
    RN2, N2, aimed, clk2, out2 = _scenario_node(["--start=-30,-30,3", "--target=-30,-30,3",
                                                 "--reach-m", "3"])
    _ekf_pose(RN2, N2, aimed, up=3.0)
    aimed.start_mission()
    _fly(RN2, N2, aimed, clk2, 1, up=3.0)
    assert aimed.done and aimed.reached, "the target under the aircraft was not reached"
    assert json.loads((out2 / "metrics.json").read_text(encoding="utf-8"))["reached"] is True


def test_scenario_events_reach_the_flying_shield_and_every_record():
    """--events: a ScenarioEvent list applied in the tick once the mission
    time reaches each, through guardrail.scenario_events (the harness's own
    code) with the Shield given t=None. The spawn lands on the aircraft: a
    new generation (policy_g1.json), a scenario_event, live_from for the
    dwell, the fences re-published, and on its row `rule_events` and the
    previous tick's judgement (not unsafe, in no zone) - so the harness
    counts no breach and no polygon entry. An event the Shield refuses and
    the scenario does not expect refused is recorded, flown past and named
    in metrics.json. --scenario-meta goes into metrics.json."""
    tmp = Path(tempfile.mkdtemp(prefix="scenario_files_"))
    evs = [{"at_sim_t": 0.3, "type": "spawn_polygon_fence",
            "payload": {"id": "dyn-top", "width_m": 10, "height_m": 10,
                        "ahead_of_vehicle_m": 0, "anchor": "center"}},
           {"at_sim_t": 0.3, "type": "scale_radius",
            "payload": {"id": "no-such-zone", "factor": 2.0}}]
    (tmp / "events.json").write_text(json.dumps(evs), encoding="utf-8")
    meta = {"run_id": "t_001_x", "run_nonce": "n0", "backend": "hil",
            "template": "dynamic_nfz_movement", "cell": "x@y=1",
            "family": "hil-stress/dynamic_nfz_movement/x@y=1/shield-on"}
    (tmp / "scenario.json").write_text(json.dumps(meta), encoding="utf-8")
    RN, N, node, clk, out = _scenario_node(
        ["--start=-100,0,15", "--target=0,0,15", f"--events={tmp / 'events.json'}",
         f"--scenario-meta={tmp / 'scenario.json'}"])
    assert node._ev_declared == 2
    _ekf_pose(RN, N, node)
    node.start_mission()
    assert node._ev_heading == (1.0, 0.0), node._ev_heading     # start -> target
    sent = len(node.pubs[N.TOPIC_FENCES].sent)
    _fly(RN, N, node, clk, 6)
    assert node.policy.generation == 1 and (out / "policy_g1.json").is_file()
    recs = [e for e in RN._events(out) if e["kind"] == "scenario_event"]
    assert [r["event"] for r in recs] == ["spawn_polygon_fence", "scale_radius"], recs
    assert not recs[0].get("refused") and recs[0]["rule_id"] == "dyn-top"
    assert recs[1]["refused"] and recs[1]["expected"] is False, recs[1]
    assert node._ev_live_from == {"dyn-top": recs[0]["t"]}, node._ev_live_from
    assert len(node.pubs[N.TOPIC_FENCES].sent) > sent, "the fences did not follow"
    k = next(i for i, r in enumerate(node.rows) if r["rule_events"])
    row = node.rows[k]
    assert k > 0 and row["unsafe"] and not node.rows[k - 1]["unsafe"], row
    assert row["unsafe_prev_rules"] == [] and row["zones_in_prev"] == [], row
    assert row["zones_in"] == ["dyn-top"] and row["zone_d_m"] < 0, row
    assert all("clock" in r for r in node.rows)
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    m = S.rail_measurements(node.rows, node.policy)
    assert m["breaches"] == 0 and m["polygon_entries"] == 0, m
    assert m["ticks_inside_fence_polygon"] >= 1 and m["min_true_dist_to_fence_m"] < 0, m
    node._finish("test")
    met = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    sev = met["scenario_events"]
    assert sev["declared"] == 2 and len(sev["applied"]) == 1 and len(sev["refused"]) == 1
    assert sev["unexpected_refusals"] and sev["event_refusals_as_expected"] is False, sev
    assert met["scenario"]["run_id"] == "t_001_x"
    assert met["scenario_family"] == meta["family"]


def test_every_row_carries_the_rule_clock_and_the_previous_judgement_reads_its_own():
    """Each row records the rule clock's reading it was judged at, and the
    next tick's judgement of the position under the previous rules pins the
    clock to that reading for its one query - then releases it."""
    from datetime import datetime
    RN, N, node, clk, out = _scenario_node(["--clock-start", "2026-09-07T17:29:50"])
    _ekf_pose(RN, N, node)
    node.start_mission()
    _fly(RN, N, node, clk, 3)
    clocks = [datetime.fromisoformat(r["clock"]) for r in node.rows]
    assert clocks[0] >= datetime(2026, 9, 7, 17, 29, 50) and clocks == sorted(clocks)
    assert node._last_clock == clocks[-1] and node._clock_pin is None
    seen = []
    real = node.shield.state_is_unsafe

    def spy(st, **kw):
        seen.append(node.clock())
        return real(st, **kw)
    node.shield.state_is_unsafe = spy
    node._prev_judgement(node.state, True)
    assert seen == [clocks[-1]] and node._clock_pin is None, seen
    clk.t += 0.1                                         # unpinned again: it moves on
    assert node.clock() > clocks[-1]


def test_g9_home_needs_the_rtl_climb_as_the_harness_asks_it():
    """G9 on the ROS 2 rail (2026-11-07): home is reached at the END of RTL's
    return leg - over the start within REACH_M AFTER the climb to RTL_ALT
    (guardrail.rtl_path.home_reached, the harness's own test since the same
    day). Until then the node asked the distance alone, and on the rails
    home IS the scenario's start: rh01_038's RTL, entered beside the start
    at 3 m, reported home on its next tick and the mission ended below the
    floor. The null: the same tick at the return altitude is home."""
    from guardrail.rtl_path import home_reached
    assert not home_reached(0.0, 0.0, 3.0, (0.0, 0.0))
    assert home_reached(0.0, 0.0, 15.0, (0.0, 0.0)) and home_reached(1.9, 0.0, 14.95, (0.0, 0.0))
    assert not home_reached(2.1, 0.0, 15.0, (0.0, 0.0))
    assert home_reached(0.0, 0.0, 22.0, (0.0, 0.0)), "an RTL above RTL_ALT returns there"
    RN, N, node, clk, out = _scenario_node(["--start=-30,-30,3", "--no-target", "--max-s", "30"])
    seen = []
    real = N.tick_input_from_decision

    def spy(*a, **kw):
        seen.append(kw.get("home_reached"))
        return real(*a, **kw)
    N.tick_input_from_decision = spy
    try:
        _ekf_pose(RN, N, node, up=3.0)
        node.start_mission()
        node.mode = "RTL"
        _fly(RN, N, node, clk, 1, up=3.0)          # over the start, below the climb
        _fly(RN, N, node, clk, 1, up=15.0)         # over it at the return altitude
        node.mode = "GUIDED"
        _fly(RN, N, node, clk, 1, up=15.0)         # not in RTL: never home
    finally:
        N.tick_input_from_decision = real
    assert seen == [False, True, False], seen


def test_after_the_fsm_ends_the_mission_the_rail_logs_on_to_the_landing():
    """The ROS 2 rail no longer ends the episode on the FSM's terminal edge
    (G9 / G10 / X5) or FAILSAFE_GRACE_S after a fail-safe: it logs on until
    the autopilot has landed or disarmed, or the time cap (2026-11-07), so
    what RTL flies to its end is in the log (the stress harness gates its
    safety invariants over it). `mission_terminal` marks the edge once;
    metrics.json `after_terminal` says when and how it ended. The null: an
    aircraft still flying after the edge is not the end."""
    ends = []
    for how in ("landed", "disarmed"):
        RN, N, node, clk, out = _scenario_node(["--start=-30,-30,15", "--no-target",
                                                "--max-s", "30"])
        assert node.fsm is not None
        _ekf_pose(RN, N, node)
        node.start_mission()
        _fly(RN, N, node, clk, 2)
        node.fsm._terminal = True                     # G9: the FSM ended the mission
        _fly(RN, N, node, clk, 3)
        assert not node.done and len(node.rows) == 5, (node.done, len(node.rows))
        assert len([e for e in RN._events(out) if e["kind"] == "mission_terminal"]) == 1
        if how == "landed":
            node.landed = True
        else:
            node.armed = False
        _fly(RN, N, node, clk, 1)
        assert node.done, how
        m = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
        end = [e for e in RN._events(out) if e["kind"] == "mission_end"][-1]
        assert end["why"] == "escalation FSM ended the mission", end
        ends.append(m["after_terminal"]["ended"])
        assert m["after_terminal"]["terminal_t"] is not None
    assert ends == ["landed", "disarmed"], ends


def test_every_row_carries_the_decisions_trace_words_the_harness_counts():
    """`shield_trace` (2026-11-07): the words of the decision's trace the
    stress harness counts per tick (window_opens_on / window_opened_on,
    cap_refused / floor_refused: shield_trace_fields) on every row, {} when
    there are none, so an empty one is a measured nothing. The school zone's
    window opens at 07:30:00 on an aircraft 10 m inside its ring, 2 s away:
    every row before it names the zone (the anticipated count's reading).
    The null: the same hover at 06:00, the opening 90 minutes off, names
    nothing."""
    pol = ROOT / "policies" / "corridor_survey.yaml"
    names = []
    for clock in ("2026-09-07T07:29:58", "2026-09-07T06:00:00"):
        RN, N, node, clk, out = _scenario_node([f"--policy={pol}", "--clock-start", clock,
                                                "--start=60,21,15", "--target=60,55,15"])
        _ekf_pose(RN, N, node)
        node.start_mission()
        _fly(RN, N, node, clk, 3)
        assert node.rows and all(isinstance(r.get("shield_trace"), dict) for r in node.rows)
        names.append([r["shield_trace"].get("window_opens_on") for r in node.rows])
    assert names[0] == [["nfz-school"]] * 3, names[0]
    assert names[1] == [None] * 3, names[1]
    N = _node()
    from types import SimpleNamespace
    d = SimpleNamespace(trace={"window_opens_on": {"b", "a"}, "cap_refused": "flown at 2.00 m/s",
                               "floor_refused": "", "chain": [1, 2]})
    assert N.shield_trace_fields(d) == {"window_opens_on": ["a", "b"],
                                        "cap_refused": "flown at 2.00 m/s"}
    assert N.shield_trace_fields(SimpleNamespace(trace=None)) == {}


def test_the_vla_stub_flies_the_harness_laws_in_the_policy_frame():
    """sitl/ros2_vla_stub_node.py --pilot goto / constant / stub_vla: the
    laws are guardrail.vla_stub's, the harness's own (one implementation),
    on a pose offset by --start, turned into the body frame; each announces
    its model id, which the manifest pins by the module's source. Arguments
    a law cannot fly are refused before ROS starts, and a negative --start
    or --action parses as a value."""
    import math
    import ros2_vla_stub_node as V
    from guardrail.compiler import Mission
    from guardrail import vla_stub as VS
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    m0 = Mission(task_text="", target_x=30.0, target_y=30.0, cruise_alt_m=15.0,
                 speed_pref_mps=6.0)

    def parse(argv):
        return V.make_parser().parse_known_args(argv)[0]
    a = parse(["--pilot", "goto", "--start", "-200,0,15", "--target", "-80,0,15",
               "--speed", "4", "--shield", "on", "--tag", "x", "--events=e.json"])
    mission, tgt, start = V.mission_from_args(a, m0)
    pilot, rec = V.pilot_from_args(a, mission, tgt)
    assert rec["model_id"] == "guardrail.vla_stub.GotoPilot" and rec["approach_accel_mps2"] == 2.4
    north = (0.0, 0.0, math.sin(math.radians(45.0)), math.cos(math.radians(45.0)))
    st = V.pose_state(0.0, 0.0, 15.0, north, start)
    assert (st.x, st.y) == (-200.0, 0.0) and abs(st.yaw_deg) < 1e-9, st
    mm = S._Mission(start=st, target=(-80.0, 0.0, 15.0), subject=None, action=None, speed=4.0)
    assert pilot.act(st) == S._pilot(mm, "goto", st, VS.APPROACH_ACCEL_MPS2), "two laws"
    assert VS.APPROACH_ACCEL_MPS2 == S.AP_RESPONSE.accel_mps2 and VS.APPROACH_P == S.AP_POSXY_P
    body = V.body_action(pilot, st, 0.0)
    assert abs(body[0] - 4.0) < 1e-9 and abs(body[1]) < 1e-9, body      # forward, North
    c = parse(["--pilot", "constant", "--action", "-3,0,0,0", "--no-target"])
    assert c.action == "-3,0,0,0"
    cm, ct, _ = V.mission_from_args(c, m0)
    cp, crec = V.pilot_from_args(c, cm, ct)
    east = (0.0, 0.0, 0.0, 1.0)                                    # ENU yaw 0: heading East
    st_e = V.pose_state(0.0, 0.0, 15.0, east)
    assert abs(st_e.yaw_deg - 90.0) < 1e-9
    b = V.body_action(cp, st_e, None)
    assert abs(b[0]) < 1e-9 and abs(b[1] - 3.0) < 1e-9, b      # South, heading East: right
    assert crec["model_id"] == "guardrail.vla_stub.ConstantPilot"
    for bad in (["--pilot", "goto", "--no-target"], ["--pilot", "constant"],
                ["--pilot", "constant", "--action=nan,0,0"], ["--pilot", "goto", "--action=1,0,0"],
                ["--start=1,2,3,4"], ["--pilot", "stub_vla", "--speed", "-1"]):
        try:
            V.check_args(parse(bad))
        except ValueError:
            pass
        else:
            raise AssertionError(f"{bad} was accepted")
    V.check_args(parse(["--start=-1,2", "--target=3,4"]))
    ident = V.identity_record(load_policy(POLICY), {"kind": "yaml"}, mission, yaw_rate=0.0,
                              seed=1, host={}, pilot=rec, start=start)
    assert ident["model_id"] == rec["model_id"] and ident["pilot"]["type"] == "goto"
    assert ident["start"] == [-200.0, 0.0, 15.0] and ident["target"] == [-80.0, 0.0]
    from guardrail.manifest import model_hash
    assert model_hash(rec["model_id"]).startswith("guardrail.vla_stub.GotoPilot@src:")
    # The Shield node reads a negative --start and --action as values too.
    N = _node()
    ns, _ = N.make_parser().parse_known_args(["--start", "-200,0,15", "--action", "-3,0,0,0"])
    assert ns.start == "-200,0,15"


def test_the_shield_node_learns_landed_from_the_autopilot():
    """WP3-07, G10 / X5 (2026-11-07). ArduCopter puts EXTENDED_SYS_STATE in
    no REQUEST_DATA_STREAM group: every hil run of 6-7 Nov recorded 0
    messages on /mavros/extended_state, so "landed" never reached the FSM
    and G10 never fired after rh01_032's G12 landing. The node asks for
    message 245 by id through /mavros/set_message_interval (allowed in its
    SROS 2 profile) and, until extended_state arrives, reads ArduCopter's
    disarm on touchdown in RTL / LAND."""
    N = _node()
    f = N.landed_from
    assert f("LAND", True, True, True) and f("RTL", True, False, True)
    assert not f("LAND", False, False, True), "extended_state, once sent, decides"
    assert f("LAND", None, False, True), "no extended_state: the disarm after flight"
    assert not f("LAND", None, False, False), "disarmed before it ever armed"
    assert not f("LAND", None, True, True) and not f("LAND", None, None, True)
    assert not f("GUIDED", True, False, True), "only RTL / LAND, as before"
    assert not f("BRAKE", None, False, True) and not f(None, None, False, True)
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "MSG_EXTENDED_SYS_STATE = 245" in src
    assert '"/mavros/set_message_interval"' in src
    assert "self.request_extended_state()" in src
    assert "landed=self.landed_input()," in src
    assert 'landed=bool(self.landed) and self.mode in ("RTL", "LAND")' not in src
    pol = (SITL / "sros2_policy.xml").read_text(encoding="utf-8")
    shield = pol.split('<profile ns="/" node="safety_shield">')[1].split("</profile>")[0]
    assert "<service>/mavros/set_message_interval</service>" in shield


def test_no_sitl_script_reads_a_name_nothing_binds():
    """The ROS node classes exist only where rclpy imports, so no desktop test
    runs their methods. a152a75's request_extended_state logged through
    `log`, a local of bring_up: on the AGX it raised NameError right after
    FCU connect and every mission of the hold shakedown hs01 (7 Nov) ended
    with no flight. symtable sees every function's free names without
    importing anything: each must be bound at module level or a builtin."""
    import builtins
    import symtable
    allowed = set(dir(builtins)) | {"__file__", "__name__"}
    for path in sorted(SITL.glob("*.py")):
        top = symtable.symtable(path.read_text(encoding="utf-8"), str(path), "exec")
        bound = {s.get_name() for s in top.get_symbols()
                 if s.is_assigned() or s.is_imported() or s.is_namespace()}
        unbound = []

        def walk(t):
            for s in t.get_symbols():
                if (s.is_referenced() and s.is_global() and not s.is_declared_global()
                        and s.get_name() not in bound and s.get_name() not in allowed):
                    unbound.append(f"{t.get_name()}:{t.get_lineno()} {s.get_name()}")
            for c in t.get_children():
                walk(c)

        walk(top)
        assert not unbound, f"{path.name}: names nothing binds: {unbound}"


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
