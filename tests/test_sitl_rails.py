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
        id="nfz-dynamic", type="polygon_fence",
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
    src = (SITL / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "VehicleInfoGet" in src and "describe_autopilot_version(" in src


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
    # and the GeoFence kept it on the pad).
    assert 'pos, pos_src = (0.0, 0.0), "assumed home' in src
    assert "self._recheck_deferred(st," in src
    assert "def read_autopilot_fence" in src and '"backstop_active"' in src
    assert "fence_alt_plan(" in src and '"FENCE_ALT_MAX"' in src
    assert "read_fence_alt_max(FENCE_PARM) or 30.0" not in src
    # A 2026-10-07 flight read FENCE_ENABLE before MAVROS had FENCE_ALT_MAX,
    # so no zone was exported: the read waits for all of them, and a column
    # still unknown at altitude is read again before the mission.
    assert "all(vals.get(k) is not None for k in FENCE_PARAMS)" in src
    assert '"fence parameters read late"' in src
    # A hardware autopilot's GeoFence is GCS-set: only SITL's is changed.
    assert "and is_sitl:" in src
    # The take-off publish waits for the autopilot's values and a position.
    i = src.index("def bring_up")
    body = src[i:src.index("def start_mission")]
    assert body.index("self.read_autopilot_fence()") < \
        body.index('self._publish_fences("take-off")')


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
    recs = [json.loads(l) for l in (out / "audit.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len({r["policy_hash"] for r in recs}) == 2, (
        "the audit records must carry the generation after the hot-apply")


def test_run_demo_counts_time_inside_its_dynamic_zone_with_the_shield_off():
    """The control arm, with the zone put on the straight path and spawned
    early: the dwell names it, and only from when it went live."""
    from guardrail.models import PolygonFence, XY

    def on_the_path(RD):
        RD.DYNAMIC_AT_S = 2.0
        RD.DYNAMIC_FENCE = PolygonFence(
            id="nfz-dynamic", type="polygon_fence", margin_m=1.0,
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
    with _argv(["run_sitl_demo.py", "--policy", str(POLICY), "--tag", "t"] + args):
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


def _zone_on_the_path(at_s: float):
    """Patch: the --dynamic zone put on the straight path to (30, 30) and
    applied `at_s` seconds in. The stand-in, shield off, is inside its
    footprint from t = 5.7 s to 6.8 s."""
    from guardrail.models import PolygonFence, XY

    def patch(R):
        R.DYNAMIC_AT_S = at_s
        R.DYNAMIC_FENCE = PolygonFence(
            id="nfz-dynamic", type="polygon_fence", margin_m=1.0,
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
    reaches zone_dwell as live_from, on the trajectory's 0.01 s grid."""
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
    # once the zone went live, None before (a `None and {...}` or a constant
    # condition would still name both).
    lf = kw["live_from"]
    assert isinstance(lf, ast.IfExp) and isinstance(lf.body, ast.Dict), ast.unparse(lf)
    assert ast.unparse(lf.body) == "{self.dynamic_fence_id: round(self.spawn_t, 2)}", \
        ast.unparse(lf)
    assert ast.unparse(lf.test) == "self.dynamic_fence_id and self.spawn_t is not None", \
        ast.unparse(lf)
    assert ast.unparse(lf.orelse) == "None", ast.unparse(lf)


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
        {"id": "poly", "type": "polygon_fence", "vertices": sq},
        {"id": "circ", "type": "circle_fence", "center": {"x": 40, "y": 0},
         "radius_m": 5},
        {"id": "dyn", "type": "dynamic_nfz", "vertices": sq},
        {"id": "mov", "type": "dynamic_nfz", "motion": {"vx_mps": 1.0}, "vertices": sq},
        {"id": "alt", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20},
    ]})
    A = _load(DEMO / "aerialvla_pas_demo.py", "aerialvla_pas_demo_unit")
    assert [z.id for z in A.planning_zones(pol)] == ["poly", "circ", "dyn"]


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
