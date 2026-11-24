"""tools/bag_hops.py: the cross-node hop pairing, without ROS (WP3-13)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import bag_hops as B                                       # noqa: E402

MS = 1_000_000


def test_each_setpoint_pairs_with_the_action_before_it():
    a = [0, 100 * MS, 200 * MS]
    s = [8 * MS, 112 * MS, 205 * MS]
    m = [9 * MS, 113 * MS, 206 * MS]
    h = B.pair_hops(a, s, m)
    assert [round(x, 3) for x in h["action_to_shield_ms"]] == [8.0, 12.0, 5.0]
    assert [round(x, 3) for x in h["shield_to_mavros_ms"]] == [1.0, 1.0, 1.0]
    assert [round(x, 3) for x in h["action_to_mavros_ms"]] == [9.0, 13.0, 6.0]


def test_a_reordered_log_pairs_with_its_own_setpoint_not_the_one_before():
    # The hil case: the MAVROS setpoint logged 0.1 ms before the Shield
    # setpoint it carried. Without the tolerance it would pair with the Shield
    # setpoint 100 ms earlier and read as a 100 ms hop.
    a = [0, 100 * MS]
    s = [5 * MS, 106 * MS + MS // 10]
    m = [6 * MS, 106 * MS]
    h = B.pair_hops(a, s, m)
    assert round(h["shield_to_mavros_ms"][1], 3) == -0.1
    assert round(h["action_to_mavros_ms"][1], 3) == 6.0
    strict = B.pair_hops(a, s, m, tol_ms=0.0)
    assert round(strict["shield_to_mavros_ms"][1], 3) == 101.0


def test_messages_before_any_cause_are_left_out():
    h = B.pair_hops([50 * MS], [10 * MS, 60 * MS], [11 * MS, 61 * MS])
    assert len(h["action_to_shield_ms"]) == 1
    assert len(h["action_to_mavros_ms"]) == 1
    assert len(h["shield_to_mavros_ms"]) == 2


def test_a_hop_longer_than_the_input_gap_reads_short_the_stated_limit():
    # The 25 Nov review: a setpoint 150 ms after its action, with the next
    # action 100 ms after the first, pairs with the next action and reads
    # 50 ms. The tool states this limit; it cannot fail a 100 ms budget alone.
    a = [0, 100 * MS]
    s = [150 * MS]
    h = B.pair_hops(a, s, [151 * MS])
    assert round(h["action_to_shield_ms"][0], 3) == 50.0
    assert B.min_gap_ms(a) == 100.0
    assert B.min_gap_ms([5 * MS]) is None


def test_summary_counts_the_budget_and_the_reordered():
    s = B.summarise([1.0, 2.0, -0.1, 150.0])
    assert s["n"] == 4 and s["over_budget"] == 1 and s["negative_reordered"] == 1
    assert s["max_ms"] == 150.0 and s["min_ms"] == -0.1
    assert B.summarise([]) == {"n": 0}


def test_reading_a_bag_needs_ros_but_importing_the_tool_does_not():
    src = (ROOT / "tools" / "bag_hops.py").read_text(encoding="utf-8")
    head = src.split("def read_times")[0]
    assert "import rosbag2_py" not in head


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
