from demo.mid_term_demo import main


def test_mid_term_gate_passes() -> None:
    # The mid-term acceptance demo must exit 0: P0 escape rate 0, audit hash
    # matches the loaded bundle, and lateral projection fires at least once.
    assert main() == 0
