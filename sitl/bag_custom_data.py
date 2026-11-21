"""What the launch knows of an episode's manifest, for the bag's own metadata.

    python sitl/bag_custom_data.py [--topology hil] -- --shield on --seed 7 ...

Prints one KEY=VALUE line per field, the form `ros2 bag record --custom-data`
takes; rosbag2 writes them into the bag's metadata.yaml under `custom_data`
when it closes the bag. The two record commands (sitl/run_ros2_demo.sh with
BAG_MANIFEST=1, deploy/docker/entrypoint.sh with VLAGUARD_BAG_MANIFEST=1)
append them. Both are off by default: a run without the switch records the
bag exactly as before.

WHY (ARCH-22, 2026-11-16). The grant says the six manifest fields are
"emitted by the orchestrator at episode start and stored as the first frame
of every rosbag" (03-simulation/data-flow.md). Until now no bag carried any
of them: manifest.json sat beside the bag, written by the Shield node at the
end of the flight. This puts in the bag what is known BEFORE the recorder
starts, and nothing else:

  manifest.code_revision    guardrail.manifest.code_revision() of this
                            checkout: the function, and the checkout, the
                            node's manifest uses
  manifest.random_seed      --seed of the mission's arguments, read with the
                            Shield node's own parser (its default when absent)
  launch.policy_hash        the hash of the policy the launch names, loaded
                            the way the node loads it (resolve_policy +
                            load_for_flight): generation 0. The manifest's
                            policy_hash is the LAST generation's, which is
                            this one only when nothing is hot-applied.
  launch.topology_declared  the topology the launch was configured for
                            (VLAGUARD_TOPOLOGY), when given. A declaration:
                            the manifest's `topology` is decided from the
                            evidence of the flight, never from a flag.
  launch.ground_sha256      (X-11, 2026-11-18) the sha256 of the mission's
                            --ground file, when it names one: the ground the
                            policy hash above was loaded over. Absent without
                            --ground, so every other launch prints what it did.

A DECLARED GROUND. A policy with an MSL rule passes the flight gate only over
the ground it flies over; until 2026-11-18 this helper loaded the policy
without one and left launch.policy_hash out of every such bag with a note.
It now reads the mission's own --ground / --ground-sha256 (the Shield node's
flags, so the file must hash to the named sha256 when one is given) and
loads the policy over that ground, as the node does. No ground is guessed.

WHAT IS NOT KNOWN AT LAUNCH, and so is not here: vla_model_hash (resolved
from /vla/identity once the pilot is up), sim_speedup (read from the
autopilot at take-off) and the topology the evidence supports. The bag's
metadata therefore holds part of the manifest, not the six fields, and it is
metadata, not the bag's first frame. tools/replay_from_bag.py reads these
back and compares them with manifest.json.

THE FRAMES (2026-11-21). The frame the grant names is now the Shield node's
own: it publishes the manifest latched on /guardrail/manifest at the
episode's start (the fields known then) and at its end (all six), and both
record commands record that topic by default (guardrail/manifest.py,
"THE MANIFEST AS A FRAME OF THE BAG"). This helper and its switch are
unchanged and still off by default: the Jazzy recorder's --custom-data has
not been checked on a flight.

A field that cannot be computed is left out and said on stderr; this never
fails a launch (exit 0 unless the arguments themselves are wrong). rosbag2
splits each pair on "=", so a value holding "=" or white space is refused
rather than recorded cut short.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

CODE_REVISION_KEY = "manifest.code_revision"
RANDOM_SEED_KEY = "manifest.random_seed"
POLICY_HASH_KEY = "launch.policy_hash"
TOPOLOGY_DECLARED_KEY = "launch.topology_declared"
KEYS = (CODE_REVISION_KEY, RANDOM_SEED_KEY, POLICY_HASH_KEY, TOPOLOGY_DECLARED_KEY)
# Only a launch that names a ground carries this (module docstring); kept out
# of KEYS, the fields every launch is asked for.
GROUND_SHA256_KEY = "launch.ground_sha256"
GROUND_KEYS = (GROUND_SHA256_KEY,)

# The manifest fields no launch can know, and when each is known instead.
NOT_KNOWN_AT_LAUNCH = {
    "vla_model_hash": "resolved from /vla/identity once the pilot is up",
    "sim_speedup": "read from the autopilot at take-off",
    "topology": ("decided from the evidence of the flight at its end "
                 "(guardrail.manifest.detect_topology), never from a flag"),
}


def recordable(value: object) -> bool:
    """Can rosbag2 store this value whole? It splits KEY=VALUE on every "="
    and takes the argument list from the shell, so a value with "=", white
    space or nothing in it would be recorded cut short or not at all."""
    text = str(value)
    return bool(text) and "=" not in text and not any(c.isspace() for c in text)


def launch_fields(mission_args: list[str], topology: str | None = None
                  ) -> tuple[dict[str, str], list[str]]:
    """(the fields known at launch, why each missing one is missing)."""
    fields: dict[str, str] = {}
    notes: list[str] = []

    def put(key: str, value: object) -> None:
        if recordable(value):
            fields[key] = str(value)
        else:
            notes.append(f"{key}: {str(value)!r} cannot be stored whole in custom_data")

    try:
        from guardrail.manifest import code_revision
        put(CODE_REVISION_KEY, code_revision())
    except Exception as e:                                       # noqa: BLE001
        notes.append(f"{CODE_REVISION_KEY}: {type(e).__name__}: {e}")
    args = None
    said = io.StringIO()          # argparse's own words when it refuses the arguments
    try:
        import ros2_shield_node as node
        with contextlib.redirect_stderr(said):
            args, _ = node.make_parser().parse_known_args(list(mission_args))
        put(RANDOM_SEED_KEY, int(args.seed))
    except (Exception, SystemExit) as e:                         # noqa: BLE001
        why = (said.getvalue().strip().splitlines() or [f"{type(e).__name__}: {e}"])[-1]
        notes.append(f"{RANDOM_SEED_KEY}: the mission's arguments could not be read "
                     f"({why})")
    if args is not None:
        try:
            from guardrail.bundle import load_for_flight
            from guardrail.terrain import ground_from_args
            bundle, yaml_path, _ = node.resolve_policy(args)
            # The mission's named ground (--ground / --ground-sha256), as
            # the node loads it; (None, None) without --ground.
            ground, gfile = ground_from_args(args, bundle, yaml_path, root=ROOT)
            policy, _ = load_for_flight(bundle, yaml_path,
                                        allow_unverified=args.allow_unverified_bundle,
                                        **({} if ground is None else {"ground": ground}))
            put(POLICY_HASH_KEY, policy.policy_hash)
            if gfile is not None:
                put(GROUND_SHA256_KEY, gfile["sha256"])
        except (Exception, SystemExit) as e:                     # noqa: BLE001
            notes.append(f"{POLICY_HASH_KEY}: {type(e).__name__}: {e}")
    if topology:
        put(TOPOLOGY_DECLARED_KEY, topology)
    return fields, notes


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="KEY=VALUE lines for `ros2 bag record --custom-data`: the manifest "
                    "fields known before the recorder starts.")
    ap.add_argument("--topology", default=None,
                    help="the topology this launch was configured for (declared, "
                         "recorded as such)")
    ap.add_argument("mission_args", nargs=argparse.REMAINDER,
                    help="after --: the arguments the Shield node will be given")
    ns = ap.parse_args(argv)
    mission = ns.mission_args[1:] if ns.mission_args[:1] == ["--"] else ns.mission_args
    fields, notes = launch_fields(mission, ns.topology)
    # One pair a line, LF only: the shell reads them with mapfile, and a
    # carriage return (Python on Windows) would become part of the value.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(newline="\n")
    for key in KEYS + GROUND_KEYS:
        if key in fields:
            print(f"{key}={fields[key]}")
    for note in notes:
        print(f"[bag_custom_data] not recorded: {note}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
