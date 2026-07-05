"""``policy-dsl`` CLI: ingest a DSL file into a bundle, or export the JSON Schema."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from policy_dsl.ingest import ingest_file, write_bundle
from policy_dsl.models import PolicyDoc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="policy-dsl")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_ingest = sub.add_parser("ingest", help="parse a DSL file and emit a signed bundle")
    p_ingest.add_argument("source", help="path to the authored YAML/JSON policy")
    p_ingest.add_argument("-o", "--output", required=True, help="output bundle path (.tar.gz)")
    p_ingest.add_argument("--changelog", default="initial")

    p_schema = sub.add_parser("schema", help="export the DSL JSON Schema")
    p_schema.add_argument("-o", "--output", default="policy_dsl.schema.json")

    args = parser.parse_args(argv)

    if args.cmd == "ingest":
        ir = ingest_file(args.source)
        write_bundle(ir, args.output, changelog=args.changelog)
        print(f"wrote {args.output}  policy_hash={ir.policy_hash}  generation={ir.generation}")
        return 0

    if args.cmd == "schema":
        schema = PolicyDoc.model_json_schema()
        Path(args.output).write_text(json.dumps(schema, indent=2))
        print(f"wrote {args.output}")
        return 0

    return 1


if __name__ == "__main__":
    sys.exit(main())
