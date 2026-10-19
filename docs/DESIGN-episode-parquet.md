# Episode exporters - Parquet beside the JSONL, read back before it is kept

**Date:** 2026-10-19
**Status:** built in `tools/export_episodes.py`, `pyarrow` optional (pyproject extra
`export`). Installed in the `vla-drone` env on 2026-10-19 (`pip install pyarrow==26.0.0`,
a plain cp311 win_amd64 wheel, no dependencies pulled, numpy 2.4.6 untouched). The other
envs (`vla-real`, `pas`, the WSL venvs) and every CI job still have none, so there the
Parquet tests SKIP and the tool says SKIP. The JSONL is unchanged.

## What the grant asks for

Stress Testing page, Outputs: "Episode exporters - Python CLI (rosbag2 -> JSONL /
Parquet)"; Data record schema: "Two export formats - episode-based (one row per
mission, full trajectory) and step-based (one row per tick) - produced by exporters that
consume the same episode bundle."

## The first writer had never run, and could not have run on everything

`write_parquet` (2026-10-12) was `pa.Table.from_pylist(...)` with dict and list cells
turned into JSON text. pyarrow was installed nowhere, so it had never executed here.
With pyarrow 26.0.0:

| Records | First writer | Now |
|---|---|---|
| `[{a: 1}, {a: 2, b: 3}]` | column `b` dropped from every row, no message (columns come from the first record only) | `b` kept, null where absent |
| a float column holding a NaN | `ArrowInvalid` after the JSONL was written (JSON safety had made it float + the string "NaN") | float64 column, NaN stored natively, read back as the JSONL's "NaN" |
| a bool / int mix | `ArrowInvalid` | a JSON-text column |
| a column that is a dict in one row and a string in another | half the cells JSON, half plain, indistinguishable | a JSON-text column, every cell decoded exactly |
| an int beyond int64 | `OverflowError` | a JSON-text column |
| which columns are JSON text | not recorded | `vlaguard.json_columns` in the file's metadata, with `vlaguard.schema` |

On four real episodes (a headless bundle, two rail runs, a replay bundle) none of the
failure rows happened, which is why tests build the cases instead of waiting for a
flight to hit them.

## What the writer does now

One type per column, decided over all records in first-seen key order: bool, int64,
float64 (ints exact in a double, non-finite floats native), string, or JSON text for
everything else. `read_parquet` decodes exactly the listed JSON columns and gives
non-finite floats the JSONL's strings, so each record read back equals `json.loads` of
the same JSONL line (numbers compared by value). The writer reads its own file back and
compares before it reports success; a mismatch deletes both Parquet files and exits 2.
Without pyarrow, `--format both` prints a SKIP line with the install hint and removes a
Parquet file an earlier export left in the folder (it would disagree with the new JSONL
without a word); `--format parquet` writes nothing and exits 3, as before.

Measured on 2026-10-19 on six stored episodes (the four above plus
`ros2_shield_on_dynamic` and the 19 Oct CityLife NFZ flight, read-only from the main
checkout): 4593 step rows and 6 episode rows, `read_parquet` equal to the JSONL for every
row; 0.70 MB of Parquet against 5.57 MB of JSONL for the steps, 0.93 MB against 2.31 MB
for the episodes. Typed columns: `step`, `tick`, `emitted_violations`, `n_steps` int64;
`t` float64; `risk_level_defaulted`, `audit_record`, `mission_success` bool; the nested
records (`observation`, `policy_state`, `action`, `violations`, `trajectory`, ...) JSON
text.

## CI

No job installs pyarrow, so the three new Parquet tests in
`tests/test_wp4outputs_exporters.py` SKIP on all three and the absent-pyarrow paths (SKIP
line, stale-file removal, exit 3) run there. Adding `pyarrow==26.0.0` to the **3.11
extras job's** `extras` line (cp311 manylinux wheels exist; about 40 MB) would run them on
CI; the 3.10 job could take it too (cp310 wheels exist). The requirements-only job must
stay without it: it is the job that proves the export works with no optional package.
