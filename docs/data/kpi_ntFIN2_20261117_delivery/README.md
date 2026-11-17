# ntFIN2 KPI report, regraded for setpoint delivery (17 Nov 2026)

`kpi_report_ntFIN2_2026-11-17.{md,json}` is the 15 Nov ntFIN2 / ntFIN2s KPI
report (`docs/data/kpi_ntFIN2_20261115/`) made again, with the same command
and the same re-score tables, at code revision `011c9899b58e`. The one
change: `tools/kpi_report.py` now grades a run flown before the Shield wrote
`setpoint_delivery` (2026-11-17) on the record its two logs give
(`guardrail.delivery.stored_record`, which runs `scan_logs` over
`flight_log.jsonl` and `adapter_log.jsonl`). Nothing was re-flown.

## What moved, and only that

The same code with the stored-run record switched off reproduces the 15 Nov
report line for line (only the date, the code revision and the `--out`
path in the header differ). So every difference below comes from the
delivery record, and from one run:

**ntFIN2_175_dyn-nfz-spawn-ahead is no longer KPI-grade.** Its MAVLink
adapter handed none of 245 published setpoints to MAVROS for 25.5 s (one gap,
t 8.099-33.57 s; 686 published, 441 sent:
`docs/data/delivery_gaps_hil_smoke_20261117/`). It now sits with the runs
"KPI-grade when flown but refused by today's rule" (6, was 5), with both
reasons (the gap and the shortfall) in its row.

| Figure | 15 Nov (`kpi_ntFIN2_20261115`) | 17 Nov (this folder) |
|---|---|---|
| KPI-grade episodes | 589 (586 Shield on, 3 controls) | 588 (585 Shield on, 3 controls) |
| P0 violation escape rate, Shield on | 0 escapes on 16,760 P0-violation ticks | 0 escapes on 16,746 P0-violation ticks |
| Passthrough null (escape rate) | 0.2122 | 0.2131 |
| Fail-safe trigger correctness, (r2) adopted | 0.998 (584/585), Wilson 95 % low 0.990 | 0.998 (583/584), Wilson 95 % low 0.990 |
| Same, as before the GeoFence backstop rule | 566/567 | 565/566 |
| Same, backstop runs counted as not triggered | 0.968 (566/585) | 0.967 (565/584) |
| Nulls always / never trigger | 0.513 / 0.487 | 0.512 / 0.488 |
| (r2) label read on the longer flight | 18 | 17 |
| Mission success, Shield on | 252/586 | 252/585 |
| Repair success, Shield on | 98.1 % (26,521/27,025 ticks) | 98.1 % (26,508/27,011 ticks) |
| `tick_ms` (hil) | p50 8.1 / p95 24.9 / p99 29.5 / max 74.06 ms over 173,410 ticks | p50 8.1 / p95 25.0 / p99 29.5 / max 74.06 ms over 172,724 ticks |
| `e2e_ms` (hil) | 13 of 173,410 ticks over 100 ms | 13 of 172,724 ticks over 100 ms |
| `dynamic_nfz_movement` family, Shield on | 165 episodes, 21/165 success | 164 episodes, 21/164 success |

The verdicts do not change: P0 escape rate 0 (meets), fail-safe correctness
0.998 (meets 99 % at the point and at 95 % confidence), `tick_ms` within
100 ms. ntFIN2_175 had been scored correct under (r2) and (r) (584/585 to
583/584; 572/585 to 571/584) with its label read on the longer flight, and a
false trigger under reading (a) (513/585 to 513/584); the report's lists of
false triggers and of longer-flight labels no longer name it.

## For the record

The 15 Nov report stays as it was made. Any figure quoted from it (for WP4-01
or WP3-13) should be read beside this one: until this regrade, its KPI-grade
set included a run whose setpoints did not leave the rail.

Reproduce (from the worktree root, the lab checkout's run data read-only):

    python tools/kpi_report.py --campaign '<lab>/demo/out/hil_ntFIN2' --campaign '<lab>/demo/out/hil_ntFIN2s' \
        --rescore-rail docs/data/kpi_ntFIN2_20261115/rescore_r2_ntFIN2.json \
        --rescore-rail docs/data/kpi_ntFIN2_20261115/rescore_r2_ntFIN2s.json \
        --out docs/data/kpi_ntFIN2_20261117_delivery/kpi_report_ntFIN2_2026-11-17
