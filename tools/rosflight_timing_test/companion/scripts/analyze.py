#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def read_column(path: Path, column: str) -> list[int]:
    with path.open(newline="") as source:
        return [int(row[column]) for row in csv.DictReader(source) if row.get(column)]


def describe(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "stddev": statistics.pstdev(values),
        "min": min(values),
        "p50": percentile(values, 50),
        "p90": percentile(values, 90),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "p99_9": percentile(values, 99.9),
        "max": max(values),
    }


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} RUN_DIRECTORY", file=sys.stderr)
        return 2
    run_dir = Path(sys.argv[1]).resolve()
    rtt_ns = read_column(run_dir / "rtt_samples.csv", "rtt_ns")
    publish_ns = read_column(run_dir / "publish_intervals.csv", "publish_monotonic_ns")
    intervals_ns = read_column(run_dir / "publish_intervals.csv", "interval_ns")[1:]
    all_echoes_path = run_dir / "all_echoes.csv"
    classifications: Counter[str] = Counter()
    raw_echoes = len(rtt_ns)
    if all_echoes_path.is_file():
        with all_echoes_path.open(newline="") as source:
            all_echo_rows = list(csv.DictReader(source))
        raw_echoes = len(all_echo_rows)
        classifications.update(row["classification"] for row in all_echo_rows)

    achieved_hz = math.nan
    if len(publish_ns) >= 2 and publish_ns[-1] > publish_ns[0]:
        achieved_hz = (len(publish_ns) - 1) * 1.0e9 / (publish_ns[-1] - publish_ns[0])

    summary = {
        "published_commands": len(publish_ns),
        "matched_echoes": len(rtt_ns),
        "raw_echoes": raw_echoes,
        "duplicate_echoes": classifications["duplicate"],
        "foreign_run_echoes": classifications["foreign_run"],
        "invalid_sequence_echoes": classifications["invalid_sequence"],
        "out_of_order_first_echoes": classifications["first_out_of_order"],
        "missing_echoes": max(0, len(publish_ns) - len(rtt_ns)),
        "response_percent": (100.0 * len(rtt_ns) / len(publish_ns)) if publish_ns else 0.0,
        "achieved_publish_hz": achieved_hz,
        "rtt_ms": describe([value / 1.0e6 for value in rtt_ns]),
        "publish_interval_ms": describe([value / 1.0e6 for value in intervals_ns]),
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    rtt = summary["rtt_ms"]
    lines = [
        f"published commands: {summary['published_commands']}",
        f"raw echoes received: {summary['raw_echoes']}",
        f"unique matched echoes: {summary['matched_echoes']}",
        f"duplicate echoes: {summary['duplicate_echoes']}",
        f"missing echoes: {summary['missing_echoes']}",
        f"foreign-run echoes: {summary['foreign_run_echoes']}",
        f"invalid-sequence echoes: {summary['invalid_sequence_echoes']}",
        f"out-of-order first echoes: {summary['out_of_order_first_echoes']}",
        f"unique response rate: {summary['response_percent']:.3f}%",
        f"achieved publish rate: {summary['achieved_publish_hz']:.3f} Hz",
    ]
    if rtt["count"]:
        lines.extend([
            f"RTT mean: {rtt['mean']:.6f} ms",
            f"RTT stddev: {rtt['stddev']:.6f} ms",
            f"RTT min: {rtt['min']:.6f} ms",
            f"RTT p50: {rtt['p50']:.6f} ms",
            f"RTT p90: {rtt['p90']:.6f} ms",
            f"RTT p95: {rtt['p95']:.6f} ms",
            f"RTT p99: {rtt['p99']:.6f} ms",
            f"RTT p99.9: {rtt['p99_9']:.6f} ms",
            f"RTT max: {rtt['max']:.6f} ms",
        ])
    else:
        lines.append("RTT: no samples; verify both timing patches and the overlay rosflight_io")

    report = "\n".join(lines) + "\n"
    (run_dir / "summary.txt").write_text(report)
    print(report, end="")
    return 0 if rtt_ns else 1


if __name__ == "__main__":
    raise SystemExit(main())
