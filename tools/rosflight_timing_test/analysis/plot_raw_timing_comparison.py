#!/usr/bin/env python3
"""Plot separate central and outlier comparisons from every raw timing echo."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_raw(run: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    with (run / "all_echoes.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no raw echoes in {run / 'all_echoes.csv'}")

    receipt_ns = np.asarray([int(row["receipt_monotonic_ns"]) for row in rows], dtype=np.int64)
    elapsed_s = (receipt_ns - receipt_ns.min()) / 1.0e9
    rtt_ms = np.asarray([int(row["rtt_ns"]) / 1.0e6 for row in rows])
    duplicate = np.asarray([row["classification"] == "duplicate" for row in rows])
    with (run / "summary.json").open() as stream:
        summary = json.load(stream)
    return elapsed_s, rtt_ms, duplicate, summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("c_run", type=Path)
    parser.add_argument("veloxity_run", type=Path)
    parser.add_argument("--threshold-ms", type=float, default=3.0)
    parser.add_argument("--output-dir", type=Path,
                        help="Save both standalone plots in this directory.")
    parser.add_argument("--central-output", type=Path,
                        help="Save the at-or-below-threshold histogram.")
    parser.add_argument("--output", type=Path,
                        help="Save the above-threshold outlier figure.")
    args = parser.parse_args()
    if args.threshold_ms < 0.0:
        parser.error("--threshold-ms cannot be negative")
    if args.output_dir:
        if args.central_output or args.output:
            parser.error("--output-dir cannot be combined with explicit output paths")
        threshold_name = f"{args.threshold_ms:g}".replace(".", "p")
        args.central_output = args.output_dir / f"raw_rtt_at_or_below_{threshold_name}ms.png"
        args.output = args.output_dir / f"raw_rtt_outliers_above_{threshold_name}ms.png"

    c_elapsed, c_rtt, c_duplicate, c_summary = load_raw(args.c_run)
    v_elapsed, v_rtt, v_duplicate, v_summary = load_raw(args.veloxity_run)

    central_fig, central_axis = plt.subplots(figsize=(13, 5), constrained_layout=True)
    bins = np.linspace(0.0, args.threshold_ms, 61)
    for label, rtt, summary, color in [
        ("C firmware", c_rtt, c_summary, "#1f77b4"),
        ("Veloxity", v_rtt, v_summary, "#ff7f0e"),
    ]:
        central = rtt[(rtt >= 0.0) & (rtt <= args.threshold_ms)]
        central_axis.hist(
            central, bins=bins, alpha=0.65, edgecolor="black", linewidth=0.7,
            color=color,
            label=(f"{label}: median={np.median(rtt):.3f} ms, "
                   f"plotted={len(central):,}, "
                   f"raw={len(rtt):,}/{summary['published_commands']:,} commands"),
        )
    central_axis.set(
        title=f"All raw RTT observations at or below {args.threshold_ms:g} ms",
        xlabel="Round-trip latency (ms)", ylabel="Count",
        xlim=(0.0, args.threshold_ms),
    )
    central_axis.grid(alpha=0.2)
    central_axis.legend()

    categories = [
        ("C non-duplicate", c_elapsed, c_rtt, ~c_duplicate, "#1f77b4", "o"),
        ("C duplicate", c_elapsed, c_rtt, c_duplicate, "#6a3d9a", "x"),
        ("Veloxity non-duplicate", v_elapsed, v_rtt, ~v_duplicate, "#ff7f0e", "o"),
        ("Veloxity duplicate", v_elapsed, v_rtt, v_duplicate, "#d62728", "x"),
    ]

    fig, axis = plt.subplots(figsize=(13, 7.5), constrained_layout=True)
    try:
        fig.canvas.manager.set_window_title("ROSflight raw timing outliers")
    except AttributeError:
        pass

    total_outliers = 0
    for label, elapsed, rtt, category, color, marker in categories:
        selected = category & (rtt > args.threshold_ms)
        count = int(np.count_nonzero(selected))
        total_outliers += count
        print(f"{label}: {count:,} outliers > {args.threshold_ms:g} ms")
        axis.scatter(
            elapsed[selected], rtt[selected], s=20 if marker == "o" else 25,
            alpha=0.75, color=color, marker=marker, linewidths=0.9,
            label=f"{label} (n={count:,})", zorder=3 if marker == "x" else 2,
        )

    axis.axhline(
        args.threshold_ms, color="black", linestyle="--", linewidth=1.0,
        label=f"Outlier threshold ({args.threshold_ms:g} ms)", zorder=1,
    )
    axis.set(
        title=f"All raw RTT observations above {args.threshold_ms:g} ms (n={total_outliers:,})",
        xlabel="Elapsed receive time within each run (s)",
        ylabel="Observed round-trip delay (ms)",
    )
    axis.grid(alpha=0.22)
    axis.legend(loc="best")

    fig.suptitle(
        "No outlier sampling or deduplication\n"
        f"Missing responses (no arrival time): C={c_summary['missing_echoes']:,}, "
        f"Veloxity={v_summary['missing_echoes']:,}",
        fontsize=12,
    )
    if args.central_output:
        args.central_output.parent.mkdir(parents=True, exist_ok=True)
        central_fig.savefig(args.central_output, dpi=180)
        print(args.central_output)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=180)
        print(args.output)
    if not args.central_output and not args.output:
        plt.show()


if __name__ == "__main__":
    main()
