#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HARNESS_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
[[ -d "$HARNESS_DIR/results" ]] || { echo "error: no results directory" >&2; exit 1; }
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ARCHIVE="$HARNESS_DIR/../rosflight_timing_results_${STAMP}.tar.gz"
tar -C "$HARNESS_DIR" -czf "$ARCHIVE" results
echo "$ARCHIVE"
