#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="${1:-$SCRIPT_DIR/dist/rosflight_timing_companion_current_main.zip}"

command -v zip >/dev/null || {
  echo "error: zip is required to build the companion bundle" >&2
  exit 1
}

mkdir -p "$(dirname -- "$OUTPUT")"
OUTPUT_DIR="$(cd -- "$(dirname -- "$OUTPUT")" && pwd)"
OUTPUT="$OUTPUT_DIR/$(basename -- "$OUTPUT")"
STAGING_DIR="$(mktemp -d)"
trap 'rm -rf -- "$STAGING_DIR"' EXIT

cp -a "$SCRIPT_DIR/companion" "$STAGING_DIR/rosflight_timing_companion"
find "$STAGING_DIR" -type d -name __pycache__ -prune -exec rm -rf -- {} +
find "$STAGING_DIR" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete

rm -f -- "$OUTPUT"
(
  cd "$STAGING_DIR"
  zip -qr "$OUTPUT" rosflight_timing_companion
)

sha256sum "$OUTPUT"
