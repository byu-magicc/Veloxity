#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HARNESS_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OVERLAY_SRC="$HARNESS_DIR/timing_overlay_ws/src/rosflight_ros_pkgs"
PATCH="$HARNESS_DIR/patches/rosflight_ros_pkgs_main_exact_timing.patch"
PINNED_COMMIT="d7d8cbdb7c318fa0583deb7002217af2196a0977"
SOURCE_REPO="${1:-https://github.com/rosflight/rosflight_ros_pkgs.git}"

if [[ ! -d "$OVERLAY_SRC/.git" ]]; then
  mkdir -p "$(dirname -- "$OVERLAY_SRC")"
  git clone "$SOURCE_REPO" "$OVERLAY_SRC"
fi

CURRENT_ORIGIN="$(git -C "$OVERLAY_SRC" remote get-url origin)"
if [[ "$CURRENT_ORIGIN" != "$SOURCE_REPO" && -n "${1:-}" ]]; then
  echo "error: existing overlay origin is $CURRENT_ORIGIN, not requested source $SOURCE_REPO" >&2
  exit 1
fi

if ! git -C "$OVERLAY_SRC" diff --quiet || ! git -C "$OVERLAY_SRC" diff --cached --quiet; then
  if git -C "$OVERLAY_SRC" apply --reverse --check "$PATCH" >/dev/null 2>&1; then
    echo "timing overlay is already prepared"
  else
    echo "error: timing overlay has unexpected local changes" >&2
    exit 1
  fi
else
  git -C "$OVERLAY_SRC" checkout --detach "$PINNED_COMMIT"
  git -C "$OVERLAY_SRC" submodule update --init --recursive rosflight_firmware
  git -C "$OVERLAY_SRC" apply --check "$PATCH"
  git -C "$OVERLAY_SRC" apply "$PATCH"
fi

COMMAND_MSG="$OVERLAY_SRC/rosflight_msgs/msg/Command.msg"
grep -Eq 'float32\[10\][[:space:]]+u' "$COMMAND_MSG" || {
  echo "error: prepared overlay is not the current ten-channel Command layout" >&2
  exit 1
}

echo "prepared isolated timing overlay at $OVERLAY_SRC"
echo "base rosflight_ros_pkgs commit: $PINNED_COMMIT"
