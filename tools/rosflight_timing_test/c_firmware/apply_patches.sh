#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 /path/to/rosflight_firmware" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$(cd -- "$1" && pwd)"
HEADER="$FIRMWARE_DIR/comms/mavlink/v1.0/rosflight/mavlink_msg_offboard_control.h"

git -C "$FIRMWARE_DIR" rev-parse --show-toplevel >/dev/null
grep -q '#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN 43' "$HEADER" || {
  echo "error: firmware does not use the current 43-byte ten-channel OFFBOARD_CONTROL" >&2
  exit 1
}
grep -q '#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC 90' "$HEADER" || {
  echo "error: firmware OFFBOARD_CONTROL CRC is not 90" >&2
  exit 1
}

apply_patch_once() {
  local patch="$1"
  local description="$2"
  if git -C "$FIRMWARE_DIR" apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "$description is already applied"
    return
  fi
  git -C "$FIRMWARE_DIR" apply --check "$patch"
  git -C "$FIRMWARE_DIR" apply "$patch"
  echo "applied $description"
}

apply_patch_once \
  "$SCRIPT_DIR/rosflight_firmware_main_offboard_echo.patch" \
  "test-only OFFBOARD_CONTROL echo patch"
apply_patch_once \
  "$SCRIPT_DIR/rosflight_firmware_mixer_pseudoinverse.patch" \
  "rank-deficient mixer pseudoinverse safety fix"

echo "patched C timing firmware at $FIRMWARE_DIR"
echo "build and flash this checkout with the normal board-specific procedure"
