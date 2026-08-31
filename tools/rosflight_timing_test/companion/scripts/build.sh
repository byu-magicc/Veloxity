#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HARNESS_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OVERLAY_WS="$HARNESS_DIR/timing_overlay_ws"
DRIVER_WS="$HARNESS_DIR/ros2_ws"

command -v ros2 >/dev/null || { echo "error: source ROS 2 first" >&2; exit 1; }
command -v colcon >/dev/null || { echo "error: colcon is not installed" >&2; exit 1; }
"$SCRIPT_DIR/prepare_timing_overlay.sh" "${1:-}"

cd "$OVERLAY_WS"
colcon build --symlink-install --packages-select rosflight_msgs rosflight_io \
  --allow-overriding rosflight_msgs rosflight_io

set +u
# shellcheck disable=SC1091
source "$OVERLAY_WS/install/setup.bash"
set -u

INTERFACE="$(ros2 interface show rosflight_msgs/msg/Command)"
grep -Eq 'float32\[10\][[:space:]]+u' <<<"$INTERFACE" || {
  echo "error: overlay Command interface is not float32[10] u" >&2
  exit 1
}
TIME_DELAY_INTERFACE="$(ros2 interface show rosflight_msgs/msg/TimeDelay)"
grep -Eq 'uint32[[:space:]]+sequence_id' <<<"$TIME_DELAY_INTERFACE" || {
  echo "error: timing TimeDelay interface has no sequence_id" >&2
  exit 1
}
grep -Eq 'uint32[[:space:]]+run_id' <<<"$TIME_DELAY_INTERFACE" || {
  echo "error: timing TimeDelay interface has no run_id" >&2
  exit 1
}

cd "$DRIVER_WS"
colcon build --symlink-install --packages-select rosflight_timing_driver

echo "build complete"
echo "run scripts/run_fixed_rate.sh from the already-sourced ROS shell"
