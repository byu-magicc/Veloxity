#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HARNESS_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PORT="auto"
BAUD="921600"
RATE="400"
DURATION="45"
WARMUP="5"
LABEL="test"

usage() {
  echo "usage: $0 [--port DEVICE|auto] [--baud N] [--rate HZ] [--duration SEC] [--warmup SEC] [--label NAME]" >&2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --baud) BAUD="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --warmup) WARMUP="$2"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; echo "error: unknown argument: $1" >&2; exit 2 ;;
  esac
done

command -v ros2 >/dev/null || { echo "error: source ROS 2 and ROSflight first" >&2; exit 1; }

if [[ "$PORT" == "auto" ]]; then
  shopt -s nullglob
  port_candidates=(
    /dev/serial/by-id/usb-STMicroelectronics_STM32_COMPOSITE_DEVICE_*-if00
    /dev/serial/by-id/usb-Embassy_USB-serial_example_*-if00
  )
  shopt -u nullglob
  if [[ ${#port_candidates[@]} -eq 0 && -e /dev/ttyACM0 ]]; then
    port_candidates=(/dev/ttyACM0)
  fi
  if [[ ${#port_candidates[@]} -ne 1 ]]; then
    echo "error: auto port selection found ${#port_candidates[@]} supported devices; use --port DEVICE" >&2
    printf '  %s\n' "${port_candidates[@]}" >&2
    exit 1
  fi
  PORT="${port_candidates[0]}"
  echo "auto-selected serial device: $PORT"
fi

[[ -e "$PORT" ]] || { echo "error: serial device does not exist: $PORT" >&2; exit 1; }
[[ -r "$PORT" && -w "$PORT" ]] || {
  echo "error: serial device is not readable/writable: $PORT" >&2
  echo "hint: add your user to dialout or temporarily run: sudo chmod a+rw $(readlink -f "$PORT")" >&2
  exit 1
}
[[ "$WARMUP" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "error: --warmup must be a nonnegative number of seconds" >&2
  exit 2
}

OVERLAY_SETUP="$HARNESS_DIR/timing_overlay_ws/install/setup.bash"
DRIVER_SETUP="$HARNESS_DIR/ros2_ws/install/setup.bash"
DRIVER_PACKAGE_SETUP="$HARNESS_DIR/ros2_ws/install/rosflight_timing_driver/share/rosflight_timing_driver/local_setup.bash"
OVERLAY_SOURCE="$HARNESS_DIR/timing_overlay_ws/src/rosflight_ros_pkgs"
[[ -f "$OVERLAY_SETUP" && -f "$DRIVER_SETUP" && -f "$DRIVER_PACKAGE_SETUP" ]] || {
  echo "error: harness is not built; run scripts/build.sh first" >&2
  exit 1
}

set +u
# shellcheck disable=SC1090,SC1091
source "$OVERLAY_SETUP"
source "$DRIVER_SETUP"
source "$DRIVER_PACKAGE_SETUP"
set -u

INTERFACE="$(ros2 interface show rosflight_msgs/msg/Command)"
grep -Eq 'float32\[10\][[:space:]]+u' <<<"$INTERFACE" || {
  echo "error: timing overlay Command interface is not float32[10] u" >&2
  exit 1
}
ros2 interface show rosflight_msgs/msg/TimeDelay >/dev/null

IO_PREFIX="$(ros2 pkg prefix rosflight_io)"
IO_BINARY="$IO_PREFIX/lib/rosflight_io/rosflight_io"
[[ "$IO_PREFIX" == "$HARNESS_DIR/timing_overlay_ws/install/rosflight_io" ]] || {
  echo "error: rosflight_io did not resolve to the isolated timing overlay: $IO_PREFIX" >&2
  exit 1
}
[[ -x "$IO_BINARY" ]] || { echo "error: cannot locate timing rosflight_io executable" >&2; exit 1; }

SAFE_LABEL="${LABEL//[^A-Za-z0-9_.-]/_}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$HARNESS_DIR/results/${STAMP}_${SAFE_LABEL}_${RATE}hz"
mkdir -p "$RUN_DIR"
RTT_CSV="$RUN_DIR/rtt_samples.csv"
ALL_ECHOES_CSV="$RUN_DIR/all_echoes.csv"
PUBLISH_CSV="$RUN_DIR/publish_intervals.csv"

# ROS 2 infers an integer parameter type for values such as "400", but the
# timing driver declares rate and duration as doubles. Preserve decimal and
# scientific notation while making integer-looking command-line values floats.
RATE_PARAM="$RATE"
DURATION_PARAM="$DURATION"
[[ "$RATE_PARAM" == *.* || "$RATE_PARAM" == *[eE]* ]] || RATE_PARAM="${RATE_PARAM}.0"
[[ "$DURATION_PARAM" == *.* || "$DURATION_PARAM" == *[eE]* ]] || DURATION_PARAM="${DURATION_PARAM}.0"

IO_PID=""
cleanup() {
  if [[ -n "$IO_PID" ]] && kill -0 "$IO_PID" 2>/dev/null; then
    kill -INT "$IO_PID" 2>/dev/null || true
    wait "$IO_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

{
  echo "utc_start=$STAMP"
  echo "label=$LABEL"
  echo "port=$PORT"
  echo "baud=$BAUD"
  echo "requested_rate_hz=$RATE"
  echo "duration_s=$DURATION"
  echo "warmup_s=$WARMUP"
  echo "rosflight_io_prefix=$IO_PREFIX"
  echo "rosflight_msgs_prefix=$(ros2 pkg prefix rosflight_msgs)"
  echo "timing_boundary=rosflight_io_command_callback_to_decoded_offboard_echo"
  echo "rosflight_ros_pkgs_base_commit=$(git -C "$OVERLAY_SOURCE" rev-parse HEAD)"
  echo "rosflight_firmware_header_commit=$(git -C "$OVERLAY_SOURCE/rosflight_firmware" rev-parse HEAD)"
  echo "kernel=$(uname -srmo)"
} > "$RUN_DIR/run_metadata.txt"

echo "starting isolated timing-instrumented rosflight_io"
"$IO_BINARY" --ros-args -p port:="$PORT" -p baud_rate:="$BAUD" \
  >"$RUN_DIR/rosflight_io.log" 2>&1 &
IO_PID=$!

sleep 2
kill -0 "$IO_PID" 2>/dev/null || {
  echo "error: rosflight_io exited during startup; see $RUN_DIR/rosflight_io.log" >&2
  exit 1
}

echo "warming up serial link and allowing parameter synchronization for $WARMUP seconds"
sleep "$WARMUP"

set +e
ros2 run rosflight_timing_driver timing_driver --ros-args \
  -p rate_hz:="$RATE_PARAM" -p duration_s:="$DURATION_PARAM" -p output_path:="$PUBLISH_CSV" \
  -p rtt_output_path:="$RTT_CSV" -p all_echoes_output_path:="$ALL_ECHOES_CSV" \
  >"$RUN_DIR/driver.log" 2>&1
DRIVER_STATUS=$?
set -e

cleanup
IO_PID=""
trap - EXIT INT TERM

if [[ $DRIVER_STATUS -ne 0 ]]; then
  echo "error: timing driver failed; see $RUN_DIR/driver.log" >&2
  exit "$DRIVER_STATUS"
fi

python3 "$HARNESS_DIR/scripts/analyze.py" "$RUN_DIR"
echo "results: $RUN_DIR"
