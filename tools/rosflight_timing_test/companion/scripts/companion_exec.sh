#!/usr/bin/env bash
set -euo pipefail

ROOT="${RF_TIMING_ROOT:?source setup.zsh first}"
ROS_SETUP="${RF_TIMING_ROS_SETUP:?missing ROS setup path}"
PARAM_DIR="${RF_TIMING_PARAM_DIR:?missing parameter directory}"
PARAM_LOADER="${RF_TIMING_PARAM_LOADER:?missing parameter loader path}"
export ROS_DOMAIN_ID="${RF_TIMING_ROS_DOMAIN_ID:-47}"

[[ -r "$ROS_SETUP" ]] || { echo "error: cannot read ROS setup: $ROS_SETUP" >&2; exit 1; }

set +u
# These are deliberately sourced only in this child process.
source "$ROS_SETUP"
set -u

usage() {
  echo "usage: companion_exec.sh build [ROSFLIGHT_SOURCE] | smoke c|veloxity | run c|veloxity | package" >&2
  exit 2
}

select_port() {
  local firmware="$1"
  local pattern
  case "$firmware" in
    c) pattern='usb-STMicroelectronics_STM32_COMPOSITE_DEVICE_*-if00' ;;
    veloxity) pattern='usb-Embassy_USB-serial_example_*-if00' ;;
    *) usage ;;
  esac

  shopt -s nullglob
  local matches=(/dev/serial/by-id/$pattern)
  shopt -u nullglob
  [[ ${#matches[@]} -eq 1 ]] || {
    echo "error: expected one $firmware device matching /dev/serial/by-id/$pattern; found ${#matches[@]}" >&2
    exit 1
  }
  [[ -r "${matches[0]}" && -w "${matches[0]}" ]] || {
    echo "error: ${matches[0]} is not readable/writable" >&2
    echo "add the user to dialout and install a persistent ttyACM udev rule" >&2
    exit 1
  }
  printf '%s\n' "${matches[0]}"
}

prepare_parameters() {
  local firmware="$1"
  local port="$2"
  local params io_prefix io_binary
  case "$firmware" in
    c) params="$PARAM_DIR/firmware-startup-c.yaml" ;;
    veloxity)
      if [[ -n "${RF_TIMING_VELOXITY_PARAM_FILE:-}" ]]; then
        params="$RF_TIMING_VELOXITY_PARAM_FILE"
      elif [[ -r "$PARAM_DIR/snapshots/quad_x_slim.yaml" ]]; then
        params="$PARAM_DIR/snapshots/quad_x_slim.yaml"
      elif [[ -r "$PARAM_DIR/quad_x_slim.yaml" ]]; then
        params="$PARAM_DIR/quad_x_slim.yaml"
      else
        params="$PARAM_DIR/firmware-startup-veloxity.yaml"
      fi
      ;;
    *) usage ;;
  esac

  [[ -r "$params" ]] || { echo "error: cannot read parameter file: $params" >&2; exit 1; }
  [[ -r "$PARAM_LOADER" ]] || { echo "error: cannot read parameter loader: $PARAM_LOADER" >&2; exit 1; }

  io_prefix="$(ros2 pkg prefix rosflight_io)"
  io_binary="$io_prefix/lib/rosflight_io/rosflight_io"
  [[ "$io_prefix" == "$ROOT/timing_overlay_ws/install/rosflight_io" ]] || {
    echo "error: rosflight_io resolved outside the private timing overlay: $io_prefix" >&2
    exit 1
  }
  [[ -x "$io_binary" ]] || { echo "error: cannot execute private rosflight_io: $io_binary" >&2; exit 1; }

  if ros2 node list 2>/dev/null | grep -qx '/rosflight_io'; then
    echo "error: /rosflight_io is already running; stop it before this test" >&2
    exit 1
  fi

  mkdir -p "$ROOT/results"
  local stamp log io_pid
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  log="$ROOT/results/${stamp}_${firmware}_parameter_preflight.log"
  io_pid=""
  cleanup_io() {
    if [[ -n "$io_pid" ]] && kill -0 "$io_pid" 2>/dev/null; then
      kill -INT "$io_pid" 2>/dev/null || true
      for _ in {1..30}; do
        kill -0 "$io_pid" 2>/dev/null || break
        sleep 0.1
      done
      if kill -0 "$io_pid" 2>/dev/null; then
        echo "rosflight_io did not stop after SIGINT; sending SIGTERM" >&2
        kill -TERM "$io_pid" 2>/dev/null || true
        for _ in {1..20}; do
          kill -0 "$io_pid" 2>/dev/null || break
          sleep 0.1
        done
      fi
      if kill -0 "$io_pid" 2>/dev/null; then
        echo "error: rosflight_io did not stop after SIGTERM; forcing the private process down" >&2
        kill -KILL "$io_pid" 2>/dev/null || true
        wait "$io_pid" 2>/dev/null || true
        return 1
      fi
      wait "$io_pid" 2>/dev/null || true
    fi
  }
  trap cleanup_io EXIT INT TERM

  echo "starting private overlay rosflight_io for verified $firmware parameter loading"
  "$io_binary" --ros-args -p port:="$port" -p baud_rate:=921600 >"$log" 2>&1 &
  io_pid=$!
  sleep 2
  kill -0 "$io_pid" 2>/dev/null || {
    echo "error: private overlay rosflight_io exited; see $log" >&2
    exit 1
  }

  python3 "$PARAM_LOADER" "$params" --check-only
  python3 "$PARAM_LOADER" "$params"
  sleep 1
  cleanup_io
  io_pid=""
  trap - EXIT INT TERM

  if grep -q 'Output from mixer is' "$log"; then
    echo "error: firmware reported an excessive mixer output; refusing timing run" >&2
    echo "see $log" >&2
    exit 1
  fi
  echo "parameter verification passed; preflight log: $log"
}

command_name="${1:-}"
shift || true

case "$command_name" in
  build)
    exec "$ROOT/scripts/build.sh" "$@"
    ;;
  smoke|run)
    [[ $# -eq 1 ]] || usage
    firmware="$1"
    [[ "$firmware" == c || "$firmware" == veloxity ]] || usage

    overlay_setup="$ROOT/timing_overlay_ws/install/setup.bash"
    driver_setup="$ROOT/ros2_ws/install/setup.bash"
    [[ -r "$overlay_setup" && -r "$driver_setup" ]] || {
      echo "error: private timing overlay is not built; run timing_build first" >&2
      exit 1
    }
    set +u
    source "$overlay_setup"
    source "$driver_setup"
    set -u

    if [[ "${RF_TIMING_SAFETY_CONFIRMED:-}" != YES ]]; then
      [[ -t 0 ]] || {
        echo "error: interactive safety confirmation required (or set RF_TIMING_SAFETY_CONFIRMED=YES)" >&2
        exit 1
      }
      read -r -p "Confirm propellers removed, controller disarmed, and motor power disconnected [yes/NO]: " answer
      [[ "$answer" == yes ]] || { echo "aborted" >&2; exit 1; }
    fi

    port="$(select_port "$firmware")"
    prepare_parameters "$firmware" "$port"
    if [[ "$command_name" == smoke ]]; then
      exec "$ROOT/scripts/run_firmware_test.sh" "$firmware" --smoke
    else
      exec "$ROOT/scripts/run_firmware_test.sh" "$firmware"
    fi
    ;;
  package)
    exec "$ROOT/scripts/package_results.sh"
    ;;
  *) usage ;;
esac
