#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat >&2 <<'EOF'
usage: run_firmware_test.sh c|veloxity [--smoke] [run_fixed_rate options]

Defaults: automatic USB device, 921600 baud, 400 Hz, 5-second warm-up,
and a 180-second measurement. --smoke uses a 2-second warm-up and a
5-second measurement.
EOF
}

[[ $# -ge 1 ]] || { usage; exit 2; }
firmware="$1"
shift

case "$firmware" in
  c)
    expected_pattern='usb-STMicroelectronics_STM32_COMPOSITE_DEVICE_*-if00'
    label='c-main-warm'
    ;;
  veloxity)
    expected_pattern='usb-Embassy_USB-serial_example_*-if00'
    label='veloxity-current-main-warm'
    ;;
  *) usage; exit 2 ;;
esac

duration=180
warmup=5
if [[ ${1:-} == "--smoke" ]]; then
  duration=5
  warmup=2
  label="${label}-smoke"
  shift
fi

shopt -s nullglob
matches=(/dev/serial/by-id/$expected_pattern)
shopt -u nullglob
if [[ ${#matches[@]} -ne 1 ]]; then
  echo "error: expected exactly one $firmware USB device matching:" >&2
  echo "  /dev/serial/by-id/$expected_pattern" >&2
  echo "found: ${#matches[@]}" >&2
  exit 1
fi

exec "$SCRIPT_DIR/run_fixed_rate.sh" \
  --port "${matches[0]}" --baud 921600 --rate 400 \
  --warmup "$warmup" --duration "$duration" --label "$label" "$@"
