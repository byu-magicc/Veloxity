# Source this file; do not execute it.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "error: source this file: source setup.bash" >&2
  exit 2
fi

RF_TIMING_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${RF_TIMING_ROS_SETUP:-}" ]]; then
  for candidate in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -r "$candidate" ]]; then RF_TIMING_ROS_SETUP="$candidate"; break; fi
  done
  RF_TIMING_ROS_SETUP="${RF_TIMING_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
fi
if [[ -z "${RF_TIMING_WS_SETUP:-}" ]]; then
  for candidate in \
    "$HOME/rosflight_ws/install/setup.bash" \
    "$HOME/Veloxity/workspace/install/setup.bash" \
    "$HOME/rosflight/rosflight/workspace/install/setup.bash"; do
    if [[ -r "$candidate" ]]; then RF_TIMING_WS_SETUP="$candidate"; break; fi
  done
  RF_TIMING_WS_SETUP="${RF_TIMING_WS_SETUP:-$HOME/rosflight_ws/install/setup.bash}"
fi
RF_TIMING_PARAM_DIR="${RF_TIMING_PARAM_DIR:-$HOME/.config/veloxity/airframes/3dquad/firmware}"
if [[ -z "${RF_TIMING_PARAM_LOADER:-}" ]]; then
  RF_TIMING_PARAM_LOADER="$HOME/.config/veloxity/airframes/3dquad/verified_param_loader.py"
  [[ -r "$RF_TIMING_PARAM_LOADER" ]] || RF_TIMING_PARAM_LOADER="$RF_TIMING_ROOT/params/verified_param_loader.py"
fi
RF_TIMING_ROS_DOMAIN_ID="${RF_TIMING_ROS_DOMAIN_ID:-47}"

_timing_exec() {
  RF_TIMING_ROOT="$RF_TIMING_ROOT" \
  RF_TIMING_ROS_SETUP="$RF_TIMING_ROS_SETUP" \
  RF_TIMING_WS_SETUP="$RF_TIMING_WS_SETUP" \
  RF_TIMING_PARAM_DIR="$RF_TIMING_PARAM_DIR" \
  RF_TIMING_PARAM_LOADER="$RF_TIMING_PARAM_LOADER" \
  RF_TIMING_VELOXITY_PARAM_FILE="${RF_TIMING_VELOXITY_PARAM_FILE:-}" \
  RF_TIMING_ROS_DOMAIN_ID="$RF_TIMING_ROS_DOMAIN_ID" \
    "$RF_TIMING_ROOT/scripts/companion_exec.sh" "$@"
}

timing_build() {
  _timing_exec build "$@"
}

timing_smoke() {
  [[ $# -eq 1 ]] || { echo "usage: timing_smoke c|veloxity" >&2; return 2; }
  _timing_exec smoke "$1"
}

timing_run() {
  [[ $# -eq 1 ]] || { echo "usage: timing_run c|veloxity" >&2; return 2; }
  _timing_exec run "$1"
}

timing_package() {
  _timing_exec package
}

timing_deactivate() {
  unset -f timing_build timing_smoke timing_run timing_package timing_deactivate _timing_exec
  unset RF_TIMING_ROOT RF_TIMING_ROS_SETUP RF_TIMING_WS_SETUP
  unset RF_TIMING_PARAM_DIR RF_TIMING_PARAM_LOADER RF_TIMING_VELOXITY_PARAM_FILE
  unset RF_TIMING_ROS_DOMAIN_ID
  echo "ROSflight timing helpers removed; the parent ROS environment was never modified."
}

echo "ROSflight timing helpers loaded (parent ROS environment unchanged)."
echo "Commands: timing_build, timing_smoke c|veloxity, timing_run c|veloxity, timing_package, timing_deactivate"
