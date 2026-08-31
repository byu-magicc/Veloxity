# Source this file from Zsh; do not execute it.

if [[ "${ZSH_EVAL_CONTEXT:-}" != *:file ]]; then
  print -u2 "error: source this file: source setup.zsh"
  return 2
fi

RF_TIMING_ROOT="${${(%):-%N}:A:h}"
if [[ -z "${RF_TIMING_ROS_SETUP:-}" ]]; then
  for candidate in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -r "$candidate" ]]; then RF_TIMING_ROS_SETUP="$candidate"; break; fi
  done
  RF_TIMING_ROS_SETUP="${RF_TIMING_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
fi
# The bundle intentionally does not source or depend on a normal ROSflight workspace.
RF_TIMING_WS_SETUP=""
RF_TIMING_PARAM_DIR="${RF_TIMING_PARAM_DIR:-$HOME/.config/veloxity/airframes/3dquad/firmware}"
if [[ -z "${RF_TIMING_PARAM_LOADER:-}" ]]; then
  RF_TIMING_PARAM_LOADER="$HOME/.config/veloxity/airframes/3dquad/verified_param_loader.py"
  [[ -r "$RF_TIMING_PARAM_LOADER" ]] || RF_TIMING_PARAM_LOADER="$RF_TIMING_ROOT/params/verified_param_loader.py"
fi
RF_TIMING_ROS_DOMAIN_ID="${RF_TIMING_ROS_DOMAIN_ID:-47}"

function _timing_exec() {
  RF_TIMING_ROOT="$RF_TIMING_ROOT" \
  RF_TIMING_ROS_SETUP="$RF_TIMING_ROS_SETUP" \
  RF_TIMING_WS_SETUP="$RF_TIMING_WS_SETUP" \
  RF_TIMING_PARAM_DIR="$RF_TIMING_PARAM_DIR" \
  RF_TIMING_PARAM_LOADER="$RF_TIMING_PARAM_LOADER" \
  RF_TIMING_VELOXITY_PARAM_FILE="${RF_TIMING_VELOXITY_PARAM_FILE:-}" \
  RF_TIMING_ROS_DOMAIN_ID="$RF_TIMING_ROS_DOMAIN_ID" \
    "$RF_TIMING_ROOT/scripts/companion_exec.sh" "$@"
}

function timing_build() {
  _timing_exec build "$@"
}

function timing_smoke() {
  [[ $# -eq 1 ]] || { print -u2 "usage: timing_smoke c|veloxity"; return 2; }
  _timing_exec smoke "$1"
}

function timing_run() {
  [[ $# -eq 1 ]] || { print -u2 "usage: timing_run c|veloxity"; return 2; }
  _timing_exec run "$1"
}

function timing_package() {
  _timing_exec package
}

function timing_deactivate() {
  unfunction timing_build timing_smoke timing_run timing_package _timing_exec timing_deactivate
  unset RF_TIMING_ROOT RF_TIMING_ROS_SETUP RF_TIMING_WS_SETUP
  unset RF_TIMING_PARAM_DIR RF_TIMING_PARAM_LOADER RF_TIMING_VELOXITY_PARAM_FILE
  unset RF_TIMING_ROS_DOMAIN_ID
  print "ROSflight timing helpers removed; the parent ROS environment was never modified."
}

print "ROSflight timing helpers loaded for Zsh (parent ROS environment unchanged)."
print "Commands: timing_build, timing_smoke c|veloxity, timing_run c|veloxity, timing_package, timing_deactivate"
