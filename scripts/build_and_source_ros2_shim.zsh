#!/usr/bin/env zsh
emulate -L zsh
set -uo pipefail

if [[ "${ZSH_EVAL_CONTEXT:-}" == *:file ]]; then
  _veloxity_sourced=true
else
  _veloxity_sourced=false
fi

_veloxity_abort() {
  print -u2 "$1"
  if [[ "${_veloxity_sourced}" == "true" ]]; then
    return 1
  fi
  exit 1
}

SCRIPT_DIR="${${(%):-%x}:A:h}"
VELOXITY_ROOT="${VELOXITY_ROOT:-${SCRIPT_DIR:h}}"
VELOXITY_WORKSPACE="${VELOXITY_WORKSPACE:-${VELOXITY_ROOT}/workspace}"
mkdir -p "${VELOXITY_WORKSPACE}"

COLCON_BUILD_BASE="${VELOXITY_COLCON_BUILD_BASE:-${VELOXITY_WORKSPACE}/build}"
COLCON_INSTALL_BASE="${VELOXITY_COLCON_INSTALL_BASE:-${VELOXITY_WORKSPACE}/install}"
COLCON_LOG_BASE="${VELOXITY_COLCON_LOG_BASE:-${VELOXITY_WORKSPACE}/log}"
SHIM_PACKAGE_PATH="${VELOXITY_SHIM_PACKAGE_PATH:-${VELOXITY_ROOT}/sim/ros2/veloxity_sil_board_shim}"

if [[ ! -d "${SHIM_PACKAGE_PATH}" ]]; then
  _veloxity_abort "Veloxity ROS 2 shim package not found: ${SHIM_PACKAGE_PATH}" || return $?
fi

if ! command -v cargo >/dev/null 2>&1; then
  _veloxity_abort "cargo is required to build the veloxity_sil_board node." || return $?
fi

if ! command -v colcon >/dev/null 2>&1; then
  _veloxity_abort "colcon is required to build veloxity_sil_board_shim." || return $?
fi

print -P "%F{cyan}building the veloxity_sil_board node%f"
if ! (
  cd "${VELOXITY_ROOT}"
  cargo xtask build-sil-node
); then
  _veloxity_abort "failed to build the veloxity_sil_board node." || return $?
fi

PACKAGE_BUILD_DIR="${COLCON_BUILD_BASE}/veloxity_sil_board_shim"
PACKAGE_CMAKE_CACHE="${PACKAGE_BUILD_DIR}/CMakeCache.txt"
if [[ -f "${PACKAGE_CMAKE_CACHE}" ]] &&
   grep -qE "Voloxide|voloxide_sil_board_shim" "${PACKAGE_CMAKE_CACHE}"; then
  print -P "%F{yellow}removing stale ROS 2 shim CMake cache from pre-rename build%f"
  rm -rf "${PACKAGE_BUILD_DIR}"
fi

print -P "%F{cyan}building ROS 2 shim package%f"
if ! (
  cd "${VELOXITY_WORKSPACE}"
  colcon --log-base "${COLCON_LOG_BASE}" build \
    --base-paths "${SHIM_PACKAGE_PATH}" \
    --build-base "${COLCON_BUILD_BASE}" \
    --install-base "${COLCON_INSTALL_BASE}" \
    --packages-select veloxity_sil_board_shim
); then
  _veloxity_abort "failed to build veloxity_sil_board_shim." || return $?
fi

if [[ ! -f "${COLCON_INSTALL_BASE}/setup.zsh" ]]; then
  _veloxity_abort "colcon install overlay was not created: ${COLCON_INSTALL_BASE}/setup.zsh" || return $?
fi

_veloxity_restore_nounset=false
if [[ -o nounset ]]; then
  _veloxity_restore_nounset=true
fi
set +u
if ! source "${COLCON_INSTALL_BASE}/setup.zsh"; then
  if [[ "${_veloxity_restore_nounset}" == "true" ]]; then
    set -u
  fi
  _veloxity_abort "failed to source ROS 2 shim overlay: ${COLCON_INSTALL_BASE}/setup.zsh" || return $?
fi
if [[ "${_veloxity_restore_nounset}" == "true" ]]; then
  set -u
fi

print -P "%F{green}Veloxity ROS 2 shim built and sourced.%f"
print "overlay: ${COLCON_INSTALL_BASE}/setup.zsh"

if [[ "${_veloxity_sourced}" != "true" ]]; then
  print -u2 "Note: run this with 'source' to keep the overlay in your current shell:"
  print -u2 "  source ${(%):-%x}"
fi

if [[ "${_veloxity_sourced}" == "true" ]]; then
  return 0
fi
