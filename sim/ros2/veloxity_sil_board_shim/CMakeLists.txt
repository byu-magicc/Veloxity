cmake_minimum_required(VERSION 3.16)
project(veloxity_sil_board_shim)

find_package(ament_cmake REQUIRED)

# `veloxity_sil_board` is a pure-Rust Hiroz node (`sim/node`, crate
# `veloxity_sil_node`). Nothing is compiled or linked here any more: this
# package exists to hand colcon the binary, the launch files, the config and
# the parameter loader under the names the outer workspace already uses.
set(VELOXITY_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
# Redirect cargo's output into the CMake build tree so a colcon build never
# collides with a developer's `cargo build` in the repository root.
set(VELOXITY_CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(VELOXITY_SIL_BOARD_BIN "${VELOXITY_CARGO_TARGET_DIR}/release/veloxity_sil_board")

add_custom_target(
  veloxity_sil_board ALL
  COMMAND ${CMAKE_COMMAND} -E env
          "CARGO_TARGET_DIR=${VELOXITY_CARGO_TARGET_DIR}"
          cargo build -p veloxity_sil_node --release
  WORKING_DIRECTORY "${VELOXITY_REPO_ROOT}"
  COMMENT "Building the veloxity_sil_board ROS 2 node"
  BYPRODUCTS "${VELOXITY_SIL_BOARD_BIN}"
)

install(PROGRAMS "${VELOXITY_SIL_BOARD_BIN}"
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/verified_param_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
