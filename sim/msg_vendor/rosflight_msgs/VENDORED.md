# Vendored package: rosflight_msgs

This is a verbatim vendored copy of the `rosflight_msgs` interface package.

- Upstream: https://github.com/rosflight/rosflight_ros_pkgs (`rosflight_msgs/`)
- Pinned upstream commit: `0c34caa1ad8b4b96cb30d4c99e945e41a3765b47` (vendored 2026-09-01)
- Source checkout: `/workspaces/rosflightrusthiroz/src/rosflight_ros_pkgs`
  (this is the same repo/commit the workspace's colcon build compiles
  `rosflight_msgs` from via `rosidl`)

The message and service definitions (`msg/`, `srv/`) and `package.xml` are
**byte-identical** to the `src/rosflight_ros_pkgs/rosflight_msgs/` checkout —
verified with `diff -rq`. `CMakeLists.txt` is intentionally not vendored: this
copy feeds `hiroz-codegen` (via `veloxity_ros_msgs`'s `build.rs` and
`HIROZ_MSG_PATH`) rather than `ament_cmake`/`rosidl`, so no CMake build
description is needed here.

**This copy must stay byte-identical to the workspace checkout.** The
original (`src/rosflight_ros_pkgs/rosflight_msgs`) feeds `rosidl`, which
generates the C++/Python ROS 2 message types used by `rosflight_sim` and
friends; this vendored copy feeds `hiroz-codegen`, which generates the Rust
types used by the Hiroz (pure-Rust ROS 2 over Zenoh) side. Wire compatibility
between the two — same DDS/RIHS type hashes, same field layout — depends on
both generators consuming identical `.msg`/`.srv` definitions. Never hand-edit
`msg/` or `srv/` here; if upstream changes, re-copy from
`src/rosflight_ros_pkgs/rosflight_msgs/` in full and re-run the diff check
above. Behavioral changes belong upstream, not in this vendored copy.

## Local divergences (manifests only)

None. `package.xml` already declares `<depend>` on `builtin_interfaces`,
`geometry_msgs`, and `std_msgs`, which matches the `hiroz-msgs` bundled
features (`std_msgs`, `geometry_msgs`, `rcl_interfaces`) enabled by
`veloxity_ros_msgs`. No edits were needed.
