# Tools

This directory contains standalone development and validation tools. These tools are not part of
the embedded flight core. Some are generic Python scripts; `espnow_uart_bridge/` is a separate
ESP-IDF firmware project.

Run Python tools from the repository root unless a script says otherwise.

## Canonical ROS 2 timing test

The current Raspberry Pi/Pixracer Pro serial RTT timing experiment is the
[ROSflight timing test](rosflight_timing_test/README.md). It contains the Pi companion ZIP and
source, the 400 Hz ROS 2 timing driver, current-main Veloxity firmware instructions, C comparison
patches, and separate raw regular/outlier plotting tools. Use it instead of older ad hoc timing
archives.

## Python Tools

| Tool | Purpose | Typical context |
| --- | --- | --- |
| `mavlink_tester.py` | Receives, decodes, times, and validates Veloxity/ROSflight MAVLink telemetry. It can also inject heartbeat, TIMESYNC, version, and parameter request traffic. | Hardware telemetry validation for Pico 2 W or Pixracer Pro. |
| `udp_latency_test.py` | Measures UDP echo round-trip latency against a Pico 2 W endpoint. | Network/bridge latency experiments. |
| `analyze_scope_timing_csv.py` | Analyzes Saleae digital CSV exports from scope-timing firmware builds and reports pulse widths, periods, rates, and budget misses. | Logic-analyzer timing validation. |
| `plot_scope_timing_csv.py` | Plots Saleae timing distributions from the same CSV style consumed by `analyze_scope_timing_csv.py`. | Visual inspection of timing captures. |
| `rc_override_takeover_profile.py` | Publishes a deterministic RC override takeover/release profile through ROS 2. | Simulator or integration tests involving RC takeover behavior. |
| `replay_rosflight_command_yaml.py` | Replays a `ros2 topic echo` YAML stream as `rosflight_msgs/msg/Command`. | Reproducing command traffic captured from a prior run. |
| `roscopter_resume_mission.py` | Builds and loads a ROScopter resume mission from current vehicle state. | ROScopter autonomy experiments. |
| `wait_roscopter_state_convergence.py` | Waits until ROScopter estimated state agrees with simulator truth within configured tolerances. | Simulator test gating before starting a mission or profile. |

ROS 2 tools require the caller's shell to have ROS 2, ROSflight messages, and any relevant
workspace overlays already sourced. Veloxity tools do not source ROSflight helper scripts for you.

## ESP32C5 ESP-NOW UART Bridge

`tools/espnow_uart_bridge/` is a separate ESP-IDF project for the ESP32C5 UART bridge used during
Pico 2 W telemetry work. It is MAVLink-frame-aware and has role-specific default configs:

| File | Purpose |
| --- | --- |
| `sdkconfig.air.defaults` | Air-side bridge defaults. |
| `sdkconfig.ground.defaults` | Ground-side bridge defaults. |
| `sdkconfig.stats.defaults` | Statistics/diagnostic defaults. |
| `sdkconfig.test-pattern.defaults` | Test-pattern defaults. |
| `main/bridge.c` | Main bridge firmware. |
| `main/mavlink_frame_packer.c` | MAVLink frame packing helper. |
| `tests/test_mavlink_frame_packer.c` | Unit tests for the frame packer. |

See [ESP32C5 ESP-NOW UART bridge](espnow_uart_bridge/README.md) for build and flashing commands.

## Generated Tool Artifacts

`cargo xtask clean-generated` removes generated tool outputs:

```text
tools/__pycache__/
tools/espnow_uart_bridge/build*/
tools/espnow_uart_bridge/sdkconfig
tools/espnow_uart_bridge/dependencies.lock
```

Do not delete checked-in defaults such as `tools/espnow_uart_bridge/sdkconfig.*.defaults`.
