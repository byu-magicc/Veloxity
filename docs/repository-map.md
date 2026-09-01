# Repository Map

This repository is a Rust workspace plus a ROS 2 shim (to embed veloxity in the SIL ros graph as a firmware node). Source code lives in the workspace
members. Generated build outputs live in `target/` and `workspace/`.

## Top-Level Folders

| Path | Purpose |
| --- | --- |
| `assets/` | Static project assets used by documentation. |
| `boards/` | Board applications. Each board crate chooses pins, peripherals, board I/O, PWM driver, and the concrete `World` type. |
| `comms/` | Communication adapters. `veloxity_mavlink` implements ROSflight MAVLink transport for the core firmware. |
| `crates/` | Firmware libraries. `veloxity_core` is the board-independent flight stack. |
| `docs/` | Current branch documentation. This should describe tested workflows and retained implementation paths. |
| `platforms/` | Reusable platform support shared by board crates. |
| `scripts/` | Repository helper scripts that orchestrate local builds. |
| `sim/` | Host simulator firmware crate and ROS 2 shim package. |
| `third_party/` | Vendored dependency patches needed by the workspace. |
| `tools/` | Standalone tools that are not part of the flight core, including the ESP32C5 ESP-NOW UART bridge. |
| `xtask/` | Repository command wrapper used for common build, test, flash, and cleanup commands. |
| `target/` | Cargo build output. Remove with `cargo xtask clean-generated`. |
| `workspace/` | Local colcon workspace used to build the ROS 2 shim overlay. Remove with `cargo xtask clean-generated`. |
| `rosflight_memory/` | Local runtime parameter/state artifact path from older runs. Remove with `cargo xtask clean-generated`. |

## Vocabulary

| Term | Meaning |
| --- | --- |
| Cargo package/crate | A Rust build unit with its own `Cargo.toml`. In this repo, `pico2w`, `veloxity_core`, `sim`, and `xtask` are packages. |
| `build.rs` | A Cargo build script. Cargo runs it before compiling that package. For `boards/pico2w`, it points the linker at `memory.x`, the RP2350 memory layout file. |
| Feature flag | A compile-time switch enabled with `--features`. Features can include optional dependencies, select alternate hardware behavior, or enable diagnostics. See [Feature flags](features.md). |
| Board crate | A package that adapts real hardware to `veloxity_core`, such as `boards/pico2w` or `boards/pixracerpro`. |
| Virtual board / runtime adapter | A non-physical board boundary. `sim/firmware` adapts ROSflight simulator data into the same core firmware contracts used by hardware boards. |
| `BoardIo` | The core trait a board/runtime implements to provide sensors, outputs, service hooks, and board status to `World`. |
| Sensor bus | Core-owned sensor resources containing the latest raw or processed packets handed in by a board/runtime. |
| Service sensor bus/path | The slower sensor/service side of the realtime scheduler. It handles non-IMU sensors, RC packet drain, command handling, telemetry, and board service work outside the fast IMU/control path. |
| PWM | Actuator output command path. The core owns generic PWM/output concepts, while a board driver may implement PWM-like or DShot-facing physical output. |
| DShot | Pico 2 W configuration includes DShot-oriented ESC pinout, while the core output layer stays protocol-neutral. |
| MAVLink frame | A complete MAVLink packet with header, payload, checksum, and sequence number. |
| MAVLink message | The decoded semantic payload inside a frame, such as heartbeat, RC channels, or ROSflight status. |
| Downlink | Vehicle-to-ground-station traffic emitted by Veloxity. |
| Sequence number | MAVLink packet counter used by receivers/test tools to detect gaps or dropped frames. |
| ODR | Output data rate: the hardware sample production rate of a sensor, especially the IMU. |

## Rust Workspace Members

The workspace is declared in `Cargo.toml`.

| Member | Role |
| --- | --- |
| `crates/veloxity_core` | `no_std` flight stack: params, state machine, sensors, estimator, controller, mixer, PWM, telemetry scheduling, and the `World` scheduler. |
| `comms/veloxity_mavlink` | MAVLink parser and ROSflight MAVLink adapter implementing `CommInterface`. |
| `sim/firmware` | Host-side Rust firmware library, exposing the `sim::runtime::SimFirmware` handle the ROS 2 node drives in-process (no FFI boundary). |
| `sim/node` | `veloxity_sil_node`: the pure-Rust Hiroz ROS 2 node (binary `veloxity_sil_board`) that used to be the C++ `veloxity_sil_board_shim`. |
| `boards/pico2w` | RP2350/Pico 2 W board firmware and hardware probes. |
| `boards/nucleo` | Nucleo-H753ZI board firmware. |
| `boards/pixracerpro` | Pixracer Pro board firmware. |
| `platforms/rp2350` | RP2350 platform crate for re-exporting the Embassy RP HAL as `rp2350_platform::hal` (also holds early shared metadata for multicore and PIO allocation). |
| `platforms/stm_32` | Shared STM32/Embassy peripheral drivers. |
| `xtask` | Local command runner invoked as `cargo xtask ...`. |


Running `cargo build` from the repository root builds only the host-compatible crates listed in the workspace's `default-members`. It doesn ot build the embedded board targets. The default members are:

```text
comms/veloxity_mavlink
crates/veloxity_core
sim/firmware
xtask
```

Additional features, such as embedded board crates are built explicitly with `cargo xtask check-board ...` or
`cargo xtask build-board ...` because they target Cortex-M. For help understanding what fields are available, run `cargo xtask` for a list. Cargo output is humab-readable, and paired with xtask the compilation process is intuitive.

## Source Boundaries

### Core

`crates/veloxity_core` must stay board-independent. It defines traits such as `BoardIo`,
`CommInterface`, `Estimator`, `Controller`, `Mixer`, and `PwmDriver`. Board crates and sim crates
provide concrete implementations.

### Communication

`comms/veloxity_mavlink` is the protocol adapter. Core owns protocol-neutral message structs and
command handling; `veloxity_mavlink` parses and emits MAVLink frames.

### Simulation

The simulator is a pure-Rust ROS 2 node running the Veloxity firmware in the same process — no C++,
no FFI boundary. `sim/firmware` runs the Veloxity flight-control code on the host computer and
exposes it through the safe `sim::runtime::SimFirmware` handle. It provides simulated
implementations of the hardware interfaces used by `veloxity_core`, including sensor input and PWM
output. It also handles MAVLink communication and simulated parameter storage.

`sim/node` (crate `veloxity_sil_node`, binary `veloxity_sil_board`) starts the firmware by calling
`SimFirmware::new()`, which constructs the Veloxity `World` and starts a Rust thread that
continuously runs the firmware scheduler at 400hz. `SimFirmware::new()` returns the handle itself
(or a descriptive `anyhow::Error`) rather than an opaque pointer, since there is no ABI to cross.

The node then calls methods on that handle: `set_sensors` to provide sensor readings,
`sync_latest_imu` to wait until the newest IMU reading has been processed, and `pwm` to read the
resulting PWM outputs. Dropping the handle on node shutdown stops the worker thread and joins it.

`sim/ros2/veloxity_sil_board_shim` is the colcon package that ships the `veloxity_sil_board` binary,
its launch files, and its config — cargo-only, no C++ left to build. It bridges ROS2 sensor topics
into the firmware, and PWM output from the firmware back to ROS2. For instance, ROSflight calls the
`sil_board/run` service, causing the node to wait for the latest IMU reading to be processed. Upon
receiving IMU data, the node then waits for the PWM outputs from the firmware and bridges them back
to `sim/pwm_output`.

### Board Crates

Board crates are responsible for the physical integration of components into the core through traits. They handle:

- pin assignments
- peripheral initialization
- sensor queues
- serial or mailbox transport
- PWM output driver
- World instantiation

Current RP2350/Pico 2 W source is organized as:

| Path | Purpose |
| --- | --- |
| `boards/pico2w/build.rs` | Cargo build script that adds the board crate directory to the linker search path so `memory.x` is found. |
| `boards/pico2w/memory.x` | RP2350 memory layout consumed by the embedded linker. |
| `boards/pico2w/src/bin/veloxity.rs` | Main RP2350 firmware entry point, Embassy task setup, core split, and IMU output-data-rate feature selection. |
| `boards/pico2w/src/lib.rs` | Library module declarations for the Pico board crate. |
| `boards/pico2w/src/config.rs` | Board pinout, core-role, mailbox, and PIO allocation metadata. |
| `boards/pico2w/src/board.rs` | `BoardIo` implementation, sensor queue drains, serial flush budget, and service hooks. |
| `boards/pico2w/src/pwm.rs` | PIO PWM/DShot-facing actuator output driver implementation. |
| `boards/pico2w/src/ism330dhcx.rs` | ISM330DHCX packet queue, counters, and diagnostics. The current setup/read register transactions live in `boards/pico2w/src/bin/veloxity.rs`. |
| `boards/pico2w/src/barometer.rs` | Barometer packet path. |
| `boards/pico2w/src/gy91.rs` | Legacy GY-91/BMP280 support used as a low-rate pressure path. |
| `boards/pico2w/src/gps.rs` | GPS and magnetometer path. |
| `boards/pico2w/src/rc_receiver.rs` | CRSF receiver path feeding service-side RC state. |
| `boards/pico2w/src/comms_core.rs` | Core-to-transport MAVLink mailbox used by UART/ESP-NOW testing. |
| `boards/pico2w/src/pio_uart_dma.rs` | PIO UART helper used by Pico serial paths. |
| `boards/pico2w/src/bin/*_probe.rs` | Hardware bring-up probes for individual buses, sensors, and serial paths. |
| `platforms/rp2350/src/lib.rs` | Re-exports Embassy RP as `rp2350_platform::hal`. Pico code imports the HAL through this crate. |
| `platforms/rp2350/src/multicore.rs` | Shared RP2350 core-role metadata used by Pico config. |
| `platforms/rp2350/src/pio.rs` | Shared PIO allocation metadata used by Pico config. |

Current STM32 work is concentrated in:

| Path | Purpose |
| --- | --- |
| `boards/nucleo/src/bin/veloxity.rs` | Nucleo-H753ZI firmware entry point and `World` construction. |
| `boards/nucleo/src/board.rs` | Nucleo `BoardIo` and board setup. |
| `boards/pixracerpro/src/bin/veloxity.rs` | Pixracer Pro firmware entry point and `World` construction. |
| `boards/pixracerpro/src/board.rs` | Pixracer Pro `BoardIo` and board setup. |
| `boards/pixracerpro/src/pwm.rs` | Pixracer Pro PWM driver. |
| `platforms/stm_32/src/peripherals/` | Shared STM32 sensor, serial, and signal-task drivers. |
| `platforms/stm_32/stm32h7x3_common.rs` | Shared STM32H7 configuration. |

### Platform Crates

Platform crates own reusable chip-family code. Board crates depend on them when a concept applies
to more than one board.

`platforms/rp2350` is intentionally sparse: Embassy RP HAL imports and uses a small amount of RP2350
metadata. If future Pico code stops using it as we move the rp2350 board from experimental to suppored, this dependency may be removed.

## Vendored Dependency Patches

The root `Cargo.toml` contains:

```toml
[patch.crates-io]
ism330dhcx-rs = { path = "third_party/ism330dhcx-rs" }
```

This is a deliberate Cargo patch. Any dependency that asks for `ism330dhcx-rs` is resolved to the
checked-in `third_party/ism330dhcx-rs` directory instead of the version from crates.io.

The reason for the patch is `no_std` firmware compatibility. The published `ism330dhcx-rs 2.0.0`
manifest depends on `half = "2.7.1"`. The `half` crate enables its `std` feature by default, which
is not appropriate for the Pico 2 W firmware target. The vendored manifest changes that dependency
to:

```toml
half = { version = "2.7.1", default-features = false }
```

The driver source is otherwise kept aligned with the published crate. Treat
`third_party/ism330dhcx-rs` as a vendored external driver plus this local manifest patch, not as
Veloxity-owned flight logic.

The current Pico flight path still performs the high-rate IMU setup and sample reads with
board-local register transactions in `boards/pico2w/src/bin/veloxity.rs`; it does not use the
driver's high-level API for the hot IMU timing path. That distinction matters when investigating
non-consistent IMU delay: inspect the board-local data-ready wait, chip-select/SPI transfer,
register burst read, byte conversion, and queue-push stages first. The optional `ism330dhcx-driver`
dependency remains part of the ISM330DHCX feature surface and must stay `no_std`-clean when that
feature is enabled. If upstream changes `ism330dhcx-rs` to disable `half` default features itself,
remove the `[patch.crates-io]` entry and verify the Pico IMU build against the published crate.

## Generated Files

These are disposable:

```text
target/
workspace/build/
workspace/install/
workspace/log/
rosflight_memory/
tools/__pycache__/
tools/espnow_uart_bridge/build*/
tools/espnow_uart_bridge/sdkconfig
tools/espnow_uart_bridge/dependencies.lock
```

Clean them with:

```bash
cargo xtask clean-generated
```

Do not delete source defaults such as `tools/espnow_uart_bridge/sdkconfig.*.defaults`; those are the
checked-in build configurations for the bridge roles.
