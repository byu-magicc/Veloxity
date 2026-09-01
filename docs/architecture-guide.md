# Veloxity Core Architecture

Veloxity separates reusable flight behavior from the code that connects it to hardware,
communications, and simulation. The reusable flight stack lives in `veloxity_core`. Board crates
and the simulator provide the concrete inputs and outputs.

The architecture is loosely inspired by a dynamic entity-component-system framework. However, there are no runtime entity IDs or
component tables. `World` owns the long-lived state, systems operate on explicit context
structures, and bounded event queues carry requests and responses between stages. The design is loosely understood as a composition of four parts:

```text
resources + systems + events + an explicit scheduler
```



## Main Layers

```text
ROS 2 simulator or physical hardware
                │
                ▼
runtime adapter: sim, pixracerpro, or nucleo
                │
       BoardIo / PwmDriver
                │
                ▼
        veloxity_core::World
                │
          CommInterface
                │
                ▼
         veloxity_mavlink
```

The important source locations for getting started are are:

```text
Veloxity/
├── crates/
│   └── veloxity_core/
│       └── src/
│           ├── world.rs
│           ├── world/
│           │   ├── control.rs
│           │   ├── service.rs
│           │   └── telemetry.rs
│           ├── board.rs
│           ├── comm.rs
│           ├── comm/
│           ├── params.rs
│           ├── params/
│           ├── command.rs
│           ├── command/
│           ├── companion.rs
│           ├── sensors.rs
│           ├── sensors/
│           ├── rc.rs
│           ├── rc/
│           ├── control.rs
│           ├── estimator.rs
│           ├── controller.rs
│           ├── mixer.rs
│           ├── pwm.rs
│           ├── pwm/
│           ├── state_machine.rs
│           ├── events.rs
│           ├── ports.rs
│           └── log.rs
├── comms/
│   └── veloxity_mavlink/
├── platforms/
│   └── stm_32/
├── boards/
│   ├── nucleo/
│   └── pixracerpro/
└── sim/
    ├── firmware/
    │   └── src/
    │       ├── lib.rs
    │       └── runtime.rs
    ├── node/
    │   └── src/
    │       └── main.rs
    └── ros2/
        └── veloxity_sil_board_shim/
```

## Vocabulary

| Term | Meaning in Veloxity | Examples |
| --- | --- | --- |
| Contract | A trait that defines behavior supplied by an adapter. | `BoardIo`, `CommInterface`, `Estimator`, `Controller`, `Mixer`, `PwmDriver` |
| Resource | Long-lived state owned by `World` or an adapter. | `Params`, `ProcessedSensors`, `CommandManager`, `StateManager` |
| System | A function that performs one operation over a specific set of resources. | `service_param_events`, `run_rc_command_state`, `run_control_pipeline_if_new_imu` |
| Context | A structure containing the references a system is allowed to use. | `ParamServiceCtx`, `RcCommandStateCtx`, `ControlPipelineCtx` |
| Event | A small message that requests work or records an outcome for a later stage. | `ParamSetRequested`, `CalibrationRequested`, `CommResponse` |
| Adapter | Code outside core that implements contracts for a runtime or protocol. | `pixracerpro`, `nucleo`, `sim`, `veloxity_mavlink` |
| Stage | A named scheduler operation that runs related systems in a defined order. | communication/parameters, sensor processing, RC/state, control, telemetry |

## Dependency Direction

Dependencies point toward core:

```text
veloxity_core
├── owns flight behavior and scheduling
├── defines runtime and protocol contracts
└── does not depend on ROS 2, MAVLink encoding, or a board crate

veloxity_mavlink
├── depends on veloxity_core
├── implements CommInterface
└── converts between MAVLink frames and core messages

sim
├── depends on veloxity_core and veloxity_mavlink
├── implements BoardIo and PwmDriver
└── exposes a C ABI used by the ROS 2 shim

pixracerpro / nucleo
├── depend on veloxity_core and veloxity_mavlink
├── provide board and PWM implementations
└── construct World with STM32 peripherals
```

Runtime differences belong in contract implementations or in the runtime's scheduler loop.

## Core Contracts

`World` is generic over the implementations that connect it to a runtime:

```text
World<B, E, C, M, CI, PD, R>
├── B:  BoardIo
├── E:  Estimator<R>
├── C:  Controller<R, State = E::State> + RcTrimCalibrator
├── M:  Mixer<R, MixerInput = C::ControlOutput>
├── CI: CommInterface<B>
├── PD: PwmDriver<R>
└── R:  FlightFloat
```

### `BoardIo`

`BoardIo` is the core's connection to clocks, sensors, communication bytes, parameter storage,
status outputs, and deferred board actions.

Important method groups include:

```text
sensor input
├── update_sensor_bus
├── imu_pending
├── update_imu_sensor
└── update_service_sensor_bus

communication transport
├── serial_rx_read / serial_rx_frame_read
├── serial_tx_write / serial_tx_write_priority
├── serial_tx_enqueue_downlink
├── serial_rx_pending
├── serial_flush
└── serial_flush_budgeted

board services
├── clock_millis / clock_micros
├── read_params / write_params
├── backup_memory_read / write / clear
├── reboot / reboot_to_bootloader
├── LED and test-pin methods
└── run_deferred_board_actions
```

Most optional methods have safe defaults. A board overrides the methods supported by its runtime.

### `CommInterface`

`CommInterface` translates between protocol-neutral core messages and a wire protocol. The current
implementation is `veloxity_mavlink::MavlinkInterface`.

The interface:

- decodes incoming transport data into `Messages`;
- sends command acknowledgements and parameter responses;
- sends status, sensor, control-output, and other telemetry;
- writes encoded data through `BoardIo`.

### `Estimator`, `Controller`, and `Mixer`

The control components are replaceable contracts:

```text
Estimator::estimate(EstimatorCtx) -> estimator state
Controller::control(state, ControllerCtx) -> control output
Mixer::mix(control output, MixerCtx) -> actuator commands
```

The current vehicle wiring uses the quadrotor implementations exposed through
`veloxity_core::vehicle::quadrotor`.

### `PwmDriver`

`PwmDriver` owns the runtime-specific actuator output:

```text
PwmDriver
├── enable / disable channels
├── configure_output_rates
├── set_duty_cycle
├── send_commands
└── flush
```

After core composes normalized actuator commands and output state, the driver converts those commands
into physical PWM, DShot, or simulated outputs.

## What `World` Owns

`World` is the root object for one firmware instance. Its resources fall into these groups:

| Group | Examples |
| --- | --- |
| Runtime | board, communication manager, PWM driver |
| Configuration | parameters and parameter-list state |
| Events | parameter, command, companion, and communication queues |
| Inputs | raw and processed sensors, IMU accumulator, sensor processors |
| Flight state | RC, command manager, state machine, calibration flags |
| Control | estimator, controller, mixer, latest control-pipeline results |
| Scheduling | control rate, control deadlines, service deadline, last IMU/control times |

Rather than letting systems receive `&mut World`, `World` constructs a context containing the exact resources
needed by each system. Contexts are flexible enough to be used as anything from a grab-bag of entities the function deems important to detailed lists specifying only sub-entities the function intends to modify.

For example, current parameter service receives:

```rust
pub struct ParamServiceCtx<'a> {
    pub params: &'a mut Params,
    pub state: &'a mut ParamListState,
    pub events: &'a mut ParamEventQueues,
    pub comm_events: &'a mut CommEventQueues,
}
```

RC, command, and state processing receive a different context:

```rust
pub struct RcCommandStateCtx<'a> {
    pub now_ms: u32,
    pub fresh_rc: Option<RcPacket>,
    pub rc: &'a mut Rc,
    pub command: &'a mut CommandManager,
    pub state: &'a mut StateManager,
    pub params: &'a mut Params,
    pub param_events: Option<&'a mut ParamEventQueues>,
}
```

These structures make data access and mutation visible at the call site.

## Events And Queues

Events are stored in fixed-capacity `heapless::Deque` queues (allocated at compile time)

```text
ParamEventQueues
├── set_requests
├── read_requests
├── list_requests
├── changes
└── full_refresh

CommandEventQueues
├── calibration_requests
├── offboard_control_requests
├── param_defaults_requests
├── board_command_requests
├── rc_trim_calibration_requests
├── version_requests
├── reset_origin_requests
└── config_info_requests

CompanionEventQueues
├── heartbeats
├── aux_commands
└── external_attitudes

CommEventQueues
└── responses
```

`CommResponse` is the shared output queue for:

- parameter values;
- command acknowledgements;
- version responses;
- status text;
- hard-error reports.

The small port types in `ports.rs` can further restrict queue or parameter access:

- `EventEmitPort` can append events;
- `EventDrainPort` can consume events;
- `EventReadPort` can inspect events without consuming them;
- `ParamsReadPort` and `ParamsWritePort` limit parameter access.

## Standard Scheduler

`World::run_once` is the complete sequential scheduler. Nucleo uses this path.

```text
run_once
├── run_communication_and_parameter_service_stage
├── run_sensor_ingestion_and_health_stage
├── run_rc_command_state_stages
│   ├── run_rc_command_state
│   ├── run_pwm_output_stage
│   └── update_board_leds
├── run_control_and_mixing_stage_if_new_imu
├── run_telemetry_stage
├── board.serial_flush
└── board.run_deferred_board_actions
```

The order is deliberate. Incoming requests are applied before sensor and control work, fresh RC
input reaches command/state processing before the control pipeline, and communication output is
flushed after responses and telemetry have been queued.

## Realtime Scheduler

Pixracer Pro and the simulator use the finer-grained realtime scheduler. This scheduler separates
IMU intake, fixed-rate control, and lower-priority service work.

The decision order is:

```text
realtime_scheduler_step
├── ImuControl    if BoardIo::imu_pending()
├── ControlUpdate if a fixed-rate control deadline is due and IMU samples are accumulated
├── Service       if service is due and enough control slack remains
└── Idle
```

### IMU And Control Timing

`ControlLoopRates` supports two modes:

| Setting | Behavior |
| --- | --- |
| `every_imu_sample()` | Run control for each new processed IMU sample. |
| `fixed_rate_hz(rate)` | Accumulate IMU samples, average them, and run control at the configured rate. |

`run_imu_control_tick`:

1. asks the board for the pending IMU sample;
2. processes that sample;
3. adds it to the control IMU accumulator;
4. updates sensor health and calibration;
5. runs control immediately if the selected control mode is due.

If a fixed-rate deadline becomes due after IMU intake, `run_control_update_tick` consumes the
averaged samples and runs the control pipeline.

### Realtime Service Work

`run_prioritized_service_steps_with_policy` performs service work only while the scheduler still
has control slack. One service step attempts, in order:

```text
service sensor input
communication and parameter input
RC / command / state processing
limited response and log drain
a bounded number of telemetry streams
a bounded serial flush
deferred board actions
```

The control-slack condition is checked between operations. `RealtimeServicePolicy` selects minimum
spacing, the telemetry budget, and whether continuous polling may continue when no activity was
observed.

Pixracer Pro configures a `400 Hz` control loop and continuous polling with two telemetry streams
per service step. Its STM32 peripheral tasks produce sensor data, while the board's main loop owns
`World` and repeatedly executes the scheduler decision.

Nucleo-H753ZI currently uses the standard `run_once` loop. Its target is compile-current, but its
hardware behavior should be revalidated before adopting the Pixracer Pro realtime policy.

## Control Pipeline

The control system runs only when it receives an IMU sample with a timestamp newer than the sample
used by the preceding control update.

```text
run_control_pipeline_if_new_imu
├── validate advancing IMU time
├── calculate dt
├── Estimator::estimate
├── update estimator health
├── Controller::control
├── Mixer::mix
├── update mixer health
├── configure output rates when needed
├── compose PWM outputs
├── write PWM commands
└── store the latest estimator, actuator, PWM, and timing results
```

External attitude is consumed as a one-shot input for the estimator. Auxiliary commands and RC
override state are included when actuator outputs are composed.

## Parameter And Command Flow

Incoming protocol messages are decoded before they reach the parameter or command systems:

```text
transport bytes
└── MavlinkInterface::handle_incoming_messages
    └── CommManager::act_on_messages
        ├── ParamEventQueues
        ├── CommandEventQueues
        └── CompanionEventQueues
```

A parameter-set request follows this path:

```text
ParamSetRequested
└── params::service::service_param_events
    ├── validate parameter name, type, and allowed value
    ├── update Params
    ├── emit ParamChanged
    └── queue CommResponse::ParamValue
        └── params::reactions updates dependent resources
```

Command service handles calibration, parameter defaults, board commands, RC trim, version,
origin-reset, and configuration requests. It queues acknowledgements as `CommResponse` values
instead of writing protocol bytes directly.

## Sensor, RC, And State Flow

```text
BoardIo sensor update
└── SensorBus
    └── sensors::ingestion
        └── ProcessedSensors
            ├── sensor health and calibration
            ├── RcCommandStateCtx for a fresh RC packet
            └── ControlPipelineCtx for a fresh control IMU sample
```

`Rc` interprets channels and switches. `CommandManager` combines RC and offboard control sources.
`StateManager` owns arming, failsafe, calibration, and error state. The PWM synchronization stage
enables or disables outputs based on that state and the configured output-kill behavior.

## Telemetry And Logs

Core systems queue protocol-neutral responses and logs. The communication adapter encodes them
later.

```text
logs
└── log::drain
    └── CommResponse::Statustext

CommEventQueues.responses
└── CommManager::send_comm_responses
    └── CommInterface

run_telemetry_stage
└── CommManager::send_named_telemetry_streams
    └── CommInterface
```

The realtime scheduler can send a bounded number of due streams at a time. Board policy controls
the budget; core still owns the stream definitions, rates, and freshness checks.

## Simulator Architecture

The simulator is an adapter around the same `World` used by hardware targets.

```text
ROS 2 topics
└── veloxity_sil_board sensor subscriptions (Hiroz, same process)
    └── SimFirmware::set_sensors
        └── shared Rust sensor state
            └── Rust firmware worker thread
                ├── realtime_scheduler_step
                ├── estimator / controller / mixer
                └── shared PWM outputs
                    └── sil_board/run
                        ├── SimFirmware::sync_latest_imu
                        ├── SimFirmware::pwm
                        └── publish sim/pwm_output
```

### `sim::runtime::SimFirmware`

`SimFirmware::new()`:

1. creates shared sensor and PWM storage;
2. constructs `SimBoard`, `SimPwmDriver`, and `World`;
3. configures the simulator control loop for `400 Hz`;
4. starts the `veloxity-sim-firmware` worker thread;
5. returns a safe `SimFirmware` handle — construction failures are `anyhow::Error`s that name what
   went wrong, not a null handle.

The handle's surface (`sim/firmware/src/runtime.rs`) is:

| Method | Purpose |
| --- | --- |
| `SimFirmware::new` | Creates one simulator firmware instance and starts its worker thread. |
| `SimFirmware`'s `Drop` | Stops the worker and joins it. |
| `SimFirmware::set_sensors` | Merges a sensor snapshot into shared firmware input. |
| `SimFirmware::sync_latest_imu` | Waits until the worker has processed the latest submitted IMU generation. |
| `SimFirmware::pwm` | Copies the most recent PWM outputs into a caller-provided slice. |
| `SimFirmware::clock_micros` | Returns the firmware instance's monotonic time since creation. |

`veloxity_sil_board` never calls a "run once" function. Firmware scheduling runs continuously on the
Rust worker thread; the ROS service synchronizes with that worker before reading PWM.

### ROS 2 Node

`veloxity_sil_board` (`sim/node`, crate `veloxity_sil_node`) is a plain Rust binary speaking ROS 2
natively through Hiroz, in the same process as `SimFirmware` — no rclcpp, no C ABI, no generated
bindings to keep in sync. It provides:

- the `sil_board/run` service expected by `rosflight_sil_manager`;
- a `sim/pwm_output` publisher;
- subscriptions for IMU, IMU temperature, magnetometer, barometer, GNSS, differential pressure,
  range, battery, and RC.

Two tokio tasks split work the old single-threaded rclcpp executor used to interleave on one thread:
the run service (`async_take_request` → gap bookkeeping → `block_in_place(sync_latest_imu)` →
publish → reply) and the sensor pump (a `select!` over the nine subscriptions). Each primary sensor
callback builds a `SensorSnapshot`, marks the newly received sensor as present, and hands it to
`SimFirmware::set_sensors`. Sensor timestamps come from the firmware instance's monotonic
clock—the same clock used by `SimBoard`—so they are not affected by changes to the computer's wall
clock. The IMU-temperature callback is the exception: it caches the latest temperature for the next
IMU snapshot.

When `sil_board/run` is called, the node waits for the newest IMU to be processed, retrieves the
latest PWM array, and publishes it. Warnings report long service gaps, slow synchronization, or
unexpected output sizes — the same text and thresholds the old C++ node used, since the flight
scripts grep for them.

`VELOXITY_SIM_PARAM_DIR` must identify a writable parameter directory before the firmware instance
is created.

## Adding Or Changing Core Behavior

Use this sequence:

1. Identify the domain that owns the behavior.
2. Decide whether it belongs in a resource method or a multi-resource system.
3. Add an event if another stage requests the work.
4. Define or update a context with only the required references.
5. Implement the system without passing `&mut World`.
6. Wire the system into the correct scheduler stage.
7. Add focused system tests and a `World` ordering test when scheduling changes.

Runtime-specific behavior should normally be implemented through `BoardIo`, `PwmDriver`,
`CommInterface`, or the runtime's outer scheduler loop.

## File Ownership Map

| Path | Responsibility |
| --- | --- |
| `board.rs` | Runtime I/O contract |
| `comm.rs` | Protocol-neutral communication manager and telemetry scheduling |
| `comm/interface.rs` | Communication-adapter contract |
| `comm/messages.rs` | Protocol-neutral communication message types |
| `events.rs` | Bounded event types and queue groups |
| `ports.rs` | Narrow event and parameter access wrappers |
| `params.rs` | Parameter definitions, defaults, values, and storage |
| `params/service.rs` | Parameter read, list, and set processing |
| `params/reactions.rs` | Updates caused by parameter changes |
| `command.rs` | Command resource and control-source selection |
| `command/service.rs` | Command request handling and acknowledgement generation |
| `companion.rs` | Companion heartbeat, auxiliary command, and external-attitude handling |
| `sensors.rs` | Raw and processed sensor resources |
| `sensors/ingestion.rs` | Raw-to-processed sensor conversion |
| `sensors/processors.rs` | Sensor calibration and correction |
| `sensors/health.rs` | Sensor health and calibration progress |
| `rc.rs` | RC channel and switch interpretation |
| `rc/command_state.rs` | RC, command manager, and state-machine handoff |
| `control.rs` | Estimator/controller/mixer/PWM control pipeline |
| `pwm.rs` | PWM driver contract and protocol helpers |
| `pwm/output_sync.rs` | Output enable state, composition, and writes |
| `state_machine.rs` | Arming, failsafe, calibration, and error state |
| `world.rs` | Resource ownership and shared scheduler state |
| `world/control.rs` | Standard and realtime control scheduling |
| `world/service.rs` | Communication, sensors, responses, and realtime service policy |
| `world/telemetry.rs` | Normal and budgeted telemetry scheduling |

## Recommended Reading Paths

### Core Flight Update

```text
1. crates/veloxity_core/src/world.rs
2. crates/veloxity_core/src/world/control.rs
3. crates/veloxity_core/src/sensors/ingestion.rs
4. crates/veloxity_core/src/rc/command_state.rs
5. crates/veloxity_core/src/control.rs
6. crates/veloxity_core/src/pwm/output_sync.rs
```

### Simulator

```text
1. sim/node/src/main.rs
2. sim/firmware/src/runtime.rs
3. crates/veloxity_core/src/world/control.rs
4. crates/veloxity_core/src/world/service.rs
5. crates/veloxity_core/src/control.rs
6. comms/veloxity_mavlink/src/
```

### STM32 Boards

```text
1. boards/nucleo/src/bin/veloxity.rs
2. boards/nucleo/src/board.rs
3. boards/pixracerpro/src/bin/veloxity.rs
4. boards/pixracerpro/src/board.rs
5. boards/pixracerpro/src/pwm.rs
6. platforms/stm_32/src/peripherals/
7. crates/veloxity_core/src/world/
```

See [STM32 boards](boards/stm32.md) for board status, builds, flashing, and hardware validation.
