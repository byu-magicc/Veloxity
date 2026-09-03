//! `veloxity_sil_board` — the ROSflight SIL board, in one Rust process.
//!
//! This is the pure-Rust replacement for the rclcpp node that used to live in
//! `sim/ros2/veloxity_sil_board_shim/src/veloxity_sil_board.cpp` and drive the
//! firmware over the `veloxity_sim_*` C ABI. That ABI is gone: the firmware is
//! now the safe [`sim::runtime::SimFirmware`] handle in this same process, and
//! ROS 2 is spoken natively through Hiroz.
//!
//! The observable behaviour is unchanged, and deliberately so — `rosflight_sim`
//! on the other side of the wire cannot tell the difference:
//!
//! * a `std_srvs/Trigger` server on `sil_board/run`, which blocks until the
//!   firmware has consumed the newest IMU sample, publishes the resulting PWM
//!   frame, and only then replies;
//! * nine depth-1 sensor subscriptions feeding [`SensorSnapshot`]s into the
//!   firmware, timestamped with the *firmware* clock rather than the message
//!   header;
//! * `rosflight_msgs/PwmOutput` on `sim/pwm_output`, first published on the
//!   first successful run call;
//! * the same warning texts and thresholds, which the flight scripts grep for.
//!
//! # Task structure
//!
//! Two tokio worker threads carry two never-returning tasks:
//!
//! 1. **the run service** — `async_take_request` → gap bookkeeping →
//!    `block_in_place(sync_latest_imu)` → publish → reply. The blocking hop
//!    matters: `sync_latest_imu` parks on a condvar for up to
//!    `FIRMWARE_SYNC_TIMEOUT` (5 ms), which must not stall the scheduler.
//! 2. **the sensor pump** — a `select!` over the nine subscriptions.
//!
//! Separating them is what keeps the 400 Hz service loop free of sensor
//! deserialization; the C++ node had both on one rclcpp executor thread.
//!
//! The firmware's own realtime scheduler is neither of these — it is the
//! `veloxity-sim-firmware` OS thread that [`SimFirmware`] owns.

use std::process::ExitCode;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::anyhow;
use hiroz::Builder;
use hiroz::parameter::{ParameterDescriptor, ParameterValue};
use hiroz::qos::{QosHistory, QosProfile};
use hiroz::time::ZClock;
use sim::runtime::{
    AirspeedSample, BaroSample, BatterySample, GnssSample, ImuSample, MagSample, NUM_PWM_CHANNELS,
    RangeSample, RcSample, SensorSnapshot, SimFirmware, Vector3,
};
use tracing::{error, info, warn};
use veloxity_ros_msgs::builtin_interfaces::Time;
use veloxity_ros_msgs::geometry_msgs::Vector3 as RosVector3;
use veloxity_ros_msgs::ros::rosflight_msgs::{
    Airspeed, Barometer, BatteryStatus, GNSS, PwmOutput, RCRaw,
};
use veloxity_ros_msgs::sensor_msgs::{Imu, MagneticField, Range, Temperature};
use veloxity_ros_msgs::std_msgs::Header;
use veloxity_ros_msgs::std_srvs::{TriggerResponse, srv::Trigger};

use veloxity_sil_node::shim::context::{self, NodeHandle};

/// The compiled-in node name, overridable with `__node:=`. The launch files
/// pass exactly this, so it also fixes the log prefix the flight scripts grep.
const NODE_NAME: &str = "veloxity_sil_board";

// Parity constants, one for one with the anonymous namespace at the top of
// `veloxity_sil_board.cpp` (lines 27-33).
const DISABLED_PWM_MICROS: u16 = 1000;
const RC_LOW_MICROS: u16 = 1000;
const RC_CENTER_MICROS: u16 = 1500;
/// One 400 Hz frame; the divisor that turns a gap into "missed periods".
const EXPECTED_SIL_RUN_PERIOD: Duration = Duration::from_micros(2500);
const WARN_SIL_RUN_GAP: Duration = Duration::from_millis(10);
const WARN_SIL_RUN_DURATION: Duration = Duration::from_millis(4);

/// The IMU temperature reported before any `sim/sensors/imu/temperature`
/// message arrives (`veloxity_sil_board.cpp:84-86`).
const DEFAULT_IMU_TEMPERATURE_KELVIN: f32 = 298.15;

// The two `std_srvs/Trigger` reply texts (`veloxity_sil_board.cpp:70-71`).
// `rosflight_sim` only reads `success`, but the flight logs carry these.
const PWM_SYNCHRONIZED: &str = "Veloxity SIL PWM synchronized";
const PWM_SYNCHRONIZATION_FAILED: &str = "Veloxity SIL PWM synchronization failed";

// Names as the C++ node spells them: relative, so they expand against the root
// namespace into `/sim/...`, and remappable from the command line.
const RUN_SERVICE: &str = "sil_board/run";
const PWM_TOPIC: &str = "sim/pwm_output";
const IMU_TOPIC: &str = "sim/sensors/imu/data";
const IMU_TEMPERATURE_TOPIC: &str = "sim/sensors/imu/temperature";
const MAG_TOPIC: &str = "sim/sensors/mag";
const BARO_TOPIC: &str = "sim/sensors/baro";
const GNSS_TOPIC: &str = "sim/sensors/gnss";
const DIFF_PRESSURE_TOPIC: &str = "sim/sensors/diff_pressure";
const RANGE_TOPIC: &str = "sim/sensors/range";
const BATTERY_TOPIC: &str = "sim/sensors/battery";
const RC_TOPIC: &str = "sim/RC";

/// `rclcpp::QoS(1)`: keep the newest sample only, reliable and volatile —
/// which are already hiroz's defaults for the other fields.
fn depth_one() -> QosProfile {
    QosProfile {
        history: QosHistory::from_depth(1),
        ..QosProfile::default()
    }
}

/// Build one depth-1 subscription, reporting the topic that could not be
/// created rather than a bare zenoh error.
macro_rules! subscribe {
    ($handle:expr, $ty:ty, $topic:expr) => {
        $handle
            .node
            .create_sub::<$ty>(&$handle.remap_topic($topic))
            .with_qos(depth_one())
            .build()
            .map_err(|error| anyhow!("cannot subscribe to {}: {error}", $topic))?
    };
}

/// Unwrap one received message, or warn and wait for the next one. A decode
/// failure is a wire-level mismatch on one topic, never a reason to take the
/// whole board down.
macro_rules! received {
    ($result:expr, $what:expr) => {
        match $result {
            Ok(message) => message,
            Err(error) => {
                warn!("cannot decode a {} message: {error}", $what);
                continue;
            }
        }
    };
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> ExitCode {
    // `bootstrap` installs the logging subscriber part-way through, so a
    // failure inside it may have nowhere to log; report those on stderr.
    let handle = match context::bootstrap(NODE_NAME) {
        Ok(handle) => handle,
        Err(error) => {
            eprintln!("{NODE_NAME}: {error}");
            return ExitCode::FAILURE;
        }
    };

    match run(handle).await {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            error!("{error:#}");
            ExitCode::FAILURE
        }
    }
}

async fn run(handle: NodeHandle) -> anyhow::Result<()> {
    declare_parity_parameters(&handle);

    // The C++ node logged this failure and then carried on serving requests
    // that could only ever fail; there is nothing to run without the firmware,
    // so exit nonzero and let the launch file notice.
    let firmware = Arc::new(SimFirmware::new().map_err(|error| {
        anyhow!(
            "failed to initialize Veloxity; check VELOXITY_SIM_PARAM_DIR and \
             MAVLink UDP port availability: {error:#}"
        )
    })?);

    // Seed the firmware with a neutral stick position before any RC message
    // arrives (`veloxity_sil_board.cpp:61-62`).
    submit_rc(&firmware, &initialize_default_rc());

    let pwm_publisher = handle
        .node
        .create_pub::<PwmOutput>(&handle.remap_topic(PWM_TOPIC))
        .with_qos(depth_one())
        .build()
        .map_err(|error| anyhow!("cannot advertise {PWM_TOPIC}: {error}"))?;

    // Services keep hiroz's default depth (10), matching
    // `rmw_qos_profile_services_default`, so a burst of run calls queues
    // instead of being dropped.
    let mut run_service = handle
        .node
        .create_service::<Trigger>(&handle.remap_service(RUN_SERVICE))
        .build()
        .map_err(|error| anyhow!("cannot advertise the {RUN_SERVICE} service: {error}"))?;

    let imu_subscription = subscribe!(handle, Imu, IMU_TOPIC);
    let imu_temperature_subscription = subscribe!(handle, Temperature, IMU_TEMPERATURE_TOPIC);
    let mag_subscription = subscribe!(handle, MagneticField, MAG_TOPIC);
    let baro_subscription = subscribe!(handle, Barometer, BARO_TOPIC);
    let gnss_subscription = subscribe!(handle, GNSS, GNSS_TOPIC);
    let diff_pressure_subscription = subscribe!(handle, Airspeed, DIFF_PRESSURE_TOPIC);
    let range_subscription = subscribe!(handle, Range, RANGE_TOPIC);
    let battery_subscription = subscribe!(handle, BatteryStatus, BATTERY_TOPIC);
    let rc_subscription = subscribe!(handle, RCRaw, RC_TOPIC);

    let sensor_firmware = Arc::clone(&firmware);
    let sensor_task = tokio::spawn(async move {
        let firmware = sensor_firmware;
        // The C++ node caches the whole `sensor_msgs/Temperature` plus an
        // `imu_temperature_available_` flag; only the value is ever read, so
        // one `Option` carries both.
        let mut imu_temperature: Option<f64> = None;

        loop {
            tokio::select! {
                message = imu_subscription.async_recv() => {
                    let message = received!(message, "IMU");
                    let snapshot = SensorSnapshot {
                        has_imu: true,
                        imu: ImuSample {
                            timestamp_us: firmware.clock_micros(),
                            angular_velocity: to_vector3(&message.angular_velocity),
                            linear_acceleration: to_vector3(&message.linear_acceleration),
                            temperature_kelvin: imu_temperature
                                .map_or(DEFAULT_IMU_TEMPERATURE_KELVIN, |value| value as f32),
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "IMU");
                }
                message = imu_temperature_subscription.async_recv() => {
                    let message = received!(message, "IMU temperature");
                    imu_temperature = Some(message.temperature);
                }
                message = mag_subscription.async_recv() => {
                    let message = received!(message, "magnetometer");
                    let snapshot = SensorSnapshot {
                        has_mag: true,
                        mag: MagSample {
                            timestamp_us: firmware.clock_micros(),
                            magnetic_field: to_vector3(&message.magnetic_field),
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "magnetometer");
                }
                message = baro_subscription.async_recv() => {
                    let message = received!(message, "barometer");
                    let snapshot = SensorSnapshot {
                        has_baro: true,
                        baro: BaroSample {
                            timestamp_us: firmware.clock_micros(),
                            altitude: message.altitude,
                            pressure: message.pressure,
                            temperature_kelvin: message.temperature,
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "barometer");
                }
                message = gnss_subscription.async_recv() => {
                    let message = received!(message, "GNSS");
                    let snapshot = SensorSnapshot {
                        has_gnss: true,
                        gnss: GnssSample {
                            timestamp_us: firmware.clock_micros(),
                            fix_type: message.fix_type,
                            num_sat: message.num_sat,
                            lat_degrees: message.lat,
                            lon_degrees: message.lon,
                            alt: message.alt,
                            horizontal_accuracy: message.horizontal_accuracy,
                            vertical_accuracy: message.vertical_accuracy,
                            vel_n: message.vel_n,
                            vel_e: message.vel_e,
                            vel_d: message.vel_d,
                            speed_accuracy: message.speed_accuracy,
                            unix_seconds: message.gnss_unix_seconds,
                            unix_nanos: message.gnss_unix_nanos,
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "GNSS");
                }
                message = diff_pressure_subscription.async_recv() => {
                    let message = received!(message, "airspeed");
                    let snapshot = SensorSnapshot {
                        has_airspeed: true,
                        airspeed: AirspeedSample {
                            timestamp_us: firmware.clock_micros(),
                            differential_pressure: message.differential_pressure,
                            temperature_kelvin: message.temperature,
                            indicated_airspeed: message.velocity,
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "airspeed");
                }
                message = range_subscription.async_recv() => {
                    let message = received!(message, "range");
                    let snapshot = SensorSnapshot {
                        has_range: true,
                        range: RangeSample {
                            timestamp_us: firmware.clock_micros(),
                            range: message.range,
                            min_range: message.min_range,
                            max_range: message.max_range,
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "range");
                }
                message = battery_subscription.async_recv() => {
                    let message = received!(message, "battery");
                    let snapshot = SensorSnapshot {
                        has_battery: true,
                        battery: BatterySample {
                            timestamp_us: firmware.clock_micros(),
                            voltage: message.voltage,
                            current: message.current,
                        },
                        ..SensorSnapshot::default()
                    };
                    submit_snapshot(&firmware, &snapshot, "battery");
                }
                message = rc_subscription.async_recv() => {
                    let message = received!(message, "RC");
                    submit_rc(&firmware, &message);
                }
            }
        }
    });

    let clock = handle.node.clock().clone();
    let service_firmware = Arc::clone(&firmware);
    let service_task = tokio::spawn(async move {
        let firmware = service_firmware;
        // Mirrors the C++ `pwm_outputs_` member: seeded to the disarmed 1000 us
        // state and refilled with it before every read, so a short copy leaves
        // channels disabled rather than stale. Nothing is published until the
        // first successful run call.
        let mut pwm_outputs = [DISABLED_PWM_MICROS; NUM_PWM_CHANNELS];
        let mut last_run_start: Option<Instant> = None;

        loop {
            let request = match run_service.async_take_request().await {
                Ok(request) => request,
                Err(error) => {
                    warn!("cannot decode a {RUN_SERVICE} request: {error}");
                    continue;
                }
            };

            let run_start = Instant::now();
            if let Some(previous) = last_run_start {
                let gap = run_start.duration_since(previous);
                if gap > WARN_SIL_RUN_GAP {
                    warn!("{}", gap_warning(gap));
                }
            }
            last_run_start = Some(run_start);

            // `sync_latest_imu` parks on a condvar for up to 5 ms waiting for
            // the firmware worker to consume the newest IMU sample; keep that
            // off the async scheduler.
            let success = tokio::task::block_in_place(|| firmware.sync_latest_imu());

            if success {
                pwm_outputs.fill(DISABLED_PWM_MICROS);
                let copied = firmware.pwm(&mut pwm_outputs);
                if copied != pwm_outputs.len() {
                    warn!("Veloxity returned {copied} PWM channels");
                }
                // Publish before replying: `rosflight_sim` reads the PWM frame
                // as soon as the run call returns.
                let message = PwmOutput {
                    header: Header {
                        stamp: stamp(&clock),
                        frame_id: String::new(),
                    },
                    values: pwm_outputs,
                };
                if let Err(error) = pwm_publisher.publish(&message) {
                    warn!("cannot publish {PWM_TOPIC}: {error}");
                }
                let duration = run_start.elapsed();
                if duration > WARN_SIL_RUN_DURATION {
                    warn!("{}", duration_warning(duration));
                }
            } else {
                warn!("timed out waiting for Veloxity firmware IMU processing");
            }

            let response = TriggerResponse {
                success,
                message: if success {
                    PWM_SYNCHRONIZED
                } else {
                    PWM_SYNCHRONIZATION_FAILED
                }
                .to_owned(),
            };
            if let Err(error) = request.reply(&response).await {
                warn!("cannot reply to a {RUN_SERVICE} request: {error}");
            }
        }
    });

    info!("veloxity_sil_board ready: service=sil_board/run, pwm=sim/pwm_output");

    // Both tasks loop forever, so a resolved join handle means one of them
    // panicked or was cancelled. `handle` stays alive across this await: the
    // node and its zenoh session tear down only once it drops.
    tokio::select! {
        joined = sensor_task => {
            joined.map_err(|error| anyhow!("the sensor task stopped: {error}"))?;
        }
        joined = service_task => {
            joined.map_err(|error| anyhow!("the {RUN_SERVICE} task stopped: {error}"))?;
        }
        result = tokio::signal::ctrl_c() => {
            result.map_err(|error| anyhow!("cannot listen for SIGINT: {error}"))?;
            info!("veloxity_sil_board shutting down");
        }
    }

    Ok(())
}

/// The five parameters the C++ node declares and never reads
/// (`veloxity_sil_board.cpp:47-51`). They are kept so that `ros2 param list`
/// and any launch file setting them behave as they did.
fn declare_parity_parameters(handle: &NodeHandle) {
    let parameters = [
        (
            "simulation_host",
            ParameterValue::String("localhost".into()),
        ),
        ("simulation_port", ParameterValue::Integer(14525)),
        ("ROS_host", ParameterValue::String("localhost".into())),
        ("ROS_port", ParameterValue::Integer(14520)),
        ("serial_delay_ns", ParameterValue::Integer(6_000_000)),
    ];

    for (name, default) in parameters {
        let descriptor = ParameterDescriptor::new(name, default.parameter_type());
        if let Err(error) = handle.node.declare_parameter(name, default, descriptor) {
            warn!("cannot declare the `{name}` parameter: {error}");
        }
    }
}

/// The missed-frame warning (`veloxity_sil_board.cpp:209-213`). Kept as a
/// function so its exact rendering — which the flight logs are grepped for —
/// can be pinned by a test rather than inspected by eye.
fn gap_warning(gap: Duration) -> String {
    let gap_us = gap.as_micros();
    let missed_periods = gap_us / EXPECTED_SIL_RUN_PERIOD.as_micros();
    format!("sil_board/run service gap: {gap_us} us (~{missed_periods} x 400 Hz periods)")
}

/// The slow-call warning (`veloxity_sil_board.cpp:235-238`).
fn duration_warning(duration: Duration) -> String {
    format!(
        "sil_board/run service duration: {} us",
        duration.as_micros()
    )
}

/// `rclcpp::Node::now()`: the node clock, split into a `builtin_interfaces/Time`.
fn stamp(clock: &ZClock) -> Time {
    let nanos = clock.now().as_unix_nanos();
    Time {
        sec: nanos.div_euclid(1_000_000_000) as i32,
        nanosec: nanos.rem_euclid(1_000_000_000) as u32,
    }
}

/// `vector_to_ffi` (`veloxity_sil_board.cpp:35-38`).
fn to_vector3(vector: &RosVector3) -> Vector3 {
    Vector3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}

/// `submit_snapshot` (`veloxity_sil_board.cpp:243-250`). The C++ version
/// returns a bool no caller reads.
fn submit_snapshot(firmware: &SimFirmware, snapshot: &SensorSnapshot, sensor_name: &str) {
    if !firmware.set_sensors(snapshot) {
        warn!("failed to submit {sensor_name} sample to Veloxity");
    }
}

/// `submit_rc` (`veloxity_sil_board.cpp:252-261`). `RCRaw::values` and the
/// firmware's RC slots are both exactly eight channels wide, so the C++ loop
/// is a whole-array copy.
fn submit_rc(firmware: &SimFirmware, message: &RCRaw) {
    let snapshot = SensorSnapshot {
        has_rc: true,
        rc: RcSample {
            timestamp_us: firmware.clock_micros(),
            values: message.values,
        },
        ..SensorSnapshot::default()
    };
    submit_snapshot(firmware, &snapshot, "RC");
}

/// `initialize_default_rc` (`veloxity_sil_board.cpp:263-269`): sticks centred,
/// throttle low, and the two switch channels the C SIL board cached low.
fn initialize_default_rc() -> RCRaw {
    let mut values = [RC_CENTER_MICROS; 8];
    values[2] = RC_LOW_MICROS; // throttle/F
    values[4] = RC_LOW_MICROS; // C SIL cached default before first RC message
    values[5] = RC_LOW_MICROS; // C SIL cached default before first RC message
    RCRaw {
        values,
        ..RCRaw::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The thresholds the flight scripts' log analysis keys on
    /// (`veloxity_sil_board.cpp:31-33`).
    #[test]
    fn run_loop_thresholds_match_the_cpp_shim() {
        assert_eq!(EXPECTED_SIL_RUN_PERIOD, Duration::from_micros(2500));
        assert_eq!(WARN_SIL_RUN_GAP, Duration::from_millis(10));
        assert_eq!(WARN_SIL_RUN_DURATION, Duration::from_millis(4));
        // 2500 us is one 400 Hz period, which is what the gap warning counts.
        assert_eq!(1_000_000 / EXPECTED_SIL_RUN_PERIOD.as_micros(), 400);
    }

    /// A 10 ms gap is four missed 400 Hz frames, not three or five — the
    /// integer division in the warning has to stay truncating. The rendered
    /// line is `RCLCPP_WARN(…, "sil_board/run service gap: %ld us (~%ld x 400
    /// Hz periods)", …)` (`veloxity_sil_board.cpp:209-213`), character for
    /// character.
    #[test]
    fn the_gap_warning_reads_exactly_as_the_cpp_shim_wrote_it() {
        assert_eq!(
            gap_warning(Duration::from_millis(10)),
            "sil_board/run service gap: 10000 us (~4 x 400 Hz periods)"
        );
        assert_eq!(
            gap_warning(Duration::from_micros(12_400)),
            "sil_board/run service gap: 12400 us (~4 x 400 Hz periods)"
        );
        assert_eq!(
            gap_warning(Duration::from_micros(2499)),
            "sil_board/run service gap: 2499 us (~0 x 400 Hz periods)"
        );
    }

    /// `RCLCPP_WARN(…, "sil_board/run service duration: %ld us", …)`
    /// (`veloxity_sil_board.cpp:235-238`).
    #[test]
    fn the_duration_warning_reads_exactly_as_the_cpp_shim_wrote_it() {
        assert_eq!(
            duration_warning(Duration::from_millis(4)),
            "sil_board/run service duration: 4000 us"
        );
    }

    /// The two `std_srvs/Trigger` reply texts
    /// (`veloxity_sil_board.cpp:70-71`).
    #[test]
    fn the_trigger_reply_texts_are_unchanged() {
        assert_eq!(PWM_SYNCHRONIZED, "Veloxity SIL PWM synchronized");
        assert_eq!(
            PWM_SYNCHRONIZATION_FAILED,
            "Veloxity SIL PWM synchronization failed"
        );
    }

    #[test]
    fn the_default_rc_frame_is_centred_with_throttle_low() {
        let rc = initialize_default_rc();

        assert_eq!(
            rc.values,
            [1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500],
            "channels 2, 4 and 5 are low; the rest are centred"
        );
        assert_eq!(RC_CENTER_MICROS, 1500);
        assert_eq!(RC_LOW_MICROS, 1000);
    }

    /// The PWM frame the node publishes is exactly as wide as the firmware
    /// drives; a mismatch would silently truncate outputs.
    #[test]
    fn the_pwm_message_is_as_wide_as_the_firmware() {
        assert_eq!(PwmOutput::default().values.len(), NUM_PWM_CHANNELS);
        assert_eq!(DISABLED_PWM_MICROS, 1000);
    }

    /// Depth 1 is `rclcpp::QoS(1)`: newest sample wins, reliable, volatile.
    #[test]
    fn sensor_qos_keeps_only_the_newest_sample() {
        let qos = depth_one();

        assert_eq!(
            qos.history,
            QosHistory::KeepLast(std::num::NonZeroUsize::new(1).unwrap())
        );
        assert_eq!(qos.reliability, hiroz::qos::QosReliability::Reliable);
        assert_eq!(qos.durability, hiroz::qos::QosDurability::Volatile);
    }

    /// A unix-epoch nanosecond count has to split into the same
    /// `sec`/`nanosec` pair rclcpp would produce.
    #[test]
    fn stamps_split_into_seconds_and_nanoseconds() {
        let clock = ZClock::simulated(hiroz::time::ZTime::from_unix_nanos(
            1_756_742_400_123_456_789,
        ));
        let stamp = stamp(&clock);

        assert_eq!(stamp.sec, 1_756_742_400);
        assert_eq!(stamp.nanosec, 123_456_789);
    }

    /// Names are relative in the C++ node, so the shim has to expand them into
    /// the root namespace exactly as rclcpp does.
    #[test]
    fn relative_names_expand_to_the_root_namespace() {
        let args =
            veloxity_sil_node::shim::rosargs::RosArgs::parse(NODE_NAME, [NODE_NAME]).unwrap();

        assert_eq!(args.remap_service(RUN_SERVICE), "/sil_board/run");
        assert_eq!(args.remap_topic(PWM_TOPIC), "/sim/pwm_output");
        assert_eq!(args.remap_topic(IMU_TOPIC), "/sim/sensors/imu/data");
        assert_eq!(args.remap_topic(RC_TOPIC), "/sim/RC");
    }
}
