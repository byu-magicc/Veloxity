use super::*;
use crate::{
    comm::messages::{
        Store,
        enums::{
            OffboardControlIgnore, OffboardControlMode, ParamIdentifier, RosflightAuxCmdType,
            RosflightCmd, RosflightCmdResponse,
        },
        messages::{
            ExternalAttitudeMsg, HeartbeatMsg, OffboardControlMsg, ParamRequestListMsg,
            ParamRequestReadMsg, ParamSetMsg, RosflightAuxCmdMsg, RosflightCmdMsg,
        },
    },
    estimator::AttitudeEstimate,
    packets::{ImuPacket, RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader},
    params::{ParamId, ParamValue},
    pwm::{PwmDriver, PwmError, output_sync::PWM_OUTPUT_CHANNELS},
    state_machine::ErrorFlag,
    test_support::{RecordingCommLink, TestBoard},
    vehicle::quadrotor,
};

#[derive(Default)]
struct SensorStageBoard {
    current_time_us: u64,
    imu: Option<ImuPacket<f64>>,
    rc: Option<RcPacket>,
    update_count: usize,
    serial_flush_count: usize,
    deferred_board_action_count: usize,
    rx_pending: bool,
    service_poll_time_advance_us: u64,
}

impl BoardIo for SensorStageBoard {
    fn update_sensor_bus<R: crate::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        self.update_count += 1;
        if let Some(imu) = self.imu.take() {
            sensors.imu = Some(Ok(imu.cast()));
        }
        if let Some(rc) = self.rc.take() {
            sensors.rc = Some(Ok(rc));
        }
    }

    fn imu_pending(&self) -> bool {
        self.imu.is_some()
    }

    fn update_imu_sensor<R: crate::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        self.update_count += 1;
        if let Some(imu) = self.imu.take() {
            sensors.imu = Some(Ok(imu.cast()));
        }
    }

    fn update_service_sensor_bus<R: crate::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.update_count += 1;
        self.current_time_us = self
            .current_time_us
            .saturating_add(self.service_poll_time_advance_us);
        if let Some(rc) = self.rc.take() {
            sensors.rc = Some(Ok(rc));
        }
    }

    fn serial_rx_read(
        &mut self,
        _buf: &mut [u8],
    ) -> Option<Result<usize, crate::errors::TelemError>> {
        None
    }

    fn serial_tx_write(
        &mut self,
        bytes: &[u8],
    ) -> Option<Result<usize, crate::errors::TelemError>> {
        Some(Ok(bytes.len()))
    }

    fn serial_rx_pending(&self) -> bool {
        self.rx_pending
    }

    fn clock_millis(&self) -> u32 {
        (self.current_time_us / 1000) as u32
    }

    fn clock_micros(&self) -> u64 {
        self.current_time_us
    }

    fn serial_flush(&mut self) {
        self.serial_flush_count += 1;
    }

    fn run_deferred_board_actions(&mut self) {
        self.deferred_board_action_count += 1;
    }
}

#[derive(Default)]
struct SensorStageCommLink {
    baro_count: usize,
    imu_count: usize,
    mag_count: usize,
}

impl CommInterface<SensorStageBoard> for SensorStageCommLink {
    fn send_heartbeat(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: HeartbeatMsg,
    ) -> bool {
        true
    }

    fn send_named_value(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::ParamValueMsg,
    ) {
    }

    fn send_status(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RosflightStatusMsg,
    ) {
    }

    fn send_timesync(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::TimesyncMsg,
    ) -> bool {
        true
    }

    fn send_version(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RosflightVersionMsg,
    ) {
    }

    fn send_output_raw(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RosflightOutputRawMsg,
    ) {
    }

    fn send_attitude(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::AttitudeQuaternionMsg,
    ) {
    }

    fn send_baro(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::SmallBaroMsg,
    ) {
        self.baro_count += 1;
    }

    fn send_diff_pressure(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::DiffPressureMsg,
    ) {
    }

    fn send_imu(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::SmallImuMsg,
    ) {
        self.imu_count += 1;
    }

    fn send_mag(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::SmallMagMsg,
    ) {
        self.mag_count += 1;
    }

    fn send_rc_raw(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RcChannelsMsg,
    ) {
    }

    fn send_range(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::SmallRangeMsg,
    ) {
    }

    fn send_gnss(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RosflightGnssMsg,
    ) {
    }

    fn send_cmd_ack(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RosflightCmdAckMsg,
    ) {
    }

    fn send_rc_channels(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RcChannelsMsg,
    ) {
    }

    fn send_battery_status(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::BatteryStatusMsg,
    ) {
    }

    fn send_statustext(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::StatustextMsg,
    ) {
    }

    fn send_hard_error(
        &mut self,
        _board: &mut SensorStageBoard,
        _system_id: u8,
        _msg: crate::comm::messages::messages::RosflightHardErrorMsg,
    ) {
    }

    fn handle_incoming_messages(
        &mut self,
        _board: &mut SensorStageBoard,
        _msgs: &mut crate::comm::messages::Messages,
    ) {
    }
}

pub struct TestPwm {
    enabled_mask: u32,
    enable_all_count: usize,
    disable_all_count: usize,
    flush_count: usize,
    send_count: usize,
    configure_count: usize,
    last_commands: [f64; 14],
    last_command_len: usize,
    last_rates: [f64; 10],
    last_rate_len: usize,
}

impl TestPwm {
    fn new() -> Self {
        Self {
            enabled_mask: 0,
            enable_all_count: 0,
            disable_all_count: 0,
            flush_count: 0,
            send_count: 0,
            configure_count: 0,
            last_commands: [0.0; 14],
            last_command_len: 0,
            last_rates: [0.0; 10],
            last_rate_len: 0,
        }
    }
}

impl PwmDriver<f64> for TestPwm {
    fn len(&self) -> usize {
        PWM_OUTPUT_CHANNELS
    }

    fn is_enabled(&self) -> bool {
        self.enabled_mask == (1_u32 << PWM_OUTPUT_CHANNELS) - 1
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        self.enabled_mask |= 1_u32 << channel;
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        self.enabled_mask &= !(1_u32 << channel);
        Ok(())
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        self.enabled_mask = (1_u32 << PWM_OUTPUT_CHANNELS) - 1;
        self.enable_all_count += 1;
        Ok(())
    }

    fn disable_all(&mut self) {
        self.enabled_mask = 0;
        self.disable_all_count += 1;
    }

    fn set_duty_cycle(&mut self, _channel: usize, _duty: u16) -> Result<(), PwmError> {
        Ok(())
    }

    fn flush<Board: BoardIo>(&mut self, _board: &mut Board) {
        self.flush_count += 1;
    }

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        self.configure_count += 1;
        self.last_rate_len = rates_hz.len().min(self.last_rates.len());
        self.last_rates[..self.last_rate_len].copy_from_slice(&rates_hz[..self.last_rate_len]);
        Ok(())
    }

    fn send_commands<Board: BoardIo>(
        &mut self,
        _board: &mut Board,
        commands: &[f64],
    ) -> Result<(), PwmError> {
        self.send_count += 1;
        self.last_command_len = commands.len().min(self.last_commands.len());
        self.last_commands[..self.last_command_len]
            .copy_from_slice(&commands[..self.last_command_len]);
        Ok(())
    }
}

type TestWorld = World<
    TestBoard,
    quadrotor::Estimator<f64>,
    quadrotor::Controller<f64>,
    quadrotor::Mixer<f64>,
    RecordingCommLink,
    TestPwm,
    f64,
>;

fn test_world_with_params(params: Params) -> TestWorld {
    let mixer = quadrotor::mixer::<f64>(&params);

    TestWorld::init(
        TestBoard::default(),
        params,
        RecordingCommLink::new(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    )
}

fn test_world() -> TestWorld {
    test_world_with_params(Params::new())
}

#[test]
fn world_configures_battery_monitor_from_persisted_params_at_init() {
    let mut params = Params::new();
    params.set_by_id(
        ParamId::PARAM_BATTERY_VOLTAGE_MULTIPLIER,
        ParamValue::Float(7.675),
    );
    params.set_by_id(
        ParamId::PARAM_BATTERY_CURRENT_MULTIPLIER,
        ParamValue::Float(0.0),
    );

    let world = test_world_with_params(params);

    assert_eq!(world.board.battery_configure_count, 1);
    assert_eq!(world.board.battery_multipliers, Some((7.675, 0.0)));
}

#[test]
fn world_reconfigures_battery_monitor_after_live_param_change() {
    let mut world = test_world();
    param_service::set_param_and_emit_change(
        &mut world.params,
        &mut world.param_events.changes,
        ParamId::PARAM_BATTERY_VOLTAGE_MULTIPLIER,
        ParamValue::Float(8.25),
    );
    param_service::set_param_and_emit_change(
        &mut world.params,
        &mut world.param_events.changes,
        ParamId::PARAM_BATTERY_CURRENT_MULTIPLIER,
        ParamValue::Float(51.0),
    );

    world.apply_param_reactions();

    assert_eq!(world.board.battery_configure_count, 2);
    assert_eq!(world.board.battery_multipliers, Some((8.25, 51.0)));
}

fn seed_healthy_rc(world: &mut TestWorld) {
    let mut channels = [0.5; RC_PACKET_CHANNELS];
    channels[2] = 0.0;
    world.rc.receive(&RcPacket {
        header: RosflightPacketHeader {
            timestamp: world.board.current_time_us,
            status: 0,
        },
        n_chan: 8,
        chan: channels,
        lol: false,
    });
    world
        .rc
        .run(world.board.clock_millis(), &world.params, &mut world.state);
}

#[test]
fn world_init_reconciles_reflected_mixer_params_from_persisted_mixer_choice() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(10));
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0, ParamValue::Int(2));
    params.set_by_id(
        ParamId::PARAM_PRIMARY_MIXER_3_0,
        ParamValue::Float(-25303.715),
    );

    let world = test_world_with_params(params);

    assert_eq!(
        world
            .params
            .get_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0),
        ParamValue::Int(1)
    );
    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_3_0),
        ParamValue::Float(1.0)
    );
}

#[test]
fn world_init_blocks_arming_when_persisted_imu_biases_are_all_zero() {
    let mut world = test_world();

    assert!(
        world
            .state
            .get_errors()
            .contains(ErrorFlag::UNCALIBRATED_IMU)
    );

    world.state.update_arming_safety(true, true);
    world.state.update(Event::REQUEST_ARM, &world.params);

    assert!(!world.state.is_armed());
    assert!(world.state.is_in_error_state());
}

#[test]
fn world_init_accepts_persisted_imu_calibration() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_ACC_X_BIAS, ParamValue::Float(0.01));

    let world = test_world_with_params(params);

    assert!(
        !world
            .state
            .get_errors()
            .contains(ErrorFlag::UNCALIBRATED_IMU)
    );
}

fn armed_test_world_with_params(params: Params) -> TestWorld {
    let mut state = StateManager::new();
    state.update(Event::INITIALIZED, &params);
    state.update_arming_safety(true, true);
    state.update(Event::REQUEST_ARM, &params);
    let mixer = quadrotor::mixer(&params);

    TestWorld::init(
        TestBoard::default(),
        params,
        RecordingCommLink::new(),
        state,
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    )
}

#[test]
fn world_scheduler_runs_deferred_param_pipeline() {
    let mut world = test_world();

    crate::comm::messages::Store::store(
        &mut world.comm.msgs,
        ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        },
    );

    assert!(world.run_once());

    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
        ParamValue::Int(42)
    );
    assert_eq!(world.comm.sysid, 42);
}

#[test]
fn world_scheduler_streams_param_request_list_through_param_system() {
    let mut world = test_world();

    world.comm.msgs.param_request_list = Some(ParamRequestListMsg {
        target_system: 1,
        target_component: 1,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.param_events.list_requests.is_empty());
    assert!(world.param_list_state.is_active());
    assert_eq!(world.comm.comm_link().sent_param_value_count, 1);
    let first = world.comm.comm_link().sent_param_values[0].unwrap();
    assert_eq!(first.param_index, ParamId::PARAM_BAUD_RATE as u16);
    assert_eq!(first.param_value, ParamValue::Int(921600));

    world.run_comm_param_sensor_stages();

    assert_eq!(world.comm.comm_link().sent_param_value_count, 2);
    let second = world.comm.comm_link().sent_param_values[1].unwrap();
    assert_eq!(second.param_index, ParamId::PARAM_SERIAL_DEVICE as u16);
}

#[test]
fn world_scheduler_answers_param_request_read_through_param_system() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
    let mut world = test_world_with_params(params);

    Store::store(
        &mut world.comm.msgs,
        ParamRequestReadMsg {
            target_system: 1,
            target_component: 1,
            param_identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        },
    );

    world.run_comm_param_sensor_stages();

    assert!(world.param_events.read_requests.is_empty());
    assert_eq!(world.comm.comm_link().sent_param_value_count, 1);
    let response = world.comm.comm_link().sent_param_values[0].unwrap();
    assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
    assert_eq!(response.param_value, ParamValue::Int(42));
}

#[test]
fn world_sensor_stage_ingests_board_sensor_bus_without_hlist_fixture() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
    let board = SensorStageBoard {
        current_time_us: 25_000,
        imu: Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 25_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.1, 0.2, 0.3],
            temperature: 25.0,
            seq: 7,
        }),
        rc: Some(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 24_000,
                status: 0,
            },
            n_chan: 1,
            chan: [0.5; RC_PACKET_CHANNELS],
            lol: false,
        }),
        update_count: 0,
        serial_flush_count: 0,
        deferred_board_action_count: 0,
        rx_pending: false,
        service_poll_time_advance_us: 0,
    };
    let state = StateManager::new();
    let mixer = quadrotor::mixer::<f64>(&params);

    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        board,
        params,
        SensorStageCommLink::default(),
        state,
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );

    world.run_comm_param_sensor_stages();

    assert_eq!(world.board.update_count, 1);
    assert!(world.raw_sensors.imu.is_none());
    assert!(world.raw_sensors.rc.is_none());
    assert_eq!(
        world.processed_sensors.imu.unwrap().header.timestamp,
        25_000
    );
    assert_eq!(world.processed_sensors.rc.unwrap().chan[0], 0.5);
    assert!(
        !world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
    );

    let fresh_rc = world.processed_sensors.rc;
    world.run_rc_command_state_stages(fresh_rc);

    assert!(
        !world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::RC_LOST)
    );
}

#[test]
fn world_fast_tick_runs_sensor_rc_control_without_service_output() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 10_000,
            imu: Some(ImuPacket {
                header: RosflightPacketHeader {
                    timestamp: 10_000,
                    status: 0,
                },
                accel: [0.0, 0.0, -9.80665],
                gyro: [0.0, 0.0, 0.0],
                temperature: 25.0,
                seq: 1,
            }),
            rc: Some(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 10_000,
                    status: 0,
                },
                n_chan: 1,
                chan: [0.5; RC_PACKET_CHANNELS],
                lol: false,
            }),
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );

    assert!(!world.run_imu_control_tick());
    assert_eq!(world.board.update_count, 1);
    assert_eq!(world.board.serial_flush_count, 0);
    assert_eq!(world.board.deferred_board_action_count, 0);

    world.board.current_time_us = 12_500;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 12_500,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 2,
    });

    assert!(world.run_imu_control_tick());
    assert_eq!(world.board.update_count, 2);
    assert_eq!(world.board.serial_flush_count, 0);
    assert_eq!(world.board.deferred_board_action_count, 0);
}

#[test]
fn prioritized_service_runs_service_sensors_comm_telemetry_and_board_service() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 1_100_000,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.processed_sensors.baro = Some(crate::packets::BaroPacket {
        altitude: 42.0,
        pressure: 90_000.0,
        temperature: 21.0,
        ..Default::default()
    });

    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 1));

    assert_eq!(world.board.update_count, 1);
    assert_eq!(world.board.serial_flush_count, 1);
    assert_eq!(world.board.deferred_board_action_count, 1);
    assert_eq!(world.comm.comm_link().baro_count, 1);
}

#[test]
fn prioritized_service_sends_telemetry_before_variable_service_work_closes_slack() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 10_000,
            service_poll_time_advance_us: 300,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
    world.last_control_update_us = 10_000;
    world.control_imu_accumulator.push(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 9_999,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0; 3],
        temperature: 294.15,
        seq: 1,
    });
    world.processed_sensors.baro = Some(crate::packets::BaroPacket {
        header: RosflightPacketHeader {
            timestamp: 9_999,
            status: 0,
        },
        altitude: 42.0,
        pressure: 90_000.0,
        temperature: 294.15,
    });

    let report =
        world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 1));

    assert!(report.telemetry_due);
    assert_eq!(world.comm.comm_link().baro_count, 1);
    assert_eq!(world.board.update_count, 1);
    assert!(!world.realtime_service_has_control_slack(world.board.current_time_us));
}

#[test]
fn slack_driven_service_sends_all_due_streams_without_a_stream_count_limit() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 1_100_000,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.processed_sensors.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1_099_000,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0; 3],
        temperature: 294.15,
        seq: 1,
    });
    world.processed_sensors.baro = Some(crate::packets::BaroPacket {
        header: RosflightPacketHeader {
            timestamp: 1_098_000,
            status: 0,
        },
        altitude: 42.0,
        pressure: 90_000.0,
        temperature: 294.15,
    });
    world.processed_sensors.mag = Some(crate::packets::MagPacket {
        header: RosflightPacketHeader {
            timestamp: 1_097_000,
            status: 0,
        },
        flux: [1.0, 2.0, 3.0],
        temperature: 294.15,
    });

    let report =
        world.run_prioritized_service_step(RealtimeServicePolicy::continuous_slack_driven());

    assert!(report.telemetry_due);
    assert_eq!(world.comm.comm_link().imu_count, 1);
    assert_eq!(world.comm.comm_link().baro_count, 1);
    assert_eq!(world.comm.comm_link().mag_count, 1);
}

#[test]
fn service_sensor_stage_preserves_previous_imu_for_health_when_service_poll_omits_imu() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: IMU_TIMEOUT_US + 1,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.processed_sensors.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::continuous(0));

    assert!(world.processed_sensors.imu.is_some());
    assert!(
        !world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
    );
}

#[test]
fn realtime_scheduler_prefers_imu_and_idles_until_service_deadline() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 10_000,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );

    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::Service
    );
    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1_000, 0));
    assert_eq!(world.realtime_scheduler_step(), RealtimeSchedulerStep::Idle);

    world.control_pipeline.set_last_imu_time(10_000);
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 10_500,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::ImuControl
    );

    world.board.current_time_us = 11_001;
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::ImuControl
    );
    let _ = world.run_imu_control_tick();
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::Service
    );
}

#[test]
fn fixed_control_rate_ingests_imu_without_running_every_sample() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard::default(),
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));

    world.board.current_time_us = 500;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 500,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });
    assert!(!world.run_imu_control_tick());

    world.board.current_time_us = 780;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 780,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 2,
    });
    assert!(!world.run_imu_control_tick());

    world.board.current_time_us = 1_000;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1_000,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 3,
    });
    assert!(world.run_imu_control_tick());
    assert!(world.control_pipeline.latest_actuator_commands.is_some());
}

#[test]
fn fixed_control_rate_blocks_service_inside_control_deadline_guard() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 10_299,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
    world.last_control_update_us = 10_000;
    world.last_realtime_control_us = 10_200;
    world.next_realtime_service_us = 0;
    let pending_imu = ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 10_000,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    };
    world.processed_sensors.imu = Some(pending_imu);
    world.control_imu_accumulator.push(pending_imu);

    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::Service
    );

    world.board.current_time_us = 10_300;
    assert_eq!(world.realtime_scheduler_step(), RealtimeSchedulerStep::Idle);
}

#[test]
fn fixed_control_rate_allows_service_inside_nominal_guard_without_accumulated_imu() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 10_300,
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
    world.last_control_update_us = 10_000;

    assert!(!world.control_imu_accumulator.has_samples());
    assert!(world.realtime_service_has_control_slack(10_300));
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::Service
    );
}

#[test]
fn prioritized_service_applies_fresh_rc_in_one_service_opportunity() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 1_100_000,
            rc: Some(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 1_100_000,
                    status: 0,
                },
                n_chan: 1,
                chan: [0.5; RC_PACKET_CHANNELS],
                lol: false,
            }),
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.processed_sensors.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1_100_000,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    let result =
        world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::continuous(0));

    assert_eq!(world.board.update_count, 2);
    assert!(result.had_raw_rc);
    assert_eq!(world.processed_sensors.rc.map(|rc| rc.n_chan), Some(1));
}

#[test]
fn prioritized_service_does_not_replay_retained_rc_as_fresh_input() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard {
            current_time_us: 1_000,
            rc: Some(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 1_000,
                    status: 0,
                },
                n_chan: 1,
                chan: [1.0; RC_PACKET_CHANNELS],
                lol: false,
            }),
            ..Default::default()
        },
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );

    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 0));
    let selected_qx = world.command.combined_control().qx.value;

    // Telemetry retains the latest packet, but without a new board sample it
    // must not be consumed again as a new RC command.
    world.processed_sensors.rc = Some(RcPacket {
        header: RosflightPacketHeader {
            timestamp: 1_001,
            status: 0,
        },
        n_chan: 1,
        chan: [0.0; RC_PACKET_CHANNELS],
        lol: false,
    });
    world.board.current_time_us = 1_001;
    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 0));

    assert_eq!(world.command.combined_control().qx.value, selected_qx);
}

#[test]
fn fixed_control_rate_can_run_between_imu_edges() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard::default(),
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));

    world.board.current_time_us = 280;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 280,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::ImuControl
    );
    assert!(!world.run_imu_control_tick());

    world.board.current_time_us = 500;
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::ControlUpdate
    );
    assert!(!world.run_control_update_tick());

    world.board.current_time_us = 840;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 840,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 2,
    });
    assert!(!world.run_imu_control_tick());

    world.board.current_time_us = 1_000;
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::ControlUpdate
    );
    assert!(world.run_control_update_tick());
    assert!(world.control_pipeline.latest_actuator_commands.is_some());
}

#[test]
fn fixed_control_rate_does_not_rerun_stale_imu_sample() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard::default(),
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(400));

    world.board.current_time_us = 1_000;
    world.board.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1_000,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });
    assert!(!world.run_imu_control_tick());

    world.board.current_time_us = 2_500;
    assert!(!world.run_control_update_tick());

    world.board.current_time_us = 5_000;
    assert!(!world.run_control_update_tick());
}

#[test]
fn fixed_control_rate_does_not_consume_deadline_without_accumulated_imu() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard::default(),
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));

    world.board.current_time_us = 1_000;
    assert!(world.control_update_due_at(1_000));
    assert!(!world.run_control_update_tick());
    assert_eq!(world.last_control_update_us, 0);
}

#[test]
fn fixed_control_rate_allows_service_when_deadline_overdue_without_accumulated_imu() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard::default(),
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
    world.next_realtime_service_us = 0;

    world.board.current_time_us = 1_000;
    assert!(world.control_update_due_at(1_000));
    assert!(!world.control_update_can_run_at(1_000));
    assert!(world.realtime_service_has_control_slack(1_000));
    assert_eq!(
        world.realtime_scheduler_step(),
        RealtimeSchedulerStep::Service
    );
}

#[test]
fn fixed_control_rate_late_wake_skips_missed_intervals_without_bursting() {
    let params = Params::new();
    let mixer = quadrotor::mixer::<f64>(&params);
    let mut world = World::<
        SensorStageBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        SensorStageCommLink,
        TestPwm,
        f64,
    >::init(
        SensorStageBoard::default(),
        params,
        SensorStageCommLink::default(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
    world.control_imu_accumulator.push(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1_600,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    world.board.current_time_us = 1_600;
    assert!(world.control_update_can_run_at(1_600));
    world.consume_control_update_deadline(1_600);
    assert_eq!(world.last_control_update_us, 1_500);

    world.control_imu_accumulator.push(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 1_601,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 2,
    });
    world.board.current_time_us = 1_601;
    assert!(!world.control_update_can_run_at(1_601));
    assert_eq!(world.last_control_update_us, 1_500);
}

#[test]
fn imu_accumulator_averages_samples_for_control_deadline() {
    let mut accumulator = ImuSampleAccumulator::<f64>::default();
    accumulator.push(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 100,
            status: 1,
        },
        accel: [1.0, 2.0, 3.0],
        gyro: [4.0, 5.0, 6.0],
        temperature: 20.0,
        seq: 7,
    });
    accumulator.push(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 200,
            status: 2,
        },
        accel: [3.0, 4.0, 5.0],
        gyro: [6.0, 7.0, 8.0],
        temperature: 22.0,
        seq: 8,
    });

    let averaged = accumulator.take_average().expect("averaged sample");

    assert_eq!(averaged.header.timestamp, 200);
    assert_eq!(averaged.header.status, 2);
    assert_eq!(averaged.seq, 8);
    assert_eq!(averaged.accel, [2.0, 3.0, 4.0]);
    assert_eq!(averaged.gyro, [5.0, 6.0, 7.0]);
    assert_eq!(averaged.temperature, 21.0);
    assert!(accumulator.take_average().is_none());
}

#[test]
fn world_scheduler_processes_named_rc_packet() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
    let mut world = test_world_with_params(params);

    world.processed_sensors.rc = Some(crate::packets::RcPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 0,
            status: 0,
        },
        n_chan: 1,
        chan: [0.5; crate::packets::RC_PACKET_CHANNELS],
        lol: false,
    });

    let fresh_rc = world.processed_sensors.rc;
    world.run_rc_command_state_stages(fresh_rc);

    assert!(
        !world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::RC_LOST)
    );
}

#[test]
fn world_control_stage_runs_once_per_imu_timestamp() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
    params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.2));
    params.set_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED, ParamValue::Int(1));
    let mut world = test_world_with_params(params);

    world.state.update_arming_safety(true, true);
    world
        .state
        .update(crate::state_machine::Event::REQUEST_ARM, &world.params);
    assert!(world.run_pwm_output_stage());

    world.board.current_time_us = 1_100_000;
    world.external_attitude.latest = Some(ExternalAttitudeMsg {
        qw: 0.0,
        qx: 1.0,
        qy: 0.0,
        qz: 0.0,
    });
    let mut aux = RosflightAuxCmdMsg {
        type_array: [RosflightAuxCmdType::Disabled; 14],
        aux_cmd_array: [0.0; 14],
    };
    aux.type_array[4] = RosflightAuxCmdType::Servo;
    aux.aux_cmd_array[4] = -0.5;
    aux.type_array[5] = RosflightAuxCmdType::Motor;
    aux.aux_cmd_array[5] = 0.1;
    world.aux_commands.latest = Some(aux);
    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    assert!(!world.run_control_stages_if_new_imu());
    world
        .processed_sensors
        .imu
        .as_mut()
        .unwrap()
        .header
        .timestamp = 2;

    assert!(world.run_control_stages_if_new_imu());
    world.run_telemetry_stage();
    assert_eq!(world.pwm.send_count, 1);
    assert_eq!(world.comm.comm_link().heartbeat_count, 1);
    assert_eq!(world.comm.comm_link().status_count, 1);
    assert_eq!(world.comm.comm_link().imu_count, 1);
    assert_eq!(world.comm.comm_link().attitude_count, 1);
    assert_eq!(world.comm.comm_link().output_raw_count, 1);
    assert!(world.control_pipeline.latest_actuator_commands.is_some());
    assert!(world.external_attitude.latest.is_none());
    assert_eq!(
        world.control_pipeline.latest_estimator_state.q(),
        [1.0, 0.0, 0.0, 0.0]
    );
    assert_eq!(world.pwm.last_command_len, 14);
    assert_eq!(world.pwm.last_commands[4], 0.25);
    assert!((world.pwm.last_commands[5] - 0.2).abs() < 1e-6);
    let output_raw = world.comm.comm_link().last_output_raw.unwrap();
    assert_eq!(output_raw.values[4], 0.25);
    assert!((output_raw.values[5] - 0.2).abs() < 1e-6);

    assert!(!world.run_control_stages_if_new_imu());
    assert_eq!(world.pwm.send_count, 1);
    assert!(
        world
            .state
            .get_errors()
            .contains(ErrorFlag::TIME_GOING_BACKWARDS)
    );
    assert_eq!(world.comm.comm_link().output_raw_count, 1);

    world
        .processed_sensors
        .imu
        .as_mut()
        .unwrap()
        .header
        .timestamp = 3;

    assert!(world.run_control_stages_if_new_imu());
    assert_eq!(world.pwm.send_count, 2);
    assert!(
        !world
            .state
            .get_errors()
            .contains(ErrorFlag::TIME_GOING_BACKWARDS)
    );
    assert_eq!(world.comm.comm_link().output_raw_count, 1);
}

#[test]
fn world_control_stage_flags_non_advancing_imu_time() {
    let mut world = test_world();
    world.processed_sensors.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 10,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        ..Default::default()
    });

    assert!(!world.run_control_stages_if_new_imu());

    world.processed_sensors.imu = Some(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: 9,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        ..Default::default()
    });

    assert!(!world.run_control_stages_if_new_imu());
    assert!(
        world
            .state
            .get_errors()
            .contains(ErrorFlag::TIME_GOING_BACKWARDS)
    );
}

#[test]
fn world_telemetry_stage_streams_non_imu_sensor_without_control_update() {
    let mut world = test_world();
    world.board.current_time_us = 1_100_000;
    world.processed_sensors.baro = Some(crate::packets::BaroPacket {
        altitude: 42.0,
        pressure: 90_000.0,
        temperature: 21.0,
        ..Default::default()
    });

    world.run_telemetry_stage();

    assert_eq!(world.comm.comm_link().baro_count, 1);
    assert_eq!(world.comm.comm_link().imu_count, 0);
    assert_eq!(world.comm.comm_link().last_baro.unwrap().altitude, 42.0);
}

#[test]
fn world_telemetry_rates_follow_configured_default_stream_cadence() {
    let mut world = test_world();

    for sample in 0..40 {
        world.board.current_time_us = 1_000_000 + sample * 2_500;
        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: world.board.current_time_us,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: sample as u32,
        });

        world.run_telemetry_stage();
    }

    assert_eq!(world.comm.comm_link().heartbeat_count, 1);
    assert_eq!(world.comm.comm_link().status_count, 1);
    assert_eq!(world.comm.comm_link().imu_count, 40);
    assert_eq!(world.comm.comm_link().attitude_count, 40);
    assert_eq!(world.comm.comm_link().output_raw_count, 5);
}

#[test]
fn world_applies_live_telemetry_parameter_without_resetting_other_rates() {
    let mut world = test_world();
    let original = world.comm.telemetry_rates();
    world.board.current_time_us = 2_000_000;
    world
        .params
        .set_by_id(ParamId::PARAM_TELEM_BARO_HZ, ParamValue::Int(25));
    world
        .param_events
        .changes
        .push(crate::events::ParamChanged {
            id: ParamId::PARAM_TELEM_BARO_HZ,
            old: ParamValue::Int(0),
            new: ParamValue::Int(25),
            param_id_bytes: crate::comm::str_to_fixed_bytes("TEL_BARO_HZ"),
        })
        .unwrap();

    world.apply_param_reactions();

    let updated = world.comm.telemetry_rates();
    assert_eq!(updated.baro_hz, 25);
    assert_eq!(updated.imu_hz, original.imu_hz);
    assert_eq!(updated.attitude_hz, original.attitude_hz);
    assert!(world.param_events.changes.is_empty());
}

#[test]
fn world_status_uses_board_error_count_and_control_loop_time() {
    let mut world = test_world();
    world.board.current_time_us = 1_100_000;
    world.board.sensor_errors_count = 7;
    world.control_pipeline.latest_loop_time_us = 123;

    world.run_telemetry_stage();

    let status = world.comm.comm_link().last_status.unwrap();
    assert_eq!(status.num_errors, 7);
    assert_eq!(status.loop_time_us, 123);
}

#[test]
fn world_led_outputs_follow_rc_override_armed_error_and_failsafe_states() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
    let mut world = test_world_with_params(params);

    world.update_board_leds(0);
    assert!(!world.board.led0_high);
    assert!(world.board.led1_high);
    world.update_board_leds(500);
    assert!(!world.board.led1_high);

    let mut channels = [0.5; RC_PACKET_CHANNELS];
    channels[0] = 0.8;
    world.processed_sensors.rc = Some(RcPacket {
        header: RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        n_chan: 1,
        chan: channels,
        lol: false,
    });
    let fresh_rc = world.processed_sensors.rc;
    world.run_rc_command_state_stages(fresh_rc);
    world.update_board_leds(0);
    assert!(world.board.led0_high);

    world.state.update_arming_safety(true, true);
    world.state.update(Event::REQUEST_ARM, &world.params);
    world.update_board_leds(0);
    assert!(world.board.led1_high);

    world.state.update(
        Event::ERROR_OCCURRED(ErrorFlag::UNCALIBRATED_IMU),
        &world.params,
    );
    world.update_board_leds(500);
    assert!(!world.board.led1_high);

    let mut failsafe_world = armed_test_world_with_params(Params::new());
    failsafe_world.state.update(
        Event::ERROR_OCCURRED(ErrorFlag::RC_LOST),
        &failsafe_world.params,
    );
    failsafe_world.update_board_leds(100);
    assert!(!failsafe_world.board.led1_high);
    failsafe_world.update_board_leds(200);
    assert!(failsafe_world.board.led1_high);
}

#[test]
fn world_replays_backup_hard_error_after_companion_heartbeat() {
    let params = Params::new();
    let mixer = quadrotor::mixer(&params);
    let board = TestBoard {
        backup_data: Some(crate::board::BackupData {
            error_code: 4,
            pc: 0x1234,
            reset_count: 2,
            do_rearm: 1,
        }),
        ..Default::default()
    };
    let mut world = TestWorld::init(
        board,
        params,
        RecordingCommLink::new(),
        StateManager::new(),
        Default::default(),
        Default::default(),
        mixer,
        TestPwm::new(),
    );
    assert_eq!(world.board.backup_clear_count, 1);
    assert_eq!(world.pending_hard_error.unwrap().do_rearm, 1);
    assert!(world.state.is_armed());

    world.comm.msgs.heartbeat = Some(HeartbeatMsg {
        type_: 1,
        autopilot: 2,
        base_mode: 3,
        custom_mode: 4,
        system_status: 5,
        mavlink_version: 6,
    });
    world.run_comm_param_sensor_stages();

    assert_eq!(world.comm.comm_link().hard_error_count, 1);
    assert_eq!(world.comm.comm_link().last_hard_error.unwrap().pc, 0x1234);
}

#[test]
fn world_control_stage_propagates_custom_zero_pwm_rates() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(11));
    let mut world = test_world_with_params(params);
    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    assert!(!world.run_control_stages_if_new_imu());
    world
        .processed_sensors
        .imu
        .as_mut()
        .unwrap()
        .header
        .timestamp = 2;

    assert!(world.run_control_stages_if_new_imu());

    assert_eq!(world.pwm.configure_count, 1);
    assert_eq!(world.pwm.last_rate_len, 10);
    assert_eq!(world.pwm.last_rates, [0.0; 10]);

    world
        .processed_sensors
        .imu
        .as_mut()
        .unwrap()
        .header
        .timestamp = 3;
    assert!(world.run_control_stages_if_new_imu());
    assert_eq!(world.pwm.configure_count, 1);
}

#[test]
fn world_keeps_motor_outputs_low_while_custom_mixer_params_load_disarmed() {
    use crate::mixer::Mixer as _;

    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
    let mut world = test_world_with_params(params);
    assert!(world.run_pwm_output_stage());
    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });
    assert!(!world.run_control_stages_if_new_imu());

    // Replay a parameter file that selects the custom mixer before it marks
    // outputs 0-3 as motors, checking the outputs after every step.
    let steps = [
        (ParamId::PARAM_PRIMARY_MIXER, 11),
        (ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0, 2),
        (ParamId::PARAM_PRIMARY_MIXER_OUTPUT_1, 2),
        (ParamId::PARAM_PRIMARY_MIXER_OUTPUT_2, 2),
        (ParamId::PARAM_PRIMARY_MIXER_OUTPUT_3, 2),
    ];
    for (step, (id, value)) in steps.into_iter().enumerate() {
        param_service::set_param_and_emit_change(
            &mut world.params,
            &mut world.param_events.changes,
            id,
            ParamValue::Int(value),
        );
        world.apply_param_reactions();
        if step == 0 {
            assert!(
                world.mixer.output_types()[..4]
                    .iter()
                    .all(|output_type| *output_type == crate::mixer::MixerOutputType::Aux)
            );
        }

        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 2 + step as u64;
        let send_count = world.pwm.send_count;
        assert!(world.run_control_stages_if_new_imu());
        assert!(!world.state.is_armed());
        assert_eq!(world.pwm.send_count, send_count + 1);
        assert_eq!(world.pwm.last_commands[..4], [0.0; 4], "step {step}");
        assert_eq!(
            world.control_pipeline.latest_pwm_outputs[..4],
            [0.0; 4],
            "step {step}"
        );
    }
    assert!(
        world.mixer.output_types()[..4]
            .iter()
            .all(|output_type| *output_type == crate::mixer::MixerOutputType::Motor)
    );
}

#[test]
fn world_control_stage_reconfigures_pwm_rates_after_mixer_param_change() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(11));
    let mut world = test_world_with_params(params);
    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    assert!(!world.run_control_stages_if_new_imu());
    world
        .processed_sensors
        .imu
        .as_mut()
        .unwrap()
        .header
        .timestamp = 2;
    assert!(world.run_control_stages_if_new_imu());
    assert_eq!(world.pwm.configure_count, 1);

    world
        .params
        .set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(0));
    world
        .param_events
        .changes
        .push(crate::events::ParamChanged {
            id: ParamId::PARAM_PRIMARY_MIXER,
            old: ParamValue::Int(11),
            new: ParamValue::Int(0),
            param_id_bytes: [0; 16],
        })
        .unwrap();
    world.apply_param_reactions();

    world
        .processed_sensors
        .imu
        .as_mut()
        .unwrap()
        .header
        .timestamp = 3;
    assert!(world.run_control_stages_if_new_imu());
    assert_eq!(world.pwm.configure_count, 2);
    assert_eq!(world.pwm.last_rate_len, 10);
    assert_eq!(world.pwm.last_rates, [50.0; 10]);
}

#[test]
fn world_sensor_health_sets_and_clears_imu_timeout() {
    let mut world = test_world();

    world.board.current_time_us = IMU_TIMEOUT_US + 1;
    world.update_sensor_health_and_calibration(world.board.clock_micros());

    assert!(
        world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
    );

    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: IMU_TIMEOUT_US + 2,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });
    world.update_sensor_health_and_calibration(world.board.clock_micros());

    assert!(
        !world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
    );
}

#[test]
fn world_sensor_health_sets_uncalibrated_imu_when_all_bias_params_are_zero() {
    let mut world = test_world();
    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    world.update_sensor_health_and_calibration(world.board.clock_micros());

    assert!(
        world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::UNCALIBRATED_IMU)
    );
}

#[test]
fn world_sensor_health_keeps_uncalibrated_imu_latched_after_live_bias_change() {
    let mut world = test_world();
    world.state.update(
        crate::state_machine::Event::ERROR_OCCURRED(
            crate::state_machine::ErrorFlag::UNCALIBRATED_IMU,
        ),
        &world.params,
    );
    world
        .params
        .set_by_id(ParamId::PARAM_ACC_X_BIAS, ParamValue::Float(0.01));
    world.processed_sensors.imu = Some(crate::packets::ImuPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        accel: [0.0, 0.0, -9.80665],
        gyro: [0.0, 0.0, 0.0],
        temperature: 25.0,
        seq: 1,
    });

    world.update_sensor_health_and_calibration(world.board.clock_micros());

    assert!(
        world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::UNCALIBRATED_IMU)
    );
}

#[test]
fn world_gyro_only_calibration_does_not_clear_latched_imu_error() {
    let mut world = test_world();
    world.cal_flags.insert(CalibrationFlags::GYRO);

    for seq in 0..=1000 {
        world.raw_sensors.imu = Some(Ok(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: u64::from(seq) + 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.02, -0.01, 0.03],
            temperature: 298.15,
            seq,
        }));
        world.process_imu_sensor_after_update();
    }

    assert!(!world.cal_flags.contains(CalibrationFlags::GYRO));
    assert!(
        world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::UNCALIBRATED_IMU)
    );
}

#[test]
fn world_successful_full_imu_calibration_clears_latched_error() {
    let mut world = test_world();
    world.cal_flags.insert(CalibrationFlags::IMU);

    for seq in 0..=1000 {
        world.raw_sensors.imu = Some(Ok(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: u64::from(seq) + 1,
                status: 0,
            },
            accel: [0.1, -0.2, -9.50665],
            gyro: [0.02, -0.01, 0.03],
            temperature: 298.15,
            seq,
        }));
        world.process_imu_sensor_after_update();
    }

    assert!(!world.cal_flags.intersects(CalibrationFlags::IMU));
    assert!(
        !world
            .cal_flags
            .intersects(CalibrationFlags::GYRO_FAILED | CalibrationFlags::ACCEL_FAILED)
    );
    assert!(
        !world
            .state
            .get_errors()
            .contains(crate::state_machine::ErrorFlag::UNCALIBRATED_IMU)
    );
}

#[test]
fn world_sends_calibration_ack_when_calibration_starts() {
    let mut world = test_world();

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::GyroCalibration,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.cal_flags.contains(CalibrationFlags::GYRO));
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::GyroCalibration));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdSuccess
    ));
}

#[test]
fn world_default_channel_output_mask_physically_enables_only_first_four_channels() {
    let mut world = test_world_with_params(Params::new());

    assert!(world.run_pwm_output_stage());
    assert_eq!(world.pwm.enabled_mask, 0x0f);
    assert_eq!(world.pwm_output.enabled_mask(), 0x0f);
    assert_eq!(world.pwm.enable_all_count, 0);
}

#[test]
fn world_pwm_output_stage_follows_armed_state_transitions() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
    let mut world = test_world_with_params(params);

    assert!(world.run_pwm_output_stage());
    assert!(world.pwm_output.is_enabled());
    assert_eq!(world.pwm.enable_all_count, 1);
    assert_eq!(world.pwm.disable_all_count, 0);

    world.state.update_arming_safety(true, true);
    world
        .state
        .update(crate::state_machine::Event::REQUEST_ARM, &world.params);

    assert!(!world.run_pwm_output_stage());
    assert!(world.pwm_output.is_enabled());
    assert_eq!(world.pwm.enable_all_count, 1);

    assert!(!world.run_pwm_output_stage());
    assert_eq!(world.pwm.enable_all_count, 1);

    world
        .state
        .update(crate::state_machine::Event::REQUEST_DISARM, &world.params);

    assert!(!world.run_pwm_output_stage());
    assert!(world.pwm_output.is_enabled());
    assert_eq!(world.pwm.disable_all_count, 0);
    assert_eq!(world.pwm.flush_count, 0);
}

#[test]
fn world_channel_output_mask_zero_disables_the_entire_pwm_driver() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
    let mut world = test_world_with_params(params);

    assert!(world.run_pwm_output_stage());
    assert!(world.pwm_output.is_enabled());
    assert_eq!(world.pwm.enable_all_count, 1);

    world
        .params
        .set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(0));

    assert!(world.run_pwm_output_stage());
    assert!(!world.pwm_output.is_enabled());
    assert_eq!(world.pwm.disable_all_count, 1);
    assert_eq!(world.pwm.flush_count, 1);
}

#[test]
fn world_pwm_output_stage_disables_outputs_while_rc_kill_switch_is_active() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
    params.set_by_id(ParamId::PARAM_RC_OUTPUT_KILL_CHANNEL, ParamValue::Int(4));
    let mut world = test_world_with_params(params);

    assert!(world.run_pwm_output_stage());
    assert!(world.pwm_output.is_enabled());
    assert_eq!(world.pwm.enable_all_count, 1);

    let mut channels = [0.5; RC_PACKET_CHANNELS];
    channels[4] = 1.0;
    world.rc.receive(&RcPacket {
        header: RosflightPacketHeader {
            timestamp: 1_000,
            status: 0,
        },
        n_chan: 6,
        chan: channels,
        lol: false,
    });
    world.rc.run(1, &world.params, &mut world.state);

    assert!(world.run_pwm_output_stage());
    assert!(!world.pwm_output.is_enabled());
    assert_eq!(world.pwm.disable_all_count, 1);
    assert_eq!(world.pwm.flush_count, 1);

    channels[4] = 0.0;
    world.rc.receive(&RcPacket {
        header: RosflightPacketHeader {
            timestamp: 2_000,
            status: 0,
        },
        n_chan: 6,
        chan: channels,
        lol: false,
    });
    world.rc.run(2, &world.params, &mut world.state);

    assert!(world.run_pwm_output_stage());
    assert!(world.pwm_output.is_enabled());
    assert_eq!(world.pwm.enable_all_count, 2);
    assert_eq!(world.pwm.disable_all_count, 1);
}

#[test]
fn world_applies_offboard_control_command_event() {
    let mut world = test_world();
    world.board.current_time_us = 1_100_000;
    seed_healthy_rc(&mut world);

    world.comm.msgs.offboard_control = Some(OffboardControlMsg {
        mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
        ignore: OffboardControlIgnore::IGNORE_QY,
        qx: 0.1,
        qy: 0.2,
        qz: 0.3,
        fx: 0.4,
        fy: 0.5,
        fz: 0.6,
        passthrough: [0.0; 4],
    });

    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 0));

    assert!(world.command.is_offboard_active());
    assert!(world.command_events.offboard_control_requests.is_empty());
    assert_eq!(world.command.combined_control().qx.value, 0.1);
    assert_eq!(
        world.command.combined_control().qx.control_type,
        crate::command::ControlType::Rate
    );
    assert_eq!(world.pwm.send_count, 0);
}

#[test]
fn prioritized_service_expires_offboard_without_fresh_rc() {
    let mut world = test_world();
    world.board.current_time_us = 1_100_000;
    seed_healthy_rc(&mut world);
    world.comm.msgs.offboard_control = Some(OffboardControlMsg {
        mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
        ignore: OffboardControlIgnore::empty(),
        qx: 0.25,
        qy: 0.0,
        qz: 0.0,
        fx: 0.0,
        fy: 0.0,
        fz: 0.0,
        passthrough: [0.0; 4],
    });

    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 0));
    assert!(world.command.is_offboard_active());
    assert_eq!(world.command.combined_control().qx.value, 0.25);

    world.board.current_time_us = 1_201_000;
    world.run_prioritized_service_steps_with_policy(RealtimeServicePolicy::with_spacing(1, 0));

    assert!(!world.command.is_offboard_active());
    assert_ne!(world.command.combined_control().qx.value, 0.25);
    assert_eq!(world.pwm.send_count, 0);
}

#[test]
fn world_applies_companion_input_events() {
    let mut world = test_world();

    world.comm.msgs.heartbeat = Some(HeartbeatMsg {
        type_: 1,
        autopilot: 2,
        base_mode: 3,
        custom_mode: 4,
        system_status: 5,
        mavlink_version: 6,
    });
    let mut aux = RosflightAuxCmdMsg {
        type_array: [RosflightAuxCmdType::Disabled; 14],
        aux_cmd_array: [0.0; 14],
    };
    aux.type_array[3] = RosflightAuxCmdType::Servo;
    aux.aux_cmd_array[3] = 0.8;
    world.comm.msgs.aux_cmd = Some(aux);
    world.comm.msgs.external_attitude = Some(ExternalAttitudeMsg {
        qw: 1.0,
        qx: 0.1,
        qy: 0.2,
        qz: 0.3,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.companion_link.connected);
    assert_eq!(
        world.companion_link.last_heartbeat.unwrap().system_status,
        5
    );
    let aux = world.aux_commands.latest.unwrap();
    assert!(matches!(aux.type_array[3], RosflightAuxCmdType::Servo));
    assert_eq!(aux.aux_cmd_array[3], 0.8);
    assert_eq!(world.external_attitude.latest.unwrap().qz, 0.3);
    assert!(world.companion_events.heartbeats.is_empty());
    assert!(world.companion_events.aux_commands.is_empty());
    assert!(world.companion_events.external_attitudes.is_empty());
}

#[test]
fn world_applies_param_defaults_and_sends_ack_after_apply() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
    let mut world = test_world_with_params(params);

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::SetParamDefaults,
    });

    world.run_comm_param_sensor_stages();

    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
        ParamValue::Int(1)
    );
    assert!(world.command_events.param_defaults_requests.is_empty());
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);

    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdSuccess
    ));
}

#[test]
fn world_routes_board_command_and_acks_unsupported_after_apply_stage() {
    let mut world = test_world();

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::WriteParams,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.command_events.board_command_requests.is_empty());
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::WriteParams));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdFailed
    ));
}

#[test]
fn world_drains_logs_through_comm_response_stage() {
    while crate::log::Logger::pop().is_some() {}

    let mut world = test_world();
    world.companion_link.connected = true;
    while crate::log::Logger::pop().is_some() {}

    crate::log_info!("world log");
    world.run_comm_param_sensor_stages();

    assert_eq!(world.comm.comm_link().statustext_count, 1);
    let msg = world.comm.comm_link().last_statustext.unwrap();
    assert_eq!(&msg.text[..9], b"world log");
}

#[test]
fn world_routes_rc_trim_calibration_and_sets_equilibrium_torques() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_RC_ATTITUDE_MODE, ParamValue::Int(0));
    params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(4));
    params.set_by_id(ParamId::PARAM_RC_MAX_ROLLRATE, ParamValue::Float(1.0));
    params.set_by_id(ParamId::PARAM_RC_MAX_PITCHRATE, ParamValue::Float(1.0));
    params.set_by_id(ParamId::PARAM_RC_MAX_YAWRATE, ParamValue::Float(1.0));
    params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_P, ParamValue::Float(2.0));
    params.set_by_id(ParamId::PARAM_PID_PITCH_RATE_P, ParamValue::Float(3.0));
    params.set_by_id(ParamId::PARAM_PID_YAW_RATE_P, ParamValue::Float(4.0));
    params.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(0.5));
    params.set_by_id(ParamId::PARAM_Y_EQ_TORQUE, ParamValue::Float(-0.5));
    params.set_by_id(ParamId::PARAM_Z_EQ_TORQUE, ParamValue::Float(0.25));
    let mut world = test_world_with_params(params);

    let mut channels = [0.5; crate::packets::RC_PACKET_CHANNELS];
    channels[0] = 0.55;
    channels[1] = 0.45;
    channels[3] = 0.60;
    world.processed_sensors.rc = Some(crate::packets::RcPacket {
        header: crate::packets::RosflightPacketHeader {
            timestamp: 1,
            status: 0,
        },
        n_chan: 4,
        chan: channels,
        lol: false,
    });
    let fresh_rc = world.processed_sensors.rc;
    world.run_rc_command_state_stages(fresh_rc);

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::RcCalibration,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.command_events.rc_trim_calibration_requests.is_empty());
    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_X_EQ_TORQUE),
        ParamValue::Float(0.70000005)
    );
    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_Y_EQ_TORQUE),
        ParamValue::Float(-0.8000001)
    );
    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_Z_EQ_TORQUE),
        ParamValue::Float(1.0500002)
    );
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::RcCalibration));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdSuccess
    ));
}

#[test]
fn world_routes_reset_origin_and_acks_unsupported_after_apply_stage() {
    let mut world = test_world();

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::ResetOrigin,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.command_events.reset_origin_requests.is_empty());
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::ResetOrigin));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdFailed
    ));
}

#[test]
fn world_routes_config_info_and_acks_unsupported_after_apply_stage() {
    let mut world = test_world();

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::SendAllConfigInfos,
    });

    world.run_comm_param_sensor_stages();

    assert!(world.command_events.config_info_requests.is_empty());
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::SendAllConfigInfos));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdFailed
    ));
}

#[test]
fn world_rejects_command_actions_while_armed() {
    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
    params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
    let mut world = armed_test_world_with_params(params);
    assert!(world.state.is_armed());

    world.comm.msgs.cmd = Some(RosflightCmdMsg {
        command: RosflightCmd::SetParamDefaults,
    });

    world.run_comm_param_sensor_stages();

    assert_eq!(
        world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
        ParamValue::Int(42)
    );
    assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
    let ack = world.comm.comm_link().last_cmd_ack.unwrap();
    assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
    assert!(matches!(
        ack.success,
        RosflightCmdResponse::RosflightCmdFailed
    ));
}
