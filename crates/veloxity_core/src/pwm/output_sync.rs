use crate::{
    board::BoardIo,
    comm::messages::{enums::RosflightAuxCmdType, messages::RosflightAuxCmdMsg},
    math::FlightFloat,
    mixer::MixerOutputType,
    params::{ParamId, ParamValue, Params},
    pwm::{PwmDriver, PwmError, safe_disarmed_command},
    state_machine::StateManager,
};

pub const PWM_OUTPUT_CHANNELS: usize = 14;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmOutputState {
    enabled_mask: u32,
}

impl PwmOutputState {
    pub fn new(enabled: bool) -> Self {
        Self {
            enabled_mask: if enabled { u32::MAX } else { 0 },
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled_mask != 0
    }

    pub fn enabled_mask(&self) -> u32 {
        self.enabled_mask
    }
}

pub struct PwmSyncCtx<'a, B, P>
where
    B: BoardIo,
{
    pub board: &'a mut B,
    pub pwm: &'a mut P,
    pub output: &'a mut PwmOutputState,
    pub output_kill_active: bool,
    pub channel_output_mask: i32,
}

pub fn sync_pwm_output_state<B, P, R>(ctx: PwmSyncCtx<'_, B, P>) -> Result<bool, PwmError>
where
    B: BoardIo,
    P: PwmDriver<R>,
    R: FlightFloat,
{
    let channel_count = ctx.pwm.len().min(u32::BITS as usize);
    let available_mask = if channel_count == u32::BITS as usize {
        u32::MAX
    } else {
        (1_u32 << channel_count) - 1
    };
    let desired_mask = if ctx.output_kill_active {
        0
    } else if ctx.channel_output_mask == -1 {
        available_mask
    } else if ctx.channel_output_mask >= 0 {
        ctx.channel_output_mask as u32 & available_mask
    } else {
        0
    };
    let current_mask = ctx.output.enabled_mask & available_mask;

    if desired_mask == current_mask {
        ctx.output.enabled_mask = current_mask;
        return Ok(false);
    }

    if desired_mask == available_mask && current_mask == 0 {
        ctx.pwm.enable_all()?;
        ctx.output.enabled_mask = desired_mask;
        return Ok(true);
    }

    if desired_mask == 0 {
        ctx.pwm.disable_all();
        ctx.pwm.flush(ctx.board);
        ctx.output.enabled_mask = 0;
        return Ok(true);
    }

    let mut applied_mask = current_mask;
    let disable_mask = current_mask & !desired_mask;
    for channel in 0..channel_count {
        let bit = 1_u32 << channel;
        if disable_mask & bit != 0 {
            ctx.pwm.disable(channel)?;
            applied_mask &= !bit;
            ctx.output.enabled_mask = applied_mask;
        }
    }
    if disable_mask != 0 {
        ctx.pwm.flush(ctx.board);
    }

    let enable_mask = desired_mask & !current_mask;
    for channel in 0..channel_count {
        let bit = 1_u32 << channel;
        if enable_mask & bit != 0 {
            ctx.pwm.enable(channel)?;
            applied_mask |= bit;
            ctx.output.enabled_mask = applied_mask;
        }
    }

    Ok(true)
}

pub fn write_pwm_commands<B, P, R>(
    board: &mut B,
    pwm: &mut P,
    output: &PwmOutputState,
    commands: &[R],
    output_types: &[MixerOutputType],
    state: &StateManager,
) -> Result<bool, PwmError>
where
    B: BoardIo,
    P: PwmDriver<R>,
    R: FlightFloat,
{
    if !output.is_enabled() {
        return Ok(false);
    }

    if state.is_armed() {
        pwm.send_commands(board, commands)?;
    } else {
        pwm.send_disarmed_commands(board, output_types)?;
    }
    Ok(true)
}

pub fn compose_pwm_outputs<R: FlightFloat>(
    primary_commands: &[R],
    primary_output_types: &[MixerOutputType],
    aux_command: Option<&RosflightAuxCmdMsg>,
    state: &StateManager,
    params: &Params,
) -> [R; PWM_OUTPUT_CHANNELS] {
    let idle_throttle = match params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) {
        ParamValue::Float(value) => <R as FlightFloat>::from_f32(value),
        _ => <R as FlightFloat>::from_f32(0.0),
    };
    let spin_when_armed = match params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) {
        ParamValue::Int(value) => value != 0,
        _ => false,
    };
    let channel_output_mask = match params.get_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK) {
        ParamValue::Int(value) => value,
        _ => 0,
    };

    let mut outputs = [<R as FlightFloat>::from_f32(0.0); PWM_OUTPUT_CHANNELS];

    for channel in 0..PWM_OUTPUT_CHANNELS {
        let primary_type = primary_output_types
            .get(channel)
            .copied()
            .unwrap_or(MixerOutputType::Aux);
        let primary_value = primary_commands
            .get(channel)
            .copied()
            .unwrap_or_else(|| <R as FlightFloat>::from_f32(0.0));
        let (output_type, value) = if primary_type == MixerOutputType::Aux {
            aux_output_for_channel(aux_command, channel)
        } else {
            (primary_type, primary_value)
        };

        outputs[channel] = if !state.is_armed() {
            safe_disarmed_command(output_type)
        } else if !channel_output_enabled(channel_output_mask, channel) {
            safe_disarmed_command(output_type)
        } else {
            raw_output_for_type(output_type, value, state, idle_throttle, spin_when_armed)
        };
    }

    outputs
}

fn channel_output_enabled(mask: i32, channel: usize) -> bool {
    mask == -1 || (mask >= 0 && channel < i32::BITS as usize && (mask & (1_i32 << channel)) != 0)
}

fn aux_output_for_channel<R: FlightFloat>(
    aux_command: Option<&RosflightAuxCmdMsg>,
    channel: usize,
) -> (MixerOutputType, R) {
    let Some(aux_command) = aux_command else {
        return (MixerOutputType::Aux, <R as FlightFloat>::from_f32(0.0));
    };

    match aux_command.type_array[channel] {
        RosflightAuxCmdType::Disabled => (MixerOutputType::Aux, <R as FlightFloat>::from_f32(0.0)),
        RosflightAuxCmdType::Servo => (
            MixerOutputType::Servo,
            <R as FlightFloat>::from_f32(aux_command.aux_cmd_array[channel]),
        ),
        RosflightAuxCmdType::Motor => (
            MixerOutputType::Motor,
            <R as FlightFloat>::from_f32(aux_command.aux_cmd_array[channel]),
        ),
    }
}

fn raw_output_for_type<R: FlightFloat>(
    output_type: MixerOutputType,
    value: R,
    state: &StateManager,
    idle_throttle: R,
    spin_when_armed: bool,
) -> R {
    match output_type {
        MixerOutputType::Aux => <R as FlightFloat>::from_f32(0.0),
        MixerOutputType::Servo => {
            value.clamp(
                <R as FlightFloat>::from_f32(-1.0),
                <R as FlightFloat>::from_f32(1.0),
            ) * <R as FlightFloat>::from_f32(0.5)
                + <R as FlightFloat>::from_f32(0.5)
        }
        MixerOutputType::Gpio => {
            if value > <R as FlightFloat>::from_f32(0.0) {
                <R as FlightFloat>::from_f32(1.0)
            } else {
                <R as FlightFloat>::from_f32(0.0)
            }
        }
        MixerOutputType::Motor => {
            if !state.is_armed() {
                <R as FlightFloat>::from_f32(0.0)
            } else if value > <R as FlightFloat>::from_f32(1.0) {
                <R as FlightFloat>::from_f32(1.0)
            } else if value < idle_throttle && spin_when_armed {
                idle_throttle
            } else if value < <R as FlightFloat>::from_f32(0.0) {
                <R as FlightFloat>::from_f32(0.0)
            } else {
                value
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        errors,
        params::{ParamId, ParamValue, Params},
        state_machine::Event,
    };

    struct TestBoard {
        now_us: u64,
    }

    impl BoardIo for TestBoard {
        fn serial_rx_read(&mut self, _buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
            None
        }

        fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
            Some(Ok(bytes.len()))
        }

        fn clock_millis(&self) -> u32 {
            (self.now_us / 1000) as u32
        }

        fn clock_micros(&self) -> u64 {
            self.now_us
        }
    }

    struct TestPwm {
        enabled_mask: u32,
        enable_all_count: usize,
        disable_all_count: usize,
        flush_count: usize,
        send_count: usize,
        last_commands: [f64; PWM_OUTPUT_CHANNELS],
    }

    impl TestPwm {
        fn new(enabled: bool) -> Self {
            Self {
                enabled_mask: if enabled { 0b1111 } else { 0 },
                enable_all_count: 0,
                disable_all_count: 0,
                flush_count: 0,
                send_count: 0,
                last_commands: [f64::NAN; PWM_OUTPUT_CHANNELS],
            }
        }
    }

    impl PwmDriver<f64> for TestPwm {
        fn len(&self) -> usize {
            4
        }

        fn is_enabled(&self) -> bool {
            self.enabled_mask == 0b1111
        }

        fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
            self.enabled_mask |= 1 << channel;
            Ok(())
        }

        fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
            self.enabled_mask &= !(1 << channel);
            Ok(())
        }

        fn enable_all(&mut self) -> Result<(), PwmError> {
            self.enabled_mask = 0b1111;
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

        fn send_commands<Board: BoardIo>(
            &mut self,
            _board: &mut Board,
            commands: &[f64],
        ) -> Result<(), PwmError> {
            self.send_count += 1;
            let len = commands.len().min(PWM_OUTPUT_CHANNELS);
            self.last_commands[..len].copy_from_slice(&commands[..len]);
            Ok(())
        }
    }

    #[test]
    fn pwm_output_state_defaults_to_enabled_safe_disarmed_outputs() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let mut output = PwmOutputState::new(pwm.is_enabled());

        assert!(
            sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert!(output.is_enabled());
        assert_eq!(pwm.enable_all_count, 1);
        assert_eq!(pwm.disable_all_count, 0);

        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);

        assert!(
            !sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert!(output.is_enabled());
        assert_eq!(pwm.enable_all_count, 1);

        assert!(
            !sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert_eq!(pwm.enable_all_count, 1);

        state.update(Event::REQUEST_DISARM, &params);

        assert!(
            !sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert!(output.is_enabled());
        assert_eq!(pwm.disable_all_count, 0);
        assert_eq!(pwm.flush_count, 0);
    }

    #[test]
    fn pwm_output_state_physically_applies_channel_mask() {
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let mut output = PwmOutputState::new(false);

        assert!(
            sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: 0b0011,
            })
            .unwrap()
        );
        assert_eq!(pwm.enabled_mask, 0b0011);
        assert_eq!(output.enabled_mask(), 0b0011);
        assert_eq!(pwm.enable_all_count, 0);

        assert!(
            sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: 0b0101,
            })
            .unwrap()
        );
        assert_eq!(pwm.enabled_mask, 0b0101);
        assert_eq!(output.enabled_mask(), 0b0101);
        assert_eq!(pwm.flush_count, 1);
    }

    #[test]
    fn pwm_output_state_disables_outputs_while_kill_switch_is_active() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let mut output = PwmOutputState::new(pwm.is_enabled());

        assert!(
            sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert!(output.is_enabled());

        assert!(
            sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: true,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert!(!output.is_enabled());
        assert_eq!(pwm.disable_all_count, 1);
        assert_eq!(pwm.flush_count, 1);

        assert!(
            sync_pwm_output_state(PwmSyncCtx {
                board: &mut board,
                pwm: &mut pwm,
                output: &mut output,
                output_kill_active: false,
                channel_output_mask: -1,
            })
            .unwrap()
        );
        assert!(output.is_enabled());
        assert_eq!(pwm.enable_all_count, 2);
    }

    #[test]
    fn write_pwm_commands_only_writes_when_output_enabled() {
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let disabled = PwmOutputState::new(false);
        let enabled = PwmOutputState::new(true);
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);

        assert_eq!(
            write_pwm_commands(&mut board, &mut pwm, &disabled, &[0.1, 0.2], &[], &state),
            Ok(false)
        );
        assert_eq!(pwm.send_count, 0);

        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        assert_eq!(
            write_pwm_commands(&mut board, &mut pwm, &enabled, &[0.1, 0.2], &[], &state),
            Ok(true)
        );
        assert_eq!(pwm.send_count, 1);
    }

    #[test]
    fn write_pwm_commands_sends_safe_defaults_when_disarmed() {
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let enabled = PwmOutputState::new(true);
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);

        assert_eq!(
            write_pwm_commands(
                &mut board,
                &mut pwm,
                &enabled,
                &[0.9, 0.9],
                &[MixerOutputType::Motor, MixerOutputType::Aux],
                &state
            ),
            Ok(true)
        );
        assert_eq!(pwm.send_count, 1);
        assert_eq!(pwm.last_commands, [0.0; PWM_OUTPUT_CHANNELS]);
    }

    #[test]
    fn safe_disarmed_command_holds_only_servos_at_mid_range() {
        assert_eq!(safe_disarmed_command::<f64>(MixerOutputType::Motor), 0.0);
        assert_eq!(safe_disarmed_command::<f64>(MixerOutputType::Gpio), 0.0);
        assert_eq!(safe_disarmed_command::<f64>(MixerOutputType::Aux), 0.0);
        assert_eq!(safe_disarmed_command::<f64>(MixerOutputType::Servo), 0.5);
    }

    #[test]
    fn send_disarmed_commands_drives_untyped_channels_low_and_centers_servos() {
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(true);

        assert_eq!(
            pwm.send_disarmed_commands(&mut board, &[MixerOutputType::Servo]),
            Ok(())
        );

        // Channel 0 is a servo; channels 1-3 are within the driver but have no
        // mixer type, and the rest are beyond the driver's channel count.
        let mut expected = [0.0; PWM_OUTPUT_CHANNELS];
        expected[0] = 0.5;
        assert_eq!(pwm.last_commands, expected);
    }

    #[test]
    fn custom_mixer_with_unassigned_outputs_keeps_disarmed_outputs_low() {
        use crate::mixer::Mixer as _;

        // Selecting the custom mixer before any PRI_MIXER_OUT_* parameter has
        // arrived leaves every mixer output typed as Aux.
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(11));
        let mut mixer = crate::vehicle::quadrotor::mixer::<f64>(&params);
        assert!(
            mixer
                .output_types()
                .iter()
                .all(|output_type| *output_type == MixerOutputType::Aux)
        );
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        assert!(!state.is_armed());

        let outputs: [f64; PWM_OUTPUT_CHANNELS] =
            compose_pwm_outputs(&[0.9; 10], mixer.output_types(), None, &state, &params);
        assert_eq!(outputs, [0.0; PWM_OUTPUT_CHANNELS]);

        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(true);
        let enabled = PwmOutputState::new(true);
        assert_eq!(
            write_pwm_commands(
                &mut board,
                &mut pwm,
                &enabled,
                &outputs,
                mixer.output_types(),
                &state
            ),
            Ok(true)
        );
        assert_eq!(pwm.last_commands, [0.0; PWM_OUTPUT_CHANNELS]);

        // An output explicitly assigned as a servo is still centered.
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_3, ParamValue::Int(1));
        mixer.on_param_changed(&params, ParamId::PARAM_PRIMARY_MIXER_OUTPUT_3);
        assert_eq!(mixer.output_types()[3], MixerOutputType::Servo);

        let outputs: [f64; PWM_OUTPUT_CHANNELS] =
            compose_pwm_outputs(&[0.9; 10], mixer.output_types(), None, &state, &params);
        let mut expected = [0.0; PWM_OUTPUT_CHANNELS];
        expected[3] = 0.5;
        assert_eq!(outputs, expected);

        assert_eq!(
            write_pwm_commands(
                &mut board,
                &mut pwm,
                &enabled,
                &outputs,
                mixer.output_types(),
                &state
            ),
            Ok(true)
        );
        assert_eq!(pwm.last_commands, expected);
    }

    #[test]
    fn compose_pwm_outputs_preserves_primary_and_applies_aux_to_unused_channels() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.2));
        params.set_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; PWM_OUTPUT_CHANNELS],
            aux_cmd_array: [0.0; PWM_OUTPUT_CHANNELS],
        };
        aux.type_array[4] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[4] = -0.5;
        aux.type_array[5] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[5] = 0.1;

        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];
        let outputs: [f64; PWM_OUTPUT_CHANNELS] = compose_pwm_outputs(
            &[0.1, 0.2, 0.3, 0.4],
            &output_types,
            Some(&aux),
            &state,
            &params,
        );

        assert!((outputs[0] - 0.2).abs() < 1e-6);
        assert!((outputs[1] - 0.2).abs() < 1e-6);
        assert_eq!(outputs[2], 0.3);
        assert_eq!(outputs[3], 0.4);
        assert_eq!(outputs[4], 0.25);
        assert!((outputs[5] - 0.2).abs() < 1e-6);
        assert_eq!(outputs[6], 0.0);
    }

    #[test]
    fn compose_pwm_outputs_forces_aux_motors_low_when_disarmed() {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; PWM_OUTPUT_CHANNELS],
            aux_cmd_array: [0.0; PWM_OUTPUT_CHANNELS],
        };
        aux.type_array[4] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[4] = 0.8;

        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];
        let outputs: [f64; PWM_OUTPUT_CHANNELS] = compose_pwm_outputs(
            &[0.1, 0.2, 0.3, 0.4],
            &output_types,
            Some(&aux),
            &state,
            &params,
        );

        assert_eq!(outputs[4], 0.0);
        assert_eq!(outputs[6], 0.0);
    }

    #[test]
    fn compose_pwm_outputs_applies_channel_output_mask_to_every_output_type() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.2));
        params.set_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(0b0101));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Servo,
            MixerOutputType::Gpio,
            MixerOutputType::Aux,
        ];

        let outputs: [f64; PWM_OUTPUT_CHANNELS] =
            compose_pwm_outputs(&[0.1, 0.4, 0.3, 0.5], &output_types, None, &state, &params);

        assert!((outputs[0] - 0.2).abs() < 1e-6);
        assert_eq!(outputs[1], 0.5);
        assert_eq!(outputs[2], 1.0);
        assert_eq!(outputs[3], 0.0);
    }

    #[test]
    fn compose_pwm_outputs_default_channel_mask_enables_first_four_channels() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Servo,
            MixerOutputType::Gpio,
            MixerOutputType::Aux,
        ];

        let outputs: [f64; PWM_OUTPUT_CHANNELS] =
            compose_pwm_outputs(&[0.1, 0.2, 0.3, 0.4], &output_types, None, &state, &params);

        for (output, expected) in outputs.iter().zip([0.1, 0.6, 1.0, 0.0]) {
            assert!((output - expected).abs() < 1e-6);
        }
    }

    #[test]
    fn compose_pwm_outputs_uses_aux_inside_primary_range_only_for_aux_owned_slots() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; PWM_OUTPUT_CHANNELS],
            aux_cmd_array: [0.0; PWM_OUTPUT_CHANNELS],
        };
        aux.type_array[1] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[1] = 1.0;
        aux.type_array[2] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[2] = -1.0;
        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Aux,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];

        let outputs: [f64; PWM_OUTPUT_CHANNELS] = compose_pwm_outputs(
            &[0.1, 0.2, 0.3, 0.4],
            &output_types,
            Some(&aux),
            &state,
            &params,
        );

        assert!((outputs[0] - 0.1).abs() < 1e-6);
        assert_eq!(outputs[1], 1.0);
        assert_eq!(outputs[2], 0.3);
        assert_eq!(outputs[3], 0.4);
    }

    #[test]
    fn compose_pwm_outputs_clamps_servo_gpio_and_motor_ranges() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_CHANNEL_OUTPUT_MASK, ParamValue::Int(-1));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let output_types = [
            MixerOutputType::Servo,
            MixerOutputType::Servo,
            MixerOutputType::Gpio,
            MixerOutputType::Gpio,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];

        let outputs: [f64; PWM_OUTPUT_CHANNELS] = compose_pwm_outputs(
            &[-2.0, 2.0, -0.1, 0.1, -0.5, 1.5],
            &output_types,
            None,
            &state,
            &params,
        );

        assert_eq!(outputs[0], 0.0);
        assert_eq!(outputs[1], 1.0);
        assert_eq!(outputs[2], 0.0);
        assert_eq!(outputs[3], 1.0);
        assert!((outputs[4] - 0.1).abs() < 1e-6);
        assert_eq!(outputs[5], 1.0);
    }
}
