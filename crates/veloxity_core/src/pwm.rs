use crate::{board::BoardIo, math::FlightFloat, mixer::MixerOutputType};

pub mod output_sync;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PwmError {
    ChannelOutOfRange,
    GenericError,
    InvalidRate,
    UnsupportedProtocol,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PwmOutputProtocol {
    StandardPwm,
    Dshot,
}

pub const STANDARD_PWM_DEFAULT_RATE_HZ: f32 = 50.0;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DshotCommand {
    pub throttle: u16,
    pub telemetry: bool,
}

impl DshotCommand {
    pub const STOP: u16 = 0;
    pub const MIN_THROTTLE: u16 = 48;
    pub const MAX_THROTTLE: u16 = 2047;
    pub const FRAME_BITS: usize = 16;

    pub const fn stop() -> Self {
        Self {
            throttle: Self::STOP,
            telemetry: false,
        }
    }

    pub fn from_normalized<R: FlightFloat>(value: R) -> Self {
        let normalized = value.clamp(
            <R as FlightFloat>::from_f32(0.0),
            <R as FlightFloat>::from_f32(1.0),
        );
        let span = <R as FlightFloat>::from_u64((Self::MAX_THROTTLE - Self::MIN_THROTTLE) as u64);
        Self {
            throttle: (normalized * span + <R as FlightFloat>::from_u64(Self::MIN_THROTTLE as u64))
                .to_f32_lossy() as u16,
            telemetry: false,
        }
    }

    pub fn frame(self) -> u16 {
        let value = ((self.throttle & 0x07ff) << 1) | self.telemetry as u16;
        let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x000f;
        (value << 4) | crc
    }

    pub fn bit_is_high(frame: u16, bit_index: usize) -> bool {
        let mask = 0x8000u16 >> bit_index.min(Self::FRAME_BITS - 1);
        frame & mask != 0
    }
}

pub fn output_protocol_for_rate<R: FlightFloat>(rate_hz: R) -> Result<PwmOutputProtocol, PwmError> {
    if !rate_hz.is_finite() || rate_hz < <R as FlightFloat>::from_f32(0.0) {
        return Err(PwmError::InvalidRate);
    }

    if rate_hz <= <R as FlightFloat>::from_f32(490.0) {
        Ok(PwmOutputProtocol::StandardPwm)
    } else if rate_hz >= <R as FlightFloat>::from_f32(150_000.0)
        && rate_hz <= <R as FlightFloat>::from_f32(1_200_000.0)
    {
        Ok(PwmOutputProtocol::Dshot)
    } else {
        Err(PwmError::InvalidRate)
    }
}

pub fn effective_output_rate_hz<R: FlightFloat>(rate_hz: R) -> Result<R, PwmError> {
    let protocol = output_protocol_for_rate(rate_hz)?;
    match protocol {
        PwmOutputProtocol::StandardPwm if rate_hz == <R as FlightFloat>::from_f32(0.0) => {
            Ok(<R as FlightFloat>::from_f32(STANDARD_PWM_DEFAULT_RATE_HZ))
        }
        _ => Ok(rate_hz),
    }
}

pub trait PwmDriver<R: FlightFloat> {
    fn len(&self) -> usize;
    fn is_enabled(&self) -> bool;

    fn enable(&mut self, channel: usize) -> Result<(), PwmError>;
    fn disable(&mut self, channel: usize) -> Result<(), PwmError>;

    fn enable_all(&mut self) -> Result<(), PwmError>;
    fn disable_all(&mut self);

    /// Sets the duty cycle for a specific channel.
    ///
    /// # Arguments
    /// * `channel` - The output channel index (0-based).
    /// * `duty`    - The desired duty cycle, typically represented as a u16 value.
    ///             The exact interpretation (e.g., 0-ARR, 0-u16::MAX) depends
    ///             on the implementation. For simulation, we'll map 0-u16::MAX
    ///             to the simulator's expected range (e.g., 1000-2000us).
    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError>;

    /// Sends the current state of all PWM channels to the output/simulator.
    /// This should be called once per control loop after all individual
    /// `set_duty_cycle` calls for that loop iteration are complete.
    ///
    /// # Arguments
    /// * `now_us` - The current flight controller time in microseconds for timestamping.
    fn flush<B: BoardIo>(&mut self, board: &mut B);

    /// Configures default output update rates in Hz for mixer-owned channels.
    ///
    /// Boards that cannot change rates at runtime may ignore this hook, but
    /// the core still propagates the mixer-owned rate resource explicitly.
    fn configure_output_rates(&mut self, _rates_hz: &[R]) -> Result<(), PwmError> {
        Ok(())
    }

    fn output_protocol(&self, _channel: usize) -> Result<PwmOutputProtocol, PwmError> {
        Ok(PwmOutputProtocol::StandardPwm)
    }

    // actually loops over the channels (up to self.len()) and sends pwm commands via set_duty_cycle
    fn send_commands<B: BoardIo>(&mut self, board: &mut B, commands: &[R]) -> Result<(), PwmError>;

    fn send_disarmed_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        output_types: &[MixerOutputType],
    ) -> Result<(), PwmError> {
        let mut commands =
            [<R as FlightFloat>::from_f32(0.0); crate::pwm::output_sync::PWM_OUTPUT_CHANNELS];
        for (channel, command) in commands.iter_mut().enumerate().take(self.len()) {
            let output_type = output_types
                .get(channel)
                .copied()
                .unwrap_or(MixerOutputType::Aux);
            *command = safe_disarmed_command(output_type);
        }
        self.send_commands(board, &commands)
    }
}

/// Normalized command an output holds while disarmed or masked off.
///
/// Servos are held at mid-range so control surfaces stay centered.
/// All other output types are held low.
pub fn safe_disarmed_command<R: FlightFloat>(output_type: MixerOutputType) -> R {
    match output_type {
        MixerOutputType::Motor | MixerOutputType::Gpio | MixerOutputType::Aux => {
            <R as FlightFloat>::from_f32(0.0)
        }
        MixerOutputType::Servo => <R as FlightFloat>::from_f32(0.5),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifies_rosflight_pwm_and_dshot_rate_ranges() {
        assert_eq!(
            output_protocol_for_rate(50.0),
            Ok(PwmOutputProtocol::StandardPwm)
        );
        assert_eq!(
            output_protocol_for_rate(0.0),
            Ok(PwmOutputProtocol::StandardPwm)
        );
        assert_eq!(
            output_protocol_for_rate(490.0),
            Ok(PwmOutputProtocol::StandardPwm)
        );
        assert_eq!(
            output_protocol_for_rate(300_000.0),
            Ok(PwmOutputProtocol::Dshot)
        );
        assert_eq!(
            output_protocol_for_rate(10_000.0),
            Err(PwmError::InvalidRate)
        );
    }

    #[test]
    fn zero_rate_uses_standard_pwm_default_rate() {
        assert_eq!(effective_output_rate_hz(0.0), Ok(50.0));
        assert_eq!(effective_output_rate_hz(490.0), Ok(490.0));
        assert_eq!(effective_output_rate_hz(300_000.0), Ok(300_000.0));
    }

    #[test]
    fn dshot_frame_matches_rosflight_checksum_formula() {
        let command = DshotCommand {
            throttle: 48,
            telemetry: false,
        };
        let value = 48u16 << 1;
        let expected_crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x000f;
        assert_eq!(command.frame(), (value << 4) | expected_crc);
    }

    #[test]
    fn dshot_stop_frame_uses_zero_throttle() {
        assert_eq!(DshotCommand::stop().throttle, 0);
        assert_eq!(DshotCommand::stop().frame(), 0);
    }
}
