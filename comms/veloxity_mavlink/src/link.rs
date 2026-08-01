// ******************************************************************************
// * File     : comms/veloxity_mavlink/src/link.rs
// * Date     : June 28, 2026
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************

use crate::generated::dialects::rosflight::{Rosflight, messages as mav_messages};
use crate::parser;
use mavio::Frame;
use mavio::prelude::*;
use veloxity_core::board::{self, SerialTxPriority};
use veloxity_core::comm::interface::CommInterface;
use veloxity_core::comm::messages::{
    Messages, Store,
    messages::{self as core_messages, DownlinkMessage},
};

static RX_BUFF_SIZE: usize = 2048;
const MAV_COMP_ID_ROSFLIGHT_FIRMWARE: u8 = 250;
const MAVLINK_V1_MESSAGE_SIZE: usize = 263;

// Serial-delay (RTT) timing test support. Mirrors the C firmware's
// `jacob/timing-tests` branch, which echoes every received OFFBOARD_CONTROL frame
// straight back over the same link before decoding it, so a companion computer can
// time serial round trips.
const OFFBOARD_CONTROL_MSG_ID: u8 = 180; // rosflight.xml message id="180" name="OFFBOARD_CONTROL"
const MAVLINK_V1_MSGID_OFFSET: usize = 5;

/// Retransmit `frame` verbatim if it is an OFFBOARD_CONTROL message.
///
/// The raw received bytes are sent back unchanged (sender's own seq/sysid/compid
/// preserved), matching the C firmware's `send_message(*msg)` echo.
fn echo_if_offboard_control<B: board::BoardIo>(board: &mut B, frame: &parser::CompleteFrame) {
    if frame.len > MAVLINK_V1_MSGID_OFFSET
        && frame.data[MAVLINK_V1_MSGID_OFFSET] == OFFBOARD_CONTROL_MSG_ID
    {
        board.serial_tx_write_priority(&frame.data[..frame.len], SerialTxPriority::CRITICAL);
    }
}

pub struct MavlinkInterface {
    pub component_id: u8,
    sequence: u8,
    mav_parser: parser::MavlinkParser,
}

pub struct MavlinkFrameEncoder {
    pub component_id: u8,
    sequence: u8,
}

impl MavlinkFrameEncoder {
    pub fn new() -> Self {
        Self {
            component_id: MAV_COMP_ID_ROSFLIGHT_FIRMWARE,
            sequence: 0,
        }
    }

    fn frame_builder<T: Message>(&mut self, system_id: u8, msg: T) -> mavio::Result<Frame<V1>> {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        self.frame_builder_with_sequence(system_id, sequence, msg)
    }

    fn frame_builder_with_sequence<T: Message>(
        &self,
        system_id: u8,
        sequence: u8,
        msg: T,
    ) -> mavio::Result<Frame<V1>> {
        let frame = Frame::builder()
            .version(V1)
            .system_id(system_id)
            .component_id(self.component_id)
            .sequence(sequence)
            .message(&msg)?
            .build();

        Ok(frame)
    }

    fn encode_message<T: Message>(
        &mut self,
        system_id: u8,
        msg: T,
        out: &mut [u8],
    ) -> Option<usize> {
        let frame = self.frame_builder(system_id, msg).ok()?;
        self.encode_frame(frame, out)
    }

    fn encode_message_with_sequence<T: Message>(
        &self,
        system_id: u8,
        sequence: u8,
        msg: T,
        out: &mut [u8],
    ) -> Option<usize> {
        let frame = self
            .frame_builder_with_sequence(system_id, sequence, msg)
            .ok()?;
        self.encode_frame(frame, out)
    }

    fn encode_frame(&self, frame: Frame<V1>, out: &mut [u8]) -> Option<usize> {
        let mut pos = 0;
        let header = frame.header();
        let payload = frame.payload().bytes();
        let crc = frame.checksum();
        let frame_len = payload.len() + 8;

        if frame_len > out.len() {
            return None;
        }

        out[pos] = 0xFE;
        pos += 1;
        out[pos] = payload.len() as u8;
        pos += 1;
        out[pos] = header.sequence();
        pos += 1;
        out[pos] = header.system_id();
        pos += 1;
        out[pos] = header.component_id();
        pos += 1;
        out[pos] = header.message_id() as u8;
        pos += 1;
        out[pos..pos + payload.len()].copy_from_slice(payload);
        pos += payload.len();
        out[pos..pos + 2].copy_from_slice(&crc.to_le_bytes());
        pos += 2;

        Some(pos)
    }

    pub fn encode_downlink(
        &mut self,
        system_id: u8,
        msg: DownlinkMessage,
        out: &mut [u8],
    ) -> Option<usize> {
        match msg {
            DownlinkMessage::Heartbeat(msg) => {
                self.encode_message(system_id, mav_messages::Heartbeat::from(msg), out)
            }
            DownlinkMessage::ParamValue(msg) => {
                self.encode_message(system_id, mav_messages::ParamValue::from(msg), out)
            }
            DownlinkMessage::Status(msg) => {
                self.encode_message(system_id, mav_messages::RosflightStatus::from(msg), out)
            }
            DownlinkMessage::Timesync(msg) => {
                self.encode_message(system_id, mav_messages::Timesync::from(msg), out)
            }
            DownlinkMessage::Version(msg) => {
                self.encode_message(system_id, mav_messages::RosflightVersion::from(msg), out)
            }
            DownlinkMessage::OutputRaw(msg) => {
                self.encode_message(system_id, mav_messages::RosflightOutputRaw::from(msg), out)
            }
            DownlinkMessage::Attitude(msg) => {
                self.encode_message(system_id, mav_messages::AttitudeQuaternion::from(msg), out)
            }
            DownlinkMessage::Baro(msg) => {
                self.encode_message(system_id, mav_messages::SmallBaro::from(msg), out)
            }
            DownlinkMessage::DiffPressure(msg) => {
                self.encode_message(system_id, mav_messages::DiffPressure::from(msg), out)
            }
            DownlinkMessage::Imu(msg) => {
                self.encode_message(system_id, mav_messages::SmallImu::from(msg), out)
            }
            DownlinkMessage::Mag(msg) => {
                self.encode_message(system_id, mav_messages::SmallMag::from(msg), out)
            }
            DownlinkMessage::RcRaw(msg) | DownlinkMessage::RcChannels(msg) => {
                self.encode_message(system_id, mav_messages::RcChannels::from(msg), out)
            }
            DownlinkMessage::Range(msg) => {
                self.encode_message(system_id, mav_messages::SmallRange::from(msg), out)
            }
            DownlinkMessage::Gnss(msg) => {
                self.encode_message(system_id, mav_messages::RosflightGnss::from(msg), out)
            }
            DownlinkMessage::CmdAck(msg) => {
                self.encode_message(system_id, mav_messages::RosflightCmdAck::from(msg), out)
            }
            DownlinkMessage::BatteryStatus(msg) => self.encode_message(
                system_id,
                mav_messages::RosflightBatteryStatus::from(msg),
                out,
            ),
            DownlinkMessage::Statustext(msg) => {
                self.encode_message(system_id, mav_messages::Statustext::from(msg), out)
            }
            DownlinkMessage::HardError(msg) => {
                self.encode_message(system_id, mav_messages::RosflightHardError::from(msg), out)
            }
        }
    }

    pub fn encode_downlink_with_sequence(
        &self,
        system_id: u8,
        sequence: u8,
        msg: DownlinkMessage,
        out: &mut [u8],
    ) -> Option<usize> {
        match msg {
            DownlinkMessage::Heartbeat(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::Heartbeat::from(msg),
                out,
            ),
            DownlinkMessage::ParamValue(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::ParamValue::from(msg),
                out,
            ),
            DownlinkMessage::Status(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightStatus::from(msg),
                out,
            ),
            DownlinkMessage::Timesync(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::Timesync::from(msg),
                out,
            ),
            DownlinkMessage::Version(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightVersion::from(msg),
                out,
            ),
            DownlinkMessage::OutputRaw(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightOutputRaw::from(msg),
                out,
            ),
            DownlinkMessage::Attitude(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::AttitudeQuaternion::from(msg),
                out,
            ),
            DownlinkMessage::Baro(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::SmallBaro::from(msg),
                out,
            ),
            DownlinkMessage::DiffPressure(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::DiffPressure::from(msg),
                out,
            ),
            DownlinkMessage::Imu(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::SmallImu::from(msg),
                out,
            ),
            DownlinkMessage::Mag(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::SmallMag::from(msg),
                out,
            ),
            DownlinkMessage::RcRaw(msg) | DownlinkMessage::RcChannels(msg) => self
                .encode_message_with_sequence(
                    system_id,
                    sequence,
                    mav_messages::RcChannels::from(msg),
                    out,
                ),
            DownlinkMessage::Range(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::SmallRange::from(msg),
                out,
            ),
            DownlinkMessage::Gnss(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightGnss::from(msg),
                out,
            ),
            DownlinkMessage::CmdAck(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightCmdAck::from(msg),
                out,
            ),
            DownlinkMessage::BatteryStatus(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightBatteryStatus::from(msg),
                out,
            ),
            DownlinkMessage::Statustext(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::Statustext::from(msg),
                out,
            ),
            DownlinkMessage::HardError(msg) => self.encode_message_with_sequence(
                system_id,
                sequence,
                mav_messages::RosflightHardError::from(msg),
                out,
            ),
        }
    }
}

impl MavlinkInterface {
    pub fn new() -> Self {
        Self {
            component_id: MAV_COMP_ID_ROSFLIGHT_FIRMWARE, // In latest rosflight_firmware this is hardcoded to 250
            sequence: 0,
            mav_parser: parser::MavlinkParser::new(),
        }
    }

    fn frame_builder_with_sequence<T: Message>(
        &self,
        system_id: u8,
        sequence: u8,
        msg: T,
    ) -> mavio::Result<Frame<V1>> {
        let frame = Frame::builder()
            .version(V1)
            .system_id(system_id)
            .component_id(self.component_id)
            .sequence(sequence)
            .message(&msg)?
            .build();

        Ok(frame)
    }

    fn next_sequence(&mut self) -> u8 {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        sequence
    }

    fn send_downlink_or_message<B: board::BoardIo, T: Message>(
        &mut self,
        board: &mut B,
        system_id: u8,
        downlink: DownlinkMessage,
        msg: impl FnOnce() -> T,
        priority: SerialTxPriority,
    ) {
        match board.serial_tx_enqueue_downlink(system_id, downlink, priority) {
            Some(Ok(n)) if n > 0 => return,
            Some(_) => return,
            None => {}
        }

        let sequence = self.next_sequence();
        let frame = match self.frame_builder_with_sequence(system_id, sequence, msg()) {
            Ok(f) => f,
            Err(_) => {
                return;
            }
        };

        // MAVLink V1 specification shows messages max size of 263 bytes
        let mut buf = [0u8; MAVLINK_V1_MESSAGE_SIZE];

        if frame.body_length() > buf.len() {
            return;
        }

        let mut pos = 0;
        let header = frame.header();
        let payload = frame.payload().bytes();
        let crc = frame.checksum();

        // MAVLink v1 wire format
        buf[pos] = 0xFE;
        pos += 1; // Start marker
        buf[pos] = payload.len() as u8;
        pos += 1; // Payload length
        buf[pos] = header.sequence();
        pos += 1; // Sequence
        buf[pos] = header.system_id();
        pos += 1; // System ID
        buf[pos] = header.component_id();
        pos += 1; // Component ID
        buf[pos] = header.message_id() as u8;
        pos += 1; // Message ID (low byte)

        // Copy payload
        buf[pos..pos + payload.len()].copy_from_slice(payload);
        pos += payload.len();

        // CRC (little-endian)
        buf[pos..pos + 2].copy_from_slice(&crc.to_le_bytes());
        pos += 2;

        board.serial_tx_write_priority(&buf[..pos], priority);
    }

    fn process_rosflight_message(&mut self, message: Rosflight, msgs: &mut Messages) {
        match message {
            Rosflight::ExternalAttitude(es) => {
                msgs.store(core_messages::ExternalAttitudeMsg::from(es))
            }
            Rosflight::Timesync(ts) => msgs.store(core_messages::TimesyncMsg::from(ts)),
            Rosflight::RosflightCmd(cmd) => msgs.store(core_messages::RosflightCmdMsg::from(cmd)),
            Rosflight::RosflightAuxCmd(aux_cmd) => {
                msgs.store(core_messages::RosflightAuxCmdMsg::from(aux_cmd))
            }
            Rosflight::OffboardControl(oc) => {
                msgs.store(core_messages::OffboardControlMsg::from(oc))
            }
            Rosflight::ParamRequestRead(pr) => {
                msgs.store(core_messages::ParamRequestReadMsg::from(pr))
            }
            Rosflight::ParamSet(ps) => msgs.store(core_messages::ParamSetMsg::from(ps)),
            Rosflight::ParamRequestList(pl) => {
                msgs.store(core_messages::ParamRequestListMsg::from(pl))
            }
            Rosflight::Heartbeat(hb) => msgs.store(core_messages::HeartbeatMsg::from(hb)),
            _ => {}
        }
    }
}

impl<B: board::BoardIo> CommInterface<B> for MavlinkInterface {
    fn handle_incoming_messages(&mut self, board: &mut B, msgs: &mut Messages) {
        while let Some(frame) = board.serial_rx_frame_read() {
            match frame {
                Ok(frame) => {
                    let mut mavlink_frame = parser::CompleteFrame {
                        data: [0; 280],
                        len: frame.len.min(280),
                    };
                    mavlink_frame.data[..mavlink_frame.len]
                        .copy_from_slice(&frame.data[..mavlink_frame.len]);
                    echo_if_offboard_control(board, &mavlink_frame);
                    if let Some(message) = parser::process_mavlink_frame(mavlink_frame) {
                        self.process_rosflight_message(message, msgs);
                    }
                }
                Err(_) => break,
            }
        }

        let mut buf = [0u8; RX_BUFF_SIZE];
        match board.serial_rx_read(&mut buf) {
            Some(Ok(n)) => {
                for i in 0..n {
                    if let Some(frame) = self.mav_parser.feed_byte(buf[i]) {
                        echo_if_offboard_control(board, &frame);
                        if let Some(message) = parser::process_mavlink_frame(frame) {
                            self.process_rosflight_message(message, msgs);
                        }
                    }
                }
            }
            Some(Err(_)) => {}
            None => {}
        }
    }

    fn send_status(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightStatusMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Status(msg),
            || mav_messages::RosflightStatus::from(msg),
            SerialTxPriority::CRITICAL,
        );
    }
    fn send_timesync(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::TimesyncMsg,
    ) -> bool {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Timesync(msg),
            || mav_messages::Timesync::from(msg),
            SerialTxPriority::CRITICAL,
        );
        return true;
    }
    fn send_named_value(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::ParamValueMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::ParamValue(msg),
            || mav_messages::ParamValue::from(msg),
            SerialTxPriority::CRITICAL,
        );
    }
    fn send_heartbeat(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::HeartbeatMsg,
    ) -> bool {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Heartbeat(msg),
            || mav_messages::Heartbeat::from(msg),
            SerialTxPriority::CRITICAL,
        );
        return true;
    }
    fn send_version(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightVersionMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Version(msg),
            || mav_messages::RosflightVersion::from(msg),
            SerialTxPriority::CRITICAL,
        );
    }
    fn send_diff_pressure(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::DiffPressureMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::DiffPressure(msg),
            || mav_messages::DiffPressure::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_baro(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallBaroMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Baro(msg),
            || mav_messages::SmallBaro::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_imu(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallImuMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Imu(msg),
            || mav_messages::SmallImu::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_attitude(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::AttitudeQuaternionMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Attitude(msg),
            || mav_messages::AttitudeQuaternion::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_output_raw(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightOutputRawMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::OutputRaw(msg),
            || mav_messages::RosflightOutputRaw::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_rc_raw(&mut self, board: &mut B, system_id: u8, msg: core_messages::RcChannelsMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::RcRaw(msg),
            || mav_messages::RcChannels::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_range(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallRangeMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Range(msg),
            || mav_messages::SmallRange::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_mag(&mut self, board: &mut B, system_id: u8, msg: core_messages::SmallMagMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Mag(msg),
            || mav_messages::SmallMag::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_gnss(&mut self, board: &mut B, system_id: u8, msg: core_messages::RosflightGnssMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Gnss(msg),
            || mav_messages::RosflightGnss::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_cmd_ack(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightCmdAckMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::CmdAck(msg),
            || mav_messages::RosflightCmdAck::from(msg),
            SerialTxPriority::CRITICAL,
        );
    }
    fn send_rc_channels(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RcChannelsMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::RcChannels(msg),
            || mav_messages::RcChannels::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_battery_status(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::BatteryStatusMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::BatteryStatus(msg),
            || mav_messages::RosflightBatteryStatus::from(msg),
            SerialTxPriority::REPLACEABLE_TELEMETRY,
        );
    }
    fn send_statustext(&mut self, board: &mut B, system_id: u8, msg: core_messages::StatustextMsg) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::Statustext(msg),
            || mav_messages::Statustext::from(msg),
            SerialTxPriority::CRITICAL,
        );
    }
    fn send_hard_error(
        &mut self,
        board: &mut B,
        system_id: u8,
        msg: core_messages::RosflightHardErrorMsg,
    ) {
        self.send_downlink_or_message(
            board,
            system_id,
            DownlinkMessage::HardError(msg),
            || mav_messages::RosflightHardError::from(msg),
            SerialTxPriority::CRITICAL,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generated::dialects::rosflight::{enums as mav_enums, messages as mav_messages};
    use veloxity_core::{
        board::{BoardIo, SerialRxFrame, SerialTxPriority},
        comm::interface::CommInterface,
        errors,
        params::Params,
        sensors::SensorBus,
    };

    #[derive(Default)]
    struct FramedBoard {
        frame: Option<SerialRxFrame>,
        byte_reads: usize,
    }

    struct TxCaptureBoard {
        bytes: [u8; 280],
        len: usize,
    }

    /// Injects a framed RX message *and* captures TX bytes, so the OFFBOARD_CONTROL
    /// echo added for the serial-delay timing test can be observed.
    struct EchoCaptureBoard {
        frame: Option<SerialRxFrame>,
        tx_bytes: [u8; 280],
        tx_len: usize,
        tx_writes: usize,
        last_priority: SerialTxPriority,
    }

    impl Default for EchoCaptureBoard {
        fn default() -> Self {
            Self {
                frame: None,
                tx_bytes: [0; 280],
                tx_len: 0,
                tx_writes: 0,
                last_priority: SerialTxPriority::default(),
            }
        }
    }

    struct DownlinkCaptureBoard {
        accepted_len: usize,
        enqueued: usize,
        bytes_written: usize,
        last_priority: SerialTxPriority,
    }

    impl Default for DownlinkCaptureBoard {
        fn default() -> Self {
            Self {
                accepted_len: 1,
                enqueued: 0,
                bytes_written: 0,
                last_priority: SerialTxPriority::default(),
            }
        }
    }

    impl Default for TxCaptureBoard {
        fn default() -> Self {
            Self {
                bytes: [0; 280],
                len: 0,
            }
        }
    }

    impl BoardIo for FramedBoard {
        fn update_sensor_bus<R: veloxity_core::math::FlightFloat>(
            &mut self,
            sensors: &mut SensorBus<R>,
        ) {
            sensors.clear();
        }

        fn serial_rx_read(
            &mut self,
            _buf: &mut [u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            self.byte_reads += 1;
            Some(Ok(0))
        }

        fn serial_rx_frame_read(
            &mut self,
        ) -> Option<core::result::Result<SerialRxFrame, errors::TelemError>> {
            self.frame.take().map(Ok)
        }

        fn serial_tx_write(
            &mut self,
            bytes: &[u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            Some(Ok(bytes.len()))
        }

        fn clock_millis(&self) -> u32 {
            0
        }

        fn clock_micros(&self) -> u64 {
            0
        }
    }

    impl BoardIo for TxCaptureBoard {
        fn update_sensor_bus<R: veloxity_core::math::FlightFloat>(
            &mut self,
            sensors: &mut SensorBus<R>,
        ) {
            sensors.clear();
        }

        fn serial_rx_read(
            &mut self,
            _buf: &mut [u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            Some(Ok(0))
        }

        fn serial_tx_write(
            &mut self,
            bytes: &[u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            self.bytes[..bytes.len()].copy_from_slice(bytes);
            self.len = bytes.len();
            Some(Ok(bytes.len()))
        }

        fn clock_millis(&self) -> u32 {
            0
        }

        fn clock_micros(&self) -> u64 {
            0
        }
    }

    impl BoardIo for EchoCaptureBoard {
        fn update_sensor_bus<R: veloxity_core::math::FlightFloat>(
            &mut self,
            sensors: &mut SensorBus<R>,
        ) {
            sensors.clear();
        }

        fn serial_rx_read(
            &mut self,
            _buf: &mut [u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            Some(Ok(0))
        }

        fn serial_rx_frame_read(
            &mut self,
        ) -> Option<core::result::Result<SerialRxFrame, errors::TelemError>> {
            self.frame.take().map(Ok)
        }

        fn serial_tx_write(
            &mut self,
            bytes: &[u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            self.tx_bytes[..bytes.len()].copy_from_slice(bytes);
            self.tx_len = bytes.len();
            self.tx_writes += 1;
            Some(Ok(bytes.len()))
        }

        fn serial_tx_write_priority(
            &mut self,
            bytes: &[u8],
            priority: SerialTxPriority,
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            self.last_priority = priority;
            self.serial_tx_write(bytes)
        }

        fn clock_millis(&self) -> u32 {
            0
        }

        fn clock_micros(&self) -> u64 {
            0
        }
    }

    impl BoardIo for DownlinkCaptureBoard {
        fn update_sensor_bus<R: veloxity_core::math::FlightFloat>(
            &mut self,
            sensors: &mut SensorBus<R>,
        ) {
            sensors.clear();
        }

        fn serial_rx_read(
            &mut self,
            _buf: &mut [u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            Some(Ok(0))
        }

        fn serial_tx_write(
            &mut self,
            bytes: &[u8],
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            self.bytes_written += bytes.len();
            Some(Ok(bytes.len()))
        }

        fn serial_tx_enqueue_downlink(
            &mut self,
            _system_id: u8,
            _msg: DownlinkMessage,
            priority: SerialTxPriority,
        ) -> Option<core::result::Result<usize, errors::TelemError>> {
            self.enqueued += 1;
            self.last_priority = priority;
            Some(Ok(self.accepted_len))
        }

        fn clock_millis(&self) -> u32 {
            0
        }

        fn clock_micros(&self) -> u64 {
            0
        }
    }

    fn offboard_control_serial_frame() -> SerialRxFrame {
        let frame = Frame::builder()
            .version(V1)
            .sequence(7)
            .system_id(1)
            .component_id(1)
            .message(&mav_messages::OffboardControl {
                mode: mav_enums::OffboardControlMode::ModeRollPitchYawrateThrottle,
                ignore: mav_enums::OffboardControlIgnore::IgnoreNone,
                u: [0.0, 0.0, 0.85, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0],
            })
            .unwrap()
            .build();

        let mut out = SerialRxFrame::default();
        let mut pos = 0;
        let header = frame.header();
        let payload = frame.payload().bytes();
        let crc = frame.checksum();

        out.data[pos] = 0xFE;
        pos += 1;
        out.data[pos] = payload.len() as u8;
        pos += 1;
        out.data[pos] = header.sequence();
        pos += 1;
        out.data[pos] = header.system_id();
        pos += 1;
        out.data[pos] = header.component_id();
        pos += 1;
        out.data[pos] = header.message_id() as u8;
        pos += 1;
        out.data[pos..pos + payload.len()].copy_from_slice(payload);
        pos += payload.len();
        out.data[pos..pos + 2].copy_from_slice(&crc.to_le_bytes());
        pos += 2;
        out.len = pos;
        out
    }

    #[test]
    fn framed_rx_path_stores_rosflight_message() {
        let mut board = FramedBoard {
            frame: Some(offboard_control_serial_frame()),
            byte_reads: 0,
        };
        let mut link = MavlinkInterface::new();
        let mut messages = Messages::default();

        link.handle_incoming_messages(&mut board, &mut messages);

        let msg = messages.offboard_control.expect("offboard message");
        assert_eq!(msg.fz, 0.85);
        assert_eq!(board.byte_reads, 1);
    }

    #[test]
    fn offboard_control_frame_is_echoed_verbatim() {
        let frame = offboard_control_serial_frame();
        let mut board = EchoCaptureBoard {
            frame: Some(frame),
            ..Default::default()
        };
        let mut link = MavlinkInterface::new();
        let mut messages = Messages::default();

        link.handle_incoming_messages(&mut board, &mut messages);

        assert_eq!(board.tx_writes, 1);
        assert_eq!(board.tx_len, frame.len);
        assert_eq!(&board.tx_bytes[..board.tx_len], &frame.data[..frame.len]);
        assert_eq!(board.last_priority, SerialTxPriority::CRITICAL);
        // The echo must not short-circuit normal decode + store.
        let msg = messages.offboard_control.expect("offboard message");
        assert_eq!(msg.fz, 0.85);
    }

    #[test]
    fn invalid_framed_rx_does_not_store_message() {
        let mut frame = offboard_control_serial_frame();
        frame.data[frame.len - 1] ^= 0x55;
        let mut board = FramedBoard {
            frame: Some(frame),
            byte_reads: 0,
        };
        let mut link = MavlinkInterface::new();
        let mut messages = Messages::default();

        link.handle_incoming_messages(&mut board, &mut messages);

        assert!(messages.offboard_control.is_none());
    }

    fn assert_encoded_matches_inline(
        downlink: DownlinkMessage,
        send_inline: impl FnOnce(&mut MavlinkInterface, &mut TxCaptureBoard),
    ) {
        let mut link = MavlinkInterface::new();
        let mut board = TxCaptureBoard::default();
        send_inline(&mut link, &mut board);

        let mut encoder = MavlinkFrameEncoder::new();
        let mut encoded = [0_u8; 280];
        let len = encoder
            .encode_downlink(1, downlink, &mut encoded)
            .expect("encoded downlink");

        assert_eq!(&encoded[..len], &board.bytes[..board.len]);
    }

    #[test]
    fn core1_downlink_encoder_matches_inline_imu_frame() {
        let msg = core_messages::SmallImuMsg {
            time_boot_us: 123_456_789,
            xacc: -0.1,
            yacc: 0.2,
            zacc: 9.81,
            xgyro: 0.01,
            ygyro: -0.02,
            zgyro: 0.03,
            temperature: 23.5,
        };

        assert_encoded_matches_inline(DownlinkMessage::Imu(msg), |link, board| {
            link.send_imu(board, 1, msg)
        });
    }

    #[test]
    fn core1_downlink_encoder_matches_inline_rc_frame() {
        let mut channels = [0_u16; core_messages::RC_PACKET_CHANNELS];
        channels[..8].copy_from_slice(&[1500, 1501, 1100, 1499, 1000, 1000, 1000, 1000]);
        let msg = core_messages::RcChannelsMsg {
            time_boot_ms: 42_000,
            chancount: 8,
            channels,
            rssi: 99,
        };

        assert_encoded_matches_inline(DownlinkMessage::RcRaw(msg), |link, board| {
            link.send_rc_raw(board, 1, msg)
        });
    }

    #[test]
    fn core1_downlink_encoder_matches_inline_attitude_frame() {
        let msg = core_messages::AttitudeQuaternionMsg {
            time_boot_ms: 42,
            q1: 1.0,
            q2: 0.0,
            q3: 0.1,
            q4: -0.1,
            rollspeed: 0.01,
            pitchspeed: -0.02,
            yawspeed: 0.03,
        };

        assert_encoded_matches_inline(DownlinkMessage::Attitude(msg), |link, board| {
            link.send_attitude(board, 1, msg)
        });
    }

    #[test]
    fn core1_downlink_encoder_matches_inline_output_raw_frame() {
        let msg = core_messages::RosflightOutputRawMsg {
            stamp: 987_654,
            values: [0.25; 14],
        };

        assert_encoded_matches_inline(DownlinkMessage::OutputRaw(msg), |link, board| {
            link.send_output_raw(board, 1, msg)
        });
    }

    #[test]
    fn offload_downlink_path_does_not_encode_or_write_on_core0() {
        let msg = core_messages::SmallImuMsg {
            time_boot_us: 123_456_789,
            xacc: -0.1,
            yacc: 0.2,
            zacc: 9.81,
            xgyro: 0.01,
            ygyro: -0.02,
            zgyro: 0.03,
            temperature: 23.5,
        };
        let mut link = MavlinkInterface::new();
        let mut board = DownlinkCaptureBoard::default();

        link.send_imu(&mut board, 1, msg);

        assert_eq!(board.enqueued, 1);
        assert_eq!(board.bytes_written, 0);
        assert_eq!(link.sequence, 0);
        assert_eq!(board.last_priority, SerialTxPriority::REPLACEABLE_TELEMETRY);
    }

    #[test]
    fn full_replaceable_downlink_queue_drops_without_core0_encode() {
        let msg = core_messages::SmallImuMsg {
            time_boot_us: 123_456_789,
            xacc: -0.1,
            yacc: 0.2,
            zacc: 9.81,
            xgyro: 0.01,
            ygyro: -0.02,
            zgyro: 0.03,
            temperature: 23.5,
        };
        let mut link = MavlinkInterface::new();
        let mut board = DownlinkCaptureBoard {
            accepted_len: 0,
            ..Default::default()
        };

        link.send_imu(&mut board, 1, msg);

        assert_eq!(board.enqueued, 1);
        assert_eq!(board.bytes_written, 0);
        assert_eq!(link.sequence, 0);
    }

    #[test]
    fn full_critical_downlink_queue_drops_without_core0_encode() {
        let msg = core_messages::HeartbeatMsg {
            type_: 2,
            autopilot: 0,
            base_mode: 1,
            custom_mode: 0,
            system_status: 4,
            mavlink_version: 3,
        };
        let mut link = MavlinkInterface::new();
        let mut board = DownlinkCaptureBoard {
            accepted_len: 0,
            ..Default::default()
        };

        link.send_heartbeat(&mut board, 1, msg);

        assert_eq!(board.enqueued, 1);
        assert_eq!(board.bytes_written, 0);
        assert_eq!(link.sequence, 0);
    }

    #[allow(dead_code)]
    fn _params_type_is_available(_: Params) {}
}
