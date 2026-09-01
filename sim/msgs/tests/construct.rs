//! Smoke test: construct every generated `rosflight_msgs` message and
//! service request/response type, plus a sample of bundled `hiroz-msgs`
//! types, and check a couple of field-shape defaults.
//!
//! This is deliberately minimal — construction and field shape only, no ROS
//! runtime (no publishing, no zenoh session).

use veloxity_ros_msgs::ros::rosflight_msgs::{
    Airspeed, Barometer, BatteryStatus, Command, GNSS, OutputRaw, ParamFileRequest,
    ParamFileResponse, ParamGetRequest, ParamGetResponse, ParamSetRequest, ParamSetResponse,
    PwmOutput, RCRaw, SimState, Status,
};
use veloxity_ros_msgs::sensor_msgs::{Imu, MagneticField, Range, Temperature};
use veloxity_ros_msgs::std_srvs::{TriggerRequest, TriggerResponse};

#[test]
fn construct_rosflight_messages() {
    let _ = PwmOutput::default();
    let _ = RCRaw::default();
    let _ = Barometer::default();
    let _ = GNSS::default();
    let _ = Airspeed::default();
    let _ = BatteryStatus::default();
    let _ = Status::default();
    let _ = Command::default();
    let _ = SimState::default();
    let _ = OutputRaw::default();
}

#[test]
fn construct_rosflight_services() {
    let _ = ParamGetRequest::default();
    let _ = ParamGetResponse::default();
    let _ = ParamSetRequest::default();
    let _ = ParamSetResponse::default();
    let _ = ParamFileRequest::default();
    let _ = ParamFileResponse::default();

    // The zero-sized service marker types (implementing `hiroz::msg::ZService`
    // with `Request`/`Response` associated types) live in a nested `srv`
    // module, e.g. `ros::rosflight_msgs::srv::ParamGet` — not
    // `ros::rosflight_msgs::srv::ParamGet::{Request,Response}` as in the
    // rosidl/C++ convention. Reference them here to confirm that module
    // exists and compiles.
    let _ = veloxity_ros_msgs::ros::rosflight_msgs::srv::ParamGet;
    let _ = veloxity_ros_msgs::ros::rosflight_msgs::srv::ParamSet;
    let _ = veloxity_ros_msgs::ros::rosflight_msgs::srv::ParamFile;
}

#[test]
fn construct_bundled_sensor_msgs() {
    let _ = Imu::default();
    let _ = Temperature::default();
    let _ = MagneticField::default();
    let _ = Range::default();
}

#[test]
fn construct_bundled_std_srvs() {
    let _ = TriggerRequest::default();
    let _ = TriggerResponse::default();
    let _ = veloxity_ros_msgs::std_srvs::srv::Trigger;
}

#[test]
fn pwm_output_has_14_channels() {
    assert_eq!(PwmOutput::default().values.len(), 14);
}

#[test]
fn rc_raw_has_8_channels() {
    assert_eq!(RCRaw::default().values.len(), 8);
}
