//! Type-hash tripwire: every `rosflight_msgs` message and service
//! request/response this crate generates must carry the same RIHS01 type
//! hash `rosidl` computes for it on the C++ side. On Jazzy that hash is part
//! of the zenoh key expression, so a codegen drift here silently splits a
//! topic by type instead of failing loudly (see `rosflight_hiroz::typehash`
//! for the full story).
//!
//! No-op unless `AMENT_PREFIX_PATH` is set (i.e. sourced from a built ROS 2
//! install), so this stays green without a ROS environment.

use rosflight_hiroz::typehash::testing::assert_type_hash_matches_install as check_one;
use veloxity_ros_msgs::ros::rosflight_msgs;

/// Pin the RIHS01 hash for one type per line; `check!` is just repetition
/// sugar over `check_one::<T>()`.
macro_rules! check {
    ($($ty:path),+ $(,)?) => {
        $( check_one::<$ty>(); )+
    };
}

#[test]
fn type_hashes_match_install() {
    check!(
        // rosflight_msgs messages (msg_vendor/rosflight_msgs)
        rosflight_msgs::Airspeed,
        rosflight_msgs::Attitude,
        rosflight_msgs::AuxCommand,
        rosflight_msgs::Barometer,
        rosflight_msgs::BatteryStatus,
        rosflight_msgs::Command,
        rosflight_msgs::Error,
        rosflight_msgs::GNSS,
        rosflight_msgs::OutputRaw,
        rosflight_msgs::PwmOutput,
        rosflight_msgs::RCRaw,
        rosflight_msgs::RGBCamera,
        rosflight_msgs::RangeFinderSensor,
        rosflight_msgs::SimState,
        rosflight_msgs::Status,
        // rosflight_msgs services: hiroz-codegen implements MessageTypeInfo
        // on the Request/Response structs themselves, not on the `srv::*`
        // marker (which implements ServiceTypeInfo instead) -- see
        // hiroz-codegen's generate_service_impl.
        rosflight_msgs::ParamFileRequest,
        rosflight_msgs::ParamFileResponse,
        rosflight_msgs::ParamGetRequest,
        rosflight_msgs::ParamGetResponse,
        rosflight_msgs::ParamSetRequest,
        rosflight_msgs::ParamSetResponse,
        rosflight_msgs::SetSimStateRequest,
        rosflight_msgs::SetSimStateResponse,
    );
}
