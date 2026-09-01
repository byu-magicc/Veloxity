//! The rclcpp-shaped bootstrap layer hiroz does not provide.
//!
//! Hiroz is a ROS 2 middleware, not a client library: it has no
//! `rclcpp::init(argc, argv)`, no `--ros-args` grammar, no remap rules, no
//! `--params-file` handling, and it installs no `tracing` subscriber. These
//! four modules supply exactly that much of rclcpp, so the ported node reads
//! like its C++ original and keeps working with the launch files and the
//! flight scripts' log greps.
//!
//! Every module here is a verbatim copy of the corresponding file in
//! `rosplane_rs/rosplane_nodes/src/shim/`, which has a further sibling in
//! `roscopter_rs/roscopter_nodes/src/shim/`. See each module's own header.

pub mod context;
pub mod logging;
pub mod rosargs;
pub mod yamlparams;
