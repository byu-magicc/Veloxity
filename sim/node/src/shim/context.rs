//! Node bootstrap: context, node, parameters, logging, remaps.
//!
//! Sibling copies: `rosplane_rs/rosplane_nodes/src/shim/context.rs` — the
//! original this file was copied from verbatim — and
//! `roscopter_rs/roscopter_nodes/src/shim/context.rs`. The three live in
//! separate repositories and nothing links them, so a fix here has to be
//! made in all three or they drift apart.
//!
//! This is the Rust replacement for `rclcpp::init(argc, argv)` plus the
//! `Node("name")` constructor. It does four things, in this order:
//!
//! 1. parse `--ros-args` ([`crate::shim::rosargs`]),
//! 2. install the rcutils-shaped subscriber ([`crate::shim::logging`]) — before
//!    anything that can log,
//! 3. build a `ZContext` on the router endpoint and ROS domain, and
//! 4. build the node under its effective name with **one** merged parameter
//!    override map.
//!
//! # One override map, never a file
//!
//! `ZNodeBuilder::with_parameter_file` silently ignores bare node-name YAML
//! sections and `with_parameter_overrides` *replaces* rather than extends, so
//! mixing the two loses one side. The shim therefore merges every
//! `--params-file` and every `-p` itself
//! ([`crate::shim::yamlparams::merged_overrides`]) and makes exactly one
//! `with_parameter_overrides` call. Do not add a `with_parameter_file` call to
//! a port.
//!
//! # Environment
//!
//! * `ROS_DOMAIN_ID` — the ROS domain, default 0. Note the spike finding:
//!   topic and service traffic is correctly domain-scoped, but hiroz hardcodes
//!   domain 0 for the parameter and type-description services
//!   (`hiroz/src/parameter/service.rs:299-306`), so `ros2 param …` against
//!   these nodes only works when the whole stack runs on domain 0.
//! * `HIROZ_CONNECT_ENDPOINT` — the zenoh router, default
//!   [`DEFAULT_ENDPOINT`]. The node connects in `client` mode, exactly like
//!   `rmw_zenoh_cpp`, so it joins the same `rmw_zenohd` the C++ nodes use.

use std::collections::HashMap;

use hiroz::Builder;
use hiroz::context::{ZContext, ZContextBuilder};
use hiroz::node::ZNode;
use hiroz::parameter::ParameterValue;

use crate::shim::logging;
use crate::shim::rosargs::{RosArgs, RosArgsError};
use crate::shim::yamlparams::{self, YamlParamError};

/// The `rmw_zenohd` endpoint the devcontainer runs.
pub const DEFAULT_ENDPOINT: &str = "tcp/127.0.0.1:7447";
pub const ENDPOINT_ENV: &str = "HIROZ_CONNECT_ENDPOINT";
pub const DOMAIN_ENV: &str = "ROS_DOMAIN_ID";

#[derive(Debug)]
pub enum BootstrapError {
    Args(RosArgsError),
    Params(YamlParamError),
    /// The zenoh session or the node could not be created.
    Session(String),
}

impl std::fmt::Display for BootstrapError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Args(e) => write!(f, "{e}"),
            Self::Params(e) => write!(f, "{e}"),
            Self::Session(e) => write!(f, "cannot start the ROS 2 session: {e}"),
        }
    }
}

impl std::error::Error for BootstrapError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Args(e) => Some(e),
            Self::Params(e) => Some(e),
            Self::Session(_) => None,
        }
    }
}

impl From<RosArgsError> for BootstrapError {
    fn from(e: RosArgsError) -> Self {
        Self::Args(e)
    }
}

impl From<YamlParamError> for BootstrapError {
    fn from(e: YamlParamError) -> Self {
        Self::Params(e)
    }
}

/// `ROS_DOMAIN_ID`, or 0. An unparseable value falls back to 0 rather than
/// failing, matching `rcl`, which ignores a malformed domain.
pub fn domain_id() -> usize {
    std::env::var(DOMAIN_ENV)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(0)
}

/// `HIROZ_CONNECT_ENDPOINT`, or [`DEFAULT_ENDPOINT`].
pub fn connect_endpoint() -> String {
    match std::env::var(ENDPOINT_ENV) {
        Ok(v) if !v.trim().is_empty() => v,
        _ => DEFAULT_ENDPOINT.to_string(),
    }
}

/// A live node, its context, and the command line that produced them.
///
/// Field order is drop order: `node` is declared first so it tears down before
/// the session it borrows from.
pub struct NodeHandle {
    pub node: ZNode,
    pub context: ZContext,
    pub args: RosArgs,
    /// The merged parameter overrides that were handed to the node builder,
    /// kept for diagnostics (which file supplied which gain).
    pub overrides: HashMap<String, ParameterValue>,
}

impl NodeHandle {
    /// Resolve a topic name through the command-line remap rules. Always
    /// returns a fully-qualified name.
    pub fn remap_topic(&self, name: &str) -> String {
        self.args.remap_topic(name)
    }

    /// Resolve a service or client name through the command-line remap rules.
    pub fn remap_service(&self, name: &str) -> String {
        self.args.remap_service(name)
    }

    /// The effective node name (after `__node:=`).
    pub fn node_name(&self) -> &str {
        &self.args.node_name
    }
}

/// Build the context and node from already-parsed arguments. Logging is *not*
/// installed here — call [`logging::init`] first if you are using this instead
/// of [`bootstrap`].
pub fn build(args: RosArgs) -> Result<NodeHandle, BootstrapError> {
    let overrides =
        yamlparams::merged_overrides(&args.params_files, &args.param_overrides, &args.node_name)?;

    let context = ZContextBuilder::default()
        .with_domain_id(domain_id())
        .with_mode("client")
        .with_connect_endpoints([connect_endpoint()])
        .build()
        .map_err(|e| BootstrapError::Session(e.to_string()))?;

    // Exactly one `with_parameter_overrides`, and no `with_parameter_file`.
    let node = context
        .create_node(&args.node_name)
        .with_parameter_overrides(overrides.clone())
        .build()
        .map_err(|e| BootstrapError::Session(e.to_string()))?;

    Ok(NodeHandle {
        node,
        context,
        args,
        overrides,
    })
}

/// The whole startup sequence from the real process arguments:
/// parse, install logging, connect, create the node.
///
/// `default_node_name` is the port's compiled-in name, used unless
/// `__node:=` overrides it.
pub fn bootstrap(default_node_name: &str) -> Result<NodeHandle, BootstrapError> {
    bootstrap_from(default_node_name, std::env::args())
}

/// [`bootstrap`] with an explicit argument vector (including argv[0]).
pub fn bootstrap_from<I, S>(default_node_name: &str, argv: I) -> Result<NodeHandle, BootstrapError>
where
    I: IntoIterator<Item = S>,
    S: Into<String>,
{
    let args = RosArgs::parse(default_node_name, argv)?;
    // Before the session: hiroz logs during `build()`, and those lines should
    // already be in the node's format.
    logging::init(&args.node_name, args.log_level);
    build(args)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `build()` needs a zenoh session, so the tests here cover the pure
    /// configuration surface only. Everything else is exercised by the node
    /// ports' own integration runs.
    ///
    /// Both environment variables are covered by this ONE test on purpose:
    /// `set_var` is process-global and `cargo test` runs test functions on
    /// parallel threads, so a second env-mutating test in this binary would be
    /// a genuine data race (which is why edition 2024 made `set_var` unsafe).
    #[test]
    fn environment_configuration_has_safe_defaults() {
        // SAFETY: this is the only test in the crate that touches the
        // environment, and it does so from a single thread.
        unsafe {
            std::env::remove_var(ENDPOINT_ENV);
            std::env::remove_var(DOMAIN_ENV);
        }

        assert_eq!(DEFAULT_ENDPOINT, "tcp/127.0.0.1:7447");
        assert_eq!(connect_endpoint(), DEFAULT_ENDPOINT);
        assert_eq!(domain_id(), 0);

        unsafe {
            std::env::set_var(ENDPOINT_ENV, "tcp/10.0.0.5:7447");
            std::env::set_var(DOMAIN_ENV, "23");
        }
        assert_eq!(connect_endpoint(), "tcp/10.0.0.5:7447");
        assert_eq!(domain_id(), 23);

        // Malformed values fall back rather than aborting startup: an empty
        // endpoint would produce an empty connect list, and `rcl` likewise
        // ignores an unparseable ROS_DOMAIN_ID.
        unsafe {
            std::env::set_var(ENDPOINT_ENV, "   ");
            std::env::set_var(DOMAIN_ENV, "not-a-number");
        }
        assert_eq!(connect_endpoint(), DEFAULT_ENDPOINT);
        assert_eq!(domain_id(), 0);

        unsafe {
            std::env::remove_var(ENDPOINT_ENV);
            std::env::remove_var(DOMAIN_ENV);
        }
    }
}
