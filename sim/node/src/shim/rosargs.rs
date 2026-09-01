//! `--ros-args` command-line parsing, to the grammar this stack actually
//! launches with.
//!
//! Sibling copies: `rosplane_rs/rosplane_nodes/src/shim/rosargs.rs` — the
//! original this file was copied from verbatim — and
//! `roscopter_rs/roscopter_nodes/src/shim/rosargs.rs`. The three live in
//! separate repositories and nothing links them, so a fix here has to be
//! made in all three or they drift apart.
//!
//! Hiroz has no CLI layer at all: no `rclcpp::init(argc, argv)` equivalent, no
//! `--ros-args`, no remap rules, no `--params-file`. Everything a node is told
//! on the command line has to be parsed here and turned into builder calls.
//!
//! # Grammar
//!
//! ```text
//! <exe> [plain...] [--ros-args <ros-arg>... [-- [plain...]]]...
//! ```
//!
//! * Arguments before the first `--ros-args` are **plain**: the controller
//!   selects its variant with a bare `total_energy` there
//!   (`controller_node.cpp:222-237`).
//! * Inside a `--ros-args` section:
//!   * `-r RULE`, `--remap RULE` — a remap rule (below)
//!   * `-p NAME:=VALUE`, `--param NAME:=VALUE` — one parameter override
//!   * `--params-file PATH` — a parameter file, applied in order
//!   * `--log-level LEVEL` or `--log-level NAME:=LEVEL`
//!   * `--` ends the section; anything after it is plain again
//! * `--ros-args` may appear more than once; the sections concatenate.
//! * **Any other `--ros-args` token is a hard error.** rclcpp accepts a longer
//!   list (`--enclave`, the rosout/stdout log toggles, `--log-file-name`), and
//!   silently ignoring one of those here would mean running with a
//!   configuration nobody asked for; failing loudly is the safer default for
//!   an autopilot. Add them here deliberately when a launch file needs one.
//!
//! # Remap rules
//!
//! * `__node:=NAME` / `__name:=NAME` — rename this node.
//! * `__ns:=NS` — **hard error**. Nothing in this stack runs in a namespace,
//!   and half-supporting one (names remapped here but not in the parameter
//!   sections or the hiroz node builder) would fail in ways that only show up
//!   in flight.
//! * `FROM:=TO`, `rostopic://FROM:=TO`, `rosservice://FROM:=TO` — name remap,
//!   optionally restricted to topics or to services.
//!
//! Both sides are expanded before matching: a relative name gains a leading
//! `/`, and a leading `~` becomes `/<node name>`. Matching is first-rule-wins,
//! as in `rcl_remap_topic_name`.

use std::path::PathBuf;

use hiroz::parameter::ParameterValue;

use crate::shim::logging::LogLevel;
use crate::shim::yamlparams;

/// Which names a remap rule applies to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RemapKind {
    /// `rostopic://from:=to`
    Topic,
    /// `rosservice://from:=to`
    Service,
    /// `from:=to` — applies to both.
    Both,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Remap {
    pub kind: RemapKind,
    /// Left-hand side, already expanded to a fully-qualified name.
    pub from: String,
    /// Right-hand side, already expanded to a fully-qualified name.
    pub to: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RosArgsError {
    /// A flag that takes a value came last.
    MissingValue(&'static str),
    /// A token inside a `--ros-args` section that this shim does not implement.
    UnknownRosArg(String),
    /// A remap rule without `:=`, or with an empty side.
    BadRemapRule(String),
    /// `-p` without `:=`, or with an empty name.
    BadParamAssignment(String),
    /// `__ns:=…`, which this shim refuses rather than half-implements.
    NamespacesUnsupported(String),
    /// An unusable value for `--log-level`.
    BadLogLevel(String),
    /// `__node:=` with an empty or invalid node name.
    BadNodeName(String),
}

impl std::fmt::Display for RosArgsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingValue(flag) => write!(f, "{flag} requires a value"),
            Self::UnknownRosArg(a) => write!(
                f,
                "unsupported ROS argument '{a}' (this shim implements -r/--remap, \
                 -p/--param, --params-file, --log-level and --)"
            ),
            Self::BadRemapRule(r) => {
                write!(f, "invalid remap rule '{r}': expected 'from:=to'")
            }
            Self::BadParamAssignment(p) => {
                write!(
                    f,
                    "invalid parameter assignment '{p}': expected 'name:=value'"
                )
            }
            Self::NamespacesUnsupported(ns) => write!(
                f,
                "'__ns:={ns}' is not supported: this shim runs nodes in the root \
                 namespace only"
            ),
            Self::BadLogLevel(l) => write!(f, "invalid --log-level '{l}'"),
            Self::BadNodeName(n) => write!(f, "invalid node name '{n}'"),
        }
    }
}

impl std::error::Error for RosArgsError {}

/// Everything the command line said, in the shape the node bootstrap needs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RosArgs {
    /// The effective node name: the `__node:=`/`__name:=` value if one was
    /// given, otherwise the port's compiled-in default.
    pub node_name: String,
    /// Remap rules in command-line order (first match wins).
    pub remaps: Vec<Remap>,
    /// `-p` overrides in command-line order. These beat every parameter file.
    pub param_overrides: Vec<(String, ParameterValue)>,
    /// `--params-file` paths in command-line order (later files win).
    pub params_files: Vec<PathBuf>,
    /// `--log-level`, if given.
    pub log_level: Option<LogLevel>,
    /// Positional arguments outside every `--ros-args` section, in order,
    /// **excluding** argv[0].
    pub plain_args: Vec<String>,
}

/// Expand a ROS name for matching: `~/x` -> `/<node>/x`, `x` -> `/x`,
/// `/x` unchanged. The stack has no namespaces, so this is the whole of
/// rclcpp's name expansion that can be observed here.
fn expand(name: &str, node_name: &str) -> String {
    if let Some(rest) = name.strip_prefix("~/") {
        format!("/{node_name}/{rest}")
    } else if name == "~" {
        format!("/{node_name}")
    } else if name.starts_with('/') {
        name.to_string()
    } else {
        format!("/{name}")
    }
}

fn valid_node_name(name: &str) -> bool {
    !name.is_empty()
        && name
            .chars()
            .next()
            .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
}

impl RosArgs {
    /// Parse `argv` (including argv[0], which is ignored). `default_node_name`
    /// is the port's compiled-in node name, used unless `__node:=` overrides it.
    pub fn parse<I, S>(default_node_name: &str, argv: I) -> Result<Self, RosArgsError>
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        let argv: Vec<String> = argv.into_iter().map(Into::into).collect();
        let mut out = RosArgs {
            node_name: default_node_name.to_string(),
            ..Default::default()
        };

        // Pass 1: the node name only. It has to be settled first because `~`
        // expansion in a remap rule depends on it and `__node:=` may appear
        // *after* the rule that uses it (the shadow-twin command lines do
        // exactly that). Section tracking is duplicated rather than shortcut,
        // so a positional `-r` outside a section stays positional in both
        // passes.
        for rule in remap_rules(&argv) {
            let Some((lhs, rhs)) = rule.split_once(":=") else {
                continue;
            };
            match lhs {
                "__node" | "__name" => {
                    if !valid_node_name(rhs) {
                        return Err(RosArgsError::BadNodeName(rhs.to_string()));
                    }
                    out.node_name = rhs.to_string();
                }
                "__ns" => return Err(RosArgsError::NamespacesUnsupported(rhs.to_string())),
                _ => {}
            }
        }

        // Pass 2: everything else, now that the node name is settled.
        let mut in_ros_args = false;
        let mut i = 1;
        while i < argv.len() {
            let arg = argv[i].as_str();
            if !in_ros_args {
                if arg == "--ros-args" {
                    in_ros_args = true;
                } else {
                    out.plain_args.push(arg.to_string());
                }
                i += 1;
                continue;
            }

            match arg {
                "--" => in_ros_args = false,
                // Two adjacent sections: `--ros-args ... --ros-args ...`.
                "--ros-args" => {}
                "-r" | "--remap" => {
                    let rule = take_value(&argv, &mut i, "-r/--remap")?;
                    out.push_remap(&rule)?;
                }
                "-p" | "--param" => {
                    let assign = take_value(&argv, &mut i, "-p/--param")?;
                    let (name, value) = assign
                        .split_once(":=")
                        .ok_or_else(|| RosArgsError::BadParamAssignment(assign.clone()))?;
                    if name.is_empty() {
                        return Err(RosArgsError::BadParamAssignment(assign.clone()));
                    }
                    out.param_overrides
                        .push((name.to_string(), yamlparams::parse_value(value)));
                }
                "--params-file" => {
                    let path = take_value(&argv, &mut i, "--params-file")?;
                    out.params_files.push(PathBuf::from(path));
                }
                "--log-level" => {
                    let spec = take_value(&argv, &mut i, "--log-level")?;
                    // rclcpp also accepts `logger_name:=LEVEL`; the shim has
                    // one logger per process, so a per-logger spec sets it.
                    let level = spec.rsplit(":=").next().unwrap_or(&spec);
                    out.log_level = Some(
                        LogLevel::parse(level)
                            .ok_or_else(|| RosArgsError::BadLogLevel(spec.clone()))?,
                    );
                }
                other => return Err(RosArgsError::UnknownRosArg(other.to_string())),
            }
            i += 1;
        }

        Ok(out)
    }

    /// Parse the real process arguments.
    pub fn parse_env(default_node_name: &str) -> Result<Self, RosArgsError> {
        Self::parse(default_node_name, std::env::args())
    }

    fn push_remap(&mut self, rule: &str) -> Result<(), RosArgsError> {
        let (lhs, rhs) = rule
            .split_once(":=")
            .ok_or_else(|| RosArgsError::BadRemapRule(rule.to_string()))?;
        if lhs.is_empty() || rhs.is_empty() {
            return Err(RosArgsError::BadRemapRule(rule.to_string()));
        }
        match lhs {
            // Handled in pass 1; not a name remap.
            "__node" | "__name" => return Ok(()),
            "__ns" => return Err(RosArgsError::NamespacesUnsupported(rhs.to_string())),
            _ => {}
        }
        let (kind, from) = if let Some(rest) = lhs.strip_prefix("rostopic://") {
            (RemapKind::Topic, rest)
        } else if let Some(rest) = lhs.strip_prefix("rosservice://") {
            (RemapKind::Service, rest)
        } else {
            (RemapKind::Both, lhs)
        };
        if from.is_empty() {
            return Err(RosArgsError::BadRemapRule(rule.to_string()));
        }
        self.remaps.push(Remap {
            kind,
            from: expand(from, &self.node_name),
            to: expand(rhs, &self.node_name),
        });
        Ok(())
    }

    fn remap(&self, name: &str, kinds: [RemapKind; 2]) -> String {
        let expanded = expand(name, &self.node_name);
        self.remaps
            .iter()
            .find(|r| kinds.contains(&r.kind) && r.from == expanded)
            .map(|r| r.to.clone())
            .unwrap_or(expanded)
    }

    /// Resolve a topic name the node is about to publish or subscribe on.
    /// Always returns a fully-qualified name, so the caller can hand it
    /// straight to `create_pub`/`create_sub`.
    pub fn remap_topic(&self, name: &str) -> String {
        self.remap(name, [RemapKind::Topic, RemapKind::Both])
    }

    /// Resolve a service or client name. Same contract as `remap_topic`.
    pub fn remap_service(&self, name: &str) -> String {
        self.remap(name, [RemapKind::Service, RemapKind::Both])
    }
}

/// Yield the value of the flag at `argv[*i]`, advancing past it.
fn take_value(argv: &[String], i: &mut usize, flag: &'static str) -> Result<String, RosArgsError> {
    *i += 1;
    argv.get(*i)
        .cloned()
        .ok_or(RosArgsError::MissingValue(flag))
}

/// Pass-1 scanner: every `-r`/`--remap` value that sits inside a `--ros-args`
/// section, in order. Section state is tracked exactly as in the main pass, so
/// a positional `-r` before `--ros-args` is a plain argument in both.
/// Malformed rules are left for the main pass to report.
fn remap_rules(argv: &[String]) -> Vec<String> {
    let mut rules = Vec::new();
    let mut in_ros_args = false;
    let mut i = 1;
    while i < argv.len() {
        match argv[i].as_str() {
            "--ros-args" => in_ros_args = true,
            "--" => in_ros_args = false,
            "-r" | "--remap" if in_ros_args => {
                if let Some(rule) = argv.get(i + 1) {
                    rules.push(rule.clone());
                }
                i += 1;
            }
            _ => {}
        }
        i += 1;
    }
    rules
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> RosArgs {
        RosArgs::parse(
            "controller",
            std::iter::once("controller").chain(args.iter().copied()),
        )
        .expect("parse must succeed")
    }

    fn parse_err(args: &[&str]) -> RosArgsError {
        RosArgs::parse(
            "controller",
            std::iter::once("controller").chain(args.iter().copied()),
        )
        .expect_err("parse must fail")
    }

    // ---------------------------------------------- the real launch commands

    /// `tools/flight_run.sh` line ~215:
    /// `ros2 run rosplane_rs controller default --ros-args -r __node:=controller
    ///  --params-file $AUTOPILOT_PARAMS -r estimated_state:=/sim/rosplane/state
    ///  -p use_sim_time:=false`
    #[test]
    fn flight_run_controller_invocation() {
        let a = parse(&[
            "default",
            "--ros-args",
            "-r",
            "__node:=controller",
            "--params-file",
            "/opt/params/anaconda_autopilot_params.yaml",
            "-r",
            "estimated_state:=/sim/rosplane/state",
            "-p",
            "use_sim_time:=false",
        ]);
        assert_eq!(a.node_name, "controller");
        assert_eq!(a.plain_args, vec!["default".to_string()]);
        assert_eq!(
            a.params_files,
            vec![PathBuf::from("/opt/params/anaconda_autopilot_params.yaml")]
        );
        assert_eq!(
            a.param_overrides,
            vec![("use_sim_time".to_string(), ParameterValue::Bool(false))]
        );
        assert_eq!(a.remap_topic("estimated_state"), "/sim/rosplane/state");
        // Unrelated names pass through, fully qualified.
        assert_eq!(a.remap_topic("controller_command"), "/controller_command");
        assert_eq!(a.remap_topic("/command"), "/command");
    }

    /// The `total_energy` variant selector must survive as a plain arg —
    /// `controller_node.cpp:222` reads `argv[1]`, and this is the only thing
    /// that turns the TECS parameters on.
    #[test]
    fn controller_variant_selector_is_a_plain_arg() {
        let a = parse(&["total_energy", "--ros-args", "-r", "__node:=controller"]);
        assert_eq!(a.plain_args, vec!["total_energy".to_string()]);
        let a = parse(&["--ros-args", "-r", "__node:=controller"]);
        assert!(
            a.plain_args.is_empty(),
            "no selector means the default variant"
        );
    }

    /// The shadow-twin invocation (`flight_run.sh --shadow`): a rename plus
    /// two output remaps, so the reference node's topics do not collide.
    #[test]
    fn shadow_twin_invocation() {
        let a = RosArgs::parse(
            "controller",
            [
                "controller",
                "default",
                "--ros-args",
                "-r",
                "__node:=controller_cxx",
                "-r",
                "command:=command_cxx",
                "-r",
                "controller_internals:=controller_internals_cxx",
                "-r",
                "estimated_state:=/sim/rosplane/state",
            ],
        )
        .unwrap();
        assert_eq!(a.node_name, "controller_cxx");
        assert_eq!(a.remap_topic("command"), "/command_cxx");
        assert_eq!(
            a.remap_topic("controller_internals"),
            "/controller_internals_cxx"
        );
        assert_eq!(a.remap_topic("estimated_state"), "/sim/rosplane/state");
    }

    /// `run_bg truth ros2 run rosplane_rs sim_state_transcriber --ros-args
    ///  -r __node:=rosplane_truth -p use_sim_time:=false`
    #[test]
    fn transcriber_invocation() {
        let a = RosArgs::parse(
            "rosplane_state_transcription",
            [
                "sim_state_transcriber",
                "--ros-args",
                "-r",
                "__node:=rosplane_truth",
                "-p",
                "use_sim_time:=false",
            ],
        )
        .unwrap();
        assert_eq!(a.node_name, "rosplane_truth");
        assert_eq!(a.remap_service("param_get"), "/param_get");
    }

    #[test]
    fn estimator_invocation_with_a_double_override() {
        let a = RosArgs::parse(
            "estimator",
            [
                "estimator",
                "--ros-args",
                "-r",
                "__node:=estimator",
                "--params-file",
                "/opt/params/estimator.yaml",
                "-p",
                "rho:=-1.0",
                "-p",
                "use_sim_time:=false",
            ],
        )
        .unwrap();
        assert_eq!(
            a.param_overrides,
            vec![
                ("rho".to_string(), ParameterValue::Double(-1.0)),
                ("use_sim_time".to_string(), ParameterValue::Bool(false)),
            ]
        );
    }

    // ------------------------------------------------------------- structure

    #[test]
    fn plain_args_before_and_after_a_section() {
        let a = parse(&["first", "--ros-args", "-p", "x:=1", "--", "second", "third"]);
        assert_eq!(a.plain_args, vec!["first", "second", "third"]);
        assert_eq!(a.param_overrides.len(), 1);
    }

    #[test]
    fn multiple_ros_args_sections_concatenate() {
        let a = parse(&[
            "--ros-args",
            "-p",
            "a:=1",
            "--",
            "plain",
            "--ros-args",
            "-p",
            "b:=2",
        ]);
        assert_eq!(a.plain_args, vec!["plain".to_string()]);
        assert_eq!(
            a.param_overrides,
            vec![
                ("a".to_string(), ParameterValue::Integer(1)),
                ("b".to_string(), ParameterValue::Integer(2)),
            ]
        );
        // Back-to-back sections without an intervening `--`.
        let a = parse(&["--ros-args", "-p", "a:=1", "--ros-args", "-p", "b:=2"]);
        assert_eq!(a.param_overrides.len(), 2);
    }

    #[test]
    fn params_files_keep_their_order() {
        let a = parse(&[
            "--ros-args",
            "--params-file",
            "/one.yaml",
            "--params-file",
            "/two.yaml",
        ]);
        assert_eq!(
            a.params_files,
            vec![PathBuf::from("/one.yaml"), PathBuf::from("/two.yaml")]
        );
    }

    #[test]
    fn long_flag_spellings_work() {
        let a = parse(&[
            "--ros-args",
            "--remap",
            "__node:=ctrl",
            "--param",
            "gain:=2.5",
        ]);
        assert_eq!(a.node_name, "ctrl");
        assert_eq!(
            a.param_overrides,
            vec![("gain".to_string(), ParameterValue::Double(2.5))]
        );
    }

    // ---------------------------------------------------------------- values

    #[test]
    fn param_values_take_ros_scalar_types() {
        let a = parse(&[
            "--ros-args",
            "-p",
            "b:=true",
            "-p",
            "i:=100",
            "-p",
            "d:=100.0",
            "-p",
            "neg:=-1.0",
            "-p",
            "s:=total_energy",
            "-p",
            "quoted:='42'",
            "-p",
            "arr:=[1.0, 2.0]",
        ]);
        let v: Vec<_> = a.param_overrides.iter().map(|(_, v)| v.clone()).collect();
        assert_eq!(
            v,
            vec![
                ParameterValue::Bool(true),
                ParameterValue::Integer(100),
                ParameterValue::Double(100.0),
                ParameterValue::Double(-1.0),
                ParameterValue::String("total_energy".to_string()),
                ParameterValue::String("42".to_string()),
                ParameterValue::DoubleArray(vec![1.0, 2.0]),
            ]
        );
    }

    // ---------------------------------------------------------------- remaps

    #[test]
    fn remap_kinds_are_respected() {
        let a = parse(&[
            "--ros-args",
            "-r",
            "rostopic:///a:=/topic_b",
            "-r",
            "rosservice:///a:=/service_b",
        ]);
        assert_eq!(a.remap_topic("/a"), "/topic_b");
        assert_eq!(a.remap_service("/a"), "/service_b");
    }

    #[test]
    fn first_matching_rule_wins() {
        let a = parse(&["--ros-args", "-r", "a:=/first", "-r", "a:=/second"]);
        assert_eq!(a.remap_topic("a"), "/first");
    }

    #[test]
    fn private_names_expand_against_the_effective_node_name() {
        // The rename is applied before rules are expanded, even though it
        // appears after them on the command line.
        let a = parse(&["--ros-args", "-r", "~/dbg:=/debug", "-r", "__node:=ctrl"]);
        assert_eq!(a.node_name, "ctrl");
        assert_eq!(a.remaps[0].from, "/ctrl/dbg");
        assert_eq!(a.remap_topic("~/dbg"), "/debug");
        assert_eq!(a.remap_topic("~/other"), "/ctrl/other");
    }

    #[test]
    fn log_level_parses_both_spellings() {
        assert_eq!(
            parse(&["--ros-args", "--log-level", "debug"]).log_level,
            Some(LogLevel::Debug)
        );
        assert_eq!(
            parse(&["--ros-args", "--log-level", "WARN"]).log_level,
            Some(LogLevel::Warn)
        );
        assert_eq!(
            parse(&["--ros-args", "--log-level", "controller:=error"]).log_level,
            Some(LogLevel::Error)
        );
        assert_eq!(parse(&[]).log_level, None);
    }

    // ---------------------------------------------------------------- errors

    #[test]
    fn namespaces_are_a_hard_error() {
        assert_eq!(
            parse_err(&["--ros-args", "-r", "__ns:=/uav1"]),
            RosArgsError::NamespacesUnsupported("/uav1".to_string())
        );
    }

    #[test]
    fn unknown_ros_args_are_a_hard_error() {
        assert_eq!(
            parse_err(&["--ros-args", "--enclave", "/foo"]),
            RosArgsError::UnknownRosArg("--enclave".to_string())
        );
        assert_eq!(
            parse_err(&["--ros-args", "--enable-rosout-logs"]),
            RosArgsError::UnknownRosArg("--enable-rosout-logs".to_string())
        );
    }

    #[test]
    fn malformed_arguments_are_rejected() {
        assert_eq!(
            parse_err(&["--ros-args", "-r", "no_assignment"]),
            RosArgsError::BadRemapRule("no_assignment".to_string())
        );
        assert_eq!(
            parse_err(&["--ros-args", "-r", "a:="]),
            RosArgsError::BadRemapRule("a:=".to_string())
        );
        assert_eq!(
            parse_err(&["--ros-args", "-p", "no_assignment"]),
            RosArgsError::BadParamAssignment("no_assignment".to_string())
        );
        assert_eq!(
            parse_err(&["--ros-args", "-p"]),
            RosArgsError::MissingValue("-p/--param")
        );
        assert_eq!(
            parse_err(&["--ros-args", "--params-file"]),
            RosArgsError::MissingValue("--params-file")
        );
        assert_eq!(
            parse_err(&["--ros-args", "--log-level", "chatty"]),
            RosArgsError::BadLogLevel("chatty".to_string())
        );
        assert_eq!(
            parse_err(&["--ros-args", "-r", "__node:=not a name"]),
            RosArgsError::BadNodeName("not a name".to_string())
        );
    }

    #[test]
    fn plain_args_are_never_interpreted() {
        // A bare `-p` outside a section is a positional argument, not a flag.
        let a = parse(&["-p", "x:=1"]);
        assert_eq!(a.plain_args, vec!["-p", "x:=1"]);
        assert!(a.param_overrides.is_empty());

        // The same must hold for `-r`, which the node-name pre-scan also
        // looks for — the two passes have to agree about what is positional.
        let a = parse(&["-r", "__node:=renamed"]);
        assert_eq!(a.node_name, "controller", "a positional -r must not rename");
        assert_eq!(a.plain_args, vec!["-r", "__node:=renamed"]);
        assert!(a.remaps.is_empty());
    }
}
