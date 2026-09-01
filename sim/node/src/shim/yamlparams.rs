//! ROS 2 parameter-file parsing, with `rclcpp` section semantics.
//!
//! Sibling copies: `rosplane_rs/rosplane_nodes/src/shim/yamlparams.rs` — the
//! original this file was copied from verbatim — and
//! `roscopter_rs/roscopter_nodes/src/shim/yamlparams.rs`. The three live in
//! separate repositories and nothing links them, so a fix here has to be
//! made in all three or they drift apart.
//!
//! # Why this exists at all
//!
//! `ZNodeBuilder::with_parameter_file` is unusable for this stack, for two
//! independent reasons found in the interop spike:
//!
//! 1. `hiroz::parameter::yaml::matches_node` accepts only `/**`, `**`,
//!    `<prefix>/**`, `<prefix>/*` or an exact `/<ns>/<name>`. A **bare**
//!    node-name section — `controller:` with no leading slash — matches
//!    nothing and is skipped with no warning.
//!    `ros2/rosplane_rs/params/anaconda_autopilot_params.yaml` opens with a
//!    bare `controller:`, so half of this repo's flight gains would silently
//!    revert to their compiled-in defaults. rclcpp accepts the bare form.
//! 2. `with_parameter_file` *extends* the override map while
//!    `with_parameter_overrides` *replaces* it, so any order that mixes the two
//!    silently drops one side.
//!
//! So the shim parses the files itself and hands hiroz exactly ONE merged
//! `.with_parameter_overrides` map (see `shim::context`).
//!
//! # Semantics reproduced
//!
//! * Section keys: `/**`, `**` (everything), `/*`, `*` (root-namespace nodes),
//!   `/ns/**`, `/ns/*`, an exact FQN `/name` or `/ns/name`, and a **bare**
//!   `name` (equivalent to `/name`).
//! * Namespace nesting: a section whose value has no `ros__parameters` key is
//!   treated as a namespace and its children are matched at the deeper path.
//! * Wildcards lose to exact matches regardless of document order, matching
//!   `rcl_node_resolve_parameter_overrides` (which applies `/**` first).
//! * Maps under `ros__parameters` flatten with `.`, as ROS 2 nests parameter
//!   names.
//! * Across files: later files win. `-p` overrides beat every file, whatever
//!   the command-line order — `rcl` keeps files and `-p` in separate lists and
//!   applies the files first.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use hiroz::parameter::ParameterValue;
use serde_yaml::Value as Yaml;

/// The key that separates a section header from the parameters it carries.
const ROS_PARAMETERS: &str = "ros__parameters";

#[derive(Debug)]
pub enum YamlParamError {
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    Parse {
        path: PathBuf,
        source: serde_yaml::Error,
    },
    /// The document root is not a mapping of section names.
    NotAMapping { path: PathBuf },
    /// A `ros__parameters` block is not a mapping.
    BadParameterBlock { path: PathBuf, section: String },
    /// A value shape ROS 2 parameters cannot represent (nested sequences,
    /// mixed-type sequences, null, …).
    UnsupportedValue { path: PathBuf, key: String },
}

impl std::fmt::Display for YamlParamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Read { path, source } => {
                write!(f, "cannot read parameter file {}: {source}", path.display())
            }
            Self::Parse { path, source } => {
                write!(
                    f,
                    "cannot parse parameter file {}: {source}",
                    path.display()
                )
            }
            Self::NotAMapping { path } => write!(
                f,
                "parameter file {} is not a mapping of node sections",
                path.display()
            ),
            Self::BadParameterBlock { path, section } => write!(
                f,
                "'{section}: {ROS_PARAMETERS}:' in {} is not a mapping",
                path.display()
            ),
            Self::UnsupportedValue { path, key } => write!(
                f,
                "parameter '{key}' in {} has a value ROS 2 parameters cannot represent",
                path.display()
            ),
        }
    }
}

impl std::error::Error for YamlParamError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Read { source, .. } => Some(source),
            Self::Parse { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Expand a node name to the fully-qualified form parameter sections are
/// matched against. The stack runs in the root namespace, so this is just a
/// leading slash — but everything below goes through here so that a namespaced
/// name passed in already-expanded stays untouched.
pub fn node_fqn(node_name: &str) -> String {
    if node_name.starts_with('/') {
        node_name.to_string()
    } else {
        format!("/{node_name}")
    }
}

/// How specifically a section matched, lowest applied first so that a more
/// specific section always wins. `rcl` applies `/**` before the node-specific
/// block regardless of where they appear in the document.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Specificity {
    /// `/**`, `**`, `/ns/**`
    DoubleWildcard,
    /// `/*`, `*`, `/ns/*`
    SingleWildcard,
    /// exact FQN, with or without the leading slash
    Exact,
}

/// rclcpp section matching. `section` is the YAML key as written; `fqn` is the
/// node's fully-qualified name (always leading-slashed).
fn match_section(section: &str, fqn: &str) -> Option<Specificity> {
    // A bare `name:` means `/name:` — the case hiroz drops on the floor.
    let sec = if section.starts_with('/') || section.starts_with('*') {
        section.to_string()
    } else {
        format!("/{section}")
    };
    let sec = sec.as_str();

    if sec == "/**" || sec == "**" {
        return Some(Specificity::DoubleWildcard);
    }
    if sec == "/*" || sec == "*" {
        // One level below the root namespace: matches `/name`, not `/ns/name`.
        return (fqn.matches('/').count() == 1).then_some(Specificity::SingleWildcard);
    }
    if let Some(prefix) = sec.strip_suffix("/**") {
        return (fqn == prefix || fqn.starts_with(&format!("{prefix}/")))
            .then_some(Specificity::DoubleWildcard);
    }
    if let Some(prefix) = sec.strip_suffix("/*") {
        return fqn
            .strip_prefix(&format!("{prefix}/"))
            .filter(|rest| !rest.is_empty() && !rest.contains('/'))
            .map(|_| Specificity::SingleWildcard);
    }
    (sec == fqn).then_some(Specificity::Exact)
}

/// Resolve one YAML scalar the way ROS 2 does: bool, then integer, then
/// double, else string. Shared with `shim::rosargs` so that `-p k:=v` and a
/// `--params-file` entry holding the same text produce the same typed value.
///
/// Only `true`/`false` and their capitalised forms are booleans — this is the
/// YAML 1.2 core schema, which is also what `rcl_yaml_param_parser` accepts.
/// `yes`/`no`/`on`/`off` are strings, in ROS 2 and here.
pub fn parse_scalar(text: &str) -> ParameterValue {
    // A quoted scalar is always a string; strip one layer of matching quotes.
    if text.len() >= 2 {
        let b = text.as_bytes();
        if (b[0] == b'\'' && b[text.len() - 1] == b'\'')
            || (b[0] == b'"' && b[text.len() - 1] == b'"')
        {
            return ParameterValue::String(text[1..text.len() - 1].to_string());
        }
    }
    match text {
        "true" | "True" | "TRUE" => return ParameterValue::Bool(true),
        "false" | "False" | "FALSE" => return ParameterValue::Bool(false),
        _ => {}
    }
    if let Ok(i) = text.parse::<i64>() {
        return ParameterValue::Integer(i);
    }
    if let Ok(d) = text.parse::<f64>() {
        return ParameterValue::Double(d);
    }
    ParameterValue::String(text.to_string())
}

/// Resolve one command-line parameter value: an `[a, b, c]` array literal if
/// it is one, otherwise a scalar. This is what `-p name:=value` goes through.
pub fn parse_value(text: &str) -> ParameterValue {
    parse_sequence_literal(text).unwrap_or_else(|| parse_scalar(text))
}

/// `[a, b, c]` array literals, as accepted on the command line.
pub fn parse_sequence_literal(text: &str) -> Option<ParameterValue> {
    let inner = text.strip_prefix('[')?.strip_suffix(']')?.trim();
    if inner.is_empty() {
        // ROS 2 cannot infer an element type here; an empty list is a
        // zero-length string array, matching rclcpp's `ParameterValue({})`.
        return Some(ParameterValue::StringArray(Vec::new()));
    }
    let items: Vec<ParameterValue> = inner.split(',').map(|s| parse_scalar(s.trim())).collect();
    homogeneous_array(&items)
}

/// Collapse a list of scalars into the matching ROS 2 array type. Mixed
/// numeric lists widen to double (rclcpp does the same); anything else falls
/// back to a string array.
fn homogeneous_array(items: &[ParameterValue]) -> Option<ParameterValue> {
    if items.iter().all(|v| matches!(v, ParameterValue::Bool(_))) {
        return Some(ParameterValue::BoolArray(
            items
                .iter()
                .map(|v| match v {
                    ParameterValue::Bool(b) => *b,
                    _ => unreachable!(),
                })
                .collect(),
        ));
    }
    if items
        .iter()
        .all(|v| matches!(v, ParameterValue::Integer(_)))
    {
        return Some(ParameterValue::IntegerArray(
            items
                .iter()
                .map(|v| match v {
                    ParameterValue::Integer(i) => *i,
                    _ => unreachable!(),
                })
                .collect(),
        ));
    }
    if items
        .iter()
        .all(|v| matches!(v, ParameterValue::Integer(_) | ParameterValue::Double(_)))
    {
        return Some(ParameterValue::DoubleArray(
            items
                .iter()
                .map(|v| match v {
                    ParameterValue::Integer(i) => *i as f64,
                    ParameterValue::Double(d) => *d,
                    _ => unreachable!(),
                })
                .collect(),
        ));
    }
    Some(ParameterValue::StringArray(
        items
            .iter()
            .map(|v| match v {
                ParameterValue::String(s) => s.clone(),
                other => format!("{other:?}"),
            })
            .collect(),
    ))
}

/// Convert one already-parsed YAML node into a parameter value. `serde_yaml`
/// has already applied the core-schema resolution, so integers and doubles
/// arrive pre-separated — which is exactly the distinction ROS 2 parameter
/// typing turns on (`100` is an integer parameter, `100.0` a double one).
fn value_from_yaml(y: &Yaml) -> Option<ParameterValue> {
    match y {
        Yaml::Bool(b) => Some(ParameterValue::Bool(*b)),
        Yaml::Number(n) => {
            if let Some(i) = n.as_i64() {
                Some(ParameterValue::Integer(i))
            } else {
                n.as_f64().map(ParameterValue::Double)
            }
        }
        Yaml::String(s) => Some(ParameterValue::String(s.clone())),
        Yaml::Sequence(items) => {
            let scalars: Option<Vec<ParameterValue>> = items.iter().map(value_from_yaml).collect();
            homogeneous_array(&scalars?)
        }
        _ => None,
    }
}

/// Flatten a `ros__parameters` block into dotted parameter names.
fn flatten(
    prefix: &str,
    map: &serde_yaml::Mapping,
    out: &mut BTreeMap<String, ParameterValue>,
    path: &Path,
) -> Result<(), YamlParamError> {
    for (k, v) in map {
        let key = k.as_str().map(str::to_string).unwrap_or_else(|| {
            serde_yaml::to_string(k)
                .unwrap_or_default()
                .trim()
                .to_string()
        });
        let full = if prefix.is_empty() {
            key
        } else {
            format!("{prefix}.{key}")
        };
        match v {
            Yaml::Mapping(sub) => flatten(&full, sub, out, path)?,
            other => {
                let value =
                    value_from_yaml(other).ok_or_else(|| YamlParamError::UnsupportedValue {
                        path: path.to_path_buf(),
                        key: full.clone(),
                    })?;
                out.insert(full, value);
            }
        }
    }
    Ok(())
}

/// Walk one section, which is either a `ros__parameters` carrier or a
/// namespace holding more sections.
fn collect_section(
    section_path: &str,
    value: &Yaml,
    fqn: &str,
    path: &Path,
    out: &mut Vec<(Specificity, usize, BTreeMap<String, ParameterValue>)>,
    order: &mut usize,
) -> Result<(), YamlParamError> {
    let Yaml::Mapping(map) = value else {
        return Ok(());
    };

    if let Some(params) = map.get(Yaml::String(ROS_PARAMETERS.to_string())) {
        if let Some(spec) = match_section(section_path, fqn) {
            let Yaml::Mapping(params) = params else {
                return Err(YamlParamError::BadParameterBlock {
                    path: path.to_path_buf(),
                    section: section_path.to_string(),
                });
            };
            let mut flat = BTreeMap::new();
            flatten("", params, &mut flat, path)?;
            out.push((spec, *order, flat));
            *order += 1;
        }
        return Ok(());
    }

    // No `ros__parameters` here: this key is a namespace level. Descend,
    // building the deeper section path, so `/ns: { node: { ros__parameters }}`
    // matches a node named `/ns/node`.
    for (k, v) in map {
        let Some(child) = k.as_str() else { continue };
        let base = if section_path.starts_with('/') {
            section_path.to_string()
        } else {
            format!("/{section_path}")
        };
        let deeper = format!("{}/{child}", base.trim_end_matches('/'));
        collect_section(&deeper, v, fqn, path, out, order)?;
    }
    Ok(())
}

/// Parse one parameter file's text and return the parameters that apply to
/// `node_name`, with more specific sections overriding wildcards.
pub fn parse_params_str(
    text: &str,
    node_name: &str,
    path: &Path,
) -> Result<BTreeMap<String, ParameterValue>, YamlParamError> {
    let doc: Yaml = serde_yaml::from_str(text).map_err(|source| YamlParamError::Parse {
        path: path.to_path_buf(),
        source,
    })?;
    let fqn = node_fqn(node_name);

    let root = match &doc {
        Yaml::Mapping(m) => m,
        // An empty document has no parameters, which is not an error.
        Yaml::Null => return Ok(BTreeMap::new()),
        _ => {
            return Err(YamlParamError::NotAMapping {
                path: path.to_path_buf(),
            });
        }
    };

    let mut matched = Vec::new();
    let mut order = 0usize;
    for (k, v) in root {
        let Some(section) = k.as_str() else { continue };
        collect_section(section, v, &fqn, path, &mut matched, &mut order)?;
    }

    // Wildcards first, then exact; ties broken by document order. `sort_by_key`
    // is stable, so `(specificity, order)` gives exactly that.
    matched.sort_by_key(|(spec, order, _)| (*spec, *order));

    let mut out = BTreeMap::new();
    for (_, _, params) in matched {
        out.extend(params);
    }
    Ok(out)
}

/// Read and parse one parameter file for `node_name`.
pub fn parse_params_file(
    path: &Path,
    node_name: &str,
) -> Result<BTreeMap<String, ParameterValue>, YamlParamError> {
    let text = std::fs::read_to_string(path).map_err(|source| YamlParamError::Read {
        path: path.to_path_buf(),
        source,
    })?;
    parse_params_str(&text, node_name, path)
}

/// The single map handed to `ZNodeBuilder::with_parameter_overrides`.
///
/// Files are applied in the order given (later wins per key), then the `-p`
/// overrides — which therefore beat every file regardless of where they sat on
/// the command line, matching `rcl_node_resolve_parameter_overrides`.
pub fn merged_overrides(
    files: &[PathBuf],
    overrides: &[(String, ParameterValue)],
    node_name: &str,
) -> Result<std::collections::HashMap<String, ParameterValue>, YamlParamError> {
    let mut merged: BTreeMap<String, ParameterValue> = BTreeMap::new();
    for file in files {
        merged.extend(parse_params_file(file, node_name)?);
    }
    for (k, v) in overrides {
        merged.insert(k.clone(), v.clone());
    }
    Ok(merged.into_iter().collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    // The rosplane copy also defines `ANACONDA`, `ESTIMATOR`, `repo_file` and
    // `load` here, for the corpus-replay tests that read that repo's own
    // parameter files. See the trimmed section further down.

    fn double(map: &BTreeMap<String, ParameterValue>, key: &str) -> f64 {
        match map.get(key) {
            Some(ParameterValue::Double(d)) => *d,
            other => panic!("{key} is {other:?}, expected a double"),
        }
    }

    // ------------------------------------------------------ section matching

    #[test]
    fn bare_node_name_sections_match() {
        // The bug that makes `with_parameter_file` unusable: hiroz drops this.
        assert_eq!(
            match_section("controller", "/controller"),
            Some(Specificity::Exact)
        );
        assert_eq!(
            match_section("/controller", "/controller"),
            Some(Specificity::Exact)
        );
        assert_eq!(match_section("controller", "/estimator"), None);
    }

    #[test]
    fn wildcard_sections_match() {
        assert_eq!(
            match_section("/**", "/controller"),
            Some(Specificity::DoubleWildcard)
        );
        assert_eq!(
            match_section("**", "/ns/controller"),
            Some(Specificity::DoubleWildcard)
        );
        assert_eq!(
            match_section("/*", "/controller"),
            Some(Specificity::SingleWildcard)
        );
        // `/*` is one level only.
        assert_eq!(match_section("/*", "/ns/controller"), None);
        assert_eq!(
            match_section("/ns/**", "/ns/controller"),
            Some(Specificity::DoubleWildcard)
        );
        assert_eq!(match_section("/other/**", "/ns/controller"), None);
    }

    #[test]
    fn exact_sections_beat_wildcards_whatever_the_document_order() {
        let yaml = "\
/controller:
  ros__parameters:
    c_kp: 1.0
/**:
  ros__parameters:
    c_kp: 99.0
    only_in_wildcard: 7.0
";
        let m = parse_params_str(yaml, "controller", Path::new("<test>")).unwrap();
        assert_eq!(double(&m, "c_kp"), 1.0, "the exact section must win");
        assert_eq!(double(&m, "only_in_wildcard"), 7.0);

        // Same content, wildcard first: the result must not change.
        let yaml_swapped = "\
/**:
  ros__parameters:
    c_kp: 99.0
/controller:
  ros__parameters:
    c_kp: 1.0
";
        let m = parse_params_str(yaml_swapped, "controller", Path::new("<test>")).unwrap();
        assert_eq!(double(&m, "c_kp"), 1.0);
    }

    #[test]
    fn other_nodes_sections_are_ignored() {
        let yaml = "\
/path_follower:
  ros__parameters:
    k_path: 3.0
";
        let m = parse_params_str(yaml, "controller", Path::new("<test>")).unwrap();
        assert!(m.is_empty());
    }

    #[test]
    fn namespaced_nesting_resolves() {
        let yaml = "\
/uav1:
  controller:
    ros__parameters:
      c_kp: 4.0
";
        let m = parse_params_str(yaml, "/uav1/controller", Path::new("<test>")).unwrap();
        assert_eq!(double(&m, "c_kp"), 4.0);
        let m = parse_params_str(yaml, "controller", Path::new("<test>")).unwrap();
        assert!(m.is_empty(), "a namespaced section must not match the root");
    }

    #[test]
    fn nested_maps_flatten_with_dots() {
        let yaml = "\
/controller:
  ros__parameters:
    qos_overrides:
      /tf:
        publisher:
          depth: 10
";
        let m = parse_params_str(yaml, "controller", Path::new("<test>")).unwrap();
        assert_eq!(
            m.get("qos_overrides./tf.publisher.depth"),
            Some(&ParameterValue::Integer(10))
        );
    }

    // ------------------------------------- the real repo files (trimmed here)

    // The rosplane copy of this module ends this section with three tests that
    // replay `rosplane_rs/params/anaconda_autopilot_params.yaml` and
    // `estimator.yaml` — the corpus that made the bare-section finding
    // concrete. Those files do not exist in this repository, so the two
    // corpus-replay tests are dropped here and the merge-order test they
    // shared is reproduced below on temporary files instead. Everything the
    // section-matching tests above cover is unchanged.

    // ---------------------------------------------------------- merge order

    /// Later files win, and `-p` overrides beat every file whatever the
    /// command-line order — `rcl` keeps the two lists separate and applies the
    /// files first.
    #[test]
    fn later_files_win_and_p_overrides_beat_every_file() {
        let dir = std::env::temp_dir().join(format!(
            "veloxity-yamlparams-test-{}-{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let first = dir.join("first.yaml");
        let second = dir.join("second.yaml");
        std::fs::write(
            &first,
            "controller:\n  ros__parameters:\n    c_kp: 1.0\n    alt_hz: 20.0\n",
        )
        .unwrap();
        std::fs::write(&second, "controller:\n  ros__parameters:\n    c_kp: 2.0\n").unwrap();

        let overrides = vec![(
            "c_kp".to_string(),
            ParameterValue::Double(std::f64::consts::PI),
        )];
        let merged =
            merged_overrides(&[first.clone(), second.clone()], &overrides, "controller").unwrap();

        assert_eq!(
            merged.get("c_kp"),
            Some(&ParameterValue::Double(std::f64::consts::PI)),
            "a -p override beats every file"
        );
        // Untouched keys still come from the file.
        assert_eq!(merged.get("alt_hz"), Some(&ParameterValue::Double(20.0)));

        // Without the override, the later file wins.
        let merged = merged_overrides(&[first, second], &[], "controller").unwrap();
        assert_eq!(merged.get("c_kp"), Some(&ParameterValue::Double(2.0)));

        std::fs::remove_dir_all(dir).unwrap();
    }

    // -------------------------------------------------------- scalar parsing

    #[test]
    fn scalars_resolve_like_ros() {
        assert_eq!(parse_scalar("true"), ParameterValue::Bool(true));
        assert_eq!(parse_scalar("False"), ParameterValue::Bool(false));
        assert_eq!(parse_scalar("TRUE"), ParameterValue::Bool(true));
        // YAML 1.1 booleans are strings in ROS 2's core-schema resolver.
        assert_eq!(
            parse_scalar("yes"),
            ParameterValue::String("yes".to_string())
        );
        assert_eq!(parse_scalar("100"), ParameterValue::Integer(100));
        assert_eq!(parse_scalar("-3"), ParameterValue::Integer(-3));
        assert_eq!(parse_scalar("100.0"), ParameterValue::Double(100.0));
        assert_eq!(parse_scalar(".5"), ParameterValue::Double(0.5));
        assert_eq!(parse_scalar("-1000000."), ParameterValue::Double(-1e6));
        assert_eq!(parse_scalar("1.0e-3"), ParameterValue::Double(1.0e-3));
        assert_eq!(
            parse_scalar("total_energy"),
            ParameterValue::String("total_energy".to_string())
        );
        // Quoting forces a string, so a numeric-looking name survives.
        assert_eq!(
            parse_scalar("'42'"),
            ParameterValue::String("42".to_string())
        );
        assert_eq!(
            parse_scalar("\"42\""),
            ParameterValue::String("42".to_string())
        );
    }

    #[test]
    fn sequence_literals_resolve() {
        assert_eq!(
            parse_sequence_literal("[1, 2, 3]"),
            Some(ParameterValue::IntegerArray(vec![1, 2, 3]))
        );
        assert_eq!(
            parse_sequence_literal("[1.0, 2, 3]"),
            Some(ParameterValue::DoubleArray(vec![1.0, 2.0, 3.0])),
            "a mixed numeric list widens to double"
        );
        assert_eq!(
            parse_sequence_literal("[true, false]"),
            Some(ParameterValue::BoolArray(vec![true, false]))
        );
        assert_eq!(
            parse_sequence_literal("[a, b]"),
            Some(ParameterValue::StringArray(vec![
                "a".to_string(),
                "b".to_string()
            ]))
        );
        assert_eq!(
            parse_sequence_literal("[]"),
            Some(ParameterValue::StringArray(Vec::new()))
        );
        assert_eq!(parse_sequence_literal("nope"), None);
    }

    #[test]
    fn yaml_sequences_become_typed_arrays() {
        let yaml = "\
/controller:
  ros__parameters:
    ints: [1, 2]
    doubles: [1.5, 2.5]
    strings: [a, b]
    flags: [true, false]
";
        let m = parse_params_str(yaml, "controller", Path::new("<test>")).unwrap();
        assert_eq!(
            m.get("ints"),
            Some(&ParameterValue::IntegerArray(vec![1, 2]))
        );
        assert_eq!(
            m.get("doubles"),
            Some(&ParameterValue::DoubleArray(vec![1.5, 2.5]))
        );
        assert_eq!(
            m.get("strings"),
            Some(&ParameterValue::StringArray(vec![
                "a".to_string(),
                "b".to_string()
            ]))
        );
        assert_eq!(
            m.get("flags"),
            Some(&ParameterValue::BoolArray(vec![true, false]))
        );
    }

    #[test]
    fn missing_file_is_an_error_not_a_silent_empty_map() {
        let err = parse_params_file(Path::new("/nonexistent/params.yaml"), "controller")
            .expect_err("a missing --params-file must not load as empty");
        assert!(matches!(err, YamlParamError::Read { .. }));
    }
}
