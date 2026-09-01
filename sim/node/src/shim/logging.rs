//! rcutils-shaped logging on top of `tracing`.
//!
//! Sibling copies: `rosplane_rs/rosplane_nodes/src/shim/logging.rs` — the
//! original this file was copied from verbatim — and
//! `roscopter_rs/roscopter_nodes/src/shim/logging.rs`. The three live in
//! separate repositories and nothing links them, so a fix here has to be
//! made in all three or they drift apart.
//!
//! Hiroz logs through `tracing` and installs **no** subscriber, so a node that
//! does nothing here is completely silent — including hiroz's own warnings. We
//! install one, formatted exactly like `rcutils`' default so that the flight
//! scripts' log greps (`wait_for_log`, `tools/flight_metrics.py`) keep working
//! against Rust nodes:
//!
//! ```text
//! [INFO] [1756742400.123456789] [controller]: Using total energy control.
//! ```
//!
//! Everything goes to **stderr**, as in rclcpp.
//!
//! Two filter rules are always applied on top of whatever level was requested:
//!
//! * `zenoh::net::routing::hat::client::token=off` — that target emits
//!   `ERROR … Unknown token id=` for perfectly ordinary liveliness churn, once
//!   per peer disconnect, so every `ros2` CLI invocation would print a burst of
//!   red herrings during a flight.
//! * `RUST_LOG`, when set, wins over `--log-level`, so an operator can turn one
//!   subsystem up without touching the launch command.

use std::fmt;
use std::sync::OnceLock;

use tracing::{Event, Level, Subscriber};
use tracing_subscriber::EnvFilter;
use tracing_subscriber::fmt::format::Writer;
use tracing_subscriber::fmt::{FmtContext, FormatEvent, FormatFields};
use tracing_subscriber::registry::LookupSpan;

/// The zenoh target that reports benign liveliness churn as ERROR.
pub const NOISY_ZENOH_TARGET: &str = "zenoh::net::routing::hat::client::token";

/// ROS 2 severities, as `--log-level` spells them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    /// rcutils has FATAL; `tracing` does not, so it maps onto ERROR.
    Fatal,
}

impl LogLevel {
    /// Case-insensitive, matching `rcl`'s `--log-level` handling.
    pub fn parse(text: &str) -> Option<Self> {
        match text.to_ascii_lowercase().as_str() {
            "debug" => Some(Self::Debug),
            "info" => Some(Self::Info),
            "warn" | "warning" => Some(Self::Warn),
            "error" => Some(Self::Error),
            "fatal" => Some(Self::Fatal),
            _ => None,
        }
    }

    /// The `tracing` level a port should use when it emits at this severity.
    pub fn tracing_level(self) -> Level {
        match self {
            Self::Debug => Level::DEBUG,
            Self::Info => Level::INFO,
            Self::Warn => Level::WARN,
            Self::Error | Self::Fatal => Level::ERROR,
        }
    }

    fn directive(self) -> &'static str {
        match self {
            Self::Debug => "debug",
            Self::Info => "info",
            Self::Warn => "warn",
            Self::Error | Self::Fatal => "error",
        }
    }
}

/// The node name printed in the third bracket. Set once by [`init`]; empty
/// until then so the formatter can never panic on an un-initialised logger.
static NODE_NAME: OnceLock<String> = OnceLock::new();

/// `[LEVEL] [<sec>.<nanosec>] [<node>]: <message>`
struct RcutilsFormat;

/// Pulls the `message` field out of an event; other fields are appended as
/// `key=value`, which `tracing`-native code (hiroz) uses and rcutils has no
/// equivalent for.
#[derive(Default)]
struct MessageVisitor {
    message: String,
    fields: String,
}

impl tracing::field::Visit for MessageVisitor {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn fmt::Debug) {
        use std::fmt::Write as _;
        if field.name() == "message" {
            let _ = write!(self.message, "{value:?}");
        } else {
            let _ = write!(self.fields, " {}={value:?}", field.name());
        }
    }

    fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
        use std::fmt::Write as _;
        if field.name() == "message" {
            self.message.push_str(value);
        } else {
            let _ = write!(self.fields, " {}={value}", field.name());
        }
    }
}

/// The exact rcutils severity spelling for a `tracing` level. TRACE has no
/// rcutils counterpart and folds into DEBUG, which is where rclcpp would have
/// put the same statement.
fn severity(level: &Level) -> &'static str {
    match *level {
        Level::TRACE | Level::DEBUG => "DEBUG",
        Level::INFO => "INFO",
        Level::WARN => "WARN",
        Level::ERROR => "ERROR",
    }
}

/// Format one stamp the way rcutils does: whole seconds, a dot, then exactly
/// nine digits of nanoseconds.
fn format_stamp(nanos: u128) -> String {
    format!("{}.{:09}", nanos / 1_000_000_000, nanos % 1_000_000_000)
}

impl<S, N> FormatEvent<S, N> for RcutilsFormat
where
    S: Subscriber + for<'a> LookupSpan<'a>,
    N: for<'a> FormatFields<'a> + 'static,
{
    fn format_event(
        &self,
        _ctx: &FmtContext<'_, S, N>,
        mut writer: Writer<'_>,
        event: &Event<'_>,
    ) -> fmt::Result {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);

        let mut visitor = MessageVisitor::default();
        event.record(&mut visitor);

        writeln!(
            writer,
            "[{}] [{}] [{}]: {}{}",
            severity(event.metadata().level()),
            format_stamp(nanos),
            NODE_NAME.get().map(String::as_str).unwrap_or(""),
            visitor.message,
            visitor.fields,
        )
    }
}

/// Build the filter: `RUST_LOG` if set, else the requested level, else `info`;
/// with the noisy zenoh target forced off in every case.
fn filter(level: Option<LogLevel>) -> EnvFilter {
    let base = match std::env::var("RUST_LOG") {
        Ok(v) if !v.trim().is_empty() => v,
        _ => level.unwrap_or(LogLevel::Info).directive().to_string(),
    };
    let filter = EnvFilter::new(base);
    match format!("{NOISY_ZENOH_TARGET}=off").parse() {
        Ok(d) => filter.add_directive(d),
        Err(_) => filter,
    }
}

/// Install the process-wide subscriber. Idempotent: a second call (a test, a
/// node that bootstraps twice) is a no-op rather than a panic, and the node
/// name from the first call stands.
pub fn init(node_name: &str, level: Option<LogLevel>) {
    let _ = NODE_NAME.set(node_name.to_string());
    // NOTE: no `.with_max_level()` here. It does not narrow the env filter, it
    // *replaces* it (`SubscriberBuilder::with_max_level` returns a builder
    // parameterised on `LevelFilter`), which would silently discard both
    // `RUST_LOG` and the zenoh-token silencing. The level lives in the
    // `EnvFilter` and nowhere else.
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter(level))
        .with_writer(std::io::stderr)
        .event_format(RcutilsFormat)
        .try_init();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_parse_case_insensitively() {
        assert_eq!(LogLevel::parse("debug"), Some(LogLevel::Debug));
        assert_eq!(LogLevel::parse("DEBUG"), Some(LogLevel::Debug));
        assert_eq!(LogLevel::parse("Info"), Some(LogLevel::Info));
        assert_eq!(LogLevel::parse("warn"), Some(LogLevel::Warn));
        assert_eq!(LogLevel::parse("warning"), Some(LogLevel::Warn));
        assert_eq!(LogLevel::parse("error"), Some(LogLevel::Error));
        assert_eq!(LogLevel::parse("fatal"), Some(LogLevel::Fatal));
        assert_eq!(LogLevel::parse("unset"), None);
        assert_eq!(LogLevel::parse(""), None);
    }

    #[test]
    fn fatal_maps_onto_error() {
        assert_eq!(LogLevel::Fatal.tracing_level(), Level::ERROR);
        assert_eq!(LogLevel::Fatal.directive(), "error");
    }

    #[test]
    fn severities_use_the_rcutils_spellings() {
        assert_eq!(severity(&Level::TRACE), "DEBUG");
        assert_eq!(severity(&Level::DEBUG), "DEBUG");
        assert_eq!(severity(&Level::INFO), "INFO");
        assert_eq!(severity(&Level::WARN), "WARN");
        assert_eq!(severity(&Level::ERROR), "ERROR");
    }

    /// rcutils prints nine nanosecond digits, zero-padded — the flight logs'
    /// timestamps are parsed by `tools/flight_metrics.py`, so the shape is a
    /// contract, not cosmetics.
    #[test]
    fn stamps_are_nine_digit_padded() {
        assert_eq!(format_stamp(0), "0.000000000");
        assert_eq!(format_stamp(1_000_000_000), "1.000000000");
        assert_eq!(
            format_stamp(1_756_742_400_123_456_789),
            "1756742400.123456789"
        );
        assert_eq!(
            format_stamp(1_756_742_400_000_000_001),
            "1756742400.000000001"
        );
    }

    #[test]
    fn the_noisy_zenoh_target_is_always_silenced() {
        // Parsing is what would fail if the target string were malformed.
        let directive = format!("{NOISY_ZENOH_TARGET}=off");
        assert!(
            directive
                .parse::<tracing_subscriber::filter::Directive>()
                .is_ok()
        );
        assert_eq!(
            NOISY_ZENOH_TARGET,
            "zenoh::net::routing::hat::client::token"
        );

        // …and the built filter really carries it, at every requested level.
        // `EnvFilter`'s Display lists its directives.
        for level in [None, Some(LogLevel::Debug), Some(LogLevel::Error)] {
            let rendered = filter(level).to_string();
            assert!(
                rendered.contains(NOISY_ZENOH_TARGET),
                "the zenoh directive is missing from {rendered:?}"
            );
        }
    }

    /// Capture-to-buffer writer, so the formatter can be exercised end to end
    /// without installing a global subscriber.
    #[derive(Clone, Default)]
    struct Capture(std::sync::Arc<std::sync::Mutex<Vec<u8>>>);

    impl std::io::Write for Capture {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for Capture {
        type Writer = Self;
        fn make_writer(&'a self) -> Self::Writer {
            self.clone()
        }
    }

    /// The line shape is a contract with the flight scripts
    /// (`tools/flight_run.sh`'s `wait_for_log`, `tools/flight_metrics.py`),
    /// which grep rcutils-formatted output. Pin it end to end.
    #[test]
    fn events_render_in_the_rcutils_line_format() {
        let _ = NODE_NAME.set("controller".to_string());
        let capture = Capture::default();
        let subscriber = tracing_subscriber::fmt()
            .with_writer(capture.clone())
            .event_format(RcutilsFormat)
            .finish();

        tracing::subscriber::with_default(subscriber, || {
            tracing::info!("Using total energy control.");
            tracing::error!(
                "One of the parameters given is not a parameter of the controller node. \
                 Parameter: bogus"
            );
        });

        let out = String::from_utf8(capture.0.lock().unwrap().clone()).unwrap();
        let lines: Vec<&str> = out.lines().collect();
        assert_eq!(lines.len(), 2, "one line per event, got {out:?}");

        assert!(
            lines[0].starts_with("[INFO] ["),
            "unexpected prefix in {:?}",
            lines[0]
        );
        assert!(
            lines[0].ends_with("] [controller]: Using total energy control."),
            "unexpected suffix in {:?}",
            lines[0]
        );
        assert!(lines[1].starts_with("[ERROR] ["));
        assert!(lines[1].contains("] [controller]: One of the parameters given is not"));

        // The stamp between the first two brackets is `<sec>.<9 digits>`.
        let stamp = lines[0]
            .split_once("] [")
            .and_then(|(_, rest)| rest.split_once(']'))
            .map(|(s, _)| s)
            .expect("no stamp field");
        let (sec, nanos) = stamp.split_once('.').expect("stamp has no fraction");
        assert!(sec.parse::<u64>().is_ok(), "seconds not numeric: {stamp:?}");
        assert_eq!(nanos.len(), 9, "nanoseconds not 9 digits: {stamp:?}");
        assert!(nanos.chars().all(|c| c.is_ascii_digit()));
    }

    /// The requested level must reach the filter — the whole reason `init` has
    /// no `.with_max_level()` call (which would replace the filter entirely).
    #[test]
    fn the_requested_level_lands_in_the_filter() {
        // Only meaningful when the environment is not overriding us; RUST_LOG
        // beating `--log-level` is itself the documented behaviour.
        if std::env::var("RUST_LOG").is_ok() {
            return;
        }
        assert!(filter(Some(LogLevel::Debug)).to_string().contains("debug"));
        assert!(filter(Some(LogLevel::Warn)).to_string().contains("warn"));
        assert!(
            filter(None).to_string().contains("info"),
            "the default level is info, as in rclcpp"
        );
    }
}
