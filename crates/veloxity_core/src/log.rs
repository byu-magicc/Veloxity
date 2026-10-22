use core::cell::RefCell;
use core::fmt;

pub mod drain;

// WARNING: Critical Section forces interrupts to wait while processing
use critical_section::Mutex;

use crate::comm::messages::enums::Severity;

// --- Configuration ---
const LOG_QUEUE_SIZE: usize = 16;
const MAX_LOG_LEN: usize = 50; // Matches MAVLink STATUSTEXT length

// --- Data Structures ---

/// A minimal fixed-capacity string buffer (Replaces heapless::String)
#[derive(Clone, Copy)]
pub struct LogString {
    buffer: [u8; MAX_LOG_LEN],
    len: usize,
}

impl LogString {
    pub const fn new() -> Self {
        Self {
            buffer: [0; MAX_LOG_LEN],
            len: 0,
        }
    }

    pub fn as_str(&self) -> &str {
        // write_str only stores whole characters, so this never falls back.
        core::str::from_utf8(&self.buffer[..self.len]).unwrap_or("")
    }
}

impl fmt::Write for LogString {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let remaining = MAX_LOG_LEN - self.len;
        let mut copy_len = bytes.len().min(remaining);
        // Truncate on a character boundary so the buffer stays valid UTF-8.
        while !s.is_char_boundary(copy_len) {
            copy_len -= 1;
        }

        // Copy data
        self.buffer[self.len..self.len + copy_len].copy_from_slice(&bytes[..copy_len]);
        self.len += copy_len;

        if bytes.len() > remaining {
            return Err(fmt::Error); // Indicate truncation
        }
        Ok(())
    }
}

/// A combined entry holding severity and text
#[derive(Clone, Copy)]
pub struct LogEntry {
    pub severity: Severity,
    pub message: LogString,
}

impl LogEntry {
    pub const fn empty() -> Self {
        Self {
            // Keep this aligned with the core communication message severity surface.
            severity: Severity::Info,
            message: LogString {
                buffer: [0; MAX_LOG_LEN],
                len: 0,
            },
        }
    }
}

/// A minimal Ring Buffer (Replaces heapless::Deque)
struct LogQueue {
    storage: [LogEntry; LOG_QUEUE_SIZE],
    head: usize, // Write index
    tail: usize, // Read index
    full: bool,
}

impl LogQueue {
    const fn new() -> Self {
        Self {
            storage: [LogEntry::empty(); LOG_QUEUE_SIZE],
            head: 0,
            tail: 0,
            full: false,
        }
    }

    fn push(&mut self, entry: LogEntry) {
        self.storage[self.head] = entry;
        self.head = (self.head + 1) % LOG_QUEUE_SIZE;

        if self.full {
            // If full, head bumped into tail, so move tail (overwrite oldest)
            self.tail = (self.tail + 1) % LOG_QUEUE_SIZE;
        }

        self.full = self.head == self.tail;
    }

    fn pop(&mut self) -> Option<LogEntry> {
        if !self.full && self.head == self.tail {
            return None; // Empty
        }

        let entry = self.storage[self.tail];
        self.tail = (self.tail + 1) % LOG_QUEUE_SIZE;
        self.full = false;
        Some(entry)
    }
}

// --- Global State ---

static LOG_QUEUE: Mutex<RefCell<LogQueue>> = Mutex::new(RefCell::new(LogQueue::new()));

// --- Public API ---

pub struct Logger;

impl Logger {
    pub fn log(severity: Severity, args: fmt::Arguments) {
        critical_section::with(|cs| {
            let mut queue = LOG_QUEUE.borrow_ref_mut(cs);

            let mut entry = LogEntry::empty();
            entry.severity = severity;

            // Write the formatted string into our buffer
            // We ignore errors (truncation) to ensure we always log something
            let _ = fmt::Write::write_fmt(&mut entry.message, args);

            queue.push(entry);
        });
    }

    // Keep variants aligned with the core communication message severity surface.
    pub fn info(args: fmt::Arguments) {
        Self::log(Severity::Info, args);
    }
    pub fn warn(args: fmt::Arguments) {
        Self::log(Severity::Warning, args);
    }
    pub fn error(args: fmt::Arguments) {
        Self::log(Severity::Error, args);
    }
    pub fn debug(args: fmt::Arguments) {
        Self::log(Severity::Debug, args);
    }

    /// Called by Main Loop to drain queue
    pub fn pop() -> Option<LogEntry> {
        critical_section::with(|cs| LOG_QUEUE.borrow_ref_mut(cs).pop())
    }
}

// --- Macros ---
// These allow you to use log_info!("val: {}", x) anywhere in your code.
// The Macros can generally be used either by placing
//  `use crate::log_<info, warn, error, or debug>;`
// at the top of the crate, then using `log_<info, warn, error, or debug>!("{}", var);`,
// or by using `crate::log_<info, warn, error, or debug>!("{}", var);` directly.

#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => { $crate::log::Logger::info(format_args!($($arg)*)) };
}
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => { $crate::log::Logger::warn(format_args!($($arg)*)) };
}
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => { $crate::log::Logger::error(format_args!($($arg)*)) };
}
// Add debug macro if you want it exposed
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => { $crate::log::Logger::debug(format_args!($($arg)*))};
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;

    #[test]
    fn truncation_keeps_whole_characters() {
        // 'é' is two bytes; after "a" the 50-byte buffer has one byte left for the last 'é'.
        let mut message = LogString::new();
        message.write_str("a").unwrap();
        let mut result = Ok(());
        while result.is_ok() {
            result = message.write_str("é");
        }
        assert_eq!(message.len, MAX_LOG_LEN - 1);
        assert_eq!(message.as_str().len(), message.len);
        assert!(message.as_str()[1..].chars().all(|c| c == 'é'));
    }
}
