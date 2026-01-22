//! Console Handle Object
//!
//! Handles for stdin, stdout, stderr console I/O.

use super::traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
use crate::kernel::task::TaskId;

/// Console type (input or output)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleType {
    /// Standard input (stdin)
    In,
    /// Standard output (stdout)
    Out,
    /// Standard error (stderr)
    Err,
    /// Null device (/dev/null)
    Null,
}

/// Console object for handle system
#[derive(Clone, Copy, Debug)]
pub struct ConsoleObject {
    /// Type of console
    pub console_type: ConsoleType,
}

impl ConsoleObject {
    /// Create stdin handle object
    pub const fn stdin() -> Self {
        Self { console_type: ConsoleType::In }
    }

    /// Create stdout handle object
    pub const fn stdout() -> Self {
        Self { console_type: ConsoleType::Out }
    }

    /// Create stderr handle object
    pub const fn stderr() -> Self {
        Self { console_type: ConsoleType::Err }
    }

    /// Create null handle object
    pub const fn null() -> Self {
        Self { console_type: ConsoleType::Null }
    }

    /// Check if this is readable
    pub fn is_readable(&self) -> bool {
        matches!(self.console_type, ConsoleType::In | ConsoleType::Null)
    }

    /// Check if this is writable
    pub fn is_writable(&self) -> bool {
        matches!(self.console_type, ConsoleType::Out | ConsoleType::Err | ConsoleType::Null)
    }
}

impl Waitable for ConsoleObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match filter {
            WaitFilter::Readable => {
                if self.console_type == ConsoleType::In {
                    // Check if UART has input available
                    if crate::platform::mt7988::uart::rx_buffer_has_data() {
                        Some(WaitResult::new(
                            super::Handle::INVALID,
                            WaitFilter::Readable,
                            1, // At least 1 byte available
                        ))
                    } else {
                        None
                    }
                } else if self.console_type == ConsoleType::Null {
                    // Null always returns EOF immediately
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Closed, // EOF
                        0,
                    ))
                } else {
                    None // stdout/stderr not readable
                }
            }
            WaitFilter::Writable => {
                if self.is_writable() {
                    // Console is always writable (buffered)
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Writable,
                        256, // Buffer space available
                    ))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        if self.console_type == ConsoleType::In {
            // Register with UART for input notification
            crate::platform::mt7988::uart::block_for_input(task_id);
        }
    }

    fn unregister_waker(&mut self, _task_id: TaskId) {
        if self.console_type == ConsoleType::In {
            crate::platform::mt7988::uart::clear_blocked();
        }
    }
}

impl Closable for ConsoleObject {
    fn close(&self, _owner_pid: u32) -> CloseAction {
        // Console handles don't need cleanup
        CloseAction::None
    }
}
