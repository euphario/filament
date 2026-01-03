#![allow(dead_code)]  // State machine variants reserved for future use

//! USB Mass Storage BOT (Bulk-Only Transport) State Machine
//!
//! Based on:
//! - USB Mass Storage Class Bulk-Only Transport Spec v1.0
//! - Linux kernel drivers/usb/storage/transport.c
//! - EDK2/TianoCore UsbMassBot.c
//!
//! ## State Machine Overview
//!
//! ```text
//!                          +-----------+
//!                          |  Unknown  |
//!                          +-----+-----+
//!                                |
//!                                v (enumerate)
//!     +------------------>+-----------+<------------------+
//!     |                   |   Ready   |                   |
//!     |                   +-----+-----+                   |
//!     |                         |                         |
//!     |                         v (start command)         |
//!     |                   +-----------+                   |
//!     |         +-------->| SendCBW   |                   |
//!     |         |         +-----+-----+                   |
//!     |         |               |                         |
//!     |         |    +----------+----------+              |
//!     |         |    | success      | STALL               |
//!     |         |    v              v                     |
//!     |         |  +-----------+  +-----------+           |
//!     |         |  | DataPhase |  | Recovery  |--> ResetRecovery
//!     |         |  +-----+-----+  +-----------+           |
//!     |         |        |                                |
//!     |         |   +----+----+                           |
//!     |         |   | ok  | STALL                         |
//!     |         |   v     v                               |
//!     |         | +-----------+                           |
//!     |         | | RecvCSW   |----> (STALL) ClearHaltIn  |
//!     |         | +-----+-----+                           |
//!     |         |       |                                 |
//!     |    retry|  +----+----+----+                       |
//!     |         |  | ok | fail| phase_err                 |
//!     |         |  v    v     v                           |
//!     |         |  |  Error  ResetRecovery                |
//!     +---------+--+          |                           |
//!                             v                           |
//!                       +-----------+                     |
//!                       | BOTReset  |                     |
//!                       +-----+-----+                     |
//!                             |                           |
//!                        +----+----+                      |
//!                        | ok      | fail                 |
//!                        v         v                      |
//!                      Ready   +-----------+              |
//!                              | PortReset |              |
//!                              +-----+-----+              |
//!                                    |                    |
//!                               +----+----+               |
//!                               | ok      | fail          |
//!                               v         v               |
//!                             Ready    Disconnected       |
//! ```

use core::fmt;

// =============================================================================
// Transfer Result (from Linux kernel)
// =============================================================================

/// Result of a single USB transfer operation (based on Linux USB_STOR_XFER_*)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XferResult {
    /// Transfer completed successfully
    Good,
    /// Transfer completed with less data than expected (short packet)
    Short,
    /// Endpoint stalled - needs halt clearing
    Stalled,
    /// Transfer error (timeout, babble, etc.)
    Error,
    /// Device sent more data than expected (babble)
    Long,
}

// =============================================================================
// Transport Result (from Linux kernel)
// =============================================================================

/// Result of a complete BOT command (CBW + Data + CSW)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportResult {
    /// Command completed successfully
    Good,
    /// Command failed (CSW status = 1) - SCSI error, check sense
    Failed,
    /// Transport error - needs reset recovery
    Error,
    /// No sense data available
    NoSense,
}

// =============================================================================
// Device State
// =============================================================================

/// High-level state of the USB mass storage device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    /// Device not yet enumerated
    Unknown,
    /// Device ready for commands
    Ready,
    /// Device in error state, may recover
    Error,
    /// Device needs BOT reset
    NeedsBotReset,
    /// Device needs USB port reset
    NeedsPortReset,
    /// Device disconnected or unrecoverable
    Disconnected,
}

impl Default for DeviceState {
    fn default() -> Self {
        DeviceState::Unknown
    }
}

// =============================================================================
// Transfer Phase State
// =============================================================================

/// Current phase of BOT transfer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferPhase {
    /// No transfer in progress
    Idle,
    /// Sending Command Block Wrapper
    SendingCBW,
    /// Data IN phase (device -> host)
    DataIn,
    /// Data OUT phase (host -> device)
    DataOut,
    /// Receiving Command Status Wrapper
    ReceivingCSW,
    /// Clearing halt on bulk-in endpoint
    ClearingHaltIn,
    /// Clearing halt on bulk-out endpoint
    ClearingHaltOut,
}

impl Default for TransferPhase {
    fn default() -> Self {
        TransferPhase::Idle
    }
}

// =============================================================================
// Recovery State
// =============================================================================

/// Current recovery operation in progress
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryState {
    /// No recovery in progress
    None,
    /// Clearing endpoint halt (step 1)
    ClearingHalt,
    /// BOT class reset (step 2)
    BotReset,
    /// USB port reset (step 3 - escalation)
    PortReset,
    /// Re-enumerating device after port reset
    ReEnumerating,
}

impl Default for RecoveryState {
    fn default() -> Self {
        RecoveryState::None
    }
}

// =============================================================================
// CSW Status (from USB BOT spec)
// =============================================================================

/// Command Status Wrapper status values
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CswStatus {
    /// Command passed
    Passed = 0,
    /// Command failed (check SCSI sense)
    Failed = 1,
    /// Phase error - requires reset recovery
    PhaseError = 2,
}

impl From<u8> for CswStatus {
    fn from(val: u8) -> Self {
        match val {
            0 => CswStatus::Passed,
            1 => CswStatus::Failed,
            _ => CswStatus::PhaseError,
        }
    }
}

// =============================================================================
// BOT State Machine
// =============================================================================

/// USB BOT Protocol State Machine
///
/// Tracks device state, transfer phase, and recovery state.
/// Implements the error handling from USB BOT spec section 5.3.
pub struct BotStateMachine {
    /// High-level device state
    pub device_state: DeviceState,
    /// Current transfer phase
    pub transfer_phase: TransferPhase,
    /// Recovery state (if in recovery)
    pub recovery_state: RecoveryState,
    /// Current command tag (incremented per command)
    pub tag: u32,
    /// Consecutive error count
    pub error_count: u32,
    /// Maximum errors before escalating recovery
    pub max_errors: u32,
    /// CSW retry count for current command
    pub csw_retries: u32,
    /// Maximum CSW retries before reset
    pub max_csw_retries: u32,
}

impl Default for BotStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl BotStateMachine {
    /// Create new state machine
    pub const fn new() -> Self {
        Self {
            device_state: DeviceState::Unknown,
            transfer_phase: TransferPhase::Idle,
            recovery_state: RecoveryState::None,
            tag: 1,
            error_count: 0,
            max_errors: 3,
            csw_retries: 0,
            max_csw_retries: 3,
        }
    }

    /// Device enumerated successfully
    pub fn on_enumerated(&mut self) {
        self.device_state = DeviceState::Ready;
        self.transfer_phase = TransferPhase::Idle;
        self.recovery_state = RecoveryState::None;
        self.error_count = 0;
    }

    /// Get next tag for CBW (incremented even on errors per BOT spec)
    pub fn next_tag(&mut self) -> u32 {
        let tag = self.tag;
        self.tag = self.tag.wrapping_add(1);
        if self.tag == 0 {
            self.tag = 1; // Tag 0 is reserved
        }
        tag
    }

    /// Check if device is ready for commands
    pub fn is_ready(&self) -> bool {
        self.device_state == DeviceState::Ready &&
        self.transfer_phase == TransferPhase::Idle &&
        self.recovery_state == RecoveryState::None
    }

    /// Check if recovery is in progress
    pub fn in_recovery(&self) -> bool {
        self.recovery_state != RecoveryState::None
    }

    // =========================================================================
    // Transfer Phase Transitions
    // =========================================================================

    /// Start sending CBW
    pub fn start_command(&mut self) {
        self.transfer_phase = TransferPhase::SendingCBW;
        self.csw_retries = 0;
    }

    /// CBW sent successfully, move to data phase or CSW
    pub fn cbw_sent(&mut self, has_data_in: bool, has_data_out: bool) {
        if has_data_in {
            self.transfer_phase = TransferPhase::DataIn;
        } else if has_data_out {
            self.transfer_phase = TransferPhase::DataOut;
        } else {
            self.transfer_phase = TransferPhase::ReceivingCSW;
        }
    }

    /// CBW send failed with STALL
    pub fn cbw_stalled(&mut self) -> RecoveryAction {
        // Per BOT spec 5.3.1: Host shall respond with Reset Recovery
        self.transfer_phase = TransferPhase::Idle;
        self.start_recovery(RecoveryState::BotReset)
    }

    /// Data phase completed successfully
    pub fn data_complete(&mut self) {
        self.transfer_phase = TransferPhase::ReceivingCSW;
    }

    /// Data phase failed with STALL
    pub fn data_stalled(&mut self, is_in: bool) -> RecoveryAction {
        // Per BOT spec: Clear the stalled endpoint, then get CSW
        if is_in {
            self.transfer_phase = TransferPhase::ClearingHaltIn;
        } else {
            self.transfer_phase = TransferPhase::ClearingHaltOut;
        }
        RecoveryAction::ClearHalt { is_bulk_in: is_in }
    }

    /// Halt cleared, continue to CSW
    pub fn halt_cleared(&mut self) {
        self.transfer_phase = TransferPhase::ReceivingCSW;
    }

    /// CSW received and validated
    pub fn csw_received(&mut self, status: CswStatus) -> TransportResult {
        self.transfer_phase = TransferPhase::Idle;
        self.csw_retries = 0;

        match status {
            CswStatus::Passed => {
                self.error_count = 0;
                TransportResult::Good
            }
            CswStatus::Failed => {
                // Command failed but transport OK - caller should check sense
                TransportResult::Failed
            }
            CswStatus::PhaseError => {
                // Phase error - must do reset recovery
                self.device_state = DeviceState::NeedsBotReset;
                TransportResult::Error
            }
        }
    }

    /// CSW receive failed with STALL
    pub fn csw_stalled(&mut self) -> RecoveryAction {
        // Per BOT spec Figure 2: Clear Bulk-In, retry CSW
        self.transfer_phase = TransferPhase::ClearingHaltIn;
        RecoveryAction::ClearHalt { is_bulk_in: true }
    }

    /// CSW was invalid (bad signature or tag)
    pub fn csw_invalid(&mut self) -> RecoveryAction {
        self.csw_retries += 1;
        if self.csw_retries < self.max_csw_retries {
            // Retry receiving CSW
            self.transfer_phase = TransferPhase::ReceivingCSW;
            RecoveryAction::RetryCSW
        } else {
            // Too many retries, need reset
            self.transfer_phase = TransferPhase::Idle;
            self.start_recovery(RecoveryState::BotReset)
        }
    }

    /// Generic transfer error (timeout, etc.)
    pub fn transfer_error(&mut self) -> RecoveryAction {
        self.error_count += 1;
        self.transfer_phase = TransferPhase::Idle;

        if self.error_count >= self.max_errors {
            self.start_recovery(RecoveryState::BotReset)
        } else {
            RecoveryAction::Retry
        }
    }

    // =========================================================================
    // Recovery State Machine
    // =========================================================================

    /// Start recovery at given level
    fn start_recovery(&mut self, level: RecoveryState) -> RecoveryAction {
        self.recovery_state = level;

        match level {
            RecoveryState::None => RecoveryAction::None,
            RecoveryState::ClearingHalt => RecoveryAction::ClearHalt { is_bulk_in: true },
            RecoveryState::BotReset => {
                self.device_state = DeviceState::NeedsBotReset;
                RecoveryAction::BotReset
            }
            RecoveryState::PortReset => {
                self.device_state = DeviceState::NeedsPortReset;
                RecoveryAction::PortReset
            }
            RecoveryState::ReEnumerating => RecoveryAction::ReEnumerate,
        }
    }

    /// BOT reset completed
    pub fn bot_reset_complete(&mut self, success: bool) -> RecoveryAction {
        if success {
            self.recovery_state = RecoveryState::None;
            self.device_state = DeviceState::Ready;
            self.error_count = 0;
            RecoveryAction::None
        } else {
            // Escalate to port reset
            self.start_recovery(RecoveryState::PortReset)
        }
    }

    /// Port reset completed
    pub fn port_reset_complete(&mut self, success: bool) -> RecoveryAction {
        if success {
            // After port reset, device needs re-enumeration
            self.recovery_state = RecoveryState::ReEnumerating;
            RecoveryAction::ReEnumerate
        } else {
            self.recovery_state = RecoveryState::None;
            self.device_state = DeviceState::Disconnected;
            RecoveryAction::GiveUp
        }
    }

    /// Re-enumeration completed
    pub fn reenumeration_complete(&mut self, success: bool) {
        self.recovery_state = RecoveryState::None;
        if success {
            self.device_state = DeviceState::Ready;
            self.error_count = 0;
        } else {
            self.device_state = DeviceState::Disconnected;
        }
    }

    /// Device disconnected
    pub fn on_disconnect(&mut self) {
        self.device_state = DeviceState::Disconnected;
        self.transfer_phase = TransferPhase::Idle;
        self.recovery_state = RecoveryState::None;
    }

    /// Reset state machine for new device
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

// =============================================================================
// Recovery Actions
// =============================================================================

/// Action to take based on state machine transition
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryAction {
    /// No action needed
    None,
    /// Retry the current operation
    Retry,
    /// Retry receiving CSW
    RetryCSW,
    /// Clear endpoint halt
    ClearHalt { is_bulk_in: bool },
    /// Perform BOT class reset (0x21/0xFF)
    BotReset,
    /// Perform USB port reset
    PortReset,
    /// Re-enumerate device
    ReEnumerate,
    /// Device is unrecoverable
    GiveUp,
}

impl fmt::Display for RecoveryAction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RecoveryAction::None => write!(f, "None"),
            RecoveryAction::Retry => write!(f, "Retry"),
            RecoveryAction::RetryCSW => write!(f, "RetryCSW"),
            RecoveryAction::ClearHalt { is_bulk_in } => {
                write!(f, "ClearHalt({})", if *is_bulk_in { "IN" } else { "OUT" })
            }
            RecoveryAction::BotReset => write!(f, "BotReset"),
            RecoveryAction::PortReset => write!(f, "PortReset"),
            RecoveryAction::ReEnumerate => write!(f, "ReEnumerate"),
            RecoveryAction::GiveUp => write!(f, "GiveUp"),
        }
    }
}

// =============================================================================
// Debug Display
// =============================================================================

impl fmt::Display for BotStateMachine {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BOT[{:?}/{:?}/{:?} tag={} err={}]",
            self.device_state,
            self.transfer_phase,
            self.recovery_state,
            self.tag,
            self.error_count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_successful_command() {
        let mut sm = BotStateMachine::new();
        sm.on_enumerated();

        assert!(sm.is_ready());

        // Start command
        sm.start_command();
        assert_eq!(sm.transfer_phase, TransferPhase::SendingCBW);

        // CBW sent, data IN phase
        sm.cbw_sent(true, false);
        assert_eq!(sm.transfer_phase, TransferPhase::DataIn);

        // Data complete
        sm.data_complete();
        assert_eq!(sm.transfer_phase, TransferPhase::ReceivingCSW);

        // CSW received OK
        let result = sm.csw_received(CswStatus::Passed);
        assert_eq!(result, TransportResult::Good);
        assert!(sm.is_ready());
    }

    #[test]
    fn test_stall_recovery() {
        let mut sm = BotStateMachine::new();
        sm.on_enumerated();
        sm.start_command();
        sm.cbw_sent(true, false);

        // Data stalled
        let action = sm.data_stalled(true);
        assert_eq!(action, RecoveryAction::ClearHalt { is_bulk_in: true });

        // Halt cleared
        sm.halt_cleared();
        assert_eq!(sm.transfer_phase, TransferPhase::ReceivingCSW);
    }

    #[test]
    fn test_phase_error_recovery() {
        let mut sm = BotStateMachine::new();
        sm.on_enumerated();
        sm.start_command();
        sm.cbw_sent(false, false);

        // CSW with phase error
        let result = sm.csw_received(CswStatus::PhaseError);
        assert_eq!(result, TransportResult::Error);
        assert_eq!(sm.device_state, DeviceState::NeedsBotReset);
    }
}
