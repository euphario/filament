//! Bus Transport - ControlTransport implementations
//!
//! Provides concrete implementations of the `ControlTransport` trait
//! for moving `BusMsg` between parent and child drivers.
//!
//! ## V1: Channel-backed transport
//!
//! Uses the existing Channel (IPC) mechanism. BusMsg at 256 bytes fits
//! within the 576-byte IPC message limit. This is simple and integrates
//! with existing Mux infrastructure.
//!
//! ## Future: Shmem-backed ring
//!
//! A shmem-backed ring with BusMsg-sized slots would avoid kernel
//! involvement per message. The `ControlTransport` trait boundary
//! allows this swap without changing any driver code.

use crate::bus::{BusMsg, BusError, ControlTransport};
use crate::ipc::Channel;

// ============================================================================
// Channel-backed ControlTransport
// ============================================================================

/// ControlTransport implementation using kernel Channels (IPC).
///
/// Each BusMsg is serialized to 256 bytes and sent as a single IPC message.
/// The Channel handle can be registered with Mux for event-driven wakeup.
pub struct ChannelTransport {
    channel: Channel,
}

impl ChannelTransport {
    /// Create from an existing connected Channel.
    pub fn new(channel: Channel) -> Self {
        Self { channel }
    }

    /// Connect to a named port and create transport.
    pub fn connect(port_name: &[u8]) -> Result<Self, BusError> {
        let channel = Channel::connect(port_name).map_err(|_| BusError::LinkDown)?;
        Ok(Self { channel })
    }

    /// Get the underlying channel (for advanced use).
    pub fn channel(&self) -> &Channel {
        &self.channel
    }
}

impl ControlTransport for ChannelTransport {
    fn send(&mut self, msg: &BusMsg) -> Result<(), BusError> {
        let bytes = msg.to_bytes();
        self.channel.send(&bytes).map_err(|_| BusError::LinkDown)
    }

    fn try_recv(&mut self) -> Option<BusMsg> {
        let mut buf = [0u8; 256];
        match self.channel.try_recv(&mut buf) {
            Ok(Some(256)) => Some(BusMsg::from_bytes(&buf)),
            Ok(Some(n)) if n >= 16 => {
                // Partial message — pad remaining with zeros (already zeroed)
                Some(BusMsg::from_bytes(&buf))
            }
            _ => None,
        }
    }

    fn mux_handle(&self) -> crate::syscall::Handle {
        self.channel.handle()
    }

    fn is_alive(&self) -> bool {
        self.channel.state() == crate::ipc::ChannelState::Open
    }
}
