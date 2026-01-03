//! Low-level Channel Wrapper
//!
//! Provides a safe wrapper around raw channel syscalls with proper
//! error handling and automatic cleanup.

use super::error::{IpcError, IpcResult};
use super::MAX_MESSAGE_SIZE;
use crate::syscall;

/// Channel state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    /// Channel is connected and usable
    Connected,
    /// Channel has been closed
    Closed,
}

/// Low-level channel wrapper with automatic cleanup
pub struct Channel {
    id: u32,
    state: ChannelState,
}

impl Channel {
    /// Create channel from raw ID (takes ownership)
    ///
    /// # Safety
    /// Caller must ensure the channel ID is valid and not already wrapped.
    pub fn from_raw(id: u32) -> Self {
        Self {
            id,
            state: ChannelState::Connected,
        }
    }

    /// Get the raw channel ID
    pub fn id(&self) -> u32 {
        self.id
    }

    /// Get channel state
    pub fn state(&self) -> ChannelState {
        self.state
    }

    /// Check if channel is connected
    pub fn is_connected(&self) -> bool {
        self.state == ChannelState::Connected
    }

    /// Send raw bytes (non-blocking)
    pub fn send_raw(&self, data: &[u8]) -> IpcResult<()> {
        if self.state != ChannelState::Connected {
            return Err(IpcError::ChannelClosed);
        }

        if data.len() > MAX_MESSAGE_SIZE {
            return Err(IpcError::MessageTooLarge);
        }

        let result = syscall::send(self.id, data);
        if result < 0 {
            Err(IpcError::from_errno(result as i64))
        } else {
            Ok(())
        }
    }

    /// Receive raw bytes (blocking)
    ///
    /// Returns the number of bytes received, or error.
    /// A return of 0 bytes indicates the channel was closed by peer.
    pub fn receive_raw(&mut self, buf: &mut [u8]) -> IpcResult<usize> {
        if self.state != ChannelState::Connected {
            return Err(IpcError::ChannelClosed);
        }

        let result = syscall::receive(self.id, buf);

        if result < 0 {
            let err = IpcError::from_errno(result as i64);
            if err.is_fatal() {
                self.state = ChannelState::Closed;
            }
            Err(err)
        } else if result == 0 {
            // Peer closed the channel
            self.state = ChannelState::Closed;
            Err(IpcError::ChannelClosed)
        } else {
            Ok(result as usize)
        }
    }

    /// Receive raw bytes (non-blocking)
    ///
    /// Returns WouldBlock if no data available.
    pub fn receive_raw_nonblock(&mut self, buf: &mut [u8]) -> IpcResult<usize> {
        if self.state != ChannelState::Connected {
            return Err(IpcError::ChannelClosed);
        }

        let result = syscall::receive_nonblock(self.id, buf);

        if result < 0 {
            let err = IpcError::from_errno(result as i64);
            if err.is_fatal() {
                self.state = ChannelState::Closed;
            }
            Err(err)
        } else if result == 0 {
            self.state = ChannelState::Closed;
            Err(IpcError::ChannelClosed)
        } else {
            Ok(result as usize)
        }
    }

    /// Receive raw bytes with timeout
    ///
    /// Returns Timeout if no data received within timeout_ms.
    pub fn receive_raw_timeout(&mut self, buf: &mut [u8], timeout_ms: u32) -> IpcResult<usize> {
        if self.state != ChannelState::Connected {
            return Err(IpcError::ChannelClosed);
        }

        let result = syscall::receive_timeout(self.id, buf, timeout_ms);

        if result < 0 {
            let err = IpcError::from_errno(result as i64);
            if err.is_fatal() {
                self.state = ChannelState::Closed;
            }
            Err(err)
        } else if result == 0 {
            self.state = ChannelState::Closed;
            Err(IpcError::ChannelClosed)
        } else {
            Ok(result as usize)
        }
    }

    /// Close the channel explicitly
    pub fn close(&mut self) {
        if self.state == ChannelState::Connected {
            syscall::close(self.id);
            self.state = ChannelState::Closed;
        }
    }

    /// Consume self and return raw ID without closing
    ///
    /// Use this when transferring ownership to another abstraction.
    pub fn into_raw(self) -> u32 {
        let id = self.id;
        core::mem::forget(self); // Don't run Drop
        id
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        self.close();
    }
}

/// Create a pair of connected channels
pub fn create_pair() -> IpcResult<(Channel, Channel)> {
    let result = syscall::channel_create();
    if result < 0 {
        return Err(IpcError::from_errno(result));
    }

    let ch1 = (result & 0xFFFFFFFF) as u32;
    let ch2 = ((result >> 32) & 0xFFFFFFFF) as u32;

    Ok((Channel::from_raw(ch1), Channel::from_raw(ch2)))
}
