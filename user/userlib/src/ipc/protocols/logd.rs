//! Log Daemon Protocol
//!
//! Simple protocol for log distribution.
//!
//! - logd reads kernel logs, broadcasts to registered sinks
//! - Fire and forget: logd never blocks on slow sinks
//! - Sinks send periodic acks for health monitoring (not flow control)

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message};

/// Log daemon protocol
pub struct LogdProtocol;

impl Protocol for LogdProtocol {
    type Request = LogdRequest;
    type Response = LogdEvent;
    const PORT_NAME: &'static [u8] = b"logd";
}

/// Max log record size (matches kernel)
pub const MAX_LOG_RECORD: usize = 512;

// Wire format codes
const CMD_REGISTER: u8 = 1;
const CMD_ACK: u8 = 2;
const EVT_LOG: u8 = 1;
const EVT_OK: u8 = 0;

/// Requests from sink to logd
#[derive(Debug, Clone)]
pub enum LogdRequest {
    /// Register as a log sink
    Register,
    /// Health ack: received count, processed count
    Ack { received: u64, processed: u64 },
}

impl Message for LogdRequest {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            LogdRequest::Register => {
                buf[0] = CMD_REGISTER;
                Ok(1)
            }
            LogdRequest::Ack { received, processed } => {
                if buf.len() < 17 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_ACK;
                buf[1..9].copy_from_slice(&received.to_le_bytes());
                buf[9..17].copy_from_slice(&processed.to_le_bytes());
                Ok(17)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }
        match buf[0] {
            CMD_REGISTER => Ok((LogdRequest::Register, 1)),
            CMD_ACK => {
                if buf.len() < 17 {
                    return Err(IpcError::Truncated);
                }
                let received = u64::from_le_bytes(buf[1..9].try_into().unwrap());
                let processed = u64::from_le_bytes(buf[9..17].try_into().unwrap());
                Ok((LogdRequest::Ack { received, processed }, 17))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            LogdRequest::Register => 1,
            LogdRequest::Ack { .. } => 17,
        }
    }
}

/// Events from logd to sink (fire and forget)
#[derive(Debug, Clone)]
pub enum LogdEvent {
    /// Registration confirmed
    Ok,
    /// Log record (raw binary, same format as kernel ring)
    Log { data: [u8; MAX_LOG_RECORD], len: u16 },
}

impl LogdEvent {
    /// Create log event from slice
    pub fn log(data: &[u8]) -> Self {
        let mut buf = [0u8; MAX_LOG_RECORD];
        let len = data.len().min(MAX_LOG_RECORD);
        buf[..len].copy_from_slice(&data[..len]);
        LogdEvent::Log { data: buf, len: len as u16 }
    }
}

impl Message for LogdEvent {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            LogdEvent::Ok => {
                buf[0] = EVT_OK;
                Ok(1)
            }
            LogdEvent::Log { data, len } => {
                let total = 3 + *len as usize;
                if buf.len() < total {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = EVT_LOG;
                buf[1] = (*len & 0xFF) as u8;
                buf[2] = (*len >> 8) as u8;
                buf[3..total].copy_from_slice(&data[..*len as usize]);
                Ok(total)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }
        match buf[0] {
            EVT_OK => Ok((LogdEvent::Ok, 1)),
            EVT_LOG => {
                if buf.len() < 3 {
                    return Err(IpcError::Truncated);
                }
                let len = (buf[1] as u16) | ((buf[2] as u16) << 8);
                let total = 3 + len as usize;
                if buf.len() < total {
                    return Err(IpcError::Truncated);
                }
                let mut data = [0u8; MAX_LOG_RECORD];
                data[..len as usize].copy_from_slice(&buf[3..total]);
                Ok((LogdEvent::Log { data, len }, total))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            LogdEvent::Ok => 1,
            LogdEvent::Log { len, .. } => 3 + *len as usize,
        }
    }
}
