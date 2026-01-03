//! GPIO Protocol
//!
//! Type-safe IPC protocol for controlling GPIO expanders.
//!
//! ## Commands
//!
//! - `SetPin`: Set a GPIO pin high or low
//! - `GetPin`: Read a GPIO pin value
//!
//! ## Example
//!
//! ```rust
//! use userlib::ipc::protocols::GpioClient;
//!
//! let mut gpio = GpioClient::connect()?;
//! gpio.set_pin(11, true)?;  // Enable USB VBUS (pin 11)
//! let value = gpio.get_pin(11)?;
//! ```

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message};

/// GPIO service protocol
pub struct GpioProtocol;

impl Protocol for GpioProtocol {
    type Request = GpioRequest;
    type Response = GpioResponse;
    const PORT_NAME: &'static [u8] = b"gpio";
}

/// GPIO command codes
const GPIO_CMD_SET_PIN: u8 = 1;
const GPIO_CMD_GET_PIN: u8 = 2;

/// GPIO request messages
#[derive(Debug, Clone, Copy)]
pub enum GpioRequest {
    /// Set a GPIO pin to high or low
    SetPin { pin: u8, high: bool },
    /// Read a GPIO pin value
    GetPin { pin: u8 },
}

impl Message for GpioRequest {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            GpioRequest::SetPin { pin, high } => {
                if buf.len() < 3 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = GPIO_CMD_SET_PIN;
                buf[1] = *pin;
                buf[2] = if *high { 1 } else { 0 };
                Ok(3)
            }
            GpioRequest::GetPin { pin } => {
                if buf.len() < 2 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = GPIO_CMD_GET_PIN;
                buf[1] = *pin;
                Ok(2)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            GPIO_CMD_SET_PIN => {
                if buf.len() < 3 {
                    return Err(IpcError::Truncated);
                }
                Ok((
                    GpioRequest::SetPin {
                        pin: buf[1],
                        high: buf[2] != 0,
                    },
                    3,
                ))
            }
            GPIO_CMD_GET_PIN => {
                if buf.len() < 2 {
                    return Err(IpcError::Truncated);
                }
                Ok((GpioRequest::GetPin { pin: buf[1] }, 2))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            GpioRequest::SetPin { .. } => 3,
            GpioRequest::GetPin { .. } => 2,
        }
    }
}

/// GPIO response messages
#[derive(Debug, Clone, Copy)]
pub enum GpioResponse {
    /// Operation succeeded
    Ok,
    /// Pin value (for GetPin)
    PinValue(bool),
    /// Operation failed
    Error(i8),
}

impl Message for GpioResponse {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.is_empty() {
            return Err(IpcError::MessageTooLarge);
        }
        match self {
            GpioResponse::Ok => {
                buf[0] = 0;
                Ok(1)
            }
            GpioResponse::PinValue(high) => {
                buf[0] = if *high { 1 } else { 0 };
                Ok(1)
            }
            GpioResponse::Error(code) => {
                buf[0] = *code as u8;
                Ok(1)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }
        let val = buf[0] as i8;
        if val < 0 {
            Ok((GpioResponse::Error(val), 1))
        } else if val == 0 {
            Ok((GpioResponse::Ok, 1))
        } else {
            Ok((GpioResponse::PinValue(val != 0), 1))
        }
    }

    fn serialized_size(&self) -> usize {
        1
    }
}

/// Convenience client for GPIO operations
pub struct GpioClient {
    inner: super::super::Client<GpioProtocol>,
}

impl GpioClient {
    /// Connect to GPIO driver
    pub fn connect() -> IpcResult<Self> {
        let inner = super::super::Client::<GpioProtocol>::connect()?;
        Ok(Self { inner })
    }

    /// Set a GPIO pin
    pub fn set_pin(&mut self, pin: u8, high: bool) -> IpcResult<()> {
        let request = GpioRequest::SetPin { pin, high };
        let response = self.inner.request(&request)?;
        match response {
            GpioResponse::Ok => Ok(()),
            GpioResponse::Error(e) => Err(IpcError::ServerError(e as i32)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Read a GPIO pin
    pub fn get_pin(&mut self, pin: u8) -> IpcResult<bool> {
        let request = GpioRequest::GetPin { pin };
        let response = self.inner.request(&request)?;
        match response {
            GpioResponse::PinValue(high) => Ok(high),
            GpioResponse::Error(e) => Err(IpcError::ServerError(e as i32)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }
}
