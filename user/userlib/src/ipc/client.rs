//! IPC Client
//!
//! Generic client implementation for connecting to IPC services.

use super::channel::Channel;
use super::error::{IpcError, IpcResult};
use super::protocol::{Message, Protocol};
use super::DEFAULT_TIMEOUT_MS;
use crate::syscall;

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientState {
    /// Not connected
    Disconnected,
    /// Connected and ready
    Connected,
    /// Connection failed or was lost
    Failed,
}

/// Generic IPC client
///
/// Provides type-safe request/response communication with a server
/// implementing the specified protocol.
pub struct Client<P: Protocol> {
    channel: Channel,
    state: ClientState,
    /// Server's PID (if handshake provides it)
    server_pid: Option<u32>,
    /// Timeout for operations (milliseconds)
    timeout_ms: u32,
    /// Phantom for protocol type
    _protocol: core::marker::PhantomData<P>,
}

impl<P: Protocol> Client<P> {
    /// Connect to a service by port name
    ///
    /// Uses the protocol's PORT_NAME to connect.
    pub fn connect() -> IpcResult<Self> {
        Self::connect_with_timeout(DEFAULT_TIMEOUT_MS)
    }

    /// Connect with custom timeout
    pub fn connect_with_timeout(timeout_ms: u32) -> IpcResult<Self> {
        let result = syscall::port_connect(P::PORT_NAME);
        if result < 0 {
            return Err(IpcError::from_errno(result));
        }

        let channel = Channel::from_raw(result as u32);

        Ok(Self {
            channel,
            state: ClientState::Connected,
            server_pid: None,
            timeout_ms,
            _protocol: core::marker::PhantomData,
        })
    }

    /// Connect using an existing channel
    ///
    /// Useful when the connection was established elsewhere (e.g., devd).
    pub fn from_channel(channel: Channel) -> Self {
        Self {
            channel,
            state: ClientState::Connected,
            server_pid: None,
            timeout_ms: DEFAULT_TIMEOUT_MS,
            _protocol: core::marker::PhantomData,
        }
    }

    /// Set timeout for operations
    pub fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    /// Get the current timeout
    pub fn timeout(&self) -> u32 {
        self.timeout_ms
    }

    /// Get client state
    pub fn state(&self) -> ClientState {
        self.state
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.state == ClientState::Connected && self.channel.is_connected()
    }

    /// Get server PID (if known from handshake)
    pub fn server_pid(&self) -> Option<u32> {
        self.server_pid
    }

    /// Set server PID (for protocols that exchange PIDs)
    pub fn set_server_pid(&mut self, pid: u32) {
        self.server_pid = Some(pid);
    }

    /// Get the underlying channel ID (for advanced use)
    pub fn channel_id(&self) -> u32 {
        self.channel.id()
    }

    /// Send a request and wait for response
    pub fn request(&mut self, request: &P::Request) -> IpcResult<P::Response> {
        self.request_with_timeout(request, self.timeout_ms)
    }

    /// Send a request with custom timeout
    pub fn request_with_timeout(
        &mut self,
        request: &P::Request,
        timeout_ms: u32,
    ) -> IpcResult<P::Response> {
        if self.state != ClientState::Connected {
            return Err(IpcError::NotConnected);
        }

        // Serialize request
        let mut send_buf = [0u8; 576];
        let send_len = request.serialize(&mut send_buf)?;

        // Send request
        self.channel.send_raw(&send_buf[..send_len])?;

        // Wait for response
        let mut recv_buf = [0u8; 576];
        let recv_len = if timeout_ms == 0 {
            self.channel.receive_raw(&mut recv_buf)?
        } else {
            match self.channel.receive_raw_timeout(&mut recv_buf, timeout_ms) {
                Ok(len) => len,
                Err(e) => {
                    if e.is_fatal() {
                        self.state = ClientState::Failed;
                    }
                    return Err(e);
                }
            }
        };

        // Deserialize response
        let (response, _) = P::Response::deserialize(&recv_buf[..recv_len])?;

        Ok(response)
    }

    /// Send a request without waiting for response (fire-and-forget)
    pub fn send(&mut self, request: &P::Request) -> IpcResult<()> {
        if self.state != ClientState::Connected {
            return Err(IpcError::NotConnected);
        }

        let mut buf = [0u8; 576];
        let len = request.serialize(&mut buf)?;
        self.channel.send_raw(&buf[..len])
    }

    /// Wait for a response (after send)
    pub fn receive(&mut self) -> IpcResult<P::Response> {
        self.receive_with_timeout(self.timeout_ms)
    }

    /// Wait for a response with custom timeout
    pub fn receive_with_timeout(&mut self, timeout_ms: u32) -> IpcResult<P::Response> {
        if self.state != ClientState::Connected {
            return Err(IpcError::NotConnected);
        }

        let mut buf = [0u8; 576];
        let len = if timeout_ms == 0 {
            self.channel.receive_raw(&mut buf)?
        } else {
            self.channel.receive_raw_timeout(&mut buf, timeout_ms)?
        };

        let (response, _) = P::Response::deserialize(&buf[..len])?;
        Ok(response)
    }

    /// Perform handshake to exchange PIDs
    ///
    /// Many protocols require exchanging PIDs so the server can grant
    /// shared memory access. Call this after connect() if the protocol
    /// requires it.
    pub fn handshake_pid(&mut self) -> IpcResult<u32> {
        let my_pid = syscall::getpid();

        // Send our PID
        let mut buf = [0u8; 4];
        buf.copy_from_slice(&(my_pid as u32).to_le_bytes());
        self.channel.send_raw(&buf)?;

        // Receive server PID
        let len = self.channel.receive_raw_timeout(&mut buf, self.timeout_ms)?;
        if len < 4 {
            return Err(IpcError::HandshakeFailed);
        }

        let server_pid = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        self.server_pid = Some(server_pid);

        Ok(server_pid)
    }

    /// Close the connection
    pub fn close(&mut self) {
        self.channel.close();
        self.state = ClientState::Disconnected;
    }
}

impl<P: Protocol> Drop for Client<P> {
    fn drop(&mut self) {
        // Channel's Drop will close it
    }
}

/// Builder for Client with options
pub struct ClientBuilder<P: Protocol> {
    timeout_ms: u32,
    handshake: bool,
    _protocol: core::marker::PhantomData<P>,
}

impl<P: Protocol> ClientBuilder<P> {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            timeout_ms: DEFAULT_TIMEOUT_MS,
            handshake: false,
            _protocol: core::marker::PhantomData,
        }
    }

    /// Set timeout for operations
    pub fn timeout(mut self, timeout_ms: u32) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Enable PID handshake on connect
    pub fn with_handshake(mut self) -> Self {
        self.handshake = true;
        self
    }

    /// Connect and optionally perform handshake
    pub fn connect(self) -> IpcResult<Client<P>> {
        let mut client = Client::<P>::connect_with_timeout(self.timeout_ms)?;

        if self.handshake {
            client.handshake_pid()?;
        }

        Ok(client)
    }
}

impl<P: Protocol> Default for ClientBuilder<P> {
    fn default() -> Self {
        Self::new()
    }
}
