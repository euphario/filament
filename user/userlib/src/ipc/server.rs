//! IPC Server
//!
//! Generic server implementation for providing IPC services.

use super::channel::Channel;
use super::error::{IpcError, IpcResult};
use super::protocol::{Message, Protocol};
use super::DEFAULT_TIMEOUT_MS;
use crate::syscall;

/// Server state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServerState {
    /// Not registered
    Unregistered,
    /// Registered and listening
    Listening,
    /// Server has been stopped
    Stopped,
}

/// Generic IPC server
///
/// Provides type-safe request handling for clients implementing
/// the specified protocol.
pub struct Server<P: Protocol> {
    listen_channel: u32,
    state: ServerState,
    _protocol: core::marker::PhantomData<P>,
}

impl<P: Protocol> Server<P> {
    /// Register a new server on the protocol's port
    pub fn register() -> IpcResult<Self> {
        let result = syscall::port_register(P::PORT_NAME);
        if result < 0 {
            return Err(IpcError::from_errno(result));
        }

        Ok(Self {
            listen_channel: result as u32,
            state: ServerState::Listening,
            _protocol: core::marker::PhantomData,
        })
    }

    /// Check if server is listening
    pub fn is_listening(&self) -> bool {
        self.state == ServerState::Listening
    }

    /// Get server state
    pub fn state(&self) -> ServerState {
        self.state
    }

    /// Get the listen channel ID
    ///
    /// Used for event subscription (IpcReady events)
    pub fn listen_channel(&self) -> u32 {
        self.listen_channel
    }

    /// Accept a new connection
    ///
    /// Blocks until a client connects.
    pub fn accept(&self) -> IpcResult<Connection<P>> {
        if self.state != ServerState::Listening {
            return Err(IpcError::NotConnected);
        }

        loop {
            let result = syscall::port_accept(self.listen_channel);
            if result >= 0 {
                let channel = Channel::from_raw(result as u32);
                return Ok(Connection::new(channel));
            }

            let err = IpcError::from_errno(result as i64);
            if err == IpcError::WouldBlock {
                syscall::yield_now();
                continue;
            }

            return Err(err);
        }
    }

    /// Accept a connection (non-blocking)
    ///
    /// Returns WouldBlock if no connection pending.
    pub fn try_accept(&self) -> IpcResult<Connection<P>> {
        if self.state != ServerState::Listening {
            return Err(IpcError::NotConnected);
        }

        let result = syscall::port_accept(self.listen_channel);
        if result >= 0 {
            let channel = Channel::from_raw(result as u32);
            return Ok(Connection::new(channel));
        }

        Err(IpcError::from_errno(result as i64))
    }

    /// Serve requests in a loop
    ///
    /// This is the main server loop. It accepts connections and handles
    /// requests using the provided handler function.
    ///
    /// The handler receives a request and returns a response.
    pub fn serve<F>(&self, mut handler: F) -> !
    where
        F: FnMut(&P::Request) -> P::Response,
    {
        loop {
            // Accept connection
            let mut conn = match self.accept() {
                Ok(c) => c,
                Err(_) => {
                    syscall::yield_now();
                    continue;
                }
            };

            // Handle requests until client disconnects
            loop {
                match conn.receive() {
                    Ok(request) => {
                        let response = handler(&request);
                        if conn.send(&response).is_err() {
                            break; // Client disconnected
                        }
                    }
                    Err(IpcError::ChannelClosed) => break,
                    Err(IpcError::WouldBlock) => {
                        syscall::yield_now();
                        continue;
                    }
                    Err(_) => break,
                }
            }
        }
    }

    /// Serve with connection callback
    ///
    /// More flexible version that provides the connection object,
    /// allowing multi-message protocols.
    pub fn serve_with_connection<F>(&self, mut handler: F) -> !
    where
        F: FnMut(Connection<P>),
    {
        loop {
            match self.accept() {
                Ok(conn) => handler(conn),
                Err(_) => syscall::yield_now(),
            }
        }
    }

    /// Stop the server
    pub fn stop(&mut self) {
        if self.state == ServerState::Listening {
            syscall::close(self.listen_channel);
            self.state = ServerState::Stopped;
        }
    }
}

impl<P: Protocol> Drop for Server<P> {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// Connection is active
    Active,
    /// Connection has been closed
    Closed,
}

/// Server-side connection to a client
pub struct Connection<P: Protocol> {
    channel: Channel,
    state: ConnectionState,
    /// Client's PID (if handshake was performed)
    client_pid: Option<u32>,
    _protocol: core::marker::PhantomData<P>,
}

impl<P: Protocol> Connection<P> {
    /// Create from channel
    fn new(channel: Channel) -> Self {
        // Query the client PID via syscall
        let client_pid = {
            let pid = crate::syscall::channel_get_peer(channel.id());
            if pid > 0 { Some(pid as u32) } else { None }
        };

        Self {
            channel,
            state: ConnectionState::Active,
            client_pid,
            _protocol: core::marker::PhantomData,
        }
    }

    /// Get connection state
    pub fn state(&self) -> ConnectionState {
        self.state
    }

    /// Check if connection is active
    pub fn is_active(&self) -> bool {
        self.state == ConnectionState::Active && self.channel.is_connected()
    }

    /// Get client PID
    /// Returns the PID of the process on the other end of this channel.
    /// Used for security checks in protocol handlers.
    pub fn client_pid(&self) -> Option<u32> {
        self.client_pid
    }

    /// Get client's capabilities
    /// Returns the capability bits of the client process.
    /// Use crate::syscall::caps::has() to check specific capabilities.
    pub fn client_capabilities(&self) -> Option<u64> {
        self.client_pid.map(|pid| {
            let caps = crate::syscall::get_capabilities(pid);
            if caps >= 0 { caps as u64 } else { 0 }
        })
    }

    /// Check if client has a specific capability
    pub fn client_has_capability(&self, cap: u64) -> bool {
        self.client_capabilities()
            .map(|caps| crate::syscall::caps::has(caps, cap))
            .unwrap_or(false)
    }

    /// Get channel ID
    pub fn channel_id(&self) -> u32 {
        self.channel.id()
    }

    /// Receive a request from the client (blocking)
    pub fn receive(&mut self) -> IpcResult<P::Request> {
        if self.state != ConnectionState::Active {
            return Err(IpcError::ChannelClosed);
        }

        let mut buf = [0u8; 576];
        let len = match self.channel.receive_raw(&mut buf) {
            Ok(len) => len,
            Err(e) => {
                if e.is_fatal() {
                    self.state = ConnectionState::Closed;
                }
                return Err(e);
            }
        };

        let (request, _) = P::Request::deserialize(&buf[..len])?;
        Ok(request)
    }

    /// Receive with timeout
    pub fn receive_timeout(&mut self, timeout_ms: u32) -> IpcResult<P::Request> {
        if self.state != ConnectionState::Active {
            return Err(IpcError::ChannelClosed);
        }

        let mut buf = [0u8; 576];
        let len = match self.channel.receive_raw_timeout(&mut buf, timeout_ms) {
            Ok(len) => len,
            Err(e) => {
                if e.is_fatal() {
                    self.state = ConnectionState::Closed;
                }
                return Err(e);
            }
        };

        let (request, _) = P::Request::deserialize(&buf[..len])?;
        Ok(request)
    }

    /// Receive (non-blocking)
    pub fn try_receive(&mut self) -> IpcResult<P::Request> {
        if self.state != ConnectionState::Active {
            return Err(IpcError::ChannelClosed);
        }

        let mut buf = [0u8; 576];
        let len = self.channel.receive_raw_nonblock(&mut buf)?;

        let (request, _) = P::Request::deserialize(&buf[..len])?;
        Ok(request)
    }

    /// Send a response to the client
    pub fn send(&mut self, response: &P::Response) -> IpcResult<()> {
        if self.state != ConnectionState::Active {
            return Err(IpcError::ChannelClosed);
        }

        let mut buf = [0u8; 576];
        let len = response.serialize(&mut buf)?;

        match self.channel.send_raw(&buf[..len]) {
            Ok(()) => Ok(()),
            Err(e) => {
                if e.is_fatal() {
                    self.state = ConnectionState::Closed;
                }
                Err(e)
            }
        }
    }

    /// Perform PID handshake (receive client PID, send our PID)
    pub fn handshake_pid(&mut self) -> IpcResult<u32> {
        // Receive client PID
        let mut buf = [0u8; 4];
        let len = self.channel.receive_raw_timeout(&mut buf, DEFAULT_TIMEOUT_MS)?;
        if len < 4 {
            return Err(IpcError::HandshakeFailed);
        }

        let client_pid = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        self.client_pid = Some(client_pid);

        // Send our PID
        let my_pid = syscall::getpid() as u32;
        buf.copy_from_slice(&my_pid.to_le_bytes());
        self.channel.send_raw(&buf)?;

        Ok(client_pid)
    }

    /// Close the connection
    pub fn close(&mut self) {
        self.channel.close();
        self.state = ConnectionState::Closed;
    }
}

impl<P: Protocol> Drop for Connection<P> {
    fn drop(&mut self) {
        // Channel's Drop will close it
    }
}
