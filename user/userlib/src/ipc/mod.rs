//! IPC Library - Type-safe inter-process communication
//!
//! This module provides traits and utilities for building robust IPC
//! between kernel services, devd, and device drivers.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Protocol Layer (type-safe message definitions)            │
//! │  └── Protocol trait + concrete protocols                   │
//! ├─────────────────────────────────────────────────────────────┤
//! │  Transport Layer (reliable delivery)                       │
//! │  ├── Client<P>  - connect, request/response                │
//! │  ├── Server<P>  - register, accept, serve                  │
//! │  └── Channel    - low-level send/receive                   │
//! ├─────────────────────────────────────────────────────────────┤
//! │  Error Handling                                             │
//! │  └── IpcError   - comprehensive error enum                 │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Available Protocols
//!
//! | Protocol | Port | Description |
//! |----------|------|-------------|
//! | [`DevdProtocol`](protocols::devd::DevdProtocol) | `devd` | Device tree queries and subscriptions |
//! | [`GpioProtocol`](protocols::gpio::GpioProtocol) | `gpio` | GPIO pin control |
//! | [`PcieProtocol`](protocols::pcie::PcieProtocol) | `pcie` | PCIe device enumeration |
//! | [`FsProtocol`](protocols::filesystem::FsProtocol) | `fatfs` | Filesystem operations |
//! | [`BlockProtocol`](protocols::block::BlockProtocol) | - | Block device ring buffer handshake |
//!
//! # Quick Start
//!
//! ## Client Example
//!
//! ```rust,ignore
//! use userlib::ipc::{Client, DevdClient};
//! use userlib::ipc::protocols::devd::PropertyList;
//!
//! // Option 1: Use the convenience wrapper
//! let mut devd = DevdClient::connect()?;
//! let nodes = devd.query("/bus/i2c2/*")?;
//!
//! // Option 2: Use the generic Client
//! let mut client = Client::<GpioProtocol>::connect()?;
//! let response = client.request(&GpioRequest::GetPin { pin: 11 })?;
//! ```
//!
//! ## Server Example (Simple Loop)
//!
//! ```rust,ignore
//! use userlib::ipc::Server;
//!
//! let server = Server::<MyProtocol>::register()?;
//!
//! // Simple blocking serve loop
//! server.serve(|request| {
//!     match request {
//!         MyRequest::GetValue => MyResponse::Value(42),
//!         MyRequest::SetValue(v) => { /* ... */ MyResponse::Ok },
//!     }
//! });
//! ```
//!
//! ## Server Example (Event-Driven)
//!
//! For servers that need to handle multiple event sources (IPC, timers, IRQs):
//!
//! ```rust,ignore
//! use userlib::syscall::{Event, event_type, event_flags};
//!
//! let server = Server::<MyProtocol>::register()?;
//!
//! // Subscribe to IPC events on the server's listen channel
//! syscall::event_subscribe(event_type::IPC_READY, server.listen_channel() as u64);
//! syscall::event_subscribe(event_type::TIMER, 0);
//!
//! let mut event = Event::empty();
//! loop {
//!     syscall::event_wait(&mut event, event_flags::BLOCKING);
//!
//!     match event.event_type {
//!         et if et == event_type::IPC_READY => {
//!             if let Ok(mut conn) = server.try_accept() {
//!                 if let Ok(req) = conn.receive() {
//!                     let resp = handle_request(&req);
//!                     let _ = conn.send(&resp);
//!                 }
//!             }
//!         }
//!         et if et == event_type::TIMER => {
//!             // Handle timer event
//!         }
//!         _ => {}
//!     }
//! }
//! ```
//!
//! # Defining a Protocol
//!
//! ```rust,ignore
//! use userlib::ipc::protocol::{Protocol, Message};
//!
//! // 1. Define the protocol marker
//! pub struct MyProtocol;
//!
//! impl Protocol for MyProtocol {
//!     type Request = MyRequest;
//!     type Response = MyResponse;
//!     const PORT_NAME: &'static [u8] = b"myservice";
//! }
//!
//! // 2. Define request/response enums
//! pub enum MyRequest {
//!     Ping,
//!     GetValue { key: u32 },
//! }
//!
//! pub enum MyResponse {
//!     Pong,
//!     Value(u32),
//!     Error(i32),
//! }
//!
//! // 3. Implement Message trait for serialization
//! impl Message for MyRequest {
//!     fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> { /* ... */ }
//!     fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> { /* ... */ }
//!     fn serialized_size(&self) -> usize { /* ... */ }
//! }
//! ```
//!
//! # Error Handling
//!
//! All IPC operations return [`IpcResult<T>`], which is `Result<T, IpcError>`.
//!
//! Common errors:
//! - [`IpcError::NotConnected`] - No connection to server
//! - [`IpcError::Timeout`] - Operation timed out
//! - [`IpcError::ChannelClosed`] - Peer closed the connection
//! - [`IpcError::WouldBlock`] - Non-blocking operation would block
//! - [`IpcError::ServerError(i32)`] - Server returned an error code

mod error;
mod channel;
mod protocol;
mod client;
mod server;
pub mod protocols;

pub use error::{IpcError, IpcResult};
pub use channel::{Channel, create_pair};
pub use protocol::{Protocol, Message, Serialize, Deserialize, Empty, ReprCMessage};
pub use client::{Client, ClientBuilder, ClientState};
pub use server::{Server, Connection, ServerState, ConnectionState};

// Re-export common protocols for convenience
pub use protocols::devd::DevdClient;
pub use protocols::pcie::{PcieClient, PcieDeviceInfo};
pub use protocols::gpio::GpioClient;
pub use protocols::filesystem::FsClient;

/// Default timeout for IPC operations (5 seconds)
pub const DEFAULT_TIMEOUT_MS: u32 = 5000;

/// Maximum message size (matches kernel MAX_INLINE_PAYLOAD)
pub const MAX_MESSAGE_SIZE: usize = 576;
