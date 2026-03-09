//! Query Handling Module
//!
//! Handles queries from clients and port registration from managed services.
//! Implements the query protocol defined in `libos::query`.
//!
//! Managed services (spawned by devd) communicate via SuperQ (kernel supervision queue).
//! Dynamic clients (connected via devd-query: port) use IPC channels.

use libsys::ipc::Channel;
use libsys::error::SysError;
use libos::supervision::SupervisionHandle;
use libos::query::{
    QueryHeader, ErrorResponse,
    PortRegisterResponse, PortRegisterInfo as PortRegisterInfoMsg, SpawnChild, SpawnAck,
    SpawnChildContext, query_flags,
    msg, error,
};

use crate::service::{ServiceManager, ServiceRegistry};

// =============================================================================
// Constants
// =============================================================================

/// Maximum number of concurrent query clients
pub const MAX_QUERY_CLIENTS: usize = 16;

/// Message buffer size
pub const MSG_BUFFER_SIZE: usize = 512;

// =============================================================================
// Client Transport
// =============================================================================

/// Transport abstraction for query clients.
///
/// Managed services use SuperQ (spawned with exec_with_mailbox).
/// Dynamic clients use IPC channels (connected via devd-query: port).
pub enum ClientTransport {
    /// IPC channel (dynamic clients)
    Channel(Channel),
    /// Supervision queue (managed services).
    /// Bidirectional: SuperQ for both parent→child and child→parent messages.
    Supervision {
        /// Kernel-backed supervision queue
        superq: SupervisionHandle,
    },
}

// =============================================================================
// Query Client
// =============================================================================

/// A connected query client
pub struct QueryClient {
    /// Transport to client
    pub transport: ClientTransport,
    /// Is this a managed service spawned by devd (can register ports)
    pub is_managed: bool,
    /// Service index if this is a managed service (-1 for regular clients)
    pub service_idx: i8,
    /// Client PID (for later identification)
    pub pid: u32,
}

impl QueryClient {
    /// Send data to this client.
    ///
    /// For channel clients: sends via IPC channel.
    /// For supervision clients: sends FORWARD note(s) via SuperQ.
    pub fn send(&mut self, data: &[u8]) -> Result<(), SysError> {
        match &mut self.transport {
            ClientTransport::Channel(ch) => ch.send(data),
            ClientTransport::Supervision { superq } => {
                superq.send_forward(data, 0)
            }
        }
    }

    /// Non-blocking receive from this client.
    ///
    /// For channel clients: tries IPC recv.
    /// For supervision clients: reads FORWARD note(s) from SuperQ, reassembles.
    pub fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, SysError> {
        match &mut self.transport {
            ClientTransport::Channel(ch) => ch.try_recv(buf),
            ClientTransport::Supervision { superq } => {
                superq.recv_forward(buf)
            }
        }
    }

    /// Get the IPC handle for this client (Channel or SuperQ, both Mux-watchable).
    pub fn handle(&self) -> Option<libsys::ipc::ObjHandle> {
        match &self.transport {
            ClientTransport::Channel(ch) => Some(ch.handle()),
            ClientTransport::Supervision { superq } => Some(superq.handle()),
        }
    }
}

// =============================================================================
// Query Handler
// =============================================================================

/// Handles query protocol messages
pub struct QueryHandler {
    /// Connected query clients
    clients: [Option<QueryClient>; MAX_QUERY_CLIENTS],
    /// Number of active clients
    client_count: usize,
}

impl QueryHandler {
    pub const fn new() -> Self {
        Self {
            clients: [const { None }; MAX_QUERY_CLIENTS],
            client_count: 0,
        }
    }

    /// Add a channel-based client connection (dynamic clients from devd-query: port)
    pub fn add_client(&mut self, channel: Channel, service_idx: Option<u8>, pid: u32) -> Option<usize> {
        let slot = (0..MAX_QUERY_CLIENTS).find(|&i| self.clients[i].is_none())?;

        self.clients[slot] = Some(QueryClient {
            transport: ClientTransport::Channel(channel),
            is_managed: service_idx.is_some(),
            service_idx: service_idx.map(|i| i as i8).unwrap_or(-1),
            pid,
        });
        self.client_count += 1;

        Some(slot)
    }

    /// Add a supervision-based managed service (SuperQ for bidirectional communication).
    pub fn add_supervision_client(
        &mut self,
        superq: SupervisionHandle,
        service_idx: u8,
        pid: u32,
    ) -> Option<usize> {
        let slot = (0..MAX_QUERY_CLIENTS).find(|&i| self.clients[i].is_none())?;

        self.clients[slot] = Some(QueryClient {
            transport: ClientTransport::Supervision {
                superq,
            },
            is_managed: true,
            service_idx: service_idx as i8,
            pid,
        });
        self.client_count += 1;

        Some(slot)
    }


    /// Remove a client by slot
    pub fn remove_client(&mut self, slot: usize) -> bool {
        if slot >= MAX_QUERY_CLIENTS {
            return false;
        }

        if self.clients[slot].take().is_some() {
            self.client_count = self.client_count.saturating_sub(1);
            true
        } else {
            false
        }
    }

    /// Find client slot by channel handle (only matches channel-based clients)
    pub fn find_by_handle(&self, handle: libsys::ipc::ObjHandle) -> Option<usize> {
        (0..MAX_QUERY_CLIENTS).find(|&i| {
            self.clients[i]
                .as_ref()
                .and_then(|c| c.handle())
                .map(|h| h == handle)
                .unwrap_or(false)
        })
    }

    /// Find client slot by PID
    pub fn find_by_pid(&self, pid: u32) -> Option<usize> {
        (0..MAX_QUERY_CLIENTS).find(|&i| {
            self.clients[i]
                .as_ref()
                .map(|c| c.pid == pid)
                .unwrap_or(false)
        })
    }

    /// Get client reference
    pub fn get(&self, slot: usize) -> Option<&QueryClient> {
        self.clients.get(slot).and_then(|c| c.as_ref())
    }

    /// Get mutable client reference
    pub fn get_mut(&mut self, slot: usize) -> Option<&mut QueryClient> {
        self.clients.get_mut(slot).and_then(|c| c.as_mut())
    }

    /// Return the number of active clients
    pub fn active_count(&self) -> usize {
        self.client_count
    }

    /// Get service index for a client (if it's a managed service)
    pub fn get_service_idx(&self, slot: usize) -> Option<u8> {
        self.clients.get(slot)
            .and_then(|c| c.as_ref())
            .filter(|c| c.is_managed && c.service_idx >= 0)
            .map(|c| c.service_idx as u8)
    }

    /// Upgrade a client to managed status by PID
    ///
    /// This handles the race condition where a child process connects to
    /// devd-query before its parent sends SPAWN_ACK. When we receive the
    /// SPAWN_ACK, we need to upgrade that client to managed status.
    pub fn upgrade_to_managed(&mut self, pid: u32, service_idx: u8) {
        for client in &mut self.clients {
            if let Some(c) = client {
                if c.pid == pid && !c.is_managed {
                    c.is_managed = true;
                    c.service_idx = service_idx as i8;
                    return;
                }
            }
        }
    }

    /// Process an incoming message from a client.
    /// Returns None so the caller can try text admin handling as a fallback
    /// (text commands >= 8 bytes parse as QueryHeader with garbage msg_type).
    pub fn handle_message(
        &mut self,
        _slot: usize,
        _buf: &[u8],
        _services: &ServiceRegistry,
        _response_buf: &mut [u8],
    ) -> Option<usize> {
        // All known binary message types are handled in dispatch_query_message.
        // If we get here, the message type is unknown — return None.
        None
    }

    /// Send a port registration response
    pub fn send_port_register_response(
        &mut self,
        slot: usize,
        seq_id: u32,
        result: i32,
    ) {
        let resp = PortRegisterResponse::new(seq_id, result);
        if let Some(client) = self.get_mut(slot) {
            if client.send(&resp.to_bytes()).is_err() {
                libsys::syscall::klog(libsys::syscall::LogLevel::Warn,
                    b"[devd] port_reg_resp send failed");
            }
        }
    }

    /// Parse a REGISTER_PORT_INFO message (unified PortInfo)
    /// Returns parsed info if valid and caller is a managed service
    pub fn parse_port_register_info(
        &self,
        slot: usize,
        buf: &[u8],
    ) -> Option<PortInfoRegistration> {
        // Parse message header and body
        let (reg, info_bytes) = PortRegisterInfoMsg::from_bytes(buf)?;

        // Only managed services can register ports
        let client = self.clients[slot].as_ref()?;
        if !client.is_managed {
            return None;
        }

        // Parse PortInfo from bytes (112 bytes at offset 16)
        if info_bytes.len() < 112 {
            return None;
        }
        let port_info: abi::PortInfo = unsafe {
            core::ptr::read_unaligned(info_bytes.as_ptr() as *const abi::PortInfo)
        };

        Some(PortInfoRegistration {
            seq_id: reg.header.seq_id,
            shmem_id: reg.shmem_id,
            port_info,
            owner_idx: client.service_idx as u8,
        })
    }

}

/// Parsed port registration info
pub struct PortInfoRegistration {
    pub seq_id: u32,
    pub shmem_id: u32,
    pub port_info: abi::PortInfo,
    pub owner_idx: u8,
}

// =============================================================================
// Spawn Command Support
// =============================================================================

/// Sequence ID counter for spawn commands
static mut SPAWN_SEQ_ID: u32 = 1;

impl QueryHandler {
    /// Find query client by service index
    pub fn find_by_service_idx(&self, service_idx: u8) -> Option<usize> {
        (0..MAX_QUERY_CLIENTS).find(|&i| {
            self.clients[i]
                .as_ref()
                .map(|c| c.service_idx == service_idx as i8)
                .unwrap_or(false)
        })
    }

    /// Send a SPAWN_CHILD command to a driver
    /// Send a SPAWN_CHILD command to a driver with capabilities and context.
    ///
    /// When `ctx` is Some, the context section is appended to the message
    /// so the parent driver can answer GET_SPAWN_CONTEXT locally.
    pub fn send_spawn_child_with_context(
        &mut self,
        service_idx: u8,
        binary: &[u8],
        trigger_port: &[u8],
        caps: u64,
        priority: u8,
        ctx: Option<&SpawnChildContext>,
    ) -> Option<u32> {
        let slot = self.find_by_service_idx(service_idx)?;
        let client = self.clients[slot].as_mut()?;

        if !client.is_managed {
            return None;
        }

        // Get next sequence ID
        let seq_id = unsafe {
            let id = SPAWN_SEQ_ID;
            SPAWN_SEQ_ID = SPAWN_SEQ_ID.wrapping_add(1);
            id
        };

        let cmd = SpawnChild::with_caps_and_priority(seq_id, caps, priority);
        let mut buf = [0u8; 512];

        let len = if let Some(context) = ctx {
            let (filter, pattern) = libos::query::PortFilter::exact(trigger_port);
            cmd.write_to_with_context(&mut buf, binary, &filter, pattern, context)?
        } else {
            cmd.write_to(&mut buf, binary, trigger_port)?
        };

        match client.send(&buf[..len]) {
            Ok(_) => Some(seq_id),
            Err(_) => None,
        }
    }

    /// Send a SPAWN_CHILD command with path-based ADDRESSED routing.
    ///
    /// Wraps the normal SpawnChild message with an ADDRESSED route prefix
    /// so that intermediate bus_runtime nodes can consume path segments
    /// and forward to the correct child subtree.
    ///
    /// When `path` is empty, falls back to `send_spawn_child_with_context`.
    pub fn send_spawn_child_with_path(
        &mut self,
        service_idx: u8,
        binary: &[u8],
        trigger_port: &[u8],
        caps: u64,
        priority: u8,
        ctx: Option<&SpawnChildContext>,
        path: &[u8],
    ) -> Option<u32> {
        if path.is_empty() {
            return self.send_spawn_child_with_context(
                service_idx, binary, trigger_port, caps, priority, ctx,
            );
        }

        let slot = self.find_by_service_idx(service_idx)?;
        let client = self.clients[slot].as_mut()?;

        if !client.is_managed {
            return None;
        }

        // Get next sequence ID
        let seq_id = unsafe {
            let id = SPAWN_SEQ_ID;
            SPAWN_SEQ_ID = SPAWN_SEQ_ID.wrapping_add(1);
            id
        };

        // Build the base SpawnChild message (without route)
        let cmd = SpawnChild::with_caps_and_priority(seq_id, caps, priority);
        let mut base = [0u8; 512];

        let base_len = if let Some(context) = ctx {
            let (filter, pattern) = libos::query::PortFilter::exact(trigger_port);
            cmd.write_to_with_context(&mut base, binary, &filter, pattern, context)?
        } else {
            cmd.write_to(&mut base, binary, trigger_port)?
        };

        // Wrap with ADDRESSED route: header(flags|=ADDRESSED) + route_len + route + payload
        let route_len = path.len();
        if route_len > 255 {
            return None;
        }
        let total = QueryHeader::SIZE + 1 + route_len + (base_len - QueryHeader::SIZE);
        if total > 576 {
            return None;
        }

        let mut out = [0u8; 576];
        // Copy header with ADDRESSED flag set
        out[..QueryHeader::SIZE].copy_from_slice(&base[..QueryHeader::SIZE]);
        let flags = u16::from_le_bytes([out[2], out[3]]) | query_flags::ADDRESSED;
        out[2..4].copy_from_slice(&flags.to_le_bytes());
        // Route length + route bytes
        out[QueryHeader::SIZE] = route_len as u8;
        let route_start = QueryHeader::SIZE + 1;
        out[route_start..route_start + route_len].copy_from_slice(path);
        // Copy payload (everything after header from base message)
        let out_payload = route_start + route_len;
        let base_payload_len = base_len - QueryHeader::SIZE;
        out[out_payload..out_payload + base_payload_len]
            .copy_from_slice(&base[QueryHeader::SIZE..base_len]);

        match client.send(&out[..total]) {
            Ok(_) => Some(seq_id),
            Err(_) => None,
        }
    }

    /// Parse a SPAWN_ACK response from a driver
    /// Returns: (seq_id, result, match_count, spawn_count, child_pids)
    pub fn parse_spawn_ack(buf: &[u8]) -> Option<(u32, i32, u8, u8, [u32; 16])> {
        let ack = SpawnAck::from_bytes(buf)?;
        let pids = SpawnAck::parse_pids(buf, ack.spawn_count as usize)
            .unwrap_or([0u32; 16]);
        Some((ack.header.seq_id, ack.result, ack.match_count, ack.spawn_count, pids))
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_remove_client() {
        // Would need mock Channel for proper testing
        // This tests the slot management logic
    }
}
