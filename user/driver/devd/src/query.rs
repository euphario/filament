//! Query Handling Module
//!
//! Handles queries from clients and port registration from managed services.
//! Implements the query protocol defined in `userlib::query`.

use userlib::ipc::Channel;
use userlib::query::{
    QueryHeader, ErrorResponse,
    PortRegisterResponse, PortRegisterInfo as PortRegisterInfoMsg, SpawnChild, SpawnAck,
    SpawnChildContext,
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
// Query Client
// =============================================================================

/// A connected query client
pub struct QueryClient {
    /// Channel to client
    pub channel: Channel,
    /// Is this a managed service spawned by devd (can register ports)
    pub is_managed: bool,
    /// Service index if this is a managed service (-1 for regular clients)
    pub service_idx: i8,
    /// Client PID (for later identification)
    pub pid: u32,
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

    /// Add a new client connection
    pub fn add_client(&mut self, channel: Channel, service_idx: Option<u8>, pid: u32) -> Option<usize> {
        // Find empty slot
        let slot = (0..MAX_QUERY_CLIENTS).find(|&i| self.clients[i].is_none())?;

        self.clients[slot] = Some(QueryClient {
            channel,
            is_managed: service_idx.is_some(),
            service_idx: service_idx.map(|i| i as i8).unwrap_or(-1),
            pid,
        });
        self.client_count += 1;

        Some(slot)
    }

    /// Remove a client by slot
    pub fn remove_client(&mut self, slot: usize) -> Option<Channel> {
        if slot >= MAX_QUERY_CLIENTS {
            return None;
        }

        if let Some(client) = self.clients[slot].take() {
            self.client_count = self.client_count.saturating_sub(1);
            Some(client.channel)
        } else {
            None
        }
    }

    /// Find client slot by channel handle
    pub fn find_by_handle(&self, handle: userlib::ipc::ObjHandle) -> Option<usize> {
        (0..MAX_QUERY_CLIENTS).find(|&i| {
            self.clients[i]
                .as_ref()
                .map(|c| c.channel.handle() == handle)
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

    /// Process an incoming message from a client
    pub fn handle_message(
        &mut self,
        _slot: usize,
        buf: &[u8],
        _services: &ServiceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let header = QueryHeader::from_bytes(buf)?;

        // Unknown message type
        let resp = ErrorResponse::new(header.seq_id, error::INVALID_REQUEST);
        let bytes = resp.to_bytes();
        response_buf[..bytes.len()].copy_from_slice(&bytes);
        Some(bytes.len())
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
            if client.channel.send(&resp.to_bytes()).is_err() {
                userlib::syscall::klog(userlib::syscall::LogLevel::Warn,
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
    ///
    /// Returns the sequence ID used (for tracking acknowledgement)
    pub fn send_spawn_child(
        &mut self,
        service_idx: u8,
        binary: &[u8],
        trigger_port: &[u8],
    ) -> Option<u32> {
        self.send_spawn_child_with_caps(service_idx, binary, trigger_port, 0)
    }

    /// Send a SPAWN_CHILD command to a driver with explicit capabilities
    ///
    /// Returns the sequence ID used (for tracking acknowledgement)
    pub fn send_spawn_child_with_caps(
        &mut self,
        service_idx: u8,
        binary: &[u8],
        trigger_port: &[u8],
        caps: u64,
    ) -> Option<u32> {
        self.send_spawn_child_with_context(service_idx, binary, trigger_port, caps, None)
    }

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

        let cmd = SpawnChild::with_caps(seq_id, caps);
        let mut buf = [0u8; 512];

        let len = if let Some(context) = ctx {
            let (filter, pattern) = userlib::query::PortFilter::exact(trigger_port);
            cmd.write_to_with_context(&mut buf, binary, &filter, pattern, context)?
        } else {
            cmd.write_to(&mut buf, binary, trigger_port)?
        };

        match client.channel.send(&buf[..len]) {
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
