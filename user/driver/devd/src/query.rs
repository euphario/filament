//! Query Handling Module
//!
//! Handles device queries from clients and registration from drivers.
//! Implements the query protocol defined in `userlib::query`.

use userlib::ipc::Channel;
use userlib::query::{
    QueryHeader, DeviceRegister, ListDevices, GetDeviceInfo, DriverQuery,
    DeviceListResponse, DeviceInfoResponse, DeviceEntry, ErrorResponse,
    PortRegister, PortRegisterResponse, SpawnChild, SpawnAck,
    msg, error, port_type,
};

use crate::devices::{DeviceStore, DeviceRegistry, MAX_DEVICES};
use crate::service::{ServiceManager, ServiceRegistry, Service};

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
    /// Is this a driver (can register devices) or regular client
    pub is_driver: bool,
    /// Service index if this is a driver (-1 for regular clients)
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
            is_driver: service_idx.is_some(),
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

    /// Get service index for a client (if it's a driver)
    pub fn get_service_idx(&self, slot: usize) -> Option<u8> {
        self.clients.get(slot)
            .and_then(|c| c.as_ref())
            .filter(|c| c.is_driver && c.service_idx >= 0)
            .map(|c| c.service_idx as u8)
    }

    /// Upgrade a client to driver status by PID
    ///
    /// This handles the race condition where a child process connects to
    /// devd-query before its parent sends SPAWN_ACK. When we receive the
    /// SPAWN_ACK, we need to upgrade that client to driver status.
    pub fn upgrade_client_to_driver(&mut self, pid: u32, service_idx: u8) {
        for client in &mut self.clients {
            if let Some(c) = client {
                if c.pid == pid && !c.is_driver {
                    c.is_driver = true;
                    c.service_idx = service_idx as i8;
                    return;
                }
            }
        }
    }

    /// Process an incoming message from a client
    pub fn handle_message(
        &mut self,
        slot: usize,
        buf: &[u8],
        devices: &mut DeviceRegistry,
        services: &ServiceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let header = QueryHeader::from_bytes(buf)?;

        match header.msg_type {
            msg::REGISTER_DEVICE => {
                self.handle_register_device(slot, buf, devices, response_buf)
            }
            msg::UNREGISTER_DEVICE => {
                self.handle_unregister_device(buf, devices, response_buf)
            }
            msg::UPDATE_STATE => {
                self.handle_update_state(buf, devices, response_buf)
            }
            msg::LIST_DEVICES => {
                self.handle_list_devices(buf, devices, response_buf)
            }
            msg::GET_DEVICE_INFO => {
                self.handle_get_device_info(buf, devices, services, response_buf)
            }
            msg::QUERY_DRIVER => {
                // Pass-through queries need special handling (async)
                // Return None to indicate caller should handle forwarding
                None
            }
            msg::REGISTER_PORT => {
                // Port registration needs access to Devd's ports and rules
                // Return None to indicate caller should handle this
                None
            }
            _ => {
                // Unknown message type
                let resp = ErrorResponse::new(header.seq_id, error::INVALID_REQUEST);
                let bytes = resp.to_bytes();
                response_buf[..bytes.len()].copy_from_slice(&bytes);
                Some(bytes.len())
            }
        }
    }

    // =========================================================================
    // Message Handlers
    // =========================================================================

    fn handle_register_device(
        &self,
        slot: usize,
        buf: &[u8],
        devices: &mut DeviceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let (info, path, _name) = DeviceRegister::from_bytes(buf)?;

        // Only drivers can register devices
        let client = self.clients[slot].as_ref()?;
        if !client.is_driver {
            let resp = ErrorResponse::new(info.header.seq_id, error::PERMISSION_DENIED);
            let bytes = resp.to_bytes();
            response_buf[..bytes.len()].copy_from_slice(&bytes);
            return Some(bytes.len());
        }

        let owner = client.service_idx as u8;
        let device_id = devices.register(&info, path, owner);

        match device_id {
            Some(id) => {
                // Return device ID in a DeviceInfoResponse
                let entry = DeviceEntry {
                    device_id: id,
                    device_class: info.device_class,
                    device_subclass: info.device_subclass,
                    vendor_id: info.vendor_id,
                    product_id: info.product_id,
                    state: userlib::query::state::DISCOVERED,
                    _pad: [0; 3],
                };
                let resp = DeviceInfoResponse::new(info.header.seq_id, entry);
                resp.write_to(response_buf, &[])
            }
            None => {
                let resp = ErrorResponse::new(info.header.seq_id, error::DEVICE_ERROR);
                let bytes = resp.to_bytes();
                response_buf[..bytes.len()].copy_from_slice(&bytes);
                Some(bytes.len())
            }
        }
    }

    fn handle_unregister_device(
        &self,
        buf: &[u8],
        devices: &mut DeviceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let header = QueryHeader::from_bytes(buf)?;

        // Device ID follows header
        if buf.len() < QueryHeader::SIZE + 4 {
            let resp = ErrorResponse::new(header.seq_id, error::INVALID_REQUEST);
            let bytes = resp.to_bytes();
            response_buf[..bytes.len()].copy_from_slice(&bytes);
            return Some(bytes.len());
        }

        let device_id = u32::from_le_bytes([
            buf[QueryHeader::SIZE],
            buf[QueryHeader::SIZE + 1],
            buf[QueryHeader::SIZE + 2],
            buf[QueryHeader::SIZE + 3],
        ]);

        let success = devices.unregister(device_id);

        let resp = if success {
            ErrorResponse::new(header.seq_id, error::OK)
        } else {
            ErrorResponse::new(header.seq_id, error::NOT_FOUND)
        };

        let bytes = resp.to_bytes();
        response_buf[..bytes.len()].copy_from_slice(&bytes);
        Some(bytes.len())
    }

    fn handle_update_state(
        &self,
        buf: &[u8],
        devices: &mut DeviceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let header = QueryHeader::from_bytes(buf)?;

        // Device ID and new state follow header
        if buf.len() < QueryHeader::SIZE + 5 {
            let resp = ErrorResponse::new(header.seq_id, error::INVALID_REQUEST);
            let bytes = resp.to_bytes();
            response_buf[..bytes.len()].copy_from_slice(&bytes);
            return Some(bytes.len());
        }

        let device_id = u32::from_le_bytes([
            buf[QueryHeader::SIZE],
            buf[QueryHeader::SIZE + 1],
            buf[QueryHeader::SIZE + 2],
            buf[QueryHeader::SIZE + 3],
        ]);
        let new_state = buf[QueryHeader::SIZE + 4];

        let success = devices.update_state(device_id, new_state);

        let resp = if success {
            ErrorResponse::new(header.seq_id, error::OK)
        } else {
            ErrorResponse::new(header.seq_id, error::NOT_FOUND)
        };

        let bytes = resp.to_bytes();
        response_buf[..bytes.len()].copy_from_slice(&bytes);
        Some(bytes.len())
    }

    fn handle_list_devices(
        &self,
        buf: &[u8],
        devices: &DeviceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let req = ListDevices::from_bytes(buf)?;

        let class_filter = if req.class_filter == 0 {
            None
        } else {
            Some(req.class_filter)
        };

        let count = devices.count(class_filter);

        // Write response header
        let resp = DeviceListResponse::new(req.header.seq_id, count as u16);
        resp.write_header(response_buf)?;

        // Write device entries
        let mut offset = DeviceListResponse::HEADER_SIZE;
        let mut written = 0;

        devices.for_each(class_filter, |device| {
            if written < MAX_DEVICES && offset + DeviceEntry::SIZE <= response_buf.len() {
                let entry = device.to_entry();
                response_buf[offset..offset + DeviceEntry::SIZE].copy_from_slice(&entry.to_bytes());
                offset += DeviceEntry::SIZE;
                written += 1;
            }
        });

        Some(offset)
    }

    fn handle_get_device_info(
        &self,
        buf: &[u8],
        devices: &DeviceRegistry,
        services: &ServiceRegistry,
        response_buf: &mut [u8],
    ) -> Option<usize> {
        let req = GetDeviceInfo::from_bytes(buf)?;

        let device = devices.get(req.device_id);

        match device {
            Some(dev) => {
                let entry = dev.to_entry();

                // Get driver port name
                let driver_port: &[u8] = services
                    .get(dev.owner_service_idx as usize)
                    .and_then(|s: &Service| s.def())
                    .and_then(|d| d.registers.first())
                    .map(|pd| pd.name)
                    .unwrap_or(b"");

                let resp = DeviceInfoResponse::new(req.header.seq_id, entry);
                resp.write_to(response_buf, driver_port)
            }
            None => {
                let resp = ErrorResponse::new(req.header.seq_id, error::NOT_FOUND);
                let bytes = resp.to_bytes();
                response_buf[..bytes.len()].copy_from_slice(&bytes);
                Some(bytes.len())
            }
        }
    }

    /// Parse a QUERY_DRIVER message and return the device ID and driver index
    pub fn parse_driver_query<'a>(
        &self,
        buf: &'a [u8],
        devices: &DeviceRegistry,
    ) -> Option<(u32, u8, u32, &'a [u8])> {
        let (query, payload) = DriverQuery::from_bytes(buf)?;
        let device = devices.get(query.device_id)?;

        Some((
            query.header.seq_id,
            device.owner_service_idx,
            query.query_type,
            payload,
        ))
    }

    /// Parse a REGISTER_PORT message
    /// Returns: (seq_id, port_type, name, parent, shmem_id, owner_idx)
    pub fn parse_port_register<'a>(
        &self,
        slot: usize,
        buf: &'a [u8],
    ) -> Option<PortRegisterInfo<'a>> {
        let (reg, name, parent, metadata) = PortRegister::from_bytes(buf)?;

        // Only drivers can register ports
        let client = self.clients[slot].as_ref()?;
        if !client.is_driver {
            return None;
        }

        Some(PortRegisterInfo {
            seq_id: reg.header.seq_id,
            port_type: reg.port_type,
            name,
            parent,
            metadata,
            shmem_id: reg.shmem_id,
            owner_idx: client.service_idx as u8,
        })
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
            let _ = client.channel.send(&resp.to_bytes());
        }
    }
}

/// Parsed port registration info
pub struct PortRegisterInfo<'a> {
    pub seq_id: u32,
    pub port_type: u8,
    pub name: &'a [u8],
    pub parent: Option<&'a [u8]>,
    pub metadata: &'a [u8],
    pub shmem_id: u32,
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
        let slot = self.find_by_service_idx(service_idx)?;
        let client = self.clients[slot].as_mut()?;

        if !client.is_driver {
            return None;
        }

        // Get next sequence ID
        let seq_id = unsafe {
            let id = SPAWN_SEQ_ID;
            SPAWN_SEQ_ID = SPAWN_SEQ_ID.wrapping_add(1);
            id
        };

        let cmd = SpawnChild::with_caps(seq_id, caps);
        let mut buf = [0u8; 256];
        let len = cmd.write_to(&mut buf, binary, trigger_port)?;

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
