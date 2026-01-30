//! Driver Command Ring
//!
//! Typed wrapper around CommandRing for devd ↔ driver communication.
//! Provides zero-copy, lock-free message passing between devd and drivers.
//!
//! ## Protocol Overview
//!
//! ```text
//! devd (Producer)              Shared Memory              driver (Consumer)
//!       │                           │                           │
//!       │──── SpawnChild ──────────►│◄────────────────────────►│
//!       │                           │                           │
//!       │◄──── SpawnAck ────────────│◄──────────────────────────│
//!       │                           │                           │
//! ```
//!
//! ## Usage
//!
//! **devd side (producer):**
//! ```rust
//! let ring = DriverRingProducer::create(RING_SIZE)?;
//! let shmem_id = ring.shmem_id();
//! // Send shmem_id to driver via initial handshake
//!
//! // Send command
//! ring.send_spawn_child(b"xhcid", b"pci/00:01.0:xhci")?;
//!
//! // Receive response
//! if let Some(resp) = ring.try_recv_response()? {
//!     match resp {
//!         DriverResponse::SpawnAck { result, pid } => { ... }
//!         _ => {}
//!     }
//! }
//! ```
//!
//! **driver side (consumer):**
//! ```rust
//! let ring = DriverRingConsumer::open(shmem_id)?;
//!
//! // Receive command
//! if let Some(cmd) = ring.try_recv_command()? {
//!     match cmd {
//!         DriverCommand::SpawnChild { seq_id, binary, trigger } => {
//!             let pid = syscall::exec(binary);
//!             ring.send_spawn_ack(seq_id, 0, pid)?;
//!         }
//!         _ => {}
//!     }
//! }
//! ```

use crate::command_ring::{CommandRing, Command, Response};
use crate::error::{SysError, SysResult};

// =============================================================================
// Command Types
// =============================================================================

/// Command types sent from devd to driver
pub mod cmd_type {
    /// Spawn a child driver process
    pub const SPAWN_CHILD: u32 = 1;
    /// Stop a child driver process
    pub const STOP_CHILD: u32 = 2;
    /// Query driver info
    pub const QUERY_INFO: u32 = 3;
    /// Attach to a disk (devd → partd)
    pub const ATTACH_DISK: u32 = 4;
    /// Register a partition (devd → partd)
    pub const REGISTER_PARTITION: u32 = 5;
    /// Mount a partition (devd → fatfsd)
    pub const MOUNT_PARTITION: u32 = 6;
}

/// Response types sent from driver to devd
pub mod resp_type {
    /// Acknowledge spawn command
    pub const SPAWN_ACK: u32 = 1;
    /// Driver state change
    pub const STATE_CHANGE: u32 = 2;
    /// Port registration
    pub const PORT_REGISTER: u32 = 3;
    /// Partition is ready
    pub const PARTITION_READY: u32 = 4;
    /// Mount is ready
    pub const MOUNT_READY: u32 = 5;
    /// Info query result
    pub const INFO_RESULT: u32 = 6;
    /// Update port shmem_id
    pub const UPDATE_SHMEM: u32 = 7;
}

// =============================================================================
// Command Payloads
// =============================================================================

/// SpawnChild command payload (48 bytes)
/// Layout: binary_len(1) + trigger_len(1) + pad(2) + binary(24) + trigger(20)
#[derive(Clone, Copy)]
#[repr(C)]
pub struct SpawnChildPayload {
    pub binary_len: u8,
    pub trigger_len: u8,
    pub _pad: [u8; 2],
    pub binary: [u8; 24],
    pub trigger: [u8; 20],
}

impl SpawnChildPayload {
    pub const SIZE: usize = 48;

    pub fn new(binary: &[u8], trigger: &[u8]) -> Self {
        let mut p = Self {
            binary_len: binary.len().min(24) as u8,
            trigger_len: trigger.len().min(20) as u8,
            _pad: [0; 2],
            binary: [0; 24],
            trigger: [0; 20],
        };
        let b_len = p.binary_len as usize;
        let t_len = p.trigger_len as usize;
        p.binary[..b_len].copy_from_slice(&binary[..b_len]);
        p.trigger[..t_len].copy_from_slice(&trigger[..t_len]);
        p
    }

    pub fn from_bytes(payload: &[u8; 48]) -> Self {
        let mut p = Self {
            binary_len: payload[0],
            trigger_len: payload[1],
            _pad: [0; 2],
            binary: [0; 24],
            trigger: [0; 20],
        };
        p.binary.copy_from_slice(&payload[4..28]);
        p.trigger.copy_from_slice(&payload[28..48]);
        p
    }

    pub fn to_bytes(&self) -> [u8; 48] {
        let mut buf = [0u8; 48];
        buf[0] = self.binary_len;
        buf[1] = self.trigger_len;
        buf[4..28].copy_from_slice(&self.binary);
        buf[28..48].copy_from_slice(&self.trigger);
        buf
    }

    pub fn binary(&self) -> &[u8] {
        &self.binary[..self.binary_len as usize]
    }

    pub fn trigger(&self) -> &[u8] {
        &self.trigger[..self.trigger_len as usize]
    }
}

/// StopChild command payload
#[derive(Clone, Copy)]
#[repr(C)]
pub struct StopChildPayload {
    pub child_pid: u32,
}

impl StopChildPayload {
    pub fn from_bytes(payload: &[u8; 48]) -> Self {
        Self {
            child_pid: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
        }
    }

    pub fn to_bytes(&self) -> [u8; 48] {
        let mut buf = [0u8; 48];
        buf[0..4].copy_from_slice(&self.child_pid.to_le_bytes());
        buf
    }
}

/// SpawnAck response payload
#[derive(Clone, Copy)]
#[repr(C)]
pub struct SpawnAckPayload {
    pub result: i32,
    pub match_count: u8,
    pub spawn_count: u8,
    pub _pad: [u8; 2],
    pub pids: [u32; 12], // Up to 12 PIDs fit in 56 - 8 = 48 bytes
}

impl SpawnAckPayload {
    pub fn new(result: i32, pid: u32) -> Self {
        let mut p = Self {
            result,
            match_count: if pid > 0 { 1 } else { 0 },
            spawn_count: if pid > 0 { 1 } else { 0 },
            _pad: [0; 2],
            pids: [0; 12],
        };
        if pid > 0 {
            p.pids[0] = pid;
        }
        p
    }

    pub fn from_bytes(payload: &[u8; 56]) -> Self {
        let mut p = Self {
            result: i32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            match_count: payload[4],
            spawn_count: payload[5],
            _pad: [0; 2],
            pids: [0; 12],
        };
        for i in 0..12 {
            let off = 8 + i * 4;
            p.pids[i] = u32::from_le_bytes([
                payload[off], payload[off + 1], payload[off + 2], payload[off + 3]
            ]);
        }
        p
    }

    pub fn to_bytes(&self) -> [u8; 56] {
        let mut buf = [0u8; 56];
        buf[0..4].copy_from_slice(&self.result.to_le_bytes());
        buf[4] = self.match_count;
        buf[5] = self.spawn_count;
        for i in 0..12 {
            let off = 8 + i * 4;
            buf[off..off + 4].copy_from_slice(&self.pids[i].to_le_bytes());
        }
        buf
    }
}

/// StateChange response payload
#[derive(Clone, Copy)]
#[repr(C)]
pub struct StateChangePayload {
    pub new_state: u8,
}

impl StateChangePayload {
    pub fn from_bytes(payload: &[u8; 56]) -> Self {
        Self { new_state: payload[0] }
    }

    pub fn to_bytes(&self) -> [u8; 56] {
        let mut buf = [0u8; 56];
        buf[0] = self.new_state;
        buf
    }
}

/// PortRegister response payload
/// Layout: port_type(1) + name_len(1) + pad(2) + shmem_id(4) + name(24) + parent(24)
#[derive(Clone, Copy)]
#[repr(C)]
pub struct PortRegisterPayload {
    pub port_type: u8,
    pub name_len: u8,
    pub parent_len: u8,
    pub _pad: u8,
    pub shmem_id: u32,
    pub name: [u8; 24],
    pub parent: [u8; 24],
}

impl PortRegisterPayload {
    pub fn new(port_type: u8, name: &[u8], parent: Option<&[u8]>, shmem_id: u32) -> Self {
        let mut p = Self {
            port_type,
            name_len: name.len().min(24) as u8,
            parent_len: parent.map(|p| p.len().min(24) as u8).unwrap_or(0),
            _pad: 0,
            shmem_id,
            name: [0; 24],
            parent: [0; 24],
        };
        let n_len = p.name_len as usize;
        p.name[..n_len].copy_from_slice(&name[..n_len]);
        if let Some(par) = parent {
            let par_len = p.parent_len as usize;
            p.parent[..par_len].copy_from_slice(&par[..par_len]);
        }
        p
    }

    pub fn from_bytes(payload: &[u8; 56]) -> Self {
        let mut p = Self {
            port_type: payload[0],
            name_len: payload[1],
            parent_len: payload[2],
            _pad: 0,
            shmem_id: u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            name: [0; 24],
            parent: [0; 24],
        };
        p.name.copy_from_slice(&payload[8..32]);
        p.parent.copy_from_slice(&payload[32..56]);
        p
    }

    pub fn to_bytes(&self) -> [u8; 56] {
        let mut buf = [0u8; 56];
        buf[0] = self.port_type;
        buf[1] = self.name_len;
        buf[2] = self.parent_len;
        buf[4..8].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[8..32].copy_from_slice(&self.name);
        buf[32..56].copy_from_slice(&self.parent);
        buf
    }

    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    pub fn parent(&self) -> Option<&[u8]> {
        if self.parent_len > 0 {
            Some(&self.parent[..self.parent_len as usize])
        } else {
            None
        }
    }
}

// =============================================================================
// Typed Command and Response
// =============================================================================

/// Typed command received from devd
#[derive(Clone, Debug)]
pub enum DriverCommand {
    /// Spawn a child driver
    SpawnChild {
        seq_id: u32,
        binary: [u8; 24],
        binary_len: usize,
        trigger: [u8; 20],
        trigger_len: usize,
    },
    /// Stop a child driver
    StopChild {
        seq_id: u32,
        child_pid: u32,
    },
    /// Query driver info
    QueryInfo {
        seq_id: u32,
    },
    /// Unknown command
    Unknown {
        seq_id: u32,
        cmd_type: u32,
    },
}

impl DriverCommand {
    /// Get the sequence ID
    pub fn seq_id(&self) -> u32 {
        match self {
            DriverCommand::SpawnChild { seq_id, .. } => *seq_id,
            DriverCommand::StopChild { seq_id, .. } => *seq_id,
            DriverCommand::QueryInfo { seq_id } => *seq_id,
            DriverCommand::Unknown { seq_id, .. } => *seq_id,
        }
    }

    /// Get binary name (for SpawnChild)
    pub fn binary(&self) -> Option<&[u8]> {
        match self {
            DriverCommand::SpawnChild { binary, binary_len, .. } => {
                Some(&binary[..*binary_len])
            }
            _ => None,
        }
    }

    /// Get trigger port name (for SpawnChild)
    pub fn trigger(&self) -> Option<&[u8]> {
        match self {
            DriverCommand::SpawnChild { trigger, trigger_len, .. } => {
                Some(&trigger[..*trigger_len])
            }
            _ => None,
        }
    }
}

/// Typed response from driver
#[derive(Clone, Debug)]
pub enum DriverResponse {
    /// Spawn acknowledgement
    SpawnAck {
        seq_id: u32,
        result: i32,
        match_count: u8,
        spawn_count: u8,
        pids: [u32; 12],
    },
    /// State change notification
    StateChange {
        seq_id: u32,
        new_state: u8,
    },
    /// Port registration
    PortRegister {
        seq_id: u32,
        port_type: u8,
        name: [u8; 24],
        name_len: usize,
        parent: [u8; 24],
        parent_len: usize,
        shmem_id: u32,
    },
    /// Unknown response
    Unknown {
        seq_id: u32,
        resp_type: u32,
    },
}

impl DriverResponse {
    /// Get the sequence ID
    pub fn seq_id(&self) -> u32 {
        match self {
            DriverResponse::SpawnAck { seq_id, .. } => *seq_id,
            DriverResponse::StateChange { seq_id, .. } => *seq_id,
            DriverResponse::PortRegister { seq_id, .. } => *seq_id,
            DriverResponse::Unknown { seq_id, .. } => *seq_id,
        }
    }
}

// =============================================================================
// Producer (devd side)
// =============================================================================

/// Driver ring producer (devd side)
///
/// Creates and owns the shared memory. Sends commands to driver,
/// receives responses from driver.
pub struct DriverRingProducer {
    ring: CommandRing,
}

impl DriverRingProducer {
    /// Create a new driver ring
    ///
    /// Allocates shared memory and initializes the ring header.
    /// Call `shmem_id()` to get the ID to send to the driver.
    pub fn create(ring_size: usize) -> SysResult<Self> {
        let ring = CommandRing::create_producer(ring_size)?;
        Ok(Self { ring })
    }

    /// Get the shared memory ID
    ///
    /// Send this to the driver so it can open the ring.
    pub fn shmem_id(&self) -> u32 {
        self.ring.shmem_id()
    }

    /// Get the wait handle for event loop integration
    pub fn handle(&self) -> crate::syscall::Handle {
        self.ring.handle()
    }

    /// Send a SpawnChild command (non-blocking)
    pub fn try_send_spawn_child(&mut self, binary: &[u8], trigger: &[u8]) -> SysResult<u32> {
        let index = self.ring.try_reserve_cmd().ok_or(SysError::WouldBlock)?;
        let cmd = self.ring.get_cmd_mut(index);

        cmd.cmd_type = cmd_type::SPAWN_CHILD;
        let payload = SpawnChildPayload::new(binary, trigger);
        cmd.payload = payload.to_bytes();

        let seq_id = cmd.seq_id;
        self.ring.publish_cmd();
        let _ = self.ring.notify();

        Ok(seq_id)
    }

    /// Send a SpawnChild command (blocking with timeout)
    pub fn send_spawn_child(&mut self, binary: &[u8], trigger: &[u8], timeout_ms: u32) -> SysResult<u32> {
        let index = self.ring.reserve_cmd(timeout_ms)?;
        let cmd = self.ring.get_cmd_mut(index);

        cmd.cmd_type = cmd_type::SPAWN_CHILD;
        let payload = SpawnChildPayload::new(binary, trigger);
        cmd.payload = payload.to_bytes();

        let seq_id = cmd.seq_id;
        self.ring.publish_cmd();
        let _ = self.ring.notify();

        Ok(seq_id)
    }

    /// Send a StopChild command
    pub fn try_send_stop_child(&mut self, child_pid: u32) -> SysResult<u32> {
        let index = self.ring.try_reserve_cmd().ok_or(SysError::WouldBlock)?;
        let cmd = self.ring.get_cmd_mut(index);

        cmd.cmd_type = cmd_type::STOP_CHILD;
        let payload = StopChildPayload { child_pid };
        cmd.payload = payload.to_bytes();

        let seq_id = cmd.seq_id;
        self.ring.publish_cmd();
        let _ = self.ring.notify();

        Ok(seq_id)
    }

    /// Send a QueryInfo command
    pub fn try_send_query_info(&mut self) -> SysResult<u32> {
        let index = self.ring.try_reserve_cmd().ok_or(SysError::WouldBlock)?;
        let cmd = self.ring.get_cmd_mut(index);

        cmd.cmd_type = cmd_type::QUERY_INFO;
        cmd.payload.fill(0);

        let seq_id = cmd.seq_id;
        self.ring.publish_cmd();
        let _ = self.ring.notify();

        Ok(seq_id)
    }

    /// Try to receive a response (non-blocking)
    pub fn try_recv_response(&mut self) -> SysResult<Option<DriverResponse>> {
        let resp = match self.ring.try_recv_rsp() {
            Some(r) => r,
            None => return Ok(None),
        };

        let result = parse_response(resp);
        self.ring.consume_rsp();
        Ok(Some(result))
    }

    /// Poll for responses until timeout (blocking)
    ///
    /// This loops checking for responses and waiting on the shmem handle.
    pub fn recv_response(&mut self, timeout_ms: u32) -> SysResult<DriverResponse> {
        use crate::syscall::gettime;

        let timeout = if timeout_ms == 0 { 5000 } else { timeout_ms };
        let start_ns = gettime();
        let deadline_ns = start_ns + (timeout as u64) * 1_000_000;

        loop {
            if let Some(resp) = self.ring.try_recv_rsp() {
                let result = parse_response(resp);
                self.ring.consume_rsp();
                return Ok(result);
            }

            let now_ns = gettime();
            if now_ns >= deadline_ns {
                return Err(SysError::Timeout);
            }

            // Wait on shmem for notification
            let remaining_ms = ((deadline_ns - now_ns) / 1_000_000) as u32;
            let wait_ms = remaining_ms.min(100);
            let _ = self.ring.wait(wait_ms);
        }
    }

    /// Check if the ring is closed
    pub fn is_closed(&self) -> bool {
        !self.ring.is_ready()
    }
}

// =============================================================================
// Consumer (driver side)
// =============================================================================

/// Driver ring consumer (driver side)
///
/// Opens an existing shared memory ring. Receives commands from devd,
/// sends responses to devd.
pub struct DriverRingConsumer {
    ring: CommandRing,
}

impl DriverRingConsumer {
    /// Open an existing driver ring by shmem_id
    pub fn open(shmem_id: u32) -> SysResult<Self> {
        let ring = CommandRing::attach_consumer(shmem_id)?;
        Ok(Self { ring })
    }

    /// Get the wait handle for event loop integration
    pub fn handle(&self) -> crate::syscall::Handle {
        self.ring.handle()
    }

    /// Try to receive a command (non-blocking)
    pub fn try_recv_command(&mut self) -> SysResult<Option<DriverCommand>> {
        let cmd = match self.ring.try_recv_cmd() {
            Some(c) => c,
            None => return Ok(None),
        };

        let result = parse_command(cmd);
        self.ring.consume_cmd();
        Ok(Some(result))
    }

    /// Receive a command (blocking with timeout)
    pub fn recv_command(&mut self, timeout_ms: u32) -> SysResult<DriverCommand> {
        use crate::syscall::gettime;

        let timeout = if timeout_ms == 0 { 5000 } else { timeout_ms };
        let start_ns = gettime();
        let deadline_ns = start_ns + (timeout as u64) * 1_000_000;

        loop {
            if let Some(cmd) = self.ring.try_recv_cmd() {
                let result = parse_command(cmd);
                self.ring.consume_cmd();
                return Ok(result);
            }

            let now_ns = gettime();
            if now_ns >= deadline_ns {
                return Err(SysError::Timeout);
            }

            // Wait on shmem for notification
            let remaining_ms = ((deadline_ns - now_ns) / 1_000_000) as u32;
            let wait_ms = remaining_ms.min(100);
            let _ = self.ring.wait(wait_ms);
        }
    }

    /// Send SpawnAck response
    pub fn send_spawn_ack(&mut self, seq_id: u32, result: i32, pid: u32) -> SysResult<()> {
        let resp = self.ring.try_reserve_rsp().ok_or(SysError::WouldBlock)?;

        resp.seq_id = seq_id;
        resp.result = result;
        // First 4 bytes of payload = response type
        resp.payload[0..4].copy_from_slice(&resp_type::SPAWN_ACK.to_le_bytes());
        // Rest = SpawnAckPayload (starting at offset 4)
        let payload = SpawnAckPayload::new(result, pid);
        let payload_bytes = payload.to_bytes();
        resp.payload[4..56].copy_from_slice(&payload_bytes[..52]);

        self.ring.publish_rsp();
        let _ = self.ring.notify();
        Ok(())
    }

    /// Send StateChange response
    pub fn send_state_change(&mut self, new_state: u8) -> SysResult<u32> {
        let resp = self.ring.try_reserve_rsp().ok_or(SysError::WouldBlock)?;

        resp.result = 0;
        // First 4 bytes = response type
        resp.payload[0..4].copy_from_slice(&resp_type::STATE_CHANGE.to_le_bytes());
        // State at offset 4
        resp.payload[4] = new_state;

        let seq_id = resp.seq_id;
        self.ring.publish_rsp();
        let _ = self.ring.notify();
        Ok(seq_id)
    }

    /// Send PortRegister response
    pub fn send_port_register(
        &mut self,
        port_type: u8,
        name: &[u8],
        parent: Option<&[u8]>,
        shmem_id: u32,
    ) -> SysResult<u32> {
        let resp = self.ring.try_reserve_rsp().ok_or(SysError::WouldBlock)?;

        resp.result = 0;
        // First 4 bytes = response type
        resp.payload[0..4].copy_from_slice(&resp_type::PORT_REGISTER.to_le_bytes());
        // PortRegisterPayload at offset 4 (52 bytes max)
        let payload = PortRegisterPayload::new(port_type, name, parent, shmem_id);
        let payload_bytes = payload.to_bytes();
        resp.payload[4..56].copy_from_slice(&payload_bytes[..52]);

        let seq_id = resp.seq_id;
        self.ring.publish_rsp();
        let _ = self.ring.notify();
        Ok(seq_id)
    }

    /// Check if the ring is closed
    pub fn is_closed(&self) -> bool {
        !self.ring.is_ready()
    }
}

// =============================================================================
// Helpers
// =============================================================================

fn parse_command(cmd: &Command) -> DriverCommand {
    match cmd.cmd_type {
        cmd_type::SPAWN_CHILD => {
            let p = SpawnChildPayload::from_bytes(&cmd.payload);
            DriverCommand::SpawnChild {
                seq_id: cmd.seq_id,
                binary: p.binary,
                binary_len: p.binary_len as usize,
                trigger: p.trigger,
                trigger_len: p.trigger_len as usize,
            }
        }
        cmd_type::STOP_CHILD => {
            let p = StopChildPayload::from_bytes(&cmd.payload);
            DriverCommand::StopChild {
                seq_id: cmd.seq_id,
                child_pid: p.child_pid,
            }
        }
        cmd_type::QUERY_INFO => {
            DriverCommand::QueryInfo {
                seq_id: cmd.seq_id,
            }
        }
        _ => {
            DriverCommand::Unknown {
                seq_id: cmd.seq_id,
                cmd_type: cmd.cmd_type,
            }
        }
    }
}

fn parse_response(resp: &Response) -> DriverResponse {
    // Response type is in first 4 bytes of payload
    let rtype = u32::from_le_bytes([
        resp.payload[0], resp.payload[1], resp.payload[2], resp.payload[3]
    ]);

    match rtype {
        resp_type::SPAWN_ACK => {
            // SpawnAckPayload data starts at offset 4
            let result = i32::from_le_bytes([
                resp.payload[4], resp.payload[5], resp.payload[6], resp.payload[7]
            ]);
            let match_count = resp.payload[8];
            let spawn_count = resp.payload[9];
            let mut pids = [0u32; 12];
            for i in 0..12 {
                let off = 12 + i * 4;
                pids[i] = u32::from_le_bytes([
                    resp.payload[off], resp.payload[off + 1],
                    resp.payload[off + 2], resp.payload[off + 3]
                ]);
            }
            DriverResponse::SpawnAck {
                seq_id: resp.seq_id,
                result,
                match_count,
                spawn_count,
                pids,
            }
        }
        resp_type::STATE_CHANGE => {
            // State at offset 4
            DriverResponse::StateChange {
                seq_id: resp.seq_id,
                new_state: resp.payload[4],
            }
        }
        resp_type::PORT_REGISTER => {
            // PortRegisterPayload starts at offset 4
            let port_type = resp.payload[4];
            let name_len = resp.payload[5];
            let parent_len = resp.payload[6];
            let shmem_id = u32::from_le_bytes([
                resp.payload[8], resp.payload[9], resp.payload[10], resp.payload[11]
            ]);
            let mut name = [0u8; 24];
            let mut parent = [0u8; 24];
            name.copy_from_slice(&resp.payload[12..36]);
            parent.copy_from_slice(&resp.payload[36..56]); // Only 20 bytes fit
            DriverResponse::PortRegister {
                seq_id: resp.seq_id,
                port_type,
                name,
                name_len: name_len as usize,
                parent,
                parent_len: parent_len as usize,
                shmem_id,
            }
        }
        _ => {
            DriverResponse::Unknown {
                seq_id: resp.seq_id,
                resp_type: rtype,
            }
        }
    }
}
