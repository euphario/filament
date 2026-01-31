//! NIC Interface Table
//!
//! Manages up to MAX_NICS network interfaces, each backed by a DataPort
//! connection to a NIC driver (netd, ethd, wifid).

use crate::types::{EthAddr, Ipv4Addr};
use crate::arp::ArpTable;
use userlib::bus::PortId;

/// Maximum number of attached NICs.
pub const MAX_NICS: usize = 4;

/// State of a NIC interface.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum NicState {
    /// Slot is empty.
    Empty,
    /// DataPort connected, waiting for info query response.
    Probing,
    /// NIC is up and processing packets.
    Up,
    /// NIC is down (link lost or driver exited).
    Down,
}

/// A single network interface.
pub struct Nic {
    pub state: NicState,
    /// DataPort block port ID (from connect_block_port).
    pub port_id: PortId,
    /// MAC address (from NIC query).
    pub mac: EthAddr,
    /// Configured IPv4 address.
    pub ipv4: Ipv4Addr,
    /// Subnet mask.
    pub netmask: Ipv4Addr,
    /// Gateway address.
    pub gateway: Ipv4Addr,
    /// MTU (default 1500).
    pub mtu: u16,
    /// Per-interface ARP table.
    pub arp: ArpTable,
    /// Name of the port we connected to (for logging).
    pub name: [u8; 16],
    pub name_len: usize,
}

impl Nic {
    pub const fn empty() -> Self {
        Self {
            state: NicState::Empty,
            port_id: PortId(0),
            mac: EthAddr::ZERO,
            ipv4: Ipv4Addr::ZERO,
            netmask: Ipv4Addr::ZERO,
            gateway: Ipv4Addr::ZERO,
            mtu: 1500,
            arp: ArpTable::new(),
            name: [0; 16],
            name_len: 0,
        }
    }

    pub fn set_name(&mut self, name: &[u8]) {
        let len = name.len().min(16);
        self.name[..len].copy_from_slice(&name[..len]);
        self.name_len = len;
    }
}

/// NIC interface table.
pub struct NicTable {
    pub nics: [Nic; MAX_NICS],
    pub count: usize,
}

impl NicTable {
    pub const fn new() -> Self {
        Self {
            nics: [Nic::empty(), Nic::empty(), Nic::empty(), Nic::empty()],
            count: 0,
        }
    }

    /// Add a new NIC. Returns the index, or None if full.
    pub fn add(&mut self, port_id: PortId, name: &[u8]) -> Option<usize> {
        for i in 0..MAX_NICS {
            if self.nics[i].state == NicState::Empty {
                self.nics[i] = Nic::empty();
                self.nics[i].state = NicState::Probing;
                self.nics[i].port_id = port_id;
                self.nics[i].set_name(name);
                if i >= self.count {
                    self.count = i + 1;
                }
                return Some(i);
            }
        }
        None
    }

    /// Find NIC by block port ID.
    pub fn find_by_port(&self, port_id: PortId) -> Option<usize> {
        for i in 0..self.count {
            if self.nics[i].state != NicState::Empty && self.nics[i].port_id == port_id {
                return Some(i);
            }
        }
        None
    }

    /// Find NIC that owns the given IPv4 address.
    pub fn find_by_ip(&self, ip: &Ipv4Addr) -> Option<usize> {
        for i in 0..self.count {
            if self.nics[i].state == NicState::Up && self.nics[i].ipv4 == *ip {
                return Some(i);
            }
        }
        None
    }

    /// Find the NIC to use for reaching a destination IP (simple subnet match).
    pub fn route(&self, dst: &Ipv4Addr) -> Option<usize> {
        // Direct subnet match
        for i in 0..self.count {
            if self.nics[i].state == NicState::Up
                && self.nics[i].ipv4.same_subnet(dst, &self.nics[i].netmask)
            {
                return Some(i);
            }
        }
        // Default: first Up NIC with a gateway
        for i in 0..self.count {
            if self.nics[i].state == NicState::Up && self.nics[i].gateway != Ipv4Addr::ZERO {
                return Some(i);
            }
        }
        None
    }

    /// Remove a NIC (mark as empty).
    pub fn remove(&mut self, idx: usize) {
        if idx < MAX_NICS {
            self.nics[idx].state = NicState::Empty;
        }
    }
}
