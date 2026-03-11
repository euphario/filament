//! Bus Daemon (busd)
//!
//! Message switch for the Filament microkernel. All drivers and clients
//! connect here. busd routes messages by address — unicast (by path or name),
//! multicast (by class or name), broadcast.
//!
//! busd never interprets message content — pure routing.

#![no_std]
#![no_main]

use abi::{BusMsgHeader, bus_msg_type, bus_msg_flags};
use libsys::syscall::{self, Handle};
use libsys::ipc::{Port, Channel, Mux, MuxFilter, ObjHandle};
use libsys::error::SysError;
use libos::{uinfo, unotice, uwarn, uerror, udebug};

const MAX_CLIENTS: usize = 64;
const MAX_PENDING_REPLIES: usize = 64;
const MSG_BUF_SIZE: usize = 576;

// =============================================================================
// Route Entry
// =============================================================================

struct Route {
    channel: Channel,
    name: [u8; 32],
    name_len: u8,
    path: [u8; 128],
    path_len: u8,
    class: u8,
    subscribe_events: bool,
}

impl Route {
    fn name_bytes(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    fn path_bytes(&self) -> &[u8] {
        &self.path[..self.path_len as usize]
    }

    fn handle_raw(&self) -> u32 {
        self.channel.handle().raw()
    }
}

// =============================================================================
// Pending Reply Tracker
// =============================================================================

struct PendingReply {
    seq_id: u16,
    sender_idx: u8,
    expected: u8,
}

// =============================================================================
// Bus Daemon
// =============================================================================

struct BusDaemon {
    port: Option<Port>,
    mux: Option<Mux>,
    clients: [Option<Route>; MAX_CLIENTS],
    pending: [Option<PendingReply>; MAX_PENDING_REPLIES],
    client_count: usize,
}

impl BusDaemon {
    const fn new() -> Self {
        const NONE_ROUTE: Option<Route> = None;
        const NONE_PENDING: Option<PendingReply> = None;
        Self {
            port: None,
            mux: None,
            clients: [NONE_ROUTE; MAX_CLIENTS],
            pending: [NONE_PENDING; MAX_PENDING_REPLIES],
            client_count: 0,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        let mux = Mux::new()?;
        let port = Port::register(b"bus:")?;
        mux.add(port.handle(), MuxFilter::Readable)?;

        self.port = Some(port);
        self.mux = Some(mux);

        unotice!("busd", "ready";);
        Ok(())
    }

    fn run(&mut self) -> ! {
        loop {
            let event = match self.mux.as_ref().unwrap().wait() {
                Ok(e) => e,
                Err(_) => continue,
            };

            let handle = event.handle;

            // Check if it's the port (new connection)
            if let Some(port) = &self.port {
                if handle == port.handle() {
                    self.accept_connection();
                    continue;
                }
            }

            // Find which client this handle belongs to
            let idx = self.find_client_by_handle(handle);
            if let Some(idx) = idx {
                self.handle_client_message(idx);
            }
        }
    }

    fn find_client_by_handle(&self, handle: ObjHandle) -> Option<usize> {
        let raw = handle.raw();
        for (i, client) in self.clients.iter().enumerate() {
            if let Some(route) = client {
                if route.handle_raw() == raw {
                    return Some(i);
                }
            }
        }
        None
    }

    // =========================================================================
    // Connection Management
    // =========================================================================

    fn accept_connection(&mut self) {
        let port = match &mut self.port {
            Some(p) => p,
            None => return,
        };

        match port.accept_with_pid() {
            Ok((channel, _pid)) => {
                let slot = match self.clients.iter().position(|c| c.is_none()) {
                    Some(s) => s,
                    None => {
                        uwarn!("busd", "full";);
                        drop(channel);
                        return;
                    }
                };

                // Watch this channel in the mux
                if let Some(mux) = &self.mux {
                    if mux.add(channel.handle(), MuxFilter::Readable).is_err() {
                        uwarn!("busd", "mux_full";);
                        drop(channel);
                        return;
                    }
                }

                self.clients[slot] = Some(Route {
                    channel,
                    name: [0; 32],
                    name_len: 0,
                    path: [0; 128],
                    path_len: 0,
                    class: 0,
                    subscribe_events: false,
                });
                self.client_count += 1;
            }
            Err(SysError::WouldBlock) => {}
            Err(_) => {}
        }
    }

    fn remove_client(&mut self, idx: usize) {
        if let Some(route) = &self.clients[idx] {
            // Remove from mux before dropping
            if let Some(mux) = &self.mux {
                let _ = mux.remove(route.channel.handle());
            }
        }
        if self.clients[idx].take().is_some() {
            for pending in &mut self.pending {
                if let Some(p) = pending {
                    if p.sender_idx as usize == idx {
                        *pending = None;
                    }
                }
            }
            self.client_count -= 1;
        }
    }

    // =========================================================================
    // Message Handling
    // =========================================================================

    fn handle_client_message(&mut self, idx: usize) {
        let mut buf = [0u8; MSG_BUF_SIZE];

        let len = {
            let route = match &self.clients[idx] {
                Some(r) => r,
                None => return,
            };
            // Non-blocking read via raw syscall
            match libsys::syscall::read(route.channel.handle(), &mut buf) {
                Ok(n) => n,
                Err(SysError::WouldBlock) => return,
                Err(_) => {
                    self.remove_client(idx);
                    return;
                }
            }
        };

        if len < BusMsgHeader::SIZE {
            return;
        }

        let header = match BusMsgHeader::read_from(&buf[..len]) {
            Some(h) => h,
            None => return,
        };

        match header.msg_type {
            bus_msg_type::REGISTER => self.handle_register(idx, &header, &buf[BusMsgHeader::SIZE..len]),
            bus_msg_type::GET | bus_msg_type::SET | bus_msg_type::LIFECYCLE => {
                self.route_request(idx, &header, &buf[..len]);
            }
            bus_msg_type::REPLY => self.route_reply(&header, &buf[..len]),
            bus_msg_type::EVENT => self.route_event(idx, &buf[..len]),
            _ => {}
        }
    }

    // =========================================================================
    // REGISTER
    // =========================================================================

    fn handle_register(&mut self, idx: usize, header: &BusMsgHeader, payload: &[u8]) {
        // REGISTER: addr = name, key = path, value[0] = class, value[1] = subscribe_events
        let addr_end = header.addr_len as usize;
        let key_end = addr_end + header.key_len as usize;

        if payload.len() < key_end {
            return;
        }

        let name = &payload[..addr_end];
        let path = &payload[addr_end..key_end];
        let value_start = key_end;
        let class = if header.value_len > 0 && payload.len() > value_start {
            payload[value_start]
        } else {
            0
        };
        let subscribe = header.value_len > 1
            && payload.len() > value_start + 1
            && payload[value_start + 1] != 0;

        let route = match &mut self.clients[idx] {
            Some(r) => r,
            None => return,
        };

        let nlen = name.len().min(route.name.len());
        route.name[..nlen].copy_from_slice(&name[..nlen]);
        route.name_len = nlen as u8;

        let plen = path.len().min(route.path.len());
        route.path[..plen].copy_from_slice(&path[..plen]);
        route.path_len = plen as u8;

        route.class = class;
        route.subscribe_events = subscribe;

        udebug!("busd", "registered";
            name = core::str::from_utf8(route.name_bytes()).unwrap_or("?"),
            path = core::str::from_utf8(route.path_bytes()).unwrap_or("?"));
    }

    // =========================================================================
    // Request Routing (GET, SET, LIFECYCLE)
    // =========================================================================

    fn route_request(&mut self, sender_idx: usize, header: &BusMsgHeader, buf: &[u8]) {
        let payload = &buf[BusMsgHeader::SIZE..];
        let addr = if header.addr_len > 0 && payload.len() >= header.addr_len as usize {
            &payload[..header.addr_len as usize]
        } else {
            b""
        };

        let mut targets = [0u8; MAX_CLIENTS];
        let target_count = self.resolve_address(addr, &mut targets);

        if target_count == 0 {
            self.send_error_reply(sender_idx, header.seq_id, b"no_route");
            return;
        }

        self.track_request(header.seq_id, sender_idx as u8, target_count as u8);

        for i in 0..target_count {
            let target_idx = targets[i] as usize;
            if target_idx == sender_idx {
                continue;
            }
            if let Some(route) = &self.clients[target_idx] {
                let _ = route.channel.send(buf);
            }
        }
    }

    // =========================================================================
    // Reply Routing
    // =========================================================================

    fn route_reply(&mut self, header: &BusMsgHeader, buf: &[u8]) {
        let is_eol = (header.flags & bus_msg_flags::EOL) != 0;

        for pending in &mut self.pending {
            if let Some(p) = pending {
                if p.seq_id == header.seq_id {
                    let sender_idx = p.sender_idx as usize;
                    if let Some(route) = &self.clients[sender_idx] {
                        let _ = route.channel.send(buf);
                    }
                    if is_eol {
                        p.expected = p.expected.saturating_sub(1);
                        if p.expected == 0 {
                            *pending = None;
                        }
                    }
                    return;
                }
            }
        }
    }

    // =========================================================================
    // Event Routing
    // =========================================================================

    fn route_event(&mut self, sender_idx: usize, buf: &[u8]) {
        for (i, client) in self.clients.iter().enumerate() {
            if i == sender_idx {
                continue;
            }
            if let Some(route) = client {
                if route.subscribe_events {
                    let _ = route.channel.send(buf);
                }
            }
        }
    }

    // =========================================================================
    // Address Resolution
    // =========================================================================

    fn resolve_address(&self, addr: &[u8], targets: &mut [u8]) -> usize {
        if addr.is_empty() || addr == b"*" {
            return self.resolve_broadcast(targets);
        }
        if addr.len() > 0 && addr[0] == b'@' {
            return self.resolve_by_class_name(&addr[1..], targets);
        }
        if addr.len() > 0 && addr[0] == b'/' {
            return self.resolve_by_path(addr, targets);
        }
        self.resolve_by_name(addr, targets)
    }

    fn resolve_broadcast(&self, targets: &mut [u8]) -> usize {
        let mut count = 0;
        for (i, client) in self.clients.iter().enumerate() {
            if client.is_some() && count < targets.len() {
                targets[count] = i as u8;
                count += 1;
            }
        }
        count
    }

    fn resolve_by_name(&self, name: &[u8], targets: &mut [u8]) -> usize {
        let mut count = 0;
        for (i, client) in self.clients.iter().enumerate() {
            if let Some(route) = client {
                if route.name_bytes() == name && count < targets.len() {
                    targets[count] = i as u8;
                    count += 1;
                }
            }
        }
        count
    }

    fn resolve_by_path(&self, path: &[u8], targets: &mut [u8]) -> usize {
        let mut count = 0;
        for (i, client) in self.clients.iter().enumerate() {
            if let Some(route) = client {
                if route.path_bytes() == path && count < targets.len() {
                    targets[count] = i as u8;
                    count += 1;
                }
            }
        }
        count
    }

    fn resolve_by_class_name(&self, class_name: &[u8], targets: &mut [u8]) -> usize {
        let class = match class_name {
            b"block" => 1u8,       // PortClass::Block
            b"network" => 3u8,     // PortClass::Network
            b"console" => 7u8,     // PortClass::Console
            b"filesystem" => 6u8,  // PortClass::Filesystem
            _ => return 0,
        };

        let mut count = 0;
        for (i, client) in self.clients.iter().enumerate() {
            if let Some(route) = client {
                if route.class == class && count < targets.len() {
                    targets[count] = i as u8;
                    count += 1;
                }
            }
        }
        count
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    fn track_request(&mut self, seq_id: u16, sender_idx: u8, expected: u8) {
        for pending in &mut self.pending {
            if pending.is_none() {
                *pending = Some(PendingReply { seq_id, sender_idx, expected });
                return;
            }
        }
        // Evict oldest
        self.pending[0] = Some(PendingReply { seq_id, sender_idx, expected });
    }

    fn send_error_reply(&self, target_idx: usize, seq_id: u16, error_msg: &[u8]) {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let len = abi::BusMsgBuilder::new(&mut buf, bus_msg_type::REPLY, seq_id)
            .flags(bus_msg_flags::EOL | bus_msg_flags::ERROR)
            .value(error_msg)
            .finish();

        if let Some(route) = &self.clients[target_idx] {
            let _ = route.channel.send(&buf[..len]);
        }
    }
}

// =============================================================================
// Entry Point
// =============================================================================

static mut BUSD: BusDaemon = BusDaemon::new();

#[unsafe(no_mangle)]
fn main() -> ! {
    libsys::set_panic_flush(libos::ulog::flush);
    libsys::io::disable_stdout();

    let busd = unsafe { &mut *(&raw mut BUSD) };

    if let Err(e) = busd.init() {
        uerror!("busd", "init_failed"; err = e.to_errno());
        syscall::exit(1);
    }

    busd.run()
}
