//! devd2 — Minimal device supervisor (busd-native)
//!
//! Connects to busd (not the old devd protocol), discovers kernel buses,
//! spawns drivers per boot rules, handles child exits and restarts.
//!
//! Bootstrap target: QEMU boot to shell via consoled.

#![no_std]
#![no_main]

use abi::{self, PortClass, PortInfo, MailboxHeader, BusMsgHeader, bus_msg_type};
use libsys::syscall;
use libsys::ipc::{Channel, Port, Mux, MuxFilter};
use libos::{uinfo, uwarn, uerror, unotice, udebug};
use libos::bus_client::BusClient;

// =============================================================================
// Constants
// =============================================================================

const MAX_SERVICES: usize = 16;
const MSG_BUF_SIZE: usize = 576;

/// Capability preset for drivers (IPC | MEM | SPAWN | SCHEME | IRQ | MMIO | DMA | RAW_DEVICE | GRANT)
const DRIVER_CAPS: u64 = 0x067F;

const MAX_ADMIN_CLIENTS: usize = 4;

// =============================================================================
// Config Format
// =============================================================================

/// Config file magic (little-endian)
const CONFIG_MAGIC: u16 = 0xDE02;

const MAX_RULES: usize = 16;

#[derive(Clone, Copy)]
struct LoadedRule {
    class: PortClass,
    binary: [u8; 16],
    binary_len: u8,
    caps: u64,
    priority: u8,
}

// =============================================================================
// Service Entry
// =============================================================================

#[derive(Clone, Copy, PartialEq)]
#[allow(dead_code)]
enum ServiceState {
    Starting,
    Ready,
    Running,
    Stopped,
    Crashed,
}

impl ServiceState {
    fn as_str(&self) -> &'static str {
        match self {
            ServiceState::Starting => "Starting",
            ServiceState::Ready => "Ready",
            ServiceState::Running => "Running",
            ServiceState::Stopped => "Stopped",
            ServiceState::Crashed => "Crashed",
        }
    }
}

#[derive(Clone, Copy)]
struct Service {
    pid: u32,
    name: [u8; 16],
    name_len: u8,
    state: ServiceState,
    trigger_port: [u8; 32],
    trigger_port_len: u8,
    caps: u64,
    priority: u8,
    /// Crash timestamp (ms) for rate-limiting restarts
    last_crash_ms: u64,
}

impl Service {
    fn name_str(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len as usize]).unwrap_or("?")
    }
}

// =============================================================================
// Admin Client
// =============================================================================

struct AdminClient {
    channel: Channel,
}

// =============================================================================
// Devd2
// =============================================================================

struct Devd2 {
    bus: BusClient,
    query_port: Option<Port>,
    mux: Mux,
    services: [Option<Service>; MAX_SERVICES],
    service_count: usize,
    admin_clients: [Option<AdminClient>; MAX_ADMIN_CLIENTS],
    rules: [Option<LoadedRule>; MAX_RULES],
    rule_count: usize,
}

impl Devd2 {
    fn new(bus: BusClient, mux: Mux) -> Self {
        Self {
            bus,
            query_port: None,
            mux,
            services: [const { None }; MAX_SERVICES],
            service_count: 0,
            admin_clients: [const { None }; MAX_ADMIN_CLIENTS],
            rules: [const { None }; MAX_RULES],
            rule_count: 0,
        }
    }

    // =========================================================================
    // Boot
    // =========================================================================

    fn boot(&mut self) {
        // Load config from initrd (falls back to defaults)
        self.load_config();

        // Register devd-query: port for shell admin commands
        match Port::register(b"devd-query:") {
            Ok(port) => {
                let h = port.handle();
                if let Err(e) = self.mux.add(h, MuxFilter::Readable) {
                    uerror!("devd2", "query_port_mux_failed"; err = e.as_str());
                }
                self.query_port = Some(port);
                uinfo!("devd2", "query_port_created");
            }
            Err(e) => {
                uwarn!("devd2", "query_port_failed"; err = e.as_str());
            }
        }

        // Discover kernel buses and spawn drivers
        self.discover_kernel_buses();
    }

    fn load_config(&mut self) {
        let mut buf = [0u8; 1024];
        let n = syscall::ramfs_read(b"etc/devd.conf", &mut buf);
        if n < 4 {
            uwarn!("devd2", "config_not_found; using_defaults");
            self.load_default_rules();
            return;
        }
        let n = n as usize;

        let magic = u16::from_le_bytes([buf[0], buf[1]]);
        let version = buf[2];
        let count = buf[3] as usize;

        if magic != CONFIG_MAGIC || version != 1 {
            uwarn!("devd2", "config_bad_format"; magic = magic as u32, version = version as u32);
            self.load_default_rules();
            return;
        }

        for i in 0..count.min(MAX_RULES) {
            let off = 4 + i * 32;
            if off + 32 > n { break; }

            let class = PortClass::from_u8(buf[off]);
            let blen = (buf[off + 1] as usize).min(16);
            let mut binary = [0u8; 16];
            binary[..blen].copy_from_slice(&buf[off + 2..off + 2 + blen]);
            let caps = u16::from_le_bytes([buf[off + 18], buf[off + 19]]) as u64;
            let priority = buf[off + 20];

            self.rules[self.rule_count] = Some(LoadedRule {
                class, binary, binary_len: blen as u8, caps, priority,
            });
            self.rule_count += 1;
        }

        uinfo!("devd2", "config_loaded"; rules = self.rule_count as u32);
    }

    fn load_default_rules(&mut self) {
        let defaults: &[(PortClass, &[u8], u8)] = &[
            (PortClass::Uart,     b"consoled", abi::priority::ABOVE_NORM),
            (PortClass::Klog,     b"klogd",    abi::priority::NORMAL),
            (PortClass::Cpu,      b"cpud",     abi::priority::NORMAL),
            (PortClass::Pcie,     b"pcied",    abi::priority::HIGH),
            (PortClass::Usb,      b"usbd",     abi::priority::NORMAL),
            (PortClass::Ethernet, b"ethd",     abi::priority::NORMAL),
            (PortClass::Pwm,      b"pwmd",     abi::priority::NORMAL),
        ];
        for &(class, name, priority) in defaults {
            let mut binary = [0u8; 16];
            let blen = name.len().min(16);
            binary[..blen].copy_from_slice(&name[..blen]);
            self.rules[self.rule_count] = Some(LoadedRule {
                class, binary, binary_len: blen as u8, caps: DRIVER_CAPS, priority,
            });
            self.rule_count += 1;
        }
        uinfo!("devd2", "default_rules_loaded"; rules = self.rule_count as u32);
    }

    fn discover_kernel_buses(&mut self) {
        let mut buses = [PortInfo::empty(); 16];
        let count = match libsys::ipc::bus_list(&mut buses) {
            Ok(n) => n,
            Err(e) => {
                uerror!("devd2", "bus_list_failed"; err = e.to_errno());
                return;
            }
        };

        uinfo!("devd2", "bus_discovery"; count = count as u32);

        for i in 0..count {
            let port_info = &buses[i];
            let path = &port_info.name[..port_info.name_len as usize];

            // Skip unknown/service ports
            if port_info.port_class == PortClass::Unknown || port_info.port_class == PortClass::Service {
                continue;
            }

            let path_str = core::str::from_utf8(path).unwrap_or("?");
            udebug!("devd2", "bus_found"; name = path_str, class = port_info.port_class.as_str());

            // Match against boot rules
            self.match_and_spawn(port_info);
        }
    }

    fn match_and_spawn(&mut self, port_info: &PortInfo) {
        // Copy matched rule data to avoid borrow conflict with spawn_driver(&mut self)
        let matched = (0..self.rule_count).find_map(|i| {
            self.rules[i].as_ref().and_then(|rule| {
                if rule.class == port_info.port_class {
                    Some((rule.binary, rule.binary_len, rule.caps, rule.priority))
                } else {
                    None
                }
            })
        });

        if let Some((binary_buf, binary_len, caps, priority)) = matched {
            let binary = core::str::from_utf8(&binary_buf[..binary_len as usize]).unwrap_or("?");
            let port_name = &port_info.name[..port_info.name_len as usize];
            self.spawn_driver(binary, port_name, caps, priority, port_info);
        }
    }

    // =========================================================================
    // Spawning
    // =========================================================================

    fn spawn_driver(
        &mut self,
        binary: &str,
        port_name: &[u8],
        caps: u64,
        priority: u8,
        _port_info: &PortInfo,
    ) {
        // Build mailbox with port info so child knows what to claim.
        // No TREE_MODE flag — child connects to busd directly, not via SuperQ.
        let mb_buf = build_mailbox(port_name, priority);

        // Format binary path: "bin/<binary>"
        let mut path_buf = [0u8; 32];
        path_buf[..4].copy_from_slice(b"bin/");
        let blen = binary.len().min(28);
        path_buf[4..4 + blen].copy_from_slice(&binary.as_bytes()[..blen]);
        let path = core::str::from_utf8(&path_buf[..4 + blen]).unwrap_or("bin/???");

        match syscall::exec_with_mailbox(path, caps, &mb_buf) {
            Ok((pid, _shmem_handle, _superq_handle)) => {
                uinfo!("devd2", "spawned"; binary = binary, pid = pid, port = core::str::from_utf8(port_name).unwrap_or("?"));
                self.add_service(pid, binary.as_bytes(), port_name, caps, priority);
            }
            Err(errno) => {
                uerror!("devd2", "spawn_failed"; binary = binary, errno = errno as u32);
            }
        }
    }


    fn add_service(&mut self, pid: u32, binary: &[u8], port_name: &[u8], caps: u64, priority: u8) {
        for (i, slot) in self.services.iter_mut().enumerate() {
            if slot.is_none() {
                let mut name = [0u8; 16];
                let nlen = binary.len().min(16);
                name[..nlen].copy_from_slice(&binary[..nlen]);

                let mut trigger = [0u8; 32];
                let tlen = port_name.len().min(32);
                trigger[..tlen].copy_from_slice(&port_name[..tlen]);

                *slot = Some(Service {
                    pid,
                    name,
                    name_len: nlen as u8,
                    state: ServiceState::Starting,
                    trigger_port: trigger,
                    trigger_port_len: tlen as u8,
                    caps,
                    priority,
                    last_crash_ms: 0,
                });
                self.service_count += 1;
                return;
            }
        }
        uwarn!("devd2", "service_table_full");
    }

    // =========================================================================
    // Event Loop
    // =========================================================================

    fn run(&mut self) -> ! {
        loop {
            let event = match self.mux.wait() {
                Ok(ev) => ev,
                Err(_) => continue,
            };

            // Signal events (CHILD_EXIT)
            if event.is_signal() {
                if event.signal_event as u32 & abi::signal_event::CHILD_EXIT != 0 {
                    let child_pid = (event.signal_value >> 32) as u32;
                    let exit_code = event.signal_value as i32;
                    self.handle_child_exit(child_pid, exit_code);
                }
                continue;
            }

            let handle = event.handle;

            // Dispatch by handle identity
            if let Some(ref port) = self.query_port {
                if handle == port.handle() {
                    self.accept_admin_client();
                    continue;
                }
            }

            if handle == self.bus.handle() {
                self.handle_bus_message();
                continue;
            }

            // Check admin clients
            let mut admin_idx = None;
            for (i, client) in self.admin_clients.iter().enumerate() {
                if let Some(ref c) = client {
                    if c.channel.handle() == handle {
                        admin_idx = Some(i);
                        break;
                    }
                }
            }
            if let Some(idx) = admin_idx {
                self.handle_admin_command(idx);
            }

            libos::ulog::flush();
        }
    }

    // =========================================================================
    // Child Exit
    // =========================================================================

    fn handle_child_exit(&mut self, pid: u32, exit_code: i32) {
        let mut found = false;
        for slot in self.services.iter_mut() {
            if let Some(ref mut svc) = slot {
                if svc.pid == pid {
                    let binary = svc.name_str();
                    if exit_code == 0 {
                        uinfo!("devd2", "child_exited"; binary = binary, pid = pid);
                        svc.state = ServiceState::Stopped;
                    } else {
                        uwarn!("devd2", "child_crashed"; binary = binary, pid = pid, code = exit_code as u32);
                        svc.state = ServiceState::Crashed;
                        svc.last_crash_ms = now_ms();
                    }
                    found = true;
                    break;
                }
            }
        }

        if !found {
            udebug!("devd2", "unknown_child_exit"; pid = pid);
        }

        // Restart crashed services after a brief delay (future: timer-based)
        // For now, restart immediately — consoled must come back fast.
        self.restart_crashed();
    }

    fn restart_crashed(&mut self) {
        let now = now_ms();

        for i in 0..MAX_SERVICES {
            let should_restart = self.services[i].as_ref().map(|svc| {
                svc.state == ServiceState::Crashed && (now - svc.last_crash_ms) > 500
            }).unwrap_or(false);

            if !should_restart { continue; }

            let (name_buf, name_len, port_buf, port_len, caps, priority) = {
                let svc = self.services[i].as_ref().unwrap();
                (svc.name, svc.name_len, svc.trigger_port, svc.trigger_port_len, svc.caps, svc.priority)
            };

            let binary = core::str::from_utf8(&name_buf[..name_len as usize]).unwrap_or("?");
            let port_name = &port_buf[..port_len as usize];
            let mb_buf = build_mailbox(port_name, priority);

            let mut path_buf = [0u8; 32];
            path_buf[..4].copy_from_slice(b"bin/");
            let blen = (name_len as usize).min(28);
            path_buf[4..4 + blen].copy_from_slice(&name_buf[..blen]);
            let path = core::str::from_utf8(&path_buf[..4 + blen]).unwrap_or("bin/???");

            match syscall::exec_with_mailbox(path, caps, &mb_buf) {
                Ok((pid, _, _)) => {
                    uinfo!("devd2", "restarted"; binary = binary, pid = pid);
                    if let Some(ref mut svc) = self.services[i] {
                        svc.pid = pid;
                        svc.state = ServiceState::Starting;
                    }
                }
                Err(errno) => {
                    uerror!("devd2", "restart_failed"; binary = binary, errno = errno as u32);
                }
            }
        }
    }

    // =========================================================================
    // Bus Message Handling
    // =========================================================================

    fn handle_bus_message(&mut self) {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let len = match self.bus.try_recv(&mut buf) {
            Ok(Some(n)) => n,
            _ => return,
        };

        if len < BusMsgHeader::SIZE { return; }

        let header = match BusMsgHeader::read_from(&buf[..len]) {
            Some(h) => h,
            None => return,
        };

        match header.msg_type {
            bus_msg_type::EVENT => {
                // Driver state change or port registration events
                let payload_start = BusMsgHeader::SIZE;
                let key_start = payload_start + header.addr_len as usize;
                let value_start = key_start + header.key_len as usize;
                let value_end = (value_start + header.value_len as usize).min(len);

                let key = &buf[key_start..key_start + header.key_len as usize];
                let _value = &buf[value_start..value_end];

                if key == b"state" {
                    // Driver reported state change via busd
                    let addr = &buf[payload_start..payload_start + header.addr_len as usize];
                    udebug!("devd2", "state_event";
                        from = core::str::from_utf8(addr).unwrap_or("?"),
                        state = core::str::from_utf8(_value).unwrap_or("?")
                    );
                }
            }
            bus_msg_type::GET => {
                // Config query routed to us — reply with service list or forward
                self.handle_bus_get(&buf[..len], &header);
            }
            _ => {}
        }
    }

    fn handle_bus_get(&mut self, buf: &[u8], header: &BusMsgHeader) {
        let payload_start = BusMsgHeader::SIZE;
        let key_start = payload_start + header.addr_len as usize;
        let key_end = (key_start + header.key_len as usize).min(buf.len());
        let key = &buf[key_start..key_end];

        if key == b"services" || key == b"list" {
            // Reply with service list
            let mut reply = [0u8; 512];
            let mut pos = 0;
            for slot in &self.services {
                if let Some(svc) = slot {
                    let name = svc.name_str();
                    let state = svc.state.as_str();
                    // Format: "name state pid\n"
                    let line = name.as_bytes();
                    let sline = state.as_bytes();
                    if pos + line.len() + sline.len() + 12 < reply.len() {
                        reply[pos..pos + line.len()].copy_from_slice(line);
                        pos += line.len();
                        reply[pos] = b' ';
                        pos += 1;
                        reply[pos..pos + sline.len()].copy_from_slice(sline);
                        pos += sline.len();
                        reply[pos] = b' ';
                        pos += 1;
                        // PID as decimal
                        let pid_str = format_u32(svc.pid);
                        let plen = pid_str.len();
                        reply[pos..pos + plen].copy_from_slice(pid_str.as_bytes());
                        pos += plen;
                        reply[pos] = b'\n';
                        pos += 1;
                    }
                }
            }
            let _ = self.bus.send_reply(header.seq_id, &reply[..pos]);
        } else {
            // Unknown key — error reply
            let _ = self.bus.send_error_reply(header.seq_id, b"unknown key");
        }
    }

    // =========================================================================
    // Admin (shell) Client Handling
    // =========================================================================

    fn accept_admin_client(&mut self) {
        let port = match &mut self.query_port {
            Some(p) => p,
            None => return,
        };

        match port.accept() {
            Ok(channel) => {
                let handle = channel.handle();
                for (i, slot) in self.admin_clients.iter_mut().enumerate() {
                    if slot.is_none() {
                        if let Err(e) = self.mux.add(handle, MuxFilter::Readable) {
                            uwarn!("devd2", "admin_mux_failed"; err = e.as_str());
                            return;
                        }
                        *slot = Some(AdminClient { channel });
                        udebug!("devd2", "admin_connected"; idx = i as u32);
                        return;
                    }
                }
                uwarn!("devd2", "admin_slots_full");
            }
            Err(_) => {}
        }
    }

    fn handle_admin_command(&mut self, idx: usize) {
        let mut buf = [0u8; 256];
        let channel_handle;

        let n = {
            let client = match &mut self.admin_clients[idx] {
                Some(c) => c,
                None => return,
            };
            channel_handle = client.channel.handle();
            match client.channel.try_recv(&mut buf) {
                Ok(Some(n)) => n,
                Ok(None) => return,
                Err(_) => {
                    // Client disconnected
                    let _ = self.mux.remove(channel_handle);
                    self.admin_clients[idx] = None;
                    return;
                }
            }
        };

        if n == 0 {
            // Disconnect
            let _ = self.mux.remove(channel_handle);
            self.admin_clients[idx] = None;
            return;
        }

        let _cmd = &buf[..n];

        // Parse admin commands (simple text protocol for shell compatibility)
        // The shell sends raw query protocol messages (QueryHeader format)
        // For now, handle the most common: CONFIG_GET for "devc list"
        // which is routed through devd → busd → driver

        // Respond with service list for any query
        let mut reply = [0u8; 512];
        let mut pos = 0;
        for slot in &self.services {
            if let Some(svc) = slot {
                let name = svc.name_str();
                let state = svc.state.as_str();
                let line = name.as_bytes();
                let sline = state.as_bytes();
                if pos + line.len() + sline.len() + 12 < reply.len() {
                    reply[pos..pos + line.len()].copy_from_slice(line);
                    pos += line.len();
                    reply[pos] = b' ';
                    pos += 1;
                    reply[pos..pos + sline.len()].copy_from_slice(sline);
                    pos += sline.len();
                    reply[pos] = b' ';
                    pos += 1;
                    let pid_str = format_u32(svc.pid);
                    let plen = pid_str.len();
                    reply[pos..pos + plen].copy_from_slice(pid_str.as_bytes());
                    pos += plen;
                    reply[pos] = b'\n';
                    pos += 1;
                }
            }
        }

        if let Some(ref client) = self.admin_clients[idx] {
            let _ = client.channel.send(&reply[..pos]);
        }
    }
}

// =============================================================================
// Helpers
// =============================================================================

/// Build a mailbox buffer with port name for a spawned child.
/// No TREE_MODE flag — child reads spawn context from mailbox but
/// connects to busd directly (no SuperQ relay).
fn build_mailbox(port_name: &[u8], priority: u8) -> [u8; 256] {
    let mut buf = [0u8; 256];

    // Header at [0..64)
    let mut hdr = MailboxHeader::empty();
    hdr.priority = priority;
    hdr.flags = abi::mailbox_flags::PARENT_WRITTEN; // no TREE_MODE

    // KV: "port.name" = port_name at [64..)
    let key = b"port.name";
    let klen = key.len();
    let vlen = port_name.len().min(64);
    let mut pos = 64usize;
    buf[pos] = klen as u8; pos += 1;
    buf[pos..pos + klen].copy_from_slice(key); pos += klen;
    buf[pos] = vlen as u8; pos += 1;
    buf[pos..pos + vlen].copy_from_slice(&port_name[..vlen]);

    hdr.kv_count = 1;

    // Write header
    let hdr_bytes = unsafe {
        core::slice::from_raw_parts(&hdr as *const MailboxHeader as *const u8, 64)
    };
    buf[..64].copy_from_slice(hdr_bytes);

    buf
}

fn now_ms() -> u64 {
    syscall::gettime() / 1_000_000
}

/// Format u32 as decimal string — returns slice of static buffer.
fn format_u32(mut val: u32) -> &'static str {
    // Thread-unsafe but we're single-threaded
    static mut BUF: [u8; 10] = [0; 10];
    unsafe {
        let mut pos = 10;
        if val == 0 {
            pos -= 1;
            BUF[pos] = b'0';
        } else {
            while val > 0 {
                pos -= 1;
                BUF[pos] = b'0' + (val % 10) as u8;
                val /= 10;
            }
        }
        core::str::from_utf8_unchecked(&BUF[pos..10])
    }
}

// =============================================================================
// Entry Point
// =============================================================================

#[unsafe(no_mangle)]
fn main() -> ! {
    // Register ulog flush for panic handler
    libsys::set_panic_flush(libos::ulog::flush);

    uinfo!("devd2", "starting");
    libos::ulog::flush();

    // Connect to busd — retry since busd may not have registered bus: port yet
    let mut bus = {
        let mut attempt = 0u32;
        loop {
            match BusClient::connect() {
                Ok(bc) => break bc,
                Err(_) if attempt < 50 => {
                    // busd not ready yet — yield and retry
                    syscall::sleep_us(10_000); // 10ms
                    attempt += 1;
                }
                Err(e) => {
                    uerror!("devd2", "busd_connect_failed"; err = e.as_str(), attempts = attempt);
                    libos::ulog::flush();
                    syscall::exit(1);
                }
            }
        }
    };

    // Register as "devd" with event subscription
    if let Err(e) = bus.register(b"devd", b"", 0, true) {
        uerror!("devd2", "register_failed"; err = e.as_str());
        libos::ulog::flush();
        syscall::exit(1);
    }

    uinfo!("devd2", "busd_connected");

    // Create Mux
    let mux = match Mux::new() {
        Ok(m) => m,
        Err(e) => {
            uerror!("devd2", "mux_failed"; err = e.as_str());
            syscall::exit(1);
        }
    };

    // Register busd channel with Mux
    let bus_handle = bus.handle();
    if let Err(e) = mux.add(bus_handle, MuxFilter::Readable) {
        uerror!("devd2", "busd_mux_failed"; err = e.as_str());
        syscall::exit(1);
    }

    let mut devd = Devd2::new(bus, mux);

    // Discover buses and spawn drivers
    devd.boot();

    // Flush logs before entering event loop
    libos::ulog::flush();

    // Run event loop
    devd.run()
}
