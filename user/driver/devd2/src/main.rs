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
use libos::{uinfo, uwarn, uerror, udebug, unotice};
use libos::bus_client::BusClient;
use libos::wire::WireWriter;

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
const MAX_SERVICE_RULES: usize = 8;
const MAX_DRIVER_CONFIGS: usize = 16;
const MAX_CONFIG_KVS: usize = 8;

/// Config file magic for driver configs (little-endian)
const DRIVER_CONFIG_MAGIC: u16 = 0xCF02;

#[derive(Clone, Copy)]
struct LoadedRule {
    class: PortClass,
    binary: [u8; 16],
    binary_len: u8,
    caps: u64,
    priority: u8,
    /// Port subclass for finer matching (0xFFFF = match any subclass).
    subclass: u16,
}

/// Service rule: spawn a service when a dependency driver reaches Running.
/// Unlike driver rules (triggered by port registration), service rules trigger
/// on a binary name reaching Running state.
#[derive(Clone, Copy)]
struct ServiceRule {
    /// Binary name of the dependency (e.g. "fatfsd", "ipd")
    dep_binary: [u8; 16],
    dep_binary_len: u8,
    /// Binary to spawn when dependency is Running
    binary: [u8; 16],
    binary_len: u8,
    caps: u64,
    priority: u8,
    /// Only spawn once (set to true after first spawn)
    spawned: bool,
}

// =============================================================================
// Driver Config (per-driver KVs from etc/drivers.conf)
// =============================================================================

#[derive(Clone, Copy)]
struct ConfigKV {
    key: [u8; 32],
    key_len: u8,
    value: [u8; 64],
    value_len: u8,
}

#[derive(Clone, Copy)]
struct DriverConfig {
    /// Match key: binary name (e.g. "fatfsd") or trigger port path (e.g. "/pcie:0/nvme:0/block:0/fat:0").
    /// Port paths start with '/' and match against the service's trigger_port.
    name: [u8; 48],
    name_len: u8,
    kvs: [ConfigKV; MAX_CONFIG_KVS],
    kv_count: u8,
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
    /// Bus path — unique identity, e.g. "/uart:0/consoled"
    path: [u8; 48],
    path_len: u8,
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

    fn path_bytes(&self) -> &[u8] {
        &self.path[..self.path_len as usize]
    }
}

// =============================================================================
// Admin Client
// =============================================================================

struct AdminClient {
    channel: Channel,
}

// =============================================================================
// Pending Port (registered but not yet Ready)
// =============================================================================

const MAX_PENDING_PORTS: usize = 16;

#[derive(Clone, Copy)]
struct PendingPort {
    info: PortInfo,
    shmem_id: u32,
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
    service_rules: [Option<ServiceRule>; MAX_SERVICE_RULES],
    service_rule_count: usize,
    configs: [Option<DriverConfig>; MAX_DRIVER_CONFIGS],
    config_count: usize,
    pending_ports: [Option<PendingPort>; MAX_PENDING_PORTS],
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
            service_rules: [const { None }; MAX_SERVICE_RULES],
            service_rule_count: 0,
            configs: [const { None }; MAX_DRIVER_CONFIGS],
            config_count: 0,
            pending_ports: [const { None }; MAX_PENDING_PORTS],
        }
    }

    // =========================================================================
    // Boot
    // =========================================================================

    fn boot(&mut self) {
        // Load config from initrd (falls back to defaults)
        self.load_config();
        self.load_service_rules();
        self.load_driver_configs();

        // Register devd-query: port for shell admin commands
        match Port::register(b"devd-query:") {
            Ok(port) => {
                let h = port.handle();
                if let Err(e) = self.mux.add(h, MuxFilter::Readable) {
                    uerror!("devd2", "query_port_mux_failed"; err = e.as_str());
                }
                self.query_port = Some(port);
                udebug!("devd2", "query_port_created");
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

            // Subclass at [21..23] (u16 LE), 0xFFFF = match any
            let subclass = u16::from_le_bytes([buf[off + 21], buf[off + 22]]);

            self.rules[self.rule_count] = Some(LoadedRule {
                class, binary, binary_len: blen as u8, caps, priority,
                subclass,
            });
            self.rule_count += 1;
        }

        unotice!("devd2", "config_loaded"; rules = self.rule_count as u32);
    }

    fn load_default_rules(&mut self) {
        // (class, binary, priority, subclass)
        // subclass 0xFFFF = match any subclass within the class
        const ANY: u16 = 0xFFFF;
        let defaults: &[(PortClass, &[u8], u8, u16)] = &[
            (PortClass::Uart,              b"consoled", abi::priority::ABOVE_NORM, ANY),
            (PortClass::Klog,              b"klogd",    abi::priority::NORMAL,     ANY),
            (PortClass::Cpu,               b"cpud",     abi::priority::NORMAL,     ANY),
            (PortClass::Pcie,              b"pcied",    abi::priority::HIGH,       ANY),
            (PortClass::Usb,               b"usbd",     abi::priority::NORMAL,     ANY),
            (PortClass::StorageController, b"nvmed",    abi::priority::NORMAL,     ANY),
            (PortClass::Ethernet,          b"ethd",     abi::priority::NORMAL,     ANY),
            (PortClass::Pwm,               b"pwmd",     abi::priority::NORMAL,     ANY),
            // Block class: disambiguated by subclass
            (PortClass::Block,             b"partd",    abi::priority::NORMAL,     abi::port_subclass::BLOCK_RAW),
            (PortClass::Block,             b"fatfsd",   abi::priority::NORMAL,     abi::port_subclass::BLOCK_FAT16),
            (PortClass::Block,             b"fatfsd",   abi::priority::NORMAL,     abi::port_subclass::BLOCK_FAT32),
            (PortClass::Block,             b"fatfsd",   abi::priority::NORMAL,     abi::port_subclass::BLOCK_FAT32_LBA),
        ];
        for &(class, name, priority, subclass) in defaults {
            let mut binary = [0u8; 16];
            let blen = name.len().min(16);
            binary[..blen].copy_from_slice(&name[..blen]);
            self.rules[self.rule_count] = Some(LoadedRule {
                class, binary, binary_len: blen as u8, caps: DRIVER_CAPS, priority,
                subclass,
            });
            self.rule_count += 1;
        }
        uinfo!("devd2", "default_rules_loaded"; rules = self.rule_count as u32);
    }

    fn load_service_rules(&mut self) {
        // (dependency_binary, service_binary, priority)
        // When dependency reaches Running, spawn the service.
        let defaults: &[(&[u8], &[u8], u8)] = &[
            (b"fatfsd",  b"vfsd",  abi::priority::NORMAL),
        ];
        for &(dep, svc, priority) in defaults {
            let mut dep_binary = [0u8; 16];
            let dlen = dep.len().min(16);
            dep_binary[..dlen].copy_from_slice(&dep[..dlen]);
            let mut binary = [0u8; 16];
            let blen = svc.len().min(16);
            binary[..blen].copy_from_slice(&svc[..blen]);
            self.service_rules[self.service_rule_count] = Some(ServiceRule {
                dep_binary, dep_binary_len: dlen as u8,
                binary, binary_len: blen as u8,
                caps: DRIVER_CAPS, priority,
                spawned: false,
            });
            self.service_rule_count += 1;
        }
    }

    fn load_driver_configs(&mut self) {
        let mut buf = [0u8; 2048];
        let n = syscall::ramfs_read(b"etc/drivers.conf", &mut buf);
        if n < 4 {
            // No driver config file — that's fine, drivers just won't get initial config
            return;
        }
        let n = n as usize;

        let magic = u16::from_le_bytes([buf[0], buf[1]]);
        let version = buf[2];
        let count = buf[3] as usize;

        if magic != DRIVER_CONFIG_MAGIC || version != 1 {
            uwarn!("devd2", "driver_config_bad_format"; magic = magic as u32, version = version as u32);
            return;
        }

        let mut pos = 4;
        for _ in 0..count.min(MAX_DRIVER_CONFIGS) {
            if pos >= n { break; }

            // Driver name or trigger port path
            let name_len = buf[pos] as usize;
            pos += 1;
            if pos + name_len > n || name_len > 48 { break; }
            let mut name = [0u8; 48];
            name[..name_len].copy_from_slice(&buf[pos..pos + name_len]);
            pos += name_len;

            // KV count
            if pos >= n { break; }
            let kv_count = buf[pos] as usize;
            pos += 1;

            let mut kvs = [ConfigKV {
                key: [0u8; 32], key_len: 0,
                value: [0u8; 64], value_len: 0,
            }; MAX_CONFIG_KVS];

            let mut valid_kvs = 0u8;
            for ki in 0..kv_count.min(MAX_CONFIG_KVS) {
                // Key
                if pos >= n { break; }
                let klen = buf[pos] as usize;
                pos += 1;
                if pos + klen > n || klen > 32 { break; }
                kvs[ki].key[..klen].copy_from_slice(&buf[pos..pos + klen]);
                kvs[ki].key_len = klen as u8;
                pos += klen;

                // Value
                if pos >= n { break; }
                let vlen = buf[pos] as usize;
                pos += 1;
                if pos + vlen > n || vlen > 64 { break; }
                kvs[ki].value[..vlen].copy_from_slice(&buf[pos..pos + vlen]);
                kvs[ki].value_len = vlen as u8;
                pos += vlen;

                valid_kvs += 1;
            }

            self.configs[self.config_count] = Some(DriverConfig {
                name,
                name_len: name_len as u8,
                kvs,
                kv_count: valid_kvs,
            });
            self.config_count += 1;
        }

        uinfo!("devd2", "driver_configs_loaded"; count = self.config_count as u32);
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

        unotice!("devd2", "bus_discovery"; count = count as u32);

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
            self.match_and_spawn(port_info, 0);
        }
    }

    fn match_and_spawn(&mut self, port_info: &PortInfo, shmem_id: u32) {
        // Copy matched rule data to avoid borrow conflict with spawn_driver(&mut self)
        let matched = (0..self.rule_count).find_map(|i| {
            self.rules[i].as_ref().and_then(|rule| {
                if rule.class != port_info.port_class { return None; }
                // If rule has a subclass filter, port subclass must match
                if rule.subclass != 0xFFFF && rule.subclass != port_info.port_subclass {
                    return None;
                }
                Some((rule.binary, rule.binary_len, rule.caps, rule.priority))
            })
        });

        if let Some((binary_buf, binary_len, caps, priority)) = matched {
            let binary = core::str::from_utf8(&binary_buf[..binary_len as usize]).unwrap_or("?");
            let port_name = &port_info.name[..port_info.name_len as usize];
            self.spawn_driver(binary, port_name, caps, priority, port_info, shmem_id);
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
        port_info: &PortInfo,
        shmem_id: u32,
    ) {
        let metadata = unsafe { &port_info.metadata.raw };
        match exec_driver(binary.as_bytes(), port_name, caps, priority, metadata, shmem_id) {
            Ok(pid) => {
                uinfo!("devd2", "spawned"; binary = binary, pid = pid, port = core::str::from_utf8(port_name).unwrap_or("?"));
                self.add_service(pid, binary.as_bytes(), port_name, caps, priority);
            }
            Err(errno) => {
                uerror!("devd2", "spawn_failed"; binary = binary, errno = errno);
            }
        }
    }


    /// Spawn a service daemon (not a driver — no port, no bus framework).
    fn spawn_service(&mut self, binary: &str, caps: u64, priority: u8) {
        let mut path_buf = [0u8; 32];
        let path_len = {
            let mut w = WireWriter::new(&mut path_buf);
            w.put_bytes(b"bin/");
            w.put_bytes(binary.as_bytes());
            w.finish()
        };
        let path = core::str::from_utf8(&path_buf[..path_len]).unwrap_or("bin/???");

        // Minimal mailbox: just bus path and priority
        let svc_path_str = binary.as_bytes();
        let mut bus_path = [0u8; 48];
        let bus_path_len = {
            let mut w = WireWriter::new(&mut bus_path);
            w.put_bytes(b"/");
            w.put_bytes(svc_path_str);
            w.finish()
        };
        let mut mb_buf = [0u8; 256];
        let mut hdr = MailboxHeader::empty();
        hdr.priority = priority;
        hdr.flags = abi::mailbox_flags::PARENT_WRITTEN;
        hdr.kv_count = 1;
        let hdr_bytes = unsafe {
            core::slice::from_raw_parts(&hdr as *const MailboxHeader as *const u8, 64)
        };
        mb_buf[..64].copy_from_slice(hdr_bytes);
        let mut w = WireWriter::new(&mut mb_buf[64..]);
        w.put_len_u8_bytes(b"bus.path");
        w.put_len_u8_bytes(&bus_path[..bus_path_len]);

        match syscall::exec_with_mailbox(path, caps, &mb_buf) {
            Ok((pid, _)) => {
                uinfo!("devd2", "spawned_service"; binary = binary, pid = pid);
                self.add_service(pid, binary.as_bytes(), binary.as_bytes(), caps, priority);
            }
            Err(errno) => {
                uerror!("devd2", "service_spawn_failed"; binary = binary, errno = errno as u32);
            }
        }
    }

    fn add_service(&mut self, pid: u32, binary: &[u8], port_name: &[u8], caps: u64, priority: u8) {
        for slot in self.services.iter_mut() {
            if slot.is_none() {
                let mut name = [0u8; 16];
                let nlen = binary.len().min(16);
                name[..nlen].copy_from_slice(&binary[..nlen]);

                let mut trigger = [0u8; 32];
                let tlen = port_name.len().min(32);
                trigger[..tlen].copy_from_slice(&port_name[..tlen]);

                // Build path: /trigger_port/binary (e.g. /block:0/partd)
                let mut path = [0u8; 48];
                let plen = {
                    let mut w = WireWriter::new(&mut path);
                    if port_name.first() != Some(&b'/') {
                        w.put_bytes(b"/");
                    }
                    w.put_bytes(port_name);
                    w.put_bytes(b"/");
                    w.put_bytes(binary);
                    w.finish()
                };

                *slot = Some(Service {
                    pid,
                    name,
                    name_len: nlen as u8,
                    path,
                    path_len: plen as u8,
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
                // Always drain bus events — mux may not re-trigger for
                // messages that arrived while processing other handles
                self.handle_bus_message();
                continue;
            }

            let handle = event.handle;

            // Dispatch by handle identity
            if let Some(ref port) = self.query_port {
                if handle == port.handle() {
                    self.accept_admin_client();
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

            // Always drain bus events on every wake
            self.handle_bus_message();

            libos::ulog::flush();
        }
    }

    // =========================================================================
    // Service State
    // =========================================================================

    /// Update service state from a bus event (e.g. state=Ready from a driver).
    fn update_service_state(&mut self, sender: &[u8], state: &[u8]) {
        let new_state = match state {
            b"Ready" => ServiceState::Ready,
            b"Running" => ServiceState::Running,
            _ => return,
        };

        // Find the service and extract what we need before mutating
        let mut svc_name = [0u8; 16];
        let mut svc_name_len = 0u8;
        let mut svc_path = [0u8; 48];
        let mut svc_path_len = 0u8;
        let mut found = false;

        for slot in self.services.iter_mut() {
            if let Some(ref mut svc) = slot {
                if svc.path_bytes() == sender {
                    svc.state = new_state;
                    svc_name = svc.name;
                    svc_name_len = svc.name_len;
                    svc_path = svc.path;
                    svc_path_len = svc.path_len;
                    found = true;
                    // Ready = NOTICE (expected step), Running/Failed = INFO (final state transition)
                    let path_str = core::str::from_utf8(svc.path_bytes()).unwrap_or("?");
                    if new_state == ServiceState::Ready {
                        unotice!("devd2", "service_state"; path = path_str, state = svc.state.as_str());
                    } else {
                        uinfo!("devd2", "service_state"; path = path_str, state = svc.state.as_str());
                    }
                    break;
                }
            }
        }

        if !found { return; }

        // On Ready: send config KVs from file, then config.done
        if new_state == ServiceState::Ready {
            self.send_driver_config(
                &svc_name[..svc_name_len as usize],
                &svc_path[..svc_path_len as usize],
            );
        }

        // On Running: check service rules for dependent services to spawn
        if new_state == ServiceState::Running {
            let name = &svc_name[..svc_name_len as usize];
            self.check_service_rules(name);
        }
    }

    /// Check service rules — spawn services whose dependency just reached Running.
    fn check_service_rules(&mut self, dep_name: &[u8]) {
        for i in 0..self.service_rule_count {
            let rule = match &self.service_rules[i] {
                Some(r) if !r.spawned => r,
                _ => continue,
            };
            if &rule.dep_binary[..rule.dep_binary_len as usize] != dep_name {
                continue;
            }
            let binary = rule.binary;
            let binary_len = rule.binary_len;
            let caps = rule.caps;
            let priority = rule.priority;
            // Mark spawned before spawn to avoid re-entry
            if let Some(ref mut r) = self.service_rules[i] {
                r.spawned = true;
            }
            let bin_str = core::str::from_utf8(&binary[..binary_len as usize]).unwrap_or("?");
            self.spawn_service(bin_str, caps, priority);
        }
    }

    /// Send config KVs to a driver that just became Ready, then config.done.
    ///
    /// Matches config entries by binary name OR trigger port path.
    /// Port paths (starting with '/') match against the service's trigger port.
    fn send_driver_config(&mut self, name: &[u8], path: &[u8]) {
        // Extract trigger port from bus path: "/trigger/binary" → "/trigger"
        let trigger = {
            let p = if path.len() > 0 && path[0] == b'/' { &path[1..] } else { path };
            match p.iter().rposition(|&b| b == b'/') {
                Some(pos) => &path[..pos + 1], // include leading '/'
                None => path,
            }
        };

        for i in 0..self.config_count {
            if let Some(ref cfg) = self.configs[i] {
                let cfg_name = &cfg.name[..cfg.name_len as usize];
                // Match by binary name or trigger port path
                let matches = cfg_name == name
                    || (cfg_name.first() == Some(&b'/') && cfg_name == trigger);
                if !matches { continue; }

                // Send each KV as a SET
                for ki in 0..cfg.kv_count as usize {
                    let kv = &cfg.kvs[ki];
                    let key = &kv.key[..kv.key_len as usize];
                    let value = &kv.value[..kv.value_len as usize];
                    let _ = self.bus.send_set(path, key, value);
                }

                uinfo!("devd2", "config_sent"; name = core::str::from_utf8(name).unwrap_or("?"), kvs = cfg.kv_count as u32);
                // Don't break — multiple configs can match (by name AND by path)
            }
        }

        // Always send config.done — even if no config was found
        let _ = self.bus.send_set(path, b"config.done", b"");
    }

    /// Handle port.registered event — store port info, wait for port.state=Ready to spawn.
    /// Format: [class(1), subclass(1), name_len(1), name(N), metadata(24), shmem_id(4)]
    fn handle_port_registered(&mut self, _sender: &[u8], value: &[u8]) {
        if value.len() < 3 { return; }
        let class = PortClass::from_u8(value[0]);
        let subclass = value[1] as u16;
        let name_len = value[2] as usize;
        if value.len() < 3 + name_len { return; }

        let mut info = PortInfo::new(&value[3..3 + name_len], class);
        info.port_subclass = subclass;

        let meta_start = 3 + name_len;
        let mut shmem_id: u32 = 0;
        if value.len() >= meta_start + 24 {
            let mut raw = [0u8; 24];
            raw.copy_from_slice(&value[meta_start..meta_start + 24]);
            info.metadata = abi::PortMetadata { raw };

            if value.len() >= meta_start + 28 {
                shmem_id = u32::from_le_bytes([
                    value[meta_start + 24], value[meta_start + 25],
                    value[meta_start + 26], value[meta_start + 27],
                ]);
            }
        }

        // Store for later — spawn happens when port.state transitions to Ready
        for slot in self.pending_ports.iter_mut() {
            if slot.is_none() {
                *slot = Some(PendingPort { info, shmem_id });
                return;
            }
        }
        uwarn!("devd2", "pending_ports_full";);
    }

    /// Handle port.state event — spawn driver when port becomes Ready.
    /// Format: [state(1), name_len(1), name(N)]
    fn handle_port_state(&mut self, _sender: &[u8], value: &[u8]) {
        if value.len() < 2 { return; }
        let state = value[0];
        let name_len = value[1] as usize;
        if value.len() < 2 + name_len { return; }
        let name = &value[2..2 + name_len];

        // Only spawn on Ready transitions (port_event_state::READY = 1)
        if state != 1 { return; }

        // Find matching pending port by name
        let mut found = None;
        for (i, slot) in self.pending_ports.iter().enumerate() {
            if let Some(pp) = slot {
                let pp_name = &pp.info.name[..pp.info.name_len as usize];
                if pp_name == name {
                    found = Some((i, pp.info, pp.shmem_id));
                    break;
                }
            }
        }

        if let Some((idx, info, shmem_id)) = found {
            self.pending_ports[idx] = None;
            self.match_and_spawn(&info, shmem_id);
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

            let binary = &name_buf[..name_len as usize];
            let port_name = &port_buf[..port_len as usize];

            match exec_driver(binary, port_name, caps, priority, &[0u8; 24], 0) {
                Ok(pid) => {
                    uinfo!("devd2", "restarted";
                        binary = core::str::from_utf8(binary).unwrap_or("?"), pid = pid);
                    if let Some(ref mut svc) = self.services[i] {
                        svc.pid = pid;
                        svc.state = ServiceState::Starting;
                    }
                }
                Err(errno) => {
                    uerror!("devd2", "restart_failed";
                        binary = core::str::from_utf8(binary).unwrap_or("?"), errno = errno as u32);
                }
            }
        }
    }

    // =========================================================================
    // Bus Message Handling
    // =========================================================================

    /// Drain all queued bus messages (call after boot, before event loop).
    fn drain_bus_events(&mut self) {
        loop {
            let mut buf = [0u8; MSG_BUF_SIZE];
            match self.bus.try_recv(&mut buf) {
                Ok(Some(n)) if n > 0 => {
                    self.process_bus_message(&buf[..n]);
                }
                _ => break,
            }
        }
    }

    fn handle_bus_message(&mut self) {
        // Drain all queued messages — mux is edge-triggered, so we
        // won't get another wake for messages already in the buffer.
        loop {
            let mut buf = [0u8; MSG_BUF_SIZE];
            match self.bus.try_recv(&mut buf) {
                Ok(Some(n)) if n > 0 => self.process_bus_message(&buf[..n]),
                _ => break,
            }
        }
    }

    fn process_bus_message(&mut self, msg: &[u8]) {
        let header = match BusMsgHeader::read_from(msg) {
            Some(h) => h,
            None => return,
        };
        let (addr, key, value) = extract_bus_fields(msg, &header);

        match header.msg_type {
            bus_msg_type::EVENT => {
                if key == b"state" {
                    self.update_service_state(addr, value);
                } else if key == b"port.registered" {
                    self.handle_port_registered(addr, value);
                } else if key == b"port.state" {
                    self.handle_port_state(addr, value);
                }
            }
            bus_msg_type::GET => {
                if key == b"services" || key == b"list" {
                    let mut reply = [0u8; 512];
                    let pos = self.format_service_list(&mut reply);
                    let _ = self.bus.send_reply(header.seq_id, &reply[..pos]);
                } else {
                    let _ = self.bus.send_error_reply(header.seq_id, b"unknown key");
                }
            }
            _ => {}
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
        let n = match self.recv_admin(idx, &mut buf) {
            Some(n) => n,
            None => return,
        };

        let parsed = match parse_config_cmd(&buf[..n]) {
            Some(p) => p,
            None => return,
        };

        // Local queries devd2 can answer without busd
        if parsed.key == b"services" || parsed.key == b"list" {
            let mut reply = [0u8; 512];
            let pos = self.format_service_list(&mut reply);
            self.send_admin_reply_buf(idx, &reply[..pos]);
            return;
        }

        if parsed.is_set {
            self.handle_config_set(&parsed, idx);
        } else if parsed.service.is_empty() {
            self.handle_broadcast_get(parsed.key, idx);
        } else {
            self.handle_targeted_get(parsed.service, parsed.key, idx);
        }
    }

    /// Receive from admin client. Returns None on disconnect or empty read.
    fn recv_admin(&mut self, idx: usize, buf: &mut [u8]) -> Option<usize> {
        let client = self.admin_clients[idx].as_mut()?;
        let handle = client.channel.handle();
        match client.channel.try_recv(buf) {
            Ok(Some(n)) if n > 0 => Some(n),
            Ok(Some(_)) | Ok(None) => None,
            Err(_) => {
                let _ = self.mux.remove(handle);
                self.admin_clients[idx] = None;
                None
            }
        }
    }

    /// Resolve a service name to its bus path. If already a path (starts with /),
    /// return as-is. Otherwise, find the first service with matching binary name.
    fn resolve_service_addr(&self, service: &[u8]) -> Option<([u8; 48], u8)> {
        if service.starts_with(b"/") {
            // Already a path — use directly
            let mut path = [0u8; 48];
            let len = service.len().min(48);
            path[..len].copy_from_slice(&service[..len]);
            return Some((path, len as u8));
        }
        for slot in &self.services {
            if let Some(svc) = slot {
                if &svc.name[..svc.name_len as usize] == service {
                    return Some((svc.path, svc.path_len));
                }
            }
        }
        None
    }

    fn handle_targeted_get(&mut self, service: &[u8], key: &[u8], admin_idx: usize) {
        // Path-based query (starts with '/') — single target
        if service.starts_with(b"/") {
            let mut buf = [0u8; 512];
            let pos = {
                let mut w = WireWriter::new(&mut buf);
                match self.bus.query(service, key, 500_000_000) {
                    Ok(qr) if !qr.is_error => { w.put_bytes(qr.value()); }
                    Ok(qr) => write_error(&mut w, qr.value()),
                    Err(_) => write_error(&mut w, b"timeout"),
                };
                w.finish()
            };
            self.send_admin_reply_buf(admin_idx, &buf[..pos]);
            return;
        }

        // Name-based query — find all matching services
        let snapshot = self.snapshot_service_paths();
        let mut matches = [([0u8; 48], 0u8); MAX_SERVICES];
        let mut match_count = 0;
        for (i, slot) in self.services.iter().enumerate() {
            if let Some(svc) = slot {
                if &svc.name[..svc.name_len as usize] == service {
                    matches[match_count] = (svc.path, svc.path_len);
                    match_count += 1;
                }
            }
        }

        if match_count == 0 {
            let mut buf = [0u8; 64];
            let pos = { let mut w = WireWriter::new(&mut buf); write_error(&mut w, b"unknown service"); w.finish() };
            self.send_admin_reply_buf(admin_idx, &buf[..pos]);
            return;
        }

        // Single match — simple response (no path header)
        if match_count == 1 {
            let path = &matches[0].0[..matches[0].1 as usize];
            let mut buf = [0u8; 512];
            let pos = {
                let mut w = WireWriter::new(&mut buf);
                match self.bus.query(path, key, 500_000_000) {
                    Ok(qr) if !qr.is_error => { w.put_bytes(qr.value()); }
                    Ok(qr) => write_error(&mut w, qr.value()),
                    Err(_) => write_error(&mut w, b"timeout"),
                };
                w.finish()
            };
            self.send_admin_reply_buf(admin_idx, &buf[..pos]);
            return;
        }

        // Multiple matches — aggregate with path headers
        let mut buf = [0u8; 1024];
        let pos = {
            let mut w = WireWriter::new(&mut buf);
            for i in 0..match_count {
                let path = &matches[i].0[..matches[i].1 as usize];
                match self.bus.query(path, key, 200_000_000) {
                    Ok(qr) if !qr.is_error && !qr.value().is_empty() => {
                        let val = qr.value();
                        w.put_bytes(b"[");
                        w.put_bytes(path);
                        w.put_bytes(b"]\n");
                        w.put_bytes(val);
                        if !val.ends_with(b"\n") { w.put_bytes(b"\n"); }
                    }
                    _ => {}
                }
            }
            w.finish()
        };
        self.send_admin_reply_buf(admin_idx, &buf[..pos]);
    }

    fn handle_broadcast_get(&mut self, key: &[u8], admin_idx: usize) {
        let mut buf = [0u8; 1024];
        let pos = {
            let mut w = WireWriter::new(&mut buf);

            // Snapshot service paths to avoid borrowing self during bus queries
            let svc_snapshot = self.snapshot_service_paths();

            for (path, len) in &svc_snapshot {
                if *len == 0 { break; }
                let path = &path[..*len as usize];
                match self.bus.query(path, key, 200_000_000) {
                    Ok(qr) if !qr.is_error && !qr.value().is_empty() => {
                        let val = qr.value();
                        w.put_bytes(b"[");
                        w.put_bytes(path);
                        w.put_bytes(b"]\n");
                        w.put_bytes(val);
                        if !val.ends_with(b"\n") {
                            w.put_bytes(b"\n");
                        }
                    }
                    _ => {}
                }
            }
            w.finish()
        };
        self.send_admin_reply_buf(admin_idx, &buf[..pos]);
    }

    fn handle_config_set(&mut self, cmd: &ConfigCmd<'_>, admin_idx: usize) {
        let addr_resolved;
        let addr: &[u8] = if cmd.service.is_empty() {
            b"*"
        } else {
            match self.resolve_service_addr(cmd.service) {
                Some((path, len)) => { addr_resolved = (path, len); &addr_resolved.0[..addr_resolved.1 as usize] }
                None => {
                    let mut buf = [0u8; 64];
                    let pos = { let mut w = WireWriter::new(&mut buf); write_error(&mut w, b"unknown service"); w.finish() };
                    self.send_admin_reply_buf(admin_idx, &buf[..pos]);
                    return;
                }
            }
        };

        let mut buf = [0u8; 256];
        let pos = {
            let mut w = WireWriter::new(&mut buf);
            match self.bus.send_set(addr, cmd.key, cmd.value) {
                Ok(seq) => match self.bus.wait_reply(seq, 500_000_000) {
                    Ok(qr) if !qr.is_error => {
                        let val = qr.value();
                        if val.is_empty() { w.put_bytes(b"OK\n"); }
                        else { w.put_bytes(val); }
                    }
                    Ok(qr) => write_error(&mut w, qr.value()),
                    Err(_) => write_error(&mut w, b"timeout"),
                },
                Err(_) => write_error(&mut w, b"send failed"),
            };
            w.finish()
        };
        self.send_admin_reply_buf(admin_idx, &buf[..pos]);
    }

    /// Send a reply buffer to an admin client.
    fn send_admin_reply_buf(&self, idx: usize, data: &[u8]) {
        if let Some(ref client) = self.admin_clients[idx] {
            let _ = client.channel.send(data);
        }
    }

    /// Snapshot service paths into a stack array for iteration without borrowing self.
    fn snapshot_service_paths(&self) -> [([u8; 48], u8); MAX_SERVICES] {
        let mut out = [([0u8; 48], 0u8); MAX_SERVICES];
        for (i, slot) in self.services.iter().enumerate() {
            if let Some(svc) = slot {
                out[i] = (svc.path, svc.path_len);
            }
        }
        out
    }

    /// Format "path state pid\n" for each service into buf. Returns bytes written.
    fn format_service_list(&self, buf: &mut [u8]) -> usize {
        let mut w = WireWriter::new(buf);
        for slot in &self.services {
            if let Some(svc) = slot {
                w.put_bytes(svc.path_bytes());
                w.put_bytes(b" ");
                w.put_bytes(svc.state.as_str().as_bytes());
                w.put_bytes(b" ");
                w.put_bytes(format_u32(svc.pid).as_bytes());
                w.put_bytes(b"\n");
            }
        }
        w.finish()
    }
}

// =============================================================================
// Helpers
// =============================================================================

/// Extract addr, key, value slices from a BusMsg buffer.
fn extract_bus_fields<'a>(buf: &'a [u8], header: &BusMsgHeader) -> (&'a [u8], &'a [u8], &'a [u8]) {
    let p = BusMsgHeader::SIZE;
    let addr_end = p + header.addr_len as usize;
    let key_end = addr_end + header.key_len as usize;
    let val_end = (key_end + header.value_len as usize).min(buf.len());
    (
        &buf[p..addr_end.min(buf.len())],
        &buf[addr_end..key_end.min(buf.len())],
        &buf[key_end..val_end],
    )
}

/// Write "ERR <msg>\n" into a WireWriter.
fn write_error(w: &mut WireWriter<'_>, msg: &[u8]) {
    w.put_bytes(b"ERR ");
    w.put_bytes(msg);
    w.put_bytes(b"\n");
}

// =============================================================================
// CONFIG Command Parser
// =============================================================================

struct ConfigCmd<'a> {
    service: &'a [u8],  // empty = broadcast
    is_set: bool,
    key: &'a [u8],      // empty = full summary
    value: &'a [u8],    // only for SET
}

/// Parse the text CONFIG protocol from config.rs.
///
/// Formats:
///   CONFIG <service> GET [key]\n
///   CONFIG GET [key]\n
///   CONFIG <service> SET <key> <value>\n
///   CONFIG SET <key> <value>\n
fn parse_config_cmd(cmd: &[u8]) -> Option<ConfigCmd<'_>> {
    // Strip trailing newline/whitespace
    let mut end = cmd.len();
    while end > 0 && (cmd[end - 1] == b'\n' || cmd[end - 1] == b'\r' || cmd[end - 1] == b' ') {
        end -= 1;
    }
    let cmd = &cmd[..end];

    // Must start with "CONFIG "
    if cmd.len() < 10 || &cmd[..7] != b"CONFIG " {
        return None;
    }
    let rest = &cmd[7..];

    // Split into up to 4 space-separated tokens
    let mut tokens: [&[u8]; 4] = [&[], &[], &[], &[]];
    let mut token_count = 0;
    let mut pos = 0;

    while pos < rest.len() && token_count < 4 {
        // Skip spaces
        while pos < rest.len() && rest[pos] == b' ' { pos += 1; }
        if pos >= rest.len() { break; }

        let start = pos;
        // For the 4th token (SET value), take everything remaining
        if token_count == 3 {
            tokens[token_count] = &rest[start..];
            token_count += 1;
            break;
        }
        while pos < rest.len() && rest[pos] != b' ' { pos += 1; }
        tokens[token_count] = &rest[start..pos];
        token_count += 1;
    }

    if token_count == 0 { return None; }

    // First token is GET/SET → broadcast (no service name).
    // Otherwise first token is the service, second must be GET/SET.
    let (service, op_idx) = if eq_ci(tokens[0], b"GET") || eq_ci(tokens[0], b"SET") {
        (&[] as &[u8], 0)
    } else if token_count >= 2 && (eq_ci(tokens[1], b"GET") || eq_ci(tokens[1], b"SET")) {
        (tokens[0], 1)
    } else {
        return None;
    };

    let is_set = eq_ci(tokens[op_idx], b"SET");
    let key = tokens.get(op_idx + 1).copied().unwrap_or(&[]);
    let value = if is_set { tokens.get(op_idx + 2).copied().unwrap_or(&[]) } else { &[] };

    Some(ConfigCmd { service, is_set, key, value })
}

/// Case-insensitive byte comparison.
fn eq_ci(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() { return false; }
    for i in 0..a.len() {
        if a[i].to_ascii_uppercase() != b[i].to_ascii_uppercase() { return false; }
    }
    true
}

/// Build "bin/<binary>" path, create mailbox, exec. Returns pid on success.
fn exec_driver(binary: &[u8], port_name: &[u8], caps: u64, priority: u8, metadata: &[u8; 24], shmem_id: u32) -> Result<u32, u32> {
    // Build bus path: /port_name/binary (e.g. /block:0/partd)
    let mut bus_path = [0u8; 48];
    let bus_path_len = {
        let mut w = WireWriter::new(&mut bus_path);
        if port_name.first() != Some(&b'/') {
            w.put_bytes(b"/");
        }
        w.put_bytes(port_name);
        w.put_bytes(b"/");
        w.put_bytes(binary);
        w.finish()
    };

    let mb_buf = build_mailbox(port_name, &bus_path[..bus_path_len], priority, metadata, shmem_id);

    let mut path_buf = [0u8; 32];
    let path_len = {
        let mut w = WireWriter::new(&mut path_buf);
        w.put_bytes(b"bin/");
        w.put_bytes(binary);
        w.finish()
    };
    let path = core::str::from_utf8(&path_buf[..path_len]).unwrap_or("bin/???");

    match syscall::exec_with_mailbox(path, caps, &mb_buf) {
        Ok((pid, _)) => Ok(pid),
        Err(errno) => Err(errno as u32),
    }
}

/// Build a mailbox buffer with port name, bus path, and metadata for a spawned child.
fn build_mailbox(port_name: &[u8], bus_path: &[u8], priority: u8, metadata: &[u8; 24], shmem_id: u32) -> [u8; 256] {
    let mut buf = [0u8; 256];

    // Check if metadata is non-empty (any non-zero byte)
    let has_meta = metadata.iter().any(|&b| b != 0);
    let has_shmem = shmem_id != 0;

    // Header at [0..64)
    let mut hdr = MailboxHeader::empty();
    hdr.priority = priority;
    hdr.flags = abi::mailbox_flags::PARENT_WRITTEN;
    hdr.kv_count = 2 + has_meta as u8 + has_shmem as u8;

    let hdr_bytes = unsafe {
        core::slice::from_raw_parts(&hdr as *const MailboxHeader as *const u8, 64)
    };
    buf[..64].copy_from_slice(hdr_bytes);

    // KVs at [64..)
    let mut w = WireWriter::new(&mut buf[64..]);
    w.put_len_u8_bytes(b"port.name");
    w.put_len_u8_bytes(port_name);
    w.put_len_u8_bytes(b"bus.path");
    w.put_len_u8_bytes(bus_path);
    if has_meta {
        w.put_len_u8_bytes(b"port.metadata");
        w.put_len_u8_bytes(metadata);
    }
    if has_shmem {
        w.put_len_u8_bytes(b"port.shmem_id");
        w.put_len_u8_bytes(&shmem_id.to_le_bytes());
    }

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

    unotice!("devd2", "starting");
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

    unotice!("devd2", "busd_connected");

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

    // Drain any bus events queued during boot (mux is edge-triggered —
    // messages that arrived before the event loop won't trigger a wake)
    devd.drain_bus_events();

    // Flush logs before entering event loop
    libos::ulog::flush();

    // Run event loop
    devd.run()
}
