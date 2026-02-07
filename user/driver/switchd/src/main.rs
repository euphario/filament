//! MT7531 Gigabit Switch Driver
//!
//! Manages the MT7531 internal switch on MT7988A SoC:
//! - VLAN configuration (802.1Q)
//! - FDB (MAC address table) management
//! - STP state control
//! - Port mirroring
//! - MIB counters (port statistics)
//!
//! Spawned by devd when ethd registers the switch: port.
//! Registers individual switch ports (lan0:-lan3:, wan:) as Network ports.

#![no_std]
#![no_main]

extern crate abi;

use abi::{PortInfo, PortClass, PortState, port_subclass};
use userlib::{uinfo, uerror};
use userlib::mmio::MmioRegion;
use userlib::bus::{Driver, BusCtx, BusMsg, Disposition};
use userlib::bus_runtime::driver_main;

// =============================================================================
// MT7531 Switch Registers
// =============================================================================

// Switch base address (memory-mapped within MT7988A)
const SWITCH_BASE: usize = 0x15020000;
const SWITCH_SIZE: usize = 0x8000;

// Port Control Registers
const MT7530_PCR_P: fn(u8) -> u32 = |p| 0x2004 + (p as u32) * 0x100;
const MT7530_PVC_P: fn(u8) -> u32 = |p| 0x2010 + (p as u32) * 0x100;
const MT7530_PMCR_P: fn(u8) -> u32 = |p| 0x3000 + (p as u32) * 0x100;

// VLAN Registers
const MT7530_VTCR: u32 = 0x90;       // VLAN Table Control
const MT7530_VAWD1: u32 = 0x94;      // VLAN Table Write Data 1
const MT7530_VAWD2: u32 = 0x98;      // VLAN Table Write Data 2
const MT7530_VARD0: u32 = 0x9C;      // VLAN Table Read Data 0
const MT7530_VARD1: u32 = 0xA0;      // VLAN Table Read Data 1

// VLAN control bits
const VTCR_BUSY: u32 = 1 << 31;
const VTCR_INVALID: u32 = 1 << 16;
const VTCR_FUNC_RD_VID: u32 = 0 << 12;
const VTCR_FUNC_WR_VID: u32 = 1 << 12;

// VAWD1 bits
const VAWD1_IVL_MAC: u32 = 1 << 30;
const VAWD1_VALID: u32 = 1 << 0;

// FDB (Address Table) Registers
const MT7530_ATA1: u32 = 0x74;       // MAC Address bytes 0-3
const MT7530_ATA2: u32 = 0x78;       // MAC Address bytes 4-5 + VID
const MT7530_ATWD: u32 = 0x7C;       // Address Table Write Data
const MT7530_ATC: u32 = 0x80;        // Address Table Control
const MT7530_ATRD: u32 = 0x8C;       // Address Table Read Data

// ATC bits
const ATC_BUSY: u32 = 1 << 15;
const ATC_SRCH_END: u32 = 1 << 14;
const ATC_SRCH_HIT: u32 = 1 << 13;
const ATC_SAT_SRCH_START: u32 = 0x4 << 0;
const ATC_SAT_SRCH_NEXT: u32 = 0x5 << 0;

// MIB Counter base (per-port)
const MT7530_MIB_PORT: fn(u8) -> u32 = |p| 0x4000 + (p as u32) * 0x100;

// Port counts
const NUM_PORTS: u8 = 5;       // lan0-lan3 + wan
const CPU_PORT: u8 = 6;        // Internal CPU port

// =============================================================================
// Switch Driver
// =============================================================================

struct SwitchDriver {
    sw: Option<MmioRegion>,
    ports_registered: bool,
}

impl SwitchDriver {
    fn new() -> Self {
        Self {
            sw: None,
            ports_registered: false,
        }
    }

    // =========================================================================
    // Register Access
    // =========================================================================

    fn read(&self, reg: u32) -> u32 {
        self.sw.as_ref().map(|sw| sw.read32(reg as usize)).unwrap_or(0)
    }

    fn write(&self, reg: u32, val: u32) {
        if let Some(sw) = &self.sw {
            sw.write32(reg as usize, val);
        }
    }

    fn rmw(&self, reg: u32, clr: u32, set: u32) {
        let val = self.read(reg);
        self.write(reg, (val & !clr) | set);
    }

    // =========================================================================
    // VLAN Operations
    // =========================================================================

    fn wait_vtcr(&self) -> bool {
        for _ in 0..1000 {
            let val = self.read(MT7530_VTCR);
            if val & VTCR_BUSY == 0 {
                return (val & VTCR_INVALID) == 0;
            }
            userlib::mmio::delay_us(10);
        }
        false
    }

    fn vlan_read(&self, vid: u16) -> Option<(u8, u8)> {
        // Issue read command
        self.write(MT7530_VTCR, VTCR_BUSY | VTCR_FUNC_RD_VID | (vid as u32));
        if !self.wait_vtcr() {
            return None;
        }

        let vawd1 = self.read(MT7530_VAWD1);
        if vawd1 & VAWD1_VALID == 0 {
            return None;
        }

        let members = ((vawd1 >> 16) & 0x7f) as u8;
        let vawd2 = self.read(MT7530_VAWD2);

        // Extract untagged ports from VAWD2 (2 bits per port, 0=untag)
        let mut untagged = 0u8;
        for p in 0..7 {
            if (vawd2 >> (p * 2)) & 3 == 0 {
                untagged |= 1 << p;
            }
        }
        Some((members, untagged))
    }

    fn vlan_write(&self, vid: u16, members: u8, untagged: u8) -> bool {
        let vawd1 = VAWD1_IVL_MAC | ((members as u32) << 16) | VAWD1_VALID;
        let mut vawd2 = 0u32;
        for p in 0..7 {
            let tag_mode = if (untagged >> p) & 1 != 0 { 0 } else { 2 };
            vawd2 |= tag_mode << (p * 2);
        }

        self.write(MT7530_VAWD1, vawd1);
        self.write(MT7530_VAWD2, vawd2);
        self.write(MT7530_VTCR, VTCR_BUSY | VTCR_FUNC_WR_VID | (vid as u32));
        self.wait_vtcr()
    }

    fn vlan_delete(&self, vid: u16) -> bool {
        self.write(MT7530_VAWD1, 0);
        self.write(MT7530_VAWD2, 0);
        self.write(MT7530_VTCR, VTCR_BUSY | VTCR_FUNC_WR_VID | (vid as u32));
        self.wait_vtcr()
    }

    // =========================================================================
    // FDB Operations
    // =========================================================================

    fn wait_atc(&self) -> bool {
        for _ in 0..1000 {
            let val = self.read(MT7530_ATC);
            if val & ATC_BUSY == 0 {
                return true;
            }
            userlib::mmio::delay_us(10);
        }
        false
    }

    // =========================================================================
    // MIB Counters
    // =========================================================================

    fn read_mib(&self, port: u8, offset: u32) -> u32 {
        if port >= 7 {
            return 0;
        }
        self.read(MT7530_MIB_PORT(port) + offset)
    }

    // =========================================================================
    // Port Link Status
    // =========================================================================

    fn port_link_status(&self, port: u8) -> (bool, u32) {
        if port >= 7 {
            return (false, 0);
        }
        let pmcr = self.read(MT7530_PMCR_P(port));
        let link = (pmcr & (1 << 24)) != 0;  // MAC_LNK_STS
        let speed = match (pmcr >> 28) & 3 {
            0 => 10,
            1 => 100,
            2 => 1000,
            _ => 0,
        };
        (link, speed)
    }

    // =========================================================================
    // Config API Helpers
    // =========================================================================

    fn config_vlan_list(&self, buf: &mut [u8]) -> usize {
        let mut offset = 0;
        for vid in 1..4096u16 {
            if let Some((members, untagged)) = self.vlan_read(vid) {
                if members != 0 {
                    let n = fmt_line(
                        &mut buf[offset..],
                        &[b"vid=", &fmt_u16(vid), b" members=0x", &fmt_hex8(members),
                          b" untagged=0x", &fmt_hex8(untagged), b"\n"],
                    );
                    offset += n;
                    if offset + 64 > buf.len() {
                        break;
                    }
                }
            }
        }
        offset
    }

    fn config_vlan_get(&self, vid: u16, buf: &mut [u8]) -> usize {
        if let Some((members, untagged)) = self.vlan_read(vid) {
            fmt_line(buf, &[b"vid=", &fmt_u16(vid), b" members=0x", &fmt_hex8(members),
                           b" untagged=0x", &fmt_hex8(untagged), b"\n"])
        } else {
            copy_to(buf, b"ERR vlan not found\n")
        }
    }

    fn config_vlan_set(&self, vid: u16, val: &str, buf: &mut [u8]) -> usize {
        // Parse "members,untagged" e.g., "0x1f,0x0f"
        let parts: [&str; 2] = {
            let mut iter = val.split(',');
            [iter.next().unwrap_or(""), iter.next().unwrap_or("")]
        };

        let members = parse_hex_u8(parts[0]).unwrap_or(0);
        let untagged = parse_hex_u8(parts[1]).unwrap_or(0);

        if self.vlan_write(vid, members, untagged) {
            copy_to(buf, b"OK\n")
        } else {
            copy_to(buf, b"ERR vlan write failed\n")
        }
    }

    fn config_vlan_del(&self, val: &str, buf: &mut [u8]) -> usize {
        let vid: u16 = val.trim().parse().unwrap_or(0);
        if vid == 0 {
            return copy_to(buf, b"ERR invalid vid\n");
        }
        if self.vlan_delete(vid) {
            copy_to(buf, b"OK\n")
        } else {
            copy_to(buf, b"ERR vlan delete failed\n")
        }
    }

    fn config_stats_get(&self, port: u8, buf: &mut [u8]) -> usize {
        if port >= NUM_PORTS {
            return copy_to(buf, b"ERR invalid port\n");
        }

        // Read common MIB counters
        let tx_bytes = self.read_mib(port, 0x00);  // TxByteCnt
        let rx_bytes = self.read_mib(port, 0x24);  // RxByteCnt
        let tx_pkts = self.read_mib(port, 0x10);   // TxPktCnt
        let rx_pkts = self.read_mib(port, 0x30);   // RxPktCnt
        let tx_drop = self.read_mib(port, 0x18);   // TxDropCnt
        let rx_drop = self.read_mib(port, 0x60);   // RxDropCnt

        fmt_line(buf, &[
            b"tx_bytes=", &fmt_u32(tx_bytes), b" rx_bytes=", &fmt_u32(rx_bytes), b"\n",
            b"tx_pkts=", &fmt_u32(tx_pkts), b" rx_pkts=", &fmt_u32(rx_pkts), b"\n",
            b"tx_drop=", &fmt_u32(tx_drop), b" rx_drop=", &fmt_u32(rx_drop), b"\n",
        ])
    }

    fn config_link_get(&self, port: u8, buf: &mut [u8]) -> usize {
        if port >= NUM_PORTS {
            return copy_to(buf, b"ERR invalid port\n");
        }

        let (link, speed) = self.port_link_status(port);
        fmt_line(buf, &[
            b"port=", &fmt_u8(port),
            b" link=", if link { b"up" } else { b"down" },
            b" speed=", &fmt_u32(speed), b"Mbps\n",
        ])
    }

    fn config_ports_get(&self, buf: &mut [u8]) -> usize {
        let mut offset = 0;
        for port in 0..NUM_PORTS {
            let (link, speed) = self.port_link_status(port);
            let name = match port {
                0 => b"lan0",
                1 => b"lan1",
                2 => b"lan2",
                3 => b"lan3",
                4 => b"wan ",
                _ => b"????",
            };
            let n = fmt_line(&mut buf[offset..], &[
                name, b": ", if link { b"up  " } else { b"down" },
                b" ", &fmt_u32(speed), b"Mbps\n",
            ]);
            offset += n;
        }
        offset
    }
}

impl Driver for SwitchDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), userlib::bus::BusError> {
        uinfo!("switchd", "init_start";);

        // Map switch MMIO region
        match MmioRegion::open(SWITCH_BASE as u64, SWITCH_SIZE as u64) {
            Some(sw) => {
                uinfo!("switchd", "mmio_mapped"; base = SWITCH_BASE as u32, size = SWITCH_SIZE as u32);
                self.sw = Some(sw);
            }
            None => {
                uerror!("switchd", "mmio_map_failed";);
                return Err(userlib::bus::BusError::SpawnFailed);
            }
        }

        // Read switch ID to verify access
        let id = self.read(0x7ffc);
        uinfo!("switchd", "switch_id"; id = id);

        // Register switch ports
        let port_names: [&[u8]; 5] = [b"lan0:", b"lan1:", b"lan2:", b"lan3:", b"wan:"];
        for (i, name) in port_names.iter().enumerate() {
            let mut info = PortInfo::new(name, PortClass::Network);
            info.port_subclass = port_subclass::NET_SWITCH_PORT;

            if let Err(e) = ctx.register_port_with_info(&info, 0) {
                uerror!("switchd", "port_register_failed"; port = i as u32, err = e as u32);
            } else {
                let _ = ctx.set_port_state(name, PortState::Ready);
                uinfo!("switchd", "port_registered"; port = i as u32);
            }
        }
        self.ports_registered = true;

        uinfo!("switchd", "init_done";);
        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Handled
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        let key_str = core::str::from_utf8(key).unwrap_or("");

        if key_str == "ports" {
            return self.config_ports_get(buf);
        }
        if key_str == "vlan" {
            return self.config_vlan_list(buf);
        }
        if key_str.starts_with("vlan.") {
            if let Ok(vid) = key_str[5..].parse::<u16>() {
                return self.config_vlan_get(vid, buf);
            }
        }
        if key_str.starts_with("stats.") {
            if let Ok(port) = key_str[6..].parse::<u8>() {
                return self.config_stats_get(port, buf);
            }
        }
        if key_str.starts_with("link.") {
            if let Ok(port) = key_str[5..].parse::<u8>() {
                return self.config_link_get(port, buf);
            }
        }

        0
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        let key_str = core::str::from_utf8(key).unwrap_or("");
        let val_str = core::str::from_utf8(value).unwrap_or("");

        if key_str.starts_with("vlan.") && key_str != "vlan.del" {
            if let Ok(vid) = key_str[5..].parse::<u16>() {
                return self.config_vlan_set(vid, val_str, buf);
            }
        }
        if key_str == "vlan.del" {
            return self.config_vlan_del(val_str, buf);
        }

        copy_to(buf, b"ERR unknown key\n")
    }
}

// =============================================================================
// Formatting Helpers (no_std)
// =============================================================================

fn copy_to(buf: &mut [u8], src: &[u8]) -> usize {
    let len = src.len().min(buf.len());
    buf[..len].copy_from_slice(&src[..len]);
    len
}

fn fmt_line(buf: &mut [u8], parts: &[&[u8]]) -> usize {
    let mut offset = 0;
    for part in parts {
        let len = part.len().min(buf.len().saturating_sub(offset));
        buf[offset..offset + len].copy_from_slice(&part[..len]);
        offset += len;
    }
    offset
}

fn fmt_u8(val: u8) -> [u8; 3] {
    let mut buf = [b' '; 3];
    let mut n = val;
    let mut i = 2;
    loop {
        buf[i] = b'0' + (n % 10);
        n /= 10;
        if n == 0 || i == 0 {
            break;
        }
        i -= 1;
    }
    buf
}

fn fmt_u16(val: u16) -> [u8; 5] {
    let mut buf = [b' '; 5];
    let mut n = val;
    let mut i = 4;
    loop {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 || i == 0 {
            break;
        }
        i -= 1;
    }
    buf
}

fn fmt_u32(val: u32) -> [u8; 10] {
    let mut buf = [b' '; 10];
    let mut n = val;
    let mut i = 9;
    loop {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 || i == 0 {
            break;
        }
        i -= 1;
    }
    buf
}

fn fmt_hex8(val: u8) -> [u8; 2] {
    const HEX: &[u8] = b"0123456789abcdef";
    [HEX[(val >> 4) as usize], HEX[(val & 0xf) as usize]]
}

fn parse_hex_u8(s: &str) -> Option<u8> {
    let s = s.trim();
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    u8::from_str_radix(s, 16).ok()
}

// =============================================================================
// Entry Point
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    driver_main(b"switchd", SwitchDriver::new());
}
