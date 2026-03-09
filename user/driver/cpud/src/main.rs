#![no_std]
#![no_main]

extern crate libsys;

use libos::bus::{BusCtx, BusError, BusMsg, ConfigKey, Disposition, Driver, KernelBusId};
use libos::bus_runtime::driver_main;

const MAX_CPUS: usize = 4;
const MAX_OPPS: usize = 8;

// Governor IDs (matches abi::governor)
const GOV_PERFORMANCE: u8 = 0;
const GOV_BALANCED: u8 = 1;
const GOV_POWERSAVE: u8 = 2;

// Bus protocol message types
const MSG_CPU_STATE_UPDATE: u8 = 32;
const MSG_SET_GOVERNOR: u8 = 33;
const MSG_SET_OPP: u8 = 34;
const MSG_OPP_TABLE: u8 = 35;

struct CpuPowerDriver {
    bus_id: Option<KernelBusId>,
    governor: u8,
    num_cpus: u8,
    active_cpus: u8,
    current_opp: u8,
    freq_khz: u32,
    util: [u8; MAX_CPUS],
    opp_count: u8,
    opp_freqs: [u32; MAX_OPPS],
}

impl CpuPowerDriver {
    fn new() -> Self {
        Self {
            bus_id: None,
            governor: GOV_PERFORMANCE,
            num_cpus: 0,
            active_cpus: 0,
            current_opp: 0,
            freq_khz: 0,
            util: [0; MAX_CPUS],
            opp_count: 0,
            opp_freqs: [0; MAX_OPPS],
        }
    }
}

fn governor_name(id: u8) -> &'static [u8] {
    match id {
        GOV_PERFORMANCE => b"performance",
        GOV_BALANCED => b"balanced",
        GOV_POWERSAVE => b"powersave",
        _ => b"unknown",
    }
}

fn parse_governor(value: &[u8]) -> Option<u8> {
    match value {
        b"performance" => Some(GOV_PERFORMANCE),
        b"balanced" => Some(GOV_BALANCED),
        b"powersave" => Some(GOV_POWERSAVE),
        _ => None,
    }
}

fn copy_to(buf: &mut [u8], pos: usize, src: &[u8]) -> usize {
    let len = src.len().min(buf.len().saturating_sub(pos));
    buf[pos..pos + len].copy_from_slice(&src[..len]);
    pos + len
}

fn fmt_u8(buf: &mut [u8], pos: usize, val: u8) -> usize {
    let mut tmp = [0u8; 4];
    let len = format_u32(&mut tmp, val as u32);
    copy_to(buf, pos, &tmp[..len])
}

fn fmt_u32(buf: &mut [u8], pos: usize, val: u32) -> usize {
    let mut tmp = [0u8; 12];
    let len = format_u32(&mut tmp, val);
    copy_to(buf, pos, &tmp[..len])
}

fn format_u32(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut n = val;
    let mut len = 0;
    while n > 0 { len += 1; n /= 10; }
    n = val;
    let w = len.min(buf.len());
    for i in (0..w).rev() {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    w
}

/// Parse a decimal u32 from a byte slice.
fn parse_u32(s: &[u8]) -> Option<u32> {
    if s.is_empty() { return None; }
    let mut val: u32 = 0;
    for &b in s {
        if b < b'0' || b > b'9' { return None; }
        val = val.checked_mul(10)?.checked_add((b - b'0') as u32)?;
    }
    Some(val)
}

const CPU_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_write(b"governor"),
    ConfigKey::read_write(b"freq"),
    ConfigKey::read_only(b"freqs"),
    ConfigKey::read_only(b"cpus"),
    ConfigKey::read_only(b"util"),
];

impl Driver for CpuPowerDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        let (bus_id, info) = ctx.claim_kernel_bus(b"/cpu:0")?;
        self.bus_id = Some(bus_id);
        self.num_cpus = info.device_count;
        self.active_cpus = info.device_count;
        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Forward
    }

    fn bus_event(&mut self, _bus_id: KernelBusId, msg_type: u8, data: &[u8], _ctx: &mut dyn BusCtx) {
        match msg_type {
            MSG_CPU_STATE_UPDATE if data.len() >= 11 => {
                // [governor(1), num_cpus(1), active(1), current_opp(1), freq_khz_le32(4), util0..3(4)]
                self.governor = data[0];
                self.num_cpus = data[1];
                self.active_cpus = data[2];
                self.current_opp = data[3];
                if data.len() >= 8 {
                    self.freq_khz = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
                }
                let util_start = 8;
                let n = (data.len() - util_start).min(MAX_CPUS);
                self.util[..n].copy_from_slice(&data[util_start..util_start + n]);
            }
            MSG_OPP_TABLE if data.len() >= 1 => {
                // [count(1), (freq_khz_le32(4))*count]
                let count = (data[0] as usize).min(MAX_OPPS);
                self.opp_count = count as u8;
                for i in 0..count {
                    let off = 1 + i * 4;
                    if off + 4 <= data.len() {
                        self.opp_freqs[i] = u32::from_le_bytes([
                            data[off], data[off + 1], data[off + 2], data[off + 3],
                        ]);
                    }
                }
            }
            _ => {}
        }
    }

    fn config_keys(&self) -> &[ConfigKey] {
        CPU_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"governor" => {
                copy_to(buf, 0, governor_name(self.governor))
            }
            b"freq" => {
                // "1800MHz"
                let mhz = self.freq_khz / 1000;
                let mut p = fmt_u32(buf, 0, mhz);
                p = copy_to(buf, p, b"MHz");
                p
            }
            b"freqs" => {
                // "800 1100 1500 1800" — space-separated MHz values
                let mut p = 0;
                for i in 0..self.opp_count as usize {
                    if i > 0 { p = copy_to(buf, p, b" "); }
                    p = fmt_u32(buf, p, self.opp_freqs[i] / 1000);
                }
                p
            }
            b"cpus" => {
                let mut p = 0;
                p = fmt_u8(buf, p, self.num_cpus);
                p = copy_to(buf, p, b" active=");
                p = fmt_u8(buf, p, self.active_cpus);
                p
            }
            b"util" => {
                let mut p = 0;
                for i in 0..self.num_cpus as usize {
                    if i < MAX_CPUS {
                        if i > 0 { p = copy_to(buf, p, b" "); }
                        p = fmt_u8(buf, p, self.util[i]);
                    }
                }
                p
            }
            _ => 0,
        }
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], ctx: &mut dyn BusCtx) -> usize {
        match key {
            b"governor" => {
                if let Some(gov) = parse_governor(value) {
                    if let Some(bus_id) = self.bus_id {
                        let msg = [MSG_SET_GOVERNOR, gov];
                        if ctx.bus_send(bus_id, &msg).is_err() {
                            return copy_to(buf, 0, b"ERR send failed\n");
                        }
                    }
                    self.governor = gov;
                    copy_to(buf, 0, governor_name(gov))
                } else {
                    copy_to(buf, 0, b"ERR unknown governor\n")
                }
            }
            b"freq" => {
                // Accept frequency in MHz, find matching OPP index
                if let Some(mhz) = parse_u32(value) {
                    let target_khz = mhz * 1000;
                    // Find OPP with matching frequency
                    let mut found = None;
                    for i in 0..self.opp_count as usize {
                        if self.opp_freqs[i] == target_khz {
                            found = Some(i as u8);
                            break;
                        }
                    }
                    if let Some(opp_idx) = found {
                        if let Some(bus_id) = self.bus_id {
                            let msg = [MSG_SET_OPP, opp_idx];
                            if ctx.bus_send(bus_id, &msg).is_err() {
                                return copy_to(buf, 0, b"ERR send failed\n");
                            }
                        }
                        let mut p = fmt_u32(buf, 0, mhz);
                        p = copy_to(buf, p, b"MHz");
                        p
                    } else {
                        copy_to(buf, 0, b"ERR unknown frequency\n")
                    }
                } else {
                    copy_to(buf, 0, b"ERR invalid number\n")
                }
            }
            _ => 0,
        }
    }
}

#[unsafe(no_mangle)]
fn main() {
    driver_main(b"cpud", CpuPowerDriver::new());
}
