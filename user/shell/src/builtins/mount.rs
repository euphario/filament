//! Mount Builtin - Display filesystem mount table
//!
//! Usage:
//!   mount              - List all registered mounts

use crate::println;
use libf::time::Duration;
use libsys::ipc::{Channel, Timer, Mux, MuxFilter};
use libsys::query::{QueryHeader, MountsListResponse, MountListEntry, mount_transport, msg};
use crate::output::CommandResult;

pub fn run(_args: &[u8]) -> CommandResult {
    let mut channel = match Channel::connect(b"devd-query:") {
        Ok(ch) => ch,
        Err(_) => {
            return CommandResult::Error("failed to connect to devd");
        }
    };

    // Send LIST_MOUNTS request
    let header = QueryHeader::new(msg::LIST_MOUNTS, 1);
    if channel.send(&header.to_bytes()).is_err() {
        return CommandResult::Error("send failed");
    }

    // Wait for response
    let response = match recv_with_timeout(&mut channel, 2000) {
        Some(r) => r,
        None => return CommandResult::Error("timeout"),
    };

    let resp_header = match QueryHeader::from_bytes(&response) {
        Some(h) => h,
        None => return CommandResult::Error("invalid response"),
    };

    if resp_header.msg_type != msg::MOUNTS_LIST {
        return CommandResult::Error("unexpected response type");
    }

    let (entry_count, total) = match MountsListResponse::parse_header(&response) {
        Some(ct) => ct,
        None => return CommandResult::Error("invalid mount list"),
    };

    if total == 0 {
        println!("No mounts registered.");
        return CommandResult::None;
    }

    println!("MOUNT TABLE ({} entries)", total);
    println!("{:<24} {:<12} {}", "PREFIX", "TYPE", "TARGET");
    println!("────────────────────────────────────────────────────────────");

    let data = &response[MountsListResponse::HEADER_SIZE..];
    for i in 0..entry_count as usize {
        let offset = i * MountListEntry::SIZE;
        if offset + MountListEntry::SIZE > data.len() {
            break;
        }
        if let Some((transport, prefix, port_name, shmem_id)) =
            MountListEntry::from_bytes(&data[offset..])
        {
            let prefix_str = core::str::from_utf8(prefix).unwrap_or("?");
            match transport {
                mount_transport::PORT => {
                    let pn = core::str::from_utf8(port_name).unwrap_or("?");
                    println!("{:<24} {:<12} {}", prefix_str, "Port", pn);
                }
                mount_transport::DATAPORT => {
                    println!("{:<24} {:<12} shmem_id={}", prefix_str, "DataPort", shmem_id);
                }
                _ => {
                    println!("{:<24} {:<12} ?", prefix_str, "Unknown");
                }
            }
        }
    }

    CommandResult::None
}

fn recv_with_timeout(channel: &mut Channel, timeout_ms: u64) -> Option<[u8; 576]> {
    let mux = Mux::new().ok()?;
    let mut timer = Timer::new().ok()?;
    timer.set(Duration::from_millis(timeout_ms).as_nanos_saturating()).ok()?;
    mux.add(channel.handle(), MuxFilter::Readable).ok()?;
    mux.add(timer.handle(), MuxFilter::Readable).ok()?;

    let event = mux.wait().ok()?;
    if event.handle == timer.handle() {
        return None;
    }

    let mut response = [0u8; 576];
    match channel.recv(&mut response) {
        Ok(n) if n > 0 => Some(response),
        _ => None,
    }
}
