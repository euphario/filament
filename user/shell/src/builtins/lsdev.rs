//! Device Query Commands
//!
//! Commands for querying devices registered with devd.
//!
//! Usage:
//!   lsdev [class]            - List all devices (optionally filtered by class)
//!   devinfo <id>             - Get detailed info about a device
//!   devquery <id> <type>     - Send pass-through query to driver

use crate::println;
use userlib::ipc::{Channel, Timer, Mux, MuxFilter};
use userlib::syscall;
use userlib::query::{
    QueryHeader, ListDevices, GetDeviceInfo, DriverQuery,
    DeviceListResponse, DeviceInfoResponse, DeviceEntry, ErrorResponse,
    msg, class, state, error, msc, BlockInfo,
};
use crate::output::CommandResult;

// =============================================================================
// lsdev - List Devices
// =============================================================================

/// List all registered devices
pub fn cmd_lsdev(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Parse optional class filter
    let class_filter = if args.is_empty() {
        None
    } else {
        match core::str::from_utf8(args) {
            Ok("msc") | Ok("storage") => Some(class::MASS_STORAGE),
            Ok("hub") => Some(class::HUB),
            Ok("hid") => Some(class::HID),
            Ok("net") | Ok("network") => Some(class::NETWORK),
            Ok("nvme") => Some(class::NVME),
            Ok(s) => {
                // Try parsing as number
                if let Ok(n) = s.parse::<u16>() {
                    Some(n)
                } else {
                    println!("Unknown class: {}", s);
                    println!("Known classes: msc, hub, hid, net, nvme");
                    return CommandResult::None;
                }
            }
            Err(_) => {
                println!("Invalid class argument");
                return CommandResult::None;
            }
        }
    };

    // Connect to devd-query
    let mut channel = match Channel::connect(b"devd-query:") {
        Ok(ch) => ch,
        Err(_) => {
            println!("Failed to connect to devd-query");
            return CommandResult::None;
        }
    };

    // Build LIST_DEVICES request
    let req = ListDevices::new(1, class_filter);
    let msg = req.to_bytes();

    if channel.send(&msg).is_err() {
        println!("Failed to send request");
        return CommandResult::None;
    }

    // Wait for response with timeout
    let response = match recv_with_timeout(&mut channel, 2000) {
        Some(r) => r,
        None => {
            println!("No response from devd (timeout)");
            return CommandResult::None;
        }
    };

    // Parse response
    let header = match QueryHeader::from_bytes(&response) {
        Some(h) => h,
        None => {
            println!("Invalid response");
            return CommandResult::None;
        }
    };

    if header.msg_type == msg::ERROR {
        if let Some(err) = ErrorResponse::from_bytes(&response) {
            println!("Error: {}", error_str(err.error_code));
        }
        return CommandResult::None;
    }

    if header.msg_type != msg::DEVICE_LIST {
        println!("Unexpected response type");
        return CommandResult::None;
    }

    let list_resp = match DeviceListResponse::from_bytes(&response) {
        Some(r) => r,
        None => {
            println!("Invalid device list response");
            return CommandResult::None;
        }
    };

    if list_resp.count == 0 {
        println!("No devices registered");
        return CommandResult::None;
    }

    // Print header
    println!("ID   CLASS       VENDOR:PROD  STATE");
    println!("---  ----------  -----------  -----------");

    // Parse and print device entries
    let mut offset = DeviceListResponse::HEADER_SIZE;
    for _ in 0..list_resp.count {
        if offset + DeviceEntry::SIZE > response.len() {
            break;
        }

        if let Some(entry) = DeviceEntry::from_bytes(&response[offset..]) {
            print_device_entry(&entry);
            offset += DeviceEntry::SIZE;
        }
    }

    CommandResult::None
}

fn print_device_entry(entry: &DeviceEntry) {
    let class_str = class_name(entry.device_class);
    let state_str = state_name(entry.state);

    println!(
        "{:<3}  {:<10}  {:04x}:{:04x}    {}",
        entry.device_id,
        class_str,
        entry.vendor_id,
        entry.product_id,
        state_str
    );
}

// =============================================================================
// devinfo - Device Info
// =============================================================================

/// Get detailed info about a specific device
pub fn cmd_devinfo(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Parse device ID
    let device_id = match core::str::from_utf8(args) {
        Ok(s) => match s.parse::<u32>() {
            Ok(id) => id,
            Err(_) => {
                println!("Usage: devinfo <device_id>");
                return CommandResult::None;
            }
        },
        Err(_) => {
            println!("Invalid device ID");
            return CommandResult::None;
        }
    };

    // Connect to devd-query
    let mut channel = match Channel::connect(b"devd-query:") {
        Ok(ch) => ch,
        Err(_) => {
            println!("Failed to connect to devd-query");
            return CommandResult::None;
        }
    };

    // Build GET_DEVICE_INFO request
    let req = GetDeviceInfo::new(1, device_id);
    let msg = req.to_bytes();

    if channel.send(&msg).is_err() {
        println!("Failed to send request");
        return CommandResult::None;
    }

    // Wait for response
    let response = match recv_with_timeout(&mut channel, 2000) {
        Some(r) => r,
        None => {
            println!("No response from devd (timeout)");
            return CommandResult::None;
        }
    };

    // Parse response
    let header = match QueryHeader::from_bytes(&response) {
        Some(h) => h,
        None => {
            println!("Invalid response");
            return CommandResult::None;
        }
    };

    if header.msg_type == msg::ERROR {
        if let Some(err) = ErrorResponse::from_bytes(&response) {
            println!("Error: {}", error_str(err.error_code));
        }
        return CommandResult::None;
    }

    if header.msg_type != msg::DEVICE_INFO {
        println!("Unexpected response type");
        return CommandResult::None;
    }

    // Parse device info (entry is at offset 8)
    if response.len() < DeviceInfoResponse::FIXED_SIZE {
        println!("Response too short");
        return CommandResult::None;
    }

    if let Some(entry) = DeviceEntry::from_bytes(&response[8..]) {
        println!("Device ID:    {}", entry.device_id);
        println!("Class:        {} (0x{:04x})", class_name(entry.device_class), entry.device_class);
        println!("Subclass:     0x{:04x}", entry.device_subclass);
        println!("Vendor:       0x{:04x}", entry.vendor_id);
        println!("Product:      0x{:04x}", entry.product_id);
        println!("State:        {}", state_name(entry.state));

        // Driver port name follows the fixed part
        let driver_port_len = response[24] as usize;
        if driver_port_len > 0 && response.len() >= DeviceInfoResponse::FIXED_SIZE + driver_port_len {
            let port_start = DeviceInfoResponse::FIXED_SIZE;
            if let Ok(port) = core::str::from_utf8(&response[port_start..port_start + driver_port_len]) {
                println!("Driver port:  {}", port);
            }
        }
    }

    CommandResult::None
}

// =============================================================================
// devquery - Driver Query (pass-through)
// =============================================================================

/// Send pass-through query to driver
pub fn cmd_devquery(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Parse: <device_id> <query_type>
    let (device_id, query_type) = match parse_devquery_args(args) {
        Some((id, qt)) => (id, qt),
        None => {
            println!("Usage: devquery <device_id> <query_type>");
            println!("Query types for MSC devices:");
            println!("  blockinfo   - Get block size and count");
            println!("  partition   - Get partition table");
            return CommandResult::None;
        }
    };

    // Connect to devd-query
    let mut channel = match Channel::connect(b"devd-query:") {
        Ok(ch) => ch,
        Err(_) => {
            println!("Failed to connect to devd-query");
            return CommandResult::None;
        }
    };

    // Build QUERY_DRIVER request
    let req = DriverQuery::new(1, device_id, query_type);
    let mut msg_buf = [0u8; 64];
    let msg_len = match req.write_to(&mut msg_buf, &[]) {
        Some(len) => len,
        None => {
            println!("Failed to build query");
            return CommandResult::None;
        }
    };

    if channel.send(&msg_buf[..msg_len]).is_err() {
        println!("Failed to send query");
        return CommandResult::None;
    }

    // Wait for response
    let response = match recv_with_timeout(&mut channel, 5000) {
        Some(r) => r,
        None => {
            println!("No response from driver (timeout)");
            return CommandResult::None;
        }
    };

    // Parse response
    let header = match QueryHeader::from_bytes(&response) {
        Some(h) => h,
        None => {
            println!("Invalid response");
            return CommandResult::None;
        }
    };

    if header.msg_type == msg::ERROR {
        if let Some(err) = ErrorResponse::from_bytes(&response) {
            println!("Error: {}", error_str(err.error_code));
        }
        return CommandResult::None;
    }

    if header.msg_type == msg::QUERY_RESULT {
        // Parse based on query type
        match query_type {
            msc::GET_BLOCK_INFO => {
                // Response payload starts after header (8 bytes)
                if let Some(info) = BlockInfo::from_bytes(&response[8..]) {
                    println!("Block size:  {} bytes", info.block_size);
                    println!("Block count: {}", info.block_count);
                    let size_mb = (info.block_count * info.block_size as u64) / (1024 * 1024);
                    println!("Capacity:    {} MB", size_mb);
                }
            }
            msc::GET_PARTITION_TABLE => {
                println!("Partition table response received");
                // TODO: Parse partition entries
            }
            _ => {
                println!("Query response received ({} bytes)", response.len());
            }
        }
    } else {
        println!("Unexpected response type: 0x{:04x}", header.msg_type);
    }

    CommandResult::None
}

fn parse_devquery_args(args: &[u8]) -> Option<(u32, u32)> {
    let args_str = core::str::from_utf8(args).ok()?;
    let mut parts = args_str.split_whitespace();

    let device_id: u32 = parts.next()?.parse().ok()?;

    let query_str = parts.next()?;
    let query_type = match query_str {
        "blockinfo" | "block" => msc::GET_BLOCK_INFO,
        "partition" | "part" => msc::GET_PARTITION_TABLE,
        "capacity" | "cap" => msc::GET_CAPACITY,
        _ => {
            // Try parsing as number
            query_str.parse().ok()?
        }
    };

    Some((device_id, query_type))
}

// =============================================================================
// Helpers
// =============================================================================

/// Receive with timeout (in ms)
fn recv_with_timeout(channel: &mut Channel, timeout_ms: u64) -> Option<[u8; 512]> {
    let mux = Mux::new().ok()?;
    let mut timer = Timer::new().ok()?;

    let now = syscall::gettime();
    let deadline = now + timeout_ms * 1_000_000;
    timer.set(deadline).ok()?;

    mux.add(channel.handle(), MuxFilter::Readable).ok()?;
    mux.add(timer.handle(), MuxFilter::Readable).ok()?;

    let event = mux.wait().ok()?;
    if event.handle == timer.handle() {
        return None; // Timeout
    }

    let mut response = [0u8; 512];
    match channel.recv(&mut response) {
        Ok(n) if n > 0 => Some(response),
        _ => None,
    }
}

fn class_name(class: u16) -> &'static str {
    match class {
        class::HID => "HID",
        class::MASS_STORAGE => "MSC",
        class::HUB => "HUB",
        class::NETWORK => "NET",
        class::NVME => "NVMe",
        _ => "Unknown",
    }
}

fn state_name(state: u8) -> &'static str {
    match state {
        state::DISCOVERED => "discovered",
        state::BOUND => "bound",
        state::OPERATIONAL => "operational",
        state::ERROR => "error",
        state::REMOVED => "removed",
        _ => "unknown",
    }
}

fn error_str(code: i32) -> &'static str {
    match code {
        error::OK => "OK",
        error::NOT_FOUND => "device not found",
        error::INVALID_REQUEST => "invalid request",
        error::NO_DRIVER => "no driver available",
        error::DEVICE_ERROR => "device error",
        error::PERMISSION_DENIED => "permission denied",
        error::NOT_SUPPORTED => "not supported",
        error::TIMEOUT => "timeout",
        _ => "unknown error",
    }
}
