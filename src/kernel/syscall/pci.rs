//! PCI and Bus Syscall Handlers
//!
//! This module contains syscall handlers for:
//! - PCI configuration space read/write
//! - MSI allocation for PCI devices
//! - Bus and device enumeration
//!
//! All functions are pub(super) so they can be called from the main syscall handler.

use super::super::bus::{BusInfo, DeviceInfo, get_bus_list, get_bus_count, get_device_list, get_device_count};
use super::super::caps::Capabilities;
use super::super::pci::{self, PciBdf};
use super::super::uaccess;
use super::{current_pid, require_capability, SyscallError};

/// Read PCI config space
/// bdf: device address (port:8 | bus:8 | device:5 | function:3)
/// offset: register offset (must be aligned)
/// size: 1, 2, or 4 bytes
/// Returns: value or -errno
pub(super) fn sys_pci_config_read(bdf: u32, offset: u16, size: u8) -> i64 {
    // Require RAW_DEVICE capability for PCI config access
    if let Err(e) = require_capability(Capabilities::RAW_DEVICE) {
        return e;
    }

    // Validate offset and size
    // PCIe extended config space is 4096 bytes (0x000-0xFFF)
    // Standard PCI config is 256 bytes (0x00-0xFF)
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return SyscallError::InvalidArgument as i64;
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return SyscallError::InvalidArgument as i64;
    }
    // Check alignment (2-byte access must be 2-aligned, 4-byte must be 4-aligned)
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return SyscallError::InvalidArgument as i64;
    }

    let bdf = PciBdf::from_u32(bdf);

    // Check device exists and caller has access
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return SyscallError::NotFound as i64,
    };

    // Check ownership (0 = unclaimed, anyone can read)
    let pid = current_pid();
    if dev.owner_pid != 0 && dev.owner_pid != pid {
        return SyscallError::PermissionDenied as i64;
    }

    match pci::config_read32(bdf, offset & !0x3) {
        Ok(val32) => {
            // Extract the requested portion
            match size {
                1 => {
                    let shift = (offset & 3) * 8;
                    ((val32 >> shift) & 0xFF) as i64
                }
                2 => {
                    let shift = (offset & 2) * 8;
                    ((val32 >> shift) & 0xFFFF) as i64
                }
                4 => val32 as i64,
                _ => SyscallError::InvalidArgument as i64,
            }
        }
        Err(_) => SyscallError::IoError as i64,
    }
}

/// Write PCI config space
/// bdf: device address
/// offset: register offset
/// size: 1, 2, or 4 bytes
/// value: value to write
/// Returns: 0 or -errno
pub(super) fn sys_pci_config_write(bdf: u32, offset: u16, size: u8, value: u32) -> i64 {
    // Require RAW_DEVICE capability for PCI config access
    if let Err(e) = require_capability(Capabilities::RAW_DEVICE) {
        return e;
    }

    // Validate offset and size
    // PCIe extended config space is 4096 bytes (0x000-0xFFF)
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return SyscallError::InvalidArgument as i64;
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return SyscallError::InvalidArgument as i64;
    }
    // Check alignment (2-byte access must be 2-aligned, 4-byte must be 4-aligned)
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return SyscallError::InvalidArgument as i64;
    }

    let bdf = PciBdf::from_u32(bdf);

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return SyscallError::NotFound as i64,
    };

    let pid = current_pid();
    if dev.owner_pid != pid {
        return SyscallError::PermissionDenied as i64;
    }

    // For partial writes, do read-modify-write
    let aligned_offset = offset & !0x3;

    let result = if size == 4 {
        pci::config_write32(bdf, aligned_offset, value)
    } else {
        // Read current value
        let current = match pci::config_read32(bdf, aligned_offset) {
            Ok(v) => v,
            Err(_) => return SyscallError::IoError as i64,
        };

        let (mask, shift) = match size {
            1 => (0xFFu32, ((offset & 3) * 8) as u32),
            2 => (0xFFFFu32, ((offset & 2) * 8) as u32),
            _ => return SyscallError::InvalidArgument as i64,
        };

        let new_val = (current & !(mask << shift)) | ((value & mask) << shift);
        pci::config_write32(bdf, aligned_offset, new_val)
    };

    match result {
        Ok(()) => 0,
        Err(_) => SyscallError::IoError as i64,
    }
}

/// Allocate MSI vector(s) for a device
/// bdf: device address
/// count: number of vectors requested (power of 2)
/// Returns: first IRQ number or -errno
pub(super) fn sys_pci_msi_alloc(bdf: u32, count: u8) -> i64 {
    // Require IRQ_CLAIM capability for MSI allocation
    if let Err(e) = require_capability(Capabilities::IRQ_CLAIM) {
        return e;
    }

    let bdf = PciBdf::from_u32(bdf);
    let pid = current_pid();

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return SyscallError::NotFound as i64,
    };

    if dev.owner_pid != pid {
        return SyscallError::PermissionDenied as i64;
    }

    // Check device supports MSI
    if !dev.has_msi() && !dev.has_msix() {
        return SyscallError::NotImplemented as i64;
    }

    // Allocate vectors
    match pci::msi_alloc(bdf, count) {
        Ok(irq) => irq as i64,
        Err(pci::PciError::NoMsiVectors) => SyscallError::OutOfMemory as i64,
        Err(_) => SyscallError::IoError as i64,
    }
}

/// List available buses
/// Args: buf_ptr (pointer to BusInfo array), max_count
/// Returns: number of buses written, or negative error
pub(super) fn sys_bus_list(buf_ptr: u64, max_count: u64) -> i64 {
    if buf_ptr == 0 {
        // Just return count
        return get_bus_count() as i64;
    }

    let max = max_count as usize;
    if max == 0 {
        return 0;
    }

    // Create temporary buffer on stack (max 16 buses)
    let mut temp = [BusInfo::empty(); 16];
    let count = get_bus_list(&mut temp[..max.min(16)]);

    // Copy to userspace using safe VA→PA translation
    let copy_size = count * core::mem::size_of::<BusInfo>();
    let src_bytes = unsafe {
        core::slice::from_raw_parts(temp.as_ptr() as *const u8, copy_size)
    };
    if uaccess::copy_to_user(buf_ptr, src_bytes).is_err() {
        return SyscallError::BadAddress as i64;
    }

    count as i64
}

/// List available devices (unified view of platform + bus controllers)
/// Args: buf_ptr (pointer to DeviceInfo array), max_count
/// Returns: number of devices written, or negative error
pub(super) fn sys_device_list(buf_ptr: u64, max_count: u64) -> i64 {
    if buf_ptr == 0 {
        // Just return count
        return get_device_count() as i64;
    }

    let max = max_count as usize;
    if max == 0 {
        return 0;
    }

    // Create temporary buffer on stack (max 16 devices)
    let mut temp = [DeviceInfo::empty(); 16];
    let count = get_device_list(&mut temp[..max.min(16)]);

    // Copy to userspace using safe VA→PA translation
    let copy_size = count * core::mem::size_of::<DeviceInfo>();
    let src_bytes = unsafe {
        core::slice::from_raw_parts(temp.as_ptr() as *const u8, copy_size)
    };
    if uaccess::copy_to_user(buf_ptr, src_bytes).is_err() {
        return SyscallError::BadAddress as i64;
    }

    count as i64
}
