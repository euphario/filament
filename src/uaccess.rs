//! User Space Memory Access Validation
//!
//! Provides safe functions for accessing user space memory from syscalls.
//! This is critical for kernel security - prevents userspace from tricking
//! the kernel into reading/writing arbitrary memory.
//!
//! Memory layout (48-bit VA):
//! - User space:   0x0000_0000_0000_0000 - 0x0000_FFFF_FFFF_FFFF (TTBR0)
//! - Kernel space: 0xFFFF_0000_0000_0000 - 0xFFFF_FFFF_FFFF_FFFF (TTBR1)
//!
//! IMPORTANT: During syscall handling, TTBR0 is switched to the kernel's identity
//! mapping. To access user memory, we must:
//! 1. Translate user VA to PA using the user's page tables
//! 2. Access the physical memory via TTBR1 (kernel virtual address)

use crate::mmu::{flags, PAGE_SIZE};
use crate::task;

/// Kernel virtual address base (TTBR1)
const KERNEL_VIRT_BASE: u64 = 0xFFFF_0000_0000_0000;

/// Maximum valid user space address (48-bit, upper half is kernel)
pub const USER_SPACE_END: u64 = 0x0000_FFFF_FFFF_FFFF;

/// User space start (after null page protection)
pub const USER_SPACE_START: u64 = 0x0000_0000_0000_1000; // Skip first page (null ptr protection)

/// Error types for user memory access
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UAccessError {
    /// Pointer is null
    NullPointer,
    /// Address is in kernel space
    KernelAddress,
    /// Address range overflows
    Overflow,
    /// Address is not properly aligned
    Unaligned,
    /// Memory region is not mapped
    NotMapped,
    /// Memory region is not readable
    NotReadable,
    /// Memory region is not writable
    NotWritable,
    /// Length is invalid (zero or too large)
    InvalidLength,
}

impl UAccessError {
    /// Convert to errno-style error code
    pub fn to_errno(self) -> i64 {
        match self {
            UAccessError::NullPointer => -14,      // EFAULT
            UAccessError::KernelAddress => -14,    // EFAULT
            UAccessError::Overflow => -14,         // EFAULT
            UAccessError::Unaligned => -14,        // EFAULT
            UAccessError::NotMapped => -14,        // EFAULT
            UAccessError::NotReadable => -14,      // EFAULT
            UAccessError::NotWritable => -14,      // EFAULT
            UAccessError::InvalidLength => -22,    // EINVAL
        }
    }
}

/// Check if an address is in user space
#[inline]
pub fn is_user_address(addr: u64) -> bool {
    // User addresses have upper 16 bits = 0
    // Kernel addresses have upper 16 bits = 0xFFFF
    (addr >> 48) == 0 && addr >= USER_SPACE_START
}

/// Check if an address range is entirely in user space
#[inline]
pub fn is_user_range(addr: u64, len: usize) -> bool {
    if len == 0 {
        return true; // Empty range is valid
    }

    // Check for overflow
    let end = match addr.checked_add(len as u64) {
        Some(e) => e,
        None => return false,
    };

    // Check start is in user space and end doesn't overflow into kernel
    is_user_address(addr) && end <= USER_SPACE_END + 1
}

/// Validate a user pointer for reading
///
/// Checks:
/// 1. Pointer is not null
/// 2. Pointer is in user address space
/// 3. Range doesn't overflow
/// 4. Memory is actually mapped (walks page tables)
pub fn validate_user_read(ptr: u64, len: usize) -> Result<(), UAccessError> {
    // Null check
    if ptr == 0 {
        return Err(UAccessError::NullPointer);
    }

    // Zero length is always valid
    if len == 0 {
        return Ok(());
    }

    // Check address is in user space
    if !is_user_address(ptr) {
        return Err(UAccessError::KernelAddress);
    }

    // Check for overflow
    let end = ptr.checked_add(len as u64)
        .ok_or(UAccessError::Overflow)?;

    if end > USER_SPACE_END + 1 {
        return Err(UAccessError::KernelAddress);
    }

    // Walk page tables to verify mapping
    verify_mapping(ptr, len, false)?;

    Ok(())
}

/// Validate a user pointer for writing
///
/// Same checks as validate_user_read, plus verifies write permission
pub fn validate_user_write(ptr: u64, len: usize) -> Result<(), UAccessError> {
    // Null check
    if ptr == 0 {
        return Err(UAccessError::NullPointer);
    }

    // Zero length is always valid
    if len == 0 {
        return Ok(());
    }

    // Check address is in user space
    if !is_user_address(ptr) {
        return Err(UAccessError::KernelAddress);
    }

    // Check for overflow
    let end = ptr.checked_add(len as u64)
        .ok_or(UAccessError::Overflow)?;

    if end > USER_SPACE_END + 1 {
        return Err(UAccessError::KernelAddress);
    }

    // Walk page tables to verify mapping with write permission
    verify_mapping(ptr, len, true)?;

    Ok(())
}

/// Validate a user string (null-terminated)
/// Returns the length if valid (not including null terminator)
pub fn validate_user_string(ptr: u64, max_len: usize) -> Result<usize, UAccessError> {
    if ptr == 0 {
        return Err(UAccessError::NullPointer);
    }

    if !is_user_address(ptr) {
        return Err(UAccessError::KernelAddress);
    }

    // Scan for null terminator, checking each page as we go
    let mut len = 0;
    let mut current_page = ptr & !(PAGE_SIZE as u64 - 1);
    let mut checked_page = false;

    while len < max_len {
        let addr = ptr + len as u64;
        let page = addr & !(PAGE_SIZE as u64 - 1);

        // Check new page when we cross a boundary
        if page != current_page || !checked_page {
            verify_mapping(addr, 1, false)?;
            current_page = page;
            checked_page = true;
        }

        // Read the byte
        let byte = unsafe { core::ptr::read_volatile(addr as *const u8) };
        if byte == 0 {
            return Ok(len);
        }
        len += 1;
    }

    // No null terminator found within max_len
    Err(UAccessError::InvalidLength)
}

/// Walk page tables to verify a memory range is mapped
///
/// This function walks the current process's page tables to check if the
/// specified memory range is properly mapped.
fn verify_mapping(addr: u64, len: usize, needs_write: bool) -> Result<(), UAccessError> {
    if len == 0 {
        return Ok(());
    }

    // Get current task's TTBR0 (page table root) through address_space
    let ttbr0 = unsafe {
        let sched = task::scheduler();
        match &sched.tasks[sched.current] {
            Some(task) => {
                match &task.address_space {
                    Some(addr_space) => addr_space.ttbr0,
                    None => return Err(UAccessError::NotMapped), // Kernel task, no user space
                }
            }
            None => return Err(UAccessError::NotMapped),
        }
    };

    // Check each page in the range
    let start_page = addr & !(PAGE_SIZE as u64 - 1);
    let end_addr = addr + len as u64;
    let end_page = (end_addr + PAGE_SIZE as u64 - 1) & !(PAGE_SIZE as u64 - 1);

    let mut page = start_page;
    while page < end_page {
        verify_page_mapped(ttbr0, page, needs_write)?;
        page += PAGE_SIZE as u64;
    }

    Ok(())
}

/// Check if a single page is mapped in the page tables
/// NOTE: Accesses page tables via TTBR1 kernel mapping since TTBR0 is switched during syscall
fn verify_page_mapped(ttbr0: u64, virt_addr: u64, needs_write: bool) -> Result<(), UAccessError> {
    // Extract table indices from virtual address
    let l0_index = ((virt_addr >> 39) & 0x1FF) as usize;
    let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
    let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
    let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

    unsafe {
        // Read L0 entry via TTBR1 mapping (page tables are at physical addresses)
        let l0_table = (KERNEL_VIRT_BASE | ttbr0) as *const u64;
        let l0_entry = core::ptr::read_volatile(l0_table.add(l0_index));

        if (l0_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // L0 must be a table entry (pointing to L1)
        if (l0_entry & flags::TABLE) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Read L1 entry via TTBR1 mapping
        let l1_table_phys = l0_entry & 0x0000_FFFF_FFFF_F000;
        let l1_table = (KERNEL_VIRT_BASE | l1_table_phys) as *const u64;
        let l1_entry = core::ptr::read_volatile(l1_table.add(l1_index));

        if (l1_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Check if L1 is a block entry (1GB block) or table entry
        if (l1_entry & flags::TABLE) == 0 {
            // 1GB block mapping - check permissions
            return check_permissions(l1_entry, needs_write);
        }

        // Read L2 entry via TTBR1 mapping
        let l2_table_phys = l1_entry & 0x0000_FFFF_FFFF_F000;
        let l2_table = (KERNEL_VIRT_BASE | l2_table_phys) as *const u64;
        let l2_entry = core::ptr::read_volatile(l2_table.add(l2_index));

        if (l2_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Check if L2 is a block entry (2MB block) or table entry
        if (l2_entry & flags::TABLE) == 0 {
            // 2MB block mapping - check permissions
            return check_permissions(l2_entry, needs_write);
        }

        // Read L3 entry via TTBR1 mapping
        let l3_table_phys = l2_entry & 0x0000_FFFF_FFFF_F000;
        let l3_table = (KERNEL_VIRT_BASE | l3_table_phys) as *const u64;
        let l3_entry = core::ptr::read_volatile(l3_table.add(l3_index));

        if (l3_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // L3 entries use PAGE bit, not TABLE
        if (l3_entry & flags::PAGE) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Check page permissions
        check_permissions(l3_entry, needs_write)
    }
}

/// Check if page table entry has required permissions
fn check_permissions(entry: u64, needs_write: bool) -> Result<(), UAccessError> {
    // AP[2:1] field is at bits 7:6
    let ap = (entry >> 6) & 0x3;

    // AP encoding for EL0 access:
    // 00: No EL0 access
    // 01: RW at all ELs
    // 10: No EL0 access
    // 11: RO at all ELs

    match ap {
        0b00 | 0b10 => {
            // No EL0 access
            Err(UAccessError::NotReadable)
        }
        0b01 => {
            // RW at all ELs - full access
            Ok(())
        }
        0b11 => {
            // RO at all ELs
            if needs_write {
                Err(UAccessError::NotWritable)
            } else {
                Ok(())
            }
        }
        _ => unreachable!(),
    }
}

/// Translate a user virtual address to physical address using the user's page tables
/// This is needed because during syscall, TTBR0 points to kernel's identity mapping,
/// not the user's address space. We must walk the user's page tables to find the PA.
fn user_virt_to_phys(ttbr0: u64, virt_addr: u64) -> Result<u64, UAccessError> {
    // Extract table indices from virtual address
    let l0_index = ((virt_addr >> 39) & 0x1FF) as usize;
    let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
    let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
    let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;
    let page_offset = virt_addr & 0xFFF;

    unsafe {
        // Read L0 entry - page tables are at physical addresses, access via TTBR1
        let l0_table_phys = ttbr0;
        let l0_table = (KERNEL_VIRT_BASE | l0_table_phys) as *const u64;
        let l0_entry = core::ptr::read_volatile(l0_table.add(l0_index));

        if (l0_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // L0 must be a table entry
        if (l0_entry & flags::TABLE) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Read L1 entry
        let l1_table_phys = l0_entry & 0x0000_FFFF_FFFF_F000;
        let l1_table = (KERNEL_VIRT_BASE | l1_table_phys) as *const u64;
        let l1_entry = core::ptr::read_volatile(l1_table.add(l1_index));

        if (l1_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Check if L1 is a 1GB block or table entry
        if (l1_entry & flags::TABLE) == 0 {
            // 1GB block mapping
            let block_base = l1_entry & 0x0000_FFFC_0000_0000; // Bits 47:30
            let offset = virt_addr & 0x3FFF_FFFF; // Lower 30 bits
            return Ok(block_base | offset);
        }

        // Read L2 entry
        let l2_table_phys = l1_entry & 0x0000_FFFF_FFFF_F000;
        let l2_table = (KERNEL_VIRT_BASE | l2_table_phys) as *const u64;
        let l2_entry = core::ptr::read_volatile(l2_table.add(l2_index));

        if (l2_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // Check if L2 is a 2MB block or table entry
        if (l2_entry & flags::TABLE) == 0 {
            // 2MB block mapping
            let block_base = l2_entry & 0x0000_FFFF_FFE0_0000; // Bits 47:21
            let offset = virt_addr & 0x001F_FFFF; // Lower 21 bits
            return Ok(block_base | offset);
        }

        // Read L3 entry
        let l3_table_phys = l2_entry & 0x0000_FFFF_FFFF_F000;
        let l3_table = (KERNEL_VIRT_BASE | l3_table_phys) as *const u64;
        let l3_entry = core::ptr::read_volatile(l3_table.add(l3_index));

        if (l3_entry & flags::VALID) == 0 {
            return Err(UAccessError::NotMapped);
        }

        // L3 entry - 4KB page
        if (l3_entry & flags::PAGE) == 0 {
            return Err(UAccessError::NotMapped);
        }

        let page_base = l3_entry & 0x0000_FFFF_FFFF_F000;
        Ok(page_base | page_offset)
    }
}

/// Get the current task's TTBR0 (user page table root)
fn get_current_ttbr0() -> Result<u64, UAccessError> {
    unsafe {
        let sched = task::scheduler();
        match &sched.tasks[sched.current] {
            Some(task) => {
                match &task.address_space {
                    Some(addr_space) => Ok(addr_space.ttbr0),
                    None => Err(UAccessError::NotMapped), // Kernel task, no user space
                }
            }
            None => Err(UAccessError::NotMapped),
        }
    }
}

/// Safely copy data from user space to kernel buffer
///
/// Returns the number of bytes copied, or an error.
///
/// IMPORTANT: This function translates user virtual addresses to physical addresses
/// using the user's page tables, then accesses the physical memory via TTBR1.
/// This is necessary because during syscall handling, TTBR0 points to the kernel's
/// identity mapping, not the user's address space.
pub fn copy_from_user(kernel_buf: &mut [u8], user_ptr: u64) -> Result<usize, UAccessError> {
    let len = kernel_buf.len();
    if len == 0 {
        return Ok(0);
    }

    validate_user_read(user_ptr, len)?;
    let ttbr0 = get_current_ttbr0()?;

    // Copy byte by byte, handling page boundaries
    // (A more efficient implementation would copy page-at-a-time)
    let mut copied = 0;
    while copied < len {
        let user_addr = user_ptr + copied as u64;
        let phys_addr = user_virt_to_phys(ttbr0, user_addr)?;

        // Access physical memory via TTBR1 kernel mapping
        let kernel_va = KERNEL_VIRT_BASE | phys_addr;
        unsafe {
            kernel_buf[copied] = core::ptr::read_volatile(kernel_va as *const u8);
        }
        copied += 1;
    }

    Ok(len)
}

/// Safely copy data from kernel buffer to user space
///
/// Returns the number of bytes copied, or an error.
///
/// IMPORTANT: This function translates user virtual addresses to physical addresses
/// using the user's page tables, then accesses the physical memory via TTBR1.
pub fn copy_to_user(user_ptr: u64, kernel_buf: &[u8]) -> Result<usize, UAccessError> {
    let len = kernel_buf.len();
    if len == 0 {
        return Ok(0);
    }

    validate_user_write(user_ptr, len)?;
    let ttbr0 = get_current_ttbr0()?;

    // Copy byte by byte, handling page boundaries
    let mut copied = 0;
    while copied < len {
        let user_addr = user_ptr + copied as u64;
        let phys_addr = user_virt_to_phys(ttbr0, user_addr)?;

        // Access physical memory via TTBR1 kernel mapping
        let kernel_va = KERNEL_VIRT_BASE | phys_addr;
        unsafe {
            core::ptr::write_volatile(kernel_va as *mut u8, kernel_buf[copied]);
        }
        copied += 1;
    }

    Ok(len)
}

/// Read a single value from user space
///
/// IMPORTANT: Translates user VA to PA and accesses via TTBR1.
pub fn get_user<T: Copy>(user_ptr: u64) -> Result<T, UAccessError> {
    let size = core::mem::size_of::<T>();

    // Check alignment
    if user_ptr % (core::mem::align_of::<T>() as u64) != 0 {
        return Err(UAccessError::Unaligned);
    }

    validate_user_read(user_ptr, size)?;
    let ttbr0 = get_current_ttbr0()?;
    let phys_addr = user_virt_to_phys(ttbr0, user_ptr)?;
    let kernel_va = KERNEL_VIRT_BASE | phys_addr;

    unsafe {
        Ok(core::ptr::read_volatile(kernel_va as *const T))
    }
}

/// Write a single value to user space
///
/// IMPORTANT: Translates user VA to PA and accesses via TTBR1.
pub fn put_user<T: Copy>(user_ptr: u64, value: T) -> Result<(), UAccessError> {
    let size = core::mem::size_of::<T>();

    // Check alignment
    if user_ptr % (core::mem::align_of::<T>() as u64) != 0 {
        return Err(UAccessError::Unaligned);
    }

    validate_user_write(user_ptr, size)?;
    let ttbr0 = get_current_ttbr0()?;
    let phys_addr = user_virt_to_phys(ttbr0, user_ptr)?;
    let kernel_va = KERNEL_VIRT_BASE | phys_addr;

    unsafe {
        core::ptr::write_volatile(kernel_va as *mut T, value);
    }

    Ok(())
}

/// DEPRECATED: This function does NOT work correctly!
///
/// During syscall handling, TTBR0 points to kernel's identity mapping, not user's.
/// The returned slice would point to wrong physical memory.
///
/// Use `copy_from_user()` instead to safely read user memory.
#[deprecated(note = "Use copy_from_user() - user slices don't work with TTBR0 switching")]
pub unsafe fn user_slice(_ptr: u64, _len: usize) -> Result<&'static [u8], UAccessError> {
    // This function cannot work correctly - user VA != correct PA during syscall
    Err(UAccessError::NotMapped)
}

/// DEPRECATED: This function does NOT work correctly!
///
/// During syscall handling, TTBR0 points to kernel's identity mapping, not user's.
/// The returned slice would point to wrong physical memory.
///
/// Use `copy_to_user()` instead to safely write user memory.
#[deprecated(note = "Use copy_to_user() - user slices don't work with TTBR0 switching")]
pub unsafe fn user_slice_mut(_ptr: u64, _len: usize) -> Result<&'static mut [u8], UAccessError> {
    // This function cannot work correctly - user VA != correct PA during syscall
    Err(UAccessError::NotMapped)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_user_address() {
        // Valid user addresses
        assert!(is_user_address(0x1000));
        assert!(is_user_address(0x4000_0000));
        assert!(is_user_address(0x0000_FFFF_FFFF_F000));

        // Invalid: null page
        assert!(!is_user_address(0));
        assert!(!is_user_address(0x100));

        // Invalid: kernel addresses
        assert!(!is_user_address(0xFFFF_0000_0000_0000));
        assert!(!is_user_address(0xFFFF_FFFF_FFFF_FFFF));
    }

    #[test]
    fn test_is_user_range() {
        // Valid ranges
        assert!(is_user_range(0x1000, 0x1000));
        assert!(is_user_range(0x4000_0000, 0x1000));

        // Invalid: overflow
        assert!(!is_user_range(0x0000_FFFF_FFFF_F000, 0x2000));

        // Invalid: starts in kernel space
        assert!(!is_user_range(0xFFFF_0000_0000_0000, 1));
    }
}
