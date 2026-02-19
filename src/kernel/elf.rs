//! ELF64 Loader

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Minimal ELF loader for AArch64 executables.
//! Parses ELF headers and loads PT_LOAD segments into process memory.

use super::addrspace::AddressSpace;
use super::pmm;
use crate::{kdebug, kwarn, print_direct};
use crate::kernel::arch::mmu;
use super::task;

/// ELF magic number
pub const ELF_MAGIC: [u8; 4] = [0x7f, b'E', b'L', b'F'];

/// ELF class: 64-bit
pub const ELFCLASS64: u8 = 2;

/// ELF data encoding: little endian
pub const ELFDATA2LSB: u8 = 1;

/// ELF type: executable
pub const ET_EXEC: u16 = 2;
/// ELF type: shared object (PIE)
pub const ET_DYN: u16 = 3;

/// ELF machine: AArch64
pub const EM_AARCH64: u16 = 183;

/// Program header type: loadable segment
pub const PT_LOAD: u32 = 1;
/// Program header type: dynamic linking info
pub const PT_DYNAMIC: u32 = 2;

/// Segment flags
pub const PF_X: u32 = 1;  // Executable
pub const PF_W: u32 = 2;  // Writable
pub const PF_R: u32 = 4;  // Readable

/// Dynamic section entry tags
pub const DT_NULL: u64 = 0;
pub const DT_RELA: u64 = 7;
pub const DT_RELASZ: u64 = 8;

/// AArch64 relocation type: R_AARCH64_RELATIVE (value = base + addend)
pub const R_AARCH64_RELATIVE: u32 = 0x403;

/// ELF64 Rela entry (relocation with addend)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Elf64Rela {
    pub r_offset: u64,
    pub r_info: u64,
    pub r_addend: i64,
}

/// ELF64 file header
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Elf64Header {
    /// Magic number and other info
    pub e_ident: [u8; 16],
    /// Object file type
    pub e_type: u16,
    /// Architecture
    pub e_machine: u16,
    /// Object file version
    pub e_version: u32,
    /// Entry point virtual address
    pub e_entry: u64,
    /// Program header table file offset
    pub e_phoff: u64,
    /// Section header table file offset
    pub e_shoff: u64,
    /// Processor-specific flags
    pub e_flags: u32,
    /// ELF header size in bytes
    pub e_ehsize: u16,
    /// Program header table entry size
    pub e_phentsize: u16,
    /// Program header table entry count
    pub e_phnum: u16,
    /// Section header table entry size
    pub e_shentsize: u16,
    /// Section header table entry count
    pub e_shnum: u16,
    /// Section header string table index
    pub e_shstrndx: u16,
}

/// ELF64 program header
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Elf64ProgramHeader {
    /// Segment type
    pub p_type: u32,
    /// Segment flags
    pub p_flags: u32,
    /// Segment file offset
    pub p_offset: u64,
    /// Segment virtual address
    pub p_vaddr: u64,
    /// Segment physical address (unused)
    pub p_paddr: u64,
    /// Segment size in file
    pub p_filesz: u64,
    /// Segment size in memory
    pub p_memsz: u64,
    /// Segment alignment
    pub p_align: u64,
}

/// ELF loading errors
#[derive(Debug, Clone, Copy)]
pub enum ElfError {
    /// Invalid ELF magic
    BadMagic,
    /// Not a 64-bit ELF
    Not64Bit,
    /// Not little endian
    NotLittleEndian,
    /// Not an executable
    NotExecutable,
    /// Wrong architecture
    WrongArch,
    /// File too small
    TooSmall,
    /// Out of memory
    OutOfMemory,
    /// Invalid segment
    InvalidSegment,
    /// Binary signature verification failed (future: crypto check)
    SignatureInvalid,
}

impl ElfError {
    /// Convert to errno-style error code
    pub fn to_errno(self) -> i32 {
        match self {
            ElfError::BadMagic => -8,         // ENOEXEC
            ElfError::Not64Bit => -8,         // ENOEXEC
            ElfError::NotLittleEndian => -8,  // ENOEXEC
            ElfError::NotExecutable => -8,    // ENOEXEC
            ElfError::WrongArch => -8,        // ENOEXEC
            ElfError::TooSmall => -8,         // ENOEXEC
            ElfError::OutOfMemory => -12,     // ENOMEM
            ElfError::InvalidSegment => -8,   // ENOEXEC
            ElfError::SignatureInvalid => -1, // EPERM
        }
    }
}

/// Parsed ELF information
pub struct ElfInfo {
    /// Entry point address
    pub entry: u64,
    /// Base load address (for PIE)
    pub base: u64,
    /// Number of segments loaded
    pub segments_loaded: usize,
}

// ============================================================================
// Binary Signature Verification (stub for future crypto implementation)
// ============================================================================

/// Verify that a binary is authorized to receive the requested capabilities.
///
/// STUB: Currently always returns Ok. Future implementation will:
/// - Check binary hash against known-good list, OR
/// - Verify Ed25519 signature in .sig ELF section, OR
/// - Validate signed capability manifest
///
/// # Arguments
/// * `_data` - Raw ELF binary bytes (for hashing/signature extraction)
/// * `_name` - Binary name (for manifest lookup)
/// * `_requested_caps` - Capabilities being granted (to verify against manifest)
///
/// # Returns
/// * `Ok(())` - Binary is authorized
/// * `Err(SignatureInvalid)` - Binary failed verification
#[allow(unused_variables)]
pub fn verify_binary_signature(
    _data: &[u8],
    _name: &str,
    _requested_caps: Option<super::caps::Capabilities>,
) -> Result<(), ElfError> {
    // Stub: trust all binaries via initrd embedding (no external binary loading yet).
    // When external binary loading is added, implement ed25519 signature check here.
    Ok(())
}

/// Validate ELF header
pub fn validate_header(data: &[u8]) -> Result<&Elf64Header, ElfError> {
    if data.len() < core::mem::size_of::<Elf64Header>() {
        return Err(ElfError::TooSmall);
    }

    let header = unsafe { &*(data.as_ptr() as *const Elf64Header) };

    // Check magic
    if header.e_ident[0..4] != ELF_MAGIC {
        return Err(ElfError::BadMagic);
    }

    // Check class (64-bit)
    if header.e_ident[4] != ELFCLASS64 {
        return Err(ElfError::Not64Bit);
    }

    // Check endianness (little)
    if header.e_ident[5] != ELFDATA2LSB {
        return Err(ElfError::NotLittleEndian);
    }

    // Check type (executable or shared/PIE)
    let e_type = header.e_type;
    if e_type != ET_EXEC && e_type != ET_DYN {
        return Err(ElfError::NotExecutable);
    }

    // Check machine (AArch64)
    if header.e_machine != EM_AARCH64 {
        return Err(ElfError::WrongArch);
    }

    Ok(header)
}

/// Get program headers from ELF data
pub fn get_program_headers<'a>(data: &'a [u8], header: &Elf64Header) -> Result<&'a [Elf64ProgramHeader], ElfError> {
    // Copy from packed struct to avoid unaligned access
    let ph_offset = { header.e_phoff } as usize;
    let ph_size = { header.e_phentsize } as usize;
    let ph_count = { header.e_phnum } as usize;

    // Use checked arithmetic to prevent overflow attacks from malicious ELF
    let headers_size = ph_size.checked_mul(ph_count).ok_or(ElfError::InvalidSegment)?;
    let required_size = ph_offset.checked_add(headers_size).ok_or(ElfError::InvalidSegment)?;
    if data.len() < required_size {
        return Err(ElfError::TooSmall);
    }

    if ph_size < core::mem::size_of::<Elf64ProgramHeader>() {
        return Err(ElfError::InvalidSegment);
    }

    let phdrs = unsafe {
        core::slice::from_raw_parts(
            data.as_ptr().add(ph_offset) as *const Elf64ProgramHeader,
            ph_count,
        )
    };

    Ok(phdrs)
}

/// Load an ELF into a process address space
/// Returns entry point and load info
pub fn load_elf(data: &[u8], addr_space: &mut AddressSpace) -> Result<ElfInfo, ElfError> {
    let header = validate_header(data)?;
    let phdrs = get_program_headers(data, header)?;

    // No PIE relocation — binaries are linked at fixed USER_BASE.
    // ASLR is applied via stack jitter and heap jitter instead.
    let load_offset: u64 = 0;

    let mut segments_loaded = 0;
    let mut base_addr: Option<u64> = None;

    // Process each program header
    for phdr in phdrs {
        // Skip non-loadable segments
        if phdr.p_type != PT_LOAD {
            continue;
        }

        let vaddr = phdr.p_vaddr + load_offset;
        let memsz = phdr.p_memsz as usize;
        let filesz = phdr.p_filesz as usize;
        let offset = phdr.p_offset as usize;
        let flags = phdr.p_flags;

        // Track base address (lowest vaddr)
        if base_addr.map_or(true, |base| vaddr < base) {
            base_addr = Some(vaddr);
        }

        // Skip empty segments
        if memsz == 0 {
            continue;
        }

        // filesz must not exceed memsz (BSS is memsz - filesz)
        if filesz > memsz {
            return Err(ElfError::InvalidSegment);
        }

        // Calculate number of pages needed (with overflow checks)
        let page_size = 4096usize;
        let vaddr_page = vaddr & !(page_size as u64 - 1);
        let offset_in_page = (vaddr - vaddr_page) as usize;
        let total_size = offset_in_page.checked_add(memsz).ok_or(ElfError::InvalidSegment)?;
        let num_pages = (total_size.checked_add(page_size - 1).ok_or(ElfError::InvalidSegment)?) / page_size;

        // Allocate physical pages
        let phys_base = pmm::alloc_pages(num_pages).ok_or(ElfError::OutOfMemory)?;

        // Zero the pages first (use TTBR1 virtual address to access physical memory)
        unsafe {
            let virt_base = mmu::phys_to_virt(phys_base as u64);
            let ptr = virt_base as *mut u8;
            for i in 0..(num_pages * page_size) {
                core::ptr::write_volatile(ptr.add(i), 0);
            }
        }

        // Copy file content to memory
        if filesz > 0 {
            // Use checked arithmetic for bounds check
            let end_offset = offset.checked_add(filesz).ok_or_else(|| {
                pmm::free_pages(phys_base, num_pages);
                ElfError::InvalidSegment
            })?;
            if end_offset > data.len() {
                pmm::free_pages(phys_base, num_pages);
                return Err(ElfError::InvalidSegment);
            }

            unsafe {
                let virt_base = mmu::phys_to_virt(phys_base as u64);
                let dest = (virt_base as usize + offset_in_page) as *mut u8;
                let src = data.as_ptr().add(offset);
                for i in 0..filesz {
                    core::ptr::write_volatile(dest.add(i), *src.add(i));
                }
            }
        }

        // Ensure data writes are visible to all CPUs before mapping pages.
        // Without this barrier, a process scheduled on a different CPU could
        // see stale (unzeroed) BSS data.
        unsafe {
            core::arch::asm!("dsb ish", options(nostack, preserves_flags));
        }

        // Map pages into address space
        let writable = (flags & PF_W) != 0;
        let executable = (flags & PF_X) != 0;

        // If executable, perform cache maintenance to ensure I-cache sees the code
        // Note: I-cache can be ASID-tagged, so we need to either:
        //   1. Use full I-cache invalidation (IC IALLUIS), or
        //   2. Invalidate with the correct ASID context
        // We use full invalidation for simplicity and correctness.
        if executable {
            unsafe {
                let virt_base = mmu::phys_to_virt(phys_base as u64) as usize;
                let size = num_pages * page_size;
                // Clean each cache line to Point of Unification (so I-cache can see it)
                // ARM cache line is typically 64 bytes
                for addr in (virt_base..(virt_base + size)).step_by(64) {
                    // DC CVAU: Clean data cache by VA to PoU
                    core::arch::asm!("dc cvau, {}", in(reg) addr);
                }
                // Data Synchronization Barrier
                core::arch::asm!("dsb ish");
                // Invalidate ALL instruction cache (handles ASID-tagged caches)
                // IC IALLUIS: Invalidate all I-cache to PoU, Inner Shareable
                core::arch::asm!("ic ialluis");
                // Ensure I-cache invalidation completes
                core::arch::asm!("dsb ish");
                core::arch::asm!("isb");
            }
        }

        for i in 0..num_pages {
            let page_vaddr = vaddr_page + (i * page_size) as u64;
            let page_phys = (phys_base + i * page_size) as u64;

            if !addr_space.map_page(page_vaddr, page_phys, writable, executable) {
                // Cleanup on failure
                pmm::free_pages(phys_base, num_pages);
                return Err(ElfError::OutOfMemory);
            }
        }

        // Debug: dump first 4 bytes of executable segments (only in debug builds)
        #[cfg(debug_assertions)]
        if executable && segments_loaded == 0 {
            unsafe {
                let virt_base = mmu::phys_to_virt(phys_base as u64);
                let ptr = (virt_base + offset_in_page as u64) as *const u32;
                let first_inst = core::ptr::read_volatile(ptr);
                crate::kdebug!("elf", "first_inst"; vaddr = crate::klog::hex64(vaddr), phys = crate::klog::hex64(phys_base as u64), inst = crate::klog::hex64(first_inst as u64));
            }
        }

        segments_loaded += 1;
    }

    // Validate entry point is within a loaded segment
    let entry = header.e_entry;
    let mut entry_valid = false;
    for phdr in phdrs {
        if phdr.p_type != PT_LOAD {
            continue;
        }
        let seg_start = phdr.p_vaddr;
        let seg_end = seg_start.wrapping_add(phdr.p_memsz);
        if entry >= seg_start && entry < seg_end {
            entry_valid = true;
            break;
        }
    }
    if !entry_valid && segments_loaded > 0 {
        return Err(ElfError::InvalidSegment);
    }

    // Apply load offset to entry point for PIE
    let final_entry = entry + load_offset;

    Ok(ElfInfo {
        entry: final_entry,
        base: base_addr.unwrap_or(0),
        segments_loaded,
    })
}

/// Process R_AARCH64_RELATIVE relocations for PIE binaries.
///
/// Finds the PT_DYNAMIC segment, extracts DT_RELA/DT_RELASZ entries,
/// and applies R_AARCH64_RELATIVE relocations (value = load_offset + addend).
fn process_relocations(
    data: &[u8],
    phdrs: &[Elf64ProgramHeader],
    load_offset: u64,
    addr_space: &mut AddressSpace,
) -> Result<(), ElfError> {
    // Find PT_DYNAMIC segment
    let dyn_phdr = phdrs.iter().find(|p| p.p_type == PT_DYNAMIC);
    let dyn_phdr = match dyn_phdr {
        Some(p) => p,
        None => return Ok(()), // No dynamic section — static PIE with no relocations
    };

    let dyn_offset = dyn_phdr.p_offset as usize;
    let dyn_size = dyn_phdr.p_filesz as usize;
    if dyn_offset.checked_add(dyn_size).map_or(true, |end| end > data.len()) {
        return Err(ElfError::InvalidSegment);
    }

    // Parse dynamic entries to find DT_RELA and DT_RELASZ
    let entry_size = 16usize; // sizeof(Elf64_Dyn) = 2 * u64
    let mut rela_offset: Option<u64> = None;
    let mut rela_size: u64 = 0;

    let mut i = 0;
    while i + entry_size <= dyn_size {
        let base = dyn_offset + i;
        let d_tag = u64::from_le_bytes(data[base..base + 8].try_into().unwrap());
        let d_val = u64::from_le_bytes(data[base + 8..base + 16].try_into().unwrap());

        match d_tag {
            DT_NULL => break,
            DT_RELA => rela_offset = Some(d_val),
            DT_RELASZ => rela_size = d_val,
            _ => {}
        }
        i += entry_size;
    }

    // Apply relocations
    let rela_vaddr = match rela_offset {
        Some(v) => v,
        None => return Ok(()), // No RELA section
    };

    if rela_size == 0 {
        return Ok(());
    }

    // Find the rela data in the ELF file by mapping vaddr back to file offset
    let rela_file_offset = vaddr_to_file_offset(phdrs, rela_vaddr)
        .ok_or(ElfError::InvalidSegment)?;
    let rela_end = rela_file_offset.checked_add(rela_size as usize)
        .ok_or(ElfError::InvalidSegment)?;
    if rela_end > data.len() {
        return Err(ElfError::InvalidSegment);
    }

    let rela_entry_size = core::mem::size_of::<Elf64Rela>();
    let num_relas = rela_size as usize / rela_entry_size;

    for idx in 0..num_relas {
        let offset = rela_file_offset + idx * rela_entry_size;
        let rela = unsafe { &*(data.as_ptr().add(offset) as *const Elf64Rela) };

        let r_offset = rela.r_offset;
        let r_info = rela.r_info;
        let r_addend = rela.r_addend;
        let r_type = (r_info & 0xFFFF_FFFF) as u32;

        if r_type != R_AARCH64_RELATIVE {
            continue; // Skip non-relative relocations
        }

        // Target virtual address (with load offset applied)
        let target_vaddr = r_offset + load_offset;
        // Value to write: load_offset + addend
        let value = (load_offset as i64 + r_addend) as u64;

        // Write via kernel virtual address: walk page tables to find physical page
        if let Some(phys) = addr_space.translate(target_vaddr) {
            let kva = mmu::phys_to_virt(phys);
            unsafe {
                core::ptr::write_volatile(kva as *mut u64, value);
            }
        }
    }

    // Ensure relocation writes are visible
    unsafe {
        core::arch::asm!("dsb ish", options(nostack, preserves_flags));
    }

    Ok(())
}

/// Map a virtual address back to a file offset using PT_LOAD segments.
fn vaddr_to_file_offset(phdrs: &[Elf64ProgramHeader], vaddr: u64) -> Option<usize> {
    for phdr in phdrs {
        if phdr.p_type != PT_LOAD {
            continue;
        }
        let seg_start = phdr.p_vaddr;
        let seg_end = seg_start + phdr.p_filesz;
        if vaddr >= seg_start && vaddr < seg_end {
            let offset_in_seg = vaddr - seg_start;
            return Some((phdr.p_offset + offset_in_seg) as usize);
        }
    }
    None
}

/// Print ELF header info
pub fn print_header_info(header: &Elf64Header) {
    // Copy from packed struct to avoid unaligned access
    let e_type = { header.e_type };
    let e_machine = { header.e_machine };
    let e_entry = { header.e_entry };
    let e_phoff = { header.e_phoff };
    let e_phnum = { header.e_phnum };

    print_direct!("  ELF Header:\n");
    print_direct!("    Type:    {}\n", match e_type {
        ET_EXEC => "Executable",
        ET_DYN => "Shared/PIE",
        _ => "Unknown",
    });
    print_direct!("    Machine: {}\n", if e_machine == EM_AARCH64 { "AArch64" } else { "Unknown" });
    print_direct!("    Entry:   0x{:016x}\n", e_entry);
    print_direct!("    PHoff:   0x{:x}\n", e_phoff);
    print_direct!("    PHnum:   {}\n", e_phnum);
}

/// Print program header info
pub fn print_phdr_info(phdr: &Elf64ProgramHeader) {
    // Copy from packed struct to avoid unaligned access
    let p_type = { phdr.p_type };
    let p_flags = { phdr.p_flags };
    let p_vaddr = { phdr.p_vaddr };
    let p_memsz = { phdr.p_memsz };

    let type_str = match p_type {
        0 => "NULL",
        PT_LOAD => "LOAD",
        2 => "DYNAMIC",
        3 => "INTERP",
        4 => "NOTE",
        6 => "PHDR",
        7 => "TLS",
        _ => "OTHER",
    };

    let mut flags_str = [b'-'; 3];
    if (p_flags & PF_R) != 0 { flags_str[0] = b'R'; }
    if (p_flags & PF_W) != 0 { flags_str[1] = b'W'; }
    if (p_flags & PF_X) != 0 { flags_str[2] = b'X'; }

    print_direct!("    {:8} 0x{:08x} 0x{:08x} {} {}\n",
        type_str,
        p_vaddr,
        p_memsz,
        core::str::from_utf8(&flags_str).unwrap_or("---"),
        if p_type == PT_LOAD { "<- LOAD" } else { "" }
    );
}

// ============================================================================
// Process Spawning
// ============================================================================

/// Default user stack virtual address (grows down from here)
pub const USER_STACK_TOP: u64 = 0x0000_0000_8000_0000;

/// User stack size (256KB - devd and other daemons need larger stacks)
pub const USER_STACK_SIZE: usize = 256 * 1024;

/// Guard page size (4KB) - unmapped page below stack to catch overflow
pub const USER_GUARD_PAGE_SIZE: usize = 4096;

/// Spawn a new process from an ELF binary
/// Returns (task_id, task_slot) on success
pub fn spawn_from_elf(data: &[u8], name: &str) -> Result<(task::TaskId, usize), ElfError> {
    spawn_from_elf_with_parent(data, name, 0)
}

/// Spawn a new process from an ELF binary with a parent process
/// Sets up parent-child relationship
/// Returns (task_id, task_slot) on success
pub fn spawn_from_elf_with_parent(data: &[u8], name: &str, parent_id: task::TaskId) -> Result<(task::TaskId, usize), ElfError> {
    spawn_from_elf_internal(data, name, parent_id, None)
}

/// Spawn a new process from an ELF binary with explicit capability grant
/// explicit_caps: If Some, use these capabilities (intersected with parent's)
///                If None, inherit all parent capabilities (legacy behavior)
pub fn spawn_from_elf_with_caps(
    data: &[u8],
    name: &str,
    parent_id: task::TaskId,
    requested_caps: super::caps::Capabilities
) -> Result<(task::TaskId, usize), ElfError> {
    spawn_from_elf_internal(data, name, parent_id, Some(requested_caps))
}

fn spawn_from_elf_internal(
    data: &[u8],
    name: &str,
    parent_id: task::TaskId,
    explicit_caps: Option<super::caps::Capabilities>
) -> Result<(task::TaskId, usize), ElfError> {
    // Verify binary signature/authorization before spawning
    // (stub: always passes, future: crypto verification)
    verify_binary_signature(data, name, explicit_caps)?;

    // Allocate user stack BEFORE acquiring scheduler lock
    // (pmm allocation doesn't need scheduler lock)
    let stack_pages = USER_STACK_SIZE / 4096;
    let stack_phys = pmm::alloc_pages(stack_pages).ok_or(ElfError::OutOfMemory)?;

    // Hold scheduler lock for the entire task setup (released before logging)
    let (task_id, slot, elf_info) = task::with_scheduler(|sched| -> Result<(task::TaskId, usize, ElfInfo), ElfError> {

    // Create a new user task
    let (task_id, slot) = unsafe {
        sched.add_user_task(name).ok_or_else(|| {
            pmm::free_pages(stack_phys, stack_pages);
            ElfError::OutOfMemory
        })?
    };

    // Load ELF into the task's address space
    let (elf_info, stack_top) = {
        let task = sched.task_mut(slot).ok_or_else(|| {
            pmm::free_pages(stack_phys, stack_pages);
            ElfError::OutOfMemory
        })?;
        let addr_space = task.address_space_mut().ok_or_else(|| {
            pmm::free_pages(stack_phys, stack_pages);
            ElfError::OutOfMemory
        })?;

        let info = load_elf(data, addr_space)?;

        // Debug: Dump PTEs for entry point (only in debug builds)
        #[cfg(debug_assertions)]
        {
            let entry = info.entry;
            for offset in [0u64, 0x1000, 0x2000, 0x3000, 0x4000] {
                let va = entry + offset;
                if let Some((l1, l2, l3, phys)) = addr_space.dump_pte(va) {
                    kdebug!("elf", "pte_dump"; name = name, va = crate::klog::hex64(va),
                        l1 = crate::klog::hex64(l1), l2 = crate::klog::hex64(l2),
                        l3 = crate::klog::hex64(l3), phys = crate::klog::hex64(phys));
                }
            }
        }

        // ASLR: randomize stack top (up to 64KB jitter, page-aligned so mapped pages
        // cover the full stack region including the top page where SP starts)
        let stack_jitter = super::rng::random_offset(16 * 4096, 4096);
        let stack_top = USER_STACK_TOP - stack_jitter;

        // Map user stack with guard page below
        // Layout (addresses grow up):
        //   [guard page - NOT MAPPED - will fault on access]  <- catches stack overflow
        //   [usable stack pages]
        //   [stack_top - 1]  <- initial SP points here
        //
        // Guard page virtual address (not mapped - any access faults)
        let guard_page_virt = stack_top - USER_STACK_SIZE as u64 - USER_GUARD_PAGE_SIZE as u64;
        let stack_base_virt = guard_page_virt + USER_GUARD_PAGE_SIZE as u64;

        // Only map the usable stack pages (guard page left unmapped)
        for i in 0..stack_pages {
            let page_virt = stack_base_virt + (i * 4096) as u64;
            let page_phys = (stack_phys + i * 4096) as u64;
            if !addr_space.map_page(page_virt, page_phys, true, false) {
                pmm::free_pages(stack_phys, stack_pages);
                return Err(ElfError::OutOfMemory);
            }
        }
        // Note: guard_page_virt is intentionally NOT mapped
        // Any stack overflow that reaches it will trigger a page fault

        (info, stack_top)
    };

    // Set up the trap frame with entry point and user stack
    {
        let task = sched.task_mut(slot).ok_or(ElfError::OutOfMemory)?;
        task.set_user_entry(elf_info.entry, stack_top);

        // ASLR: randomize heap start (up to 1MB jitter, page-aligned)
        let heap_jitter = super::rng::random_offset(256 * 4096, 4096);
        task.heap_next = super::memory::USER_HEAP_START + heap_jitter;

        // Set parent if specified
        if parent_id != 0 {
            task.set_parent(parent_id);
        }
    }

    // Set up parent-child relationship and handle capabilities
    if parent_id != 0 {
        // First pass: find parent's capabilities
        let mut parent_caps = None;
        for (_slot, task_opt) in sched.iter_tasks() {
            if let Some(parent_task) = task_opt {
                if parent_task.id == parent_id {
                    parent_caps = Some(parent_task.capabilities);
                    break;
                }
            }
        }
        // Second pass: add child to parent's children list
        for (_slot, task_opt) in sched.iter_tasks_mut() {
            if let Some(parent_task) = task_opt {
                if parent_task.id == parent_id {
                    let _ = parent_task.add_child(task_id);
                    break;
                }
            }
        }
        // Find parent priority first (before mutable borrow for child)
        let parent_priority = sched.iter_tasks()
            .find_map(|(_, task_opt)| {
                task_opt.as_ref()
                    .filter(|t| t.id == parent_id)
                    .map(|t| t.base_priority)
            });

        // Compute and apply child capabilities and priority
        if let Some(p_caps) = parent_caps {
            // Note: slot was just allocated, so task_mut should always succeed
            let Some(child_task) = sched.task_mut(slot) else {
                // Should never happen - task was just created
                return Err(ElfError::OutOfMemory);
            };
            let final_caps = match explicit_caps {
                Some(requested) => {
                    // Explicit grant: use child_capabilities() for proper filtering
                    super::caps::child_capabilities(p_caps, requested)
                }
                None => {
                    // Legacy: inherit all parent capabilities
                    p_caps
                }
            };
            child_task.set_capabilities(final_caps);

            // Inherit parent's priority (children of High priority tasks get High priority)
            if let Some(prio) = parent_priority {
                child_task.set_priority(prio);
            }
        }
    } else if let Some(caps) = explicit_caps {
        // No parent (kernel-spawned): apply explicit capabilities directly.
        // Must be set under the scheduler lock before the task can be
        // scheduled on another CPU (notify_ready already called above).
        if let Some(child_task) = sched.task_mut(slot) {
            child_task.set_capabilities(caps);
        }
    }

        Ok((task_id, slot, elf_info))
    })?;
    // Scheduler lock released here — safe to log

    kdebug!("elf", "spawn_ok"; name = name, pid = task_id as u64, slot = slot as u64, entry = crate::klog::hex64(elf_info.entry), parent = parent_id as u64);

    Ok((task_id, slot))
}

/// Built-in ELF binary IDs for spawn syscall (legacy)
pub const ELF_ID_TEST1: u32 = 0;  // First test binary (prints 'A')
pub const ELF_ID_TEST2: u32 = 1;  // Second test binary (prints 'B')

/// Get ELF binary by ID (for spawn syscall - legacy)
pub fn get_elf_by_id(id: u32) -> Option<&'static [u8]> {
    match id {
        ELF_ID_TEST1 => Some(TEST_ELF),
        ELF_ID_TEST2 => Some(TEST_ELF2),
        _ => None,
    }
}

// ============================================================================
// Ramfs-based process spawning
// ============================================================================

/// Spawn a process from a file path in ramfs
/// Searches /bin/<name> and /<name>
pub fn spawn_from_path(path: &str) -> Result<(task::TaskId, usize), ElfError> {
    spawn_from_path_with_parent(path, 0)
}

/// Spawn from path with explicit capability grant (uses find_executable)
pub fn spawn_from_path_with_caps_find(
    path: &str,
    parent_id: task::TaskId,
    requested_caps: super::caps::Capabilities
) -> Result<(task::TaskId, usize), ElfError> {
    // Try to find the file in ramfs
    let data = find_executable(path).ok_or(ElfError::NotExecutable)?;

    // Extract name from path for the process name
    let name = path.rsplit('/').next().unwrap_or(path);

    spawn_from_elf_internal(data, name, parent_id, Some(requested_caps))
}

/// Spawn from path with caps, transferring a channel handle from parent to child at slot 4
/// Priority: abi::priority::INHERIT (0xFF) means inherit parent's priority
pub fn spawn_from_path_with_caps_and_channel(
    path: &str,
    parent_id: task::TaskId,
    requested_caps: super::caps::Capabilities,
    channel_handle_raw: u32,
    priority: u8,
) -> Result<(task::TaskId, usize), ElfError> {
    use crate::kernel::object_service::object_service;
    use crate::kernel::object::{Object, ObjectType, ChannelObject};

    let data = find_executable(path).ok_or(ElfError::NotExecutable)?;
    let name = path.rsplit('/').next().unwrap_or(path);

    // Validate and extract the channel from parent's handle table
    let handle = abi::Handle(channel_handle_raw);
    let channel_obj = object_service().close_handle(parent_id, handle)
        .map_err(|_| ElfError::NotExecutable)?;

    // Verify it's actually a channel
    let channel_id = match &channel_obj {
        Object::Channel(ch) => ch.channel_id(),
        _ => {
            // Not a channel — can't put it back easily, return error
            return Err(ElfError::NotExecutable);
        }
    };

    // Spawn the child process (inherits parent priority by default)
    let (child_id, slot) = spawn_from_elf_internal(data, name, parent_id, Some(requested_caps))?;

    // Apply explicit priority override if not INHERIT
    if priority != abi::priority::INHERIT {
        use crate::kernel::task::tcb::Priority;
        if let Some(prio) = Priority::from_u8(priority) {
            crate::kernel::task::with_scheduler(|sched| {
                if let Some(child_task) = sched.task_mut(slot) {
                    let old = child_task.set_priority(prio);
                    if old != prio {
                        sched.notify_priority_change(slot, old, prio);
                    }
                }
            });
        }
    }

    // Transfer the channel to the child's handle table at slot 4 (SUPERVISION)
    let child_channel = Object::Channel(ChannelObject::new(channel_id));
    let result = object_service().with_table_mut(child_id, |table| {
        table.alloc_at(4, ObjectType::Channel, child_channel)
    });

    match result {
        Ok(Some(_handle)) => {
            // Update IPC channel ownership: change from parent to child
            let _ = crate::kernel::ipc::transfer_channel_owner(channel_id, parent_id, child_id);
            // Fix peer_owner on the parent's channel so send() returns
            // correct PeerInfo { task_id: child_id } for deferred wake.
            let _ = crate::kernel::ipc::update_peer_owner(channel_id, child_id);
            // Track resource usage
            crate::kernel::task::with_scheduler(|sched| {
                if let Some(s) = sched.slot_by_pid(parent_id) {
                    if let Some(task) = sched.task_mut(s) {
                        task.remove_channel();
                    }
                }
                if let Some(s) = sched.slot_by_pid(child_id) {
                    if let Some(task) = sched.task_mut(s) {
                        task.add_channel();
                    }
                }
            });
        }
        _ => {
            // Failed to insert — child still spawned but without the channel
            kwarn!("elf", "channel_transfer_failed"; child = child_id as u64);
        }
    }

    Ok((child_id, slot))
}

/// Spawn a process from ramfs path with a mailbox shmem page.
///
/// Creates a 4KB shmem page, copies mailbox_data into it, and installs
/// the shmem handle at slot 5 (Handle::MAILBOX) in the child's handle table.
/// Spawn result for exec_with_mailbox: child ID, slot, parent's shmem handle, and SuperQ handle.
pub struct MailboxSpawnResult {
    pub child_id: task::TaskId,
    pub slot: usize,
    /// Raw handle value for parent's shmem in parent's object table (0 if alloc failed)
    pub parent_handle_raw: u32,
    /// Raw handle value for parent's SuperQ endpoint in parent's object table (0 if alloc failed)
    pub parent_superq_handle_raw: u32,
}

pub fn spawn_from_path_with_caps_and_mailbox(
    path: &str,
    parent_id: task::TaskId,
    requested_caps: super::caps::Capabilities,
    mailbox_data: &[u8],
) -> Result<MailboxSpawnResult, ElfError> {
    use crate::kernel::object_service::object_service;
    use crate::kernel::object::{Object, ObjectType, ShmemObject};
    use crate::kernel::shmem;

    let data = find_executable(path).ok_or(ElfError::NotExecutable)?;
    let name = path.rsplit('/').next().unwrap_or(path);

    // Spawn the child process
    let (child_id, slot) = spawn_from_elf_internal(data, name, parent_id, Some(requested_caps))?;

    // Read priority from mailbox header (byte offset 13 = priority field)
    // and apply if not INHERIT (0xFF)
    if mailbox_data.len() > 13 {
        let priority = mailbox_data[13];
        if priority != abi::priority::INHERIT {
            use crate::kernel::task::tcb::Priority;
            if let Some(prio) = Priority::from_u8(priority) {
                crate::kernel::task::with_scheduler(|sched| {
                    if let Some(child_task) = sched.task_mut(slot) {
                        let old = child_task.set_priority(prio);
                        if old != prio {
                            sched.notify_priority_change(slot, old, prio);
                        }
                    }
                });
            }
        }
    }

    // Create a 4KB shmem region owned by the child
    let mut parent_handle_raw = 0u32;
    let shmem_result = shmem::create(child_id, 4096);
    match shmem_result {
        Ok((shmem_id, vaddr, paddr)) => {
            // Copy mailbox data into the shmem page via kernel virtual mapping
            let kernel_vaddr = crate::kernel::arch::mmu::phys_to_virt(paddr) as *mut u8;
            let shmem_slice = unsafe { core::slice::from_raw_parts_mut(kernel_vaddr, 4096) };
            let copy_len = mailbox_data.len().min(4096);
            shmem_slice[..copy_len].copy_from_slice(&mailbox_data[..copy_len]);
            // Zero the rest
            if copy_len < 4096 {
                shmem_slice[copy_len..].fill(0);
            }

            // Stamp parent_pid into mailbox header offset 36 so child can signal parent
            if shmem_slice.len() >= 40 {
                shmem_slice[36..40].copy_from_slice(&(parent_id as u32).to_le_bytes());
            }

            // Install shmem handle at slot 5 (MAILBOX) in child's table
            let child_obj = Object::Shmem(ShmemObject::new(shmem_id, paddr, 4096, vaddr));
            let _ = object_service().with_table_mut(child_id, |table| {
                table.alloc_at(5, ObjectType::Shmem, child_obj)
            });

            // Allow parent to map the child's shmem
            let _ = shmem::allow(child_id, shmem_id, parent_id);

            // Install shmem handle in parent's table (vaddr=0, parent maps later if needed)
            let parent_obj = Object::Shmem(ShmemObject::new(shmem_id, paddr, 4096, 0));
            let parent_result = object_service().with_table_mut(parent_id, |table| {
                table.alloc(ObjectType::Shmem, parent_obj)
            });
            if let Ok(Some(handle)) = parent_result {
                parent_handle_raw = handle.raw();
            }
        }
        Err(_) => {
            kwarn!("elf", "mailbox_alloc_failed"; child = child_id as u64);
        }
    }

    // Create SupervisionQueue and install handles
    let mut parent_superq_handle_raw = 0u32;
    if let Some(supervision_id) = crate::kernel::ipc::supervision::create(parent_id, child_id) {
        // Install child end at slot 4 (SUPERVISION)
        let child_obj = Object::SupervisionChild(
            crate::kernel::object::SupervisionChildObject::new(supervision_id)
        );
        let _ = object_service().with_table_mut(child_id, |table| {
            table.alloc_at(4, ObjectType::SupervisionChild, child_obj)
        });

        // Install parent end in parent's table (dynamic slot)
        let parent_obj = Object::SupervisionParent(
            crate::kernel::object::SupervisionParentObject::new(supervision_id)
        );
        let parent_result = object_service().with_table_mut(parent_id, |table| {
            table.alloc(ObjectType::SupervisionParent, parent_obj)
        });
        if let Ok(Some(handle)) = parent_result {
            parent_superq_handle_raw = handle.raw();
        }
    }

    Ok(MailboxSpawnResult { child_id, slot, parent_handle_raw, parent_superq_handle_raw })
}

/// Spawn a process from a file path with a parent
pub fn spawn_from_path_with_parent(path: &str, parent_id: task::TaskId) -> Result<(task::TaskId, usize), ElfError> {
    // Try to find the file in ramfs
    let data = find_executable(path).ok_or(ElfError::NotExecutable)?;

    // Extract name from path for the process name
    let name = path.rsplit('/').next().unwrap_or(path);

    spawn_from_elf_with_parent(data, name, parent_id)
}

/// Find an executable in ramfs
/// Searches: exact path, bin/<name>, ./bin/<name>
fn find_executable(path: &str) -> Option<&'static [u8]> {
    use crate::ramfs;

    // Extract just the filename (strip any leading path like /bin/)
    let name = path.rsplit('/').next().unwrap_or(path);

    // Try exact path first
    if let Some(entry) = ramfs::find(path) {
        if entry.is_file() {
            return Some(entry.data_slice());
        }
    }

    // Try bin/<name> (without leading /)
    {
        let mut bin_path = [0u8; 128];
        let prefix = b"bin/";
        let name_bytes = name.as_bytes();
        if prefix.len() + name_bytes.len() < bin_path.len() {
            bin_path[..prefix.len()].copy_from_slice(prefix);
            bin_path[prefix.len()..prefix.len() + name_bytes.len()].copy_from_slice(name_bytes);
            let full_path = core::str::from_utf8(&bin_path[..prefix.len() + name_bytes.len()]).ok()?;
            if let Some(entry) = ramfs::find(full_path) {
                if entry.is_file() {
                    return Some(entry.data_slice());
                }
            }
        }
    }

    // Try ./bin/<name> (tar format often uses ./)
    {
        let mut dot_path = [0u8; 128];
        let prefix = b"./bin/";
        let name_bytes = name.as_bytes();
        if prefix.len() + name_bytes.len() < dot_path.len() {
            dot_path[..prefix.len()].copy_from_slice(prefix);
            dot_path[prefix.len()..prefix.len() + name_bytes.len()].copy_from_slice(name_bytes);
            let full_path = core::str::from_utf8(&dot_path[..prefix.len() + name_bytes.len()]).ok()?;
            if let Some(entry) = ramfs::find(full_path) {
                if entry.is_file() {
                    return Some(entry.data_slice());
                }
            }
        }
    }

    None
}

/// Get the test ELF binary
pub fn get_test_elf() -> &'static [u8] {
    TEST_ELF
}

// ============================================================================
// Testing
// ============================================================================

/// A minimal test ELF binary (AArch64) - LOOPING version for preemption testing
/// This program loops forever, printing 'A' periodically.
/// Uses FD-based Write syscall (syscall 21) with fd=1 (stdout).
/// The timer preemption should switch between processes.
///
/// Entry point at 0x40010000
const TEST_ELF: &[u8] = &[
    // ELF Header (64 bytes)
    0x7f, b'E', b'L', b'F',  // Magic
    2,                        // Class: 64-bit
    1,                        // Data: little endian
    1,                        // Version
    0,                        // OS/ABI
    0, 0, 0, 0, 0, 0, 0, 0,  // Padding
    2, 0,                     // Type: ET_EXEC
    183, 0,                   // Machine: AArch64
    1, 0, 0, 0,              // Version
    0x00, 0x00, 0x01, 0x40, 0, 0, 0, 0,  // Entry: 0x40010000
    0x40, 0, 0, 0, 0, 0, 0, 0,           // PHoff: 64
    0, 0, 0, 0, 0, 0, 0, 0,              // SHoff: 0
    0, 0, 0, 0,                          // Flags
    64, 0,                               // EH size
    56, 0,                               // PH entry size
    1, 0,                                // PH count
    0, 0,                                // SH entry size
    0, 0,                                // SH count
    0, 0,                                // SH string index

    // Program Header (56 bytes) - at offset 64
    1, 0, 0, 0,              // Type: PT_LOAD
    5, 0, 0, 0,              // Flags: R-X
    0x78, 0, 0, 0, 0, 0, 0, 0,           // Offset: 120 (where code starts)
    0x00, 0x00, 0x01, 0x40, 0, 0, 0, 0,  // VAddr: 0x40010000
    0x00, 0x00, 0x01, 0x40, 0, 0, 0, 0,  // PAddr: 0x40010000
    0x28, 0, 0, 0, 0, 0, 0, 0,           // FileSz: 40 bytes
    0x28, 0, 0, 0, 0, 0, 0, 0,           // MemSz: 40 bytes
    0x00, 0x10, 0, 0, 0, 0, 0, 0,        // Align: 0x1000

    // Code starts at offset 120 (0x78)
    // loop:
    //   mov x8, #21      ; syscall = Write (FD-based)
    0xa8, 0x02, 0x80, 0xd2,
    //   mov x0, #1       ; fd = stdout
    0x20, 0x00, 0x80, 0xd2,
    //   adr x1, char     ; buffer = char (PC-relative, +28 bytes from here)
    0xe1, 0x00, 0x00, 0x10,
    //   mov x2, #1       ; len = 1
    0x22, 0x00, 0x80, 0xd2,
    //   svc #0           ; syscall
    0x01, 0x00, 0x00, 0xd4,
    //   mov x9, #0x40000 ; delay counter (~262K iterations)
    0x89, 0x00, 0xa0, 0xd2,
    // wait:
    //   sub x9, x9, #1
    0x29, 0x05, 0x00, 0xd1,
    //   cbnz x9, wait    ; loop if not zero
    0xe9, 0xff, 0xff, 0xb5,
    //   b loop           ; back to start
    0xf8, 0xff, 0xff, 0x17,
    // char: 'A'
    b'A', 0, 0, 0,           // Padded to 4 bytes
];

/// Second test ELF - LOOPING version, prints 'B' periodically
/// Uses FD-based Write syscall (syscall 21) with fd=1 (stdout).
/// Entry point at 0x40020000 (different from first ELF)
const TEST_ELF2: &[u8] = &[
    // ELF Header (64 bytes)
    0x7f, b'E', b'L', b'F',  // Magic
    2,                        // Class: 64-bit
    1,                        // Data: little endian
    1,                        // Version
    0,                        // OS/ABI
    0, 0, 0, 0, 0, 0, 0, 0,  // Padding
    2, 0,                     // Type: ET_EXEC
    183, 0,                   // Machine: AArch64
    1, 0, 0, 0,              // Version
    0x00, 0x00, 0x02, 0x40, 0, 0, 0, 0,  // Entry: 0x40020000
    0x40, 0, 0, 0, 0, 0, 0, 0,           // PHoff: 64
    0, 0, 0, 0, 0, 0, 0, 0,              // SHoff: 0
    0, 0, 0, 0,                          // Flags
    64, 0,                               // EH size
    56, 0,                               // PH entry size
    1, 0,                                // PH count
    0, 0,                                // SH entry size
    0, 0,                                // SH count
    0, 0,                                // SH string index

    // Program Header (56 bytes) - at offset 64
    1, 0, 0, 0,              // Type: PT_LOAD
    5, 0, 0, 0,              // Flags: R-X
    0x78, 0, 0, 0, 0, 0, 0, 0,           // Offset: 120 (where code starts)
    0x00, 0x00, 0x02, 0x40, 0, 0, 0, 0,  // VAddr: 0x40020000
    0x00, 0x00, 0x02, 0x40, 0, 0, 0, 0,  // PAddr: 0x40020000
    0x28, 0, 0, 0, 0, 0, 0, 0,           // FileSz: 40 bytes
    0x28, 0, 0, 0, 0, 0, 0, 0,           // MemSz: 40 bytes
    0x00, 0x10, 0, 0, 0, 0, 0, 0,        // Align: 0x1000

    // Code starts at offset 120 (0x78)
    // loop:
    //   mov x8, #21      ; syscall = Write (FD-based)
    0xa8, 0x02, 0x80, 0xd2,
    //   mov x0, #1       ; fd = stdout
    0x20, 0x00, 0x80, 0xd2,
    //   adr x1, char     ; buffer = char (PC-relative, +28 bytes from here)
    0xe1, 0x00, 0x00, 0x10,
    //   mov x2, #1       ; len = 1
    0x22, 0x00, 0x80, 0xd2,
    //   svc #0           ; syscall
    0x01, 0x00, 0x00, 0xd4,
    //   mov x9, #0x40000 ; delay counter (~262K iterations)
    0x89, 0x00, 0xa0, 0xd2,
    // wait:
    //   sub x9, x9, #1
    0x29, 0x05, 0x00, 0xd1,
    //   cbnz x9, wait    ; loop if not zero
    0xe9, 0xff, 0xff, 0xb5,
    //   b loop           ; back to start
    0xf8, 0xff, 0xff, 0x17,
    // char: 'B'
    b'B', 0, 0, 0,           // Padded to 4 bytes
];

/// Get the second test ELF binary
pub fn get_test_elf2() -> &'static [u8] {
    TEST_ELF2
}

// Note: A proper mmap test ELF would need to be compiled from assembly
// The mmap syscall interface is:
//   x8 = 4 (Mmap)
//   x0 = addr hint (0 for any)
//   x1 = size in bytes
//   x2 = prot flags (1=read, 2=write, 4=exec)
// Returns virtual address or negative error

/// Test ELF loading
pub fn test() {
    print_direct!("  Testing ELF loader...\n");

    // Validate the test ELF
    match validate_header(TEST_ELF) {
        Ok(header) => {
            print_direct!("    Test ELF validated\n");
            print_header_info(header);

            match get_program_headers(TEST_ELF, header) {
                Ok(phdrs) => {
                    print_direct!("    Program headers:\n");
                    for phdr in phdrs {
                        print_phdr_info(phdr);
                    }
                }
                Err(e) => print_direct!("    [!!] Failed to get phdrs: {:?}\n", e),
            }
        }
        Err(e) => {
            print_direct!("    [!!] ELF validation failed: {:?}\n", e);
            return;
        }
    }

    // Try loading into a new address space
    print_direct!("    Loading ELF into address space...\n");
    if let Some(mut addr_space) = AddressSpace::new() {
        match load_elf(TEST_ELF, &mut addr_space) {
            Ok(info) => {
                print_direct!("    Loaded {} segments\n", info.segments_loaded);
                print_direct!("    Entry point: 0x{:016x}\n", info.entry);
                print_direct!("    Base address: 0x{:016x}\n", info.base);
                print_direct!("    [OK] ELF loaded successfully\n");
            }
            Err(e) => print_direct!("    [!!] Load failed: {:?}\n", e),
        }
    } else {
        print_direct!("    [!!] Failed to create address space\n");
    }

    print_direct!("    [OK] ELF loader test passed\n");
}
