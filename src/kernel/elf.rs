//! ELF64 Loader

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Minimal ELF loader for AArch64 executables.
//! Parses ELF headers and loads PT_LOAD segments into process memory.

use super::addrspace::AddressSpace;
use super::pmm;
use crate::logln;
use crate::arch::aarch64::mmu;
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

/// Segment flags
pub const PF_X: u32 = 1;  // Executable
pub const PF_W: u32 = 2;  // Writable
pub const PF_R: u32 = 4;  // Readable

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

    let required_size = ph_offset + ph_size * ph_count;
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

    let mut segments_loaded = 0;
    let mut base_addr: Option<u64> = None;

    // Process each program header
    for phdr in phdrs {
        // Skip non-loadable segments
        if phdr.p_type != PT_LOAD {
            continue;
        }

        let vaddr = phdr.p_vaddr;
        let memsz = phdr.p_memsz as usize;
        let filesz = phdr.p_filesz as usize;
        let offset = phdr.p_offset as usize;
        let flags = phdr.p_flags;

        // Track base address (lowest vaddr)
        if base_addr.is_none() || vaddr < base_addr.unwrap() {
            base_addr = Some(vaddr);
        }

        // Skip empty segments
        if memsz == 0 {
            continue;
        }

        // Calculate number of pages needed
        let page_size = 4096usize;
        let vaddr_page = vaddr & !(page_size as u64 - 1);
        let offset_in_page = (vaddr - vaddr_page) as usize;
        let total_size = offset_in_page + memsz;
        let num_pages = (total_size + page_size - 1) / page_size;

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
            if offset + filesz > data.len() {
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

        // Map pages into address space
        let writable = (flags & PF_W) != 0;
        let executable = (flags & PF_X) != 0;

        // If executable, perform cache maintenance to ensure I-cache sees the code
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
                // Invalidate instruction cache
                for addr in (virt_base..(virt_base + size)).step_by(64) {
                    // IC IVAU: Invalidate instruction cache by VA to PoU
                    core::arch::asm!("ic ivau, {}", in(reg) addr);
                }
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

        segments_loaded += 1;
    }

    Ok(ElfInfo {
        entry: header.e_entry,
        base: base_addr.unwrap_or(0),
        segments_loaded,
    })
}

/// Print ELF header info
pub fn print_header_info(header: &Elf64Header) {
    // Copy from packed struct to avoid unaligned access
    let e_type = { header.e_type };
    let e_machine = { header.e_machine };
    let e_entry = { header.e_entry };
    let e_phoff = { header.e_phoff };
    let e_phnum = { header.e_phnum };

    logln!("  ELF Header:");
    logln!("    Type:    {}", match e_type {
        ET_EXEC => "Executable",
        ET_DYN => "Shared/PIE",
        _ => "Unknown",
    });
    logln!("    Machine: {}", if e_machine == EM_AARCH64 { "AArch64" } else { "Unknown" });
    logln!("    Entry:   0x{:016x}", e_entry);
    logln!("    PHoff:   0x{:x}", e_phoff);
    logln!("    PHnum:   {}", e_phnum);
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

    logln!("    {:8} 0x{:08x} 0x{:08x} {} {}",
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

/// User stack size (64KB)
pub const USER_STACK_SIZE: usize = 64 * 1024;

/// Spawn a new process from an ELF binary
/// Returns (task_id, task_slot) on success
pub fn spawn_from_elf(data: &[u8], name: &str) -> Result<(task::TaskId, usize), ElfError> {
    spawn_from_elf_with_parent(data, name, 0)
}

/// Spawn a new process from an ELF binary with a parent process
/// Sets up parent-child relationship
/// Returns (task_id, task_slot) on success
pub fn spawn_from_elf_with_parent(data: &[u8], name: &str, parent_id: task::TaskId) -> Result<(task::TaskId, usize), ElfError> {
    // Create a new user task
    let (task_id, slot) = unsafe {
        task::scheduler().add_user_task(name).ok_or(ElfError::OutOfMemory)?
    };

    // Get access to the task's address space
    let task = unsafe {
        task::scheduler().task_mut(slot).ok_or(ElfError::OutOfMemory)?
    };

    let addr_space = task.address_space_mut().ok_or(ElfError::OutOfMemory)?;

    // Load the ELF into the address space
    let elf_info = load_elf(data, addr_space)?;

    // Allocate user stack
    let stack_pages = USER_STACK_SIZE / 4096;
    let stack_phys = pmm::alloc_pages(stack_pages).ok_or(ElfError::OutOfMemory)?;

    // Map user stack (at USER_STACK_TOP - USER_STACK_SIZE to USER_STACK_TOP)
    let stack_base_virt = USER_STACK_TOP - USER_STACK_SIZE as u64;
    for i in 0..stack_pages {
        let page_virt = stack_base_virt + (i * 4096) as u64;
        let page_phys = (stack_phys + i * 4096) as u64;
        if !addr_space.map_page(page_virt, page_phys, true, false) {
            pmm::free_pages(stack_phys, stack_pages);
            return Err(ElfError::OutOfMemory);
        }
    }

    // Set up the trap frame with entry point and user stack
    let task = unsafe {
        task::scheduler().task_mut(slot).ok_or(ElfError::OutOfMemory)?
    };
    task.set_user_entry(elf_info.entry, USER_STACK_TOP);

    // Set up parent-child relationship
    if parent_id != 0 {
        task.set_parent(parent_id);

        // Add child to parent's children list
        unsafe {
            let sched = task::scheduler();
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut parent_task) = task_opt {
                    if parent_task.id == parent_id {
                        parent_task.add_child(task_id);
                        break;
                    }
                }
            }
        }
    }

    logln!("    Spawned '{}' (PID {}) entry=0x{:x} parent={}", name, task_id, elf_info.entry, parent_id);

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

/// Spawn a process from a file path with a parent
pub fn spawn_from_path_with_parent(path: &str, parent_id: task::TaskId) -> Result<(task::TaskId, usize), ElfError> {
    // Try to find the file in ramfs
    let data = find_executable(path).ok_or(ElfError::NotExecutable)?;

    // Extract name from path for the process name
    let name = path.rsplit('/').next().unwrap_or(path);

    spawn_from_elf_with_parent(data, name, parent_id)
}

/// Find an executable in ramfs
/// Searches: exact path, /bin/<name>, /<name>
fn find_executable(path: &str) -> Option<&'static [u8]> {
    use crate::ramfs;

    // Try exact path first
    if let Some(entry) = ramfs::find(path) {
        if entry.is_file() {
            return Some(entry.data_slice());
        }
    }

    // Try with bin/ prefix
    if !path.starts_with("bin/") && !path.starts_with("/bin/") {
        let mut bin_path = [0u8; 128];
        let prefix = b"bin/";
        let path_bytes = path.as_bytes();
        if prefix.len() + path_bytes.len() < bin_path.len() {
            bin_path[..prefix.len()].copy_from_slice(prefix);
            bin_path[prefix.len()..prefix.len() + path_bytes.len()].copy_from_slice(path_bytes);
            let full_path = core::str::from_utf8(&bin_path[..prefix.len() + path_bytes.len()]).ok()?;
            if let Some(entry) = ramfs::find(full_path) {
                if entry.is_file() {
                    return Some(entry.data_slice());
                }
            }
        }
    }

    // Try ./bin/<name> (tar format often uses ./)
    if !path.starts_with("./") {
        let mut dot_path = [0u8; 128];
        let prefix = b"./bin/";
        let path_bytes = path.as_bytes();
        if prefix.len() + path_bytes.len() < dot_path.len() {
            dot_path[..prefix.len()].copy_from_slice(prefix);
            dot_path[prefix.len()..prefix.len() + path_bytes.len()].copy_from_slice(path_bytes);
            let full_path = core::str::from_utf8(&dot_path[..prefix.len() + path_bytes.len()]).ok()?;
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
    logln!("  Testing ELF loader...");

    // Validate the test ELF
    match validate_header(TEST_ELF) {
        Ok(header) => {
            logln!("    Test ELF validated");
            print_header_info(header);

            match get_program_headers(TEST_ELF, header) {
                Ok(phdrs) => {
                    logln!("    Program headers:");
                    for phdr in phdrs {
                        print_phdr_info(phdr);
                    }
                }
                Err(e) => logln!("    [!!] Failed to get phdrs: {:?}", e),
            }
        }
        Err(e) => {
            logln!("    [!!] ELF validation failed: {:?}", e);
            return;
        }
    }

    // Try loading into a new address space
    logln!("    Loading ELF into address space...");
    if let Some(mut addr_space) = AddressSpace::new() {
        match load_elf(TEST_ELF, &mut addr_space) {
            Ok(info) => {
                logln!("    Loaded {} segments", info.segments_loaded);
                logln!("    Entry point: 0x{:016x}", info.entry);
                logln!("    Base address: 0x{:016x}", info.base);
                logln!("    [OK] ELF loaded successfully");
            }
            Err(e) => logln!("    [!!] Load failed: {:?}", e),
        }
    } else {
        logln!("    [!!] Failed to create address space");
    }

    logln!("    [OK] ELF loader test passed");
}
