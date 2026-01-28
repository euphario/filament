//! Flattened Device Tree (FDT) Parser
//!
//! Parses the device tree blob passed from U-Boot to discover hardware.
//! FDT format: https://devicetree-specification.readthedocs.io/

// For device tree dump (visual/structured output), we use direct UART output
// For status/error messages, we use structured klog

use crate::{kinfo, kwarn, kerror, kdebug, print_direct};

/// FDT magic number (big-endian)
const FDT_MAGIC: u32 = 0xd00dfeed;

/// FDT structure tokens
const FDT_BEGIN_NODE: u32 = 0x00000001;
const FDT_END_NODE: u32 = 0x00000002;
const FDT_PROP: u32 = 0x00000003;
const FDT_NOP: u32 = 0x00000004;
const FDT_END: u32 = 0x00000009;

/// FDT Header (all fields are big-endian)
#[repr(C)]
struct FdtHeader {
    magic: u32,
    totalsize: u32,
    off_dt_struct: u32,
    off_dt_strings: u32,
    off_mem_rsvmap: u32,
    version: u32,
    last_comp_version: u32,
    boot_cpuid_phys: u32,
    size_dt_strings: u32,
    size_dt_struct: u32,
}

/// Convert big-endian u32 to native
fn be32(val: u32) -> u32 {
    u32::from_be(val)
}

/// Get DTB address from boot code
pub fn get_dtb_address() -> u64 {
    extern "C" {
        static BOOT_DTB_ADDR: u64;
    }
    unsafe { BOOT_DTB_ADDR }
}

/// FDT parser state
pub struct Fdt {
    base: *const u8,
    struct_base: *const u8,
    strings_base: *const u8,
    totalsize: u32,
}

impl Fdt {
    /// Parse FDT from a pointer (for embedded DTB already in kernel memory)
    pub fn parse_from_ptr(ptr: *const u8) -> Option<Self> {
        let base = ptr;

        // Read header
        let header = unsafe { &*(base as *const FdtHeader) };
        let magic = be32(header.magic);

        if magic != FDT_MAGIC {
            kerror!("fdt", "bad_magic"; got = crate::klog::hex32(magic), expected = crate::klog::hex32(FDT_MAGIC));
            return None;
        }

        let totalsize = be32(header.totalsize);
        let off_dt_struct = be32(header.off_dt_struct);
        let off_dt_strings = be32(header.off_dt_strings);
        let _version = be32(header.version);

        // FDT parse logging moved to single message after global FDT is set

        let struct_base = unsafe { base.add(off_dt_struct as usize) };
        let strings_base = unsafe { base.add(off_dt_strings as usize) };

        Some(Self {
            base,
            struct_base,
            strings_base,
            totalsize,
        })
    }

    /// Parse FDT at the given physical address
    /// Returns None if magic doesn't match or address is invalid
    pub fn parse(phys_addr: u64) -> Option<Self> {
        if phys_addr == 0 || phys_addr < 0x40000000 {
            kwarn!("fdt", "invalid_addr"; addr = crate::klog::hex64(phys_addr));
            return None;
        }

        // Convert physical to virtual (TTBR1 mapping)
        let virt_addr = phys_addr | 0xFFFF_0000_0000_0000;
        let base = virt_addr as *const u8;

        // Read header
        let header = unsafe { &*(base as *const FdtHeader) };
        let magic = be32(header.magic);

        if magic != FDT_MAGIC {
            kerror!("fdt", "bad_magic"; got = crate::klog::hex32(magic), expected = crate::klog::hex32(FDT_MAGIC));
            return None;
        }

        let totalsize = be32(header.totalsize);
        let off_dt_struct = be32(header.off_dt_struct);
        let off_dt_strings = be32(header.off_dt_strings);
        let _version = be32(header.version);

        kinfo!("fdt", "parsed"; source = "memory", addr = crate::klog::hex64(phys_addr));

        let struct_base = unsafe { base.add(off_dt_struct as usize) };
        let strings_base = unsafe { base.add(off_dt_strings as usize) };

        Some(Self {
            base,
            struct_base,
            strings_base,
            totalsize,
        })
    }

    /// Get a null-terminated string from the strings block
    fn get_string(&self, offset: u32) -> &str {
        let ptr = unsafe { self.strings_base.add(offset as usize) };
        let mut len = 0;
        while unsafe { *ptr.add(len) } != 0 && len < 256 {
            len += 1;
        }
        let slice = unsafe { core::slice::from_raw_parts(ptr, len) };
        core::str::from_utf8(slice).unwrap_or("<invalid>")
    }

    /// Read a big-endian u32 from the structure block
    fn read_u32(&self, offset: usize) -> u32 {
        let ptr = unsafe { self.struct_base.add(offset) as *const u32 };
        be32(unsafe { *ptr })
    }

    /// Dump the entire device tree (visual output to UART)
    pub fn dump(&self) {
        // Device tree dump uses direct UART output (visual diagnostic)
        print_direct!("\r\n========================================\r\n");
        print_direct!("  Device Tree Dump\r\n");
        print_direct!("========================================\r\n");

        let mut offset: usize = 0;
        let mut depth: usize = 0;

        loop {
            let token = self.read_u32(offset);
            offset += 4;

            match token {
                FDT_BEGIN_NODE => {
                    // Node name follows (null-terminated, aligned to 4 bytes)
                    let name_ptr = unsafe { self.struct_base.add(offset) };
                    let mut name_len = 0;
                    while unsafe { *name_ptr.add(name_len) } != 0 && name_len < 256 {
                        name_len += 1;
                    }
                    let name_slice = unsafe { core::slice::from_raw_parts(name_ptr, name_len) };
                    let name = core::str::from_utf8(name_slice).unwrap_or("<invalid>");

                    // Print with indentation
                    for _ in 0..depth {
                        print_direct!("  ");
                    }
                    if name.is_empty() {
                        print_direct!("/\r\n");
                    } else {
                        print_direct!("{}/\r\n", name);
                    }

                    // Align to next 4-byte boundary
                    offset += (name_len + 4) & !3;
                    depth += 1;
                }

                FDT_END_NODE => {
                    if depth > 0 {
                        depth -= 1;
                    }
                }

                FDT_PROP => {
                    // Property: len (4 bytes), nameoff (4 bytes), value (len bytes, aligned)
                    let len = self.read_u32(offset) as usize;
                    let nameoff = self.read_u32(offset + 4);
                    offset += 8;

                    let prop_name = self.get_string(nameoff);

                    // Print property with indentation
                    for _ in 0..depth {
                        print_direct!("  ");
                    }

                    if len == 0 {
                        print_direct!("  {} (empty)\r\n", prop_name);
                    } else if len == 4 {
                        // Likely a u32
                        let val = self.read_u32(offset);
                        print_direct!("  {} = <0x{:x}>\r\n", prop_name, val);
                    } else if len == 8 {
                        // Likely a u64 (reg or ranges)
                        let hi = self.read_u32(offset);
                        let lo = self.read_u32(offset + 4);
                        print_direct!("  {} = <0x{:x} 0x{:x}>\r\n", prop_name, hi, lo);
                    } else if self.is_printable_string(offset, len) {
                        // String property
                        let val_ptr = unsafe { self.struct_base.add(offset) };
                        let val_slice = unsafe { core::slice::from_raw_parts(val_ptr, len.saturating_sub(1)) };
                        let val = core::str::from_utf8(val_slice).unwrap_or("<invalid>");
                        print_direct!("  {} = \"{}\"\r\n", prop_name, val);
                    } else {
                        // Binary data - show first few bytes
                        print_direct!("  {} = [", prop_name);
                        let show = len.min(16);
                        for i in 0..show {
                            let b = unsafe { *self.struct_base.add(offset + i) };
                            print_direct!("{:02x}", b);
                            if i < show - 1 {
                                print_direct!(" ");
                            }
                        }
                        if len > 16 {
                            print_direct!("...");
                        }
                        print_direct!("] ({} bytes)\r\n", len);
                    }

                    // Align to next 4-byte boundary
                    offset += (len + 3) & !3;
                }

                FDT_NOP => {
                    // Skip
                }

                FDT_END => {
                    kdebug!("fdt", "dump_complete");
                    break;
                }

                _ => {
                    kwarn!("fdt", "unknown_token"; token = crate::klog::hex32(token), offset = offset as u64 - 4);
                    break;
                }
            }

            // Safety check
            if offset > self.totalsize as usize {
                kerror!("fdt", "walk_overflow");
                break;
            }
        }
    }

    /// Check if data at offset looks like a printable string
    fn is_printable_string(&self, offset: usize, len: usize) -> bool {
        if len == 0 {
            return false;
        }
        for i in 0..len {
            let b = unsafe { *self.struct_base.add(offset + i) };
            if i == len - 1 {
                // Last byte should be null terminator
                if b != 0 {
                    return false;
                }
            } else if b == 0 {
                // Null in middle = string list, still printable
                continue;
            } else if b < 0x20 || b > 0x7e {
                // Non-printable
                return false;
            }
        }
        true
    }

    /// Get a property from a specific node path
    /// Returns the raw property data as a byte slice
    ///
    /// Path format: "/" for root, "/soc" for /soc node, etc.
    /// Property name: "compatible", "reg", etc.
    pub fn get_property<'a>(&'a self, path: &str, prop_name: &str) -> Option<&'a [u8]> {
        // Parse path into components (max 16 levels deep)
        let mut target_components: [&str; 16] = [""; 16];
        let mut target_depth = 0;
        for component in path.split('/').filter(|s| !s.is_empty()) {
            if target_depth < 16 {
                target_components[target_depth] = component;
                target_depth += 1;
            }
        }

        let mut offset: usize = 0;
        let mut depth: usize = 0;

        loop {
            let token = self.read_u32(offset);
            offset += 4;

            match token {
                FDT_BEGIN_NODE => {
                    // Node name follows (null-terminated, aligned to 4 bytes)
                    let name_ptr = unsafe { self.struct_base.add(offset) };
                    let mut name_len = 0;
                    while unsafe { *name_ptr.add(name_len) } != 0 && name_len < 256 {
                        name_len += 1;
                    }
                    let name_slice = unsafe { core::slice::from_raw_parts(name_ptr, name_len) };
                    let name = core::str::from_utf8(name_slice).unwrap_or("");

                    // Update current path
                    if depth < 16 {
                        // Strip @address suffix for matching (e.g., "pcie@11300000" -> "pcie")
                        let match_name = name.split('@').next().unwrap_or(name);

                        // Check if we're at the target path
                        if depth < target_depth {
                            // Check if this component matches
                            let target = target_components[depth];
                            let matches = if target.contains('@') {
                                // Exact match including address
                                name == target
                            } else {
                                // Match without address suffix
                                match_name == target || name == target
                            };

                            if !matches && target_depth > 0 {
                                // Skip this subtree by finding matching END_NODE
                                offset += (name_len + 4) & !3;
                                self.skip_subtree(&mut offset);
                                continue;
                            }
                        }
                    }

                    // Align to next 4-byte boundary
                    offset += (name_len + 4) & !3;
                    depth += 1;
                }

                FDT_END_NODE => {
                    if depth > 0 {
                        depth -= 1;
                    }
                    // If we've left the target node without finding the property
                    if depth < target_depth {
                        return None;
                    }
                }

                FDT_PROP => {
                    // Property: len (4 bytes), nameoff (4 bytes), value (len bytes, aligned)
                    let len = self.read_u32(offset) as usize;
                    let nameoff = self.read_u32(offset + 4);
                    offset += 8;

                    let pname = self.get_string(nameoff);

                    // Check if we're at the target node and this is the property we want
                    let at_target = if target_depth == 0 {
                        depth == 1  // Root node
                    } else {
                        depth == target_depth
                    };

                    if at_target && pname == prop_name {
                        // Found it! Return the property data
                        let data_ptr = unsafe { self.struct_base.add(offset) };
                        let data = unsafe { core::slice::from_raw_parts(data_ptr, len) };
                        return Some(data);
                    }

                    // Align to next 4-byte boundary
                    offset += (len + 3) & !3;
                }

                FDT_NOP => {
                    // Skip
                }

                FDT_END => {
                    // End of tree, not found
                    return None;
                }

                _ => {
                    // Unknown token
                    return None;
                }
            }

            // Safety check
            if offset > self.totalsize as usize {
                return None;
            }
        }
    }

    /// Skip a subtree (helper for get_property)
    fn skip_subtree(&self, offset: &mut usize) {
        let mut skip_depth = 1usize;

        while skip_depth > 0 {
            let token = self.read_u32(*offset);
            *offset += 4;

            match token {
                FDT_BEGIN_NODE => {
                    // Skip node name
                    let name_ptr = unsafe { self.struct_base.add(*offset) };
                    let mut name_len = 0;
                    while unsafe { *name_ptr.add(name_len) } != 0 && name_len < 256 {
                        name_len += 1;
                    }
                    *offset += (name_len + 4) & !3;
                    skip_depth += 1;
                }
                FDT_END_NODE => {
                    skip_depth -= 1;
                }
                FDT_PROP => {
                    let len = self.read_u32(*offset) as usize;
                    *offset += 8; // len + nameoff
                    *offset += (len + 3) & !3;
                }
                FDT_NOP => {}
                FDT_END | _ => {
                    return;
                }
            }

            if *offset > self.totalsize as usize {
                return;
            }
        }
    }

    /// Get the compatible strings from the root node
    /// Returns an iterator over the null-separated compatible strings
    pub fn get_root_compatible(&self) -> Option<CompatibleIter<'_>> {
        let data = self.get_property("/", "compatible")?;
        Some(CompatibleIter { data, pos: 0 })
    }

    /// Check if any root compatible string matches the given pattern
    pub fn is_compatible(&self, pattern: &str) -> bool {
        if let Some(iter) = self.get_root_compatible() {
            for compat in iter {
                if compat == pattern {
                    return true;
                }
            }
        }
        false
    }
}

/// Iterator over null-separated compatible strings
pub struct CompatibleIter<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Iterator for CompatibleIter<'a> {
    type Item = &'a str;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.data.len() {
            return None;
        }

        // Find the null terminator
        let start = self.pos;
        while self.pos < self.data.len() && self.data[self.pos] != 0 {
            self.pos += 1;
        }

        if self.pos == start {
            return None;
        }

        let s = core::str::from_utf8(&self.data[start..self.pos]).ok()?;

        // Skip the null terminator
        self.pos += 1;

        Some(s)
    }
}

/// Well-known DTB addresses (check these first before scanning)
const DTB_KNOWN_ADDRS: &[u64] = &[
    0x48000000,  // Our recommended location (fatload mmc 0:1 0x48000000 bpi-r4.dtb)
    0x47f00000,  // Common U-Boot location
];

/// Scan range for DTB (DRAM region)
const DTB_SCAN_START: u64 = 0x40000000;
const DTB_SCAN_END: u64 = 0x60000000;
const DTB_SCAN_STEP: u64 = 0x1000;  // 4KB alignment (DTB is usually page-aligned)

/// Try to find DTB by scanning memory for magic
fn find_dtb() -> Option<u64> {
    // First check well-known addresses
    for &addr in DTB_KNOWN_ADDRS {
        let virt = addr | 0xFFFF_0000_0000_0000;
        let ptr = virt as *const u32;
        let magic = unsafe { u32::from_be(*ptr) };
        if magic == FDT_MAGIC {
            let header = unsafe { &*(virt as *const FdtHeader) };
            let totalsize = be32(header.totalsize);
            if totalsize > 0x100 && totalsize < 0x100000 {
                // Found at known location, parsed log will follow
                return Some(addr);
            }
        }
    }

    // Full scan
    let mut addr = DTB_SCAN_START;
    let mut found_count = 0u32;
    let mut first_found: Option<u64> = None;

    kdebug!("fdt", "scan_start"; from = crate::klog::hex64(DTB_SCAN_START), to = crate::klog::hex64(DTB_SCAN_END));

    while addr < DTB_SCAN_END {
        // Skip our kernel region (0x46000000-0x47000000)
        if addr >= 0x46000000 && addr < 0x47000000 {
            addr = 0x47000000;
            continue;
        }

        let virt = addr | 0xFFFF_0000_0000_0000;
        let ptr = virt as *const u32;
        let magic = unsafe { u32::from_be(*ptr) };

        if magic == FDT_MAGIC {
            // Verify it looks like a valid DTB (check size is reasonable)
            let header = unsafe { &*(virt as *const FdtHeader) };
            let totalsize = be32(header.totalsize);

            if totalsize > 0x100 && totalsize < 0x100000 {
                kdebug!("fdt", "found_scan"; addr = crate::klog::hex64(addr), size = totalsize);
                found_count += 1;
                if first_found.is_none() {
                    first_found = Some(addr);
                }
            }
        }

        addr += DTB_SCAN_STEP;
    }

    kdebug!("fdt", "scan_complete"; count = found_count);
    first_found
}

/// Global FDT instance (stored after parsing for later use)
static mut GLOBAL_FDT: Option<Fdt> = None;

/// Get the global FDT instance if available
pub fn get() -> Option<&'static Fdt> {
    // Use addr_of! to avoid creating shared reference to mutable static (Rust 2024)
    unsafe { (*core::ptr::addr_of!(GLOBAL_FDT)).as_ref() }
}

/// Store the global FDT instance
fn store_fdt(fdt: Fdt) {
    unsafe {
        GLOBAL_FDT = Some(fdt);
    }
}

/// Initialize the device tree and select platform
pub fn init() {
    // First check for embedded DTB (compiled into kernel)
    if let Some(dtb_data) = crate::dtb::get_embedded_dtb() {
        // Using embedded DTB
        if let Some(fdt) = Fdt::parse_from_ptr(dtb_data.as_ptr()) {
            store_fdt(fdt);
            select_platform();
            return;
        }
    }

    // Try address from U-Boot (only valid with booti, not go)
    let boot_dtb = get_dtb_address();
    kdebug!("fdt", "boot_addr"; addr = crate::klog::hex64(boot_dtb));
    if let Some(fdt) = Fdt::parse(boot_dtb) {
        store_fdt(fdt);
        select_platform();
        return;
    }

    // Fallback: scan known locations for DTB magic
    kdebug!("fdt", "scanning");
    if let Some(addr) = find_dtb() {
        if let Some(fdt) = Fdt::parse(addr) {
            store_fdt(fdt);
            select_platform();
            return;
        }
    }

    kwarn!("fdt", "not_found");
    // No FDT found - use compile-time default platform
    crate::kernel::bus::use_default_platform();
}

/// Select platform based on FDT compatible string
fn select_platform() {
    if let Some(fdt) = get() {
        crate::kernel::bus::select_platform_from_fdt(fdt);
    } else {
        crate::kernel::bus::use_default_platform();
    }
}
