//! USB userspace driver test for BPI-R4 kernel
//!
//! Demonstrates microkernel userspace driver model:
//! - MMIO scheme for device register access
//! - IRQ scheme for interrupt handling
//! - Event system for async notification

#![no_std]
#![no_main]

use userlib::{println, print, syscall};

// MT7988A SSUSB1 controller addresses
#[allow(dead_code)]
const SSUSB1_MAC_BASE: u64 = 0x1120_0000;
#[allow(dead_code)]
const SSUSB1_MAC_SIZE: u64 = 0x4000; // 16KB for xHCI registers
const SSUSB1_IRQ: u32 = 172; // GIC SPI 172

// xHCI capability register offsets
#[allow(dead_code)]
mod xhci_cap {
    pub const CAPLENGTH: usize = 0x00;
    pub const HCIVERSION: usize = 0x02;
    pub const HCSPARAMS1: usize = 0x04;
    pub const HCSPARAMS2: usize = 0x08;
    pub const HCSPARAMS3: usize = 0x0C;
    pub const HCCPARAMS1: usize = 0x10;
    pub const DBOFF: usize = 0x14;
    pub const RTSOFF: usize = 0x18;
}

// xHCI operational register offsets (from op_base = cap_base + CAPLENGTH)
#[allow(dead_code)]
mod xhci_op {
    pub const USBCMD: usize = 0x00;
    pub const USBSTS: usize = 0x04;
    pub const PAGESIZE: usize = 0x08;
    pub const DNCTRL: usize = 0x14;
    pub const CRCR: usize = 0x18;
    pub const DCBAAP: usize = 0x30;
    pub const CONFIG: usize = 0x38;
}

// Port status register offsets (from port_base = op_base + 0x400)
#[allow(dead_code)]
mod xhci_port {
    pub const PORTSC: usize = 0x00;
    pub const PORTPMSC: usize = 0x04;
    pub const PORTLI: usize = 0x08;
}

/// Read a 32-bit register from mapped MMIO region
#[inline(always)]
unsafe fn mmio_read32(base: u64, offset: usize) -> u32 {
    let ptr = (base + offset as u64) as *const u32;
    unsafe { core::ptr::read_volatile(ptr) }
}

/// Print a 32-bit hex value
fn print_hex32(val: u32) {
    let hex_chars = b"0123456789abcdef";
    let mut buf = [b'0'; 8];
    let mut n = val;
    for i in (0..8).rev() {
        buf[i] = hex_chars[(n & 0xF) as usize];
        n >>= 4;
    }
    let _ = syscall::write(syscall::STDOUT, &buf);
}

/// Print a 64-bit hex value
fn print_hex64(val: u64) {
    let hex_chars = b"0123456789abcdef";
    let mut buf = [b'0'; 16];
    let mut n = val;
    for i in (0..16).rev() {
        buf[i] = hex_chars[(n & 0xF) as usize];
        n >>= 4;
    }
    let _ = syscall::write(syscall::STDOUT, &buf);
}

#[unsafe(no_mangle)]
fn main() {
    println!("========================================");
    println!("  USB Userspace Driver Test");
    println!("  MT7988A SSUSB1 Controller");
    println!("========================================");
    println!();

    // Step 1: Map USB controller registers via MMIO scheme
    println!("Step 1: Opening MMIO scheme for USB registers...");

    // Format: mmio:phys_addr/size (hex)
    let mmio_url = "mmio:11200000/4000";
    let mmio_fd = syscall::scheme_open(mmio_url, 0);

    if mmio_fd < 0 {
        println!("  ERROR: Failed to open MMIO scheme: {}", mmio_fd);
        println!("  (This requires kernel scheme support)");
        syscall::exit(1);
    }

    println!("  MMIO scheme opened, fd = {}", mmio_fd);

    // Read the virtual address from the scheme
    let mut virt_buf = [0u8; 8];
    let read_result = syscall::read(mmio_fd as u32, &mut virt_buf);

    if read_result < 8 {
        println!("  ERROR: Failed to read virtual address: {}", read_result);
        syscall::exit(1);
    }

    let usb_base = u64::from_le_bytes(virt_buf);
    print!("  USB registers mapped at VA: 0x");
    print_hex64(usb_base);
    println!();
    println!();

    // Step 2: Read xHCI capability registers
    println!("Step 2: Reading xHCI capability registers...");

    unsafe {
        let caplength = mmio_read32(usb_base, xhci_cap::CAPLENGTH) & 0xFF;
        let hciversion = (mmio_read32(usb_base, xhci_cap::CAPLENGTH) >> 16) & 0xFFFF;
        let hcsparams1 = mmio_read32(usb_base, xhci_cap::HCSPARAMS1);
        let hcsparams2 = mmio_read32(usb_base, xhci_cap::HCSPARAMS2);
        let hccparams1 = mmio_read32(usb_base, xhci_cap::HCCPARAMS1);

        print!("  CAPLENGTH:  0x");
        print_hex32(caplength);
        println!();

        print!("  HCIVERSION: 0x");
        print_hex32(hciversion);
        if hciversion == 0x0110 {
            println!(" (xHCI 1.16)");
        } else {
            println!();
        }

        print!("  HCSPARAMS1: 0x");
        print_hex32(hcsparams1);
        let max_slots = hcsparams1 & 0xFF;
        let max_intrs = (hcsparams1 >> 8) & 0x7FF;
        let max_ports = (hcsparams1 >> 24) & 0xFF;
        println!();
        println!("    Max Slots: {}", max_slots);
        println!("    Max Intrs: {}", max_intrs);
        println!("    Max Ports: {}", max_ports);

        print!("  HCSPARAMS2: 0x");
        print_hex32(hcsparams2);
        println!();

        print!("  HCCPARAMS1: 0x");
        print_hex32(hccparams1);
        let ac64 = (hccparams1 >> 0) & 1;
        let csz = (hccparams1 >> 2) & 1;
        println!();
        println!("    64-bit capable: {}", if ac64 == 1 { "yes" } else { "no" });
        println!("    Context size: {} bytes", if csz == 1 { 64 } else { 32 });

        // Read operational registers
        let op_base = usb_base + caplength as u64;
        println!();
        println!("Step 3: Reading xHCI operational registers...");
        print!("  Op base: 0x");
        print_hex64(op_base);
        println!();

        let usbcmd = mmio_read32(op_base, xhci_op::USBCMD);
        let usbsts = mmio_read32(op_base, xhci_op::USBSTS);
        let pagesize = mmio_read32(op_base, xhci_op::PAGESIZE);

        print!("  USBCMD:   0x");
        print_hex32(usbcmd);
        let run = (usbcmd >> 0) & 1;
        let hcrst = (usbcmd >> 1) & 1;
        let inte = (usbcmd >> 2) & 1;
        println!();
        println!("    Run/Stop: {}", if run == 1 { "Running" } else { "Stopped" });
        println!("    HCRST: {}", if hcrst == 1 { "Reset" } else { "Normal" });
        println!("    INTE: {}", if inte == 1 { "Enabled" } else { "Disabled" });

        print!("  USBSTS:   0x");
        print_hex32(usbsts);
        let hch = (usbsts >> 0) & 1;
        let hse = (usbsts >> 2) & 1;
        let eint = (usbsts >> 3) & 1;
        let pcd = (usbsts >> 4) & 1;
        let cnr = (usbsts >> 11) & 1;
        println!();
        println!("    HCHalted: {}", if hch == 1 { "yes" } else { "no" });
        println!("    HSE: {}", if hse == 1 { "error" } else { "ok" });
        println!("    EINT: {}", if eint == 1 { "pending" } else { "none" });
        println!("    PCD: {}", if pcd == 1 { "changed" } else { "stable" });
        println!("    CNR: {}", if cnr == 1 { "not ready" } else { "ready" });

        print!("  PAGESIZE: 0x");
        print_hex32(pagesize);
        println!(" ({} bytes)", (pagesize & 0xFFFF) << 12);

        // Read port status
        let port_base = op_base + 0x400;
        println!();
        println!("Step 4: Reading USB port status...");

        for port in 0..max_ports {
            let port_offset = port as usize * 0x10;
            let portsc = mmio_read32(port_base, port_offset + xhci_port::PORTSC);

            let ccs = (portsc >> 0) & 1;
            let _ped = (portsc >> 1) & 1;
            let _pr = (portsc >> 4) & 1;
            let _pls = (portsc >> 5) & 0xF;
            let _pp = (portsc >> 9) & 1;
            let speed = (portsc >> 10) & 0xF;

            print!("  Port {}: PORTSC=0x", port + 1);
            print_hex32(portsc);

            if ccs == 1 {
                let speed_str = match speed {
                    1 => "Full-Speed",
                    2 => "Low-Speed",
                    3 => "High-Speed",
                    4 => "Super-Speed",
                    5 => "Super-Speed+",
                    _ => "Unknown",
                };
                println!(" [Connected, {}]", speed_str);
            } else {
                println!(" [Not connected]");
            }
        }
    }

    // Step 5: Register for USB IRQ (optional - may not be needed for polling)
    println!();
    println!("Step 5: Registering for USB IRQ {}...", SSUSB1_IRQ);

    // Format: irq:num
    let irq_url = "irq:172";
    let irq_fd = syscall::scheme_open(irq_url, 0);

    if irq_fd < 0 {
        println!("  WARNING: Failed to register IRQ: {}", irq_fd);
        println!("  (Continuing without interrupt support)");
    } else {
        println!("  IRQ registered, fd = {}", irq_fd);

        // Check for pending IRQs (non-blocking)
        let mut irq_count_buf = [0u8; 4];
        let irq_result = syscall::read(irq_fd as u32, &mut irq_count_buf);

        if irq_result == 4 {
            let count = u32::from_le_bytes(irq_count_buf);
            println!("  Pending IRQ count: {}", count);
        } else if irq_result == -11 {
            println!("  No pending IRQs (EAGAIN)");
        } else {
            println!("  IRQ read returned: {}", irq_result);
        }

        // Close IRQ handle
        syscall::close(irq_fd as u32);
    }

    // Cleanup
    println!();
    println!("Step 6: Cleanup...");
    syscall::close(mmio_fd as u32);
    println!("  MMIO mapping closed");

    println!();
    println!("========================================");
    println!("  USB userspace driver test complete!");
    println!("========================================");
}
