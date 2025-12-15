//! USB userspace driver for BPI-R4 kernel
//!
//! Full USB host controller driver running entirely in userspace:
//! - MMIO scheme for device register access
//! - IRQ scheme for interrupt handling
//! - Complete IPPC, PHY, and xHCI initialization

#![no_std]
#![no_main]

use userlib::{println, print, syscall};

// =============================================================================
// MT7988A SSUSB1 hardware addresses
// =============================================================================

// Physical addresses (for MMIO scheme)
const SSUSB1_MAC_PHYS: u64 = 0x1120_0000;
const SSUSB1_MAC_SIZE: u64 = 0x4000;      // Includes IPPC at offset 0x3e00
const SSUSB1_IPPC_OFFSET: usize = 0x3e00; // IPPC is within MAC region
const SSUSB1_U2PHY_PHYS: u64 = 0x11c5_0000;
const SSUSB1_U2PHY_SIZE: u64 = 0x1000;    // Must be page-aligned (actual regs ~0x800)

const SSUSB1_IRQ: u32 = 172;

// =============================================================================
// IPPC (IP Port Control) registers
// =============================================================================

mod ippc {
    pub const IP_PW_CTRL0: usize = 0x00;
    pub const IP_PW_CTRL1: usize = 0x04;
    pub const IP_PW_CTRL2: usize = 0x08;
    pub const IP_PW_STS1: usize = 0x10;
    pub const IP_XHCI_CAP: usize = 0x24;
    pub const U3_CTRL_P0: usize = 0x30;
    pub const U2_CTRL_P0: usize = 0x50;

    // Control bits
    pub const IP_SW_RST: u32 = 1 << 0;
    pub const IP_HOST_PDN: u32 = 1 << 0;
    pub const IP_DEV_PDN: u32 = 1 << 0;

    // Status bits
    pub const SYSPLL_STABLE: u32 = 1 << 0;
    pub const REF_RST: u32 = 1 << 8;
    pub const SYS125_RST: u32 = 1 << 10;
    pub const XHCI_RST: u32 = 1 << 11;

    // Port control bits
    pub const PORT_HOST_SEL: u32 = 1 << 2;
    pub const PORT_PDN: u32 = 1 << 1;
    pub const PORT_DIS: u32 = 1 << 0;
}

// =============================================================================
// T-PHY USB2 registers (with COM bank offset)
// =============================================================================

mod tphy {
    // T-PHY v2 COM bank is at offset 0x300 from port base
    const COM_OFFSET: usize = 0x300;

    pub const U2PHYDTM1: usize = COM_OFFSET + 0x06C;

    // Host mode bits for DTM1
    pub const FORCE_VBUSVALID: u32 = 1 << 5;
    pub const FORCE_AVALID: u32 = 1 << 4;
    pub const RG_VBUSVALID: u32 = 1 << 1;
    pub const RG_AVALID: u32 = 1 << 0;
    pub const HOST_MODE: u32 = FORCE_VBUSVALID | FORCE_AVALID | RG_VBUSVALID | RG_AVALID;
}

// =============================================================================
// xHCI registers
// =============================================================================

mod xhci_cap {
    pub const CAPLENGTH: usize = 0x00;
    pub const HCSPARAMS1: usize = 0x04;
    pub const HCCPARAMS1: usize = 0x10;
}

mod xhci_op {
    pub const USBCMD: usize = 0x00;
    pub const USBSTS: usize = 0x04;
    pub const PAGESIZE: usize = 0x08;
}

mod usbcmd {
    pub const RUN: u32 = 1 << 0;
    pub const HCRST: u32 = 1 << 1;
}

mod usbsts {
    pub const HCH: u32 = 1 << 0;
    pub const CNR: u32 = 1 << 11;
}

mod xhci_port {
    pub const PORTSC: usize = 0x00;
}

// =============================================================================
// MMIO helpers
// =============================================================================

struct MmioRegion {
    fd: i32,
    base: u64,
}

impl MmioRegion {
    fn open(phys_addr: u64, size: u64) -> Option<Self> {
        // Format URL as hex without 0x prefix
        let mut url_buf = [0u8; 32];
        let url_len = format_mmio_url(&mut url_buf, phys_addr, size);
        let url = unsafe { core::str::from_utf8_unchecked(&url_buf[..url_len]) };

        let fd = syscall::scheme_open(url, 0);
        if fd < 0 {
            return None;
        }

        // Read virtual address
        let mut virt_buf = [0u8; 8];
        if syscall::read(fd as u32, &mut virt_buf) < 8 {
            syscall::close(fd as u32);
            return None;
        }

        let base = u64::from_le_bytes(virt_buf);
        Some(Self { fd, base })
    }

    #[inline(always)]
    fn read32(&self, offset: usize) -> u32 {
        unsafe {
            let ptr = (self.base + offset as u64) as *const u32;
            core::ptr::read_volatile(ptr)
        }
    }

    #[inline(always)]
    fn write32(&self, offset: usize, value: u32) {
        unsafe {
            let ptr = (self.base + offset as u64) as *mut u32;
            core::ptr::write_volatile(ptr, value);
        }
    }
}

impl Drop for MmioRegion {
    fn drop(&mut self) {
        syscall::close(self.fd as u32);
    }
}

fn format_mmio_url(buf: &mut [u8], phys: u64, size: u64) -> usize {
    // Format: "mmio:ADDR/SIZE" in hex
    let prefix = b"mmio:";
    buf[..5].copy_from_slice(prefix);

    let mut pos = 5;
    pos += format_hex(&mut buf[pos..], phys);
    buf[pos] = b'/';
    pos += 1;
    pos += format_hex(&mut buf[pos..], size);
    pos
}

fn format_hex(buf: &mut [u8], val: u64) -> usize {
    let hex = b"0123456789abcdef";
    let mut v = val;
    let mut len = 0;

    // Find number of digits
    let mut temp = val;
    loop {
        len += 1;
        temp >>= 4;
        if temp == 0 {
            break;
        }
    }

    // Write digits in reverse
    for i in (0..len).rev() {
        buf[i] = hex[(v & 0xF) as usize];
        v >>= 4;
    }
    len
}

fn delay(iterations: u32) {
    for _ in 0..iterations {
        unsafe { core::arch::asm!("nop") };
    }
}

fn print_hex32(val: u32) {
    let hex = b"0123456789abcdef";
    let mut buf = [b'0'; 8];
    let mut n = val;
    for i in (0..8).rev() {
        buf[i] = hex[(n & 0xF) as usize];
        n >>= 4;
    }
    let _ = syscall::write(syscall::STDOUT, &buf);
}

// =============================================================================
// USB Driver
// =============================================================================

struct UsbDriver {
    mac: MmioRegion,  // Also contains IPPC at offset 0x3e00
    phy: MmioRegion,
    caplength: u32,
    num_ports: u32,
}

impl UsbDriver {
    fn new() -> Option<Self> {
        println!("  Mapping MMIO regions...");

        let mac = MmioRegion::open(SSUSB1_MAC_PHYS, SSUSB1_MAC_SIZE)?;
        print!("    MAC+IPPC @ 0x");
        print_hex32(SSUSB1_MAC_PHYS as u32);
        println!(" -> OK (IPPC at +0x3e00)");

        let phy = MmioRegion::open(SSUSB1_U2PHY_PHYS, SSUSB1_U2PHY_SIZE)?;
        print!("    PHY @ 0x");
        print_hex32(SSUSB1_U2PHY_PHYS as u32);
        println!(" -> OK");

        Some(Self {
            mac,
            phy,
            caplength: 0,
            num_ports: 0,
        })
    }

    /// Read from IPPC registers (offset from MAC base)
    #[inline(always)]
    fn ippc_read32(&self, offset: usize) -> u32 {
        self.mac.read32(SSUSB1_IPPC_OFFSET + offset)
    }

    /// Write to IPPC registers (offset from MAC base)
    #[inline(always)]
    fn ippc_write32(&self, offset: usize, value: u32) {
        self.mac.write32(SSUSB1_IPPC_OFFSET + offset, value)
    }

    fn init(&mut self) -> bool {
        println!();
        println!("=== IPPC Initialization ===");

        // Software reset
        println!("  Software reset...");
        self.ippc_write32(ippc::IP_PW_CTRL0, ippc::IP_SW_RST);
        delay(1000);
        self.ippc_write32(ippc::IP_PW_CTRL0, 0);
        delay(1000);

        // Power on host, power down device
        println!("  Host power on...");
        self.ippc_write32(ippc::IP_PW_CTRL2, ippc::IP_DEV_PDN);
        self.ippc_write32(ippc::IP_PW_CTRL1, 0); // Clear host PDN

        // Read port counts
        let cap = self.ippc_read32(ippc::IP_XHCI_CAP);
        let u3_ports = (cap >> 8) & 0xF;
        let u2_ports = cap & 0xF;
        println!("  Ports: {} USB3, {} USB2", u3_ports, u2_ports);

        // Configure U3 ports (if any)
        for i in 0..u3_ports {
            let offset = ippc::U3_CTRL_P0 + (i as usize * 8);
            let mut ctrl = self.ippc_read32(offset);
            ctrl &= !(ippc::PORT_PDN | ippc::PORT_DIS);
            ctrl |= ippc::PORT_HOST_SEL;
            self.ippc_write32(offset, ctrl);
        }

        // Configure U2 ports
        for i in 0..u2_ports {
            let offset = ippc::U2_CTRL_P0 + (i as usize * 8);
            let mut ctrl = self.ippc_read32(offset);
            ctrl &= !(ippc::PORT_PDN | ippc::PORT_DIS);
            ctrl |= ippc::PORT_HOST_SEL;
            self.ippc_write32(offset, ctrl);
        }

        // Wait for clocks
        println!("  Waiting for clocks...");
        let required = ippc::SYSPLL_STABLE | ippc::REF_RST | ippc::SYS125_RST | ippc::XHCI_RST;
        for _ in 0..1000 {
            let sts = self.ippc_read32(ippc::IP_PW_STS1);
            if (sts & required) == required {
                println!("  Clocks stable");
                break;
            }
            delay(1000);
        }

        println!();
        println!("=== xHCI Reset ===");

        // Read capability length
        self.caplength = self.mac.read32(xhci_cap::CAPLENGTH) & 0xFF;
        let version = (self.mac.read32(xhci_cap::CAPLENGTH) >> 16) & 0xFFFF;
        println!("  xHCI version: {}.{}", version >> 8, version & 0xFF);

        let hcsparams1 = self.mac.read32(xhci_cap::HCSPARAMS1);
        self.num_ports = (hcsparams1 >> 24) & 0xFF;
        println!("  Max ports: {}", self.num_ports);

        // Halt controller
        println!("  Halting controller...");
        let op_base = self.caplength as usize;
        let cmd = self.mac.read32(op_base + xhci_op::USBCMD);
        self.mac.write32(op_base + xhci_op::USBCMD, cmd & !usbcmd::RUN);

        // Wait for halt
        for _ in 0..100 {
            let sts = self.mac.read32(op_base + xhci_op::USBSTS);
            if (sts & usbsts::HCH) != 0 {
                break;
            }
            delay(1000);
        }

        // Reset
        println!("  Resetting...");
        self.mac.write32(op_base + xhci_op::USBCMD, usbcmd::HCRST);

        // Wait for reset complete
        for _ in 0..100 {
            let cmd = self.mac.read32(op_base + xhci_op::USBCMD);
            let sts = self.mac.read32(op_base + xhci_op::USBSTS);
            if (cmd & usbcmd::HCRST) == 0 && (sts & usbsts::CNR) == 0 {
                println!("  Reset complete");
                break;
            }
            delay(1000);
        }

        println!();
        println!("=== PHY Initialization ===");

        // Initialize T-PHY for host mode
        println!("  Setting T-PHY host mode...");
        self.phy.write32(tphy::U2PHYDTM1, tphy::HOST_MODE);
        let readback = self.phy.read32(tphy::U2PHYDTM1);
        print!("    DTM1 = 0x");
        print_hex32(readback);
        if readback == tphy::HOST_MODE {
            println!(" (OK)");
        } else {
            println!(" (MISMATCH!)");
        }

        println!();
        println!("=== Starting Controller ===");

        // Start controller
        let cmd = self.mac.read32(op_base + xhci_op::USBCMD);
        self.mac.write32(op_base + xhci_op::USBCMD, cmd | usbcmd::RUN);

        // Wait for running
        delay(10000);
        let sts = self.mac.read32(op_base + xhci_op::USBSTS);
        if (sts & usbsts::HCH) == 0 {
            println!("  Controller running");
        } else {
            println!("  ERROR: Controller not running!");
            return false;
        }

        // Power on ports
        println!("  Powering ports...");
        let port_base = self.caplength as usize + 0x400;
        for p in 0..self.num_ports {
            let portsc_off = port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let portsc = self.mac.read32(portsc_off);
            // Set PP (Port Power) bit 9
            self.mac.write32(portsc_off, portsc | (1 << 9));
        }

        // Wait for link stabilization
        println!("  Waiting for link...");
        delay(100000);

        true
    }

    fn print_port_status(&self) {
        println!();
        println!("=== Port Status ===");

        let port_base = self.caplength as usize + 0x400;
        for p in 0..self.num_ports {
            let portsc_off = port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let portsc = self.mac.read32(portsc_off);

            let ccs = (portsc >> 0) & 1;
            let ped = (portsc >> 1) & 1;
            let pp = (portsc >> 9) & 1;
            let speed = (portsc >> 10) & 0xF;
            let pls = (portsc >> 5) & 0xF;

            print!("  Port {}: ", p + 1);
            if pp == 0 {
                println!("Not powered");
                continue;
            }

            if ccs == 0 {
                println!("No device");
                continue;
            }

            let speed_str = match speed {
                1 => "Full (12 Mbps)",
                2 => "Low (1.5 Mbps)",
                3 => "High (480 Mbps)",
                4 => "Super (5 Gbps)",
                5 => "Super+ (10 Gbps)",
                _ => "Unknown",
            };

            let pls_str = match pls {
                0 => "U0 (active)",
                1 => "U1",
                2 => "U2",
                3 => "U3 (suspended)",
                5 => "RxDetect",
                7 => "Polling",
                _ => "Other",
            };

            print!("{}", speed_str);
            if ped == 1 {
                print!(" [Enabled]");
            }
            println!(" PLS={}", pls_str);
        }
    }
}

// =============================================================================
// Main
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    println!("========================================");
    println!("  USB Userspace Driver");
    println!("  MT7988A SSUSB1 (Full Init)");
    println!("========================================");
    println!();

    // Create and initialize driver
    let mut driver = match UsbDriver::new() {
        Some(d) => d,
        None => {
            println!("ERROR: Failed to map MMIO regions");
            syscall::exit(1);
        }
    };

    if !driver.init() {
        println!("ERROR: USB initialization failed");
        syscall::exit(1);
    }

    driver.print_port_status();

    // Register for IRQ
    println!();
    println!("=== IRQ Registration ===");
    let irq_url = "irq:172";
    let irq_fd = syscall::scheme_open(irq_url, 0);
    if irq_fd >= 0 {
        println!("  IRQ {} registered (fd={})", SSUSB1_IRQ, irq_fd);
        syscall::close(irq_fd as u32);
    } else {
        println!("  IRQ registration failed: {}", irq_fd);
    }

    println!();
    println!("========================================");
    println!("  USB initialization complete!");
    println!("========================================");

    syscall::exit(0);
}
