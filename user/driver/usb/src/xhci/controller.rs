//! Pure xHCI Controller Implementation
//!
//! This module contains ONLY standard xHCI operations. NO vendor-specific code.
//!
//! ## Responsibilities:
//! - Reading capability registers
//! - Halt/reset sequence
//! - Programming operational registers (DCBAAP, CRCR, CONFIG)
//! - Programming interrupters (ERST, ERDP)
//! - Starting/stopping the controller
//! - Port status reading and manipulation
//! - Doorbell ringing
//!
//! ## What does NOT belong here:
//! - IPPC registers (MediaTek)
//! - PHY initialization (all vendors)
//! - Clock/reset control (SoC-specific)
//! - Any `if mediatek` or `if synopsys` branching

use crate::mmio::MmioRegion;
use crate::transfer::dsb;
use super::regs::*;
use userlib::{print, println};

/// xHCI capability information read from hardware
#[derive(Debug, Clone)]
pub struct XhciCaps {
    pub version: u16,
    pub caplength: u8,
    pub max_slots: u8,
    pub max_intrs: u16,
    pub max_ports: u8,
    pub max_scratchpad: u16,
    pub op_offset: usize,
    pub rt_offset: usize,
    pub db_offset: usize,
}

/// Pure xHCI Controller
///
/// This struct contains ONLY standard xHCI functionality.
/// NO vendor-specific code belongs here.
///
/// If you need to add MediaTek/Synopsys/etc. specific code,
/// put it in the `soc` or `phy` modules instead.
pub struct Controller {
    /// MMIO region for xHCI registers
    mmio: MmioRegion,

    /// Cached offsets (read from capability registers)
    caplength: usize,
    op_offset: usize,
    rt_offset: usize,
    db_offset: usize,
    port_offset: usize,

    /// Controller capabilities
    max_slots: u8,
    max_ports: u8,
}

impl Controller {
    /// Create a new xHCI controller from an MMIO region
    ///
    /// The MMIO region must be mapped to the xHCI register base.
    /// The caller (SoC wrapper) is responsible for providing the correct address.
    pub fn new(mmio: MmioRegion) -> Self {
        Self {
            mmio,
            caplength: 0,
            op_offset: 0,
            rt_offset: 0,
            db_offset: 0,
            port_offset: 0,
            max_slots: 0,
            max_ports: 0,
        }
    }

    // =========================================================================
    // Register Access (all offsets relative to base)
    // =========================================================================

    /// Read capability register
    #[inline(always)]
    pub fn cap_read32(&self, offset: usize) -> u32 {
        self.mmio.read32(offset)
    }

    /// Read operational register
    #[inline(always)]
    pub fn op_read32(&self, offset: usize) -> u32 {
        self.mmio.read32(self.op_offset + offset)
    }

    /// Write operational register
    #[inline(always)]
    pub fn op_write32(&self, offset: usize, value: u32) {
        self.mmio.write32(self.op_offset + offset, value);
    }

    /// Write operational register (64-bit)
    #[inline(always)]
    pub fn op_write64(&self, offset: usize, value: u64) {
        self.mmio.write64(self.op_offset + offset, value);
    }

    /// Read runtime register
    #[inline(always)]
    pub fn rt_read32(&self, offset: usize) -> u32 {
        self.mmio.read32(self.rt_offset + offset)
    }

    /// Read runtime register (64-bit)
    #[inline(always)]
    pub fn rt_read64(&self, offset: usize) -> u64 {
        self.mmio.read64(self.rt_offset + offset)
    }

    /// Write runtime register
    #[inline(always)]
    pub fn rt_write32(&self, offset: usize, value: u32) {
        self.mmio.write32(self.rt_offset + offset, value);
    }

    /// Write runtime register (64-bit)
    #[inline(always)]
    pub fn rt_write64(&self, offset: usize, value: u64) {
        self.mmio.write64(self.rt_offset + offset, value);
    }

    /// Ring a doorbell
    #[inline(always)]
    pub fn ring_doorbell(&self, slot: u8, target: u8) {
        dsb();
        let db_off = self.db_offset + (slot as usize * 4);
        self.mmio.write32(db_off, target as u32);
        dsb();
    }

    /// Read port status register
    #[inline(always)]
    pub fn port_read32(&self, port: u8, offset: usize) -> u32 {
        let port_off = self.port_offset + (port as usize * 0x10) + offset;
        self.mmio.read32(port_off)
    }

    /// Write port status register
    #[inline(always)]
    pub fn port_write32(&self, port: u8, offset: usize, value: u32) {
        let port_off = self.port_offset + (port as usize * 0x10) + offset;
        self.mmio.write32(port_off, value);
    }

    // =========================================================================
    // Capability Reading
    // =========================================================================

    /// Read xHCI capability registers
    ///
    /// Must be called before any other xHCI operations to determine offsets.
    pub fn read_capabilities(&mut self) -> Option<XhciCaps> {
        // Read capability length and version
        let cap_reg = self.cap_read32(cap::CAPLENGTH);
        self.caplength = (cap_reg & 0xFF) as usize;
        let version = ((cap_reg >> 16) & 0xFFFF) as u16;

        // Read structural parameters
        let hcsparams1 = self.cap_read32(cap::HCSPARAMS1);
        self.max_slots = (hcsparams1 & cap::HCSPARAMS1_MAX_SLOTS_MASK) as u8;
        let max_intrs = ((hcsparams1 & cap::HCSPARAMS1_MAX_INTRS_MASK) >> cap::HCSPARAMS1_MAX_INTRS_SHIFT) as u16;
        self.max_ports = ((hcsparams1 & cap::HCSPARAMS1_MAX_PORTS_MASK) >> cap::HCSPARAMS1_MAX_PORTS_SHIFT) as u8;

        let hcsparams2 = self.cap_read32(cap::HCSPARAMS2);
        let max_scratchpad = (((hcsparams2 >> 27) & 0x1F) | ((hcsparams2 >> 16) & 0x3E0)) as u16;

        // Calculate register offsets
        self.op_offset = self.caplength;
        self.rt_offset = (self.cap_read32(cap::RTSOFF) & !0x1F) as usize;
        self.db_offset = (self.cap_read32(cap::DBOFF) & !0x3) as usize;
        self.port_offset = self.op_offset + 0x400;

        Some(XhciCaps {
            version,
            caplength: self.caplength as u8,
            max_slots: self.max_slots,
            max_intrs,
            max_ports: self.max_ports,
            max_scratchpad,
            op_offset: self.op_offset,
            rt_offset: self.rt_offset,
            db_offset: self.db_offset,
        })
    }

    /// Get max slots
    pub fn max_slots(&self) -> u8 {
        self.max_slots
    }

    /// Get max ports
    pub fn max_ports(&self) -> u8 {
        self.max_ports
    }

    /// Get MMIO base address
    pub fn mmio_base(&self) -> u64 {
        self.mmio.virt_base()
    }

    /// Get operational registers offset
    pub fn op_offset(&self) -> usize {
        self.op_offset
    }

    /// Get runtime registers offset
    pub fn rt_offset(&self) -> usize {
        self.rt_offset
    }

    /// Get doorbell offset
    pub fn db_offset(&self) -> usize {
        self.db_offset
    }

    /// Get port registers offset
    pub fn port_offset(&self) -> usize {
        self.port_offset
    }

    // =========================================================================
    // Halt and Reset
    // =========================================================================

    /// Halt the xHCI controller
    ///
    /// Clears the Run/Stop bit and waits for HCH (Halted) bit.
    pub fn halt(&mut self) -> bool {
        let cmd = self.op_read32(op::USBCMD);
        self.op_write32(op::USBCMD, cmd & !usbcmd::RUN);

        // Wait for halt (HCH bit)
        for _ in 0..100 {
            let sts = self.op_read32(op::USBSTS);
            if (sts & usbsts::HCH) != 0 {
                return true;
            }
            crate::mmio::delay_ms(1);
        }
        false
    }

    /// Reset the xHCI controller
    ///
    /// Sets HCRST and waits for it to clear along with CNR.
    ///
    /// Returns true if reset completes, false on timeout.
    /// Note: Some controllers (e.g., MediaTek MT7988A) may not complete
    /// reset until PHY initialization is done. The caller should handle this.
    pub fn reset(&mut self) -> bool {
        self.op_write32(op::USBCMD, usbcmd::HCRST);

        for _ in 0..100 {
            let cmd = self.op_read32(op::USBCMD);
            let sts = self.op_read32(op::USBSTS);
            if (cmd & usbcmd::HCRST) == 0 && (sts & usbsts::CNR) == 0 {
                return true;
            }
            crate::mmio::delay_ms(1);
        }
        false
    }

    /// Combined halt and reset
    ///
    /// Returns (halt_ok, reset_ok) tuple.
    pub fn halt_and_reset(&mut self) -> (bool, bool) {
        let halt_ok = self.halt();
        let reset_ok = self.reset();
        (halt_ok, reset_ok)
    }

    // =========================================================================
    // Programming Operational Registers
    // =========================================================================

    /// Set maximum enabled slots
    pub fn set_max_slots(&mut self, slots: u8) {
        let slots = slots.min(self.max_slots);
        self.op_write32(op::CONFIG, slots as u32);
    }

    /// Set Device Context Base Address Array Pointer
    pub fn set_dcbaap(&mut self, phys: u64) {
        self.op_write64(op::DCBAAP, phys);
    }

    /// Set Command Ring Control Register
    ///
    /// The address must be 64-byte aligned. Bit 0 is the Ring Cycle State.
    pub fn set_crcr(&mut self, phys: u64, cycle: bool) {
        let value = (phys & !0x3F) | if cycle { 1 } else { 0 };
        self.op_write64(op::CRCR, value);
    }

    // =========================================================================
    // Interrupter Programming
    // =========================================================================

    /// Program interrupter for event ring
    ///
    /// # Arguments
    /// * `intr` - Interrupter number (typically 0)
    /// * `erst_phys` - Physical address of ERST
    /// * `erst_size` - Number of segments in ERST
    /// * `erdp_phys` - Initial Event Ring Dequeue Pointer
    pub fn program_interrupter(&mut self, intr: u8, erst_phys: u64, erst_size: u16, erdp_phys: u64) {
        let ir_base = rt::IR0 + (intr as usize * rt::IR_SPACING);

        // Set ERSTSZ
        self.rt_write32(ir_base + ir::ERSTSZ, erst_size as u32);

        // Set ERDP
        self.rt_write64(ir_base + ir::ERDP, erdp_phys);

        // Set ERSTBA (must be after ERSTSZ)
        self.rt_write64(ir_base + ir::ERSTBA, erst_phys);

        // Enable interrupter (IE=1)
        self.rt_write32(ir_base + ir::IMAN, iman::IE);
    }

    /// Update Event Ring Dequeue Pointer
    pub fn update_erdp(&mut self, intr: u8, erdp_phys: u64) {
        let ir_base = rt::IR0 + (intr as usize * rt::IR_SPACING);
        // Set EHB (Event Handler Busy) bit to clear interrupt
        self.rt_write64(ir_base + ir::ERDP, erdp_phys | (1 << 3));
    }

    /// Clear interrupter pending flag
    pub fn clear_interrupt(&mut self, intr: u8) {
        let ir_base = rt::IR0 + (intr as usize * rt::IR_SPACING);
        let iman = self.rt_read32(ir_base + ir::IMAN);
        // Write 1 to IP bit to clear it (RW1C)
        self.rt_write32(ir_base + ir::IMAN, iman | iman::IP);
    }

    // =========================================================================
    // Controller Start/Stop
    // =========================================================================

    /// Start the xHCI controller
    ///
    /// Sets Run/Stop and Interrupt Enable bits.
    pub fn start(&mut self) -> bool {
        let cmd = self.op_read32(op::USBCMD);
        self.op_write32(op::USBCMD, cmd | usbcmd::RUN | usbcmd::INTE);

        crate::mmio::delay_ms(10);

        let sts = self.op_read32(op::USBSTS);
        (sts & usbsts::HCH) == 0  // Running = NOT halted
    }

    /// Stop the xHCI controller
    pub fn stop(&mut self) {
        let cmd = self.op_read32(op::USBCMD);
        self.op_write32(op::USBCMD, cmd & !usbcmd::RUN);
    }

    /// Check if controller is running
    pub fn is_running(&self) -> bool {
        let sts = self.op_read32(op::USBSTS);
        (sts & usbsts::HCH) == 0
    }

    // =========================================================================
    // Port Operations
    // =========================================================================

    /// Read PORTSC for a port (0-indexed)
    pub fn read_portsc(&self, port: u8) -> u32 {
        self.port_read32(port, port::PORTSC)
    }

    /// Write PORTSC for a port
    pub fn write_portsc(&self, port: u8, value: u32) {
        self.port_write32(port, port::PORTSC, value);
    }

    /// Power on a single port
    pub fn power_on_port(&mut self, port: u8) {
        let portsc = self.read_portsc(port);
        self.write_portsc(port, portsc | portsc::PP);
    }

    /// Power on all ports
    pub fn power_on_all_ports(&mut self) {
        for p in 0..self.max_ports {
            self.power_on_port(p);
        }
    }

    /// Reset a port
    ///
    /// Initiates a port reset by setting PR bit.
    /// Caller should wait for PRC (Port Reset Change) event.
    pub fn reset_port(&mut self, port: u8) {
        let portsc = self.read_portsc(port);
        // Preserve PP, clear change bits, set PR
        let value = (portsc & portsc::PRESERVE_BITS & !portsc::RW1C_BITS) | portsc::PR;
        self.write_portsc(port, value);
    }

    /// Clear port status change bits
    pub fn clear_port_changes(&mut self, port: u8) {
        let portsc = self.read_portsc(port);
        // Write 1 to change bits to clear them
        let value = (portsc & portsc::PRESERVE_BITS) | (portsc & portsc::RW1C_BITS);
        self.write_portsc(port, value);
    }

    /// Get port link state
    pub fn port_link_state(&self, port: u8) -> u8 {
        let portsc = self.read_portsc(port);
        ((portsc & portsc::PLS_MASK) >> portsc::PLS_SHIFT) as u8
    }

    /// Get port speed
    pub fn port_speed(&self, port: u8) -> u8 {
        let portsc = self.read_portsc(port);
        ((portsc & portsc::SPEED_MASK) >> portsc::SPEED_SHIFT) as u8
    }

    /// Check if port has a device connected
    pub fn port_connected(&self, port: u8) -> bool {
        let portsc = self.read_portsc(port);
        (portsc & portsc::CCS) != 0
    }

    /// Check if port is enabled
    pub fn port_enabled(&self, port: u8) -> bool {
        let portsc = self.read_portsc(port);
        (portsc & portsc::PED) != 0
    }

    /// Wait for any port to have a device connected
    ///
    /// Returns the first port with a connection, or None if timeout.
    pub fn wait_for_connection(&self, timeout_us: u32) -> Option<u8> {
        let iterations = timeout_us / 1000;
        for _ in 0..iterations {
            for p in 0..self.max_ports {
                if self.port_connected(p) {
                    return Some(p);
                }
            }
            crate::mmio::delay_ms(1);
        }
        None
    }

    /// Direct MMIO read (for cases that need raw access)
    #[inline(always)]
    pub fn mmio_read32(&self, offset: usize) -> u32 {
        self.mmio.read32(offset)
    }

    /// Direct MMIO write (for cases that need raw access)
    #[inline(always)]
    pub fn mmio_write32(&self, offset: usize, value: u32) {
        self.mmio.write32(offset, value);
    }

    // =========================================================================
    // Debug Helpers
    // =========================================================================

    /// Print controller status (for debugging)
    pub fn print_status(&self) {
        let cmd = self.op_read32(op::USBCMD);
        let sts = self.op_read32(op::USBSTS);
        print!("  USBCMD=0x");
        crate::mmio::print_hex32(cmd);
        print!(" USBSTS=0x");
        crate::mmio::print_hex32(sts);
        println!();
    }

    /// Print all port status (for debugging)
    pub fn print_all_ports(&self) {
        for p in 0..self.max_ports {
            let portsc = self.read_portsc(p);
            print!("  Port {}: 0x", p + 1);
            crate::mmio::print_hex32(portsc);
            let ccs = portsc & portsc::CCS;
            let ped = portsc & portsc::PED;
            let pls = (portsc & portsc::PLS_MASK) >> portsc::PLS_SHIFT;
            let speed = (portsc & portsc::SPEED_MASK) >> portsc::SPEED_SHIFT;
            println!(" CCS={} PED={} PLS={} SPD={}",
                     ccs != 0, ped != 0, pls, speed);
        }
    }
}
