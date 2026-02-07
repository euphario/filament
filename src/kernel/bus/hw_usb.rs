//! USB/xHCI Hardware Constants and Operations
//!
//! This module contains USB hardware addresses, IPPC registers,
//! xHCI operations, and power management. Base addresses are stored
//! per-bus in BusController (set by probed via open(Bus) syscall).

use crate::kernel::arch::mmio::{MmioRegion, delay_ms, delay_us, dsb};
use super::hw_poll::poll_until;

/// IPPC (IP Port Control) offset from MAC base
pub const IPPC_OFFSET: usize = 0x3E00;

/// IPPC register offsets (from Linux xhci-mtk.h struct mu3c_ippc_regs)
pub const IP_PW_CTRL0: usize = 0x00;
pub const IP_PW_CTRL1: usize = 0x04;
pub const IP_PW_CTRL2: usize = 0x08;
pub const IP_PW_STS1: usize = 0x10;
pub const IP_XHCI_CAP: usize = 0x24;
pub const U3_CTRL_P0: usize = 0x30;  // Each port is 8 bytes
pub const U2_CTRL_P0: usize = 0x50;  // Each port is 8 bytes

/// CTRL0 bits
pub const CTRL0_IP_SW_RST: u32 = 1 << 0;

/// CTRL1 bits
pub const CTRL1_IP_HOST_PDN: u32 = 1 << 0;

/// CTRL2 bits
pub const CTRL2_IP_DEV_PDN: u32 = 1 << 0;

/// STS1 bits
pub const STS1_SYSPLL_STABLE: u32 = 1 << 0;
pub const STS1_REF_RST: u32 = 1 << 8;
pub const STS1_SYS125_RST: u32 = 1 << 10;
pub const STS1_XHCI_RST: u32 = 1 << 11;
pub const STS1_U3_MAC_RST: u32 = 1 << 16;
pub const STS1_IP_SLEEP_STS: u32 = 1 << 30;

/// Port control bits (for U3_CTRL_Px and U2_CTRL_Px)
pub const PORT_DIS: u32 = 1 << 0;
pub const PORT_PDN: u32 = 1 << 1;
pub const PORT_HOST_SEL: u32 = 1 << 2;

/// xHCI capability registers
pub const CAPLENGTH: usize = 0x00;

/// xHCI operational registers (offset from CAPLENGTH)
pub const USBCMD: usize = 0x00;
pub const USBSTS: usize = 0x04;

/// USBCMD bits
pub const CMD_RUN: u32 = 1 << 0;
pub const CMD_HCRST: u32 = 1 << 1;

/// USBSTS bits
pub const STS_HCH: u32 = 1 << 0;    // HC Halted
pub const STS_CNR: u32 = 1 << 11;   // Controller Not Ready
pub const STS_HCE: u32 = 1 << 12;   // Host Controller Error

// ─────────────────────────────────────────────────────────────────────────
// Hardware Operations
// ─────────────────────────────────────────────────────────────────────────

/// USB/xHCI hardware reset sequence
///
/// Based on Linux xhci-mtk.c driver power-on sequence:
/// 1. IPPC: Assert IP software reset, wait 1us, deassert
/// 2. IPPC: Power down device IP (set CTRL2_IP_DEV_PDN)
/// 3. IPPC: Power on host IP (clear CTRL1_IP_HOST_PDN)
/// 4. IPPC: Enable U3 ports (clear PDN/DIS, set HOST_SEL)
/// 5. IPPC: Enable U2 ports (clear PDN/DIS, set HOST_SEL)
/// 6. Wait for clock stability
/// 7. xHCI: Wait for controller ready (CNR=0)
/// 8. xHCI: Verify halted state
///
/// Returns (hardware_verified, u3_port_count, u2_port_count)
pub fn reset_sequence(bus_index: u8, mac_base_addr: u64) -> (bool, usize, usize) {
    let base = mac_base_addr as usize;
    if base == 0 {
        crate::kerror!("bus", "usb_invalid_base"; index = bus_index as u64);
        return (false, 0, 0);
    }
    let ippc_base = base + IPPC_OFFSET;
    let ippc = MmioRegion::new(ippc_base);

    // Step 1: Reset the whole SSUSB IP (quiesces DMA)
    let ctrl0 = ippc.read32(IP_PW_CTRL0);
    ippc.write32(IP_PW_CTRL0, ctrl0 | CTRL0_IP_SW_RST);
    dsb();
    delay_us(10);
    ippc.write32(IP_PW_CTRL0, ctrl0 & !CTRL0_IP_SW_RST);
    dsb();
    delay_ms(1);  // Wait for in-flight DMA to complete

    // Step 2: Power down device IP (host mode only)
    let ctrl2 = ippc.read32(IP_PW_CTRL2);
    ippc.write32(IP_PW_CTRL2, ctrl2 | CTRL2_IP_DEV_PDN);
    dsb();

    // Step 3: Power on host IP
    let ctrl1 = ippc.read32(IP_PW_CTRL1);
    ippc.write32(IP_PW_CTRL1, ctrl1 & !CTRL1_IP_HOST_PDN);
    dsb();

    // Step 4: Enable ports
    let xhci_cap = ippc.read32(IP_XHCI_CAP);
    let u3_port_num = ((xhci_cap >> 8) & 0xF) as usize;
    let u2_port_num = ((xhci_cap >> 0) & 0xF) as usize;

    for i in 0..u3_port_num.min(4) {
        let offset = U3_CTRL_P0 + (i * 8);
        let ctrl = ippc.read32(offset);
        ippc.write32(offset, (ctrl & !(PORT_PDN | PORT_DIS)) | PORT_HOST_SEL);
    }
    for i in 0..u2_port_num.min(5) {
        let offset = U2_CTRL_P0 + (i * 8);
        let ctrl = ippc.read32(offset);
        ippc.write32(offset, (ctrl & !(PORT_PDN | PORT_DIS)) | PORT_HOST_SEL);
    }
    dsb();

    // Step 5: Wait for clocks to stabilize (200ms timeout)
    let stable_mask = STS1_SYSPLL_STABLE | STS1_REF_RST |
                      STS1_SYS125_RST | STS1_XHCI_RST;
    let stable = poll_until(200, 1, || {
        (ippc.read32(IP_PW_STS1) & stable_mask) == stable_mask
    });
    if !stable {
        crate::kerror!("bus", "usb_clk_timeout"; sts1 = crate::klog::hex32(ippc.read32(IP_PW_STS1)));
    }

    // Step 6: Access xHCI registers
    let mac = MmioRegion::new(base);
    let caplength = (mac.read32(CAPLENGTH) & 0xFF) as usize;
    if caplength == 0 || caplength > 0x40 {
        crate::kerror!("bus", "usb_invalid_caplength"; caplength = caplength as u64);
        return (false, u3_port_num, u2_port_num);
    }

    // Step 7: Wait for Controller Not Ready to clear (100ms timeout)
    let op = MmioRegion::new(base + caplength);
    let ready = poll_until(100, 1, || {
        (op.read32(USBSTS) & STS_CNR) == 0
    });
    if !ready {
        crate::kerror!("bus", "usb_not_ready"; usbsts = crate::klog::hex32(op.read32(USBSTS)));
        return (false, u3_port_num, u2_port_num);
    }

    // Verify state
    let verified = verify_state(mac_base_addr);
    (verified, u3_port_num, u2_port_num)
}

/// Verify USB controller is in expected state after power-on
pub fn verify_state(mac_base_addr: u64) -> bool {
    let base = mac_base_addr as usize;
    if base == 0 {
        return false;
    }
    let mac = MmioRegion::new(base);
    let ippc = MmioRegion::new(base + IPPC_OFFSET);

    let caplength = (mac.read32(CAPLENGTH) & 0xFF) as usize;
    if caplength == 0 || caplength > 0x40 {
        crate::kerror!("bus", "usb_invalid_caplength"; caplength = caplength as u64);
        return false;
    }

    let op = MmioRegion::new(base + caplength);
    let usbsts = op.read32(USBSTS);

    // After power-on: CNR=0 (ready), HCE=0 (no error)
    if (usbsts & STS_CNR) != 0 {
        crate::kerror!("bus", "usb_cnr_set"; base = base as u64);
        return false;
    }
    if (usbsts & STS_HCE) != 0 {
        crate::kerror!("bus", "usb_hce_set"; base = base as u64);
        return false;
    }

    // Verify IP is not in sleep mode
    if (ippc.read32(IP_PW_STS1) & STS1_IP_SLEEP_STS) != 0 {
        crate::kerror!("bus", "usb_sleep_mode"; base = base as u64);
        return false;
    }

    true
}

/// Power down USB controller to put in safe state
/// Powers down host IP and all ports - no DMA can occur.
pub fn power_down(_bus_index: u8, mac_base_addr: u64) {
    let base = mac_base_addr as usize;
    if base == 0 {
        return;
    }
    let ippc = MmioRegion::new(base + IPPC_OFFSET);

    let xhci_cap = ippc.read32(IP_XHCI_CAP);
    let u3_port_num = ((xhci_cap >> 8) & 0xF) as usize;
    let u2_port_num = ((xhci_cap >> 0) & 0xF) as usize;

    // Power down all ports
    for i in 0..u3_port_num.min(4) {
        let offset = U3_CTRL_P0 + (i * 8);
        ippc.write32(offset, ippc.read32(offset) | PORT_PDN);
    }
    for i in 0..u2_port_num.min(5) {
        let offset = U2_CTRL_P0 + (i * 8);
        ippc.write32(offset, ippc.read32(offset) | PORT_PDN);
    }
    dsb();

    // Power down host IP
    let ctrl1 = ippc.read32(IP_PW_CTRL1);
    ippc.write32(IP_PW_CTRL1, ctrl1 | CTRL1_IP_HOST_PDN);
    dsb();
}

/// Force halt the USB controller (emergency stop)
pub fn force_halt(bus_index: u8, mac_base_addr: u64) -> Result<(), super::BusError> {
    let base = mac_base_addr as usize;
    if base == 0 {
        return Err(super::BusError::NotFound);
    }
    let mac = MmioRegion::new(base);

    // Read capability length
    let caplength = (mac.read32(CAPLENGTH) & 0xFF) as usize;
    let op = MmioRegion::new(base + caplength);

    // Check if already halted
    let usbsts = op.read32(USBSTS);
    if (usbsts & STS_HCH) != 0 {
        return Ok(());
    }

    // Clear Run/Stop
    let usbcmd = op.read32(USBCMD);
    op.write32(USBCMD, usbcmd & !CMD_RUN);
    dsb();

    // Wait for halt (max 16ms)
    for _ in 0..20 {
        delay_ms(1);
        let sts = op.read32(USBSTS);
        if (sts & STS_HCH) != 0 {
            return Ok(());
        }
    }

    crate::kerror!("bus", "usb_force_halt_failed"; index = bus_index as u64);
    Err(super::BusError::HardwareError)
}

/// Set or clear DMA permission for USB
///
/// For USB, DMA is controlled by xHCI Run/Stop bit
/// We don't directly control R/S here - that's the driver's job
/// Instead, we track permission and can forcibly halt if needed
pub fn set_dma_allowed(bus_index: u8, device_id: u16, allow: bool, mac_base_addr: u64) -> Result<(), super::BusError> {
    if mac_base_addr == 0 {
        return Err(super::BusError::NotFound);
    }

    // For USB, device_id could be:
    // 0x0000 = xHCI controller itself
    // 0x0001+ = USB devices (slot IDs)

    if device_id == 0 {
        // xHCI controller - this controls whether the controller can DMA at all
        if !allow {
            // Force halt the controller
            force_halt(bus_index, mac_base_addr)?;
        }
        // If allowing, driver will set Run/Stop when ready
    }

    Ok(())
}

/// Query USB-specific capabilities by reading xHCI registers
pub fn query_capabilities(_bus_index: u8, mac_base_addr: u64) -> u8 {
    use super::bus_caps;
    let mut caps = 0u8;

    // All USB controllers are xHCI (USB 3.0) with USB 2.0 support
    caps |= bus_caps::USB_2_0;
    caps |= bus_caps::USB_3_0;

    // Check if controller is running (not halted)
    let base = mac_base_addr as usize;
    if base != 0 {
        let mmio = MmioRegion::new(base);

        // Read CAPLENGTH to find operational registers offset
        let caplength = mmio.read32(CAPLENGTH) & 0xFF;
        let op_base = caplength as usize;

        // Read USBSTS - check HCH (Host Controller Halted) bit
        let usbsts = mmio.read32(op_base + USBSTS);
        if (usbsts & STS_HCH) == 0 {
            // Not halted = running
            caps |= bus_caps::USB_RUNNING;
        }
    }

    caps
}
