//! USB xHCI Host Controller Driver for MT7988A
//!
//! MT7988A has two USB 3.2 Gen1 controllers (SSUSB0 and SSUSB1).
//! Each has a MediaTek-specific IPPC (IP Port Control) block and
//! a standard xHCI host controller.
//!
//! Reference: Linux drivers/usb/host/xhci-mtk.c

use core::ptr::{read_volatile, write_volatile};
use crate::println;

// =============================================================================
// Base Addresses (from MT7988A device tree)
// =============================================================================

/// SSUSB0 xHCI MAC base address
const SSUSB0_MAC_BASE: usize = 0x1119_0000;
/// SSUSB0 IPPC base address
const SSUSB0_IPPC_BASE: usize = 0x1119_3e00;
/// SSUSB0 uses XS-PHY
const SSUSB0_U2PHY_BASE: usize = 0x11e1_0000;
const SSUSB0_U3PHY_BASE: usize = 0x11e1_3400;

/// SSUSB1 xHCI MAC base address
const SSUSB1_MAC_BASE: usize = 0x1120_0000;
/// SSUSB1 IPPC base address
const SSUSB1_IPPC_BASE: usize = 0x1120_3e00;
/// SSUSB1 uses T-PHY
const SSUSB1_U2PHY_BASE: usize = 0x11c5_0000;
const SSUSB1_U3PHY_BASE: usize = 0x11c5_0700;

// =============================================================================
// XS-PHY Register Offsets (for SSUSB0)
// =============================================================================

/// XS-PHY USB2 registers
mod xsphy_u2 {
    pub const USBPHYACR0: usize = 0x300;   // Interrupt/PLL enable
    pub const USBPHYACR1: usize = 0x304;   // Calibration/VRT/term
    pub const USBPHYACR2: usize = 0x308;   // More config
    pub const USBPHYACR4: usize = 0x310;   // Line state, etc.
    pub const USBPHYACR5: usize = 0x314;   // HS slew rate
    pub const USBPHYACR6: usize = 0x318;   // BC1.1 / OTG VBUS
    pub const U2PHYMON0: usize = 0x32C;    // Monitor register 0
    pub const U2PHYDTM0: usize = 0x368;    // Force/data control
    pub const U2PHYDTM1: usize = 0x36C;    // VBUS/session control

    // USBPHYACR0 bits
    pub const RG_INTR_EN: u32 = 1 << 5;
    pub const RG_U2PLL_FORCE_ON: u32 = 1 << 24;  // Force USB2 PLL on

    // USBPHYACR2 bits
    pub const RG_USB20_OTG_VBUSSCMP_EN: u32 = 1 << 27;

    // USBPHYACR6 bits
    pub const RG_BC11_SW_EN: u32 = 1 << 23;
    pub const RG_OTG_VBUSCMP_EN: u32 = 1 << 20;

    // U2PHYDTM0 bits
    pub const FORCE_SUSPENDM: u32 = 1 << 18;
    pub const RG_SUSPENDM: u32 = 1 << 3;
    pub const FORCE_TERMSEL: u32 = 1 << 17;
    pub const RG_TERMSEL: u32 = 1 << 2;
    pub const FORCE_DATAIN: u32 = 1 << 23;
    pub const FORCE_DP_PULLDOWN: u32 = 1 << 20;
    pub const FORCE_DM_PULLDOWN: u32 = 1 << 21;
    pub const FORCE_UART_BIAS_EN: u32 = 1 << 25;

    // U2PHYDTM1 bits
    pub const RG_VBUSVALID: u32 = 1 << 5;
    pub const RG_SESSEND: u32 = 1 << 4;
    pub const RG_AVALID: u32 = 1 << 2;
}

/// XS-PHY USB3 registers
mod xsphy_u3 {
    pub const PHYA_GLB_00: usize = 0x100;  // Global bias control
    pub const PHYA_LN_04: usize = 0x404;   // TX impedance
    pub const PHYA_LN_14: usize = 0x414;   // RX impedance

    // Default impedance values
    pub const DEFAULT_BIAS_CTRL: u32 = 0x1a << 16;  // bits 21:16
    pub const DEFAULT_TX_IMP: u32 = 0x8;            // bits 4:0
    pub const DEFAULT_RX_IMP: u32 = 0x8;            // bits 4:0
}

// =============================================================================
// T-PHY Register Offsets (for SSUSB1)
// =============================================================================

/// T-PHY USB2 registers (from Linux phy-mtk-tphy.c)
/// Note: T-PHY v2 COM bank is at offset 0x300 from port base
mod tphy_u2 {
    // COM bank offset for T-PHY v2
    const COM_OFFSET: usize = 0x300;

    pub const USBPHYACR0: usize = COM_OFFSET + 0x000;   // PLL/interrupt control
    pub const USBPHYACR5: usize = COM_OFFSET + 0x014;   // HS TX src
    pub const USBPHYACR6: usize = COM_OFFSET + 0x018;   // Disconnect threshold
    pub const U2PHYDTM0: usize = COM_OFFSET + 0x068;    // Force/suspend
    pub const U2PHYDTM1: usize = COM_OFFSET + 0x06C;    // VBUS/session

    // USBPHYACR0 bits
    pub const RG_USB20_INTR_EN: u32 = 1 << 5;

    // U2PHYDTM0 bits
    pub const FORCE_UART_EN: u32 = 1 << 26;
    pub const FORCE_SUSPENDM: u32 = 1 << 18;
    pub const RG_SUSPENDM: u32 = 1 << 3;

    // U2PHYDTM1 bits - T-PHY has FORCE and RG bits separately!
    // FORCE bits enable forcing, RG bits are the values
    pub const FORCE_VBUSVALID: u32 = 1 << 5;  // Force VBUSVALID
    pub const FORCE_AVALID: u32 = 1 << 4;     // Force AVALID
    pub const FORCE_SESSEND: u32 = 1 << 3;    // Force SESSEND
    pub const RG_VBUSVALID: u32 = 1 << 1;     // VBUSVALID value
    pub const RG_AVALID: u32 = 1 << 0;        // AVALID value
    pub const RG_SESSEND: u32 = 1 << 2;       // SESSEND value

    // Host mode value: force VBUS valid and A-valid, but not sessend
    pub const HOST_MODE: u32 = FORCE_VBUSVALID | FORCE_AVALID | RG_VBUSVALID | RG_AVALID;
}

// =============================================================================
// IPPC Register Offsets (MediaTek-specific)
// =============================================================================

/// IPPC register block offsets
mod ippc {
    pub const IP_PW_CTRL0: usize = 0x00;  // Power control 0
    pub const IP_PW_CTRL1: usize = 0x04;  // Power control 1
    pub const IP_PW_CTRL2: usize = 0x08;  // Power control 2
    pub const IP_PW_CTRL3: usize = 0x0C;  // Power control 3
    pub const IP_PW_STS1: usize = 0x10;   // Power status 1
    pub const IP_PW_STS2: usize = 0x14;   // Power status 2
    pub const IP_XHCI_CAP: usize = 0x24;  // xHCI capability
    pub const U3_CTRL_P0: usize = 0x30;   // USB3 port 0 control
    pub const U3_CTRL_P1: usize = 0x38;   // USB3 port 1 control
    pub const U2_CTRL_P0: usize = 0x50;   // USB2 port 0 control
    pub const U2_CTRL_P1: usize = 0x58;   // USB2 port 1 control
    pub const U2_CTRL_P2: usize = 0x60;   // USB2 port 2 control
    pub const U2_PHY_PLL: usize = 0x7C;   // USB2 PHY PLL control
}

/// IPPC IP_PW_CTRL0 bits
mod ctrl0 {
    pub const IP_SW_RST: u32 = 1 << 0;  // Software reset
}

/// IPPC IP_PW_CTRL1 bits
mod ctrl1 {
    pub const IP_HOST_PDN: u32 = 1 << 0;  // Host power down
}

/// IPPC IP_PW_CTRL2 bits
mod ctrl2 {
    pub const IP_DEV_PDN: u32 = 1 << 0;  // Device power down
}

/// IPPC IP_PW_STS1 bits
mod sts1 {
    pub const IP_SLEEP_STS: u32 = 1 << 30;   // Sleep status
    pub const U3_MAC_RST: u32 = 1 << 16;     // USB3 MAC reset done
    pub const XHCI_RST: u32 = 1 << 11;       // xHCI reset done
    pub const SYS125_RST: u32 = 1 << 10;     // 125MHz clock reset done
    pub const REF_RST: u32 = 1 << 8;         // Reference clock reset done
    pub const SYSPLL_STABLE: u32 = 1 << 0;   // System PLL stable
}

/// U3 port control bits
mod u3_ctrl {
    pub const PORT_HOST_SEL: u32 = 1 << 2;  // Host mode select
    pub const PORT_PDN: u32 = 1 << 1;       // Power down
    pub const PORT_DIS: u32 = 1 << 0;       // Disable
}

/// U2 port control bits
mod u2_ctrl {
    pub const PORT_HOST_SEL: u32 = 1 << 2;  // Host mode select
    pub const PORT_PDN: u32 = 1 << 1;       // Power down
    pub const PORT_DIS: u32 = 1 << 0;       // Disable
}

// =============================================================================
// xHCI Register Offsets (Standard)
// =============================================================================

/// xHCI Capability Registers
mod xhci_cap {
    pub const CAPLENGTH: usize = 0x00;      // Capability register length
    pub const HCIVERSION: usize = 0x02;     // Interface version (16-bit)
    pub const HCSPARAMS1: usize = 0x04;     // Structural parameters 1
    pub const HCSPARAMS2: usize = 0x08;     // Structural parameters 2
    pub const HCSPARAMS3: usize = 0x0C;     // Structural parameters 3
    pub const HCCPARAMS1: usize = 0x10;     // Capability parameters 1
    pub const DBOFF: usize = 0x14;          // Doorbell offset
    pub const RTSOFF: usize = 0x18;         // Runtime register space offset
    pub const HCCPARAMS2: usize = 0x1C;     // Capability parameters 2
}

/// xHCI Operational Registers (offset from cap_base + CAPLENGTH)
mod xhci_op {
    pub const USBCMD: usize = 0x00;         // USB Command
    pub const USBSTS: usize = 0x04;         // USB Status
    pub const PAGESIZE: usize = 0x08;       // Page size
    pub const DNCTRL: usize = 0x14;         // Device notification control
    pub const CRCR: usize = 0x18;           // Command ring control (64-bit)
    pub const DCBAAP: usize = 0x30;         // Device context base address (64-bit)
    pub const CONFIG: usize = 0x38;         // Configure
    pub const PORTSC_BASE: usize = 0x400;   // Port status/control (per port)
}

/// USBCMD bits
mod usbcmd {
    pub const RUN: u32 = 1 << 0;            // Run/Stop
    pub const HCRST: u32 = 1 << 1;          // Host controller reset
    pub const INTE: u32 = 1 << 2;           // Interrupter enable
    pub const HSEE: u32 = 1 << 3;           // Host system error enable
}

/// USBSTS bits
mod usbsts {
    pub const HCH: u32 = 1 << 0;            // Host controller halted
    pub const HSE: u32 = 1 << 2;            // Host system error
    pub const EINT: u32 = 1 << 3;           // Event interrupt
    pub const PCD: u32 = 1 << 4;            // Port change detect
    pub const CNR: u32 = 1 << 11;           // Controller not ready
}

/// PORTSC bits
mod portsc {
    pub const CCS: u32 = 1 << 0;            // Current connect status
    pub const PED: u32 = 1 << 1;            // Port enabled/disabled
    pub const PR: u32 = 1 << 4;             // Port reset
    pub const PLS_MASK: u32 = 0xF << 5;     // Port link state
    pub const PP: u32 = 1 << 9;             // Port power
    pub const SPEED_MASK: u32 = 0xF << 10;  // Port speed
    pub const PIC_MASK: u32 = 0x3 << 14;    // Port indicator control
    pub const LWS: u32 = 1 << 16;           // Port link state write strobe
    pub const CSC: u32 = 1 << 17;           // Connect status change
    pub const PEC: u32 = 1 << 18;           // Port enabled/disabled change
    pub const WRC: u32 = 1 << 19;           // Warm port reset change
    pub const PRC: u32 = 1 << 21;           // Port reset change
    pub const PLC: u32 = 1 << 22;           // Port link state change
    pub const CEC: u32 = 1 << 23;           // Port config error change
    pub const WPR: u32 = 1 << 31;           // Warm port reset

    // Write-1-to-clear bits (must preserve when writing)
    pub const W1C_BITS: u32 = CSC | PEC | WRC | PRC | PLC | CEC;
}

// =============================================================================
// USB Controller State
// =============================================================================

/// PHY type
#[derive(Clone, Copy)]
pub enum PhyType {
    XsPhy,  // Used by SSUSB0
    TPhy,   // Used by SSUSB1
}

/// USB controller instance
pub struct UsbController {
    /// Controller index (0 or 1)
    index: u8,
    /// MAC (xHCI) base address
    mac_base: usize,
    /// IPPC base address
    ippc_base: usize,
    /// USB2 PHY base address
    u2phy_base: usize,
    /// USB3 PHY base address
    u3phy_base: usize,
    /// PHY type
    phy_type: PhyType,
    /// Operational registers base (mac_base + cap_length)
    op_base: usize,
    /// Number of USB3 ports
    num_u3_ports: u8,
    /// Number of USB2 ports
    num_u2_ports: u8,
    /// Controller initialized
    initialized: bool,
}

impl UsbController {
    /// Create a new USB controller instance
    pub const fn new(
        index: u8,
        mac_base: usize,
        ippc_base: usize,
        u2phy_base: usize,
        u3phy_base: usize,
        phy_type: PhyType,
    ) -> Self {
        Self {
            index,
            mac_base,
            ippc_base,
            u2phy_base,
            u3phy_base,
            phy_type,
            op_base: 0,
            num_u3_ports: 0,
            num_u2_ports: 0,
            initialized: false,
        }
    }

    /// Read U2 PHY register
    fn u2phy_read(&self, offset: usize) -> u32 {
        unsafe { read_volatile((self.u2phy_base + offset) as *const u32) }
    }

    /// Write U2 PHY register
    fn u2phy_write(&self, offset: usize, value: u32) {
        unsafe { write_volatile((self.u2phy_base + offset) as *mut u32, value) }
    }

    /// Read U3 PHY register
    fn u3phy_read(&self, offset: usize) -> u32 {
        unsafe { read_volatile((self.u3phy_base + offset) as *const u32) }
    }

    /// Write U3 PHY register
    fn u3phy_write(&self, offset: usize, value: u32) {
        unsafe { write_volatile((self.u3phy_base + offset) as *mut u32, value) }
    }

    /// Initialize XS-PHY USB2 port
    fn init_xsphy_u2(&self) {
        println!("    XS-PHY U2 init @ 0x{:08x}", self.u2phy_base);

        // 1. Clear BC1.1 switch enable
        let acr6 = self.u2phy_read(xsphy_u2::USBPHYACR6);
        self.u2phy_write(xsphy_u2::USBPHYACR6, acr6 & !xsphy_u2::RG_BC11_SW_EN);

        // 2. Enable interrupt and force USB2 PLL on
        let acr0 = self.u2phy_read(xsphy_u2::USBPHYACR0);
        self.u2phy_write(xsphy_u2::USBPHYACR0, acr0 | xsphy_u2::RG_INTR_EN | xsphy_u2::RG_U2PLL_FORCE_ON);

        // 3. Enable OTG VBUS comparators (both in ACR6 and ACR2)
        let acr6 = self.u2phy_read(xsphy_u2::USBPHYACR6);
        self.u2phy_write(xsphy_u2::USBPHYACR6, acr6 | xsphy_u2::RG_OTG_VBUSCMP_EN);

        let acr2 = self.u2phy_read(xsphy_u2::USBPHYACR2);
        self.u2phy_write(xsphy_u2::USBPHYACR2, acr2 | xsphy_u2::RG_USB20_OTG_VBUSSCMP_EN);

        // 4. Clear force suspend - ensure PHY is active, not in low power mode
        //    Also clear force data-in, pulldowns, and UART bias
        let dtm0 = self.u2phy_read(xsphy_u2::U2PHYDTM0);
        let new_dtm0 = dtm0 & !(xsphy_u2::FORCE_SUSPENDM | xsphy_u2::FORCE_DATAIN
                               | xsphy_u2::FORCE_DP_PULLDOWN | xsphy_u2::FORCE_DM_PULLDOWN
                               | xsphy_u2::FORCE_UART_BIAS_EN);
        self.u2phy_write(xsphy_u2::U2PHYDTM0, new_dtm0);

        // 5. Set VBUS valid and A-valid, clear session end
        let dtm1 = self.u2phy_read(xsphy_u2::U2PHYDTM1);
        let new_dtm1 = (dtm1 & !xsphy_u2::RG_SESSEND)
                     | xsphy_u2::RG_VBUSVALID
                     | xsphy_u2::RG_AVALID;
        self.u2phy_write(xsphy_u2::U2PHYDTM1, new_dtm1);

        // 6. Wait for PLL to stabilize
        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        println!("    XS-PHY U2 init done");
    }

    /// Initialize XS-PHY USB3 port
    fn init_xsphy_u3(&self) {
        println!("    XS-PHY U3 init @ 0x{:08x}", self.u3phy_base);

        // Set default bias control
        let glb00 = self.u3phy_read(xsphy_u3::PHYA_GLB_00);
        let new_glb00 = (glb00 & !(0x3F << 16)) | xsphy_u3::DEFAULT_BIAS_CTRL;
        self.u3phy_write(xsphy_u3::PHYA_GLB_00, new_glb00);

        // Set TX impedance
        let ln04 = self.u3phy_read(xsphy_u3::PHYA_LN_04);
        let new_ln04 = (ln04 & !0x1F) | xsphy_u3::DEFAULT_TX_IMP;
        self.u3phy_write(xsphy_u3::PHYA_LN_04, new_ln04);

        // Set RX impedance
        let ln14 = self.u3phy_read(xsphy_u3::PHYA_LN_14);
        let new_ln14 = (ln14 & !0x1F) | xsphy_u3::DEFAULT_RX_IMP;
        self.u3phy_write(xsphy_u3::PHYA_LN_14, new_ln14);

        println!("    XS-PHY U3 init done");
    }

    /// Initialize T-PHY USB2 port
    fn init_tphy_u2(&self) {
        println!("    T-PHY U2 init @ 0x{:08x}", self.u2phy_base);

        // 1. Clear UART enable and suspend mode force
        let dtm0 = self.u2phy_read(tphy_u2::U2PHYDTM0);
        let new_dtm0 = dtm0 & !(tphy_u2::FORCE_UART_EN | tphy_u2::FORCE_SUSPENDM);
        self.u2phy_write(tphy_u2::U2PHYDTM0, new_dtm0);

        // 2. Enable interrupt
        let acr0 = self.u2phy_read(tphy_u2::USBPHYACR0);
        self.u2phy_write(tphy_u2::USBPHYACR0, acr0 | tphy_u2::RG_USB20_INTR_EN);

        // 3. Set host mode: force VBUS valid and A-valid (0x33)
        // T-PHY needs both FORCE bits AND value bits set
        self.u2phy_write(tphy_u2::U2PHYDTM1, tphy_u2::HOST_MODE);

        // Debug: verify write
        let dtm1_verify = self.u2phy_read(tphy_u2::U2PHYDTM1);
        println!("    T-PHY DTM1 written: 0x{:08x}, read back: 0x{:08x}",
                 tphy_u2::HOST_MODE, dtm1_verify);

        println!("    T-PHY U2 init done");
    }

    /// Initialize PHY for this controller
    fn init_phy(&self) {
        println!("  Initializing PHY...");

        match self.phy_type {
            PhyType::XsPhy => {
                self.init_xsphy_u2();
                self.init_xsphy_u3();
            }
            PhyType::TPhy => {
                self.init_tphy_u2();
                // T-PHY U3 init is more complex, skip for now
            }
        }
    }

    /// Read IPPC register
    fn ippc_read(&self, offset: usize) -> u32 {
        unsafe { read_volatile((self.ippc_base + offset) as *const u32) }
    }

    /// Write IPPC register
    fn ippc_write(&self, offset: usize, value: u32) {
        unsafe { write_volatile((self.ippc_base + offset) as *mut u32, value) }
    }

    /// Read xHCI capability register
    fn cap_read(&self, offset: usize) -> u32 {
        unsafe { read_volatile((self.mac_base + offset) as *const u32) }
    }

    /// Read xHCI operational register
    fn op_read(&self, offset: usize) -> u32 {
        unsafe { read_volatile((self.op_base + offset) as *const u32) }
    }

    /// Write xHCI operational register
    fn op_write(&self, offset: usize, value: u32) {
        unsafe { write_volatile((self.op_base + offset) as *mut u32, value) }
    }

    /// Perform IPPC software reset
    fn ippc_reset(&self) {
        // Set software reset
        self.ippc_write(ippc::IP_PW_CTRL0, ctrl0::IP_SW_RST);

        // Small delay
        for _ in 0..1000 {
            core::hint::spin_loop();
        }

        // Clear software reset
        self.ippc_write(ippc::IP_PW_CTRL0, 0);

        // Wait for reset to complete
        for _ in 0..10000 {
            core::hint::spin_loop();
        }
    }

    /// Power on the host IP
    fn host_power_on(&self) {
        // Power down device IP
        let ctrl2 = self.ippc_read(ippc::IP_PW_CTRL2);
        self.ippc_write(ippc::IP_PW_CTRL2, ctrl2 | ctrl2::IP_DEV_PDN);

        // Power on host IP (clear power down bit)
        let ctrl1 = self.ippc_read(ippc::IP_PW_CTRL1);
        self.ippc_write(ippc::IP_PW_CTRL1, ctrl1 & !ctrl1::IP_HOST_PDN);
    }

    /// Configure USB3 ports for host mode
    fn configure_u3_ports(&self, num_ports: u8) {
        for i in 0..num_ports {
            let offset = ippc::U3_CTRL_P0 + (i as usize * 8);
            let ctrl = self.ippc_read(offset);

            // Clear power down and disable, set host mode
            let new_ctrl = (ctrl & !(u3_ctrl::PORT_PDN | u3_ctrl::PORT_DIS))
                         | u3_ctrl::PORT_HOST_SEL;
            self.ippc_write(offset, new_ctrl);
        }
    }

    /// Configure USB2 ports for host mode
    fn configure_u2_ports(&self, num_ports: u8) {
        for i in 0..num_ports {
            let offset = ippc::U2_CTRL_P0 + (i as usize * 8);
            let ctrl = self.ippc_read(offset);

            // Clear power down and disable, set host mode
            let new_ctrl = (ctrl & !(u2_ctrl::PORT_PDN | u2_ctrl::PORT_DIS))
                         | u2_ctrl::PORT_HOST_SEL;
            self.ippc_write(offset, new_ctrl);
        }
    }

    /// Wait for IP clocks to stabilize
    fn wait_clocks_stable(&self) -> bool {
        let required = sts1::SYSPLL_STABLE | sts1::REF_RST | sts1::SYS125_RST | sts1::XHCI_RST;

        for _ in 0..100000 {
            let sts = self.ippc_read(ippc::IP_PW_STS1);
            if (sts & required) == required {
                return true;
            }
            core::hint::spin_loop();
        }

        false
    }

    /// Force USB2 PHY power on via IPPC
    fn force_u2_phy_power(&self) {
        // U2_PHY_PLL register at offset 0x7C controls USB2 PHY power
        // Setting bit 0 forces the PHY to stay powered
        let pll = self.ippc_read(ippc::U2_PHY_PLL);
        self.ippc_write(ippc::U2_PHY_PLL, pll | 1);
    }

    /// Reset the xHCI controller
    fn xhci_reset(&mut self) -> bool {
        // Read capability length to find operational registers
        let cap_length = (self.cap_read(xhci_cap::CAPLENGTH) & 0xFF) as usize;
        self.op_base = self.mac_base + cap_length;

        // Halt the controller first
        let cmd = self.op_read(xhci_op::USBCMD);
        self.op_write(xhci_op::USBCMD, cmd & !usbcmd::RUN);

        // Wait for halt
        for _ in 0..10000 {
            if self.op_read(xhci_op::USBSTS) & usbsts::HCH != 0 {
                break;
            }
            core::hint::spin_loop();
        }

        // Issue reset
        self.op_write(xhci_op::USBCMD, usbcmd::HCRST);

        // Wait for reset to complete (CNR bit clears)
        for _ in 0..100000 {
            let sts = self.op_read(xhci_op::USBSTS);
            let cmd = self.op_read(xhci_op::USBCMD);
            if (sts & usbsts::CNR) == 0 && (cmd & usbcmd::HCRST) == 0 {
                return true;
            }
            core::hint::spin_loop();
        }

        false
    }

    /// Initialize the USB controller
    pub fn init(&mut self) -> bool {
        println!("USB{}: Initializing...", self.index);
        println!("  MAC base:  0x{:08x}", self.mac_base);
        println!("  IPPC base: 0x{:08x}", self.ippc_base);

        // Step 1: IPPC software reset
        println!("  IPPC reset...");
        self.ippc_reset();

        // Step 2: Power on host IP
        println!("  Host power on...");
        self.host_power_on();

        // Step 3: Read port counts from capability register
        let cap = self.ippc_read(ippc::IP_XHCI_CAP);
        self.num_u3_ports = ((cap >> 8) & 0xF) as u8;
        self.num_u2_ports = (cap & 0xF) as u8;
        println!("  Ports: {} USB3, {} USB2", self.num_u3_ports, self.num_u2_ports);

        // Step 4: Configure ports for host mode
        println!("  Configuring IPPC ports...");
        self.configure_u3_ports(self.num_u3_ports);
        self.configure_u2_ports(self.num_u2_ports);

        // Step 5: Wait for clocks
        println!("  Waiting for clocks...");
        if !self.wait_clocks_stable() {
            println!("  [!!] Clock stabilization timeout");
            return false;
        }

        // Step 6: Read xHCI version and parameters
        let version = (self.cap_read(xhci_cap::CAPLENGTH) >> 16) as u16;
        let hcsparams1 = self.cap_read(xhci_cap::HCSPARAMS1);
        let max_slots = hcsparams1 & 0xFF;
        let max_intrs = (hcsparams1 >> 8) & 0x7FF;
        let max_ports = (hcsparams1 >> 24) & 0xFF;

        println!("  xHCI version: {}.{}", version >> 8, version & 0xFF);
        println!("  Max slots: {}, interrupters: {}, ports: {}",
                 max_slots, max_intrs, max_ports);

        // Step 7: Reset xHCI controller
        println!("  xHCI reset...");
        if !self.xhci_reset() {
            println!("  [!!] xHCI reset timeout");
            return false;
        }

        // Step 8: Read page size
        let page_size = self.op_read(xhci_op::PAGESIZE);
        println!("  Page size: {} bytes", (page_size & 0xFFFF) << 12);

        // Step 9: Initialize PHY AFTER xHCI reset (reset clears PHY settings)
        println!("  Initializing PHY...");
        self.init_phy();

        // Step 10: Force USB2 PHY power on via IPPC
        println!("  Forcing U2 PHY power...");
        self.force_u2_phy_power();

        // Step 11: Start the xHCI controller (needed for port status to work)
        // Note: Full enumeration needs DCBAA and command ring, but basic
        // port detection works with just RUN bit set
        println!("  Starting xHCI controller...");
        let cmd = self.op_read(xhci_op::USBCMD);
        self.op_write(xhci_op::USBCMD, cmd | usbcmd::RUN);

        // Wait for controller to start (HCH bit clears)
        for _ in 0..10000 {
            if self.op_read(xhci_op::USBSTS) & usbsts::HCH == 0 {
                println!("  xHCI controller running");
                break;
            }
            core::hint::spin_loop();
        }

        // Give PHY and link time to stabilize after controller starts
        println!("  Waiting for link stabilization...");
        for _ in 0..10_000_000 {
            core::hint::spin_loop();
        }

        self.initialized = true;
        println!("USB{}: Initialization complete", self.index);
        true
    }

    /// Power on all ports
    pub fn power_on_ports(&self) {
        if !self.initialized {
            return;
        }

        let hcsparams1 = self.cap_read(xhci_cap::HCSPARAMS1);
        let max_ports = ((hcsparams1 >> 24) & 0xFF) as usize;

        println!("  Powering on {} ports...", max_ports);

        for port in 0..max_ports {
            let offset = xhci_op::PORTSC_BASE + (port * 0x10);
            let portsc = self.op_read(offset);

            // Clear any stale W1C status bits and port reset, then set power
            let new_portsc = (portsc & !(portsc::W1C_BITS | portsc::PR)) | portsc::PP;
            self.op_write(offset, new_portsc);

            println!("    Port {}: PORTSC before=0x{:08x} after=0x{:08x}",
                     port + 1, portsc, self.op_read(offset));
        }

        // Check USBSTS for any events
        let usbsts = self.op_read(xhci_op::USBSTS);
        println!("  USBSTS: 0x{:08x} (PCD={}, HCH={})",
                 usbsts,
                 (usbsts & usbsts::PCD) != 0,
                 (usbsts & usbsts::HCH) != 0);

        // Wait for power to stabilize (longer delay - ~200ms)
        println!("  Waiting for device detection...");
        for _ in 0..10_000_000 {
            core::hint::spin_loop();
        }

        // Check if any device connected after power on
        for port in 0..max_ports {
            let offset = xhci_op::PORTSC_BASE + (port * 0x10);
            let portsc = self.op_read(offset);

            let pp = (portsc & portsc::PP) != 0;
            let ccs = (portsc & portsc::CCS) != 0;
            let pls = (portsc >> 5) & 0xF;

            println!("  Port {}: Power={} Connected={} PLS={} (raw=0x{:08x})",
                     port + 1,
                     if pp { "ON" } else { "OFF" },
                     if ccs { "YES" } else { "NO" },
                     pls,
                     portsc);
        }

        // Debug: dump some PHY registers
        self.debug_phy_status();
    }

    /// Debug: print PHY status registers
    fn debug_phy_status(&self) {
        println!("  PHY Debug:");

        match self.phy_type {
            PhyType::XsPhy => {
                // Read XS-PHY status registers
                let acr0 = self.u2phy_read(xsphy_u2::USBPHYACR0);
                let acr2 = self.u2phy_read(xsphy_u2::USBPHYACR2);
                let acr4 = self.u2phy_read(xsphy_u2::USBPHYACR4);
                let acr6 = self.u2phy_read(xsphy_u2::USBPHYACR6);
                let mon0 = self.u2phy_read(xsphy_u2::U2PHYMON0);
                let dtm0 = self.u2phy_read(xsphy_u2::U2PHYDTM0);
                let dtm1 = self.u2phy_read(xsphy_u2::U2PHYDTM1);

                println!("    U2PHY ACR0=0x{:08x} ACR2=0x{:08x}", acr0, acr2);
                println!("    U2PHY ACR4=0x{:08x} ACR6=0x{:08x}", acr4, acr6);
                println!("    U2PHY MON0=0x{:08x}", mon0);
                println!("    U2PHY DTM0=0x{:08x} DTM1=0x{:08x}", dtm0, dtm1);

                // Check specific bits
                let pll_on = (acr0 & xsphy_u2::RG_U2PLL_FORCE_ON) != 0;
                let force_susp = (dtm0 & xsphy_u2::FORCE_SUSPENDM) != 0;
                let uart_bias = (dtm0 & xsphy_u2::FORCE_UART_BIAS_EN) != 0;
                let vbus_valid = (dtm1 & xsphy_u2::RG_VBUSVALID) != 0;
                let avalid = (dtm1 & xsphy_u2::RG_AVALID) != 0;
                let sessend = (dtm1 & xsphy_u2::RG_SESSEND) != 0;
                println!("    PLL_ON={} UART_BIAS={} FORCE_SUSP={}",
                         pll_on, uart_bias, force_susp);
                println!("    VBUS_VALID={} AVALID={} SESSEND={}",
                         vbus_valid, avalid, sessend);

                let glb00 = self.u3phy_read(xsphy_u3::PHYA_GLB_00);
                println!("    U3PHY GLB00=0x{:08x}", glb00);
            }
            PhyType::TPhy => {
                let acr0 = self.u2phy_read(tphy_u2::USBPHYACR0);
                let dtm0 = self.u2phy_read(tphy_u2::U2PHYDTM0);
                let dtm1 = self.u2phy_read(tphy_u2::U2PHYDTM1);
                println!("    U2PHY ACR0=0x{:08x} DTM0=0x{:08x} DTM1=0x{:08x}", acr0, dtm0, dtm1);

                // Check specific bits (T-PHY uses different bit positions)
                let intr_en = (acr0 & tphy_u2::RG_USB20_INTR_EN) != 0;
                let force_susp = (dtm0 & tphy_u2::FORCE_SUSPENDM) != 0;
                // T-PHY DTM1: bit5=FORCE_VBUS, bit4=FORCE_AVALID, bit1=RG_VBUS, bit0=RG_AVALID
                let force_vbus = (dtm1 & tphy_u2::FORCE_VBUSVALID) != 0;
                let vbus_valid = (dtm1 & tphy_u2::RG_VBUSVALID) != 0;
                let avalid = (dtm1 & tphy_u2::RG_AVALID) != 0;
                println!("    INTR_EN={} FORCE_SUSP={}", intr_en, force_susp);
                println!("    FORCE_VBUS={} VBUS_VALID={} AVALID={}", force_vbus, vbus_valid, avalid);
            }
        }

        // Also check IPPC status
        let sts1 = self.ippc_read(ippc::IP_PW_STS1);
        let sts2 = self.ippc_read(ippc::IP_PW_STS2);
        println!("    IPPC STS1=0x{:08x} STS2=0x{:08x}", sts1, sts2);
    }

    /// Reset a specific port
    pub fn reset_port(&self, port: usize) -> bool {
        if !self.initialized {
            return false;
        }

        let offset = xhci_op::PORTSC_BASE + (port * 0x10);

        // Issue port reset
        let portsc = self.op_read(offset);
        let new_portsc = (portsc & !portsc::W1C_BITS) | portsc::PR;
        self.op_write(offset, new_portsc);

        // Wait for reset to complete (PR bit clears, PRC bit sets)
        for _ in 0..100000 {
            let portsc = self.op_read(offset);
            if (portsc & portsc::PR) == 0 {
                // Reset complete, clear PRC
                let clear_prc = (portsc & !portsc::W1C_BITS) | portsc::PRC;
                self.op_write(offset, clear_prc);

                // Check if device is now enabled
                let final_portsc = self.op_read(offset);
                let enabled = (final_portsc & portsc::PED) != 0;
                let speed = (final_portsc >> 10) & 0xF;

                if enabled {
                    let speed_str = match speed {
                        1 => "Full-speed (12 Mbps)",
                        2 => "Low-speed (1.5 Mbps)",
                        3 => "High-speed (480 Mbps)",
                        4 => "SuperSpeed (5 Gbps)",
                        5 => "SuperSpeed+ (10 Gbps)",
                        _ => "Unknown",
                    };
                    println!("  Port {} reset complete: {}", port + 1, speed_str);
                    return true;
                }
            }
            core::hint::spin_loop();
        }

        println!("  Port {} reset timeout", port + 1);
        false
    }

    /// Scan for connected devices and reset them
    pub fn scan_devices(&self) {
        if !self.initialized {
            return;
        }

        let hcsparams1 = self.cap_read(xhci_cap::HCSPARAMS1);
        let max_ports = ((hcsparams1 >> 24) & 0xFF) as usize;

        println!("  Scanning for devices on {} ports...", max_ports);

        for port in 0..max_ports {
            let offset = xhci_op::PORTSC_BASE + (port * 0x10);
            let portsc = self.op_read(offset);

            let ccs = (portsc & portsc::CCS) != 0;
            let ped = (portsc & portsc::PED) != 0;
            let pls = (portsc >> 5) & 0xF;
            let speed = (portsc >> 10) & 0xF;

            if ccs {
                if ped && pls == 0 {
                    // Device already connected and in U0 - no reset needed!
                    let speed_str = match speed {
                        1 => "Full-speed (12 Mbps)",
                        2 => "Low-speed (1.5 Mbps)",
                        3 => "High-speed (480 Mbps)",
                        4 => "SuperSpeed (5 Gbps)",
                        5 => "SuperSpeed+ (10 Gbps)",
                        _ => "Unknown",
                    };
                    println!("  Port {}: Device connected and active - {}", port + 1, speed_str);
                } else {
                    println!("  Port {}: Device detected (PLS={}), resetting...", port + 1, pls);
                    self.reset_port(port);
                }
            }
        }
    }

    /// Print port status
    pub fn print_port_status(&self) {
        if !self.initialized {
            println!("USB{}: Not initialized", self.index);
            return;
        }

        let hcsparams1 = self.cap_read(xhci_cap::HCSPARAMS1);
        let max_ports = ((hcsparams1 >> 24) & 0xFF) as usize;

        println!("USB{} Port Status:", self.index);
        for port in 0..max_ports {
            let offset = xhci_op::PORTSC_BASE + (port * 0x10);
            let portsc = self.op_read(offset);

            let ccs = (portsc & portsc::CCS) != 0;
            let ped = (portsc & portsc::PED) != 0;
            let pls = (portsc >> 5) & 0xF;
            let pp = (portsc & portsc::PP) != 0;
            let speed = (portsc >> 10) & 0xF;

            let speed_str = match speed {
                0 => "N/A",
                1 => "Full",
                2 => "Low",
                3 => "High",
                4 => "Super",
                5 => "Super+",
                _ => "Unknown",
            };

            let pls_str = match pls {
                0 => "U0",       // Active
                1 => "U1",       // Standby
                2 => "U2",       // Sleep
                3 => "U3",       // Suspend
                4 => "Disabled",
                5 => "RxDetect",
                6 => "Inactive",
                7 => "Polling",
                8 => "Recovery",
                9 => "HotReset",
                10 => "CompMode",
                11 => "TestMode",
                15 => "Resume",
                _ => "Unknown",
            };

            println!("  Port {}: {} {} {} PLS={} Speed={}",
                     port + 1,
                     if ccs { "Connected" } else { "Disconnected" },
                     if ped { "Enabled" } else { "Disabled" },
                     if pp { "Powered" } else { "Unpowered" },
                     pls_str,
                     speed_str);
        }
    }
}

// =============================================================================
// Global Instances
// =============================================================================

static mut USB0: UsbController = UsbController::new(
    0,
    SSUSB0_MAC_BASE,
    SSUSB0_IPPC_BASE,
    SSUSB0_U2PHY_BASE,
    SSUSB0_U3PHY_BASE,
    PhyType::XsPhy,
);
static mut USB1: UsbController = UsbController::new(
    1,
    SSUSB1_MAC_BASE,
    SSUSB1_IPPC_BASE,
    SSUSB1_U2PHY_BASE,
    SSUSB1_U3PHY_BASE,
    PhyType::TPhy,
);

/// Initialize USB controller 0
pub fn init() {
    unsafe {
        let usb = &mut *core::ptr::addr_of_mut!(USB0);
        usb.init();
    }
}

/// Initialize both USB controllers
pub fn init_all() {
    unsafe {
        let usb0 = &mut *core::ptr::addr_of_mut!(USB0);
        let usb1 = &mut *core::ptr::addr_of_mut!(USB1);
        usb0.init();
        usb1.init();
    }
}

/// Print USB port status
pub fn print_status() {
    unsafe {
        let usb0 = &*core::ptr::addr_of!(USB0);
        let usb1 = &*core::ptr::addr_of!(USB1);
        usb0.print_port_status();
        usb1.print_port_status();
    }
}

/// Test function
pub fn test() {
    println!("\n=== USB Subsystem Test ===");

    // Note: BPI-R4 physical USB Type-A port is on SSUSB1, not SSUSB0
    // U-Boot shows: xhci@11200000 (SSUSB1)
    unsafe {
        let usb = &mut *core::ptr::addr_of_mut!(USB1);

        // Initialize
        if usb.init() {
            // Power on ports
            usb.power_on_ports();

            // Scan for devices
            usb.scan_devices();

            // Print final status
            usb.print_port_status();
        }
    }

    println!("=== USB Test Complete ===\n");
}
