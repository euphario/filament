//! USB PHY register definitions

// =============================================================================
// IPPC (IP Port Control) registers
// =============================================================================

pub mod ippc {
    pub const IP_PW_CTRL0: usize = 0x00;
    pub const IP_PW_CTRL1: usize = 0x04;
    pub const IP_PW_CTRL2: usize = 0x08;
    pub const IP_PW_STS1: usize = 0x10;
    pub const IP_XHCI_CAP: usize = 0x24;
    pub const U3_CTRL_P0: usize = 0x30;
    pub const U2_CTRL_P0: usize = 0x50;
    pub const U2_PHY_PLL: usize = 0x7C;  // USB2 PHY PLL control - bit 0 forces PHY power on

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

pub mod tphy {
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
// XS-PHY USB2 registers (for SSUSB0)
// =============================================================================

pub mod xsphy {
    // XS-PHY register offsets
    // Based on phy-mtk-xsphy.c from Linux kernel

    // Port 0 USB2 PHY base (within XS-PHY region)
    pub const U2PHY_OFFSET: usize = 0x0;

    // USB2 PHY registers
    pub const U2PHYACR4: usize = U2PHY_OFFSET + 0x20;
    pub const U2PHYDTM0: usize = U2PHY_OFFSET + 0x68;
    pub const U2PHYDTM1: usize = U2PHY_OFFSET + 0x6C;

    // ACR4 bits
    pub const P2A_USB20_BGR_EN: u32 = 1 << 0;  // Bandgap enable
    pub const P2A_USB20_HS_100U_U3_EN: u32 = 1 << 4;  // HS termination enable

    // DTM0 bits
    pub const RG_USB20_FORCE_UART_EN: u32 = 1 << 26;
    pub const RG_USB20_UART_EN: u32 = 1 << 25;
    pub const RG_USB20_BGR_EN: u32 = 1 << 24;

    // DTM1 bits (host mode - similar to T-PHY)
    pub const RG_USB20_FORCE_VBUSVALID: u32 = 1 << 5;
    pub const RG_USB20_FORCE_SESSEND: u32 = 1 << 4;
    pub const RG_USB20_FORCE_BVALID: u32 = 1 << 3;
    pub const RG_USB20_FORCE_AVALID: u32 = 1 << 2;
    pub const RG_USB20_FORCE_IDDIG: u32 = 1 << 1;

    pub const RG_USB20_VBUSVALID: u32 = 1 << 29;
    pub const RG_USB20_SESSEND: u32 = 0 << 28;  // Clear for host
    pub const RG_USB20_BVALID: u32 = 1 << 27;
    pub const RG_USB20_AVALID: u32 = 1 << 26;
    pub const RG_USB20_IDDIG: u32 = 0 << 25;    // Clear for host

    // Combined host mode mask for DTM1
    pub const HOST_MODE_FORCE: u32 = RG_USB20_FORCE_VBUSVALID | RG_USB20_FORCE_SESSEND |
                                     RG_USB20_FORCE_BVALID | RG_USB20_FORCE_AVALID |
                                     RG_USB20_FORCE_IDDIG;
    pub const HOST_MODE_VAL: u32 = RG_USB20_VBUSVALID | RG_USB20_BVALID | RG_USB20_AVALID;
}
