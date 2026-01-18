//! PCIe Host Controller
//!
//! MediaTek Gen3 PCIe controller initialization and device enumeration.
//! Uses TLP-based config space access through MAC registers.

use crate::{MmioRegion, delay_ms};
use crate::soc::PciePortConfig;
use crate::regs::{self, rst_ctrl, link_status, misc_ctrl, atu};
use crate::config::{PcieBdf, PcieDeviceId, PcieConfigSpace};

/// PCIe initialization error
#[derive(Debug, Clone, Copy)]
pub enum PcieError {
    /// Failed to map MAC MMIO region
    MacMmioFailed,
    /// Link training timeout
    LinkTimeout,
    /// Invalid port number
    InvalidPort,
}

/// Discovered PCIe device
#[derive(Debug, Clone)]
pub struct PcieDevice {
    /// Bus/Device/Function address
    pub bdf: PcieBdf,
    /// Device identification
    pub id: PcieDeviceId,
    /// Header type (0 = endpoint, 1 = bridge)
    pub header_type: u8,
    /// Is this a PCI-to-PCI bridge?
    pub is_bridge: bool,
    /// BAR0 physical address (0 if not present)
    pub bar0_addr: u64,
    /// BAR0 size in bytes (0 if not present)
    pub bar0_size: u32,
    /// PCI command register value (after bus master enable)
    pub command: u16,
}

/// PCIe host controller
///
/// Represents a single PCIe root port. Uses MediaTek Gen3 TLP-based
/// config space access through MAC registers.
pub struct PcieController {
    /// MAC MMIO region (also used for config space access)
    mac: MmioRegion,
    /// Port configuration (from SoC layer)
    config: PciePortConfig,
    /// Link is up
    link_up: bool,
    /// Next available address in memory window for BAR allocation
    next_mem_addr: u64,
}

impl PcieController {
    /// Open an existing (already initialized) PCIe port for operations
    ///
    /// This does NOT perform reset or re-initialization - just maps the MAC
    /// registers for access. Use this for operations on an already-active port.
    pub fn open_existing(config: PciePortConfig) -> Result<Self, PcieError> {
        // Map MAC MMIO region
        let mac = MmioRegion::open(config.mac_base, config.mac_size)
            .ok_or(PcieError::MacMmioFailed)?;

        // Check if link is up (port should already be initialized)
        let link_status = mac.read32(regs::PCIE_LINK_STATUS_REG);
        let link_up = (link_status & link_status::PCIE_PORT_LINKUP) != 0;

        Ok(Self {
            mac,
            config,
            link_up,
            next_mem_addr: config.mem_base,
        })
    }

    /// Initialize a PCIe port with the given configuration
    ///
    /// Performs the full initialization sequence:
    /// 1. Map MAC registers
    /// 2. Assert resets
    /// 3. Wait for stabilization
    /// 4. De-assert resets
    /// 5. Wait for link up
    ///
    /// The `debug` and `debug_status` callbacks are for progress reporting.
    pub fn init(
        config: PciePortConfig,
        debug: fn(&str),
        debug_status: fn(u32),
    ) -> Result<Self, PcieError> {
        debug("map");
        // Map MAC MMIO region
        let mac = MmioRegion::open(config.mac_base, config.mac_size)
            .ok_or(PcieError::MacMmioFailed)?;

        // Show virtual address for debugging
        debug_status((mac.virt_base() >> 32) as u32);
        debug_status(mac.virt_base() as u32);

        // Read initial state
        let rst_init = mac.read32(regs::PCIE_RST_CTRL_REG);
        debug_status(rst_init);

        debug("misc");
        // Disable DVFSRC voltage request (MediaTek specific)
        let misc = mac.read32(regs::PCIE_MISC_CTRL_REG);
        mac.write32(regs::PCIE_MISC_CTRL_REG, misc | misc_ctrl::DISABLE_DVFSRC_VLT_REQ);

        // Set Root Complex mode
        // Linux: mtk_pcie_startup_port_v2() sets PCIE_RC_MODE in PCIE_SETTING_REG
        // Without RC_MODE, the controller may not properly route inbound (device->host) transactions
        // NOTE: We don't force link speed - let hardware auto-negotiate like Linux does
        // (Linux only sets GEN_SUPPORT if max-link-speed is specified in device tree, which MT7988 doesn't have)
        let setting = mac.read32(regs::PCIE_SETTING_REG);
        mac.write32(regs::PCIE_SETTING_REG, setting | regs::pcie_setting::RC_MODE);

        // Reset sequence - MUST match Linux exactly (single-stage release)
        // Linux: mtk_pcie_startup_port() lines 474-493
        debug("rst");

        // Assert all resets (read-modify-write like Linux)
        let mut rst = mac.read32(regs::PCIE_RST_CTRL_REG);
        rst |= rst_ctrl::ALL;
        mac.write32(regs::PCIE_RST_CTRL_REG, rst);
        debug_status(mac.read32(regs::PCIE_RST_CTRL_REG));

        // Wait 100ms per PCIe CEM spec (TPVPERL) for power and clock to stabilize
        delay_ms(100);

        // De-assert ALL resets at once (critical: Linux does this in one write)
        rst &= !rst_ctrl::ALL;
        mac.write32(regs::PCIE_RST_CTRL_REG, rst);
        debug_status(mac.read32(regs::PCIE_RST_CTRL_REG));

        debug("link");
        // Step 4: Read status registers for debug
        let ltssm_val = mac.read32(regs::PCIE_LTSSM_STATUS_REG);
        let link_val = mac.read32(regs::PCIE_LINK_STATUS_REG);
        debug_status(ltssm_val);
        debug_status(link_val);

        let link_up = Self::wait_for_link(&mac, 500);

        // Debug: show final link status after training
        if link_up {
            let final_ltssm = mac.read32(regs::PCIE_LTSSM_STATUS_REG);
            let final_link = mac.read32(regs::PCIE_LINK_STATUS_REG);
            debug_status(final_ltssm);
            debug_status(final_link);
        }

        debug("atu");
        // Setup ATU for memory transactions (1:1 mapping)
        // CPU address 0x30200000 -> PCIe bus address 0x30200000
        Self::setup_atu(&mac, config.mem_base, config.mem_base, config.mem_size);

        debug("msi");
        // Enable MSI on root complex
        // Required for devices that use MSI for internal state machine transitions
        Self::enable_msi(&mac, config.mac_base);

        debug("done");
        Ok(Self {
            mac,
            config,
            link_up,
            next_mem_addr: config.mem_base,
        })
    }

    /// Wait for link to come up
    fn wait_for_link(mac: &MmioRegion, timeout_ms: u32) -> bool {
        for _ in 0..(timeout_ms / 10) {
            let status = mac.read32(regs::PCIE_LINK_STATUS_REG);
            if (status & link_status::PCIE_PORT_LINKUP) != 0 {
                return true;
            }
            delay_ms(10);
        }
        false
    }

    /// Setup Address Translation Unit (ATU) for outbound memory transactions
    ///
    /// Programs the translation table to map CPU addresses to PCIe bus addresses.
    /// Uses 1:1 mapping for simplicity (CPU addr == PCIe addr).
    ///
    /// # Outbound vs Inbound
    ///
    /// This configures **outbound** translation (CPU → PCIe device BARs).
    ///
    /// **Inbound** translation (PCIe device DMA → CPU memory) on MT7988A uses
    /// identity mapping by default - the AXI interconnect passes PCIe memory
    /// transactions directly to DRAM without explicit ATU configuration.
    /// If your platform requires explicit inbound windows, see `setup_inbound_atu`.
    fn setup_atu(mac: &MmioRegion, cpu_addr: u64, pci_addr: u64, size: u64) {
        // Validate size - must be non-zero for ATU
        if size == 0 {
            return;
        }

        // Calculate size encoding using helper
        let atr_size = atu::size_encoding(size);
        if atr_size == 0 {
            return; // Size too small
        }

        // Use translation table 0 for memory
        let table_base = regs::PCIE_TRANS_TABLE_BASE_REG;

        // Write source (CPU) address LSB with size/enable
        mac.write32(table_base, (cpu_addr as u32) | atr_size);

        // Write source (CPU) address MSB
        mac.write32(table_base + atu::SRC_ADDR_MSB_OFFSET, (cpu_addr >> 32) as u32);

        // Write translation (PCIe) address LSB
        mac.write32(table_base + atu::TRSL_ADDR_LSB_OFFSET, pci_addr as u32);

        // Write translation (PCIe) address MSB
        mac.write32(table_base + atu::TRSL_ADDR_MSB_OFFSET, (pci_addr >> 32) as u32);

        // Param register (0x10) - memory type, no special params
        mac.write32(table_base + atu::TRSL_PARAM_OFFSET, 0);
    }

    /// Setup inbound ATU for device DMA to host memory (if needed)
    ///
    /// On MT7988A, inbound DMA uses identity mapping by default through the AXI
    /// interconnect, so this function is a no-op. The kernel's `mmap_dma` syscall
    /// returns DMA addresses via the platform's `phys_to_dma()` translation.
    ///
    /// For platforms that require explicit inbound windows (e.g., those with
    /// PCIe-to-AXI bridges that need configuration), this function would program
    /// the inbound translation tables.
    ///
    /// # Arguments
    /// * `_pci_addr` - PCIe bus address that devices will use for DMA
    /// * `_cpu_addr` - CPU physical address where DMA should land
    /// * `_size` - Size of the inbound window
    #[allow(dead_code)]
    fn setup_inbound_atu(&self, _pci_addr: u64, _cpu_addr: u64, _size: u64) {
        // MT7988A: No explicit inbound ATU configuration needed.
        // The AXI interconnect provides identity-mapped access to DRAM.
        //
        // For platforms needing explicit inbound windows, you would:
        // 1. Find the inbound ATU register base (may be different from outbound)
        // 2. Program: PCIe addr → CPU physical addr translation
        // 3. Enable the window with appropriate size encoding
        //
        // Example for a hypothetical platform:
        // let table_base = INBOUND_ATU_BASE + table_index * atu::TLB_SET_OFFSET;
        // self.mac.write32(table_base, (pci_addr as u32) | atu::size_encoding(size));
        // self.mac.write32(table_base + atu::SRC_ADDR_MSB_OFFSET, (pci_addr >> 32) as u32);
        // self.mac.write32(table_base + atu::TRSL_ADDR_LSB_OFFSET, cpu_addr as u32);
        // self.mac.write32(table_base + atu::TRSL_ADDR_MSB_OFFSET, (cpu_addr >> 32) as u32);
        // self.mac.write32(table_base + atu::TRSL_PARAM_OFFSET, INBOUND_MEM_TYPE);
    }

    /// Enable MSI (Message Signaled Interrupts) on the root complex
    ///
    /// EXACT copy of Linux mtk_pcie_enable_msi() in pcie-mediatek-gen3.c:377-404
    ///
    /// # Arguments
    /// * `mac_phys_base` - Physical base address of the PCIe MAC (e.g., 0x11300000)
    fn enable_msi(mac: &MmioRegion, mac_phys_base: u64) {
        use regs::{msi, PCIE_MSI_SET_BASE_REG, PCIE_MSI_SET_ADDR_HI_BASE,
                   PCIE_MSI_SET_ENABLE_REG, PCIE_INT_ENABLE_REG};

        // Linux: for (i = 0; i < PCIE_MSI_SET_NUM; i++) {
        for i in 0..msi::SET_NUM {
            // Linux: msi_set->msg_addr = pcie->reg_base + PCIE_MSI_SET_BASE_REG + i * PCIE_MSI_SET_OFFSET;
            let msg_addr = mac_phys_base + PCIE_MSI_SET_BASE_REG as u64 + (i * msi::SET_OFFSET) as u64;

            // Linux: writel_relaxed(lower_32_bits(msi_set->msg_addr), msi_set->base);
            let set_base = PCIE_MSI_SET_BASE_REG + i * msi::SET_OFFSET;
            mac.write32(set_base, msg_addr as u32);

            // Linux: writel_relaxed(upper_32_bits(msi_set->msg_addr),
            //                       pcie->base + PCIE_MSI_SET_ADDR_HI_BASE + i * PCIE_MSI_SET_ADDR_HI_OFFSET);
            let addr_hi_reg = PCIE_MSI_SET_ADDR_HI_BASE + i * msi::ADDR_HI_OFFSET;
            mac.write32(addr_hi_reg, (msg_addr >> 32) as u32);
        }

        // Linux: val = readl_relaxed(pcie->base + PCIE_MSI_SET_ENABLE_REG);
        //        val |= PCIE_MSI_SET_ENABLE;
        //        writel_relaxed(val, pcie->base + PCIE_MSI_SET_ENABLE_REG);
        let val = mac.read32(PCIE_MSI_SET_ENABLE_REG);
        mac.write32(PCIE_MSI_SET_ENABLE_REG, val | msi::SET_ENABLE_BITS);

        // Linux: val = readl_relaxed(pcie->base + PCIE_INT_ENABLE_REG);
        //        val |= PCIE_MSI_ENABLE;
        //        writel_relaxed(val, pcie->base + PCIE_INT_ENABLE_REG);
        let val = mac.read32(PCIE_INT_ENABLE_REG);
        mac.write32(PCIE_INT_ENABLE_REG, val | msi::INT_ENABLE_BITS);
    }

    /// Check if link is up
    pub fn is_link_up(&self) -> bool {
        self.link_up
    }

    /// Get link speed (Gen1/2/3)
    ///
    /// Reads from PCIe capability Link Status register in Root Port config space.
    /// PCIe cap is at 0x1080 (since Link Control 2 @ 0x10b0 = cap + 0x30).
    /// Link Control is at cap + 0x10 = 0x1090, Link Status is at cap + 0x12 = 0x1092.
    /// We read 32-bit aligned at 0x1090 and extract upper 16 bits for Link Status.
    pub fn link_speed(&self) -> u8 {
        // Read 32-bit at aligned address 0x1090 (Link Control + Link Status)
        // Link Status is in upper 16 bits
        const PCIE_CAP_LINK_CTL_STA: usize = 0x1090;
        let val = self.mac.read32(PCIE_CAP_LINK_CTL_STA);
        // Link Status is upper 16 bits, speed is bits 3:0 of Link Status
        let link_status = (val >> 16) as u16;
        (link_status & 0xF) as u8
    }

    /// Get link width (x1, x2, etc.)
    ///
    /// Reads from PCIe capability Link Status register in Root Port config space.
    pub fn link_width(&self) -> u8 {
        // Read 32-bit at aligned address 0x1090 (Link Control + Link Status)
        // Link Status is in upper 16 bits
        const PCIE_CAP_LINK_CTL_STA: usize = 0x1090;
        let val = self.mac.read32(PCIE_CAP_LINK_CTL_STA);
        // Link Status is upper 16 bits, width is bits 9:4 of Link Status
        let link_status = (val >> 16) as u16;
        ((link_status >> 4) & 0x3F) as u8
    }

    /// Get raw LTSSM register value for debugging
    ///
    /// Use `regs::ltssm::decode()` to interpret the value.
    pub fn ltssm_raw(&self) -> u32 {
        self.mac.read32(regs::PCIE_LTSSM_STATUS_REG)
    }

    /// Dump all SoC PCIe controller registers for debugging
    ///
    /// This prints all the important registers from the MediaTek PCIe MAC
    /// for comparison with OpenWRT/Linux.
    pub fn dump_soc_registers(&self) {
        use userlib::println;

        // Register addresses from pcie-mediatek-gen3.c
        const PCIE_BASE_CFG_REG: usize = 0x14;
        const PCIE_SETTING_REG: usize = 0x80;
        const PCIE_PCI_IDS_1: usize = 0x9c;
        const PCIE_EQ_PRESET_01_REG: usize = 0x100;
        const PCIE_CFGNUM_REG: usize = 0x140;
        const PCIE_RST_CTRL_REG: usize = 0x148;
        const PCIE_LTSSM_STATUS_REG: usize = 0x150;
        const PCIE_LINK_STATUS_REG: usize = 0x154;
        const PCIE_INT_ENABLE_REG: usize = 0x180;
        const PCIE_INT_STATUS_REG: usize = 0x184;
        const PCIE_MSI_SET_ENABLE_REG: usize = 0x190;
        const PCIE_ICMD_PM_REG: usize = 0x198;
        const PCIE_PIPE4_PIE8_REG: usize = 0x338;
        const PCIE_MISC_CTRL_REG: usize = 0x348;
        const PCIE_TRANS_TABLE_BASE: usize = 0x800;
        const PCIE_MSI_SET_BASE_REG: usize = 0xc00;
        const PCIE_RESOURCE_CTRL_REG: usize = 0xd2c;
        const PCIE_CONF_LINK2_CTL_STS: usize = 0x10b0;

        println!("=== SoC PCIe MAC Registers (base 0x{:08x}) ===",
                 self.config.mac_base);

        println!("  BASE_CFG (0x14):     0x{:08x}", self.mac.read32(PCIE_BASE_CFG_REG));
        println!("  SETTING (0x80):      0x{:08x}", self.mac.read32(PCIE_SETTING_REG));
        println!("  PCI_IDS_1 (0x9c):    0x{:08x}", self.mac.read32(PCIE_PCI_IDS_1));
        println!("  EQ_PRESET (0x100):   0x{:08x}", self.mac.read32(PCIE_EQ_PRESET_01_REG));
        println!("  CFGNUM (0x140):      0x{:08x}", self.mac.read32(PCIE_CFGNUM_REG));
        println!("  RST_CTRL (0x148):    0x{:08x}", self.mac.read32(PCIE_RST_CTRL_REG));
        println!("  LTSSM (0x150):       0x{:08x}", self.mac.read32(PCIE_LTSSM_STATUS_REG));
        println!("  LINK_STATUS (0x154): 0x{:08x}", self.mac.read32(PCIE_LINK_STATUS_REG));
        println!("  INT_ENABLE (0x180):  0x{:08x}", self.mac.read32(PCIE_INT_ENABLE_REG));
        println!("  INT_STATUS (0x184):  0x{:08x}", self.mac.read32(PCIE_INT_STATUS_REG));
        println!("  MSI_SET_EN (0x190):  0x{:08x}", self.mac.read32(PCIE_MSI_SET_ENABLE_REG));
        println!("  ICMD_PM (0x198):     0x{:08x}", self.mac.read32(PCIE_ICMD_PM_REG));
        println!("  PIPE4_PIE8 (0x338):  0x{:08x}", self.mac.read32(PCIE_PIPE4_PIE8_REG));
        println!("  MISC_CTRL (0x348):   0x{:08x}", self.mac.read32(PCIE_MISC_CTRL_REG));
        println!("  RSRC_CTRL (0xd2c):   0x{:08x}", self.mac.read32(PCIE_RESOURCE_CTRL_REG));
        println!("  LINK2_CTL (0x10b0):  0x{:08x}", self.mac.read32(PCIE_CONF_LINK2_CTL_STS));

        // ATU table entry 0 (5 registers per entry)
        println!("  ATU[0] (0x800-0x810):");
        println!("    SRC_LSB:   0x{:08x}", self.mac.read32(PCIE_TRANS_TABLE_BASE));
        println!("    SRC_MSB:   0x{:08x}", self.mac.read32(PCIE_TRANS_TABLE_BASE + 0x4));
        println!("    TRSL_LSB:  0x{:08x}", self.mac.read32(PCIE_TRANS_TABLE_BASE + 0x8));
        println!("    TRSL_MSB:  0x{:08x}", self.mac.read32(PCIE_TRANS_TABLE_BASE + 0xc));
        println!("    PARAM:     0x{:08x}", self.mac.read32(PCIE_TRANS_TABLE_BASE + 0x10));

        // MSI set 0 (3 registers per set)
        println!("  MSI[0] (0xc00-0xc08):");
        println!("    BASE:      0x{:08x}", self.mac.read32(PCIE_MSI_SET_BASE_REG));
        println!("    STATUS:    0x{:08x}", self.mac.read32(PCIE_MSI_SET_BASE_REG + 0x4));
        println!("    ENABLE:    0x{:08x}", self.mac.read32(PCIE_MSI_SET_BASE_REG + 0x8));

        // Decode LTSSM bits for debugging
        let ltssm = self.mac.read32(PCIE_LTSSM_STATUS_REG);
        let state = (ltssm >> 24) & 0x1F;
        let bit11 = (ltssm >> 11) & 1;
        println!("  LTSSM decode: state={}, bit11={} (OpenWRT has bit11=1)",
                 state, bit11);

        // Decode SETTING bits
        let setting = self.mac.read32(PCIE_SETTING_REG);
        let rc_mode = setting & 1;
        let link_width = (setting >> 8) & 0xF;
        let gen_support = (setting >> 12) & 0x7;
        println!("  SETTING decode: RC_MODE={}, LINK_WIDTH={}, GEN_SUPPORT={}",
                 rc_mode, link_width, gen_support);

        // Root Port config space (at offset 0x1000 from MAC base)
        // This is the RC's own config space, not device config space
        const RC_CFG_BASE: usize = 0x1000;
        println!("=== Root Port Config Space ===");
        println!("  VID/DID (0x00):      0x{:08x}", self.mac.read32(RC_CFG_BASE + 0x00));
        println!("  CMD/STS (0x04):      0x{:08x}", self.mac.read32(RC_CFG_BASE + 0x04));
        println!("  CLASS (0x08):        0x{:08x}", self.mac.read32(RC_CFG_BASE + 0x08));
        println!("  HDR/BIST (0x0c):     0x{:08x}", self.mac.read32(RC_CFG_BASE + 0x0c));

        // PCIe capability is typically at offset 0x80 in config space
        // For MediaTek it's at 0x1080 from MAC base
        const RC_PCIE_CAP: usize = 0x1080;
        println!("  PCIe Cap (0x80):");
        println!("    CAP_ID (0x80):     0x{:08x}", self.mac.read32(RC_PCIE_CAP + 0x00));
        println!("    DEV_CAP (0x84):    0x{:08x}", self.mac.read32(RC_PCIE_CAP + 0x04));
        println!("    DEV_CTL (0x88):    0x{:08x}", self.mac.read32(RC_PCIE_CAP + 0x08));
        println!("    LINK_CAP (0x8c):   0x{:08x}", self.mac.read32(RC_PCIE_CAP + 0x0c));
        println!("    LINK_CTL (0x90):   0x{:08x}", self.mac.read32(RC_PCIE_CAP + 0x10));
        println!("    LINK_CTL2 (0xb0):  0x{:08x}", self.mac.read32(RC_PCIE_CAP + 0x30));
    }

    /// Get port configuration
    pub fn port_config(&self) -> &PciePortConfig {
        &self.config
    }

    /// Get port description
    pub fn port_desc(&self) -> &'static str {
        self.config.desc
    }

    /// Reset the link by asserting PERST# (fundamental reset to endpoint)
    ///
    /// This puts the endpoint device back into its power-on state.
    /// Used when a driver exits to reset the device for the next user.
    ///
    /// Returns true if link came back up, false on timeout.
    pub fn reset_link(&mut self) -> bool {
        // Assert PERST# only (keep MAC/PHY/BRG running)
        let rst = self.mac.read32(regs::PCIE_RST_CTRL_REG);
        self.mac.write32(regs::PCIE_RST_CTRL_REG, rst | rst_ctrl::PCIE_PE_RSTB);

        // PCIe spec requires PERST# asserted for at least 100ms
        delay_ms(100);

        // De-assert PERST#
        self.mac.write32(regs::PCIE_RST_CTRL_REG, rst & !rst_ctrl::PCIE_PE_RSTB);

        // Wait for link to come back up (up to 500ms)
        for _ in 0..50 {
            delay_ms(10);
            let status = self.mac.read32(regs::PCIE_LINK_STATUS_REG);
            if (status & link_status::PCIE_PORT_LINKUP) != 0 {
                self.link_up = true;
                return true;
            }
        }

        self.link_up = false;
        false
    }

    /// Get config space accessor (uses MAC region for TLP-based access)
    pub fn config_space(&self) -> PcieConfigSpace<'_> {
        PcieConfigSpace::new(&self.mac)
    }

    /// Enumerate all devices on this PCIe port
    ///
    /// Returns a list of discovered devices with BARs programmed.
    pub fn enumerate(&mut self) -> PcieDeviceList {
        let mut devices = PcieDeviceList::new();

        if !self.link_up {
            return devices;
        }

        // Use raw pointer to avoid borrow issues - we need to mutate next_mem_addr
        // while also reading from mac for config space access
        let mac_ptr = &self.mac as *const MmioRegion;
        self.scan_bus(mac_ptr, 0, &mut devices);

        devices
    }

    /// Scan a single bus for devices
    fn scan_bus(&mut self, mac: *const MmioRegion, bus: u8, devices: &mut PcieDeviceList) {
        for device in 0..32 {
            self.scan_device(mac, bus, device, devices);
        }
    }

    /// Allocate and program BAR0 for a device
    ///
    /// Returns (addr, size) where addr is the allocated physical address.
    /// Allocate BAR0 and enable bus master. Returns (addr, size, command).
    fn allocate_bar0(&mut self, mac: *const MmioRegion, bdf: PcieBdf) -> (u64, u32, u16) {
        use crate::regs::cfg as pci_cfg;
        use crate::regs::command;

        // SAFETY: mac pointer is valid for duration of enumerate()
        let cfg = PcieConfigSpace::new(unsafe { &*mac });

        // First, probe BAR0 to get size
        let bar0_orig = cfg.read32(bdf, pci_cfg::BAR0);

        // Check if memory BAR (bit 0 = 0)
        if (bar0_orig & 0x1) != 0 {
            return (0, 0, 0); // I/O BAR, skip
        }

        // Check if 64-bit (bits 2:1 = 10)
        let is_64bit = ((bar0_orig >> 1) & 0x3) == 2;

        // Write all 1s to probe size
        cfg.write32(bdf, pci_cfg::BAR0, 0xFFFF_FFFF);
        let size_mask = cfg.read32(bdf, pci_cfg::BAR0) & !0xF;

        if size_mask == 0 {
            // Restore and return - BAR not implemented
            cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
            return (0, 0, 0);
        }

        // Calculate size (lowest set bit) - use checked arithmetic
        let size = (!size_mask).wrapping_add(1) & size_mask;
        if size == 0 {
            cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
            return (0, 0, 0);
        }

        // Align next_mem_addr to BAR size (use checked arithmetic)
        let size64 = size as u64;
        let aligned_addr = match self.next_mem_addr.checked_add(size64 - 1) {
            Some(addr) => addr & !(size64 - 1),
            None => {
                cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
                return (0, 0, 0); // Overflow
            }
        };

        // Check if we have space (use checked arithmetic)
        let end_addr = match aligned_addr.checked_add(size64) {
            Some(addr) => addr,
            None => {
                cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
                return (0, 0, 0); // Overflow
            }
        };
        let mem_end = match self.config.mem_base.checked_add(self.config.mem_size) {
            Some(end) => end,
            None => {
                cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
                return (0, 0, 0); // Config overflow
            }
        };
        if end_addr > mem_end {
            // Out of memory window space
            cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
            return (0, 0, 0);
        }

        // Program BAR0 with allocated address
        cfg.write32(bdf, pci_cfg::BAR0, aligned_addr as u32);
        if is_64bit {
            cfg.write32(bdf, pci_cfg::BAR0 + 4, (aligned_addr >> 32) as u32);
        }

        // Enable memory space access, bus master, and disable INTx
        // OpenWRT sets Command = 0x0406: MEM_SPACE | BUS_MASTER | INT_DISABLE
        let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
        let new_cmd = cmd | command::MEM_SPACE | command::BUS_MASTER | command::INT_DISABLE;
        cfg.write32(bdf, pci_cfg::COMMAND, new_cmd as u32);

        // Read back to verify
        let cmd_readback = cfg.read16(bdf, pci_cfg::COMMAND);

        // Update allocation pointer
        self.next_mem_addr = end_addr;

        (aligned_addr, size, cmd_readback)
    }

    /// Scan a device (check all functions if multi-function)
    fn scan_device(&mut self, mac: *const MmioRegion, bus: u8, device: u8, devices: &mut PcieDeviceList) {
        // SAFETY: mac pointer is valid for duration of enumerate()
        let cfg = PcieConfigSpace::new(unsafe { &*mac });
        let bdf = PcieBdf::new(bus, device, 0);

        // Check function 0
        if let Some(id) = cfg.read_device_id(bdf) {
            let header_type = cfg.read_header_type(bdf);
            let is_multifunction = (header_type & 0x80) != 0;
            let is_bridge = (header_type & 0x7F) == 0x01;

            // Allocate and program BAR0 for non-bridge devices
            // For bridges, just enable BME so DMA can flow through
            let (bar0_addr, bar0_size, command) = if !is_bridge {
                self.allocate_bar0(mac, bdf)
            } else {
                // Enable BME on bridge so downstream devices can DMA
                // OpenWRT sets Command = 0x0406: MEM_SPACE | BUS_MASTER | INT_DISABLE
                // Use TLP access - direct MMIO to RC config space (0x1004) is read-only
                use crate::regs::cfg as pci_cfg;
                use crate::regs::command;
                let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
                let new_cmd = cmd | command::MEM_SPACE | command::BUS_MASTER | command::INT_DISABLE;
                cfg.write16(bdf, pci_cfg::COMMAND, new_cmd);
                let readback = cfg.read16(bdf, pci_cfg::COMMAND);
                (0, 0, readback)
            };

            // Configure all PCIe registers to match OpenWRT exactly
            // Use TLP-based config access for all devices (including root port)
            // Direct MMIO to RC config space is read-only
            cfg.disable_aspm(bdf);
            cfg.configure_device_control(bdf);
            cfg.configure_link_control2(bdf, is_bridge);

            // PCIe capability register configuration already done by configure_* methods

            devices.push(PcieDevice {
                bdf,
                id,
                header_type: header_type & 0x7F,
                is_bridge,
                bar0_addr,
                bar0_size,
                command,
            });

            // If bridge, configure and scan secondary bus
            if is_bridge {
                let cfg = PcieConfigSpace::new(unsafe { &*mac });
                let secondary = cfg.read_secondary_bus(bdf);
                if secondary == 0 || secondary == 0xFF {
                    // Assign bus 1 as secondary (simple single-level hierarchy)
                    self.configure_bridge(&cfg, bdf, bus, bus + 1, 0xFF);
                    // Scan downstream bus
                    self.scan_bus(mac, bus + 1, devices);
                } else {
                    self.scan_bus(mac, secondary, devices);
                }
            }

            // Check other functions if multi-function device
            if is_multifunction {
                for func in 1..8 {
                    let cfg = PcieConfigSpace::new(unsafe { &*mac });
                    let bdf = PcieBdf::new(bus, device, func);
                    if let Some(id) = cfg.read_device_id(bdf) {
                        let ht = cfg.read_header_type(bdf);
                        let func_is_bridge = (ht & 0x7F) == 0x01;
                        let (bar0_addr, bar0_size, command) = if !func_is_bridge {
                            self.allocate_bar0(mac, bdf)
                        } else {
                            // Enable BME on bridge using TLP access
                            use crate::regs::cfg as pci_cfg;
                            use crate::regs::command;
                            let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
                            let new_cmd = cmd | command::MEM_SPACE | command::BUS_MASTER | command::INT_DISABLE;
                            cfg.write16(bdf, pci_cfg::COMMAND, new_cmd);
                            let readback = cfg.read16(bdf, pci_cfg::COMMAND);
                            (0, 0, readback)
                        };
                        // Configure all PCIe registers to match OpenWRT
                        cfg.disable_aspm(bdf);
                        cfg.configure_device_control(bdf);
                        cfg.configure_link_control2(bdf, func_is_bridge);
                        devices.push(PcieDevice {
                            bdf,
                            id,
                            header_type: ht & 0x7F,
                            is_bridge: func_is_bridge,
                            bar0_addr,
                            bar0_size,
                            command,
                        });
                    }
                }
            }
        }
    }

    /// Configure a PCI bridge's bus numbers and memory window
    fn configure_bridge(&self, cfg: &PcieConfigSpace, bdf: PcieBdf, primary: u8, secondary: u8, subordinate: u8) {
        use crate::regs::cfg as pci_cfg;
        use crate::regs::command;

        // Write bus numbers
        let bus_config = (primary as u32)
            | ((secondary as u32) << 8)
            | ((subordinate as u32) << 16);
        cfg.write32(bdf, pci_cfg::PRIMARY_BUS, bus_config);

        // Configure memory window for this port
        // Memory Base/Limit registers: bits 15:4 = address bits 31:20
        let mem_base = self.config.mem_base;
        let mem_end = self.config.mem_base + self.config.mem_size - 1;

        // Memory base: (addr >> 16) with bits 3:0 = 0
        let mem_base_reg = ((mem_base >> 16) & 0xFFF0) as u16;
        // Memory limit: (addr >> 16) with bits 3:0 = 0
        let mem_limit_reg = ((mem_end >> 16) & 0xFFF0) as u16;

        // Write as a single 32-bit value: [limit:16][base:16]
        let mem_window = (mem_base_reg as u32) | ((mem_limit_reg as u32) << 16);
        cfg.write32(bdf, pci_cfg::MEMORY_BASE, mem_window);

        // Disable prefetchable memory window (set base > limit)
        cfg.write32(bdf, pci_cfg::PREF_MEMORY_BASE, 0xFFF0_0000);

        // Enable memory space, bus master, and disable INTx
        // OpenWRT sets Command = 0x0406: MEM_SPACE | BUS_MASTER | INT_DISABLE
        let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
        cfg.write32(bdf, pci_cfg::COMMAND,
            (cmd | command::MEM_SPACE | command::BUS_MASTER | command::INT_DISABLE) as u32);
    }

    /// Enable Bus Master on a specific device
    ///
    /// This is called by drivers that need DMA capability.
    /// Returns true on success.
    pub fn enable_bus_master(&self, bus: u8, device: u8, function: u8) -> bool {
        use crate::regs::cfg as pci_cfg;
        use crate::regs::command;

        let cfg = PcieConfigSpace::new(&self.mac);
        let bdf = PcieBdf::new(bus, device, function);

        // Check if device exists
        if cfg.read_device_id(bdf).is_none() {
            return false;
        }

        // Enable memory space, bus master, and disable INTx
        // OpenWRT sets Command = 0x0406: MEM_SPACE | BUS_MASTER | INT_DISABLE
        let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
        cfg.write32(bdf, pci_cfg::COMMAND,
            (cmd | command::MEM_SPACE | command::BUS_MASTER | command::INT_DISABLE) as u32);

        // Verify it was written
        let readback = cfg.read16(bdf, pci_cfg::COMMAND);
        (readback & command::BUS_MASTER) != 0
    }

    /// Read PCIe Device Status register
    ///
    /// Returns the Device Status value (16-bit), or None if device not found.
    /// Key bits:
    /// - Bit 0: Correctable Error Detected
    /// - Bit 1: Non-Fatal Error Detected
    /// - Bit 2: Fatal Error Detected
    /// - Bit 3: Unsupported Request Detected (indicates bad DMA address!)
    /// - Bit 5: Transactions Pending
    pub fn read_device_status(&self, bus: u8, device: u8, function: u8) -> Option<u16> {
        let cfg = PcieConfigSpace::new(&self.mac);
        let bdf = PcieBdf::new(bus, device, function);
        cfg.read_device_status(bdf)
    }

    /// Clear PCIe Device Status error bits (W1C - Write 1 to Clear)
    ///
    /// Returns true on success.
    pub fn clear_device_status(&self, bus: u8, device: u8, function: u8) -> bool {
        let cfg = PcieConfigSpace::new(&self.mac);
        let bdf = PcieBdf::new(bus, device, function);
        cfg.clear_device_status_errors(bdf)
    }
}

/// Maximum devices that can be tracked
pub const MAX_PCIE_DEVICES: usize = 16;

/// List of discovered PCIe devices (fixed-size, no heap)
pub struct PcieDeviceList {
    devices: [Option<PcieDevice>; MAX_PCIE_DEVICES],
    count: usize,
    /// Number of devices dropped due to overflow
    overflow_count: usize,
}

impl PcieDeviceList {
    /// Create empty device list
    pub fn new() -> Self {
        Self {
            devices: [const { None }; MAX_PCIE_DEVICES],
            count: 0,
            overflow_count: 0,
        }
    }

    /// Add a device to the list
    /// Returns false if the list is full (device dropped)
    pub fn push(&mut self, device: PcieDevice) -> bool {
        if self.count < MAX_PCIE_DEVICES {
            self.devices[self.count] = Some(device);
            self.count += 1;
            true
        } else {
            self.overflow_count += 1;
            false
        }
    }

    /// Get number of devices dropped due to overflow
    pub fn overflow_count(&self) -> usize {
        self.overflow_count
    }

    /// Get number of devices
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Iterate over devices
    pub fn iter(&self) -> impl Iterator<Item = &PcieDevice> {
        self.devices[..self.count].iter().filter_map(|d| d.as_ref())
    }
}
