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

        debug("rst+");
        // Step 1: Assert all reset signals
        mac.write32(regs::PCIE_RST_CTRL_REG, rst_ctrl::ALL);
        let rst_after = mac.read32(regs::PCIE_RST_CTRL_REG);
        debug_status(rst_after);

        // Step 2: Wait 100ms for power/clock stabilization (TPVPERL per PCIe spec)
        delay_ms(100);

        debug("rst-");
        // Step 3: De-assert all resets at once
        mac.write32(regs::PCIE_RST_CTRL_REG, 0);
        let rst_final = mac.read32(regs::PCIE_RST_CTRL_REG);
        debug_status(rst_final);

        // Wait for device to come out of reset and PHY to stabilize
        delay_ms(200);

        debug("link");
        // Step 4: Read status registers for debug
        let ltssm_val = mac.read32(regs::PCIE_LTSSM_STATUS_REG);
        let link_val = mac.read32(regs::PCIE_LINK_STATUS_REG);
        debug_status(ltssm_val);
        debug_status(link_val);

        let link_up = Self::wait_for_link(&mac, 500);

        debug("atu");
        // Setup ATU for memory transactions (1:1 mapping)
        // CPU address 0x30200000 -> PCIe bus address 0x30200000
        Self::setup_atu(&mac, config.mem_base, config.mem_base, config.mem_size);

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
    fn setup_atu(mac: &MmioRegion, cpu_addr: u64, pci_addr: u64, size: u64) {
        // Validate size - must be non-zero and power of 2 for ATU
        if size == 0 {
            return; // Cannot program ATU with zero size
        }

        // Calculate size encoding per Linux pcie-mediatek-gen3.c:
        // #define PCIE_ATR_SIZE(size) (((((size) - 1) << 1) & GENMASK(6, 1)) | PCIE_ATR_EN)
        // Where size = fls(bytes) - 1, and PCIE_ATR_EN = bit 0 (enable)
        // This goes in the SOURCE ADDRESS LSB register, not a separate param register!
        let leading = size.leading_zeros();
        if leading >= 63 {
            return; // Size too small (< 2 bytes)
        }
        let size_log2 = 63 - leading;  // fls(size) - 1
        // Ensure size_log2 >= 1 to avoid underflow
        let atr_size = if size_log2 >= 1 {
            (((size_log2 - 1) << 1) & 0x7E) | 0x01  // Shifted + enable bit
        } else {
            0x01  // Minimum size, just enable bit
        };

        // Use translation table 0 for memory
        let table_base = regs::PCIE_TRANS_TABLE_BASE_REG;

        // Write source (CPU) address LSB with size/enable (Linux: cpu_addr | PCIE_ATR_SIZE(...))
        mac.write32(table_base, (cpu_addr as u32) | atr_size);

        // Write source (CPU) address MSB
        mac.write32(table_base + atu::SRC_ADDR_MSB_OFFSET, (cpu_addr >> 32) as u32);

        // Write translation (PCIe) address LSB
        mac.write32(table_base + atu::TRSL_ADDR_LSB_OFFSET, pci_addr as u32);

        // Write translation (PCIe) address MSB
        mac.write32(table_base + atu::TRSL_ADDR_MSB_OFFSET, (pci_addr >> 32) as u32);

        // Param register (0x10) - may contain additional translation parameters
        // For now, write 0 (memory type, no special params)
        mac.write32(table_base + atu::TRSL_PARAM_OFFSET, 0);
    }

    /// Check if link is up
    pub fn is_link_up(&self) -> bool {
        self.link_up
    }

    /// Get link speed (Gen1/2/3)
    pub fn link_speed(&self) -> u8 {
        let status = self.mac.read32(regs::PCIE_LINK_STATUS_REG);
        ((status & link_status::SPEED_MASK) >> link_status::SPEED_SHIFT) as u8
    }

    /// Get link width (x1, x2, etc.)
    pub fn link_width(&self) -> u8 {
        let status = self.mac.read32(regs::PCIE_LINK_STATUS_REG);
        ((status & link_status::WIDTH_MASK) >> link_status::WIDTH_SHIFT) as u8
    }

    /// Get port configuration
    pub fn port_config(&self) -> &PciePortConfig {
        &self.config
    }

    /// Get port description
    pub fn port_desc(&self) -> &'static str {
        self.config.desc
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
    fn allocate_bar0(&mut self, mac: *const MmioRegion, bdf: PcieBdf) -> (u64, u32) {
        use crate::regs::cfg as pci_cfg;
        use crate::regs::command;

        // SAFETY: mac pointer is valid for duration of enumerate()
        let cfg = PcieConfigSpace::new(unsafe { &*mac });

        // First, probe BAR0 to get size
        let bar0_orig = cfg.read32(bdf, pci_cfg::BAR0);

        // Check if memory BAR (bit 0 = 0)
        if (bar0_orig & 0x1) != 0 {
            return (0, 0); // I/O BAR, skip
        }

        // Check if 64-bit (bits 2:1 = 10)
        let is_64bit = ((bar0_orig >> 1) & 0x3) == 2;

        // Write all 1s to probe size
        cfg.write32(bdf, pci_cfg::BAR0, 0xFFFF_FFFF);
        let size_mask = cfg.read32(bdf, pci_cfg::BAR0) & !0xF;

        if size_mask == 0 {
            // Restore and return - BAR not implemented
            cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
            return (0, 0);
        }

        // Calculate size (lowest set bit) - use checked arithmetic
        let size = (!size_mask).wrapping_add(1) & size_mask;
        if size == 0 {
            cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
            return (0, 0);
        }

        // Align next_mem_addr to BAR size (use checked arithmetic)
        let size64 = size as u64;
        let aligned_addr = match self.next_mem_addr.checked_add(size64 - 1) {
            Some(addr) => addr & !(size64 - 1),
            None => {
                cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
                return (0, 0); // Overflow
            }
        };

        // Check if we have space (use checked arithmetic)
        let end_addr = match aligned_addr.checked_add(size64) {
            Some(addr) => addr,
            None => {
                cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
                return (0, 0); // Overflow
            }
        };
        let mem_end = match self.config.mem_base.checked_add(self.config.mem_size) {
            Some(end) => end,
            None => {
                cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
                return (0, 0); // Config overflow
            }
        };
        if end_addr > mem_end {
            // Out of memory window space
            cfg.write32(bdf, pci_cfg::BAR0, bar0_orig);
            return (0, 0);
        }

        // Program BAR0 with allocated address
        cfg.write32(bdf, pci_cfg::BAR0, aligned_addr as u32);
        if is_64bit {
            cfg.write32(bdf, pci_cfg::BAR0 + 4, (aligned_addr >> 32) as u32);
        }

        // Enable memory space access
        let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
        cfg.write32(bdf, pci_cfg::COMMAND, (cmd | command::MEM_SPACE | command::BUS_MASTER) as u32);

        // Update allocation pointer
        self.next_mem_addr = end_addr;

        (aligned_addr, size)
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
            let (bar0_addr, bar0_size) = if !is_bridge {
                self.allocate_bar0(mac, bdf)
            } else {
                (0, 0)
            };

            devices.push(PcieDevice {
                bdf,
                id,
                header_type: header_type & 0x7F,
                is_bridge,
                bar0_addr,
                bar0_size,
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
                        let (bar0_addr, bar0_size) = if !func_is_bridge {
                            self.allocate_bar0(mac, bdf)
                        } else {
                            (0, 0)
                        };
                        devices.push(PcieDevice {
                            bdf,
                            id,
                            header_type: ht & 0x7F,
                            is_bridge: func_is_bridge,
                            bar0_addr,
                            bar0_size,
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

        // Enable memory space and bus master
        let cmd = cfg.read16(bdf, pci_cfg::COMMAND);
        cfg.write32(bdf, pci_cfg::COMMAND,
            (cmd | command::MEM_SPACE | command::BUS_MASTER) as u32);
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
