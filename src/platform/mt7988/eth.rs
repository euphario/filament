//! MT7988 Ethernet Driver
//!
//! Basic ethernet driver for MediaTek MT7988's GMAC.
//! MT7988 has multiple ethernet ports with USXGMII, 10GBase-R, 2500Base-X support.
//!
//! This is a minimal driver for basic packet TX/RX.

use core::ptr::{read_volatile, write_volatile};
use crate::println;

/// Ethernet frame constants
pub const ETH_FRAME_MIN: usize = 64;
pub const ETH_FRAME_MAX: usize = 1518;
pub const ETH_HEADER_SIZE: usize = 14;
pub const ETH_FCS_SIZE: usize = 4;

/// MT7988 Ethernet base addresses
/// Based on MT7988 datasheet and Linux mtk_eth_soc driver
mod regs {
    /// Frame Engine base address
    pub const FE_BASE: usize = 0x15100000;

    /// GMAC base (within FE)
    pub const GMAC_BASE: usize = FE_BASE + 0x10000;

    /// PDMA (Packet DMA) base
    pub const PDMA_BASE: usize = FE_BASE + 0x6000;

    /// QDMA base
    pub const QDMA_BASE: usize = FE_BASE + 0x4400;

    /// PPE (Packet Processing Engine) base
    pub const PPE_BASE: usize = FE_BASE + 0x2000;

    /// Frame Engine global registers
    pub const FE_GLO_CFG: usize = FE_BASE + 0x00;
    pub const FE_RST_GL: usize = FE_BASE + 0x04;
    pub const FE_INT_STATUS: usize = FE_BASE + 0x08;
    pub const FE_INT_ENABLE: usize = FE_BASE + 0x0C;

    /// GMAC registers (offset from GMAC_BASE)
    pub const GMAC_PIAC: usize = 0x04;       // PHY Indirect Access Control
    pub const GMAC_PORT_MCR: usize = 0x100;  // MAC Control Register
    pub const GMAC_PORT_STS: usize = 0x108;  // MAC Status

    /// MAC Control Register bits
    pub const MCR_FORCE_MODE: u32 = 1 << 15;
    pub const MCR_TX_EN: u32 = 1 << 14;
    pub const MCR_RX_EN: u32 = 1 << 13;
    pub const MCR_FORCE_LINK: u32 = 1 << 0;

    /// PDMA TX ring registers
    pub const PDMA_TX_BASE: usize = PDMA_BASE + 0x000;
    pub const PDMA_TX_CNT: usize = PDMA_BASE + 0x004;
    pub const PDMA_TX_CPU_IDX: usize = PDMA_BASE + 0x008;
    pub const PDMA_TX_DMA_IDX: usize = PDMA_BASE + 0x00C;

    /// PDMA RX ring registers
    pub const PDMA_RX_BASE: usize = PDMA_BASE + 0x100;
    pub const PDMA_RX_CNT: usize = PDMA_BASE + 0x104;
    pub const PDMA_RX_CPU_IDX: usize = PDMA_BASE + 0x108;
    pub const PDMA_RX_DMA_IDX: usize = PDMA_BASE + 0x10C;

    /// PDMA global config
    pub const PDMA_GLO_CFG: usize = PDMA_BASE + 0x204;
    pub const PDMA_RST_IDX: usize = PDMA_BASE + 0x208;

    /// PDMA global config bits
    pub const PDMA_TX_DMA_EN: u32 = 1 << 0;
    pub const PDMA_RX_DMA_EN: u32 = 1 << 2;
    pub const PDMA_TX_WB_DDONE: u32 = 1 << 6;
    pub const PDMA_RX_DMA_BUSY: u32 = 1 << 3;
    pub const PDMA_TX_DMA_BUSY: u32 = 1 << 1;
}

/// Number of TX/RX descriptors
const TX_RING_SIZE: usize = 64;
const RX_RING_SIZE: usize = 64;

/// DMA descriptor (32 bytes aligned)
#[repr(C, align(32))]
#[derive(Clone, Copy)]
pub struct DmaDescriptor {
    /// Buffer address (physical)
    pub buf_addr: u32,
    /// Buffer address high bits
    pub buf_addr_hi: u32,
    /// Control/Status word 1
    pub ctrl1: u32,
    /// Control/Status word 2
    pub ctrl2: u32,
    /// Reserved
    pub _reserved: [u32; 4],
}

impl DmaDescriptor {
    pub const fn new() -> Self {
        Self {
            buf_addr: 0,
            buf_addr_hi: 0,
            ctrl1: 0,
            ctrl2: 0,
            _reserved: [0; 4],
        }
    }
}

/// Descriptor control bits
mod desc {
    /// Descriptor owned by DMA
    pub const DDONE: u32 = 1 << 31;
    /// Last segment
    pub const LS: u32 = 1 << 30;
    /// First segment
    pub const FS: u32 = 1 << 29;
    /// Buffer length mask (16 bits)
    pub const LEN_MASK: u32 = 0xFFFF;
}

/// MAC address (6 bytes)
#[derive(Clone, Copy, Debug)]
pub struct MacAddr(pub [u8; 6]);

impl MacAddr {
    pub const BROADCAST: MacAddr = MacAddr([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);

    pub const fn new(b0: u8, b1: u8, b2: u8, b3: u8, b4: u8, b5: u8) -> Self {
        MacAddr([b0, b1, b2, b3, b4, b5])
    }
}

/// TX/RX ring buffers (statically allocated)
#[repr(C, align(4096))]
struct DmaRings {
    tx_ring: [DmaDescriptor; TX_RING_SIZE],
    rx_ring: [DmaDescriptor; RX_RING_SIZE],
}

static mut DMA_RINGS: DmaRings = DmaRings {
    tx_ring: [DmaDescriptor::new(); TX_RING_SIZE],
    rx_ring: [DmaDescriptor::new(); RX_RING_SIZE],
};

/// Packet buffers
#[repr(C, align(4096))]
struct PacketBuffers {
    tx_buffers: [[u8; 2048]; TX_RING_SIZE],
    rx_buffers: [[u8; 2048]; RX_RING_SIZE],
}

static mut PACKET_BUFFERS: PacketBuffers = PacketBuffers {
    tx_buffers: [[0; 2048]; TX_RING_SIZE],
    rx_buffers: [[0; 2048]; RX_RING_SIZE],
};

/// Ethernet driver state
pub struct EthDriver {
    /// Base address
    base: usize,
    /// Our MAC address
    mac_addr: MacAddr,
    /// TX ring index
    tx_idx: usize,
    /// RX ring index
    rx_idx: usize,
    /// Initialized flag
    initialized: bool,
}

impl EthDriver {
    pub const fn new() -> Self {
        Self {
            base: regs::FE_BASE,
            mac_addr: MacAddr::new(0x02, 0x00, 0x00, 0x00, 0x00, 0x01), // Local MAC
            tx_idx: 0,
            rx_idx: 0,
            initialized: false,
        }
    }

    /// Read a register
    #[inline]
    unsafe fn read_reg(&self, addr: usize) -> u32 {
        read_volatile(addr as *const u32)
    }

    /// Write a register
    #[inline]
    unsafe fn write_reg(&self, addr: usize, val: u32) {
        write_volatile(addr as *mut u32, val);
    }

    /// Initialize the ethernet controller
    pub fn init(&mut self) -> Result<(), &'static str> {
        println!("    Initializing MT7988 Ethernet...");

        unsafe {
            // Reset frame engine
            self.write_reg(regs::FE_RST_GL, 0x1);
            for _ in 0..10000 {
                core::hint::spin_loop();
            }
            self.write_reg(regs::FE_RST_GL, 0x0);

            // Initialize DMA rings
            self.init_dma_rings()?;

            // Configure GMAC
            self.init_gmac()?;

            // Enable DMA
            self.enable_dma();
        }

        self.initialized = true;
        println!("    [OK] Ethernet initialized");
        println!("    MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            self.mac_addr.0[0], self.mac_addr.0[1], self.mac_addr.0[2],
            self.mac_addr.0[3], self.mac_addr.0[4], self.mac_addr.0[5]);

        Ok(())
    }

    /// Initialize DMA descriptor rings
    unsafe fn init_dma_rings(&mut self) -> Result<(), &'static str> {
        let rings = &mut *core::ptr::addr_of_mut!(DMA_RINGS);
        let bufs = &mut *core::ptr::addr_of_mut!(PACKET_BUFFERS);

        // Initialize TX ring
        for i in 0..TX_RING_SIZE {
            rings.tx_ring[i].buf_addr = bufs.tx_buffers[i].as_ptr() as u32;
            rings.tx_ring[i].buf_addr_hi = 0;
            rings.tx_ring[i].ctrl1 = 0; // Not owned by DMA yet
            rings.tx_ring[i].ctrl2 = 0;
        }

        // Initialize RX ring
        for i in 0..RX_RING_SIZE {
            rings.rx_ring[i].buf_addr = bufs.rx_buffers[i].as_ptr() as u32;
            rings.rx_ring[i].buf_addr_hi = 0;
            // Mark as owned by DMA, ready to receive
            rings.rx_ring[i].ctrl1 = desc::DDONE | (2048 & desc::LEN_MASK);
            rings.rx_ring[i].ctrl2 = 0;
        }

        // Set up PDMA registers
        let tx_ring_addr = rings.tx_ring.as_ptr() as u32;
        let rx_ring_addr = rings.rx_ring.as_ptr() as u32;

        self.write_reg(regs::PDMA_TX_BASE, tx_ring_addr);
        self.write_reg(regs::PDMA_TX_CNT, TX_RING_SIZE as u32);
        self.write_reg(regs::PDMA_TX_CPU_IDX, 0);

        self.write_reg(regs::PDMA_RX_BASE, rx_ring_addr);
        self.write_reg(regs::PDMA_RX_CNT, RX_RING_SIZE as u32);
        self.write_reg(regs::PDMA_RX_CPU_IDX, (RX_RING_SIZE - 1) as u32);

        Ok(())
    }

    /// Initialize GMAC
    unsafe fn init_gmac(&mut self) -> Result<(), &'static str> {
        // Set MAC control register
        // Force mode, enable TX/RX, force link up
        let mcr = regs::MCR_FORCE_MODE | regs::MCR_TX_EN | regs::MCR_RX_EN | regs::MCR_FORCE_LINK;
        self.write_reg(regs::GMAC_BASE + regs::GMAC_PORT_MCR, mcr);

        // TODO: Configure PHY via MDIO
        // TODO: Set up MAC address filter

        Ok(())
    }

    /// Enable DMA engine
    unsafe fn enable_dma(&mut self) {
        let cfg = regs::PDMA_TX_DMA_EN | regs::PDMA_RX_DMA_EN | regs::PDMA_TX_WB_DDONE;
        self.write_reg(regs::PDMA_GLO_CFG, cfg);
    }

    /// Transmit a packet
    pub fn send(&mut self, data: &[u8]) -> Result<(), &'static str> {
        if !self.initialized {
            return Err("Ethernet not initialized");
        }

        if data.len() > 2048 - ETH_HEADER_SIZE {
            return Err("Packet too large");
        }

        unsafe {
            let rings = &mut *core::ptr::addr_of_mut!(DMA_RINGS);
            let bufs = &mut *core::ptr::addr_of_mut!(PACKET_BUFFERS);

            let idx = self.tx_idx;
            let desc = &mut rings.tx_ring[idx];

            // Check if descriptor is available (not owned by DMA)
            if desc.ctrl1 & desc::DDONE != 0 {
                return Err("TX ring full");
            }

            // Copy data to buffer
            let buf = &mut bufs.tx_buffers[idx];
            buf[..data.len()].copy_from_slice(data);

            // Set up descriptor
            desc.ctrl1 = desc::DDONE | desc::FS | desc::LS | (data.len() as u32 & desc::LEN_MASK);

            // Memory barrier
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            // Update TX index
            self.tx_idx = (idx + 1) % TX_RING_SIZE;
            self.write_reg(regs::PDMA_TX_CPU_IDX, self.tx_idx as u32);
        }

        Ok(())
    }

    /// Receive a packet (non-blocking)
    /// Returns the packet data if available
    pub fn recv(&mut self, buf: &mut [u8]) -> Option<usize> {
        if !self.initialized {
            return None;
        }

        unsafe {
            let rings = &mut *core::ptr::addr_of_mut!(DMA_RINGS);
            let bufs = &*core::ptr::addr_of!(PACKET_BUFFERS);

            let idx = self.rx_idx;
            let desc = &mut rings.rx_ring[idx];

            // Check if packet received (descriptor done)
            if desc.ctrl1 & desc::DDONE != 0 {
                return None; // Still owned by DMA
            }

            // Get packet length
            let len = (desc.ctrl1 & desc::LEN_MASK) as usize;
            let copy_len = core::cmp::min(len, buf.len());

            // Copy data from buffer
            buf[..copy_len].copy_from_slice(&bufs.rx_buffers[idx][..copy_len]);

            // Return descriptor to DMA
            desc.ctrl1 = desc::DDONE | (2048 & desc::LEN_MASK);

            // Memory barrier
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            // Update RX index
            self.rx_idx = (idx + 1) % RX_RING_SIZE;
            self.write_reg(regs::PDMA_RX_CPU_IDX, idx as u32);

            Some(copy_len)
        }
    }

    /// Get link status
    pub fn link_status(&self) -> bool {
        if !self.initialized {
            return false;
        }

        unsafe {
            let status = self.read_reg(regs::GMAC_BASE + regs::GMAC_PORT_STS);
            // Bit 0 is typically link status
            (status & 0x1) != 0
        }
    }

    /// Get MAC address
    pub fn mac_addr(&self) -> MacAddr {
        self.mac_addr
    }
}

/// Global ethernet driver instance
static mut ETH_DRIVER: EthDriver = EthDriver::new();

/// Get the ethernet driver
pub unsafe fn driver() -> &'static mut EthDriver {
    &mut *core::ptr::addr_of_mut!(ETH_DRIVER)
}

/// Initialize ethernet
pub fn init() -> Result<(), &'static str> {
    unsafe { driver().init() }
}

/// Send a packet
pub fn send(data: &[u8]) -> Result<(), &'static str> {
    unsafe { driver().send(data) }
}

/// Receive a packet
pub fn recv(buf: &mut [u8]) -> Option<usize> {
    unsafe { driver().recv(buf) }
}

/// Get link status
pub fn link_status() -> bool {
    unsafe { driver().link_status() }
}

/// Test ethernet (basic init check)
pub fn test() {
    println!("  Testing ethernet...");

    // Just check if we can read registers without crashing
    // Real testing requires network connectivity
    unsafe {
        let fe_cfg = read_volatile(regs::FE_GLO_CFG as *const u32);
        println!("    FE_GLO_CFG: 0x{:08x}", fe_cfg);
    }

    println!("    Note: Full ethernet test requires network connection");
    println!("    [OK] Ethernet registers accessible");
}
