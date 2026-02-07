//! MT7988A I2C Controller Driver
//!
//! Provides low-level I2C bus access for the MediaTek MT7988A SoC.
//! Uses polling mode (no DMA) for simplicity.

use crate::arch::aarch64::mmio::MmioRegion;

/// MT7988A I2C controller base addresses
const I2C0_BASE: usize = 0x11003000;
const I2C1_BASE: usize = 0x11004000;
const I2C2_BASE: usize = 0x11005000;

/// MT7988A I2C register offsets (v3 layout)
mod regs {
    pub const DATA_PORT: usize = 0x00;
    pub const INTR_MASK: usize = 0x08;
    pub const INTR_STAT: usize = 0x0C;
    pub const CONTROL: usize = 0x10;
    pub const TRANSFER_LEN: usize = 0x14;
    pub const TRANSAC_LEN: usize = 0x18;
    pub const DELAY_LEN: usize = 0x1C;
    pub const TIMING: usize = 0x20;
    pub const START: usize = 0x24;
    pub const EXT_CONF: usize = 0x28;
    pub const LTIMING: usize = 0x2C;
    pub const HS: usize = 0x30;
    pub const IO_CONFIG: usize = 0x34;
    pub const FIFO_ADDR_CLR: usize = 0x38;
    pub const SDA_TIMING: usize = 0x3C;
    pub const SLAVE_ADDR: usize = 0x94;  // v3 specific offset!
    pub const FIFO_STAT: usize = 0xF4;
    pub const SOFTRESET: usize = 0x50;
}

/// Control register bits
mod ctrl {
    pub const RS: u32 = 1 << 1;              // Repeated start
    pub const DMA_EN: u32 = 1 << 2;          // DMA enable
    pub const CLK_EXT_EN: u32 = 1 << 3;      // Clock extension
    pub const DIR_CHANGE: u32 = 1 << 4;      // Direction change
    pub const ACKERR_DET_EN: u32 = 1 << 5;   // ACK error detect
    pub const TRANSFER_LEN_CHANGE: u32 = 1 << 6;
}

/// Interrupt status bits
mod intr {
    pub const TRANSAC_COMP: u32 = 1 << 0;    // Transaction complete
    pub const ACKERR: u32 = 1 << 1;          // ACK error
    pub const HS_NACKERR: u32 = 1 << 2;      // High-speed NACK error
}

/// I2C controller instance
pub struct I2cController {
    mmio: MmioRegion,
}

impl I2cController {
    /// Create controller for specified bus
    pub fn new(bus: u32) -> Option<Self> {
        let base = match bus {
            0 => I2C0_BASE,
            1 => I2C1_BASE,
            2 => I2C2_BASE,
            _ => return None,
        };

        Some(Self {
            mmio: MmioRegion::new(base),
        })
    }

    /// Initialize the I2C controller
    pub fn init(&self) {
        // Soft reset
        self.mmio.write32(regs::SOFTRESET, 1);

        // Small delay
        for _ in 0..1000 {
            unsafe { core::arch::asm!("nop") };
        }

        // Clear soft reset
        self.mmio.write32(regs::SOFTRESET, 0);

        // Configure timing for ~100kHz (assuming 26MHz clock)
        // TIMING register: [15:8] = high time, [7:0] = low time
        // For 100kHz with 26MHz source: period = 260 clocks, half = 130
        self.mmio.write32(regs::TIMING, (130 << 8) | 130);

        // Configure IO - use open drain mode
        self.mmio.write32(regs::IO_CONFIG, 0x3);  // SDA and SCL open drain

        // Clear interrupts
        self.mmio.write32(regs::INTR_STAT, 0xFFFF);

        // Disable all interrupts (we'll poll)
        self.mmio.write32(regs::INTR_MASK, 0);
    }

    /// Wait for transaction complete or error
    fn wait_complete(&self) -> Result<(), i32> {
        for _ in 0..100000 {
            let stat = self.mmio.read32(regs::INTR_STAT);

            if (stat & intr::ACKERR) != 0 {
                // Clear and return error
                self.mmio.write32(regs::INTR_STAT, intr::ACKERR);
                return Err(-5);  // EIO
            }

            if (stat & intr::TRANSAC_COMP) != 0 {
                // Clear and return success
                self.mmio.write32(regs::INTR_STAT, intr::TRANSAC_COMP);
                return Ok(());
            }

            // Small delay
            for _ in 0..10 {
                unsafe { core::arch::asm!("nop") };
            }
        }

        Err(-110)  // ETIMEDOUT
    }

    /// Write data to I2C device
    pub fn write(&self, addr: u8, data: &[u8]) -> Result<(), i32> {
        if data.is_empty() || data.len() > 8 {
            return Err(-22);  // EINVAL - only support up to 8 bytes
        }

        // Clear FIFO
        self.mmio.write32(regs::FIFO_ADDR_CLR, 1);

        // Set slave address (write mode = address << 1)
        self.mmio.write32(regs::SLAVE_ADDR, (addr as u32) << 1);

        // Set transfer length
        self.mmio.write32(regs::TRANSFER_LEN, data.len() as u32);

        // Set transaction length (1 message)
        self.mmio.write32(regs::TRANSAC_LEN, 1);

        // Control: enable ACK error detection, no DMA
        self.mmio.write32(regs::CONTROL, ctrl::ACKERR_DET_EN);

        // Write data to FIFO
        for &byte in data {
            self.mmio.write32(regs::DATA_PORT, byte as u32);
        }

        // Clear any pending interrupts
        self.mmio.write32(regs::INTR_STAT, 0xFFFF);

        // Start transfer
        self.mmio.write32(regs::START, 1);

        // Wait for completion
        self.wait_complete()
    }

    /// Read data from I2C device
    pub fn read(&self, addr: u8, data: &mut [u8]) -> Result<(), i32> {
        if data.is_empty() || data.len() > 8 {
            return Err(-22);  // EINVAL
        }

        // Clear FIFO
        self.mmio.write32(regs::FIFO_ADDR_CLR, 1);

        // Set slave address (read mode = (address << 1) | 1)
        self.mmio.write32(regs::SLAVE_ADDR, ((addr as u32) << 1) | 1);

        // Set transfer length
        self.mmio.write32(regs::TRANSFER_LEN, data.len() as u32);

        // Set transaction length (1 message)
        self.mmio.write32(regs::TRANSAC_LEN, 1);

        // Control: enable ACK error detection, no DMA
        self.mmio.write32(regs::CONTROL, ctrl::ACKERR_DET_EN);

        // Clear any pending interrupts
        self.mmio.write32(regs::INTR_STAT, 0xFFFF);

        // Start transfer
        self.mmio.write32(regs::START, 1);

        // Wait for completion
        self.wait_complete()?;

        // Read data from FIFO
        for byte in data.iter_mut() {
            *byte = self.mmio.read32(regs::DATA_PORT) as u8;
        }

        Ok(())
    }

    /// Write then read (common I2C pattern)
    pub fn write_read(&self, addr: u8, write_data: &[u8], read_data: &mut [u8]) -> Result<(), i32> {
        if write_data.is_empty() || write_data.len() > 8 {
            return Err(-22);
        }
        if read_data.is_empty() || read_data.len() > 8 {
            return Err(-22);
        }

        // Clear FIFO
        self.mmio.write32(regs::FIFO_ADDR_CLR, 1);

        // Set slave address (write first)
        self.mmio.write32(regs::SLAVE_ADDR, (addr as u32) << 1);

        // Set transfer lengths for write phase
        self.mmio.write32(regs::TRANSFER_LEN, write_data.len() as u32);

        // For write-then-read, we need 2 transactions with repeated start
        self.mmio.write32(regs::TRANSAC_LEN, 2);

        // Control: repeated start, direction change, ACK error detection
        self.mmio.write32(regs::CONTROL, ctrl::RS | ctrl::DIR_CHANGE | ctrl::ACKERR_DET_EN | ctrl::TRANSFER_LEN_CHANGE);

        // Write data to FIFO
        for &byte in write_data {
            self.mmio.write32(regs::DATA_PORT, byte as u32);
        }

        // Clear any pending interrupts
        self.mmio.write32(regs::INTR_STAT, 0xFFFF);

        // Start transfer
        self.mmio.write32(regs::START, 1);

        // Wait for completion
        self.wait_complete()?;

        // Read data from FIFO
        for byte in read_data.iter_mut() {
            *byte = self.mmio.read32(regs::DATA_PORT) as u8;
        }

        Ok(())
    }
}

use crate::kernel::lock::SpinLock;

/// I2C subsystem state (protected by SpinLock for SMP safety)
struct I2cState {
    controllers: [Option<I2cController>; 3],
    initialized: [bool; 3],
}

impl I2cState {
    const fn new() -> Self {
        Self {
            controllers: [None, None, None],
            initialized: [false, false, false],
        }
    }
}

/// Global I2C state protected by SpinLock
static I2C: SpinLock<I2cState> = SpinLock::new(0, I2cState::new());

/// Get I2C controller for bus and perform operation under lock
/// Returns None if bus is invalid or initialization failed
pub fn with_controller<R, F: FnOnce(&I2cController) -> R>(bus: u32, f: F) -> Option<R> {
    if bus > 2 {
        return None;
    }

    let mut guard = I2C.lock();
    let idx = bus as usize;

    // Initialize on first use
    if !guard.initialized[idx] {
        if let Some(ctrl) = I2cController::new(bus) {
            ctrl.init();
            guard.controllers[idx] = Some(ctrl);
            guard.initialized[idx] = true;
        }
    }

    // Call the user function with the controller
    guard.controllers[idx].as_ref().map(f)
}
