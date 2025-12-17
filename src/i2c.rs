//! MT7988A I2C Controller Driver
//!
//! Provides low-level I2C bus access for the MediaTek MT7988A SoC.
//! Uses polling mode (no DMA) for simplicity.

use crate::mmu::KERNEL_VIRT_BASE;

/// MT7988A I2C controller base addresses
const I2C0_BASE: u64 = 0x11003000;
const I2C1_BASE: u64 = 0x11004000;
const I2C2_BASE: u64 = 0x11005000;

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
    base: u64,  // Kernel virtual address
}

impl I2cController {
    /// Create controller for specified bus
    pub fn new(bus: u32) -> Option<Self> {
        let phys_base = match bus {
            0 => I2C0_BASE,
            1 => I2C1_BASE,
            2 => I2C2_BASE,
            _ => return None,
        };

        Some(Self {
            base: KERNEL_VIRT_BASE | phys_base,
        })
    }

    /// Read register
    #[inline(always)]
    fn read32(&self, offset: usize) -> u32 {
        unsafe {
            let ptr = (self.base + offset as u64) as *const u32;
            core::ptr::read_volatile(ptr)
        }
    }

    /// Write register
    #[inline(always)]
    fn write32(&self, offset: usize, value: u32) {
        unsafe {
            let ptr = (self.base + offset as u64) as *mut u32;
            core::ptr::write_volatile(ptr, value);
        }
    }

    /// Initialize the I2C controller
    pub fn init(&self) {
        // Soft reset
        self.write32(regs::SOFTRESET, 1);

        // Small delay
        for _ in 0..1000 {
            unsafe { core::arch::asm!("nop") };
        }

        // Clear soft reset
        self.write32(regs::SOFTRESET, 0);

        // Configure timing for ~100kHz (assuming 26MHz clock)
        // TIMING register: [15:8] = high time, [7:0] = low time
        // For 100kHz with 26MHz source: period = 260 clocks, half = 130
        self.write32(regs::TIMING, (130 << 8) | 130);

        // Configure IO - use open drain mode
        self.write32(regs::IO_CONFIG, 0x3);  // SDA and SCL open drain

        // Clear interrupts
        self.write32(regs::INTR_STAT, 0xFFFF);

        // Disable all interrupts (we'll poll)
        self.write32(regs::INTR_MASK, 0);
    }

    /// Wait for transaction complete or error
    fn wait_complete(&self) -> Result<(), i32> {
        for _ in 0..100000 {
            let stat = self.read32(regs::INTR_STAT);

            if (stat & intr::ACKERR) != 0 {
                // Clear and return error
                self.write32(regs::INTR_STAT, intr::ACKERR);
                return Err(-5);  // EIO
            }

            if (stat & intr::TRANSAC_COMP) != 0 {
                // Clear and return success
                self.write32(regs::INTR_STAT, intr::TRANSAC_COMP);
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
        self.write32(regs::FIFO_ADDR_CLR, 1);

        // Set slave address (write mode = address << 1)
        self.write32(regs::SLAVE_ADDR, (addr as u32) << 1);

        // Set transfer length
        self.write32(regs::TRANSFER_LEN, data.len() as u32);

        // Set transaction length (1 message)
        self.write32(regs::TRANSAC_LEN, 1);

        // Control: enable ACK error detection, no DMA
        self.write32(regs::CONTROL, ctrl::ACKERR_DET_EN);

        // Write data to FIFO
        for &byte in data {
            self.write32(regs::DATA_PORT, byte as u32);
        }

        // Clear any pending interrupts
        self.write32(regs::INTR_STAT, 0xFFFF);

        // Start transfer
        self.write32(regs::START, 1);

        // Wait for completion
        self.wait_complete()
    }

    /// Read data from I2C device
    pub fn read(&self, addr: u8, data: &mut [u8]) -> Result<(), i32> {
        if data.is_empty() || data.len() > 8 {
            return Err(-22);  // EINVAL
        }

        // Clear FIFO
        self.write32(regs::FIFO_ADDR_CLR, 1);

        // Set slave address (read mode = (address << 1) | 1)
        self.write32(regs::SLAVE_ADDR, ((addr as u32) << 1) | 1);

        // Set transfer length
        self.write32(regs::TRANSFER_LEN, data.len() as u32);

        // Set transaction length (1 message)
        self.write32(regs::TRANSAC_LEN, 1);

        // Control: enable ACK error detection, no DMA
        self.write32(regs::CONTROL, ctrl::ACKERR_DET_EN);

        // Clear any pending interrupts
        self.write32(regs::INTR_STAT, 0xFFFF);

        // Start transfer
        self.write32(regs::START, 1);

        // Wait for completion
        self.wait_complete()?;

        // Read data from FIFO
        for byte in data.iter_mut() {
            *byte = self.read32(regs::DATA_PORT) as u8;
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
        self.write32(regs::FIFO_ADDR_CLR, 1);

        // Set slave address (write first)
        self.write32(regs::SLAVE_ADDR, (addr as u32) << 1);

        // Set transfer lengths for write phase
        self.write32(regs::TRANSFER_LEN, write_data.len() as u32);

        // For write-then-read, we need 2 transactions with repeated start
        self.write32(regs::TRANSAC_LEN, 2);

        // Control: repeated start, direction change, ACK error detection
        self.write32(regs::CONTROL, ctrl::RS | ctrl::DIR_CHANGE | ctrl::ACKERR_DET_EN | ctrl::TRANSFER_LEN_CHANGE);

        // Write data to FIFO
        for &byte in write_data {
            self.write32(regs::DATA_PORT, byte as u32);
        }

        // Clear any pending interrupts
        self.write32(regs::INTR_STAT, 0xFFFF);

        // Start transfer
        self.write32(regs::START, 1);

        // Wait for completion
        self.wait_complete()?;

        // Read data from FIFO
        for byte in read_data.iter_mut() {
            *byte = self.read32(regs::DATA_PORT) as u8;
        }

        Ok(())
    }
}

/// Global I2C controllers (initialized on first use)
static mut I2C_CONTROLLERS: [Option<I2cController>; 3] = [None, None, None];
static mut I2C_INITIALIZED: [bool; 3] = [false, false, false];

/// Get I2C controller for bus
pub fn get_controller(bus: u32) -> Option<&'static I2cController> {
    if bus > 2 {
        return None;
    }

    unsafe {
        if !I2C_INITIALIZED[bus as usize] {
            if let Some(ctrl) = I2cController::new(bus) {
                ctrl.init();
                I2C_CONTROLLERS[bus as usize] = Some(ctrl);
                I2C_INITIALIZED[bus as usize] = true;
            }
        }

        I2C_CONTROLLERS[bus as usize].as_ref()
    }
}
