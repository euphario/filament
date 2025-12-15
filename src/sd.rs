//! MT7988 SD/eMMC Driver
//!
//! Basic SD/eMMC driver for MediaTek MT7988's MSDC controller.
//! Supports basic block read/write operations.

use core::ptr::{read_volatile, write_volatile};
use crate::println;

/// Block size (always 512 for SD/eMMC)
pub const BLOCK_SIZE: usize = 512;

/// MT7988 MSDC (SD/MMC controller) registers
mod regs {
    /// MSDC0 base address (for eMMC)
    pub const MSDC0_BASE: usize = 0x11230000;
    /// MSDC1 base address (for SD card)
    pub const MSDC1_BASE: usize = 0x11240000;

    /// Register offsets
    pub const MSDC_CFG: usize = 0x00;        // Configuration
    pub const MSDC_IOCON: usize = 0x04;      // IO Configuration
    pub const MSDC_PS: usize = 0x08;         // Pin Status
    pub const MSDC_INT: usize = 0x0C;        // Interrupt
    pub const MSDC_INTEN: usize = 0x10;      // Interrupt Enable
    pub const MSDC_FIFOCS: usize = 0x14;     // FIFO Control/Status
    pub const MSDC_TXDATA: usize = 0x18;     // TX Data
    pub const MSDC_RXDATA: usize = 0x1C;     // RX Data
    pub const SDC_CFG: usize = 0x20;         // SD Configuration
    pub const SDC_CMD: usize = 0x24;         // SD Command
    pub const SDC_ARG: usize = 0x28;         // SD Argument
    pub const SDC_STS: usize = 0x2C;         // SD Status
    pub const SDC_RESP0: usize = 0x30;       // Response 0
    pub const SDC_RESP1: usize = 0x34;       // Response 1
    pub const SDC_RESP2: usize = 0x38;       // Response 2
    pub const SDC_RESP3: usize = 0x3C;       // Response 3
    pub const SDC_BLK_NUM: usize = 0x40;     // Block Number
    pub const SDC_VOL_CHG: usize = 0x44;     // Voltage Change
    pub const SDC_CSTS: usize = 0x48;        // Card Status
    pub const SDC_CSTS_EN: usize = 0x4C;     // Card Status Enable
    pub const SDC_DCRC_STS: usize = 0x50;    // Data CRC Status

    /// MSDC_CFG bits
    pub const MSDC_CFG_RST: u32 = 1 << 3;
    pub const MSDC_CFG_CKPDN: u32 = 1 << 1;
    pub const MSDC_CFG_MODE: u32 = 1 << 0;   // 0=MMC, 1=SD

    /// SDC_CMD bits
    pub const SDC_CMD_STOP: u32 = 1 << 14;
    pub const SDC_CMD_WR: u32 = 1 << 13;
    pub const SDC_CMD_DTYPE_SINGLE: u32 = 1 << 11;
    pub const SDC_CMD_DTYPE_MULTI: u32 = 2 << 11;
    pub const SDC_CMD_RSPTYP_NONE: u32 = 0 << 7;
    pub const SDC_CMD_RSPTYP_R1: u32 = 1 << 7;
    pub const SDC_CMD_RSPTYP_R2: u32 = 2 << 7;
    pub const SDC_CMD_RSPTYP_R3: u32 = 3 << 7;
    pub const SDC_CMD_RSPTYP_R1B: u32 = 7 << 7;

    /// SDC_STS bits
    pub const SDC_STS_CMDBUSY: u32 = 1 << 1;
    pub const SDC_STS_SDCBUSY: u32 = 1 << 0;

    /// MSDC_INT bits
    pub const MSDC_INT_CMDRDY: u32 = 1 << 8;
    pub const MSDC_INT_CMDTMO: u32 = 1 << 9;
    pub const MSDC_INT_RSPCRCERR: u32 = 1 << 10;
    pub const MSDC_INT_XFER_COMPL: u32 = 1 << 12;
    pub const MSDC_INT_DATTMO: u32 = 1 << 14;
    pub const MSDC_INT_DATCRCERR: u32 = 1 << 15;

    /// MSDC_PS bits
    pub const MSDC_PS_CDEN: u32 = 1 << 0;    // Card detect enable
    pub const MSDC_PS_CDSTS: u32 = 1 << 1;   // Card detect status
}

/// SD card commands
mod cmd {
    pub const GO_IDLE_STATE: u32 = 0;
    pub const SEND_OP_COND: u32 = 1;
    pub const ALL_SEND_CID: u32 = 2;
    pub const SEND_RELATIVE_ADDR: u32 = 3;
    pub const SET_BUS_WIDTH: u32 = 6;
    pub const SELECT_CARD: u32 = 7;
    pub const SEND_IF_COND: u32 = 8;
    pub const SEND_CSD: u32 = 9;
    pub const SEND_CID: u32 = 10;
    pub const STOP_TRANSMISSION: u32 = 12;
    pub const SET_BLOCKLEN: u32 = 16;
    pub const READ_SINGLE_BLOCK: u32 = 17;
    pub const READ_MULTIPLE_BLOCK: u32 = 18;
    pub const WRITE_SINGLE_BLOCK: u32 = 24;
    pub const WRITE_MULTIPLE_BLOCK: u32 = 25;
    pub const APP_CMD: u32 = 55;
    pub const SD_APP_OP_COND: u32 = 41;  // ACMD41
}

/// SD card type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CardType {
    Unknown,
    SdV1,
    SdV2,
    SdHC,
    Mmc,
    Emmc,
}

/// SD driver state
pub struct SdDriver {
    /// Base address (MSDC0 or MSDC1)
    base: usize,
    /// Card type detected
    card_type: CardType,
    /// Relative Card Address
    rca: u32,
    /// Card capacity in blocks
    capacity: u64,
    /// Initialized flag
    initialized: bool,
}

impl SdDriver {
    pub const fn new(base: usize) -> Self {
        Self {
            base,
            card_type: CardType::Unknown,
            rca: 0,
            capacity: 0,
            initialized: false,
        }
    }

    /// Read a register
    #[inline]
    unsafe fn read_reg(&self, offset: usize) -> u32 {
        read_volatile((self.base + offset) as *const u32)
    }

    /// Write a register
    #[inline]
    unsafe fn write_reg(&self, offset: usize, val: u32) {
        write_volatile((self.base + offset) as *mut u32, val);
    }

    /// Wait for command to complete
    unsafe fn wait_cmd_done(&self) -> Result<(), &'static str> {
        for _ in 0..100000 {
            let int_status = self.read_reg(regs::MSDC_INT);
            if int_status & regs::MSDC_INT_CMDRDY != 0 {
                self.write_reg(regs::MSDC_INT, regs::MSDC_INT_CMDRDY);
                return Ok(());
            }
            if int_status & regs::MSDC_INT_CMDTMO != 0 {
                self.write_reg(regs::MSDC_INT, regs::MSDC_INT_CMDTMO);
                return Err("Command timeout");
            }
            if int_status & regs::MSDC_INT_RSPCRCERR != 0 {
                self.write_reg(regs::MSDC_INT, regs::MSDC_INT_RSPCRCERR);
                return Err("Response CRC error");
            }
            core::hint::spin_loop();
        }
        Err("Command timeout (loop)")
    }

    /// Wait for data transfer to complete
    unsafe fn wait_data_done(&self) -> Result<(), &'static str> {
        for _ in 0..1000000 {
            let int_status = self.read_reg(regs::MSDC_INT);
            if int_status & regs::MSDC_INT_XFER_COMPL != 0 {
                self.write_reg(regs::MSDC_INT, regs::MSDC_INT_XFER_COMPL);
                return Ok(());
            }
            if int_status & regs::MSDC_INT_DATTMO != 0 {
                self.write_reg(regs::MSDC_INT, regs::MSDC_INT_DATTMO);
                return Err("Data timeout");
            }
            if int_status & regs::MSDC_INT_DATCRCERR != 0 {
                self.write_reg(regs::MSDC_INT, regs::MSDC_INT_DATCRCERR);
                return Err("Data CRC error");
            }
            core::hint::spin_loop();
        }
        Err("Data transfer timeout")
    }

    /// Send a command
    unsafe fn send_cmd(&self, cmd_idx: u32, arg: u32, resp_type: u32) -> Result<u32, &'static str> {
        // Wait for controller ready
        while self.read_reg(regs::SDC_STS) & (regs::SDC_STS_CMDBUSY | regs::SDC_STS_SDCBUSY) != 0 {
            core::hint::spin_loop();
        }

        // Clear interrupts
        self.write_reg(regs::MSDC_INT, 0xFFFFFFFF);

        // Set argument
        self.write_reg(regs::SDC_ARG, arg);

        // Send command
        let cmd_val = cmd_idx | resp_type;
        self.write_reg(regs::SDC_CMD, cmd_val);

        // Wait for completion
        self.wait_cmd_done()?;

        // Return response
        Ok(self.read_reg(regs::SDC_RESP0))
    }

    /// Send application command (preceded by CMD55)
    unsafe fn send_acmd(&self, cmd_idx: u32, arg: u32, resp_type: u32) -> Result<u32, &'static str> {
        self.send_cmd(cmd::APP_CMD, self.rca << 16, regs::SDC_CMD_RSPTYP_R1)?;
        self.send_cmd(cmd_idx, arg, resp_type)
    }

    /// Initialize the SD controller and card
    pub fn init(&mut self) -> Result<(), &'static str> {
        println!("    Initializing SD/MMC controller...");

        unsafe {
            // Reset controller
            let cfg = self.read_reg(regs::MSDC_CFG);
            self.write_reg(regs::MSDC_CFG, cfg | regs::MSDC_CFG_RST);
            for _ in 0..10000 {
                core::hint::spin_loop();
            }
            self.write_reg(regs::MSDC_CFG, cfg & !regs::MSDC_CFG_RST);

            // Enable card detect
            let ps = self.read_reg(regs::MSDC_PS);
            self.write_reg(regs::MSDC_PS, ps | regs::MSDC_PS_CDEN);

            // Check card presence
            let ps = self.read_reg(regs::MSDC_PS);
            if ps & regs::MSDC_PS_CDSTS != 0 {
                println!("    No card detected");
                return Err("No card");
            }

            // Initialize card
            self.init_card()?;
        }

        self.initialized = true;
        println!("    [OK] SD card initialized");
        println!("    Type: {:?}", self.card_type);
        println!("    Capacity: {} MB", self.capacity * BLOCK_SIZE as u64 / (1024 * 1024));

        Ok(())
    }

    /// Initialize the card (SD protocol)
    unsafe fn init_card(&mut self) -> Result<(), &'static str> {
        // GO_IDLE_STATE (CMD0)
        self.send_cmd(cmd::GO_IDLE_STATE, 0, regs::SDC_CMD_RSPTYP_NONE)?;

        // SEND_IF_COND (CMD8) - Check for SD v2
        let cmd8_result = self.send_cmd(cmd::SEND_IF_COND, 0x1AA, regs::SDC_CMD_RSPTYP_R1);
        let is_sd_v2 = cmd8_result.is_ok();

        // ACMD41 - SD_APP_OP_COND (initialize and get OCR)
        let ocr_arg = if is_sd_v2 { 0x40FF8000 } else { 0x00FF8000 };
        let mut ocr = 0u32;

        for _ in 0..1000 {
            match self.send_acmd(cmd::SD_APP_OP_COND, ocr_arg, regs::SDC_CMD_RSPTYP_R3) {
                Ok(resp) => {
                    ocr = resp;
                    if ocr & 0x80000000 != 0 {
                        // Card is ready
                        break;
                    }
                }
                Err(_) => {
                    // Try MMC init
                    return self.init_mmc();
                }
            }
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
        }

        if ocr & 0x80000000 == 0 {
            return Err("Card initialization failed");
        }

        // Determine card type
        self.card_type = if is_sd_v2 && (ocr & 0x40000000 != 0) {
            CardType::SdHC
        } else if is_sd_v2 {
            CardType::SdV2
        } else {
            CardType::SdV1
        };

        // ALL_SEND_CID (CMD2)
        self.send_cmd(cmd::ALL_SEND_CID, 0, regs::SDC_CMD_RSPTYP_R2)?;

        // SEND_RELATIVE_ADDR (CMD3)
        let resp = self.send_cmd(cmd::SEND_RELATIVE_ADDR, 0, regs::SDC_CMD_RSPTYP_R1)?;
        self.rca = resp >> 16;

        // SELECT_CARD (CMD7)
        self.send_cmd(cmd::SELECT_CARD, self.rca << 16, regs::SDC_CMD_RSPTYP_R1B)?;

        // Set block length (for non-SDHC cards)
        if self.card_type != CardType::SdHC {
            self.send_cmd(cmd::SET_BLOCKLEN, BLOCK_SIZE as u32, regs::SDC_CMD_RSPTYP_R1)?;
        }

        // TODO: Get actual capacity from CSD register
        self.capacity = 1024 * 1024; // Default 512MB

        Ok(())
    }

    /// Initialize as MMC/eMMC card
    unsafe fn init_mmc(&mut self) -> Result<(), &'static str> {
        // SEND_OP_COND (CMD1) for MMC
        for _ in 0..1000 {
            match self.send_cmd(cmd::SEND_OP_COND, 0x40FF8000, regs::SDC_CMD_RSPTYP_R3) {
                Ok(ocr) => {
                    if ocr & 0x80000000 != 0 {
                        self.card_type = CardType::Mmc;

                        // Continue initialization
                        self.send_cmd(cmd::ALL_SEND_CID, 0, regs::SDC_CMD_RSPTYP_R2)?;
                        self.rca = 1; // MMC uses assigned RCA
                        self.send_cmd(cmd::SEND_RELATIVE_ADDR, self.rca << 16, regs::SDC_CMD_RSPTYP_R1)?;
                        self.send_cmd(cmd::SELECT_CARD, self.rca << 16, regs::SDC_CMD_RSPTYP_R1B)?;
                        self.send_cmd(cmd::SET_BLOCKLEN, BLOCK_SIZE as u32, regs::SDC_CMD_RSPTYP_R1)?;
                        self.capacity = 1024 * 1024;
                        return Ok(());
                    }
                }
                Err(_) => break,
            }
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
        }
        Err("MMC initialization failed")
    }

    /// Read blocks from the card
    pub fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), &'static str> {
        if !self.initialized {
            return Err("SD not initialized");
        }

        if buf.len() % BLOCK_SIZE != 0 {
            return Err("Buffer size must be multiple of block size");
        }

        let num_blocks = buf.len() / BLOCK_SIZE;
        let addr = if self.card_type == CardType::SdHC {
            lba as u32
        } else {
            (lba * BLOCK_SIZE as u64) as u32
        };

        unsafe {
            // Set block count
            self.write_reg(regs::SDC_BLK_NUM, num_blocks as u32);

            // Send read command
            let cmd = if num_blocks == 1 {
                cmd::READ_SINGLE_BLOCK
            } else {
                cmd::READ_MULTIPLE_BLOCK
            };

            let cmd_val = cmd | regs::SDC_CMD_RSPTYP_R1 |
                if num_blocks == 1 { regs::SDC_CMD_DTYPE_SINGLE } else { regs::SDC_CMD_DTYPE_MULTI };

            self.write_reg(regs::SDC_ARG, addr);
            self.write_reg(regs::SDC_CMD, cmd_val);

            self.wait_cmd_done()?;

            // Read data from FIFO
            for block in 0..num_blocks {
                for word in 0..(BLOCK_SIZE / 4) {
                    // Wait for FIFO data
                    while self.read_reg(regs::MSDC_FIFOCS) & 0xFF == 0 {
                        core::hint::spin_loop();
                    }

                    let data = self.read_reg(regs::MSDC_RXDATA);
                    let offset = block * BLOCK_SIZE + word * 4;
                    buf[offset..offset + 4].copy_from_slice(&data.to_le_bytes());
                }
            }

            self.wait_data_done()?;

            // Stop transmission for multi-block
            if num_blocks > 1 {
                self.send_cmd(cmd::STOP_TRANSMISSION, 0, regs::SDC_CMD_RSPTYP_R1B)?;
            }
        }

        Ok(())
    }

    /// Write blocks to the card
    pub fn write_blocks(&mut self, lba: u64, buf: &[u8]) -> Result<(), &'static str> {
        if !self.initialized {
            return Err("SD not initialized");
        }

        if buf.len() % BLOCK_SIZE != 0 {
            return Err("Buffer size must be multiple of block size");
        }

        let num_blocks = buf.len() / BLOCK_SIZE;
        let addr = if self.card_type == CardType::SdHC {
            lba as u32
        } else {
            (lba * BLOCK_SIZE as u64) as u32
        };

        unsafe {
            // Set block count
            self.write_reg(regs::SDC_BLK_NUM, num_blocks as u32);

            // Send write command
            let cmd = if num_blocks == 1 {
                cmd::WRITE_SINGLE_BLOCK
            } else {
                cmd::WRITE_MULTIPLE_BLOCK
            };

            let cmd_val = cmd | regs::SDC_CMD_RSPTYP_R1 | regs::SDC_CMD_WR |
                if num_blocks == 1 { regs::SDC_CMD_DTYPE_SINGLE } else { regs::SDC_CMD_DTYPE_MULTI };

            self.write_reg(regs::SDC_ARG, addr);
            self.write_reg(regs::SDC_CMD, cmd_val);

            self.wait_cmd_done()?;

            // Write data to FIFO
            for block in 0..num_blocks {
                for word in 0..(BLOCK_SIZE / 4) {
                    // Wait for FIFO space
                    while self.read_reg(regs::MSDC_FIFOCS) & 0xFF00 == 0 {
                        core::hint::spin_loop();
                    }

                    let offset = block * BLOCK_SIZE + word * 4;
                    let data = u32::from_le_bytes([
                        buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]
                    ]);
                    self.write_reg(regs::MSDC_TXDATA, data);
                }
            }

            self.wait_data_done()?;

            // Stop transmission for multi-block
            if num_blocks > 1 {
                self.send_cmd(cmd::STOP_TRANSMISSION, 0, regs::SDC_CMD_RSPTYP_R1B)?;
            }
        }

        Ok(())
    }

    /// Get card capacity in blocks
    pub fn capacity(&self) -> u64 {
        self.capacity
    }

    /// Check if card is present
    pub fn card_present(&self) -> bool {
        unsafe {
            let ps = self.read_reg(regs::MSDC_PS);
            (ps & regs::MSDC_PS_CDSTS) == 0
        }
    }
}

/// Global SD driver instances
static mut SD_DRIVER: SdDriver = SdDriver::new(regs::MSDC1_BASE);  // SD card slot
static mut EMMC_DRIVER: SdDriver = SdDriver::new(regs::MSDC0_BASE);  // eMMC

/// Get SD card driver
pub unsafe fn sd_driver() -> &'static mut SdDriver {
    &mut *core::ptr::addr_of_mut!(SD_DRIVER)
}

/// Get eMMC driver
pub unsafe fn emmc_driver() -> &'static mut SdDriver {
    &mut *core::ptr::addr_of_mut!(EMMC_DRIVER)
}

/// Initialize SD card
pub fn init_sd() -> Result<(), &'static str> {
    unsafe { sd_driver().init() }
}

/// Initialize eMMC
pub fn init_emmc() -> Result<(), &'static str> {
    unsafe { emmc_driver().init() }
}

/// Test SD/eMMC
pub fn test() {
    println!("  Testing SD/eMMC...");

    // Just check if we can read registers without crashing
    unsafe {
        let cfg0 = read_volatile(regs::MSDC0_BASE as *const u32);
        let cfg1 = read_volatile(regs::MSDC1_BASE as *const u32);
        println!("    MSDC0 CFG: 0x{:08x}", cfg0);
        println!("    MSDC1 CFG: 0x{:08x}", cfg1);
    }

    println!("    Note: Full SD test requires card insertion");
    println!("    [OK] SD/eMMC registers accessible");
}
