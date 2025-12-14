//! MediaTek MT7988A UART driver (8250/16550 compatible)
//!
//! The MT7988A uses a standard 8250-style UART with 32-bit register spacing.
//! UART0 (debug console) is at base address 0x11002000.

#![allow(dead_code)]

use core::fmt::{self, Write};
use core::ptr::{read_volatile, write_volatile};

/// UART0 base address for MT7988A (confirmed working)
const UART_BASE: usize = 0x11000000;

/// UART register offsets (32-bit aligned, multiply standard 8250 offsets by 4)
mod regs {
    /// Transmit Holding Register (write) / Receive Buffer Register (read)
    pub const THR: usize = 0x00;
    /// Interrupt Enable Register
    pub const IER: usize = 0x04;
    /// FIFO Control Register
    pub const FCR: usize = 0x08;
    /// Line Control Register
    pub const LCR: usize = 0x0C;
    /// Modem Control Register
    pub const MCR: usize = 0x10;
    /// Line Status Register
    pub const LSR: usize = 0x14;
}

/// Line Status Register bits
mod lsr {
    /// Data Ready - set when data is available to read
    pub const DR: u32 = 1 << 0;
    /// Transmitter Holding Register Empty - safe to write new data
    pub const THRE: u32 = 1 << 5;
    /// Transmitter Empty - both THR and shift register are empty
    pub const TEMT: u32 = 1 << 6;
}

/// UART driver instance
pub struct Uart {
    base: usize,
}

impl Uart {
    /// Create a new UART instance at the default MT7988A address
    pub const fn new() -> Self {
        Self { base: UART_BASE }
    }

    /// Initialize the UART
    ///
    /// Note: U-Boot has already configured baud rate and other settings,
    /// so we just need to ensure the UART is in a known state.
    pub fn init(&self) {
        unsafe {
            // Disable all interrupts
            self.write_reg(regs::IER, 0x00);

            // Enable FIFO, clear buffers, set 1-byte trigger
            self.write_reg(regs::FCR, 0x07);

            // 8 data bits, 1 stop bit, no parity (8N1)
            self.write_reg(regs::LCR, 0x03);

            // Enable DTR and RTS
            self.write_reg(regs::MCR, 0x03);
        }
    }

    /// Write a single byte to the UART
    pub fn putc(&self, byte: u8) {
        unsafe {
            // Wait until transmit holding register is empty
            while (self.read_reg(regs::LSR) & lsr::THRE) == 0 {
                core::hint::spin_loop();
            }
            // Write the byte
            self.write_reg(regs::THR, byte as u32);
        }
    }

    /// Read a single byte from the UART (blocking)
    pub fn getc(&self) -> u8 {
        unsafe {
            // Wait until data is available
            while (self.read_reg(regs::LSR) & lsr::DR) == 0 {
                core::hint::spin_loop();
            }
            // Read the byte
            self.read_reg(regs::THR) as u8
        }
    }

    /// Check if data is available to read
    pub fn data_available(&self) -> bool {
        unsafe { (self.read_reg(regs::LSR) & lsr::DR) != 0 }
    }

    /// Write a string to the UART
    pub fn puts(&self, s: &str) {
        for byte in s.bytes() {
            if byte == b'\n' {
                self.putc(b'\r'); // Add carriage return for terminals
            }
            self.putc(byte);
        }
    }

    /// Write to a UART register
    #[inline]
    unsafe fn write_reg(&self, offset: usize, value: u32) {
        write_volatile((self.base + offset) as *mut u32, value);
    }

    /// Read from a UART register
    #[inline]
    unsafe fn read_reg(&self, offset: usize) -> u32 {
        read_volatile((self.base + offset) as *const u32)
    }
}

/// Implement Write trait for formatted printing
impl Write for Uart {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.puts(s);
        Ok(())
    }
}

/// Global UART instance (using raw pointer for Rust 2024 compatibility)
/// pub for macro access
pub static mut UART: Uart = Uart::new();

/// Initialize the global UART
pub fn init() {
    // SAFETY: Single-threaded bare-metal environment, called once at startup
    unsafe {
        (*core::ptr::addr_of_mut!(UART)).init();
    }
}

/// Print a string to the console
pub fn print(s: &str) {
    // SAFETY: Single-threaded bare-metal environment
    unsafe {
        (*core::ptr::addr_of_mut!(UART)).puts(s);
    }
}

/// Print a single character
pub fn putc(c: char) {
    // SAFETY: Single-threaded bare-metal environment
    unsafe {
        (*core::ptr::addr_of_mut!(UART)).putc(c as u8);
    }
}

/// Print formatted output to the console
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        // SAFETY: Single-threaded bare-metal environment
        unsafe {
            let _ = write!(&mut *core::ptr::addr_of_mut!($crate::uart::UART), $($arg)*);
        }
    }};
}

/// Print formatted output with newline
#[macro_export]
macro_rules! println {
    () => ($crate::print!("\n"));
    ($($arg:tt)*) => {{
        $crate::print!($($arg)*);
        $crate::print!("\n");
    }};
}

/// Get mutable reference to global UART (unsafe)
/// SAFETY: Caller must ensure single-threaded access
pub unsafe fn get_uart() -> &'static mut Uart {
    &mut *core::ptr::addr_of_mut!(UART)
}
