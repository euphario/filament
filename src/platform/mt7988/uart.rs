//! MediaTek MT7988A UART driver (8250/16550 compatible)
//!
//! The MT7988A uses a standard 8250-style UART with 32-bit register spacing.
//! Uses platform constants for base address.
//! Implements the HAL Uart trait for portability.
//!
//! Features buffered output - writes go to a ring buffer and are flushed
//! asynchronously (on timer tick) to avoid blocking userspace.

#![allow(dead_code)]

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicUsize, Ordering};
use crate::arch::aarch64::mmio::MmioRegion;
use crate::hal::Uart as UartTrait;
use super::UART0_BASE;

// ============================================================================
// Ring Buffer for buffered UART output
// ============================================================================

/// UART output buffer size (4KB)
const UART_BUFFER_SIZE: usize = 4096;

/// Ring buffer for UART output
struct UartBuffer {
    buffer: [u8; UART_BUFFER_SIZE],
    write_pos: AtomicUsize,
    read_pos: AtomicUsize,
}

impl UartBuffer {
    const fn new() -> Self {
        Self {
            buffer: [0; UART_BUFFER_SIZE],
            write_pos: AtomicUsize::new(0),
            read_pos: AtomicUsize::new(0),
        }
    }

    /// Write a byte to the ring buffer (non-blocking)
    #[inline]
    fn push(&mut self, byte: u8) -> bool {
        let write = self.write_pos.load(Ordering::Relaxed);
        let read = self.read_pos.load(Ordering::Acquire);
        let next_write = (write + 1) % UART_BUFFER_SIZE;

        if next_write == read {
            return false; // Buffer full
        }

        self.buffer[write] = byte;
        self.write_pos.store(next_write, Ordering::Release);
        true
    }

    /// Pop a byte from the buffer
    #[inline]
    fn pop(&mut self) -> Option<u8> {
        let write = self.write_pos.load(Ordering::Acquire);
        let read = self.read_pos.load(Ordering::Relaxed);

        if read == write {
            return None;
        }

        let byte = self.buffer[read];
        self.read_pos.store((read + 1) % UART_BUFFER_SIZE, Ordering::Release);
        Some(byte)
    }

    /// Check if buffer has data
    #[inline]
    fn has_data(&self) -> bool {
        self.write_pos.load(Ordering::Acquire) != self.read_pos.load(Ordering::Acquire)
    }
}

/// Global UART output buffer
static mut UART_BUFFER: UartBuffer = UartBuffer::new();

// ============================================================================
// UART Hardware Driver
// ============================================================================

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

/// Interrupt Enable Register bits
mod ier {
    /// Enable Received Data Available Interrupt
    pub const ERBFI: u32 = 1 << 0;
}

/// UART driver instance
pub struct Uart {
    regs: MmioRegion,
}

impl Uart {
    /// Create a new UART instance at the default MT7988A address
    pub const fn new() -> Self {
        Self { regs: MmioRegion::new(UART0_BASE) }
    }

    /// Initialize the UART
    ///
    /// Note: U-Boot has already configured baud rate and other settings,
    /// so we just need to ensure the UART is in a known state.
    pub fn init(&self) {
        // Disable all interrupts
        self.regs.write32(regs::IER, 0x00);

        // Enable FIFO, clear buffers, set 1-byte trigger
        self.regs.write32(regs::FCR, 0x07);

        // 8 data bits, 1 stop bit, no parity (8N1)
        self.regs.write32(regs::LCR, 0x03);

        // Enable DTR and RTS
        self.regs.write32(regs::MCR, 0x03);
    }

    /// Write a single byte to the UART
    pub fn putc(&self, byte: u8) {
        // Wait until transmit holding register is empty
        while (self.regs.read32(regs::LSR) & lsr::THRE) == 0 {
            core::hint::spin_loop();
        }
        // Write the byte
        self.regs.write32(regs::THR, byte as u32);
    }

    /// Read a single byte from the UART (blocking)
    /// Spins waiting for input - for cooperative multitasking, use try_getc() instead
    pub fn getc(&self) -> u8 {
        // Wait until data is available
        while (self.regs.read32(regs::LSR) & lsr::DR) == 0 {
            core::hint::spin_loop();
        }
        // Read the byte
        self.regs.read32(regs::THR) as u8
    }

    /// Check if data is available to read
    pub fn data_available(&self) -> bool {
        (self.regs.read32(regs::LSR) & lsr::DR) != 0
    }

    /// Try to read a single byte (non-blocking)
    /// Returns Some(byte) if data available, None otherwise
    pub fn try_getc(&self) -> Option<u8> {
        if self.data_available() {
            Some(self.regs.read32(regs::THR) as u8)
        } else {
            None
        }
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
}

/// Implement Write trait for formatted printing
impl Write for Uart {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.puts(s);
        Ok(())
    }
}

// ============================================================================
// HAL Uart Implementation
// ============================================================================

impl UartTrait for Uart {
    fn init(&self) {
        // Disable all interrupts
        self.regs.write32(regs::IER, 0x00);
        // Enable FIFO, clear buffers, set 1-byte trigger
        self.regs.write32(regs::FCR, 0x07);
        // 8 data bits, 1 stop bit, no parity (8N1)
        self.regs.write32(regs::LCR, 0x03);
        // Enable DTR and RTS
        self.regs.write32(regs::MCR, 0x03);
    }

    fn putc(&self, c: u8) {
        // Wait until transmit holding register is empty
        while (self.regs.read32(regs::LSR) & lsr::THRE) == 0 {
            core::hint::spin_loop();
        }
        self.regs.write32(regs::THR, c as u32);
    }

    fn getc(&self) -> Option<u8> {
        if (self.regs.read32(regs::LSR) & lsr::DR) != 0 {
            Some(self.regs.read32(regs::THR) as u8)
        } else {
            None
        }
    }

    fn tx_ready(&self) -> bool {
        (self.regs.read32(regs::LSR) & lsr::THRE) != 0
    }

    fn rx_ready(&self) -> bool {
        (self.regs.read32(regs::LSR) & lsr::DR) != 0
    }

    fn flush(&self) {
        // Wait until transmitter is empty
        while (self.regs.read32(regs::LSR) & lsr::TEMT) == 0 {
            core::hint::spin_loop();
        }
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

/// Try to read a character (non-blocking)
pub fn try_getc() -> Option<char> {
    // SAFETY: Single-threaded bare-metal environment
    unsafe {
        (*core::ptr::addr_of!(UART)).try_getc().map(|b| b as char)
    }
}

/// Read a character (blocking)
pub fn getc() -> char {
    // SAFETY: Single-threaded bare-metal environment
    unsafe {
        (*core::ptr::addr_of!(UART)).getc() as char
    }
}

/// Print formatted output to the console
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        // SAFETY: Single-threaded bare-metal environment
        unsafe {
            let _ = write!(&mut *core::ptr::addr_of_mut!($crate::platform::mt7988::uart::UART), $($arg)*);
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

// ============================================================================
// Buffered Output (for userspace writes via syscall)
// ============================================================================

/// Write a string to the output buffer (non-blocking)
/// Used by sys_write for STDOUT - doesn't block waiting for UART
pub fn print_buffered(s: &str) {
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(UART_BUFFER);
        for byte in s.bytes() {
            if byte == b'\n' {
                let _ = buf.push(b'\r');
            }
            if !buf.push(byte) {
                // Buffer full - flush and retry
                flush_buffer();
                let _ = buf.push(byte);
            }
        }
    }
}

/// Write raw bytes to the output buffer (non-blocking)
pub fn write_buffered(data: &[u8]) {
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(UART_BUFFER);
        for &byte in data {
            if !buf.push(byte) {
                // Buffer full - flush and retry
                flush_buffer();
                let _ = buf.push(byte);
            }
        }
    }
}

/// Flush the output buffer to UART hardware
/// Called from timer interrupt or explicitly
pub fn flush_buffer() {
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(UART_BUFFER);
        let uart = &*core::ptr::addr_of!(UART);

        // Drain up to 64 bytes per flush call to limit time spent
        // (will continue on next timer tick if more data)
        for _ in 0..64 {
            if let Some(byte) = buf.pop() {
                uart.putc(byte);
            } else {
                break;
            }
        }
    }
}

/// Check if output buffer has pending data
pub fn has_buffered_output() -> bool {
    unsafe {
        (*core::ptr::addr_of!(UART_BUFFER)).has_data()
    }
}

// ============================================================================
// UART RX Interrupt Support
// ============================================================================

use core::sync::atomic::AtomicU32;

/// PID of process blocked waiting for console input (0 = none)
static CONSOLE_BLOCKED_PID: AtomicU32 = AtomicU32::new(0);

/// Enable UART RX interrupt
pub fn enable_rx_interrupt() {
    unsafe {
        let uart = &*core::ptr::addr_of!(UART);
        // Enable RX data available interrupt
        uart.regs.write32(regs::IER, ier::ERBFI);
    }
    // Enable in GIC
    super::gic::enable_irq(super::irq::UART0);
}

/// Disable UART RX interrupt (called from IRQ handler to prevent re-triggering)
pub fn disable_rx_interrupt() {
    unsafe {
        let uart = &*core::ptr::addr_of!(UART);
        uart.regs.write32(regs::IER, 0);
    }
}

/// Re-enable UART RX interrupt (called after reading data)
pub fn reenable_rx_interrupt() {
    unsafe {
        let uart = &*core::ptr::addr_of!(UART);
        uart.regs.write32(regs::IER, ier::ERBFI);
    }
}

/// Register a process as blocked waiting for console input
/// Returns true if registered, false if another process is already waiting
pub fn block_for_input(pid: u32) -> bool {
    // Only one process can wait at a time
    CONSOLE_BLOCKED_PID.compare_exchange(
        0, pid,
        Ordering::SeqCst, Ordering::SeqCst
    ).is_ok()
}

/// Clear the blocked process (called when woken or process exits)
pub fn clear_blocked() {
    CONSOLE_BLOCKED_PID.store(0, Ordering::Release);
}

/// Get the PID waiting for console input (0 if none)
pub fn get_blocked_pid() -> u32 {
    CONSOLE_BLOCKED_PID.load(Ordering::Acquire)
}

/// Handle UART RX interrupt - returns true if it was handled
pub fn handle_rx_irq() -> bool {
    unsafe {
        let uart = &*core::ptr::addr_of!(UART);
        // Check if data is available
        if (uart.regs.read32(regs::LSR) & lsr::DR) != 0 {
            // Data available - clear interrupt by reading (but don't consume)
            // The blocked process will read the actual data
            return true;
        }
    }
    false
}

/// Get a reference to the UART as a Uart trait object
/// # Safety
/// Must only be called after init()
pub fn as_uart() -> &'static dyn UartTrait {
    unsafe { &*core::ptr::addr_of!(UART) }
}
