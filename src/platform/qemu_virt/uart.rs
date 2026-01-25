//! PL011 UART driver for QEMU virt
//!
//! The ARM PL011 is a simple UART with FIFO support.
//! QEMU's virt machine uses PL011 at 0x09000000.

#![allow(dead_code)]

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use crate::arch::aarch64::mmu;
use crate::hal::Uart as UartTrait;
use crate::kernel::lock::SpinLock;
use super::UART0_BASE;

// ============================================================================
// PL011 Register Offsets
// ============================================================================

/// Data Register
const UARTDR: usize = 0x000;
/// Receive Status / Error Clear
const UARTRSR: usize = 0x004;
/// Flag Register
const UARTFR: usize = 0x018;
/// Integer Baud Rate
const UARTIBRD: usize = 0x024;
/// Fractional Baud Rate
const UARTFBRD: usize = 0x028;
/// Line Control Register
const UARTLCR_H: usize = 0x02c;
/// Control Register
const UARTCR: usize = 0x030;
/// Interrupt FIFO Level Select
const UARTIFLS: usize = 0x034;
/// Interrupt Mask Set/Clear
const UARTIMSC: usize = 0x038;
/// Raw Interrupt Status
const UARTRIS: usize = 0x03c;
/// Masked Interrupt Status
const UARTMIS: usize = 0x040;
/// Interrupt Clear
const UARTICR: usize = 0x044;

// Flag Register bits
const FR_TXFE: u32 = 1 << 7;  // TX FIFO empty
const FR_RXFF: u32 = 1 << 6;  // RX FIFO full
const FR_TXFF: u32 = 1 << 5;  // TX FIFO full
const FR_RXFE: u32 = 1 << 4;  // RX FIFO empty
const FR_BUSY: u32 = 1 << 3;  // UART busy

// Control Register bits
const CR_RXE: u32 = 1 << 9;   // Receive enable
const CR_TXE: u32 = 1 << 8;   // Transmit enable
const CR_UARTEN: u32 = 1 << 0; // UART enable

// Line Control bits
const LCR_FEN: u32 = 1 << 4;  // FIFO enable
const LCR_WLEN_8: u32 = 3 << 5; // 8-bit word length

// ============================================================================
// Ring Buffers
// ============================================================================

const UART_BUFFER_SIZE: usize = 4096;
const UART_RX_BUFFER_SIZE: usize = 256;

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

    #[inline]
    fn push(&mut self, byte: u8) -> bool {
        let write = self.write_pos.load(Ordering::Relaxed);
        let read = self.read_pos.load(Ordering::Acquire);
        let next_write = (write + 1) % UART_BUFFER_SIZE;
        if next_write == read {
            return false;
        }
        self.buffer[write] = byte;
        self.write_pos.store(next_write, Ordering::Release);
        true
    }

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

    #[inline]
    fn has_data(&self) -> bool {
        self.write_pos.load(Ordering::Acquire) != self.read_pos.load(Ordering::Acquire)
    }
}

struct RxBuffer {
    buffer: [u8; UART_RX_BUFFER_SIZE],
    write_pos: AtomicUsize,
    read_pos: AtomicUsize,
}

impl RxBuffer {
    const fn new() -> Self {
        Self {
            buffer: [0; UART_RX_BUFFER_SIZE],
            write_pos: AtomicUsize::new(0),
            read_pos: AtomicUsize::new(0),
        }
    }

    #[inline]
    fn push(&mut self, byte: u8) -> bool {
        let write = self.write_pos.load(Ordering::Relaxed);
        let read = self.read_pos.load(Ordering::Acquire);
        let next_write = (write + 1) % UART_RX_BUFFER_SIZE;
        if next_write == read {
            return false;
        }
        self.buffer[write] = byte;
        self.write_pos.store(next_write, Ordering::Release);
        true
    }

    #[inline]
    fn pop(&mut self) -> Option<u8> {
        let write = self.write_pos.load(Ordering::Acquire);
        let read = self.read_pos.load(Ordering::Relaxed);
        if read == write {
            return None;
        }
        let byte = self.buffer[read];
        self.read_pos.store((read + 1) % UART_RX_BUFFER_SIZE, Ordering::Release);
        Some(byte)
    }

    #[inline]
    fn has_data(&self) -> bool {
        self.write_pos.load(Ordering::Acquire) != self.read_pos.load(Ordering::Acquire)
    }
}

static UART_BUFFER: SpinLock<UartBuffer> = SpinLock::new(UartBuffer::new());
static RX_BUFFER: SpinLock<RxBuffer> = SpinLock::new(RxBuffer::new());
static INITIALIZED: AtomicBool = AtomicBool::new(false);

use core::sync::atomic::AtomicU32;

/// PID of process blocked waiting for console input (0 = none)
static CONSOLE_BLOCKED_PID: AtomicU32 = AtomicU32::new(0);

// ============================================================================
// Low-level Register Access
// ============================================================================

#[inline]
fn uart_read(offset: usize) -> u32 {
    let addr = mmu::phys_to_virt(UART0_BASE as u64) as usize + offset;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
fn uart_write(offset: usize, val: u32) {
    let addr = mmu::phys_to_virt(UART0_BASE as u64) as usize + offset;
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

// ============================================================================
// Public API
// ============================================================================

/// Initialize the UART
pub fn init() {
    // Disable UART while configuring
    uart_write(UARTCR, 0);

    // Clear all pending interrupts
    uart_write(UARTICR, 0x7FF);

    // Set baud rate (QEMU ignores this but set reasonable values)
    // For 115200 baud with 24MHz clock: IBRD=13, FBRD=1
    uart_write(UARTIBRD, 13);
    uart_write(UARTFBRD, 1);

    // 8N1, enable FIFO
    uart_write(UARTLCR_H, LCR_WLEN_8 | LCR_FEN);

    // Configure FIFO interrupt levels:
    // RX FIFO: 1/8 full (2 bytes) - trigger interrupt on first few chars
    // TX FIFO: 1/8 full (default)
    // UARTIFLS bits [5:3] = RXIFLSEL, 0b000 = 1/8 full
    uart_write(UARTIFLS, 0);

    // Enable UART, TX, and RX
    uart_write(UARTCR, CR_UARTEN | CR_TXE | CR_RXE);

    INITIALIZED.store(true, Ordering::Release);
}

/// Write a single character directly (blocking, no buffering)
pub fn putc(c: char) {
    // Wait for TX FIFO to have space
    while (uart_read(UARTFR) & FR_TXFF) != 0 {
        core::hint::spin_loop();
    }
    uart_write(UARTDR, c as u32);
}

/// Write a string directly (for early boot)
pub fn print(s: &str) {
    for c in s.chars() {
        if c == '\n' {
            putc('\r');
        }
        putc(c);
    }
}

/// Read a character (non-blocking)
pub fn getc() -> Option<char> {
    // First check software buffer
    {
        let mut rx = RX_BUFFER.lock();
        if let Some(b) = rx.pop() {
            return Some(b as char);
        }
    }

    // Check hardware FIFO
    if (uart_read(UARTFR) & FR_RXFE) == 0 {
        let data = uart_read(UARTDR);
        return Some((data & 0xFF) as u8 as char);
    }

    None
}

/// Check if TX is ready
pub fn tx_ready() -> bool {
    (uart_read(UARTFR) & FR_TXFF) == 0
}

/// Check if RX has data
pub fn rx_ready() -> bool {
    // Check software buffer first
    if RX_BUFFER.lock().has_data() {
        return true;
    }
    // Check hardware
    (uart_read(UARTFR) & FR_RXFE) == 0
}

/// Buffered write - goes to ring buffer (string version)
pub fn print_buffered(s: &str) {
    let mut buf = UART_BUFFER.lock();
    for byte in s.bytes() {
        if byte == b'\n' {
            let _ = buf.push(b'\r');
        }
        let _ = buf.push(byte);
    }
}

/// Buffered write - goes to ring buffer (bytes version)
pub fn write_buffered(data: &[u8]) {
    let mut buf = UART_BUFFER.lock();
    for &byte in data {
        if !buf.push(byte) {
            // Buffer full - drop or flush
            break;
        }
    }
}

/// Flush buffer to hardware
pub fn flush() {
    let mut buf = UART_BUFFER.lock();
    while let Some(byte) = buf.pop() {
        // Wait for space
        while (uart_read(UARTFR) & FR_TXFF) != 0 {
            core::hint::spin_loop();
        }
        uart_write(UARTDR, byte as u32);
    }
}

/// Enable RX interrupt
pub fn enable_rx_interrupt() {
    let imsc = uart_read(UARTIMSC);
    // Enable RXIM (bit 4) and RTIM (bit 6 - receive timeout)
    // RTIM fires when data sits in FIFO for 32 bit periods without more data
    uart_write(UARTIMSC, imsc | (1 << 4) | (1 << 6));
}

/// Handle UART RX interrupt - reads all available data into ring buffer
/// Returns true if data was received
pub fn handle_rx_irq() -> bool {
    let mis = uart_read(UARTMIS);
    let mut received = false;

    // RX interrupt (RXIM bit 4) or receive timeout (RTIM bit 6)
    if (mis & ((1 << 4) | (1 << 6))) != 0 {
        let mut rx = RX_BUFFER.lock();
        while (uart_read(UARTFR) & FR_RXFE) == 0 {
            let data = uart_read(UARTDR);
            let byte = (data & 0xFF) as u8;
            let _ = rx.push(byte);
            received = true;
        }
        // Clear both RXIC and RTIC interrupts
        uart_write(UARTICR, (1 << 4) | (1 << 6));
    }

    received
}

/// Register a process as blocked waiting for console input
/// Returns true if registered, false if another process is already waiting
pub fn block_for_input(pid: u32) -> bool {
    CONSOLE_BLOCKED_PID.compare_exchange(
        0, pid,
        Ordering::SeqCst, Ordering::SeqCst
    ).is_ok()
}

/// Clear the blocked process (called when woken or process exits)
pub fn clear_blocked() {
    CONSOLE_BLOCKED_PID.store(0, Ordering::Release);
}

/// Clear blocked PID only if it matches the given PID
/// Used when a task returns from mux.wait() to allow re-registration
pub fn clear_blocked_if_pid(pid: u32) {
    let _ = CONSOLE_BLOCKED_PID.compare_exchange(
        pid, 0,
        Ordering::SeqCst, Ordering::SeqCst
    );
}

/// Get the PID waiting for console input (0 if none)
pub fn get_blocked_pid() -> u32 {
    CONSOLE_BLOCKED_PID.load(Ordering::Acquire)
}

/// Check if output buffer has pending data
pub fn has_buffered_output() -> bool {
    UART_BUFFER.lock().has_data()
}

/// Check if RX buffer has data available
pub fn rx_buffer_has_data() -> bool {
    RX_BUFFER.lock().has_data()
}

/// Read a byte from the RX ring buffer (non-blocking)
/// Returns Some(byte) if data available, None otherwise
pub fn rx_buffer_read() -> Option<u8> {
    RX_BUFFER.lock().pop()
}

/// Write raw bytes to console (atomic - holds lock for entire write)
pub fn write_bytes(bytes: &[u8]) {
    for &byte in bytes {
        if byte == 0 {
            break;
        }
        putc(byte as char);
    }
}

/// Flush the output buffer to UART hardware
/// Called from timer interrupt or explicitly
pub fn flush_buffer() {
    let mut buf = UART_BUFFER.lock();

    // Drain all buffered data to UART
    // QEMU's virtual UART is fast so no need to limit bytes per call
    while let Some(byte) = buf.pop() {
        // Wait for space in TX FIFO
        while (uart_read(UARTFR) & FR_TXFF) != 0 {
            core::hint::spin_loop();
        }
        uart_write(UARTDR, byte as u32);
    }
}

// ============================================================================
// HAL Trait Implementation
// ============================================================================

pub struct Pl011Uart;

impl Pl011Uart {
    pub const fn new() -> Self {
        Self
    }
}

impl UartTrait for Pl011Uart {
    fn init(&self) {
        init();
    }

    fn putc(&self, c: u8) {
        putc(c as char);
    }

    fn getc(&self) -> Option<u8> {
        getc().map(|c| c as u8)
    }

    fn tx_ready(&self) -> bool {
        tx_ready()
    }

    fn rx_ready(&self) -> bool {
        rx_ready()
    }

    fn flush(&self) {
        flush();
    }
}

// ============================================================================
// fmt::Write for print macros
// ============================================================================

pub struct UartWriter;

impl Write for UartWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        print(s);
        Ok(())
    }
}

/// Print directly to UART (for early boot / debugging)
#[macro_export]
#[cfg(feature = "platform-qemu-virt")]
macro_rules! print_direct {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let _ = write!($crate::platform::qemu_virt::uart::UartWriter, $($arg)*);
    }};
}

/// Print directly to UART with newline
#[macro_export]
#[cfg(feature = "platform-qemu-virt")]
macro_rules! println_direct {
    () => ($crate::print_direct!("\r\n"));
    ($($arg:tt)*) => {{
        $crate::print_direct!($($arg)*);
        $crate::print_direct!("\r\n");
    }};
}

// ============================================================================
// HAL Uart Wrapper (for trait object access)
// ============================================================================

/// Wrapper struct that implements the HAL Uart trait
/// This allows returning a &dyn Uart from platform code
pub struct UartWrapper;

/// Static instance for returning as trait object
static UART_WRAPPER: UartWrapper = UartWrapper;

impl UartTrait for UartWrapper {
    fn init(&self) {
        init();
    }

    fn putc(&self, c: u8) {
        putc(c as char);
    }

    fn getc(&self) -> Option<u8> {
        getc().map(|c| c as u8)
    }

    fn tx_ready(&self) -> bool {
        tx_ready()
    }

    fn rx_ready(&self) -> bool {
        rx_ready()
    }

    fn flush(&self) {
        flush();
    }
}

/// Get a reference to the UART as a Uart trait object
pub fn as_uart() -> &'static dyn UartTrait {
    &UART_WRAPPER
}
