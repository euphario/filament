//! Structured Output Model
//!
//! PowerShell-inspired object model for shell command output.
//! Commands return structured data that can be filtered, transformed,
//! and displayed as formatted tables.
//!
//! # Example
//! ```
//! let mut table = Table::new(&["PID", "NAME", "STATE"]);
//! table.add_row().uint(pid).str(name).str(state);
//! table.add_row().uint(pid2).str(name2).str(state2);
//! table.print();
//! ```

// Use shell's own print!/println! macros which route through console
use crate::{print, println};
use crate::color;

/// Trait for simple output (used by handle tests and other builtins that need raw byte output)
pub trait Output {
    fn write(&mut self, data: &[u8]);
}

/// Simple shell output that writes to stdout
pub struct ShellOutput;

impl ShellOutput {
    pub fn new() -> Self {
        Self
    }
}

impl Output for ShellOutput {
    fn write(&mut self, data: &[u8]) {
        if let Ok(s) = core::str::from_utf8(data) {
            print!("{}", s);
        } else {
            // Print byte-by-byte if not valid UTF-8
            for &b in data {
                print!("{}", b as char);
            }
        }
    }
}

/// Maximum columns per table
pub const MAX_COLS: usize = 8;

/// Maximum rows per table (for static allocation)
pub const MAX_ROWS: usize = 64;

/// Maximum string length in a cell
pub const MAX_CELL_STR: usize = 32;

/// Cell value types
#[derive(Clone, Copy)]
pub enum Value {
    Empty,
    Bool(bool),
    Int(i64),
    Uint(u64),
    Hex32(u32),
    Hex64(u64),
    /// Static string reference
    StaticStr(&'static str),
    /// Inline string buffer for dynamic strings
    Str { buf: [u8; MAX_CELL_STR], len: u8 },
}

impl Value {
    /// Create from a dynamic string (copies into buffer)
    pub fn from_str(s: &str) -> Self {
        let mut buf = [0u8; MAX_CELL_STR];
        let len = s.len().min(MAX_CELL_STR);
        buf[..len].copy_from_slice(&s.as_bytes()[..len]);
        Value::Str { buf, len: len as u8 }
    }

    /// Create from byte slice
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut buf = [0u8; MAX_CELL_STR];
        let len = b.len().min(MAX_CELL_STR);
        buf[..len].copy_from_slice(&b[..len]);
        Value::Str { buf, len: len as u8 }
    }

    /// Get display width (for column sizing)
    pub fn display_width(&self) -> usize {
        match self {
            Value::Empty => 0,
            Value::Bool(true) => 4,  // "true"
            Value::Bool(false) => 5, // "false"
            Value::Int(n) => int_width(*n),
            Value::Uint(n) => uint_width(*n),
            Value::Hex32(_) => 10, // "0x" + 8 digits
            Value::Hex64(_) => 18, // "0x" + 16 digits
            Value::StaticStr(s) => s.len(),
            Value::Str { len, .. } => *len as usize,
        }
    }

    /// Print value to stdout
    pub fn print(&self) {
        match self {
            Value::Empty => {}
            Value::Bool(b) => print!("{}", if *b { "true" } else { "false" }),
            Value::Int(n) => print!("{}", n),
            Value::Uint(n) => print!("{}", n),
            Value::Hex32(n) => print!("0x{:08x}", n),
            Value::Hex64(n) => print!("0x{:016x}", n),
            Value::StaticStr(s) => print!("{}", s),
            Value::Str { buf, len } => {
                if let Ok(s) = core::str::from_utf8(&buf[..*len as usize]) {
                    print!("{}", s);
                }
            }
        }
    }

    /// Print value right-aligned in given width
    pub fn print_right(&self, width: usize) {
        let w = self.display_width();
        if w < width {
            for _ in 0..(width - w) {
                print!(" ");
            }
        }
        self.print();
    }

    /// Print value left-aligned in given width
    pub fn print_left(&self, width: usize) {
        self.print();
        let w = self.display_width();
        if w < width {
            for _ in 0..(width - w) {
                print!(" ");
            }
        }
    }
}

/// A row of values
#[derive(Clone, Copy)]
pub struct Row {
    cells: [Value; MAX_COLS],
    count: usize,
}

impl Row {
    pub const fn empty() -> Self {
        Self {
            cells: [Value::Empty; MAX_COLS],
            count: 0,
        }
    }

    /// Add a boolean value
    pub fn bool(mut self, v: bool) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::Bool(v);
            self.count += 1;
        }
        self
    }

    /// Add a signed integer
    pub fn int(mut self, v: i64) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::Int(v);
            self.count += 1;
        }
        self
    }

    /// Add an unsigned integer
    pub fn uint(mut self, v: u64) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::Uint(v);
            self.count += 1;
        }
        self
    }

    /// Add a hex32 value
    pub fn hex32(mut self, v: u32) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::Hex32(v);
            self.count += 1;
        }
        self
    }

    /// Add a hex64 value
    pub fn hex64(mut self, v: u64) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::Hex64(v);
            self.count += 1;
        }
        self
    }

    /// Add a static string
    pub fn str(mut self, s: &'static str) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::StaticStr(s);
            self.count += 1;
        }
        self
    }

    /// Add a dynamic string (copies into buffer)
    pub fn string(mut self, s: &str) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::from_str(s);
            self.count += 1;
        }
        self
    }

    /// Add bytes as string
    pub fn bytes(mut self, b: &[u8]) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::from_bytes(b);
            self.count += 1;
        }
        self
    }

    /// Add an empty cell (skip a column)
    pub fn skip(mut self) -> Self {
        if self.count < MAX_COLS {
            self.cells[self.count] = Value::Empty;
            self.count += 1;
        }
        self
    }

    /// Get cell at index
    pub fn get(&self, idx: usize) -> Option<&Value> {
        if idx < self.count {
            Some(&self.cells[idx])
        } else {
            None
        }
    }
}

/// Column alignment
#[derive(Clone, Copy, PartialEq)]
pub enum Align {
    Left,
    Right,
}

/// Table with headers and rows
pub struct Table {
    headers: [&'static str; MAX_COLS],
    aligns: [Align; MAX_COLS],
    col_count: usize,
    rows: [Row; MAX_ROWS],
    row_count: usize,
    /// Computed column widths
    widths: [usize; MAX_COLS],
}

impl Table {
    /// Create a new table with headers
    pub fn new(headers: &[&'static str]) -> Self {
        let mut h = [""; MAX_COLS];
        let mut widths = [0usize; MAX_COLS];
        let aligns = [Align::Left; MAX_COLS];
        let count = headers.len().min(MAX_COLS);

        for (i, &hdr) in headers.iter().take(count).enumerate() {
            h[i] = hdr;
            widths[i] = hdr.len();
        }

        Self {
            headers: h,
            aligns,
            col_count: count,
            rows: [Row::empty(); MAX_ROWS],
            row_count: 0,
            widths,
        }
    }

    /// Set column alignment
    pub fn align(mut self, col: usize, align: Align) -> Self {
        if col < self.col_count {
            self.aligns[col] = align;
        }
        self
    }

    /// Set all numeric-looking columns to right-align
    pub fn auto_align(mut self) -> Self {
        // Right-align columns with numeric headers
        for i in 0..self.col_count {
            let h = self.headers[i];
            // Simple case-insensitive substring check
            if contains_ignore_case(h, "PID") || contains_ignore_case(h, "SIZE")
                || contains_ignore_case(h, "COUNT") || contains_ignore_case(h, "PORT")
                || contains_ignore_case(h, "ADDR") || contains_ignore_case(h, "ID")
            {
                self.aligns[i] = Align::Right;
            }
        }
        self
    }

    /// Add a row (returns Row builder for chaining)
    pub fn row(&mut self) -> &mut Row {
        if self.row_count < MAX_ROWS {
            self.rows[self.row_count] = Row::empty();
            let idx = self.row_count;
            self.row_count += 1;
            &mut self.rows[idx]
        } else {
            // Return last row (will be overwritten) - shouldn't happen in practice
            &mut self.rows[MAX_ROWS - 1]
        }
    }

    /// Add a complete row
    pub fn add_row(&mut self, row: Row) {
        if self.row_count < MAX_ROWS {
            // Update column widths
            for i in 0..self.col_count.min(row.count) {
                let w = row.cells[i].display_width();
                if w > self.widths[i] {
                    self.widths[i] = w;
                }
            }
            self.rows[self.row_count] = row;
            self.row_count += 1;
        }
    }

    /// Get number of rows
    pub fn len(&self) -> usize {
        self.row_count
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.row_count == 0
    }

    /// Print the table with headers and formatting
    /// Uses line buffering to reduce channel message count
    pub fn print(&self) {
        // Compute final widths including all rows
        let mut widths = self.widths;
        for i in 0..self.row_count {
            for j in 0..self.col_count {
                let w = self.rows[i].cells[j].display_width();
                if w > widths[j] {
                    widths[j] = w;
                }
            }
        }

        // Line buffer for batching output
        let mut line = LineBuf::new();

        // Print header (bold cyan)
        line.push_str("\x1b[1m\x1b[36m"); // BOLD + CYAN
        for i in 0..self.col_count {
            if i > 0 {
                line.push_str("  ");
            }
            let hdr = self.headers[i];
            let w = widths[i];
            if self.aligns[i] == Align::Right {
                line.pad(w.saturating_sub(hdr.len()));
                line.push_str(hdr);
            } else {
                line.push_str(hdr);
                line.pad(w.saturating_sub(hdr.len()));
            }
        }
        line.push_str("\x1b[0m\r\n"); // RESET + newline
        line.flush();

        // Print separator (dim)
        line.push_str("\x1b[2m"); // DIM
        for i in 0..self.col_count {
            if i > 0 {
                line.push_str("  ");
            }
            line.dashes(widths[i]);
        }
        line.push_str("\x1b[0m\r\n"); // RESET + newline
        line.flush();

        // Print rows
        for r in 0..self.row_count {
            let row = &self.rows[r];
            for i in 0..self.col_count {
                if i > 0 {
                    line.push_str("  ");
                }
                let cell = &row.cells[i];
                let w = widths[i];
                if self.aligns[i] == Align::Right {
                    line.pad(w.saturating_sub(cell.display_width()));
                    line.push_value(cell);
                } else {
                    line.push_value(cell);
                    line.pad(w.saturating_sub(cell.display_width()));
                }
            }
            line.push_str("\r\n");
            line.flush();
        }
    }

    /// Print as simple list (no headers, one row per line)
    pub fn print_list(&self) {
        for r in 0..self.row_count {
            let row = &self.rows[r];
            for i in 0..self.col_count {
                if i > 0 {
                    print!(": ");
                }
                print!("{}", self.headers[i]);
                print!("=");
                row.cells[i].print();
            }
            println!();
        }
    }

    /// Iterate over rows
    pub fn iter(&self) -> impl Iterator<Item = &Row> {
        self.rows[..self.row_count].iter()
    }
}

// Helper functions for width calculation
fn int_width(n: i64) -> usize {
    if n == 0 {
        return 1;
    }
    let mut w = if n < 0 { 1 } else { 0 };
    let mut v = n.abs() as u64;
    while v > 0 {
        w += 1;
        v /= 10;
    }
    w
}

fn uint_width(n: u64) -> usize {
    if n == 0 {
        return 1;
    }
    let mut w = 0;
    let mut v = n;
    while v > 0 {
        w += 1;
        v /= 10;
    }
    w
}

/// Case-insensitive substring check
fn contains_ignore_case(haystack: &str, needle: &str) -> bool {
    if needle.len() > haystack.len() {
        return false;
    }
    let h = haystack.as_bytes();
    let n = needle.as_bytes();
    for i in 0..=(h.len() - n.len()) {
        let mut matched = true;
        for j in 0..n.len() {
            let hc = if h[i + j] >= b'A' && h[i + j] <= b'Z' { h[i + j] + 32 } else { h[i + j] };
            let nc = if n[j] >= b'A' && n[j] <= b'Z' { n[j] + 32 } else { n[j] };
            if hc != nc {
                matched = false;
                break;
            }
        }
        if matched {
            return true;
        }
    }
    false
}

/// Result of a command - either a table or an error message
pub enum CommandResult {
    /// Structured table output
    Table(Table),
    /// Simple text message (success)
    Ok(&'static str),
    /// Error message
    Error(&'static str),
    /// Nothing to display (command handled its own output)
    None,
}

impl CommandResult {
    /// Display the result
    pub fn print(self) {
        match self {
            CommandResult::Table(t) => t.print(),
            CommandResult::Ok(msg) => {
                color::set(color::GREEN);
                println!("{}", msg);
                color::reset();
            }
            CommandResult::Error(msg) => {
                color::set(color::BOLD);
                color::set(color::RED);
                print!("Error: ");
                color::reset();
                color::set(color::RED);
                println!("{}", msg);
                color::reset();
            }
            CommandResult::None => {}
        }
    }
}

// =============================================================================
// Line Buffer - batches output to reduce channel message count
// =============================================================================

/// Line buffer for batching output into single sends
struct LineBuf {
    buf: [u8; 256],
    len: usize,
}

impl LineBuf {
    fn new() -> Self {
        Self { buf: [0u8; 256], len: 0 }
    }

    fn push_str(&mut self, s: &str) {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
    }

    fn push_byte(&mut self, b: u8) {
        if self.len < self.buf.len() {
            self.buf[self.len] = b;
            self.len += 1;
        }
    }

    fn pad(&mut self, n: usize) {
        for _ in 0..n {
            self.push_byte(b' ');
        }
    }

    fn dashes(&mut self, n: usize) {
        for _ in 0..n {
            self.push_byte(b'-');
        }
    }

    fn push_value(&mut self, val: &Value) {
        match val {
            Value::Empty => {}
            Value::Bool(b) => self.push_str(if *b { "true" } else { "false" }),
            Value::Int(n) => self.push_int(*n),
            Value::Uint(n) => self.push_uint(*n),
            Value::Hex32(n) => {
                self.push_str("0x");
                self.push_hex32(*n);
            }
            Value::Hex64(n) => {
                self.push_str("0x");
                self.push_hex64(*n);
            }
            Value::StaticStr(s) => self.push_str(s),
            Value::Str { buf, len } => {
                let bytes = &buf[..*len as usize];
                for &b in bytes {
                    self.push_byte(b);
                }
            }
        }
    }

    fn push_int(&mut self, n: i64) {
        if n < 0 {
            self.push_byte(b'-');
            self.push_uint((-n) as u64);
        } else {
            self.push_uint(n as u64);
        }
    }

    fn push_uint(&mut self, n: u64) {
        if n == 0 {
            self.push_byte(b'0');
            return;
        }
        let mut tmp = [0u8; 20];
        let mut i = 0;
        let mut val = n;
        while val > 0 {
            tmp[i] = b'0' + (val % 10) as u8;
            val /= 10;
            i += 1;
        }
        while i > 0 {
            i -= 1;
            self.push_byte(tmp[i]);
        }
    }

    fn push_hex32(&mut self, n: u32) {
        for i in (0..8).rev() {
            let nibble = ((n >> (i * 4)) & 0xF) as u8;
            self.push_byte(if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 });
        }
    }

    fn push_hex64(&mut self, n: u64) {
        for i in (0..16).rev() {
            let nibble = ((n >> (i * 4)) & 0xF) as u8;
            self.push_byte(if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 });
        }
    }

    fn flush(&mut self) {
        if self.len > 0 {
            crate::console::write(&self.buf[..self.len]);
            self.len = 0;
        }
    }
}
