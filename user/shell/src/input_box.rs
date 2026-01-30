//! Claude Code-style Input Box
//!
//! Renders a bordered input box at the bottom of the terminal.
//! The box grows upward as text wraps to multiple lines.
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────────────────┐
//! │ > text that wraps to multiple lines continues here with proper          │
//! │   indentation on continuation lines                                      │
//! └──────────────────────────────────────────────────────────────────────────┘
//! ```

use crate::console;

/// Box drawing characters (UTF-8)
const BOX_TL: &[u8] = "┌".as_bytes();     // Top-left
const BOX_TR: &[u8] = "┐".as_bytes();     // Top-right
const BOX_BL: &[u8] = "└".as_bytes();     // Bottom-left
const BOX_BR: &[u8] = "┘".as_bytes();     // Bottom-right
const BOX_H: &[u8] = "─".as_bytes();      // Horizontal
const BOX_V: &[u8] = "│".as_bytes();      // Vertical

/// Prompt shown at start of first line
const PROMPT: &[u8] = b"> ";
/// Continuation indent (same width as "> ")
const CONTINUATION: &[u8] = b"  ";

/// Input box state
pub struct InputBox {
    /// Terminal columns
    pub cols: u16,
    /// Terminal rows
    pub rows: u16,
    /// Current box height (content lines)
    pub height: u16,
    /// Row where cursor is (0 = first content line)
    pub cursor_row: u16,
    /// Column where cursor is (0 = after prompt/indent)
    pub cursor_col: u16,
    /// Whether box is currently drawn
    pub drawn: bool,
}

impl InputBox {
    pub const fn new() -> Self {
        Self {
            cols: 80,
            rows: 24,
            height: 1,
            cursor_row: 0,
            cursor_col: 0,
            drawn: false,
        }
    }

    /// Initialize with terminal dimensions
    pub fn init(&mut self, cols: u16, rows: u16) {
        self.cols = cols.max(20);
        self.rows = rows.max(5);
        self.height = 1;
        self.cursor_row = 0;
        self.cursor_col = 0;
        self.drawn = false;
    }

    /// Width available for text on each line
    /// Box layout: "│ > text │" or "│   text │"
    /// Border(1) + space(1) + prompt/indent(2) + text + space(1) + border(1) = 6 chars overhead
    pub fn text_width(&self) -> u16 {
        self.cols.saturating_sub(6)
    }

    /// Calculate how many lines needed for given text length
    pub fn lines_for_text(&self, text_len: usize) -> u16 {
        let tw = self.text_width() as usize;
        if tw == 0 {
            return 1;
        }
        if text_len == 0 {
            return 1;
        }
        ((text_len + tw - 1) / tw) as u16
    }

    /// Draw the complete box with content
    /// Returns cursor position after drawing (for subsequent edits)
    pub fn draw(&mut self, content: &[u8], cursor_pos: usize) {
        let text_width = self.text_width() as usize;
        if text_width == 0 {
            return;
        }

        self.height = 1; // Fixed single line for now

        // Simple approach: just draw a basic box without ANSI positioning
        // This avoids any potential issues with escape sequences

        // Clear line and draw simple box
        console::write(b"\r\n");

        // Top border
        console::write(b"+");
        for _ in 0..self.cols.saturating_sub(2) {
            console::write(b"-");
        }
        console::write(b"+\r\n");

        // Content line
        console::write(b"| > ");
        if !content.is_empty() {
            let show_len = content.len().min(text_width);
            console::write(&content[..show_len]);
        }
        // Padding
        let content_shown = content.len().min(text_width);
        for _ in content_shown..text_width {
            console::write(b" ");
        }
        console::write(b" |\r\n");

        // Bottom border
        console::write(b"+");
        for _ in 0..self.cols.saturating_sub(2) {
            console::write(b"-");
        }
        console::write(b"+");

        // Move cursor back to content area (simple approach)
        // Go up 2 lines, right to after "> "
        console::write(b"\x1b[2A\x1b[4C");

        // Move to cursor position within content
        if cursor_pos > 0 && cursor_pos <= text_width {
            for _ in 0..cursor_pos {
                console::write(b"\x1b[C");
            }
        }

        self.drawn = true;
    }

    /// Update box after text change (optimized redraw)
    /// If height changed, do full redraw. Otherwise just update content.
    pub fn update(&mut self, content: &[u8], cursor_pos: usize) {
        // Simple approach: always redraw the content line
        let text_width = self.text_width() as usize;
        if text_width == 0 {
            return;
        }

        // Go to start of content (up 2 from bottom border, column 5)
        console::write(b"\x1b[2A\r\x1b[4C");

        // Clear and rewrite content
        let show_len = content.len().min(text_width);
        if show_len > 0 {
            console::write(&content[..show_len]);
        }

        // Clear rest with spaces
        for _ in show_len..text_width {
            console::write(b" ");
        }

        // Move cursor back to correct position
        // First go back to start of content area
        console::write(b"\r\x1b[4C");
        // Then move forward to cursor_pos
        for _ in 0..cursor_pos.min(text_width) {
            console::write(b"\x1b[C");
        }
    }

    /// Redraw just the content lines (height unchanged)
    fn redraw_content(&mut self, content: &[u8], cursor_pos: usize) {
        self.update(content, cursor_pos);
    }

    /// Clear the box from screen
    pub fn clear(&mut self) {
        if !self.drawn {
            return;
        }

        let box_start_row = self.rows.saturating_sub(self.height + 2);

        let mut buf = [0u8; 32];
        let mut pos = 0;

        // Move to box start and clear to end of screen
        pos += write_ansi_goto(&mut buf[pos..], box_start_row + 1, 1);
        buf[pos..pos+4].copy_from_slice(b"\x1b[J");
        pos += 4;

        console::write(&buf[..pos]);
        self.drawn = false;
        self.height = 1;
    }

    /// Move cursor to output area (above the box)
    pub fn cursor_to_output(&self) {
        let box_start_row = self.rows.saturating_sub(self.height + 2);
        let output_row = box_start_row; // Last row before box

        let mut buf = [0u8; 16];
        let pos = write_ansi_goto(&mut buf, output_row, 1);
        console::write(&buf[..pos]);
    }

    /// Position cursor correctly within the box
    pub fn position_cursor(&self, cursor_pos: usize) {
        let text_width = self.text_width() as usize;
        if text_width == 0 {
            return;
        }

        // Simple approach: go to content start then move forward
        // Go up 2 from current position (assuming we're at bottom border)
        // then to column 5 (after "| > ")
        console::write(b"\x1b[2A\r\x1b[4C");

        // Move forward to cursor position
        for _ in 0..cursor_pos.min(text_width) {
            console::write(b"\x1b[C");
        }
    }
}

/// Write ANSI goto sequence: ESC[row;colH
fn write_ansi_goto(buf: &mut [u8], row: u16, col: u16) -> usize {
    let mut pos = 0;
    buf[pos..pos+2].copy_from_slice(b"\x1b[");
    pos += 2;
    pos += write_u16(&mut buf[pos..], row);
    buf[pos] = b';';
    pos += 1;
    pos += write_u16(&mut buf[pos..], col);
    buf[pos] = b'H';
    pos += 1;
    pos
}

/// Write u16 as decimal, return bytes written
fn write_u16(buf: &mut [u8], n: u16) -> usize {
    if n == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 5];
    let mut val = n;
    let mut len = 0;
    while val > 0 {
        tmp[len] = b'0' + (val % 10) as u8;
        val /= 10;
        len += 1;
    }
    for i in 0..len {
        buf[i] = tmp[len - 1 - i];
    }
    len
}

/// Write bytes, return count written
fn write_bytes(buf: &mut [u8], src: &[u8]) -> usize {
    let len = src.len().min(buf.len());
    buf[..len].copy_from_slice(&src[..len]);
    len
}
