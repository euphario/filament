//! Readline - Line Editing with History
//!
//! Provides bash-like line editing:
//! - Left/Right arrows: move cursor
//! - Home/End: move to start/end of line
//! - Backspace: delete before cursor
//! - Delete: delete at cursor
//! - Up/Down arrows: history navigation
//! - Ctrl+A/E: move to start/end
//! - Ctrl+K: delete to end of line
//! - Ctrl+U: delete entire line
//! - Ctrl+W: delete word before cursor
//! - Ctrl+C: cancel line
//! - Ctrl+D: exit (if line empty)

use userlib::syscall;
use crate::console;

/// Maximum command line length
pub const MAX_LINE: usize = 128;

/// Maximum history entries
const MAX_HISTORY: usize = 16;

/// History buffer
pub struct History {
    entries: [[u8; MAX_LINE]; MAX_HISTORY],
    lengths: [usize; MAX_HISTORY],
    count: usize,
    /// Write position (ring buffer)
    write_pos: usize,
    /// Current browsing position (-1 = current line, 0..count-1 = history)
    browse_pos: isize,
}

impl History {
    pub const fn new() -> Self {
        Self {
            entries: [[0u8; MAX_LINE]; MAX_HISTORY],
            lengths: [0; MAX_HISTORY],
            count: 0,
            write_pos: 0,
            browse_pos: -1,
        }
    }

    /// Add entry to history (doesn't add empty or duplicate of last)
    pub fn add(&mut self, line: &[u8]) {
        if line.is_empty() {
            return;
        }

        // Don't add if same as most recent
        if self.count > 0 {
            let last = if self.write_pos == 0 { MAX_HISTORY - 1 } else { self.write_pos - 1 };
            let last_len = self.lengths[last];
            if last_len == line.len() && &self.entries[last][..last_len] == line {
                return;
            }
        }

        let len = line.len().min(MAX_LINE);
        self.entries[self.write_pos][..len].copy_from_slice(&line[..len]);
        self.lengths[self.write_pos] = len;

        self.write_pos = (self.write_pos + 1) % MAX_HISTORY;
        if self.count < MAX_HISTORY {
            self.count += 1;
        }
    }

    /// Get entry by index (0 = most recent)
    pub fn get(&self, idx: usize) -> Option<&[u8]> {
        if idx >= self.count {
            return None;
        }
        // Calculate actual index in ring buffer
        let actual = if self.write_pos > idx {
            self.write_pos - 1 - idx
        } else {
            MAX_HISTORY - 1 - (idx - self.write_pos)
        };
        Some(&self.entries[actual][..self.lengths[actual]])
    }

    /// Reset browse position
    pub fn reset_browse(&mut self) {
        self.browse_pos = -1;
    }

    /// Go to older entry
    pub fn browse_older(&mut self) -> Option<&[u8]> {
        let next = self.browse_pos + 1;
        if (next as usize) < self.count {
            self.browse_pos = next;
            self.get(next as usize)
        } else {
            None
        }
    }

    /// Go to newer entry
    pub fn browse_newer(&mut self) -> Option<&[u8]> {
        if self.browse_pos > 0 {
            self.browse_pos -= 1;
            self.get(self.browse_pos as usize)
        } else if self.browse_pos == 0 {
            self.browse_pos = -1;
            Some(&[])  // Return to empty current line
        } else {
            None
        }
    }
}

/// Line editor state
pub struct LineEditor<'a> {
    buf: &'a mut [u8],
    len: usize,
    cursor: usize,
    history: &'a mut History,
    /// Saved current line when browsing history
    saved_line: [u8; MAX_LINE],
    saved_len: usize,
}

impl<'a> LineEditor<'a> {
    pub fn new(buf: &'a mut [u8], history: &'a mut History) -> Self {
        history.reset_browse();
        Self {
            buf,
            len: 0,
            cursor: 0,
            history,
            saved_line: [0u8; MAX_LINE],
            saved_len: 0,
        }
    }

    /// Read a line with editing support
    /// Returns the length of the line, or 0 if cancelled
    pub fn read(&mut self) -> usize {
        while self.len < self.buf.len() - 1 {
            if let Some(ch) = console::read_byte() {
                match ch {
                    // Enter - end of line
                    b'\r' | b'\n' => {
                        write_str("\r\n");
                        // Add to history
                        if self.len > 0 {
                            self.history.add(&self.buf[..self.len]);
                        }
                        return self.len;
                    }

                    // Backspace - delete before cursor
                    0x7F | 0x08 => {
                        self.delete_before_cursor();
                    }

                    // Ctrl+C - cancel line
                    0x03 => {
                        write_str("^C\r\n");
                        return 0;
                    }

                    // Ctrl+D - EOF (exit if empty line)
                    0x04 => {
                        if self.len == 0 {
                            write_str("\r\n");
                            syscall::exit(0);
                        }
                    }

                    // Ctrl+A - move to start
                    0x01 => {
                        self.move_to_start();
                    }

                    // Ctrl+E - move to end
                    0x05 => {
                        self.move_to_end();
                    }

                    // Ctrl+K - delete to end of line
                    0x0B => {
                        self.delete_to_end();
                    }

                    // Ctrl+U - delete entire line
                    0x15 => {
                        self.delete_line();
                    }

                    // Ctrl+W - delete word before cursor
                    0x17 => {
                        self.delete_word();
                    }

                    // Tab - completion
                    b'\t' => {
                        self.handle_tab();
                    }

                    // Escape sequence start
                    0x1B => {
                        self.handle_escape();
                    }

                    // Printable characters
                    0x20..=0x7E => {
                        self.insert_char(ch);
                    }

                    _ => {
                        // Ignore other control characters
                    }
                }
            }
        }

        self.len
    }

    /// Handle escape sequences (arrow keys, etc.)
    fn handle_escape(&mut self) {
        // Read next character (should be '[')
        let ch = match console::read_byte() {
            Some(c) => c,
            None => return,
        };

        if ch != b'[' {
            return;
        }

        // Read the actual code
        let code = match console::read_byte() {
            Some(c) => c,
            None => return,
        };

        match code {
            b'A' => self.history_older(),  // Up
            b'B' => self.history_newer(),  // Down
            b'C' => self.move_right(),     // Right
            b'D' => self.move_left(),      // Left
            b'H' => self.move_to_start(),  // Home
            b'F' => self.move_to_end(),    // End
            b'3' => {
                // Delete key: ESC [ 3 ~
                if console::read_byte() == Some(b'~') {
                    self.delete_at_cursor();
                }
            }
            _ => {}
        }
    }

    /// Insert character at cursor
    fn insert_char(&mut self, ch: u8) {
        if self.len >= self.buf.len() - 1 {
            return;
        }

        // Inserting at end vs middle requires different handling
        let inserting_at_end = self.cursor == self.len;

        // Shift characters after cursor to the right
        if self.cursor < self.len {
            for i in (self.cursor..self.len).rev() {
                self.buf[i + 1] = self.buf[i];
            }
        }

        self.buf[self.cursor] = ch;
        self.len += 1;
        self.cursor += 1;

        if inserting_at_end {
            // Simple case: just echo the character
            console::write(&[ch]);
        } else {
            // Redraw from cursor to end (cursor is now past the inserted char)
            self.redraw_from_cursor();
        }
    }

    /// Delete character before cursor
    fn delete_before_cursor(&mut self) {
        if self.cursor == 0 {
            return;
        }

        // Shift characters left
        for i in self.cursor..self.len {
            self.buf[i - 1] = self.buf[i];
        }

        self.len -= 1;
        self.cursor -= 1;

        // Move cursor back and redraw
        write_str("\x08");  // Move back
        self.redraw_from_cursor();
        write_str(" \x08");  // Clear last char
    }

    /// Delete character at cursor
    fn delete_at_cursor(&mut self) {
        if self.cursor >= self.len {
            return;
        }

        // Shift characters left
        for i in self.cursor..self.len - 1 {
            self.buf[i] = self.buf[i + 1];
        }

        self.len -= 1;
        self.redraw_from_cursor();
        write_str(" \x08");  // Clear last char
    }

    /// Move cursor left
    fn move_left(&mut self) {
        if self.cursor > 0 {
            self.cursor -= 1;
            write_str("\x08");  // Move back
        }
    }

    /// Move cursor right
    fn move_right(&mut self) {
        if self.cursor < self.len {
            console::write(&[self.buf[self.cursor]]);
            self.cursor += 1;
        }
    }

    /// Move cursor to start
    fn move_to_start(&mut self) {
        while self.cursor > 0 {
            self.cursor -= 1;
            write_str("\x08");
        }
    }

    /// Move cursor to end
    fn move_to_end(&mut self) {
        while self.cursor < self.len {
            console::write(&[self.buf[self.cursor]]);
            self.cursor += 1;
        }
    }

    /// Delete from cursor to end of line
    fn delete_to_end(&mut self) {
        if self.cursor >= self.len {
            return;
        }

        let chars_to_clear = self.len - self.cursor;
        self.len = self.cursor;

        // Clear to end of line
        for _ in 0..chars_to_clear {
            write_str(" ");
        }
        for _ in 0..chars_to_clear {
            write_str("\x08");
        }
    }

    /// Delete entire line
    fn delete_line(&mut self) {
        self.move_to_start();
        self.delete_to_end();
    }

    /// Delete word before cursor
    fn delete_word(&mut self) {
        if self.cursor == 0 {
            return;
        }

        // Skip trailing spaces
        let mut end = self.cursor;
        while end > 0 && self.buf[end - 1] == b' ' {
            end -= 1;
        }

        // Find word start
        let mut start = end;
        while start > 0 && self.buf[start - 1] != b' ' {
            start -= 1;
        }

        if start == self.cursor {
            return;
        }

        // Delete from start to cursor
        let chars_deleted = self.cursor - start;
        for i in self.cursor..self.len {
            self.buf[i - chars_deleted] = self.buf[i];
        }

        self.len -= chars_deleted;

        // Move cursor back
        for _ in 0..chars_deleted {
            write_str("\x08");
        }
        self.cursor = start;

        // Redraw and clear
        self.redraw_from_cursor();
        for _ in 0..chars_deleted {
            write_str(" ");
        }
        for _ in 0..chars_deleted {
            write_str("\x08");
        }
    }

    /// Browse to older history entry
    fn history_older(&mut self) {
        // Save current line on first browse
        if self.history.browse_pos == -1 {
            self.saved_line[..self.len].copy_from_slice(&self.buf[..self.len]);
            self.saved_len = self.len;
        }

        // Copy entry to temp buffer to avoid borrow issues
        let mut temp = [0u8; MAX_LINE];
        let temp_len;

        if let Some(entry) = self.history.browse_older() {
            temp_len = entry.len().min(MAX_LINE);
            temp[..temp_len].copy_from_slice(&entry[..temp_len]);
        } else {
            return;
        }

        self.replace_line(&temp[..temp_len]);
    }

    /// Browse to newer history entry
    fn history_newer(&mut self) {
        // Copy entry to temp buffer to avoid borrow issues
        let mut temp = [0u8; MAX_LINE];
        let temp_len;
        let restore_saved;

        if let Some(entry) = self.history.browse_newer() {
            if entry.is_empty() {
                restore_saved = true;
                temp_len = 0;
            } else {
                restore_saved = false;
                temp_len = entry.len().min(MAX_LINE);
                temp[..temp_len].copy_from_slice(&entry[..temp_len]);
            }
        } else {
            return;
        }

        if restore_saved {
            // Copy saved_line to temp first
            let saved_len = self.saved_len;
            temp[..saved_len].copy_from_slice(&self.saved_line[..saved_len]);
            self.replace_line(&temp[..saved_len]);
        } else {
            self.replace_line(&temp[..temp_len]);
        }
    }

    /// Replace current line with new content
    fn replace_line(&mut self, new: &[u8]) {
        // Move to start
        self.move_to_start();

        // Clear current line
        let old_len = self.len;
        for _ in 0..old_len {
            write_str(" ");
        }
        for _ in 0..old_len {
            write_str("\x08");
        }

        // Copy new content
        let new_len = new.len().min(self.buf.len() - 1);
        self.buf[..new_len].copy_from_slice(&new[..new_len]);
        self.len = new_len;
        self.cursor = new_len;

        // Display new line
        console::write(&self.buf[..self.len]);
    }

    /// Redraw from cursor to end of line
    fn redraw_from_cursor(&mut self) {
        if self.cursor < self.len {
            console::write(&self.buf[self.cursor..self.len]);
            // Move cursor back to position
            for _ in self.cursor..self.len {
                write_str("\x08");
            }
        }
    }

    /// Handle tab completion
    fn handle_tab(&mut self) {
        // Find word start (for completion)
        let mut word_start = self.cursor;
        while word_start > 0 && self.buf[word_start - 1] != b' ' {
            word_start -= 1;
        }

        let word = &self.buf[word_start..self.cursor];

        // Determine if this is a command (first word) or path completion
        let is_command = word_start == 0 || {
            // Check if there's only whitespace before
            let mut only_space = true;
            for i in 0..word_start {
                if self.buf[i] != b' ' {
                    only_space = false;
                    break;
                }
            }
            only_space
        };

        // Get completions
        let completions = if is_command {
            crate::completion::complete_command(word)
        } else {
            crate::completion::complete_path(word)
        };

        if completions.count == 0 {
            // No matches - beep or do nothing
            return;
        }

        // Calculate how much we can complete
        let word_len = self.cursor - word_start;
        if completions.common_len > word_len {
            // Insert the additional characters
            let extra = &completions.common[word_len..completions.common_len];
            for &ch in extra {
                self.insert_char(ch);
            }

            // If unique match, add space (for commands) or / (for directories)
            if completions.count == 1 {
                self.insert_char(b' ');
            }
        } else if completions.count > 1 {
            // Multiple matches with same prefix - show options
            write_str("\r\n");

            // Print all matches
            for i in 0..completions.count {
                if let Some(m) = completions.get_match(i) {
                    if i > 0 {
                        write_str("  ");
                    }
                    console::write(m);
                }
            }
            write_str("\r\n");

            // Redraw prompt and line (with colors)
            crate::color::set(crate::color::BOLD);
            crate::color::set(crate::color::GREEN);
            write_str("> ");
            crate::color::reset();
            console::write(&self.buf[..self.len]);

            // Move cursor back to position
            for _ in self.cursor..self.len {
                write_str("\x08");
            }
        }
    }
}

fn write_str(s: &str) {
    console::write_str(s);
}
