//! ANSI terminal color and style support
//!
//! Provides escape codes and helpers for colorized terminal output.
//!
//! # Usage
//! ```
//! use userlib::ansi::{Color, Style};
//!
//! // Direct color codes
//! println!("{}Error:{} Something failed", Color::RED, Color::RESET);
//!
//! // Semantic styles (recommended)
//! println!("{}[ERROR]{} msg", Style::ERROR, Style::RESET);
//! println!("{}[WARN]{} msg", Style::WARN, Style::RESET);
//! println!("{}[INFO]{} msg", Style::INFO, Style::RESET);
//! println!("{}[DEBUG]{} msg", Style::DEBUG, Style::RESET);
//! ```

/// Basic ANSI colors
pub struct Color;

impl Color {
    /// Reset all attributes
    pub const RESET: &str = "\x1b[0m";

    /// Bold/bright
    pub const BOLD: &str = "\x1b[1m";

    /// Dim/faint
    pub const DIM: &str = "\x1b[2m";

    // Foreground colors
    pub const BLACK: &str = "\x1b[30m";
    pub const RED: &str = "\x1b[31m";
    pub const GREEN: &str = "\x1b[32m";
    pub const YELLOW: &str = "\x1b[33m";
    pub const BLUE: &str = "\x1b[34m";
    pub const MAGENTA: &str = "\x1b[35m";
    pub const CYAN: &str = "\x1b[36m";
    pub const WHITE: &str = "\x1b[37m";

    // Bright/intense foreground colors
    pub const BRIGHT_BLACK: &str = "\x1b[90m";   // Gray
    pub const BRIGHT_RED: &str = "\x1b[91m";
    pub const BRIGHT_GREEN: &str = "\x1b[92m";
    pub const BRIGHT_YELLOW: &str = "\x1b[93m";
    pub const BRIGHT_BLUE: &str = "\x1b[94m";
    pub const BRIGHT_MAGENTA: &str = "\x1b[95m";
    pub const BRIGHT_CYAN: &str = "\x1b[96m";
    pub const BRIGHT_WHITE: &str = "\x1b[97m";
}

/// Semantic styles for consistent UI
pub struct Style;

impl Style {
    /// Reset - use after any styled text
    pub const RESET: &str = Color::RESET;

    // Log levels
    /// Error messages - bold red
    pub const ERROR: &str = "\x1b[1;31m";
    /// Warning messages - yellow
    pub const WARN: &str = "\x1b[33m";
    /// Info messages - green
    pub const INFO: &str = "\x1b[32m";
    /// Debug messages - dim/gray
    pub const DEBUG: &str = "\x1b[2;37m";
    /// Trace messages - dim cyan
    pub const TRACE: &str = "\x1b[2;36m";

    // UI elements
    /// Success indicator - bright green
    pub const SUCCESS: &str = "\x1b[92m";
    /// Active/running state - green
    pub const ACTIVE: &str = "\x1b[32m";
    /// Inactive/idle state - dim
    pub const INACTIVE: &str = "\x1b[2m";
    /// Emphasis - bold
    pub const EMPHASIS: &str = "\x1b[1m";
    /// Muted/secondary text - dim
    pub const MUTED: &str = "\x1b[2m";

    // Device tree
    /// Bus name - cyan
    pub const BUS: &str = "\x1b[36m";
    /// Device name - white
    pub const DEVICE: &str = "\x1b[37m";
    /// Driver name - magenta
    pub const DRIVER: &str = "\x1b[35m";
    /// Path/location - blue
    pub const PATH: &str = "\x1b[34m";
    /// Vendor:device ID - yellow
    pub const ID: &str = "\x1b[33m";

    // Status indicators (for use with ● ○ symbols)
    /// Bound/active device - green dot
    pub const STATUS_BOUND: &str = "\x1b[32m";
    /// Discovered/idle device - dim dot
    pub const STATUS_IDLE: &str = "\x1b[2m";
    /// Error state - red dot
    pub const STATUS_ERROR: &str = "\x1b[31m";
}

/// Box-drawing characters for tree output
pub struct Box;

impl Box {
    /// Vertical line │
    pub const VERT: &str = "│";
    /// Horizontal line ─
    pub const HORIZ: &str = "─";
    /// T-junction ├
    pub const TEE: &str = "├";
    /// L-corner └
    pub const CORNER: &str = "└";
    /// Vertical and right ├─
    pub const BRANCH: &str = "├─";
    /// Corner and right └─
    pub const LAST: &str = "└─";
    /// Continuation with space │
    pub const CONT: &str = "│  ";
    /// Empty continuation (spaces)
    pub const EMPTY: &str = "   ";
}

/// Status symbols
pub struct Symbol;

impl Symbol {
    /// Filled circle (active/bound)
    pub const ACTIVE: &str = "●";
    /// Empty circle (idle/discovered)
    pub const IDLE: &str = "○";
    /// Check mark (success)
    pub const CHECK: &str = "✓";
    /// Cross mark (error)
    pub const CROSS: &str = "✗";
    /// Arrow right
    pub const ARROW: &str = "→";
    /// Bullet point
    pub const BULLET: &str = "•";
}

/// Helper to wrap text in a color/style
/// Returns a struct that implements Display
pub struct Colored<'a> {
    style: &'a str,
    text: &'a str,
}

impl<'a> Colored<'a> {
    pub const fn new(style: &'a str, text: &'a str) -> Self {
        Self { style, text }
    }
}

impl<'a> core::fmt::Display for Colored<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}{}{}", self.style, self.text, Color::RESET)
    }
}

/// Create colored text
///
/// # Example
/// ```
/// println!("{}", colored(Style::ERROR, "Error!"));
/// ```
pub const fn colored<'a>(style: &'a str, text: &'a str) -> Colored<'a> {
    Colored::new(style, text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_escape_codes() {
        assert_eq!(Color::RESET, "\x1b[0m");
        assert_eq!(Color::RED, "\x1b[31m");
        assert_eq!(Style::ERROR, "\x1b[1;31m");
    }
}
