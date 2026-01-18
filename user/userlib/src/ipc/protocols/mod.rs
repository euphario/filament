//! Concrete Protocol Implementations
//!
//! Type-safe protocol definitions for kernel services and drivers.
//!
//! # Available Protocols
//!
//! | Protocol | Port Name | Description |
//! |----------|-----------|-------------|
//! | [`DevdProtocol`](devd::DevdProtocol) | `devd` | Device tree queries, registration, subscriptions |
//! | [`PcieProtocol`](pcie::PcieProtocol) | `pcie` | PCIe device enumeration and port control |
//! | [`GpioProtocol`](gpio::GpioProtocol) | `gpio` | GPIO pin read/write control |
//! | [`FsProtocol`](filesystem::FsProtocol) | `fatfs` | USB filesystem operations (read-write) |
//! | [`VfsProtocol`](filesystem::VfsProtocol) | `vfs` | Virtual filesystem (initrd, read-only) |
//! | [`BlockProtocol`](block::BlockProtocol) | - | Block device handshake (uses shared memory ring) |
//!
//! # Usage
//!
//! Most protocols have a convenience client wrapper:
//!
//! ```rust,ignore
//! use userlib::ipc::{DevdClient, GpioClient, PcieClient, FsClient, VfsClient};
//!
//! // Device tree
//! let mut devd = DevdClient::connect()?;
//! let nodes = devd.query("/bus/**")?;
//!
//! // GPIO control
//! let mut gpio = GpioClient::connect()?;
//! gpio.set_pin(11, true)?;
//!
//! // PCIe enumeration
//! let mut pcie = PcieClient::connect()?;
//! let devices = pcie.list_devices()?;
//!
//! // USB Filesystem (read-write)
//! let mut fs = FsClient::connect()?;
//! let stat = fs.stat("/boot/kernel.bin")?;
//!
//! // VFS (initrd, read-only)
//! let mut vfs = VfsClient::connect()?;
//! let info = vfs.get_info()?;  // Check capabilities
//! let (entries, count) = vfs.read_dir(b"/bin")?;
//! ```
//!
//! # Adding New Protocols
//!
//! 1. Create a new module (e.g., `myproto.rs`)
//! 2. Define a protocol marker struct implementing [`Protocol`](super::Protocol)
//! 3. Define Request and Response enums implementing [`Message`](super::Message)
//! 4. Optionally add a convenience client wrapper
//! 5. Re-export from this module

pub mod block;
pub mod console;
pub mod devd;
pub mod filesystem;
pub mod gpio;
pub mod logd;
pub mod pcie;

// devd protocol (device supervisor)
// Note: Constants available via devd::MAX_PATH, devd::MAX_KEY, etc.
pub use devd::{
    DevdProtocol, DevdRequest, DevdResponse, DevdClient,
    Property, PropertyList, Node, NodeList, EventType,
};

// PCIe protocol
pub use pcie::{PcieProtocol, PcieRequest, PcieResponse, PcieDeviceInfo, PcieClient, DeviceList};

// GPIO protocol
pub use gpio::{GpioProtocol, GpioRequest, GpioResponse, GpioClient};

// Filesystem protocol
// Note: Constants available via filesystem::MAX_PATH, filesystem::MAX_INLINE_DATA, etc.
pub use filesystem::{
    FsProtocol, VfsProtocol, FsRequest, FsResponse, FsClient, VfsClient,
    FileStat, FsInfo, DirEntry, FileType,
    flags as fs_flags, error as fs_error, caps as fs_caps,
};

// Block device protocol
pub use block::{
    BlockProtocol, BlockHandshake, BlockHandshakeResponse,
    BlockOp, status as block_status, port as block_port,
    BlockRing, BlockRequest, BlockResponse,
};

// Console protocol
pub use console::{
    ConsoleProtocol, ConsoleRequest, ConsoleResponse, ConsoleClient,
    InputState, MAX_INPUT_SIZE, MAX_WRITE_SIZE,
};

// Log daemon protocol
pub use logd::{
    LogdProtocol, LogdRequest, LogdEvent, MAX_LOG_RECORD,
};
