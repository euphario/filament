//! User-space runtime library for BPI-R4 kernel
//!
//! Minimal library for unified 5-syscall interface.

#![no_std]

pub mod error;
pub mod syscall;
pub mod ipc;
pub mod io;
pub mod console_ring;
pub mod query;
pub mod mmio;
pub mod ring;
pub mod data_port;
pub mod devd;
pub mod blk;
pub mod sync;
pub mod config;

// Legacy modules — kept for compilation but not re-exported.
// console_ring: still used by consoled.
// command_ring, driver_ring, filter_chain, blk_client: superseded by bus framework.
mod filter_chain;
mod blk_client;
mod command_ring;
mod driver_ring;
pub mod bus;
pub mod bus_transport;
pub mod bus_runtime;
pub mod bus_block;
pub mod vfs_proto;
pub mod vfs_client;
pub mod serialize;
#[macro_use]
pub mod ulog;

pub use error::{SysError, SysResult};
pub use syscall::{LogLevel, Handle, ObjHandle, ObjectType};
pub use syscall::{exit, exec, klog};
pub use syscall::{open, read, write, map, close, channel_pair};
pub use ipc::{Channel, Port, Timer, Mux, MuxFilter, MuxEvent, Process, Message, ObjHandle as IpcHandle, EventLoop, Shmem, PciDevice, Msi, MsiInfo, PipeRing};
pub use mmio::{MmioRegion, DmaPool, delay_ms, delay_us, poll_until, poll_interval};
pub use ring::{Ring, LayeredRing, IoSqe, IoCqe, SideEntry, PoolAlloc, io_op, io_status, side_msg, side_status, Doorbell, ChannelDoorbell};
pub use data_port::{DataPort, DataPortConfig, PortRole, Layer, ConnectedLayer, GeometryInfo};
pub use devd::{
    DevdClient, DeviceClass, DeviceInfo, ClientState, DriverState,
    DevdCommand, SpawnFilter, SpawnResult,
};
pub use bus::{
    BusMsg, BusMsgFlags, BusError, BusCtx, Driver, Disposition, ConfigKey,
    ChildId, PortId, BlockTransport, BlockPortConfig, BlockGeometry,
    BlockCompletion, PortError, bus_msg as bus_msg_types,
};
pub use bus_block::ShmemBlockPort;
pub use vfs_proto::{VfsDirEntry, VfsStat, fs_op, open_flags, file_type, vfs_error};
pub use vfs_client::{VfsClient, VfsError};
pub use bus_runtime::driver_main;

// Entry point - called by _start
unsafe extern "Rust" {
    fn main();
}

/// Program entry point
#[unsafe(no_mangle)]
#[unsafe(naked)]
#[unsafe(link_section = ".text._start")]
pub extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        "bl {main}",
        "mov x8, #0",
        "svc #0",
        "b .",
        main = sym main,
    )
}

/// Panic handler
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Minimal panic - just log and exit
    let mut buf = [0u8; 64];
    let msg = if let Some(loc) = info.location() {
        let file = loc.file().as_bytes();
        let len = file.len().min(40);
        buf[..7].copy_from_slice(b"PANIC: ");
        buf[7..7+len].copy_from_slice(&file[..len]);
        &buf[..7+len]
    } else {
        b"PANIC: unknown"
    };
    syscall::klog(syscall::LogLevel::Error, msg);
    syscall::exit(1)
}
