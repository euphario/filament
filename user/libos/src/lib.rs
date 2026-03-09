//! Filament OS driver SDK
//!
//! High-level framework for userspace drivers: bus framework, query protocol,
//! data ports, devd client, VFS, structured logging, MMIO/DMA.
//!
//! Depends on `libsys` for kernel boundary (syscalls, IPC, error types,
//! ring protocol, data ports).

#![no_std]

extern crate alloc;

pub mod serialize;
pub mod hash_map;
pub mod blk;

#[macro_use]
pub mod ulog;

pub mod wire;
pub mod query;
pub mod mmio;
pub mod dma;
pub mod config;
pub mod supervision;

pub mod devd;
pub mod vfs_proto;
pub mod vfs_client;

pub mod bus;
pub mod bus_transport;
pub mod bus_block;
pub mod bus_runtime;

// Re-export ring and data_port from libsys for convenience
pub use libsys::ring;
pub use libsys::data_port;

pub use bus::{
    BusMsg, BusMsgFlags, BusError, BusCtx, Driver, Disposition, ConfigKey,
    PortId, BlockTransport, BlockPortConfig, BlockGeometry,
    BlockCompletion, PortError, bus_msg as bus_msg_types,
};
pub use abi::{BusDevice, bus_device_flags};
pub use bus_block::ShmemBlockPort;
pub use bus_runtime::driver_main;
pub use libsys::ring::{Ring, LayeredRing, IoSqe, IoCqe, SideEntry, PoolAlloc, io_op, io_status, side_msg, side_status, Doorbell, ChannelDoorbell};
pub use libsys::data_port::{DataPort, DataPortConfig, PortRole, Layer, ConnectedLayer, GeometryInfo};
pub use devd::{
    DevdClient, ClientState,
    DevdCommand, SpawnFilter,
};
pub use mmio::{MmioRegion, DmaPool, delay_ms, delay_us, poll_until, poll_interval};
pub use dma::{DmaBuf, DmaDirection, CoherentBuf, StreamingBuf};
pub use vfs_proto::{VfsDirEntry, VfsStat, fs_op, open_flags, file_type, vfs_error};
pub use vfs_client::{VfsClient, VfsError};
pub use supervision::SupervisionHandle;
