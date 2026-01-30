//! Bus Block Transport - BlockTransport wrapping DataPort
//!
//! Implements `BlockTransport` using the existing `DataPort`/`LayeredRing`
//! infrastructure. Both provider and consumer roles are supported.

use crate::bus::{BlockTransport, BlockCompletion, BlockGeometry, PortError};
use crate::data_port::{DataPort, DataPortConfig};
use crate::ring::{IoSqe, IoCqe, SideEntry, io_status};
use crate::error::SysError;

// ============================================================================
// ShmemBlockPort
// ============================================================================

/// Block transport implementation wrapping DataPort.
///
/// Supports both provider (lower layer, receives requests) and
/// consumer (upper layer, sends requests) roles.
pub struct ShmemBlockPort {
    port: DataPort,
}

impl ShmemBlockPort {
    /// Create a new block port as provider.
    pub fn create(config: &crate::bus::BlockPortConfig) -> Result<Self, PortError> {
        let dp_config = DataPortConfig {
            ring_size: config.ring_size,
            side_size: config.side_size,
            pool_size: config.pool_size,
        };
        let port = DataPort::create(dp_config).map_err(|_| PortError::NoSpace)?;
        Ok(Self { port })
    }

    /// Connect to an existing block port as consumer.
    pub fn connect(shmem_id: u32) -> Result<Self, PortError> {
        let port = DataPort::connect(shmem_id).map_err(|_| PortError::NotConnected)?;
        Ok(Self { port })
    }

    /// Get the underlying DataPort for direct access (advanced use).
    pub fn data_port(&self) -> &DataPort {
        &self.port
    }

    /// Get mutable reference to underlying DataPort.
    pub fn data_port_mut(&mut self) -> &mut DataPort {
        &mut self.port
    }
}

impl BlockTransport for ShmemBlockPort {
    fn submit_read(&mut self, lba: u64, buf_offset: u32, len: u32) -> Result<u32, PortError> {
        self.port.submit_read(lba, buf_offset, len).ok_or(PortError::Full)
    }

    fn submit_write(&mut self, lba: u64, buf_offset: u32, len: u32) -> Result<u32, PortError> {
        self.port.submit_write(lba, buf_offset, len).ok_or(PortError::Full)
    }

    fn submit(&self, sqe: &IoSqe) -> bool {
        self.port.submit(sqe)
    }

    fn recv_request(&self) -> Option<IoSqe> {
        self.port.recv()
    }

    fn complete_ok(&self, tag: u32, transferred: u32) -> bool {
        self.port.complete_ok(tag, transferred)
    }

    fn complete_error(&self, tag: u32, status: u16) -> bool {
        self.port.complete_error(tag, status)
    }

    fn complete_with_result(&self, tag: u32, result: u32, transferred: u32) -> bool {
        use crate::ring::{IoCqe, io_status};
        self.port.complete(&IoCqe {
            status: io_status::OK as u16,
            flags: 0,
            tag,
            transferred,
            result,
        })
    }

    fn complete_error_with_result(&self, tag: u32, status: u16, result: u32) -> bool {
        use crate::ring::IoCqe;
        self.port.complete(&IoCqe {
            status,
            flags: 0,
            tag,
            transferred: 0,
            result,
        })
    }

    fn complete(&self, cqe: &IoCqe) -> bool {
        self.port.complete(cqe)
    }

    fn poll_completion(&self) -> Option<BlockCompletion> {
        let cqe = self.port.poll_cq()?;
        Some(BlockCompletion {
            tag: cqe.tag,
            status: cqe.status,
            transferred: cqe.transferred,
            result: cqe.result,
        })
    }

    fn alloc(&mut self, size: u32) -> Option<u32> {
        self.port.alloc(size)
    }

    fn reset_pool(&mut self) {
        self.port.reset_pool();
    }

    fn pool_slice(&self, offset: u32, len: u32) -> Option<&[u8]> {
        self.port.pool_slice(offset, len)
    }

    fn pool_slice_mut(&self, offset: u32, len: u32) -> Option<&mut [u8]> {
        self.port.pool_slice_mut(offset, len)
    }

    fn pool_write(&self, offset: u32, data: &[u8]) -> bool {
        self.port.pool_write(offset, data)
    }

    fn pool_phys(&self) -> u64 {
        self.port.pool_phys()
    }

    fn pool_size(&self) -> u32 {
        self.port.pool_size()
    }

    fn query_geometry(&mut self) -> Option<BlockGeometry> {
        let info = self.port.query_geometry()?;
        Some(BlockGeometry {
            block_size: info.block_size,
            block_count: info.block_count,
            max_transfer: info.max_transfer,
        })
    }

    fn respond_geometry(&self, entry: &SideEntry, info: &BlockGeometry) -> bool {
        let geo = crate::data_port::GeometryInfo {
            block_size: info.block_size,
            block_count: info.block_count,
            max_transfer: info.max_transfer,
        };
        self.port.respond_geometry(entry, &geo)
    }

    fn poll_side_request(&self) -> Option<SideEntry> {
        self.port.poll_side_request()
    }

    fn side_send(&self, entry: &SideEntry) -> bool {
        self.port.side_send(entry)
    }

    fn shmem_id(&self) -> u32 {
        self.port.shmem_id()
    }

    fn set_public(&self) -> bool {
        self.port.set_public()
    }

    fn notify(&self) {
        self.port.notify();
    }

    fn mux_handle(&self) -> Option<crate::syscall::Handle> {
        Some(self.port.shmem_handle())
    }
}
