//! High-level Data Port API for Layered Drivers
//!
//! DataPort wraps the layered ring protocol with a role-based API:
//! - **Provider** (lower layer): receives requests, sends completions
//! - **Consumer** (upper layer): sends requests, receives completions
//!
//! ```text
//!  ┌──────────────┐                    ┌──────────────┐
//!  │   Consumer   │                    │   Provider   │
//!  │ (upper layer)│                    │ (lower layer)│
//!  │              │                    │              │
//!  │  alloc() ────┼──► pool ◄─────────┼── DMA        │
//!  │  submit() ───┼──► SQ ───────────►┼── recv()     │
//!  │  poll_cq() ◄─┼─── CQ ◄───────────┼── complete() │
//!  │  query() ────┼──► side ─────────►┼── poll_side()│
//!  └──────────────┘                    └──────────────┘
//! ```
//!
//! ## Usage
//!
//! ```no_run
//! // Provider creates the port
//! let port = DataPort::create(DataPortConfig {
//!     ring_size: 64,
//!     side_size: 8,
//!     pool_size: 1024 * 1024,
//! })?;
//! port.allow(consumer_pid);
//!
//! // Consumer connects
//! let port = DataPort::connect(shmem_id)?;
//!
//! // Consumer: allocate, submit, wait for completion
//! let offset = port.alloc(512)?;
//! port.submit(&IoSqe { data_offset: offset, ... });
//! port.notify();
//! let cqe = port.poll_cq()?;
//! let data = port.pool_slice(offset, 512);
//!
//! // Provider: receive, process, complete
//! let sqe = port.recv()?;
//! // DMA to pool at sqe.data_offset
//! port.complete(&IoCqe { tag: sqe.tag, status: OK, ... });
//! port.notify();
//! ```

use crate::ring::{
    LayeredRing, IoSqe, IoCqe, SideEntry, PoolAlloc,
    io_status, side_status, side_msg,
};
use crate::error::SysError;

// =============================================================================
// Configuration
// =============================================================================

/// Configuration for creating a DataPort
#[derive(Clone, Copy)]
pub struct DataPortConfig {
    /// Number of SQ/CQ entries (power of 2)
    pub ring_size: u16,
    /// Number of sidechannel entries (power of 2, 0 to disable)
    pub side_size: u16,
    /// Size of data buffer pool in bytes
    pub pool_size: u32,
}

impl Default for DataPortConfig {
    fn default() -> Self {
        Self {
            ring_size: 64,
            side_size: 8,
            pool_size: 256 * 1024, // 256KB default pool
        }
    }
}

// =============================================================================
// Port Role
// =============================================================================

/// Role of this side of the data port
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortRole {
    /// Provider (lower layer): receives requests, sends completions
    Provider,
    /// Consumer (upper layer): sends requests, receives completions
    Consumer,
}

// =============================================================================
// DataPort
// =============================================================================

/// High-level data port for layered I/O
///
/// Wraps a LayeredRing with role-based API and pool allocation.
pub struct DataPort {
    ring: LayeredRing,
    role: PortRole,
    /// Pool allocator (only used by Consumer)
    alloc: Option<PoolAlloc>,
    /// Next tag for requests (only used by Consumer)
    next_tag: u32,
}

impl DataPort {
    /// Create a new data port (Provider side)
    ///
    /// The provider creates the shared memory and waits for consumers.
    pub fn create(config: DataPortConfig) -> Result<Self, SysError> {
        let ring = LayeredRing::create(config.ring_size, config.side_size, config.pool_size)
            .ok_or(SysError::OutOfMemory)?;

        Ok(Self {
            ring,
            role: PortRole::Provider,
            alloc: None, // Provider doesn't allocate from pool
            next_tag: 0,
        })
    }

    /// Connect to an existing data port (Consumer side)
    ///
    /// The consumer maps the shared memory created by the provider.
    pub fn connect(shmem_id: u32) -> Result<Self, SysError> {
        let ring = LayeredRing::map(shmem_id)
            .ok_or(SysError::InvalidArgument)?;

        // Consumer manages pool allocation
        let alloc = PoolAlloc::new(0, ring.pool_size());

        Ok(Self {
            ring,
            role: PortRole::Consumer,
            alloc: Some(alloc),
            next_tag: 1,
        })
    }

    /// Get the shared memory ID (to send to peer)
    pub fn shmem_id(&self) -> u32 {
        self.ring.shmem_id()
    }

    /// Allow another process to connect
    pub fn allow(&self, peer_pid: u32) -> bool {
        self.ring.allow(peer_pid)
    }

    /// Get the role of this port
    pub fn role(&self) -> PortRole {
        self.role
    }

    // =========================================================================
    // Pool Allocation (Consumer only)
    // =========================================================================

    /// Allocate buffer in pool
    ///
    /// Returns offset into pool, or None if no space.
    /// Only valid for Consumer role.
    pub fn alloc(&mut self, size: u32) -> Option<u32> {
        self.alloc.as_mut()?.alloc(size)
    }

    /// Reset pool allocator (free all allocations)
    ///
    /// Only valid for Consumer role.
    /// WARNING: Only call when no requests are in flight!
    pub fn reset_pool(&mut self) {
        if let Some(alloc) = &mut self.alloc {
            alloc.reset();
        }
    }

    /// Get remaining pool space
    pub fn pool_remaining(&self) -> u32 {
        self.alloc.as_ref().map(|a| a.remaining()).unwrap_or(self.ring.pool_size())
    }

    // =========================================================================
    // Submission Queue (Consumer → Provider)
    // =========================================================================

    /// Generate next tag for request tracking
    pub fn next_tag(&mut self) -> u32 {
        let tag = self.next_tag;
        self.next_tag = self.next_tag.wrapping_add(1);
        if self.next_tag == 0 {
            self.next_tag = 1; // Skip 0
        }
        tag
    }

    /// Check if SQ has space
    pub fn sq_space(&self) -> u32 {
        self.ring.sq_space()
    }

    /// Submit a request (Consumer → Provider)
    pub fn submit(&self, sqe: &IoSqe) -> bool {
        self.ring.sq_submit(sqe)
    }

    /// Submit a read request (convenience method)
    pub fn submit_read(&mut self, lba: u64, offset: u32, len: u32) -> Option<u32> {
        let tag = self.next_tag();
        let sqe = IoSqe {
            opcode: crate::ring::io_op::READ,
            flags: 0,
            priority: 0,
            tag,
            lba,
            data_offset: offset,
            data_len: len,
            param: 0,
        };
        if self.submit(&sqe) {
            Some(tag)
        } else {
            None
        }
    }

    /// Submit a write request (convenience method)
    pub fn submit_write(&mut self, lba: u64, offset: u32, len: u32) -> Option<u32> {
        let tag = self.next_tag();
        let sqe = IoSqe {
            opcode: crate::ring::io_op::WRITE,
            flags: 0,
            priority: 0,
            tag,
            lba,
            data_offset: offset,
            data_len: len,
            param: 0,
        };
        if self.submit(&sqe) {
            Some(tag)
        } else {
            None
        }
    }

    /// Receive next request (Provider side)
    pub fn recv(&self) -> Option<IoSqe> {
        self.ring.sq_consume()
    }

    /// Check pending requests
    pub fn sq_pending(&self) -> u32 {
        self.ring.sq_pending()
    }

    // =========================================================================
    // Completion Queue (Provider → Consumer)
    // =========================================================================

    /// Complete a request (Provider → Consumer)
    pub fn complete(&self, cqe: &IoCqe) -> bool {
        self.ring.cq_complete(cqe)
    }

    /// Complete with success (convenience method)
    pub fn complete_ok(&self, tag: u32, transferred: u32) -> bool {
        self.complete(&IoCqe {
            status: io_status::OK,
            flags: 0,
            tag,
            transferred,
            result: 0,
        })
    }

    /// Complete with error (convenience method)
    pub fn complete_error(&self, tag: u32, status: u16) -> bool {
        self.complete(&IoCqe {
            status,
            flags: 0,
            tag,
            transferred: 0,
            result: 0,
        })
    }

    /// Poll for completion (Consumer side)
    pub fn poll_cq(&self) -> Option<IoCqe> {
        self.ring.cq_consume()
    }

    /// Check pending completions
    pub fn cq_pending(&self) -> u32 {
        self.ring.cq_pending()
    }

    // =========================================================================
    // Sidechannel (bidirectional queries/control)
    // =========================================================================

    /// Check if sidechannel is enabled
    pub fn has_sidechannel(&self) -> bool {
        self.ring.has_sidechannel()
    }

    /// Send sidechannel entry
    pub fn side_send(&self, entry: &SideEntry) -> bool {
        self.ring.side_send(entry)
    }

    /// Send a query and wait for response
    ///
    /// Returns the response, or None on timeout.
    pub fn query(&mut self, msg_type: u16, payload: &[u8]) -> Option<SideEntry> {
        use crate::ring::side_status;

        if !self.has_sidechannel() {
            return None;
        }

        let tag = (self.next_tag() & 0xFFFF) as u16;
        let mut entry = SideEntry {
            msg_type,
            flags: 0,
            tag,
            status: side_status::REQUEST, // Mark as request (not a response)
            payload: [0; 24],
        };
        let copy_len = payload.len().min(24);
        entry.payload[..copy_len].copy_from_slice(&payload[..copy_len]);

        if !self.side_send(&entry) {
            return None;
        }
        self.notify();

        // Wait for response (simple blocking wait)
        // In production, integrate with event loop
        for _ in 0..100 {
            // Use poll_side_response() which doesn't consume REQUEST entries
            // This allows the provider to read our request while we wait
            if let Some(resp) = self.poll_side_response() {
                if resp.tag == tag {
                    return Some(resp);
                }
            }
            // Use short sleep since ring.wait() doesn't support timeout properly
            crate::syscall::sleep_us(1000);
        }
        None
    }

    /// Poll for sidechannel entry (consumes any entry)
    pub fn poll_side(&self) -> Option<SideEntry> {
        self.ring.side_recv()
    }

    /// Poll for sidechannel response (Consumer helper)
    ///
    /// Only consumes entries with status != REQUEST.
    /// This allows the provider to consume REQUEST entries while
    /// the consumer waits for RESPONSE entries.
    pub fn poll_side_response(&self) -> Option<SideEntry> {
        use crate::ring::side_status;

        // Peek at the next entry without consuming
        let entry = self.ring.side_peek()?;

        // If it's a request (not a response), don't consume it
        // Let the provider read it instead
        if entry.status == side_status::REQUEST {
            return None;
        }

        // It's a response - consume and return it
        self.ring.side_recv()
    }

    /// Poll for sidechannel request (Provider helper)
    ///
    /// Only consumes entries with status == REQUEST.
    /// This allows the consumer to receive responses while
    /// provider only processes requests.
    pub fn poll_side_request(&self) -> Option<SideEntry> {
        use crate::ring::side_status;

        // Peek at the next entry without consuming
        let entry = self.ring.side_peek()?;

        // If it's NOT a request, don't consume it
        // It's a response for the consumer to read
        if entry.status != side_status::REQUEST {
            return None;
        }

        // It's a request - consume and return it
        self.ring.side_recv()
    }

    /// Check pending sidechannel entries
    pub fn side_pending(&self) -> u32 {
        self.ring.side_pending()
    }

    /// Handle a sidechannel query (Provider helper)
    ///
    /// Calls handler with the query. Handler returns:
    /// - Some(response) to answer the query
    /// - None to pass it down (sets PASS_DOWN status)
    pub fn handle_query<F>(&self, entry: &SideEntry, handler: F) -> bool
    where
        F: FnOnce(&SideEntry) -> Option<SideEntry>,
    {
        match handler(entry) {
            Some(mut response) => {
                response.tag = entry.tag;
                response.status = side_status::OK;
                self.side_send(&response)
            }
            None => {
                // Pass down - send back with PASS_DOWN status
                let mut pass = *entry;
                pass.status = side_status::PASS_DOWN;
                self.side_send(&pass)
            }
        }
    }

    // =========================================================================
    // Notifications
    // =========================================================================

    /// Notify peer (ring doorbell)
    pub fn notify(&self) {
        self.ring.notify();
    }

    /// Wait for notification
    pub fn wait(&self, timeout_ms: u32) -> bool {
        self.ring.wait(timeout_ms)
    }

    // =========================================================================
    // Pool Access
    // =========================================================================

    /// Get pool size
    pub fn pool_size(&self) -> u32 {
        self.ring.pool_size()
    }

    /// Get pool physical address (for DMA)
    pub fn pool_phys(&self) -> u64 {
        self.ring.pool_phys()
    }

    /// Get slice into pool at offset
    pub fn pool_slice(&self, offset: u32, len: u32) -> Option<&[u8]> {
        self.ring.pool_slice(offset, len)
    }

    /// Get mutable slice into pool at offset
    pub fn pool_slice_mut(&self, offset: u32, len: u32) -> Option<&mut [u8]> {
        self.ring.pool_slice_mut(offset, len)
    }

    /// Write data to pool at offset
    pub fn pool_write(&self, offset: u32, data: &[u8]) -> bool {
        if let Some(slice) = self.ring.pool_slice_mut(offset, data.len() as u32) {
            slice.copy_from_slice(data);
            true
        } else {
            false
        }
    }

    /// Read data from pool at offset
    pub fn pool_read(&self, offset: u32, buf: &mut [u8]) -> bool {
        if let Some(slice) = self.ring.pool_slice(offset, buf.len() as u32) {
            buf.copy_from_slice(slice);
            true
        } else {
            false
        }
    }
}

// =============================================================================
// Query Helpers
// =============================================================================

/// Query: Get block device geometry
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct GeometryInfo {
    pub block_size: u32,
    pub block_count: u64,
    pub max_transfer: u32,
}

impl DataPort {
    /// Query block geometry from provider
    pub fn query_geometry(&mut self) -> Option<GeometryInfo> {
        let response = self.query(side_msg::QUERY_GEOMETRY, &[])?;
        if response.status != side_status::OK {
            return None;
        }

        // Parse from payload
        let payload = &response.payload;
        Some(GeometryInfo {
            block_size: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            block_count: u64::from_le_bytes([
                payload[4], payload[5], payload[6], payload[7],
                payload[8], payload[9], payload[10], payload[11],
            ]),
            max_transfer: u32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
        })
    }

    /// Respond to geometry query (Provider helper)
    pub fn respond_geometry(&self, entry: &SideEntry, info: &GeometryInfo) -> bool {
        let mut response = SideEntry::default();
        response.msg_type = side_msg::QUERY_GEOMETRY;
        response.tag = entry.tag;
        response.status = side_status::OK;

        // Pack into payload
        response.payload[0..4].copy_from_slice(&info.block_size.to_le_bytes());
        response.payload[4..12].copy_from_slice(&info.block_count.to_le_bytes());
        response.payload[12..16].copy_from_slice(&info.max_transfer.to_le_bytes());

        self.side_send(&response)
    }
}

// =============================================================================
// Layer Trait
// =============================================================================

/// Trait for implementing a processing layer
///
/// Each layer sits between consumer (above) and provider (below):
/// - Receives requests from above via `process_request()`
/// - Transforms and forwards to layer below
/// - Receives completions from below via `process_completion()`
/// - Transforms and forwards to layer above
pub trait Layer {
    /// Process a request from above
    ///
    /// Called when a request arrives from the consumer.
    /// The layer should transform it and submit to the provider below.
    fn process_request(&mut self, sqe: IoSqe);

    /// Process a completion from below
    ///
    /// Called when a completion arrives from the provider.
    /// The layer should transform it and complete to the consumer above.
    fn process_completion(&mut self, cqe: IoCqe);

    /// Handle a sidechannel query
    ///
    /// Return Some(response) to answer, None to pass down.
    fn handle_query(&mut self, entry: &SideEntry) -> Option<SideEntry> {
        // Default: pass down
        let _ = entry;
        None
    }

    /// Tick - called periodically for housekeeping
    fn tick(&mut self) {}
}

// =============================================================================
// LayerStack - connects layers with DataPorts
// =============================================================================

/// A connected layer with its ports
pub struct ConnectedLayer<L: Layer> {
    /// The layer implementation
    pub layer: L,
    /// Port to layer above (we are provider)
    pub above: Option<DataPort>,
    /// Port to layer below (we are consumer)
    pub below: Option<DataPort>,
}

impl<L: Layer> ConnectedLayer<L> {
    /// Create a new connected layer
    pub fn new(layer: L) -> Self {
        Self {
            layer,
            above: None,
            below: None,
        }
    }

    /// Connect to layer above (create port as provider)
    pub fn create_port_above(&mut self, config: DataPortConfig) -> Result<u32, SysError> {
        let port = DataPort::create(config)?;
        let id = port.shmem_id();
        self.above = Some(port);
        Ok(id)
    }

    /// Connect to layer below (connect as consumer)
    pub fn connect_below(&mut self, shmem_id: u32) -> Result<(), SysError> {
        let port = DataPort::connect(shmem_id)?;
        self.below = Some(port);
        Ok(())
    }

    /// Process pending work
    pub fn poll(&mut self) {
        // Process requests from above
        if let Some(above) = &self.above {
            while let Some(sqe) = above.recv() {
                self.layer.process_request(sqe);
            }
        }

        // Process completions from below
        if let Some(below) = &self.below {
            while let Some(cqe) = below.poll_cq() {
                self.layer.process_completion(cqe);
            }
        }

        // Process sidechannel queries from above
        if let Some(above) = &self.above {
            while let Some(entry) = above.poll_side() {
                if let Some(response) = self.layer.handle_query(&entry) {
                    let mut resp = response;
                    resp.tag = entry.tag;
                    resp.status = side_status::OK;
                    above.side_send(&resp);
                } else {
                    // Pass down to layer below
                    if let Some(below) = &self.below {
                        below.side_send(&entry);
                    } else {
                        // End of line
                        let mut eol = entry;
                        eol.status = side_status::EOL;
                        above.side_send(&eol);
                    }
                }
            }
        }

        // Tick for housekeeping
        self.layer.tick();
    }

    /// Notify both directions
    pub fn notify(&self) {
        if let Some(above) = &self.above {
            above.notify();
        }
        if let Some(below) = &self.below {
            below.notify();
        }
    }
}
