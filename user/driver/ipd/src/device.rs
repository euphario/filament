//! smoltcp Device trait implementation bridging to DataPort.
//!
//! Zero-copy design: RX frames stay in the DataPort pool and smoltcp reads
//! them directly via raw pointer. TX frames are written directly into pool
//! space allocated on demand.

use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use libos::bus::BlockTransport;
use libos::ring::{IoSqe, IoCqe, io_op, io_status};
use libos::udebug;

/// Ethernet header: dst(6) + src(6) + ethertype(2).
const ETH_HEADER_LEN: usize = 14;
/// IP MTU — matches smoltcp capabilities and NIC sidechannel.
pub(crate) const MTU: usize = 1500;
/// Maximum Ethernet frame size (header + payload, no FCS).
pub(crate) const MAX_FRAME_SIZE: usize = MTU + ETH_HEADER_LEN;

/// Number of buffered RX frame references.
const RX_QUEUE_SIZE: usize = 16;

/// Lightweight reference to a received frame in the DataPort pool.
#[derive(Clone, Copy)]
struct RxFrameRef {
    offset: u32,
    len: u32,
}

/// Ring buffer of pool-offset references to received Ethernet frames.
///
/// Populated in `drain_rx()` from DataPort CQEs (offset + len only),
/// drained by smoltcp via `Device::receive()`. No frame data is copied —
/// 128 bytes total instead of 24KB.
pub struct RxOffsetQueue {
    refs: [RxFrameRef; RX_QUEUE_SIZE],
    head: usize,
    tail: usize,
    count: usize,
}

impl RxOffsetQueue {
    pub const fn new() -> Self {
        Self {
            refs: [RxFrameRef { offset: 0, len: 0 }; RX_QUEUE_SIZE],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Push a received frame reference. Drops if full.
    pub fn push(&mut self, offset: u32, len: u32) {
        if self.count >= RX_QUEUE_SIZE || len == 0 || len > MAX_FRAME_SIZE as u32 {
            return;
        }
        self.refs[self.tail] = RxFrameRef { offset, len };
        self.tail = (self.tail + 1) % RX_QUEUE_SIZE;
        self.count += 1;
    }

    /// Pop the oldest frame reference.
    fn pop(&mut self) -> Option<RxFrameRef> {
        if self.count == 0 {
            return None;
        }
        let r = self.refs[self.head];
        self.head = (self.head + 1) % RX_QUEUE_SIZE;
        self.count -= 1;
        Some(r)
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn count(&self) -> usize {
        self.count
    }
}

// =============================================================================
// TX Offset Tracking
// =============================================================================

/// Maximum in-flight TX requests (matches ring_size).
const TX_INFLIGHT_SIZE: usize = 256;

/// Tracks consumer-side TX pool offsets keyed by SQE tag.
///
/// When ipd submits a TX frame to the provider (wifid/ethd), it stores
/// the pool offset here. When the TX CQE comes back (CQE_FLAG_TX_DONE),
/// ipd looks up the offset by tag and frees the pool slot.
pub struct TxTracker {
    /// Pool offsets indexed by (tag & mask). 0 = unused.
    offsets: [u32; TX_INFLIGHT_SIZE],
    /// Next tag counter (wraps, skips 0).
    next_tag: u32,
}

impl TxTracker {
    pub const fn new() -> Self {
        Self {
            offsets: [0; TX_INFLIGHT_SIZE],
            next_tag: 1,
        }
    }

    /// Allocate a tag and record the pool offset. Returns the tag.
    pub fn track(&mut self, offset: u32, mask: u32) -> u32 {
        let tag = self.next_tag;
        self.next_tag = self.next_tag.wrapping_add(1);
        if self.next_tag == 0 {
            self.next_tag = 1;
        }
        let idx = (tag & mask) as usize;
        if idx < TX_INFLIGHT_SIZE {
            self.offsets[idx] = offset;
        }
        tag
    }

    /// Look up and remove the pool offset for a completed TX tag.
    pub fn complete(&mut self, tag: u32, mask: u32) -> Option<u32> {
        let idx = (tag & mask) as usize;
        if idx < TX_INFLIGHT_SIZE {
            let offset = self.offsets[idx];
            self.offsets[idx] = 0;
            if offset != 0 { Some(offset) } else { None }
        } else {
            None
        }
    }
}

// =============================================================================
// Data Path Statistics
// =============================================================================

/// Counters for the ipd data path.
pub struct DataPathStats {
    pub rx_frames: u32,
    pub tx_frames: u32,
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub tx_pool_drops: u32,
    pub rx_pool_full: u32,
    pub tx_cqe_reclaimed: u32,
    pub tx_pool_full: u32,
}

impl DataPathStats {
    pub const fn new() -> Self {
        Self {
            rx_frames: 0,
            tx_frames: 0,
            rx_bytes: 0,
            tx_bytes: 0,
            tx_pool_drops: 0,
            rx_pool_full: 0,
            tx_cqe_reclaimed: 0,
            tx_pool_full: 0,
        }
    }
}

// =============================================================================
// SmolDevice
// =============================================================================

/// smoltcp Device backed by a netd DataPort.
///
/// The lifetime `'a` is for the BlockTransport reference used during a single
/// `iface.poll()` call.
pub struct SmolDevice<'a> {
    port: &'a mut dyn BlockTransport,
    rx_queue: &'a mut RxOffsetQueue,
    tx_tracker: &'a mut TxTracker,
    stats: &'a mut DataPathStats,
    pool_base: *const u8,
    pool_size: u32,
    group_id: u8,
    ring_mask: u32,
}

impl<'a> SmolDevice<'a> {
    pub fn new(
        port: &'a mut dyn BlockTransport,
        rx_queue: &'a mut RxOffsetQueue,
        tx_tracker: &'a mut TxTracker,
        stats: &'a mut DataPathStats,
        group_id: u8,
    ) -> Self {
        let pool_base = port.pool_ptr();
        let pool_size = port.pool_size();
        let ring_mask = port.ring_mask();
        Self { port, rx_queue, tx_tracker, stats, pool_base, pool_size, group_id, ring_mask }
    }
}

impl<'a> Device for SmolDevice<'a> {
    type RxToken<'t>
        = IpdRxToken
    where
        Self: 't;
    type TxToken<'t>
        = IpdTxToken<'t>
    where
        Self: 't;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if let Some(r) = self.rx_queue.pop() {
            let rx = IpdRxToken {
                pool_base: self.pool_base,
                pool_size: self.pool_size,
                offset: r.offset,
                len: r.len,
            };
            // RX path: don't pre-allocate for the response TX token.
            // Processing the incoming frame (TCP ACK, etc.) is more important
            // than sending a response. If pool is full, consume() uses a
            // scratch buffer and the response is silently dropped.
            let tx = IpdTxToken {
                port: self.port,
                tx_tracker: self.tx_tracker,
                stats: self.stats,
                group_id: self.group_id,
                ring_mask: self.ring_mask,
                pre_offset: None,
            };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        // Pre-allocate a pool slot. If the pool is full, return None so
        // smoltcp backs off and retries on the next poll cycle.
        // This prevents the panic from handing smoltcp a 0-byte buffer.
        let offset = match self.port.alloc(MAX_FRAME_SIZE as u32) {
            Some(o) => o,
            None => {
                self.stats.tx_pool_full += 1;
                return None;
            }
        };
        Some(IpdTxToken {
            port: self.port,
            tx_tracker: self.tx_tracker,
            stats: self.stats,
            group_id: self.group_id,
            ring_mask: self.ring_mask,
            pre_offset: Some(offset),
        })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = None;
        caps
    }
}

/// RX token: zero-copy reference into the DataPort pool.
///
/// The pool memory is shared and stable for the lifetime of the DataPort.
/// The token is consumed synchronously within a single `iface.poll()` call,
/// so the raw pointer is safe to use.
pub struct IpdRxToken {
    pool_base: *const u8,
    pool_size: u32,
    offset: u32,
    len: u32,
}

impl phy::RxToken for IpdRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        let end = self.offset as u64 + self.len as u64;
        if end <= self.pool_size as u64 {
            // SAFETY: pool_base points to mapped shared memory that outlives
            // this token. The offset+len range was validated by the CQE from
            // netd (provider side). This token is consumed synchronously.
            let slice = unsafe {
                core::slice::from_raw_parts(
                    self.pool_base.add(self.offset as usize),
                    self.len as usize,
                )
            };
            f(slice)
        } else {
            f(&[])
        }
    }
}

/// TX token: writes a frame directly to DataPort pool and submits to provider.
///
/// The group_id is encoded in IoSqe.flags so ethd inserts the correct
/// MTK special tag destination port mask for this bridge group.
/// A unique tag is assigned via TxTracker so the TX CQE can be matched
/// for pool reclaim.
///
/// `pre_offset` holds a pre-allocated pool slot from `transmit()`.
/// When set, `consume()` uses it directly (guaranteed space).
/// When None (from `receive()` fallback), `consume()` tries to alloc
/// on demand, using a scratch buffer if the pool is full.
pub struct IpdTxToken<'a> {
    port: &'a mut dyn BlockTransport,
    tx_tracker: &'a mut TxTracker,
    stats: &'a mut DataPathStats,
    group_id: u8,
    ring_mask: u32,
    pre_offset: Option<u32>,
}

impl<'a> phy::TxToken for IpdTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let offset = self.pre_offset.or_else(|| self.port.alloc(len as u32));

        if let Some(offset) = offset {
            // Zero-copy: smoltcp writes directly into pool memory.
            let result = if let Some(buf) = self.port.pool_slice_mut(offset, len as u32) {
                f(buf)
            } else {
                // pool_slice_mut failed (shouldn't happen) — scratch buffer.
                let mut scratch = [0u8; MAX_FRAME_SIZE];
                f(&mut scratch[..len.min(MAX_FRAME_SIZE)])
            };

            let tag = self.tx_tracker.track(offset, self.ring_mask);

            let sqe = IoSqe {
                opcode: io_op::NET_SEND,
                flags: self.group_id,
                priority: 0,
                tag,
                data_offset: offset,
                data_len: len as u32,
                lba: 0,
                param: 0,
            };
            if !self.port.submit(&sqe) {
                udebug!("ipd", "tx_sq_full"; len = len as u32);
            }
            self.port.notify();
            self.stats.tx_frames += 1;
            self.stats.tx_bytes += len as u64;

            // Log first few TX frames for diagnostics
            if self.stats.tx_frames <= 5 || self.stats.tx_frames % 1000 == 0 {
                udebug!("ipd", "tx_frame"; len = len as u32,
                    total = self.stats.tx_frames, drops = self.stats.tx_pool_drops);
            }

            result
        } else {
            // No pool space — absorb the frame into a scratch buffer.
            // smoltcp considers this "sent" but no packet goes on the wire.
            // TCP will retransmit when RTO fires.
            self.stats.tx_pool_drops += 1;
            udebug!("ipd", "tx_pool_drop"; len = len as u32,
                drops = self.stats.tx_pool_drops);
            let mut scratch = [0u8; MAX_FRAME_SIZE];
            f(&mut scratch[..len.min(MAX_FRAME_SIZE)])
        }
    }
}
