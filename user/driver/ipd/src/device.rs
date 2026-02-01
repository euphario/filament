//! smoltcp Device trait implementation bridging to DataPort.
//!
//! Zero-copy design: RX frames stay in the DataPort pool and smoltcp reads
//! them directly via raw pointer. TX frames are written directly into pool
//! space allocated on demand.

use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use userlib::bus::BlockTransport;
use userlib::ring::{IoSqe, io_op};

/// Maximum Ethernet frame size (header + payload, no FCS).
const MAX_FRAME_SIZE: usize = 1514;

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
/// Populated in `data_ready()` from DataPort CQEs (offset + len only),
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
}

/// smoltcp Device backed by a netd DataPort.
///
/// The lifetime `'a` is for the BlockTransport reference used during a single
/// `iface.poll()` call.
pub struct SmolDevice<'a> {
    port: &'a mut dyn BlockTransport,
    rx_queue: &'a mut RxOffsetQueue,
    pool_base: *const u8,
    pool_size: u32,
}

impl<'a> SmolDevice<'a> {
    pub fn new(port: &'a mut dyn BlockTransport, rx_queue: &'a mut RxOffsetQueue) -> Self {
        let pool_base = port.pool_ptr();
        let pool_size = port.pool_size();
        Self { port, rx_queue, pool_base, pool_size }
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
            let tx = IpdTxToken { port: self.port };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(IpdTxToken { port: self.port })
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = 1500;
        caps.max_burst_size = Some(1);
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

/// TX token: writes a frame directly to DataPort pool and submits to netd.
pub struct IpdTxToken<'a> {
    port: &'a mut dyn BlockTransport,
}

impl<'a> phy::TxToken for IpdTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        if let Some(offset) = self.port.alloc(len as u32) {
            // Zero-copy: smoltcp writes directly into pool memory.
            let result = if let Some(buf) = self.port.pool_slice_mut(offset, len as u32) {
                f(buf)
            } else {
                // Alloc succeeded but slice failed — shouldn't happen.
                let mut empty = [0u8; 0];
                f(&mut empty)
            };

            let sqe = IoSqe {
                opcode: io_op::NET_SEND,
                flags: 0,
                priority: 0,
                tag: 0,
                data_offset: offset,
                data_len: len as u32,
                lba: 0,
                param: 0,
            };
            self.port.submit(&sqe);
            self.port.notify();

            result
        } else {
            // No pool space — drop the frame.
            let mut empty = [0u8; 0];
            f(&mut empty)
        }
    }
}
