//! Virtio-Net PCI Driver
//!
//! Full virtio-net 1.0 driver using the bus framework with DataPort integration
//! for zero-copy packet I/O. Auto-spawned by devd when pcied registers a
//! Network port.
//!
//! Architecture:
//! - pcied discovers virtio-net as class 0x02, spawns netd with BAR0+BDF metadata
//! - netd walks virtio PCI capabilities, initializes device, sets up RX/TX queues
//! - RX: pre-filled descriptors, timer-polled used ring, posts to DataPort CQ
//! - TX: dequeues DataPort SQ, fills descriptors, notifies device
//! - DataPort (block port) exposes network I/O to upper-layer consumers

#![no_std]
#![no_main]

mod virtio;

use userlib::syscall::{self, Handle};
use userlib::mmio::{MmioRegion, DmaPool};
use userlib::ipc::{PciDevice, Timer};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, bus_msg,
    PortInfo, PortClass, port_subclass, NetworkMetadata,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::{IoSqe, IoCqe, io_op, io_status, side_msg, side_status, SideEntry};
use userlib::{uinfo, uerror, uwarn};

use virtio::{VirtioPciCaps, Virtqueue};

// =============================================================================
// Constants
// =============================================================================

// Virtio-net feature bits
const VIRTIO_NET_F_MAC: u64 = 1 << 5;
const VIRTIO_NET_F_STATUS: u64 = 1 << 16;

// Virtio-net header (without MRG_RXBUF)
const VIRTIO_NET_HDR_SIZE: usize = 10;

// Queue configuration
const RX_QUEUE_IDX: u16 = 0;
const TX_QUEUE_IDX: u16 = 1;
const QUEUE_SIZE: u16 = 64;

// Buffer sizes
const RX_BUF_SIZE: u32 = 2048; // 10 (hdr) + 1514 (max eth frame) + padding
const NUM_RX_BUFS: usize = 64;

// Total DMA: virtqueues + RX buffers + TX staging buffer
// Each virtqueue ~= queue_size * 16 + avail_ring + padding + used_ring ≈ 12KB
// RX buffers = 64 * 2048 = 128KB
// TX staging = 2048
const DMA_POOL_SIZE: usize = 2 * 16384 + NUM_RX_BUFS * RX_BUF_SIZE as usize + 4096;

// Network opcodes (using io_op::NET_BASE = 0x60)
mod net_op {
    pub const TX: u8 = super::io_op::NET_BASE;       // 0x60 - transmit packet
    pub const RX: u8 = super::io_op::NET_BASE + 1;   // 0x61 - received packet (CQ only)
}

// Sidechannel query: network info
const SIDE_QUERY_NET_INFO: u16 = side_msg::QUERY_INFO;

// RX poll timer: 10ms interval (until MSI-X interrupt support)
const RX_POLL_INTERVAL_NS: u64 = 10_000_000;
const TAG_RX_POLL: u32 = 1;

// =============================================================================
// Virtio-Net Header
// =============================================================================

#[repr(C)]
#[derive(Clone, Copy)]
struct VirtioNetHdr {
    flags: u8,
    gso_type: u8,
    hdr_len: u16,
    gso_size: u16,
    csum_start: u16,
    csum_offset: u16,
}

impl VirtioNetHdr {
    const fn zeroed() -> Self {
        Self {
            flags: 0,
            gso_type: 0,
            hdr_len: 0,
            gso_size: 0,
            csum_start: 0,
            csum_offset: 0,
        }
    }
}

// =============================================================================
// NetDriver State
// =============================================================================

struct NetDriver {
    bar: Option<MmioRegion>,
    caps: Option<VirtioPciCaps>,
    rx_queue: Option<Virtqueue>,
    tx_queue: Option<Virtqueue>,
    dma: Option<DmaPool>,
    // Offsets within DMA pool for RX buffers
    rx_bufs_offset: usize,
    // TX staging area offset within DMA pool
    tx_staging_offset: usize,
    mac: [u8; 6],
    port_id: Option<PortId>,
    rx_seq: u32,
    rx_poll_timer: Option<Timer>,
}

impl NetDriver {
    const fn new() -> Self {
        Self {
            bar: None,
            caps: None,
            rx_queue: None,
            tx_queue: None,
            dma: None,
            rx_bufs_offset: 0,
            tx_staging_offset: 0,
            mac: [0; 6],
            port_id: None,
            rx_seq: 0,
            rx_poll_timer: None,
        }
    }

    /// Parse spawn context metadata: [bar0_phys: u64, bar0_size: u32] = 12 bytes
    fn parse_metadata(meta: &[u8]) -> Option<(u64, u64)> {
        if meta.len() < 12 {
            return None;
        }
        let bar0_addr = u64::from_le_bytes([
            meta[0], meta[1], meta[2], meta[3],
            meta[4], meta[5], meta[6], meta[7],
        ]);
        let bar0_size = u32::from_le_bytes([
            meta[8], meta[9], meta[10], meta[11],
        ]) as u64;
        Some((bar0_addr, bar0_size))
    }

    /// Parse BDF from port name like "BB:DD.F:type"
    /// Returns (bus << 8) | (dev << 3) | func
    fn parse_bdf_from_name(name: &[u8]) -> Option<u32> {
        // Format: "BB:DD.F:type" — starts directly with hex BDF
        if name.len() < 7 {
            return None;
        }
        let hex = |b: u8| -> Option<u8> {
            match b {
                b'0'..=b'9' => Some(b - b'0'),
                b'a'..=b'f' => Some(b - b'a' + 10),
                b'A'..=b'F' => Some(b - b'A' + 10),
                _ => None,
            }
        };
        // "BB:DD.F:..."
        //  01234567...
        let bus = (hex(name[0])? as u32) << 4 | hex(name[1])? as u32;
        // name[2] = ':'
        let dev = (hex(name[3])? as u32) << 4 | hex(name[4])? as u32;
        // name[5] = '.'
        let func = hex(name[6])? as u32;
        Some((bus << 8) | (dev << 3) | func)
    }

    /// Pre-fill RX queue with buffers
    fn prefill_rx(&mut self) {
        let rx_queue = match self.rx_queue.as_mut() {
            Some(q) => q,
            None => return,
        };
        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return,
        };

        for i in 0..NUM_RX_BUFS {
            let buf_phys = dma.paddr() + (self.rx_bufs_offset + i * RX_BUF_SIZE as usize) as u64;
            rx_queue.add_buf(buf_phys, RX_BUF_SIZE, virtio::VIRTQ_DESC_F_WRITE);
        }

        // Kick the device to start receiving
        if let (Some(bar), Some(caps)) = (self.bar.as_ref(), self.caps.as_ref()) {
            rx_queue.kick(bar, caps.notify_off);
        }
    }

    /// Poll RX used ring for received packets
    fn poll_rx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };

        // Collect completed RX descriptors
        let mut completions: [(u16, u32); 16] = [(0, 0); 16];
        let mut count = 0;

        if let Some(ref mut rx_queue) = self.rx_queue {
            while count < 16 {
                match rx_queue.poll_used() {
                    Some(c) => {
                        completions[count] = c;
                        count += 1;
                    }
                    None => break,
                }
            }
        }

        if count == 0 {
            return;
        }

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return,
        };

        // Process each received packet
        for i in 0..count {
            let (desc_idx, bytes_written) = completions[i];

            if bytes_written <= VIRTIO_NET_HDR_SIZE as u32 {
                // No actual frame data, just reclaim
                if let Some(ref mut rx_queue) = self.rx_queue {
                    rx_queue.reclaim(desc_idx);
                }
                continue;
            }

            let frame_len = bytes_written - VIRTIO_NET_HDR_SIZE as u32;
            let buf_base = self.rx_bufs_offset + desc_idx as usize * RX_BUF_SIZE as usize;
            let frame_start = buf_base + VIRTIO_NET_HDR_SIZE;

            // Copy frame to DataPort pool for consumer
            if let Some(port) = ctx.block_port(port_id) {
                // Try to allocate space in the pool
                if let Some(pool_offset) = port.alloc(frame_len) {
                    // Copy frame data from DMA buffer to pool
                    let src = (dma.vaddr() + frame_start as u64) as *const u8;
                    let frame_data = unsafe {
                        core::slice::from_raw_parts(src, frame_len as usize)
                    };
                    port.pool_write(pool_offset, frame_data);

                    // Post CQE to notify consumer of received packet
                    let tag = self.rx_seq;
                    self.rx_seq = self.rx_seq.wrapping_add(1);
                    let cqe = IoCqe {
                        status: io_status::OK,
                        flags: 0,
                        tag,
                        transferred: frame_len,
                        result: pool_offset,  // Consumer reads frame from this offset
                    };
                    port.complete(&cqe);
                    port.notify();
                }
            }

            // Reclaim descriptor and re-add to RX ring
            if let Some(ref mut rx_queue) = self.rx_queue {
                rx_queue.reclaim(desc_idx);
                // Re-fill with same buffer
                let buf_phys = dma.paddr() + buf_base as u64;
                rx_queue.add_buf(buf_phys, RX_BUF_SIZE, virtio::VIRTQ_DESC_F_WRITE);
            }
        }

        // Kick device to process newly available RX buffers
        if let (Some(bar), Some(caps), Some(ref rx_queue)) =
            (self.bar.as_ref(), self.caps.as_ref(), &self.rx_queue)
        {
            rx_queue.kick(bar, caps.notify_off);
        }
    }

    /// Process TX requests from the DataPort SQ
    fn process_tx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };

        // Collect TX requests
        let mut requests: [Option<IoSqe>; 8] = [None; 8];
        let mut req_count = 0;

        if let Some(port) = ctx.block_port(port_id) {
            while req_count < 8 {
                if let Some(sqe) = port.recv_request() {
                    requests[req_count] = Some(sqe);
                    req_count += 1;
                } else {
                    break;
                }
            }
        }

        if req_count == 0 {
            return;
        }

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return,
        };

        for i in 0..req_count {
            let sqe = match requests[i].take() {
                Some(s) => s,
                None => continue,
            };

            if sqe.opcode != net_op::TX {
                // Unknown opcode
                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                }
                continue;
            }

            let packet_len = sqe.data_len;
            if packet_len == 0 || packet_len > 1514 {
                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                }
                continue;
            }

            // Read packet from pool into TX staging area (prepend virtio-net header)
            let staging_virt = dma.vaddr() + self.tx_staging_offset as u64;
            let staging_phys = dma.paddr() + self.tx_staging_offset as u64;

            // Write virtio-net header (all zeros — no offload)
            let hdr = VirtioNetHdr::zeroed();
            unsafe {
                core::ptr::write_volatile(staging_virt as *mut VirtioNetHdr, hdr);
            }

            // Copy packet data from pool to staging buffer after header
            if let Some(port) = ctx.block_port(port_id) {
                if let Some(src) = port.pool_slice(sqe.data_offset, packet_len) {
                    let dst = (staging_virt + VIRTIO_NET_HDR_SIZE as u64) as *mut u8;
                    unsafe {
                        core::ptr::copy_nonoverlapping(src.as_ptr(), dst, packet_len as usize);
                    }
                } else {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                    continue;
                }
            }

            let total_len = VIRTIO_NET_HDR_SIZE as u32 + packet_len;

            // Add to TX virtqueue
            let sent = if let Some(ref mut tx_queue) = self.tx_queue {
                tx_queue.add_buf(staging_phys, total_len, 0).is_some()
            } else {
                false
            };

            if sent {
                // Kick device
                if let (Some(bar), Some(caps), Some(ref tx_queue)) =
                    (self.bar.as_ref(), self.caps.as_ref(), &self.tx_queue)
                {
                    tx_queue.kick(bar, caps.notify_off);
                }

                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_ok(sqe.tag, packet_len);
                    port.notify();
                }
            } else {
                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_error(sqe.tag, io_status::NO_SPACE);
                    port.notify();
                }
            }
        }
    }

    /// Reclaim completed TX descriptors
    fn poll_tx_completions(&mut self) {
        if let Some(ref mut tx_queue) = self.tx_queue {
            while let Some((desc_idx, _)) = tx_queue.poll_used() {
                tx_queue.reclaim(desc_idx);
            }
        }
    }

    /// Handle sidechannel queries
    fn handle_side_queries(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };

        let mut queries: [Option<SideEntry>; 4] = [None; 4];
        let mut query_count = 0;

        if let Some(port) = ctx.block_port(port_id) {
            while query_count < 4 {
                if let Some(entry) = port.poll_side_request() {
                    queries[query_count] = Some(entry);
                    query_count += 1;
                } else {
                    break;
                }
            }
        }

        for i in 0..query_count {
            if let Some(entry) = queries[i].take() {
                match entry.msg_type {
                    SIDE_QUERY_NET_INFO => {
                        // Respond with MAC address and link status
                        let mut response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::OK,
                            payload: [0; 24],
                        };
                        // payload[0..6] = MAC address
                        response.payload[0..6].copy_from_slice(&self.mac);
                        // payload[6] = link status (1 = up)
                        response.payload[6] = 1;
                        // payload[7..9] = MTU (1500)
                        response.payload[7..9].copy_from_slice(&1500u16.to_le_bytes());

                        if let Some(port) = ctx.block_port(port_id) {
                            port.side_send(&response);
                            port.notify();
                        }
                    }
                    _ => {
                        // Unknown query — respond with EOL
                        let response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::EOL,
                            payload: [0; 24],
                        };
                        if let Some(port) = ctx.block_port(port_id) {
                            port.side_send(&response);
                            port.notify();
                        }
                    }
                }
            }
        }
    }

    fn format_mac(&self) -> [u8; 17] {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut buf = [0u8; 17];
        for i in 0..6 {
            buf[i * 3] = HEX[(self.mac[i] >> 4) as usize];
            buf[i * 3 + 1] = HEX[(self.mac[i] & 0xf) as usize];
            if i < 5 {
                buf[i * 3 + 2] = b':';
            }
        }
        buf
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for NetDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("netd", "init";);

        // 1. Parse spawn context metadata
        let spawn_ctx = ctx.spawn_context().map_err(|e| {
            uerror!("netd", "no_spawn_context";);
            e
        })?;

        let meta = spawn_ctx.metadata();
        let port_name = spawn_ctx.port_name();
        let (bar0_addr, bar0_size) = NetDriver::parse_metadata(meta).ok_or_else(|| {
            uerror!("netd", "bad_metadata"; len = meta.len() as u32);
            BusError::Internal
        })?;

        let bdf = NetDriver::parse_bdf_from_name(port_name).ok_or_else(|| {
            uerror!("netd", "bad_bdf";);
            BusError::Internal
        })?;

        uinfo!("netd", "pci_device";
            bar0 = userlib::ulog::hex64(bar0_addr),
            size = userlib::ulog::hex32(bar0_size as u32),
            bdf = userlib::ulog::hex32(bdf));

        // 2. Open PCI device for config space access
        let pci = PciDevice::open(bdf).map_err(|e| {
            uerror!("netd", "pci_open_failed";);
            BusError::Internal
        })?;

        // 3. Discover virtio PCI capabilities
        let caps = virtio::discover_caps(&pci).ok_or_else(|| {
            uerror!("netd", "no_virtio_caps";);
            BusError::Internal
        })?;

        uinfo!("netd", "virtio_caps";
            bar = caps.bar as u32,
            common = userlib::ulog::hex32(caps.common_off),
            notify = userlib::ulog::hex32(caps.notify_off),
            device = userlib::ulog::hex32(caps.device_off));

        // 4. Map the BAR that contains the virtio capabilities
        let (bar_addr, bar_size) = if caps.bar == 0 {
            (bar0_addr, bar0_size)
        } else {
            // Read BAR address from PCI config space
            let bar_offset = 0x10u16 + caps.bar as u16 * 4;
            let bar_lo = pci.config_read(bar_offset, 4).map_err(|_| {
                uerror!("netd", "bar_read_failed"; bar = caps.bar as u32);
                BusError::Internal
            })?;

            // Check if 64-bit BAR (type bits [2:1] == 0b10)
            let is_64bit = (bar_lo & 0x06) == 0x04;
            let bar_phys = if is_64bit {
                let bar_hi = pci.config_read(bar_offset + 4, 4).map_err(|_| {
                    uerror!("netd", "bar_hi_read_failed"; bar = caps.bar as u32);
                    BusError::Internal
                })?;
                ((bar_hi as u64) << 32) | (bar_lo as u64 & !0xF)
            } else {
                (bar_lo as u64) & !0xF
            };

            // Compute BAR size from capability offsets (avoids config_write probe)
            let bar_sz = caps.min_bar_size();

            uinfo!("netd", "bar_resolved";
                bar = caps.bar as u32,
                phys = userlib::ulog::hex64(bar_phys),
                size = userlib::ulog::hex64(bar_sz));
            (bar_phys, bar_sz)
        };

        let bar = MmioRegion::open(bar_addr, bar_size).ok_or_else(|| {
            uerror!("netd", "bar_map_failed";
                bar = caps.bar as u32,
                addr = userlib::ulog::hex64(bar_addr));
            BusError::Internal
        })?;

        // 5. Virtio initialization (reset → features → FEATURES_OK)
        let driver_features = VIRTIO_NET_F_MAC | VIRTIO_NET_F_STATUS;
        let negotiated = virtio::virtio_init(&bar, &caps, driver_features).map_err(|_| {
            uerror!("netd", "virtio_init_failed";);
            BusError::Internal
        })?;

        uinfo!("netd", "features_ok"; features = userlib::ulog::hex64(negotiated));

        // 6. Allocate DMA pool
        let mut dma = DmaPool::alloc(DMA_POOL_SIZE).ok_or_else(|| {
            uerror!("netd", "dma_alloc_failed";);
            BusError::Internal
        })?;
        dma.zero();

        // 7. Set up RX queue (index 0)
        let rx_max_size = virtio::read_queue_size(&bar, &caps, RX_QUEUE_IDX);
        let rx_size = rx_max_size.min(QUEUE_SIZE);
        virtio::write_queue_size(&bar, &caps, RX_QUEUE_IDX, rx_size);
        let rx_notify_off = virtio::read_queue_notify_off(&bar, &caps, RX_QUEUE_IDX) as u32
            * caps.notify_mul;

        let (rx_queue, rx_consumed) = Virtqueue::new(&dma, 0, rx_size, rx_notify_off);
        virtio::setup_queue(&bar, &caps, RX_QUEUE_IDX, &rx_queue);

        // 8. Set up TX queue (index 1)
        let tx_max_size = virtio::read_queue_size(&bar, &caps, TX_QUEUE_IDX);
        let tx_size = tx_max_size.min(QUEUE_SIZE);
        virtio::write_queue_size(&bar, &caps, TX_QUEUE_IDX, tx_size);
        let tx_notify_off = virtio::read_queue_notify_off(&bar, &caps, TX_QUEUE_IDX) as u32
            * caps.notify_mul;

        let tx_start = (rx_consumed + 4095) & !4095;
        let (tx_queue, tx_consumed) = Virtqueue::new(&dma, tx_start, tx_size, tx_notify_off);
        virtio::setup_queue(&bar, &caps, TX_QUEUE_IDX, &tx_queue);

        let rx_bufs_start = (tx_start + tx_consumed + 4095) & !4095;
        self.rx_bufs_offset = rx_bufs_start;

        let tx_staging_start = rx_bufs_start + NUM_RX_BUFS * RX_BUF_SIZE as usize;
        self.tx_staging_offset = tx_staging_start;

        uinfo!("netd", "queues_ready"; rx = rx_size as u32, tx = tx_size as u32);

        virtio::set_driver_ok(&bar, &caps);

        // 9. Read MAC from device config
        if negotiated & VIRTIO_NET_F_MAC != 0 {
            for i in 0..6 {
                self.mac[i] = bar.read8(caps.device_off as usize + i);
            }
        }

        // Store device state
        self.caps = Some(caps);
        self.rx_queue = Some(rx_queue);
        self.tx_queue = Some(tx_queue);
        self.dma = Some(dma);
        self.bar = Some(bar);

        // 10. Pre-fill RX queue
        self.prefill_rx();

        // 11. Create DataPort for network I/O
        let config = BlockPortConfig {
            ring_size: 64,
            side_size: 4,
            pool_size: 128 * 1024,
        };
        let port_id = ctx.create_block_port(config).map_err(|e| {
            uerror!("netd", "create_block_port_failed";);
            e
        })?;
        if let Some(port) = ctx.block_port(port_id) {
            port.set_public();
        }
        self.port_id = Some(port_id);

        // 12. Register with devd using unified PortInfo
        let shmem_id = ctx.block_port(port_id).map(|p| p.shmem_id()).unwrap_or(0);
        let mut info = PortInfo::new(b"net0", PortClass::Network);
        info.port_subclass = port_subclass::NET_ETHERNET;
        let mut meta = NetworkMetadata::empty();
        meta.mac.copy_from_slice(&self.mac);
        info.set_network_metadata(meta);
        let _ = ctx.register_port_with_info(&info, shmem_id);

        let mac_str = self.format_mac();
        uinfo!("netd", "ready"; mac = core::str::from_utf8(&mac_str).unwrap_or("?"));

        // 13. Start RX poll timer (until MSI-X interrupt support)
        if let Ok(mut timer) = Timer::new() {
            let _ = timer.set(RX_POLL_INTERVAL_NS);
            let handle = timer.handle();
            let _ = ctx.watch_handle(handle, TAG_RX_POLL);
            self.rx_poll_timer = Some(timer);
        }

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let mac_str = self.format_mac();
                let mut info = [0u8; 128];
                let mut pos = 0;
                let prefix = b"virtio-net mac=";
                info[..prefix.len()].copy_from_slice(prefix);
                pos += prefix.len();
                info[pos..pos + mac_str.len()].copy_from_slice(&mac_str);
                pos += mac_str.len();
                let _ = ctx.respond_info(msg.seq_id, &info[..pos]);
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        if self.port_id != Some(port) {
            return;
        }

        // 1. Process TX requests from consumer
        self.process_tx(ctx);

        // 2. Check for received packets (poll RX used ring)
        self.poll_rx(ctx);

        // 3. Reclaim completed TX descriptors
        self.poll_tx_completions();

        // 4. Handle sidechannel queries
        self.handle_side_queries(ctx);
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        if tag == TAG_RX_POLL {
            // Read timer to clear fired state
            if let Some(ref mut timer) = self.rx_poll_timer {
                let _ = timer.wait();
            }

            // Poll RX and reclaim TX
            self.poll_rx(ctx);
            self.poll_tx_completions();

            // Re-arm timer
            if let Some(ref mut timer) = self.rx_poll_timer {
                let _ = timer.set(RX_POLL_INTERVAL_NS);
            }
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: NetDriver = NetDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"netd", NetDriverWrapper(driver));
}

struct NetDriverWrapper(&'static mut NetDriver);

impl Driver for NetDriverWrapper {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.data_ready(port, ctx)
    }

    fn handle_event(&mut self, tag: u32, handle: Handle, ctx: &mut dyn BusCtx) {
        self.0.handle_event(tag, handle, ctx)
    }
}
