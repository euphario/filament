//! NVMe Driver
//!
//! Provides block device access to NVMe SSDs connected via PCIe.
//! Uses the bus framework for lifecycle, devd communication, and DataPort
//! management.
//!
//! ## Spawning
//!
//! nvmed is rule-spawned by devd when pcied registers a Storage port.
//! BAR0 address and size are delivered via spawn context metadata
//! (set by pcied during port registration). nvmed maps BAR0, then
//! initializes the NVMe controller hardware.

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::mmio::{MmioRegion, DmaPool};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockTransport, BlockPortConfig, BlockGeometry, bus_msg,
    PortInfo, PortClass, port_subclass,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::{io_op, io_status, side_msg};
use userlib::{uinfo, uerror};

// =============================================================================
// NVMe Register Definitions (NVMe 1.4 spec)
// =============================================================================

mod regs {
    pub const CAP: u32 = 0x00;
    pub const VS: u32 = 0x08;
    pub const CC: u32 = 0x14;
    pub const CSTS: u32 = 0x1C;
    pub const AQA: u32 = 0x24;
    pub const ASQ: u32 = 0x28;
    pub const ACQ: u32 = 0x30;
    pub const DOORBELL_BASE: u32 = 0x1000;
}

mod cap {
    pub const MQES_MASK: u64 = 0xFFFF;
    pub const DSTRD_SHIFT: u64 = 32;
    pub const DSTRD_MASK: u64 = 0xF << DSTRD_SHIFT;
}

mod cc {
    pub const EN: u32 = 1 << 0;
    pub const CSS_NVM: u32 = 0 << 4;
    pub const AMS_RR: u32 = 0 << 11;
    pub const IOSQES_SHIFT: u32 = 16;
    pub const IOCQES_SHIFT: u32 = 20;
}

mod csts {
    pub const RDY: u32 = 1 << 0;
    pub const CFS: u32 = 1 << 1;
}

mod admin_opcode {
    pub const CREATE_SQ: u8 = 0x01;
    pub const CREATE_CQ: u8 = 0x05;
    pub const IDENTIFY: u8 = 0x06;
}

mod nvm_opcode {
    pub const WRITE: u8 = 0x01;
    pub const READ: u8 = 0x02;
}

mod identify_cns {
    pub const NAMESPACE: u8 = 0x00;
    pub const CONTROLLER: u8 = 0x01;
}

// =============================================================================
// Queue Structures
// =============================================================================

#[repr(C, align(64))]
#[derive(Clone, Copy, Default)]
struct SqEntry {
    cdw0: u32,
    nsid: u32,
    cdw2: u32,
    cdw3: u32,
    mptr: u64,
    prp1: u64,
    prp2: u64,
    cdw10: u32,
    cdw11: u32,
    cdw12: u32,
    cdw13: u32,
    cdw14: u32,
    cdw15: u32,
}

impl SqEntry {
    fn new() -> Self { Self::default() }
    fn set_opcode(&mut self, opcode: u8) {
        self.cdw0 = (self.cdw0 & !0xFF) | (opcode as u32);
    }
    fn set_cid(&mut self, cid: u16) {
        self.cdw0 = (self.cdw0 & 0xFFFF) | ((cid as u32) << 16);
    }
}

#[repr(C, align(16))]
#[derive(Clone, Copy, Default)]
struct CqEntry {
    dw0: u32,
    dw1: u32,
    dw2: u32,
    dw3: u32,
}

impl CqEntry {
    fn phase(&self) -> bool { (self.dw3 & (1 << 16)) != 0 }
    fn status(&self) -> u16 { ((self.dw3 >> 17) & 0x7FFF) as u16 }
}

#[repr(C)]
struct IdentifyController {
    vid: u16,
    ssvid: u16,
    sn: [u8; 20],
    mn: [u8; 40],
    fr: [u8; 8],
    rab: u8,
    ieee: [u8; 3],
    cmic: u8,
    mdts: u8,
    cntlid: u16,
    ver: u32,
}

#[repr(C)]
struct IdentifyNamespace {
    nsze: u64,
    ncap: u64,
    nuse: u64,
    nsfeat: u8,
    nlbaf: u8,
    flbas: u8,
    _pad: [u8; 128 - 27],
    lbaf0: u32,
}

const ADMIN_QUEUE_SIZE: usize = 32;
const IO_QUEUE_SIZE: usize = 64;
const QUEUE_MEM_SIZE: usize = 4096 * 4;

// =============================================================================
// NVMe Controller (hardware access only — no IPC)
// =============================================================================

struct NvmeController {
    bar0: u64,
    doorbell_stride: u32,
    queue_mem_phys: u64,
    queue_mem_virt: u64,
    asq_tail: u16,
    acq_head: u16,
    acq_phase: bool,
    iosq_tail: u16,
    iocq_head: u16,
    iocq_phase: bool,
    cid: u16,
    ns_size: u64,
    block_size: u32,
}

impl NvmeController {
    fn read32(&self, offset: u32) -> u32 {
        unsafe { core::ptr::read_volatile((self.bar0 + offset as u64) as *const u32) }
    }
    fn write32(&self, offset: u32, value: u32) {
        unsafe { core::ptr::write_volatile((self.bar0 + offset as u64) as *mut u32, value); }
    }
    fn read64(&self, offset: u32) -> u64 {
        unsafe { core::ptr::read_volatile((self.bar0 + offset as u64) as *const u64) }
    }
    fn write64(&self, offset: u32, value: u64) {
        unsafe { core::ptr::write_volatile((self.bar0 + offset as u64) as *mut u64, value); }
    }

    fn ring_asq_doorbell(&self) {
        self.write32(regs::DOORBELL_BASE, self.asq_tail as u32);
    }
    fn ring_acq_doorbell(&self) {
        self.write32(regs::DOORBELL_BASE + self.doorbell_stride, self.acq_head as u32);
    }
    fn ring_iosq_doorbell(&self) {
        self.write32(regs::DOORBELL_BASE + 2 * self.doorbell_stride, self.iosq_tail as u32);
    }
    fn ring_iocq_doorbell(&self) {
        self.write32(regs::DOORBELL_BASE + 3 * self.doorbell_stride, self.iocq_head as u32);
    }

    fn asq_entry(&self, index: u16) -> &mut SqEntry {
        unsafe { &mut *(self.queue_mem_virt as *mut SqEntry).add(index as usize) }
    }
    fn acq_entry(&self, index: u16) -> &CqEntry {
        unsafe { &*((self.queue_mem_virt + 4096) as *const CqEntry).add(index as usize) }
    }
    fn iosq_entry(&self, index: u16) -> &mut SqEntry {
        unsafe { &mut *((self.queue_mem_virt + 8192) as *mut SqEntry).add(index as usize) }
    }
    fn iocq_entry(&self, index: u16) -> &CqEntry {
        unsafe { &*((self.queue_mem_virt + 12288) as *const CqEntry).add(index as usize) }
    }

    fn admin_cmd(&mut self, entry: &SqEntry) -> Result<CqEntry, u16> {
        let sq_entry = self.asq_entry(self.asq_tail);
        *sq_entry = *entry;
        sq_entry.set_cid(self.cid);
        self.cid = self.cid.wrapping_add(1);
        self.asq_tail = (self.asq_tail + 1) % ADMIN_QUEUE_SIZE as u16;
        dsb();
        self.ring_asq_doorbell();

        for i in 0..1000u32 {
            let cq_entry = self.acq_entry(self.acq_head);
            if cq_entry.phase() == self.acq_phase {
                let status = cq_entry.status();
                let result = *cq_entry;
                self.acq_head = (self.acq_head + 1) % ADMIN_QUEUE_SIZE as u16;
                if self.acq_head == 0 { self.acq_phase = !self.acq_phase; }
                self.ring_acq_doorbell();
                if status != 0 { return Err(status); }
                return Ok(result);
            }
            // Check for CFS early to avoid wasting 100ms polling a dead controller
            if i % 100 == 99 {
                if (self.read32(regs::CSTS) & csts::CFS) != 0 { break; }
            }
            syscall::sleep_us(100);
        }
        let csts_val = self.read32(regs::CSTS);
        let cq0 = self.acq_entry(self.acq_head);
        uerror!("nvmed", "admin_timeout";
            csts = csts_val as u64,
            cq_dw2 = cq0.dw2 as u64,
            cq_dw3 = cq0.dw3 as u64,
            asq_phys = self.queue_mem_phys,
            acq_phys = (self.queue_mem_phys + 4096));
        Err(0xFFFF)
    }

    fn io_cmd(&mut self, entry: &SqEntry) -> Result<CqEntry, u16> {
        let sq_entry = self.iosq_entry(self.iosq_tail);
        *sq_entry = *entry;
        sq_entry.set_cid(self.cid);
        self.cid = self.cid.wrapping_add(1);
        self.iosq_tail = (self.iosq_tail + 1) % IO_QUEUE_SIZE as u16;
        dsb();
        self.ring_iosq_doorbell();

        for _ in 0..10000 {
            let cq_entry = self.iocq_entry(self.iocq_head);
            if cq_entry.phase() == self.iocq_phase {
                let status = cq_entry.status();
                let result = *cq_entry;
                self.iocq_head = (self.iocq_head + 1) % IO_QUEUE_SIZE as u16;
                if self.iocq_head == 0 { self.iocq_phase = !self.iocq_phase; }
                self.ring_iocq_doorbell();
                if status != 0 { return Err(status); }
                return Ok(result);
            }
            syscall::sleep_us(100);
        }
        Err(0xFFFF)
    }

    fn identify_controller(&mut self, data_phys: u64) -> Result<(), u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(admin_opcode::IDENTIFY);
        cmd.prp1 = data_phys;
        cmd.cdw10 = identify_cns::CONTROLLER as u32;
        self.admin_cmd(&cmd)?;
        Ok(())
    }

    fn identify_namespace(&mut self, nsid: u32, data_phys: u64) -> Result<(), u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(admin_opcode::IDENTIFY);
        cmd.nsid = nsid;
        cmd.prp1 = data_phys;
        cmd.cdw10 = identify_cns::NAMESPACE as u32;
        self.admin_cmd(&cmd)?;
        Ok(())
    }

    fn create_iocq(&mut self, qid: u16, size: u16, prp: u64) -> Result<(), u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(admin_opcode::CREATE_CQ);
        cmd.prp1 = prp;
        cmd.cdw10 = (qid as u32) | (((size - 1) as u32) << 16);
        cmd.cdw11 = 1;
        self.admin_cmd(&cmd)?;
        Ok(())
    }

    fn create_iosq(&mut self, qid: u16, size: u16, prp: u64, cqid: u16) -> Result<(), u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(admin_opcode::CREATE_SQ);
        cmd.prp1 = prp;
        cmd.cdw10 = (qid as u32) | (((size - 1) as u32) << 16);
        cmd.cdw11 = 1 | ((cqid as u32) << 16);
        self.admin_cmd(&cmd)?;
        Ok(())
    }

    fn read_blocks(&mut self, lba: u64, count: u16, data_phys: u64) -> Result<u32, u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(nvm_opcode::READ);
        cmd.nsid = 1;
        cmd.prp1 = data_phys;
        cmd.cdw10 = lba as u32;
        cmd.cdw11 = (lba >> 32) as u32;
        cmd.cdw12 = (count - 1) as u32;
        self.io_cmd(&cmd)?;
        Ok(count as u32 * self.block_size)
    }

    fn write_blocks(&mut self, lba: u64, count: u16, data_phys: u64) -> Result<u32, u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(nvm_opcode::WRITE);
        cmd.nsid = 1;
        cmd.prp1 = data_phys;
        cmd.cdw10 = lba as u32;
        cmd.cdw11 = (lba >> 32) as u32;
        cmd.cdw12 = (count - 1) as u32;
        self.io_cmd(&cmd)?;
        Ok(count as u32 * self.block_size)
    }

    fn format_info(&self) -> [u8; 512] {
        let mut buf = [0u8; 512];
        let mut pos = 0;

        let mut write_str = |s: &str| {
            let bytes = s.as_bytes();
            let len = bytes.len().min(buf.len() - pos);
            buf[pos..pos + len].copy_from_slice(&bytes[..len]);
            pos += len;
        };

        write_str("NVMe Storage Controller (bus framework)\n");
        write_str("  Block size: ");
        write_str(itoa(self.block_size as u64, &mut [0u8; 20]));
        write_str(" bytes\n");
        write_str("  Namespace size: ");
        write_str(itoa(self.ns_size, &mut [0u8; 20]));
        write_str(" blocks\n");

        let capacity_mb = (self.ns_size * self.block_size as u64) / (1024 * 1024);
        write_str("  Capacity: ");
        write_str(itoa(capacity_mb, &mut [0u8; 20]));
        write_str(" MB\n");

        write_str("\nQueues:\n");
        write_str("  Admin SQ tail: ");
        write_str(itoa(self.asq_tail as u64, &mut [0u8; 20]));
        write_str(", CQ head: ");
        write_str(itoa(self.acq_head as u64, &mut [0u8; 20]));
        write_str("\n");
        write_str("  I/O SQ tail: ");
        write_str(itoa(self.iosq_tail as u64, &mut [0u8; 20]));
        write_str(", CQ head: ");
        write_str(itoa(self.iocq_head as u64, &mut [0u8; 20]));
        write_str("\n");

        write_str("\nProtocol: DataPort (zero-copy via bus framework)\n");

        buf
    }
}

// =============================================================================
// Helpers
// =============================================================================

fn itoa(mut n: u64, buf: &mut [u8]) -> &str {
    if n == 0 {
        buf[0] = b'0';
        return core::str::from_utf8(&buf[..1]).unwrap();
    }
    let mut i = buf.len();
    while n > 0 && i > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    core::str::from_utf8(&buf[i..]).unwrap()
}

#[inline]
fn dsb() {
    unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }
}


// =============================================================================
// NVMe Controller Initialization (one-time, during init)
// =============================================================================

/// Initialize NVMe hardware. Returns the controller or an error string.
///
/// All errors are returned (not exit()) so the bus framework can flush
/// the structured log ring before terminating — otherwise uerror! messages
/// are silently lost.
fn init_nvme_hardware(bar0_addr: u64, bar0_size: u64) -> Result<NvmeController, BusError> {
    // Map BAR0
    let bar0 = match MmioRegion::open(bar0_addr, bar0_size) {
        Some(m) => m,
        None => {
            uerror!("nvmed", "bar0_map_failed"; addr = bar0_addr, size = bar0_size);
            return Err(BusError::Internal);
        }
    };

    // Allocate queue memory
    let queue_pool = match DmaPool::alloc(QUEUE_MEM_SIZE) {
        Some(p) => p,
        None => {
            uerror!("nvmed", "dma_alloc_failed";);
            return Err(BusError::Internal);
        }
    };

    let queue_mem_virt = queue_pool.vaddr();
    let queue_mem_phys = queue_pool.paddr();

    unsafe { core::ptr::write_bytes(queue_mem_virt as *mut u8, 0, QUEUE_MEM_SIZE); }

    let mut ctrl = NvmeController {
        bar0: bar0.virt_base(),
        doorbell_stride: 4,
        queue_mem_phys,
        queue_mem_virt,
        asq_tail: 0,
        acq_head: 0,
        acq_phase: true,
        iosq_tail: 0,
        iocq_head: 0,
        iocq_phase: true,
        cid: 0,
        ns_size: 0,
        block_size: 512,
    };

    // Read capabilities
    let cap_val = ctrl.read64(regs::CAP);
    let dstrd = ((cap_val & cap::DSTRD_MASK) >> cap::DSTRD_SHIFT) as u32;
    ctrl.doorbell_stride = 4 << dstrd;
    let vs = ctrl.read32(regs::VS);
    uinfo!("nvmed", "hw_probe"; cap = cap_val, vs = vs as u64, stride = ctrl.doorbell_stride);

    // Disable controller
    ctrl.write32(regs::CC, 0);
    for _ in 0..1000 {
        if (ctrl.read32(regs::CSTS) & csts::RDY) == 0 { break; }
        syscall::sleep_ms(1);
    }

    // Configure admin queues
    let aqa = ((ADMIN_QUEUE_SIZE as u32 - 1) << 16) | (ADMIN_QUEUE_SIZE as u32 - 1);
    ctrl.write32(regs::AQA, aqa);
    ctrl.write64(regs::ASQ, queue_mem_phys);
    ctrl.write64(regs::ACQ, queue_mem_phys + 4096);
    uinfo!("nvmed", "admin_queues";
        asq = queue_mem_phys, acq = queue_mem_phys + 4096,
        virt = queue_mem_virt, aqa = aqa as u64);

    // Verify registers were written (readback)
    let asq_rb = ctrl.read64(regs::ASQ);
    let acq_rb = ctrl.read64(regs::ACQ);
    let aqa_rb = ctrl.read32(regs::AQA);
    uinfo!("nvmed", "reg_readback"; asq_rb = asq_rb, acq_rb = acq_rb, aqa = aqa_rb as u64);

    // Enable controller
    let cc_val = cc::EN | cc::CSS_NVM | cc::AMS_RR
        | (6 << cc::IOSQES_SHIFT) | (4 << cc::IOCQES_SHIFT);
    ctrl.write32(regs::CC, cc_val);
    let mut rdy = false;
    for _ in 0..1000 {
        let csts_val = ctrl.read32(regs::CSTS);
        if (csts_val & csts::RDY) != 0 { rdy = true; break; }
        if (csts_val & csts::CFS) != 0 {
            uerror!("nvmed", "controller_fatal"; csts = csts_val);
            return Err(BusError::Internal);
        }
        syscall::sleep_ms(1);
    }
    if !rdy {
        uerror!("nvmed", "controller_timeout";);
        return Err(BusError::Timeout);
    }
    uinfo!("nvmed", "controller_ready"; cc = cc_val as u64, csts = ctrl.read32(regs::CSTS) as u64);

    // Identify buffer
    let id_pool = match DmaPool::alloc(4096) {
        Some(p) => p,
        None => {
            uerror!("nvmed", "identify_alloc_failed";);
            return Err(BusError::Internal);
        }
    };
    let id_buf_virt = id_pool.vaddr();
    let id_buf_phys = id_pool.paddr();

    // Identify controller
    match ctrl.identify_controller(id_buf_phys) {
        Ok(_) => {
            let id_ctrl = unsafe { &*(id_buf_virt as *const IdentifyController) };
            uinfo!("nvmed", "controller"; vendor = id_ctrl.vid as u32);
        }
        Err(e) => {
            uerror!("nvmed", "identify_ctrl_err"; status = e as u32);
            return Err(BusError::Internal);
        }
    }

    // Identify namespace 1
    match ctrl.identify_namespace(1, id_buf_phys) {
        Ok(_) => {
            let id_ns = unsafe { &*(id_buf_virt as *const IdentifyNamespace) };
            ctrl.ns_size = id_ns.nsze;
            let lbaf = id_ns.lbaf0;
            let lba_ds = (lbaf >> 16) & 0xFF;
            ctrl.block_size = 1 << lba_ds;
        }
        Err(e) => {
            uerror!("nvmed", "identify_ns_err"; status = e as u32);
            return Err(BusError::Internal);
        }
    }

    // Create I/O queues
    if let Err(e) = ctrl.create_iocq(1, IO_QUEUE_SIZE as u16, queue_mem_phys + 12288) {
        uerror!("nvmed", "create_iocq_err"; status = e as u32);
        return Err(BusError::Internal);
    }
    if let Err(e) = ctrl.create_iosq(1, IO_QUEUE_SIZE as u16, queue_mem_phys + 8192, 1) {
        uerror!("nvmed", "create_iosq_err"; status = e as u32);
        return Err(BusError::Internal);
    }

    uinfo!("nvmed", "controller_ready"; blocks = ctrl.ns_size, block_size = ctrl.block_size);
    Ok(ctrl)
}

// =============================================================================
// NVMe Driver (Bus Framework)
// =============================================================================

struct NvmeDriver {
    ctrl: Option<NvmeController>,
    port_id: Option<PortId>,
}

impl NvmeDriver {
    const fn new() -> Self {
        Self {
            ctrl: None,
            port_id: None,
        }
    }

    fn process_ring_requests(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };
        let ctrl = match &mut self.ctrl {
            Some(c) => c,
            None => return,
        };

        // Collect pending SQ requests
        let mut requests: [Option<userlib::ring::IoSqe>; 8] = [None; 8];
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

        // Process each request
        for i in 0..req_count {
            if let Some(sqe) = requests[i].take() {
                match sqe.opcode {
                    io_op::READ => {
                        let count = (sqe.data_len / ctrl.block_size) as u16;
                        if count == 0 {
                            if let Some(port) = ctx.block_port(port_id) {
                                port.complete_error(sqe.tag, io_status::INVALID);
                                port.notify();
                            }
                            continue;
                        }

                        // Calculate physical address in pool
                        let target_phys = if let Some(port) = ctx.block_port(port_id) {
                            port.pool_phys() + sqe.data_offset as u64
                        } else {
                            continue;
                        };

                        // Read directly to pool memory (zero-copy DMA)
                        match ctrl.read_blocks(sqe.lba, count, target_phys) {
                            Ok(transferred) => {
                                if let Some(port) = ctx.block_port(port_id) {
                                    port.complete_ok(sqe.tag, transferred);
                                    port.notify();
                                }
                            }
                            Err(_) => {
                                if let Some(port) = ctx.block_port(port_id) {
                                    port.complete_error(sqe.tag, io_status::IO_ERROR);
                                    port.notify();
                                }
                            }
                        }
                    }
                    io_op::WRITE => {
                        let count = (sqe.data_len / ctrl.block_size) as u16;
                        if count == 0 {
                            if let Some(port) = ctx.block_port(port_id) {
                                port.complete_error(sqe.tag, io_status::INVALID);
                                port.notify();
                            }
                            continue;
                        }

                        let target_phys = if let Some(port) = ctx.block_port(port_id) {
                            port.pool_phys() + sqe.data_offset as u64
                        } else {
                            continue;
                        };

                        match ctrl.write_blocks(sqe.lba, count, target_phys) {
                            Ok(transferred) => {
                                if let Some(port) = ctx.block_port(port_id) {
                                    port.complete_ok(sqe.tag, transferred);
                                    port.notify();
                                }
                            }
                            Err(_) => {
                                if let Some(port) = ctx.block_port(port_id) {
                                    port.complete_error(sqe.tag, io_status::IO_ERROR);
                                    port.notify();
                                }
                            }
                        }
                    }
                    _ => {
                        if let Some(port) = ctx.block_port(port_id) {
                            port.complete_error(sqe.tag, io_status::INVALID);
                            port.notify();
                        }
                    }
                }
            }
        }

        // Process sidechannel queries (geometry, etc.)
        let mut queries: [Option<userlib::ring::SideEntry>; 4] = [None; 4];
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
                    side_msg::QUERY_GEOMETRY => {
                        let info = BlockGeometry {
                            block_size: ctrl.block_size,
                            block_count: ctrl.ns_size,
                            max_transfer: 64 * 1024,
                        };
                        if let Some(port) = ctx.block_port(port_id) {
                            port.respond_geometry(&entry, &info);
                            port.notify();
                        }
                    }
                    _ => {
                        if let Some(port) = ctx.block_port(port_id) {
                            port.notify();
                        }
                    }
                }
            }
        }
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for NvmeDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("nvmed", "starting";);

        // Get spawn context — the port name and BAR0 metadata from pcied
        // (e.g., "pci/00:02.0:nvme" registered by pcied with BAR0 info)
        let spawn_ctx = ctx.spawn_context().map_err(|e| {
            uerror!("nvmed", "no_spawn_context";);
            e
        })?;

        // Read BAR0 info from spawn context metadata
        // pcied embeds [bar0_phys: u64 LE, bar0_size: u32 LE] = 12 bytes
        let meta = spawn_ctx.metadata();
        if meta.len() < 12 {
            uerror!("nvmed", "metadata_too_short"; len = meta.len() as u32);
            return Err(BusError::Internal);
        }

        let bar0_addr = u64::from_le_bytes([
            meta[0], meta[1], meta[2], meta[3],
            meta[4], meta[5], meta[6], meta[7],
        ]);
        let bar0_size = u32::from_le_bytes([
            meta[8], meta[9], meta[10], meta[11],
        ]) as u64;

        uinfo!("nvmed", "device_found"; bar0 = bar0_addr, size = bar0_size);

        // Initialize NVMe hardware
        let ctrl = init_nvme_hardware(bar0_addr, bar0_size)?;

        let block_size = ctrl.block_size;
        let block_count = ctrl.ns_size;
        self.ctrl = Some(ctrl);

        // Create block DataPort via framework
        let config = BlockPortConfig {
            ring_size: 64,
            side_size: 8,
            pool_size: 256 * 1024,
        };

        let port_id = ctx.create_block_port(config)?;
        if let Some(port) = ctx.block_port(port_id) {
            port.set_public();
        }
        self.port_id = Some(port_id);

        // Register block port with devd using unified PortInfo
        // Non-zero shmem_id triggers block orchestration (devd spawns partd)
        let shmem_id = ctx.block_port(port_id).map(|p| p.shmem_id()).unwrap_or(0);
        let mut info = PortInfo::new(b"nvme0:", PortClass::Block);
        info.port_subclass = port_subclass::BLOCK_RAW;
        let _ = ctx.register_port_with_info(&info, shmem_id);

        uinfo!("nvmed", "ready"; blocks = block_count, block_size = block_size);

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                if let Some(ref ctrl) = self.ctrl {
                    let info = ctrl.format_info();
                    let len = info.iter().rposition(|&b| b != 0).map(|p| p + 1).unwrap_or(0);
                    let _ = ctx.respond_info(msg.seq_id, &info[..len]);
                }
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        if self.port_id == Some(port) {
            self.process_ring_requests(ctx);
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: NvmeDriver = NvmeDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"nvmed", NvmeDriverWrapper(driver));
}

struct NvmeDriverWrapper(&'static mut NvmeDriver);

impl Driver for NvmeDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.data_ready(port, ctx)
    }
}
