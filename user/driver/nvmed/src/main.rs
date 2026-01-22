//! NVMe Driver for BPI-R4
//!
//! Provides block device access to NVMe SSDs connected via PCIe.
//! Uses shared ring buffer protocol compatible with BlockClient.

#![no_std]
#![no_main]

use userlib::{uinfo, uerror, syscall};
use userlib::syscall::{
    Handle, WaitFilter, WaitRequest, WaitResult,
    handle_timer_create, handle_timer_set, handle_wait,
    handle_wrap_channel,
};
use userlib::ulog;
use userlib::ring::{BlockRing, BlockRequest, BlockResponse};
use pcie::{PcieClient, PcieDeviceInfo};

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
    fn phase(&self) -> bool { (self.dw3 & 1) != 0 }
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
// NVMe Controller
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

        for _ in 0..1000 {
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
            syscall::yield_now();
        }
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
            syscall::yield_now();
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
        cmd.cdw10 = ((size - 1) as u32) | ((qid as u32) << 16);
        cmd.cdw11 = 1;
        self.admin_cmd(&cmd)?;
        Ok(())
    }

    fn create_iosq(&mut self, qid: u16, size: u16, prp: u64, cqid: u16) -> Result<(), u16> {
        let mut cmd = SqEntry::new();
        cmd.set_opcode(admin_opcode::CREATE_SQ);
        cmd.prp1 = prp;
        cmd.cdw10 = ((size - 1) as u32) | ((qid as u32) << 16);
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
}

#[inline]
fn dsb() {
    unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }
}

fn flush_buffer(addr: u64, size: usize) {
    let start = addr & !63;
    let end = (addr + size as u64 + 63) & !63;
    let mut a = start;
    while a < end {
        unsafe { core::arch::asm!("dc cvac, {}", in(reg) a, options(nostack, preserves_flags)); }
        a += 64;
    }
    dsb();
}

fn invalidate_buffer(addr: u64, size: usize) {
    let start = addr & !63;
    let end = (addr + size as u64 + 63) & !63;
    let mut a = start;
    while a < end {
        unsafe { core::arch::asm!("dc ivac, {}", in(reg) a, options(nostack, preserves_flags)); }
        a += 64;
    }
    dsb();
}

// =============================================================================
// Block Server (ring buffer protocol)
// =============================================================================

struct BlockServer {
    ring: BlockRing,
    client_pid: u32,
}

impl BlockServer {
    fn try_handshake(channel: u32) -> Option<Self> {
        let mut buf = [0u8; 64];

        // Receive client's PID
        let recv_len = syscall::receive(channel, &mut buf);
        if recv_len < 4 {
            return None;
        }

        let client_pid = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);

        // Create shared ring buffer (64 entries, 1MB data buffer)
        let ring = BlockRing::create(64, 1024 * 1024)?;

        // Grant client access
        if !ring.allow(client_pid) {
            uerror!("nvmed", "client_ring_failed"; op = "allow", err = "grant_denied", next = "drop");
            return None;
        }

        // Send shmem_id to client
        let shmem_id_bytes = ring.shmem_id().to_le_bytes();
        buf[..4].copy_from_slice(&shmem_id_bytes);
        syscall::send(channel, &buf[..4]);

        Some(Self { ring, client_pid })
    }

    fn process_requests(&mut self, ctrl: &mut NvmeController) -> u32 {
        let mut processed = 0u32;

        while let Some(req) = self.ring.next_request() {
            let resp = self.handle_request(ctrl, &req);
            self.ring.complete(&resp);
            processed += 1;
        }

        if processed > 0 {
            self.ring.notify();
        }

        processed
    }

    fn handle_request(&mut self, ctrl: &mut NvmeController, req: &BlockRequest) -> BlockResponse {
        match req.cmd {
            BlockRequest::CMD_INFO => {
                BlockResponse::info(req.tag, ctrl.block_size, ctrl.ns_size)
            }
            BlockRequest::CMD_READ => {
                self.handle_read(ctrl, req)
            }
            _ => {
                BlockResponse::error(req.tag, -38) // ENOSYS
            }
        }
    }

    fn handle_read(&mut self, ctrl: &mut NvmeController, req: &BlockRequest) -> BlockResponse {
        if req.count == 0 {
            return BlockResponse::error(req.tag, -22);
        }

        let bytes = (req.count as u64) * (ctrl.block_size as u64);
        let buf_offset = req.buf_offset as usize;

        if buf_offset + bytes as usize > self.ring.data_size() {
            return BlockResponse::error(req.tag, -22);
        }

        // Read directly into shared ring buffer
        let target_phys = self.ring.data_phys() + buf_offset as u64;

        match ctrl.read_blocks(req.lba, req.count as u16, target_phys) {
            Ok(bytes_read) => BlockResponse::ok(req.tag, bytes_read),
            Err(_) => BlockResponse::error(req.tag, -5), // EIO
        }
    }
}

// =============================================================================
// Main
// =============================================================================

const MAX_CLIENTS: usize = 4;

#[unsafe(no_mangle)]
fn main() {
    uinfo!("nvmed", "init_start");

    // Step 1: Query pcied for NVMe device
    let client = match PcieClient::connect() {
        Some(c) => c,
        None => {
            uerror!("nvmed", "init_failed"; op = "pcie_connect", err = "no_pcied", next = "exit");
            syscall::exit(1);
        }
    };

    let devices = client.find_devices(0xFFFF, 0xFFFF);

    let mut nvme_dev: Option<PcieDeviceInfo> = None;
    for info in devices.iter() {
        let class = info.class_code >> 8;
        if class == 0x0108 {
            uinfo!("nvmed", "device_found"; vendor = info.vendor_id as u64, device = info.device_id as u64, bar0 = ulog::hex64(info.bar0_addr));
            nvme_dev = Some(*info);
            break;
        }
    }

    let info = match nvme_dev {
        Some(i) => i,
        None => {
            uerror!("nvmed", "init_failed"; op = "device_scan", err = "no_nvme", next = "exit");
            syscall::exit(1);
        }
    };

    // Step 2: Map BAR0 using mmap_device
    let bar0_virt = match syscall::mmap_device(info.bar0_addr, info.bar0_size as u64) {
        Ok(v) => v,
        Err(e) => {
            uerror!("nvmed", "init_failed"; op = "bar0_map", err = e, next = "exit");
            syscall::exit(1);
        }
    };

    // Step 3: Allocate queue memory
    let mut queue_mem_phys: u64 = 0;
    let queue_mem_virt = syscall::mmap_dma(QUEUE_MEM_SIZE, &mut queue_mem_phys);
    if queue_mem_virt < 0 {
        uerror!("nvmed", "init_failed"; op = "dma_alloc", err = queue_mem_virt as u64, next = "exit");
        syscall::exit(1);
    }
    let queue_mem_virt = queue_mem_virt as u64;

    unsafe { core::ptr::write_bytes(queue_mem_virt as *mut u8, 0, QUEUE_MEM_SIZE); }
    flush_buffer(queue_mem_virt, QUEUE_MEM_SIZE);

    let mut ctrl = NvmeController {
        bar0: bar0_virt,
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

    // Step 4: Read capabilities and initialize
    let cap = ctrl.read64(regs::CAP);
    let _mqes = (cap & cap::MQES_MASK) as u32 + 1;
    let dstrd = ((cap & cap::DSTRD_MASK) >> cap::DSTRD_SHIFT) as u32;
    ctrl.doorbell_stride = 4 << dstrd;
    let _vs = ctrl.read32(regs::VS);

    // Disable controller
    ctrl.write32(regs::CC, 0);
    for _ in 0..1000 {
        if (ctrl.read32(regs::CSTS) & csts::RDY) == 0 { break; }
        syscall::yield_now();
    }

    // Configure Admin queues
    let aqa = ((ADMIN_QUEUE_SIZE as u32 - 1) << 16) | (ADMIN_QUEUE_SIZE as u32 - 1);
    ctrl.write32(regs::AQA, aqa);
    ctrl.write64(regs::ASQ, queue_mem_phys);
    ctrl.write64(regs::ACQ, queue_mem_phys + 4096);

    // Enable controller
    let cc = cc::EN | cc::CSS_NVM | cc::AMS_RR | (6 << cc::IOSQES_SHIFT) | (4 << cc::IOCQES_SHIFT);
    ctrl.write32(regs::CC, cc);
    for _ in 0..1000 {
        let csts = ctrl.read32(regs::CSTS);
        if (csts & csts::RDY) != 0 { break; }
        if (csts & csts::CFS) != 0 {
            uerror!("nvmed", "init_failed"; op = "ctrl_enable", err = "cfs", next = "exit");
            syscall::exit(1);
        }
        syscall::yield_now();
    }

    // Allocate identify buffer
    let mut id_buf_phys: u64 = 0;
    let id_buf_virt = syscall::mmap_dma(4096, &mut id_buf_phys);
    if id_buf_virt < 0 {
        uerror!("nvmed", "init_failed"; op = "id_buf_alloc", err = id_buf_virt as u64, next = "exit");
        syscall::exit(1);
    }
    let id_buf_virt = id_buf_virt as u64;

    // Identify controller
    invalidate_buffer(id_buf_virt, 4096);
    match ctrl.identify_controller(id_buf_phys) {
        Ok(_) => {
            invalidate_buffer(id_buf_virt, 4096);
            let id_ctrl = unsafe { &*(id_buf_virt as *const IdentifyController) };
            uinfo!("nvmed", "controller_id"; vendor = id_ctrl.vid as u64);
        }
        Err(e) => {
            uerror!("nvmed", "init_failed"; op = "identify_ctrl", err = e as u64, next = "exit");
            syscall::exit(1);
        }
    }

    // Identify namespace 1
    invalidate_buffer(id_buf_virt, 4096);
    match ctrl.identify_namespace(1, id_buf_phys) {
        Ok(_) => {
            invalidate_buffer(id_buf_virt, 4096);
            let id_ns = unsafe { &*(id_buf_virt as *const IdentifyNamespace) };
            ctrl.ns_size = id_ns.nsze;
            let lbaf = id_ns.lbaf0;
            let lba_ds = (lbaf >> 16) & 0xFF;
            ctrl.block_size = 1 << lba_ds;
        }
        Err(e) => {
            uerror!("nvmed", "init_failed"; op = "identify_ns", err = e as u64, next = "exit");
            syscall::exit(1);
        }
    }

    // Create I/O queues
    match ctrl.create_iocq(1, IO_QUEUE_SIZE as u16, queue_mem_phys + 12288) {
        Ok(_) => {}
        Err(e) => {
            uerror!("nvmed", "init_failed"; op = "create_iocq", err = e as u64, next = "exit");
            syscall::exit(1);
        }
    }

    match ctrl.create_iosq(1, IO_QUEUE_SIZE as u16, queue_mem_phys + 8192, 1) {
        Ok(_) => {}
        Err(e) => {
            uerror!("nvmed", "init_failed"; op = "create_iosq", err = e as u64, next = "exit");
            syscall::exit(1);
        }
    }

    uinfo!("nvmed", "controller_ready"; namespaces = 1u64, blocks = ctrl.ns_size, block_size = ctrl.block_size as u64);

    // Register as block device
    let port = syscall::port_register(b"nvme");
    if port < 0 {
        uerror!("nvmed", "init_failed"; op = "port_register", err = port as u64, next = "exit");
        syscall::exit(1);
    }
    let port_id = port as u32;

    // Create timer handle for polling (Handle API)
    // 1ms polling interval for ring buffer requests
    const POLL_INTERVAL_NS: u64 = 1_000_000; // 1ms
    let timer_handle = match handle_timer_create() {
        Ok(h) => h,
        Err(_) => {
            uerror!("nvmed", "init_failed"; op = "timer_create", err = 0u64, next = "exit");
            syscall::exit(1);
        }
    };

    // Set timer as recurring
    if handle_timer_set(timer_handle, POLL_INTERVAL_NS, POLL_INTERVAL_NS).is_err() {
        uerror!("nvmed", "init_failed"; op = "timer_set", err = 0u64, next = "exit");
        syscall::exit(1);
    }

    // Wrap the listen channel as a handle (Handle API)
    let port_handle = match handle_wrap_channel(port_id) {
        Ok(h) => h,
        Err(_) => {
            uerror!("nvmed", "init_failed"; op = "port_wrap", err = 0u64, next = "exit");
            syscall::exit(1);
        }
    };

    uinfo!("nvmed", "ready");

    // Client management
    let mut clients: [Option<BlockServer>; MAX_CLIENTS] = [None, None, None, None];

    // Event-driven main loop (Handle API)
    let requests = [
        WaitRequest::new(timer_handle, WaitFilter::Timer),
        WaitRequest::new(port_handle, WaitFilter::Readable),
    ];
    let mut results = [WaitResult::empty(); 2];

    loop {
        // Wait for timer or client connection (blocking)
        let count = match handle_wait(&requests, &mut results, u64::MAX) {
            Ok(n) => n,
            Err(_) => {
                syscall::yield_now();
                continue;
            }
        };

        // Process each ready handle
        for result in &results[..count] {
            if result.handle.raw() == timer_handle.raw() {
                // Timer fired - process ring buffer requests from all clients
                for client in clients.iter_mut().flatten() {
                    client.process_requests(&mut ctrl);
                }
            } else if result.handle.raw() == port_handle.raw() {
                // New client connection ready
                let new_channel = syscall::port_accept(port_id);
                if new_channel >= 0 {
                    if let Some(server) = BlockServer::try_handshake(new_channel as u32) {
                        uinfo!("nvmed", "client_connected"; pid = server.client_pid as u64);
                        // Find empty slot
                        for slot in clients.iter_mut() {
                            if slot.is_none() {
                                *slot = Some(server);
                                break;
                            }
                        }
                    }
                    syscall::close(new_channel as u32);
                }
            }
        }
    }
}
