//! Composable block storage stack
//!
//! Each layer:
//! - Has clear in/out interface (trait)
//! - Hides its internals completely
//! - Doesn't know what's above or below
//! - Can be swapped without touching other layers
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  Application / Filesystem               │
//! │  sees: BlockDevice                      │
//! │  knows: nothing about storage           │
//! ├─────────────────────────────────────────┤
//! │  Block Layer                            │
//! │  in:  read_blocks/write_blocks          │
//! │  out: ScsiTransport                     │
//! │  hides: CDB construction, retries       │
//! ├─────────────────────────────────────────┤
//! │  SCSI Layer                             │
//! │  in:  execute(cdb, data)                │
//! │  out: ByteTransport                     │
//! │  hides: protocol framing                │
//! ├─────────────────────────────────────────┤
//! │  Protocol Layer (BOT/UAS/AHCI/NVMe)     │
//! │  in:  send/recv bytes                   │
//! │  out: RingTransport                     │
//! │  hides: descriptor format, queuing      │
//! ├─────────────────────────────────────────┤
//! │  Ring Layer                             │
//! │  in:  submit(opaque)                    │
//! │  out: HwOps                             │
//! │  hides: ring management, cycle bits     │
//! ├─────────────────────────────────────────┤
//! │  SoC Layer                              │
//! │  in:  write_reg/read_reg/dma            │
//! │  hides: MMIO addresses, DMA setup       │
//! └─────────────────────────────────────────┘
//! ```

// =============================================================================
// Layer 1: Block Device (what applications see)
// =============================================================================

/// Block device - the top of the stack
/// Filesystems, apps only see this. No idea what's underneath.
pub trait BlockDevice {
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), BlockError>;
    fn write_blocks(&mut self, lba: u64, buf: &[u8]) -> Result<(), BlockError>;
    fn block_size(&self) -> u32;
    fn block_count(&self) -> u64;
}

#[derive(Debug, Clone, Copy)]
pub enum BlockError {
    Io,
    OutOfRange,
    NotReady,
}

// =============================================================================
// Layer 2: SCSI Transport (block layer talks to this)
// =============================================================================

/// SCSI transport - execute commands, get status
/// Block layer uses this. Doesn't know if it's USB, SATA, or SAS.
pub trait ScsiTransport {
    fn execute(
        &mut self,
        cdb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Result<u8, TransportError>;
}

#[derive(Debug, Clone, Copy)]
pub enum TransportError {
    Timeout,
    Stall,
    Disconnected,
    Protocol,
}

// =============================================================================
// Layer 3: Byte Transport (SCSI/protocol layers talk to this)
// =============================================================================

/// Byte transport - send/receive raw bytes
/// Protocol layers (BOT, UAS, AHCI) use this.
pub trait ByteTransport {
    fn send(&mut self, data: &[u8]) -> Result<(), TransportError>;
    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;
}

// =============================================================================
// Layer 4: Ring Transport (byte transport talks to this)
// =============================================================================

/// Ring transport - submit descriptors, poll completions
/// This is where USB/SATA/NVMe converge - all use descriptor rings.
pub trait RingTransport {
    type Descriptor;
    type Completion;

    fn submit(&mut self, desc: Self::Descriptor) -> Result<(), TransportError>;
    fn poll(&mut self) -> Option<Self::Completion>;
    fn doorbell(&mut self);
}

// =============================================================================
// Layer 5: Hardware Ops (ring layer talks to this)
// =============================================================================

/// Hardware operations - the SoC-specific bottom
/// Only this layer touches MMIO, knows about physical addresses.
pub trait HwOps {
    fn write_reg(&mut self, offset: usize, value: u32);
    fn read_reg(&self, offset: usize) -> u32;
    fn dma_alloc(&mut self, size: usize) -> Option<DmaBuffer>;
    fn dma_free(&mut self, buf: DmaBuffer);
}

/// DMA buffer - opaque to upper layers
/// Only the SoC layer knows how to create/manage these
pub struct DmaBuffer {
    vaddr: *mut u8,
    paddr: u64,
    size: usize,
}

impl DmaBuffer {
    /// Create a new DMA buffer (only SoC layer should call this)
    pub(crate) fn new(vaddr: *mut u8, paddr: u64, size: usize) -> Self {
        Self { vaddr, paddr, size }
    }

    /// Get slice for CPU access (hides the raw pointer)
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.vaddr, self.size) }
    }

    /// Get mutable slice for CPU access
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.vaddr, self.size) }
    }

    /// Physical address (only transport layers need this)
    pub(crate) fn phys(&self) -> u64 {
        self.paddr
    }

    pub fn len(&self) -> usize {
        self.size
    }
}

// =============================================================================
// Implementation: SCSI Block Device (generic over transport)
// =============================================================================

/// Block device backed by SCSI transport
/// Works with USB-MSC, SATA, SAS - anything that speaks SCSI.
///
/// All internals are hidden. Users only see BlockDevice trait.
pub struct ScsiBlockDevice<T: ScsiTransport> {
    // All fields private - no leaking internals
    transport: T,
    block_size: u32,
    block_count: u64,
}

impl<T: ScsiTransport> ScsiBlockDevice<T> {
    /// Create from any SCSI transport - probes device automatically
    pub fn new(mut transport: T) -> Option<Self> {
        // First, issue TEST UNIT READY commands to wait for device to be ready
        // Some devices need time after configuration before accepting commands
        let tur_cdb = [0x00u8, 0, 0, 0, 0, 0]; // TEST UNIT READY

        for retry in 0..10 {
            match transport.execute(&tur_cdb, None, None) {
                Ok(0) => break,
                Ok(_status) => {
                    // Device not ready, try again with small delay
                    for _ in 0..100_000 { core::hint::spin_loop(); }
                }
                Err(_) => {
                    if retry == 9 { return None; }
                }
            }
        }

        // Now read capacity
        let mut cap_buf = [0u8; 8];
        let cdb = [0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        let status = match transport.execute(&cdb, Some(&mut cap_buf), None) {
            Ok(s) => s,
            Err(_) => return None,
        };

        if status != 0 {
            return None;
        }

        // Internal: parse response (big-endian)
        let last_lba = u32::from_be_bytes([cap_buf[0], cap_buf[1], cap_buf[2], cap_buf[3]]);
        let block_size = u32::from_be_bytes([cap_buf[4], cap_buf[5], cap_buf[6], cap_buf[7]]);

        Some(Self {
            transport,
            block_size,
            block_count: last_lba as u64 + 1,
        })
    }

    // Internal helper - builds READ CDB, hidden from users
    fn build_read_cdb(lba: u32, blocks: u16) -> [u8; 10] {
        [
            0x28, 0,
            (lba >> 24) as u8, (lba >> 16) as u8, (lba >> 8) as u8, lba as u8,
            0,
            (blocks >> 8) as u8, blocks as u8,
            0,
        ]
    }

    // Internal helper - builds WRITE CDB, hidden from users
    fn build_write_cdb(lba: u32, blocks: u16) -> [u8; 10] {
        [
            0x2A, 0,
            (lba >> 24) as u8, (lba >> 16) as u8, (lba >> 8) as u8, lba as u8,
            0,
            (blocks >> 8) as u8, blocks as u8,
            0,
        ]
    }
}

impl<T: ScsiTransport> BlockDevice for ScsiBlockDevice<T> {
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), BlockError> {
        // Validate - all internal logic hidden
        let blocks = buf.len() / self.block_size as usize;
        if blocks == 0 || lba + blocks as u64 > self.block_count {
            return Err(BlockError::OutOfRange);
        }

        // Use internal helper - CDB format hidden
        let cdb = Self::build_read_cdb(lba as u32, blocks as u16);

        match self.transport.execute(&cdb, Some(buf), None) {
            Ok(0) => Ok(()),
            Ok(_) => Err(BlockError::Io),
            Err(_) => Err(BlockError::Io),
        }
    }

    fn write_blocks(&mut self, lba: u64, buf: &[u8]) -> Result<(), BlockError> {
        let blocks = buf.len() / self.block_size as usize;
        if blocks == 0 || lba + blocks as u64 > self.block_count {
            return Err(BlockError::OutOfRange);
        }

        let cdb = Self::build_write_cdb(lba as u32, blocks as u16);

        match self.transport.execute(&cdb, None, Some(buf)) {
            Ok(0) => Ok(()),
            _ => Err(BlockError::Io),
        }
    }

    fn block_size(&self) -> u32 {
        self.block_size
    }

    fn block_count(&self) -> u64 {
        self.block_count
    }
}

// =============================================================================
// Implementation: BOT over ByteTransport (generic)
// =============================================================================

/// BOT protocol - implements ScsiTransport over any ByteTransport
///
/// Internals hidden: CBW/CSW format, tag management, protocol state
pub struct BotScsi<T: ByteTransport> {
    // Private - users don't see BOT details
    transport: T,
    tag: u32,
}

// Private constants - not exposed
const CBW_SIG: u32 = 0x43425355;
const CSW_SIG: u32 = 0x53425355;

impl<T: ByteTransport> BotScsi<T> {
    pub fn new(transport: T) -> Self {
        Self { transport, tag: 1 }
    }

    // Internal: build CBW - format completely hidden
    fn build_cbw(&mut self, cdb: &[u8], data_len: u32, dir_in: bool) -> [u8; 31] {
        let tag = self.tag;
        self.tag = self.tag.wrapping_add(1);

        let mut cbw = [0u8; 31];
        cbw[0..4].copy_from_slice(&CBW_SIG.to_le_bytes());
        cbw[4..8].copy_from_slice(&tag.to_le_bytes());
        cbw[8..12].copy_from_slice(&data_len.to_le_bytes());
        cbw[12] = if dir_in { 0x80 } else { 0x00 };
        cbw[14] = cdb.len().min(16) as u8;
        cbw[15..15 + cdb.len().min(16)].copy_from_slice(&cdb[..cdb.len().min(16)]);
        cbw
    }

    // Internal: parse CSW - format completely hidden
    fn parse_csw(&self, csw: &[u8], expected_tag: u32) -> Result<u8, TransportError> {
        if csw.len() < 13 {
            return Err(TransportError::Stall);
        }
        let sig = u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]);
        let tag = u32::from_le_bytes([csw[4], csw[5], csw[6], csw[7]]);
        if sig != CSW_SIG || tag != expected_tag {
            return Err(TransportError::Stall);
        }
        Ok(csw[12])
    }
}

impl<T: ByteTransport> ScsiTransport for BotScsi<T> {
    fn execute(
        &mut self,
        cdb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Result<u8, TransportError> {
        // Determine direction and length
        let (data_len, dir_in) = match (&data_in, &data_out) {
            (Some(b), None) => (b.len() as u32, true),
            (None, Some(b)) => (b.len() as u32, false),
            _ => (0, true),
        };

        let expected_tag = self.tag; // Capture before build_cbw increments
        let cbw = self.build_cbw(cdb, data_len, dir_in);

        // Protocol sequence - all internal
        self.transport.send(&cbw)?;

        if let Some(buf) = data_in {
            self.transport.recv(buf)?;
        } else if let Some(buf) = data_out {
            self.transport.send(buf)?;
        }

        let mut csw = [0u8; 13];
        self.transport.recv(&mut csw)?;

        self.parse_csw(&csw, expected_tag)
    }
}
