//! Composable transport layers for USB
//!
//! Each layer has a simple interface and doesn't care what's above or below.
//! Layers can operate at different speeds - hardware is async/queued,
//! protocol layers work at their own granularity.

/// Bulk transport - the interface between protocol and hardware
///
/// This is what BOT/SCSI sees. It doesn't know about TRBs, rings, or xHCI.
/// Implementation handles all the hardware details.
pub trait BulkTransport {
    /// Send data to device (bulk OUT)
    fn send(&mut self, data: &[u8]) -> Result<(), TransportError>;

    /// Receive data from device (bulk IN)
    /// Returns number of bytes received
    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;
}

/// Transport errors - generic, no hardware details leak through
#[derive(Debug, Clone, Copy)]
pub enum TransportError {
    Timeout,
    Stall,
    Disconnected,
    Protocol,
}

/// SCSI transport - what the block layer sees
///
/// Even simpler: just execute commands. Doesn't know about CBW/CSW or USB.
pub trait ScsiTransport {
    /// Execute a SCSI command
    /// - cdb: command descriptor block
    /// - data_in: buffer for data from device (reads)
    /// - data_out: data to send to device (writes)
    /// Returns SCSI status (0 = good)
    fn execute(
        &mut self,
        cdb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Result<u8, TransportError>;
}

/// BOT (Bulk-Only Transport) - implements ScsiTransport over BulkTransport
///
/// This layer knows about CBW/CSW but not about xHCI or TRBs.
pub struct BotTransport<B: BulkTransport> {
    bulk: B,
    tag: u32,
}

impl<B: BulkTransport> BotTransport<B> {
    pub fn new(bulk: B) -> Self {
        Self { bulk, tag: 1 }
    }
}

impl<B: BulkTransport> ScsiTransport for BotTransport<B> {
    fn execute(
        &mut self,
        cdb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Result<u8, TransportError> {
        use crate::bot::{Cbw, Csw, CBW_SIGNATURE, CSW_SIGNATURE};

        let tag = self.tag;
        self.tag = self.tag.wrapping_add(1);

        // Determine transfer direction and length
        let (data_len, direction_in) = match (&data_in, &data_out) {
            (Some(buf), None) => (buf.len() as u32, true),
            (None, Some(buf)) => (buf.len() as u32, false),
            _ => (0, true),
        };

        // Build and send CBW
        let cbw = Cbw::new(tag, data_len, direction_in, 0, cdb);
        self.bulk.send(cbw.as_bytes())?;

        // Data phase
        if let Some(buf) = data_in {
            self.bulk.recv(buf)?;
        } else if let Some(buf) = data_out {
            self.bulk.send(buf)?;
        }

        // Receive CSW
        let mut csw_buf = [0u8; 13];
        self.bulk.recv(&mut csw_buf)?;

        let csw = Csw::from_bytes(&csw_buf);
        if csw.signature != CSW_SIGNATURE || csw.tag != tag {
            return Err(TransportError::Protocol);
        }

        Ok(csw.status)
    }
}

/// SCSI command helpers - pure functions, no state
pub mod scsi {
    /// Build INQUIRY CDB
    pub fn inquiry(alloc_len: u8) -> [u8; 10] {
        [0x12, 0, 0, 0, alloc_len, 0, 0, 0, 0, 0]
    }

    /// Build READ CAPACITY(10) CDB
    pub fn read_capacity() -> [u8; 10] {
        [0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    /// Build READ(10) CDB
    pub fn read_10(lba: u32, blocks: u16) -> [u8; 10] {
        [
            0x28, 0,
            (lba >> 24) as u8, (lba >> 16) as u8, (lba >> 8) as u8, lba as u8,
            0,
            (blocks >> 8) as u8, blocks as u8,
            0,
        ]
    }

    /// Build TEST UNIT READY CDB
    pub fn test_unit_ready() -> [u8; 10] {
        [0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    /// Parse READ CAPACITY response
    pub fn parse_capacity(data: &[u8]) -> Option<(u32, u32)> {
        if data.len() < 8 { return None; }
        let last_lba = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let block_size = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
        Some((last_lba, block_size))
    }
}
