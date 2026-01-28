//! xHCI Bulk Transport - thin hardware layer
//!
//! Implements ByteTransport trait. All the ring management, cycle bits,
//! TRBs, doorbells hide here. Upper layers just see send/recv.

use crate::block::{ByteTransport, TransportError};
use usb::trb::{Trb, trb_type};
use usb::ring::EventRing;
use usb::XhciController;
use usb::transfer::dsb;

/// xHCI bulk endpoint pair (IN + OUT)
pub struct XhciBulk<'a> {
    // Hardware references
    xhci: &'a mut XhciController,
    evt_ring: &'a mut EventRing,

    // Endpoint info
    slot_id: u8,
    bulk_in_dci: u8,
    bulk_out_dci: u8,

    // Ring state - this is what upper layers don't see
    out_ring: *mut Trb,
    out_ring_phys: u64,
    out_enqueue: usize,
    out_cycle: bool,

    in_ring: *mut Trb,
    in_ring_phys: u64,
    in_enqueue: usize,
    in_cycle: bool,

    // DMA buffer for transfers
    dma_buf: *mut u8,
    dma_phys: u64,
    dma_size: usize,
}

const RING_SIZE: usize = 64;
const RING_USABLE: usize = 63; // Last slot is Link TRB

impl<'a> XhciBulk<'a> {
    pub fn new(
        xhci: &'a mut XhciController,
        evt_ring: &'a mut EventRing,
        slot_id: u8,
        bulk_in_dci: u8,
        bulk_out_dci: u8,
        out_ring: *mut Trb,
        out_ring_phys: u64,
        in_ring: *mut Trb,
        in_ring_phys: u64,
        dma_buf: *mut u8,
        dma_phys: u64,
        dma_size: usize,
    ) -> Self {
        Self {
            xhci,
            evt_ring,
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            out_ring,
            out_ring_phys,
            out_enqueue: 0,
            out_cycle: true,
            in_ring,
            in_ring_phys,
            in_enqueue: 0,
            in_cycle: true,
            dma_buf,
            dma_phys,
            dma_size,
        }
    }

    /// Create with specified initial ring state (for persistent state across calls)
    pub fn with_state(
        xhci: &'a mut XhciController,
        evt_ring: &'a mut EventRing,
        slot_id: u8,
        bulk_in_dci: u8,
        bulk_out_dci: u8,
        out_ring: *mut Trb,
        out_ring_phys: u64,
        in_ring: *mut Trb,
        in_ring_phys: u64,
        dma_buf: *mut u8,
        dma_phys: u64,
        dma_size: usize,
        out_enqueue: usize,
        out_cycle: bool,
        in_enqueue: usize,
        in_cycle: bool,
    ) -> Self {
        Self {
            xhci,
            evt_ring,
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            out_ring,
            out_ring_phys,
            out_enqueue,
            out_cycle,
            in_ring,
            in_ring_phys,
            in_enqueue,
            in_cycle,
            dma_buf,
            dma_phys,
            dma_size,
        }
    }

    /// Get current ring state (for saving after use)
    pub fn ring_state(&self) -> (usize, bool, usize, bool) {
        (self.out_enqueue, self.out_cycle, self.in_enqueue, self.in_cycle)
    }

    /// Enqueue a TRB on the OUT ring, handle wrap-around
    fn enqueue_out(&mut self, trb: Trb) {
        unsafe {
            let ptr = self.out_ring.add(self.out_enqueue);
            core::ptr::write_volatile(ptr, trb);
        }

        self.out_enqueue += 1;
        if self.out_enqueue >= RING_USABLE {
            // Update link TRB cycle bit and wrap
            self.out_cycle = !self.out_cycle;
            unsafe {
                let link = self.out_ring.add(RING_SIZE - 1);
                let mut link_trb = core::ptr::read_volatile(link);
                if self.out_cycle {
                    link_trb.control |= 1;
                } else {
                    link_trb.control &= !1;
                }
                core::ptr::write_volatile(link, link_trb);
            }
            self.out_enqueue = 0;
        }
    }

    /// Enqueue a TRB on the IN ring, handle wrap-around
    fn enqueue_in(&mut self, trb: Trb) {
        unsafe {
            let ptr = self.in_ring.add(self.in_enqueue);
            core::ptr::write_volatile(ptr, trb);
        }

        self.in_enqueue += 1;
        if self.in_enqueue >= RING_USABLE {
            self.in_cycle = !self.in_cycle;
            unsafe {
                let link = self.in_ring.add(RING_SIZE - 1);
                let mut link_trb = core::ptr::read_volatile(link);
                if self.in_cycle {
                    link_trb.control |= 1;
                } else {
                    link_trb.control &= !1;
                }
                core::ptr::write_volatile(link, link_trb);
            }
            self.in_enqueue = 0;
        }
    }

    /// Wait for transfer completion
    fn wait_completion(&mut self) -> Result<usize, TransportError> {
        for _i in 0..100_000 {
            if let Some(evt) = self.evt_ring.dequeue() {
                if evt.get_type() == trb_type::TRANSFER_EVENT {
                    let cc = evt.event_completion_code();
                    let residual = (evt.status & 0xFFFFFF) as usize;

                    self.xhci.update_erdp(0, self.evt_ring.erdp());

                    // Debug output completion code for failures
                    if cc != 1 && cc != 13 {
                        // Non-success completion code - log it
                        let msg: &[u8] = match cc {
                            6 => b"[xhci_bulk] Stall\n",
                            _ => b"[xhci_bulk] Protocol error\n",
                        };
                        userlib::syscall::debug_write(msg);
                    }

                    match cc {
                        1 => return Ok(residual),      // Success
                        13 => return Ok(residual),     // Short packet
                        6 => return Err(TransportError::Stall),
                        _ => return Err(TransportError::Protocol),
                    }
                }
            }
            self.xhci.update_erdp(0, self.evt_ring.erdp());
            core::hint::spin_loop();
        }
        userlib::syscall::debug_write(b"[xhci_bulk] timeout\n");
        Err(TransportError::Timeout)
    }
}

impl<'a> ByteTransport for XhciBulk<'a> {
    fn send(&mut self, data: &[u8]) -> Result<(), TransportError> {
        let len = data.len().min(self.dma_size);

        // Copy to DMA buffer
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), self.dma_buf, len);
        }

        // Build Normal TRB
        let cycle = if self.out_cycle { 1 } else { 0 };
        let trb = Trb {
            param: self.dma_phys,
            status: len as u32,
            control: (trb_type::NORMAL << 10) | (1 << 5) | cycle, // IOC=1
        };

        self.enqueue_out(trb);
        dsb();

        // Ring doorbell
        self.xhci.ring_doorbell(self.slot_id, self.bulk_out_dci);

        // Wait for completion
        self.wait_completion()?;
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        let max_len = buf.len().min(self.dma_size);

        // Build Normal TRB for receive
        let cycle = if self.in_cycle { 1 } else { 0 };
        let trb = Trb {
            param: self.dma_phys,
            status: max_len as u32,
            control: (trb_type::NORMAL << 10) | (1 << 5) | cycle, // IOC=1
        };

        self.enqueue_in(trb);
        dsb();

        // Ring doorbell
        self.xhci.ring_doorbell(self.slot_id, self.bulk_in_dci);

        // Wait for completion
        let residual = self.wait_completion()?;
        let transferred = max_len.saturating_sub(residual);

        // Copy from DMA buffer
        unsafe {
            core::ptr::copy_nonoverlapping(self.dma_buf, buf.as_mut_ptr(), transferred);
        }

        Ok(transferred)
    }
}

// =============================================================================
// Batched BOT - queue all TRBs for a command, single doorbell per endpoint
// =============================================================================
// This is where "layers at different speeds" matters:
// - We queue CBW + data TRBs on OUT ring
// - We queue CSW TRB on IN ring
// - Ring both doorbells
// - Hardware processes while we wait once

impl<'a> XhciBulk<'a> {
    /// Execute a complete BOT command with batched TRBs
    ///
    /// Instead of: send CBW → wait → send data → wait → recv CSW → wait
    /// We do:      queue CBW → queue data → ring OUT → queue CSW → ring IN → wait
    ///
    /// Hardware runs through the queued TRBs at wire speed.
    pub fn bot_command_batched(
        &mut self,
        cbw: &[u8],           // 31 bytes
        data_out: Option<&[u8]>,
        data_in: Option<&mut [u8]>,
        csw: &mut [u8],       // 13 bytes
    ) -> Result<(), TransportError> {
        // Use internal DMA buffer split: [0..64] = CBW, [64..128] = CSW, [128..] = data
        const CBW_OFF: usize = 0;
        const CSW_OFF: usize = 64;
        const DATA_OFF: usize = 128;

        let data_size = self.dma_size - DATA_OFF;

        // Copy CBW to DMA
        unsafe {
            core::ptr::copy_nonoverlapping(cbw.as_ptr(), self.dma_buf.add(CBW_OFF), 31);
        }

        // Queue CBW on OUT ring
        let out_cycle = if self.out_cycle { 1 } else { 0 };

        // Data phase
        if let Some(data) = data_out {
            // CBW without IOC (data TRB will have IOC)
            let cbw_trb = Trb {
                param: self.dma_phys + CBW_OFF as u64,
                status: 31,
                control: (trb_type::NORMAL << 10) | out_cycle, // No IOC
            };
            self.enqueue_out(cbw_trb);

            let len = data.len().min(data_size);
            unsafe {
                core::ptr::copy_nonoverlapping(data.as_ptr(), self.dma_buf.add(DATA_OFF), len);
            }
            // Data OUT TRB with IOC (last OUT transfer)
            let data_trb = Trb {
                param: self.dma_phys + DATA_OFF as u64,
                status: len as u32,
                control: (trb_type::NORMAL << 10) | (1 << 5) | out_cycle, // IOC=1
            };
            self.enqueue_out(data_trb);
        } else {
            // No data out - CBW is last OUT TRB, needs IOC
            let cbw_trb = Trb {
                param: self.dma_phys + CBW_OFF as u64,
                status: 31,
                control: (trb_type::NORMAL << 10) | (1 << 5) | out_cycle, // IOC=1
            };
            self.enqueue_out(cbw_trb);
        }

        dsb();
        self.xhci.ring_doorbell(self.slot_id, self.bulk_out_dci);

        // Wait for OUT phase to complete
        self.wait_completion()?;

        // Data IN phase (if any)
        if let Some(buf) = data_in {
            let len = buf.len().min(data_size);
            let in_cycle = if self.in_cycle { 1 } else { 0 };
            let data_trb = Trb {
                param: self.dma_phys + DATA_OFF as u64,
                status: len as u32,
                control: (trb_type::NORMAL << 10) | (1 << 5) | in_cycle, // IOC=1
            };
            self.enqueue_in(data_trb);
            dsb();
            self.xhci.ring_doorbell(self.slot_id, self.bulk_in_dci);
            let residual = self.wait_completion()?;
            let transferred = len.saturating_sub(residual);
            unsafe {
                core::ptr::copy_nonoverlapping(self.dma_buf.add(DATA_OFF), buf.as_mut_ptr(), transferred);
            }
        }

        // CSW on IN ring
        let in_cycle = if self.in_cycle { 1 } else { 0 };
        let csw_trb = Trb {
            param: self.dma_phys + CSW_OFF as u64,
            status: 13,
            control: (trb_type::NORMAL << 10) | (1 << 5) | in_cycle, // IOC=1
        };
        self.enqueue_in(csw_trb);
        dsb();
        self.xhci.ring_doorbell(self.slot_id, self.bulk_in_dci);

        self.wait_completion()?;

        // Copy CSW out
        unsafe {
            core::ptr::copy_nonoverlapping(self.dma_buf.add(CSW_OFF), csw.as_mut_ptr(), 13);
        }

        Ok(())
    }

    /// Execute BOT READ with zero-copy - data DMAs directly to caller's buffer
    ///
    /// CBW/CSW use internal scratch, DATA TRB points to caller's physical address.
    pub fn bot_read_zero_copy(
        &mut self,
        cbw: &[u8],              // 31 bytes
        data_phys: u64,          // Physical address for data DMA (caller's buffer)
        data_len: u32,           // Length of data to receive
        csw: &mut [u8],          // 13 bytes output
    ) -> Result<u32, TransportError> {
        // CBW and CSW use our internal scratch buffer
        // [0..64] = CBW, [64..128] = CSW
        const CBW_OFF: usize = 0;
        const CSW_OFF: usize = 64;

        // Copy CBW to scratch
        unsafe {
            core::ptr::copy_nonoverlapping(cbw.as_ptr(), self.dma_buf.add(CBW_OFF), 31);
        }

        // Queue CBW on OUT ring with IOC (no data-out phase for reads)
        let out_cycle = if self.out_cycle { 1 } else { 0 };
        let cbw_trb = Trb {
            param: self.dma_phys + CBW_OFF as u64,
            status: 31,
            control: (trb_type::NORMAL << 10) | (1 << 5) | out_cycle, // IOC=1
        };
        self.enqueue_out(cbw_trb);

        dsb();
        self.xhci.ring_doorbell(self.slot_id, self.bulk_out_dci);
        self.wait_completion()?;

        // Data IN - DMA directly to caller's physical address (ZERO COPY!)
        let in_cycle = if self.in_cycle { 1 } else { 0 };
        let data_trb = Trb {
            param: data_phys,  // Caller's buffer, not our scratch!
            status: data_len,
            control: (trb_type::NORMAL << 10) | (1 << 5) | in_cycle, // IOC=1
        };
        self.enqueue_in(data_trb);
        dsb();
        self.xhci.ring_doorbell(self.slot_id, self.bulk_in_dci);
        let residual = self.wait_completion()?;
        let transferred = (data_len as usize).saturating_sub(residual) as u32;

        // CSW - to our scratch buffer
        let in_cycle = if self.in_cycle { 1 } else { 0 };
        let csw_trb = Trb {
            param: self.dma_phys + CSW_OFF as u64,
            status: 13,
            control: (trb_type::NORMAL << 10) | (1 << 5) | in_cycle, // IOC=1
        };
        self.enqueue_in(csw_trb);
        dsb();
        self.xhci.ring_doorbell(self.slot_id, self.bulk_in_dci);
        self.wait_completion()?;

        // Copy CSW out
        unsafe {
            core::ptr::copy_nonoverlapping(self.dma_buf.add(CSW_OFF), csw.as_mut_ptr(), 13);
        }

        Ok(transferred)
    }
}
