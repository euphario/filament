//! Mass Storage Class (MSC) Driver
//!
//! This driver handles USB Mass Storage devices using the Bulk-Only Transport (BOT)
//! protocol. It communicates with the usbd service via IPC to perform USB operations
//! and exposes block device access via the "block" scheme.

#![no_std]
#![no_main]

use userlib::{println, print, syscall};
use usb::{
    // Protocol
    UsbRequest, UsbStatus, UsbMessageHeader, UsbResponseHeader,
    BulkTransferRequest, BulkTransferResponse,
    USB_MSG_MAX_SIZE,
    // MSC/SCSI types
    msc_const as msc, scsi, Cbw, Csw,
    // Utilities
    print_hex32, print_hex8,
};

/// MSC driver context
struct MscDriver {
    /// Channel to usbd service
    usb_channel: u32,
    /// Slot ID of the connected device
    slot_id: u32,
    /// Bulk IN endpoint address
    bulk_in_ep: u8,
    /// Bulk OUT endpoint address
    bulk_out_ep: u8,
    /// Command tag counter
    next_tag: u32,
    /// Message buffer
    msg_buf: [u8; USB_MSG_MAX_SIZE],
}

impl MscDriver {
    /// Connect to usbd service
    fn connect() -> Option<Self> {
        println!("  Connecting to usbd service...");

        let result = syscall::port_connect(b"usb");
        if result < 0 {
            println!("  ERROR: Failed to connect to usbd: {}", result);
            return None;
        }

        let channel = result as u32;
        println!("  Connected to usbd (channel {})", channel);

        Some(Self {
            usb_channel: channel,
            slot_id: 0,
            bulk_in_ep: 0,
            bulk_out_ep: 0,
            next_tag: 1,
            msg_buf: [0u8; USB_MSG_MAX_SIZE],
        })
    }

    /// Get next command tag
    fn next_tag(&mut self) -> u32 {
        let tag = self.next_tag;
        self.next_tag = self.next_tag.wrapping_add(1);
        if self.next_tag == 0 {
            self.next_tag = 1;  // Tag 0 is reserved
        }
        tag
    }

    /// Send a request to usbd and receive response
    fn usb_request(&mut self, request: UsbRequest, payload: &[u8]) -> Result<usize, UsbStatus> {
        // Build message header
        let header = UsbMessageHeader::new(request, payload.len() as u32);
        let header_bytes = header.to_bytes();

        // Copy header and payload to buffer
        self.msg_buf[..UsbMessageHeader::SIZE].copy_from_slice(&header_bytes);
        if !payload.is_empty() {
            self.msg_buf[UsbMessageHeader::SIZE..UsbMessageHeader::SIZE + payload.len()]
                .copy_from_slice(payload);
        }

        // Send request
        let send_len = UsbMessageHeader::SIZE + payload.len();
        let send_result = syscall::send(self.usb_channel, &self.msg_buf[..send_len]);
        if send_result < 0 {
            println!("  USB send failed: {}", send_result);
            return Err(UsbStatus::Error);
        }

        // Receive response
        let recv_result = syscall::receive(self.usb_channel, &mut self.msg_buf);
        if recv_result < 0 {
            println!("  USB receive failed: {}", recv_result);
            return Err(UsbStatus::Error);
        }

        let recv_len = recv_result as usize;
        if recv_len < UsbResponseHeader::SIZE {
            println!("  Response too short: {}", recv_len);
            return Err(UsbStatus::InvalidRequest);
        }

        // Parse response header
        let resp_header = UsbResponseHeader::from_bytes(&self.msg_buf)
            .ok_or(UsbStatus::InvalidRequest)?;

        if resp_header.status != UsbStatus::Ok {
            return Err(resp_header.status);
        }

        Ok(recv_len - UsbResponseHeader::SIZE)
    }

    /// Send bulk OUT data
    fn bulk_out(&mut self, data: &[u8]) -> Result<usize, UsbStatus> {
        // Build bulk transfer request
        let req = BulkTransferRequest {
            slot_id: self.slot_id,
            endpoint: self.bulk_out_ep,
            _pad: [0; 3],
            length: data.len() as u32,
        };

        // Serialize request + data
        let req_bytes: [u8; core::mem::size_of::<BulkTransferRequest>()] =
            unsafe { core::mem::transmute(req) };

        let payload_len = req_bytes.len() + data.len();
        let mut payload = [0u8; USB_MSG_MAX_SIZE - UsbMessageHeader::SIZE];
        payload[..req_bytes.len()].copy_from_slice(&req_bytes);
        payload[req_bytes.len()..req_bytes.len() + data.len()].copy_from_slice(data);

        let bytes = self.usb_request(UsbRequest::BulkOut, &payload[..payload_len])?;

        // Parse response
        if bytes >= core::mem::size_of::<BulkTransferResponse>() {
            let resp_bytes = &self.msg_buf[UsbResponseHeader::SIZE..];
            let resp: BulkTransferResponse = unsafe {
                core::ptr::read_unaligned(resp_bytes.as_ptr() as *const BulkTransferResponse)
            };
            Ok(resp.bytes_transferred as usize)
        } else {
            Ok(0)
        }
    }

    /// Receive bulk IN data
    fn bulk_in(&mut self, max_len: usize) -> Result<&[u8], UsbStatus> {
        // Build bulk transfer request
        let req = BulkTransferRequest {
            slot_id: self.slot_id,
            endpoint: self.bulk_in_ep,
            _pad: [0; 3],
            length: max_len as u32,
        };

        let req_bytes: [u8; core::mem::size_of::<BulkTransferRequest>()] =
            unsafe { core::mem::transmute(req) };

        let bytes = self.usb_request(UsbRequest::BulkIn, &req_bytes)?;

        // Response contains BulkTransferResponse header + data
        if bytes >= core::mem::size_of::<BulkTransferResponse>() {
            let data_offset = UsbResponseHeader::SIZE + core::mem::size_of::<BulkTransferResponse>();
            let data_len = bytes - core::mem::size_of::<BulkTransferResponse>();
            Ok(&self.msg_buf[data_offset..data_offset + data_len])
        } else {
            Ok(&[])
        }
    }

    /// Send CBW (Command Block Wrapper)
    fn send_cbw(&mut self, cbw: &Cbw) -> Result<(), UsbStatus> {
        let cbw_bytes: &[u8] = unsafe {
            core::slice::from_raw_parts(
                cbw as *const Cbw as *const u8,
                core::mem::size_of::<Cbw>()
            )
        };

        let bytes_sent = self.bulk_out(cbw_bytes)?;
        if bytes_sent != core::mem::size_of::<Cbw>() {
            println!("  CBW send incomplete: {} bytes", bytes_sent);
            return Err(UsbStatus::Error);
        }
        Ok(())
    }

    /// Receive CSW (Command Status Wrapper)
    fn receive_csw(&mut self) -> Result<Csw, UsbStatus> {
        let data = self.bulk_in(core::mem::size_of::<Csw>())?;

        if data.len() < core::mem::size_of::<Csw>() {
            println!("  CSW too short: {} bytes", data.len());
            return Err(UsbStatus::Error);
        }

        let csw: Csw = unsafe {
            core::ptr::read_unaligned(data.as_ptr() as *const Csw)
        };

        if csw.signature != msc::CSW_SIGNATURE {
            print!("  Invalid CSW signature: ");
            print_hex32(csw.signature);
            println!();
            return Err(UsbStatus::Error);
        }

        Ok(csw)
    }

    /// Execute SCSI command with data IN phase
    fn scsi_command_in(&mut self, cmd: &[u8], data_length: u32) -> Result<(Csw, Vec<u8>), UsbStatus> {
        let tag = self.next_tag();

        // Build and send CBW
        let cbw = Cbw::new(tag, data_length, true, 0, cmd);
        self.send_cbw(&cbw)?;

        // Data IN phase (if needed)
        let data = if data_length > 0 {
            let d = self.bulk_in(data_length as usize)?;
            // We need to copy because data references self.msg_buf
            let mut v = Vec::new();
            v.extend_from_slice(d);
            v
        } else {
            Vec::new()
        };

        // Status phase
        let csw = self.receive_csw()?;

        let csw_tag = csw.tag;
        if csw_tag != tag {
            println!("  CSW tag mismatch: expected {}, got {}", tag, csw_tag);
            return Err(UsbStatus::Error);
        }

        Ok((csw, data))
    }

    /// SCSI READ CAPACITY (10)
    fn scsi_read_capacity(&mut self) -> Result<(u32, u32), UsbStatus> {
        let cmd = [scsi::READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        println!("  SCSI READ_CAPACITY(10)...");

        let (csw, data) = self.scsi_command_in(&cmd, 8)?;

        if csw.status != msc::CSW_STATUS_PASSED {
            println!("  READ_CAPACITY failed: status={}", csw.status);
            return Err(UsbStatus::Error);
        }

        if data.len() < 8 {
            println!("  READ_CAPACITY response too short");
            return Err(UsbStatus::Error);
        }

        let last_lba = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let block_size = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);

        println!("    Last LBA: {}", last_lba);
        println!("    Block size: {} bytes", block_size);
        println!("    Capacity: {} MB", ((last_lba as u64 + 1) * block_size as u64) / (1024 * 1024));

        Ok((last_lba, block_size))
    }

    /// SCSI READ (10)
    fn scsi_read(&mut self, lba: u32, count: u16, block_size: u32) -> Result<Vec<u8>, UsbStatus> {
        let cmd = [
            scsi::READ_10,
            0,
            ((lba >> 24) & 0xFF) as u8,
            ((lba >> 16) & 0xFF) as u8,
            ((lba >> 8) & 0xFF) as u8,
            (lba & 0xFF) as u8,
            0,
            ((count >> 8) & 0xFF) as u8,
            (count & 0xFF) as u8,
            0,
        ];

        let data_length = (count as u32) * block_size;
        println!("  SCSI READ(10): LBA={}, count={}, bytes={}", lba, count, data_length);

        let (csw, data) = self.scsi_command_in(&cmd, data_length)?;

        if csw.status != msc::CSW_STATUS_PASSED {
            println!("  READ failed: status={}", csw.status);
            return Err(UsbStatus::Error);
        }

        Ok(data)
    }

    /// SCSI INQUIRY
    fn scsi_inquiry(&mut self) -> Result<(), UsbStatus> {
        let cmd = [scsi::INQUIRY, 0, 0, 0, 36, 0, 0, 0, 0, 0];

        println!("  SCSI INQUIRY...");

        let (csw, data) = self.scsi_command_in(&cmd, 36)?;

        if csw.status != msc::CSW_STATUS_PASSED {
            println!("  INQUIRY failed: status={}", csw.status);
            return Err(UsbStatus::Error);
        }

        if data.len() >= 36 {
            // Parse inquiry data
            let peripheral = data[0];
            let device_type = peripheral & 0x1F;
            print!("    Device type: ");
            match device_type {
                0x00 => println!("Direct access (disk)"),
                0x05 => println!("CD/DVD-ROM"),
                0x07 => println!("Optical memory"),
                0x0E => println!("Simplified direct access"),
                _ => println!("0x{:02x}", device_type),
            }

            // Vendor (bytes 8-15)
            print!("    Vendor: ");
            for i in 8..16 {
                if data[i] >= 0x20 && data[i] < 0x7F {
                    print!("{}", data[i] as char);
                }
            }
            println!();

            // Product (bytes 16-31)
            print!("    Product: ");
            for i in 16..32 {
                if data[i] >= 0x20 && data[i] < 0x7F {
                    print!("{}", data[i] as char);
                }
            }
            println!();
        }

        Ok(())
    }

    /// Run storage tests
    fn run_tests(&mut self) {
        println!();
        println!("=== MSC Driver Tests ===");

        // INQUIRY
        if let Err(e) = self.scsi_inquiry() {
            println!("  INQUIRY failed: {:?}", e);
            return;
        }

        // READ CAPACITY
        let (_last_lba, block_size) = match self.scsi_read_capacity() {
            Ok(v) => v,
            Err(e) => {
                println!("  READ_CAPACITY failed: {:?}", e);
                return;
            }
        };

        // Read first sector
        println!();
        match self.scsi_read(0, 1, block_size) {
            Ok(data) => {
                println!("  Read {} bytes from LBA 0", data.len());

                // Dump first 64 bytes
                println!("  First 64 bytes:");
                for row in 0..4 {
                    print!("    {:04x}: ", row * 16);
                    for col in 0..16 {
                        let idx = row * 16 + col;
                        if idx < data.len() {
                            print_hex8(data[idx]);
                            print!(" ");
                        }
                    }
                    println!();
                }

                // Check for MBR signature
                if data.len() >= 512 {
                    if data[510] == 0x55 && data[511] == 0xAA {
                        println!("  MBR signature found!");
                    }
                }
            }
            Err(e) => {
                println!("  READ failed: {:?}", e);
            }
        }
    }
}

// Simple Vec implementation for no_std
struct Vec<T> {
    data: [T; 4096],
    len: usize,
}

impl<T: Copy + Default> Vec<T> {
    fn new() -> Self {
        Self {
            data: [T::default(); 4096],
            len: 0,
        }
    }

    fn extend_from_slice(&mut self, slice: &[T]) {
        for &item in slice {
            if self.len < self.data.len() {
                self.data[self.len] = item;
                self.len += 1;
            }
        }
    }

    fn len(&self) -> usize {
        self.len
    }
}

impl<T> core::ops::Deref for Vec<T> {
    type Target = [T];
    fn deref(&self) -> &[T] {
        &self.data[..self.len]
    }
}

impl<T> core::ops::Index<usize> for Vec<T> {
    type Output = T;
    fn index(&self, index: usize) -> &T {
        &self.data[index]
    }
}

#[unsafe(no_mangle)]
fn main() {
    println!("========================================");
    println!("  Mass Storage Class Driver");
    println!("  USB BOT/SCSI Protocol");
    println!("========================================");
    println!();

    // Register "block" port for block device access
    let port_result = syscall::port_register(b"block");
    if port_result < 0 {
        if port_result == -17 {
            println!("ERROR: Block driver already running");
        } else {
            println!("ERROR: Failed to register block port: {}", port_result);
        }
        syscall::exit(1);
    }
    let block_port = port_result as u32;
    println!("  Registered 'block' port (channel {})", block_port);

    // Connect to USB daemon
    let mut driver = match MscDriver::connect() {
        Some(d) => d,
        None => {
            println!("ERROR: Failed to connect to usbd");
            syscall::exit(1);
        }
    };

    // TODO: Enumerate USB devices and find mass storage device
    // For now, we need usbd to tell us about connected devices
    println!();
    println!("  Waiting for device notification from usbd...");

    // The driver would normally:
    // 1. Request device list from usbd
    // 2. Find mass storage devices (class 0x08)
    // 3. Request endpoint configuration
    // 4. Run SCSI commands
    // 5. Expose block device via "block" scheme

    // For now, exit as usbd doesn't yet have IPC handling
    println!();
    println!("  MSC driver initialized (waiting for usbd IPC support)");
    println!();

    syscall::exit(0);
}
