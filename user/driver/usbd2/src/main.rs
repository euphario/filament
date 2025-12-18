//! usbd2 - Ring Buffer Block Device Server
//!
//! Test implementation of a block device server using shared ring buffers.
//! For now, simulates a block device with test data to verify the ring
//! buffer infrastructure works correctly.

#![no_std]
#![no_main]

use userlib::{println, syscall};
use userlib::ring::{BlockRing, BlockRequest, BlockResponse};

/// Simulated block device parameters
const BLOCK_SIZE: u32 = 512;
const BLOCK_COUNT: u64 = 1024; // 512KB simulated device

#[unsafe(no_mangle)]
fn main() {
    println!("[usbd2] Ring-based block device server starting...");

    // Register port for clients to connect
    let result = syscall::port_register(b"blk2");
    if result < 0 {
        println!("[usbd2] ERROR: Failed to register port: {}", result);
        syscall::exit(1);
    }
    let listen_channel = result as u32;
    println!("[usbd2] Registered 'blk2' port, listening on channel {}", listen_channel);

    // Create ring buffer (64 entries, 1MB data buffer)
    let ring = match BlockRing::create(64, 1024 * 1024) {
        Some(r) => r,
        None => {
            println!("[usbd2] ERROR: Failed to create ring buffer");
            syscall::exit(1);
        }
    };
    println!("[usbd2] Created ring buffer (shmem_id={})", ring.shmem_id());
    println!("[usbd2]   Data buffer: {} bytes at phys 0x{:x}", ring.data_size(), ring.data_phys());

    // Initialize simulated disk data in the data buffer
    // Each sector contains its LBA number as a pattern
    init_simulated_disk(&ring);

    // Wait for client connection
    println!("[usbd2] Waiting for client connection...");
    loop {
        let client_channel = syscall::port_accept(listen_channel);
        if client_channel < 0 {
            syscall::yield_now();
            continue;
        }
        let client_channel = client_channel as u32;
        println!("[usbd2] Client connected on channel {}", client_channel);

        // Get client's PID (we need it to allow shmem access)
        // For now, we'll use a simple message exchange
        let mut msg_buf = [0u8; 64];

        // Wait for client's "hello" message with their PID
        loop {
            let recv_len = syscall::receive(client_channel, &mut msg_buf);
            if recv_len > 0 {
                break;
            }
            syscall::yield_now();
        }

        // Parse client PID from message (first 4 bytes)
        let client_pid = u32::from_le_bytes([msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3]]);
        println!("[usbd2] Client PID: {}", client_pid);

        // Allow client to map our ring buffer
        if !ring.allow(client_pid) {
            println!("[usbd2] ERROR: Failed to allow client access to ring");
            continue;
        }
        println!("[usbd2] Allowed client access to ring buffer");

        // Send shmem_id to client
        let shmem_id_bytes = ring.shmem_id().to_le_bytes();
        msg_buf[..4].copy_from_slice(&shmem_id_bytes);
        syscall::send(client_channel, &msg_buf[..4]);
        println!("[usbd2] Sent shmem_id {} to client", ring.shmem_id());

        // Process requests from ring buffer
        println!("[usbd2] Processing requests...");
        process_requests(&ring);

        println!("[usbd2] Client disconnected");
    }
}

/// Initialize simulated disk with test pattern
fn init_simulated_disk(ring: &BlockRing) {
    let data = unsafe {
        core::slice::from_raw_parts_mut(
            ring.data_ptr(),
            ring.data_size()
        )
    };

    // Fill first few sectors with recognizable patterns
    let sectors = ring.data_size() / BLOCK_SIZE as usize;
    for sector in 0..sectors.min(64) {
        let offset = sector * BLOCK_SIZE as usize;
        let sector_data = &mut data[offset..offset + BLOCK_SIZE as usize];

        // First 8 bytes: sector number (LBA)
        sector_data[0..8].copy_from_slice(&(sector as u64).to_le_bytes());

        // Next 8 bytes: magic pattern
        sector_data[8..16].copy_from_slice(b"USBD2TST");

        // Rest: sector number repeated
        for i in (16..BLOCK_SIZE as usize).step_by(8) {
            if i + 8 <= BLOCK_SIZE as usize {
                sector_data[i..i+8].copy_from_slice(&(sector as u64).to_le_bytes());
            }
        }
    }
    println!("[usbd2] Initialized {} simulated sectors", sectors.min(64));
}

/// Process requests from the ring buffer
fn process_requests(ring: &BlockRing) {
    let mut requests_processed = 0u32;

    loop {
        // Wait for requests (with 5 second timeout for testing)
        if ring.sq_pending() == 0 {
            if !ring.wait(5000) {
                // Timeout - check if client is still there
                if requests_processed > 0 {
                    println!("[usbd2] Timeout after {} requests, assuming client done", requests_processed);
                    return;
                }
            }
        }

        // Process all pending requests
        while let Some(req) = ring.next_request() {
            let resp = handle_request(ring, &req);
            ring.complete(&resp);
            requests_processed += 1;

            if requests_processed % 100 == 0 {
                println!("[usbd2] Processed {} requests", requests_processed);
            }
        }

        // Notify client of completions
        ring.notify();
    }
}

/// Handle a single block request
fn handle_request(ring: &BlockRing, req: &BlockRequest) -> BlockResponse {
    match req.cmd {
        BlockRequest::CMD_INFO => {
            // Return device info
            BlockResponse::info(req.tag, BLOCK_SIZE, BLOCK_COUNT)
        }
        BlockRequest::CMD_READ => {
            // Validate request
            if req.lba >= BLOCK_COUNT {
                return BlockResponse::error(req.tag, -22); // EINVAL
            }
            if req.count == 0 {
                return BlockResponse::error(req.tag, -22);
            }

            let bytes = (req.count as u64) * (BLOCK_SIZE as u64);
            let buf_offset = req.buf_offset as usize;

            // Check buffer bounds
            if buf_offset + bytes as usize > ring.data_size() {
                return BlockResponse::error(req.tag, -22);
            }

            // For simulated disk, data is already in buffer from init
            // In real implementation, we'd DMA from USB device here

            // Just return success with byte count
            BlockResponse::ok(req.tag, bytes as u32)
        }
        BlockRequest::CMD_WRITE => {
            // Validate request
            if req.lba >= BLOCK_COUNT {
                return BlockResponse::error(req.tag, -22);
            }
            if req.count == 0 {
                return BlockResponse::error(req.tag, -22);
            }

            let bytes = (req.count as u64) * (BLOCK_SIZE as u64);

            // For simulated disk, just return success
            // In real implementation, we'd DMA to USB device here

            BlockResponse::ok(req.tag, bytes as u32)
        }
        _ => {
            BlockResponse::error(req.tag, -38) // ENOSYS
        }
    }
}
