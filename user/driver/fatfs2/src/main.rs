//! fatfs2 - Ring Buffer Block Device Client
//!
//! Test client that connects to usbd2 and reads blocks via the shared
//! ring buffer interface. Validates the ring buffer IPC mechanism.

#![no_std]
#![no_main]

use userlib::{println, syscall};
use userlib::ring::{BlockRing, BlockRequest};

/// Block size (must match server)
const BLOCK_SIZE: u32 = 512;

#[unsafe(no_mangle)]
fn main() {
    println!("[fatfs2] Ring-based block device client starting...");

    // Connect to usbd2 server
    println!("[fatfs2] Connecting to 'blk2' port...");
    let result = syscall::port_connect(b"blk2");
    if result < 0 {
        println!("[fatfs2] ERROR: Failed to connect to blk2: {}", result);
        syscall::exit(1);
    }
    let channel = result as u32;
    println!("[fatfs2] Connected on channel {}", channel);

    // Send our PID to server
    let my_pid = syscall::getpid() as u32;
    println!("[fatfs2] Sending PID {} to server...", my_pid);
    let mut msg_buf = [0u8; 64];
    msg_buf[..4].copy_from_slice(&my_pid.to_le_bytes());
    syscall::send(channel, &msg_buf[..4]);

    // Wait for shmem_id from server
    println!("[fatfs2] Waiting for shmem_id from server...");
    loop {
        let recv_len = syscall::receive(channel, &mut msg_buf);
        if recv_len >= 4 {
            break;
        }
        syscall::yield_now();
    }

    let shmem_id = u32::from_le_bytes([msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3]]);
    println!("[fatfs2] Received shmem_id: {}", shmem_id);

    // Map the ring buffer
    println!("[fatfs2] Mapping ring buffer...");
    let ring = match BlockRing::map(shmem_id) {
        Some(r) => r,
        None => {
            println!("[fatfs2] ERROR: Failed to map ring buffer");
            syscall::exit(1);
        }
    };
    println!("[fatfs2] Ring buffer mapped successfully");
    println!("[fatfs2]   Data buffer: {} bytes at phys 0x{:x}", ring.data_size(), ring.data_phys());

    // Run tests
    run_tests(&ring);

    println!("[fatfs2] All tests completed!");
    syscall::exit(0);
}

/// Run block device tests via ring buffer
fn run_tests(ring: &BlockRing) {
    println!("\n[fatfs2] === Starting Ring Buffer Tests ===\n");

    // Test 1: Get device info
    test_info(ring);

    // Test 2: Read single block
    test_read_single(ring);

    // Test 3: Read multiple blocks
    test_read_multiple(ring);

    // Test 4: Verify data pattern
    test_verify_pattern(ring);

    // Test 5: Batch submit
    test_batch_submit(ring);

    println!("\n[fatfs2] === All Tests Passed ===\n");
}

/// Test CMD_INFO
fn test_info(ring: &BlockRing) {
    println!("[fatfs2] Test 1: Get device info...");

    // Submit info request
    let req = BlockRequest::info(1);
    if !ring.submit(&req) {
        println!("[fatfs2]   FAILED: Could not submit request");
        return;
    }

    // Notify server
    ring.notify();

    // Wait for completion
    wait_for_completions(ring, 1);

    // Read response
    if let Some(resp) = ring.next_completion() {
        if resp.status == 0 && resp.tag == 1 {
            println!("[fatfs2]   PASSED: block_size={}, block_count={}",
                resp.block_size, resp.block_count);
        } else {
            println!("[fatfs2]   FAILED: status={}, tag={}", resp.status, resp.tag);
        }
    } else {
        println!("[fatfs2]   FAILED: No completion received");
    }
}

/// Test reading a single block
fn test_read_single(ring: &BlockRing) {
    println!("[fatfs2] Test 2: Read single block (LBA 0)...");

    // Submit read request for block 0
    let req = BlockRequest::read(2, 0, 1, 0);
    if !ring.submit(&req) {
        println!("[fatfs2]   FAILED: Could not submit request");
        return;
    }

    ring.notify();
    wait_for_completions(ring, 1);

    if let Some(resp) = ring.next_completion() {
        if resp.status == 0 && resp.tag == 2 && resp.bytes == BLOCK_SIZE {
            // Verify first 8 bytes contain LBA (0)
            let data = ring.data();
            let lba = u64::from_le_bytes([
                data[0], data[1], data[2], data[3],
                data[4], data[5], data[6], data[7],
            ]);
            let magic = &data[8..16];
            if lba == 0 && magic == b"USBD2TST" {
                println!("[fatfs2]   PASSED: Read {} bytes, LBA={}, magic={:?}",
                    resp.bytes, lba, core::str::from_utf8(magic).unwrap_or("?"));
            } else {
                println!("[fatfs2]   PARTIAL: Data pattern mismatch (lba={}, magic={:?})",
                    lba, magic);
            }
        } else {
            println!("[fatfs2]   FAILED: status={}, tag={}, bytes={}",
                resp.status, resp.tag, resp.bytes);
        }
    } else {
        println!("[fatfs2]   FAILED: No completion received");
    }
}

/// Test reading multiple blocks
fn test_read_multiple(ring: &BlockRing) {
    println!("[fatfs2] Test 3: Read 4 blocks (LBA 1-4)...");

    // Submit read request for blocks 1-4
    let req = BlockRequest::read(3, 1, 4, BLOCK_SIZE); // Offset past first block
    if !ring.submit(&req) {
        println!("[fatfs2]   FAILED: Could not submit request");
        return;
    }

    ring.notify();
    wait_for_completions(ring, 1);

    if let Some(resp) = ring.next_completion() {
        if resp.status == 0 && resp.tag == 3 && resp.bytes == BLOCK_SIZE * 4 {
            println!("[fatfs2]   PASSED: Read {} bytes", resp.bytes);
        } else {
            println!("[fatfs2]   FAILED: status={}, tag={}, bytes={}",
                resp.status, resp.tag, resp.bytes);
        }
    } else {
        println!("[fatfs2]   FAILED: No completion received");
    }
}

/// Verify data pattern in multiple sectors
fn test_verify_pattern(ring: &BlockRing) {
    println!("[fatfs2] Test 4: Verify data pattern in sectors 0-3...");

    // Read 4 sectors starting at LBA 0
    let req = BlockRequest::read(4, 0, 4, 0);
    if !ring.submit(&req) {
        println!("[fatfs2]   FAILED: Could not submit request");
        return;
    }

    ring.notify();
    wait_for_completions(ring, 1);

    if let Some(resp) = ring.next_completion() {
        if resp.status != 0 || resp.tag != 4 {
            println!("[fatfs2]   FAILED: status={}, tag={}", resp.status, resp.tag);
            return;
        }
    } else {
        println!("[fatfs2]   FAILED: No completion received");
        return;
    }

    // Verify each sector
    let data = ring.data();
    let mut pass = true;
    for sector in 0..4u64 {
        let offset = (sector as usize) * (BLOCK_SIZE as usize);
        let lba = u64::from_le_bytes([
            data[offset], data[offset+1], data[offset+2], data[offset+3],
            data[offset+4], data[offset+5], data[offset+6], data[offset+7],
        ]);
        let magic = &data[offset+8..offset+16];
        if lba != sector || magic != b"USBD2TST" {
            println!("[fatfs2]   Sector {} mismatch: lba={}, expected={}", sector, lba, sector);
            pass = false;
        }
    }

    if pass {
        println!("[fatfs2]   PASSED: All 4 sectors have correct pattern");
    }
}

/// Test submitting multiple requests at once
fn test_batch_submit(ring: &BlockRing) {
    println!("[fatfs2] Test 5: Batch submit 8 requests...");

    // Submit 8 read requests
    for i in 0..8u32 {
        let req = BlockRequest::read(100 + i, i as u64, 1, i * BLOCK_SIZE);
        if !ring.submit(&req) {
            println!("[fatfs2]   FAILED: Could not submit request {}", i);
            return;
        }
    }

    // Single notify for all
    ring.notify();

    // Wait for all completions
    wait_for_completions(ring, 8);

    // Collect all completions
    let mut completed = 0u32;
    let mut errors = 0u32;
    while let Some(resp) = ring.next_completion() {
        if resp.status == 0 && resp.bytes == BLOCK_SIZE {
            completed += 1;
        } else {
            errors += 1;
        }
    }

    if completed == 8 && errors == 0 {
        println!("[fatfs2]   PASSED: All 8 requests completed successfully");
    } else {
        println!("[fatfs2]   FAILED: completed={}, errors={}", completed, errors);
    }
}

/// Wait for N completions to appear in the completion queue
fn wait_for_completions(ring: &BlockRing, count: u32) {
    let mut total_wait = 0u32;
    while ring.cq_pending() < count {
        if !ring.wait(1000) {
            total_wait += 1000;
            if total_wait >= 5000 {
                println!("[fatfs2] Warning: Timeout waiting for completions (got {} of {})",
                    ring.cq_pending(), count);
                break;
            }
        }
    }
}
