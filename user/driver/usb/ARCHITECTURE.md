# USB Driver Architecture

## Zero-Copy DMA Ring Buffer System

This document describes the ring buffer architecture used for high-performance
communication between userspace drivers and USB hardware.

## Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        User Applications                                 │
│                     open("disk0:/file.txt")                              │
├─────────────────────────────────────────────────────────────────────────┤
│                          fatfs Driver                                    │
│  - FAT12/16/32 filesystem parser                                        │
│  - Uses BlockClient for block access                                    │
│  - Reads/writes directly from shared DMA buffer                         │
├────────────────────────────┬────────────────────────────────────────────┤
│      Shared Memory Region  │  (physically contiguous, DMA-capable)      │
│  ┌─────────────────────────┴────────────────────────────────────────┐   │
│  │  RingHeader (64 bytes)                                            │   │
│  │    sq_head, sq_tail (submission queue indices)                    │   │
│  │    cq_head, cq_tail (completion queue indices)                    │   │
│  ├──────────────────────────────────────────────────────────────────┤   │
│  │  Submission Queue [64 entries]                                    │   │
│  │    BlockRequest { cmd, tag, lba, count, buf_offset }              │   │
│  ├──────────────────────────────────────────────────────────────────┤   │
│  │  Completion Queue [64 entries]                                    │   │
│  │    BlockResponse { status, tag, bytes, block_info }               │   │
│  ├──────────────────────────────────────────────────────────────────┤   │
│  │  Data Buffer (1MB)                                                │   │
│  │    DMA target for USB transfers                                   │   │
│  │    Physical address passed directly to xHCI TRBs                  │   │
│  └──────────────────────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────────────┤
│                           usbd Driver                                    │
│  - USB enumeration, hub support                                         │
│  - Block request processing                                             │
│  - Programs xHCI with shared buffer physical address                    │
├─────────────────────────────────────────────────────────────────────────┤
│                      xHCI Host Controller                                │
│  - TRB rings (command, transfer, event)                                 │
│  - DMA engine reads/writes directly to shared buffer                    │
├─────────────────────────────────────────────────────────────────────────┤
│                         USB Hardware                                     │
│  - Mass storage device                                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Kernel Primitives

The kernel provides shared memory primitives for DMA-capable buffers:

### Syscalls

| Syscall | Description |
|---------|-------------|
| `shmem_create(size, &vaddr, &paddr)` | Allocate physically contiguous memory, return virtual and physical addresses |
| `shmem_map(id, &vaddr, &paddr)` | Map existing region into caller's address space |
| `shmem_allow(id, peer_pid)` | Grant access permission to another process |
| `shmem_wait(id, timeout_ms)` | Block until notification |
| `shmem_notify(id)` | Wake all processes waiting on this region |
| `shmem_destroy(id)` | Free the shared memory region |

### Key Properties

- **Physically Contiguous**: Memory is allocated via `pmm::alloc_contiguous()`
- **DMA-Capable**: Physical address can be programmed into hardware DMA descriptors
- **Multi-Process**: Same physical memory mapped into multiple address spaces
- **Synchronized**: wait/notify for blocking communication

## Ring Buffer Protocol

### userlib::ring::Ring<S, C>

Generic ring buffer built on shared memory:

```rust
pub struct Ring<S, C> {
    shmem_id: u32,
    base: *mut u8,      // Virtual address (for CPU access)
    phys: u64,          // Physical address (for DMA)
    size: usize,
    is_owner: bool,
}
```

### BlockRing

Specialization for block devices:

```rust
type BlockRing = Ring<BlockRequest, BlockResponse>;
```

### Request/Response Format

```rust
// Submission Queue Entry
struct BlockRequest {
    cmd: u8,           // CMD_READ=0, CMD_WRITE=1, CMD_INFO=2
    tag: u32,          // Request identifier (for matching responses)
    lba: u64,          // Logical Block Address
    count: u32,        // Number of blocks
    buf_offset: u32,   // Offset into data buffer
}

// Completion Queue Entry
struct BlockResponse {
    status: i32,       // 0 = success, negative = error
    tag: u32,          // Matching request tag
    bytes: u32,        // Bytes transferred
    block_size: u32,   // For CMD_INFO response
    block_count: u64,  // For CMD_INFO response
}
```

## Zero-Copy Data Flow

### Read Operation

```
1. fatfs: submit BlockRequest { cmd=READ, lba=100, count=8, buf_offset=0 }
         ring.submit(&req); ring.notify();

2. usbd:  req = ring.next_request()
          target_phys = ring.data_phys() + buf_offset

3. usbd:  Program xHCI TRB with target_phys
          trb.param = target_phys  // DMA destination
          ring_doorbell()

4. xHCI:  DMA from USB device directly into shared buffer
          (hardware writes to physical address)

5. usbd:  ring.complete(&BlockResponse { status=0, bytes=4096 })
          ring.notify()

6. fatfs: resp = ring.next_completion()
          invalidate_buffer(ring.data())  // CPU cache sync
          data = ring.data()  // Zero-copy access!
```

### Key Points

- **No memory copies**: Data goes USB → shared buffer → application
- **Single DMA target**: xHCI TRB points directly to shared buffer
- **Cache coherence**: Client invalidates cache after DMA completes
- **Physical addresses**: Ring provides `data_phys()` for hardware programming

## Connection Handshake

Initial connection uses a minimal channel exchange:

```
1. Client: port_connect("usb")      → channel
2. Client: send(channel, my_pid)
3. Server: recv(channel) → client_pid
4. Server: BlockRing::create(64, 1MB)
5. Server: ring.allow(client_pid)
6. Server: send(channel, shmem_id)
7. Client: recv(channel) → shmem_id
8. Client: BlockRing::map(shmem_id)
9. (channel no longer needed - all communication via ring)
```

## Module Structure

```
user/driver/usb/src/
├── lib.rs              # Module exports
├── block_client.rs     # BlockClient - ring buffer client for block access
├── ring.rs             # xHCI-specific rings (TRBs, event ring)
├── transfer.rs         # Cache maintenance, TRB builders
├── trb.rs              # TRB structures
└── ...

user/userlib/src/
├── ring.rs             # Generic Ring<S,C> on shared memory
│   ├── Ring::create()  # Server creates ring
│   ├── Ring::map()     # Client maps ring
│   ├── submit/complete # Queue operations
│   └── wait/notify     # Synchronization
└── syscall.rs          # shmem_* syscall wrappers

src/
├── shmem.rs            # Kernel shared memory implementation
└── pmm.rs              # Physical memory allocator
```

## Future: NVMe, PCIe

The same infrastructure supports other DMA-based drivers:

```rust
// NVMe would use:
type NvmeRing = Ring<NvmeSubmissionEntry, NvmeCompletionEntry>;

// Same pattern:
// 1. shmem_create() for DMA buffers
// 2. Ring<S,C> for command submission
// 3. Program hardware with data_phys()
```

The kernel primitives (shmem_*) and generic ring (Ring<S,C>) are protocol-agnostic.
Protocol-specific code (TRB format, NVMe SQE format) stays in respective drivers.
