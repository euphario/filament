# Composable Driver Stack Architecture

Zero-copy ring-based I/O with dynamic layer composition via devd.

## Overview

The driver stack uses **shared memory rings** for bulk data transfer, avoiding IPC payload limits (576 bytes). Data is written once by DMA and read once by the consumer - never copied between layers.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              devd                                       │
│  - Port tree (parent/child relationships)                               │
│  - Rules: "when port X appears, spawn driver Y"                         │
│  - Route queries to port owner                                          │
│  - Does NOT touch data plane                                            │
└─────────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         Port = Ring + Sidechannel                       │
│                                                                         │
│  Provider (lower layer) ←──────────────────→ Consumer (upper layer)     │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      Shared Memory                              │    │
│  │  ┌──────────────────────────────────────────────────────────┐   │    │
│  │  │  Data Pool (actual bytes, DMA-capable, never copied)     │   │    │
│  │  └──────────────────────────────────────────────────────────┘   │    │
│  │  ┌───────────────┐  ┌───────────────┐                           │    │
│  │  │  Data Ring    │  │  Sidechannel  │                           │    │
│  │  │  SQ: ────────→│  │  Query/Ctrl   │                           │    │
│  │  │  CQ: ←────────│  │  chain-pass   │                           │    │
│  │  └───────────────┘  └───────────────┘                           │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

## Key Concepts

### Data Plane (Ring Protocol)

- **Submission Queue (SQ)**: Requests flow down (consumer → provider)
- **Completion Queue (CQ)**: Completions flow up (provider → consumer)
- **Data Pool**: Shared memory buffer for actual data bytes
- **Zero-copy**: SQ/CQ entries contain **offsets** into the pool, not data

### Control Plane (Sidechannel)

- Small messages for queries and control commands
- Passes through layer chain until someone answers
- Used by devd to communicate with drivers
- Examples: BLOCK_SIZE, PARTITION_INFO, USB_SERIAL queries

### Port Hierarchy

Ports form a tree with parent/child relationships:

```
disk0:          (provided by usbd/msc)
  └─ part0:     (provided by partition, parent=disk0:)
  └─ part1:
      └─ fat0:  (provided by fatfs, parent=part1:)
```

## Data Structures

### Ring Header (64 bytes)

```rust
#[repr(C)]
pub struct RingHeader {
    pub magic: u32,
    pub version: u16,
    pub ring_size: u16,        // Number of entries (power of 2)
    pub sq_head: AtomicU32,    // Consumer (provider) advances
    pub sq_tail: AtomicU32,    // Producer (consumer) advances
    pub cq_head: AtomicU32,    // Consumer (consumer) advances
    pub cq_tail: AtomicU32,    // Producer (provider) advances
    pub side_head: AtomicU32,
    pub side_tail: AtomicU32,
    pub pool_offset: u32,
    pub pool_size: u32,
    pub doorbell: AtomicU32,
    _reserved: [u32; 8],
}
```

### I/O Submission Queue Entry (32 bytes)

```rust
#[repr(C)]
pub struct IoSqe {
    pub opcode: u8,
    pub flags: u8,
    pub priority: u8,
    pub _pad: u8,
    pub tag: u32,
    pub lba: u64,
    pub data_offset: u32,   // Offset into pool (NOT the data itself)
    pub data_len: u32,
    pub param: u64,
}
```

### I/O Completion Queue Entry (16 bytes)

```rust
#[repr(C)]
pub struct IoCqe {
    pub status: u16,
    pub flags: u16,
    pub tag: u32,
    pub transferred: u32,
    pub result: u32,
}
```

### Sidechannel Entry (32 bytes)

```rust
#[repr(C)]
pub struct SideEntry {
    pub msg_type: u16,
    pub flags: u16,
    pub tag: u16,
    pub status: u16,         // OK, PASS_DOWN, EOL, ERROR
    pub payload: [u8; 24],
}
```

## Zero-Copy Flow Example

```
fatfs: READ lba=100 from part0:
    │
    │ alloc offset=0x1000 in shared pool
    │ submit IoSqe{READ, lba=100, data_offset=0x1000}
    ▼
part0: recv IoSqe → translate lba
    │ submit IoSqe{READ, lba=100+2048, data_offset=0x1000}  ← SAME offset!
    ▼
disk0: recv IoSqe → translate to SCSI
    │ submit IoSqe{SCSI_READ10, lba=2148, data_offset=0x1000}  ← SAME offset!
    ▼
usbd: recv IoSqe → build TRB
    │ DMA writes directly to shmem+0x1000
    │ complete IoCqe{OK}
    ▼
Completions flow back up...
    ▼
fatfs: read data from pool at 0x1000

Data written ONCE by DMA, read ONCE by fatfs. Never copied between layers.
```

## Sidechannel Query Flow

Queries pass through layers until someone answers:

```
fatfs sends: query(USB_SERIAL)
    │
    ▼
part0: handle_query() → None (don't know) → pass down
    │
    ▼
disk0: handle_query() → None (don't know) → pass down
    │
    ▼
usbd: handle_query() → Some("ABC123") ← I know!
    │
    ▼
Response flows back up through completions
```

## DataPort API

High-level wrapper for ring + sidechannel + pool:

```rust
pub struct DataPort {
    shmem: Shmem,
    role: PortRole,
    // Cached pointers into shared memory
}

pub enum PortRole {
    Provider,  // Receives requests, sends completions (lower layer)
    Consumer,  // Sends requests, receives completions (upper layer)
}

impl DataPort {
    // Creation/connection
    pub fn create(name: &[u8], config: DataPortConfig) -> Result<Self>;
    pub fn connect(name: &[u8]) -> Result<Self>;

    // Data ring operations
    pub fn alloc(&mut self, size: u32) -> Option<u32>;  // Returns pool offset
    pub fn free(&mut self, offset: u32, size: u32);
    pub fn submit(&mut self, sqe: IoSqe) -> bool;       // Consumer → Provider
    pub fn recv(&mut self) -> Option<IoSqe>;            // Provider receives
    pub fn complete(&mut self, cqe: IoCqe) -> bool;     // Provider → Consumer
    pub fn poll_completion(&mut self) -> Option<IoCqe>; // Consumer receives

    // Sidechannel operations
    pub fn query(&mut self, entry: SideEntry) -> bool;
    pub fn poll_query(&mut self) -> Option<SideEntry>;

    // Pool access (zero-copy)
    pub fn pool_slice(&self, offset: u32, len: u32) -> &[u8];
    pub fn pool_slice_mut(&mut self, offset: u32, len: u32) -> &mut [u8];
}
```

## Layer Trait

Standard pattern for composable driver layers:

```rust
pub trait Layer {
    fn provides(&self) -> &[u8];           // Port name this layer provides
    fn consumes(&self) -> Option<&[u8]>;   // Port name consumed (None = hardware)

    fn process_request(&mut self, sqe: IoSqe) -> Option<IoSqe>;
    fn process_completion(&mut self, cqe: IoCqe) -> Option<IoCqe>;
    fn handle_query(&mut self, query: &SideEntry) -> Option<SideEntry>;
    fn tick(&mut self);
}

pub struct ConnectedLayer<L: Layer> {
    layer: L,
    above: Option<DataPort>,  // Upper layer connection
    below: Option<DataPort>,  // Lower layer connection
}
```

## devd Rules System

devd automatically spawns drivers based on port events:

```rust
pub struct Rule {
    pub match_parent_type: Option<PortType>,
    pub match_port_type: Option<PortType>,
    pub driver_binary: &'static str,
    pub pass_port_name: bool,
}

// Built-in rules
pub static RULES: &[Rule] = &[
    // When Block port appears → spawn partition scanner
    Rule { match_port_type: Some(PortType::Block), driver_binary: "partition", .. },
    // When Partition port appears → spawn fatfs
    Rule { match_port_type: Some(PortType::Partition), driver_binary: "fatfs", .. },
];
```

## Implementation Status

| Component | File | Status |
|-----------|------|--------|
| Port hierarchy | `devd/src/ports.rs` | Complete |
| Ring protocol | `userlib/src/ring.rs` | Complete |
| DataPort API | `userlib/src/data_port.rs` | Complete |
| Layer trait | `userlib/src/data_port.rs` | Complete |
| devd rules | `devd/src/rules.rs` | Complete |
| **usbd conversion** | `usbd/src/main.rs` | **TODO** |
| **partition conversion** | `partition/src/main.rs` | **TODO** |
| **fatfs conversion** | `fatfs/src/main.rs` | **TODO** |

## Why Not IPC for Block Data?

The kernel IPC has a `MAX_INLINE_PAYLOAD` of 576 bytes. Block devices need to transfer 512-byte sectors, often multiple at once (4KB+ for performance). Options:

1. **IPC only**: Limited to 1 sector/request, ~10x overhead
2. **IPC + shared memory (current)**: Temporary workaround, still has IPC overhead
3. **Ring protocol (designed)**: Zero-copy, DMA-direct, no IPC in data path

The ring protocol is the correct solution - IPC is only for control plane (devd queries), never for bulk data.
