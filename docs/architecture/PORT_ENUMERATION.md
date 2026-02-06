# Port Enumeration Architecture

## Overview

This document describes a unified interface for port/device enumeration that works across all levels of the system - from kernel hardware discovery to driver-level sub-device enumeration.

## Problem Statement

Currently, device information flows through multiple disconnected mechanisms:

1. **Kernel** enumerates hardware but only exposes raw resources (MMIO, IRQ)
2. **Drivers** encode semantic info in port name suffixes (`:xhci`, `:msc`, `:fat`)
3. **Devd** pattern-matches on strings to decide which driver to spawn

This creates several issues:
- No structured data for devd to make intelligent decisions
- Naming conventions are fragile and implicit
- No way to query device capabilities before attaching a driver
- Different code paths for kernel vs driver enumeration

## Design Goals

1. **Unified interface** - Same structure for kernel and driver enumeration
2. **Structured matching** - Devd rules match on class/subclass, not strings
3. **Hierarchical** - Parent-child relationships are explicit
4. **Extensible** - Metadata field for domain-specific info
5. **Capability-aware** - Ports declare what they can do (DMA, IRQ, etc.)

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                              devd                                   │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────────────┐  │
│  │ Rule Engine │◄───│ PortRegistry │◄───│ PortInfo from kernel  │  │
│  │             │    │              │    │ PortInfo from drivers │  │
│  └──────┬──────┘    └──────────────┘    └───────────────────────┘  │
│         │                                                           │
│         ▼                                                           │
│  Match: class=Block, subclass=FAT32 → spawn fatfsd                 │
│  Match: class=Usb, subclass=xHCI → spawn usbd                      │
│  Match: class=Block, subclass=* → spawn partd (default)            │
└─────────────────────────────────────────────────────────────────────┘
         ▲                              ▲
         │                              │
    PortInfo                       PortInfo
         │                              │
┌────────┴────────┐            ┌────────┴────────┐
│     Kernel      │            │     Drivers     │
│                 │            │                 │
│ PCIe enumerate  │            │ partd: parts    │
│ USB controllers │            │ usbd: devices   │
│ Platform devs   │            │ ethd: switch    │
└─────────────────┘            └─────────────────┘
```

## Data Structures

### PortInfo

The core enumeration record, used by both kernel and drivers:

```rust
/// Unified port enumeration record
/// Lives in abi crate (shared kernel/userspace)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PortInfo {
    /// Port name (becomes the IPC port name)
    pub name: [u8; 32],
    pub name_len: u8,

    /// What kind of port is this?
    pub port_class: PortClass,

    /// Subclass for finer matching
    pub port_subclass: u16,

    /// Parent port name (empty = root level)
    pub parent: [u8; 32],
    pub parent_len: u8,

    /// Hardware identifiers (0 = not applicable)
    pub vendor_id: u16,
    pub device_id: u16,

    /// Capability flags
    pub caps: u16,  // PortCaps bitflags

    /// Opaque metadata for driver-specific info
    pub metadata: [u8; 32],
    pub metadata_len: u8,

    /// Reserved for future use
    pub _reserved: [u8; 3],
}
```

### PortClass

High-level device classification:

```rust
#[repr(u16)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PortClass {
    Unknown = 0,

    // Storage
    Block = 1,              // Disk, partition, ramdisk
    StorageController = 2,  // NVMe, AHCI, SCSI controller

    // Network
    Network = 3,            // Ethernet, WiFi, switch port

    // Bus controllers
    Usb = 4,                // USB host controller or device
    Pcie = 5,               // PCIe root port or endpoint

    // Filesystems
    Filesystem = 6,         // Mounted filesystem

    // Platform
    Console = 7,            // TTY, serial port
    Gpio = 8,               // GPIO bank
    I2c = 9,                // I2C bus
    Spi = 10,               // SPI bus

    // Services
    Service = 11,           // Generic service port
}
```

### PortSubclass

Subclass values are class-specific. Some examples:

```rust
pub mod subclass {
    // Block subclasses (partition types from MBR/GPT)
    pub const BLOCK_RAW: u16 = 0x00;      // Unpartitioned disk
    pub const BLOCK_FAT12: u16 = 0x01;
    pub const BLOCK_FAT16: u16 = 0x06;
    pub const BLOCK_FAT32: u16 = 0x0b;
    pub const BLOCK_FAT32_LBA: u16 = 0x0c;
    pub const BLOCK_LINUX: u16 = 0x83;
    pub const BLOCK_GPT: u16 = 0xee;

    // USB subclasses (from interface class)
    pub const USB_XHCI: u16 = 0x30;
    pub const USB_HUB: u16 = 0x09;
    pub const USB_MSC: u16 = 0x08;
    pub const USB_HID: u16 = 0x03;

    // Network subclasses
    pub const NET_ETHERNET: u16 = 0x00;
    pub const NET_WIFI: u16 = 0x01;
    pub const NET_SWITCH_PORT: u16 = 0x02;

    // Storage controller subclasses
    pub const STORAGE_NVME: u16 = 0x02;
    pub const STORAGE_AHCI: u16 = 0x06;
}
```

### PortCaps

Capability bitflags:

```rust
pub mod port_caps {
    pub const DMA: u16       = 1 << 0;   // Can do DMA
    pub const IRQ: u16       = 1 << 1;   // Has interrupt
    pub const MMIO: u16      = 1 << 2;   // Has MMIO region
    pub const READONLY: u16  = 1 << 3;   // Read-only device
    pub const REMOVABLE: u16 = 1 << 4;   // Hot-pluggable
    pub const INTERNAL: u16  = 1 << 5;   // Not user-facing
    pub const BOOTABLE: u16  = 1 << 6;   // Can boot from
}
```

## Usage Examples

### Kernel: PCIe Controller Enumeration

```rust
// Kernel discovers xHCI controller at BDF 00:14.0
let info = PortInfo {
    name: *b"00:14.0:usb\0...",
    name_len: 11,
    port_class: PortClass::Usb,
    port_subclass: subclass::USB_XHCI,
    parent: *b"pcie0:\0...",
    parent_len: 6,
    vendor_id: 0x8086,
    device_id: 0xa0ed,
    caps: port_caps::DMA | port_caps::IRQ | port_caps::MMIO,
    ..
};
// Kernel registers port, devd receives PortInfo
```

### Driver: partd Enumerating Partitions

```rust
// partd reads MBR, finds FAT32 partition at index 0
let info = PortInfo {
    name: *b"0:fat\0...",
    name_len: 5,
    port_class: PortClass::Block,
    port_subclass: subclass::BLOCK_FAT32,
    parent: *b"disk0:\0...",
    parent_len: 6,
    vendor_id: 0,
    device_id: 0,
    caps: port_caps::DMA,
    ..
};
ctx.register_port(&info);
```

### Driver: ethd Enumerating Switch Ports

```rust
// ethd discovers MT7531 switch port 2
let info = PortInfo {
    name: *b"sw2:\0...",
    name_len: 4,
    port_class: PortClass::Network,
    port_subclass: subclass::NET_SWITCH_PORT,
    parent: *b"eth0:\0...",
    parent_len: 5,
    vendor_id: 0x4d54,  // MediaTek
    device_id: 0x7531,
    caps: port_caps::INTERNAL,
    metadata: [2, 0, ...],  // port number in metadata
    ..
};
ctx.register_port(&info);
```

### Driver: usbd Enumerating USB Devices

```rust
// usbd discovers mass storage device
let info = PortInfo {
    name: *b"msc0:\0...",
    name_len: 5,
    port_class: PortClass::Block,
    port_subclass: subclass::USB_MSC,
    parent: *b"00:14.0:usb\0...",
    parent_len: 11,
    vendor_id: 0x0781,  // SanDisk
    device_id: 0x5567,
    caps: port_caps::DMA | port_caps::REMOVABLE,
    ..
};
ctx.register_port(&info);
```

## Devd Rule Matching

Rules match on structured fields instead of string patterns:

```rust
pub struct Rule {
    /// Required port class (None = any)
    pub match_class: Option<PortClass>,

    /// Required subclass (None = any within class)
    pub match_subclass: Option<u16>,

    /// Required parent class (None = any)
    pub match_parent_class: Option<PortClass>,

    /// Required capabilities (all must be present)
    pub require_caps: u16,

    /// Driver to spawn
    pub driver: &'static str,

    /// Capabilities to grant spawned driver
    pub grant_caps: u64,
}

// Example rules
static RULES: &[Rule] = &[
    // Any USB controller → usbd
    Rule {
        match_class: Some(PortClass::Usb),
        match_subclass: Some(subclass::USB_XHCI),
        driver: "usbd",
        ..
    },

    // FAT partition → fatfsd
    Rule {
        match_class: Some(PortClass::Block),
        match_subclass: Some(subclass::BLOCK_FAT32),
        driver: "fatfsd",
        ..
    },

    // Any block device (fallback) → partd
    Rule {
        match_class: Some(PortClass::Block),
        match_subclass: None,  // any
        driver: "partd",
        ..
    },

    // Switch port → no auto-spawn (managed by ethd)
    // (no rule = no driver spawned)
];
```

### Rule Priority

Rules are evaluated in order. More specific rules should come first:

1. Exact class + subclass match
2. Class match with any subclass
3. Fallback rules

## API Changes

### Kernel Port Registration

```rust
// Current (name only)
fn register_port(name: &[u8], owner: TaskId) -> Result<PortId>;

// New (with PortInfo)
fn register_port(info: &PortInfo, owner: TaskId) -> Result<PortId>;
```

### Userlib Bus Framework

```rust
// Current
ctx.register_port(b"disk0:");

// New
ctx.register_port(&PortInfo {
    name: *b"disk0:\0...",
    port_class: PortClass::Block,
    port_subclass: subclass::BLOCK_RAW,
    ..
});
```

### Devd Notification

When a port is registered, devd receives the full `PortInfo` instead of just the name. This allows immediate rule matching without additional queries.

## Migration Path

1. **Phase 1**: Add `PortInfo` to `abi` crate
2. **Phase 2**: Extend kernel port registration to accept optional `PortInfo`
3. **Phase 3**: Update `userlib::bus` to build and pass `PortInfo`
4. **Phase 4**: Update devd to store and match on `PortInfo`
5. **Phase 5**: Convert rules from string patterns to class/subclass matching
6. **Phase 6**: Remove legacy string-based matching

During migration, both mechanisms work - string suffix matching as fallback when `PortInfo` is not provided.

## Benefits

1. **Type-safe matching** - No string parsing, compiler-checked enums
2. **Richer decisions** - Match on capabilities, vendor ID, parent type
3. **Unified model** - Same interface for kernel and driver enumeration
4. **Queryable** - Devd can list devices by class before drivers attach
5. **Extensible** - Metadata field for future needs without ABI changes

## Open Questions

1. **Metadata format** - Should metadata be structured (sub-structs per class) or opaque bytes?
2. **Hot-plug** - How does PortInfo change when device is removed/re-added?
3. **Multiple drivers** - Can multiple drivers attach to one port? (e.g., monitoring + control)
4. **Versioning** - How to handle PortInfo struct changes across kernel versions?
