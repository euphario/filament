# Hardware Discovery and Driver Configuration

## Overview

Hardware discovery and driver configuration are two distinct concerns handled
by two separate mechanisms:

1. **probed** — an ephemeral boot-time process that reads platform tables
   (FDT on ARM64, ACPI on x86) and registers buses with the kernel. It runs
   once, exits, and the kernel permanently locks bus creation.

2. **devd config** — a persistent key/value configuration file that devd
   reads at startup. It holds per-driver settings and spawn rule overrides.
   devd injects config values to drivers during initialization.

The kernel itself contains no platform-specific device knowledge beyond the
bare minimum needed to boot (UART and GIC addresses, selected at compile
time via `--platform`).

## Boot Sequence

```
Kernel boots
    │
    ├─ Minimal platform init:
    │     UART (compile-time base + type)
    │     GIC  (compile-time dist + redist base)
    │     Timer (ARM generic timer, universal)
    │     Page tables (linker script)
    │
    ├─ Map FDT/ACPI blob as platform shmem
    │
    ├─ Spawn probed (first user process, CAP_BUS_CREATE)
    │     ├─ map() platform shmem → read FDT or ACPI
    │     ├─ For each device node:
    │     │     open(Bus, CREATE, &BusCreateInfo { type, base, size, irq, ... })
    │     └─ exit()
    │
    ├─ lock_bus_creation()          ← permanent, one-way
    │
    ├─ Spawn devd (reuses probed's slot)
    │     ├─ Read config file from initrd
    │     ├─ bus_list() → all buses probed registered
    │     ├─ Match buses against rules (compiled + config overrides)
    │     ├─ Spawn drivers, inject per-driver config
    │     └─ Normal operation
```

## probed — Hardware Discovery Process

### Role

probed is a short-lived, privileged process that translates platform-specific
hardware descriptions into the kernel's platform-agnostic bus model. It is the
only process that ever has `CAP_BUS_CREATE`. After it exits, no process can
create buses — the syscall path is permanently dead.

### Platform Table Access

The kernel maps the FDT or ACPI blob (already in physical memory from the
bootloader) as a shmem object. probed maps it via the standard `map()` syscall.

- **ARM64**: U-Boot passes DTB address in x0. Kernel preserves it and creates
  a shmem mapping.
- **x86**: RSDP is at a well-known physical address (EFI system table or
  EBDA scan). Kernel creates a shmem mapping covering the ACPI tables.

### Bus Creation Interface

probed creates buses via the unified syscall interface:

```rust
pub struct BusCreateInfo {
    pub bus_type: u8,       // PCIE, USB, ETHERNET, UART, KLOG, ...
    pub bus_index: u8,      // Index within type (0 for pcie0, 1 for pcie1)
    pub base_addr: u64,     // MMIO base address
    pub size: u64,          // MMIO region size
    pub irq: u32,           // Primary interrupt number
    pub compatible: [u8; 64], // Compatible string (e.g., "mediatek,mt7988-pcie")
}

// probed calls:
let handle = open(ObjectType::Bus, flags::CREATE, &info)?;
close(handle)?;
```

The kernel allocates a `BusController`, creates the port path
(`/kernel/bus/pcie0`), and adds to the bus list. The bus starts in
`Resetting` state (hardware not yet initialized by a driver).

### Bus Creation Lock

After probed exits, the kernel permanently locks bus creation:

```rust
static BUS_CREATION_LOCKED: AtomicBool = AtomicBool::new(false);

// Called once, after probed exits
fn lock_bus_creation() {
    BUS_CREATION_LOCKED.store(true, Ordering::Release);
}

// In bus_create syscall path
fn sys_bus_create(...) -> Result<...> {
    if BUS_CREATION_LOCKED.load(Ordering::Acquire) {
        return Err(SyscallError::NotPermitted);
    }
    // ... create bus
}
```

This is not a capability check — it's a physical gate. Even if a process
somehow acquires `CAP_BUS_CREATE`, the syscall returns NotPermitted. The
hardware topology is frozen after boot.

### probed Variants

Different platforms use different probed binaries. The correct one is
included in the initrd at build time.

| Binary | Platform | Parses | Discovers |
|--------|----------|--------|-----------|
| `probed-fdt` | ARM64 | Flattened Device Tree | PCIe, USB, Ethernet, UART, I2C, SPI, ... |
| `probed-acpi` | x86_64 | ACPI tables (MCFG, MADT, SPCR, ...) | PCIe ECAM, IOAPIC, serial, ... |

### FDT Discovery Logic (probed-fdt)

probed-fdt walks FDT nodes and matches compatible strings to bus types:

```rust
fn discover(fdt: &Fdt) {
    for node in fdt.all_nodes() {
        let compat = node.compatible();
        let reg = node.reg();     // (base, size)
        let irq = node.interrupts(); // interrupt specifier

        let bus_type = match_compatible(compat);
        if let Some(bus_type) = bus_type {
            bus_create(bus_type, reg.base, reg.size, irq);
        }
    }
}

fn match_compatible(compat: &[&str]) -> Option<u8> {
    for c in compat {
        if c.contains("-pcie") || c.contains("-pci") { return Some(PCIE); }
        if c.contains("-xhci") || c.contains("-usb")  { return Some(USB); }
        if c.contains("-eth")  || c.contains("-gmac")  { return Some(ETHERNET); }
        if c.contains("-uart") || c.contains("-serial") { return Some(UART); }
        // ...
    }
    None
}
```

The compatible-string matching is intentionally loose — probed registers
buses broadly, and devd's rules decide which drivers to spawn. probed
doesn't need to know what driver handles a device.

### ACPI Discovery Logic (probed-acpi)

probed-acpi reads specific ACPI tables:

| Table | Discovers |
|-------|-----------|
| MCFG | PCIe ECAM base addresses and segment groups |
| MADT | Interrupt controllers (IOAPIC, GIC) |
| SPCR | Serial console (UART base, type, baud) |
| HPET | High-precision event timer |
| DMAR | IOMMU / DMA remapping units |

## devd Config — Persistent Driver Configuration

### Role

devd reads a configuration file at startup that provides:

1. **Per-driver key/value settings** — injected to drivers during init
2. **Rule overrides** — extend or modify the compiled PORT_RULES table

### Config File Location

The config file is embedded in the initrd at build time. devd reads it
from a known path in the initrd filesystem (when initrd filesystem support
is implemented), or from a config partition on persistent storage.

For now, the simplest approach: the config is a second embedded blob in
the initrd, at a known offset, with a small header.

### Config File Format

Simple INI-style sections with key = value pairs:

```ini
# Hardware-discovered devices get drivers from PORT_RULES.
# This file provides per-driver configuration and rule overrides.

[ipd]
net.dhcp = off
net.ip = 10.0.0.1
net.mask = 255.255.255.0
net.gateway = 10.0.0.254
net.dns = 8.8.8.8

[ipd.br1]
# Per-instance config (WAN bridge group)
net.dhcp = on

[switchd]
group.br0 = 0,1,2,3
group.br1 = 4

[consoled]
baud = 115200

[rules]
# Additional spawn rules (extend PORT_RULES)
# Format: class[.subclass] = driver
# network.bridge_group = ipd
# storage.nvme = nvmed
```

### Config Injection

When a driver starts and calls `config_get(key)` via the bus framework,
devd looks up the key in its per-driver config map:

```
Driver init flow:

1. devd spawns ipd for br0
2. ipd calls driver_main() → init()
3. ipd init() calls ctx.config_get("net.dhcp")
4. Bus runtime sends config_get request to devd
5. devd looks up [ipd] section → net.dhcp = off
6. devd responds with "off"
7. ipd configures accordingly
```

devd holds config in memory as a simple map:

```rust
struct DriverConfig {
    // driver_name → { key → value }
    entries: BTreeMap<&str, BTreeMap<&str, &str>>,
}
```

### Per-Instance Config

When multiple instances of a driver exist (e.g., ipd per bridge group),
devd matches config sections by specificity:

1. `[ipd.br1]` — matches ipd spawned for the br1 port (most specific)
2. `[ipd]` — matches all ipd instances (default)

The instance name comes from the port name that triggered the spawn.

### Runtime Config Changes

`devc` commands modify running driver state via the existing `command()`
bus framework path:

```
devc ipd set net.dhcp off
```

To persist changes, devd writes back to the config file (if on a writable
partition) or logs a warning that the change is runtime-only (if in initrd).

## Kernel Changes

### What Gets Added

| Change | Description |
|--------|-------------|
| `open(Bus, CREATE)` | New syscall path for bus creation |
| `BUS_CREATION_LOCKED` | One-way atomic bool, checked in bus_create |
| `CAP_BUS_CREATE` | New capability bit for probed |
| Platform shmem | Map FDT/ACPI blob as shmem for probed |
| Two-phase boot | Spawn probed first, then devd after lock |

### What Gets Deleted

| File | Current Purpose | After |
|------|----------------|-------|
| `src/kernel/bus/config.rs` | BusConfig structs, PLATFORMS array, compatible matching | **Deleted** |
| `src/platform/mt7988/bus.rs` | `register_buses()`, hardcoded PCIe/USB/Eth addresses | **Deleted** |
| `src/platform/qemu_virt/bus.rs` | `register_buses()`, ECAM config | **Deleted** |
| `src/kernel/fdt.rs` | FDT parsing for platform selection | **Simplified** to just mapping the blob |

### What Stays (Minimal Platform Layer)

Each platform module retains only boot-critical constants:

```rust
// src/platform/qemu_virt/mod.rs — everything the kernel needs to boot
pub const UART_BASE: usize = 0x0900_0000;
pub const UART_TYPE: UartType = UartType::PL011;
pub const GIC_DIST: usize = 0x0800_0000;
pub const GIC_REDIST: usize = 0x080A_0000;
```

```rust
// src/platform/mt7988/mod.rs — everything the kernel needs to boot
pub const UART_BASE: usize = 0x1100_0000;
pub const UART_TYPE: UartType = UartType::Uart16550;
pub const GIC_DIST: usize = 0x0C000000;  // if applicable
pub const GIC_REDIST: usize = 0x0C080000;
```

These can be compile-time constants (`--platform` flag), environment
variables during build, or defaults that cover common boards.

## Security Model

```
Capability           │ probed │ devd │ drivers │ shell
─────────────────────┼────────┼──────┼─────────┼──────
CAP_BUS_CREATE       │   Y    │  N   │    N    │   N
CAP_BUS_DRIVER       │   N    │  Y   │    Y    │   N
CAP_DEVICE_DRIVER    │   N    │  Y   │    Y    │   N
CAP_USER_DEFAULT     │   N    │  N   │    N    │   Y
```

After probed exits and `lock_bus_creation()` is called:

- `CAP_BUS_CREATE` is **permanently gated** at the syscall level
- No process can create buses, regardless of capabilities
- The hardware topology is immutable for the lifetime of the system
- devd can query buses (`bus_list`) but never create them

## Key Design Decisions

### Why a separate probed process instead of kernel FDT/ACPI parsing?

The kernel stays platform-agnostic. It doesn't need FDT or ACPI parsers,
compatible-string databases, or device-tree binding knowledge. Different
platforms use different probed binaries (probed-fdt vs probed-acpi) with
the same kernel. The kernel's only job is to provide the bus_create
syscall and manage the resulting bus table.

### Why ephemeral instead of a daemon?

probed has no ongoing role. Hardware topology doesn't change at runtime
(no hot-plug at the bus level). Making it ephemeral means zero resident
memory cost and a smaller attack surface — the privileged discovery code
exists in memory only during the brief boot window.

### Why lock bus creation permanently?

If an attacker compromises a driver or devd, they cannot create fake buses
to inject malicious drivers into the system. The hardware topology is
frozen after boot. This is defense in depth — even if capability checks
are bypassed, the one-way lock prevents bus creation.

### Why config in devd, not in the kernel?

Configuration is policy. The kernel provides mechanisms (buses, ports,
IPC). devd provides policy (which drivers to spawn, how to configure
them). Keeping config in devd means the kernel never needs to parse config
files or store key/value pairs. It also means config changes don't require
kernel modifications.

### Why not pass config through probed?

probed speaks hardware (FDT/ACPI). Config speaks policy (DHCP on/off,
IP addresses, bridge groups). These are different concerns with different
lifetimes — hardware is discovered once, config may be modified at runtime.
Mixing them would couple hardware discovery to user preferences.

## Future Considerations

### Hot-Plug (USB, PCIe)

Bus-level topology is fixed, but device-level hot-plug (USB device
inserted, PCIe hot-plug) is handled by the bus driver (usbd, pcied),
not by creating new kernel buses. The bus driver detects the new device
and registers a new port with devd, which triggers rules as usual.

### Network Boot Config

For diskless/network-booted systems, the config file could be fetched
via DHCP/TFTP before devd reads it. This requires a minimal network
stack in the boot path — either in probed (extends its role) or in a
second ephemeral process between probed and devd.

### Config Partition

For persistent runtime config changes, devd can read/write a small FAT
partition (via fatfsd). On first boot, if no config partition exists,
devd uses compiled-in defaults. Changes via `devc set` are written back
to the partition.

### Multiple FDT Overlays

Some boards use FDT overlays for expansion cards or configuration. probed
could apply overlays before discovery, or the bootloader could merge them
before passing the final DTB to the kernel.
