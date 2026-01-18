# BPI-R4 Microkernel Documentation

A reliability-focused microkernel for the Banana Pi BPI-R4 (MT7988A SoC).

## Quick Links

| Document | Description |
|----------|-------------|
| [CLAUDE.md](../CLAUDE.md) | Main project reference - start here |
| [PRINCIPLES.md](PRINCIPLES.md) | Core design philosophy and hard rules |
| [architecture/](architecture/) | System architecture documentation |
| [decisions/ADR.md](decisions/ADR.md) | Architecture Decision Records |

## Documentation Structure

```
docs/
├── README.md              ← You are here
├── PRINCIPLES.md          ← Design philosophy (MUST READ)
│
├── architecture/          ← System design
│   ├── OVERVIEW.md        ← High-level architecture
│   ├── STATE-MACHINES.md  ← Bus, device, driver states
│   └── MICROKERNEL.md     ← Minimal kernel philosophy
│
├── subsystems/            ← Kernel subsystem details
│   ├── LOGGING.md         ← Logging framework
│   ├── TRACING.md         ← Tracing framework
│   ├── SECURITY.md        ← Capability-based security model
│   └── EVENTS.md          ← Unified event system (kevent API)
│
├── drivers/               ← Driver development
│   ├── USB.md             ← USB/xHCI implementation
│   └── mt7996-linux/      ← MT7996 WiFi reference
│
├── decisions/             ← Decision records
│   └── ADR.md             ← Architecture decisions
│
├── roadmap/               ← Future plans
│   ├── POLICY-DAEMONS.md  ← fsd, netd design
│   └── FUTURE-CAPABILITIES.md ← Capsicum, pledge, etc.
│
└── reference/             ← API reference
    └── ...
```

## Core Concepts

### What Is This?

A **microkernel** designed for embedded networking hardware:
- **Minimal kernel** - IPC, memory, scheduling, bus safety only
- **Userspace drivers** - All device management in userspace
- **State machines everywhere** - Explicit states at every boundary
- **Self-healing** - Faults trigger escalating recovery, not crashes

### Key Design Decisions

1. **No kernel VFS** - Filesystem policy in userspace (fsd)
2. **No kernel device registry** - devd is single source of truth
3. **Event-driven, never polling** - Use kernel events, not busy-wait
4. **Capability-based security** - Processes have explicit permissions
5. **Non-cacheable DMA** - Cache coherency without explicit flushes

See [PRINCIPLES.md](PRINCIPLES.md) for complete design philosophy.

## Getting Started

### Build

```bash
# Full build (kernel + userspace + initrd)
./build.sh

# Build with WiFi firmware embedded
./build.sh --with-firmware

# Build with self-tests enabled
./build.sh --test
```

### Load via U-Boot

```
1. loady 0x46000000
2. (send kernel.bin via Xmodem)
3. go 0x46000000
```

## Hardware

- **Board**: Banana Pi BPI-R4
- **SoC**: MediaTek MT7988A
- **CPU**: ARM Cortex-A73 (AArch64)
- **PCIe**: 4 ports (MT7996 WiFi on pcie0/pcie1)
- **USB**: 2 xHCI controllers (SSUSB0, SSUSB1)
