# BPI-R4 Kernel Project

## Design Principles (MUST FOLLOW)

### Architecture & Portability
- **SMP compatibility always** - All code must be safe for multi-core execution. Use atomic operations, proper locking, per-CPU data structures where needed.
- **Clear separation of concerns** - Each module has a single responsibility. No god objects.
- **Hardware abstraction** - No chip-specific (MT7988A) or board-specific (BPI-R4) code outside `src/platform/` and `src/arch/`. Upper layers use traits.
- **Use traits for abstraction** - Define behavior through traits, implement for specific hardware. Enables testing and portability.

### Code Quality & Security
- **Production hardening mindset** - Write code as if it's going to production. No "good enough for now".
- **Security first** - Capability checks for privileged operations. Validate all user input. No trust across privilege boundaries.
- **No TODOs without agreement** - Don't leave TODO comments without explicit user approval. Either implement fully or document as a known limitation.
- **Document design alongside code** - Design rationale at top of files or in adjacent markdown. Code should be self-documenting but complex decisions need explanation.

### State & Control Flow
- **State machine design** - Model complex behavior as explicit state machines. States, transitions, and events should be clear and documented.
- **No busy-wait loops** - Blocked tasks must WFI or yield. Use kernel timer events for scheduling, not polling loops.
- **Event-driven where possible** - Prefer event queues over polling. Wake on events, not timeouts.

### Scheduling & Concurrency
- **Blocked tasks never spin** - Always WFI or yield to Ready tasks
- **Timer events use kernel tick_count** - Not raw hardware counter. Consistent time base.
- **IRQ handlers set flags only** - Actual work (preemption, task switch) happens at safe points

### Event System
- **Critical events never dropped** - ChildExit, Signal use priority queue
- **Timer events direct to task** - No subscription filtering needed
- **Receiver allowlist for signals** - Opt-in model for inter-process signals

### Driver Supervision (devd)
- **Exponential backoff** - Crash recovery with increasing delays
- **Bus reset before restart** - Clean hardware state for driver restart
- **Kernel timer events** - No userspace polling for scheduling

### Layer Boundaries
```
┌─────────────────────────────────────────┐
│  Userspace (shell, drivers, apps)       │  ← No HW access except via syscalls
├─────────────────────────────────────────┤
│  Kernel services (IPC, events, shmem)   │  ← Platform-agnostic
├─────────────────────────────────────────┤
│  Platform HAL (src/platform/mt7988/)    │  ← SoC-specific, trait implementations
├─────────────────────────────────────────┤
│  Arch (src/arch/aarch64/)               │  ← CPU architecture specific
└─────────────────────────────────────────┘
```

---

## Microkernel Device Architecture

### Principles

The kernel is **minimal** - it provides raw capabilities but does NOT track devices.
Device management lives in userspace with **devd as the single source of truth**.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              KERNEL (minimal)                           │
│                                                                         │
│  Syscalls provided:                                                     │
│  • bus_list()           - Expose buses from DTB (pcie0, usb0, etc.)    │
│  • mmap_device(pa, sz)  - Map physical addr as device MMIO (cap check) │
│  • shmem_create()       - DMA-capable shared memory                    │
│  • irq_claim()          - Claim interrupt (cap check)                  │
│                                                                         │
│  NOT in kernel:                                                         │
│  • No device registry                                                   │
│  • No PCI/USB/etc specific logic                                        │
│  • No driver management                                                 │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                               ▼
┌───────────────────────────────┐   ┌───────────────────────────────────┐
│  devd (device supervisor)     │   │  Bus Drivers (pcied, usbd)        │
│                               │   │                                   │
│  • Single source of truth     │◄──│  • Enumerate devices on bus       │
│  • Receives enumeration       │   │  • Report discoveries to devd     │
│  • Controls driver access     │   │  • Answer queries from drivers    │
│  • Supervises driver lifecycle│   │  • Handle bus-level operations    │
│  • Manages restarts/recovery  │   │                                   │
└───────────────────────────────┘   └───────────────────────────────────┘
                                                    │
                                                    ▼
                                    ┌───────────────────────────────────┐
                                    │  Device Drivers (wifid, nvmed)    │
                                    │                                   │
                                    │  • Query bus driver for device    │
                                    │  • Get BAR/endpoint info via IPC  │
                                    │  • Call mmap_device() directly    │
                                    │  • Report status to devd          │
                                    └───────────────────────────────────┘
```

### Data Flow Example: WiFi Driver

```
1. Kernel boots, reads DTB → knows about pcie0, pcie1, usb0, usb1, etc.

2. devd starts, spawns pcied for PCIe buses

3. pcied:
   • Initializes PCIe MAC, brings up links
   • Enumerates: finds MT7996 at BAR0=0x30200000, size=2MB
   • Registers devices with devd (via IPC)
   • Listens for FindDevice queries

4. wifid2 starts:
   • Connects to pcied
   • Calls FindDevice(vendor=0x14c3, device=0x7990)
   • Gets: {bar0_addr: 0x30200000, bar0_size: 2MB, ...}

5. wifid2 maps device memory:
   • Calls syscall::mmap_device(0x30200000, 2MB)
   • Kernel checks MMIO capability, maps with device attributes
   • Returns virtual address 0x50000000

6. wifid2 accesses registers:
   • Writes to 0x50000000 + offset → hardware access
```

### Syscall: mmap_device

```rust
/// Map device MMIO into process address space
/// Generic - kernel doesn't know what device this is
///
/// phys_addr: Physical address of device memory (BAR, MMIO region)
/// size: Size in bytes to map (max 16MB)
/// Returns: Virtual address on success, negative error on failure
///
/// Requires: MMIO capability
/// Memory attributes: Device-nGnRnE (non-cacheable, non-gathering)
pub fn mmap_device(phys_addr: u64, size: u64) -> Result<u64, i32>
```

### Bus Driver Responsibilities

| Component | Responsibility |
|-----------|---------------|
| **Kernel** | Expose bus list from DTB, provide mmap_device syscall |
| **devd** | Supervise bus drivers, maintain device database |
| **pcied** | Enumerate PCIe, configure BARs, answer FindDevice |
| **usbd** | Enumerate USB, configure endpoints, answer FindDevice |
| **Driver** | Query bus driver for device info, mmap and use device |

### Drivers Needing Updates

| Driver | Status | Required Changes |
|--------|--------|------------------|
| wifid2 | ✅ Updated | Uses mmap_device with BAR from pcied |
| nvmed | ❌ Needs rewrite | Currently uses pci_bar_map, should use mmap_device |
| usbd | ❌ Needs rewrite | Should report devices to devd, answer queries |
| wifid | ❌ Legacy | Uses old mt7996 library, superseded by wifid2 |

### Why This Design?

1. **Kernel stays minimal** - No device-specific code, just raw capabilities
2. **Single source of truth** - devd knows all devices, controls access
3. **Bus drivers are experts** - pcied knows PCI, usbd knows USB
4. **Drivers are independent** - Only need IPC to bus driver + mmap syscall
5. **Easy to add buses** - New bus = new userspace driver, no kernel changes
6. **Supervision works** - devd can restart crashed drivers, reset buses

### Syscalls to Deprecate

With the clean architecture, these PCI-specific syscalls should be removed:

| Syscall | Replacement |
|---------|-------------|
| `pci_bar_map` | `mmap_device` (generic) + get BAR from pcied |
| `pci_enumerate` | Query pcied via IPC |
| `pci_claim` | devd controls access, not kernel |
| `pci_config_read/write` | pcied does this internally via MMIO |

The kernel should only provide:
- `bus_list` - DTB buses
- `mmap_device` - generic MMIO mapping
- `shmem_create` - DMA memory
- `irq_claim` - interrupt handling

---

## Bus & Device State Machine

### Overview

Buses and devices follow explicit state machines tracked by devd.
This enables proper supervision, restart recovery, and status reporting.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           STATE FLOW                                     │
│                                                                         │
│  BUS STATES:                                                            │
│  ┌──────┐  pcied/usbd   ┌────────┐  reset     ┌───────────┐            │
│  │ Safe │ ────claims────▶│ Active │ ──────────▶│ Resetting │            │
│  └──────┘               └────────┘            └─────┬─────┘            │
│     ▲                        ▲                      │                   │
│     │                        └──────────────────────┘                   │
│     │                              reset complete                       │
│     └─────────────────────────────────────────────────                  │
│                    bus driver exits/crashes                             │
│                                                                         │
│  DEVICE STATES:                                                         │
│  ┌────────────┐  driver    ┌───────┐                                   │
│  │ Discovered │ ──claims──▶│ Bound │                                   │
│  └────────────┘            └───────┘                                   │
│        ▲                       │                                        │
│        └───────────────────────┘                                        │
│              driver exits/crashes                                       │
└─────────────────────────────────────────────────────────────────────────┘
```

### Bus States

| State | Meaning | Transitions |
|-------|---------|-------------|
| `Safe` | No driver assigned, bus idle | → Active (bus driver claims) |
| `Active` | Bus driver (pcied/usbd) is managing | → Resetting (reset requested) |
| `Resetting` | Bus hardware being reset | → Active (reset complete) |

### Device States

| State | Meaning | Transitions |
|-------|---------|-------------|
| `Discovered` | Found by bus driver, no device driver | → Bound (driver claims) |
| `Bound` | Device driver is actively using | → Discovered (driver exits) |

### Initialization Sequence

```
1. Kernel boots
   └── DTB parsed → buses exposed via bus_list()

2. devd starts
   ├── Queries bus_list() → sees pcie0, pcie1, usb0, usb1
   ├── Connects to each bus → state = "safe"
   ├── Registers devd port (BEFORE spawning drivers!)
   └── Spawns bus drivers (pcied, usbd)

3. pcied starts
   ├── Connects to devd
   ├── Claims buses: /bus/pcie0, /bus/pcie1 → state = "active"
   ├── Initializes PCIe MAC, brings up links
   ├── Enumerates devices → registers as "discovered"
   │   └── /bus/pcie0/01:00.0 (MT7996) → state = "discovered"
   └── Starts IPC server for FindDevice queries

4. wifid2 starts
   ├── Connects to pcied → FindDevice(vendor=0x14c3)
   ├── Gets BAR info for MT7996
   ├── Connects to devd
   ├── Claims device: /bus/pcie0/01:00.0 → state = "bound"
   └── Maps MMIO via mmap_device(), operates hardware
```

### DevdProtocol Messages

| Request | Purpose | Example |
|---------|---------|---------|
| `ClaimBus` | Bus driver takes ownership | pcied claims `/bus/pcie0` |
| `ClaimDevice` | Device driver takes ownership | wifid2 claims `/bus/pcie0/01:00.0` |
| `Register` | Bus driver reports discovered device | pcied registers MT7996 |
| `Update` | Modify device properties | Change state, add driver name |
| `Query` | Search for devices | Find all network devices |

### Example: hw list Output

```
# After pcied and wifid2 have started:

> hw list
# path,bus,class,vendor:device,state
/bus/pcie0,pcie,bus,1130:0000,active          ← pcied managing
/bus/pcie1,pcie,bus,1131:0000,active          ← pcied managing
/bus/usb0,usb,bus,1119:0000,safe              ← no driver yet
/bus/pcie0/00:00.0,pcie,bridge,14c3:7988,discovered
/bus/pcie0/01:00.0,pcie,network,14c3:7990,bound    ← wifid2 using
/bus/pcie1/01:00.0,pcie,network,14c3:7991,bound    ← wifid2 using
```

### Implementation Files

| File | Role |
|------|------|
| `user/driver/devd/src/main.rs` | State machine, ClaimBus/ClaimDevice handlers |
| `user/userlib/src/ipc/protocols/devd.rs` | DevdRequest::ClaimBus, ClaimDevice |
| `user/driver/pcied/src/main.rs` | Claims buses, registers devices as "discovered" |
| `user/driver/wifid2/src/main.rs` | Claims devices before using |

---

## Quick Reference

### Build Commands

```bash
# Full build (kernel + userspace + initrd)
./build.sh

# Build with MT7996 firmware embedded in initrd (for WiFi testing)
./build.sh --with-firmware

# Build with self-tests enabled
./build.sh --test

# Output files:
# - kernel.bin     : Kernel binary for Xmodem transfer (~1.1MB without firmware)
# - initrd.img     : Initial ramdisk with userspace programs
```

### WiFi Testing

When testing WiFi drivers (wifid, wifid2, wifid3), firmware must be available.
Two options:

1. **Embedded firmware** (recommended for testing):
   ```bash
   ./build.sh --with-firmware
   ```
   Firmware files are embedded in initrd.img and available at boot.
   Results in larger kernel (~3MB more).

2. **USB firmware** (default):
   ```bash
   ./build.sh
   ```
   Firmware loaded from USB drive via fatfs at runtime.

### Loading via U-Boot

```
1. loady 0x46000000
2. (send kernel.bin via Xmodem)
3. go 0x46000000
```

### Hardware

- **Board**: Banana Pi BPI-R4
- **SoC**: MediaTek MT7988A
- **CPU**: ARM Cortex-A73 (AArch64)
- **USB Controllers**:
  - SSUSB0 (IRQ 205): M.2 slot
  - SSUSB1 (IRQ 204): 4x USB-A ports via VL822 hub

### Transfer to Hardware

Use Xmodem to transfer `kernel8.img` to the board via serial console.

## Project Structure

```
bpi-r4-kernel/
├── src/                    # Kernel code
│   ├── main.rs             # Kernel entry point
│   ├── syscall.rs          # System call handlers
│   ├── shmem.rs            # Shared memory (DMA ring buffers)
│   ├── task.rs             # Task scheduler
│   └── ...
├── user/
│   ├── userlib/            # Userspace library (syscalls, ring buffers)
│   │   └── src/ring.rs     # Generic Ring<S,C> for IPC
│   ├── shell/              # Shell program
│   ├── gpio/               # GPIO driver (I2C to PCA9555)
│   └── driver/
│       ├── usb/            # USB library (xHCI, protocols)
│       │   └── ARCHITECTURE.md  # Ring buffer documentation
│       ├── usbd/           # USB daemon (host controller driver)
│       ├── wifid3/         # MT7996 WiFi driver (latest, line-by-line Linux port)
│       └── fatfs/          # FAT filesystem driver
├── linux/                  # Linux kernel source (for reference/debugging)
│   ├── drivers/net/wireless/mediatek/mt76/  # MT76 driver source
│   │   ├── dma.c           # Core DMA implementation (with debug printks)
│   │   ├── mt7996/         # MT7996-specific code
│   │   │   ├── dma.c       # MT7996 DMA init (with debug printks)
│   │   │   ├── mcu.c       # MCU/firmware loading (with debug printks)
│   │   │   └── regs.h      # Register definitions
│   │   └── dma.h           # DMA descriptor structures
│   ├── build-bpir4.sh      # Build script for Linux kernel
│   ├── build-docker.sh     # Docker-based build (for macOS)
│   └── Dockerfile          # Ubuntu 22.04 build environment
├── build.sh                # Main build script
├── mkinitrd.sh             # Creates initrd.tar
└── kernel.bin              # Output kernel binary
```

### Linux Source Code

**IMPORTANT**: We have multiple Linux source directories:

| Directory | Source | Purpose |
|-----------|--------|---------|
| `linux/` | Local kernel 6.12 | Debug printks, older code |
| `linux-upstream/` | `git clone --depth 1 https://github.com/torvalds/linux.git` | Latest upstream kernel |
| `mt76/` | `git clone https://github.com/openwrt/mt76.git` | Latest mt76 driver (often ahead of kernel) |

**Always use `mt76/` or `linux-upstream/` for reference** - the local `linux/` has older mt7996 code
that's missing features like `GLO_CFG_EXT_EN`, `PAUSE_RX_Q` thresholds, and PCIe link speed detection.

The OpenWrt mt76 repository is typically the most current for WiFi driver development.

Key files for MT7996:
- `mt76/mt7996/dma.c` - DMA initialization (upstream has more features)
- `mt76/mt7996/mcu.c` - MCU/firmware loading
- `mt76/mt7996/init.c` - Hardware initialization
- `mt76/mt7996/pci.c` - PCIe probe sequence
- `mt76/mt7996/regs.h` - Register definitions

## Key Architecture Concepts

### Zero-Copy DMA Ring Buffers

Communication between drivers uses shared memory with physical addresses for DMA:

```
fatfs ←── shared memory ──→ usbd ←── xHCI DMA ──→ USB device
          (ring buffer)              (TRBs)
```

- Kernel provides `shmem_create/map` syscalls
- `userlib::ring::Ring<S,C>` builds on shmem
- USB DMA writes directly to shared buffer (no copies)

### USB Driver Layers

```
fatfs (FAT filesystem) ←─ BlockClient
    ↓
usbd (USB daemon) ←─ BlockServer, ring buffer
    ↓
xHCI controller ←─ TRB rings, events, doorbells
    ↓
MT7988A SSUSB + T-PHY
```

## Documentation

### MT7996 WiFi Driver

Documentation for the MT7996 WiFi driver development is in `docs/mt7996-linux/`:

- `TRANSLATION_PLAN.md` - **ACTIVE** Function-by-function Linux-to-Rust translation plan
- `CALL_CHAIN.md` - Linux driver call chain analysis for firmware loading
- `dma.c` - Annotated copy of Linux mt76 DMA code
- `deepwiki.md` - Verified register addresses and DMA sequences

---

## MT7996 Translation Rules (MANDATORY)

**STATUS**: RST register stuck at 0x30 after 2 weeks debugging. Building NEW driver `mt7996_v2` with 100% fidelity.

### NEW DRIVER: mt7996_v2

Location: `user/driver/mt7996_v2/`

This is a CLEAN, line-by-line translation of Linux mt76/mt7996 driver.
The old `mt7996` driver is kept for comparison to identify what went wrong.

### NO DEVIATION ALLOWED

The MT7996 v2 driver MUST be an EXACT translation of Linux mt76/mt7996 driver code.

1. **EXACT BIT MANIPULATION** - Use identical bit operations (AND, OR, shifts)
2. **EXACT REGISTER ACCESS ORDER** - Same sequence as Linux
3. **EXACT RMW PATTERN** - `mt76_set` = read | bits, `mt76_clear` = read & ~bits
4. **EXACT TIMING** - Same delays (usleep_range → delay_us, mdelay → delay_ms)
5. **EXACT FUNCTION STRUCTURE** - Same control flow, same conditionals
6. **NO IMPROVEMENTS** - Even if Rust way is "better", use C way
7. **NO SHORTCUTS** - Every step must be present
8. **DOCUMENT EVERY LINE** - Comment with Linux source file:line

### Function Translation Checklist

For each Linux function being ported:

1. [ ] Extract EXACT C source code with line numbers
2. [ ] Document in `TRANSLATION_PLAN.md`
3. [ ] Translate line-by-line to Rust
4. [ ] Verify bit masks match exactly
5. [ ] Verify RMW pattern matches (read, modify, write)
6. [ ] Verify conditional structure matches
7. [ ] Verify no extra operations added
8. [ ] Verify no operations removed
9. [ ] Mark as verified in plan

### Key Functions to Port (in order)

| Priority | Linux Function | File |
|----------|----------------|------|
| 1 | mt7996_dma_disable() | dma.c:123 |
| 2 | mt7996_dma_prefetch() | dma.c:78 |
| 3 | mt76_dma_queue_reset() | dma.c |
| 4 | mt7996_dma_enable() | dma.c:239 |
| 5 | mt7996_dma_start() | dma.c:176 |
| 6 | mt76_init_mcu_queue() | dma.c |
| 7 | mt7996_driver_own() | mcu.c |

### Reference Registers

```
MT_WFDMA0_BASE          = 0xd4000
MT_WFDMA0_PCIE1_BASE    = 0xd8000  (HIF2 offset = 0x4000)
MT_WFDMA0_RST           = 0xd4100
MT_WFDMA0_GLO_CFG       = 0xd4208
MT_WFDMA0_RST_DTX_PTR   = 0xd420c
MT_WFDMA_EXT_CSR_HIF_MISC = 0xd7044
```

---

Key findings so far:
- MT7996 uses **two PCIe devices**: HIF1 (0x7990) and HIF2 (0x7991)
- Both interfaces must be initialized for proper operation
- RST register stays at 0x30 during ring programming (should stay 0x30)
- RST should AUTO-RELEASE to 0x00 when GLO_CFG enables DMA
- **RST NOT auto-releasing** - root cause unknown after 2 weeks

---

## Common Issues

### USB Known Issues (Regressions)

**Status**: Multiple USB subsystem issues exist. These were working at some point but regressed.

#### Hub Interrupt Endpoint Not Working
- **Symptom**: Hub interrupt IN endpoint (EP3) never fires despite correct configuration
- **Workaround**: Control-transfer polling (GET_PORT_STATUS) every 1 second
- **Details**: Endpoint context shows state=1 (Running), DCS matches TRB cycle, dequeue pointer never advances
- **Note**: This WAS working in earlier commits - regression needs git bisect

#### MSC Data Transfer Flaky
- **Symptom**: USB flaky when fetching data from Mass Storage Class devices
- **Symptom**: BOT (Bulk-Only Transport) recovery triggered frequently
- **Symptom**: MSC recognition hit/miss due to timing issues
- **Impact**: File operations unreliable, data transfer may fail or require retries

### USB Enumeration Failures

- **"Address failed (no event)"**: Timing issue, retries should help
- **Hub ports show 0x02a0**: USB 3.0 ports powered but no connection detected
- **Intermittent detection**: Increase polling time for hub ports

### IRQ Numbers (GIC)

- IRQ = SPI + 32
- SSUSB0: SPI 173 → IRQ 205
- SSUSB1: SPI 172 → IRQ 204

## Current Work (as of last session)

- Ring buffer architecture documented
- BlockServer renamed from RingClient
- Hub port polling improved (2 second timeout)
- ADDRESS_DEVICE retry logic added (3 attempts)

## Useful Commands

```bash
# Check git status
git status

# Recent commits
git log --oneline -5

# Build only usbd
cd user/driver/usbd && cargo build --release

# Full rebuild
./build.sh
```
