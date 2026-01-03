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

## Quick Reference

### Build Commands

```bash
# Full build (kernel + userspace + initrd)
./build.sh

# Output files:
# - kernel.bin     : Kernel binary for Xmodem transfer (616KB)
# - initrd.img     : Initial ramdisk with userspace programs
```

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
│       └── fatfs/          # FAT filesystem driver
├── build.sh                # Main build script
├── mkinitrd.sh             # Creates initrd.tar
└── kernel8.img             # Output kernel binary
```

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
