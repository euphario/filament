# BPI-R4 Kernel Project

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

## Common Issues

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
