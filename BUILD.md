# BPI-R4 Kernel Build Guide

## Prerequisites

- Rust nightly toolchain with `aarch64-unknown-none` target
- `rust-objcopy` (from `cargo install cargo-binutils`)
- `picocom` with xmodem support (`lsx`)

```bash
# Install Rust nightly and target
rustup default nightly
rustup target add aarch64-unknown-none

# Install binutils for objcopy
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

## Quick Build

```bash
# Build everything (user programs + kernel + binary)
./build.sh
```

This creates `kernel.bin` ready for loading.

## Manual Build Steps

### 1. Build User Programs

```bash
cd user
./build.sh      # Builds shell, usbtest, etc.
./mkinitrd.sh   # Creates initrd.tar (embedded in kernel)
cd ..
```

### 2. Build Kernel

```bash
cargo build --release
```

The kernel embeds `user/initrd.tar` automatically (feature `embed-initrd` is default).

### 3. Create Binary Image

```bash
rust-objcopy -O binary \
    target/aarch64-unknown-none/release/bpi-r4-kernel \
    kernel.bin
```

## Loading onto Hardware

### Serial Connection

```bash
picocom -b 115200 --send-cmd "lsx /path/to/kernel.bin" /dev/cu.usbmodem*
```

### U-Boot Commands

1. **Start xmodem receive:**
   ```
   loady 0x46000000
   ```

2. **In picocom, send file:** Press `Ctrl+A` then `Ctrl+S`

3. **Execute kernel:**
   ```
   go 0x46000000
   ```

### One-liner U-Boot

```
loady 0x46000000 && go 0x46000000
```

## Memory Map

| Address      | Usage                    |
|--------------|--------------------------|
| 0x46000000   | Kernel load address      |
| 0x40000000   | DRAM start               |
| 0x40010000   | User process load address|
| 0x48000000   | External initrd (if used)|

## Build Options

### Without Embedded initrd

```bash
cargo build --release --no-default-features
```

Then load initrd separately in U-Boot:
```
loady 0x48000000  # Load initrd.img
loady 0x46000000  # Load kernel.bin
go 0x46000000
```

### Debug Build

```bash
cargo build  # Without --release
```

## Project Structure

```
bpi-r4-kernel/
├── src/                    # Kernel source
│   ├── main.rs            # Entry point, IRQ handler
│   ├── boot.S             # Assembly startup, exception vectors
│   ├── usb.rs             # USB xHCI driver
│   ├── scheme.rs          # Scheme system (mmio, irq, console)
│   └── ...
├── user/                   # Userspace programs
│   ├── userlib/           # Syscall wrappers, runtime
│   ├── shell/             # Interactive shell
│   ├── usbtest/           # USB userspace driver test
│   ├── build.sh           # Build user programs
│   ├── mkinitrd.sh        # Create initrd.tar
│   └── initrd.tar         # Embedded in kernel
├── build.sh               # Full build script
├── kernel.bin             # Output binary
└── Cargo.toml
```

## Shell Commands

Once booted, the shell provides:

```
> help
Available commands:
  help, ?     - Show this help
  exit, quit  - Exit the shell
  pid         - Show current process ID
  uptime      - Show system uptime
  mem         - Test memory allocation
  echo <msg>  - Echo a message
  spawn <id>  - Spawn process by ELF ID
  usb         - Run USB userspace driver test
  yield       - Yield CPU to other processes
  panic       - Trigger a panic (test)
```

## Troubleshooting

### Kernel doesn't start
- Verify U-Boot is at EL2 (kernel drops to EL1)
- Check serial baud rate is 115200
- Ensure binary is loaded to correct address

### USB test shows no devices
- Run `usb start` in U-Boot first to initialize PHY
- Or kernel USB driver should detect devices automatically

### Build fails with linker errors
- Ensure you're in the correct directory
- Run `cargo clean` and rebuild
