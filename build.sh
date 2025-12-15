#!/bin/bash
# Build script for BPI-R4 kernel
# Creates kernel.bin ready for loading via U-Boot xmodem

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================"
echo "  BPI-R4 Kernel Build"
echo "========================================"
echo

# Step 1: Build user programs
echo "Step 1: Building user programs..."
(cd user && ./build.sh)
echo

# Step 2: Create initrd
echo "Step 2: Creating initrd..."
(cd user && ./mkinitrd.sh)
echo

# Step 3: Build kernel
echo "Step 3: Building kernel..."
cargo build --release 2>&1 | grep -E "^(   Compiling|    Finished|error|warning:.*generated)" || true
echo

# Step 4: Create binary
echo "Step 4: Creating kernel.bin..."
rust-objcopy -O binary \
    target/aarch64-unknown-none/release/bpi-r4-kernel \
    kernel.bin

# Show results
KERNEL_SIZE=$(ls -lh kernel.bin | awk '{print $5}')
echo
echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo
echo "  kernel.bin: $KERNEL_SIZE"
echo
echo "To load via U-Boot:"
echo "  1. loady 0x46000000"
echo "  2. (send kernel.bin via xmodem)"
echo "  3. go 0x46000000"
echo
