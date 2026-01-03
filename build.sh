#!/bin/bash
# Build script for BPI-R4 kernel
# Creates kernel.bin ready for loading via U-Boot xmodem
#
# Usage:
#   ./build.sh           # Normal build (no self-tests)
#   ./build.sh --test    # Build with self-tests enabled

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
FEATURES=""
BUILD_TYPE="Production"
if [ "$1" = "--test" ] || [ "$1" = "-t" ]; then
    FEATURES="--features selftest"
    BUILD_TYPE="Development (with self-tests)"
fi

echo "========================================"
echo "  BPI-R4 Kernel Build"
echo "  Mode: $BUILD_TYPE"
echo "========================================"
echo

# Step 0: Compile device tree
echo "Step 0: Compiling device tree..."
if command -v dtc >/dev/null 2>&1; then
    dtc -I dts -O dtb -o bpi-r4.dtb bpi-r4.dts 2>/dev/null || {
        echo "  Warning: dtc failed, skipping DTB"
    }
    if [ -f bpi-r4.dtb ]; then
        DTB_SIZE=$(ls -lh bpi-r4.dtb | awk '{print $5}')
        echo "  Created: bpi-r4.dtb ($DTB_SIZE)"
    fi
else
    echo "  Warning: dtc not found, skipping DTB compilation"
    echo "  Install with: brew install dtc (macOS) or apt install device-tree-compiler (Linux)"
fi
echo

# Step 1: Build user programs
echo "Step 1: Building user programs..."
(cd user && ./build.sh)
echo

# Step 2: Create initrd
# Use --with-firmware to embed firmware (adds ~3MB)
# Without it, firmware loads from USB via fatfs
echo "Step 2: Creating initrd..."
INITRD_OPTS="${INITRD_OPTS:-}"
(cd user && ./mkinitrd.sh $INITRD_OPTS)
echo

# Step 3: Build kernel
echo "Step 3: Building kernel..."
cargo build --release $FEATURES 2>&1 | grep -E "^(   Compiling|    Finished|error|warning:.*generated)" || true
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
echo "  kernel.bin: $KERNEL_SIZE (includes embedded DTB)"
if [ -f bpi-r4.dtb ]; then
    DTB_SIZE=$(ls -lh bpi-r4.dtb | awk '{print $5}')
    echo "  bpi-r4.dtb: $DTB_SIZE (standalone copy)"
fi
echo
echo "To load via U-Boot:"
echo "  1. loady 0x46000000"
echo "  2. (send kernel.bin via xmodem)"
echo "  3. go 0x46000000"
echo
echo "DTB is embedded in kernel.bin - no separate loading needed."
echo
