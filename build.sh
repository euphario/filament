#!/bin/bash
# Build script for BPI-R4 kernel
# Creates kernel.bin ready for loading via U-Boot xmodem
#
# Usage:
#   ./build.sh                    # Normal build (incremental)
#   ./build.sh --test             # Build with self-tests enabled
#   ./build.sh --with-firmware    # Embed MT7996 firmware in initrd (~3MB larger)
#   ./build.sh --skip-user        # Skip userspace build (kernel only)
#   ./build.sh --only devd shell  # Build only specific userspace programs

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
FEATURES=""
BUILD_TYPE="Production"
INITRD_OPTS=""
FIRMWARE_MODE="USB (fatfs)"
SKIP_USER=false
USER_PROGRAMS=""

while [ $# -gt 0 ]; do
    case "$1" in
        --test|-t)
            FEATURES="--features selftest"
            BUILD_TYPE="Development (with self-tests)"
            shift
            ;;
        --with-firmware)
            INITRD_OPTS="--with-firmware"
            FIRMWARE_MODE="Embedded"
            shift
            ;;
        --skip-user)
            SKIP_USER=true
            shift
            ;;
        --only)
            shift
            # Collect remaining args as program names
            while [ $# -gt 0 ] && [[ ! "$1" =~ ^-- ]]; do
                USER_PROGRAMS="$USER_PROGRAMS $1"
                shift
            done
            ;;
        *)
            shift
            ;;
    esac
done

echo "========================================"
echo "  BPI-R4 Kernel Build"
echo "  Mode: $BUILD_TYPE"
echo "  Firmware: $FIRMWARE_MODE"
echo "========================================"
echo

# Step 0: Compile device tree (skip if unchanged)
if [ -f bpi-r4.dtb ] && [ bpi-r4.dtb -nt bpi-r4.dts ]; then
    echo "Step 0: Device tree (unchanged)"
elif command -v dtc >/dev/null 2>&1; then
    echo "Step 0: Compiling device tree..."
    dtc -I dts -O dtb -o bpi-r4.dtb bpi-r4.dts 2>/dev/null || {
        echo "  Warning: dtc failed, skipping DTB"
    }
    if [ -f bpi-r4.dtb ]; then
        DTB_SIZE=$(ls -lh bpi-r4.dtb | awk '{print $5}')
        echo "  Created: bpi-r4.dtb ($DTB_SIZE)"
    fi
else
    echo "Step 0: Warning: dtc not found, skipping DTB"
fi
echo

# Step 1: Build user programs
if [ "$SKIP_USER" = true ]; then
    echo "Step 1: Skipping user programs (--skip-user)"
elif [ -n "$USER_PROGRAMS" ]; then
    echo "Step 1: Building user programs:$USER_PROGRAMS"
    (cd user && ./build.sh $USER_PROGRAMS)
else
    echo "Step 1: Building user programs (incremental)..."
    (cd user && ./build.sh)
fi
echo

# Step 2: Create initrd
echo "Step 2: Creating initrd..."
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
echo "To load via U-Boot (Xmodem - slow but reliable):"
echo "  1. loady 0x46000000"
echo "  2. (send kernel.bin via xmodem)"
echo "  3. go 0x46000000"
echo
echo "To load via U-Boot (USB - fast):"
echo "  1. Copy kernel.bin to USB drive (FAT32)"
echo "  2. Insert USB drive into BPI-R4"
echo "  3. Run these commands:"
echo "     usb start"
echo "     fatload usb 0:1 0x46000000 kernel.bin"
echo "     go 0x46000000"
echo
echo "DTB is embedded in kernel.bin - no separate loading needed."
echo
