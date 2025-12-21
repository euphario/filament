#!/bin/bash
# Create initrd TAR archive for BPI-R4 kernel
#
# Usage: ./mkinitrd.sh [options] [output.tar]
#
# Options:
#   --no-firmware    Skip firmware files (smaller initrd for faster xmodem)
#   --help           Show this help
#
# This script builds all user programs and packages them into a TAR archive
# that can be loaded by U-Boot as an initrd.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
INCLUDE_FIRMWARE=true
OUTPUT="initrd.tar"

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-firmware)
            INCLUDE_FIRMWARE=false
            shift
            ;;
        --help|-h)
            echo "Usage: ./mkinitrd.sh [options] [output.tar]"
            echo ""
            echo "Options:"
            echo "  --no-firmware    Skip firmware files (smaller initrd for faster xmodem)"
            echo "  --help           Show this help"
            exit 0
            ;;
        *)
            OUTPUT="$1"
            shift
            ;;
    esac
done

echo "Building initrd: $OUTPUT"
if [ "$INCLUDE_FIRMWARE" = false ]; then
    echo "  (skipping firmware - use USB for firmware files)"
fi
echo ""

# Build user programs first
./build.sh

# Create staging directory
STAGING="$SCRIPT_DIR/initrd_staging"
rm -rf "$STAGING"
mkdir -p "$STAGING/bin"

# Copy ELF binaries to staging
echo "Staging files..."
for elf in bin/*.elf; do
    if [ -f "$elf" ]; then
        name=$(basename "$elf" .elf)
        cp "$elf" "$STAGING/bin/$name"
        echo "  bin/$name"
    fi
done

# Copy firmware files if present and requested
FIRMWARE_DIR="$SCRIPT_DIR/../firmware/mediatek/mt7996"
if [ "$INCLUDE_FIRMWARE" = true ] && [ -d "$FIRMWARE_DIR" ]; then
    echo ""
    echo "Staging firmware files..."
    mkdir -p "$STAGING/lib/firmware/mediatek/mt7996"
    for fw in "$FIRMWARE_DIR"/*.bin; do
        if [ -f "$fw" ]; then
            name=$(basename "$fw")
            cp "$fw" "$STAGING/lib/firmware/mediatek/mt7996/$name"
            size=$(ls -lh "$fw" | awk '{print $5}')
            echo "  lib/firmware/mediatek/mt7996/$name ($size)"
        fi
    done
elif [ "$INCLUDE_FIRMWARE" = false ]; then
    echo ""
    echo "Skipping firmware (--no-firmware specified)"
fi

# Create the TAR archive (POSIX format for compatibility)
echo ""
echo "Creating TAR archive..."
cd "$STAGING"
tar --format=ustar -cvf "$SCRIPT_DIR/$OUTPUT" .

# Show result
cd "$SCRIPT_DIR"
echo ""
echo "Created: $OUTPUT ($(ls -lh "$OUTPUT" | awk '{print $5}'))"
echo ""

# Also create a binary blob that can be embedded or loaded
# Add padding to align to 4KB for easier loading
PADDED_SIZE=$(( (($(stat -f%z "$OUTPUT") + 4095) / 4096) * 4096 ))
dd if=/dev/zero bs=1 count=$PADDED_SIZE of="${OUTPUT%.tar}.img" 2>/dev/null
dd if="$OUTPUT" of="${OUTPUT%.tar}.img" conv=notrunc 2>/dev/null
echo "Created: ${OUTPUT%.tar}.img ($(ls -lh "${OUTPUT%.tar}.img" | awk '{print $5}'), 4KB aligned)"

# Cleanup staging
rm -rf "$STAGING"

echo ""
echo "To load with U-Boot:"
echo "  fatload mmc 0:1 \$ramdisk_addr_r initrd.img"
echo "  booti \$kernel_addr_r \$ramdisk_addr_r:\$filesize \$fdt_addr_r"
