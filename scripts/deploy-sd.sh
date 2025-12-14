#!/bin/bash
#
# Copy kernel to SD card for boot
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
KERNEL_BIN="$PROJECT_DIR/kernel.bin"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "==================================="
echo "BPI-R4 SD Card Deployment"
echo "==================================="
echo ""

# Check if kernel exists
if [ ! -f "$KERNEL_BIN" ]; then
    echo -e "${RED}Error: kernel.bin not found${NC}"
    echo "Run 'make' first to build the kernel"
    exit 1
fi

# Find mounted SD card (look for common names)
find_sd_mount() {
    for name in "BPI-R4" "BOOT" "boot" "NO NAME" "UNTITLED"; do
        mount_point="/Volumes/$name"
        if [ -d "$mount_point" ]; then
            echo "$mount_point"
            return
        fi
    done
    # Linux
    for path in /media/$USER/* /mnt/*; do
        if [ -d "$path" ]; then
            echo "$path"
            return
        fi
    done
}

SD_MOUNT=${1:-$(find_sd_mount)}

if [ -z "$SD_MOUNT" ] || [ ! -d "$SD_MOUNT" ]; then
    echo -e "${RED}Error: SD card mount point not found${NC}"
    echo ""
    echo "Make sure the SD card is mounted."
    echo ""
    echo "Usage: $0 [mount_point]"
    echo "  e.g., $0 /Volumes/BOOT"
    exit 1
fi

echo "SD card found at: $SD_MOUNT"
echo ""

# Copy kernel
echo "Copying kernel.bin..."
cp "$KERNEL_BIN" "$SD_MOUNT/"
echo -e "${GREEN}Done!${NC}"
echo ""

# Show what's on the card
echo "Contents of SD card:"
ls -lh "$SD_MOUNT/"*.bin 2>/dev/null || ls -lh "$SD_MOUNT/"
echo ""

# Sync and eject instructions
sync

echo "==================================="
echo "Next Steps"
echo "==================================="
echo ""
echo "1. Safely eject the SD card"
echo "2. Insert into BPI-R4"
echo "3. Power on and interrupt U-Boot"
echo "4. Run:"
echo ""
echo -e "${GREEN}fatload mmc 0:1 0x46000000 kernel.bin${NC}"
echo -e "${GREEN}go 0x46000000${NC}"
echo ""
echo "==================================="
