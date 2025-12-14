#!/bin/bash
#
# Set up TFTP server and deploy kernel for network boot
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
KERNEL_BIN="$PROJECT_DIR/kernel.bin"
TFTP_DIR="/private/tftpboot"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "==================================="
echo "BPI-R4 TFTP Deployment"
echo "==================================="
echo ""

# Check if kernel exists
if [ ! -f "$KERNEL_BIN" ]; then
    echo -e "${RED}Error: kernel.bin not found${NC}"
    echo "Run 'make' first to build the kernel"
    exit 1
fi

# Create TFTP directory if needed
if [ ! -d "$TFTP_DIR" ]; then
    echo -e "${YELLOW}Creating TFTP directory...${NC}"
    sudo mkdir -p "$TFTP_DIR"
    sudo chmod 777 "$TFTP_DIR"
fi

# Copy kernel to TFTP directory
echo "Copying kernel.bin to $TFTP_DIR..."
cp "$KERNEL_BIN" "$TFTP_DIR/"
echo -e "${GREEN}Done!${NC}"
echo ""

# Show kernel info
echo "Kernel info:"
ls -lh "$TFTP_DIR/kernel.bin"
echo ""

# Check if TFTP is enabled on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! sudo launchctl list | grep -q tftp; then
        echo -e "${YELLOW}TFTP server is not running.${NC}"
        echo ""
        echo "To enable macOS built-in TFTP server:"
        echo "  sudo launchctl load -F /System/Library/LaunchDaemons/tftp.plist"
        echo ""
        echo "To disable later:"
        echo "  sudo launchctl unload /System/Library/LaunchDaemons/tftp.plist"
        echo ""
    else
        echo -e "${GREEN}TFTP server is running.${NC}"
    fi
fi

# Get local IP
LOCAL_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | head -1 | awk '{print $2}')

echo ""
echo "==================================="
echo "U-Boot Commands"
echo "==================================="
echo ""
echo "1. Connect BPI-R4 LAN port to your computer"
echo "2. Power on BPI-R4, interrupt U-Boot (press any key)"
echo "3. Run these commands in U-Boot:"
echo ""
echo -e "${GREEN}setenv ipaddr 192.168.1.2${NC}"
echo -e "${GREEN}setenv serverip ${LOCAL_IP:-192.168.1.1}${NC}"
echo -e "${GREEN}tftp 0x46000000 kernel.bin${NC}"
echo -e "${GREEN}go 0x46000000${NC}"
echo ""
echo "==================================="
