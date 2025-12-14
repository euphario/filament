#!/bin/bash
#
# Connect to BPI-R4 serial console
#

set -e

BAUD=115200

# Find the USB serial device
find_serial_port() {
    # macOS
    if [[ "$OSTYPE" == "darwin"* ]]; then
        ls /dev/cu.usbserial* 2>/dev/null | head -1
    # Linux
    else
        ls /dev/ttyUSB* 2>/dev/null | head -1
    fi
}

PORT=${1:-$(find_serial_port)}

if [ -z "$PORT" ]; then
    echo "Error: No USB serial device found"
    echo ""
    echo "Make sure:"
    echo "  1. USB-C cable is connected to debug port (CN41)"
    echo "  2. BPI-R4 is powered on"
    echo ""
    echo "Usage: $0 [port]"
    echo "  e.g., $0 /dev/cu.usbserial-1234"
    exit 1
fi

echo "==================================="
echo "BPI-R4 Serial Console"
echo "==================================="
echo "Port: $PORT"
echo "Baud: $BAUD"
echo ""
echo "Press Ctrl+A then Ctrl+X to exit"
echo "==================================="
echo ""

# Check for picocom
if command -v picocom &> /dev/null; then
    exec picocom -b $BAUD "$PORT"
# Fall back to screen
elif command -v screen &> /dev/null; then
    echo "(Using screen - press Ctrl+A then K to exit)"
    exec screen "$PORT" $BAUD
else
    echo "Error: Neither picocom nor screen found"
    echo "Install with: brew install picocom"
    exit 1
fi
