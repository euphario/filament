#!/bin/bash
# Build and run script for QEMU virt platform
#
# Usage:
#   ./build-qemu.sh              # Build kernel for QEMU
#   ./build-qemu.sh --run        # Build and run in QEMU
#   ./build-qemu.sh --run-only   # Run existing kernel.qemu.bin in QEMU
#   ./build-qemu.sh --test       # Build with self-tests enabled
#   ./build-qemu.sh --debug      # Run with GDB stub enabled

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
FEATURES=""
BUILD_TYPE="Production"
RUN_QEMU=false
RUN_ONLY=false
DEBUG_MODE=false
SKIP_USER=false

while [ $# -gt 0 ]; do
    case "$1" in
        --run|-r)
            RUN_QEMU=true
            shift
            ;;
        --run-only)
            RUN_ONLY=true
            RUN_QEMU=true
            shift
            ;;
        --test|-t)
            FEATURES="$FEATURES --features selftest"
            BUILD_TYPE="Development (with self-tests)"
            shift
            ;;
        --debug|-d)
            DEBUG_MODE=true
            shift
            ;;
        --skip-user)
            SKIP_USER=true
            shift
            ;;
        *)
            shift
            ;;
    esac
done

echo "========================================"
echo "  QEMU Virt Platform Build"
echo "  Mode: $BUILD_TYPE"
echo "========================================"
echo

if [ "$RUN_ONLY" = false ]; then
    # Step 1: Build user programs (needed for initrd)
    if [ "$SKIP_USER" = true ]; then
        echo "Step 1: Skipping user programs (--skip-user)"
    else
        echo "Step 1: Building user programs..."
        (cd user && ./build.sh) || {
            echo "Warning: User build failed, continuing anyway"
        }
    fi
    echo

    # Step 2: Create initrd
    echo "Step 2: Creating initrd..."
    (cd user && ./mkinitrd.sh) || {
        echo "Warning: Initrd creation failed, continuing anyway"
    }
    echo

    # Step 3: Build kernel for QEMU
    echo "Step 3: Building kernel for QEMU virt..."
    cargo build --release --no-default-features --features "platform-qemu-virt,embed-initrd" $FEATURES 2>&1 | \
        grep -E "^(   Compiling|    Finished|error|warning:.*generated)" || true
    echo

    # Step 4: Create binary
    echo "Step 4: Creating kernel.qemu.bin..."
    rust-objcopy -O binary \
        target/aarch64-unknown-none/release/bpi-r4-kernel \
        kernel.qemu.bin

    KERNEL_SIZE=$(ls -lh kernel.qemu.bin | awk '{print $5}')
    echo
    echo "========================================"
    echo "  Build Complete!"
    echo "========================================"
    echo
    echo "  kernel.qemu.bin: $KERNEL_SIZE"
    echo
fi

# Run in QEMU if requested
if [ "$RUN_QEMU" = true ]; then
    if [ ! -f kernel.qemu.bin ]; then
        echo "Error: kernel.qemu.bin not found. Run without --run-only first."
        exit 1
    fi

    echo "========================================"
    echo "  Starting QEMU"
    echo "========================================"
    echo
    echo "  Machine: virt (GICv3)"
    echo "  CPU: cortex-a72"
    echo "  RAM: 512MB"
    echo "  Load addr: 0x46000000"
    echo
    echo "  Press Ctrl-A X to exit QEMU"
    echo "========================================"
    echo

    QEMU_ARGS=(
        -M virt,gic-version=3
        -cpu cortex-a72
        -m 512
        -nographic
        -device loader,file=kernel.qemu.bin,addr=0x46000000,cpu-num=0
        -smp 1
    )

    if [ "$DEBUG_MODE" = true ]; then
        echo "  Debug mode: GDB stub on localhost:1234"
        echo "  Connect with: gdb-multiarch -ex 'target remote :1234'"
        echo
        QEMU_ARGS+=(-S -s)
    fi

    # Check for qemu-system-aarch64
    if ! command -v qemu-system-aarch64 &> /dev/null; then
        echo "Error: qemu-system-aarch64 not found"
        echo "Install with: brew install qemu (macOS) or apt install qemu-system-arm (Linux)"
        exit 1
    fi

    exec qemu-system-aarch64 "${QEMU_ARGS[@]}"
fi

echo "To run in QEMU:"
echo "  ./build-qemu.sh --run"
echo
echo "To run with GDB debugging:"
echo "  ./build-qemu.sh --run --debug"
echo
