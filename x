#!/bin/bash
# Wrapper script for cargo xtask
#
# The main project's .cargo/config.toml sets build target to aarch64-unknown-none
# which uses nightly with build-std and doesn't have std.
# xtask needs to run on the host with std, so we override the target.
#
# Usage:
#   ./x build              # Full build (kernel + userspace + initrd)
#   ./x build --platform qemu  # Build for QEMU
#   ./x build --test       # Build with self-tests enabled
#   ./x qemu               # Run in QEMU
#   ./x clean              # Clean all build artifacts

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Detect host target
HOST_TARGET=$(rustc -vV | grep "^host:" | cut -d' ' -f2)

# Run xtask on the host (not cross-compiled)
# Use --target to override the .cargo/config.toml setting
# Use +stable to avoid using nightly (which the kernel requires)
cd "$SCRIPT_DIR/xtask"
cargo +stable run --quiet --target "$HOST_TARGET" -- "$@"
