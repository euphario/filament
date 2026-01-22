#!/bin/bash
# Run unit tests on host platform
#
# The kernel uses aarch64-unknown-none target with build-std which conflicts
# with host-target testing. This script temporarily disables the cargo config
# to run tests on the native host.

set -e

cd "$(dirname "$0")"

# Temporarily disable the bare-metal cargo config
mv .cargo/config.toml .cargo/config.toml.bak
trap 'mv .cargo/config.toml.bak .cargo/config.toml' EXIT

# Run the tests
cargo test --lib "$@"
