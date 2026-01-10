#!/bin/bash
# MT7996 Macro Verification Script
#
# This script compiles and runs both the C and Rust verification programs,
# then compares their output to ensure the Rust translations match exactly.
#
# Usage: ./verify.sh
#
# Requirements:
#   - gcc (for C program)
#   - rustc (for Rust program)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== MT7996 Macro Verification ==="
echo

# Compile C program
echo "Compiling C verification program..."
gcc -o verify_macros verify_macros.c
echo "  Done: verify_macros"

# Compile Rust program
echo "Compiling Rust verification program..."
rustc verify_rust.rs -o verify_rust
echo "  Done: verify_rust"

# Run both and capture output
echo
echo "Running C program..."
./verify_macros > c_output.txt

echo "Running Rust program..."
./verify_rust > rust_output.txt

# Compare outputs
echo
echo "=== Comparing outputs ==="
if diff -q c_output.txt rust_output.txt > /dev/null 2>&1; then
    echo "SUCCESS: All values match!"
    echo
    echo "C and Rust implementations produce identical output."
else
    echo "DIFFERENCES FOUND:"
    echo
    diff c_output.txt rust_output.txt || true
    echo
    echo "Please fix the discrepancies above."
    exit 1
fi

# Cleanup
rm -f verify_macros verify_rust c_output.txt rust_output.txt

echo
echo "=== Verification Complete ==="
