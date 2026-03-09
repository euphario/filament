#!/bin/sh
# Run libf unit tests on the host.
#
# The parent .cargo/config.toml files set target=aarch64-unknown-none and
# build-std=["core","alloc"] which prevents host test builds.
# We work around this by temporarily hiding all parent configs.
#
# Usage: ./test.sh [extra cargo test args...]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Parent configs that force bare-metal target + build-std
CONFIGS="$SCRIPT_DIR/../.cargo/config.toml $SCRIPT_DIR/../../.cargo/config.toml"
HIDDEN=""

cleanup() {
    for bak in $HIDDEN; do
        orig="${bak%.bak}"
        mv "$bak" "$orig"
    done
}
trap cleanup EXIT

for cfg in $CONFIGS; do
    if [ -f "$cfg" ]; then
        mv "$cfg" "$cfg.bak"
        HIDDEN="$HIDDEN $cfg.bak"
    fi
done

cargo test --no-default-features "$@"
