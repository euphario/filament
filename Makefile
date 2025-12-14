# BPI-R4 Bare-Metal Kernel Makefile

# Configuration
TARGET := aarch64-unknown-none
KERNEL_NAME := bpi-r4-kernel
KERNEL_ELF := target/$(TARGET)/release/$(KERNEL_NAME)
KERNEL_BIN := kernel.bin

# Tools
CARGO := cargo
OBJCOPY := rust-objcopy
OBJDUMP := rust-objdump

# TFTP settings (adjust for your setup)
TFTP_DIR := /private/tftpboot

# Serial port (adjust for your system)
SERIAL_PORT := $(shell ls /dev/cu.usbserial* 2>/dev/null | head -1)
SERIAL_BAUD := 115200

.PHONY: all build clean flash tftp serial objdump check

# Default target
all: $(KERNEL_BIN)

# Build the kernel ELF
build:
	$(CARGO) build --release

# Convert ELF to raw binary
$(KERNEL_BIN): build
	$(OBJCOPY) -O binary $(KERNEL_ELF) $(KERNEL_BIN)
	@echo ""
	@echo "Kernel binary created: $(KERNEL_BIN)"
	@ls -lh $(KERNEL_BIN)
	@echo ""

# Clean build artifacts
clean:
	$(CARGO) clean
	rm -f $(KERNEL_BIN)

# Copy kernel to TFTP directory for network boot
tftp: $(KERNEL_BIN)
	@if [ ! -d "$(TFTP_DIR)" ]; then \
		echo "Error: TFTP directory $(TFTP_DIR) does not exist"; \
		echo "Create it with: sudo mkdir -p $(TFTP_DIR) && sudo chmod 777 $(TFTP_DIR)"; \
		exit 1; \
	fi
	cp $(KERNEL_BIN) $(TFTP_DIR)/
	@echo "Kernel copied to $(TFTP_DIR)/$(KERNEL_BIN)"
	@echo ""
	@echo "In U-Boot, run:"
	@echo "  setenv ipaddr 192.168.1.2"
	@echo "  setenv serverip 192.168.1.1"
	@echo "  tftp 0x46000000 $(KERNEL_BIN)"
	@echo "  go 0x46000000"

# Connect to serial console
serial:
	@if [ -z "$(SERIAL_PORT)" ]; then \
		echo "Error: No USB serial device found"; \
		echo "Check that the USB-C debug cable is connected"; \
		exit 1; \
	fi
	@echo "Connecting to $(SERIAL_PORT) at $(SERIAL_BAUD) baud..."
	@echo "Press Ctrl+A then Ctrl+X to exit"
	picocom -b $(SERIAL_BAUD) $(SERIAL_PORT)

# Disassemble the kernel (useful for debugging)
objdump: build
	$(OBJDUMP) -d $(KERNEL_ELF) | head -100

# Show kernel sections
sections: build
	$(OBJDUMP) -h $(KERNEL_ELF)

# Check that toolchain is set up correctly
check:
	@echo "Checking toolchain..."
	rustup show
	@echo ""
	@echo "Checking target..."
	rustup target list | grep $(TARGET)
	@echo ""
	@echo "Checking tools..."
	which $(OBJCOPY) || echo "Install with: cargo install cargo-binutils"
	@echo ""
	@echo "Checking serial port..."
	@if [ -n "$(SERIAL_PORT)" ]; then \
		echo "Found: $(SERIAL_PORT)"; \
	else \
		echo "No USB serial device found"; \
	fi

# Help
help:
	@echo "BPI-R4 Bare-Metal Kernel"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build kernel binary (default)"
	@echo "  build    - Compile kernel ELF"
	@echo "  clean    - Remove build artifacts"
	@echo "  tftp     - Copy kernel to TFTP directory"
	@echo "  serial   - Connect to serial console"
	@echo "  objdump  - Disassemble kernel"
	@echo "  sections - Show kernel sections"
	@echo "  check    - Verify toolchain setup"
	@echo "  help     - Show this help"
	@echo ""
	@echo "Quick start:"
	@echo "  1. make check      # Verify setup"
	@echo "  2. make            # Build kernel"
	@echo "  3. make serial     # Connect to serial (in another terminal)"
	@echo "  4. make tftp       # Deploy via TFTP"
	@echo ""
	@echo "U-Boot commands:"
	@echo "  fatload mmc 0:1 0x46000000 kernel.bin"
	@echo "  go 0x46000000"
