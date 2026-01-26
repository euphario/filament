//! Build tasks for BPI-R4 kernel
//!
//! Usage:
//!   cargo xtask build              # Full build (kernel + userspace + initrd)
//!   cargo xtask build --test       # Build with self-tests enabled
//!   cargo xtask build --firmware   # Embed MT7996 firmware in initrd
//!   cargo xtask build --skip-user  # Skip userspace build (kernel only)
//!   cargo xtask build --only devd shell  # Build specific userspace programs
//!   cargo xtask qemu               # Run in QEMU virt
//!   cargo xtask clean              # Clean all build artifacts

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Build tasks for BPI-R4 kernel
#[derive(Parser)]
#[command(name = "xtask")]
#[command(about = "Build tasks for BPI-R4 kernel")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Build kernel and userspace
    Build {
        /// Enable self-tests
        #[arg(short, long)]
        test: bool,

        /// Embed MT7996 firmware in initrd
        #[arg(long)]
        firmware: bool,

        /// Skip userspace build
        #[arg(long)]
        skip_user: bool,

        /// Build only specific programs (e.g., --only devd shell)
        #[arg(long, num_args = 1..)]
        only: Vec<String>,

        /// Target platform: mt7988a (default) or qemu
        #[arg(long, default_value = "mt7988a")]
        platform: String,
    },

    /// Run in QEMU
    Qemu {
        /// Skip build before running
        #[arg(long)]
        no_build: bool,

        /// Enable GDB server
        #[arg(short, long)]
        gdb: bool,

        /// Enable self-tests
        #[arg(short, long)]
        test: bool,
    },

    /// Clean all build artifacts
    Clean,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Find project root (where Cargo.toml is)
    let project_root = project_root()?;

    match cli.command {
        Commands::Build { test, firmware, skip_user, only, platform } => {
            cmd_build(&project_root, test, firmware, skip_user, only, &platform)?;
        }
        Commands::Qemu { no_build, gdb, test } => {
            cmd_qemu(&project_root, !no_build, gdb, test)?;
        }
        Commands::Clean => {
            cmd_clean(&project_root)?;
        }
    }

    Ok(())
}

/// Find the project root directory
fn project_root() -> Result<PathBuf> {
    // Start from the xtask directory and go up to find the kernel project root
    let xtask_manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let root = xtask_manifest.parent()
        .context("Could not find parent of xtask directory")?;

    // Verify this is the kernel project root
    if !root.join("src/main.rs").exists() || !root.join("user").exists() {
        bail!("Expected kernel project root at {}", root.display());
    }

    Ok(root.to_path_buf())
}

/// Build kernel and userspace
fn cmd_build(
    root: &Path,
    test: bool,
    firmware: bool,
    skip_user: bool,
    only: Vec<String>,
    platform: &str,
) -> Result<()> {
    let build_type = if test { "Development (self-tests)" } else { "Production" };
    let firmware_mode = if firmware { "Embedded" } else { "USB (fatfs)" };
    let platform_feature = match platform {
        "qemu" | "qemu-virt" => "platform-qemu-virt",
        _ => "platform-mt7988a",
    };

    println!("========================================");
    println!("  BPI-R4 Kernel Build");
    println!("  Mode: {}", build_type);
    println!("  Platform: {}", platform);
    println!("  Firmware: {}", firmware_mode);
    println!("========================================");
    println!();

    // Step 0: Compile device tree (if dtc available and for mt7988a)
    if platform == "mt7988a" {
        build_dtb(root)?;
    }

    // Step 1: Build user programs
    if !skip_user {
        build_user(root, &only)?;
    } else {
        println!("Step 1: Skipping user programs (--skip-user)");
    }
    println!();

    // Step 2: Create initrd
    create_initrd(root, firmware)?;
    println!();

    // Step 3: Build kernel
    build_kernel(root, test, platform_feature)?;
    println!();

    // Step 4: Create binary
    create_binary(root)?;

    println!();
    println!("========================================");
    println!("  Build Complete!");
    println!("========================================");
    println!();

    let kernel_size = fs::metadata(root.join("kernel.bin"))
        .map(|m| format_size(m.len()))
        .unwrap_or_else(|_| "?".into());
    println!("  kernel.bin: {}", kernel_size);

    if root.join("bpi-r4.dtb").exists() {
        let dtb_size = fs::metadata(root.join("bpi-r4.dtb"))
            .map(|m| format_size(m.len()))
            .unwrap_or_else(|_| "?".into());
        println!("  bpi-r4.dtb: {} (standalone copy)", dtb_size);
    }

    println!();
    println!("To load via U-Boot (Xmodem):");
    println!("  1. loady 0x46000000");
    println!("  2. (send kernel.bin via xmodem)");
    println!("  3. go 0x46000000");
    println!();

    Ok(())
}

/// Build device tree
fn build_dtb(root: &Path) -> Result<()> {
    let dts = root.join("bpi-r4.dts");
    let dtb = root.join("bpi-r4.dtb");

    if !dts.exists() {
        println!("Step 0: Device tree (no .dts file)");
        return Ok(());
    }

    // Check if DTB is up-to-date
    if dtb.exists() {
        let dts_modified = fs::metadata(&dts)?.modified()?;
        let dtb_modified = fs::metadata(&dtb)?.modified()?;
        if dtb_modified >= dts_modified {
            println!("Step 0: Device tree (unchanged)");
            return Ok(());
        }
    }

    // Check if dtc is available
    if Command::new("dtc").arg("--version").output().is_err() {
        println!("Step 0: Warning: dtc not found, skipping DTB");
        return Ok(());
    }

    println!("Step 0: Compiling device tree...");
    let status = Command::new("dtc")
        .args(["-I", "dts", "-O", "dtb", "-o"])
        .arg(&dtb)
        .arg(&dts)
        .current_dir(root)
        .status()
        .context("Failed to run dtc")?;

    if !status.success() {
        println!("  Warning: dtc failed, skipping DTB");
        return Ok(());
    }

    let size = fs::metadata(&dtb)
        .map(|m| format_size(m.len()))
        .unwrap_or_else(|_| "?".into());
    println!("  Created: bpi-r4.dtb ({})", size);

    Ok(())
}

/// Build user programs
fn build_user(root: &Path, only: &[String]) -> Result<()> {
    let user_dir = root.join("user");

    // Programs to build: name:path pairs
    let all_programs = [
        ("devd", "driver/devd"),
        ("consoled", "driver/consoled"),
        ("shell", "shell"),
    ];

    let programs: Vec<_> = if only.is_empty() {
        all_programs.to_vec()
    } else {
        all_programs
            .iter()
            .filter(|(name, _)| only.contains(&name.to_string()))
            .cloned()
            .collect()
    };

    if programs.is_empty() {
        println!("Step 1: No programs to build");
        return Ok(());
    }

    let names: Vec<_> = programs.iter().map(|(n, _)| *n).collect();
    println!("Step 1: Building user programs: {}", names.join(", "));

    let bin_dir = user_dir.join("bin");
    fs::create_dir_all(&bin_dir)?;

    for (name, path) in &programs {
        build_user_program(root, name, path)?;
    }

    Ok(())
}

/// Build a single user program
fn build_user_program(root: &Path, name: &str, path: &str) -> Result<()> {
    let user_dir = root.join("user");
    let prog_dir = user_dir.join(path);
    let bin_dir = user_dir.join("bin");

    println!("  Building {}...", name);

    // Verify the directory exists
    if !prog_dir.exists() {
        bail!("Program directory not found: {}", prog_dir.display());
    }

    // Build with cargo, overriding the kernel's linker script with user.ld
    // CARGO_ENCODED_RUSTFLAGS completely replaces [target.*.rustflags] from .cargo/config.toml
    // The separator is U+001F (unit separator)
    let rustflags = [
        "-Clink-arg=-Tuser.ld",
        "-Clink-arg=--gc-sections",
        "-Clink-arg=-n",
    ].join("\x1f");

    // Use nightly toolchain for user programs (they need build-std for aarch64-unknown-none)
    // Clear CARGO_BUILD_TARGET to let the user programs use their own target from .cargo/config.toml
    let output = Command::new("cargo")
        .args(["+nightly", "build", "--release"])
        .current_dir(&prog_dir)
        .env("CARGO_ENCODED_RUSTFLAGS", &rustflags)
        .env_remove("CARGO_BUILD_TARGET")
        .output()
        .context(format!("Failed to run cargo for {}", name))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        bail!("cargo build failed for {}:\n{}", name, stderr);
    }

    let status = output.status;

    if !status.success() {
        bail!("Failed to build {}", name);
    }

    // Find the ELF (user programs share a target directory at user/target/)
    let elf_src = user_dir.join(format!("target/aarch64-unknown-none/release/{}", name));
    let elf_dst = bin_dir.join(format!("{}.elf", name));
    let elf_rs = bin_dir.join(format!("{}_elf.rs", name));

    // Check if ELF changed
    if elf_dst.exists() {
        let src_content = fs::read(&elf_src)?;
        let dst_content = fs::read(&elf_dst)?;
        if src_content == dst_content {
            println!("    (unchanged)");
            return Ok(());
        }
    }

    // Copy ELF
    fs::copy(&elf_src, &elf_dst)?;

    // Convert to raw binary
    let bin_path = bin_dir.join(format!("{}.bin", name));
    let status = Command::new("rust-objcopy")
        .args(["-O", "binary"])
        .arg(&elf_src)
        .arg(&bin_path)
        .status()
        .context("Failed to run rust-objcopy")?;

    if !status.success() {
        bail!("rust-objcopy failed for {}", name);
    }

    let elf_size = format_size(fs::metadata(&elf_dst)?.len());
    let bin_size = format_size(fs::metadata(&bin_path)?.len());
    println!("    ELF: {}, BIN: {}", elf_size, bin_size);

    // Generate Rust include file
    println!("    Generating {}_elf.rs...", name);
    let elf_content = fs::read(&elf_dst)?;
    let upper_name = name.to_uppercase();

    let mut rs_content = String::new();
    rs_content.push_str(&format!("//! Auto-generated ELF binary for {}\n", name));
    rs_content.push_str("//! Generated by cargo xtask\n\n");
    rs_content.push_str("#[rustfmt::skip]\n");
    rs_content.push_str(&format!("pub static {}_ELF: &[u8] = &[\n", upper_name));

    for (i, byte) in elf_content.iter().enumerate() {
        if i % 12 == 0 {
            rs_content.push_str("    ");
        }
        rs_content.push_str(&format!("0x{:02x}, ", byte));
        if i % 12 == 11 {
            rs_content.push('\n');
        }
    }
    if elf_content.len() % 12 != 0 {
        rs_content.push('\n');
    }
    rs_content.push_str("];\n");

    fs::write(&elf_rs, rs_content)?;

    Ok(())
}

/// Create initrd TAR archive
fn create_initrd(root: &Path, include_firmware: bool) -> Result<()> {
    let user_dir = root.join("user");
    let bin_dir = user_dir.join("bin");
    let staging_dir = user_dir.join("initrd_staging");
    let output = user_dir.join("initrd.tar");

    println!("Step 2: Creating initrd...");

    // Clean and create staging
    if staging_dir.exists() {
        fs::remove_dir_all(&staging_dir)?;
    }
    fs::create_dir_all(staging_dir.join("bin"))?;

    // Copy ELF binaries
    for entry in fs::read_dir(&bin_dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.extension().map_or(false, |e| e == "elf") {
            let name = path.file_stem().unwrap();
            let dst = staging_dir.join("bin").join(name);
            fs::copy(&path, &dst)?;
            println!("  bin/{}", name.to_string_lossy());
        }
    }

    // Copy firmware if requested
    if include_firmware {
        let firmware_dir = root.join("firmware/mediatek/mt7996");
        if firmware_dir.exists() {
            println!();
            println!("  Including firmware files...");
            let fw_staging = staging_dir.join("lib/firmware/mediatek/mt7996");
            fs::create_dir_all(&fw_staging)?;

            for entry in fs::read_dir(&firmware_dir)? {
                let entry = entry?;
                let path = entry.path();
                if path.extension().map_or(false, |e| e == "bin") {
                    let name = path.file_name().unwrap();
                    fs::copy(&path, fw_staging.join(name))?;
                    let size = format_size(fs::metadata(&path)?.len());
                    println!("  lib/firmware/mediatek/mt7996/{} ({})", name.to_string_lossy(), size);
                }
            }
        }
    } else {
        println!("  (firmware loaded from USB via fatfs)");
    }

    // Create TAR archive
    println!();
    println!("  Creating TAR archive...");

    let status = Command::new("tar")
        .args(["--format=ustar", "-cvf"])
        .arg(&output)
        .arg(".")
        .current_dir(&staging_dir)
        .status()
        .context("Failed to create TAR")?;

    if !status.success() {
        bail!("tar failed");
    }

    let size = format_size(fs::metadata(&output)?.len());
    println!("  Created: initrd.tar ({})", size);

    // Create padded image
    let img_path = user_dir.join("initrd.img");
    let tar_content = fs::read(&output)?;
    let padded_size = ((tar_content.len() + 4095) / 4096) * 4096;
    let mut img_content = tar_content;
    img_content.resize(padded_size, 0);
    fs::write(&img_path, &img_content)?;

    let img_size = format_size(padded_size as u64);
    println!("  Created: initrd.img ({}, 4KB aligned)", img_size);

    // Cleanup
    fs::remove_dir_all(&staging_dir)?;

    Ok(())
}

/// Build the kernel
fn build_kernel(root: &Path, test: bool, platform_feature: &str) -> Result<()> {
    println!("Step 3: Building kernel...");

    // Build features list
    let features = if test {
        format!("selftest,{}", platform_feature)
    } else {
        platform_feature.to_string()
    };

    // Include embed-initrd always, embed-dtb only for real hardware
    // (QEMU doesn't need embedded DTB - it constructs its own)
    let full_features = if platform_feature == "platform-qemu-virt" {
        format!("{},embed-initrd", features)
    } else {
        format!("{},embed-initrd,embed-dtb", features)
    };

    // Use nightly toolchain with build-std
    // Clear CARGO_BUILD_TARGET to use target from .cargo/config.toml
    let mut cmd = Command::new("cargo");
    cmd.args(["+nightly", "build", "--release", "--no-default-features", "--features", &full_features])
        .current_dir(root)
        .env_remove("CARGO_BUILD_TARGET");

    // For QEMU, use the QEMU-specific linker script (kernel loads at 0x40000000)
    // For MT7988A, use the default linker script (kernel loads at 0x46000000)
    if platform_feature == "platform-qemu-virt" {
        // Override the linker script via CARGO_ENCODED_RUSTFLAGS
        let rustflags = [
            "-Clink-arg=-Tsrc/linker-qemu.ld",
        ].join("\x1f");
        cmd.env("CARGO_ENCODED_RUSTFLAGS", &rustflags);
        println!("  Using QEMU linker script (load at 0x40000000)");
    }

    let status = cmd.status().context("Failed to build kernel")?;

    if !status.success() {
        bail!("Kernel build failed");
    }

    println!("  Kernel built successfully");

    Ok(())
}

/// Create the final binary
fn create_binary(root: &Path) -> Result<()> {
    println!("Step 4: Creating kernel.bin...");

    let elf_path = root.join("target/aarch64-unknown-none/release/bpi-r4-kernel");
    let bin_path = root.join("kernel.bin");

    let status = Command::new("rust-objcopy")
        .args(["-O", "binary"])
        .arg(&elf_path)
        .arg(&bin_path)
        .status()
        .context("Failed to run rust-objcopy")?;

    if !status.success() {
        bail!("rust-objcopy failed");
    }

    Ok(())
}

/// Run in QEMU
fn cmd_qemu(root: &Path, build: bool, gdb: bool, test: bool) -> Result<()> {
    if build {
        cmd_build(root, test, false, false, vec![], "qemu")?;
    }

    println!();
    println!("Starting QEMU...");

    // With the QEMU linker script, kernel is linked at 0x40000000 which matches
    // where QEMU loads raw binaries (-kernel loads at DRAM_BASE for virt machine)
    let kernel_path = root.join("kernel.bin");
    if !kernel_path.exists() {
        bail!("kernel.bin not found - run './x build --platform qemu' first");
    }

    // Create disk images for emulated devices if they don't exist
    let usb_disk = "/tmp/bpi-r4-usb.img";
    let nvme_disk = "/tmp/bpi-r4-nvme.img";

    if !Path::new(usb_disk).exists() {
        println!("Creating USB disk image: {}", usb_disk);
        Command::new("qemu-img")
            .args(["create", "-f", "raw", usb_disk, "64M"])
            .status()
            .context("Failed to create USB disk image")?;
    }

    if !Path::new(nvme_disk).exists() {
        println!("Creating NVMe disk image: {}", nvme_disk);
        Command::new("qemu-img")
            .args(["create", "-f", "raw", nvme_disk, "128M"])
            .status()
            .context("Failed to create NVMe disk image")?;
    }

    // Use virt machine with GICv3 (our kernel uses GICv3 system registers)
    // Include: xHCI USB with hub/MSC/HID, NVMe, virtio-net
    let kernel_str = kernel_path.to_string_lossy().to_string();
    let usb_drive_str = format!("id=usbdisk,file={},format=raw,if=none", usb_disk);
    let nvme_drive_str = format!("id=nvme0,file={},format=raw,if=none", nvme_disk);
    let args = vec![
        "-M", "virt,gic-version=3",
        "-cpu", "cortex-a72",
        "-m", "512M",
        "-nographic",
        "-kernel", &kernel_str,
        // USB xHCI controller with devices
        "-device", "qemu-xhci,id=xhci",
        "-device", "usb-hub,bus=xhci.0,port=1",
        "-device", "usb-storage,bus=xhci.0,port=2,drive=usbdisk",
        "-drive", &usb_drive_str,
        "-device", "usb-kbd,bus=xhci.0,port=3",
        "-device", "usb-mouse,bus=xhci.0,port=4",
        // NVMe controller
        "-device", "nvme,drive=nvme0,serial=BPIR4NVME",
        "-drive", &nvme_drive_str,
        // Network (virtio - simpler than emulating real NIC)
        "-device", "virtio-net-pci,netdev=net0",
        "-netdev", "user,id=net0",
    ];

    if gdb {
        println!("GDB server listening on :1234");
        println!("Connect with: gdb -ex 'target remote :1234'");
    }

    let mut cmd = Command::new("qemu-system-aarch64");
    cmd.args(&args).current_dir(root);

    if gdb {
        cmd.args(["-s", "-S"]);
    }

    let status = cmd.status().context("Failed to run QEMU")?;

    if !status.success() {
        println!("QEMU exited with error");
    }

    Ok(())
}

/// Clean build artifacts
fn cmd_clean(root: &Path) -> Result<()> {
    println!("Cleaning build artifacts...");

    // Clean kernel
    let status = Command::new("cargo")
        .arg("clean")
        .current_dir(root)
        .status()?;

    if !status.success() {
        println!("  Warning: cargo clean failed");
    }

    // Clean user programs
    for dir in ["user/driver/devd", "user/driver/consoled", "user/shell"] {
        let path = root.join(dir);
        if path.exists() {
            let _ = Command::new("cargo")
                .arg("clean")
                .current_dir(&path)
                .status();
        }
    }

    // Remove generated files
    let files_to_remove = [
        "kernel.bin",
        "bpi-r4.dtb",
        "user/initrd.tar",
        "user/initrd.img",
    ];

    for file in &files_to_remove {
        let path = root.join(file);
        if path.exists() {
            fs::remove_file(&path)?;
            println!("  Removed: {}", file);
        }
    }

    // Remove bin directory
    let bin_dir = root.join("user/bin");
    if bin_dir.exists() {
        fs::remove_dir_all(&bin_dir)?;
        println!("  Removed: user/bin/");
    }

    println!("Clean complete!");

    Ok(())
}

/// Format a file size nicely
fn format_size(bytes: u64) -> String {
    if bytes >= 1024 * 1024 {
        format!("{:.1}M", bytes as f64 / (1024.0 * 1024.0))
    } else if bytes >= 1024 {
        format!("{:.1}K", bytes as f64 / 1024.0)
    } else {
        format!("{}B", bytes)
    }
}
