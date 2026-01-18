# Security Model

## Overview

The kernel uses a **capability-based security model** where processes have explicit permission bits that control access to privileged operations. Capabilities are:
- Granted at process spawn time by the parent
- Inherited but never escalated (child can only have subset of parent's capabilities)
- Checked by the kernel before allowing privileged syscalls

## Capability Bits

Defined in `src/kernel/caps.rs`:

| Capability | Bit | Description |
|------------|-----|-------------|
| IPC | 0 | Create channels, send/receive messages |
| MEMORY | 1 | Allocate memory, create mappings |
| SPAWN | 2 | Spawn new processes |
| MMIO | 3 | Map device MMIO regions |
| IRQ_CLAIM | 4 | Claim hardware interrupts |
| DMA | 5 | Allocate DMA-capable memory |
| FILESYSTEM | 6 | Direct filesystem operations |
| SCHEME_CREATE | 7 | Create scheme handlers |
| GRANT | 8 | Grant capabilities to children |

## Capability Presets

Defined in `user/userlib/src/syscall.rs`:

| Preset | Capabilities | Use Case |
|--------|-------------|----------|
| USER_DEFAULT | IPC, MEMORY, SPAWN | Regular user programs, shell |
| BUS_DRIVER | IPC, MEMORY, SPAWN, SCHEME_CREATE, IRQ_CLAIM, MMIO, DMA, GRANT | pcied - spawns device drivers |
| DEVICE_DRIVER | IPC, MEMORY, SCHEME_CREATE, IRQ_CLAIM, MMIO, DMA | usbd, nvmed, wifid |
| FS_DRIVER | IPC, MEMORY, SCHEME_CREATE, FILESYSTEM | vfsd, fatfs |
| SERVICE_DRIVER | IPC, MEMORY, SCHEME_CREATE | consoled, logd |
| GPIO_DRIVER | IPC, MEMORY, SCHEME_CREATE, MMIO | gpio, pwm |

## Trust Chain

```
Kernel (all capabilities)
  └─ devd (PID 1, all capabilities)
       ├─ pcied (BUS_DRIVER - can spawn device drivers)
       │    └─ wifid (DEVICE_DRIVER - no GRANT, can't spawn privileged children)
       ├─ usbd (DEVICE_DRIVER)
       ├─ vfsd (FS_DRIVER)
       ├─ consoled (SERVICE_DRIVER)
       └─ shell (USER_DEFAULT - minimal privileges)
```

## Syscalls

### exec_with_caps (syscall 74)

Spawn a process with explicit capabilities:

```rust
// Userspace
pub fn exec_with_caps(path: &str, caps: u64) -> i64;

// Example: devd spawning pcied
let pid = syscall::exec_with_caps("pcied", caps::BUS_DRIVER);
```

Requirements:
- Caller must have SPAWN capability
- Caller must have GRANT capability
- Requested capabilities are intersected with caller's capabilities

### get_capabilities (syscall 66)

Query a process's capabilities:

```rust
pub fn get_capabilities(pid: u32) -> i64;
```

Returns the capability bitmask or negative error.

## Capability Checking

Before privileged operations, kernel checks capabilities:

```rust
// In syscall handler
fn sys_mmap_device(phys_addr: u64, size: u64) -> i64 {
    if let Err(e) = require_capability(Capabilities::MMIO) {
        return e;  // EPERM
    }
    // ... perform operation
}
```

## Delegation Model

Capabilities follow a delegation model:

1. **Parent grants to child** - `exec_with_caps` specifies what child gets
2. **Child can't exceed parent** - `child_capabilities()` intersects requested with parent's
3. **No privilege escalation** - A process can never gain capabilities it doesn't have

```rust
pub fn child_capabilities(parent_caps: Capabilities, requested: Capabilities) -> Capabilities {
    parent_caps & requested  // Intersection
}
```

## Future: Signed Binary Manifests

The `verify_binary_signature()` function provides a hook for future security:

```rust
pub fn verify_binary_signature(
    _data: &[u8],           // Raw ELF for hashing
    _name: &str,            // Binary name for manifest lookup
    _requested_caps: Option<Capabilities>,
) -> Result<(), ElfError> {
    // TODO: Verify Ed25519 signature in .sig ELF section
    // TODO: Check binary hash against known-good list
    // TODO: Validate requested caps against signed manifest
    Ok(())  // Currently: trust all binaries
}
```

When implemented, this will allow:
- Cryptographic verification of binaries before execution
- Per-binary capability manifests signed by build system
- Rejection of unsigned or tampered binaries

## Security Boundaries

| Boundary | Protection |
|----------|------------|
| User → Kernel | Capability checks on all privileged syscalls |
| Driver → Hardware | MMIO/IRQ/DMA capabilities required |
| Parent → Child | Delegation model, no escalation |
| devd → System | devd has all caps, crash recovery implemented |

## What Capabilities Don't Protect Against

- **Compromised devd** - devd has all capabilities and can spawn anything
- **Kernel bugs** - Kernel is the trusted base
- **Side channels** - No protection against timing attacks, etc.

The capability system provides **defense in depth**: a compromised driver can't corrupt other drivers' memory, claim their IRQs, or spawn privileged children.

## Design Intent

The capability system is designed to:

1. **Limit blast radius** - A buggy driver can only affect its own bus/device
2. **Make privilege explicit** - Code reviewers can see what capabilities a driver needs
3. **Enable least privilege** - Shell doesn't need MMIO, so it doesn't have it
4. **Support future hardening** - Signature verification can be added without API changes
