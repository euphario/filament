# Architecture Decision Records (ADR)

This document captures key architectural decisions made during the development of the BPI-R4 microkernel. Each decision is documented with context, rationale, and consequences.

---

## ADR-001: TLP-Based PCIe Configuration Instead of ECAM

**Date:** 2024-12 (initial decision)

**Scope:** MT7988A SoC PCIe host controller (affects all PCIe endpoints)

### Context

PCIe configuration space access can be implemented in two ways:

#### ECAM (Enhanced Configuration Access Mechanism)
Standard memory-mapped config space where each device's 4KB config space is mapped contiguously:
```
ECAM Base + (Bus << 20) | (Device << 15) | (Function << 12) | Register
```
- Simple: Just read/write to calculated address
- Fast: Single memory access
- Standard: Works with generic PCIe drivers

#### TLP (Transaction Layer Packet)
Software configures hardware to generate Type 0/1 configuration TLPs:
```
1. Write target BDF to control register
2. Hardware generates config TLP
3. Read/write through a window register
```
- More steps: Register setup before each access
- Explicit: Software controls transaction type
- Flexible: Can target any BDF regardless of address space

### Why ECAM Doesn't Work on MT7988A

**Note:** This is an MT7988A (SoC) constraint, not related to any specific endpoint device. The MT7988A is the MediaTek networking SoC used on the BPI-R4; its PCIe host controller uses this TLP-based mechanism regardless of what devices are connected.

The MT7988A SoC has several constraints that make standard ECAM impractical:

1. **No dedicated ECAM region**: Unlike Intel/AMD platforms, MediaTek doesn't expose a flat ECAM memory range. The PCIe MAC uses a different architecture.

2. **Multiple independent root ports**: MT7988A has 3 PCIe ports (pcie0, pcie1, pcie2), each with its own MAC. There's no unified config space view.

3. **Root Complex config is special**: The Root Port's own config space (BDF 00:00.0) is partially read-only when accessed via MMIO at offset 0x1000. Only TLP-based access allows full read/write.

4. **Address translation complexity**: ECAM would require setting up inbound ATU (Address Translation Unit) entries for each bus range, adding complexity and potential for misconfiguration.

### Decision

Use MediaTek's TLP-based configuration access via the CFGNUM register:

#### Register Layout (CFGNUM at MAC + 0x140)
```
Bits 7:0   - DevFn (Device << 3 | Function)
Bits 15:8  - Bus Number
Bits 19:16 - Byte Enable (0xF for 32-bit, varies for smaller)
Bit 20     - Force Byte Enable
```

#### Access Sequence
```rust
// 1. Build CFGNUM value with target BDF
let devfn = (device << 3) | function;
let cfgnum = FORCE_BYTE_EN
    | (0xF << BYTE_EN_SHIFT)  // 32-bit access
    | (bus << BUS_SHIFT)
    | devfn;

// 2. Write to CFGNUM register
mac.write32(0x140, cfgnum);

// 3. Access config through window at MAC + 0x1000
let value = mac.read32(0x1000 + (reg & !0x3));
```

#### Config Window (MAC + 0x1000)
After setting CFGNUM, the 4KB region at MAC + 0x1000 becomes a window into the target device's config space. The hardware automatically generates:
- **Type 0 config TLP** if Bus == 0 (local device)
- **Type 1 config TLP** if Bus > 0 (downstream device)

### Rationale

1. **It's what works**: This is how MediaTek designed the hardware. The CFGNUM mechanism is documented in their reference drivers and is the only reliable way to access config space.

2. **Full access to Root Port**: The Root Port's Command register, BAR registers, and other writable fields can only be modified via TLP access, not direct MMIO.

3. **Matches Linux driver**: MediaTek's upstream Linux driver (`pcie-mediatek-gen3.c`) uses exactly this mechanism. Alignment with reference code reduces debugging time.

4. **Per-port isolation**: Each port has its own CFGNUM/window, so there's no risk of one port's config access interfering with another.

### Implementation Details

#### Sub-32-bit Accesses
For 8-bit or 16-bit config reads, the byte enable field is adjusted:
```rust
// 16-bit read at offset 0x06 (Status register)
let bytes = 0x3 << (offset & 0x3);  // 0x3 << 2 = 0xC
let cfgnum = FORCE_BYTE_EN | (bytes << BYTE_EN_SHIFT) | ...;
// Read 32-bit, extract relevant 16-bit portion
let val32 = mac.read32(0x1000);
let result = (val32 >> ((offset & 2) * 8)) & 0xFFFF;
```

#### Read-Modify-Write for 16-bit Writes
To avoid corrupting adjacent registers:
```rust
// Read full 32-bit
let val32 = config.read32(bdf, aligned_reg);
// Modify target 16-bit portion
let new_val = (val32 & mask) | (value << shift);
// Write back
config.write32(bdf, aligned_reg, new_val);
```

### Consequences

**Positive:**
- Works correctly with all MT7988A PCIe controller quirks
- Full read/write access to Root Port config space
- Clear separation between config and memory access
- No complex ATU setup required
- Proven approach (matches Linux driver)

**Negative:**
- 2-3 register accesses per config read (vs. 1 for ECAM)
- ~200ns latency per access (vs. ~50ns for ECAM)
- Cannot use generic ECAM-based PCIe libraries unchanged
- More code to implement and maintain

**Mitigations:**
- Config access is infrequent (enumeration, capability setup)
- Performance impact is negligible for real workloads
- Abstracted behind `PcieConfigSpace` struct

### Alternative Considered: Hybrid Approach

We considered using MMIO for some accesses and TLP for others:
- MMIO for simple reads (faster)
- TLP only for writes or Root Port access

**Rejected because:**
- Added complexity with minimal benefit
- Some MMIO reads return stale/incorrect values
- Consistency is more valuable than micro-optimization

### Files

| File | Purpose |
|------|---------|
| `user/driver/pcie/src/config.rs` | `PcieConfigSpace` implementation |
| `user/driver/pcie/src/regs.rs` | CFGNUM register definitions |
| `user/driver/pcie/src/controller.rs` | Uses config space for enumeration |
| `user/driver/pcied/src/main.rs` | Device enumeration |

### References

- MediaTek `pcie-mediatek-gen3.c` in Linux kernel
- PCIe Base Specification (TLP format, Type 0/1 config)
- MT7988A Technical Reference Manual (register layout)

---

## ADR-002: Fire-and-Forget Log Distribution

**Date:** 2026-01

### Context

Log distribution from kernel and userspace to multiple sinks (console, file loggers, network monitors) needs a reliable architecture that:
- Never blocks producers (kernel, time-critical drivers)
- Handles slow or unresponsive sinks gracefully
- Supports dynamic sink registration/deregistration

### Decision

Implement fire-and-forget log distribution:
1. **logd daemon** is the single reader of kernel log ring buffer
2. logd broadcasts logs to registered sinks without waiting for acknowledgment
3. Sinks send periodic health acks (informational, not flow control)
4. Slow sinks may miss messages (dropped, not queued indefinitely)

```
Kernel ring buffer → logd → [sink1, sink2, sink3, ...]
                         (fire-and-forget broadcast)
```

### Rationale

- **Non-blocking producers**: Kernel/driver logging never blocks regardless of sink state
- **Predictable memory**: No unbounded queue growth from slow sinks
- **Graceful degradation**: Losing logs is better than blocking critical paths
- **Simple implementation**: No complex flow control or backpressure

### Consequences

**Positive:**
- Logging from IRQ context is safe (just writes to ring buffer)
- Slow network loggers don't affect kernel or console
- Easy to add/remove sinks at runtime

**Negative:**
- Logs can be lost if sinks are too slow
- No guaranteed delivery (acceptable for logs)
- Sinks must be fast enough to keep up

### Files

- `src/klog.rs` - Kernel log ring buffer with KlogReady event
- `user/driver/logd/src/main.rs` - Log distribution daemon
- `user/driver/consoled/src/main.rs` - Console sink implementation

---

## ADR-003: Single Source of Truth in devd

**Date:** 2024-12 (initial decision), 2026-01 (formalized)

### Context

Device management in a microkernel requires tracking:
- What hardware exists (enumeration)
- What drivers are loaded
- Device states (discovered, bound, faulted)
- Driver lifecycle (starting, running, crashed)

Two approaches were considered:
1. **Distributed**: Each bus driver maintains its own device database
2. **Centralized**: Single daemon (devd) is the authoritative source

### Decision

devd is the single source of truth for all device and driver state:
1. Bus drivers (pcied, usbd) report discovered devices to devd
2. Device drivers claim devices through devd
3. devd tracks device states, driver assignments, fault history
4. All queries about "what devices exist" go through devd

```
           devd (single source of truth)
          /    \
    pcied        usbd
   (reports)    (reports)
         \      /
          devd has complete device tree
```

### Rationale

- **Consistency**: One canonical view of device state
- **Supervision**: devd can restart crashed drivers, knows all device states
- **Query simplicity**: "hw list" doesn't need to poll multiple bus drivers
- **Recovery coordination**: devd coordinates bus resets, driver restarts

### Consequences

**Positive:**
- `hw list` shows complete system state from one source
- Driver crashes are detected and handled uniformly
- Easy to implement "which driver owns which device" lookups

**Negative:**
- devd is a critical daemon (if it crashes, device management stops)
- Slight IPC overhead for device registration/claims
- devd must be started before any device drivers

### Files

- `user/driver/devd/src/main.rs` - Device manager (single source of truth)
- `user/userlib/src/ipc/protocols/devd.rs` - DevdProtocol for registration/queries
- `user/driver/pcied/src/main.rs` - Reports devices to devd

---

## ADR-004: Capability-Based Security for IPC

**Date:** 2026-01

### Context

In a microkernel, many services are accessible via IPC. Without security checks, any process that can connect to a service can use all its operations.

Example vulnerabilities:
- Any process could reset PCIe ports (breaking other drivers)
- Any process could claim devices (impersonating legitimate drivers)
- Any process could enable DMA on devices

### Decision

Implement capability-based security at the protocol handler level:
1. Kernel tracks capabilities per-process (capability bits)
2. Syscalls `GetCapabilities` and `ChannelGetPeer` allow servers to identify and check callers
3. Protocol handlers check required capabilities before executing privileged operations

```rust
// Server checks caller has required capability
if !conn.client_has_capability(syscall::caps::MMIO) {
    return PcieResponse::Error(-1); // Permission denied
}
```

### Rationale

- **Defense in depth**: IPC-level checks complement kernel syscall checks
- **Fine-grained**: Different operations can require different capabilities
- **Auditable**: Capability requirements documented in protocol handlers
- **Flexible**: Capabilities can be granted/revoked per-process

### Consequences

**Positive:**
- Untrusted processes cannot perform privileged operations
- Clear documentation of what capabilities each operation needs
- Graceful denial (error response) instead of crash

**Negative:**
- Additional syscall overhead for capability checks
- Server implementations must remember to check capabilities
- Capability model must be designed carefully to avoid privilege escalation

### Implementation

| Server | Operation | Required Capability |
|--------|-----------|---------------------|
| pcied | ResetPort | MMIO |
| pcied | EnableBusMaster | DMA |
| pcied | ClearDeviceStatus | MMIO |
| devd | ClaimBus | SCHEME_CREATE |
| devd | ClaimDevice | IPC |

### Files

- `src/kernel/syscall.rs` - GetCapabilities, ChannelGetPeer syscalls
- `user/userlib/src/syscall.rs` - Capability constants and helpers
- `user/userlib/src/ipc/server.rs` - Connection capability checking methods
- `user/driver/pcied/src/main.rs` - Example capability checks

---

## ADR-005: Event-Driven I/O Instead of Polling

**Date:** 2026-01

### Context

Userspace daemons need to wait for multiple event sources:
- IPC messages from clients
- Timer expirations
- Hardware interrupts (via kernel)
- File descriptor readability

Traditional approaches:
1. **Polling**: Repeatedly check each source (wastes CPU)
2. **Blocking on single source**: Can't handle multiple sources
3. **Event subscription**: Subscribe to events, wake when any fires

### Decision

Use event-driven I/O throughout userspace:
1. Tasks subscribe to event types they care about (IpcReady, Timer, FdReadable, etc.)
2. `event_wait` blocks until any subscribed event fires
3. Kernel broadcasts events to subscribers (no polling needed)

```rust
syscall::event_subscribe(event_type::IPC_READY, channel as u64);
syscall::event_subscribe(event_type::FD_READABLE, 0);

loop {
    syscall::event_wait(&mut event, event_flags::BLOCKING);
    match event.event_type {
        IPC_READY => handle_ipc(event.data),
        FD_READABLE => handle_input(),
        _ => {}
    }
}
```

### Rationale

- **CPU efficiency**: No busy-waiting or periodic polling
- **Latency**: Events delivered immediately when available
- **Scalability**: O(1) event delivery regardless of subscriber count
- **Composability**: Easy to add new event sources

### Consequences

**Positive:**
- Daemons sleep until work arrives (good for power/thermal)
- Low-latency response to events
- Clean multiplexing of multiple event sources

**Negative:**
- More complex than simple blocking I/O
- Must remember to subscribe before waiting
- Event queue has finite size (events can be dropped if full)

### Files

- `src/kernel/event.rs` - Event system implementation
- `user/userlib/src/syscall.rs` - Event types and syscall wrappers
- `user/driver/consoled/src/main.rs` - Example event-driven daemon

---

## Document History

| Date | Change |
|------|--------|
| 2026-01-16 | Initial ADR document created |
| 2026-01-16 | ADR-001 through ADR-005 documented |
