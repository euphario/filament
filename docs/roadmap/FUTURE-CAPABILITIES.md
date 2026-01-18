# Future Kernel Capabilities

Design notes for features to add to the BPI-R4 microkernel, drawing from BSD, Linux, and Redox OS.

## Philosophy

Prefer BSD-style approaches:
- "Do it right once" vs "ship it, fix later"
- Single coherent design over organic growth
- Documentation as first-class citizen
- Capability-based security over filter-based

---

## High Priority

### 1. Capsicum-Style Capabilities (FreeBSD)

Process enters "capability mode" and can only use pre-granted handles. No ambient authority - if you don't have a handle, you can't access it.

```rust
/// Enter capability mode - process can only use existing handles
/// No new file opens, no network connections, no device access
/// except through handles already held
pub fn cap_enter() -> Result<(), Error>;

/// Check if in capability mode
pub fn cap_sandboxed() -> bool;

/// Rights that can be restricted on handles
pub struct CapRights {
    pub read: bool,
    pub write: bool,
    pub seek: bool,
    pub mmap: bool,
    pub fstat: bool,
    pub ioctl: bool,
    pub connect: bool,   // for sockets
    pub accept: bool,
    pub bind: bool,
}

/// Limit rights on an existing handle (can only remove, never add)
pub fn cap_rights_limit(handle: Handle, rights: &CapRights) -> Result<(), Error>;
```

**Why:** Perfect for driver isolation. WiFi driver gets handles to PCIe BAR and DMA memory, enters cap_enter(), can never touch anything else even if compromised.

**Reference:** FreeBSD capsicum(4), cap_enter(2), cap_rights_limit(2)

---

### 2. Unified Event System (BSD kqueue-inspired)

Single interface for all event sources. No separate epoll/signalfd/timerfd/etc.

```rust
/// Event filter types - one enum for everything
pub enum EventFilter {
    /// File descriptor readable
    Read(Handle),
    /// File descriptor writable
    Write(Handle),
    /// Timer expired
    Timer { id: u32, interval: Duration, oneshot: bool },
    /// Process event (exit, fork, exec)
    Process { pid: Pid, events: ProcessEvents },
    /// Signal delivered
    Signal(Signal),
    /// IPC port has message
    Port(PortId),
    /// Shared memory region modified
    SharedMem { region: ShmemId, offset: u64, len: u64 },
    /// Device interrupt
    Interrupt(IrqNumber),
}

bitflags! {
    pub struct ProcessEvents: u32 {
        const EXIT = 0x01;
        const FORK = 0x02;
        const EXEC = 0x04;
    }
}

/// Register interest in events
pub fn event_subscribe(filter: EventFilter) -> Result<(), Error>;

/// Wait for events (returns multiple)
pub fn event_wait(events: &mut [Event], timeout: Option<Duration>) -> Result<usize, Error>;

/// Event returned from wait
pub struct Event {
    pub filter: EventFilter,
    pub data: u64,      // filter-specific data (bytes available, exit code, etc)
    pub flags: EventFlags,
}
```

**Why:** Clean, unified API. BSD kqueue is widely considered superior to Linux epoll. One syscall to wait on files, timers, processes, signals, IPC - everything.

**Reference:** FreeBSD kqueue(2), kevent(2)

---

### 3. Pledge-Style Syscall Restriction (OpenBSD)

Simpler than full Capsicum - process declares what categories of syscalls it needs.

```rust
/// Pledge categories
pub enum Pledge {
    Stdio,      // read, write, close, dup, pipe, etc
    Rpath,      // read-only file access
    Wpath,      // write file access
    Cpath,      // create/delete files
    Inet,       // network sockets
    Unix,       // unix domain sockets
    Dns,        // DNS resolution
    Proc,       // fork, kill, wait
    Exec,       // execve
    Prot,       // mprotect
    Mmap,       // mmap with exec
    Tty,        // terminal ioctls
    Audio,      // audio devices
    Video,      // video devices
}

/// Restrict process to only these pledge categories
/// Can only be made more restrictive, never loosened
pub fn pledge(promises: &[Pledge]) -> Result<(), Error>;

/// Also restrict filesystem visibility (like unveil)
pub fn unveil(path: &str, permissions: &str) -> Result<(), Error>;
// permissions: "r" read, "w" write, "x" execute, "c" create
```

**Example:**
```rust
fn main() {
    // Early: full access to set up
    let config = fs::read("/etc/myapp.conf")?;
    let socket = TcpListener::bind("0.0.0.0:8080")?;

    // Lock down: only stdio + network after this
    pledge(&[Pledge::Stdio, Pledge::Inet])?;

    // Any attempt to open files, fork, etc = SIGKILL
    loop {
        let conn = socket.accept()?;  // OK - inet pledged
        // fs::read("./secrets")      // Would kill process
    }
}
```

**Why:** Dead simple to use. Most programs need very few pledge categories. Instant defense-in-depth.

**Reference:** OpenBSD pledge(2), unveil(2)

---

## Medium Priority

### 4. Uniform Scheme Naming (Redox-inspired)

Everything accessible via uniform paths. Drivers register "schemes" they handle.

```
/scheme/pcie/0000:01:00.0/bar0      PCIe BAR0 mapping
/scheme/pcie/0000:01:00.0/config    PCIe config space
/scheme/usb/1-1.2/                  USB device
/scheme/usb/1-1.2/ep1               USB endpoint 1
/scheme/gpio/17                     GPIO pin 17
/scheme/i2c/0/0x50                  I2C device at addr 0x50
/scheme/net/eth0/                   Network interface
/scheme/block/nvme0n1               Block device
/scheme/block/nvme0n1p1             Partition
/scheme/display/0                   Display output
/scheme/input/keyboard0             Input device
```

**Implementation:**
```rust
/// Driver registers a scheme
pub fn scheme_register(name: &str) -> Result<SchemeHandle, Error>;

/// Wait for requests to the scheme
pub fn scheme_wait(handle: SchemeHandle) -> Result<SchemeRequest, Error>;

pub enum SchemeRequest {
    Open { path: String, flags: OpenFlags, respond: Responder },
    Read { handle: u64, buf_ptr: u64, len: u64, respond: Responder },
    Write { handle: u64, buf_ptr: u64, len: u64, respond: Responder },
    Close { handle: u64 },
    Stat { handle: u64, respond: Responder },
}
```

**Why:** Uniform interface for everything. User can `cat /scheme/gpio/17` to read a GPIO. Debugging becomes trivial. Fits microkernel model perfectly.

**Reference:** Redox OS schemes, Plan 9 namespaces

---

### 5. Resource Limits per Process (cgroups-lite)

Control resource consumption. Important for driver supervision - prevent runaway driver from killing system.

```rust
pub struct ResourceLimits {
    /// Max physical memory (bytes)
    pub memory_bytes: Option<u64>,
    /// Max CPU time (microseconds, then SIGXCPU)
    pub cpu_time_us: Option<u64>,
    /// Max CPU percentage (0-100)
    pub cpu_percent: Option<u8>,
    /// Max I/O bandwidth (bytes/sec)
    pub io_bandwidth: Option<u64>,
    /// Max open handles
    pub open_handles: Option<u32>,
    /// Max child processes
    pub child_processes: Option<u32>,
    /// Max IPC messages queued
    pub ipc_queue_depth: Option<u32>,
}

/// Set limits on a process (requires CAP_RLIMIT)
pub fn rlimit_set(pid: Pid, limits: &ResourceLimits) -> Result<(), Error>;

/// Get current usage
pub fn rlimit_usage(pid: Pid) -> Result<ResourceUsage, Error>;
```

**devd integration:**
```rust
// devd sets limits when spawning drivers
let limits = ResourceLimits {
    memory_bytes: Some(64 * 1024 * 1024),  // 64MB max
    cpu_percent: Some(10),                  // 10% CPU max
    open_handles: Some(256),
    ..Default::default()
};
rlimit_set(driver_pid, &limits)?;
```

**Why:** Fault isolation. Misbehaving driver can't DoS the system by consuming all memory/CPU.

**Reference:** Linux cgroups v2, FreeBSD rctl(8)

---

### 6. Packet Filter (BSD pf-inspired)

When we add networking, use pf-style rules, not netfilter.

```rust
pub enum PfAction {
    Pass,
    Block,
    Return,     // TCP RST or ICMP unreachable
}

pub struct PfRule {
    pub action: PfAction,
    pub direction: Direction,  // In, Out
    pub interface: Option<String>,
    pub proto: Option<Protocol>,
    pub src: Option<AddrMatch>,
    pub dst: Option<AddrMatch>,
    pub src_port: Option<PortMatch>,
    pub dst_port: Option<PortMatch>,
    pub flags: PfFlags,
}

// Clean, readable rules
// "block in on eth0 proto tcp to port 22"
// "pass out proto udp to port 53"
```

**Why:** pf syntax is readable by humans. netfilter/iptables is a disaster of chains, tables, and implicit ordering.

**Reference:** OpenBSD pf.conf(5), FreeBSD pf(4)

---

## Lower Priority (Future)

### 7. io_uring-Style Batched Syscalls

Our IPC rings already do this for driver communication. Could extend to general syscalls.

```rust
/// Submission queue entry
pub struct SqEntry {
    pub opcode: Opcode,
    pub fd: Handle,
    pub addr: u64,
    pub len: u64,
    pub user_data: u64,  // returned in completion
}

/// Completion queue entry
pub struct CqEntry {
    pub result: i64,
    pub user_data: u64,
}

// Submit multiple syscalls, get completions asynchronously
```

**Why:** Reduces syscall overhead for I/O-heavy workloads. Our ring buffers are similar.

**Reference:** Linux io_uring(7)

---

### 8. Safe Kernel Hooks (eBPF-lite)

Allow userspace to inject verified-safe code into kernel hot paths.

```rust
/// Hook points where user code can run
pub enum HookPoint {
    SyscallEntry(SyscallNumber),
    SyscallExit(SyscallNumber),
    IpcSend,
    IpcRecv,
    TaskSwitch,
    Interrupt(IrqNumber),
}

/// Register a verified program at a hook point
pub fn bpf_attach(hook: HookPoint, program: &BpfProgram) -> Result<(), Error>;
```

**Why:** Powerful for tracing, security policy, custom packet filtering. Complex to implement safely.

**Reference:** Linux eBPF, dtrace

---

## Implementation Order

1. **Pledge** - Simplest, immediate security benefit
2. **Unified Events** - Needed anyway, do it right from start
3. **Capsicum** - For driver isolation
4. **Resource Limits** - For devd supervision
5. **Scheme Naming** - When we refactor IPC/VFS
6. **pf** - When we add networking
7. **io_uring** - Optimization, later
8. **eBPF** - Complex, much later

---

## References

- FreeBSD Capsicum: https://wiki.freebsd.org/Capsicum
- OpenBSD pledge: https://man.openbsd.org/pledge
- BSD kqueue: https://www.freebsd.org/cgi/man.cgi?kqueue
- Redox OS: https://doc.redox-os.org/book/
- Linux io_uring: https://kernel.dk/io_uring.pdf
- OpenBSD pf: https://www.openbsd.org/faq/pf/
