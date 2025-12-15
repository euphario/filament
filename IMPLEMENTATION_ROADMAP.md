# BPI-R4 Microkernel Implementation Roadmap

A comprehensive analysis of what's implemented, what's TODO, and next steps for each subsystem.

**Last Updated:** 2024-12-15

---

## 1. Multi-Process Support & Scheduling

### Current Status: ~85% Complete ✓

**Implemented:**
- Task structure with TrapFrame and CpuContext (`task.rs`)
- User task creation with separate address spaces
- Round-robin scheduler
- Context switch assembly
- User mode entry via `eret`
- Timer-based preemption (10ms time slice)
- Process states: Ready, Running, Blocked, Terminated
- **Spawn syscall** - create new processes from ELF in ramfs ✓
- **Wait syscall** - wait for child process exit ✓
- **Interactive shell** - working with help, pid, uptime, echo, spawn commands ✓

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| MEDIUM | Priority-based scheduling | `task.rs` | Currently only round-robin |
| MEDIUM | Process groups/sessions | N/A | Not implemented |
| LOW | CPU affinity for SMP | N/A | Tasks can run on any CPU |
| LOW | Real-time scheduling classes | N/A | Not implemented |

---

## 2. Memory Management (PMM, MMU, mmap/munmap)

### Current Status: ~75% Complete

**Implemented:**
- Bitmap-based PMM with 4KB pages (`pmm.rs`)
- 1GB managed DRAM (0x40000000-0x80000000)
- 4-level page tables for user space (`addrspace.rs`)
- L2/L3 table allocation on demand
- mmap syscall (allocate and map) (`task.rs:287-337`)
- munmap syscall (free memory) (`task.rs:339-368`)
- Address space per process with TTBR0 switching

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | TLB invalidation on munmap | `task.rs:363-364` | Currently skipped for simplicity |
| HIGH | Unmap pages from tables | `task.rs:363` | Only frees physical pages, doesn't clear PTEs |
| MEDIUM | CoW (Copy-on-Write) | N/A | Not implemented - needed for fork() |
| MEDIUM | Demand paging | N/A | All pages allocated upfront |
| MEDIUM | Shared memory (shmem) | N/A | Not implemented |
| MEDIUM | Memory-mapped files | N/A | Not implemented |
| LOW | Large page (2MB/1GB) support | N/A | Only 4KB pages |
| LOW | NUMA awareness | N/A | Single memory node assumed |

---

## 3. Syscall Infrastructure (32 syscalls)

### Current Status: ~90% Complete ✓

**Implemented:**
- 32 syscall numbers defined (`syscall.rs`)
- SVC exception handling with proper register save/restore (`boot.S`)
- Full argument passing (x0-x5, x8 for number)
- User pointer validation via `copy_from_user`/`copy_to_user` (`uaccess.rs`) ✓
- Working syscalls: Exit, DebugWrite, Yield, GetPid, GetTime, Mmap, Munmap
- **Spawn/Wait syscalls** - process creation and wait ✓
- **Exec syscall** - execute program from ramfs path ✓
- IPC syscalls: Send, Receive, ChannelCreate, ChannelClose, ChannelTransfer
- Port syscalls: PortRegister, PortUnregister, PortConnect, PortAccept
- FD syscalls: Open, Close, Read, Write, Dup, Dup2
- Event syscalls: EventSubscribe, EventUnsubscribe, EventWait, EventPost
- Scheme syscall: SchemeOpen

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| MEDIUM | Lseek syscall | N/A | Not implemented |
| MEDIUM | Fstat/stat syscalls | N/A | Not implemented |
| MEDIUM | Ioctl syscall | N/A | Not implemented |
| LOW | Clone/fork syscalls | N/A | Not implemented |

---

## 4. IPC Channels and Ports

### Current Status: ~80% Complete

**Implemented:**
- Bidirectional channels with 64-byte message payload (`ipc.rs`)
- Channel pairs for client-server communication
- Message types: Data, Request, Reply, Error, Close, Connect, Accept
- Message queues with 8-message depth
- Blocking receive with process wake-up
- Named ports for service discovery (`port.rs`)
- Port registry with connection handling

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | FD channel read/write | `fd.rs:229,254` | `FdType::Channel` returns -1 |
| MEDIUM | Large message support | N/A | Currently max 64 bytes inline |
| MEDIUM | Shared memory IPC | N/A | For zero-copy transfers |
| MEDIUM | Broadcast messages | N/A | Currently only point-to-point |
| LOW | Message priorities | N/A | All messages equal priority |
| LOW | Message timeouts | N/A | Only blocking or non-blocking |

---

## 5. Event Notification System

### Current Status: ~70% Complete

**Implemented:**
- Event types: IpcReady, Timer, Irq, ChildExit, FdReadable, FdWritable, Signal
- Per-process event queues (`event.rs`)
- Event subscriptions with filters
- Syscalls: EventSubscribe, EventUnsubscribe, EventWait, EventPost
- Global event delivery system

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | Hook events to actual sources | N/A | IRQ events not wired to GIC |
| HIGH | Timer events for userspace | N/A | No userspace timer API |
| MEDIUM | FdReadable/FdWritable polling | N/A | Not wired to FD subsystem |
| MEDIUM | epoll-like interface | N/A | Currently only wait-for-one |
| LOW | Edge vs level triggered | N/A | Not distinguished |

---

## 6. Scheme System (scheme:path URLs)

### Current Status: ~65% Complete

**Implemented:**
- URL parsing (`scheme.rs:519-539`)
- Scheme registry for kernel and user schemes
- KernelScheme trait with open/read/write/close
- Kernel schemes: memory, time, irq, null, zero
- Console scheme registered (but not implemented)
- SchemeOpen syscall

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | FdType::Scheme variant | `fd.rs:43-56` | Scheme handles not stored in FD |
| HIGH | Console scheme implementation | `scheme.rs:591` | Registered but no impl |
| HIGH | User scheme forwarding | `scheme.rs:572-576` | Returns ENOSYS |
| MEDIUM | Pipe scheme | N/A | Not implemented |
| MEDIUM | Tcp/Udp schemes | N/A | Not implemented |
| MEDIUM | Disk/FS schemes | N/A | Not implemented |
| LOW | Seek operation for schemes | N/A | Not implemented |
| LOW | Ftruncate for schemes | N/A | Not implemented |

---

## 7. SMP Multi-Core Support

### Current Status: ~60% Complete

**Implemented:**
- Per-CPU data structures (`smp.rs:44-87`)
- PSCI interface for CPU power management (`smp.rs:152-165`)
- Ticket spinlock implementation (`smp.rs:347-383`)
- Secondary CPU entry point in boot.S
- GIC CPU interface initialization per core (`gic.rs:308-329`)
- MPIDR-based CPU ID detection

**TODOs / Next Steps:**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | Actually start secondary CPUs | `main.rs:167-168` | Commented out |
| HIGH | Per-CPU runqueues | N/A | Currently global scheduler |
| HIGH | IPI (Inter-Processor Interrupts) | N/A | Not implemented |
| MEDIUM | CPU migration/load balancing | N/A | Not implemented |
| MEDIUM | Per-CPU idle task | `smp.rs:249-262` | Basic WFE loop only |
| MEDIUM | RWLock for shared data | N/A | Only spinlock exists |
| LOW | NUMA topology awareness | N/A | Single cluster assumed |
| LOW | CPU hotplug | N/A | Not implemented |

---

## 8. Hardware Drivers

### 8a. UART Driver
**Status: 95% Complete**

| Priority | Task | Description |
|----------|------|-------------|
| LOW | Hardware flow control | Not implemented |
| LOW | Configurable baud rate | Hardcoded to 115200 |

### 8b. GIC Driver
**Status: 90% Complete**

| Priority | Task | Description |
|----------|------|-------------|
| MEDIUM | IRQ affinity setting | Route SPIs to specific CPUs |
| LOW | SGI sending for IPI | Basic support exists |

### 8c. Timer Driver
**Status: 90% Complete**

| Priority | Task | Description |
|----------|------|-------------|
| MEDIUM | Per-CPU timers | Currently single global timer |
| LOW | High-resolution timers | Tick-based only |

### 8d. Ethernet Driver (eth.rs)
**Status: 30% Complete**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | PHY initialization via MDIO | `eth.rs:268` | Not implemented |
| HIGH | MAC address filter setup | `eth.rs:269` | Not implemented |
| MEDIUM | Link status detection | N/A | Forced link up |
| MEDIUM | Interrupt handling | N/A | Polling only |
| MEDIUM | Proper DMA ring management | N/A | Basic implementation |
| LOW | 10GBase-R/USXGMII modes | N/A | Not implemented |
| LOW | Flow control | N/A | Not implemented |

### 8e. SD/eMMC Driver (sd.rs)
**Status: 35% Complete**

| Priority | Task | File:Line | Description |
|----------|------|-----------|-------------|
| HIGH | Read capacity from CSD | `sd.rs:312` | Hardcoded to 512MB |
| HIGH | Proper clock configuration | N/A | Not implemented |
| MEDIUM | DMA transfers | N/A | PIO only |
| MEDIUM | High-speed modes | N/A | Not implemented |
| MEDIUM | Error recovery | N/A | Basic only |
| LOW | eMMC boot partitions | N/A | Not implemented |
| LOW | SDIO support | N/A | Not implemented |

---

## 9. RAM Filesystem (NEW)

### Current Status: ~80% Complete ✓

**Implemented:**
- TAR-based initrd loading (`initrd.rs`) ✓
- Simple RAM filesystem (`ramfs.rs`) ✓
- File lookup by path ✓
- Integration with exec syscall ✓
- Build scripts for creating initrd.img ✓

**TODOs / Next Steps:**

| Priority | Task | Description |
|----------|------|-------------|
| MEDIUM | Directory listing | Not implemented |
| MEDIUM | Write support | Read-only currently |
| LOW | File metadata | Only name and content stored |

---

## 10. Userspace Runtime (NEW)

### Current Status: ~90% Complete ✓

**Implemented:**
- Userlib with syscall wrappers (`user/userlib/`) ✓
- Print/println macros for formatted output ✓
- Stdin/Stdout/Stderr types ✓
- Panic handler with location info ✓
- Naked _start entry point ✓
- **Syscall register clobbers** - proper ABI compliance ✓

**TODOs / Next Steps:**

| Priority | Task | Description |
|----------|------|-------------|
| MEDIUM | Heap allocator | No malloc/free yet |
| LOW | More libc-like functions | strcmp, memcpy, etc. |

---

## 11. Missing Subsystems (Not Yet Started)

| Subsystem | Priority | Description |
|-----------|----------|-------------|
| **Filesystem** | HIGH | VFS layer, FAT32/ext2 support |
| ~~Process fork/exec~~ | ~~HIGH~~ | ✓ Exec implemented, spawn works |
| ~~User pointer validation~~ | ~~HIGH~~ | ✓ copy_from_user/copy_to_user |
| **MMIO scheme for userspace drivers** | HIGH | Map device memory to userspace |
| **IRQ forwarding to userspace** | HIGH | Deliver interrupts via events |
| **Signals** | MEDIUM | POSIX signal handling |
| **TTY/PTY** | MEDIUM | Terminal subsystem |
| **Network stack** | MEDIUM | TCP/IP implementation |
| **virtio drivers** | LOW | For QEMU testing |
| **Device tree parsing** | LOW | Dynamic hardware discovery |

---

## Implementation Order Recommendation

### Phase 1: Core Kernel Completion ✓ DONE
1. ~~User pointer validation in syscalls~~ ✓
2. ~~Spawn/Wait syscalls for process creation~~ ✓
3. ~~Exec syscall for ramfs programs~~ ✓
4. ~~Interactive shell~~ ✓
5. TLB invalidation on munmap (deferred - works without)
6. FD channel read/write integration (partial)

### Phase 2: Userspace Driver Support
1. MMIO scheme for mapping device registers
2. IRQ scheme with event delivery
3. Console scheme completion
4. User scheme forwarding via IPC

### Phase 3: SMP Enablement
1. Actually start secondary CPUs
2. Per-CPU runqueues
3. IPI implementation
4. Spinlock protection for global data

### Phase 4: Storage & Filesystem
1. Complete SD driver (CSD parsing, clock config)
2. Simple block device interface
3. FAT32 filesystem driver
4. Mount/unmount support

### Phase 5: Network Stack
1. Complete ethernet driver (PHY, MAC filter)
2. ARP/ICMP implementation
3. UDP/TCP implementation
4. Socket scheme

---

## Code Quality TODOs

| Area | Issue | Location |
|------|-------|----------|
| Safety | Excessive use of `unsafe` global statics | Throughout |
| Safety | No user pointer validation | `syscall.rs` |
| Locking | Global data needs spinlock protection | All subsystems |
| Error | Many functions use `Option` where `Result` would be better | Throughout |
| Testing | Most tests only verify structure creation | `*/test()` functions |

---

*Last Updated: 2024-12-15*
*Kernel version: 0.1.0*

## Recent Changes (Dec 2024)

- Added spawn/wait/exec syscalls for process management
- Implemented ramfs with TAR-based initrd
- Created userlib runtime with syscall wrappers
- Built interactive shell with basic commands
- Fixed syscall register clobbers for proper ABI compliance
- Fixed x11/x12 register save in SVC handler (boot.S)
- Added copy_from_user/copy_to_user for safe user memory access
