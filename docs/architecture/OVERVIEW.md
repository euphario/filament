# Architecture Overview

## System Layers

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Applications                                    │
│                         ls, cp, cat, shell, ...                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │ IPC
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Policy Daemons                                     │
│                                                                             │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐                  │
│   │   fsd   │    │  netd   │    │  authd  │    │   ...   │                  │
│   │  files  │    │ network │    │  users  │    │         │                  │
│   └────┬────┘    └────┬────┘    └────┬────┘    └─────────┘                  │
│        │              │              │                                       │
└────────┼──────────────┼──────────────┼───────────────────────────────────────┘
         │ IPC          │ IPC          │ IPC
┌────────┴──────────────┴──────────────┴───────────────────────────────────────┐
│                              Drivers                                         │
│                                                                             │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐                  │
│   │  fatfs  │    │   eth   │    │  nvme   │    │  wlan   │                  │
│   └────┬────┘    └────┬────┘    └────┬────┘    └────┬────┘                  │
│        │              │              │              │                        │
└────────┼──────────────┼──────────────┼──────────────┼────────────────────────┘
         │ IPC          │ IPC          │ IPC          │ IPC
┌────────┴──────────────┴──────────────┴──────────────┴────────────────────────┐
│                         devd (PID 1)                                         │
│                                                                             │
│   - Hardware enumeration                                                     │
│   - Driver lifecycle                                                         │
│   - Device state machines                                                    │
│   - Fault tracking & escalation                                              │
│   - Publishes /hw/* and /svc/*                                               │
│                                                                             │
└──────────────────────────────────┬───────────────────────────────────────────┘
                                   │ Bus Control Ports
┌──────────────────────────────────┴───────────────────────────────────────────┐
│                              KERNEL                                          │
│                                                                             │
│   Provides:                      Owns:                                       │
│   - IPC (ports, channels)        - Bus safety (bus mastering)                │
│   - Memory (virtual, physical)   - Nuclear reset option                      │
│   - Scheduling                   - Process lifecycle                         │
│   - Interrupt routing            - Capability enforcement                    │
│                                                                             │
│   Does NOT contain:                                                          │
│   - Filesystem logic             - Network stack                             │
│   - Device drivers               - Policy decisions                          │
│                                                                             │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Example: WiFi Driver

```
1. Kernel boots, reads DTB → knows about pcie0, pcie1, usb0, usb1, etc.

2. devd starts, spawns pcied for PCIe buses

3. pcied:
   • Initializes PCIe MAC, brings up links
   • Enumerates: finds MT7996 at BAR0=0x30200000, size=2MB
   • Configures each device: allocates BAR0, enables MEM_SPACE + BUS_MASTER
   • Registers devices with devd (via IPC)
   • Listens for FindDevice queries

4. wifid starts:
   • Connects to pcied
   • Calls FindDevice(vendor=0x14c3, device=0x7990)
   • Gets: {bar0_addr: 0x30200000, bar0_size: 2MB, ...}

5. wifid maps device memory:
   • Calls syscall::mmap_device(0x30200000, 2MB)
   • Kernel checks MMIO capability, maps with device attributes
   • Returns virtual address 0x50000000

6. wifid accesses registers:
   • Writes to 0x50000000 + offset → hardware access
```

## Component Responsibilities

| Component | Responsibility |
|-----------|---------------|
| **Kernel** | Expose bus list from DTB, provide mmap_device syscall |
| **devd** | Supervise bus drivers, maintain device database, track states |
| **pcied** | Enumerate PCIe, configure BARs, enable bus master, answer FindDevice |
| **usbd** | Enumerate USB, configure endpoints, answer FindDevice |
| **Driver** | Query bus driver for device info, mmap and use device |

## Key Principles

### Kernel Stays Minimal
No device-specific code, just raw capabilities. The kernel knows about buses but not devices.

### Single Source of Truth
devd knows all devices, controls access. Queries about "what devices exist" go through devd.

### Bus Drivers Are Experts
pcied knows PCI, usbd knows USB. They handle bus-specific logic completely.

### Drivers Are Independent
Only need IPC to bus driver + mmap syscall. No kernel involvement for normal operation.

### Easy to Add Buses
New bus = new userspace driver, no kernel changes needed.

### Supervision Works
devd can restart crashed drivers, reset buses. Hardware faults are recoverable.

## See Also

- [STATE-MACHINES.md](STATE-MACHINES.md) - Bus, device, driver state machines
- [MICROKERNEL.md](MICROKERNEL.md) - Microkernel philosophy
- [../PRINCIPLES.md](../PRINCIPLES.md) - Design principles
