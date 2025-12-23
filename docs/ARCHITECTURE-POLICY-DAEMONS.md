# Policy Daemon Architecture

*Design notes for future implementation*

## Overview

The microkernel stays minimal - just IPC, memory, scheduling. All policy lives in userspace daemons, each owning a vertical domain.

```
                    ┌─────────────────────────────────────────┐
                    │              User Commands              │
                    │  ls, cp, cat, ifconfig, ping, mount...  │
                    └──────────────┬──────────────────────────┘
                                   │ IPC
                    ┌──────────────┴──────────────────────────┐
                    │          Policy Daemons                 │
                    │  ┌───────┐  ┌───────┐  ┌───────┐        │
                    │  │  fsd  │  │  netd │  │  ...  │        │
                    │  │(files)│  │ (net) │  │       │        │
                    │  └───┬───┘  └───┬───┘  └───────┘        │
                    └──────┼──────────┼───────────────────────┘
                           │ IPC      │ IPC
                    ┌──────┴──────────┴───────────────────────┐
                    │              Drivers                    │
                    │  ┌───────┐  ┌───────┐  ┌───────┐        │
                    │  │ fatfs │  │  eth  │  │ wlan  │        │
                    │  └───┬───┘  └───┬───┘  └───┬───┘        │
                    └──────┼──────────┼──────────┼────────────┘
                           │          │          │
                    ┌──────┴──────────┴──────────┴────────────┐
                    │              devd (PID 1)               │
                    │  - Hardware detection                   │
                    │  - Driver lifecycle                     │
                    │  - Spawns policy daemons                │
                    └─────────────────────────────────────────┘
                                   │
                    ┌──────────────┴──────────────────────────┐
                    │              Kernel                     │
                    │  IPC, Memory, Scheduling                │
                    │  (minimal - no VFS, no network stack)   │
                    └─────────────────────────────────────────┘
```

## Key Principles

### 1. Policy Daemons Own Their Domain

Each policy daemon is the single authority for its domain:

- **fsd** (filesystem daemon): All file operations route through here
- **netd** (network daemon): All network configuration and routing
- **devd**: Hardware facts + driver lifecycle (already exists)

### 2. Commands Are IPC Clients

Traditional Unix commands become thin IPC clients:

| Command | Talks to | Message |
|---------|----------|---------|
| `ls /usb/photos` | fsd | `LIST /usb/photos` |
| `cp a.txt b.txt` | fsd | `COPY /path/a.txt /path/b.txt` |
| `ifconfig eth0` | netd | `GET_INTERFACE eth0` |
| `ping 8.8.8.8` | netd | `PING 8.8.8.8` |

Benefits:
- Commands are simple, stateless
- Policy daemon handles permissions, quotas, logging
- Same command works across local/remote/virtual filesystems

### 3. No Kernel VFS

Traditional Unix:
```
App → syscall → VFS → filesystem driver → block device
         ↑
    mount table in kernel
```

This design:
```
App → IPC → fsd → IPC → fatfs → IPC → usbd
              ↑
    routing table in fsd (userspace)
```

"Mounting" = fsd learns about a new filesystem backend. No kernel involvement.

### 4. Structured Data, Not Text

Unlike /proc and /dev which are text files you grep/parse:

- IPC messages are structured (binary or well-defined format)
- Shell can *format* for display, programs don't *parse*
- Object-oriented shell is an option (like PowerShell)
- Or: shell has two modes (human-readable vs structured output)

## fsd (Filesystem Daemon)

Responsibilities:
- Unified namespace across all storage
- Routes requests to appropriate backend (fatfs, ramfs, netfs)
- Permissions and access control
- Caching policy
- Watches and notifications

Backends register with fsd:
```
fatfs: "I handle /usb/*"
ramfs: "I handle /tmp/*"
netfs: "I handle /net/*"
```

## netd (Network Daemon)

Responsibilities:
- Interface configuration
- IP assignment (static, DHCP)
- Routing tables
- Firewall rules
- Policy-based configuration

Example policies:
- "If ethernet connected with IP, put WiFi in AP mode + bridge"
- "If no ethernet, connect WiFi to network A"
- "At location X, use profile Y"

## Reactive/Event-Driven

Events flow up:
```
USB inserted → usbd → devd → fsd → "new filesystem available"
Ethernet link up → eth driver → netd → evaluate policies → configure
```

State queries when needed:
```
"Is eth0 connected with IP?" → netd → yes/no
```

Rules/policies trigger actions:
```
WHEN eth0.connected AND eth0.has_ip THEN wlan0.mode = AP
```

## Contrast with systemd

systemd approach: Absorb everything into PID 1
- Service management
- Logging
- Device management
- Network configuration
- DNS resolution
- Time sync
- ...

This approach: PID 1 spawns focused policy daemons
- devd: hardware + drivers
- fsd: filesystem policy
- netd: network policy
- Each daemon does one thing well
- Clear boundaries and interfaces

## Implementation Path

Already have:
- [x] devd (supervisor, driver lifecycle)
- [x] IPC (ports/channels)
- [x] Drivers (usbd, fatfs, shell)

Next steps:
- [ ] fsd skeleton - route file operations
- [ ] Shell commands as IPC clients (ls, cat, cp)
- [ ] netd skeleton - interface management
- [ ] Event subscription mechanism
- [ ] Policy rules engine (simple pattern matching first)

## Open Questions

1. **Shell design**: Pure IPC client? Two modes (text/structured)? Object-oriented?

2. **Policy language**: How do users express rules? Config file? DSL? Compiled-in for v1?

3. **Namespacing**: How does fsd present a unified namespace? Path-based routing? Explicit prefixes?

4. **Error handling**: How do errors propagate through the IPC chain?

5. **Performance**: Acceptable overhead for file ops through IPC? Probably fine for embedded use case.
