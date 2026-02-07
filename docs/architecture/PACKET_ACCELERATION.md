# Packet Acceleration

## Overview

The MT7988A Frame Engine contains hardware packet acceleration that can forward
established flows at wire speed (10G+) without CPU involvement. The CPU handles
connection setup (SYN, ARP, DHCP, DNS) and programs the hardware; subsequent
packets in the same flow bypass the CPU entirely.

Four acceleration subsystems are available, layered from simplest to most complex:

| Subsystem | What It Does | CPU Role | Status |
|-----------|-------------|----------|--------|
| **Checksum Offload** | RX IP/TCP/UDP checksum validation | Per-packet flag check | Implemented in ethd |
| **PPE / HNAT** | L2/L3/L4 flow table, hardware NAT | Per-flow setup only | `PPE_BASE` defined, no driver |
| **WED** | Direct WiFi-Ethernet DMA bridge | Connection setup only | Stubbed in wifid |
| **TOPS** | Tunnel encap/decap on RISC-V MCU | Config only | Proprietary, no support |

## Frame Engine Internal Architecture

### PSE (Packet Switch Engine)

The PSE is an internal crossbar that routes packets between numbered ports.
Every DMA engine, MAC, and accelerator connects to the PSE. PPE flow table
entries specify a destination PSE port number to route packets.

```
PSE Port │ Device                          │ Direction
─────────┼─────────────────────────────────┼──────────────
  0      │ ADMA (Auxiliary DMA)            │ unused
  1      │ GDM1 — GMAC1 → MT7531 switch   │ LAN (P0-P4)
  2      │ GDM2 — GMAC2 → 10G SFP         │ WAN / SFP+
  3      │ PPE0                            │ flow offload
  4      │ PPE1                            │ flow offload
  5      │ QDMA TX                         │ CPU → wire
  6      │ QDMA RX                         │ wire → CPU
  7      │ DROP                            │ discard
  8      │ WDMA0 → WiFi band 0 (via WED)  │ WiFi
  9      │ WDMA1 → WiFi band 1 (via WED)  │ WiFi
 10      │ TDMA → TOPS                     │ tunnel offload
 12      │ PPE2                            │ flow offload
 13      │ WDMA2 → WiFi band 2 (via WED)  │ WiFi
 14      │ EIP-197 → crypto engine         │ IPsec
 15      │ GDM3 — GMAC3 → 10G SFP         │ SFP+
```

### DMA Engines

| Engine | Base Offset | Purpose |
|--------|-------------|---------|
| PDMA | `0x6900` | Packet DMA: CPU RX with HW LRO, multiple RX rings |
| QDMA | `0x4400` (TX) / `0x4500` (RX) | Queue DMA: CPU TX with HW QoS |
| WDMA | `0x4800` (0) / `0x4c00` (1) | Wireless DMA: WiFi via WED |
| GDMA | per-GMAC | GigaMAC DMA: connects GMACs to PSE, forwarding control |

### GDMA Forwarding Configuration

Each GMAC has a forwarding register that controls where ingress packets go.
By default, packets go to PDMA (CPU). To enable PPE offload, GDMA is
reconfigured to forward to PPE instead.

```
MTK_GDMA_FWD_CFG(gmac_id) = 0x500 + (gmac_id * 0x1000)

Default:  GDMA → PDMA (CPU receives all packets)
Offload:  GDMA → PPE  (PPE inspects, forwards or sends to CPU)
```

## PPE / HNAT (Packet Processing Engine)

### What It Does

PPE maintains a FOE (Flow Offload Engine) table in DMA-coherent memory.
When a packet enters PPE, it hashes the flow key (IP addresses, ports,
protocol) and looks up the FOE table. If the entry is BIND, PPE rewrites
the packet (NAT, MAC, VLAN) and routes it to the destination PSE port
without CPU involvement.

MT7988 has **three independent PPE instances** (PPE0, PPE1, PPE2) at PSE
ports 3, 4, and 12. Each has its own FOE table for parallel processing.

### Register Map

PPE registers are at `FE_BASE + 0x2000` (already defined as `PPE_BASE` in
`src/platform/mt7988/eth.rs`). Key registers:

| Offset | Register | Purpose |
|--------|----------|---------|
| `0x200` | `PPE_GLO_CFG` | Enable PPE, checksum drop, flow drop update |
| `0x204` | `PPE_FLOW_CFG` | Enable flow types (NAT, NAPT, bridge, IPv6, DS-Lite) |
| `0x208` | `PPE_IP_PROTO_CHK` | IP protocol whitelist (TCP, UDP) |
| `0x21c` | `PPE_TB_CFG` | Table size (16K entries), aging config, hash mode |
| `0x220` | `PPE_TB_BASE` | Physical address of FOE table in DMA memory |
| `0x224` | `PPE_TB_USED` | Number of used entries (read-only) |
| `0x228` | `PPE_BIND_RATE` | Packets-per-second threshold for auto-bind |
| `0x22c` | `PPE_BIND_LIMIT0` | Max bind entries (quarter/half full) |
| `0x230` | `PPE_BIND_LIMIT1` | Max bind entries (full/non-L4) |
| `0x234` | `PPE_KEEPALIVE` | Keepalive intervals (TCP/UDP) |
| `0x238` | `PPE_UNBIND_AGE` | Unbind aging: min packets + time delta |
| `0x23c` | `PPE_BIND_AGE0` | Bind aging: non-L4 and UDP deltas |
| `0x240` | `PPE_BIND_AGE1` | Bind aging: TCP and TCP-FIN deltas |
| `0x244` | `PPE_HASH_SEED` | Hash seed for FOE table lookup |
| `0x248` | `PPE_DEFAULT_CPU_PORT` | CPU port for packets that miss / need CPU |
| `0x320` | `PPE_CACHE_CTL` | Entry cache enable, flush, lock clear |
| `0x334` | `PPE_MIB_CFG` | Per-flow byte/packet counters enable |
| `0x338` | `PPE_MIB_TB_BASE` | MIB counter table base address |

### FOE Entry Lifecycle

PPE flow entries follow a state machine:

```
INVALID(0) ──► UNBIND(1) ──► BIND(2) ──► FIN(3, TCP only)
    ▲              │             │            │
    └──────────────┴─────────────┴────────────┘
                      aging timeout
```

1. **INVALID**: Entry is empty or expired.
2. **UNBIND**: PPE auto-creates entries for new flows and counts packets.
   When the flow exceeds the PPS threshold (`PPE_BIND_RATE`), PPE signals
   the CPU via an "unbind rate reached" event. The CPU does NOT need to
   explicitly create UNBIND entries -- PPE does this automatically.
3. **BIND**: CPU fills in NAT rewrite rules and destination PSE port. PPE
   now forwards matching packets at wire speed. No CPU involvement.
4. **FIN**: TCP FIN detected. Entry ages out via `BIND_AGE1` timer.

This means ipd only handles the first few packets of each flow. Once bound,
PPE handles all subsequent packets in hardware.

### FOE Entry Format

MT7988 uses NetSys v2/v3 entries (96 or 128 bytes). Each entry contains:

**IB1 (Instruction Byte 1, first 32 bits)**:

| Bits | Field | Description |
|------|-------|-------------|
| [31] | STATIC | Entry is static (never ages out) |
| [30] | UDP | 1=UDP, 0=TCP |
| [29:28] | STATE | 0=INVALID, 1=UNBIND, 2=BIND, 3=FIN |
| [27:23] | PACKET_TYPE | Flow type (see below) |
| [24] | BIND_TTL | TTL decrement on bind |
| [20] | BIND_VLAN_TAG | VLAN tag action |
| [19] | BIND_PPPOE | PPPoE encap/decap |
| [14:0] | BIND_TIMESTAMP | Last-access timestamp (15-bit) |

**Packet types**:

| Value | Type | Description |
|-------|------|-------------|
| 0 | IPV4_HNAPT | IPv4 NAT with port translation (TCP/UDP) |
| 1 | IPV4_ROUTE | IPv4 routing without port translation |
| 2 | BRIDGE | Layer 2 MAC-based forwarding |
| 3 | IPV4_DSLITE | DS-Lite (IPv4-in-IPv6 tunnel) |
| 4 | IPV6_ROUTE_3T | IPv6 3-tuple routing |
| 5 | IPV6_ROUTE_5T | IPv6 5-tuple routing |
| 7 | IPV6_6RD | 6RD (IPv6-in-IPv4 tunnel) |

**IB2 (after flow-specific data)**:

| Bits | Field | Description |
|------|-------|-------------|
| [12:9] | DEST_PORT | Destination PSE port (v2) |
| [19] | WDMA_WINFO | WiFi info valid (for WED offload) |
| [6:0] | QID | QoS queue ID |

**IPv4 HNAPT entry body** (between IB1 and IB2):

```
Original tuple:  src_ip(4) + dest_ip(4) + src_port(2) + dest_port(2)
NAT tuple:       src_ip(4) + dest_ip(4) + src_port(2) + dest_port(2)
```

**L2 header info** (after IB2):

```
dest_mac(6) + src_mac(6) + vlan1(2) + vlan2(2) + pppoe_id(2)
+ winfo(2) + w3info(4) + amsdu(4)   (WiFi-specific, v2+)
```

### FOE Table Sizing

```
Entries:      16,384  (MTK_PPE_ENTRIES = 1024 << 4)
Entry size:   96 bytes (v2) or 128 bytes (v3)
Table size:   1.5 MB (v2) or 2 MB (v3) per PPE instance
Hash mask:    0x3FFF
```

The table is allocated in DMA-coherent memory and its physical address is
written to `PPE_TB_BASE`. PPE accesses it directly via DMA.

### PPE Initialization Sequence

```
1.  Allocate FOE table (DMA-coherent, 16K entries)
2.  Clear table (memset 0)
3.  Write table base:     PPE_TB_BASE = phys_addr
4.  Configure table:      PPE_TB_CFG = entry_count | aging_flags | hash_mode
5.  Set protocol check:   PPE_IP_PROTO_CHK = TCP | UDP
6.  Enable cache:         PPE_CACHE_CTL = EN
7.  Enable flow types:    PPE_FLOW_CFG = IP4_NAT | IP4_NAPT | L2_BRIDGE | ...
8.  Set aging thresholds: PPE_UNBIND_AGE, PPE_BIND_AGE0, PPE_BIND_AGE1
9.  Set bind limits:      PPE_BIND_LIMIT0, PPE_BIND_LIMIT1
10. Set bind rate:        PPE_BIND_RATE = min_pps_for_bind
11. Enable PPE:           PPE_GLO_CFG = EN | IP4_L4_CS_DROP | FLOW_DROP_UPDATE
12. Set CPU port:         PPE_DEFAULT_CPU_PORT = 0
13. If accounting:        PPE_MIB_TB_BASE = mib_phys, PPE_MIB_CFG = EN
14. Set GDMA forwarding:  GDMA_FWD_CFG(gmac) → PPE instead of PDMA
```

Step 14 is what activates offload: GDMA starts forwarding ingress packets
to PPE instead of directly to the CPU.

### Per-Flow MIB Counters

When `PPE_MIB_CFG_EN` is set, PPE maintains per-entry byte and packet
counters in a separate MIB table. Read via serial interface:

```
PPE_MIB_SER_CR  = entry_index | START
PPE_MIB_SER_R0  = byte_count[31:0]
PPE_MIB_SER_R1  = byte_count[47:32] | packet_count[15:0]
PPE_MIB_SER_R2  = packet_count[39:16]
```

## WED (Wireless Ethernet Dispatch)

### What It Does

WED creates a direct hardware DMA path between WiFi (MT7996 via PCIe) and
the Ethernet Frame Engine. For established flows, packets move between WiFi
and Ethernet without any CPU copies.

WED operates by intercepting access to the WLAN DMA (WFDMA) register space.
It hooks into the WiFi radio's TX/RX DMA queues and can inject/extract
packets directly, routing them through the PSE to/from Ethernet GMACs.

### Hardware Data Path

```
WiFi-to-Ethernet (RX offload, zero CPU copies):

  WiFi Air → MT7996 Radio → WFDMA RX → WED → PSE (WDMA port)
                                                 │
                                          PPE flow lookup
                                          NAT rewrite
                                                 │
                                          PSE → GDMA → GMAC → Wire


Ethernet-to-WiFi (TX offload, zero CPU copies):

  Wire → GMAC → GDMA → PSE → PPE flow lookup → PSE (WDMA port)
                                                      │
                                               WED → WFDMA TX → MT7996 → WiFi Air
```

### Token Management

WED uses a shared buffer pool with token-based references for zero-copy:

- **Token pool**: ~64K pre-allocated RX buffers, each assigned a unique token ID
- **Token split**: CPU tokens (0..TOKEN_SIZE) vs WED hardware tokens (TOKEN_SIZE..+rx_npkt)
- **Zero-copy**: When WED forwards a packet, it passes the token (buffer reference) to the next stage. No memcpy.
- **Release**: Tokens are returned to the pool when the packet is consumed

```
Token ranges:
  CPU:  0 .. MT7996_TOKEN_SIZE-1         (software DMA operations)
  WED:  MT7996_TOKEN_SIZE .. +rx_npkt    (hardware offload operations)
  Total: rx_nbuf = 65536 buffers
```

### WED WO (Wireless Offload) Co-Processor

The WED block contains a dedicated microcontroller (WO) that handles
RX-side offload. It communicates with the host CPU via CCIF (Cross-Core
Interface) with 256-entry DMA rings and 1504-byte command buffers.

Firmware files: `mt7988_wo_0.bin`, `mt7988_wo_1.bin` (one per WED instance).

### RRO (Receive Reorder Offload)

RRO is tightly coupled to WED and offloads 802.11 packet reordering to
hardware. WiFi packets arrive out-of-order due to block acknowledgment;
RRO maintains per-session BA bitmap tables and reorders packets before
passing them to PPE.

MT7996 uses RRO V3:

| Queue | Purpose |
|-------|---------|
| RRO_BAND0/1/2 | Reordered data packets (per WiFi band) |
| MSDU_PAGE_BAND0/1/2 | MSDU aggregate page buffers |
| RRO_IND / RXDMAD_C | Reorder indication / completion |

Reorder indication includes reason codes:
- **NORMAL**: Packet correctly reordered
- **REPEAT**: Duplicate (dropped)
- **OLDPKT**: Before BA window (dropped unless fragment)

### WiFi FOE Entry Fields

When a PPE flow entry targets WiFi (DEST_PORT = PSE_WDMA0/1/2_PORT), the
L2 header info must include WiFi-specific fields:

- **WDMA index**: Which WiFi band (0/1/2)
- **Queue**: TX queue on the radio
- **WCID**: WiFi Client ID (identifies the specific client)
- **BSS index**: Which SSID/virtual AP

This means wifid must expose a lookup interface so pped can resolve a MAC
address to WCID/BSS/queue for FOE entry programming.

### WED Initialization Requirements

1. Load WO firmware (`mt7988_wo_0.bin`, `mt7988_wo_1.bin`)
2. Configure token pool (split between CPU and WED)
3. Register WiFi DMA rings with WED (`mtk_wed_device_attach`)
4. Set up RRO queues alongside WED
5. Configure WDMA ports in PSE

## TOPS (Tunnel Offload Processor Subsystem)

### What It Does

TOPS is a programmable RISC-V microcontroller inside the MT7988A that handles
tunnel encapsulation and decapsulation in hardware. It connects to the Frame
Engine via PSE port 10 (PSE_TDMA_PORT).

### Supported Protocols

| Protocol | Operation |
|----------|-----------|
| GRE | Generic Routing Encapsulation |
| VXLAN | Virtual Extensible LAN |
| L2TP | Layer 2 Tunneling Protocol |
| PPPoE | PPP over Ethernet |

### Data Path

```
Incoming tunnel (decap):
  WAN GMAC → PSE → TOPS (strip tunnel header) → PSE → PPE → output port

Outgoing tunnel (encap):
  Input port → PPE → PSE → TOPS (add tunnel header) → PSE → WAN GMAC
```

### Status

TOPS firmware and driver are proprietary (MediaTek OpenWrt SDK only). No
upstream Linux support. Not relevant unless VPN/tunnel performance becomes
critical. The crypto engine (EIP-197, PSE port 14) handles IPsec separately.

## Architecture: Putting It Together

### Without Offload (Current)

Every forwarded packet hits the CPU twice (RX + TX):

```
WAN → GMAC2 → PDMA RX → ethd → pool → ipd (NAT) → pool → ethd → QDMA TX → GMAC1 → LAN
```

At 10G line rate (~14.8M packets/sec for 64-byte frames), CPU-based
forwarding tops out around 2-4 Gbps on 4x A73 cores.

### With PPE Offload

CPU handles connection setup. PPE handles established flows at wire speed:

```
First packets (UNBIND):
  WAN → GMAC2 → PPE → no match → CPU → ipd (NAT, conntrack) → LAN
                                    ↓
                              pped programs FOE entry
                                    ↓
Subsequent packets (BIND):
  WAN → GMAC2 → PPE → match → NAT rewrite in HW → GMAC1 → LAN
                       (CPU never sees these packets)
```

### With PPE + WED (Full Offload)

WiFi-to-Ethernet and Ethernet-to-WiFi flows are fully hardware-accelerated:

```
WiFi client → WFDMA → WED → PSE → PPE (BIND) → GMAC → WAN
                                    ↑
                              zero CPU copies

WAN → GMAC → PPE (BIND) → PSE → WDMA → WED → WFDMA → WiFi client
                                    ↑
                              zero CPU copies
```

### Component Responsibilities

```
                    ┌─────────────────────────────┐
                    │     PPE FOE Table (HW)       │
                    │  16K entries per instance     │
                    │  Wire-speed NAT + forwarding  │
                    └──────────┬──────────────────┘
                               │ programs
            ┌──────────────────┼──────────────────────┐
            │                  │                       │
     ┌──────▼──────┐    ┌─────▼──────┐    ┌──────────▼────┐
     │   ipd (br0) │    │  ipd (br1) │    │   ipd (wifi)  │
     │   LAN stack │    │  WAN stack │    │  WiFi stack   │
     │   Handles:  │    │  Handles:  │    │  Handles:     │
     │   ARP, DHCP │    │  NAT setup │    │  ARP, DHCP   │
     │   new flows │    │  conntrack │    │  new flows   │
     └──────┬──────┘    └─────┬──────┘    └──────┬────────┘
            │                  │                   │
            │          ┌──────▼──────┐             │
            │          │    pped     │             │
            │          │  FOE table  │             │
            │          │  management │             │
            │          └─────────────┘             │
            │                                      │
     ┌──────▼──────────────────────────────────────▼──┐
     │              ethd (Frame Engine driver)         │
     │   GDMA config │ MTK tag │ bridge groups        │
     └────────────────────────────────────────────────┘
```

| Component | Responsibility |
|-----------|---------------|
| **ipd** | IP stack per bridge group. Handles ARP, DHCP, DNS, TCP handshake, connection tracking. Notifies pped when flows are established. |
| **pped** | PPE flow table manager. Receives flow notifications from ipd, programs FOE entries, monitors aging/expiry. Handles GDMA forwarding config. |
| **ethd** | Frame Engine driver. Configures GDMA, manages DMA rings. With PPE active, only handles packets that PPE sends to CPU (misses, exceptions). |
| **wifid** | WiFi driver. With WED, registers DMA rings for hardware offload. Provides WCID/BSS lookup for pped to program WiFi flow entries. |

### pped Design

pped is a bus framework driver that manages the PPE hardware:

1. **Init**: Allocate FOE table in DMA memory, configure PPE registers,
   set GDMA to forward to PPE
2. **Flow bind**: Receive notification from ipd (via sidechannel or config
   API) that a flow is established. Build FOE entry with NAT rewrite rules
   and destination PSE port. Commit to table.
3. **Aging**: Monitor `PPE_TB_USED` and per-entry timestamps. Clean up
   expired entries. Notify ipd when flows expire.
4. **WiFi flows**: Query wifid for WCID/BSS/queue given a destination MAC.
   Set DEST_PORT to PSE_WDMA0/1/2_PORT and fill WiFi-specific fields.
5. **Stats**: Read per-flow MIB counters for monitoring.

### Inter-Process Communication

```
ipd ──(sidechannel)──► pped    "flow established: 1.2.3.4:80 → 10.0.0.5:54321, NAT to WAN_IP:12345"
pped ──(MMIO)──────► PPE       programs FOE entry, sets BIND state
pped ──(sidechannel)──► ipd    "flow expired: entry 1234"  (ipd cleans up conntrack)
pped ──(sidechannel)──► wifid  "resolve MAC aa:bb:cc:dd:ee:ff"  (gets WCID/BSS for WiFi flows)
```

## Key Design Decisions

### Why a separate pped daemon?

PPE is a shared hardware resource that spans all bridge groups and
interfaces. Putting PPE management in ethd would overload an already
complex driver. Putting it in ipd would require cross-group coordination.
A dedicated pped daemon owns the FOE table and mediates between all ipd
instances and interface drivers.

### Why PPE auto-learning (UNBIND) instead of explicit flow creation?

PPE hardware automatically creates UNBIND entries for new flows and counts
packets. When the rate exceeds the threshold, it notifies the CPU. This is
more efficient than having ipd explicitly create entries for every new
connection -- most short-lived flows never reach the threshold and don't
waste FOE table space.

### Why three PPE instances?

MT7988 has three PPE instances for parallel processing. Each GMAC can be
configured to forward to a different PPE. This distributes the hash lookup
load across instances, improving throughput for high packet rates.

### Why not implement PPE in the kernel?

PPE is a userspace-accessible resource (MMIO-mapped DMA memory). The
microkernel philosophy keeps device management in userspace. pped maps
the PPE registers and FOE table via the bus framework's MMIO support,
just like ethd maps Frame Engine registers.

## Implementation Roadmap

### Phase 1: Software NAT (prerequisite)

IP forwarding and NAT in ipd between bridge groups. This establishes the
connection tracking and NAT rewrite logic that pped will later offload.

### Phase 2: PPE Offload (biggest performance win)

1. Write pped driver: allocate FOE table, configure PPE registers
2. Set GDMA forwarding to PPE
3. Handle "unbind rate reached" events: query ipd conntrack, build FOE entry
4. Implement aging and flow expiry notifications
5. Result: established TCP/UDP flows forwarded at wire speed

### Phase 3: WED Integration (WiFi fast path)

1. Load WO firmware in wifid
2. Configure token pool split (CPU vs WED)
3. Register WFDMA rings with WED
4. Enable WDMA ports in PSE
5. pped programs WiFi flow entries with WCID/BSS fields
6. Result: WiFi-to-Ethernet forwarding without CPU copies

### Phase 4: TOPS (optional, tunnel offload)

Only if VPN/tunnel performance becomes critical. Requires proprietary
firmware. Lower priority than PPE and WED.

## Linux Reference Code

Key source files in the `linux/` directory for reverse-engineering register
formats and initialization sequences:

| File | Contents |
|------|----------|
| `linux/drivers/net/ethernet/mediatek/mtk_ppe_regs.h` | PPE register offsets and bit definitions |
| `linux/drivers/net/ethernet/mediatek/mtk_ppe.h` | FOE entry structures, state enum, hash functions |
| `linux/drivers/net/ethernet/mediatek/mtk_ppe.c` | PPE init, FOE table management, entry commit |
| `linux/drivers/net/ethernet/mediatek/mtk_ppe_offload.c` | nf_flow_table to PPE bridge, flow programming |
| `linux/drivers/net/ethernet/mediatek/mtk_eth_soc.h` | PSE port enum, GDMA registers, SoC constants |
| `mt76/wed.c` | WED buffer management, token pool |
| `mt76/mt7996/mmio.c` | WiFi-side WED init (`mt7996_mmio_wed_init`) |
| `mt76/mt7996/dma.c` | WED/RRO DMA queue setup |
