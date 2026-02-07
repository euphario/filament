# Per-Bridge-Group Zero-Copy Networking

## Overview

The MT7988A SoC has a 5-port MT7531 gigabit switch connected to the CPU via a
single 10G USXGMII internal link (switch CPU port 6 → GMAC1). All frames from
all physical ports (P0–P4) arrive over this one pipe. The MTK special tag
(4 bytes inserted after source MAC at offset 12) is used to demux frames by
source port, enabling per-bridge-group frame separation without additional
copies.

## Architecture

### Component Roles

| Component | Role |
|-----------|------|
| **ethd** | Frame Engine driver. Enables MTK special tag on CPU port, parses RX tags for source port, strips tags when copying to pool, inserts tags on TX. Encodes group_id in CQE/SQE flags. |
| **switchd** | Bridge group manager. Models groups (br0, br1, ...) as sets of switch ports. Registers NET_BRIDGE_GROUP ports with devd. Pushes port→group mapping to ethd via sidechannel. |
| **ipd** | IP stack per bridge group. Filters CQEs by group_id (flags field). Sets group_id on TX SQEs. One ipd instance per bridge group. |
| **devd** | Spawns ipd for each NET_BRIDGE_GROUP port that goes Ready. |

### Data Flow (RX)

```
HW DMA ──► RX buffer slot        (hardware writes frame + MTK tag)
              │
         ethd reads bytes [12..16] from DMA buf = MTK special tag
         source_port = tag[3] & 0x07
         group_id = port_to_group[source_port]
              │
         pool_write: copy MACs [0..12] + payload [16..len]  (strip 4-byte tag)
              │
         post CQE with flags=group_id
              │
         ┌────┴────┐
         ▼         ▼
       ipd₀      ipd₁    filter CQE by flags, read pool data (0 extra copy)
```

**Copy count**: 1 CPU memcpy per frame (DMA buf → pool), same as before.
The copy is reshaped to strip the 4-byte tag: two memcpy calls (12 bytes +
payload) instead of one contiguous copy. Total bytes copied is the same.

### Data Flow (TX)

```
       ipd₀      ipd₁
         │         │
    submit SQE with flags=group_id, data in pool
         │         │
         └────┬────┘
              │
         ethd reads SQE, flags → group_id
         group_to_ports[group_id] → destination port mask
         copies from pool to TX buf with 4-byte MTK tag at offset 12
         submits to QDMA
              │
         HW DMA ──► wire
```

**Copy count**: 1 CPU memcpy per frame (pool → TX buf), same as before.
The copy is reshaped to insert the 4-byte tag: MACs (12B) + tag (4B) + payload.

### CQE-Flags Demux (Single DataPort)

All ipd instances share one DataPort (`net0`) and one pool. Group demux is
via the `IoCqe.flags` field (u16, previously unused):

- **RX**: ethd sets `cqe.flags = group_id` after parsing the MTK tag
- **ipd filter**: `if cqe.flags != my_group_id { skip }`
- **TX**: ipd sets `sqe.flags = group_id` so ethd knows the destination mask

For 2–3 bridge groups, every ipd sees every CQE and skips non-matching ones.
This is negligible overhead (reading a 16-byte CQE header). No DataPort or
ring structural changes are needed.

## MTK Special Tag Format

The MT7531 inserts a 4-byte tag after the source MAC address when
PORT_SPEC_TAG (bit 5 of the PVC register) is enabled on the CPU port (port 6).

```
Offset:  0         6        12      16
         ┌─────────┬────────┬───────┬─────────────────┐
         │ dst MAC │src MAC │MTK tag│ ethertype + payload
         └─────────┴────────┴───────┴─────────────────┘
                             4 bytes
```

### RX Tag (switch → CPU)

| Byte | Bits | Field |
|------|------|-------|
| 0 | [7:0] | TPID: 0=untagged, 1=0x8100 VLAN, 2=0x88A8 double-tag |
| 1 | [7:0] | Reserved |
| 2 | [7:0] | Reserved |
| 3 | [2:0] | Source port number (0–4 for user ports, 6 for CPU) |

### TX Tag (CPU → switch)

| Byte | Bits | Field |
|------|------|-------|
| 0 | [7:0] | TPID (usually 0x00 for untagged TX) |
| 1 | [7:0] | Reserved |
| 2 | [7:0] | Reserved |
| 3 | [5:0] | Destination port bitmask (bit N = send to port N) |

## Bridge Group Model

### Default Configuration

One bridge group `br0` containing all 5 user ports (P0–P4):

```
br0: ports = {P0, P1, P2, P3, P4}  →  port_mask = 0x1F
```

All traffic is in group 0. A single ipd instance handles all frames.

### Multi-Group Example

```
br0: ports = {P0, P1, P2, P3}  →  LAN bridge, group 0
br1: ports = {P4}               →  WAN, group 1
```

switchd registers `br0:` and `br1:` as NET_BRIDGE_GROUP ports. devd spawns
two ipd instances. Each ipd connects to `net0` but only processes CQEs
matching its group_id.

### Config API

```
devc switchd set group.br0 0,1,2,3     # LAN ports 0-3
devc switchd set group.br1 4           # WAN port 4
devc switchd get groups                 # Show all groups
devc switchd get ports                  # Show port→group mapping
```

## Key Design Decisions

### Why CQE flags instead of multiple DataPorts?

Multiple DataPorts would require separate pool allocations and ring buffers
per group, doubling memory usage. The CQE-flags approach reuses the single
existing DataPort. Each ipd reads 16-byte CQE headers and skips non-matching
ones — negligible overhead for ≤8 groups.

### Why strip/insert the tag in ethd instead of ipd?

ethd owns the DMA buffers and already performs one memcpy per frame (DMA →
pool on RX, pool → TX buf on TX). Reshaping these copies to strip/insert the
4-byte tag adds no additional copies. If ipd handled tag processing, it would
need a second copy or mutable access to the shared pool.

### Why PORT_SPEC_TAG on the CPU port only?

PORT_SPEC_TAG is a per-port PVC register setting. Enabling it only on the CPU
port (port 6) means the tag is inserted/consumed only on the CPU↔switch link.
Inter-port switch forwarding is unaffected. This matches Linux DSA behavior.

## Future Considerations

### Per-Group IP Configuration

Each ipd instance currently uses the same static/DHCP config. For WAN vs LAN
separation, each ipd will need independent IP configuration. The config API
already supports per-instance settings via the bus framework's config_get/set.

### Sidechannel Group Mapping Push

switchd should push updated port→group mappings to ethd via the
`NET_SET_GROUPS` sidechannel message whenever groups change. The message format
is defined in `ring.rs::side_msg::NET_SET_GROUPS`:
- payload[0..5] = port_to_group (5 bytes, port index → group id)
- payload[5..13] = group_to_ports (8 bytes, group id → port bitmask)

This is not yet wired: switchd doesn't have a DataPort connection to ethd.
A future enhancement will add this sidechannel path, either through the bus
framework or via a control channel.

### Per-Group ipd Identity

When multiple groups exist, each ipd instance needs to know its group_id.
Currently all ipd instances start with group_id=0. Options for passing
group_id:
1. **Bus path convention**: Parse from spawning port name (br0: → 0)
2. **Sidechannel query**: Ask ethd/switchd for group_id via QUERY_INFO
3. **Spawn argument**: Pass group_id through the bus framework spawn mechanism

Option 1 requires extending the bus framework to expose the spawning port
name to the driver. Option 2 works today but requires a query protocol.

### VLAN-Based Groups

Bridge groups can map to hardware VLANs in the MT7531. When a group is
created, switchd can program the corresponding VLAN table entries so the
switch hardware enforces group isolation at L2, preventing frames from
leaking between groups even before they reach the CPU.

### Multiple DataPorts (Scaling)

For >8 groups or high frame rates, the CQE-flags polling approach may become
a bottleneck. The next step would be per-group DataPorts, where each ipd
has its own ring and pool. This requires ethd to manage multiple DataPorts
and route frames to the correct one.

## Files Modified

| File | Changes |
|------|---------|
| `user/abi/src/lib.rs` | Added `NET_BRIDGE_GROUP` port subclass |
| `user/userlib/src/ring.rs` | Added `NET_SET_GROUPS` sidechannel message type |
| `user/driver/ethd/src/main.rs` | Enabled PORT_SPEC_TAG, MTK tag parse/strip on RX, tag insert on TX, port→group mapping, NET_SET_GROUPS handler |
| `user/driver/switchd/src/main.rs` | Bridge group model replacing per-port registration, group config API |
| `user/driver/ipd/src/main.rs` | CQE group_id filtering, SQE group_id tagging |
| `user/driver/ipd/src/device.rs` | SmolDevice and IpdTxToken carry group_id |
| `user/driver/devd/src/rules.rs` | Added NET_BRIDGE_GROUP → ipd spawn rule |
