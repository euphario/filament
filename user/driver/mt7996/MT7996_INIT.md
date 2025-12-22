# MT7996/MT7990 WiFi Initialization Reference

This document details the initialization sequence from the Linux mt76 driver, focusing on what we need to implement for firmware loading.

## Device Variants

| Device ID | Chip     | Bands | Notes |
|-----------|----------|-------|-------|
| 0x7990    | MT7996   | 3     | WiFi 7, tri-band |
| 0x7991    | MT7996   | 3     | Alt ID |
| 0x7992    | MT7992   | 2     | WiFi 7, dual-band |
| 0x799a    | MT7992   | 2     | Alt ID |
| 0x7993    | MT7990   | 2     | WiFi 7, dual-band, simplified |
| 0x799b    | MT7990   | 2     | Alt ID |

**Our device**: MT7990 (0x7991 on pcie0, 0x7991 on pcie1)

---

## Register Access and Address Remapping

### Base Addresses

| Region | Physical Address | BAR Offset | Size |
|--------|------------------|------------|------|
| WFDMA0 | 0x54000000 | 0xd4000 | varies |
| WFDMA1 | 0x55000000 | 0xd5000 | varies |
| WFDMA0_PCIE1 | 0x58000000 | 0xd8000 | varies |
| CONN_INFRA | 0x7c000000 | remap | varies |
| WF_MCU_SYSRAM | 0x00400000 | 0x80000 | varies |

### HIF Remap Registers

For addresses outside the static map, dynamic remapping is used.

#### L1 Remap (MT7990)
- **Register**: `HIF_REMAP_L1` = `CONN_BUS_CR_VON_BASE` (0x155000) + offset
- **MT7990 offset**: 0x8 → `HIF_REMAP_L1` = 0x155008
- **Remap base in BAR**: `HIF_REMAP_BASE_L1` = 0x40000
- **Mask**: Lower 18 bits (256KB granularity)

```
To access addr >= 0x40000:
  offset = addr & 0x3FFFF              // Lower 18 bits
  base = (addr & 0xFFFC0000) >> 18     // Upper bits, shifted
  write base to HIF_REMAP_L1
  read from HIF_REMAP_BASE_L1 + offset
```

#### CBTOP Remap (MT7990, for 0x70000000+ addresses)
- **Register**: `HIF_REMAP_CBTOP` = 0x1f6554
- **Remap base in BAR**: `HIF_REMAP_BASE_CBTOP` = 0x1c0000
- **Mask**: Lower 16 bits (64KB granularity)

```
To access addr in 0x70000000 range:
  offset = addr & 0xFFFF               // Lower 16 bits
  base = (addr >> 16) & 0xFFFF         // Upper 16 bits
  write base to HIF_REMAP_CBTOP
  read from HIF_REMAP_BASE_CBTOP + offset
```

**Example**: `WF_SUBSYS_RST` = 0x70028600
```
  base = 0x7002
  offset = 0x8600
  write 0x7002 to 0x1f6554
  access 0x1c0000 + 0x8600 = 0x1c8600
```

---

## WFDMA Registers (at WFDMA0_BASE = 0xd4000)

### Core Registers

| Register | Offset | Description |
|----------|--------|-------------|
| RST | 0x100 | Reset control |
| BUSY_ENA | 0x13c | Busy enable |
| MCU_CMD | 0x1f0 | MCU command/status |
| INT_SRC | 0x200 | Interrupt source |
| INT_MASK | 0x204 | Interrupt mask |
| GLO_CFG | 0x208 | Global config |
| RST_DTX_PTR | 0x20c | Reset DTX pointer |
| GLO_CFG_EXT0 | 0x2b0 | Extended config 0 |
| GLO_CFG_EXT1 | 0x2b4 | Extended config 1 |

### RST Register (0xd4100)

| Bit | Name | Description |
|-----|------|-------------|
| 4 | RST_LOGIC_RST | Logic reset |
| 5 | RST_DMASHDL_ALL_RST | DMA scheduler reset |

**CRITICAL**: When RST bits are SET (0x30), DMA is in reset state and cannot process descriptors.

### GLO_CFG Register (0xd4208)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | TX_DMA_EN | Enable TX DMA |
| 2 | RX_DMA_EN | Enable RX DMA |
| 21 | OMIT_RX_INFO_PFET2 | Omit RX info PFET2 |
| 26 | EXT_EN | External enable (required!) |
| 27 | OMIT_RX_INFO | Omit RX info |
| 28 | OMIT_TX_INFO | Omit TX info |

### BUSY_ENA Register (0xd413c)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | TX_FIFO0 | TX FIFO 0 busy enable |
| 1 | TX_FIFO1 | TX FIFO 1 busy enable |
| 2 | RX_FIFO | RX FIFO busy enable |

### GLO_CFG_EXT0 (0xd42b0)

| Bit | Name | Description |
|-----|------|-------------|
| 14 | WED_MERGE_MODE | WED merge mode |
| 18 | RX_WB_RXD | RX writeback RXD |
| 24-27 | OUTSTAND_MASK | Outstanding mask |

### GLO_CFG_EXT1 (0xd42b4)

| Bit | Name | Description |
|-----|------|-------------|
| 28 | TX_FCTRL_MODE | TX flow control mode |
| 31 | CALC_MODE | Calculation mode |

---

## Ring Registers

### TX MCU Ring Base
- **Base**: WFDMA0 + 0x300
- **Per-ring size**: 0x10 bytes

| Queue | Offset from WFDMA0 |
|-------|-------------------|
| WM (0) | 0x300 |
| WA (1) | 0x310 |
| FWDL (2) | 0x320 |

### RX Ring Base
- **Base**: WFDMA0 + 0x500
- **Per-ring size**: 0x10 bytes

| Queue | Offset from WFDMA0 |
|-------|-------------------|
| MCU_WM (0) | 0x500 |
| MCU_WA (1) | 0x510 |

### Ring Register Layout (per ring, 0x10 bytes)

| Offset | Register | Description |
|--------|----------|-------------|
| 0x00 | BASE_LO | Base address (low 32 bits) |
| 0x04 | MAX_CNT | Maximum count (ring size) |
| 0x08 | CPU_IDX | CPU index (write pointer) |
| 0x0C | DMA_IDX | DMA index (read pointer) |
| 0x10 | BASE_HI | Base address (high 32 bits) |

### Prefetch Control
- **Base**: WFDMA0 + 0x600
- **Per-queue size**: 4 bytes

| Queue | Offset | Depth (MT7990) |
|-------|--------|----------------|
| FWDL (2) | 0x608 | 4 |

---

## Queue IDs and Sizes

### TX Queue IDs
| Queue | ID | Ring Size |
|-------|-----|-----------|
| FWDL | 16 | 128 |
| MCU_WM | 17 | 256 |
| BAND0 | 18 | 2048 |
| BAND1 | 19 | 2048 |
| MCU_WA | 20 | 256 |
| BAND2 | 21 | 2048 |

### RX Queue IDs
| Queue | ID | Ring Size |
|-------|-----|-----------|
| MCU_WM | 0 | 512 |
| MCU_WA | 1 | 1024 |

**IMPORTANT**: For MCU ring configuration, the ring index in hardware is different from the queue ID enum!
- FWDL uses ring index 2 at offset 0x320
- MCU_WM uses ring index 0 at offset 0x300
- RX MCU_WM uses ring index 0 at offset 0x500

---

## Initialization Sequence (from Linux driver)

### Complete Sequence from PCI Probe to Firmware Loading

```
mt7996_pci_probe()
  │
  ├── PCI setup (enable, map BARs, DMA masks)
  │
  ├── mt7996_mmio_probe()
  │     → Allocate mt7996_dev
  │     → Setup register mappings
  │     → Detect chip variant (MT7996/MT7992/MT7990)
  │
  ├── mt7996_wfsys_reset()    ← WIRELESS SUBSYSTEM RESET (at probe level!)
  │     → Set bit 0 of WF_SUBSYS_RST
  │     → Wait 20ms
  │     → Clear bit 0 of WF_SUBSYS_RST
  │     → Wait 20ms
  │
  ├── HIF2, NPU, WED, IRQ setup
  │
  └── mt7996_register_device()
        │
        └── mt7996_init_hardware()
              │
              ├── Clear interrupt source (INT_SOURCE_CSR = ~0)
              │
              ├── mt7996_dma_init()         ← DMA INIT FIRST!
              │     → Configure queue mappings
              │     → mt7996_dma_disable(reset=true)  // RST pulse
              │     → Initialize TX queues (FWDL, MCU_WM, MCU_WA, data)
              │     → Initialize RX queues (MCU, data)
              │     → mt7996_dma_enable()
              │
              ├── Set MT76_STATE_INITIALIZED
              │
              └── mt7996_mcu_init()         ← FIRMWARE LOADING SECOND!
                    │
                    └── mt7996_mcu_init_firmware()
                          ├── Force normal mode via SWDEF_MODE
                          ├── mt7996_driver_own() - claim from firmware
                          └── mt7996_load_firmware()
                                ├── Check FW_STATE == FW_DOWNLOAD (state 1)
                                ├── mt7996_load_patch() - load ROM patch
                                ├── mt7996_load_ram() - load WM, DSP, WA
                                └── Wait for FW_STATE == FW_RDY (state 7)
```

### Key Insight: Order of Operations

1. **wfsys_reset** happens at probe time (before DMA init)
2. **DMA init** happens before firmware loading
3. **Firmware loading** uses the DMA rings to send commands

---

## Driver Ownership (TOP registers at 0xe0000)

| Register | Offset | Description |
|----------|--------|-------------|
| LPCR_HOST_BAND | 0x10 | Low-power control |
| LPCR_HOST_BAND_IRQ_STAT | 0x14 | IRQ status |
| MISC | 0xf0 | Misc (contains FW state) |

### LPCR_HOST_BAND bits
| Bit | Name | Description |
|-----|------|-------------|
| 0 | FW_OWN | FW owns the device |
| 1 | DRV_OWN | Driver owns the device |
| 2 | FW_OWN_STAT | FW ownership status |

### Firmware State (MISC bits 2:0)
| Value | State |
|-------|-------|
| 0 | Initial |
| 1 | FW Download |
| 2 | Normal Operation |
| 3 | Normal TRX |
| 7 | Ready |

---

## Firmware Loading Commands

### MCU Command Format (McuTxd)
- 32-byte TXD header
- Variable MCU command fields

### Commands for Firmware Download

| Command | ID | Description |
|---------|-----|-------------|
| TARGET_ADDRESS_LEN_REQ | 0x01 | Set download address/length |
| FW_START_REQ | 0x02 | Start firmware execution |
| PATCH_START_REQ | 0x05 | Start patch |
| PATCH_FINISH_REQ | 0x07 | Finish patch loading |
| PATCH_SEM_CTRL | 0x11 | Patch semaphore control |
| FW_SCATTER | 0xEE | Send firmware data chunk |

### Patch Loading Sequence
```
1. Acquire patch semaphore (PATCH_SEM_CTRL, GET)
2. For each patch region:
   a. Send TARGET_ADDRESS_LEN_REQ with target address and length
   b. Send firmware data in chunks via FW_SCATTER
3. Send PATCH_FINISH_REQ
```

### RAM Firmware Loading Sequence
```
1. For each firmware region:
   a. Send TARGET_ADDRESS_LEN_REQ with target address and length
   b. Send firmware data in chunks via FW_SCATTER
2. Send FW_START_REQ to start execution
3. Poll for firmware ready state
```

---

## DMA Enable Sequence (mt7996_dma_enable)

```c
// 1. Reset DTX pointer
write(RST_DTX_PTR, 0xFFFFFFFF);

// 2. Disable delay interrupts
write(PRI_DLY_INT_CFG0, 0);
write(PRI_DLY_INT_CFG1, 0);
write(PRI_DLY_INT_CFG2, 0);

// 3. Configure prefetch
mt7996_dma_prefetch();

// 4. Set BUSY_ENA
write(BUSY_ENA, TX_FIFO0 | TX_FIFO1 | RX_FIFO);

// 5. Set GLO_CFG_EXT0
write(GLO_CFG_EXT0, RX_WB_RXD | WED_MERGE_MODE);

// 6. Set GLO_CFG_EXT1
write(GLO_CFG_EXT1, TX_FCTRL_MODE);

// 7. Set RX thresholds
write(PAUSE_RX_Q_45_TH, 0xc000c);
write(PAUSE_RX_Q_67_TH, 0x10008);
write(PAUSE_RX_Q_89_TH, 0x10008);
write(PAUSE_RX_Q_RRO_TH, 0x20);

// 8. Start DMA
mt7996_dma_start();
```

---

## Key Differences: Our Implementation vs Linux

### Things to Verify

1. **Initialization Order**: Linux loads firmware BEFORE full DMA init
   - We might need minimal DMA setup just for MCU commands
   - Full DMA init comes after firmware is loaded

2. **RST Register**: Are we handling it correctly?
   - Linux: clear, then set, then leave at 0x30 during config
   - Question: When/how does RST get cleared for DMA to work?

3. **Ring Configuration**: Correct offsets?
   - FWDL ring at 0x320 (queue 2)
   - RX MCU at 0x500 (queue 0)

4. **Prefetch**: Is it required for firmware loading?
   - Linux configures prefetch depth per queue

5. **WF_SUBSYS_RST**: We're using CBTOP remap correctly now

---

## Linux DMA Initialization Sequence (Detailed)

Based on analysis of mt7996/dma.c:

### 1. mt7996_dma_disable(reset=true)

```c
// RST pulse - clear then set
mt76_clear(dev, MT_WFDMA0_RST, RST_DMASHDL_ALL_RST | RST_LOGIC_RST);
mt76_set(dev, MT_WFDMA0_RST, RST_DMASHDL_ALL_RST | RST_LOGIC_RST);

// Disable DMA in GLO_CFG
mt76_clear(dev, MT_WFDMA0_GLO_CFG, TX_DMA_EN | RX_DMA_EN | ...);
```

**Result**: RST = 0x30 (bits set), GLO_CFG DMA disabled

### 2. Queue Configuration (via mt76_init_mcu_queue)

```c
// For each MCU queue (FWDL, WM, WA):
// Writes to ring registers at offset queue*0x10 from MCU_RING_BASE
Q_WRITE(q, desc_base, q->desc_dma);  // Ring base address
Q_WRITE(q, ring_size, q->ndesc);      // Ring size
Q_WRITE(q, cpu_idx, 0);               // CPU index = 0
Q_WRITE(q, dma_idx, 0);               // DMA index = 0
```

**Note**: Ring config works with RST = 0x30!

### 3. mt7996_dma_enable

```c
// Reset DTX pointer
mt76_wr(dev, MT_WFDMA0_RST_DTX_PTR, ~0);

// Configure prefetch (via mt7996_dma_prefetch)
// Sets prefetch depth per queue

// Set BUSY_ENA
mt76_set(dev, MT_WFDMA0_BUSY_ENA, TX_FIFO0 | TX_FIFO1 | RX_FIFO);

// Set GLO_CFG_EXT0
mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT0, RX_WB_RXD | WED_MERGE_MODE);

// Set GLO_CFG_EXT1
mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT1, TX_FCTRL_MODE);

// Set RX thresholds
mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
// ... more thresholds
```

### 4. mt7996_dma_start

```c
// Enable DMA in GLO_CFG
mt76_set(dev, MT_WFDMA0_GLO_CFG,
         TX_DMA_EN | RX_DMA_EN | OMIT_TX_INFO | OMIT_RX_INFO_PFET2 | EXT_EN);

// Enable interrupts
mt7996_irq_enable(dev, irq_mask);
```

**Key Finding**: RST stays at 0x30 throughout! DMA works with RST=0x30 after proper init.

---

## Comparison: Our Implementation vs Linux (UPDATED)

| Step | Linux | Our Code | Match? |
|------|-------|----------|--------|
| RST pulse | clear then set | clear then set | **FIXED** |
| RST left at 0x30 | Yes | Yes | **FIXED** |
| Ring register layout | 16 bytes, no BASE_HI | 16 bytes, no BASE_HI | **FIXED** |
| Ring config with RST=0x30 | Yes | Yes | OK |
| RST_DTX_PTR = 0xFFFFFFFF | Yes | Yes | OK |
| BUSY_ENA = 0x7 | Yes | Yes | OK |
| GLO_CFG_EXT0 RX_WB_RXD | Yes | Yes | OK |
| GLO_CFG_EXT1 TX_FCTRL_MODE | Yes | Yes | OK |
| Enable DMA after ring config | Yes | Yes | **FIXED** |
| Enable TX/RX DMA | Yes | Yes | OK |
| Enable EXT_EN | Yes | Yes | OK |

### Fixed Issues

1. **Ring register layout**: Removed BASE_HI writes (was corrupting next ring)
2. **RST handling**: Now doing proper pulse (clear then set), leaving at 0x30
3. **DMA enable order**: Now enabling after ring configuration

### Remaining Differences

1. **RX thresholds** - Linux sets PAUSE_RX_Q_* (we don't)
2. **Interrupt enable** - Linux enables interrupts (we don't - polling mode)
3. **WED offload** - Linux has WED support (we don't need it)

---

## DMA Queue Architecture

This section details the DMA queue structure based on the Linux mt76 driver.

### Queue Categories

**TX Queues:**
| Queue Type | Purpose |
|------------|---------|
| TXQ_BAND0/1/2 | Per-band packet transmission |
| TXQ_MCU_WM | MCU command queue (WM = Wireless Manager) |
| TXQ_MCU_WA | MCU command queue (WA = Wireless Agent) |
| TXQ_FWDL | Firmware download queue |

**RX Queues:**
| Queue Type | Purpose |
|------------|---------|
| RXQ_MCU_WM | MCU event queue (WM) |
| RXQ_MCU_WA | MCU event queue (WA) |
| RXQ_BAND0/1/2 | Per-band packet reception |
| RXQ_TXFREE0/1/2 | TX completion notifications |

### Ring Register Layout (struct mt76_queue_regs)

**CRITICAL: Linux uses a 16-byte structure with NO BASE_HI register!**

```c
struct mt76_queue_regs {
    u32 desc_base;   // Offset 0x00 - Descriptor base address (32-bit)
    u32 ring_size;   // Offset 0x04 - Ring size (entry count)
    u32 cpu_idx;     // Offset 0x08 - CPU index (write pointer)
    u32 dma_idx;     // Offset 0x0C - DMA index (read pointer)
} __packed __aligned(4);  // Total: 16 bytes
```

### Ring Base Addresses

| Ring Type | Base Offset | Per-Ring Size |
|-----------|-------------|---------------|
| TX MCU | WFDMA0 + 0x300 | 0x10 bytes |
| RX | WFDMA0 + 0x500 | 0x10 bytes |

**Queue-to-Ring Mapping (Firmware Download):**
| Queue | Ring Index | Absolute Offset |
|-------|------------|-----------------|
| FWDL (TX) | 2 | 0xd4000 + 0x300 + (2 × 0x10) = 0xd4320 |
| MCU_WM (TX) | 0 | 0xd4000 + 0x300 + (0 × 0x10) = 0xd4300 |
| MCU_WA (TX) | 1 | 0xd4000 + 0x300 + (1 × 0x10) = 0xd4310 |
| MCU_WM (RX) | 0 | 0xd4000 + 0x500 + (0 × 0x10) = 0xd4500 |
| MCU_WA (RX) | 1 | 0xd4000 + 0x500 + (1 × 0x10) = 0xd4510 |

### Prefetch Configuration

Prefetch settings at WFDMA0 + 0x600:
```
Each queue: 4 bytes
Format: (base << 16) | depth

Typical depths:
- MCU rings: 4
- Data rings: 8-16
```

### DMA Initialization Sequence (mt7996_dma_init)

```
1. mt7996_dma_config()       // Configure queue masks and interrupt mappings
2. mt76_dma_attach()         // Attach DMA layer
3. mt7996_dma_disable(reset=true)  // Disable DMA, RST pulse
4. Initialize TX queues:
   - TXQ_FWDL (for firmware download)
   - TXQ_MCU_WM
   - TXQ_MCU_WA
   - TXQ_BAND0/1/2
5. Initialize RX queues:
   - RXQ_MCU_WM
   - RXQ_MCU_WA
   - RXQ_BAND0/1/2
6. mt7996_dma_enable()       // Enable DMA engine
```

### DMA Enable Sequence (mt7996_dma_enable)

```c
// 1. Reset DTX pointer
mt76_wr(dev, MT_WFDMA0_RST_DTX_PTR, ~0);  // 0xd420c = 0xFFFFFFFF

// 2. Disable delay interrupts
mt76_wr(dev, MT_WFDMA0_PRI_DLY_INT_CFG0, 0);
mt76_wr(dev, MT_WFDMA0_PRI_DLY_INT_CFG1, 0);
mt76_wr(dev, MT_WFDMA0_PRI_DLY_INT_CFG2, 0);

// 3. Configure prefetch
mt7996_dma_prefetch();

// 4. Set BUSY_ENA
mt76_set(dev, MT_WFDMA0_BUSY_ENA, TX_FIFO0 | TX_FIFO1 | RX_FIFO);  // 0x7

// 5. Set GLO_CFG_EXT0
mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT0, RX_WB_RXD | WED_MERGE_MODE);

// 6. Set GLO_CFG_EXT1
mt76_set(dev, WF_WFDMA0_GLO_CFG_EXT1, TX_FCTRL_MODE);

// 7. Set RX thresholds
mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
mt76_wr(dev, MT_WFDMA0_PAUSE_RX_Q_67_TH, 0x10008);
// ... more thresholds

// 8. Start DMA
mt7996_dma_start();
```

### DMA Start Sequence (mt7996_dma_start)

```c
// Enable DMA in GLO_CFG
mt76_set(dev, MT_WFDMA0_GLO_CFG,
         TX_DMA_EN |           // Bit 0
         RX_DMA_EN |           // Bit 2
         OMIT_TX_INFO |        // Bit 28
         OMIT_RX_INFO_PFET2 |  // Bit 21
         EXT_EN);              // Bit 26 - REQUIRED!

// Enable interrupts
mt7996_irq_enable(dev, irq_mask);
```

### RST Register Handling

The RST register at 0xd4100 controls DMA reset state:

```c
// RST pulse sequence (mt7996_dma_disable with reset=true):
mt76_clear(dev, MT_WFDMA0_RST, RST_DMASHDL_ALL_RST | RST_LOGIC_RST);  // Clear bits
mt76_set(dev, MT_WFDMA0_RST, RST_DMASHDL_ALL_RST | RST_LOGIC_RST);    // Set bits

// Result: RST = 0x30 (bits 4,5 set)
```

**Key Finding:** RST stays at 0x30 throughout normal operation. Ring configuration and DMA operation work with RST=0x30.

---

## Descriptor Format Investigation

### DMA Ring Descriptor (mt76_desc)

```c
struct mt76_desc {
    __le32 buf0;   // Buffer 0 address
    __le32 ctrl;   // Control: SD_LEN0, LAST_SEC0, DMA_DONE
    __le32 buf1;   // Buffer 1 address (or high bits)
    __le32 info;   // Additional info
} __packed;
```

### Control Field Bits

| Bits | Name | Description |
|------|------|-------------|
| 15:0 | SD_LEN1 | Segment data length 1 |
| 29:16 | SD_LEN0 | Segment data length 0 |
| 30 | LAST_SEC0 | Last segment for buffer 0 |
| 31 | DMA_DONE | DMA done (set by hardware) |

Our code:
```rust
desc.ctrl = ((len as u32) << 16) & 0x3FFF0000;  // SD_LEN0 bits 29:16
if last { desc.ctrl |= 1 << 30; }                // LAST_SEC0
```

This looks correct.

---

## Fixes Applied (2024-12-21)

### 1. Ring Register Layout Fix (CRITICAL)

**Problem**: We were writing to offset 0x10 (RING_BASE_HI) which in Linux's 16-byte ring layout is actually the NEXT ring's desc_base register. This corrupted other ring configurations.

**Fix**: Removed all writes to offset 0x10. Linux only uses 16 bytes per ring:
- 0x00: desc_base
- 0x04: ring_size
- 0x08: cpu_idx
- 0x0C: dma_idx

### 2. RST Register Handling

**Problem**: We were clearing RST bits (setting to 0x00), but Linux does a pulse (clear then SET), leaving RST at 0x30.

**Fix**: Changed to match Linux:
```rust
// Pulse RST: clear bits first, then SET them
self.write(RST, val & !rst_bits);  // Clear
self.write(RST, val | rst_bits);   // Set - leaves at 0x30
```

### 3. DMA Enable Sequence

**Problem**: We were enabling DMA (GLO_CFG) before ring configuration. Linux enables AFTER.

**Fix**: Moved DMA enable to after all ring configuration:
```
1. RST pulse (leave at 0x30)
2. Configure TX rings (FWDL, MCU)
3. Configure RX rings (MCU)
4. Reset DTX pointer
5. Set BUSY_ENA
6. Set GLO_CFG_EXT0/EXT1
7. Configure prefetch
8. THEN enable DMA in GLO_CFG
```

---

## Remaining Items to Verify

1. **RX thresholds** - Linux sets PAUSE_RX_Q_* thresholds
2. **Interrupt masking** - verify doesn't block DMA
3. **Command TXD format** - verify header fields match Linux
