# MT7996 WiFi Driver Reference

## Overview

MT7996 is a WiFi 7 tri-band chip. The BPI-R4 uses two PCIe devices:
- **HIF1** (0x7990): Primary interface, WFDMA0 @ 0xd4000
- **HIF2** (0x7991): Secondary interface, WFDMA0_PCIE1 @ 0xd8000

Both must be initialized for full operation.

---

## Driver Structure

```
user/driver/wifid/     - WiFi daemon (main driver)
user/driver/mt7996/    - MT7996 library (DMA, MCU, firmware)
```

---

## Initialization Call Chain

From Linux mt76/mt7996:

```
mt7996_pci_probe()                      [pci.c]
├── pci_enable_device() + pci_set_master()
├── mt7996_mmio_probe()
│   └── mt7996_wfsys_reset()            ← Reset WiFi subsystem
├── mt7996_pci_init_hif2()              ← Link secondary interface
└── mt7996_register_device()
    └── mt7996_init_hardware()
        ├── mt7996_dma_init()           ← DMA setup
        │   ├── mt7996_dma_disable(reset=true)
        │   ├── init TX queues
        │   └── mt76_init_mcu_queue()
        └── mt7996_mcu_init()           ← Firmware loading
            └── mt7996_mcu_init_firmware()
                ├── mt7996_driver_own()
                ├── mt7996_dma_enable()
                ├── mt7996_dma_start()
                ├── mt7996_load_patch()
                └── mt7996_load_ram()
```

---

## Key Registers

### WFDMA Core (HIF1 base: 0xd4000, HIF2 base: 0xd8000)

| Offset | Name | Description |
|--------|------|-------------|
| 0x100 | RST | DMA reset control |
| 0x200 | INT_SRC | Interrupt source |
| 0x204 | INT_MASK | Interrupt mask |
| 0x208 | GLO_CFG | Global DMA config |
| 0x20c | RST_DTX_PTR | Reset TX descriptor pointers |
| 0x2b0 | GLO_CFG_EXT0 | Extended config 0 |
| 0x2b4 | GLO_CFG_EXT1 | Extended config 1 (CALC_MODE, TX_FCTRL_MODE) |

### Queue Registers (offset from base)

TX queues start at 0x400, RX at 0x500. Each queue has:
| Offset | Name | Description |
|--------|------|-------------|
| +0x00 | DESC_BASE | Descriptor ring physical address (32-bit) |
| +0x04 | RING_SIZE | Number of descriptors |
| +0x08 | CPU_IDX | CPU-side index |
| +0x0c | DMA_IDX | DMA-side index |

### Queue Layout

| hw_idx | Queue | Offset | Ring Size |
|--------|-------|--------|-----------|
| 16 | FWDL | 0x400 | 128 |
| 17 | MCU_WM | 0x410 | 256 |
| 18 | BAND0 | 0x420 | 2048 |
| 20 | MCU_WA | 0x440 | 256 |

---

## Critical Implementation Details

### 1. Queue Selection for MCU Commands

**CRITICAL**: MCU commands go to different queues based on type:

| Command | Queue | hw_idx |
|---------|-------|--------|
| PATCH_SEM_CTRL (0x10) | MCU_WM | 17 |
| TARGET_ADDRESS_LEN (0x01) | MCU_WM | 17 |
| PATCH_START (0x05) | MCU_WM | 17 |
| FW_SCATTER (0xee) | FWDL | 16 |

### 2. DESC_BASE is 32-bit

Descriptor rings MUST be in memory < 4GB because DESC_BASE register is 32-bit.
TX/RX buffers can use 36-bit addresses (high 4 bits in descriptor).

### 3. Prefetch Configuration

Prefetch values encode `(cumulative_offset << 16) | depth`. Values from Linux:

```
TX: 0xd4640=0x00000002, 0xd4644=0x00200002, 0xd4648=0x00400008,
    0xd464c=0x00c00008, 0xd4650=0x01400002, 0xd4654=0x01600008
RX: 0xd4680=0x01e00002, 0xd4684=0x02000002, 0xd4688=0x02200002,
    0xd468c=0x02400002, 0xd4690=0x02600010, 0xd4694=0x03600010
```

### 4. GLO_CFG Enable Bits

```
TX_DMA_EN      = BIT(0)
RX_DMA_EN      = BIT(2)
OMIT_TX_INFO   = BIT(20)
OMIT_RX_INFO   = BIT(27)
OMIT_RX_INFO_PFET2 = BIT(21)
EXT_EN         = BIT(26)   ← CRITICAL - enables extended features
```

### 5. Memory Model

DMA memory is mapped as Normal Non-Cacheable in userspace. No explicit cache
flushes needed in driver code - coherency is automatic.

---

## Common Issues & Debugging

### DMA_IDX Stuck at 0

If DMA_IDX never increments after kicking a queue:

1. **Check DESC_BASE** - Must be < 4GB (32-bit register)
2. **Check bus mastering** - PCI command register bit 2
3. **Check prefetch config** - Must match Linux exactly
4. **Check descriptor format** - buf0, ctrl, buf1, info fields
5. **Check HIF2 handling** - Both interfaces need INT_MASK handling

### Register Comparison

Use `[MT76_REG]` logging format for easy comparison with Linux:
```
[MT76_REG] WR 0xd4640: 0x00000000 -> 0x00000002
[MT76_REG] RMW 0xd42b4: 0x00000000 | 0x80000000 -> 0x80000000
```

---

## Reference Files

- `docs/mt7996-linux/deepwiki.md` - Detailed register definitions
- `docs/mt7996-linux/*.c` - Annotated Linux driver source
- `armbian_golden.log` - Working Linux driver register trace

---

## Translation Rules

When porting from Linux mt76:

1. **EXACT bit manipulation** - Use identical operations
2. **EXACT register order** - Same sequence
3. **EXACT RMW pattern** - mt76_set = read | bits, mt76_clear = read & ~bits
4. **EXACT timing** - Same delays
5. **Document every line** - Comment with Linux source:line
