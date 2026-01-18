## MT7996 HIF2 Initialization Flow: Device Detection to Firmware Download

This codemap traces the complete initialization flow for MT7996 WiFi device with HIF2 support, from PCI device detection through firmware download. Key stages include device detection [1a], hardware initialization [3a], MCU setup [3b], and multi-component firmware loading [4e]. The flow shows both primary and secondary host interface configuration for dual-HIF operation [6a].

---

## Verified Findings (deepwiki Q&A session 2024-12-30)

### RST Register Auto-Release

**⚠️ NEVER MANUALLY CLEAR RST! ⚠️**

RST is a PULSE at initialization. It AUTO-RELEASES when all prerequisites are met.
If RST stays 0x30 after GLO_CFG enable, we're MISSING something - do NOT force clear!

- **CONFIRMED**: RST bits (0x30) are NEVER explicitly cleared by software
- Hardware auto-releases when GLO_CFG enables DMA (TX_DMA_EN + RX_DMA_EN)
- Same pattern used across MT7915, MT792x chipsets
- If RST doesn't release → debug prerequisites, don't force it

### Queue Routing for Firmware Loading
| Command | Queue | TXD Header |
|---------|-------|------------|
| PATCH_SEM_CONTROL | WM (ring 0) | YES |
| INIT_DOWNLOAD | FWDL (ring 2) | YES |
| FW_SCATTER | FWDL (ring 2) | **NO** (raw data) |
| PATCH_FINISH | FWDL (ring 2) | YES |

Key code in `mt7996_mcu_send_message()`:
```c
if (cmd == MCU_CMD(FW_SCATTER)) {
    qid = MT_MCUQ_FWDL;
    goto exit;  // Skips TXD header!
}
```

### DMA Enable Sequence Order (mt7996_dma_enable)
1. `RST_DTX_PTR = 0xFFFFFFFF` - reset all DMA indices
2. Clear `PRI_DLY_INT_CFG0/1/2` - disable delay interrupts
3. Call `mt7996_dma_prefetch()` - configure prefetch
4. Set `BUSY_ENA` bits (TX_FIFO0, TX_FIFO1, RX_FIFO)
5. Poll `HIF_MISC_BUSY` until idle (timeout 1000ms)
6. Configure `GLO_CFG_EXT0/EXT1`
7. Set RX pause thresholds
8. `mt7996_dma_start()` → GLO_CFG enable → HW auto-clears RST

### Prerequisites for RST Auto-Release
**CRITICAL**: RST bits will NOT auto-clear unless ALL conditions met:
1. All rings properly programmed with valid descriptors
2. Prefetch configuration completed
3. **HIF_MISC_BUSY must report idle** (polled before enable)
4. BUSY_ENA configured for TX/RX FIFOs

If RST stays 0x30 after GLO_CFG enable → prerequisites not met!

### Required Queues for RST Release (verified 2024-12-30)
Hardware validates ALL expected queue types before releasing RST:

**TX Queues:**
- MCU TX: WM (ring 0), WA (ring 1), FWDL (ring 2)
- Data TX: BAND0 (at 0x400)

**RX Queues (HIF1):**
- MCU RX: MCU (rx0), WA (rx1), WA_MAIN (rx2), WA_TRI (rx3)
- Data RX: RX_MAIN (rx4 at 0x540)
- **RX_TXFREE: TX completion notifications** (dma.c:703-730) ← CRITICAL!
  - Queue IDs: MT_RXQ_TXFREE_BAND0=9, BAND1=9, BAND2=7 (mt7996.h:192-194)
  - __RXQ(q) = q + 3 (regs.h:502)
  - **BAND0 ring offset: __RXQ(9)=12 → 12*0x10 + 0x500 = 0x5C0**
  - Ring size: 512, Prefetch depth: 4, Prefetch reg: 0x6B0
- **RX_RRO: Hardware Receive Reorder** ← REQUIRED for RST release!
  - MT7996_RXQ_RRO_BAND0=8, RRO_BAND1=9, RRO_BAND2=6 (mt7996.h)
  - **BAND0 ring offset: __RXQ(8)=11 → 11*0x10 + 0x500 = 0x5B0**
  - Ring size: 512, Prefetch depth: 16, Prefetch reg: 0x6AC
  - **Hardware validates RRO rings even for firmware loading!**

**HIF2 Requirements (dma.c:129-139, 747-878):**
HIF2 needs DEDICATED rings separate from HIF1. Without these, RST won't auto-release!

**⚠️ CRITICAL: HIF2 ACCESS METHOD ⚠️**

HIF2 registers MUST be accessed via HIF1's BAR + 0x4000 offset, NOT via HIF2's separate BAR!
- Linux uses: `mt76_wr(dev, reg + hif1_ofs, value)` where `hif1_ofs = 0x4000`
- HIF2's separate BAR appears to be a "shadow" that doesn't control the actual hardware
- Verified 2024-12-30: Direct BAR writes appear to work but DMA never activates
- Correct access: `HIF1_BAR + register_offset + 0x4000`

- **GLO_CFG:** Must enable TX_DMA_EN | RX_DMA_EN on HIF2
- **TX Rings on HIF2:** (ring base = MCU_RING_BASE + ring_num * 0x10)
  - band1 (ring 19): offset 0x430, prefetch @ 0x68C
  - band2 (ring 21): offset 0x450, prefetch @ 0x694
  - Ring size: 512, prefetch depth: 8
- **RX Data Ring on HIF2:** BAND2 (RXQ 5)
  - __RXQ(5) = 8, offset: 0x580
  - Ring size: 512, prefetch @ 0x6A0, depth: 16
- **RX TXFREE Ring on HIF2:** BAND2 (RXQ 7) - CRITICAL!
  - __RXQ(7) = 10, offset: 0x5A0
  - Ring size: 512, prefetch @ 0x6A8, depth: 4
- **RX RRO Ring on HIF2:** BAND2 (RXQ 6) - REQUIRED!
  - __RXQ(6) = 9, offset: 0x590
  - Ring size: 512, prefetch @ 0x6A4, depth: 16
- **Prefetch:** All HIF2 rings need prefetch configured before CALC_MODE
- **Both interfaces must be enabled for proper RST release**

### Patch Semaphore
- Mandatory before ROM patch loading
- Uses MCU_CMD(PATCH_SEM_CONTROL) via WM queue
- Response: PATCH_IS_DL (skip) or PATCH_NOT_DL_SEM_SUCCESS (proceed)

### TXD Header (64 bytes total)
- txd[0]: TX_BYTES(15:0), PKT_FMT(24:23)=2, Q_IDX(31:25)=0
- txd[1]: LONG_FORMAT(31)=1, HDR_FORMAT(17:16)=1
- txd[2-7]: zeroed

---

## Verified Register Addresses (deepwiki Q&A session 2024-12-30)

All addresses verified against Linux mt7996 driver `regs.h`.

### WFDMA Base
| Register | Address | Source |
|----------|---------|--------|
| MT_WFDMA0_BASE | 0xd4000 | regs.h:416 |
| MT_WFDMA_EXT_CSR_BASE | 0xd7000 | regs.h:476 |

### Control Registers (relative to WFDMA0_BASE)
| Register | Offset | Full Address | Source |
|----------|--------|--------------|--------|
| RST | 0x100 | 0xd4100 | regs.h:419 |
| BUSY_ENA | 0x13c | 0xd413c | regs.h:458 |
| INT_SOURCE_CSR | 0x200 | 0xd4200 | regs.h:527 |
| INT_MASK_CSR | 0x204 | 0xd4204 | regs.h:528 |
| GLO_CFG | 0x208 | 0xd4208 | regs.h:420 |
| RST_DTX_PTR | 0x20c | 0xd420c | regs.h:421 |
| GLO_CFG_EXT0 | 0x2b0 | 0xd42b0 | regs.h:422 |
| GLO_CFG_EXT1 | 0x2b4 | 0xd42b4 | regs.h:423 |

### HIF_MISC (for BUSY polling)
| Register | Address | Source |
|----------|---------|--------|
| MT_WFDMA_EXT_CSR_HIF_MISC | 0xd7044 | regs.h:476-477 (0xd7000 + 0x44) |
| HIF_MISC_BUSY | bit 0 | dma.c:394-395 |

**Note**: Earlier docs incorrectly suggested 0x1f6540 - that's wrong for MT7996.

### HIF2 Configuration Registers (Required for dual-HIF RST release)
These registers must be configured when HIF2 is present, per dma.c:422-489.

| Register | Address | Value | Purpose | Source |
|----------|---------|-------|---------|--------|
| MT_WFDMA_HOST_CONFIG | 0xd7030 | 0x400001 | Band-to-PCIe mapping | regs.h:470-474, dma.c:422-435 |
| MT_WFDMA_AXI_R2A_CTRL | 0xd7500 | 0x14 | AXI outstanding transactions | regs.h:479-481, dma.c:437-439 |
| MT_WFDMA0_RX_INT_PCIE_SEL | 0xd4154 | 0x8 | RX interrupt routing (RING3) | regs.h:428-432, dma.c:472-489 |

**HOST_CONFIG bits:**
- BIT(0): PDMA_BAND - Enable PDMA band configuration
- BIT(20): BAND0_PCIE1 - Map band 0 to PCIe1
- BIT(21): BAND1_PCIE1 - Map band 1 to PCIe1
- BIT(22): BAND2_PCIE1 - Map band 2 to PCIe1 (set for MT7996)

**Configuration sequence (from Linux dma.c):**
1. Set PDMA_BAND (bit 0)
2. Clear BAND0/1/2_PCIE1 bits
3. Set BAND2_PCIE1 for MT7996 (or BAND1_PCIE1 for MT7992)
4. Final value for MT7996: 0x400001

**RX_INT_PCIE_SEL:**
Fixes hardware limitation where PCIE1's rx ring3 is unavailable by redirecting
interrupts from PCIE0 to PCIE1.

**HIF_MISC_BUSY 0ms is NORMAL**: If the value shows bit 0 = 0 (e.g., `0x001c2000`), the DMA engine
is already idle and the poll returns immediately. This is the ideal scenario - 1000ms timeout only
triggers if hardware is stuck busy. A 0ms result means hardware is ready for next init phase.

### Ring Register Bases (relative to WFDMA0_BASE)
| Ring Type | Base Offset | Source |
|-----------|-------------|--------|
| MCU TX rings (WM/WA/FWDL) | 0x300 + q*0x10 | regs.h:513 |
| Data TX rings | 0x300 + q*0x10 | regs.h:514 |
| RX rings | 0x500 + q*0x10 | regs.h:515 |

### Prefetch Control Registers
| Register | Base | Source |
|----------|------|--------|
| MCUQ_EXT_CTRL | 0x600 + q*4 | regs.h:520-521 |
| TXQ_EXT_CTRL | 0x600 + q*4 | regs.h:522-523 |
| RXQ_EXT_CTRL | 0x680 + q*4 | regs.h:524-525 |

---
### 1. PCI Device Detection and Initial Probe
PCI subsystem detects MT7996 device and triggers driver probe function
### 1a. PCI Probe Entry Point (`pci.c:99`)
Main probe function called when PCI device is detected
```text
static int mt7996_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
```
### 1b. MMIO Device Probe (`pci.c:133`)
Initialize MMIO interface and device structure
```text
dev = mt7996_mmio_probe(&pdev->dev, pcim_iomap_table(pdev)[0],
				id->device);
```
### 1c. HIF2 Detection (`pci.c:140`)
Detect and initialize secondary host interface
```text
hif2 = mt7996_pci_init_hif2(pdev);
```
### 2. Hardware and Device Structure Initialization
Initialize device structure, register mappings, and basic hardware setup
### 2a. Device Allocation (`mmio.c:837`)
Allocate and initialize main device structure
```text
mdev = mt76_alloc_device(pdev, sizeof(*dev), &mt7996_ops, &drv_ops);
```
### 2b. MMIO Initialization (`mmio.c:843`)
Setup register mappings and bus operations
```text
ret = mt7996_mmio_init(mdev, mem_base, device_id);
```
### 2c. Device Registration (`pci.c:192`)
Register device with mac80211 and initialize hardware
```text
ret = mt7996_register_device(dev);
```
### 3. Hardware Initialization and MCU Setup
Initialize hardware components and prepare for firmware loading
### 3a. Hardware Initialization (`init.c:1664`)
Initialize DMA, MCU, and other hardware components
```text
ret = mt7996_init_hardware(dev);
```
### 3b. MCU Initialization (`init.c:1195`)
Initialize MCU interface and prepare for firmware
```text
ret = mt7996_mcu_init(dev);
```
### 3c. Firmware Initialization (`mcu.c:3351`)
Start firmware loading process
```text
ret = mt7996_mcu_init_firmware(dev);
```
### 4. Firmware Download Process
Load and flash firmware components including patch and RAM firmware
### 4a. Set Normal Mode (`mcu.c:3002`)
Force firmware into normal operation mode
```text
mt76_wr(dev, MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);
```
### 4b. Take Driver Ownership (`mcu.c:3004`)
Assert driver control over firmware
```text
ret = mt7996_driver_own(dev, 0);
```
### 4c. Load Firmware (`mcu.c:3314`)
Main firmware loading function
```text
ret = mt7996_load_firmware(dev);
```
### 4d. Load Patch (`mcu.c:3176`)
Load ROM patch firmware
```text
ret = mt7996_load_patch(dev);
```
### 4e. Load RAM Firmware (`mcu.c:3180`)
Load main WM, DSP, and WA firmware to RAM
```text
ret = mt7996_load_ram(dev);
```
### 5. RAM Firmware Components Loading
Load individual firmware components (WM, DSP, WA) to device RAM
### 5a. Load WM Firmware (`mcu.c:3109`)
Load main WiFi MAC firmware
```text
ret = __mt7996_load_ram(dev, "WM", fw_name(dev, FIRMWARE_WM),
				MT7996_RAM_TYPE_WM);
```
### 5b. Load DSP Firmware (`mcu.c:3117`)
Load digital signal processor firmware
```text
ret = __mt7996_load_ram(dev, "DSP", fw_name(dev, FIRMWARE_DSP),
				MT7996_RAM_TYPE_DSP);
```
### 5c. Load WA Firmware (`mcu.c:3122`)
Load wrapper agent firmware
```text
return __mt7996_load_ram(dev, "WA", fw_name(dev, FIRMWARE_WA),
			 MT7996_RAM_TYPE_WA);
```
### 6. HIF2 Secondary Interface Configuration
Configure secondary host interface for dual-HIF operation
### 6a. HIF2 Driver Ownership (`mcu.c:3308`)
Take driver ownership of secondary interface
```text
if (dev->hif2) {
		ret = mt7996_driver_own(dev, 1);
```
### 6b. HIF2 WED Init (`pci.c:167`)
Initialize Wireless Ethernet Dispatcher for HIF2
```text
ret = mt7996_mmio_wed_init(dev, hif2_dev, true, &hif2_irq);
```
### 6c. HIF2 Interrupt Setup (`pci.c:187`)
Configure interrupt handling for secondary interface
```text
mt76_wr(dev, MT_INT1_MASK_CSR, 0);
```
