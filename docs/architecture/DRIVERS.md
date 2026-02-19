# Driver Inventory

Hardware driver status for the two supported platforms: Banana Pi BPI-R4 (MT7988A) and Raspberry Pi 4 (BCM2711).

## Networking

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| Ethernet MAC | GENET v5 (1GbE) | GMAC ×3 (NETSYS v3 frame engine) | `ethd` stub (MT7988A) |
| Ethernet switch | — | MT7531 built-in (4× GbE ports) | `switchd` stub |
| 10GbE / SFP | — | 2× USXGMII (10GbE SFP cages) | Not started |
| PHY / MDIO | Broadcom internal PHY | Internal + SGMII/USXGMII SerDes | Not started |
| Wi-Fi | External USB/PCIe dongle | MT7996 (Wi-Fi 7, PCIe) | `wifid` (FW load done, RX pending) |
| Packet offload (TOPS) | — | Tunnel Offload Processor System | Not started |
| Crypto/IPsec (EIP-197) | — | Inline crypto engine | Not started |
| PPE (flow offload) | — | Packet Processing Engine ×2 | Not started |
| TCP/IP stack | — | — | `ipd`/`netd` (basic) |

## Storage

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| SD/eMMC | EMMC2 (Arasan SDHCI) | MSDC (MT7988A SDHCI) | `sd.rs` kernel stub (MT7988A) |
| SPI-NAND | — | SPI-NAND controller (128MB flash) | Not started |
| NVMe (PCIe) | Via PCIe | Via PCIe (M.2 slot) | `nvmed` works |
| USB Mass Storage | Via xHCI | Via xHCI | `usbd` works (read path) |
| FAT filesystem | — | — | `fatfsd` works |
| Partition table | — | — | `partd` works |

## USB

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| xHCI host controller | VL805 (via PCIe) | SSUSB ×2 (integrated xHCI) | `usbd` works |
| USB hub driver | VL805 built-in hub | VL822 (USB-A port) | Partial (polling workaround) |
| USB device classes | HID, MSC, etc. | HID, MSC, etc. | MSC only (read path) |

## PCIe

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| PCIe host controller | Brcmstb (1× Gen2) | MT7988A MAC-TLP ×4 (Gen3) | `pcied` works (MT7988A) |
| MSI/MSI-X | GICv2m or built-in | GICv3 ITS or built-in | Basic MSI works |
| PCIe enumeration | — | — | Works via `pcied` |

## Display / Graphics

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| GPU | VideoCore VI (VC4/V3D) | — (headless SoC) | Not started |
| HDMI | 2× HDMI 2.0 (4Kp60) | — | Not started |
| DSI display | 1× DSI | — | Not started |
| CSI camera | 1× CSI-2 | — | Not started |
| HVS / pixel valve | Display compositor | — | Not started |
| Framebuffer | Mailbox-based | — | Not started |

## Serial / Low-Speed I/O

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| UART | PL011 + mini UART | 16550-compat ×3 | Kernel console works (both) |
| I2C | BSC ×6 | I2C controller | `i2c.rs` kernel (MT7988A) |
| SPI | SPI ×6 | SPI controller | Not started |
| GPIO | 58-pin controller | GPIO + pinmux | `gpio` driver exists |
| PWM | 2-channel PWM | PWM controller | `pwmd` works (fan on PWM0/GPIO57) |

## System / Platform

| Driver | RPi 4 (BCM2711) | BPI-R4 (MT7988A) | Status |
|--------|-----------------|-------------------|--------|
| Interrupt controller | GIC-400 | GICv3 (SPI/SGI/ITS) | Works (both) |
| Timer | ARM generic timer | ARM generic timer | Works (both) |
| DMA controller | BCM DMA (16 ch) | A-DMA / QDMA (frame engine) | Not started |
| Clock controller | — | TOPCKGEN + INFRACFG + APMIXEDSYS | Partial (manual enables) |
| Power domains | RPi firmware mailbox | SCPSYS power domains | Not started |
| Thermal sensor | BCM thermal | MT7988A thermal | Not started |
| Watchdog | PM watchdog | MT7988A WDT | `wdt.rs` kernel (MT7988A) |
| RNG | BCM RNG200 | — | Not started |
| DVFS / cpufreq | RPi firmware | ARM PLL via `cpufreq.rs` | `cpud` works (MT7988A) |

## Priority: Router OS (BPI-R4)

1. **Ethernet MAC + Switch** — the 4× GbE ports are the board's primary function
2. **USXGMII / SFP** — the 10GbE uplinks
3. **SPI-NAND** — boot flash, persistent config storage
4. **SD/eMMC** — complete the SDHCI stub
5. **Clock/power framework** — proper TOPCKGEN driver

## Priority: General-Purpose OS (RPi 4)

1. **BCM PCIe host** — needed for USB (VL805 is PCIe-attached)
2. **GENET Ethernet** — the only network interface
3. **EMMC2 / SD** — boot and root storage
4. **VideoCore VI** — framebuffer for graphical output
5. **BCM GPIO** — different register layout from MT7988A
