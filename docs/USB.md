# MT7988A USB Implementation Notes

## Hardware Overview

The MT7988A has two USB 3.2 Gen1 controllers (SSUSB0 and SSUSB1). Each controller consists of:
- **xHCI MAC** - Standard USB 3.0 host controller
- **IPPC** - MediaTek IP Port Control (vendor-specific initialization)
- **PHY** - USB2 (t-phy) and USB3 (x-phy) physical layer

## Base Addresses

| Controller | Component | Base Address | Size |
|------------|-----------|--------------|------|
| SSUSB0 | MAC (xHCI) | 0x11190000 | 0x2e00 |
| SSUSB0 | IPPC | 0x11193e00 | 0x0100 |
| SSUSB1 | MAC (xHCI) | 0x11200000 | 0x2e00 |
| SSUSB1 | IPPC | 0x11203e00 | 0x0100 |

## Interrupts

- SSUSB0: GIC SPI 173 (IRQ_TYPE_LEVEL_HIGH)
- SSUSB1: GIC SPI 172 (IRQ_TYPE_LEVEL_HIGH)

## IPPC Registers

MediaTek-specific IP Port Control registers at offset from IPPC base:

| Offset | Name | Description |
|--------|------|-------------|
| 0x00 | IP_PW_CTRL0 | Power control 0 (software reset) |
| 0x04 | IP_PW_CTRL1 | Power control 1 (host power down) |
| 0x08 | IP_PW_CTRL2 | Power control 2 (device power down) |
| 0x0C | IP_PW_CTRL3 | Power control 3 |
| 0x10 | IP_PW_STS1 | Power status 1 |
| 0x14 | IP_PW_STS2 | Power status 2 |
| 0x24 | IP_XHCI_CAP | xHCI capability (port counts) |
| 0x30 | U3_CTRL_P0 | USB3 port 0 control |
| 0x38 | U3_CTRL_P1 | USB3 port 1 control |
| 0x50 | U2_CTRL_P0 | USB2 port 0 control |
| 0x58 | U2_CTRL_P1 | USB2 port 1 control |
| 0x60 | U2_CTRL_P2 | USB2 port 2 control |
| 0x7C | U2_PHY_PLL | USB2 PHY PLL control |

### IP_PW_CTRL0 Bits
- Bit 0: IP_SW_RST - Software reset

### IP_PW_CTRL1 Bits
- Bit 0: IP_HOST_PDN - Host power down

### IP_PW_CTRL2 Bits
- Bit 0: IP_DEV_PDN - Device power down

### IP_PW_STS1 Bits
- Bit 30: IP_SLEEP_STS - Sleep status
- Bit 16: U3_MAC_RST - USB3 MAC reset done
- Bit 11: XHCI_RST - xHCI reset done
- Bit 10: SYS125_RST - 125MHz clock reset done
- Bit 8: REF_RST - Reference clock reset done
- Bit 0: SYSPLL_STABLE - System PLL stable

### U3/U2 Port Control Bits
- Bit 2: PORT_HOST_SEL - Host mode select
- Bit 1: PORT_PDN - Power down
- Bit 0: PORT_DIS - Disable

## Initialization Sequence

Based on Linux `drivers/usb/host/xhci-mtk.c`:

1. **IPPC Software Reset**
   - Set CTRL0.IP_SW_RST
   - Delay ~1us
   - Clear CTRL0.IP_SW_RST

2. **Power On Host IP**
   - Set CTRL2.IP_DEV_PDN (power down device mode)
   - Clear CTRL1.IP_HOST_PDN (power on host mode)

3. **Configure Ports**
   - For each U3 port: Clear PDN and DIS, set HOST_SEL
   - For each U2 port: Clear PDN and DIS, set HOST_SEL

4. **Wait for Clocks**
   - Poll STS1 until SYSPLL_STABLE, REF_RST, SYS125_RST, XHCI_RST are set

5. **xHCI Reset**
   - Read CAPLENGTH to find operational registers
   - Clear USBCMD.RUN to halt
   - Wait for USBSTS.HCH (halted)
   - Set USBCMD.HCRST
   - Wait for HCRST to clear and CNR to clear

6. **xHCI Configuration**
   - Allocate Device Context Base Address Array (DCBAA)
   - Allocate Command Ring
   - Set DCBAAP and CRCR registers
   - Configure max slots in CONFIG register
   - Set USBCMD.RUN to start

## PHY Configuration

MT7988 uses:
- **t-phy** for USB2 (High-speed, 480 Mbps)
- **x-phy** for USB3 (Super-speed, 5 Gbps)

PHY initialization is typically done by the bootloader (U-Boot) or requires
additional register configuration not documented here.

## Clocks Required

From device tree:
- infra_66m_usb_hck
- infra_usb_sys
- infra_usb_ref
- infra_usb_frmcnt
- infra_usb_pipe
- infra_usb_utmi
- infra_133m_usb_hck (for SSUSB1)
- infra_66m_usb_hck_ck_p1 (for SSUSB1)
- infra_usb_xhci (for SSUSB0)
- infra_usb_xhci_ck_p1 (for SSUSB1)

## References

- [Linux xhci-mtk.c](https://github.com/torvalds/linux/blob/master/drivers/usb/host/xhci-mtk.c)
- [Linux xhci-mtk.h](https://github.com/torvalds/linux/blob/master/drivers/usb/host/xhci-mtk.h)
- [MT7988A Device Tree](https://github.com/torvalds/linux/blob/master/arch/arm64/boot/dts/mediatek/mt7988a.dtsi)
- [MediaTek xHCI Bindings](https://www.kernel.org/doc/Documentation/devicetree/bindings/usb/mediatek,mtk-xhci.yaml)

## TODO

- [ ] PHY initialization (t-phy and x-phy)
- [ ] Memory allocation for DCBAA, command ring, event ring
- [ ] Interrupt handling
- [ ] Device enumeration
- [ ] USB descriptors parsing
- [ ] Hub support
- [ ] Mass storage class driver
- [ ] HID class driver
