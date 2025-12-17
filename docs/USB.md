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
- **XFI T-PHY** for SSUSB0 - at 0x11f20000 (64KB region, multi-protocol PHY)
- **T-PHY v2** for SSUSB1 - at 0x11c50000 (4KB region)
- Both PHYs support USB2 (High-speed) and USB3 (Super-speed)

**IMPORTANT**: This driver does NOT rely on U-Boot for PHY initialization.
PHY must be configured for host mode from scratch.

### XS-PHY Registers (SSUSB0)
| Offset | Name | Description |
|--------|------|-------------|
| 0x20 | U2PHYACR4 | Bandgap and termination enable |
| 0x68 | U2PHYDTM0 | BGR enable, UART mode |
| 0x6C | U2PHYDTM1 | Host mode control (vbusvalid, avalid, etc.) |

### T-PHY Registers (SSUSB1)
| Offset | Name | Description |
|--------|------|-------------|
| 0x36C | U2PHYDTM1 | COM_OFFSET(0x300) + 0x6C, host mode control |

## Clock and Reset Control

**IMPORTANT**: This driver does NOT rely on U-Boot for clock/reset setup.
USB clocks must be ungated and resets deasserted from scratch.

### INFRACFG_AO Registers (0x10001000)

| Offset | Name | Description |
|--------|------|-------------|
| 0x30/0x34 | RST0_SET/CLR | Module reset control |
| 0x40/0x44 | RST1_SET/CLR | Module reset control (USB bits here) |
| 0x80/0x84 | CG0_SET/CLR | Clock gate control |
| 0x88/0x8C | CG1_SET/CLR | Clock gate control |
| 0xA4/0xA8 | CG2_SET/CLR | Clock gate control |

Note: SET registers assert reset/gate clock, CLR registers deassert/ungate.

### USB Reset Bits (RST1)
- Bit 7: SSUSB_TOP0 reset
- Bit 8: SSUSB_TOP1 reset

(These bit positions need verification from Linux clk-mt7988-infracfg.c)

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

## USB 3.0 Hub Enumeration

USB 3.0 hubs require special handling compared to direct device connections.

### Hub Enumeration Sequence

1. **Address the hub** like a normal device (Enable Slot, Address Device)
2. **SET_CONFIGURATION** to activate the hub
3. **SET_HUB_DEPTH** (USB 3.0 specific, required before GET_HUB_DESCRIPTOR)
4. **GET_HUB_DESCRIPTOR** to determine port count
5. **EVALUATE_CONTEXT** to update hub's slot context with Hub bit
6. **Power each port** via SET_FEATURE(PORT_POWER)
7. **Wait for power stabilization** (hub descriptor specifies time)
8. **Check port status** and enumerate connected devices

### SET_HUB_DEPTH (Critical for USB 3.0)

USB 3.0 hubs **must** receive SET_HUB_DEPTH before GET_HUB_DESCRIPTOR will work:

```
bmRequestType: 0x20 (Host-to-device, Class, Device)
bRequest:      12 (SET_HUB_DEPTH)
wValue:        Hub depth (0 for root-connected hub)
wIndex:        0
wLength:       0
```

### EVALUATE_CONTEXT for Hub

After configuring the hub, update its slot context to inform xHCI it's a hub:

**Slot Context Fields (for EVALUATE_CONTEXT):**
- `dw0[26]` = Hub bit (set to 1)
- `dw0[23:20]` = Speed (4 = SuperSpeed)
- `dw0[31:27]` = Context Entries
- `dw1[31:24]` = Number of Downstream Ports
- `dw1[23:16]` = Root Hub Port Number

**Important**: Number of Ports goes in `dw1`, NOT `dw0`!

### Downstream Device Slot Context

For devices behind a hub, the slot context requires additional fields:

```
dw0[19:0]  = Route String (hub port number in bits [3:0] for first tier)
dw0[23:20] = Speed
dw0[31:27] = Context Entries (1 for just EP0)
dw1[23:16] = Root Hub Port Number (xHCI port where hub is connected)
dw2[7:0]   = Parent Hub Slot ID
dw2[15:8]  = Parent Port Number (port on hub where device is connected)
```

### Address Device Failures (CC=17 Slot Not Enabled)

Common causes for "Slot Not Enabled Error" when addressing devices behind hubs:

1. **Hub not configured as hub** - EVALUATE_CONTEXT must set Hub bit before addressing downstream devices
2. **Missing SET_HUB_DEPTH** - USB 3.0 hubs won't enumerate ports properly without this
3. **Incorrect route string** - Must encode the path through hub hierarchy
4. **Parent hub info missing** - dw2 must have parent slot and port
5. **Timing issues** - Allow sufficient delays between operations

### Event Ring Handling

When polling for events, **always acknowledge all consumed events** by updating ERDP, not just the expected event type. Unacknowledged events can stall the xHCI controller.

### Timing Requirements

USB devices need time to initialize:

| Operation | Minimum Wait |
|-----------|--------------|
| After port power | Hub descriptor `bPwrOn2PwrGood` × 2ms (often 100-500ms) |
| After port reset | 10ms (USB 3.0 spec) |
| Before Address Device | 10-200ms recommended |
| After SET_CONFIGURATION | 50ms |

## BPI-R4 Specific Notes

The BPI-R4 Type-A USB port has an **internal VIA Labs USB 3.0 hub** (VID 0x2109, PID 0x0822). This means:

- Devices plugged into the Type-A port appear behind a 4-port hub
- Must enumerate the hub first, then enumerate downstream devices
- Hub is detected on SSUSB1 Port 1

### USB VBUS Power Control

**IMPORTANT**: The BPI-R4 (mt7988a) and BPI-R4-Lite (mt7987a) have **different** USB power control:

| Board | USB VBUS Control | Notes |
|-------|------------------|-------|
| **BPI-R4** (mt7988a) | Hardware/always-on | No GPIO expander, GPIO 63/79 used as LEDs |
| BPI-R4-Lite (mt7987a) | PCA9555 GPIO 9 | Via I2C mux channel 3 @ 0x20 |

**BPI-R4 (mt7988a)**:
- USB VBUS is either always-on or initialized by U-Boot
- OpenWrt enables `ssusb1` with just `status = "okay"` (no vbus-supply)
- MT7988A native VBUS pins (GPIO 63, 79) are repurposed as status LEDs
- No PCA9555 GPIO expander in device tree

**BPI-R4-Lite (mt7987a)**:
- PCA9555 GPIO expander at 0x20 (via PCA9545 I2C mux channel 3)
- USB VBUS controlled via `reg_usb_5v` regulator on GPIO 9
- Different pin assignments than originally assumed

### Diagnosing USB Power Issues

Check the PORTSC.PP (Port Power) bit to verify VBUS status:
- PP=1: Port has power (VBUS OK)
- PP=0: Port has no power (VBUS problem)

The USB driver includes a `check_power_status()` function that displays this information.

## References

- [Linux xhci-mtk.c](https://github.com/torvalds/linux/blob/master/drivers/usb/host/xhci-mtk.c)
- [Linux xhci-mtk.h](https://github.com/torvalds/linux/blob/master/drivers/usb/host/xhci-mtk.h)
- [MT7988A Device Tree](https://github.com/torvalds/linux/blob/master/arch/arm64/boot/dts/mediatek/mt7988a.dtsi)
- [MediaTek xHCI Bindings](https://www.kernel.org/doc/Documentation/devicetree/bindings/usb/mediatek,mtk-xhci.yaml)
- [OSDev: USB Hub on xHCI](https://forum.osdev.org/viewtopic.php?t=37441) - Hub enumeration details
- [OSDev: Address Device behind hub](https://forum.osdev.org/viewtopic.php?t=56729) - Slot context issues

## TODO

- [x] Memory allocation for DCBAA, command ring, event ring
- [x] Interrupt handling
- [x] Device enumeration (direct connection)
- [x] USB descriptors parsing
- [x] Hub detection and configuration
- [ ] Hub downstream device enumeration (CC=17 issue)
- [ ] PHY initialization without U-Boot
- [ ] Mass storage class driver
- [ ] HID class driver
