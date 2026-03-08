//! Firmware Loading — patch, WM, DSP, WA binary parsing + upload
//!
//! Each step is a standalone function that returns Result.
//! Load order: patch → WM → DSP → WA, then poll for FW_STATE_RDY(7).

use userlib::{unotice, uinfo, uerror, udebug, ulog};
use crate::regs::*;
use crate::device::Mt76Device;
use crate::dma::TxRing;
use crate::mcu::{self, FwIrq, McuError, dl_mode, fw_feature, fw_start};

// ============================================================================
// Embedded firmware images (233 variant — BPI-R4 dual-ADIE TBTC)
// ============================================================================

pub static FW_ROM_PATCH: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_rom_patch_233.bin");
pub static FW_WM: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_wm_233.bin");
pub static FW_WA: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_wa_233.bin");
pub static FW_DSP: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_dsp.bin");
pub static FW_EEPROM: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_eeprom.bin");

// ============================================================================
// Firmware binary format structs
// ============================================================================

/// Patch header descriptor.
/// Source: mcu.c:20-27
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchHeaderDesc {
    patch_ver: u32,
    subsys: u32,
    feature: u32,
    n_region: u32,
    crc: u32,
    reserved: [u32; 11],
}

/// Patch header — at START of patch file.
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchHeader {
    build_date: [u8; 16],
    platform: [u8; 4],
    hw_sw_ver: u32,
    patch_ver: u32,
    checksum: u16,
    _reserved: u16,
    desc: PatchHeaderDesc,
}

/// Patch section — 64 bytes each.
/// Source: mcu.c:30-44
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchSec {
    sec_type: u32,
    offs: u32,
    size: u32,
    addr: u32,
    len: u32,
    sec_key_idx: u32,
    align_len: u32,
    _reserved: [u32; 9],
}

/// Firmware trailer — at END of firmware file.
/// Source: mcu.c:46-56
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct FwTrailer {
    chip_id: u8,
    eco_code: u8,
    n_region: u8,
    format_ver: u8,
    format_flag: u8,
    _reserved: [u8; 2],
    fw_ver: [u8; 10],
    build_date: [u8; 15],
    crc: u32,
}

/// Firmware region descriptor — 40 bytes.
/// Source: mt76_connac_mcu.h:184-194
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct FwRegion {
    decomp_crc: u32,
    decomp_len: u32,
    decomp_blk_sz: u32,
    _reserved: [u8; 4],
    addr: u32,
    len: u32,
    feature_set: u8,
    region_type: u8,
    _reserved1: [u8; 14],
}

// ============================================================================
// Loading functions
// ============================================================================

/// Reborrow an IRQ option for passing to MCU functions that consume `Option<&mut FwIrq>`.
fn irq<'a, 'b>(opt: &'a mut Option<&'b mut FwIrq>) -> Option<&'a mut FwIrq>
where 'b: 'a {
    opt.as_mut().map(|r| &mut **r)
}

/// Upload firmware data in 4KB chunks via FWDL ring.
fn upload_chunks(
    dev: &Mt76Device, ring: &mut TxRing,
    data: &[u8], seq: &mut u8,
    fw_irq: &mut Option<&mut FwIrq>,
) -> Result<(), McuError> {
    for chunk in data.chunks(MCU_FW_DL_BUF_SIZE) {
        mcu::send_fw_chunk(dev, ring, chunk, *seq, irq(fw_irq))?;
        *seq = seq.wrapping_add(1);
    }
    Ok(())
}

/// Load patch firmware.
/// Source: mcu.c:2208-2296
fn load_patch(
    dev: &Mt76Device, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing,
    seq: &mut u8, fw_irq: &mut Option<&mut FwIrq>,
) -> Result<(), McuError> {
    let fw = FW_ROM_PATCH;

    let hdr_size = core::mem::size_of::<PatchHeader>();
    let sec_size = core::mem::size_of::<PatchSec>();

    if fw.len() < hdr_size {
        uerror!("firmware", "patch_too_small"; bytes = fw.len());
        return Err(McuError::TxTimeout { cmd: "patch_header" });
    }

    let hdr = unsafe { &*(fw.as_ptr() as *const PatchHeader) };
    let n_region = u32::from_be(hdr.desc.n_region);

    // Acquire semaphore
    mcu::patch_sem_ctrl(dev, mcu_ring, true, *seq, irq(fw_irq))?;
    *seq = seq.wrapping_add(1);

    for i in 0..n_region {
        let sec_offset = hdr_size + (i as usize * sec_size);
        if sec_offset + sec_size > fw.len() {
            uerror!("firmware", "section_oob"; idx = i);
            return Err(McuError::TxTimeout { cmd: "patch_section" });
        }

        let sec = unsafe { &*(fw.as_ptr().add(sec_offset) as *const PatchSec) };
        let sec_type = u32::from_be(sec.sec_type);
        if (sec_type & 0xFFFF) != 0x2 {
            uerror!("firmware", "bad_sec_type"; sec_type = sec_type);
            return Err(McuError::TxTimeout { cmd: "patch_sec_type" });
        }

        let addr = u32::from_be(sec.addr);
        let len = u32::from_be(sec.len);
        let data_offs = u32::from_be(sec.offs) as usize;
        let sec_key_idx = u32::from_be(sec.sec_key_idx);

        let enc_type = (sec_key_idx >> 24) & 0xFF;
        let mode = match enc_type {
            0x00 => 0u32,
            0x01 => {
                let key = sec_key_idx & 0xFF;
                dl_mode::ENCRYPT | dl_mode::RESET_SEC_IV | ((key & 0x3) << 1)
            }
            0x02 => dl_mode::ENCRYPT,
            _ => return Err(McuError::TxTimeout { cmd: "patch_enc_type" }),
        };

        if data_offs + len as usize > fw.len() {
            uerror!("firmware", "data_oob"; idx = i);
            return Err(McuError::TxTimeout { cmd: "patch_data_oob" });
        }

        mcu::init_download(dev, mcu_ring, addr, len, mode, *seq, irq(fw_irq))?;
        *seq = seq.wrapping_add(1);

        upload_chunks(dev, fwdl_ring, &fw[data_offs..data_offs + len as usize], seq, fw_irq)?;
    }

    userlib::delay_ms(100);

    mcu::start_firmware(dev, mcu_ring, true, 0, 0, *seq, irq(fw_irq))?;
    *seq = seq.wrapping_add(1);

    uinfo!("firmware", "patch_loaded"; regions = n_region, bytes = fw.len());
    Ok(())
}

/// Load RAM firmware (WM, DSP, or WA).
/// Source: mcu.c:3032-3083
fn load_ram(
    dev: &Mt76Device, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing,
    fw: &[u8], name: &str, fw_start_bits: u32,
    seq: &mut u8, fw_irq: &mut Option<&mut FwIrq>,
) -> Result<(), McuError> {
    if fw.len() < core::mem::size_of::<FwTrailer>() {
        uerror!("firmware", "ram_too_small"; name = name, bytes = fw.len());
        return Err(McuError::TxTimeout { cmd: "ram_header" });
    }

    let trailer_offset = fw.len() - core::mem::size_of::<FwTrailer>();
    let trailer = unsafe { &*(fw.as_ptr().add(trailer_offset) as *const FwTrailer) };
    let n_region = trailer.n_region;

    let region_size = core::mem::size_of::<FwRegion>();
    let regions_start = trailer_offset - (n_region as usize * region_size);

    let mut override_addr: u32 = 0;
    let mut data_offset = 0usize;

    for i in 0..n_region {
        let region_ptr = fw.as_ptr().wrapping_add(regions_start + i as usize * region_size);
        let region = unsafe { &*(region_ptr as *const FwRegion) };

        let addr = u32::from_le(region.addr);
        let len = u32::from_le(region.len);
        let is_wa = fw_start_bits != 0;
        let mode = mcu::gen_dl_mode(region.feature_set, is_wa);

        if region.feature_set & fw_feature::OVERRIDE_ADDR != 0 {
            override_addr = addr;
        }

        // Ensure all MMIO writes complete before issuing MCU download command.
        // Source: Linux mt7996/mcu.c:3012 wmb() before mt76_mcu_send_firmware()
        unsafe { core::arch::asm!("dsb sy", "isb") };

        mcu::init_download(dev, mcu_ring, addr, len, mode, *seq, irq(fw_irq))?;
        *seq = seq.wrapping_add(1);

        upload_chunks(dev, fwdl_ring, &fw[data_offset..data_offset + len as usize], seq, fw_irq)?;
        data_offset += len as usize;
    }

    let mut option: u32 = 0;
    if override_addr != 0 { option |= fw_start::OVERRIDE; }
    option |= fw_start_bits;

    mcu::start_firmware(dev, mcu_ring, false, option, override_addr, *seq, irq(fw_irq))?;
    *seq = seq.wrapping_add(1);

    uinfo!("firmware", "ram_loaded"; name = name, regions = n_region, bytes = fw.len());
    Ok(())
}

/// Full firmware load sequence: patch → WM → DSP → WA.
/// Polls for fw_state=7 (ready) after all images loaded.
pub fn load_all(
    dev: &Mt76Device,
    mcu_ring: &mut TxRing,
    fwdl_ring: &mut TxRing,
    mut irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    // Pre-load diagnostics (matches wifid's trace_rx pattern)
    let pre_fw_state = dev.rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    let pre_wm_dma = dev.rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
    let pre_wa_dma = dev.rr(MCU_WA_RX_REGS + MT_QUEUE_DMA_IDX);
    unotice!("firmware", "load_start"; fw_state = pre_fw_state, wm_dma = pre_wm_dma, wa_dma = pre_wa_dma);
    ulog::flush();
    let mut seq: u8 = 1;

    // 1. Patch
    load_patch(dev, mcu_ring, fwdl_ring, &mut seq, &mut irq)?;
    uinfo!("firmware", "patch_done"; seq = seq);
    ulog::flush();
    userlib::delay_ms(100);

    // 2. WM
    load_ram(dev, mcu_ring, fwdl_ring, FW_WM, "WM", 0, &mut seq, &mut irq)?;
    uinfo!("firmware", "wm_done"; seq = seq);
    ulog::flush();
    userlib::delay_ms(100);

    // 3. DSP
    load_ram(dev, mcu_ring, fwdl_ring, FW_DSP, "DSP", fw_start::WORKING_PDA_DSP, &mut seq, &mut irq)?;
    uinfo!("firmware", "dsp_done"; seq = seq);
    ulog::flush();
    userlib::delay_ms(100);

    // 4. WA
    load_ram(dev, mcu_ring, fwdl_ring, FW_WA, "WA", fw_start::WORKING_PDA_CR4, &mut seq, &mut irq)?;
    uinfo!("firmware", "wa_done"; seq = seq);
    ulog::flush();

    // 5. Wait for fw_state=7
    if let Some(ref mut fw_irq) = irq {
        let mask = dev.rr(MT_INT_MASK_CSR);
        dev.wr(MT_INT_MASK_CSR, mask | MT_INT_MCU_CMD);
    }

    let mut fw_state = 0u32;
    for _ in 0..50u32 {
        fw_state = dev.rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        if fw_state == 7 { break; }

        if let Some(ref mut fw_irq) = irq {
            let _ = fw_irq.wait();
            let src = dev.rr(MT_INT_SOURCE_CSR);
            if src != 0 { dev.wr(MT_INT_SOURCE_CSR, src); }
            if src & MT_INT_MCU_CMD != 0 {
                let cmd = dev.rr(MT_MCU_CMD);
                dev.wr(MT_MCU_CMD, cmd);
            }
            fw_irq.ack();
        } else {
            userlib::delay_ms(20);
        }
    }

    if fw_state != 7 {
        uerror!("firmware", "incomplete"; fw_state = fw_state);
        return Err(McuError::RxTimeout { cmd: "fw_state", seq: 0 });
    }

    // Do NOT drain RX queues by setting CPU_IDX = DMA_IDX — that starves
    // hardware of buffer slots (DMA_IDX == CPU_IDX = zero available).
    // Stale firmware-loading responses are harmless: wait_rx_response
    // skips entries with non-matching seq numbers.

    unotice!("firmware", "load_success"; fw_state = fw_state);
    Ok(())
}
