//! Firmware Loading — patch, RAM, DSP binary format structs + load sequences
//!
//! EXACT translation of Linux mt7996/mcu.c firmware loading code.
//! Contains embedded firmware bytes, binary format parsers, and the
//! load_firmware() entry point.

use userlib::{uinfo, uerror, udebug};
use crate::regs::*;
use crate::device::Mt7996Dev;
use crate::dma::TxRing;
use crate::mcu::{dl_mode, fw_feature, fw_start, gen_dl_mode};

// ============================================================================
// Embedded firmware images (loaded at compile time)
// ============================================================================

pub static FW_ROM_PATCH: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_rom_patch.bin");
pub static FW_WM: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_wm.bin");
pub static FW_DSP: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_dsp.bin");
pub static FW_WA: &[u8] = include_bytes!("../../../../firmware/mediatek/mt7996/mt7996_wa.bin");

// ============================================================================
// Firmware Header Structures
// Source: mcu.c struct mt7996_patch_hdr, mt7996_fw_trailer, mt7996_fw_region
// ============================================================================

/// Patch header descriptor (inner struct)
/// From Linux mcu.c:20-27
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

/// Patch header — at START of patch file
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

/// Patch section — 64 bytes each
/// From Linux mcu.c:30-44
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

/// Firmware trailer — at END of firmware file
/// From Linux mcu.c:46-56
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

/// Firmware region descriptor
/// From mt76_connac_mcu.h:184-194 — MUST be 40 bytes!
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
// Firmware Loading Functions on Mt7996Dev
// ============================================================================

impl Mt7996Dev {
    /// Load patch firmware
    /// From Linux mcu.c:2208-2296
    fn load_patch(&self, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing, fw_buf: &[u8], seq: &mut u8) -> Result<(), i32> {
        uinfo!("fw", "load_patch_start");

        let fw_state = self.mt76_rr(MT_TOP_MISC);
        udebug!("fw", "fw_state_before"; val = fw_state);
        udebug!("fw", "dma_state"; glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG), rst = self.mt76_rr(MT_WFDMA0_RST));

        let fwdl_base = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DESC_BASE);
        let fwdl_cpu = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_CPU_IDX);
        let fwdl_dma = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DMA_IDX);
        udebug!("fw", "fwdl_ring"; base = fwdl_base, cpu = fwdl_cpu, dma = fwdl_dma);

        let hdr_size = core::mem::size_of::<PatchHeader>();
        let sec_size = core::mem::size_of::<PatchSec>();

        if fw_buf.len() < hdr_size {
            uerror!("fw", "patch_too_small"; size = fw_buf.len());
            return Err(-1);
        }

        let hdr = unsafe { &*(fw_buf.as_ptr() as *const PatchHeader) };
        let hw_ver = u32::from_be(hdr.hw_sw_ver);
        let patch_ver = u32::from_be(hdr.patch_ver);
        let n_region = u32::from_be(hdr.desc.n_region);
        uinfo!("fw", "patch_header"; hw = hw_ver, ver = patch_ver, regions = n_region);

        // Get semaphore (MCU command → mcu_ring)
        self.mcu_patch_sem_ctrl(mcu_ring, true, *seq)?;
        *seq = seq.wrapping_add(1);

        for i in 0..n_region {
            let sec_offset = hdr_size + (i as usize * sec_size);
            if sec_offset + sec_size > fw_buf.len() {
                uerror!("fw", "section_oob"; idx = i);
                return Err(-1);
            }

            let sec = unsafe { &*(fw_buf.as_ptr().add(sec_offset) as *const PatchSec) };

            let sec_type = u32::from_be(sec.sec_type);
            if (sec_type & 0xFFFF) != 0x2 {
                uerror!("fw", "bad_sec_type"; sec_type = sec_type);
                return Err(-1);
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
                _ => {
                    return Err(-1);
                }
            };

            udebug!("fw", "patch_region"; idx = i, addr = addr, len = len);

            if data_offs + len as usize > fw_buf.len() {
                uerror!("fw", "section_data_oob"; idx = i);
                return Err(-1);
            }

            self.mcu_init_download(mcu_ring, addr, len, mode, *seq)?;
            *seq = seq.wrapping_add(1);

            let section_data = &fw_buf[data_offs..data_offs + len as usize];
            for chunk in section_data.chunks(MCU_FW_DL_BUF_SIZE) {
                self.mcu_send_firmware_chunk(fwdl_ring, chunk, *seq)?;
                *seq = seq.wrapping_add(1);
            }

            let fwdl_cpu = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_CPU_IDX);
            let fwdl_dma = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DMA_IDX);
            udebug!("fw", "patch_region_done"; idx = i, cpu = fwdl_cpu, dma = fwdl_dma);
        }

        userlib::delay_ms(100);

        self.mcu_start_firmware(mcu_ring, true, 0, 0, *seq)?;
        *seq = seq.wrapping_add(1);

        let fwdl_cpu = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_CPU_IDX);
        let fwdl_dma = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DMA_IDX);
        uinfo!("fw", "load_patch_done"; cpu = fwdl_cpu, dma = fwdl_dma);
        Ok(())
    }

    /// Load RAM firmware (WM, DSP, or WA)
    /// Exact translation of Linux mt7996_mcu_send_ram_firmware() (mcu.c:3032-3083).
    fn load_ram(&self, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing, fw_buf: &[u8], _name: &str, fw_start_bits: u32, seq: &mut u8) -> Result<(), i32> {
        uinfo!("fw", "load_ram_start"; size = fw_buf.len());

        if fw_buf.len() < core::mem::size_of::<FwTrailer>() {
            uerror!("fw", "ram_too_small"; size = fw_buf.len());
            return Err(-1);
        }

        let trailer_offset = fw_buf.len() - core::mem::size_of::<FwTrailer>();
        let trailer = unsafe { &*(fw_buf.as_ptr().add(trailer_offset) as *const FwTrailer) };

        let chip_id = trailer.chip_id;
        let n_region = trailer.n_region;
        uinfo!("fw", "ram_header"; chip_id = chip_id, regions = n_region);

        let region_size = core::mem::size_of::<FwRegion>();
        let regions_end = trailer_offset;
        let regions_start = regions_end - (n_region as usize * region_size);

        let mut override_addr: u32 = 0;
        let mut data_offset = 0usize;

        for i in 0..n_region {
            let region_ptr = fw_buf.as_ptr().wrapping_add(regions_start + i as usize * region_size);
            let region = unsafe { &*(region_ptr as *const FwRegion) };

            let addr = u32::from_le(region.addr);
            let len = u32::from_le(region.len);

            let is_not_wm = fw_start_bits != 0;
            let mode = gen_dl_mode(region.feature_set, is_not_wm);

            if region.feature_set & fw_feature::OVERRIDE_ADDR != 0 {
                override_addr = addr;
            }

            udebug!("fw", "ram_region"; idx = i, addr = addr, len = len);

            unsafe { core::arch::asm!("dsb sy", "isb") };

            self.mcu_init_download(mcu_ring, addr, len, mode, *seq)?;
            *seq = seq.wrapping_add(1);

            let region_data = &fw_buf[data_offset..data_offset + len as usize];
            for chunk in region_data.chunks(MCU_FW_DL_BUF_SIZE) {
                self.mcu_send_firmware_chunk(fwdl_ring, chunk, *seq)?;
                *seq = seq.wrapping_add(1);
            }

            data_offset += len as usize;
        }

        let mut option: u32 = 0;
        if override_addr != 0 {
            option |= fw_start::OVERRIDE;
        }
        option |= fw_start_bits;

        self.mcu_start_firmware(mcu_ring, false, option, override_addr, *seq)?;
        *seq = seq.wrapping_add(1);

        uinfo!("fw", "load_ram_done");
        Ok(())
    }

    /// Main firmware loading entry point
    ///
    /// Loads patch → WM → DSP → WA, then polls for FW_STATE_RDY(7).
    /// After firmware boots, runs post-init MCU commands.
    pub fn load_firmware(&self, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing) -> Result<(), i32> {
        uinfo!("fw", "load_firmware_start");

        let mut seq: u8 = 1;

        let fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        uinfo!("fw", "fw_state_before"; val = fw_state);

        // 1. Load patch
        uinfo!("fw", "load_patch"; size = FW_ROM_PATCH.len() as u32);
        self.load_patch(mcu_ring, fwdl_ring, FW_ROM_PATCH, &mut seq)?;

        userlib::delay_ms(100);

        // 2. Load WM
        uinfo!("fw", "load_wm"; size = FW_WM.len() as u32);
        self.load_ram(mcu_ring, fwdl_ring, FW_WM, "WM", 0, &mut seq)?;

        userlib::delay_ms(100);

        // 3. Load DSP
        uinfo!("fw", "load_dsp"; size = FW_DSP.len() as u32);
        self.load_ram(mcu_ring, fwdl_ring, FW_DSP, "DSP", fw_start::WORKING_PDA_DSP, &mut seq)?;

        userlib::delay_ms(100);

        // 4. Load WA
        uinfo!("fw", "load_wa"; size = FW_WA.len() as u32);
        self.load_ram(mcu_ring, fwdl_ring, FW_WA, "WA", fw_start::WORKING_PDA_CR4, &mut seq)?;

        // Wait for fw_state=7
        let mut fw_state = 0u32;
        for _i in 0..50u32 {
            fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
            if fw_state == 7 {
                break;
            }
            userlib::delay_ms(20);
        }
        uinfo!("fw", "fw_state_final"; val = fw_state);

        if fw_state != 7 {
            uerror!("fw", "load_firmware_incomplete"; state = fw_state);
            return Err(-1);
        }

        uinfo!("fw", "load_firmware_success");
        Ok(())
    }
}
