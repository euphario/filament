//! Banana Pi BPI-R4 Board Configuration
//!
//! The BPI-R4 has the following PCIe slots:
//! - Port 0: mPCIe SIM2 slot (typically WiFi module)
//! - Port 1: mPCIe SIM3 slot (typically WiFi module)
//! - Port 2: M.2 Key-B slot (typically LTE/5G modem)
//! - Port 3: M.2 Key-M slot (NVMe SSD)

use super::{Board, BoardError, SlotInfo};
use crate::soc::Mt7988aSoc;

/// BPI-R4 slot configuration
const BPI_R4_SLOTS: [SlotInfo; 4] = [
    SlotInfo {
        port: 0,
        slot_name: "mPCIe SIM2",
        available: true,
        expected_device: Some("WiFi"),
    },
    SlotInfo {
        port: 1,
        slot_name: "mPCIe SIM3",
        available: true,
        expected_device: Some("WiFi"),
    },
    SlotInfo {
        port: 2,
        slot_name: "M.2 Key-B",
        available: true,
        expected_device: Some("LTE/5G Modem"),
    },
    SlotInfo {
        port: 3,
        slot_name: "M.2 Key-M",
        available: true,
        expected_device: Some("NVMe SSD"),
    },
];

/// BPI-R4 board configuration
pub struct BpiR4 {
    soc: Mt7988aSoc,
}

impl BpiR4 {
    /// Create a new BPI-R4 board configuration
    pub fn new() -> Result<Self, BoardError> {
        let soc = Mt7988aSoc::new()
            .map_err(|_| BoardError::SocInitFailed)?;
        Ok(Self { soc })
    }
}

impl Board for BpiR4 {
    type Soc = Mt7988aSoc;

    fn name(&self) -> &'static str {
        "Banana Pi BPI-R4"
    }

    fn soc(&self) -> &Self::Soc {
        &self.soc
    }

    fn soc_mut(&mut self) -> &mut Self::Soc {
        &mut self.soc
    }

    fn slot_info(&self, port: u8) -> Option<SlotInfo> {
        BPI_R4_SLOTS.get(port as usize).copied()
    }

    fn slots(&self) -> &[SlotInfo] {
        &BPI_R4_SLOTS
    }

    fn pre_init(&mut self) -> Result<(), BoardError> {
        // BPI-R4 doesn't require special power sequencing for PCIe
        // The slots are always powered when the board is on
        Ok(())
    }
}
