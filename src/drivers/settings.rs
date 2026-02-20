//! Persistent settings stored in the last sector of external SPI flash (W25Q64JVSS).
//!
//! Uses an append-log within a single 4KB sector to avoid erasing on every save.
//! Each save appends a 4-byte entry to the next slot. Only erases when the sector
//! is full (~1024 writes). A page write takes ~0.7ms vs ~45ms for a sector erase,
//! avoiding audible audio stutter.

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

/// Flash size: W25Q64JVSS = 8MB
pub const FLASH_SIZE: usize = 8 * 1024 * 1024;

/// Settings are stored in the last 4KB sector
const SETTINGS_OFFSET: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;

/// Size of each settings entry
const ENTRY_SIZE: u32 = 4;

/// Number of entries that fit in one sector
const MAX_ENTRIES: u32 = ERASE_SIZE as u32 / ENTRY_SIZE;

/// Magic byte to validate stored settings
const MAGIC: u8 = 0xA5;

/// Persisted settings loaded from / saved to flash
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct PersistedSettings {
    pub volume: u8,
    pub muted: bool,
}

impl Default for PersistedSettings {
    fn default() -> Self {
        Self {
            volume: 50,
            muted: false,
        }
    }
}

/// Find the index of the last valid entry in the sector.
/// Returns None if no valid entries exist (erased or corrupt sector).
fn find_last_entry(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) -> Option<u32> {
    // Scan forward to find the last written slot.
    // Erased flash reads as 0xFF, so the first slot where byte 0 == 0xFF
    // marks the end of written data.
    let mut buf = [0u8; 4];
    let mut last_valid = None;

    for i in 0..MAX_ENTRIES {
        let offset = SETTINGS_OFFSET + i * ENTRY_SIZE;
        if flash.blocking_read(offset, &mut buf).is_err() {
            break;
        }
        if buf[0] == MAGIC {
            last_valid = Some(i);
        } else if buf[0] == 0xFF {
            // Reached erased area, stop scanning
            break;
        }
        // Skip corrupt entries (non-magic, non-0xFF)
    }

    last_valid
}

/// Load settings from flash. Returns defaults if no valid settings found.
pub fn load(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) -> PersistedSettings {
    let Some(idx) = find_last_entry(flash) else {
        defmt::info!("No saved settings found, using defaults");
        return PersistedSettings::default();
    };

    let mut buf = [0u8; 4];
    let offset = SETTINGS_OFFSET + idx * ENTRY_SIZE;
    if flash.blocking_read(offset, &mut buf).is_err() {
        return PersistedSettings::default();
    }

    let volume = buf[1].min(100);
    let muted = buf[2] != 0;

    defmt::info!(
        "Loaded settings from slot {}: volume={}, muted={}",
        idx,
        volume,
        muted
    );
    PersistedSettings { volume, muted }
}

/// Save settings to flash by appending to the next free slot.
/// Only erases the sector when full (~1024 writes between erases).
pub fn save(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>, settings: &PersistedSettings) {
    let buf = [MAGIC, settings.volume, settings.muted as u8, 0xFF];

    // Find the next free slot (first 0xFF byte after last valid entry)
    let next_idx = match find_last_entry(flash) {
        Some(idx) => idx + 1,
        None => {
            // Check if slot 0 is erased (0xFF) or corrupt
            let mut first = [0u8; 1];
            if flash.blocking_read(SETTINGS_OFFSET, &mut first).is_ok() && first[0] != 0xFF {
                // Sector has corrupt data, erase it
                defmt::warn!("Corrupt settings sector, erasing");
                if let Err(e) =
                    flash.blocking_erase(SETTINGS_OFFSET, SETTINGS_OFFSET + ERASE_SIZE as u32)
                {
                    defmt::error!("Flash erase failed: {:?}", defmt::Debug2Format(&e));
                    return;
                }
            }
            0
        }
    };

    if next_idx >= MAX_ENTRIES {
        // Sector full — erase and start from slot 0
        defmt::info!("Settings sector full, erasing");
        if let Err(e) = flash.blocking_erase(SETTINGS_OFFSET, SETTINGS_OFFSET + ERASE_SIZE as u32) {
            defmt::error!("Flash erase failed: {:?}", defmt::Debug2Format(&e));
            return;
        }
        let offset = SETTINGS_OFFSET;
        if let Err(e) = flash.blocking_write(offset, &buf) {
            defmt::error!("Flash write failed: {:?}", defmt::Debug2Format(&e));
            return;
        }
        defmt::info!(
            "Settings saved to slot 0 (after erase): volume={}, muted={}",
            settings.volume,
            settings.muted
        );
        return;
    }

    let offset = SETTINGS_OFFSET + next_idx * ENTRY_SIZE;
    if let Err(e) = flash.blocking_write(offset, &buf) {
        defmt::error!("Flash write failed: {:?}", defmt::Debug2Format(&e));
        return;
    }

    defmt::info!(
        "Settings saved to slot {}: volume={}, muted={}",
        next_idx,
        settings.volume,
        settings.muted
    );
}
