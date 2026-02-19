//! Persistent settings stored in the last sector of external SPI flash (W25Q64JVSS).

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

/// Flash size: W25Q64JVSS = 8MB
pub const FLASH_SIZE: usize = 8 * 1024 * 1024;

/// Settings are stored in the last 4KB sector
const SETTINGS_OFFSET: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;

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

/// Load settings from flash. Returns defaults if no valid settings found.
pub fn load(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) -> PersistedSettings {
    let mut buf = [0u8; 4];
    if let Err(e) = flash.blocking_read(SETTINGS_OFFSET, &mut buf) {
        defmt::warn!("Flash read failed: {:?}", defmt::Debug2Format(&e));
        return PersistedSettings::default();
    }

    if buf[0] != MAGIC {
        defmt::info!("No saved settings found (magic={:#04x}), using defaults", buf[0]);
        return PersistedSettings::default();
    }

    let volume = buf[1].min(100);
    let muted = buf[2] != 0;

    defmt::info!("Loaded settings: volume={}, muted={}", volume, muted);
    PersistedSettings { volume, muted }
}

/// Save settings to flash (erases sector, then writes 4 bytes).
pub fn save(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>, settings: &PersistedSettings) {
    let buf = [MAGIC, settings.volume, settings.muted as u8, 0xFF];

    if let Err(e) = flash.blocking_erase(SETTINGS_OFFSET, SETTINGS_OFFSET + ERASE_SIZE as u32) {
        defmt::error!("Flash erase failed: {:?}", defmt::Debug2Format(&e));
        return;
    }

    if let Err(e) = flash.blocking_write(SETTINGS_OFFSET, &buf) {
        defmt::error!("Flash write failed: {:?}", defmt::Debug2Format(&e));
        return;
    }

    defmt::info!("Settings saved: volume={}, muted={}", settings.volume, settings.muted);
}
