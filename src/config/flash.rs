//! Flash I/O for the DSP config region.
//!
//! The config is stored as raw bytes with a small header (magic + u16 length)
//! in a dedicated 32 KB region of SPI flash.
//!
//! ```text
//! Flash layout:
//!   0x780000 - 0x787FFF  DSP config region (32 KB = 8 sectors)
//!     Bytes 0-1: Magic [0xCF, 0x67]
//!     Bytes 2-3: Length (big-endian u16)
//!     Bytes 4+:  Config file data
//! ```

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

use crate::drivers::settings::FLASH_SIZE;

/// Start offset of the config region in flash.
pub const CONFIG_OFFSET: u32 = 0x78_0000;

/// Maximum size of the config region (32 KB = 8 sectors).
pub const CONFIG_MAX_SIZE: usize = 32 * 1024;

/// Maximum config file data size (total region minus 4-byte header).
pub const CONFIG_MAX_DATA: usize = CONFIG_MAX_SIZE - HEADER_SIZE;

/// Number of sectors in the config region.
const CONFIG_SECTORS: u32 = (CONFIG_MAX_SIZE / ERASE_SIZE) as u32;

/// Header size: 2 bytes magic + 2 bytes length.
const HEADER_SIZE: usize = 4;

/// Magic bytes identifying a valid config header.
const MAGIC: [u8; 2] = [0xCF, 0x67];

/// Static read buffer for config data.
/// Allocated once, reused across reads.
static mut CONFIG_BUF: [u8; CONFIG_MAX_SIZE] = [0u8; CONFIG_MAX_SIZE];

/// Read the config file from flash.
///
/// Returns `Some(data)` if a valid config is found, or `None` if the
/// region is empty/corrupt.
///
/// # Safety
/// The returned slice references a static buffer. Must not be called
/// concurrently or while a previous result is still in use.
pub fn read_config(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) -> Option<&'static [u8]> {
    // Read the header first
    let buf = unsafe { &mut CONFIG_BUF };
    if flash.blocking_read(CONFIG_OFFSET, &mut buf[..HEADER_SIZE]).is_err() {
        defmt::warn!("Failed to read config header from flash");
        return None;
    }

    // Check magic
    if buf[0] != MAGIC[0] || buf[1] != MAGIC[1] {
        defmt::debug!("No config magic found at {:#010x}", CONFIG_OFFSET);
        return None;
    }

    // Read length (big-endian u16)
    let len = ((buf[2] as u16) << 8 | buf[3] as u16) as usize;
    if len == 0 || len > CONFIG_MAX_DATA {
        defmt::warn!("Invalid config length: {}", len);
        return None;
    }

    // Read the full config data
    let total = HEADER_SIZE + len;
    if flash.blocking_read(CONFIG_OFFSET, &mut buf[..total]).is_err() {
        defmt::warn!("Failed to read config data from flash");
        return None;
    }

    Some(&buf[HEADER_SIZE..total])
}

/// Write a config file to flash.
///
/// Erases the config sectors and writes the header + data.
pub fn write_config(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>, data: &[u8]) {
    if data.len() > CONFIG_MAX_DATA {
        defmt::error!("Config too large: {} > {}", data.len(), CONFIG_MAX_DATA);
        return;
    }

    // Erase all config sectors
    let end = CONFIG_OFFSET + CONFIG_SECTORS * ERASE_SIZE as u32;
    if let Err(e) = flash.blocking_erase(CONFIG_OFFSET, end) {
        defmt::error!("Config erase failed: {:?}", defmt::Debug2Format(&e));
        return;
    }

    // Build header
    let len = data.len() as u16;
    let header = [MAGIC[0], MAGIC[1], (len >> 8) as u8, (len & 0xFF) as u8];

    // Write header
    if let Err(e) = flash.blocking_write(CONFIG_OFFSET, &header) {
        defmt::error!("Config header write failed: {:?}", defmt::Debug2Format(&e));
        return;
    }

    // Write data in page-sized chunks (flash writes must be page-aligned)
    // RP2350 flash page size is 256 bytes
    const PAGE_SIZE: usize = 256;
    let data_offset = CONFIG_OFFSET + HEADER_SIZE as u32;
    let mut written = 0;

    while written < data.len() {
        let chunk_len = (data.len() - written).min(PAGE_SIZE);
        let offset = data_offset + written as u32;
        if let Err(e) = flash.blocking_write(offset, &data[written..written + chunk_len]) {
            defmt::error!(
                "Config data write failed at offset {:#010x}: {:?}",
                offset,
                defmt::Debug2Format(&e)
            );
            return;
        }
        written += chunk_len;
    }

    defmt::info!("Config written to flash: {} bytes", data.len());
}

/// Erase the config region.
pub fn erase_config(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) {
    let end = CONFIG_OFFSET + CONFIG_SECTORS * ERASE_SIZE as u32;
    if let Err(e) = flash.blocking_erase(CONFIG_OFFSET, end) {
        defmt::error!("Config erase failed: {:?}", defmt::Debug2Format(&e));
    } else {
        defmt::info!("Config region erased");
    }
}
