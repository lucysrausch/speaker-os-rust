//! Virtual FAT12 filesystem with a single `DSP.CFG` file.
//!
//! Presents a tiny virtual USB drive (32 KB) to the host OS.
//! The FAT and root directory sectors are writable — the host OS
//! manages them freely. On eject, we extract DSP.CFG by parsing the
//! host-written root directory and flush its data to flash.
//!
//! Layout (64 sectors × 512 bytes = 32 KB):
//! ```text
//! Sector 0:     Boot sector (BPB) — read-only, generated on the fly
//! Sector 1:     FAT table — writable
//! Sector 2:     Root directory (16 entries × 32 bytes) — writable
//! Sectors 3-63: Data area (61 sectors, ~30 KB max file) — writable
//! ```

use super::BLOCK_SIZE;

/// Total sectors in the virtual disk.
pub const TOTAL_SECTORS: usize = 64;

/// First data sector (after boot + FAT + root dir).
const DATA_START_SECTOR: u32 = 3;

/// Maximum file size (data area size).
const MAX_FILE_SIZE: usize = (TOTAL_SECTORS - DATA_START_SECTOR as usize) * BLOCK_SIZE;

/// Volume label
const VOLUME_LABEL: &[u8; 11] = b"OTTERAMP   ";

/// Filename in 8.3 format
const FILE_NAME: &[u8; 11] = b"DSP     CFG";

/// Virtual FAT12 filesystem.
///
/// FAT and root directory sectors are stored in RAM so the host OS
/// can read back its own writes. Data sectors are stored in `file_data`.
pub struct VirtualFat12 {
    /// Stored FAT sector — host reads/writes go here.
    fat_sector: [u8; BLOCK_SIZE],
    /// Stored root directory sector — host reads/writes go here.
    root_dir_sector: [u8; BLOCK_SIZE],
    /// Data area storage.
    file_data: [u8; MAX_FILE_SIZE],
    /// Whether any sector has been modified by the host.
    dirty: bool,
    /// Set after flush — signals MSC layer to reboot.
    pub ejected: bool,
    /// Flash read callback: reads current config data.
    flash_read: fn(&mut [u8]) -> u16,
    /// Flash write callback: writes new config data.
    flash_write: fn(&[u8]),
}

impl VirtualFat12 {
    /// Create a new virtual FAT12 filesystem.
    ///
    /// - `flash_read`: Read config from flash into buffer, return length.
    /// - `flash_write`: Write config data to flash.
    pub fn new(flash_read: fn(&mut [u8]) -> u16, flash_write: fn(&[u8])) -> Self {
        let mut vfs = Self {
            fat_sector: [0u8; BLOCK_SIZE],
            root_dir_sector: [0u8; BLOCK_SIZE],
            file_data: [0u8; MAX_FILE_SIZE],
            dirty: false,
            ejected: false,
            flash_read,
            flash_write,
        };
        // Load current config from flash
        let file_size = (vfs.flash_read)(&mut vfs.file_data);
        // Build initial FAT and root directory from loaded data
        vfs.init_fat(file_size);
        vfs.init_root_dir(file_size);
        vfs
    }

    /// Read a sector from the virtual disk.
    pub fn read_sector(&self, lba: u32, buf: &mut [u8; BLOCK_SIZE]) {
        *buf = [0u8; BLOCK_SIZE];

        match lba {
            0 => self.build_boot_sector(buf),
            1 => buf.copy_from_slice(&self.fat_sector),
            2 => buf.copy_from_slice(&self.root_dir_sector),
            _ => {
                // Data area — return whatever is stored (host may have written)
                let data_sector = (lba - DATA_START_SECTOR) as usize;
                let offset = data_sector * BLOCK_SIZE;
                if offset + BLOCK_SIZE <= MAX_FILE_SIZE {
                    buf.copy_from_slice(&self.file_data[offset..offset + BLOCK_SIZE]);
                } else if offset < MAX_FILE_SIZE {
                    let len = MAX_FILE_SIZE - offset;
                    buf[..len].copy_from_slice(&self.file_data[offset..]);
                }
            }
        }
    }

    /// Write a sector to the virtual disk.
    pub fn write_sector(&mut self, lba: u32, data: &[u8; BLOCK_SIZE]) {
        match lba {
            0 => {} // Ignore boot sector writes
            1 => {
                self.fat_sector.copy_from_slice(data);
                self.dirty = true;
            }
            2 => {
                self.root_dir_sector.copy_from_slice(data);
                self.dirty = true;
            }
            _ => {
                let data_sector = (lba - DATA_START_SECTOR) as usize;
                let offset = data_sector * BLOCK_SIZE;
                if offset + BLOCK_SIZE <= MAX_FILE_SIZE {
                    self.file_data[offset..offset + BLOCK_SIZE].copy_from_slice(data);
                    self.dirty = true;
                } else if offset < MAX_FILE_SIZE {
                    let len = MAX_FILE_SIZE - offset;
                    self.file_data[offset..].copy_from_slice(&data[..len]);
                    self.dirty = true;
                }
            }
        }
    }

    /// Flush DSP.CFG to flash by parsing the host-written root directory.
    ///
    /// Called on eject. Finds the DSP.CFG entry, reads its start cluster
    /// and file size, and writes the corresponding data to flash.
    pub fn flush(&mut self) {
        if !self.dirty {
            self.ejected = true;
            return;
        }

        // Scan root directory for DSP.CFG
        for i in 0..16 {
            let off = i * 32;
            let entry = &self.root_dir_sector[off..off + 32];

            // Skip empty and deleted entries
            if entry[0] == 0x00 || entry[0] == 0xE5 {
                continue;
            }
            if entry[0..11] != *FILE_NAME {
                continue;
            }

            let start_cluster = u16::from_le_bytes([entry[26], entry[27]]);
            let file_size =
                u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]) as usize;

            if file_size == 0 || start_cluster < 2 {
                break;
            }
            let file_size = file_size.min(MAX_FILE_SIZE);

            // Convert start cluster to data offset (1 sector per cluster)
            let data_offset = (start_cluster as usize - 2) * BLOCK_SIZE;
            if data_offset + file_size <= MAX_FILE_SIZE {
                defmt::info!(
                    "Flushing config to flash: {} bytes (cluster {})",
                    file_size,
                    start_cluster
                );
                (self.flash_write)(&self.file_data[data_offset..data_offset + file_size]);
            }
            break;
        }

        self.dirty = false;
        self.ejected = true;
    }

    // ── Initialization ──────────────────────────────────────────────────

    /// Build the initial FAT sector from the loaded file size.
    fn init_fat(&mut self, file_size: u16) {
        self.fat_sector[0] = 0xF8; // Media descriptor
        self.fat_sector[1] = 0xFF;
        self.fat_sector[2] = 0xFF;

        if file_size > 0 {
            let clusters_needed = (file_size as usize + BLOCK_SIZE - 1) / BLOCK_SIZE;
            for i in 0..clusters_needed {
                let cluster = (i + 2) as u16;
                let next = if i + 1 < clusters_needed {
                    (cluster + 1) as u16
                } else {
                    0xFFF // End of chain
                };
                write_fat12_entry(&mut self.fat_sector, cluster, next);
            }
        }
    }

    /// Build the initial root directory sector from the loaded file size.
    fn init_root_dir(&mut self, file_size: u16) {
        // Entry 0: Volume label
        self.root_dir_sector[0..11].copy_from_slice(VOLUME_LABEL);
        self.root_dir_sector[11] = 0x08; // Volume label attribute

        // Entry 1: DSP.CFG file
        let entry = &mut self.root_dir_sector[32..64];
        entry[0..11].copy_from_slice(FILE_NAME);
        entry[11] = 0x20; // Archive attribute
        // Starting cluster = 2
        entry[26] = 0x02;
        entry[27] = 0x00;
        // File size
        entry[28..32].copy_from_slice(&(file_size as u32).to_le_bytes());
    }

    // ── Sector builders ─────────────────────────────────────────────────

    fn build_boot_sector(&self, buf: &mut [u8; BLOCK_SIZE]) {
        // Jump boot code
        buf[0] = 0xEB;
        buf[1] = 0x3C;
        buf[2] = 0x90;

        // OEM name
        buf[3..11].copy_from_slice(b"OTTERAMP");

        // BIOS Parameter Block (BPB)
        buf[11..13].copy_from_slice(&(BLOCK_SIZE as u16).to_le_bytes()); // Bytes per sector
        buf[13] = 1; // Sectors per cluster
        buf[14..16].copy_from_slice(&1u16.to_le_bytes()); // Reserved sectors (boot sector)
        buf[16] = 1; // Number of FATs
        buf[17..19].copy_from_slice(&16u16.to_le_bytes()); // Root directory entries
        buf[19..21].copy_from_slice(&(TOTAL_SECTORS as u16).to_le_bytes()); // Total sectors
        buf[21] = 0xF8; // Media descriptor (fixed disk)
        buf[22..24].copy_from_slice(&1u16.to_le_bytes()); // Sectors per FAT
        buf[24..26].copy_from_slice(&1u16.to_le_bytes()); // Sectors per track
        buf[26..28].copy_from_slice(&1u16.to_le_bytes()); // Number of heads

        // Extended BPB (FAT12/16)
        buf[36] = 0x00; // Drive number
        buf[38] = 0x29; // Boot signature
        buf[39..43].copy_from_slice(&0x1234_5678u32.to_le_bytes()); // Volume serial
        buf[43..54].copy_from_slice(VOLUME_LABEL); // Volume label
        buf[54..62].copy_from_slice(b"FAT12   "); // FS type

        // Boot signature
        buf[510] = 0x55;
        buf[511] = 0xAA;
    }
}

/// Write a 12-bit FAT12 entry at the given cluster index.
fn write_fat12_entry(fat: &mut [u8], cluster: u16, value: u16) {
    let byte_offset = (cluster as usize * 3) / 2;
    if byte_offset + 1 >= fat.len() {
        return;
    }

    if cluster & 1 == 0 {
        // Even cluster: low 8 bits in byte[n], low 4 bits of high nibble in byte[n+1]
        fat[byte_offset] = (value & 0xFF) as u8;
        fat[byte_offset + 1] = (fat[byte_offset + 1] & 0xF0) | ((value >> 8) & 0x0F) as u8;
    } else {
        // Odd cluster: high 4 bits in byte[n], high 8 bits in byte[n+1]
        fat[byte_offset] = (fat[byte_offset] & 0x0F) | ((value << 4) & 0xF0) as u8;
        fat[byte_offset + 1] = ((value >> 4) & 0xFF) as u8;
    }
}
