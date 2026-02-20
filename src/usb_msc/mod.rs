//! USB Mass Storage Class — Bulk-Only Transport (BBB).
//!
//! Implements a minimal USB Mass Storage device using embassy-usb raw
//! bulk endpoints. Presents a virtual block device backed by a FAT12
//! filesystem with a single `DSP.CFG` file.
//!
//! Protocol flow:
//! ```text
//! Host → CBW (31 bytes) on bulk-out
//!   Device parses SCSI command
//!   Device ↔ Host: data phase (bulk-in or bulk-out)
//! Device → CSW (13 bytes) on bulk-in
//! ```

pub mod fat12;

use embassy_usb::driver::{Driver, EndpointIn, EndpointOut};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::Builder;

/// USB Mass Storage class code
const USB_CLASS_MASS_STORAGE: u8 = 0x08;
/// SCSI transparent command set
const USB_SUBCLASS_SCSI: u8 = 0x06;
/// Bulk-Only Transport
const USB_PROTOCOL_BBB: u8 = 0x50;

/// CBW signature: "USBC"
const CBW_SIGNATURE: u32 = 0x43425355;
/// CSW signature: "USBS"
const CSW_SIGNATURE: u32 = 0x53425355;

/// CBW size
const CBW_SIZE: usize = 31;
/// CSW size
const CSW_SIZE: usize = 13;

/// Block size (standard sector size)
pub const BLOCK_SIZE: usize = 512;

/// CSW status codes
const CSW_STATUS_PASSED: u8 = 0x00;
const CSW_STATUS_FAILED: u8 = 0x01;

// ── SCSI command opcodes ────────────────────────────────────────────────────

const SCSI_TEST_UNIT_READY: u8 = 0x00;
const SCSI_REQUEST_SENSE: u8 = 0x03;
const SCSI_INQUIRY: u8 = 0x12;
const SCSI_START_STOP_UNIT: u8 = 0x1B;
const SCSI_MODE_SENSE_6: u8 = 0x1A;
const SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1E;
const SCSI_READ_FORMAT_CAPACITIES: u8 = 0x23;
const SCSI_READ_CAPACITY_10: u8 = 0x25;
const SCSI_READ_10: u8 = 0x28;
const SCSI_WRITE_10: u8 = 0x2A;
const SCSI_SYNCHRONIZE_CACHE: u8 = 0x35;

/// Command Block Wrapper (parsed)
struct Cbw {
    tag: u32,
    data_transfer_length: u32,
    direction: Direction,
    cb: [u8; 16],
    cb_len: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Direction {
    Out, // Host → Device
    In,  // Device → Host
}

impl Cbw {
    fn parse(data: &[u8; CBW_SIZE]) -> Option<Self> {
        let sig = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        if sig != CBW_SIGNATURE {
            return None;
        }
        let tag = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let data_transfer_length = u32::from_le_bytes([data[8], data[9], data[10], data[11]]);
        let direction = if data[12] & 0x80 != 0 {
            Direction::In
        } else {
            Direction::Out
        };
        let cb_len = data[14] & 0x1F;
        let mut cb = [0u8; 16];
        cb.copy_from_slice(&data[15..31]);

        Some(Self {
            tag,
            data_transfer_length,
            direction,
            cb,
            cb_len,
        })
    }
}

/// Build a CSW response.
fn build_csw(tag: u32, data_residue: u32, status: u8) -> [u8; CSW_SIZE] {
    let mut csw = [0u8; CSW_SIZE];
    csw[0..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
    csw[4..8].copy_from_slice(&tag.to_le_bytes());
    csw[8..12].copy_from_slice(&data_residue.to_le_bytes());
    csw[12] = status;
    csw
}

/// USB Mass Storage class configuration.
///
/// Call [`MscClass::new()`] during USB builder setup, then run
/// [`MscClass::run()`] as a task when in config mode.
pub struct MscClass<'d, D: Driver<'d>> {
    ep_in: D::EndpointIn,
    ep_out: D::EndpointOut,
    _iface: InterfaceNumber,
    max_packet_size: usize,
}

impl<'d, D: Driver<'d>> MscClass<'d, D> {
    /// Create a new Mass Storage class on the given USB builder.
    pub fn new(builder: &mut Builder<'d, D>, max_packet_size: u16) -> Self {
        let mut func = builder.function(
            USB_CLASS_MASS_STORAGE,
            USB_SUBCLASS_SCSI,
            USB_PROTOCOL_BBB,
        );

        let mut iface = func.interface();
        let iface_num = iface.interface_number();
        let mut alt = iface.alt_setting(
            USB_CLASS_MASS_STORAGE,
            USB_SUBCLASS_SCSI,
            USB_PROTOCOL_BBB,
            None,
        );

        let ep_in = alt.endpoint_bulk_in(None, max_packet_size);
        let ep_out = alt.endpoint_bulk_out(None, max_packet_size);

        drop(func);

        Self {
            ep_in,
            ep_out,
            _iface: iface_num,
            max_packet_size: max_packet_size as usize,
        }
    }

    /// Run the MSC transport loop.
    ///
    /// This handles CBW/CSW exchanges and dispatches SCSI commands to the
    /// virtual FAT12 filesystem. Call from a task that runs while in config mode.
    pub async fn run(&mut self, vfs: &mut fat12::VirtualFat12) {
        let mut cbw_buf = [0u8; 64]; // CBW is 31 bytes, but endpoint may read up to max_packet

        loop {
            // Wait for CBW from host
            let n = match self.ep_out.read(&mut cbw_buf).await {
                Ok(n) => n,
                Err(_) => continue,
            };

            if n < CBW_SIZE {
                continue;
            }

            let cbw_bytes: &[u8; CBW_SIZE] = match cbw_buf[..CBW_SIZE].try_into() {
                Ok(b) => b,
                Err(_) => continue,
            };

            let Some(cbw) = Cbw::parse(cbw_bytes) else {
                continue;
            };

            let (status, residue) = self.handle_scsi(&cbw, vfs).await;

            // Send CSW
            let csw = build_csw(cbw.tag, residue, status);
            let _ = self.ep_in.write(&csw).await;

            // After eject, reboot into normal mode
            if vfs.ejected {
                // Brief delay to let the USB host process the CSW
                for _ in 0..1_000_000u32 {
                    cortex_m::asm::nop();
                }
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }

    /// Handle a SCSI command and perform the data phase.
    ///
    /// Returns (CSW status, data residue).
    async fn handle_scsi(&mut self, cbw: &Cbw, vfs: &mut fat12::VirtualFat12) -> (u8, u32) {
        let opcode = cbw.cb[0];

        match opcode {
            SCSI_TEST_UNIT_READY => (CSW_STATUS_PASSED, 0),

            SCSI_INQUIRY => {
                let mut resp = [0u8; 36];
                resp[0] = 0x00; // Direct access block device
                resp[1] = 0x80; // Removable media
                resp[2] = 0x02; // SPC-2 compliance
                resp[3] = 0x02; // Response data format
                resp[4] = 31; // Additional length
                // Vendor: "OtterAmp" (8 bytes, space-padded)
                resp[8..16].copy_from_slice(b"OtterAmp");
                // Product: "DSP Config      " (16 bytes, space-padded)
                resp[16..32].copy_from_slice(b"DSP Config      ");
                // Revision: "1.0 " (4 bytes)
                resp[32..36].copy_from_slice(b"1.0 ");

                let send_len = (cbw.data_transfer_length as usize).min(resp.len());
                let _ = self.ep_in.write(&resp[..send_len]).await;
                (CSW_STATUS_PASSED, cbw.data_transfer_length - send_len as u32)
            }

            SCSI_START_STOP_UNIT => {
                let loej = cbw.cb[4] & 0x02; // Load/Eject bit
                let start = cbw.cb[4] & 0x01; // Start bit
                if loej != 0 && start == 0 {
                    // Eject requested — flush and signal reboot
                    vfs.flush();
                }
                (CSW_STATUS_PASSED, 0)
            }

            SCSI_SYNCHRONIZE_CACHE => {
                (CSW_STATUS_PASSED, 0)
            }

            SCSI_READ_FORMAT_CAPACITIES => {
                // UFI command — macOS sends this to discover media capacity.
                let total_sectors = fat12::TOTAL_SECTORS as u32;
                let mut resp = [0u8; 12];
                // Capacity list header: list length = 8
                resp[3] = 8;
                // Current capacity descriptor
                resp[4..8].copy_from_slice(&total_sectors.to_be_bytes());
                resp[8] = 0x02; // Descriptor code: formatted media
                // Block length (3 bytes, big-endian)
                let bs = BLOCK_SIZE as u32;
                resp[9] = ((bs >> 16) & 0xFF) as u8;
                resp[10] = ((bs >> 8) & 0xFF) as u8;
                resp[11] = (bs & 0xFF) as u8;
                let send_len = (cbw.data_transfer_length as usize).min(resp.len());
                let _ = self.ep_in.write(&resp[..send_len]).await;
                (CSW_STATUS_PASSED, cbw.data_transfer_length - send_len as u32)
            }

            SCSI_READ_CAPACITY_10 => {
                let total_sectors = fat12::TOTAL_SECTORS as u32;
                let last_lba = total_sectors - 1;
                let mut resp = [0u8; 8];
                resp[0..4].copy_from_slice(&last_lba.to_be_bytes());
                resp[4..8].copy_from_slice(&(BLOCK_SIZE as u32).to_be_bytes());
                let _ = self.ep_in.write(&resp).await;
                (CSW_STATUS_PASSED, 0)
            }

            SCSI_READ_10 => {
                let lba = u32::from_be_bytes([cbw.cb[2], cbw.cb[3], cbw.cb[4], cbw.cb[5]]);
                let blocks = u16::from_be_bytes([cbw.cb[7], cbw.cb[8]]) as u32;

                let mut sector_buf = [0u8; BLOCK_SIZE];
                let mut residue = cbw.data_transfer_length;

                for i in 0..blocks {
                    vfs.read_sector(lba + i, &mut sector_buf);
                    let send = (residue as usize).min(BLOCK_SIZE);
                    // Send in max_packet_size chunks (endpoint rejects larger writes)
                    let mut sent = 0;
                    while sent < send {
                        let chunk = (send - sent).min(self.max_packet_size);
                        let _ = self.ep_in.write(&sector_buf[sent..sent + chunk]).await;
                        sent += chunk;
                    }
                    residue = residue.saturating_sub(send as u32);
                }

                (CSW_STATUS_PASSED, residue)
            }

            SCSI_WRITE_10 => {
                let lba = u32::from_be_bytes([cbw.cb[2], cbw.cb[3], cbw.cb[4], cbw.cb[5]]);
                let blocks = u16::from_be_bytes([cbw.cb[7], cbw.cb[8]]) as u32;

                let mut sector_buf = [0u8; BLOCK_SIZE];
                let mut residue = cbw.data_transfer_length;

                for i in 0..blocks {
                    let to_read = (residue as usize).min(BLOCK_SIZE);
                    let mut read = 0;
                    while read < to_read {
                        match self.ep_out.read(&mut sector_buf[read..to_read]).await {
                            Ok(n) => read += n,
                            Err(_) => break,
                        }
                    }
                    vfs.write_sector(lba + i, &sector_buf);
                    residue = residue.saturating_sub(to_read as u32);
                }

                (CSW_STATUS_PASSED, residue)
            }

            SCSI_REQUEST_SENSE => {
                // Return "no sense" — everything is fine
                let mut resp = [0u8; 18];
                resp[0] = 0x70; // Current errors, fixed format
                resp[7] = 10; // Additional sense length
                // Sense key 0 = NO SENSE
                let send_len = (cbw.data_transfer_length as usize).min(resp.len());
                let _ = self.ep_in.write(&resp[..send_len]).await;
                (CSW_STATUS_PASSED, cbw.data_transfer_length - send_len as u32)
            }

            SCSI_MODE_SENSE_6 => {
                // Minimal mode sense: no mode pages, write-allowed
                let resp = [0x03u8, 0x00, 0x00, 0x00]; // Mode data length=3, medium type=0, no WP
                let send_len = (cbw.data_transfer_length as usize).min(resp.len());
                let _ = self.ep_in.write(&resp[..send_len]).await;
                (CSW_STATUS_PASSED, cbw.data_transfer_length - send_len as u32)
            }

            SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL => {
                let prevent = cbw.cb[4] & 0x01;
                if prevent == 0 {
                    // "Allow removal" = eject → flush to flash
                    vfs.flush();
                }
                (CSW_STATUS_PASSED, 0)
            }

            _ => {
                defmt::warn!("Unknown SCSI opcode: {:#04x}", opcode);
                // For unknown commands with data-in, we need to stall or send zeros
                if cbw.direction == Direction::In && cbw.data_transfer_length > 0 {
                    // Send zeros to satisfy the data phase
                    let zeros = [0u8; 64];
                    let mut remaining = cbw.data_transfer_length as usize;
                    while remaining > 0 {
                        let chunk = remaining.min(zeros.len());
                        let _ = self.ep_in.write(&zeros[..chunk]).await;
                        remaining -= chunk;
                    }
                }
                (CSW_STATUS_FAILED, 0)
            }
        }
    }
}
