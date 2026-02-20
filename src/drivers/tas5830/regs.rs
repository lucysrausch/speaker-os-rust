//! TAS5830 register map and DSP memory layout
//!
//! Covers the control-port registers (Book 0, Page 0) and the
//! DSP coefficient RAM locations for EQ, mixer, channel volume,
//! and crossover filters.

// ── Control port: Book 0, Page 0 ────────────────────────────────────────────

/// Page select register — present on every page
pub const PAGE_SELECT: u8 = 0x00;

/// Reset / clock control
pub const RESET_CTRL: u8 = 0x01;

/// Device control 1 — modulation, bridge, switching frequency
pub const DEVICE_CTRL_1: u8 = 0x02;

/// Device control 2 — state machine + soft mute
///
/// Bits \[1:0\]: 0 = Deep Sleep, 1 = Sleep, 2 = Hi-Z, 3 = Play
/// Bit 0 of the mute field: 1 = muted
pub const DEVICE_CTRL_2: u8 = 0x03;

/// S/PDIF-out select (post/pre DSP)
pub const SDOUT_SEL: u8 = 0x30;

/// Clock detection status
pub const CLKDET_STATUS: u8 = 0x39;

/// Master digital volume (0.5 dB/step, 0x00 = 0 dB, 0xFF = −127.5 dB)
pub const DIG_VOL: u8 = 0x4C;

/// Auto-mute timing
pub const AUTO_MUTE_TIME: u8 = 0x51;

/// Analog control
pub const ANA_CTRL: u8 = 0x53;

/// Analog gain (bits 4:0, 0.5 dB/step, 0 = 0 dB, 31 = −15.5 dB)
pub const AGAIN: u8 = 0x54;

/// DSP misc — bit 0: EQ/biquad bypass on TAS5805M-compatible chips
pub const DSP_MISC: u8 = 0x66;

/// Die ID (read-only)
pub const DIE_ID: u8 = 0x67;

/// Power state (read-only, bits 1:0)
pub const POWER_STATE: u8 = 0x68;

/// Channel fault status
pub const CHAN_FAULT: u8 = 0x70;

/// Global fault 1
pub const GLOBAL_FAULT1: u8 = 0x71;

/// Global fault 2
pub const GLOBAL_FAULT2: u8 = 0x72;

/// Over-temperature warning
pub const OT_WARNING: u8 = 0x73;

/// Clear-fault register (write 0x80 to clear)
pub const CLEAR_FAULT: u8 = 0x78;

/// Book select — always at 0x7F on page 0
pub const BOOK_SELECT: u8 = 0x7F;

// ── Device control 1 bit fields ─────────────────────────────────────────────

/// Modulation mode (bits 1:0 of DEVICE_CTRL_1)
pub mod modulation {
    /// Both drivers switching (traditional H-bridge)
    pub const BD: u8 = 0;
    /// Single-ended PWM
    pub const ONE_SPW: u8 = 1;
    /// Adaptive hybrid
    pub const HYBRID: u8 = 2;
}

/// Switching frequency (bits 6:4 of DEVICE_CTRL_1)
pub mod switching_freq {
    pub const F_768K: u8 = 0 << 4;
    pub const F_384K: u8 = 1 << 4;
    pub const F_480K: u8 = 2 << 4;
    pub const F_576K: u8 = 3 << 4;
}

// ── DSP Books ───────────────────────────────────────────────────────────────

/// Book 0 — control port (default)
pub const BOOK_CTRL: u8 = 0x00;

/// Book 0x8C — mixer, channel volume, crossover, DRC
pub const BOOK_AUDIO: u8 = 0x8C;

/// Book 0xAA — EQ biquad coefficient RAM
pub const BOOK_EQ: u8 = 0xAA;

// ── Input Mixer (Book 0x8C, Pages 0x09–0x0A) — TAS5830 SLUUDB4 ─────────────
//
// R2R crosses a page boundary: L2L/R2L/L2R are on page 0x09,
// R2R is on page 0x0A.

/// Page containing L2L, R2L, and L2R mixer gains
pub const MIXER_PAGE: u8 = 0x09;

/// Page containing R2R mixer gain (crosses page boundary from 0x09)
pub const MIXER_PAGE_R2R: u8 = 0x0A;

/// Left-to-Left gain (4 bytes, 9.23 fixed-point)
pub const MIXER_L2L: u8 = 0x74;
/// Right-to-Left gain
pub const MIXER_R2L: u8 = 0x78;
/// Left-to-Right gain
pub const MIXER_L2R: u8 = 0x7C;
/// Right-to-Right gain (on MIXER_PAGE_R2R, not MIXER_PAGE!)
pub const MIXER_R2R: u8 = 0x08;

// ── Channel volume (Book 0x8C, Page 0x06) — TAS5830 SLUUDB4 ────────────────

/// Page containing per-channel volume
pub const CHAN_VOL_PAGE: u8 = 0x06;

/// Left channel volume (4 bytes, 9.23 fixed-point)
pub const CHAN_VOL_LEFT: u8 = 0x64;
/// Right channel volume
pub const CHAN_VOL_RIGHT: u8 = 0x68;

// ── EQ biquad RAM (Book 0xAA) — TAS5830 SLUUDB4 ────────────────────────────
//
// Two independent EQ banks of 12 biquads each:
//   - Tweeter / Left:  Pages 0x01–0x03 (BQ1 @ Page 0x01 reg 0x30)
//   - Woofer  / Right: Pages 0x03–0x05 (BQ1 @ Page 0x03 reg 0x30)
//
// Layout is identical for PF2/3/6/7 (stereo/mono 96kHz) and PF9 (1.1 96kHz).

/// Number of EQ biquads per channel (tweeter/left or woofer/right)
pub const EQ_BANDS_PER_CHANNEL: u8 = 12;

/// Total EQ biquad bands (12 tweeter/left + 12 woofer/right)
pub const EQ_BAND_COUNT: u8 = 24;

/// Bytes per biquad (5 coefficients × 4 bytes)
pub const BIQUAD_SIZE: u16 = 20;

/// First EQ page (tweeter/left channel)
pub const EQ_PAGE_FIRST: u8 = 0x01;

/// First register on the first EQ page of each channel
pub const EQ_REG_FIRST: u8 = 0x30;

/// First register on continuation pages
pub const EQ_REG_CONT: u8 = 0x08;

/// Usable bytes on the first EQ page (0x30..=0x7F)
pub const EQ_FIRST_PAGE_BYTES: u16 = 0x80 - EQ_REG_FIRST as u16; // 80

/// Usable bytes on continuation EQ pages (0x08..=0x7F)
pub const EQ_CONT_PAGE_BYTES: u16 = 0x80 - EQ_REG_CONT as u16; // 120

/// Page offset between left/tweeter and right/woofer EQ banks.
/// Woofer/right EQ starts at EQ_PAGE_FIRST + EQ_CHANNEL_PAGE_OFFSET.
pub const EQ_CHANNEL_PAGE_OFFSET: u8 = 2;

/// Compute the (page, register) for a given biquad coefficient.
///
/// `band` is 0..23: bands 0–11 = tweeter/left, bands 12–23 = woofer/right.
/// `coeff` is 0..4 (b0, b1, b2, a1, a2).
/// Returns `(page, start_register)` — each coefficient is 4 consecutive bytes.
pub const fn eq_coeff_location(band: u8, coeff: u8) -> (u8, u8) {
    // Determine which channel and the local band index within that channel
    let channel_page_offset = if band < EQ_BANDS_PER_CHANNEL {
        0
    } else {
        EQ_CHANNEL_PAGE_OFFSET
    };
    let local_band = if band < EQ_BANDS_PER_CHANNEL {
        band
    } else {
        band - EQ_BANDS_PER_CHANNEL
    };

    let byte_offset = local_band as u16 * BIQUAD_SIZE + coeff as u16 * 4;

    if byte_offset < EQ_FIRST_PAGE_BYTES {
        // Still on the first page of this channel's EQ bank
        (
            EQ_PAGE_FIRST + channel_page_offset,
            EQ_REG_FIRST + byte_offset as u8,
        )
    } else {
        // Continuation pages
        let remaining = byte_offset - EQ_FIRST_PAGE_BYTES;
        let page = EQ_PAGE_FIRST
            + channel_page_offset
            + 1
            + (remaining / EQ_CONT_PAGE_BYTES) as u8;
        let reg = EQ_REG_CONT + (remaining % EQ_CONT_PAGE_BYTES) as u8;
        (page, reg)
    }
}

// ── Fault register bit masks ────────────────────────────────────────────────

pub mod fault {
    // CHAN_FAULT (0x70)
    pub const RIGHT_OC: u8 = 1 << 0;
    pub const RIGHT_DC: u8 = 1 << 1;
    pub const LEFT_OC: u8 = 1 << 2;
    pub const LEFT_DC: u8 = 1 << 3;

    // GLOBAL_FAULT1 (0x71)
    pub const PVDD_UV: u8 = 1 << 0;
    pub const PVDD_OV: u8 = 1 << 1;
    pub const CLOCK: u8 = 1 << 2;
    pub const EEPROM_CRC: u8 = 1 << 3;
    pub const BQ_WRITE: u8 = 1 << 4;
    pub const OTP_CRC: u8 = 1 << 5;

    // GLOBAL_FAULT2 (0x72)
    pub const THERMAL_SD: u8 = 1 << 0;
    pub const THERMAL_CBC: u8 = 1 << 1;

    // OT_WARNING (0x73)
    pub const OT_112C: u8 = 1 << 0;
    pub const OT_122C: u8 = 1 << 1;
    pub const OT_134C: u8 = 1 << 2;
    pub const OT_146C: u8 = 1 << 3;
}
