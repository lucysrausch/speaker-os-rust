//! TAS5830 Class-D amplifier driver with full DSP control.
//!
//! Texas Instruments TAS5830 digital-input Class-D amplifier with
//! integrated DSP, controlled via I2C.
//!
//! # Initialisation
//!
//! The boot sequence has two phases matching the hardware requirements:
//!
//! 1. [`Tas5830::init()`] — PDN high, reset, Hi-Z mode (**before** I2S clocks)
//! 2. [`Tas5830::play()`] — DSP programming + enter Play (**after** I2S clocks)
//!
//! # DSP features
//!
//! All DSP parameters can be changed at runtime:
//!
//! - **24-band parametric EQ** — 12 biquads per channel in 5.27 fixed-point
//! - **4×4 mixer matrix** — L/R input → L/R output routing
//! - **Per-channel volume** — independent L/R gain
//! - **Master digital volume** — 0.5 dB steps
//! - **Analog gain** — output-stage gain control
//! - **EQ bypass** — disable all EQ processing
//! - **Fault monitoring** — over-current, over-voltage, thermal
//! - **Processing coefficients** — raw crossover / DRC / routing blocks

pub mod dsp;
pub mod init;
pub mod regs;

pub use dsp::*;

use embassy_rp::gpio::Output;
use embedded_hal_async::i2c::I2c;
use micromath::F32Ext;

/// TAS5830 I2C address (configured by ADDR pin)
pub const DEFAULT_ADDRESS: u8 = crate::hw::pins::i2c_addr::TAS5830;

// ── Power state ─────────────────────────────────────────────────────────────

/// TAS5830 power state (read from register 0x68)
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PowerState {
    DeepSleep,
    Sleep,
    HiZ,
    Play,
    Unknown(u8),
}

impl From<u8> for PowerState {
    fn from(val: u8) -> Self {
        match val & 0x03 {
            0 => PowerState::DeepSleep,
            1 => PowerState::Sleep,
            2 => PowerState::HiZ,
            3 => PowerState::Play,
            _ => PowerState::Unknown(val),
        }
    }
}

// ── Error ───────────────────────────────────────────────────────────────────

/// TAS5830 driver error
#[derive(Debug, defmt::Format)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// EQ band index out of range (must be 0..29)
    InvalidBand,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

// ── Volume helpers ────────────────────────────────────────────────────────

/// Convert a volume percentage (0–100) to a DIG_VOL register value.
///
/// Uses log2 scaling for a perceptually linear volume curve.
/// Returns a value suitable for [`DspConfig::digital_volume`] or
/// [`Tas5830::set_volume()`].
pub fn volume_percent_to_reg(percent: u8) -> u8 {
    use micromath::F32Ext;
    const VOL_START: f32 = 0.5;
    const VOL_SCALE: f32 = 30.0;
    const VOL_RANGE: f32 = VOL_START + VOL_SCALE;
    let vol_scaled =
        ((percent as f32) / 100.0 * VOL_SCALE).clamp(0.001, VOL_SCALE) + VOL_START;
    let percent_scaled =
        (vol_scaled.log2() / VOL_RANGE.log2() * 100.0).clamp(0.0, 100.0) as u8;
    if percent_scaled >= 100 {
        0x00
    } else {
        0xCF - ((percent_scaled as u16 * 0xCF) / 100) as u8
    }
}

// ── DSP configuration ──────────────────────────────────────────────────────

/// Complete DSP configuration for boot or full reconfiguration.
///
/// Holds every parameter the driver programs into the TAS5830's DSP.
/// Pass to [`Tas5830::play()`] to apply at boot, or call individual
/// setters to change parameters at runtime.
pub struct DspConfig<'a> {
    /// 24-band parametric EQ: 12 tweeter/left + 12 woofer/right (5.27 format)
    pub eq_bands: [BiquadCoeffs; 24],
    /// Whether EQ processing is bypassed
    pub eq_bypass: bool,
    /// 4×4 mixer matrix gains (9.23 format)
    pub mixer: MixerGains,
    /// Per-channel volume (9.23 format)
    pub channel_volume: ChannelVolume,
    /// Master digital volume (0x00 = 0 dB … 0xFF = −127.5 dB)
    pub digital_volume: u8,
    /// Analog gain (0–31, 0.5 dB/step, 0 = 0 dB)
    pub analog_gain: u8,
    /// Raw processing coefficients (crossover, DRC, routing).
    ///
    /// Flat `(register, value)` pairs including book/page switches.
    /// Use [`init::DEFAULT_PROCESSING`] for the PPC3 defaults.
    pub processing: &'a [(u8, u8)],
}

impl<'a> DspConfig<'a> {
    /// Flat passthrough defaults: no EQ, stereo mixer, −24 dB volume,
    /// no crossover/DRC processing.
    ///
    /// For PPC3 defaults, use `init::DEFAULT_PROCESSING` in the `processing` field.
    pub const fn default() -> Self {
        Self {
            eq_bands: [BiquadCoeffs::PASSTHROUGH; 24],
            eq_bypass: false,
            mixer: MixerGains::STEREO,
            channel_volume: ChannelVolume::UNITY,
            digital_volume: 0x30, // −24 dB
            analog_gain: 0,       // 0 dB
            processing: &[],
        }
    }
}

// ── Driver ──────────────────────────────────────────────────────────────────

/// TAS5830 amplifier driver
pub struct Tas5830<'a, I> {
    i2c: I,
    address: u8,
    volume: u8,
    muted: bool,
    pdn: Output<'a>,
    mute_pin: Output<'a>,
}

impl<'a, I, E> Tas5830<'a, I>
where
    I: I2c<Error = E>,
{
    /// Create a new TAS5830 driver.
    ///
    /// Both GPIO pins start low (power-down asserted, hardware-muted).
    pub fn new(i2c: I, address: u8, pdn: Output<'a>, mute_pin: Output<'a>) -> Self {
        Self {
            i2c,
            address,
            volume: 0x30, // −24 dB default
            muted: false,
            pdn,
            mute_pin,
        }
    }

    /// Create with the default I2C address (0x60).
    pub fn new_default(i2c: I, pdn: Output<'a>, mute_pin: Output<'a>) -> Self {
        Self::new(i2c, DEFAULT_ADDRESS, pdn, mute_pin)
    }

    // ════════════════════════════════════════════════════════════════════
    // Boot sequence
    // ════════════════════════════════════════════════════════════════════

    /// Phase 1: PDN high, reset, Hi-Z mode.
    ///
    /// Call **before** I2S clocks are started.
    pub async fn init(&mut self) -> Result<(), Error<E>> {
        self.pdn.set_high();
        self.mute_pin.set_low();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(5)).await;

        let die_id = self.read_reg(regs::DIE_ID).await?;
        defmt::info!("TAS5830 DIE_ID: {:#04x}", die_id);

        self.transmit_registers(init::HARDWARE_INIT).await?;

        defmt::info!("TAS5830 init phase 1 complete (Hi-Z, waiting for I2S clocks)");
        Ok(())
    }

    /// Phase 2: Program DSP + enter Play mode + unmute.
    ///
    /// Call **after** I2S clocks (BCLK/WCLK) are running.
    pub async fn play(&mut self, config: &DspConfig<'_>) -> Result<(), Error<E>> {
        // Wait for I2S clock to stabilise
        embassy_time::Timer::after(embassy_time::Duration::from_millis(5)).await;

        // ── Chip-level configuration ────────────────────────────────────
        self.transmit_registers(init::CHIP_CONFIG).await?;

        // ── DSP processing chain init ─────────────────────────────────
        // Write passthrough biquads to internal DSP EQ pages to ensure
        // the processing blocks (mixer, volume, EQ) are active.
        self.init_dsp_processing().await?;

        // ── Processing coefficients (crossover, DRC, routing) ───────────
        self.transmit_registers(config.processing).await?;

        // ── EQ biquads ──────────────────────────────────────────────────
        self.write_all_eq(&config.eq_bands).await?;

        // ── Mixer ───────────────────────────────────────────────────────
        self.set_mixer(&config.mixer).await?;

        // ── Channel volume ──────────────────────────────────────────────
        self.set_channel_volume(&config.channel_volume).await?;

        // ── EQ bypass ───────────────────────────────────────────────────
        // CHIP_CONFIG sets DSP_MISC=0x87 to disable processing while
        // writing coefficients. Now re-enable DSP processing, respecting
        // the EQ bypass setting.
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        let dsp_misc = if config.eq_bypass { 0x01 } else { 0x00 };
        self.write_reg(regs::DSP_MISC, dsp_misc).await?;
        defmt::info!("TAS5830 DSP_MISC: {:#04x} (EQ bypass={})", dsp_misc, config.eq_bypass);

        // ── Analog gain ─────────────────────────────────────────────────
        self.set_analog_gain(config.analog_gain).await?;

        // ── Enter Play mode ─────────────────────────────────────────────
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        self.write_reg(regs::SDOUT_SEL, 0x00).await?;
        self.volume = config.digital_volume;
        self.write_reg(regs::DIG_VOL, config.digital_volume).await?;

        // Clear faults before Play transition
        self.clear_faults().await?;

        // Check clock detection before entering Play
        let clkdet = self.read_reg(regs::CLKDET_STATUS).await?;
        defmt::info!("TAS5830 CLKDET_STATUS: {:#04x}", clkdet);

        self.write_reg(regs::DEVICE_CTRL_2, 0x03).await?; // Play

        // Wait for Play mode with retry — the transition takes time
        let mut state = PowerState::HiZ;
        for attempt in 0..10u8 {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
            state = self.get_power_state().await?;
            if state == PowerState::Play {
                defmt::info!("TAS5830 entered Play mode (attempt {})", attempt);
                break;
            }
        }

        if state != PowerState::Play {
            // Read faults for diagnosis
            let faults = self.read_faults().await?;
            defmt::warn!(
                "TAS5830 stuck in {:?} — CHAN={:#04x} GF1={:#04x} GF2={:#04x} OT={:#04x}",
                state,
                faults.chan_fault,
                faults.global_fault1,
                faults.global_fault2,
                faults.ot_warning,
            );
            // Try: clear faults and retry Play
            self.clear_faults().await?;
            self.write_reg(regs::DEVICE_CTRL_2, 0x03).await?;
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
            state = self.get_power_state().await?;
            defmt::info!("TAS5830 power state after retry: {:?}", state);
        }

        // Unmute hardware pin
        self.mute_pin.set_high();
        defmt::info!("TAS5830 playing, unmuted");
        Ok(())
    }

    // ════════════════════════════════════════════════════════════════════
    // Volume & mute (unchanged public API)
    // ════════════════════════════════════════════════════════════════════

    /// Set master digital volume (0x00 = 0 dB, 0xFF = −127.5 dB).
    pub async fn set_volume(&mut self, volume: u8) -> Result<(), Error<E>> {
        self.volume = volume;
        if !self.muted {
            self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
            self.write_reg(regs::DIG_VOL, volume).await?;
        }
        defmt::debug!(
            "TAS5830 volume: {:#04x} ({} dB)",
            volume,
            -(volume as i16) / 2
        );
        Ok(())
    }

    /// Set volume as a percentage (0–100) with log2 scaling.
    pub async fn set_volume_percent(&mut self, percent: u8) -> Result<(), Error<E>> {
        let volume = volume_percent_to_reg(percent);
        defmt::info!(
            "TAS5830 set volume percent: {} -> reg {:#04x}",
            percent,
            volume
        );
        self.set_volume(volume).await
    }

    /// Get volume as a percentage (0–100).
    pub fn get_volume_percent(&self) -> u8 {
        if self.volume >= 0xCF {
            0
        } else {
            (((0xCF - self.volume) as u16 * 100) / 0xCF) as u8
        }
    }

    /// Mute (digital, via volume register).
    pub async fn mute(&mut self) -> Result<(), Error<E>> {
        self.muted = true;
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        self.write_reg(regs::DIG_VOL, 0xFF).await?;
        defmt::debug!("TAS5830 muted");
        Ok(())
    }

    /// Unmute (restore previous volume).
    pub async fn unmute(&mut self) -> Result<(), Error<E>> {
        self.muted = false;
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        self.write_reg(regs::DIG_VOL, self.volume).await?;
        defmt::debug!("TAS5830 unmuted");
        Ok(())
    }

    /// Check if currently muted.
    pub fn is_muted(&self) -> bool {
        self.muted
    }

    /// Toggle mute state; returns new mute state.
    pub async fn toggle_mute(&mut self) -> Result<bool, Error<E>> {
        if self.muted {
            self.unmute().await?;
        } else {
            self.mute().await?;
        }
        Ok(self.muted)
    }

    // ════════════════════════════════════════════════════════════════════
    // EQ — 24-band parametric equaliser (12 per channel)
    // ════════════════════════════════════════════════════════════════════

    /// Write biquad coefficients to a single EQ band (0–29).
    ///
    /// Each band is a second-order IIR filter with 5 coefficients in
    /// 5.27 fixed-point format. Use [`BiquadCoeffs::peaking()`] and
    /// friends to compute them from frequency / gain / Q.
    pub async fn set_eq_band(&mut self, band: u8, coeffs: &BiquadCoeffs) -> Result<(), Error<E>> {
        if band >= regs::EQ_BAND_COUNT {
            return Err(Error::InvalidBand);
        }

        let bytes = coeffs.to_bytes();

        // Write all 5 coefficients (20 bytes). They may span a page boundary,
        // so we write each 4-byte coefficient individually after selecting the
        // correct page.
        let mut last_page: u8 = 0xFF; // sentinel
        for coeff_idx in 0u8..5 {
            let (page, reg) = regs::eq_coeff_location(band, coeff_idx);

            if page != last_page {
                self.set_book_page(regs::BOOK_EQ, page).await?;
                last_page = page;
            }

            let offset = coeff_idx as usize * 4;
            self.write_bulk(reg, &bytes[offset..offset + 4]).await?;
        }

        Ok(())
    }

    /// Clear all 24 EQ bands to passthrough (unity gain, no filtering).
    pub async fn clear_eq(&mut self) -> Result<(), Error<E>> {
        for band in 0..regs::EQ_BAND_COUNT {
            self.set_eq_band(band, &BiquadCoeffs::PASSTHROUGH).await?;
        }
        Ok(())
    }

    /// Enable or disable EQ bypass.
    ///
    /// When bypassed, the biquad coefficients are ignored and audio
    /// passes through unprocessed.
    pub async fn set_eq_bypass(&mut self, bypass: bool) -> Result<(), Error<E>> {
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        let val = self.read_reg(regs::DSP_MISC).await?;
        let new_val = if bypass { val | 0x01 } else { val & !0x01 };
        self.write_reg(regs::DSP_MISC, new_val).await?;
        defmt::debug!("TAS5830 EQ bypass: {}", bypass);
        Ok(())
    }

    // ════════════════════════════════════════════════════════════════════
    // Mixer — 4×4 matrix
    // ════════════════════════════════════════════════════════════════════

    /// Set the 4×4 mixer gains (9.23 fixed-point).
    ///
    /// Note: R2R is on a different page (0x0A) than L2L/R2L/L2R (0x09).
    pub async fn set_mixer(&mut self, gains: &MixerGains) -> Result<(), Error<E>> {
        defmt::info!(
            "TAS5830 mixer: L2L={:#010x} R2L={:#010x} L2R={:#010x} R2R={:#010x}",
            gains.l2l.0, gains.r2l.0, gains.l2r.0, gains.r2r.0,
        );
        // L2L, R2L, L2R on page 0x09
        self.set_book_page(regs::BOOK_AUDIO, regs::MIXER_PAGE)
            .await?;
        self.write_bulk(regs::MIXER_L2L, &gains.l2l.to_bytes())
            .await?;
        self.write_bulk(regs::MIXER_R2L, &gains.r2l.to_bytes())
            .await?;
        self.write_bulk(regs::MIXER_L2R, &gains.l2r.to_bytes())
            .await?;
        // R2R crosses to page 0x0A
        self.set_book_page(regs::BOOK_AUDIO, regs::MIXER_PAGE_R2R)
            .await?;
        self.write_bulk(regs::MIXER_R2R, &gains.r2r.to_bytes())
            .await?;

        // Read back mixer registers to verify writes landed
        self.set_book_page(regs::BOOK_AUDIO, regs::MIXER_PAGE)
            .await?;
        let l2l = self.read_bulk4(regs::MIXER_L2L).await?;
        let r2l = self.read_bulk4(regs::MIXER_R2L).await?;
        let l2r = self.read_bulk4(regs::MIXER_L2R).await?;
        self.set_book_page(regs::BOOK_AUDIO, regs::MIXER_PAGE_R2R)
            .await?;
        let r2r = self.read_bulk4(regs::MIXER_R2R).await?;
        defmt::info!(
            "TAS5830 mixer readback: L2L={:#010x} R2L={:#010x} L2R={:#010x} R2R={:#010x}",
            u32::from_be_bytes(l2l),
            u32::from_be_bytes(r2l),
            u32::from_be_bytes(l2r),
            u32::from_be_bytes(r2r),
        );

        Ok(())
    }

    /// Set the mixer to a named preset.
    pub async fn set_mixer_mode(&mut self, mode: MixerMode) -> Result<(), Error<E>> {
        self.set_mixer(&mode.to_gains()).await
    }

    // ════════════════════════════════════════════════════════════════════
    // Per-channel volume
    // ════════════════════════════════════════════════════════════════════

    /// Set per-channel volume (post-mixer, 9.23 fixed-point).
    pub async fn set_channel_volume(&mut self, vol: &ChannelVolume) -> Result<(), Error<E>> {
        self.set_book_page(regs::BOOK_AUDIO, regs::CHAN_VOL_PAGE)
            .await?;
        self.write_bulk(regs::CHAN_VOL_LEFT, &vol.left.to_bytes())
            .await?;
        self.write_bulk(regs::CHAN_VOL_RIGHT, &vol.right.to_bytes())
            .await?;
        defmt::debug!("TAS5830 channel volume updated");
        Ok(())
    }

    // ════════════════════════════════════════════════════════════════════
    // Analog gain
    // ════════════════════════════════════════════════════════════════════

    /// Set the analog output-stage gain.
    ///
    /// `gain`: 0–31 (0.5 dB/step). 0 = 0 dB, 31 = −15.5 dB.
    pub async fn set_analog_gain(&mut self, gain: u8) -> Result<(), Error<E>> {
        let gain = gain.min(31);
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        self.write_reg(regs::AGAIN, gain).await?;
        defmt::debug!("TAS5830 analog gain: {}", gain);
        Ok(())
    }

    // ════════════════════════════════════════════════════════════════════
    // Fault monitoring
    // ════════════════════════════════════════════════════════════════════

    /// Read all fault and warning registers.
    pub async fn read_faults(&mut self) -> Result<FaultStatus, Error<E>> {
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        Ok(FaultStatus {
            chan_fault: self.read_reg(regs::CHAN_FAULT).await?,
            global_fault1: self.read_reg(regs::GLOBAL_FAULT1).await?,
            global_fault2: self.read_reg(regs::GLOBAL_FAULT2).await?,
            ot_warning: self.read_reg(regs::OT_WARNING).await?,
        })
    }

    /// Clear all latched faults.
    pub async fn clear_faults(&mut self) -> Result<(), Error<E>> {
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        self.write_reg(regs::CLEAR_FAULT, 0x80).await?;
        Ok(())
    }

    // ════════════════════════════════════════════════════════════════════
    // Power state
    // ════════════════════════════════════════════════════════════════════

    /// Read the current power state.
    pub async fn get_power_state(&mut self) -> Result<PowerState, Error<E>> {
        self.set_book_page(regs::BOOK_CTRL, 0x00).await?;
        let state = self.read_reg(regs::POWER_STATE).await?;
        Ok(PowerState::from(state))
    }

    // ════════════════════════════════════════════════════════════════════
    // Raw DSP access
    // ════════════════════════════════════════════════════════════════════

    /// Write a raw `(register, value)` sequence.
    ///
    /// The sequence may contain book/page switches (writes to registers
    /// 0x00 and 0x7F on page 0). This is the same format as TI PPC3
    /// init sequences.
    pub async fn write_raw_sequence(&mut self, regs: &[(u8, u8)]) -> Result<(), Error<E>> {
        self.transmit_registers(regs).await
    }

    // ════════════════════════════════════════════════════════════════════
    // I2C helpers (private)
    // ════════════════════════════════════════════════════════════════════

    /// Select a DSP book and page.
    async fn set_book_page(&mut self, book: u8, page: u8) -> Result<(), Error<E>> {
        // Must go through page 0 to write the book register
        self.write_reg(regs::PAGE_SELECT, 0x00).await?;
        self.write_reg(regs::BOOK_SELECT, book).await?;
        self.write_reg(regs::PAGE_SELECT, page).await?;
        Ok(())
    }

    /// Write a flat sequence of (register, value) pairs.
    async fn transmit_registers(&mut self, regs: &[(u8, u8)]) -> Result<(), Error<E>> {
        for &(reg, val) in regs {
            self.write_reg(reg, val).await?;
        }
        Ok(())
    }

    /// Write all 24 EQ bands from a slice.
    async fn write_all_eq(&mut self, bands: &[BiquadCoeffs; 24]) -> Result<(), Error<E>> {
        for (i, band) in bands.iter().enumerate() {
            self.set_eq_band(i as u8, band).await?;
        }
        Ok(())
    }

    /// Write a single register (1 byte).
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c.write(self.address, &[reg, value]).await?;
        Ok(())
    }

    /// Write multiple bytes starting at `reg` (auto-incrementing).
    async fn write_bulk(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<E>> {
        // Build a buffer: [register_address, data...]
        // Max 5 bytes (reg + 4 data bytes for a coefficient)
        let mut buf = [0u8; 5];
        buf[0] = reg;
        let len = data.len().min(4);
        buf[1..1 + len].copy_from_slice(&data[..len]);
        self.i2c.write(self.address, &buf[..1 + len]).await?;
        Ok(())
    }

    /// Read a single register (1 byte).
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    /// Read 4 consecutive bytes starting at `reg` (auto-incrementing).
    async fn read_bulk4(&mut self, reg: u8) -> Result<[u8; 4], Error<E>> {
        let mut buf = [0u8; 4];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .await?;
        Ok(buf)
    }

    /// Write passthrough biquads to DSP EQ pages to initialize the processing chain.
    ///
    /// The TAS5830 DSP needs its EQ pages initialized before the processing
    /// blocks (mixer, volume, etc.) become active. This writes 12 passthrough
    /// biquads to the tweeter EQ bank (Book 0xAA Page 0x01) and 12 to the
    /// woofer EQ bank (Page 0x03), matching the PF9 memory map.
    async fn init_dsp_processing(&mut self) -> Result<(), Error<E>> {
        // Passthrough biquad: b0=1.0 (0x08000000 in 5.27), rest=0
        let passthrough: [u8; 20] = [
            0x08, 0x00, 0x00, 0x00, // b0
            0x00, 0x00, 0x00, 0x00, // b1
            0x00, 0x00, 0x00, 0x00, // b2
            0x00, 0x00, 0x00, 0x00, // a1
            0x00, 0x00, 0x00, 0x00, // a2
        ];

        // Write 12 passthrough biquads to each page (matching jpvc36 firmware)
        for &eq_page in &[0x01u8, 0x03] {
            self.set_book_page(regs::BOOK_EQ, eq_page).await?;
            let mut reg = 0x30u8;
            for _ in 0..12 {
                for chunk in passthrough.chunks(4) {
                    self.write_bulk(reg, chunk).await?;
                    reg = reg.wrapping_add(4);
                }
            }
        }

        defmt::info!("TAS5830 DSP processing chain initialized (Book 0xAA Pages 0x01/0x03)");
        Ok(())
    }
}
