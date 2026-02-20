//! DSP configuration file system.
//!
//! Parses a human-readable INI-like config file from SPI flash into
//! [`DspConfigFile`], which is then converted to a [`DspConfig`] for
//! programming the TAS5830 DSP.
//!
//! Supports the TAS5830 PF9 (1.1 bi-amped 96kHz) feature set:
//! - Input Mixer (4-coefficient matrix)
//! - 24-band parametric EQ (12 tweeter/left + 12 woofer/right)
//! - Per-channel volume (tweeter + woofer)
//! - Master digital volume + analog gain
//! - Raw processing register pairs for DRC / output crossbar

pub mod flash;
pub mod parser;

use crate::drivers::tas5830::{
    BiquadCoeffs, ChannelVolume, DspConfig, FilterType, Gain923, MixerGains, MixerMode,
};
use heapless::Vec;

/// Built-in default config (flat passthrough).
pub const DEFAULT_CONFIG: &[u8] = include_bytes!("default.cfg");

/// Maximum processing register pairs in the static buffer.
const MAX_PROCESSING: usize = 512;

/// Static buffer for processing register pairs (lives for the program's lifetime).
///
/// Since `DspConfig` borrows `&[(u8, u8)]`, we need a buffer that outlives
/// the config. This static is written once at boot and never modified.
static mut PROCESSING_BUF: [(u8, u8); MAX_PROCESSING] = [(0, 0); MAX_PROCESSING];

/// Length of valid data in PROCESSING_BUF.
static mut PROCESSING_LEN: usize = 0;

/// Fully parsed DSP configuration file.
///
/// This is the intermediate representation between the text config file
/// and the hardware-level [`DspConfig`].
pub struct DspConfigFile {
    /// 24-band EQ: bands 0–11 = tweeter/left, 12–23 = woofer/right
    pub eq_bands: [FilterType; 24],
    /// Whether EQ processing is bypassed
    pub eq_bypass: bool,
    /// Mixer mode preset
    pub mixer: MixerMode,
    /// Custom mixer gains (overrides mode if Some)
    pub mixer_gains: Option<MixerGains>,
    /// Per-channel volume: tweeter/left dB
    pub channel_volume_left_db: i16,
    /// Per-channel volume: woofer/right dB
    pub channel_volume_right_db: i16,
    /// Master digital volume register value
    pub digital_volume: u8,
    /// Analog gain (0-31)
    pub analog_gain: u8,
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Raw processing register pairs for DRC / output crossbar (advanced)
    pub processing: Vec<(u8, u8), 512>,
}

impl Default for DspConfigFile {
    fn default() -> Self {
        Self {
            eq_bands: [FilterType::Bypass; 24],
            eq_bypass: false,
            mixer: MixerMode::Stereo,
            mixer_gains: None,
            channel_volume_left_db: 0,
            channel_volume_right_db: 0,
            digital_volume: 0x30, // -24 dB
            analog_gain: 0,
            sample_rate: 96_000,
            processing: Vec::new(),
        }
    }
}

impl DspConfigFile {
    /// Convert this config file into a [`DspConfig`] for the TAS5830 driver.
    ///
    /// # Safety
    /// This writes to static buffers and must only be called once (at boot).
    pub fn to_dsp_config(&self) -> DspConfig<'static> {
        let sample_rate = self.sample_rate as f32;

        // Convert EQ bands to biquad coefficients
        let mut eq_bands = [BiquadCoeffs::PASSTHROUGH; 24];
        for (i, filter) in self.eq_bands.iter().enumerate() {
            eq_bands[i] = filter.to_coeffs(sample_rate);
        }

        // Determine mixer gains
        let mixer = match self.mixer_gains {
            Some(gains) => gains,
            None => self.mixer.to_gains(),
        };

        // Channel volume
        let channel_volume = ChannelVolume {
            left: Gain923::from_db(self.channel_volume_left_db),
            right: Gain923::from_db(self.channel_volume_right_db),
        };

        // Raw processing register pairs (DRC, output crossbar, etc.)
        // SAFETY: Called once at boot, no concurrent access
        let processing_slice: &'static [(u8, u8)] = if !self.processing.is_empty() {
            unsafe {
                let len = self.processing.len().min(MAX_PROCESSING);
                PROCESSING_BUF[..len].copy_from_slice(&self.processing[..len]);
                PROCESSING_LEN = len;
                &PROCESSING_BUF[..PROCESSING_LEN]
            }
        } else {
            &[]
        };

        DspConfig {
            eq_bands,
            eq_bypass: self.eq_bypass,
            mixer,
            channel_volume,
            digital_volume: self.digital_volume,
            analog_gain: self.analog_gain,
            processing: processing_slice,
        }
    }
}

/// Load config from flash, parse it, and return a `DspConfig`.
///
/// Falls back to the built-in default config if flash is empty or invalid.
///
/// # Safety
/// Must only be called once at boot (writes to static processing buffer).
pub fn load_and_build_config(
    flash: &mut embassy_rp::flash::Flash<
        '_,
        embassy_rp::peripherals::FLASH,
        embassy_rp::flash::Blocking,
        { crate::drivers::settings::FLASH_SIZE },
    >,
) -> DspConfig<'static> {
    // Try to read config from flash
    let config_data = flash::read_config(flash);

    let data = match config_data {
        Some(data) => {
            defmt::info!("Loaded DSP config from flash ({} bytes)", data.len());
            data
        }
        None => {
            defmt::info!("No config in flash, using built-in default");
            DEFAULT_CONFIG
        }
    };

    match parser::parse(data) {
        Ok(config_file) => {
            defmt::info!(
                "Config parsed: sample_rate={}, eq_bypass={}, digital_vol={:#04x}, mixer={:?}",
                config_file.sample_rate,
                config_file.eq_bypass,
                config_file.digital_volume,
                config_file.mixer,
            );
            config_file.to_dsp_config()
        }
        Err(e) => {
            defmt::warn!("Config parse error at line {}: {:?}, using defaults", e.line, e.kind);
            DspConfigFile::default().to_dsp_config()
        }
    }
}
