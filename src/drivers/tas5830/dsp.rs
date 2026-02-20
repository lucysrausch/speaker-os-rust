//! DSP types, fixed-point helpers, and biquad coefficient computation.
//!
//! Two fixed-point formats are used by the TAS5830 DSP:
//!
//! - **5.27** for biquad filter coefficients (range ±16, unity = 0x0800_0000)
//! - **9.23** for mixer gains and channel volumes (range ±256, unity = 0x0080_0000)
//!
//! Biquad formulas follow Robert Bristow-Johnson's Audio EQ Cookbook.

use core::f32::consts::PI;
use micromath::F32Ext;

// ── 5.27 fixed-point (biquad coefficients) ──────────────────────────────────

/// Fractional bits for biquad coefficients
const BQ_FRAC: i64 = 27;

/// Unity in 5.27: 1.0 = 2^27 = 0x0800_0000
const BQ_ONE: i32 = 1 << BQ_FRAC;

/// Convert an f32 to 5.27 fixed-point (saturating)
fn f32_to_5_27(v: f32) -> i32 {
    let scaled = v * (BQ_ONE as f32);
    // Clamp to i32 range (the 5.27 format uses all 32 bits as signed)
    if scaled >= i32::MAX as f32 {
        i32::MAX
    } else if scaled <= i32::MIN as f32 {
        i32::MIN
    } else {
        scaled as i32
    }
}

// ── 9.23 fixed-point (mixer / volume) ───────────────────────────────────────

/// Fractional bits for mixer/volume gains
const GAIN_FRAC: i64 = 23;

/// Unity in 9.23: 1.0 = 2^23 = 0x0080_0000
const GAIN_ONE: i32 = 1 << GAIN_FRAC;

// ── Biquad filter coefficients ──────────────────────────────────────────────

/// Five biquad coefficients in the TAS5830's 5.27 fixed-point format.
///
/// Transfer function:
/// ```text
/// H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)
/// ```
///
/// The hardware expects **pre-normalised** coefficients (divided by a0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct BiquadCoeffs {
    pub b0: i32,
    pub b1: i32,
    pub b2: i32,
    /// Negated: the hardware expects −a1 (i.e. `a1_neg = −a1/a0`)
    pub a1: i32,
    /// Negated: the hardware expects −a2 (i.e. `a2_neg = −a2/a0`)
    pub a2: i32,
}

impl BiquadCoeffs {
    /// Passthrough (unity gain, no filtering).
    pub const PASSTHROUGH: Self = Self {
        b0: BQ_ONE,
        b1: 0,
        b2: 0,
        a1: 0,
        a2: 0,
    };

    /// Silence — zeroes the signal through this biquad.
    pub const MUTE: Self = Self {
        b0: 0,
        b1: 0,
        b2: 0,
        a1: 0,
        a2: 0,
    };

    /// Serialise to 20 bytes (big-endian, 4 bytes per coefficient).
    ///
    /// Order: b0, b1, b2, a1, a2
    pub fn to_bytes(&self) -> [u8; 20] {
        let mut buf = [0u8; 20];
        let coeffs = [self.b0, self.b1, self.b2, self.a1, self.a2];
        for (i, &c) in coeffs.iter().enumerate() {
            let bytes = (c as u32).to_be_bytes();
            buf[i * 4..i * 4 + 4].copy_from_slice(&bytes);
        }
        buf
    }

    /// Build from raw 5.27 values (e.g. loaded from a config file).
    pub const fn from_raw(b0: i32, b1: i32, b2: i32, a1: i32, a2: i32) -> Self {
        Self { b0, b1, b2, a1, a2 }
    }

    // ── Standard filter constructors (Audio EQ Cookbook) ─────────────────

    /// Peaking (parametric) EQ.
    ///
    /// - `freq_hz`: centre frequency
    /// - `gain_db`: boost/cut in dB (positive = boost)
    /// - `q`: quality factor (bandwidth)
    /// - `sample_rate`: system sample rate in Hz
    pub fn peaking(freq_hz: f32, gain_db: f32, q: f32, sample_rate: f32) -> Self {
        let a = (10.0f32).powf(gain_db / 40.0); // sqrt(10^(dB/20))
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);

        let b0 = 1.0 + alpha * a;
        let b1 = -2.0 * cos_w0;
        let b2 = 1.0 - alpha * a;
        let a0 = 1.0 + alpha / a;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha / a;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// Second-order low-pass filter.
    pub fn low_pass(freq_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);

        let b0 = (1.0 - cos_w0) / 2.0;
        let b1 = 1.0 - cos_w0;
        let b2 = (1.0 - cos_w0) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// Second-order high-pass filter.
    pub fn high_pass(freq_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);

        let b0 = (1.0 + cos_w0) / 2.0;
        let b1 = -(1.0 + cos_w0);
        let b2 = (1.0 + cos_w0) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// Low-shelf filter.
    pub fn low_shelf(freq_hz: f32, gain_db: f32, q: f32, sample_rate: f32) -> Self {
        let a = (10.0f32).powf(gain_db / 40.0);
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);
        let two_sqrt_a_alpha = 2.0 * a.sqrt() * alpha;

        let b0 = a * ((a + 1.0) - (a - 1.0) * cos_w0 + two_sqrt_a_alpha);
        let b1 = 2.0 * a * ((a - 1.0) - (a + 1.0) * cos_w0);
        let b2 = a * ((a + 1.0) - (a - 1.0) * cos_w0 - two_sqrt_a_alpha);
        let a0 = (a + 1.0) + (a - 1.0) * cos_w0 + two_sqrt_a_alpha;
        let a1 = -2.0 * ((a - 1.0) + (a + 1.0) * cos_w0);
        let a2 = (a + 1.0) + (a - 1.0) * cos_w0 - two_sqrt_a_alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// High-shelf filter.
    pub fn high_shelf(freq_hz: f32, gain_db: f32, q: f32, sample_rate: f32) -> Self {
        let a = (10.0f32).powf(gain_db / 40.0);
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);
        let two_sqrt_a_alpha = 2.0 * a.sqrt() * alpha;

        let b0 = a * ((a + 1.0) + (a - 1.0) * cos_w0 + two_sqrt_a_alpha);
        let b1 = -2.0 * a * ((a - 1.0) + (a + 1.0) * cos_w0);
        let b2 = a * ((a + 1.0) + (a - 1.0) * cos_w0 - two_sqrt_a_alpha);
        let a0 = (a + 1.0) - (a - 1.0) * cos_w0 + two_sqrt_a_alpha;
        let a1 = 2.0 * ((a - 1.0) - (a + 1.0) * cos_w0);
        let a2 = (a + 1.0) - (a - 1.0) * cos_w0 - two_sqrt_a_alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// Band-pass filter (constant-skirt, peak gain = Q).
    pub fn band_pass(freq_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);

        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// Notch (band-reject) filter.
    pub fn notch(freq_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);

        let b0 = 1.0;
        let b1 = -2.0 * cos_w0;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// All-pass filter.
    pub fn all_pass(freq_hz: f32, q: f32, sample_rate: f32) -> Self {
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let sin_w0 = w0.sin();
        let cos_w0 = w0.cos();
        let alpha = sin_w0 / (2.0 * q);

        let b0 = 1.0 - alpha;
        let b1 = -2.0 * cos_w0;
        let b2 = 1.0 + alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 = 1.0 - alpha;

        Self::normalise(b0, b1, b2, a0, a1, a2)
    }

    /// Normalise by a0 and convert to 5.27 fixed-point.
    ///
    /// The TAS5830 DSP expects negated feedback coefficients, so a1 and a2
    /// are stored as −a1/a0 and −a2/a0.
    fn normalise(b0: f32, b1: f32, b2: f32, a0: f32, a1: f32, a2: f32) -> Self {
        let inv_a0 = 1.0 / a0;
        Self {
            b0: f32_to_5_27(b0 * inv_a0),
            b1: f32_to_5_27(b1 * inv_a0),
            b2: f32_to_5_27(b2 * inv_a0),
            a1: f32_to_5_27(-a1 * inv_a0),
            a2: f32_to_5_27(-a2 * inv_a0),
        }
    }
}

// ── 9.23 fixed-point gain ───────────────────────────────────────────────────

/// A gain value in the TAS5830's 9.23 fixed-point format.
///
/// Used for mixer path gains and per-channel volume.
/// Stored as an unsigned `u32` (the DSP treats values ≥ 0x8000_0000 as
/// negative, but audio gains are normally positive).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct Gain923(pub u32);

impl Gain923 {
    /// 0 dB (unity gain)
    pub const UNITY: Self = Self(GAIN_ONE as u32);

    /// −∞ dB (silence)
    pub const MUTE: Self = Self(0);

    /// −6 dB (half amplitude)
    pub const MINUS_6DB: Self = Self((GAIN_ONE >> 1) as u32);

    /// Convert a dB value to 9.23 fixed-point.
    ///
    /// Uses successive 6 dB doublings/halvings for the coarse range
    /// with linear interpolation for the remainder, matching the
    /// approach used in the Linux TAS58xx driver.
    pub fn from_db(db: i16) -> Self {
        if db <= -110 {
            return Self::MUTE;
        }

        let mut value: u32 = GAIN_ONE as u32; // 0 dB reference
        let mut remaining = db;

        // Coarse: shift by 6 dB (factor of 2) steps
        while remaining >= 6 {
            value <<= 1;
            remaining -= 6;
        }
        while remaining <= -6 {
            value >>= 1;
            remaining += 6;
        }

        // Fine: linear approximation for the remaining ±5 dB
        if remaining > 0 {
            let delta = ((value >> 1) as i32 * remaining as i32 / 6) as u32;
            value += delta;
        } else if remaining < 0 {
            let delta = ((value >> 1) as i32 * (-remaining) as i32 / 6) as u32;
            value -= delta;
        }

        Self(value)
    }

    /// Serialise as 4 big-endian bytes.
    pub fn to_bytes(self) -> [u8; 4] {
        self.0.to_be_bytes()
    }
}

// ── Mixer ───────────────────────────────────────────────────────────────────

/// 4×4 mixer matrix gains.
///
/// Controls how the left/right input channels are routed to the left/right
/// output channels. Each field is a 9.23 fixed-point gain.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct MixerGains {
    /// Left input → Left output
    pub l2l: Gain923,
    /// Right input → Left output
    pub r2l: Gain923,
    /// Left input → Right output
    pub l2r: Gain923,
    /// Right input → Right output
    pub r2r: Gain923,
}

impl MixerGains {
    /// Standard stereo (L→L, R→R at 0 dB; cross-paths muted)
    pub const STEREO: Self = Self {
        l2l: Gain923::UNITY,
        r2l: Gain923::MUTE,
        l2r: Gain923::MUTE,
        r2r: Gain923::UNITY,
    };

    /// Mono downmix (both channels summed to both outputs at −6 dB)
    pub const MONO: Self = Self {
        l2l: Gain923::MINUS_6DB,
        r2l: Gain923::MINUS_6DB,
        l2r: Gain923::MINUS_6DB,
        r2r: Gain923::MINUS_6DB,
    };

    /// Left-only (left channel to both outputs)
    pub const LEFT_ONLY: Self = Self {
        l2l: Gain923::UNITY,
        r2l: Gain923::MUTE,
        l2r: Gain923::UNITY,
        r2r: Gain923::MUTE,
    };

    /// Right-only (right channel to both outputs)
    pub const RIGHT_ONLY: Self = Self {
        l2l: Gain923::MUTE,
        r2l: Gain923::UNITY,
        l2r: Gain923::MUTE,
        r2r: Gain923::UNITY,
    };
}

/// Named mixer presets
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum MixerMode {
    Stereo,
    Mono,
    LeftOnly,
    RightOnly,
}

impl MixerMode {
    pub fn to_gains(self) -> MixerGains {
        match self {
            Self::Stereo => MixerGains::STEREO,
            Self::Mono => MixerGains::MONO,
            Self::LeftOnly => MixerGains::LEFT_ONLY,
            Self::RightOnly => MixerGains::RIGHT_ONLY,
        }
    }
}

// ── Channel volume ──────────────────────────────────────────────────────────

/// Per-channel volume (post-mixer).
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct ChannelVolume {
    pub left: Gain923,
    pub right: Gain923,
}

impl ChannelVolume {
    /// Both channels at unity (0 dB)
    pub const UNITY: Self = Self {
        left: Gain923::UNITY,
        right: Gain923::UNITY,
    };
}

// ── Fault status ────────────────────────────────────────────────────────────

/// Decoded hardware fault / warning status (all read-only).
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct FaultStatus {
    /// Raw channel-fault register (0x70)
    pub chan_fault: u8,
    /// Raw global-fault-1 register (0x71)
    pub global_fault1: u8,
    /// Raw global-fault-2 register (0x72)
    pub global_fault2: u8,
    /// Raw over-temperature warning register (0x73)
    pub ot_warning: u8,
}

impl FaultStatus {
    /// Returns `true` if any fault bit is set.
    pub fn has_fault(&self) -> bool {
        self.chan_fault != 0 || self.global_fault1 != 0 || self.global_fault2 != 0
    }

    /// Returns `true` if any over-temperature warning is active.
    pub fn has_ot_warning(&self) -> bool {
        self.ot_warning != 0
    }

    /// Right channel over-current
    pub fn right_oc(&self) -> bool {
        self.chan_fault & super::regs::fault::RIGHT_OC != 0
    }
    /// Left channel over-current
    pub fn left_oc(&self) -> bool {
        self.chan_fault & super::regs::fault::LEFT_OC != 0
    }
    /// Right channel DC fault
    pub fn right_dc(&self) -> bool {
        self.chan_fault & super::regs::fault::RIGHT_DC != 0
    }
    /// Left channel DC fault
    pub fn left_dc(&self) -> bool {
        self.chan_fault & super::regs::fault::LEFT_DC != 0
    }
    /// PVDD under-voltage
    pub fn pvdd_uv(&self) -> bool {
        self.global_fault1 & super::regs::fault::PVDD_UV != 0
    }
    /// PVDD over-voltage
    pub fn pvdd_ov(&self) -> bool {
        self.global_fault1 & super::regs::fault::PVDD_OV != 0
    }
    /// Clock fault
    pub fn clock_fault(&self) -> bool {
        self.global_fault1 & super::regs::fault::CLOCK != 0
    }
    /// Thermal shutdown
    pub fn thermal_shutdown(&self) -> bool {
        self.global_fault2 & super::regs::fault::THERMAL_SD != 0
    }
}

// ── Filter description (for UI / config serialisation) ──────────────────────

/// Describes the filter applied to one EQ band.
///
/// This is a high-level description that can be stored in a config file
/// and converted to [`BiquadCoeffs`] at a given sample rate.
#[derive(Debug, Clone, Copy)]
pub enum FilterType {
    /// Parametric peaking EQ: centre frequency, gain (dB), Q
    Peaking {
        freq_hz: f32,
        gain_db: f32,
        q: f32,
    },
    /// Second-order Butterworth low-pass
    LowPass { freq_hz: f32, q: f32 },
    /// Second-order Butterworth high-pass
    HighPass { freq_hz: f32, q: f32 },
    /// Low-shelf
    LowShelf {
        freq_hz: f32,
        gain_db: f32,
        q: f32,
    },
    /// High-shelf
    HighShelf {
        freq_hz: f32,
        gain_db: f32,
        q: f32,
    },
    /// Band-pass (constant-skirt)
    BandPass { freq_hz: f32, q: f32 },
    /// Notch (band-reject)
    Notch { freq_hz: f32, q: f32 },
    /// All-pass
    AllPass { freq_hz: f32, q: f32 },
    /// No filtering (passthrough)
    Bypass,
}

impl FilterType {
    /// Compute the biquad coefficients for this filter at the given sample rate.
    pub fn to_coeffs(&self, sample_rate: f32) -> BiquadCoeffs {
        match *self {
            Self::Peaking {
                freq_hz,
                gain_db,
                q,
            } => BiquadCoeffs::peaking(freq_hz, gain_db, q, sample_rate),
            Self::LowPass { freq_hz, q } => BiquadCoeffs::low_pass(freq_hz, q, sample_rate),
            Self::HighPass { freq_hz, q } => BiquadCoeffs::high_pass(freq_hz, q, sample_rate),
            Self::LowShelf {
                freq_hz,
                gain_db,
                q,
            } => BiquadCoeffs::low_shelf(freq_hz, gain_db, q, sample_rate),
            Self::HighShelf {
                freq_hz,
                gain_db,
                q,
            } => BiquadCoeffs::high_shelf(freq_hz, gain_db, q, sample_rate),
            Self::BandPass { freq_hz, q } => BiquadCoeffs::band_pass(freq_hz, q, sample_rate),
            Self::Notch { freq_hz, q } => BiquadCoeffs::notch(freq_hz, q, sample_rate),
            Self::AllPass { freq_hz, q } => BiquadCoeffs::all_pass(freq_hz, q, sample_rate),
            Self::Bypass => BiquadCoeffs::PASSTHROUGH,
        }
    }
}
