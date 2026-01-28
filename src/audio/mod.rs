//! Audio subsystem for OtterAmp DSP
//!
//! Handles audio I/O using PIO state machines:
//! - I2S input from PCM1822 ADC
//! - I2S output to TAS5830 amplifier
//! - S/PDIF input (IEC 60958)

pub mod i2s;
pub mod spdif;

pub use i2s::{I2sTx, I2sRx, StereoSample, calculate_clock_divider};
pub use spdif::{SpdifRx, SpdifDetector, SpdifSample, SampleRate, RxState};
