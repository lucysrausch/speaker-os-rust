//! Audio subsystem for OtterAmp DSP
//!
//! Handles audio I/O using PIO state machines:
//! - I2S output to TAS5830 amplifier (with clock recovery)
//! - S/PDIF input (IEC 60958)
//!
//! Clock recovery architecture:
//! - I2S output clock derived from detected sample rate
//! - Adaptive clock adjustment (±1 divider) keeps buffer stable
//! - Simple 2x up/downsampling for rate family conversion

pub mod i2s;
pub mod pipeline;
pub mod spdif;

pub use i2s::{I2sTx, I2sRx, ClockSpeed};
pub use pipeline::{AudioRingBuffer, StereoFrame, BUFFER_SIZE};
pub use spdif::{SpdifRx, State as SpdifState, extract_audio, SPDIF_RX_FIFO_SIZE, DMA_BLOCK_SIZE};
