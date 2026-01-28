//! Hardware abstraction layer for OtterAmp DSP
//!
//! Provides type-safe pin assignments and peripheral initialization
//! based on the hardware design in PINOUT.md.

pub mod pins;

pub use pins::*;
