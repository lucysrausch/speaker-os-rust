//! USB Audio subsystem
//!
//! Implements USB Audio Class 1.0 speaker functionality for streaming
//! audio from host to the speaker system.
//!
//! ## Configuration
//! - Sample rate: 96kHz
//! - Bit depth: 24-bit
//! - Channels: Stereo (Left/Right)
//! - USB: Full-speed (12 Mbps)
//!
//! ## Data Flow
//! Host → USB → Ring buffer → I2S output

pub mod audio;

pub use audio::{UsbAudioConfig, UsbAudioReceiver};
