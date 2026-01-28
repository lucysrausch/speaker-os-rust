//! Device drivers for OtterAmp DSP peripherals

pub mod encoder;
pub mod tas5830;

pub use encoder::RotaryEncoder;
pub use tas5830::Tas5830;
