//! Pin assignments for OtterAmp DSP (RP2350A)
//!
//! GPIO mapping from PINOUT.md with hardware errata corrections.
//!
//! ## Hardware Errata (Rev 1.0)
//!
//! The original schematic has I2C pin assignments that don't match RP2350's
//! fixed function mapping. The following bodges are required:
//!
//! - **I2C0**: GPIO0↔GPIO1 swapped (schematic had SCL/SDA backwards)
//! - **I2C1**: GPIO13→GPIO15 bridged (GPIO13 only supports I2C0, not I2C1)
//! - **UART**: TX→GPIO4, RX→GPIO5 (optional, for debug UART)
//!
//! Rev 1.1 will fix these in hardware.

use embassy_rp::peripherals::*;

/// Pin numbers as constants for reference
/// Note: Comments indicate Rev 1.0 bodge status
pub mod gpio {
    // I2C0 - OLED displays
    // BODGE Rev1.0: Swap GPIO0↔GPIO1 traces at OLED connector
    pub const I2C0_SDA: u8 = 0;
    pub const I2C0_SCL: u8 = 1;

    // S/PDIF
    pub const SPDIF_TX: u8 = 2;
    pub const SPDIF_RX: u8 = 3;
    pub const SPDIF_SEL: u8 = 4;

    // External GPIO
    pub const EXT_GPIO: u8 = 5;

    // Debug UART
    pub const UART_TX: u8 = 6;
    pub const UART_RX: u8 = 7;

    // Rotary encoder
    pub const ENC_BTN: u8 = 8;
    pub const ENC_A: u8 = 9;
    pub const ENC_B: u8 = 10;

    // Status LEDs
    pub const LED1: u8 = 11;
    pub const LED0: u8 = 12;

    // I2C1 - TAS5830 amplifier
    // BODGE Rev1.0: Bridge GPIO13→GPIO15 (GPIO13 only supports I2C0!)
    // After bodge: GPIO14=SDA (correct), GPIO15=SCL (bridged from GPIO13)
    pub const I2C1_SCL: u8 = 15; // Was 13 in schematic, bodged to 15
    pub const I2C1_SDA: u8 = 14; // Correct
    pub const I2C1_INT: u8 = 13; // Was 15 in schematic, now GPIO13 (I2C1.INT sacrificed in Rev1.0)

    // Amplifier control
    pub const AMP_FLT: u8 = 16;
    pub const AMP_PDN: u8 = 17;
    pub const AMP_MUTE: u8 = 18;

    // I2S to TAS5830 amplifier (PIO)
    pub const AMP_BCLK: u8 = 19;
    pub const AMP_WCLK: u8 = 20;
    pub const AMP_DATA: u8 = 21;
    pub const AMP_RTN: u8 = 22;

    // I2S from PCM1822 ADC (PIO)
    pub const ADC_BCLK: u8 = 23;
    pub const ADC_WCLK: u8 = 24;
    pub const ADC_DATA: u8 = 25;
}

/// Type aliases for embassy-rp peripheral pins
/// These reflect the ACTUAL pin usage after Rev 1.0 bodges
pub mod peripherals {
    use embassy_rp::peripherals::*;

    // I2C0 - OLED (after bodge: GPIO0=SDA, GPIO1=SCL)
    pub type I2c0Sda = PIN_0;
    pub type I2c0Scl = PIN_1;

    // S/PDIF
    pub type SpdifTx = PIN_2;
    pub type SpdifRx = PIN_3;
    pub type SpdifSel = PIN_4;

    // Rotary encoder
    pub type EncBtn = PIN_8;
    pub type EncA = PIN_9;
    pub type EncB = PIN_10;

    // Status LEDs
    pub type Led1 = PIN_11;
    pub type Led0 = PIN_12;

    // I2C1 - TAS5830 (after bodge: GPIO14=SDA, GPIO15=SCL)
    pub type I2c1Sda = PIN_14;
    pub type I2c1Scl = PIN_15;
    // Note: I2C1.INT on GPIO13 not usable in Rev1.0 (used for SCL bodge source)

    // Amplifier control
    pub type AmpFlt = PIN_16;
    pub type AmpPdn = PIN_17;
    pub type AmpMute = PIN_18;

    // I2S to amplifier
    pub type AmpBclk = PIN_19;
    pub type AmpWclk = PIN_20;
    pub type AmpData = PIN_21;
    pub type AmpRtn = PIN_22;

    // I2S from ADC
    pub type AdcBclk = PIN_23;
    pub type AdcWclk = PIN_24;
    pub type AdcData = PIN_25;
}

/// I2C addresses
pub mod i2c_addr {
    /// SH1106 OLED display (typically 0x3C or 0x3D)
    pub const SH1106: u8 = 0x3C;

    /// TAS5830 amplifier default address
    pub const TAS5830: u8 = 0x60;
}

/// I2C bus speeds
pub mod i2c_freq {
    /// Standard mode for OLED (400kHz)
    pub const OLED_HZ: u32 = 100_000;

    /// Fast mode for TAS5830 (1MHz supported, using 400kHz for reliability)
    pub const AMP_HZ: u32 = 400_000;
}

/// Audio configuration
pub mod audio {
    /// Sample rate in Hz
    pub const SAMPLE_RATE: u32 = 96_000;

    /// Bit depth per channel
    pub const BIT_DEPTH: u8 = 32;

    /// Number of channels
    pub const CHANNELS: u8 = 2;

    /// I2S bit clock frequency = sample_rate * bit_depth * channels
    pub const BCLK_FREQ: u32 = SAMPLE_RATE * (BIT_DEPTH as u32) * (CHANNELS as u32);

    /// S/PDIF uses 128x oversampling for the biphase encoding
    pub const SPDIF_BCLK_FREQ: u32 = SAMPLE_RATE * 128;
}

/// PIO state machine assignments
pub mod pio {
    /// PIO0 is used for S/PDIF
    pub const SPDIF_PIO: u8 = 0;
    pub const SPDIF_RX_SM: u8 = 0;
    pub const SPDIF_TX_SM: u8 = 1;

    /// PIO1 is used for I2S
    pub const I2S_PIO: u8 = 1;
    pub const I2S_CONTROLLER_SM: u8 = 0;  // Clock generation
    pub const I2S_ADC_SM: u8 = 1;     // ADC input
    pub const I2S_AMP_SM: u8 = 2;     // Amplifier output
}
