//! Pin assignments for OtterAmp DSP (RP2350A)
//!
//! GPIO mapping from PINOUT.md with hardware errata corrections.
//! This file is the SINGLE source of truth for all GPIO and hardware config.
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

/// Internal macro: generates `BoardPins` struct + `board_pins!` constructor
/// from a single pin list so GPIO assignments are defined in exactly one place.
macro_rules! define_board_pins {
    ( $( $field:ident : $pin:ident ),* $(,)? ) => {
        /// All board pins, extracted from `Peripherals` via the `board_pins!` macro.
        pub struct BoardPins {
            $( pub $field: embassy_rp::Peri<'static, embassy_rp::peripherals::$pin>, )*
        }

        /// Destructure embassy `Peripherals` into named board pins.
        /// Usage: `let pins = board_pins!(p);`
        #[macro_export]
        macro_rules! board_pins {
            ($p:ident) => {{
                $crate::hw::pins::BoardPins {
                    $( $field: $p.$pin, )*
                }
            }};
        }
    };
}

// GPIO Pin Map (active pins extracted, others unused in software)
//
// | GPIO | Function         | Notes                                          |
// |------|------------------|-------------------------------------------------|
// |  0   | I2C0 SDA (OLED)  | BODGE Rev1.0: GPIO0↔GPIO1 swapped at connector |
// |  1   | I2C0 SCL (OLED)  | BODGE Rev1.0: GPIO0↔GPIO1 swapped at connector |
// |  2   | S/PDIF TX        | (unused in software)                            |
// |  3   | S/PDIF RX        |                                                 |
// |  4   | S/PDIF SEL       | (unused in software)                            |
// |  5   | External GPIO    | (unused in software)                            |
// |  6   | Debug UART TX    | (unused in software)                            |
// |  7   | Debug UART RX    | (unused in software)                            |
// |  8   | Encoder button   |                                                 |
// |  9   | Encoder A        |                                                 |
// | 10   | Encoder B        |                                                 |
// | 11   | LED1             | (unused in software)                            |
// | 12   | LED0             |                                                 |
// | 13   | I2C1 INT         | BODGE Rev1.0: was SCL, sacrificed for bodge     |
// | 14   | I2C1 SDA (amp)   |                                                 |
// | 15   | I2C1 SCL (amp)   | BODGE Rev1.0: bridged from GPIO13               |
// | 16   | AMP_FLT          | (unused in software)                            |
// | 17   | AMP_PDN          |                                                 |
// | 18   | AMP_MUTE         |                                                 |
// | 19   | AMP I2S BCLK     |                                                 |
// | 20   | AMP I2S WCLK     |                                                 |
// | 21   | AMP I2S DATA     |                                                 |
// | 22   | AMP I2S RTN      | (unused in software)                            |
// | 23   | ADC I2S BCLK     |                                                 |
// | 24   | ADC I2S WCLK     |                                                 |
// | 25   | ADC I2S DATA     |                                                 |
// | 29   | USB VBUS sense   | Via 5.1k:10k voltage divider                    |
define_board_pins! {
    i2c0_sda: PIN_0,  // BODGE Rev1.0: swapped with SCL
    i2c0_scl: PIN_1,  // BODGE Rev1.0: swapped with SDA
    spdif_rx: PIN_3,
    enc_btn:  PIN_8,
    enc_a:    PIN_9,
    enc_b:    PIN_10,
    led0:     PIN_12,
    i2c1_sda: PIN_14,
    i2c1_scl: PIN_15, // BODGE Rev1.0: bridged from GPIO13
    amp_pdn:  PIN_17,
    amp_mute: PIN_18,
    amp_bclk: PIN_19,
    amp_wclk: PIN_20,
    amp_data: PIN_21,
    adc_bclk: PIN_23,
    adc_wclk: PIN_24,
    adc_data: PIN_25,
    usb_vbus: PIN_29,
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
    /// Fast mode for OLED (400kHz)
    pub const OLED_HZ: u32 = 400_000;

    /// Fast mode for TAS5830 (1MHz supported, using 400kHz for reliability)
    pub const AMP_HZ: u32 = 400_000;
}

/// Audio configuration
pub mod audio {
    /// I2S output sample rate in Hz
    pub const SAMPLE_RATE: u32 = 96_000;

    /// USB audio input sample rate in Hz
    pub const USB_SAMPLE_RATE: u32 = 96_000;

    /// Bit depth per channel
    pub const BIT_DEPTH: u8 = 32;

    /// Number of channels
    pub const CHANNELS: u8 = 2;

    /// I2S bit clock frequency = sample_rate * bit_depth * channels
    pub const BCLK_FREQ: u32 = SAMPLE_RATE * (BIT_DEPTH as u32) * (CHANNELS as u32);

    /// S/PDIF uses 128x oversampling for the biphase encoding
    pub const SPDIF_BCLK_FREQ: u32 = SAMPLE_RATE * 128;
}
