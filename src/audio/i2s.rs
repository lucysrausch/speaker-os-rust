//! I2S driver using PIO
//!
//! Implements I2S master mode for both input (PCM1822 ADC) and output (TAS5830 amp).
//! Uses RP2350 PIO state machines for generating/receiving I2S signals.
//!
//! ## Architecture
//! Each interface (ADC input, AMP output) uses its own state machine that generates
//! its own BCLK/WCLK clocks. By starting both SMs atomically with identical clock
//! dividers, they remain phase-locked since they share the PIO clock source.
//!
//! ## Configuration
//! - Format: 32-bit stereo I2S (64 BCLK per frame)
//! - Sample rate: 96kHz
//! - BCLK = 96000 * 64 = 6.144 MHz
//!
//! ## Pin Assignments (from PINOUT.md)
//! - ADC (PCM1822): BCLK=GPIO23, WCLK=GPIO24, DATA=GPIO25
//! - AMP (TAS5830): BCLK=GPIO19, WCLK=GPIO20, DATA=GPIO21
//!
//! ## PIO State Machine Usage (PIO1)
//! - SM0: I2S TX to AMP (generates BCLK/WCLK, outputs DATA)
//! - SM1: I2S RX from ADC (generates BCLK/WCLK, inputs DATA)

use embassy_rp::clocks;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::Peri;
use fixed::traits::ToFixed;
use fixed::types::U24F8;

/// Stereo audio sample (left in upper 32 bits, right in lower 32 bits)
pub type StereoSample = u64;

/// Calculate PIO clock divider for a given sample rate
///
/// Returns the divider as a fixed-point value for use in PIO config.
/// All I2S state machines should use this same divider for phase-lock.
pub fn calculate_clock_divider(sample_rate: u32) -> U24F8 {
    // BCLK = sample_rate * 64 (32 bits * 2 channels)
    // Each bit takes 4 PIO cycles (2 instructions with [1] delay each)
    let bclk_freq = sample_rate * 64;
    let sys_freq = clocks::clk_sys_freq();
    let div = (sys_freq as f64) / (bclk_freq as f64 * 4.0);
    div.to_fixed()
}

/// I2S Transmitter (to TAS5830 amplifier)
///
/// Generates BCLK and WCLK, outputs DATA. Uses a single state machine.
/// Pin assignments: BCLK=GPIO19, WCLK=GPIO20, DATA=GPIO21
pub struct I2sTx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> I2sTx<'d, PIO, SM> {
    /// Create I2S transmitter
    ///
    /// Does NOT start the state machine - call `start()` or use atomic enable.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        bclk_pin: Peri<'d, impl PioPin>,
        wclk_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        clock_divider: U24F8,
    ) -> Self {
        // PIO program for I2S transmit with clock generation
        // Uses 2-bit sideset for BCLK and WCLK to ensure glitch-free timing
        // Sideset bit 0 = BCLK, bit 1 = WCLK
        // OUT: DATA
        //
        // I2S standard format:
        // - WCLK low = left channel, WCLK high = right channel
        // - Data transitions on BCLK falling edge
        // - Data is sampled on BCLK rising edge
        // - MSB first, 32 bits per channel
        //
        // Each channel: 31 bits in loop + 1 bit with counter reload = 32 bits
        // Total: 64 BCLK cycles per stereo frame
        #[rustfmt::skip]
        let prg = pio::pio_asm!(
            ".side_set 2",
            // Initialize counter before first frame (runs once at startup)
            "set x, 30         side 0b00",
            ".wrap_target",
            // Left channel (WCLK=0): 31 bits in loop + 1 bit special
            "left_loop:",
            "out pins, 1       side 0b00 [1]", // Data out, WCLK=0, BCLK=0
            "jmp x-- left_loop side 0b01 [1]", // WCLK=0, BCLK=1 (sample)
            // 32nd bit of left channel, reload counter for right
            "out pins, 1       side 0b00 [1]", // Last left bit, BCLK=0
            "set x, 30         side 0b01 [1]", // Counter=30 (31 iterations), BCLK=1
            // Right channel (WCLK=1): 31 bits in loop + 1 bit special
            "right_loop:",
            "out pins, 1       side 0b10 [1]", // Data out, WCLK=1, BCLK=0
            "jmp x-- right_loop side 0b11 [1]", // WCLK=1, BCLK=1 (sample)
            // 32nd bit of right channel, reload counter for left
            "out pins, 1       side 0b10 [1]", // Last right bit, BCLK=0
            "set x, 30         side 0b11 [1]", // Counter=30, BCLK=1
            ".wrap",
        );

        let bclk = common.make_pio_pin(bclk_pin);
        let wclk = common.make_pio_pin(wclk_pin);
        let data = common.make_pio_pin(data_pin);

        let mut cfg = Config::default();
        let loaded = common.load_program(&prg.program);
        cfg.use_program(&loaded, &[&bclk, &wclk]); // Sideset = BCLK, WCLK (consecutive pins)
        cfg.set_out_pins(&[&data]);                 // OUT = DATA
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left, // MSB first
        };
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = clock_divider;

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&bclk, &wclk, &data]);

        Self { sm }
    }

    /// Start the transmitter
    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }

    /// Stop the transmitter
    pub fn stop(&mut self) {
        self.sm.set_enable(false);
    }

    /// Write left and right channel samples (32-bit each, blocking)
    pub fn write(&mut self, left: u32, right: u32) {
        while !self.sm.tx().try_push(left) {}
        while !self.sm.tx().try_push(right) {}
    }

    /// Try to write samples (non-blocking)
    pub fn try_write(&mut self, left: u32, right: u32) -> bool {
        if self.sm.tx().try_push(left) {
            self.sm.tx().try_push(right)
        } else {
            false
        }
    }

    /// Check if TX FIFO has space
    pub fn ready(&mut self) -> bool {
        !self.sm.tx().full()
    }

    /// Check if TX FIFO is empty
    pub fn empty(&mut self) -> bool {
        self.sm.tx().empty()
    }
}

/// I2S Receiver (from PCM1822 ADC)
///
/// Generates BCLK and WCLK, receives DATA. Uses a single state machine.
/// Pin assignments: BCLK=GPIO23, WCLK=GPIO24, DATA=GPIO25
pub struct I2sRx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> I2sRx<'d, PIO, SM> {
    /// Create I2S receiver
    ///
    /// Does NOT start the state machine - call `start()` or use atomic enable.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        bclk_pin: Peri<'d, impl PioPin>,
        wclk_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        clock_divider: U24F8,
    ) -> Self {
        // PIO program for I2S receive with clock generation
        // Same timing as TX for phase alignment
        // Data is sampled on BCLK rising edge
        #[rustfmt::skip]
        let prg = pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            // Left channel (WCLK=0): input 32 bits
            "set pins, 0       side 0",     // WCLK low, BCLK low
            "set x, 31         side 0",
            "left_loop:",
            "nop               side 1 [1]", // BCLK high (sample data here)
            "in pins, 1        side 0 [1]", // Read data bit, BCLK low
            "jmp x-- left_loop side 0",
            // Right channel (WCLK=1): input 32 bits
            "set pins, 1       side 0",     // WCLK high, BCLK low
            "set x, 31         side 0",
            "right_loop:",
            "nop               side 1 [1]", // BCLK high (sample data here)
            "in pins, 1        side 0 [1]", // Read data bit, BCLK low
            "jmp x-- right_loop side 0",
            ".wrap",
        );

        let bclk = common.make_pio_pin(bclk_pin);
        let wclk = common.make_pio_pin(wclk_pin);
        let data = common.make_pio_pin(data_pin);

        let mut cfg = Config::default();
        let loaded = common.load_program(&prg.program);
        cfg.use_program(&loaded, &[&bclk]); // Sideset = BCLK
        cfg.set_set_pins(&[&wclk]);          // SET = WCLK
        cfg.set_in_pins(&[&data]);           // IN = DATA
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left, // MSB first
        };
        cfg.fifo_join = FifoJoin::RxOnly;
        cfg.clock_divider = clock_divider;

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&bclk, &wclk]);
        sm.set_pin_dirs(Direction::In, &[&data]);

        Self { sm }
    }

    /// Start the receiver
    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }

    /// Stop the receiver
    pub fn stop(&mut self) {
        self.sm.set_enable(false);
    }

    /// Read left and right channel samples (32-bit each, blocking)
    pub fn read(&mut self) -> (u32, u32) {
        let left = loop {
            if let Some(v) = self.sm.rx().try_pull() {
                break v;
            }
        };
        let right = loop {
            if let Some(v) = self.sm.rx().try_pull() {
                break v;
            }
        };
        (left, right)
    }

    /// Try to read samples (non-blocking)
    pub fn try_read(&mut self) -> Option<(u32, u32)> {
        let left = self.sm.rx().try_pull()?;
        let right = self.sm.rx().try_pull()?;
        Some((left, right))
    }

    /// Check if RX FIFO has data
    pub fn available(&mut self) -> bool {
        !self.sm.rx().empty()
    }
}

