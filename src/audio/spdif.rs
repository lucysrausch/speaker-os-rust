//! S/PDIF receiver using PIO
//!
//! Implements S/PDIF (IEC 60958) input using PIO state machines.
//! Based on biphase mark coding (BMC) decoding.
//!
//! ## S/PDIF Format
//! - Biphase mark encoding at 128x sample rate (e.g., 12.288 MHz for 96kHz audio)
//! - Each subframe: preamble (4 symbols) + 28 data bits (audio + aux + control)
//! - Frames: left subframe + right subframe
//! - Blocks: 192 frames
//!
//! ## Pin Assignment
//! - SPDIF_RX: GPIO3 (input, active-high after signal conditioning)
//!
//! ## Implementation
//! Uses edge detection to decode BMC:
//! - Sample at high rate to detect transitions
//! - Short period (half bit) = '1' (transition in middle)
//! - Long period (full bit) = '0' (no middle transition)

use embassy_rp::clocks;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::Peri;
use fixed::traits::ToFixed;

/// S/PDIF preambles (after BMC decoding, these are the sync patterns)
pub mod preamble {
    /// B preamble: Start of block, left channel (channel A)
    pub const B: u8 = 0b00010111;
    /// M preamble: Left channel (channel A), not start of block
    pub const M: u8 = 0b01000111;
    /// W preamble: Right channel (channel B)
    pub const W: u8 = 0b00100111;
}

/// S/PDIF channel status bits (subset of 192-bit block)
#[derive(Debug, Clone, Copy, Default)]
pub struct ChannelStatus {
    /// Professional/consumer use (0 = consumer)
    pub professional: bool,
    /// Audio/non-audio (0 = audio)
    pub non_audio: bool,
    /// Copyright
    pub copyright: bool,
    /// Emphasis
    pub emphasis: u8,
    /// Sample rate (decoded from bits 24-27)
    pub sample_rate: SampleRate,
}

/// Detected sample rate from S/PDIF channel status
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, defmt::Format)]
pub enum SampleRate {
    #[default]
    Unknown,
    Rate44100,
    Rate48000,
    Rate88200,
    Rate96000,
    Rate176400,
    Rate192000,
}

impl SampleRate {
    pub fn hz(&self) -> u32 {
        match self {
            SampleRate::Unknown => 0,
            SampleRate::Rate44100 => 44100,
            SampleRate::Rate48000 => 48000,
            SampleRate::Rate88200 => 88200,
            SampleRate::Rate96000 => 96000,
            SampleRate::Rate176400 => 176400,
            SampleRate::Rate192000 => 192000,
        }
    }
}

/// S/PDIF receiver state
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RxState {
    /// No signal detected
    NoSignal,
    /// Synchronizing to preamble
    Syncing,
    /// Locked and receiving data
    Locked,
    /// Signal lost after being locked
    SignalLost,
}

/// S/PDIF decoded audio sample
#[derive(Debug, Clone, Copy, Default)]
pub struct SpdifSample {
    /// Left channel (24-bit audio, sign-extended to i32)
    pub left: i32,
    /// Right channel (24-bit audio, sign-extended to i32)
    pub right: i32,
    /// Validity bit (0 = valid audio)
    pub valid: bool,
    /// User data bit
    pub user: bool,
}

/// S/PDIF receiver driver
pub struct SpdifRx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    state: RxState,
    frame_count: u32,
    sample_rate: SampleRate,
}

impl<'d, PIO: Instance, const SM: usize> SpdifRx<'d, PIO, SM> {
    /// Create S/PDIF receiver
    ///
    /// The PIO program decodes BMC-encoded S/PDIF signal by measuring
    /// the time between edges. Short periods indicate '1' bits,
    /// long periods indicate '0' bits.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin>,
    ) -> Self {
        // S/PDIF BMC decoding using edge timing
        //
        // The PIO measures time between input edges:
        // - At 96kHz: bit cell = 81.4ns, half-cell = 40.7ns
        // - We sample at ~24.576 MHz (4x bit rate) for edge detection
        //
        // Algorithm:
        // 1. Wait for edge
        // 2. Count cycles until next edge
        // 3. If count < threshold: '1' (half-cell, middle transition)
        // 4. If count >= threshold: '0' (full-cell, no middle transition)
        // 5. Shift bit into ISR
        // 6. After 32 bits, push to FIFO
        #[rustfmt::skip]
        let prg = pio::pio_asm!(
            // Y register holds the threshold (set by CPU)
            // X register is the edge counter
            ".wrap_target",
            "wait_edge:",
            "wait 1 pin 0",           // Wait for rising edge
            "set x, 0",               // Reset counter
            "count_high:",
            "jmp x-- count_high [1]", // Count while high (will wrap, that's ok)
            "jmp pin count_high",     // Keep counting while pin is high
            // Pin went low - got our timing
            "mov isr, x",             // Save count
            "wait 0 pin 0",           // Wait for falling edge
            "set x, 0",
            "count_low:",
            "jmp x-- count_low [1]",
            "jmp pin done_low",       // Exit when pin goes high
            "jmp count_low",
            "done_low:",
            // Now ISR has the high-time count, X has low-time count
            // Compare to threshold to determine bit value
            "mov x, isr",             // Get high-time
            "jmp x!=y short_pulse",   // If != threshold, check if shorter
            "jmp long_pulse",         // Equal to threshold = long pulse = '0'
            "short_pulse:",
            "jmp !x long_pulse",      // If x wrapped to 0, it's actually long
            "set x, 1",               // Short pulse = '1'
            "in x, 1",                // Shift in '1'
            "jmp wait_edge",
            "long_pulse:",
            "set x, 0",
            "in x, 1",                // Shift in '0'
            ".wrap",
        );

        let rx = common.make_pio_pin(rx_pin);

        // Calculate clock divider for ~25MHz sampling (4x max bit rate)
        let sys_freq = clocks::clk_sys_freq();
        let sample_freq = 25_000_000u32;
        let div = (sys_freq as f64) / (sample_freq as f64);

        let mut cfg = Config::default();
        let loaded = common.load_program(&prg.program);
        cfg.use_program(&loaded, &[]);
        cfg.set_in_pins(&[&rx]);
        cfg.set_jmp_pin(&rx);
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left,
        };
        cfg.fifo_join = FifoJoin::RxOnly;
        cfg.clock_divider = div.to_fixed();

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::In, &[&rx]);

        Self {
            sm,
            state: RxState::NoSignal,
            frame_count: 0,
            sample_rate: SampleRate::Unknown,
        }
    }

    /// Start the S/PDIF receiver
    pub fn start(&mut self) {
        self.state = RxState::Syncing;
        self.sm.set_enable(true);
    }

    /// Stop the S/PDIF receiver
    pub fn stop(&mut self) {
        self.sm.set_enable(false);
        self.state = RxState::NoSignal;
    }

    /// Get current receiver state
    pub fn state(&self) -> RxState {
        self.state
    }

    /// Get detected sample rate
    pub fn sample_rate(&self) -> SampleRate {
        self.sample_rate
    }

    /// Check if locked to valid S/PDIF signal
    pub fn is_locked(&self) -> bool {
        self.state == RxState::Locked
    }

    /// Try to read a decoded audio sample
    ///
    /// Returns None if no complete frame is available
    pub fn try_read(&mut self) -> Option<SpdifSample> {
        // Need at least 2 words for a stereo frame
        if self.sm.rx().level() < 2 {
            return None;
        }

        let left_raw = self.sm.rx().pull();
        let right_raw = self.sm.rx().pull();

        // Decode the raw data
        // Format: [preamble:4][aux:4][audio:20][V:1][U:1][C:1][P:1]
        // But we receive LSB first, so we need to reverse

        let left = Self::decode_subframe(left_raw);
        let right = Self::decode_subframe(right_raw);

        self.frame_count = self.frame_count.wrapping_add(1);

        if self.state == RxState::Syncing {
            self.state = RxState::Locked;
        }

        Some(SpdifSample {
            left: left.0,
            right: right.0,
            valid: left.1 && right.1,
            user: false,
        })
    }

    /// Decode a subframe into audio sample and validity
    fn decode_subframe(raw: u32) -> (i32, bool) {
        // Extract 24-bit audio (bits 4-27, but our bit order might be reversed)
        // This is a simplified decode - real implementation would handle preambles
        let audio_24 = ((raw >> 4) & 0x00FFFFFF) as i32;

        // Sign extend from 24 to 32 bits
        let audio = if audio_24 & 0x800000 != 0 {
            audio_24 | 0xFF000000u32 as i32
        } else {
            audio_24
        };

        // Validity bit (bit 28, 0 = valid)
        let valid = (raw & (1 << 28)) == 0;

        (audio, valid)
    }

    /// Read raw 32-bit word from FIFO (for debugging)
    pub fn read_raw(&mut self) -> Option<u32> {
        if self.sm.rx().empty() {
            None
        } else {
            Some(self.sm.rx().pull())
        }
    }

    /// Check if RX FIFO has data
    pub fn rx_ready(&mut self) -> bool {
        !self.sm.rx().empty()
    }

    /// Get number of 32-bit words in RX FIFO
    pub fn rx_available(&mut self) -> usize {
        self.sm.rx().level() as usize
    }
}

/// S/PDIF signal detector
///
/// A simpler implementation that just detects presence of S/PDIF signal
/// by looking for edges at the expected frequency.
pub struct SpdifDetector<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> SpdifDetector<'d, PIO, SM> {
    /// Create S/PDIF signal detector
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin>,
    ) -> Self {
        // Simple edge counter program
        // Counts edges over a fixed period to detect valid S/PDIF signal
        #[rustfmt::skip]
        let prg = pio::pio_asm!(
            ".wrap_target",
            "set x, 0",              // Edge counter
            "set y, 31",             // Timeout counter (32 loops)
            "count_loop:",
            "wait 1 pin 0",          // Wait for rising edge
            "jmp y-- continue",      // Decrement loop counter
            "continue:",
            "wait 0 pin 0 [7]",      // Wait for falling edge with delay
            "jmp x-- count_loop",    // Decrement edge counter and loop
            // Done counting - push result
            "mov isr, x",
            "push",
            ".wrap",
        );

        let rx = common.make_pio_pin(rx_pin);

        let mut cfg = Config::default();
        let loaded = common.load_program(&prg.program);
        cfg.use_program(&loaded, &[]);
        cfg.set_in_pins(&[&rx]);
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: ShiftDirection::Left,
        };
        cfg.fifo_join = FifoJoin::RxOnly;
        // Run at 1MHz for easy timing
        let sys_freq = clocks::clk_sys_freq();
        cfg.clock_divider = ((sys_freq / 1_000_000) as u16).to_fixed();

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::In, &[&rx]);

        Self { sm }
    }

    /// Start detection
    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }

    /// Stop detection
    pub fn stop(&mut self) {
        self.sm.set_enable(false);
    }

    /// Check for signal presence
    ///
    /// Returns true if edges are detected at approximately the right rate
    pub fn signal_detected(&mut self) -> bool {
        if self.sm.rx().empty() {
            return false;
        }

        let edge_count = self.sm.rx().pull();
        // For valid S/PDIF, we expect many edges in our counting window
        // At 96kHz, ~12M edges/sec, over ~32us window = ~400 edges
        // We just check for "reasonable" activity
        edge_count > 100 && edge_count < 0xFFFF0000
    }
}
