//! I2S driver using PIO with dynamic sample rate and adaptive clock
//!
//! Based on pico_spdif_rx / pico_audio_i2s_32b by Elehobica (BSD-2-Clause)
//!
//! Implements I2S controller mode for output (TAS5830 amp).
//! Uses RP2350 PIO state machines for generating I2S signals.
//!
//! ## Clock Recovery
//! The I2S TX driver supports dynamic sample rate changes and adaptive clock adjustment
//! for clock recovery from S/PDIF input:
//! - `set_sample_rate()` - reconfigure for a new sample rate
//! - `adjust_clock()` - fine-tune clock ±1 divider step for buffer level control
//!
//! ## Configuration
//! - Format: 32-bit stereo I2S (64 BCLK per frame)
//! - Sample rates: 44.1kHz, 48kHz, 88.2kHz, 96kHz, 176.4kHz, 192kHz
//! - BCLK = sample_rate * 64
//!
//! ## Pin Assignments (from PINOUT.md)
//! - AMP (TAS5830): BCLK=GPIO19, WCLK=GPIO20, DATA=GPIO21
//!
//! ## PIO State Machine Usage (PIO1)
//! - SM0: I2S TX to AMP (generates BCLK/WCLK, outputs DATA)

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

/// Clock speed adjustment for adaptive rate matching
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSpeed {
    /// Slow down output (buffer low)
    Slow,
    /// Normal speed
    Normal,
    /// Speed up output (buffer high)
    Fast,
}

/// Calculate PIO clock divider for a given sample rate
///
/// Returns the divider as a fixed-point value for use in PIO config.
/// PIO runs at 2x BCLK (2 instructions per bit, no delays).
pub fn calculate_clock_divider(sample_rate: u32) -> U24F8 {
    // BCLK = sample_rate * 64 (32 bits * 2 channels)
    // PIO runs at 2x BCLK (2 instructions per bit)
    let bclk_freq = sample_rate as u64 * 64;
    let pio_freq = bclk_freq * 2;
    let sys_freq = clocks::clk_sys_freq() as u64;
    let div = sys_freq as f64 / pio_freq as f64;
    div.to_fixed()
}

/// PIO1 base address on RP2350
const PIO1_BASE: u32 = 0x5030_0000;

/// SM0 clock divider register offset (SM0_CLKDIV)
const SM0_CLKDIV_OFFSET: u32 = 0x0C8;

/// I2S Transmitter (to TAS5830 amplifier) with clock recovery support
///
/// Based on pico_audio_i2s_32b PIO program.
/// Generates BCLK and WCLK, outputs DATA. Uses a single state machine.
/// Supports dynamic sample rate changes and adaptive clock adjustment.
/// Pin assignments: BCLK=GPIO19, WCLK=GPIO20, DATA=GPIO21
pub struct I2sTx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    /// Current sample rate
    sample_rate: u32,
    /// Clock divider for normal speed (register format: int[31:16], frac[15:8])
    clkdiv_normal: u32,
    /// Clock divider for fast speed (divider - 1 in frac bits)
    clkdiv_fast: u32,
    /// Clock divider for slow speed (divider + 1 in frac bits)
    clkdiv_slow: u32,
    /// Current clock speed setting
    current_speed: ClockSpeed,
    /// PIO program origin for re-initialization
    program_origin: u8,
}

impl<'d, PIO: Instance, const SM: usize> I2sTx<'d, PIO, SM> {
    /// Create I2S transmitter
    ///
    /// Does NOT start the state machine - call `start()` after creation.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        bclk_pin: Peri<'d, impl PioPin>,
        wclk_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        sample_rate: u32,
    ) -> Self {
        let clock_divider = calculate_clock_divider(sample_rate);
        // PIO program for I2S output (from pico_audio_i2s_32b by Elehobica)
        // Side-set: bit 0 = BCLK, bit 1 = LRCLK
        // ISR holds (res_bits - 2) = 30 for 32-bit audio
        // Autopull with 32-bit threshold, shift left (MSB first)
        // NO delays - each instruction is 1 cycle, 2 cycles per bit
        #[rustfmt::skip]
        let prg = pio::pio_asm!(
            ".side_set 2",
            "                         ;        /--- LRCLK",
            "                         ;        |/-- BCLK",
            ".wrap_target              ;        ||",
            "bitloop1:",
            "    out pins, 1       side 0b00",
            "    jmp x-- bitloop1  side 0b01",
            "    out pins, 1       side 0b10",
            "    mov x, isr        side 0b11",
            "",
            "bitloop0:",
            "    out pins, 1       side 0b10",
            "    jmp x-- bitloop0  side 0b11",
            "    out pins, 1       side 0b00",
            "public entry_point:",
            "    mov x, isr        side 0b01",
            ".wrap",
        );

        let bclk = common.make_pio_pin(bclk_pin);
        let wclk = common.make_pio_pin(wclk_pin);
        let data = common.make_pio_pin(data_pin);

        let mut cfg = Config::default();
        let loaded = common.load_program(&prg.program);
        let program_origin = loaded.origin;

        cfg.use_program(&loaded, &[&bclk, &wclk]); // Sideset = BCLK, WCLK
        cfg.set_out_pins(&[&data]); // OUT = DATA
        cfg.set_set_pins(&[&data]); // SET = DATA (for initialization)
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left, // MSB first
        };
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = clock_divider;

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&bclk, &wclk, &data]);

        // Initialize ISR with (bits_per_sample - 2) = 30 for 32-bit audio
        // This is the loop counter for the PIO program
        let res_bits: u32 = 30; // 32 - 2
        sm.tx().push(res_bits);
        unsafe {
            // pull noblock: 0x8080
            sm.exec_instr(0x8080);
            // out isr, 32: 0x60c0 (move OSR to ISR)
            sm.exec_instr(0x60c0);
        }

        // Jump to entry_point to initialize X from ISR
        let entry_point_offset = 7; // "public entry_point:" is at offset 7
        let entry_point_addr = program_origin + entry_point_offset;
        unsafe {
            sm.exec_instr(entry_point_addr as u16);
        }

        // Calculate clock dividers for adaptive adjustment
        // Register format: int[31:16], frac[15:8], reserved[7:0]
        let clkdiv_normal = clock_divider.to_bits() << 8;
        let clkdiv_fast = clkdiv_normal.saturating_sub(0x100); // -1 in frac bits
        let clkdiv_slow = clkdiv_normal.saturating_add(0x100); // +1 in frac bits

        defmt::info!("I2S TX configured: {} Hz", sample_rate);

        Self {
            sm,
            sample_rate,
            clkdiv_normal,
            clkdiv_fast,
            clkdiv_slow,
            current_speed: ClockSpeed::Normal,
            program_origin,
        }
    }

    /// Start the transmitter
    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }

    /// Stop the transmitter
    pub fn stop(&mut self) {
        self.sm.set_enable(false);
    }

    /// Reconfigure for a new sample rate
    ///
    /// Must call stop() before and start() after.
    /// Recalculates clock dividers for the new rate.
    /// Returns true if rate was changed, false if already at target rate.
    pub fn set_sample_rate(&mut self, sample_rate: u32) -> bool {
        if self.sample_rate == sample_rate {
            defmt::info!("I2S TX already at {} Hz, skipping reconfigure", sample_rate);
            return false;
        }

        self.sample_rate = sample_rate;

        // Calculate new clock divider
        let div_fixed = calculate_clock_divider(sample_rate);

        // Update stored dividers for adaptive adjustment
        self.clkdiv_normal = div_fixed.to_bits() << 8;
        self.clkdiv_fast = self.clkdiv_normal.saturating_sub(0x100);
        self.clkdiv_slow = self.clkdiv_normal.saturating_add(0x100);
        self.current_speed = ClockSpeed::Normal;

        // Apply clock divider directly to register (same method as adjust_clock)
        self.write_clkdiv(self.clkdiv_normal);

        defmt::debug!("I2S clkdiv: 0x{:08x}", self.clkdiv_normal);

        defmt::info!("I2S TX reconfigured: {} Hz", sample_rate);
        true
    }

    /// Get current sample rate
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Adjust clock speed for adaptive rate matching
    ///
    /// Used for clock recovery: adjusts the I2S output clock by ±1 divider step
    /// to keep the buffer level stable.
    /// - Slow: Output slower (buffer low, give S/PDIF time to fill)
    /// - Normal: Nominal rate
    /// - Fast: Output faster (buffer high, catch up to S/PDIF)
    pub fn adjust_clock(&mut self, speed: ClockSpeed) {
        if speed == self.current_speed {
            return; // No change needed
        }

        let clkdiv = match speed {
            ClockSpeed::Slow => self.clkdiv_slow,
            ClockSpeed::Normal => self.clkdiv_normal,
            ClockSpeed::Fast => self.clkdiv_fast,
        };

        self.write_clkdiv(clkdiv);
        self.current_speed = speed;
    }

    /// Write clock divider directly to PIO SM0 register
    ///
    /// This bypasses embassy-rp for runtime adjustment while SM is running.
    /// Register format: int[31:16], frac[15:8], reserved[7:0]
    fn write_clkdiv(&self, clkdiv: u32) {
        // PIO1 SM0_CLKDIV register address
        let addr = (PIO1_BASE + SM0_CLKDIV_OFFSET) as *mut u32;
        unsafe {
            core::ptr::write_volatile(addr, clkdiv);
        }
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
/// Timing matches I2sTx: 2 sideset bits (BCLK + WCLK), 2 PIO cycles per bit, no delays.
/// Both TX and RX use the same `calculate_clock_divider()` function.
/// Pin assignments: BCLK=GPIO23, WCLK=GPIO24, DATA=GPIO25
pub struct I2sRx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> I2sRx<'d, PIO, SM> {
    /// Create I2S receiver
    ///
    /// Does NOT start the state machine - call `start()` after creation.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        bclk_pin: Peri<'d, impl PioPin>,
        wclk_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        sample_rate: u32,
    ) -> Self {
        let clock_divider = calculate_clock_divider(sample_rate);

        // PIO program for I2S receive with clock generation
        // Matches TX timing: 2 sideset bits, 2 PIO cycles per bit, no delays
        // Side-set: bit 0 = BCLK, bit 1 = WCLK
        // Data sampled on BCLK rising edge
        // WCLK transitions on BCLK falling edge (I2S standard)
        // Uses set x, 30 for loop counter (ISR is used for input data)
        //
        // Per I2S spec: WCLK transitions on BCLK falling, MSB valid 1 BCLK later.
        // The set x instruction (BCLK low) IS the "don't care" BCLK half-cycle.
        // First in pins after transition samples at BCLK rising = MSB position.
        #[rustfmt::skip]
        let prg = pio::pio_asm!(
            ".side_set 2",
            "                              ;        /--- WCLK",
            "                              ;        |/-- BCLK",
            ".wrap_target                  ;        ||",
            "bitloop_left:",
            "    in pins, 1       side 0b01",  // Sample data, BCLK high, WCLK=0
            "    jmp x-- bitloop_left side 0b00", // BCLK low, WCLK=0
            "    in pins, 1       side 0b01",  // Sample last left bit, BCLK high, WCLK still 0
            "    set x, 30        side 0b10",  // Reload counter, BCLK low, WCLK→1 (transition!)
            "",
            "bitloop_right:",
            "    in pins, 1       side 0b11",  // Sample data, BCLK high, WCLK=1
            "    jmp x-- bitloop_right side 0b10", // BCLK low, WCLK=1
            "    in pins, 1       side 0b11",  // Sample last right bit, BCLK high, WCLK still 1
            "public entry_point:",
            "    set x, 30        side 0b00",  // Reload counter, BCLK low, WCLK→0 (transition!)
            ".wrap",
        );

        let bclk = common.make_pio_pin(bclk_pin);
        let wclk = common.make_pio_pin(wclk_pin);
        let data = common.make_pio_pin(data_pin);

        let mut cfg = Config::default();
        let loaded = common.load_program(&prg.program);
        cfg.use_program(&loaded, &[&bclk, &wclk]); // Sideset = BCLK, WCLK
        cfg.set_in_pins(&[&data]); // IN = DATA
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

        // Jump to entry_point to initialize X
        let entry_point_offset = 7; // "public entry_point:" is at offset 7
        let entry_point_addr = loaded.origin + entry_point_offset;
        unsafe {
            sm.exec_instr(entry_point_addr as u16);
        }

        defmt::info!("I2S RX configured: {} Hz", sample_rate);

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
    ///
    /// Returns None if no data available. Once the left sample is consumed,
    /// busy-waits for the right sample to maintain channel sync (right follows
    /// left within one BCLK period).
    pub fn try_read(&mut self) -> Option<(u32, u32)> {
        let left = self.sm.rx().try_pull()?;
        // Left consumed - must get right to maintain sync
        let right = loop {
            if let Some(v) = self.sm.rx().try_pull() {
                break v;
            }
        };
        Some((left, right))
    }

    /// Check if RX FIFO has data
    pub fn available(&mut self) -> bool {
        !self.sm.rx().empty()
    }
}
