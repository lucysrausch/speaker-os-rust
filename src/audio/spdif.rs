//! S/PDIF Receiver module for RP2350
//!
//! Based on pico_spdif_rx by Elehobica (BSD-2-Clause)
//! Ported to Rust with embassy-rp
//!
//! This module handles S/PDIF digital audio input with:
//! - Automatic sample rate detection (44.1kHz - 192kHz)
//! - Runtime PIO program switching for different rates
//! - DMA-based buffering for low CPU overhead
//! - Signal quality monitoring via sync code validation

use defmt::*;
use embassy_rp::dma::Channel;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, Pin, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::Peri;
use embassy_time::Duration;
use fixed::traits::ToFixed;

/// PIO register offsets (from RP2350 datasheet)
const PIO_INSTR_MEM_OFFSET: usize = 0x048;

/// S/PDIF block size (sub-frames per block)
pub const SPDIF_BLOCK_SIZE: usize = 384;

/// Number of blocks in FIFO (2 blocks = ~4.4ms @ 44.1kHz)
const NUM_BLOCKS: usize = 2;

/// Total FIFO size
pub const SPDIF_RX_FIFO_SIZE: usize = NUM_BLOCKS * SPDIF_BLOCK_SIZE;

/// DMA block size for transfers
pub const DMA_BLOCK_SIZE: usize = 64;

/// System clock frequency for RP2350
const SYS_CLK_FREQ: u32 = 150_000_000;

/// PIO clock frequency
const PIO_CLK_FREQ: u32 = 128_000_000;

/// Sync codes from PIO output
const SYNC_B: u32 = 0b1111;
const SYNC_M: u32 = 0b1011;
const SYNC_W: u32 = 0b0111;

/// Supported sample frequencies
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SampleFreq {
    None,
    Hz44100,
    Hz48000,
    Hz88200,
    Hz96000,
    Hz176400,
    Hz192000,
}

impl SampleFreq {
    pub fn as_hz(self) -> u32 {
        match self {
            SampleFreq::None => 0,
            SampleFreq::Hz44100 => 44100,
            SampleFreq::Hz48000 => 48000,
            SampleFreq::Hz88200 => 88200,
            SampleFreq::Hz96000 => 96000,
            SampleFreq::Hz176400 => 176400,
            SampleFreq::Hz192000 => 192000,
        }
    }

    #[allow(dead_code)]
    fn from_actual(freq: f32) -> Self {
        const TOLERANCE: f32 = 0.01; // 1%

        let check = |target: u32| -> bool {
            let lower = target as f32 * (1.0 - TOLERANCE);
            let upper = target as f32 * (1.0 + TOLERANCE);
            freq >= lower && freq <= upper
        };

        if check(44100) {
            SampleFreq::Hz44100
        } else if check(48000) {
            SampleFreq::Hz48000
        } else if check(88200) {
            SampleFreq::Hz88200
        } else if check(96000) {
            SampleFreq::Hz96000
        } else if check(176400) {
            SampleFreq::Hz176400
        } else if check(192000) {
            SampleFreq::Hz192000
        } else {
            SampleFreq::None
        }
    }
}

/// S/PDIF receiver state
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum State {
    NoSignal,
    WaitingStable,
    Stable,
}

/// Sample frequency info for detection
struct SampFreqInfo {
    freq: SampleFreq,
    min_edge_lower: u32,
    min_edge_upper: u32,
    max_edge_lower: u32,
    max_edge_upper: u32,
}

impl SampFreqInfo {
    const fn new(freq: SampleFreq, sf: u32) -> Self {
        // min edge = 2 symbols, max edge = 6 symbols
        let min_edge_nominal = PIO_CLK_FREQ / sf * 2 / 128;
        let max_edge_nominal = PIO_CLK_FREQ / sf * 6 / 128;

        Self {
            freq,
            min_edge_lower: min_edge_nominal * 91 / 100,
            min_edge_upper: min_edge_nominal * 111 / 100,
            max_edge_lower: max_edge_nominal * 97 / 100,
            max_edge_upper: max_edge_nominal * 104 / 100,
        }
    }

    fn check_min_edge(&self, value: u32) -> bool {
        value >= self.min_edge_lower && value <= self.min_edge_upper
    }

    fn check_max_edge(&self, value: u32) -> bool {
        value >= self.max_edge_lower && value <= self.max_edge_upper
    }
}

const SF_INFO: [SampFreqInfo; 6] = [
    SampFreqInfo::new(SampleFreq::Hz44100, 44100),
    SampFreqInfo::new(SampleFreq::Hz48000, 48000),
    SampFreqInfo::new(SampleFreq::Hz88200, 88200),
    SampFreqInfo::new(SampleFreq::Hz96000, 96000),
    SampFreqInfo::new(SampleFreq::Hz176400, 176400),
    SampFreqInfo::new(SampleFreq::Hz192000, 192000),
];

/// PIO program for S/PDIF capture (frequency detection)
mod pio_capture {
    pub fn program() -> pio::Program<32> {
        use pio::pio_asm;
        let program = pio_asm!(
            "entry_point:",
            "    set x, 31",
            "loop_wait_toggle:",
            "    wait 0 pin 0",
            "    wait 1 pin 0",
            "    jmp x-- loop_wait_toggle",
            "    wait 0 pin 0",
            "    wait 1 pin 0",
            ".wrap_target",
            "    in pins, 1",
            ".wrap",
        );
        program.program
    }
}

/// PIO program for S/PDIF decode at 44.1/48 kHz
mod pio_decode_48000 {
    // cy = 20 cycles per symbol for 44.1/48 kHz
    // lp = cy/2 = 10 (latch point)
    pub fn program() -> pio::Program<32> {
        use pio::pio_asm;
        let program = pio_asm!(
            ".define cy 20",
            ".define lp 10",
            "entry_point:",
            "    wait 0 pin 0",
            ".wrap_target",
            "wait1:",
            "    wait 1 pin 0 [cy-1+lp]", // wait for 0->1
            "symbol_1x:",
            "    jmp pin symbol_11x", // if 11 -> symbol_11x
            "    in osr, 1",          // emit 1 (symbol 10)
            "    jmp wait1",
            "symbol_11x:",
            "    nop [cy-2]",
            "    jmp pin sync1110", // if 111 -> sync
            "    in null, 1",       // emit 0 (symbol 110)
            "    jmp symbol_0x [cy-3]",
            "wait0:",
            "    wait 0 pin 0 [cy-1+lp]", // wait for 1->0
            "symbol_0x:",
            "    jmp pin symbol_01", // if 01 -> symbol_01
            "    in null, 1",        // emit 0 (symbol 00)
            ".wrap",
            "symbol_01:",
            "    in osr, 1", // emit 1
            "    jmp wait0",
            "sync1110:",
            "    push block", // 32-bit boundary
            "    in osr, 2",  // emit sync (1110)
            "    wait 0 pin 0 [cy-1+lp]",
            "    jmp pin sync1xxx [cy-2]",
            "    jmp symbol_0x",
            "sync1xxx:",
            "    in osr, 2", // emit sync (1000)
            "    jmp entry_point",
        );
        program.program
    }
}

/// PIO program for S/PDIF decode at 88.2/96 kHz
mod pio_decode_96000 {
    // cy = 10 cycles per symbol for 88.2/96 kHz
    // lp = cy/2 = 5 (latch point)
    pub fn program() -> pio::Program<32> {
        use pio::pio_asm;
        let program = pio_asm!(
            ".define cy 10",
            ".define lp 5",
            "entry_point:",
            "    wait 0 pin 0",
            ".wrap_target",
            "wait1:",
            "    wait 1 pin 0 [cy-1+lp]",
            "symbol_1x:",
            "    jmp pin symbol_11x",
            "    in osr, 1",
            "    jmp wait1",
            "symbol_11x:",
            "    nop [cy-2]",
            "    jmp pin sync1110",
            "    in null, 1",
            "    jmp symbol_0x [cy-3]",
            "wait0:",
            "    wait 0 pin 0 [cy-1+lp]",
            "symbol_0x:",
            "    jmp pin symbol_01",
            "    in null, 1",
            ".wrap",
            "symbol_01:",
            "    in osr, 1",
            "    jmp wait0",
            "sync1110:",
            "    push block",
            "    in osr, 2",
            "    wait 0 pin 0 [cy-1+lp]",
            "    jmp pin sync1xxx [cy-2]",
            "    jmp symbol_0x",
            "sync1xxx:",
            "    in osr, 2",
            "    jmp entry_point",
        );
        program.program
    }
}

/// PIO program for S/PDIF decode at 176.4/192 kHz
mod pio_decode_192000 {
    // cy = 5 cycles per symbol for 176.4/192 kHz
    // lp = cy/2 = 2 (latch point)
    pub fn program() -> pio::Program<32> {
        use pio::pio_asm;
        let program = pio_asm!(
            ".define cy 5",
            ".define lp 2",
            "entry_point:",
            "    wait 0 pin 0",
            ".wrap_target",
            "wait1:",
            "    wait 1 pin 0 [cy-1+lp]",
            "symbol_1x:",
            "    jmp pin symbol_11x",
            "    in osr, 1",
            "    jmp wait1",
            "symbol_11x:",
            "    nop [cy-2]",
            "    jmp pin sync1110",
            "    in null, 1",
            "    jmp symbol_0x [cy-3]",
            "wait0:",
            "    wait 0 pin 0 [cy-1+lp]",
            "symbol_0x:",
            "    jmp pin symbol_01",
            "    in null, 1",
            ".wrap",
            "symbol_01:",
            "    in osr, 1",
            "    jmp wait0",
            "sync1110:",
            "    push block",
            "    in osr, 2",
            "    wait 0 pin 0 [cy-1+lp]",
            "    jmp pin sync1xxx [cy-2]",
            "    jmp symbol_0x",
            "sync1xxx:",
            "    in osr, 2",
            "    jmp entry_point",
        );
        program.program
    }
}

/// Captured data buffer for frequency detection
const CAPTURE_SIZE: usize = 128;

/// Number of consecutive sync losses before triggering re-detection
const SYNC_LOST_THRESHOLD: u32 = 10; // ~100ms with 10ms DMA timeout

/// Write an instruction directly to PIO instruction memory
/// SAFETY: Caller must ensure the PIO state machine is disabled
#[inline]
unsafe fn write_pio_instr(pio_num: u8, addr: u8, instr: u16) {
    // PIO0 base: 0x50200000, PIO1 base: 0x50300000
    // INSTR_MEM offset: 0x048
    let pio_base = if pio_num == 0 {
        0x5020_0000u32
    } else {
        0x5030_0000u32
    };
    let instr_mem_addr = (pio_base + PIO_INSTR_MEM_OFFSET as u32) as *mut u32;
    // SAFETY: addr is within PIO instruction memory bounds (0-31)
    // and the caller ensures the state machine is disabled
    unsafe {
        core::ptr::write_volatile(instr_mem_addr.add(addr as usize), instr as u32);
    }
}

/// Get the decode program for the specified variant
/// Returns the pio::Program struct
fn get_decode_program(variant: SampleFreq) -> pio::Program<32> {
    match variant {
        SampleFreq::Hz44100 | SampleFreq::Hz48000 => pio_decode_48000::program(),
        SampleFreq::Hz88200 | SampleFreq::Hz96000 => pio_decode_96000::program(),
        SampleFreq::Hz176400 | SampleFreq::Hz192000 => pio_decode_192000::program(),
        SampleFreq::None => pio_decode_48000::program(),
    }
}

/// S/PDIF Receiver
pub struct SpdifRx<'d, PIO: Instance, const SM: usize, DMA: Channel> {
    sm: StateMachine<'d, PIO, SM>,
    dma: Peri<'d, DMA>,
    pin: Pin<'d, PIO>,
    pio_no: u8,
    state: State,
    samp_freq: SampleFreq,
    #[allow(dead_code)]
    samp_freq_actual: f32,
    inverted: bool,
    fifo_buff: &'static mut [u32; SPDIF_RX_FIFO_SIZE],
    buff_wr_ptr: usize,
    buff_rd_ptr: usize,
    block_aligned: bool,
    stable_count: u32,
    sync_lost_count: u32,
    /// Capture program info (origin, wrap_top, wrap_bottom) if loaded
    capture_info: Option<(u8, u8, u8)>,
    /// Decode program info (origin, wrap_top, wrap_bottom, sample_freq_variant) if loaded
    decode_info: Option<(u8, u8, u8, SampleFreq)>,
}

impl<'d, PIO: Instance, const SM: usize, DMA: Channel> SpdifRx<'d, PIO, SM, DMA> {
    /// Create a new S/PDIF receiver
    ///
    /// # Arguments
    /// * `common` - PIO common resources
    /// * `sm` - PIO state machine to use
    /// * `dma` - DMA channel for async transfers
    /// * `pin` - GPIO pin connected to S/PDIF input
    /// * `fifo_buff` - Static buffer for S/PDIF FIFO (must be SPDIF_RX_FIFO_SIZE words)
    /// * `pio_no` - PIO instance number (0 for PIO0, 1 for PIO1)
    pub fn new<P: PioPin>(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: Peri<'d, DMA>,
        pin: Peri<'d, P>,
        fifo_buff: &'static mut [u32; SPDIF_RX_FIFO_SIZE],
        pio_no: u8,
    ) -> Self {
        let pio_pin = common.make_pio_pin(pin);
        sm.set_pin_dirs(Direction::In, &[&pio_pin]);

        Self {
            sm,
            dma,
            pin: pio_pin,
            pio_no,
            state: State::NoSignal,
            samp_freq: SampleFreq::None,
            samp_freq_actual: 0.0,
            inverted: false,
            fifo_buff,
            buff_wr_ptr: 0,
            buff_rd_ptr: 0,
            block_aligned: false,
            stable_count: 0,
            sync_lost_count: 0,
            capture_info: None,
            decode_info: None,
        }
    }

    /// Get current state
    pub fn state(&self) -> State {
        self.state
    }

    /// Get detected sample frequency
    pub fn sample_freq(&self) -> SampleFreq {
        self.samp_freq
    }

    /// Get actual sample frequency (measured)
    #[allow(dead_code)]
    pub fn sample_freq_actual(&self) -> f32 {
        self.samp_freq_actual
    }

    /// Get available sample count in FIFO
    pub fn fifo_count(&self) -> usize {
        if self.buff_wr_ptr >= self.buff_rd_ptr {
            self.buff_wr_ptr - self.buff_rd_ptr
        } else {
            self.buff_wr_ptr + SPDIF_RX_FIFO_SIZE - self.buff_rd_ptr
        }
    }

    /// Read samples from FIFO
    /// Returns number of samples actually read
    pub fn read_fifo(&mut self, dest: &mut [u32]) -> usize {
        let available = self.fifo_count();
        let count = dest.len().min(available);

        for i in 0..count {
            dest[i] = self.fifo_buff[self.buff_rd_ptr];
            self.buff_rd_ptr = (self.buff_rd_ptr + 1) % SPDIF_RX_FIFO_SIZE;
        }

        count
    }

    /// Detect S/PDIF signal and determine sample frequency
    pub async fn detect_signal(&mut self, common: &mut Common<'d, PIO>) -> bool {
        trace!("detect_signal: starting capture");

        let mut cfg = Config::default();

        // Load capture program only if not already loaded
        let origin = if let Some((origin, wrap_top, wrap_bottom)) = self.capture_info {
            trace!(
                "detect_signal: reusing capture at origin {}, decode_info={}",
                origin,
                self.decode_info.is_some()
            );
            // Manually configure wrap points
            let mut exec = cfg.get_exec();
            exec.wrap_top = wrap_top;
            exec.wrap_bottom = wrap_bottom;
            // SAFETY: We're setting valid wrap points for the already-loaded program
            unsafe { cfg.set_exec(exec) };
            origin
        } else {
            trace!("detect_signal: loading capture program");
            let prg = pio_capture::program();
            let installed = common.load_program(&prg);
            let origin = installed.origin;
            trace!(
                "detect_signal: capture loaded at origin {}, size {}",
                origin,
                prg.code.len()
            );
            let wrap_top = origin + installed.wrap.source;
            let wrap_bottom = origin + installed.wrap.target;
            self.capture_info = Some((origin, wrap_top, wrap_bottom));
            cfg.use_program(&installed, &[]);
            origin
        };

        // Clock divider for capture
        let div = (SYS_CLK_FREQ as f32) / (PIO_CLK_FREQ as f32);
        cfg.clock_divider = div.to_fixed();

        // Configure pin
        cfg.set_jmp_pin(&self.pin);
        cfg.set_in_pins(&[&self.pin]);
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Left,
        };
        cfg.fifo_join = FifoJoin::RxOnly;

        self.sm.set_config(&cfg);

        // Jump to program origin (needed when reusing program since cfg.origin is private)
        unsafe {
            self.sm.exec_instr(origin as u16); // JMP to origin
        }

        self.sm.set_enable(true);

        // Capture samples with timeout
        trace!("detect_signal: starting DMA capture");
        let mut capture_buf = [0u32; CAPTURE_SIZE];
        let capture_result = embassy_time::with_timeout(
            Duration::from_millis(500),
            self.sm
                .rx()
                .dma_pull(self.dma.reborrow(), &mut capture_buf, false),
        )
        .await;
        trace!("detect_signal: DMA done, result={}", capture_result.is_ok());

        self.sm.set_enable(false);

        if capture_result.is_err() {
            trace!("S/PDIF capture timeout - no signal");
            return false;
        }

        // Analyze captured data
        if let Some((freq, inverted)) = self.analyze_capture(&capture_buf) {
            self.samp_freq = freq;
            self.inverted = inverted;
            info!(
                "S/PDIF detected: {} Hz, inverted={}",
                freq.as_hz(),
                inverted
            );
            true
        } else {
            debug!("S/PDIF signal not recognized");
            false
        }
    }

    /// Analyze captured data to determine frequency and polarity
    fn analyze_capture(&self, data: &[u32]) -> Option<(SampleFreq, bool)> {
        let mut edge_pos = [0i32; 2];
        let mut max_edge_interval = [0u32; 2];
        let mut min_edge_interval = u32::MAX;

        let mut pos = 0i32;
        let mut cur = 1usize; // Start with rising edge
        let mut word_idx = 0;
        let mut bit_pos = 0;
        let mut shift_reg = data[0];

        while word_idx < data.len() {
            // Count leading zeros/ones
            let r = if cur == 1 {
                (!shift_reg).leading_zeros()
            } else {
                shift_reg.leading_zeros()
            } as i32;

            if r + bit_pos <= 31 {
                pos += r;
                cur = 1 - cur;

                if edge_pos[cur] > 0 {
                    let distance = (pos - edge_pos[cur]) as u32;
                    max_edge_interval[cur] = max_edge_interval[cur].max(distance);
                    min_edge_interval = min_edge_interval.min(distance);

                    // Early termination check
                    if min_edge_interval < SF_INFO[5].min_edge_lower {
                        return None;
                    }
                }
                edge_pos[cur] = pos;
                shift_reg <<= r;
                bit_pos += r;
            } else {
                pos += 32 - bit_pos;
                bit_pos = 0;
                word_idx += 1;
                if word_idx < data.len() {
                    shift_reg = data[word_idx];
                }
            }
        }

        // Check against known frequencies
        for info in &SF_INFO {
            if info.check_min_edge(min_edge_interval) {
                // Check polarity based on max edge interval
                if info.check_max_edge(max_edge_interval[1]) {
                    return Some((info.freq, false));
                } else if info.check_max_edge(max_edge_interval[0]) {
                    return Some((info.freq, true));
                }
            }
        }

        None
    }

    /// Start decoding with the detected sample frequency
    pub fn start_decode(&mut self, common: &mut Common<'d, PIO>) {
        // Determine which decode program variant we need (48k, 96k, or 192k)
        let decode_variant = match self.samp_freq {
            SampleFreq::Hz44100 | SampleFreq::Hz48000 => SampleFreq::Hz48000,
            SampleFreq::Hz88200 | SampleFreq::Hz96000 => SampleFreq::Hz96000,
            SampleFreq::Hz176400 | SampleFreq::Hz192000 => SampleFreq::Hz192000,
            SampleFreq::None => return,
        };

        let mut cfg = Config::default();

        // Check if we can reuse the already-loaded decode program
        let origin = if let Some((origin, _wrap_top, _wrap_bottom, loaded_variant)) =
            self.decode_info
        {
            if loaded_variant != decode_variant {
                // Different variant needed - patch the program in place
                trace!(
                    "start_decode: patching decode program from {} to {} at origin {}",
                    loaded_variant.as_hz(),
                    decode_variant.as_hz(),
                    origin
                );

                // Get the new program's instructions
                let prg = get_decode_program(decode_variant);
                let wrap_source = prg.wrap.source;
                let wrap_target = prg.wrap.target;

                // Write all instructions to PIO memory, relocating jump targets
                // SAFETY: SM is disabled (set_enable(false) was called in reset())
                for (i, &instr) in prg.code.iter().enumerate() {
                    // Relocate JMP instructions (opcode 000 in bits 15-13)
                    let relocated = if (instr >> 13) == 0 {
                        // JMP instruction - add origin to target address (bits 4:0)
                        let target = instr & 0x1f;
                        let rest = instr & !0x1f;
                        rest | ((target + origin as u16) & 0x1f)
                    } else {
                        instr
                    };
                    unsafe {
                        write_pio_instr(self.pio_no, origin + i as u8, relocated);
                    }
                }

                // Update stored variant and wrap points
                let new_wrap_top = origin + wrap_source;
                let new_wrap_bottom = origin + wrap_target;
                self.decode_info = Some((origin, new_wrap_top, new_wrap_bottom, decode_variant));

                // Configure wrap points for the new variant
                let mut exec = cfg.get_exec();
                exec.wrap_top = new_wrap_top;
                exec.wrap_bottom = new_wrap_bottom;
                unsafe { cfg.set_exec(exec) };

                trace!(
                    "start_decode: decode program patched successfully (wrap {}-{})",
                    new_wrap_bottom,
                    new_wrap_top
                );
            } else {
                // Same variant - still rewrite program to PIO memory
                // The capture program may have corrupted PIO state
                trace!(
                    "start_decode: rewriting decode program at origin {}",
                    origin
                );
                let prg = get_decode_program(decode_variant);
                let wrap_source = prg.wrap.source;
                let wrap_target = prg.wrap.target;

                // Write all instructions to PIO memory, relocating jump targets
                for (i, &instr) in prg.code.iter().enumerate() {
                    let relocated = if (instr >> 13) == 0 {
                        let target = instr & 0x1f;
                        let rest = instr & !0x1f;
                        rest | ((target + origin as u16) & 0x1f)
                    } else {
                        instr
                    };
                    unsafe {
                        write_pio_instr(self.pio_no, origin + i as u8, relocated);
                    }
                }

                // Calculate fresh wrap points from the program (matching patching path)
                let new_wrap_top = origin + wrap_source;
                let new_wrap_bottom = origin + wrap_target;
                self.decode_info = Some((origin, new_wrap_top, new_wrap_bottom, decode_variant));

                // Configure wrap points
                let mut exec = cfg.get_exec();
                exec.wrap_top = new_wrap_top;
                exec.wrap_bottom = new_wrap_bottom;
                unsafe { cfg.set_exec(exec) };
            }
            origin
        } else {
            // First time loading decode program
            trace!(
                "start_decode: loading decode program for {} Hz",
                decode_variant.as_hz()
            );
            let prg = match decode_variant {
                SampleFreq::Hz48000 => pio_decode_48000::program(),
                SampleFreq::Hz96000 => pio_decode_96000::program(),
                SampleFreq::Hz192000 => pio_decode_192000::program(),
                _ => core::unreachable!(),
            };
            let installed = common.load_program(&prg);
            let origin = installed.origin;
            trace!(
                "start_decode: decode loaded at origin {}, size {}, capture_info={:?}",
                origin,
                prg.code.len(),
                self.capture_info
            );
            let wrap_top = origin + installed.wrap.source;
            let wrap_bottom = origin + installed.wrap.target;
            self.decode_info = Some((origin, wrap_top, wrap_bottom, decode_variant));
            cfg.use_program(&installed, &[]);
            origin
        };

        // Clock divider
        let div = (SYS_CLK_FREQ as f32) / (PIO_CLK_FREQ as f32);
        cfg.clock_divider = div.to_fixed();

        // Configure pin
        cfg.set_jmp_pin(&self.pin);
        cfg.set_in_pins(&[&self.pin]);
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        cfg.fifo_join = FifoJoin::RxOnly;

        self.sm.set_config(&cfg);

        // Clear RX FIFO - may have stale data from capture phase
        while !self.sm.rx().empty() {
            let _ = self.sm.rx().pull();
        }

        // Restart state machine to clear internal state (shift registers, etc.)
        self.sm.restart();

        // Set OSR to 0xFFFFFFFF (used for emitting 1s)
        // Execute: set x, 0; mov osr, !x
        // set x, 0: opcode=111, dest=x(001), data=0 -> 0xe020
        // mov osr, !x: opcode=101, dest=osr(111), op=invert(01), src=x(001) -> 0xa0e9
        unsafe {
            self.sm.exec_instr(0xe020); // set x, 0
            self.sm.exec_instr(0xa0e9); // mov osr, !x
                                        // Jump to program origin
            self.sm.exec_instr(0x0000 | origin as u16); // JMP origin
        }

        // Handle inverted signal
        if self.inverted {
            // Note: For inverted signals, GPIO override would need direct register access
            debug!("Note: inverted signal handling may need GPIO override");
        }

        self.sm.set_enable(true);
        self.state = State::WaitingStable;
        self.buff_wr_ptr = 0;
        self.buff_rd_ptr = 0;
        self.block_aligned = false;
        self.stable_count = 0;
        self.sync_lost_count = 0;

        trace!("S/PDIF decode started");
    }

    /// Process incoming data from PIO FIFO (polling mode)
    /// Should be called frequently to drain PIO RX FIFO into software buffer
    #[allow(dead_code)]
    pub async fn process(&mut self) -> bool {
        let mut count = 0;

        // Read all available samples from PIO RX FIFO (no async delay)
        // The PIO FIFO is only 8 words deep, so we need to drain it quickly
        while !self.sm.rx().empty() {
            if let Some(word) = self.sm.rx().try_pull() {
                self.fifo_buff[self.buff_wr_ptr] = word;
                self.buff_wr_ptr = (self.buff_wr_ptr + 1) % SPDIF_RX_FIFO_SIZE;
                count += 1;

                // Limit per call to avoid starving I2S output
                if count >= 64 {
                    break;
                }
            }
        }

        if count == 0 {
            return false;
        }

        // Check for sync codes to verify signal quality
        let mut sync_found = false;
        let start = if self.buff_wr_ptr >= count {
            self.buff_wr_ptr - count
        } else {
            SPDIF_RX_FIFO_SIZE + self.buff_wr_ptr - count
        };

        for i in 0..count {
            let idx = (start + i) % SPDIF_RX_FIFO_SIZE;
            let sync = self.fifo_buff[idx] & 0xf;
            if sync == SYNC_B || sync == SYNC_M || sync == SYNC_W {
                sync_found = true;
                break;
            }
        }

        if sync_found {
            self.stable_count += 1;
            if self.stable_count >= 16 && self.state == State::WaitingStable {
                self.state = State::Stable;
                info!("S/PDIF signal stable");
            }
        } else {
            self.stable_count = 0;
            if self.state == State::Stable {
                self.state = State::WaitingStable;
                warn!("S/PDIF sync lost");
            }
        }

        true
    }

    /// Stop the receiver
    #[allow(dead_code)]
    pub fn stop(&mut self) {
        self.sm.set_enable(false);
        self.state = State::NoSignal;
    }

    /// Reset to search for new signal
    pub fn reset(&mut self) {
        self.sm.set_enable(false);
        self.state = State::NoSignal;
        self.samp_freq = SampleFreq::None;
        self.buff_wr_ptr = 0;
        self.buff_rd_ptr = 0;
        self.block_aligned = false;
        self.stable_count = 0;
        self.sync_lost_count = 0;
    }

    /// Process a block using DMA - more efficient than polling
    /// Returns number of samples read, or 0 if no space/error/timeout
    pub async fn process_dma(&mut self, block_size: usize) -> usize {
        // Calculate contiguous space available
        let space_to_end = SPDIF_RX_FIFO_SIZE - self.buff_wr_ptr;
        let space_before_rd = if self.buff_wr_ptr >= self.buff_rd_ptr {
            SPDIF_RX_FIFO_SIZE - (self.buff_wr_ptr - self.buff_rd_ptr) - 1
        } else {
            self.buff_rd_ptr - self.buff_wr_ptr - 1
        };

        // Limit DMA size to contiguous space
        let dma_size = space_to_end.min(space_before_rd).min(block_size);
        if dma_size < 8 {
            // Buffer is full but we can't write - if we're not stable, this means
            // we're stuck (not reading because muted, can't write because full)
            // Trigger re-detection
            if self.state != State::Stable {
                self.sync_lost_count += 1;
                if self.sync_lost_count >= SYNC_LOST_THRESHOLD {
                    warn!("S/PDIF buffer stuck, triggering re-detection");
                    self.state = State::NoSignal;
                }
            }
            return 0;
        }

        // DMA read from PIO RX FIFO with timeout
        // Timeout allows detection of signal loss when PIO stops producing data
        let dest = &mut self.fifo_buff[self.buff_wr_ptr..self.buff_wr_ptr + dma_size];
        let dma_result = embassy_time::with_timeout(
            Duration::from_millis(10),
            self.sm.rx().dma_pull(self.dma.reborrow(), dest, false),
        )
        .await;

        if dma_result.is_err() {
            // DMA timeout - treat as sync loss
            self.sync_lost_count += 1;
            self.stable_count = 0;
            if self.state == State::Stable {
                self.state = State::WaitingStable;
                warn!("S/PDIF sync lost (DMA timeout)");
            }
            if self.sync_lost_count >= SYNC_LOST_THRESHOLD {
                warn!("S/PDIF sync lost for too long, triggering re-detection");
                self.state = State::NoSignal;
            }
            return 0;
        }

        let start = self.buff_wr_ptr;
        self.buff_wr_ptr = (self.buff_wr_ptr + dma_size) % SPDIF_RX_FIFO_SIZE;

        // Check for sync codes - require majority of words to have valid syncs
        // (random data will occasionally match sync patterns by chance)
        let mut sync_count = 0u32;
        for i in 0..dma_size {
            let sync = self.fifo_buff[(start + i) % SPDIF_RX_FIFO_SIZE] & 0xf;
            if sync == SYNC_B || sync == SYNC_M || sync == SYNC_W {
                sync_count += 1;
            }
        }
        // In valid S/PDIF, every word has a sync code. Require at least 75% valid.
        let sync_threshold = dma_size as u32 * 3 / 4;
        let signal_ok = sync_count >= sync_threshold;

        if signal_ok {
            self.stable_count += 1;
            self.sync_lost_count = 0;
            if self.stable_count >= 16 && self.state == State::WaitingStable {
                self.state = State::Stable;
                info!("S/PDIF signal stable");
            }
        } else {
            self.stable_count = 0;
            self.sync_lost_count += 1;
            if self.state == State::Stable {
                self.state = State::WaitingStable;
                warn!(
                    "S/PDIF sync lost (bad sync codes: {}/{})",
                    sync_count, dma_size
                );
            }
            // Check if we've lost sync for too long - trigger re-detection
            if self.sync_lost_count >= SYNC_LOST_THRESHOLD {
                warn!("S/PDIF sync lost for too long, triggering re-detection");
                self.state = State::NoSignal;
            }
        }

        dma_size
    }
}

/// S/PDIF input attenuation (0-100%).
///
/// Extract audio sample from S/PDIF word
/// Returns 32-bit signed audio (no attenuation)
#[inline]
pub fn extract_audio(word: u32) -> i32 {
    // S/PDIF format: [31:28] VUCP, [27:4] 24-bit audio, [3:0] Sync
    // Shift left 4 to align 24-bit audio to MSB of i32
    ((word & 0x0FFF_FFF0) << 4) as i32
}

/// Check if word is left channel (Sync B or Sync M)
#[inline]
#[allow(dead_code)]
pub fn is_left_channel(word: u32) -> bool {
    let sync = word & 0xf;
    sync == SYNC_B || sync == SYNC_M
}
