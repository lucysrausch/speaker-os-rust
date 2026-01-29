//! Audio Pipeline for OtterAmp DSP
//!
//! Block-based audio processing pipeline supporting multiple sources:
//! - S/PDIF digital input
//! - Line-in via PCM1822 ADC
//! - USB Audio Class
//!
//! Architecture:
//! ```text
//! [Sources] -> [Input Ring Buffer] -> [DSP Block Processing] -> [Output Ring Buffer] -> [DMA -> I2S TX]
//! ```
//!
//! The pipeline uses fixed-size blocks (BLOCK_SIZE samples) for efficient DSP processing.
//! Double-buffering ensures continuous output while DSP processes the next block.

use core::sync::atomic::{AtomicUsize, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Block size for DSP processing (samples per channel)
/// 128 samples @ 96kHz = 1.33ms latency per block
pub const BLOCK_SIZE: usize = 128;

/// Number of blocks in the ring buffer
/// 4 blocks = ~5.3ms total buffer @ 96kHz
pub const BUFFER_BLOCKS: usize = 4;

/// Total samples in ring buffer
pub const BUFFER_SIZE: usize = BLOCK_SIZE * BUFFER_BLOCKS;

/// Audio sample type (32-bit signed, Q31 format)
pub type Sample = i32;

/// Stereo audio frame
#[derive(Clone, Copy, Default)]
#[repr(C)]
pub struct StereoFrame {
    pub left: Sample,
    pub right: Sample,
}

impl StereoFrame {
    pub const ZERO: Self = Self { left: 0, right: 0 };

    pub fn new(left: Sample, right: Sample) -> Self {
        Self { left, right }
    }
}

/// Ring buffer for audio samples
///
/// Lock-free single-producer single-consumer design.
/// Producer writes samples/blocks, consumer reads blocks for processing.
pub struct AudioRingBuffer {
    /// Sample storage
    data: [StereoFrame; BUFFER_SIZE],
    /// Write position (updated by producer)
    write_pos: AtomicUsize,
    /// Read position (updated by consumer)
    read_pos: AtomicUsize,
}

impl AudioRingBuffer {
    pub const fn new() -> Self {
        Self {
            data: [StereoFrame::ZERO; BUFFER_SIZE],
            write_pos: AtomicUsize::new(0),
            read_pos: AtomicUsize::new(0),
        }
    }

    /// Number of samples available to read
    pub fn available(&self) -> usize {
        let write = self.write_pos.load(Ordering::Acquire);
        let read = self.read_pos.load(Ordering::Acquire);
        write.wrapping_sub(read)
    }

    /// Space available for writing
    pub fn space(&self) -> usize {
        BUFFER_SIZE - self.available()
    }

    /// Check if a full block is available for reading
    pub fn block_available(&self) -> bool {
        self.available() >= BLOCK_SIZE
    }

    /// Write a single sample (for USB audio which arrives sample-by-sample)
    pub fn write_sample(&mut self, sample: StereoFrame) -> bool {
        if self.space() == 0 {
            return false; // Buffer full
        }
        let pos = self.write_pos.load(Ordering::Relaxed) % BUFFER_SIZE;
        self.data[pos] = sample;
        self.write_pos.fetch_add(1, Ordering::Release);
        true
    }

    /// Write multiple samples (for batch writes from S/PDIF or ADC)
    pub fn write_samples(&mut self, samples: &[StereoFrame]) -> usize {
        let space = self.space();
        let to_write = samples.len().min(space);

        for i in 0..to_write {
            let pos = self.write_pos.load(Ordering::Relaxed).wrapping_add(i) % BUFFER_SIZE;
            self.data[pos] = samples[i];
        }
        self.write_pos.fetch_add(to_write, Ordering::Release);
        to_write
    }

    /// Read a block of samples for DSP processing
    /// Returns None if not enough samples available
    pub fn read_block(&mut self, output: &mut [StereoFrame; BLOCK_SIZE]) -> bool {
        if self.available() < BLOCK_SIZE {
            return false;
        }

        let read_start = self.read_pos.load(Ordering::Relaxed);
        for i in 0..BLOCK_SIZE {
            let pos = read_start.wrapping_add(i) % BUFFER_SIZE;
            output[i] = self.data[pos];
        }
        self.read_pos.fetch_add(BLOCK_SIZE, Ordering::Release);
        true
    }

    /// Peek at samples without consuming them
    pub fn peek(&self, offset: usize) -> Option<StereoFrame> {
        if offset >= self.available() {
            return None;
        }
        let pos = self.read_pos.load(Ordering::Relaxed).wrapping_add(offset) % BUFFER_SIZE;
        Some(self.data[pos])
    }

    /// Read and consume a single sample
    pub fn read_sample(&mut self) -> Option<StereoFrame> {
        if self.available() == 0 {
            return None;
        }
        let pos = self.read_pos.load(Ordering::Relaxed) % BUFFER_SIZE;
        let sample = self.data[pos];
        self.read_pos.fetch_add(1, Ordering::Release);
        Some(sample)
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.read_pos.store(0, Ordering::Release);
        self.write_pos.store(0, Ordering::Release);
    }
}

/// Output buffer for DMA transfer to I2S
///
/// Uses ping-pong (double) buffering:
/// - While DMA transfers one buffer, DSP fills the other
/// - Ensures continuous output without gaps
pub struct OutputBuffer {
    /// Double buffer - two blocks
    buffers: [[StereoFrame; BLOCK_SIZE]; 2],
    /// Which buffer is currently being played by DMA (0 or 1)
    active: AtomicUsize,
}

impl OutputBuffer {
    pub const fn new() -> Self {
        Self {
            buffers: [[StereoFrame::ZERO; BLOCK_SIZE]; 2],
            active: AtomicUsize::new(0),
        }
    }

    /// Get the buffer that DMA should read from
    pub fn dma_buffer(&self) -> &[StereoFrame; BLOCK_SIZE] {
        let idx = self.active.load(Ordering::Acquire);
        &self.buffers[idx]
    }

    /// Get the buffer that DSP should write to (the inactive one)
    pub fn dsp_buffer_mut(&mut self) -> &mut [StereoFrame; BLOCK_SIZE] {
        let idx = self.active.load(Ordering::Acquire);
        &mut self.buffers[1 - idx]
    }

    /// Swap buffers (called when DMA completes a block)
    pub fn swap(&self) {
        let current = self.active.load(Ordering::Acquire);
        self.active.store(1 - current, Ordering::Release);
    }

    /// Get raw pointer to active buffer for DMA
    pub fn dma_ptr(&self) -> *const StereoFrame {
        self.dma_buffer().as_ptr()
    }
}

/// Audio source identifier
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum AudioSource {
    /// No active source
    None,
    /// USB Audio Class input
    Usb,
    /// S/PDIF digital input
    Spdif,
    /// Analog line input via ADC
    LineIn,
}

/// Signal to notify DSP task that a block is ready
pub static DSP_BLOCK_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal to notify that DMA completed and needs new data
pub static DMA_COMPLETE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
