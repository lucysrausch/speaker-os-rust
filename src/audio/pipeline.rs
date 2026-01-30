//! Audio Pipeline for OtterAmp DSP
//!
//! Lock-free ring buffer for audio samples between producers and consumer.
//! - Producers: S/PDIF task, USB task (Core 0)
//! - Consumer: I2S output loop (Core 1)
//!
//! Clock recovery keeps buffer stable via adaptive I2S clock adjustment.

use core::sync::atomic::{AtomicUsize, Ordering};

/// Block size for buffer calculations
pub const BLOCK_SIZE: usize = 128;

/// Number of blocks in the ring buffer
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
/// Producer writes samples, consumer reads samples for I2S output.
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

    /// Write a single sample
    pub fn write_sample(&mut self, sample: StereoFrame) -> bool {
        if self.space() == 0 {
            return false; // Buffer full
        }
        let pos = self.write_pos.load(Ordering::Relaxed) % BUFFER_SIZE;
        self.data[pos] = sample;
        self.write_pos.fetch_add(1, Ordering::Release);
        true
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
