//! Simple linear interpolation resampler
//!
//! Converts S/PDIF input at various sample rates to 96kHz output.
//! Uses linear interpolation for simplicity and low CPU overhead.
//!
//! Supported input rates: 44100, 48000, 88200, 96000, 176400, 192000 Hz
//! Output rate: Always 96000 Hz

use crate::audio::pipeline::StereoFrame;

/// Fixed-point fractional bits for phase accumulator
const FRAC_BITS: u32 = 16;
const FRAC_MASK: u32 = (1 << FRAC_BITS) - 1;
const FRAC_ONE: u32 = 1 << FRAC_BITS;

/// Linear interpolation resampler with drift compensation
pub struct Resampler {
    /// Input sample rate
    input_rate: u32,
    /// Output sample rate (always 96000)
    output_rate: u32,
    /// Phase accumulator (fixed-point: upper bits = integer, lower FRAC_BITS = fraction)
    phase: u32,
    /// Base phase increment (calculated from sample rates)
    phase_inc_base: u32,
    /// Current phase increment (adjusted for drift compensation)
    phase_inc: u32,
    /// Previous input sample (for interpolation)
    prev_left: i32,
    prev_right: i32,
    /// Current input sample (for interpolation)
    curr_left: i32,
    curr_right: i32,
    /// Drift accumulator for passthrough mode (96kHz→96kHz)
    drift_acc: i32,
    /// Pending correction for passthrough: +1 = duplicate, -1 = skip
    pending_correction: i32,
}

impl Resampler {
    /// Create a new resampler with the specified output rate
    pub fn new(output_rate: u32) -> Self {
        Self {
            input_rate: output_rate, // Start with passthrough
            output_rate,
            phase: 0,
            phase_inc_base: FRAC_ONE, // 1:1 ratio initially
            phase_inc: FRAC_ONE,
            prev_left: 0,
            prev_right: 0,
            curr_left: 0,
            curr_right: 0,
            drift_acc: 0,
            pending_correction: 0,
        }
    }

    /// Set the input sample rate
    /// Recalculates the phase increment for the new ratio
    pub fn set_input_rate(&mut self, input_rate: u32) {
        if input_rate == self.input_rate {
            return;
        }

        self.input_rate = input_rate;

        // Phase increment = input_rate / output_rate in fixed-point
        // For upsampling (48k→96k): phase_inc = 0.5 (consume half an input per output)
        // For downsampling (192k→96k): phase_inc = 2.0 (consume 2 inputs per output)
        self.phase_inc_base = ((input_rate as u64 * FRAC_ONE as u64) / self.output_rate as u64) as u32;
        self.phase_inc = self.phase_inc_base;

        // Reset state
        self.phase = 0;
        self.prev_left = 0;
        self.prev_right = 0;
        self.curr_left = 0;
        self.curr_right = 0;
        self.drift_acc = 0;
        self.pending_correction = 0;
    }

    /// Get the current input rate
    pub fn input_rate(&self) -> u32 {
        self.input_rate
    }

    /// Check if this is a passthrough configuration (96k→96k)
    pub fn is_passthrough(&self) -> bool {
        self.input_rate == self.output_rate
    }

    /// Adjust the resampling rate based on buffer level for clock drift compensation
    ///
    /// `buffer_level` is 0-255 where 128 is the target (half full).
    /// - If buffer is filling up: speed up consumption (increase phase_inc)
    /// - If buffer is emptying: slow down consumption (decrease phase_inc)
    pub fn adjust_rate(&mut self, buffer_level: u8) {
        let error = buffer_level as i32 - 128;

        if self.is_passthrough() {
            // For passthrough mode (96kHz→96kHz): accumulate drift and occasionally skip/duplicate
            // Dead zone: ±32 around target for stability
            if error.abs() < 32 {
                return;
            }

            // Accumulate drift based on buffer error
            let drift_rate = (error - error.signum() * 32) / 4;
            self.drift_acc += drift_rate;

            // When drift exceeds threshold, schedule a correction
            const CORRECTION_THRESHOLD: i32 = 500;
            if self.drift_acc > CORRECTION_THRESHOLD {
                self.pending_correction = -1; // Skip one sample (buffer too full)
                self.drift_acc = 0;
            } else if self.drift_acc < -CORRECTION_THRESHOLD {
                self.pending_correction = 1; // Duplicate one sample (buffer too empty)
                self.drift_acc = 0;
            }
        } else {
            // For interpolation mode: continuous phase adjustment
            // Adjustment range: ±0.5% of phase_inc_base
            let max_adjust = (self.phase_inc_base / 200) as i32; // 0.5%
            let adjust = (error * max_adjust) / 128;

            self.phase_inc = ((self.phase_inc_base as i32) + adjust).max(1) as u32;
        }
    }

    /// Process input samples and produce output samples
    ///
    /// For upsampling (e.g., 48k→96k), output will have more samples than input
    /// For downsampling (e.g., 192k→96k), output will have fewer samples than input
    ///
    /// Returns the number of output samples written
    pub fn process(&mut self, input: &[StereoFrame], output: &mut [StereoFrame]) -> usize {
        if self.is_passthrough() {
            // Passthrough with occasional skip/duplicate for drift compensation
            return self.process_passthrough(input, output);
        }

        let mut input_idx = 0usize;
        let mut output_idx = 0usize;

        while output_idx < output.len() {
            // Check if we need to advance to next input sample
            while self.phase >= FRAC_ONE && input_idx < input.len() {
                // Move to next input sample
                self.prev_left = self.curr_left;
                self.prev_right = self.curr_right;
                self.curr_left = input[input_idx].left;
                self.curr_right = input[input_idx].right;
                input_idx += 1;
                self.phase -= FRAC_ONE;
            }

            // If we've exhausted input, stop
            if input_idx == 0 && self.phase >= FRAC_ONE {
                // Need at least one input sample to start
                break;
            }
            if input_idx >= input.len() && self.phase >= FRAC_ONE {
                // No more input samples available
                break;
            }

            // Linear interpolation
            let frac = self.phase & FRAC_MASK;
            let left = self.interpolate(self.prev_left, self.curr_left, frac);
            let right = self.interpolate(self.prev_right, self.curr_right, frac);

            output[output_idx] = StereoFrame::new(left, right);
            output_idx += 1;

            // Advance phase
            self.phase += self.phase_inc;
        }

        output_idx
    }

    /// Process passthrough mode (96kHz→96kHz) with drift compensation
    /// Occasionally skips or duplicates a sample to compensate for clock drift
    fn process_passthrough(&mut self, input: &[StereoFrame], output: &mut [StereoFrame]) -> usize {
        let mut input_idx = 0usize;
        let mut output_idx = 0usize;

        while input_idx < input.len() && output_idx < output.len() {
            // Check for skip correction (buffer too full - consume without outputting)
            if self.pending_correction < 0 {
                self.pending_correction = 0;
                input_idx += 1; // Skip this input sample
                continue;
            }

            // Normal passthrough
            output[output_idx] = input[input_idx];
            output_idx += 1;
            input_idx += 1;

            // Check for duplicate correction (buffer too empty - output same sample again)
            if self.pending_correction > 0 && output_idx < output.len() {
                // Duplicate by outputting an interpolated sample
                if input_idx < input.len() {
                    // Interpolate between current and next for smoother duplicate
                    let curr = input[input_idx - 1];
                    let next = input[input_idx];
                    output[output_idx] = StereoFrame::new(
                        (curr.left / 2) + (next.left / 2),
                        (curr.right / 2) + (next.right / 2),
                    );
                } else {
                    // No next sample, just duplicate
                    output[output_idx] = input[input_idx - 1];
                }
                output_idx += 1;
                self.pending_correction = 0;
            }
        }

        output_idx
    }

    /// Process S/PDIF raw words directly (extracts audio and resamples)
    ///
    /// S/PDIF words contain interleaved L/R samples, so this processes pairs.
    /// Returns the number of stereo output samples written.
    pub fn process_spdif(&mut self, input: &[u32], output: &mut [StereoFrame]) -> usize {
        if input.len() < 2 {
            return 0;
        }

        // Extract stereo frames from S/PDIF words
        // S/PDIF has alternating L/R sub-frames
        let mut stereo_input = [StereoFrame::ZERO; 128]; // Max reasonable block size
        let num_frames = (input.len() / 2).min(stereo_input.len());

        for i in 0..num_frames {
            let left_word = input[i * 2];
            let right_word = input[i * 2 + 1];
            stereo_input[i] = StereoFrame::new(
                extract_audio_sample(left_word),
                extract_audio_sample(right_word),
            );
        }

        self.process(&stereo_input[..num_frames], output)
    }

    /// Linear interpolation between two samples
    /// frac is the fractional position (0 = prev, FRAC_ONE = curr)
    #[inline]
    fn interpolate(&self, prev: i32, curr: i32, frac: u32) -> i32 {
        // Linear interpolation: prev + (curr - prev) * frac
        // Using 64-bit intermediate to avoid overflow
        let diff = curr as i64 - prev as i64;
        let interp = (diff * frac as i64) >> FRAC_BITS;
        (prev as i64 + interp) as i32
    }

    /// Reset the resampler state (call when input stream changes)
    pub fn reset(&mut self) {
        self.phase = 0;
        self.phase_inc = self.phase_inc_base;
        self.prev_left = 0;
        self.prev_right = 0;
        self.curr_left = 0;
        self.curr_right = 0;
        self.drift_acc = 0;
        self.pending_correction = 0;
    }
}

/// Extract audio sample from S/PDIF word
/// Returns 32-bit signed audio, attenuated to prevent clipping
#[inline]
fn extract_audio_sample(word: u32) -> i32 {
    // S/PDIF format: [31:28] VUCP, [27:4] 24-bit audio, [3:0] Sync
    // Shift left 4 to align to MSB, then right 1 to attenuate
    (((word & 0x0FFF_FFF0) << 4) as i32) >> 1
}