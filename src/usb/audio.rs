//! USB Audio Class 1.0 speaker implementation
//!
//! Receives audio data from USB host and makes it available for playback.

use embassy_usb::class::uac1::speaker::{ControlMonitor, Feedback, Speaker, State, Stream, Volume};
use embassy_usb::class::uac1::{Channel, FeedbackRefresh, SampleWidth};
use embassy_usb::driver::Driver;
use embassy_usb::Builder;

/// USB Audio configuration
#[derive(Clone, Copy)]
pub struct UsbAudioConfig {
    /// Sample rate in Hz
    pub sample_rate: u32,
    /// Bit depth
    pub bit_depth: SampleWidth,
    /// Max packet size (must accommodate sample_rate * channels * bytes_per_sample / 1000 + margin)
    pub max_packet_size: u16,
}

impl Default for UsbAudioConfig {
    fn default() -> Self {
        Self {
            sample_rate: 96_000,
            bit_depth: SampleWidth::Width3Byte, // 24-bit
            // At 96kHz, 24-bit stereo: 96 samples/ms * 2 channels * 3 bytes = 576 bytes
            // Add margin for clock drift: 640 bytes
            max_packet_size: 640,
        }
    }
}

impl UsbAudioConfig {
    /// Create config for 96kHz 24-bit stereo
    pub fn new_96k_24bit() -> Self {
        Self::default()
    }

    /// Create config for 48kHz 24-bit stereo
    pub fn new_48k_24bit() -> Self {
        Self {
            sample_rate: 48_000,
            bit_depth: SampleWidth::Width3Byte,
            // 48 samples/ms * 2 channels * 3 bytes = 288 bytes + margin
            max_packet_size: 320,
        }
    }

    /// Bytes per stereo sample
    pub fn bytes_per_sample(&self) -> usize {
        match self.bit_depth {
            SampleWidth::Width2Byte => 4, // 2 bytes * 2 channels
            SampleWidth::Width3Byte => 6, // 3 bytes * 2 channels
            SampleWidth::Width4Byte => 8, // 4 bytes * 2 channels
        }
    }

    /// Expected bytes per USB frame (1ms at full-speed)
    pub fn bytes_per_frame(&self) -> usize {
        (self.sample_rate as usize / 1000) * self.bytes_per_sample()
    }
}

/// Stereo audio channels for UAC1
pub static STEREO_CHANNELS: [Channel; 2] = [Channel::LeftFront, Channel::RightFront];

/// USB Audio receiver
///
/// Manages the USB audio stream and provides audio data to the playback system.
pub struct UsbAudioReceiver<'d, D: Driver<'d>> {
    stream: Stream<'d, D>,
    feedback: Feedback<'d, D>,
    control: ControlMonitor<'d>,
    config: UsbAudioConfig,
    /// Current sample rate reported by host
    current_sample_rate: u32,
    /// Feedback accumulator for rate matching
    feedback_acc: u32,
    /// Sample counter for feedback calculation
    sample_count: u32,
}

impl<'d, D: Driver<'d> + 'd> UsbAudioReceiver<'d, D> {
    /// Create a new USB audio receiver
    ///
    /// Returns the receiver and the Speaker instance (which should be dropped after setup).
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        config: UsbAudioConfig,
    ) -> Self {
        let sample_rates = [config.sample_rate];

        let (stream, feedback, control) = Speaker::new(
            builder,
            state,
            config.max_packet_size,
            config.bit_depth,
            &sample_rates,
            &STEREO_CHANNELS,
            FeedbackRefresh::Period8Frames, // 8ms feedback period
        );

        Self {
            stream,
            feedback,
            control,
            config,
            current_sample_rate: config.sample_rate,
            feedback_acc: 0,
            sample_count: 0,
        }
    }

    /// Read audio packet from USB host
    ///
    /// Returns the number of bytes read into the buffer.
    /// The buffer should be at least `max_packet_size` bytes.
    pub async fn read_packet(
        &mut self,
        buf: &mut [u8],
    ) -> Result<usize, embassy_usb::driver::EndpointError> {
        self.stream.read_packet(buf).await
    }

    /// Get current sample rate as configured by host
    pub fn sample_rate(&self) -> u32 {
        self.control.sample_rate_hz()
    }

    /// Check if audio stream is muted
    pub fn is_muted(&self, channel: Channel) -> bool {
        matches!(self.control.volume(channel), Some(Volume::Muted))
    }

    /// Get volume for a channel in dB (returns None if muted)
    pub fn volume_db(&self, channel: Channel) -> Option<f32> {
        match self.control.volume(channel) {
            Some(Volume::DeciBel(db)) => Some(db),
            _ => None,
        }
    }

    /// Wait for control settings to change (volume, mute, sample rate)
    pub async fn wait_for_control_change(&self) {
        self.control.changed().await;
    }

    /// Update feedback value for rate matching
    ///
    /// This should be called periodically (every 8ms or so) to inform the host
    /// of the actual sample consumption rate to prevent buffer under/overruns.
    ///
    /// `samples_consumed` is the number of stereo samples consumed since last call.
    pub async fn update_feedback(
        &mut self,
        samples_consumed: u32,
    ) -> Result<(), embassy_usb::driver::EndpointError> {
        self.sample_count += samples_consumed;

        // Calculate feedback value
        // UAC1 feedback is in 10.14 fixed-point format for full-speed
        // representing samples per frame (samples per ms)
        //
        // For 96kHz: 96 samples/ms = 96.0 in 10.14 = 96 << 14 = 0x180000
        //
        // We average over multiple frames to smooth the value
        const FEEDBACK_FRAMES: u32 = 8;

        if self.sample_count >= self.config.sample_rate / 1000 * FEEDBACK_FRAMES {
            // Calculate actual samples per frame
            let samples_per_frame = self.sample_count / FEEDBACK_FRAMES;

            // Convert to 10.14 fixed point
            let feedback_value = samples_per_frame << 14;

            // Write feedback
            let fb_bytes = feedback_value.to_le_bytes();
            self.feedback.write_packet(&fb_bytes[..3]).await?;

            self.sample_count = 0;
        }

        Ok(())
    }

    /// Write raw feedback value (10.14 fixed-point format)
    pub async fn write_feedback_raw(
        &mut self,
        value: u32,
    ) -> Result<(), embassy_usb::driver::EndpointError> {
        let fb_bytes = value.to_le_bytes();
        self.feedback.write_packet(&fb_bytes[..3]).await
    }
}

/// Convert 24-bit USB audio packet to 32-bit I2S samples
///
/// USB 24-bit audio is packed as 3 bytes per sample, LSB first.
/// I2S expects 32-bit samples with audio in the upper 24 bits.
///
/// Returns the number of stereo samples converted.
pub fn usb_24bit_to_i2s_32bit(
    usb_data: &[u8],
    i2s_left: &mut [i32],
    i2s_right: &mut [i32],
) -> usize {
    let bytes_per_stereo = 6; // 3 bytes * 2 channels
    let num_samples = usb_data.len() / bytes_per_stereo;
    let samples = num_samples.min(i2s_left.len()).min(i2s_right.len());

    for i in 0..samples {
        let offset = i * bytes_per_stereo;

        // Left channel (first 3 bytes, LSB first)
        let l0 = usb_data[offset] as i32;
        let l1 = usb_data[offset + 1] as i32;
        let l2 = usb_data[offset + 2] as i32;
        // Pack into upper 24 bits of 32-bit word, sign-extend
        let left_24 = l0 | (l1 << 8) | (l2 << 16);
        i2s_left[i] = left_24 << 8; // Shift to upper 24 bits

        // Right channel (next 3 bytes, LSB first)
        let r0 = usb_data[offset + 3] as i32;
        let r1 = usb_data[offset + 4] as i32;
        let r2 = usb_data[offset + 5] as i32;
        let right_24 = r0 | (r1 << 8) | (r2 << 16);
        i2s_right[i] = right_24 << 8;
    }

    samples
}

/// Convert 24-bit USB audio packet to packed stereo samples (for I2S FIFO)
///
/// Packs stereo samples as u64 (left in upper 32 bits, right in lower 32 bits).
pub fn usb_24bit_to_stereo_packed(usb_data: &[u8], output: &mut [u64]) -> usize {
    let bytes_per_stereo = 6;
    let num_samples = usb_data.len() / bytes_per_stereo;
    let samples = num_samples.min(output.len());

    for i in 0..samples {
        let offset = i * bytes_per_stereo;

        // Left channel
        let l0 = usb_data[offset] as u32;
        let l1 = usb_data[offset + 1] as u32;
        let l2 = usb_data[offset + 2] as u32;
        let left = (l0 | (l1 << 8) | (l2 << 16)) << 8;

        // Right channel
        let r0 = usb_data[offset + 3] as u32;
        let r1 = usb_data[offset + 4] as u32;
        let r2 = usb_data[offset + 5] as u32;
        let right = (r0 | (r1 << 8) | (r2 << 16)) << 8;

        output[i] = ((left as u64) << 32) | (right as u64);
    }

    samples
}
