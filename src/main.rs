//! OtterAmp DSP - Smart Speaker Firmware
//!
//! Embedded Rust firmware for RP2350-based smart speaker with:
//! - USB Audio Class 1.0 input
//! - S/PDIF digital input
//! - Analog line input via PCM1822 ADC
//! - TAS5830 Class-D amplifier with DSP
//! - OLED display with menu system
//! - Rotary encoder for control

#![no_std]
#![no_main]

mod audio;
mod drivers;
mod gui;
mod hw;
mod usb;

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::USB;
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver as UsbDriver;
use embassy_rp::bind_interrupts;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use crate::audio::{I2sTx, calculate_clock_divider, AudioRingBuffer, StereoFrame}; //, SpdifReceiver, SpdifSample, RxState};
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::Common;

use cortex_m::asm;
use defmt_rtt as _;
use panic_probe as _;

// Bind interrupts for peripherals
bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO1>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

use sh1106::{prelude::*, Builder};

use crate::drivers::{RotaryEncoder, Tas5830};
use crate::gui::screens::{AppState, HomeScreen, BootScreen};
use crate::gui::widgets::AudioSource;
use crate::hw::pins::i2c_addr;

// RP2350 boot block - required for the chip to boot
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Binary info for picotool
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"OtterDSP Speaker"),
    embassy_rp::binary_info::rp_program_description!(c"Studio speaker firmware using RP2350"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

/// Shared application state channel for cross-task communication
static STATE_CHANNEL: Channel<CriticalSectionRawMutex, AppStateUpdate, 4> = Channel::new();

/// Audio ring buffer (USB writes, I2S reads)
static mut AUDIO_INPUT: AudioRingBuffer = AudioRingBuffer::new();

/// Core 1 stack (4KB should be plenty for tight audio loop)
static mut CORE1_STACK: Stack<4096> = Stack::new();

/// I2S transmitter - initialized by Core 0, consumed by Core 1
static mut I2S_TX: Option<I2sTx<'static, PIO1, 0>> = None;

/// Flag to signal Core 1 that I2S is ready
static I2S_READY: AtomicBool = AtomicBool::new(false);

/// State update messages
#[derive(Debug, Clone)]
pub enum AppStateUpdate {
    VolumeChanged(u8),
    MuteToggled(bool),
    SourceChanged(AudioSource),
    SignalDetected(bool),
    SampleRateChanged(u32),
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("OtterAmp DSP starting...");

    let p = embassy_rp::init(Default::default());

    // Status LED (GPIO12 on custom board - LED0)
    let mut led = Output::new(p.PIN_12, Level::Low);
    led.set_high();

    defmt::info!("Initializing I2C buses...");

    // I2C0 for OLED display (GPIO0=SDA, GPIO1=SCL after bodge)
    let i2c0 = I2c::new_async(
        p.I2C0,
        p.PIN_1, // SCL
        p.PIN_0, // SDA
        Irqs,
        i2c::Config::default(),
    );

    // I2C1 for TAS5830 amplifier (GPIO14=SDA, GPIO15=SCL after bodge)
    let i2c1 = I2c::new_async(
        p.I2C1,
        p.PIN_15, // SCL
        p.PIN_14, // SDA
        Irqs,
        i2c::Config::default(),
    );

    defmt::info!("Initializing OLED display...");

    // Initialize SH1106 OLED (1.3" display) with address from pins.rs
    let mut display: GraphicsMode<_> = Builder::new()
        .with_i2c_addr(i2c_addr::SH1106)
        .with_size(DisplaySize::Display128x64)
        .with_rotation(DisplayRotation::Rotate0)
        .connect_i2c(i2c0)
        .into();

    if let Err(e) = display.init() {
        defmt::error!("Failed to init display: {:?}", defmt::Debug2Format(&e));
    }

    // Clear display (removes random garbage on power-up)
    display.clear();
    let _ = display.flush();

    // Show boot screen
    let mut boot_screen = BootScreen::new();
    let app_state = AppState::default();
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    defmt::info!("Initializing TAS5830 amplifier...");
    boot_screen.set_progress(20, "Init amp...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    // Initialize TAS5830
    let mut amp = Tas5830::new_default(i2c1);
    if let Err(e) = amp.init().await {
        defmt::error!("Failed to init TAS5830: {:?}", e);
    }

    // Initialize I2S output using PIO1
    defmt::info!("Initializing I2S output...");
    boot_screen.set_progress(40, "Init I2S...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    let mut pio1 = Pio::new(p.PIO1, Irqs);
    let clock_div = calculate_clock_divider(96_000); // 96kHz sample rate

    let i2s_tx = I2sTx::new(
        &mut pio1.common,
        pio1.sm0,
        p.PIN_19, // AMP_BCLK
        p.PIN_20, // AMP_WCLK
        p.PIN_21, // AMP_DATA
        clock_div,
    );

    // Store I2S in static for Core 1 to consume
    // SAFETY: Core 1 hasn't started yet, no race condition
    unsafe {
        I2S_TX = Some(i2s_tx);
    }

    // Spawn Core 1 for dedicated audio processing
    // Core 1 runs a tight loop feeding the I2S FIFO - no executor needed
    defmt::info!("Spawning Core 1 for audio...");
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        core1_audio_main,
    );

    // Signal Core 1 that I2S is ready
    I2S_READY.store(true, Ordering::Release);

    // Initialize S/PDIF input using PIO0 (TODO)
    /*defmt::info!("Initializing S/PDIF input...");
    boot_screen.set_progress(50, "Init SPDIF...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    let pio0 = Pio::new(p.PIO0, Irqs);

    // Spawn S/PDIF receiver task with full PIO access for rate switching
    spawner.spawn(spdif_rx_task(pio0, p.PIN_3)).unwrap();*/

    boot_screen.set_progress(60, "Audio ready");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    defmt::info!("Initializing encoder...");
    boot_screen.set_progress(80, "Init UI...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    // Initialize rotary encoder
    let encoder = RotaryEncoder::new(
        p.PIN_9,  // ENC_A
        p.PIN_10, // ENC_B
        p.PIN_8,  // ENC_BTN
    );

    // Spawn the encoder task
    spawner.spawn(encoder_task(encoder)).unwrap();

    // Spawn the USB task
    spawner.spawn(usb_task(p.USB)).unwrap();

    defmt::info!("Boot complete!");
    boot_screen.set_progress(100, "Ready!");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    Timer::after(Duration::from_millis(500)).await;

    // Switch to home screen
    let mut home_screen = HomeScreen::new();
    let mut app_state = AppState::default();
    app_state.source = AudioSource::LineIn;
    app_state.signal_present = true;

    let _ = home_screen.draw(&mut display, &app_state);
    let _ = display.flush();

    led.set_low();

    defmt::info!("Entering main loop");

    // Main loop: handle state updates and refresh display
    loop {
        // Check for state updates
        if let Ok(update) = STATE_CHANNEL.try_receive() {
            match update {
                AppStateUpdate::VolumeChanged(vol) => {
                    app_state.volume = vol;
                    if let Err(e) = amp.set_volume_percent(vol).await {
                        defmt::error!("Failed to set volume: {:?}", e);
                    }
                }
                AppStateUpdate::MuteToggled(muted) => {
                    app_state.muted = muted;
                    if muted {
                        let _ = amp.mute().await;
                    } else {
                        let _ = amp.unmute().await;
                    }
                }
                AppStateUpdate::SourceChanged(source) => {
                    app_state.source = source;
                }
                AppStateUpdate::SignalDetected(detected) => {
                    app_state.signal_present = detected;
                }
                AppStateUpdate::SampleRateChanged(rate) => {
                    app_state.sample_rate = rate;
                }
            }

            // Refresh display
            let _ = home_screen.draw(&mut display, &app_state);
            let _ = display.flush();
        }

        // Small yield to prevent busy-looping
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Encoder polling task
#[embassy_executor::task]
async fn encoder_task(mut encoder: RotaryEncoder<'static>) {
    use crate::drivers::encoder::EncoderEvent;

    defmt::info!("Encoder task started");

    let mut volume: u8 = 50;
    let mut muted = false;

    loop {
        encoder.wait_for_edge().await;

        if let Some(event) = encoder.poll() {
            defmt::debug!("Encoder event: {:?}", event);

            match event {
                EncoderEvent::Increment => {
                    volume = volume.saturating_add(2).min(100);
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::VolumeChanged(volume));
                }
                EncoderEvent::Decrement => {
                    volume = volume.saturating_sub(2);
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::VolumeChanged(volume));
                }
                EncoderEvent::Press => {
                    // Short press could open menu
                    defmt::info!("Encoder pressed");
                }
                EncoderEvent::LongPress => {
                    // Long press toggles mute
                    muted = !muted;
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::MuteToggled(muted));
                }
                EncoderEvent::Release => {}
            }
        }
    }
}

/// USB audio task
#[embassy_executor::task]
async fn usb_task(usb: embassy_rp::Peri<'static, USB>) {
    use embassy_usb::class::uac1::speaker::{Speaker, State};
    use embassy_usb::class::uac1::{Channel, FeedbackRefresh, SampleWidth};
    use static_cell::StaticCell;

    defmt::info!("USB task started");

    let driver = UsbDriver::new(usb, Irqs);

    // USB device configuration
    let mut config = embassy_usb::Config::new(0x1209, 0x0001); // pid.codes test VID/PID
    config.manufacturer = Some("OtterAmp");
    config.product = Some("OtterDSP Speaker");
    config.serial_number = Some("001");
    config.max_power = 500; // 500mA max
    config.max_packet_size_0 = 64;
    // Required for USB Audio Class
    config.composite_with_iads = true;
    config.device_class = 0xEF; // Miscellaneous
    config.device_sub_class = 0x02; // Common Class
    config.device_protocol = 0x01; // Interface Association Descriptor

    // Static buffers required for USB (must outlive the task)
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static UAC_STATE: StaticCell<State> = StaticCell::new();

    let config_descriptor = CONFIG_DESC.init([0u8; 256]);
    let bos_descriptor = BOS_DESC.init([0u8; 256]);
    let msos_descriptor = MSOS_DESC.init([0u8; 256]);
    let control_buf = CONTROL_BUF.init([0u8; 64]);

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );

    // UAC1 Speaker configuration
    // 96kHz stereo 24-bit = 96 samples/ms * 2 channels * 3 bytes = 576 bytes/frame
    let state = UAC_STATE.init(State::new());

    // Static arrays for sample rates and channels
    static SAMPLE_RATES: [u32; 1] = [96_000];
    static CHANNELS: [Channel; 2] = [Channel::LeftFront, Channel::RightFront];

    let (mut stream, _feedback, control) = Speaker::new(
        &mut builder,
        state,
        640, // max packet size with margin for 96kHz
        SampleWidth::Width3Byte, // 24-bit
        &SAMPLE_RATES,
        &CHANNELS,
        FeedbackRefresh::Period8Frames,
    );

    // Build USB device
    let mut usb = builder.build();

    defmt::info!("USB Audio device configured (96kHz/24-bit stereo)");

    // Run USB device, audio stream handler, and volume monitor concurrently
    let usb_fut = usb.run();

    // Audio data receiver - convert USB audio to I2S and send to output
    let audio_fut = async {
        let mut buf = [0u8; 640];
        let mut usb_active = false;
        loop {
            match stream.read_packet(&mut buf).await {
                Ok(n) => {
                    if n > 0 {
                        // Audio data received - switch to USB source
                        if !usb_active {
                            usb_active = true;
                            let _ = STATE_CHANNEL.try_send(AppStateUpdate::SourceChanged(
                                crate::gui::widgets::AudioSource::Usb,
                            ));
                            let _ = STATE_CHANNEL.try_send(AppStateUpdate::SignalDetected(true));
                            defmt::info!("USB audio stream started");
                        }

                        // Convert USB audio (24-bit packed stereo) to ring buffer
                        // USB format: 3 bytes per sample, little-endian, alternating L/R
                        // Buffer format: 32-bit signed, MSB-aligned
                        let samples = n / 6; // 6 bytes per stereo sample (3 bytes * 2 channels)

                        // SAFETY: Single writer (USB task), reads are from DSP task
                        let input = unsafe { &mut AUDIO_INPUT };

                        for i in 0..samples {
                            let base = i * 6;
                            // Extract 24-bit samples (little-endian) and sign-extend to 32-bit
                            let left_24 = (buf[base] as i32)
                                | ((buf[base + 1] as i32) << 8)
                                | ((buf[base + 2] as i32) << 16);
                            let right_24 = (buf[base + 3] as i32)
                                | ((buf[base + 4] as i32) << 8)
                                | ((buf[base + 5] as i32) << 16);

                            // Sign-extend from 24-bit to 32-bit
                            let left = if left_24 & 0x800000 != 0 {
                                left_24 | 0xFF000000u32 as i32
                            } else {
                                left_24
                            };
                            let right = if right_24 & 0x800000 != 0 {
                                right_24 | 0xFF000000u32 as i32
                            } else {
                                right_24
                            };

                            // Shift left by 8 to MSB-align for 32-bit I2S
                            let frame = StereoFrame::new(left << 8, right << 8);
                            input.write_sample(frame);
                        }
                    }
                }
                Err(_) => {
                    if usb_active {
                        usb_active = false;
                        // Revert to line-in when USB stops
                        let _ = STATE_CHANNEL.try_send(AppStateUpdate::SourceChanged(
                            crate::gui::widgets::AudioSource::LineIn,
                        ));
                        defmt::info!("USB audio stream stopped");
                    }
                    Timer::after(Duration::from_millis(10)).await;
                }
            }
        }
    };

    let volume_fut = async {
        use embassy_usb::class::uac1::speaker::Volume;

        let mut last_muted = false;
        let mut last_volume_db: Option<f32> = None;

        loop {
            control.changed().await;

            // Check volume state (use left channel)
            if let Some(vol) = control.volume(Channel::LeftFront) {
                match vol {
                    Volume::Muted => {
                        if !last_muted {
                            last_muted = true;
                            let _ = STATE_CHANNEL.try_send(AppStateUpdate::MuteToggled(true));
                            defmt::info!("USB muted");
                        }
                    }
                    Volume::DeciBel(vol_db) => {
                        if last_muted {
                            last_muted = false;
                            let _ = STATE_CHANNEL.try_send(AppStateUpdate::MuteToggled(false));
                            defmt::info!("USB unmuted");
                        }
                        if last_volume_db != Some(vol_db) {
                            last_volume_db = Some(vol_db);
                            // Convert dB to percentage (range is -100dB to 0dB per UAC1 spec)
                            // 0dB = 100%, -100dB = 0%
                            let percent = ((vol_db + 100.0) / 100.0 * 100.0).clamp(0.0, 100.0) as u8;
                            let _ = STATE_CHANNEL.try_send(AppStateUpdate::VolumeChanged(percent));
                            defmt::info!("USB volume: {} dB ({}%)", vol_db, percent);
                        }
                    }
                }
            }
        }
    };

    // Run all futures
    embassy_futures::join::join3(usb_fut, audio_fut, volume_fut).await;
}

/// Core 1 entry point - dedicated audio processing
///
/// Runs a tight loop that feeds the I2S FIFO from the ring buffer.
/// No executor overhead - just reads samples and writes to PIO.
/// The PIO FIFO naturally paces output to 96kHz.
fn core1_audio_main() -> ! {
    // Wait for Core 0 to signal that I2S is ready
    while !I2S_READY.load(Ordering::Acquire) {
        asm::nop();
    }

    // Take ownership of I2S from the static
    // SAFETY: Core 0 has finished writing and signaled us
    let mut i2s_tx = unsafe { I2S_TX.take().unwrap() };

    defmt::info!("Core 1: I2S audio loop started");

    // Start the I2S transmitter
    i2s_tx.start();

    // Tight audio loop - runs forever on Core 1
    loop {
        // SAFETY: Core 1 is the only consumer of the input buffer
        let input = unsafe { &mut AUDIO_INPUT };

        if let Some(frame) = input.read_sample() {
            // Audio data available - output it
            // write() blocks until FIFO has space, naturally pacing to 96kHz
            i2s_tx.write(frame.left as u32, frame.right as u32);
        } else {
            // No audio data - output silence
            // This keeps the I2S clock running continuously
            i2s_tx.write(0, 0);
        }
    }
}
