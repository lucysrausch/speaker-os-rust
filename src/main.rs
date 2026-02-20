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
mod config;
mod drivers;
mod gui;
mod hw;
mod usb;
mod usb_msc;

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::USB;
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver as UsbDriver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use crate::audio::{AudioRingBuffer, ClockSpeed, I2sRx, I2sTx, StereoFrame};
use crate::audio::{
    SpdifRx, SpdifState, DMA_BLOCK_SIZE as SPDIF_DMA_BLOCK_SIZE, SPDIF_RX_FIFO_SIZE,
};
use embassy_rp::peripherals::{PIO0, PIO1};

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

use crate::drivers::settings::{self, PersistedSettings};
use crate::drivers::{RotaryEncoder, Tas5830};
use crate::gui::display::Sh1106;
use crate::gui::screens::{
    AppState, BootScreen, EqualizerScreen, HomeScreen, MainMenuScreen, ScreenAction, ScreenId,
    SettingsScreen, SourceSelectScreen, UsbConfigScreen,
};
use crate::gui::widgets::{AudioSource, SignalStatus};
use crate::hw::pins::{audio as audio_cfg, i2c_addr, i2c_freq};

/// Watchdog scratch register magic value for USB config mode.
const CONFIG_MODE_MAGIC: u32 = 0x0CF6_70DE;

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

/// I2S receiver - initialized by Core 0, consumed by Core 1
static mut I2S_RX: Option<I2sRx<'static, PIO1, 1>> = None;

/// Flag to signal Core 1 that I2S is ready
static I2S_READY: AtomicBool = AtomicBool::new(false);

/// Flag indicating S/PDIF signal is present (set/cleared by spdif_task)
static SPDIF_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Active audio source (0=LineIn, 1=Spdif, 2=Usb)
/// Managed by main loop based on connection events (last connected wins).
static ACTIVE_SOURCE: AtomicU8 = AtomicU8::new(SOURCE_LINEIN);

const SOURCE_LINEIN: u8 = 0;
const SOURCE_SPDIF: u8 = 1;
const SOURCE_USB: u8 = 2;

/// Pending I2S sample rate change (0 = no change pending)
/// Core 0 writes, Core 1 reads and clears
static PENDING_I2S_RATE: AtomicU32 = AtomicU32::new(0);

/// Current I2S output sample rate
/// Core 1 writes after rate change, Core 0 reads for DSP coefficient selection
static CURRENT_I2S_RATE: AtomicU32 = AtomicU32::new(audio_cfg::SAMPLE_RATE);

/// Peak audio levels (absolute value, updated by Core 1, read/reset by Core 0)
/// Core 1 uses fetch_max per sample; Core 0 uses swap(0) to read and reset atomically.
static PEAK_LEFT: AtomicU32 = AtomicU32::new(0);
static PEAK_RIGHT: AtomicU32 = AtomicU32::new(0);

/// Flash handle for USB config mode (only used when `config_mode == true`).
/// SAFETY: Only accessed from the config mode code path (single-threaded).
static mut CONFIG_MODE_FLASH: Option<
    Flash<'static, embassy_rp::peripherals::FLASH, Blocking, { settings::FLASH_SIZE }>,
> = None;

/// Set the watchdog scratch register and reboot into USB config mode.
fn enter_usb_config_mode() -> ! {
    let scratch0_ptr = 0x400d_800Cu32 as *mut u32;
    unsafe { core::ptr::write_volatile(scratch0_ptr, CONFIG_MODE_MAGIC) };
    cortex_m::peripheral::SCB::sys_reset();
}

/// Clip threshold: peaks above this level trigger the CLIP warning.
/// Expressed as a fraction of i32::MAX. 99 = >99% of full range.
const CLIP_THRESHOLD_PERCENT: u64 = 99;
const CLIP_LEVEL: u32 = (i32::MAX as u64 * CLIP_THRESHOLD_PERCENT / 100) as u32;

/// Convert a raw audio peak (0..i32::MAX) to a 0-100 meter level using log2.
/// ~48dB range: 0 dBFS → 100%, −48 dBFS → 0%, silence → 0%.
fn log_meter(raw: u32) -> u8 {
    use micromath::F32Ext;
    if raw == 0 {
        return 0;
    }
    const LOG2_MAX: f32 = 31.0; // log2(i32::MAX)
    const RANGE_BITS: f32 = 8.0; // floor at −48 dBFS
    const LOG2_FLOOR: f32 = LOG2_MAX - RANGE_BITS;
    let log_val = (raw as f32).log2();
    ((log_val - LOG2_FLOOR) / RANGE_BITS * 100.0).clamp(0.0, 100.0) as u8
}

/// S/PDIF FIFO buffer (static allocation)
static SPDIF_FIFO: StaticCell<[u32; SPDIF_RX_FIFO_SIZE]> = StaticCell::new();

/// I2S output sample rate
const I2S_SAMPLE_RATE: u32 = audio_cfg::SAMPLE_RATE;

/// USB audio input sample rate
const USB_SAMPLE_RATE: u32 = audio_cfg::USB_SAMPLE_RATE;

/// State update messages
#[derive(Debug, Clone)]
pub enum AppStateUpdate {
    VolumeChanged(u8),
    MuteToggled(bool),
    SourceChanged(AudioSource),
    SignalDetected(bool),
    SampleRateChanged(u32),
    EncoderPress,
    EncoderLongPress,
    EncoderRotate(i8),
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("OtterAmp DSP starting...");

    let mut p = embassy_rp::init(Default::default());
    let pins = board_pins!(p);

    // Check watchdog scratch register for config mode request.
    // If the magic value is set, reboot into USB MSC config mode.
    // Watchdog scratch0 is at base 0x400d_8000 + 0x0C (persists through soft reset).
    let config_mode = {
        let scratch0_ptr = 0x400d_800Cu32 as *mut u32;
        let scratch = unsafe { core::ptr::read_volatile(scratch0_ptr) };
        if scratch == CONFIG_MODE_MAGIC {
            // Clear the magic so we don't loop forever
            unsafe { core::ptr::write_volatile(scratch0_ptr, 0) };
            true
        } else {
            false
        }
    };

    // Drive TAS5830 PDN low immediately to ensure clean reset on reflash.
    // Without this, the pin floats high during boot and the amp may be in
    // an unknown state by the time init() runs.
    let amp_pdn = Output::new(pins.amp_pdn, Level::Low);
    let amp_mute = Output::new(pins.amp_mute, Level::Low);

    // Load saved settings from flash
    let mut flash = Flash::<_, Blocking, { settings::FLASH_SIZE }>::new_blocking(p.FLASH);
    let saved_settings = settings::load(&mut flash);

    // Status LED
    let mut led = Output::new(pins.led0, Level::Low);
    led.set_high();

    defmt::info!("Initializing I2C buses...");

    // I2C0 for OLED display
    let mut i2c0_cfg = i2c::Config::default();
    i2c0_cfg.frequency = i2c_freq::OLED_HZ;
    let i2c0 = I2c::new_async(
        p.I2C0,
        pins.i2c0_scl,
        pins.i2c0_sda,
        Irqs,
        i2c0_cfg,
    );

    // I2C1 for TAS5830 amplifier
    let mut i2c1_cfg = i2c::Config::default();
    i2c1_cfg.frequency = i2c_freq::AMP_HZ;
    let i2c1 = I2c::new_async(
        p.I2C1,
        pins.i2c1_scl,
        pins.i2c1_sda,
        Irqs,
        i2c1_cfg,
    );

    defmt::info!("Initializing OLED display...");

    // Initialize SH1106 OLED (1.3" display) with custom driver for partial page flush
    let mut display = Sh1106::new(i2c0, i2c_addr::SH1106);
    if let Err(e) = display.init().await {
        defmt::error!("Failed to init display: {:?}", defmt::Debug2Format(&e));
    }

    // Clear display (removes random garbage on power-up)
    display.clear_all(embedded_graphics::pixelcolor::BinaryColor::Off);
    let _ = display.flush_all().await;

    // Show boot screen
    let mut boot_screen = BootScreen::new();
    let app_state = AppState::default();
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    // ── USB Config Mode ─────────────────────────────────────────────────
    if config_mode {
        defmt::info!("Entering USB Config Mode");
        let usb_config_screen = UsbConfigScreen::new();
        let _ = usb_config_screen.draw(&mut display, &app_state);
        let _ = display.flush_all().await;

        // Initialize USB as Mass Storage device
        let driver = UsbDriver::new(p.USB, Irqs);

        let mut usb_config = embassy_usb::Config::new(0x1209, 0x0002); // Separate PID for MSC
        usb_config.manufacturer = Some("OtterAmp");
        usb_config.product = Some("OtterDSP Config");
        usb_config.serial_number = Some("CFG001");
        usb_config.max_power = 100;
        usb_config.max_packet_size_0 = 64;
        // Required: builder.function() creates an IAD, which requires these fields
        usb_config.composite_with_iads = true;
        usb_config.device_class = 0xEF;
        usb_config.device_sub_class = 0x02;
        usb_config.device_protocol = 0x01;

        static MSC_CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
        static MSC_BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
        static MSC_MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
        static MSC_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let config_descriptor = MSC_CONFIG_DESC.init([0u8; 256]);
        let bos_descriptor = MSC_BOS_DESC.init([0u8; 256]);
        let msos_descriptor = MSC_MSOS_DESC.init([0u8; 256]);
        let control_buf = MSC_CONTROL_BUF.init([0u8; 64]);

        let mut builder = embassy_usb::Builder::new(
            driver,
            usb_config,
            config_descriptor,
            bos_descriptor,
            msos_descriptor,
            control_buf,
        );

        let mut msc = usb_msc::MscClass::new(&mut builder, 64);
        let mut usb_dev = builder.build();

        // Create virtual FAT12 filesystem backed by flash
        fn flash_read_config(buf: &mut [u8]) -> u16 {
            // SAFETY: Flash is only accessed from this task in config mode
            let flash = unsafe {
                &mut *core::ptr::addr_of_mut!(CONFIG_MODE_FLASH)
            };
            if let Some(flash) = flash.as_mut() {
                if let Some(data) = config::flash::read_config(flash) {
                    let len = data.len().min(buf.len());
                    buf[..len].copy_from_slice(&data[..len]);
                    return len as u16;
                }
            }
            // If no config in flash, use built-in default
            let default = config::DEFAULT_CONFIG;
            let len = default.len().min(buf.len());
            buf[..len].copy_from_slice(&default[..len]);
            len as u16
        }

        fn flash_write_config(data: &[u8]) {
            let flash = unsafe {
                &mut *core::ptr::addr_of_mut!(CONFIG_MODE_FLASH)
            };
            if let Some(flash) = flash.as_mut() {
                config::flash::write_config(flash, data);
            }
        }

        // Store flash handle in a static for the callbacks
        // SAFETY: Only accessed in config mode, single-threaded
        unsafe {
            CONFIG_MODE_FLASH = Some(flash);
        }

        let mut vfs = usb_msc::fat12::VirtualFat12::new(
            flash_read_config,
            flash_write_config,
        );

        defmt::info!("USB MSC ready, waiting for host...");

        // Run USB device and MSC handler concurrently (never returns)
        let usb_fut = usb_dev.run();
        let msc_fut = msc.run(&mut vfs);
        embassy_futures::join::join(usb_fut, msc_fut).await;

        // Should never reach here, but just in case
        cortex_m::peripheral::SCB::sys_reset();
    }

    defmt::info!("Initializing TAS5830 amplifier...");
    boot_screen.set_progress(20, "Init amp...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    // Initialize TAS5830 (manages PDN and MUTE pins internally)
    let mut amp = Tas5830::new_default(i2c1, amp_pdn, amp_mute);
    if let Err(e) = amp.init().await {
        defmt::error!("Failed to init TAS5830: {:?}", e);
    }

    // Initialize I2S output using PIO1
    defmt::info!("Initializing I2S output...");
    boot_screen.set_progress(40, "Init I2S...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    let mut pio1 = Pio::new(p.PIO1, Irqs);

    let i2s_tx = I2sTx::new(
        &mut pio1.common,
        pio1.sm0,
        pins.amp_bclk,
        pins.amp_wclk,
        pins.amp_data,
        audio_cfg::SAMPLE_RATE,
    );

    // Load DSP config from flash while I2S isn't running yet
    let dsp_config = config::load_and_build_config(&mut flash);

    // Initialize I2S receiver for line-in ADC (PIO1 SM1)
    let i2s_rx = I2sRx::new(
        &mut pio1.common,
        pio1.sm1,
        pins.adc_bclk,
        pins.adc_wclk,
        pins.adc_data,
        audio_cfg::SAMPLE_RATE,
    );

    // Store I2S TX and RX in statics
    // SAFETY: Core 1 hasn't started yet, no race condition
    unsafe {
        I2S_TX = Some(i2s_tx);
        I2S_RX = Some(i2s_rx);
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

    // Wait for Core 1 to start the I2S audio loop — the TAS5830 needs
    // BCLK/WCLK actively clocking to enter Play mode.
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

    // Apply saved volume to DSP config so the amp initializes at the
    // correct level (config file has a fixed default, not the user's level).
    let mut dsp_config = dsp_config;
    dsp_config.digital_volume =
        crate::drivers::tas5830::volume_percent_to_reg(saved_settings.volume);

    // Now that I2S clocks are running, transition TAS5830 to PLAY
    if let Err(e) = amp.play(&dsp_config).await {
        defmt::error!("Failed to start TAS5830 playback: {:?}", e);
    }

    // Apply saved mute state (play() always starts unmuted)
    if saved_settings.muted {
        let _ = amp.mute().await;
    }

    // Initialize S/PDIF input using PIO0
    defmt::info!("Initializing S/PDIF input...");
    boot_screen.set_progress(50, "Init SPDIF...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    let pio0 = Pio::new(p.PIO0, Irqs);

    // Initialize S/PDIF FIFO buffer
    let spdif_fifo = SPDIF_FIFO.init([0u32; SPDIF_RX_FIFO_SIZE]);

    // Spawn S/PDIF receiver task with full PIO access for rate switching
    spawner
        .spawn(spdif_task(pio0, pins.spdif_rx, p.DMA_CH0, spdif_fifo))
        .unwrap();

    boot_screen.set_progress(60, "Audio ready");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    defmt::info!("Initializing encoder...");
    boot_screen.set_progress(80, "Loading otters...");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    Timer::after(Duration::from_millis(500)).await;

    // Initialize rotary encoder
    let encoder = RotaryEncoder::new(
        pins.enc_b,
        pins.enc_a,
        pins.enc_btn,
    );

    // Spawn the encoder task
    spawner.spawn(encoder_task(encoder)).unwrap();

    // Spawn the USB task
    spawner.spawn(usb_task(p.USB)).unwrap();

    // USB VBUS sense
    let usb_vbus = embassy_rp::gpio::Input::new(pins.usb_vbus, embassy_rp::gpio::Pull::None);

    defmt::info!("Boot complete!");
    boot_screen.set_progress(100, "Ready!");
    let _ = boot_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    Timer::after(Duration::from_millis(200)).await;

    // Switch to home screen — initialize all screens and state machine
    let mut current_screen = ScreenId::Home;
    let mut home_screen = HomeScreen::new();
    let mut main_menu_screen = MainMenuScreen::new();
    let mut source_select_screen = SourceSelectScreen::new();
    let mut equalizer_screen = EqualizerScreen::new();
    let mut settings_screen = SettingsScreen::new();
    let mut app_state = AppState {
        volume: saved_settings.volume,
        muted: saved_settings.muted,
        source: AudioSource::LineIn,
        signal_status: SignalStatus::Ok,
        ..AppState::default()
    };

    let _ = home_screen.draw(&mut display, &app_state);
    let _ = display.flush_all().await;

    led.set_low();

    defmt::info!("Entering main loop");

    // Source arbitration state (edge detection)
    let mut prev_spdif_active = false;
    let mut prev_vbus_present = usb_vbus.is_high();

    // Level metering state
    let mut peak_hold_left: u8 = 0;
    let mut peak_hold_right: u8 = 0;
    let mut peak_decay_counter: u8 = 0;
    let mut clip_hold_counter: u8 = 0;
    let mut clip_flash_counter: u8 = 0;
    let mut clip_flash_on = false;
    let mut display_refresh_counter: u8 = 0;
    let mut display_dirty = true; // Track whether display needs full redraw
    let mut volume_dirty = false; // Track whether only volume region needs redraw
    let mut prev_clip_flash = false;
    let mut save_pending = false;
    let mut save_deadline = Instant::now();
    let mut menu_rotate_accum: i8 = 0; // Accumulator to reduce menu rotation sensitivity

    // If USB is already connected at boot, start with USB
    if prev_vbus_present {
        ACTIVE_SOURCE.store(SOURCE_USB, Ordering::Release);
        app_state.source = AudioSource::Usb;
        PENDING_I2S_RATE.store(USB_SAMPLE_RATE, Ordering::Release);
        let _ = home_screen.draw(&mut display, &app_state);
        let _ = display.flush_all().await;
        defmt::info!("USB connected at boot");
    }

    // Main loop: source arbitration, state updates, display refresh
    loop {
        // --- Source arbitration (last connected wins) ---
        let spdif_active = SPDIF_ACTIVE.load(Ordering::Acquire);
        let vbus_present = usb_vbus.is_high();
        let mut source_changed = false;

        // S/PDIF rising edge: signal just appeared → switch to S/PDIF
        if spdif_active && !prev_spdif_active {
            ACTIVE_SOURCE.store(SOURCE_SPDIF, Ordering::Release);
            app_state.source = AudioSource::Spdif;
            // Signal status will be updated by metering logic below
            // Sample rate is updated via SampleRateChanged from spdif_task
            source_changed = true;
            defmt::info!("Source: S/PDIF (signal detected)");
        }

        // S/PDIF falling edge: signal just lost → fallback
        if !spdif_active && prev_spdif_active {
            if vbus_present {
                ACTIVE_SOURCE.store(SOURCE_USB, Ordering::Release);
                PENDING_I2S_RATE.store(USB_SAMPLE_RATE, Ordering::Release);
                app_state.source = AudioSource::Usb;
                app_state.sample_rate = USB_SAMPLE_RATE;
                defmt::info!("Source: USB (S/PDIF lost, USB connected)");
            } else {
                ACTIVE_SOURCE.store(SOURCE_LINEIN, Ordering::Release);
                PENDING_I2S_RATE.store(I2S_SAMPLE_RATE, Ordering::Release);
                app_state.source = AudioSource::LineIn;
                app_state.sample_rate = I2S_SAMPLE_RATE;
                defmt::info!("Source: Line-in (S/PDIF lost)");
            }
            source_changed = true;
        }

        // USB VBUS rising edge: cable just plugged in → switch to USB
        if vbus_present && !prev_vbus_present {
            ACTIVE_SOURCE.store(SOURCE_USB, Ordering::Release);
            PENDING_I2S_RATE.store(USB_SAMPLE_RATE, Ordering::Release);
            app_state.source = AudioSource::Usb;
            app_state.sample_rate = USB_SAMPLE_RATE;
            source_changed = true;
            defmt::info!("Source: USB (cable connected)");
        }

        // USB VBUS falling edge: cable just unplugged → fallback
        if !vbus_present && prev_vbus_present {
            if spdif_active {
                ACTIVE_SOURCE.store(SOURCE_SPDIF, Ordering::Release);
                app_state.source = AudioSource::Spdif;
                // Keep current S/PDIF sample rate (already set by spdif_task)
                defmt::info!("Source: S/PDIF (USB unplugged)");
            } else {
                ACTIVE_SOURCE.store(SOURCE_LINEIN, Ordering::Release);
                PENDING_I2S_RATE.store(I2S_SAMPLE_RATE, Ordering::Release);
                app_state.source = AudioSource::LineIn;
                app_state.sample_rate = I2S_SAMPLE_RATE;
                defmt::info!("Source: Line-in (USB unplugged)");
            }
            source_changed = true;
        }

        prev_spdif_active = spdif_active;
        prev_vbus_present = vbus_present;

        if source_changed {
            display_dirty = true;
        }

        // --- Handle state updates from tasks ---
        let mut pending_action: Option<ScreenAction> = None;
        if let Ok(update) = STATE_CHANNEL.try_receive() {
            match update {
                AppStateUpdate::VolumeChanged(vol) => {
                    // From USB volume control — always update amp
                    app_state.volume = vol;
                    if let Err(e) = amp.set_volume_percent(vol).await {
                        defmt::error!("Failed to set volume: {:?}", e);
                    }
                    if current_screen == ScreenId::Home {
                        volume_dirty = true;
                    }
                    save_pending = true;
                    save_deadline = Instant::now() + Duration::from_secs(2);
                }
                AppStateUpdate::MuteToggled(muted) => {
                    // From USB mute control — always update amp
                    app_state.muted = muted;
                    if muted {
                        let _ = amp.mute().await;
                    } else {
                        let _ = amp.unmute().await;
                    }
                    if current_screen == ScreenId::Home {
                        volume_dirty = true;
                    }
                    save_pending = true;
                    save_deadline = Instant::now() + Duration::from_secs(2);
                }
                AppStateUpdate::SampleRateChanged(rate) => {
                    app_state.sample_rate = rate;
                    if current_screen == ScreenId::Home {
                        display_dirty = true;
                    }
                }
                AppStateUpdate::EncoderRotate(dir) => {
                    if current_screen == ScreenId::Home {
                        // Volume: pass through every tick for fine control
                        pending_action =
                            home_screen.on_encoder_rotate(dir, &mut app_state);
                    } else {
                        // Menus: accumulate ticks, navigate every 2nd tick
                        menu_rotate_accum += dir;
                        if menu_rotate_accum >= 2 || menu_rotate_accum <= -2 {
                            let menu_dir = if menu_rotate_accum > 0 { 1 } else { -1 };
                            menu_rotate_accum = 0;
                            pending_action = match current_screen {
                                ScreenId::MainMenu => {
                                    main_menu_screen
                                        .on_encoder_rotate(menu_dir, &mut app_state)
                                }
                                ScreenId::SourceSelect => {
                                    source_select_screen
                                        .on_encoder_rotate(menu_dir, &mut app_state)
                                }
                                ScreenId::Equalizer => {
                                    equalizer_screen
                                        .on_encoder_rotate(menu_dir, &mut app_state)
                                }
                                ScreenId::Settings => {
                                    settings_screen
                                        .on_encoder_rotate(menu_dir, &mut app_state)
                                }
                                _ => None,
                            };
                        }
                    }
                }
                AppStateUpdate::EncoderPress => {
                    pending_action = match current_screen {
                        ScreenId::Home => home_screen.on_encoder_press(&mut app_state),
                        ScreenId::MainMenu => {
                            main_menu_screen.on_encoder_press(&mut app_state)
                        }
                        ScreenId::SourceSelect => {
                            source_select_screen.on_encoder_press(&mut app_state)
                        }
                        ScreenId::Equalizer => {
                            equalizer_screen.on_encoder_press(&mut app_state)
                        }
                        ScreenId::Settings => {
                            settings_screen.on_encoder_press(&mut app_state)
                        }
                        _ => None,
                    };
                }
                AppStateUpdate::EncoderLongPress => {
                    pending_action = match current_screen {
                        ScreenId::Home => {
                            home_screen.on_encoder_long_press(&mut app_state)
                        }
                        ScreenId::MainMenu => {
                            main_menu_screen.on_encoder_long_press(&mut app_state)
                        }
                        ScreenId::SourceSelect => {
                            source_select_screen.on_encoder_long_press(&mut app_state)
                        }
                        ScreenId::Equalizer => {
                            equalizer_screen.on_encoder_long_press(&mut app_state)
                        }
                        ScreenId::Settings => {
                            settings_screen.on_encoder_long_press(&mut app_state)
                        }
                        _ => None,
                    };
                }
                _ => {
                    display_dirty = true;
                }
            }
        }

        // --- Handle screen actions from encoder events ---
        if let Some(action) = pending_action {
            match action {
                ScreenAction::GoTo(screen) => {
                    current_screen = screen;
                    menu_rotate_accum = 0;
                    display_dirty = true;
                }
                ScreenAction::Back => {
                    current_screen = ScreenId::Home;
                    menu_rotate_accum = 0;
                    display_dirty = true;
                }
                ScreenAction::UpdateVolume(vol) => {
                    if let Err(e) = amp.set_volume_percent(vol).await {
                        defmt::error!("Failed to set volume: {:?}", e);
                    }
                    if current_screen == ScreenId::Home {
                        volume_dirty = true;
                    }
                    save_pending = true;
                    save_deadline = Instant::now() + Duration::from_secs(2);
                }
                ScreenAction::ToggleMute => {
                    app_state.muted = !app_state.muted;
                    if app_state.muted {
                        let _ = amp.mute().await;
                    } else {
                        let _ = amp.unmute().await;
                    }
                    if current_screen == ScreenId::Home {
                        volume_dirty = true;
                    }
                    save_pending = true;
                    save_deadline = Instant::now() + Duration::from_secs(2);
                }
                ScreenAction::ChangeSource(_source) => {
                    display_dirty = true;
                }
                ScreenAction::Refresh => {
                    display_dirty = true;
                }
                ScreenAction::EnterUsbConfigMode => {
                    enter_usb_config_mode();
                }
            }
        }

        // --- Level metering (every 30ms = 33fps for smooth bar animation) ---
        // Async I2C yields between page writes, so audio tasks aren't starved.
        display_refresh_counter += 1;
        if display_refresh_counter >= 30 {
            display_refresh_counter = 0;

            // Read and reset peak levels from Core 1
            let raw_left = PEAK_LEFT.swap(0, Ordering::Relaxed);
            let raw_right = PEAK_RIGHT.swap(0, Ordering::Relaxed);

            // Convert raw peaks to 0-100 using log2 for dB-like meter response.
            // Uses full 32-bit raw values for smooth low-end resolution.
            // 60dB range: log2(i32::MAX)≈31, minus 20 bits ≈ 60dB (6dB/bit).
            let level_left = log_meter(raw_left);
            let level_right = log_meter(raw_right);

            app_state.level_left = level_left;
            app_state.level_right = level_right;

            // Peak hold with slow decay
            peak_hold_left = peak_hold_left.max(level_left);
            peak_hold_right = peak_hold_right.max(level_right);

            peak_decay_counter += 1;
            if peak_decay_counter >= 2 {
                peak_decay_counter = 0;
                peak_hold_left = peak_hold_left.saturating_sub(1);
                peak_hold_right = peak_hold_right.saturating_sub(1);
            }

            app_state.peak_left = peak_hold_left;
            app_state.peak_right = peak_hold_right;

            // Clip detection with hold and flash
            let clipping = raw_left > CLIP_LEVEL || raw_right > CLIP_LEVEL;
            if clipping {
                clip_hold_counter = 33; // Hold CLIP for ~1s after last clip event
            }
            if clip_hold_counter > 0 {
                clip_hold_counter -= 1;
                app_state.clipping = true;
                clip_flash_counter += 1;
                if clip_flash_counter >= 6 {
                    // Flash at ~3Hz (toggle every ~300ms)
                    clip_flash_counter = 0;
                    clip_flash_on = !clip_flash_on;
                    //display_dirty = true;
                }
                app_state.clip_flash = clip_flash_on;
            } else {
                app_state.clipping = false;
                app_state.clip_flash = false;
                clip_flash_on = false;
                clip_flash_counter = 0;
            }

            // Update signal status based on actual audio content
            let new_signal_status = if app_state.clipping {
                SignalStatus::Clip
            } else if raw_left > 0 || raw_right > 0 {
                SignalStatus::Ok
            } else {
                SignalStatus::NoSignal
            };
            if new_signal_status != app_state.signal_status {
                app_state.signal_status = new_signal_status;
                display_dirty = true; // Status bar needs full redraw
            }

            // Partial update: only bar fill pixels (2 pages dirty instead of 6)
            if current_screen == ScreenId::Home {
                let _ = home_screen.draw_meters(&mut display, &app_state);

                // Only redraw volume/clip area when clip state actually changes
                if app_state.clip_flash != prev_clip_flash {
                    prev_clip_flash = app_state.clip_flash;
                    let _ = home_screen.draw_clip_region(&mut display, &app_state);
                }
            }
        }

        // Volume-only partial redraw (only the volume number region, ~2 pages)
        if volume_dirty && !display_dirty && current_screen == ScreenId::Home {
            volume_dirty = false;
            let _ = home_screen.draw_clip_region(&mut display, &app_state);
        }

        // Full redraw for UI text/state changes (source, sample rate, etc.)
        if display_dirty {
            display_dirty = false;
            volume_dirty = false;
            match current_screen {
                ScreenId::Home => {
                    let _ = home_screen.draw(&mut display, &app_state);
                }
                ScreenId::MainMenu => {
                    let _ = main_menu_screen.draw(&mut display, &app_state);
                }
                ScreenId::SourceSelect => {
                    let _ = source_select_screen.draw(&mut display, &app_state);
                }
                ScreenId::Equalizer => {
                    let _ = equalizer_screen.draw(&mut display, &app_state);
                }
                ScreenId::Settings => {
                    let _ = settings_screen.draw(&mut display, &app_state);
                }
                _ => {}
            }
        }

        // Flush at most ONE dirty page per tick (~2.5ms I2C write).
        // This spreads display updates over multiple ticks instead of
        // bursting all dirty pages at once, preventing audio stutter.
        let _ = display.flush_one_page().await;

        // Debounced settings save: write to flash 2s after last volume/mute change
        if save_pending && Instant::now() >= save_deadline {
            save_pending = false;
            settings::save(
                &mut flash,
                &PersistedSettings {
                    volume: app_state.volume,
                    muted: app_state.muted,
                },
            );
        }

        // Small yield to prevent busy-looping
        Timer::after(Duration::from_millis(1)).await;
    }
}

/// Encoder polling task — sends raw events to main loop for screen-aware handling
#[embassy_executor::task]
async fn encoder_task(mut encoder: RotaryEncoder<'static>) {
    use crate::drivers::encoder::EncoderEvent;

    defmt::info!("Encoder task started");

    loop {
        encoder.wait_for_edge().await;

        if let Some(event) = encoder.poll() {
            match event {
                EncoderEvent::Increment => {
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::EncoderRotate(1));
                }
                EncoderEvent::Decrement => {
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::EncoderRotate(-1));
                }
                EncoderEvent::Press => {
                    // Ignore button-down — wait for Release or LongPress
                    // so long-press (mute toggle) isn't preempted by
                    // short-press (menu open).
                }
                EncoderEvent::Release => {
                    // Short press (button-up within 500ms)
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::EncoderPress);
                }
                EncoderEvent::LongPress => {
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::EncoderLongPress);
                }
            }
        }
    }
}

/// Calculate output rate for S/PDIF input rate
///
/// Rate mapping strategy:
/// - 44.1k family (44.1, 88.2, 176.4) → 88.2 kHz output
/// - 48k family (48, 96, 192) → 96 kHz output
///
/// This means I2S output is always 88.2kHz or 96kHz, with simple 2x resampling:
/// - 44.1k → 88.2k (2x upsample)
/// - 48k → 96k (2x upsample)
/// - 88.2k → 88.2k (passthrough)
/// - 96k → 96k (passthrough)
/// - 176.4k → 88.2k (2x downsample)
/// - 192k → 96k (2x downsample)
fn calculate_output_rate(input_rate: u32) -> u32 {
    // Determine rate family by checking if divisible by 44100 base
    // 44.1k family: 44100, 88200, 176400
    // 48k family: 48000, 96000, 192000
    if input_rate % 44100 == 0 {
        88_200
    } else {
        96_000
    }
}

/// Calculate resampling mode for input/output rate pair
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResampleMode {
    Passthrough,
    Upsample2x,
    Downsample2x,
}

fn calculate_resample_mode(input_rate: u32, output_rate: u32) -> ResampleMode {
    if input_rate == output_rate {
        ResampleMode::Passthrough
    } else if input_rate * 2 == output_rate {
        ResampleMode::Upsample2x
    } else if input_rate == output_rate * 2 {
        ResampleMode::Downsample2x
    } else {
        // Shouldn't happen with our rate mapping, but fallback to passthrough
        ResampleMode::Passthrough
    }
}

/// S/PDIF receiver task with clock recovery
///
/// Monitors S/PDIF input for signal, detects sample rate, and streams audio
/// to the shared ring buffer. When S/PDIF is active, it takes priority over USB.
///
/// Uses clock recovery architecture:
/// - I2S output clock matches S/PDIF rate family (88.2k or 96k)
/// - Simple 2x up/downsampling when needed
/// - Adaptive clock adjustment on Core 1 keeps buffer stable
///
/// Critical timing: The PIO hardware FIFO is only 8 words deep (~83µs at 96kHz).
/// We use the same pattern as standalone spdif_to_i2s: run DMA while processing
/// the previous batch of data concurrently.
#[embassy_executor::task]
async fn spdif_task(
    mut pio0: embassy_rp::pio::Pio<'static, PIO0>,
    spdif_pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_3>,
    dma_ch: embassy_rp::Peri<'static, embassy_rp::peripherals::DMA_CH0>,
    fifo_buff: &'static mut [u32; SPDIF_RX_FIFO_SIZE],
) {
    use embassy_futures::join::join;

    defmt::info!("S/PDIF task started (clock recovery mode)");

    // Create S/PDIF receiver (PIO0, SM0)
    let mut spdif = SpdifRx::new(
        &mut pio0.common,
        pio0.sm0,
        dma_ch,
        spdif_pin,
        fifo_buff,
        0, // PIO0
    );

    // Buffers for audio processing - keep small for low latency
    const RAW_BUF_SIZE: usize = 64; // Match DMA block size
    const STEREO_BUF_SIZE: usize = RAW_BUF_SIZE / 2; // 32 stereo frames
    const OUTPUT_BUF_SIZE: usize = 128; // Room for 2x upsampling
    let mut raw_buffer = [0u32; RAW_BUF_SIZE];
    let mut stereo_buffer = [StereoFrame::ZERO; STEREO_BUF_SIZE];
    let mut output_buffer = [StereoFrame::ZERO; OUTPUT_BUF_SIZE];

    // Double buffer for previous batch processing while DMA runs
    let mut prev_raw_buffer = [0u32; RAW_BUF_SIZE];
    let mut prev_count: usize = 0;

    // Track state to avoid flooding STATE_CHANNEL
    let mut notified_stable = false;
    let mut mute = true;
    let mut current_resample_mode = ResampleMode::Passthrough;

    // Previous sample for 2x upsampling interpolation
    let mut prev_frame = StereoFrame::ZERO;

    loop {
        match spdif.state() {
            SpdifState::NoSignal => {
                // S/PDIF not active - allow USB to play
                SPDIF_ACTIVE.store(false, Ordering::Release);
                notified_stable = false;
                mute = true;
                prev_count = 0;
                prev_frame = StereoFrame::ZERO;

                // Try to detect signal
                if spdif.detect_signal(&mut pio0.common).await {
                    let sample_freq = spdif.sample_freq();
                    let input_rate = sample_freq.as_hz();
                    let output_rate = calculate_output_rate(input_rate);
                    current_resample_mode = calculate_resample_mode(input_rate, output_rate);

                    defmt::info!(
                        "S/PDIF detected: {} Hz -> {} Hz ({:?})",
                        input_rate,
                        output_rate,
                        defmt::Debug2Format(&current_resample_mode)
                    );

                    // Request I2S rate change
                    PENDING_I2S_RATE.store(output_rate, Ordering::Release);

                    // Notify main of sample rate (show input rate on display)
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::SampleRateChanged(input_rate));

                    // Start decoding
                    spdif.start_decode(&mut pio0.common);
                } else {
                    // No signal - wait before trying again
                    Timer::after(Duration::from_millis(100)).await;
                }
            }

            SpdifState::WaitingStable => {
                // Signal detected but not yet stable - keep processing DMA
                // IMPORTANT: Must also drain the software FIFO to prevent overflow!
                // We just don't write to the audio output ring buffer yet.

                // Drain software FIFO (discard samples during stabilization)
                while spdif.fifo_count() >= RAW_BUF_SIZE {
                    let _ = spdif.read_fifo(&mut raw_buffer);
                }

                // Continue filling from PIO via DMA
                let _ = spdif.process_dma(SPDIF_DMA_BLOCK_SIZE).await;

                // Check if signal was lost during stabilization
                if spdif.state() == SpdifState::NoSignal {
                    defmt::warn!("S/PDIF signal lost during stabilization");
                    spdif.reset();
                    let _ = STATE_CHANNEL.try_send(AppStateUpdate::SignalDetected(false));
                }
            }

            SpdifState::Stable => {
                // S/PDIF is stable - signal main loop via SPDIF_ACTIVE
                SPDIF_ACTIVE.store(true, Ordering::Release);

                if !notified_stable {
                    notified_stable = true;
                }

                // Unmute when software FIFO is reasonably full (like standalone code)
                let fifo_count = spdif.fifo_count();
                if mute && fifo_count >= SPDIF_RX_FIFO_SIZE / 2 {
                    mute = false;
                    defmt::info!("S/PDIF unmuted, FIFO: {}", fifo_count);
                }

                // Read from software FIFO into current buffer
                let count = if !mute && fifo_count >= RAW_BUF_SIZE {
                    spdif.read_fifo(&mut raw_buffer)
                } else {
                    0
                };

                // Process PREVIOUS batch while DMA runs for CURRENT batch
                // This is the key pattern from standalone spdif_to_i2s
                let process_fut = async {
                    if prev_count >= 2 {
                        // Extract stereo frames from S/PDIF words
                        let num_frames = prev_count / 2;
                        for i in 0..num_frames {
                            let left_word = prev_raw_buffer[i * 2];
                            let right_word = prev_raw_buffer[i * 2 + 1];
                            stereo_buffer[i] = StereoFrame::new(
                                crate::audio::extract_audio(left_word),
                                crate::audio::extract_audio(right_word),
                            );
                        }

                        // Apply resampling based on mode
                        let output_count = match current_resample_mode {
                            ResampleMode::Passthrough => {
                                // Direct copy
                                for i in 0..num_frames {
                                    output_buffer[i] = stereo_buffer[i];
                                }
                                num_frames
                            }
                            ResampleMode::Upsample2x => {
                                // 2x upsample with linear interpolation
                                let mut out_idx = 0;
                                for i in 0..num_frames {
                                    let curr = stereo_buffer[i];
                                    // Interpolated sample (midpoint)
                                    output_buffer[out_idx] = StereoFrame::new(
                                        (prev_frame.left / 2) + (curr.left / 2),
                                        (prev_frame.right / 2) + (curr.right / 2),
                                    );
                                    out_idx += 1;
                                    // Original sample
                                    output_buffer[out_idx] = curr;
                                    out_idx += 1;
                                    prev_frame = curr;
                                }
                                out_idx
                            }
                            ResampleMode::Downsample2x => {
                                // 2x downsample (take every other sample with simple averaging)
                                let mut out_idx = 0;
                                let mut i = 0;
                                while i + 1 < num_frames {
                                    let s0 = stereo_buffer[i];
                                    let s1 = stereo_buffer[i + 1];
                                    // Average of two consecutive samples
                                    output_buffer[out_idx] = StereoFrame::new(
                                        (s0.left / 2) + (s1.left / 2),
                                        (s0.right / 2) + (s1.right / 2),
                                    );
                                    out_idx += 1;
                                    i += 2;
                                }
                                out_idx
                            }
                        };

                        // Write to shared ring buffer if S/PDIF is the active source
                        if ACTIVE_SOURCE.load(Ordering::Relaxed) == SOURCE_SPDIF {
                            let input = unsafe { &mut AUDIO_INPUT };
                            for i in 0..output_count {
                                input.write_sample(output_buffer[i]);
                            }
                        }
                    }
                };

                // Run DMA and processing CONCURRENTLY - this is critical!
                // While DMA drains the PIO FIFO, we process the previous batch
                join(spdif.process_dma(SPDIF_DMA_BLOCK_SIZE), process_fut).await;

                // Save current buffer for next iteration's processing
                prev_raw_buffer[..count].copy_from_slice(&raw_buffer[..count]);
                prev_count = count;

                // Check if signal was lost
                if spdif.state() == SpdifState::NoSignal {
                    defmt::warn!("S/PDIF signal lost");
                    SPDIF_ACTIVE.store(false, Ordering::Release);
                    notified_stable = false;
                    mute = true;
                    prev_count = 0;
                    prev_frame = StereoFrame::ZERO;
                    spdif.reset();
                    // Main loop will detect SPDIF_ACTIVE falling edge and handle fallback
                }
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
    static SAMPLE_RATES: [u32; 1] = [USB_SAMPLE_RATE];
    static CHANNELS: [Channel; 2] = [Channel::LeftFront, Channel::RightFront];

    let (mut stream, _feedback, control) = Speaker::new(
        &mut builder,
        state,
        640,                     // max packet size with margin for 96kHz
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
    // Source selection is handled by main loop via ACTIVE_SOURCE.
    // This task always reads packets (to drain USB buffer) but only
    // writes to the ring buffer when USB is the active source.
    let audio_fut = async {
        let mut buf = [0u8; 640];
        loop {
            match stream.read_packet(&mut buf).await {
                Ok(n) => {
                    if n > 0 && ACTIVE_SOURCE.load(Ordering::Relaxed) == SOURCE_USB {
                        // Convert USB audio (24-bit packed stereo) to ring buffer
                        let samples = n / 6;
                        let input = unsafe { &mut AUDIO_INPUT };

                        for i in 0..samples {
                            let base = i * 6;
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

                            let frame = StereoFrame::new(left << 8, right << 8);
                            input.write_sample(frame);
                        }
                    }
                }
                Err(_) => {
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
                            let percent =
                                ((vol_db + 100.0) / 100.0 * 100.0).clamp(0.0, 100.0) as u8;
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

/// Core 1 entry point - dedicated audio processing with clock recovery
///
/// Runs a tight loop that:
/// 1. Drains I2S RX FIFO (line-in ADC) - always, to prevent overflow
/// 2. When line-in is active, writes RX data to the ring buffer
/// 3. Reads from ring buffer and writes to I2S TX
/// 4. Adaptively adjusts TX clock based on buffer level
///
/// I2S RX must be drained on Core 1 (not async Core 0) because the PIO RX FIFO
/// is only 8 words deep (~4 stereo frames at 96kHz = ~42µs). Async task scheduling
/// on Core 0 cannot guarantee the timing needed to prevent FIFO overflow.
fn core1_audio_main() -> ! {
    // Wait for Core 0 to signal that I2S is ready
    while !I2S_READY.load(Ordering::Acquire) {
        asm::nop();
    }

    // Take ownership of I2S TX and RX from statics
    // SAFETY: Core 0 has finished writing and signaled us
    let mut i2s_tx = unsafe { I2S_TX.take().unwrap() };
    let mut i2s_rx = unsafe { I2S_RX.take().unwrap() };

    defmt::info!("Core 1: I2S audio loop started (TX + RX)");

    // Start both transmitter and receiver
    i2s_tx.start();
    i2s_rx.start();

    // Counter for periodic buffer level check (don't check every sample)
    let mut check_counter: u32 = 0;
    const CHECK_INTERVAL: u32 = 256; // Check every 256 samples (~2.7ms at 96kHz)

    // Tight audio loop - runs forever on Core 1
    loop {
        // Check for rate change request from Core 0
        let pending = PENDING_I2S_RATE.load(Ordering::Acquire);
        if pending != 0 {
            // Clear the pending flag first
            PENDING_I2S_RATE.store(0, Ordering::Release);

            let current = i2s_tx.sample_rate();
            // Only reconfigure if rate is actually different
            if current != pending {
                defmt::info!("Core 1: Rate change {} -> {} Hz", current, pending);

                // Stop I2S TX, reconfigure, clear buffer, restart
                i2s_tx.stop();
                i2s_tx.set_sample_rate(pending);
                CURRENT_I2S_RATE.store(pending, Ordering::Release);
                unsafe {
                    AUDIO_INPUT.clear();
                }
                i2s_tx.start();

                // Reset check counter
                check_counter = 0;
            } else {
                defmt::debug!("Core 1: Already at {} Hz, skipping", pending);
            }
        }

        // Always drain I2S RX FIFO to prevent overflow/stall
        let active_source = ACTIVE_SOURCE.load(Ordering::Relaxed);

        if let Some((left, right)) = i2s_rx.try_read() {
            if active_source == SOURCE_LINEIN {
                // Line-in is the active source - write to ring buffer
                // Shift left by 1 to compensate for I2S "don't care" bit:
                // RX captures [don't_care, MSB, MSB-1, ..., bit1] because the first
                // sample after WCLK transition is before the PCM1822 drives MSB.
                // Shifting left moves MSB to bit 31, matching USB format.
                let input = unsafe { &mut AUDIO_INPUT };
                let frame = StereoFrame::new((left << 1) as i32, (right << 1) as i32);
                input.write_sample(frame);
            }
            // Otherwise: data is discarded (active source is writing)
        }

        // Periodic adaptive clock adjustment based on buffer level
        // Skip when line-in is active: RX and TX share the same PIO clock,
        // so there's no drift to compensate.
        check_counter = check_counter.wrapping_add(1);
        if check_counter >= CHECK_INTERVAL {
            check_counter = 0;

            if active_source == SOURCE_LINEIN {
                // Line-in: keep clock at nominal rate (no drift possible)
                i2s_tx.adjust_clock(ClockSpeed::Normal);
            } else {
                // S/PDIF or USB: adaptive clock for external source drift
                let available = unsafe { &AUDIO_INPUT }.available();
                let speed = match available {
                    0..=63 => ClockSpeed::Slow,     // Buffer low, slow down output
                    64..=191 => ClockSpeed::Normal, // Buffer OK (target ~128)
                    _ => ClockSpeed::Fast,          // Buffer high, speed up output
                };
                i2s_tx.adjust_clock(speed);
            }
        }

        // SAFETY: Core 1 is the only consumer of the input buffer
        let input = unsafe { &mut AUDIO_INPUT };

        if let Some(frame) = input.read_sample() {
            // Track peak levels for metering (absolute value, pre-EQ)
            let abs_l = (frame.left as i64).unsigned_abs() as u32;
            let abs_r = (frame.right as i64).unsigned_abs() as u32;
            PEAK_LEFT.fetch_max(abs_l, Ordering::Relaxed);
            PEAK_RIGHT.fetch_max(abs_r, Ordering::Relaxed);

            // Audio data available - output it
            // write() blocks until FIFO has space, naturally pacing output
            i2s_tx.write(frame.left as u32, frame.right as u32);
        } else {
            // No audio data - output silence
            // This keeps the I2S clock running continuously
            i2s_tx.write(0, 0);
        }
    }
}
