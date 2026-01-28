//! Screen definitions for the GUI

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};

use super::{
    clear_display, title_style, text_style,
    widgets::{AudioSource, LargeVolumeDisplay, StatusBar, VolumeBar},
    menu::{Menu, MenuAction, create_main_menu, create_source_menu},
    DISPLAY_HEIGHT,
};

/// Screen identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ScreenId {
    /// Boot/splash screen
    Boot,
    /// Home screen with volume display
    Home,
    /// Main menu
    MainMenu,
    /// Source selection
    SourceSelect,
    /// Equalizer settings
    Equalizer,
    /// System settings
    Settings,
    /// Volume adjustment overlay
    VolumeAdjust,
}

/// Application state shared with the GUI
#[derive(Debug, Clone)]
pub struct AppState {
    pub volume: u8,
    pub muted: bool,
    pub source: AudioSource,
    pub source_locked: bool,
    pub signal_present: bool,
    pub sample_rate: u32,
    pub eq_enabled: bool,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            volume: 50,
            muted: false,
            source: AudioSource::None,
            source_locked: false,
            signal_present: false,
            sample_rate: 96000,
            eq_enabled: false,
        }
    }
}

/// Actions that screens can request
#[derive(Debug, Clone, Copy)]
pub enum ScreenAction {
    /// Navigate to another screen
    GoTo(ScreenId),
    /// Go back to previous screen
    Back,
    /// Update volume on amplifier
    UpdateVolume(u8),
    /// Toggle mute
    ToggleMute,
    /// Change audio source
    ChangeSource(AudioSource),
    /// Refresh display
    Refresh,
}

/// Home screen - shows volume and source
pub struct HomeScreen {
    show_menu_hint: bool,
}

impl HomeScreen {
    pub fn new() -> Self {
        Self { show_menu_hint: true }
    }

    pub fn draw<D>(&self, display: &mut D, state: &AppState) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        clear_display(display)?;

        // Status bar at top
        let status = StatusBar::new(state.source, state.sample_rate, state.signal_present);
        status.draw(display)?;

        // Large volume display in center
        let vol_display = LargeVolumeDisplay::new(state.volume, state.muted);
        vol_display.draw(display)?;

        // Menu hint at bottom
        if self.show_menu_hint {
            Text::with_alignment(
                "Press for menu",
                Point::new(64, (DISPLAY_HEIGHT - 2) as i32),
                text_style(),
                Alignment::Center,
            )
            .draw(display)?;
        }

        Ok(())
    }

    pub fn on_encoder_rotate(&mut self, direction: i8, state: &mut AppState) -> Option<ScreenAction> {
        // Directly adjust volume
        let new_vol = if direction > 0 {
            state.volume.saturating_add(2).min(100)
        } else {
            state.volume.saturating_sub(2)
        };

        if new_vol != state.volume {
            state.volume = new_vol;
            Some(ScreenAction::UpdateVolume(new_vol))
        } else {
            None
        }
    }

    pub fn on_encoder_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::MainMenu))
    }

    pub fn on_encoder_long_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::ToggleMute)
    }
}

/// Main menu screen
pub struct MainMenuScreen {
    menu: Menu,
}

impl MainMenuScreen {
    pub fn new() -> Self {
        Self {
            menu: create_main_menu(),
        }
    }

    pub fn draw<D>(&self, display: &mut D, _state: &AppState) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        clear_display(display)?;

        Text::with_alignment(
            "MENU",
            Point::new(64, 12),
            title_style(),
            Alignment::Center,
        )
        .draw(display)?;

        self.menu.draw(display, 16)?;

        Ok(())
    }

    pub fn on_encoder_rotate(&mut self, direction: i8, _state: &mut AppState) -> Option<ScreenAction> {
        if direction > 0 {
            self.menu.select_next();
        } else {
            self.menu.select_previous();
        }
        Some(ScreenAction::Refresh)
    }

    pub fn on_encoder_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        match self.menu.selected_action() {
            MenuAction::AdjustVolume => Some(ScreenAction::GoTo(ScreenId::Home)),
            MenuAction::GoToScreen(screen) => Some(ScreenAction::GoTo(screen)),
            MenuAction::ToggleMute => Some(ScreenAction::ToggleMute),
            MenuAction::Back => Some(ScreenAction::Back),
            _ => None,
        }
    }

    pub fn on_encoder_long_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::Home))
    }
}

/// Source selection screen
pub struct SourceSelectScreen {
    menu: Menu,
}

impl SourceSelectScreen {
    pub fn new() -> Self {
        Self {
            menu: create_source_menu(),
        }
    }

    pub fn draw<D>(&self, display: &mut D, _state: &AppState) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        clear_display(display)?;

        Text::with_alignment(
            "SOURCE",
            Point::new(64, 12),
            title_style(),
            Alignment::Center,
        )
        .draw(display)?;

        self.menu.draw(display, 16)?;

        Ok(())
    }

    pub fn on_encoder_rotate(&mut self, direction: i8, _state: &mut AppState) -> Option<ScreenAction> {
        if direction > 0 {
            self.menu.select_next();
        } else {
            self.menu.select_previous();
        }
        Some(ScreenAction::Refresh)
    }

    pub fn on_encoder_press(&mut self, state: &mut AppState) -> Option<ScreenAction> {
        match self.menu.selected_action() {
            MenuAction::SelectSource(source) => {
                state.source = source;
                state.source_locked = source != AudioSource::None;
                Some(ScreenAction::ChangeSource(source))
            }
            MenuAction::Back => Some(ScreenAction::GoTo(ScreenId::MainMenu)),
            _ => None,
        }
    }

    pub fn on_encoder_long_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::Home))
    }
}

/// Boot screen with logo
pub struct BootScreen {
    progress: u8,
    status: &'static str,
}

impl BootScreen {
    pub fn new() -> Self {
        Self {
            progress: 0,
            status: "Starting...",
        }
    }

    pub fn set_progress(&mut self, progress: u8, status: &'static str) {
        self.progress = progress;
        self.status = status;
    }

    pub fn draw<D>(&self, display: &mut D, _state: &AppState) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        clear_display(display)?;

        Text::with_alignment(
            "OtterAmp",
            Point::new(64, 24),
            title_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Text::with_alignment(
            "DSP Speaker",
            Point::new(64, 38),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        let bar = super::widgets::ProgressBar::new(self.progress, self.status);
        bar.draw(display, Point::new(0, 48))?;

        Ok(())
    }
}

/// Placeholder for EQ screen
pub struct EqualizerScreen;

impl EqualizerScreen {
    pub fn new() -> Self {
        Self
    }

    pub fn draw<D>(&self, display: &mut D, _state: &AppState) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        clear_display(display)?;

        Text::with_alignment(
            "EQUALIZER",
            Point::new(64, 12),
            title_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Text::with_alignment(
            "Coming soon...",
            Point::new(64, 36),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Text::with_alignment(
            "Press to go back",
            Point::new(64, 56),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Ok(())
    }

    pub fn on_encoder_rotate(&mut self, _direction: i8, _state: &mut AppState) -> Option<ScreenAction> {
        None
    }

    pub fn on_encoder_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::MainMenu))
    }

    pub fn on_encoder_long_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::Home))
    }
}

/// Placeholder for Settings screen
pub struct SettingsScreen;

impl SettingsScreen {
    pub fn new() -> Self {
        Self
    }

    pub fn draw<D>(&self, display: &mut D, _state: &AppState) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        clear_display(display)?;

        Text::with_alignment(
            "SETTINGS",
            Point::new(64, 12),
            title_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Text::with_alignment(
            "Coming soon...",
            Point::new(64, 36),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Text::with_alignment(
            "Press to go back",
            Point::new(64, 56),
            text_style(),
            Alignment::Center,
        )
        .draw(display)?;

        Ok(())
    }

    pub fn on_encoder_rotate(&mut self, _direction: i8, _state: &mut AppState) -> Option<ScreenAction> {
        None
    }

    pub fn on_encoder_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::MainMenu))
    }

    pub fn on_encoder_long_press(&mut self, _state: &mut AppState) -> Option<ScreenAction> {
        Some(ScreenAction::GoTo(ScreenId::Home))
    }
}
