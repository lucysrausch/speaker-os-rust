//! Menu system for navigation

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::Vec;

use super::{text_style, text_style_inverted, filled_style, DISPLAY_WIDTH};

/// Maximum number of menu items
pub const MAX_MENU_ITEMS: usize = 8;

/// Maximum visible items on screen
pub const VISIBLE_ITEMS: usize = 4;

/// Action triggered by menu selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuAction {
    /// Navigate to a screen
    GoToScreen(super::ScreenId),
    /// Adjust volume
    AdjustVolume,
    /// Toggle mute
    ToggleMute,
    /// Select audio source
    SelectSource(super::widgets::AudioSource),
    /// Enter EQ adjustment
    AdjustEq,
    /// Load DSP preset
    LoadDspPreset,
    /// System settings
    Settings,
    /// Go back to previous screen
    Back,
    /// No action
    None,
}

/// A menu item
#[derive(Debug, Clone)]
pub struct MenuItem {
    pub label: &'static str,
    pub action: MenuAction,
    pub enabled: bool,
}

impl MenuItem {
    pub const fn new(label: &'static str, action: MenuAction) -> Self {
        Self {
            label,
            action,
            enabled: true,
        }
    }

    pub const fn disabled(label: &'static str, action: MenuAction) -> Self {
        Self {
            label,
            action,
            enabled: false,
        }
    }
}

/// Menu widget with scrollable items
pub struct Menu {
    items: Vec<MenuItem, MAX_MENU_ITEMS>,
    selected: usize,
    scroll_offset: usize,
    title: &'static str,
}

impl Menu {
    /// Create a new menu with a title
    pub fn new(title: &'static str) -> Self {
        Self {
            items: Vec::new(),
            selected: 0,
            scroll_offset: 0,
            title,
        }
    }

    /// Add an item to the menu
    pub fn add_item(&mut self, item: MenuItem) {
        let _ = self.items.push(item);
    }

    /// Clear all items
    pub fn clear(&mut self) {
        self.items.clear();
        self.selected = 0;
        self.scroll_offset = 0;
    }

    /// Get the number of items
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Check if menu is empty
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Move selection up
    pub fn select_previous(&mut self) {
        if self.selected > 0 {
            self.selected -= 1;
            if self.selected < self.scroll_offset {
                self.scroll_offset = self.selected;
            }
        }
    }

    /// Move selection down
    pub fn select_next(&mut self) {
        if self.selected < self.items.len().saturating_sub(1) {
            self.selected += 1;
            if self.selected >= self.scroll_offset + VISIBLE_ITEMS {
                self.scroll_offset = self.selected - VISIBLE_ITEMS + 1;
            }
        }
    }

    /// Get the currently selected item
    pub fn selected_item(&self) -> Option<&MenuItem> {
        self.items.get(self.selected)
    }

    /// Get the action for the currently selected item
    pub fn selected_action(&self) -> MenuAction {
        self.selected_item()
            .map(|item| {
                if item.enabled {
                    item.action
                } else {
                    MenuAction::None
                }
            })
            .unwrap_or(MenuAction::None)
    }

    /// Get current selection index
    pub fn selected_index(&self) -> usize {
        self.selected
    }

    /// Set selection index
    pub fn set_selected(&mut self, index: usize) {
        if index < self.items.len() {
            self.selected = index;
            // Adjust scroll to keep selection visible
            if self.selected < self.scroll_offset {
                self.scroll_offset = self.selected;
            } else if self.selected >= self.scroll_offset + VISIBLE_ITEMS {
                self.scroll_offset = self.selected - VISIBLE_ITEMS + 1;
            }
        }
    }

    /// Draw the menu
    pub fn draw<D>(&self, display: &mut D, y_offset: i32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let item_height = 12i32;
        let start_y = y_offset;

        // Draw visible items
        for (i, item) in self
            .items
            .iter()
            .enumerate()
            .skip(self.scroll_offset)
            .take(VISIBLE_ITEMS)
        {
            let y = start_y + ((i - self.scroll_offset) as i32 * item_height);
            let is_selected = i == self.selected;

            if is_selected {
                // Draw selection highlight
                Rectangle::new(
                    Point::new(0, y),
                    Size::new(DISPLAY_WIDTH, item_height as u32),
                )
                .into_styled(filled_style())
                .draw(display)?;

                // Draw text inverted
                Text::new(
                    item.label,
                    Point::new(4, y + 10),
                    text_style_inverted(),
                )
                .draw(display)?;
            } else {
                // Draw normal text
                let style = if item.enabled {
                    text_style()
                } else {
                    // Disabled items could use a different style
                    // For monochrome, we just use the same style
                    text_style()
                };
                Text::new(item.label, Point::new(4, y + 10), style).draw(display)?;
            }
        }

        // Draw scroll indicators if needed
        if self.scroll_offset > 0 {
            // Up arrow indicator
            Text::new("^", Point::new(120, start_y + 8), text_style()).draw(display)?;
        }

        if self.scroll_offset + VISIBLE_ITEMS < self.items.len() {
            // Down arrow indicator
            let y = start_y + (VISIBLE_ITEMS as i32 - 1) * item_height;
            Text::new("v", Point::new(120, y + 8), text_style()).draw(display)?;
        }

        Ok(())
    }
}

/// Main menu items
pub fn create_main_menu() -> Menu {
    let mut menu = Menu::new("Menu");
    menu.add_item(MenuItem::new("Volume", MenuAction::AdjustVolume));
    menu.add_item(MenuItem::new("Source", MenuAction::GoToScreen(super::ScreenId::SourceSelect)));
    menu.add_item(MenuItem::new("EQ", MenuAction::GoToScreen(super::ScreenId::Equalizer)));
    menu.add_item(MenuItem::new("Settings", MenuAction::GoToScreen(super::ScreenId::Settings)));
    menu
}

/// Source selection menu items
pub fn create_source_menu() -> Menu {
    use super::widgets::AudioSource;

    let mut menu = Menu::new("Source");
    menu.add_item(MenuItem::new("Auto", MenuAction::SelectSource(AudioSource::None)));
    menu.add_item(MenuItem::new("Line In", MenuAction::SelectSource(AudioSource::LineIn)));
    menu.add_item(MenuItem::new("S/PDIF", MenuAction::SelectSource(AudioSource::Spdif)));
    menu.add_item(MenuItem::new("USB Audio", MenuAction::SelectSource(AudioSource::Usb)));
    menu.add_item(MenuItem::new("< Back", MenuAction::Back));
    menu
}
