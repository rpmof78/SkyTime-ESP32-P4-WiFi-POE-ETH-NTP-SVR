#pragma once

#include <Arduino.h>
#include "skytime_types.h"

// Display hardware and backlight.
void display_startup();
void update_display_brightness(bool force = false);
void wake_display_for_configured_duration();
bool display_wake_override_active(uint32_t now_ms);

// Screen manager.
void screen_manager_init();
void screen_manager_handle_event(ScreenEvent event);
DisplayMode screen_manager_get_current();
uint32_t screen_manager_get_elapsed_ms();
bool screen_manager_take_redraw_required();

// Button handling, including boot-time commissioning request.
void detect_wifi_boot_button();
bool wait_for_wifi_boot_button_release();
void reset_button_state_after_boot_hold();
void handle_button();
void on_short_press();
void on_long_press();

// Display scheduler and cache.
void scheduler_update_display();
void update_display();
void clear_cache();
