#pragma once

// ============================================================
// SkyTime Display Configuration
// ============================================================

// Firmware version shown on the local display and web interface.
#define SKYTIME_VERSION          "2.0 RC1.0"

// ST7789 physical panel dimensions.
#define DISPLAY_WIDTH             240
#define DISPLAY_HEIGHT            320

// Display refresh intervals.
#define DISPLAY_FAST_MS           100UL
#define DISPLAY_NORMAL_MS         250UL
#define DISPLAY_SLOW_MS           1000UL

// Backlight PWM and scheduled dimming.
#define BACKLIGHT_PWM_FREQUENCY   5000UL
#define BACKLIGHT_PWM_RESOLUTION  8
#define BACKLIGHT_PWM_MAX_DUTY    255U
#define DISPLAY_DIM_UPDATE_MS     1000UL

// RGB565 colors.
#define COLOR_BLACK               0x0000
#define COLOR_WHITE               0xFFFF
#define COLOR_GREEN               0x07E0
#define COLOR_RED                 0xF800
#define COLOR_YELLOW              0xFFE0
#define COLOR_CYAN                0x07FF
#define COLOR_LIGHTGRAY           0x8410
#define COLOR_ORANGE              0xFD20
