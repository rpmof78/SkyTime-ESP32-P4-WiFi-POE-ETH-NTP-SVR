#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <time.h>

#include "board_pins.h"
#include "display_config.h"
#include "skytime_types.h"
#include "display_manager.h"

#define BUTTON_DEBOUNCE_MS              40UL
#define LONG_PRESS_TIME_MS              5000UL
#define WIFI_BOOT_BUTTON_DEBOUNCE_MS    100UL
#define WIFI_BOOT_RELEASE_TIMEOUT_MS    10000UL

// Runtime instances remain owned and initialized by main.cpp.
extern Arduino_GFX *gfx;

extern GPSData gps_data;
extern PPSData pps_data;
extern ButtonState button_state;
extern DisplayCache display_cache;
extern ScreenManager screen_manager;

extern NetworkConfig network_config;
extern WifiApRuntime wifi_ap_runtime;
extern EthernetRuntime ethernet_runtime;
extern NtpRuntime ntp_runtime;
extern Udp123Debug udp123_debug;
extern SdRuntime sd_runtime;
extern SystemConfig system_config;
extern DisplayDimmingRuntime display_dimming;
extern ConfigRuntime config_runtime;
extern WebRuntime web_runtime;

extern portMUX_TYPE state_mux;

extern uint32_t last_display_fast_ms;
extern uint32_t last_display_normal_ms;
extern uint32_t last_display_slow_ms;

extern bool boot_is_power_on_reset;
extern bool wifi_boot_button_requested;
extern bool wifi_boot_button_released;

// Cross-subsystem accessors remain implemented in their current modules.
extern void snapshot_ntp_timing(NtpTimingSnapshot *snapshot);
extern uint32_t makeTimeUTC(
  int year,
  int month,
  int day,
  int hour,
  int minute,
  int second
);
extern uint32_t atomic_read_pps_rejected();
extern bool atomic_take_button_irq();
extern void save_uptime_checkpoint_now();

extern const char *ethernet_state_text(EthernetState state);
extern const char *web_state_text(WebState state);
extern const char *config_state_text(ConfigState state);
extern const char *sd_state_text(SdCardState state);

bool get_local_minutes(uint16_t *local_minutes) {
  if (!local_minutes ||
      !gps_data.time_valid ||
      !gps_data.date_valid) {
    return false;
  }

  int32_t minutes =
    (int32_t)gps_data.hour * 60 +
    (int32_t)gps_data.minute +
    system_config.local_utc_offset_minutes;

  minutes %= 1440;

  if (minutes < 0) {
    minutes += 1440;
  }

  *local_minutes = (uint16_t)minutes;
  return true;
}

bool display_dim_schedule_active(
  uint16_t local_minutes,
  uint16_t start_minutes,
  uint16_t stop_minutes
) {
  if (start_minutes == stop_minutes) {
    return false;
  }

  if (start_minutes < stop_minutes) {
    return local_minutes >= start_minutes &&
           local_minutes < stop_minutes;
  }

  return local_minutes >= start_minutes ||
         local_minutes < stop_minutes;
}

void set_display_brightness_percent(
  uint8_t brightness_percent
) {
  if (brightness_percent > 100) {
    brightness_percent = 100;
  }

  if (display_dimming.brightness_percent ==
      brightness_percent) {
    return;
  }

  display_dimming.brightness_percent =
    brightness_percent;

  uint32_t duty =
    ((uint32_t)brightness_percent *
     BACKLIGHT_PWM_MAX_DUTY + 50U) / 100U;

  if (display_dimming.pwm_attached) {
    ledcWrite(BL_PIN, duty);
  } else {
    digitalWrite(
      BL_PIN,
      brightness_percent > 0 ? HIGH : LOW
    );
  }
}

bool display_wake_override_active(uint32_t now_ms) {
  uint32_t wake_until_ms =
    display_dimming.wake_until_ms;

  return wake_until_ms != 0 &&
         (int32_t)(wake_until_ms - now_ms) > 0;
}

void wake_display_for_configured_duration() {
  uint16_t wake_seconds = 0;

  portENTER_CRITICAL(&state_mux);
  wake_seconds =
    system_config.night_dim_wake_seconds;
  portEXIT_CRITICAL(&state_mux);

  if (wake_seconds == 0 ||
      !display_dimming.schedule_active) {
    return;
  }

  display_dimming.wake_until_ms =
    millis() +
    (uint32_t)wake_seconds * 1000UL;

  set_display_brightness_percent(100);
}

void update_display_brightness(bool force) {
  uint32_t now_ms = millis();

  if (!force &&
      display_dimming.last_update_ms != 0 &&
      now_ms - display_dimming.last_update_ms <
        DISPLAY_DIM_UPDATE_MS) {
    return;
  }

  display_dimming.last_update_ms = now_ms;

  bool enabled = false;
  uint16_t start_minutes = 0;
  uint16_t stop_minutes = 0;
  uint8_t dim_percent = 0;

  portENTER_CRITICAL(&state_mux);
  enabled = system_config.night_dim_enabled;
  start_minutes =
    system_config.night_dim_start_minutes;
  stop_minutes =
    system_config.night_dim_stop_minutes;
  dim_percent =
    system_config.night_dim_percent;
  portEXIT_CRITICAL(&state_mux);

  uint16_t local_minutes = 0;
  bool local_time_valid =
    get_local_minutes(&local_minutes);

  bool active =
    enabled &&
    local_time_valid &&
    display_dim_schedule_active(
      local_minutes,
      start_minutes,
      stop_minutes
    );

  bool wake_active =
    active &&
    display_wake_override_active(now_ms);

  if (!active) {
    display_dimming.wake_until_ms = 0;
  } else if (!wake_active &&
             display_dimming.wake_until_ms != 0) {
    display_dimming.wake_until_ms = 0;
  }

  uint8_t brightness_percent =
    wake_active ?
    100U :
    (active ?
      (uint8_t)(100U - dim_percent) :
      100U);

  display_dimming.local_time_valid =
    local_time_valid;
  display_dimming.local_minutes =
    local_minutes;
  display_dimming.schedule_active =
    active;

  set_display_brightness_percent(
    brightness_percent
  );
}

void display_startup() {
  gfx->fillScreen(COLOR_BLACK);

  gfx->setTextSize(3);
  gfx->setTextColor(COLOR_CYAN, COLOR_BLACK);
  gfx->setCursor(82, 45);
  gfx->println("SkyTime");

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE, COLOR_BLACK);
  gfx->setCursor(90, 92);
  gfx->print("Version ");
  gfx->println(SKYTIME_VERSION);

  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
  gfx->setCursor(70, 135);
  gfx->println("GPS/PPS NTP Server");

  gfx->setCursor(70, 155);
  gfx->println("Config Enabled");

  gfx->setCursor(70, 190);
  gfx->println("Short: screen   Long: reboot");

  delay(1500);

  display_cache.force_full_redraw = true;
}

void screen_manager_init() {
  portENTER_CRITICAL(&state_mux);
  screen_manager.current = PRIMARY_SCREEN;
  screen_manager.previous = PRIMARY_SCREEN;
  screen_manager.entered_ms = millis();
  screen_manager.timed_screen = false;
  screen_manager.redraw_required = true;
  portEXIT_CRITICAL(&state_mux);
}

void screen_manager_handle_event(ScreenEvent event) {
  uint32_t now_ms = millis();

  portENTER_CRITICAL(&state_mux);

  if (event == SCREEN_EVENT_FORCE_MAIN) {
    screen_manager.previous = screen_manager.current;
    screen_manager.current = PRIMARY_SCREEN;
    screen_manager.entered_ms = now_ms;
    screen_manager.timed_screen = false;
    screen_manager.redraw_required = true;
    portEXIT_CRITICAL(&state_mux);
    return;
  }

  if (event == SCREEN_EVENT_TIMEOUT) {
    if (screen_manager.timed_screen) {
      screen_manager.previous = screen_manager.current;
      screen_manager.current = PRIMARY_SCREEN;
      screen_manager.entered_ms = now_ms;
      screen_manager.timed_screen = false;
      screen_manager.redraw_required = true;
    }
    portEXIT_CRITICAL(&state_mux);
    return;
  }

  if (event == SCREEN_EVENT_SHORT_PRESS) {
    screen_manager.previous = screen_manager.current;

    if (screen_manager.current == PRIMARY_SCREEN) {
      screen_manager.current = NETWORK_SCREEN;
      screen_manager.timed_screen = true;
    } else if (screen_manager.current == NETWORK_SCREEN) {
      screen_manager.current = DIAGNOSTIC_SCREEN;
      screen_manager.timed_screen = true;
    } else {
      screen_manager.current = PRIMARY_SCREEN;
      screen_manager.timed_screen = false;
    }

    screen_manager.entered_ms = now_ms;
    screen_manager.redraw_required = true;
  }

  portEXIT_CRITICAL(&state_mux);
}

DisplayMode screen_manager_get_current() {
  DisplayMode mode;
  portENTER_CRITICAL(&state_mux);
  mode = screen_manager.current;
  portEXIT_CRITICAL(&state_mux);
  return mode;
}

uint32_t screen_manager_get_elapsed_ms() {
  uint32_t entered;
  portENTER_CRITICAL(&state_mux);
  entered = screen_manager.entered_ms;
  portEXIT_CRITICAL(&state_mux);
  return millis() - entered;
}

bool screen_manager_take_redraw_required() {
  bool value;
  portENTER_CRITICAL(&state_mux);
  value = screen_manager.redraw_required;
  screen_manager.redraw_required = false;
  portEXIT_CRITICAL(&state_mux);
  return value;
}

void detect_wifi_boot_button() {
  wifi_boot_button_requested = false;
  wifi_boot_button_released = false;

  if (!boot_is_power_on_reset) {
    Serial.println("[BUTTON] Wi-Fi boot request ignored: not POWERON");
    return;
  }

  if (digitalRead(BUTTON_PIN) != LOW) {
    Serial.println("[BUTTON] Boot button released: Wi-Fi remains off");
    return;
  }

  delay(WIFI_BOOT_BUTTON_DEBOUNCE_MS);

  if (digitalRead(BUTTON_PIN) == LOW) {
    wifi_boot_button_requested = true;
    Serial.println("[BUTTON] Boot hold detected");
    Serial.println("[WIFI-AP] Commissioning requested by boot button");
  } else {
    Serial.println("[BUTTON] Boot hold rejected by debounce");
  }
}

void reset_button_state_after_boot_hold() {
  portENTER_CRITICAL(&state_mux);
  button_state.irq_pending = false;
  button_state.state = BUTTON_IDLE;
  button_state.transition_ms = millis();
  button_state.press_start_ms = 0;
  button_state.long_press_reported = false;
  portEXIT_CRITICAL(&state_mux);
}

bool wait_for_wifi_boot_button_release() {
  if (!wifi_boot_button_requested) {
    return false;
  }

  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextColor(COLOR_WHITE, COLOR_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(58, 105);
  gfx->println("Wi-Fi Setup");
  gfx->setCursor(64, 137);
  gfx->println("Requested");
  gfx->setTextSize(1);
  gfx->setCursor(82, 178);
  gfx->println("Release Button");

  Serial.println("[WIFI-AP] Waiting for button release");

  uint32_t wait_started_ms = millis();

  while (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - wait_started_ms >= WIFI_BOOT_RELEASE_TIMEOUT_MS) {
      Serial.println("[WIFI-AP] Boot button release timeout");
      Serial.println("[WIFI-AP] Commissioning request cancelled");

      wifi_boot_button_requested = false;
      wifi_boot_button_released = false;
      reset_button_state_after_boot_hold();
      return false;
    }

    delay(10);
  }

  delay(WIFI_BOOT_BUTTON_DEBOUNCE_MS);

  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("[WIFI-AP] Button release was not stable");
    Serial.println("[WIFI-AP] Commissioning request cancelled");

    wifi_boot_button_requested = false;
    wifi_boot_button_released = false;
    reset_button_state_after_boot_hold();
    return false;
  }

  wifi_boot_button_released = true;
  reset_button_state_after_boot_hold();

  Serial.println("[BUTTON] Boot button released");
  Serial.println("[WIFI-AP] Five-minute commissioning window authorized");

  display_cache.force_full_redraw = true;
  return true;
}

void handle_button() {
  uint32_t now_ms = millis();
  bool level_low = (digitalRead(BUTTON_PIN) == LOW);

  bool irq_pending = atomic_take_button_irq();

  if (irq_pending &&
      button_state.state == BUTTON_IDLE) {
    button_state.state = BUTTON_DEBOUNCE_PRESS;
    button_state.transition_ms = now_ms;
  }

  switch (button_state.state) {
    case BUTTON_IDLE:
      break;

    case BUTTON_DEBOUNCE_PRESS:
      if ((now_ms - button_state.transition_ms) >= BUTTON_DEBOUNCE_MS) {
        if (level_low) {
          button_state.state = BUTTON_HELD;
          button_state.press_start_ms = now_ms;
          button_state.long_press_reported = false;
        } else {
          button_state.state = BUTTON_IDLE;
        }
      }
      break;

    case BUTTON_HELD:
      if (!level_low) {
        button_state.state = BUTTON_DEBOUNCE_RELEASE;
        button_state.transition_ms = now_ms;
      } else if (!button_state.long_press_reported &&
                 (now_ms - button_state.press_start_ms) >= LONG_PRESS_TIME_MS) {
        button_state.long_press_reported = true;
        on_long_press();
      }
      break;

    case BUTTON_DEBOUNCE_RELEASE:
      if ((now_ms - button_state.transition_ms) >= BUTTON_DEBOUNCE_MS) {
        if (!level_low) {
          if (!button_state.long_press_reported) {
            on_short_press();
          }
          button_state.state = BUTTON_IDLE;
        } else {
          button_state.state = BUTTON_HELD;
        }
      }
      break;
  }
}

void on_short_press() {
  bool display_was_dimmed =
    display_dimming.schedule_active &&
    display_dimming.brightness_percent < 100;

  wake_display_for_configured_duration();

  screen_manager_handle_event(
    display_was_dimmed ?
      SCREEN_EVENT_FORCE_MAIN :
      SCREEN_EVENT_SHORT_PRESS
  );

  display_cache.force_full_redraw = true;
}

void on_long_press() {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_RED, COLOR_BLACK);
  gfx->setCursor(50, 110);
  gfx->println("REBOOTING...");
  delay(500);
  save_uptime_checkpoint_now();
  delay(50);
  ESP.restart();
}

void clear_cache() {
  memset(&display_cache, 0, sizeof(display_cache));
  display_cache.active_mode = PRIMARY_SCREEN;
  display_cache.drawn_mode = PRIMARY_SCREEN;
  display_cache.force_full_redraw = true;
}

void draw_field_if_changed(
  int16_t x,
  int16_t y,
  int16_t w,
  int16_t h,
  uint8_t text_size,
  uint16_t color,
  const char *new_text,
  char *cache_text
) {
  if (!display_cache.force_full_redraw &&
      strcmp(new_text, cache_text) == 0) {
    return;
  }

  gfx->fillRect(x, y, w, h, COLOR_BLACK);
  gfx->setTextSize(text_size);
  gfx->setTextColor(color, COLOR_BLACK);
  gfx->setCursor(x, y);
  gfx->print(new_text);

  strncpy(cache_text, new_text, 39);
  cache_text[39] = '\0';
}

void draw_screen_static(DisplayMode mode) {
  gfx->fillScreen(COLOR_BLACK);

  if (mode == PRIMARY_SCREEN) {
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);

    gfx->setCursor(60, 22);
    gfx->print("Satellites");

    gfx->setCursor(60, 54);
    gfx->print("PPS");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
    gfx->setCursor(57, 218);
    gfx->print("Short : Screens  | Long : Reboot");
  }

  if (mode == NETWORK_SCREEN) {
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_CYAN, COLOR_BLACK);
    gfx->setCursor(118, 20);
    gfx->print("NETWORK");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
    gfx->setCursor(62, 210);
    gfx->print("Short: Diag | Long: Reboot");
  }

  if (mode == DIAGNOSTIC_SCREEN) {
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_CYAN, COLOR_BLACK);
    gfx->setCursor(94, 15);
    gfx->print("DIAGNOSTICS");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
    gfx->setCursor(62, 220);
    gfx->print("Short: Main | Long: Reboot");
  }

  display_cache.drawn_mode = mode;
  display_cache.force_full_redraw = true;

  memset(display_cache.local_time_text, 0, sizeof(display_cache.local_time_text));
  memset(display_cache.utc_time_text, 0, sizeof(display_cache.utc_time_text));
  memset(display_cache.date_text, 0, sizeof(display_cache.date_text));
  memset(display_cache.pps_us_text, 0, sizeof(display_cache.pps_us_text));
  memset(display_cache.pps_interval_text, 0, sizeof(display_cache.pps_interval_text));
  memset(display_cache.epoch_text, 0, sizeof(display_cache.epoch_text));
  memset(display_cache.fraction_text, 0, sizeof(display_cache.fraction_text));
  memset(display_cache.location_lat_text, 0, sizeof(display_cache.location_lat_text));
  memset(display_cache.location_lon_text, 0, sizeof(display_cache.location_lon_text));
  memset(display_cache.altitude_text, 0, sizeof(display_cache.altitude_text));
  memset(display_cache.countdown_text, 0, sizeof(display_cache.countdown_text));
  memset(display_cache.diagnostic_1, 0, sizeof(display_cache.diagnostic_1));
  memset(display_cache.diagnostic_2, 0, sizeof(display_cache.diagnostic_2));
  memset(display_cache.diagnostic_3, 0, sizeof(display_cache.diagnostic_3));
  memset(display_cache.diagnostic_4, 0, sizeof(display_cache.diagnostic_4));
  memset(display_cache.diagnostic_5, 0, sizeof(display_cache.diagnostic_5));
  memset(display_cache.diagnostic_6, 0, sizeof(display_cache.diagnostic_6));
}

void update_primary_fast_fields() {
  char text[32];
  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  snprintf(
    text,
    sizeof(text),
    "%lu us",
    (unsigned long)timing.microseconds_since_pps
  );

  draw_field_if_changed(
    57,
    188,
    230,
    16,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.pps_us_text
  );
}

void update_primary_normal_fields() {
  char text[40];

  snprintf(
    text,
    sizeof(text),
    "%u",
    gps_data.satellites
  );

  draw_field_if_changed(
    197,
    22,
    36,
    22,
    2,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_1
  );

  uint16_t gps_dot_color = COLOR_RED;
  if (gps_data.state == GPS_LOCKED) {
    gps_dot_color = COLOR_GREEN;
  } else if (gps_data.state == GPS_HOLDOVER) {
    gps_dot_color = COLOR_ORANGE;
  } else if (gps_data.satellites > 0) {
    gps_dot_color = COLOR_YELLOW;
  }

  static GPSLockState last_dot_state = GPS_STALE;
  static uint8_t last_dot_sats = 255;

  if (display_cache.force_full_redraw ||
      last_dot_state != gps_data.state ||
      last_dot_sats != gps_data.satellites) {
    // Clear both the old and new dot regions in case a previous build left
    // the indicator lower on the line.
    gfx->fillRect(240, 18, 32, 44, COLOR_BLACK);

    // Dot aligned with the shifted Satellites line.
    gfx->fillCircle(254, 32, 7, gps_dot_color);
    gfx->drawCircle(254, 32, 7, COLOR_WHITE);

    last_dot_state = gps_data.state;
    last_dot_sats = gps_data.satellites;
  }

  const char *pps_text = "WAIT";
  uint16_t pps_color = COLOR_YELLOW;

  if (pps_data.state == PPS_LOCKED) {
    pps_text = "LOCKED";
    pps_color = COLOR_GREEN;
  } else if (pps_data.state == PPS_BAD_INTERVAL) {
    pps_text = "BAD";
    pps_color = COLOR_YELLOW;
  } else if (pps_data.state == PPS_TIMEOUT) {
    pps_text = "LOST";
    pps_color = COLOR_RED;
  } else if (pps_data.state == PPS_ALIGN_WAIT) {
    pps_text = "ALIGN";
    pps_color = COLOR_ORANGE;
  } else if (pps_data.state == PPS_ALIGN_BAD) {
    pps_text = "GPS?";
    pps_color = COLOR_ORANGE;
  }

  draw_field_if_changed(
    157,
    54,
    120,
    22,
    2,
    pps_color,
    pps_text,
    display_cache.diagnostic_2
  );

  struct tm tm_local = {};
  bool local_time_valid = gps_data.time_valid && gps_data.date_valid;

  if (local_time_valid) {
    uint32_t utc_unix = makeTimeUTC(
      gps_data.year,
      gps_data.month,
      gps_data.day,
      gps_data.hour,
      gps_data.minute,
      gps_data.second
    );

    int64_t adjusted_unix =
      (int64_t)utc_unix +
      ((int64_t)system_config.local_utc_offset_minutes * 60LL);

    time_t local_raw = (time_t)adjusted_unix;
    gmtime_r(&local_raw, &tm_local);

    snprintf(
      text,
      sizeof(text),
      "LOCAL %02d:%02d:%02d",
      tm_local.tm_hour,
      tm_local.tm_min,
      tm_local.tm_sec
    );
  } else {
    snprintf(text, sizeof(text), "LOCAL --:--:--");
  }

  draw_field_if_changed(
    34,
    86,
    252,
    30,
    3,
    COLOR_WHITE,
    text,
    display_cache.local_time_text
  );

  snprintf(
    text,
    sizeof(text),
    "UTC %02d:%02d:%02d",
    gps_data.hour,
    gps_data.minute,
    gps_data.second
  );

  draw_field_if_changed(
    82,
    120,
    168,
    22,
    2,
    COLOR_CYAN,
    text,
    display_cache.utc_time_text
  );

  if (local_time_valid) {
    snprintf(
      text,
      sizeof(text),
      "%02d/%02d/%04d",
      tm_local.tm_mon + 1,
      tm_local.tm_mday,
      tm_local.tm_year + 1900
    );
  } else {
    snprintf(text, sizeof(text), "00/00/0000");
  }

  draw_field_if_changed(
    100,
    150,
    120,
    22,
    2,
    COLOR_WHITE,
    text,
    display_cache.date_text
  );
}

void update_network_screen() {
  char text[40];

  snprintf(text, sizeof(text), "Ethernet: %s",
           ethernet_state_text(ethernet_runtime.state));
  draw_field_if_changed(
    62,
    58,
    220,
    14,
    1,
    ethernet_runtime.got_ip ? COLOR_GREEN :
      (ethernet_runtime.link_up ? COLOR_YELLOW : COLOR_RED),
    text,
    display_cache.diagnostic_1
  );

  snprintf(text, sizeof(text), "IP: %s",
           ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
  draw_field_if_changed(
    62,
    78,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.location_lat_text
  );

  if (wifi_ap_runtime.started) {
    snprintf(
      text,
      sizeof(text),
      "Setup AP: UP C:%u",
      wifi_ap_runtime.clients
    );
  } else if (wifi_ap_runtime.window_expired) {
    snprintf(text, sizeof(text), "Setup AP: OFF");
  } else {
    snprintf(text, sizeof(text), "Setup AP: DISABLED");
  }
  draw_field_if_changed(
    62,
    98,
    220,
    14,
    1,
    wifi_ap_runtime.started ? COLOR_GREEN : COLOR_RED,
    text,
    display_cache.location_lon_text
  );

  if (wifi_ap_runtime.started) {
    snprintf(
      text,
      sizeof(text),
      "AP IP:%s T:%lus",
      wifi_ap_runtime.ip,
      (unsigned long)wifi_ap_runtime.remaining_seconds
    );
  } else if (wifi_ap_runtime.window_expired) {
    snprintf(text, sizeof(text), "Window expired");
  } else {
    snprintf(text, sizeof(text), "Power cycle to enable");
  }
  draw_field_if_changed(
    62,
    118,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.altitude_text
  );

  snprintf(text, sizeof(text), "System: %s",
           system_config.node_id);
  draw_field_if_changed(
    62,
    138,
    220,
    14,
    1,
    COLOR_CYAN,
    text,
    display_cache.diagnostic_2
  );

  snprintf(text, sizeof(text), "NTP RX:%lu TX:%lu",
           (unsigned long)udp123_debug.ntp_rx,
           (unsigned long)ntp_runtime.packets_tx);
  draw_field_if_changed(
    62,
    158,
    220,
    14,
    1,
    udp123_debug.ntp_rx > 0 ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.diagnostic_3
  );

  snprintf(text, sizeof(text), "Web:%s Cfg:%s",
           web_state_text(web_runtime.state),
           config_state_text(config_runtime.state));
  draw_field_if_changed(
    62,
    174,
    220,
    14,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.diagnostic_4
  );

  snprintf(text, sizeof(text), "SD: %s %s",
           sd_state_text(sd_runtime.state),
           sd_runtime.card_type);
  draw_field_if_changed(
    62,
    190,
    220,
    14,
    1,
    sd_runtime.state == SD_STATE_MOUNTED ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.diagnostic_4
  );

  uint32_t elapsed = screen_manager_get_elapsed_ms();
  uint32_t screen_timeout_ms = system_config.screen_timeout_seconds * 1000UL;
  uint32_t remaining =
    elapsed >= screen_timeout_ms ?
    0 :
    screen_timeout_ms - elapsed;

  snprintf(
    text,
    sizeof(text),
    "Auto-return: %lus",
    (unsigned long)(remaining / 1000UL)
  );

  draw_field_if_changed(
    62,
    208,
    220,
    14,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.countdown_text
  );
}

void update_diagnostic_screen() {
  char text[40];
  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  snprintf(
    text,
    sizeof(text),
    "GPS: %s SAT:%u",
    gps_data.state == GPS_LOCKED ? "LOCK" :
      (gps_data.state == GPS_HOLDOVER ? "HOLD" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites
  );
  draw_field_if_changed(
    62,
    52,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_1
  );

  snprintf(
    text,
    sizeof(text),
    "PPS count: %lu",
    (unsigned long)pps_data.pps_count
  );
  draw_field_if_changed(
    62,
    72,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_2
  );

  snprintf(
    text,
    sizeof(text),
    "PPS interval: %lu us",
    (unsigned long)pps_data.last_interval_us
  );
  draw_field_if_changed(
    62,
    92,
    220,
    14,
    1,
    pps_data.state == PPS_LOCKED ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.pps_interval_text
  );

  snprintf(
    text,
    sizeof(text),
    "NTP epoch: %lu",
    (unsigned long)timing.current_epoch
  );
  draw_field_if_changed(
    62,
    112,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.epoch_text
  );

  snprintf(
    text,
    sizeof(text),
    "NTP frac: 0x%08lX",
    (unsigned long)timing.ntp_fraction
  );
  draw_field_if_changed(
    62,
    132,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.fraction_text
  );

  snprintf(
    text,
    sizeof(text),
    "Good:%lu Bad:%lu Rj:%lu",
    (unsigned long)pps_data.valid_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long)atomic_read_pps_rejected()
  );
  draw_field_if_changed(
    62,
    152,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_3
  );

  snprintf(
    text,
    sizeof(text),
    "Jit:%lu Avg:%lu",
    (unsigned long)pps_data.jitter_us,
    (unsigned long)pps_data.avg_interval_us
  );
  draw_field_if_changed(
    62,
    172,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_4
  );

  snprintf(
    text,
    sizeof(text),
    "Align G:%lu B:%lu",
    (unsigned long)pps_data.align_good_count,
    (unsigned long)pps_data.align_bad_count
  );
  draw_field_if_changed(
    62,
    188,
    220,
    14,
    1,
    pps_data.gps_aligned ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.diagnostic_5
  );

  uint32_t elapsed = screen_manager_get_elapsed_ms();
  uint32_t screen_timeout_ms = system_config.screen_timeout_seconds * 1000UL;
  uint32_t remaining =
    elapsed >= screen_timeout_ms ?
    0 :
    screen_timeout_ms - elapsed;

  snprintf(
    text,
    sizeof(text),
    "Auto-return: %lus",
    (unsigned long)(remaining / 1000UL)
  );
  draw_field_if_changed(
    62,
    205,
    220,
    14,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.countdown_text
  );
}

void update_display() {
  DisplayMode mode = screen_manager_get_current();

  if (mode != PRIMARY_SCREEN &&
      screen_manager_get_elapsed_ms() >= (system_config.screen_timeout_seconds * 1000UL)) {
    screen_manager_handle_event(SCREEN_EVENT_TIMEOUT);
    mode = screen_manager_get_current();
    display_cache.force_full_redraw = true;
  }

  if (screen_manager_take_redraw_required()) {
    display_cache.force_full_redraw = true;
  }

  if (display_cache.force_full_redraw ||
      display_cache.drawn_mode != mode) {
    draw_screen_static(mode);
  }

  if (mode == PRIMARY_SCREEN) {
    update_primary_normal_fields();
    update_primary_fast_fields();
  } else if (mode == NETWORK_SCREEN) {
    update_network_screen();
  } else {
    update_diagnostic_screen();
  }

  display_cache.force_full_redraw = false;
}

void scheduler_update_display() {
  uint32_t now_ms = millis();

  update_display_brightness();

  static GPSLockState last_seen_gps_state = GPS_STALE;
  static PPSLockState last_seen_pps_state = PPS_TIMEOUT;
  static uint8_t last_seen_sats = 255;

  if (gps_data.state != last_seen_gps_state ||
      pps_data.state != last_seen_pps_state ||
      gps_data.satellites != last_seen_sats) {
    last_seen_gps_state = gps_data.state;
    last_seen_pps_state = pps_data.state;
    last_seen_sats = gps_data.satellites;
    display_cache.force_full_redraw = true;
  }

  DisplayMode current_mode = screen_manager_get_current();

  if (current_mode == PRIMARY_SCREEN) {
    if (now_ms - last_display_fast_ms >= DISPLAY_FAST_MS) {
      last_display_fast_ms = now_ms;

      if (display_cache.drawn_mode == PRIMARY_SCREEN &&
          !display_cache.force_full_redraw) {
        update_primary_fast_fields();
      }
    }

    if (now_ms - last_display_normal_ms >= DISPLAY_NORMAL_MS ||
        display_cache.force_full_redraw) {
      last_display_normal_ms = now_ms;
      update_display();
    }
  } else {
    if (now_ms - last_display_slow_ms >= DISPLAY_SLOW_MS ||
        display_cache.force_full_redraw) {
      last_display_slow_ms = now_ms;
      update_display();
    }
  }
}
