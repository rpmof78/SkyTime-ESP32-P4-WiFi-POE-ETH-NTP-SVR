/*
 * SkyTime GPS/PPS Stratum-1 NTP Server
 * Version 2.0 RC1.0
 *
 * Board: Waveshare ESP32-P4-WiFi6-PoE-ETH
 * GPS: ATGM336H on Serial2 with PPS input
 * Display: ST7789 240x320 SPI
 * Button: GPIO47, active LOW
 *
 * Copyright (C) 2026 RPMof78
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <TinyGPS++.h>
#include <time.h>
#include "esp_timer.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WebServer.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "driver/temperature_sensor.h"
#include <Preferences.h>
#include <stdarg.h>

// ============================================================
// Project Headers
// ============================================================

#include "board_pins.h"
#include "display_config.h"
#include "skytime_types.h"
#include "display_manager.h"
#include "config_manager.h"
#include "logging_manager.h"
#include "web_server_manager.h"
#include "gnss_manager.h"

// ============================================================
// Timing Configuration
// ============================================================

#define GPS_BAUD_RATE            9600

// ATGM336H NMEA output is UTC. Do not subtract GPS leap seconds
// from parsed NMEA time. Leap seconds apply to raw GPS time, not
// normal NMEA UTC sentences.
#define NTP_UNIX_OFFSET          2208988800UL

#define PPS_INTERVAL_MIN_US      950000ULL
#define PPS_INTERVAL_MAX_US      1050000ULL
#define PPS_TIMEOUT_US           2500000ULL

// ISR-level glitch filter.
// A valid PPS is 1 Hz, so any rising edge arriving too soon after the
// previous accepted edge is almost certainly noise, ringing, or a double edge.
#define PPS_ISR_REJECT_US        800000ULL


// PPS/GPS alignment guard.
// The GPS UTC second must be reasonably fresh when latching it to PPS.
#define PPS_GPS_ALIGN_MAX_AGE_MS 1500UL

// ============================================================
// System Health Configuration
// ============================================================

#define P4_DIE_TEMP_WARNING_C     100.0f
#define P4_DIE_TEMP_CRITICAL_C    120.0f
#define SYSTEM_HEALTH_UPDATE_MS   1000UL

// ============================================================
// Scheduler Configuration
// ============================================================

#define DEBUG_INTERVAL_MS        5000UL

// ============================================================
// FreeRTOS Task Configuration
// ============================================================

#define TASK_TIMING_STACK        8192
#define TASK_DISPLAY_STACK       8192
#define TASK_NETWORK_STACK       12288
#define TASK_TIMING_PRIORITY     3
#define TASK_NETWORK_PRIORITY    2
#define TASK_DISPLAY_PRIORITY    1
#define TASK_TIMING_CORE         0
#define TASK_NETWORK_CORE        0
#define TASK_DISPLAY_CORE        0

// ============================================================
// Arduino GFX Display Setup
// ============================================================

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  SPI_DC_PIN,
  SPI_CS_PIN,
  SPI_SCK_PIN,
  SPI_MOSI_PIN,
  -1
);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus,
  SPI_RST_PIN,
  1,
  true,
  DISPLAY_WIDTH,
  DISPLAY_HEIGHT,
  0,
  0
);

// ============================================================
// Global Objects
// ============================================================

TinyGPSPlus gps;
portMUX_TYPE pps_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Global Variables
// ============================================================

PPSCapture pps_capture = {0, 0, 0, false};
PPSData pps_data = {
  0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  PPS_WAITING, false, false
};

TimingData timing_data = {
  0, 0, 0, 0, 0, -20, 1000, 5000, false, false, 0
};

GPSData gps_data = {
  GPS_SEARCHING,
  false,
  false,
  false,
  false,
  0,
  0.0,
  0.0,
  0.0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0, 0
};


GnssRuntime gnss_runtime = {};
portMUX_TYPE gnss_mux = portMUX_INITIALIZER_UNLOCKED;

ButtonState button_state = {
  BUTTON_IDLE,
  false,
  0,
  0,
  false
};

DisplayCache display_cache;

ScreenManager screen_manager = {
  PRIMARY_SCREEN,
  PRIMARY_SCREEN,
  0,
  false,
  true
};

TaskHandle_t timing_task_handle = nullptr;
TaskHandle_t network_task_handle = nullptr;
TaskHandle_t display_task_handle = nullptr;

portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timing_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t last_display_fast_ms = 0;
uint32_t last_display_normal_ms = 0;
uint32_t last_display_slow_ms = 0;
uint32_t last_debug_time_ms = 0;

char ip_address[16] = "192.168.0.123";

// Default network values are replaced by /config/network.json when present.
NetworkConfig network_config = {
  "192.168.0.123",
  "192.168.0.1",
  "255.255.255.0",
  "192.168.0.1",
  "skytime-p4",
  false,
  false,
  false,
  true
};


// ============================================================
// Wi-Fi SoftAP Configuration
// ============================================================

#define SKYTIME_WIFI_AP_ENABLED       1
#define SKYTIME_WIFI_AP_SSID          "SkyTime-Setup"
#define SKYTIME_WIFI_AP_PASSWORD      "SkyTime123"
#define SKYTIME_WIFI_AP_CHANNEL       1
#define SKYTIME_WIFI_AP_HIDDEN        0
#define SKYTIME_WIFI_AP_MAX_CLIENTS   4
#define SKYTIME_WIFI_AP_WINDOW_MS     300000UL


IPAddress wifi_ap_ip(192, 168, 4, 123);
IPAddress wifi_ap_gateway(192, 168, 4, 123);
IPAddress wifi_ap_subnet(255, 255, 255, 0);

DNSServer commissioning_dns;
bool commissioning_dns_started = false;

WifiApRuntime wifi_ap_runtime = {
  SKYTIME_WIFI_AP_ENABLED != 0,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  false,
  "0.0.0.0",
  "not started"
};

bool boot_is_power_on_reset = false;
bool wifi_boot_button_requested = false;
bool wifi_boot_button_released = false;


// ============================================================
// Ethernet Configuration
// ============================================================

// IMPORTANT:
// PHY settings are isolated here so they can be changed easily if a
// Waveshare board revision or Arduino core requires different values.
//
// The default release build uses ETH.begin() with board-package defaults.
// If explicit PHY parameters are required, set SKYTIME_USE_EXPLICIT_ETH_PHY to 1
// and set the values below.

#define SKYTIME_USE_EXPLICIT_ETH_PHY 0

#if SKYTIME_USE_EXPLICIT_ETH_PHY
  #define SKYTIME_ETH_PHY_ADDR     1
  #define SKYTIME_ETH_PHY_POWER   -1
  #define SKYTIME_ETH_MDC_PIN     31
  #define SKYTIME_ETH_MDIO_PIN    52
  #define SKYTIME_ETH_PHY_TYPE    ETH_PHY_IP101
  #define SKYTIME_ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#endif

IPAddress eth_static_ip(192, 168, 0, 123);
IPAddress eth_gateway(192, 168, 0, 1);
IPAddress eth_subnet(255, 255, 255, 0);
IPAddress eth_dns(192, 168, 0, 1);

EthernetRuntime ethernet_runtime = {
  ETH_STATE_DISABLED,
  false,
  false,
  false,
  false,
  0,
  0,
  0,
  0,
  "0.0.0.0",
  "0.0.0.0",
  "0.0.0.0",
  "--:--:--:--:--:--"
};

bool ethernet_init_retry_done = false;


// ============================================================
// NTP Service Runtime
// ============================================================

#define NTP_PORT                 123
#define NTP_PACKET_SIZE          48

WiFiUDP ntp_udp;
WiFiUDP udp4123;

NtpRuntime ntp_runtime = {
  NTP_STATE_DISABLED,
  false,
  true,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  IPAddress(0, 0, 0, 0),
  0,
  0
};


Udp123Debug udp123_debug = {
  0,
  0,
  0,
  0,
  0,
  0,
  IPAddress(0, 0, 0, 0)
};


Udp4123Debug udp4123_debug = {
  0,
  0,
  0,
  IPAddress(0, 0, 0, 0)
};


// ============================================================
// MicroSD Runtime
// ============================================================

#define SD_MOUNT_POINT           "/sdcard"
#define SD_TEST_FILE             "/skytime_sd_test.txt"

// Waveshare ESP32-P4-ETH onboard TF card slot uses SDIO 3.0.
// Start in 4-bit SD_MMC mode. If a specific board/core revision has
// trouble, set SD_FORCE_1BIT_MODE to 1 for diagnostic fallback.
#define SD_FORCE_1BIT_MODE       0

SdRuntime sd_runtime = {
  SD_STATE_DISABLED,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  "NONE",
  ""
};


// ============================================================
// Configuration Runtime
// ============================================================

SystemConfig system_config = {
  "SkyTime",
  "SkyTime",
  "Standalone",
  "Enter Location in Configuration",
  0,
  30,
  60,
  false,
  1320,
  360,
  70,
  30,
  true,
  false
};

DisplayDimmingRuntime display_dimming = {
  false,
  false,
  false,
  100,
  0,
  0,
  0
};

ConfigRuntime config_runtime = {
  CONFIG_STATE_DEFAULTS,
  false,
  false,
  false,
  0,
  0,
  "defaults"
};


// ============================================================
// Web Server
// ============================================================

#define WEB_SERVER_PORT          80
#define WEB_ENABLE_RAW_8080_DEBUG 0

WebServer web_server(WEB_SERVER_PORT);
#if WEB_ENABLE_RAW_8080_DEBUG
WiFiServer raw_http_debug_server(8080);
#endif

WebRuntime web_runtime = {
  WEB_STATE_DISABLED,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  "",
  ""
};


// ============================================================
// System Health Runtime
// ============================================================

temperature_sensor_handle_t p4_temperature_handle = nullptr;
SystemHealthRuntime system_health = {false,0,0,SYSTEM_TEMP_UNAVAILABLE,0,0,0,0,0,0,0,0,0,0,0};

void init_system_health();
void update_system_health();
const char *system_temperature_state_text(SystemTemperatureState state);
const char *reset_reason_text(esp_reset_reason_t reason);


// ============================================================
// Persistent Reset History
// ============================================================

#define RESET_HISTORY_NAMESPACE          "skyreset"
#define UPTIME_CHECKPOINT_INTERVAL_MS     300000UL

ResetHistoryRuntime reset_history = {
  false,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0
};

void init_reset_history();
void update_uptime_checkpoint();
void save_uptime_checkpoint_now();

// ============================================================
// Operational Logging
// ============================================================


LogRuntime log_runtime = {
  true, false,
  0, 0, 0, 0,
  0, 0, 0, 0, 0,
  "",
  "OK"
};

MonitoredStateSnapshot monitored_state = {
  false,
  SYSTEM_TEMP_UNAVAILABLE,
  false,
  GPS_SEARCHING,
  PPS_WAITING,
  false
};


// ============================================================
// Operational Statistics
// ============================================================

OperationalStats operational_stats = {
  0,
  false,
  0
};

static bool format_ip_address(
  const IPAddress &ip,
  char *buffer,
  size_t buffer_size
) {
  if (!buffer || buffer_size == 0) {
    return false;
  }

  int written = snprintf(
    buffer,
    buffer_size,
    "%u.%u.%u.%u",
    (unsigned int)ip[0],
    (unsigned int)ip[1],
    (unsigned int)ip[2],
    (unsigned int)ip[3]
  );

  if (written < 0 ||
      (size_t)written >= buffer_size) {
    buffer[0] = '\0';
    return false;
  }

  return true;
}

bool append_json_uint(
  String &output,
  uint64_t value
) {
  char number[24];

  int written = snprintf(
    number,
    sizeof(number),
    "%llu",
    (unsigned long long)value
  );

  if (written < 0 ||
      (size_t)written >= sizeof(number)) {
    return false;
  }

  return output.concat(number);
}


static bool append_json_escaped(
  String &output,
  const char *value
) {
  if (!value) {
    return output.concat("");
  }

  for (size_t i = 0; value[i] != '\0'; i++) {
    char c = value[i];

    if (c == '"' || c == '\\') {
      if (!output.concat('\\')) {
        return false;
      }
    }

    if (c != '\r' && c != '\n') {
      if (!output.concat(c)) {
        return false;
      }
    }
  }

  return true;
}


// ============================================================
// Function Prototypes
// ============================================================

void init_systems();

void update_pps_state();
void update_pps_quality_stats(uint64_t interval_us);
bool validate_pps_gps_alignment();
void calculate_ntp_timestamp();

void timing_task(void *parameter);
void network_task(void *parameter);
void display_task(void *parameter);

void init_wifi_config_ap();
void update_wifi_config_ap();
void start_commissioning_dns();
void stop_commissioning_dns();
bool web_network_ready();
void init_ethernet();
void WiFiEvent(WiFiEvent_t event);
void update_ethernet_runtime();
const char *ethernet_state_text(EthernetState state);
void init_ntp_listener();
void update_ntp_listener();
const char *ntp_state_text(NtpServiceState state);
void write_ntp_u32(uint8_t *packet, int offset, uint32_t value);
uint32_t us_to_ntp_short(uint32_t microseconds);
void snapshot_ntp_timing(NtpTimingSnapshot *snapshot);
bool send_ntp_reply(const uint8_t *request_packet, uint8_t client_version, uint8_t client_mode);


void init_sd_card();
void update_sd_runtime();
bool sd_self_test();
const char *sd_state_text(SdCardState state);
const char *sd_card_type_text(uint8_t card_type);

// Shared static-file helper used by configuration and web handlers.
bool send_sd_file(const char *path, const char *content_type);

// JSON string escaping shared by configuration and status responses.
void json_escape_string(const char *src, char *dst, size_t dst_size) {
  if (!dst || dst_size == 0) {
    return;
  }

  dst[0] = '\0';

  if (!src) {
    return;
  }

  size_t out = 0;

  for (size_t i = 0; src[i] && out < dst_size - 1; i++) {
    char c = src[i];

    if (c == '"' || c == '\\') {
      if (out + 2 >= dst_size) {
        break;
      }

      dst[out++] = '\\';
      dst[out++] = c;
    } else if (c >= 32 && c <= 126) {
      dst[out++] = c;
    }
  }

  dst[out] = '\0';
}

// Hostname validation shared by configuration handlers.
bool valid_hostname_text(const char *text, size_t max_len) {
  if (!text) {
    return false;
  }

  size_t len = strlen(text);

  if (len == 0 || len >= max_len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    char c = text[i];

    bool ok =
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') ||
      c == '-' ||
      c == '_';

    if (!ok) {
      return false;
    }
  }

  return true;
}

// ============================================================
// System Reboot API
// ============================================================

void handle_api_system_reboot() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  log_event_line("CONFIG", "System reboot requested from Web UI");

  web_server.send(
    200,
    "application/json",
    "{\"ok\":true,\"message\":\"System reboot requested. SkyTime will reboot shortly.\"}"
  );

  delay(750);
  save_uptime_checkpoint_now();
  delay(50);
  ESP.restart();
}


void handle_web_ping();
void update_raw_http_debug_server();
void build_status_json(
  char *buffer,
  size_t buffer_size
);
void build_status_html(
  char *buffer,
  size_t buffer_size
);

void handle_web_ping() {
  web_runtime.requests_total++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());
  web_server.send(200, "text/plain", "pong");
}

#if WEB_ENABLE_RAW_8080_DEBUG
void update_raw_http_debug_server() {
  WiFiClient client = raw_http_debug_server.available();

  if (!client) {
    return;
  }

  web_runtime.raw8080_requests++;

  uint32_t start_ms = millis();

  while (client.connected() && millis() - start_ms < 250UL) {
    while (client.available()) {
      client.read();
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("SkyTime raw TCP debug OK");
    client.print("IP: ");
    client.println(ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
    client.print("NTP RX: ");
    client.println((unsigned long)ntp_runtime.packets_rx);
    client.print("NTP TX: ");
    client.println((unsigned long)ntp_runtime.packets_tx);
    client.print("Web state: ");
    client.println(web_state_text(web_runtime.state));
    break;
  }

  delay(1);
  client.stop();

  Serial.printf(
    "[WEB8080] RX #%lu raw TCP debug request\n",
    (unsigned long)web_runtime.raw8080_requests
  );
}

#else
void update_raw_http_debug_server() {
  // Raw TCP debug server disabled for normal operation
}
#endif


void log_event_line(const char *event, const char *detail);
void log_ntp_request(uint8_t version, uint8_t mode, bool tx_ok);

void update_operational_stats();
uint32_t gps_lock_seconds();

void debug_output();

static inline bool isLeapYear(int year);
uint32_t makeTimeUTC(
  int year,
  int month,
  int day,
  int hour,
  int minute,
  int second
);

static inline uint64_t atomic_read_pps_time();
static inline uint32_t atomic_read_pps_count();
uint32_t atomic_read_pps_rejected();
static inline bool atomic_take_pps_triggered();

// ============================================================
// Utility Functions
// ============================================================

static inline bool isLeapYear(int year) {
  return ((year % 4 == 0 && year % 100 != 0) ||
          (year % 400 == 0));
}

uint32_t makeTimeUTC(
  int year,
  int month,
  int day,
  int hour,
  int minute,
  int second
) {
  static const int monthDays[] = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31
  };

  if (year < 1970) year = 1970;
  if (month < 1) month = 1;
  if (month > 12) month = 12;
  if (day < 1) day = 1;
  if (day > 31) day = 31;
  if (hour < 0) hour = 0;
  if (hour > 23) hour = 23;
  if (minute < 0) minute = 0;
  if (minute > 59) minute = 59;
  if (second < 0) second = 0;
  if (second > 59) second = 59;

  uint32_t days = 0;

  for (int y = 1970; y < year; y++) {
    days += isLeapYear(y) ? 366 : 365;
  }

  for (int m = 1; m < month; m++) {
    days += monthDays[m - 1];
    if (m == 2 && isLeapYear(year)) {
      days++;
    }
  }

  days += day - 1;

  uint32_t unix_epoch =
    (((days * 24UL + hour) * 60UL + minute) * 60UL + second);

  return unix_epoch + NTP_UNIX_OFFSET;
}

static inline uint64_t atomic_read_pps_time() {
  uint64_t value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.pps_time_us;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

static inline uint32_t atomic_read_pps_count() {
  uint32_t value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.pps_count;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

uint32_t atomic_read_pps_rejected() {
  uint32_t value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.rejected_edges;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

static inline bool atomic_take_pps_triggered() {
  bool value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.pps_triggered;
  pps_capture.pps_triggered = false;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

bool atomic_take_button_irq() {
  bool value;

  portENTER_CRITICAL(&state_mux);
  value = button_state.irq_pending;
  button_state.irq_pending = false;
  portEXIT_CRITICAL(&state_mux);

  return value;
}

// ============================================================
// Interrupt Handlers
// ============================================================

void IRAM_ATTR pps_interrupt_handler() {
  uint64_t now_us = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&pps_mux);

  uint64_t last_us = pps_capture.pps_time_us;

  if (last_us != 0 && (now_us - last_us) < PPS_ISR_REJECT_US) {
    pps_capture.rejected_edges = pps_capture.rejected_edges + 1;
    portEXIT_CRITICAL_ISR(&pps_mux);
    return;
  }

  pps_capture.pps_time_us = now_us;
  pps_capture.pps_count = pps_capture.pps_count + 1;
  pps_capture.pps_triggered = true;

  portEXIT_CRITICAL_ISR(&pps_mux);
}

void IRAM_ATTR button_interrupt_handler() {
  portENTER_CRITICAL_ISR(&state_mux);
  button_state.irq_pending = true;
  portEXIT_CRITICAL_ISR(&state_mux);
}

// ============================================================
// System Initialization
// ============================================================

void init_systems() {
  Serial.print("[I2C] Initializing... ");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  delay(50);
  Serial.println("OK");

  Serial.print("[GFX] Initializing display... ");
  if (!gfx->begin()) {
    Serial.println("FAILED");
    while (1) {
      delay(100);
    }
  }
  gfx->fillScreen(COLOR_BLACK);
  Serial.println("OK");

  Serial.print("[GPS] Initializing Serial2... ");
  Serial2.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(50);
  Serial.println("OK");

  Serial.print("[PPS] Initializing GPIO46... ");
  pinMode(PPS_IN_PIN, INPUT_PULLDOWN);
  attachInterrupt(
    digitalPinToInterrupt(PPS_IN_PIN),
    pps_interrupt_handler,
    RISING
  );
  Serial.println("OK");

  Serial.print("[BUTTON] Initializing GPIO47... ");
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    button_interrupt_handler,
    FALLING
  );
  Serial.println("OK");

  Serial.print("[GPIO] Setting backlight PWM... ");
  pinMode(BL_PIN, OUTPUT);

  display_dimming.pwm_attached = ledcAttach(
    BL_PIN,
    BACKLIGHT_PWM_FREQUENCY,
    BACKLIGHT_PWM_RESOLUTION
  );

  if (display_dimming.pwm_attached) {
    ledcWrite(
      BL_PIN,
      BACKLIGHT_PWM_MAX_DUTY
    );
    Serial.println("OK");
  } else {
    digitalWrite(BL_PIN, HIGH);
    Serial.println("PWM unavailable; full brightness");
  }

  clear_cache();

  Serial.println();
  Serial.println("========================================");
  Serial.println(" SkyTime GPS/PPS Timing Base");
  Serial.print(" Version ");
  Serial.println(SKYTIME_VERSION);
  Serial.println(" Identity Configuration");
  Serial.println("========================================");
  Serial.println();
}

// ============================================================
// PPS / Timing Core
// ============================================================

void update_pps_quality_stats(uint64_t interval_us) {
  if (interval_us == 0) {
    return;
  }

  if (pps_data.min_interval_us == 0 || interval_us < pps_data.min_interval_us) {
    pps_data.min_interval_us = interval_us;
  }

  if (interval_us > pps_data.max_interval_us) {
    pps_data.max_interval_us = interval_us;
  }

  if (pps_data.avg_interval_us == 0) {
    pps_data.avg_interval_us = interval_us;
  } else {
    // Lightweight IIR average to avoid overflow over long runtimes.
    pps_data.avg_interval_us =
      ((pps_data.avg_interval_us * 15ULL) + interval_us) / 16ULL;
  }

  uint64_t error_us =
    (interval_us > 1000000ULL) ?
    (interval_us - 1000000ULL) :
    (1000000ULL - interval_us);

  pps_data.jitter_us = error_us;
  pps_data.jitter_accum_us += error_us;
  pps_data.jitter_samples++;
}

bool validate_pps_gps_alignment() {
  if (!gps_data.time_valid || !gps_data.date_valid) {
    pps_data.gps_aligned = false;
    pps_data.align_bad_count++;
    return false;
  }

  if (gps_data.last_time_update_ms == 0) {
    pps_data.gps_aligned = false;
    pps_data.align_bad_count++;
    return false;
  }

  uint32_t age_ms = millis() - gps_data.last_time_update_ms;

  if (age_ms > PPS_GPS_ALIGN_MAX_AGE_MS) {
    pps_data.gps_aligned = false;
    pps_data.align_bad_count++;
    return false;
  }

  pps_data.gps_aligned = true;
  pps_data.align_good_count++;
  return true;
}

void update_pps_state() {
  bool new_pps = atomic_take_pps_triggered();
  uint64_t now_us = esp_timer_get_time();

  if (new_pps) {
    pps_data.previous_pps_time_us = pps_data.current_pps_time_us;
    pps_data.current_pps_time_us = atomic_read_pps_time();
    pps_data.pps_count = atomic_read_pps_count();
    pps_data.edge_seen = true;

    if (pps_data.previous_pps_time_us > 0) {
      pps_data.last_interval_us =
        pps_data.current_pps_time_us - pps_data.previous_pps_time_us;

      update_pps_quality_stats(pps_data.last_interval_us);

      if (pps_data.last_interval_us >= PPS_INTERVAL_MIN_US &&
          pps_data.last_interval_us <= PPS_INTERVAL_MAX_US) {
        pps_data.state = PPS_LOCKED;
        pps_data.valid_count++;
      } else {
        pps_data.state = PPS_BAD_INTERVAL;
        pps_data.bad_count++;
      }
    } else {
      pps_data.state = PPS_WAITING;
    }

    if ((gps_data.locked || gps_data.state == GPS_HOLDOVER) &&
        gps_data.date_valid &&
        gps_data.time_valid) {
      if (validate_pps_gps_alignment()) {
        // NMEA UTC sentence time lags the PPS edge by one second.
        // PPS marks the start of the next UTC second, so latch +1.
        pps_data.last_pps_epoch = makeTimeUTC(
          gps_data.year,
          gps_data.month,
          gps_data.day,
          gps_data.hour,
          gps_data.minute,
          gps_data.second
        ) + 1;
      } else if (pps_data.last_pps_epoch == 0) {
        pps_data.state = PPS_ALIGN_WAIT;
      } else {
        pps_data.state = PPS_ALIGN_BAD;
      }
    }
  }

  if (pps_data.edge_seen &&
      pps_data.current_pps_time_us > 0 &&
      (now_us - pps_data.current_pps_time_us) > PPS_TIMEOUT_US) {
    pps_data.state = PPS_TIMEOUT;
    portENTER_CRITICAL(&timing_mux);
    timing_data.disciplined = false;
    portEXIT_CRITICAL(&timing_mux);
  }
}

void calculate_ntp_timestamp() {
  if (pps_data.last_pps_epoch == 0 ||
      pps_data.current_pps_time_us == 0 ||
      pps_data.state == PPS_TIMEOUT) {
    portENTER_CRITICAL(&timing_mux);
    timing_data.disciplined = false;
    portEXIT_CRITICAL(&timing_mux);
    return;
  }

  uint64_t now_us = esp_timer_get_time();
  uint64_t delta_us = now_us - pps_data.current_pps_time_us;
  uint32_t elapsed_seconds = delta_us / 1000000ULL;
  uint32_t fractional_us = delta_us % 1000000ULL;
  uint32_t current_epoch = pps_data.last_pps_epoch + elapsed_seconds;
  uint32_t holdover_start_ms = 0;

  portENTER_CRITICAL(&timing_mux);
  holdover_start_ms = timing_data.holdover_start_ms;
  portEXIT_CRITICAL(&timing_mux);

  uint32_t root_dispersion = 50000;
  bool disciplined = false;
  bool holdover = false;

  if (pps_data.state == PPS_LOCKED && gps_data.locked) {
    root_dispersion = 5000;
    disciplined = true;
  } else if (pps_data.state == PPS_LOCKED &&
             gps_data.state == GPS_HOLDOVER &&
             pps_data.last_pps_epoch > 0) {
    uint32_t holdover_age_s = holdover_start_ms > 0 ?
      ((millis() - holdover_start_ms) / 1000UL) : 0;
    root_dispersion = 5000UL + (holdover_age_s * 20UL);
    disciplined = true;
    holdover = true;
  }

  portENTER_CRITICAL(&timing_mux);
  timing_data.current_epoch = current_epoch;
  timing_data.reference_epoch = pps_data.last_pps_epoch;
  timing_data.current_time_us = now_us;
  timing_data.microseconds_since_pps = fractional_us;
  timing_data.ntp_fraction = ((uint64_t)fractional_us << 32) / 1000000ULL;
  timing_data.ntp_precision = -20;
  timing_data.root_delay = 1000;
  timing_data.root_dispersion = root_dispersion;
  timing_data.disciplined = disciplined;
  timing_data.holdover = holdover;
  portEXIT_CRITICAL(&timing_mux);
}


// ============================================================
// Wi-Fi Commissioning Runtime
// ============================================================

bool web_network_ready() {
  return ethernet_runtime.got_ip || wifi_ap_runtime.started;
}

void start_commissioning_dns() {
  if (!wifi_ap_runtime.started || commissioning_dns_started) {
    return;
  }

  commissioning_dns.setErrorReplyCode(DNSReplyCode::NoError);

  if (commissioning_dns.start(53, "*", wifi_ap_ip)) {
    commissioning_dns_started = true;
    Serial.println("[WIFI-AP] Captive DNS started on port 53");
  } else {
    wifi_ap_runtime.errors++;
    Serial.println("[WIFI-AP] Captive DNS failed to start");
  }
}

void stop_commissioning_dns() {
  if (!commissioning_dns_started) {
    return;
  }

  commissioning_dns.stop();
  commissioning_dns_started = false;
  Serial.println("[WIFI-AP] Captive DNS stopped");
}

void init_wifi_config_ap() {
#if SKYTIME_WIFI_AP_ENABLED
  if (!boot_is_power_on_reset) {
    wifi_ap_runtime.enabled = false;
    wifi_ap_runtime.started = false;
    wifi_ap_runtime.window_expired = false;
    wifi_ap_runtime.remaining_seconds = 0;

    snprintf(
      wifi_ap_runtime.last_error,
      sizeof(wifi_ap_runtime.last_error),
      "not a power-on reset"
    );

    Serial.println("[WIFI-AP] Not started: reset was not POWERON");
    return;
  }

  if (!wifi_boot_button_requested || !wifi_boot_button_released) {
    wifi_ap_runtime.enabled = false;
    wifi_ap_runtime.started = false;
    wifi_ap_runtime.window_expired = false;
    wifi_ap_runtime.remaining_seconds = 0;

    snprintf(
      wifi_ap_runtime.last_error,
      sizeof(wifi_ap_runtime.last_error),
      "boot button not requested"
    );

    Serial.println("[WIFI-AP] Not started: boot button was not held");
    return;
  }

  Serial.println("[WIFI-AP] Starting configuration access point");
  Serial.println("[WIFI-AP] Configuration window: 300 seconds");

  wifi_ap_runtime.start_attempts++;
  wifi_ap_runtime.config_ok = false;
  wifi_ap_runtime.started = false;
  wifi_ap_runtime.clients = 0;

  WiFi.mode(WIFI_AP);

  bool config_ok = WiFi.softAPConfig(
    wifi_ap_ip,
    wifi_ap_gateway,
    wifi_ap_subnet
  );

  wifi_ap_runtime.config_ok = config_ok;

  if (!config_ok) {
    wifi_ap_runtime.errors++;
    snprintf(
      wifi_ap_runtime.last_error,
      sizeof(wifi_ap_runtime.last_error),
      "softAPConfig failed"
    );
    Serial.println("[WIFI-AP] softAPConfig failed");
    return;
  }

  bool start_ok = WiFi.softAP(
    SKYTIME_WIFI_AP_SSID,
    SKYTIME_WIFI_AP_PASSWORD,
    SKYTIME_WIFI_AP_CHANNEL,
    SKYTIME_WIFI_AP_HIDDEN,
    SKYTIME_WIFI_AP_MAX_CLIENTS
  );

  if (!start_ok) {
    wifi_ap_runtime.errors++;
    snprintf(
      wifi_ap_runtime.last_error,
      sizeof(wifi_ap_runtime.last_error),
      "softAP start failed"
    );
    Serial.println("[WIFI-AP] WiFi.softAP() failed");
    return;
  }

  wifi_ap_runtime.started = true;
  wifi_ap_runtime.started_ms = millis();
  wifi_ap_runtime.remaining_seconds = SKYTIME_WIFI_AP_WINDOW_MS / 1000UL;
  wifi_ap_runtime.window_expired = false;

  format_ip_address(
    WiFi.softAPIP(),
    wifi_ap_runtime.ip,
    sizeof(wifi_ap_runtime.ip)
  );

  snprintf(
    wifi_ap_runtime.last_error,
    sizeof(wifi_ap_runtime.last_error),
    "OK"
  );

  Serial.print("[WIFI-AP] SSID: ");
  Serial.println(SKYTIME_WIFI_AP_SSID);
  Serial.print("[WIFI-AP] IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("[WIFI-AP] Gateway: ");
  Serial.println(wifi_ap_gateway);
  Serial.print("[WIFI-AP] Subnet: ");
  Serial.println(wifi_ap_subnet);
  Serial.println("[WIFI-AP] URL: http://192.168.4.123");
  start_commissioning_dns();
#else
  wifi_ap_runtime.enabled = false;
  wifi_ap_runtime.started = false;
  snprintf(
    wifi_ap_runtime.last_error,
    sizeof(wifi_ap_runtime.last_error),
    "disabled at compile time"
  );
  Serial.println("[WIFI-AP] Disabled at compile time");
#endif
}

void update_wifi_config_ap() {
#if SKYTIME_WIFI_AP_ENABLED
  static uint32_t last_client_poll_ms = 0;
  static uint32_t last_countdown_report_second = UINT32_MAX;

  if (!wifi_ap_runtime.started) {
    return;
  }

  commissioning_dns.processNextRequest();

  uint32_t now_ms = millis();
  uint32_t elapsed_ms = now_ms - wifi_ap_runtime.started_ms;

  if (elapsed_ms >= SKYTIME_WIFI_AP_WINDOW_MS) {
    uint8_t disconnecting_clients = WiFi.softAPgetStationNum();

    Serial.println("[WIFI-AP] Configuration window expired");
    Serial.print("[WIFI-AP] Disconnecting ");
    Serial.print((unsigned int)disconnecting_clients);
    Serial.println(" client(s)");

    stop_commissioning_dns();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    wifi_ap_runtime.clients = 0;
    wifi_ap_runtime.remaining_seconds = 0;
    wifi_ap_runtime.started = false;
    wifi_ap_runtime.window_expired = true;

    snprintf(
      wifi_ap_runtime.ip,
      sizeof(wifi_ap_runtime.ip),
      "0.0.0.0"
    );

    snprintf(
      wifi_ap_runtime.last_error,
      sizeof(wifi_ap_runtime.last_error),
      "window expired"
    );

    Serial.println("[WIFI-AP] Radio stopped");
    return;
  }

  uint32_t remaining_ms = SKYTIME_WIFI_AP_WINDOW_MS - elapsed_ms;
  wifi_ap_runtime.remaining_seconds = (remaining_ms + 999UL) / 1000UL;

  if (wifi_ap_runtime.remaining_seconds <= 60UL &&
      wifi_ap_runtime.remaining_seconds != last_countdown_report_second &&
      (wifi_ap_runtime.remaining_seconds % 10UL == 0UL ||
       wifi_ap_runtime.remaining_seconds <= 5UL)) {
    last_countdown_report_second = wifi_ap_runtime.remaining_seconds;

    Serial.print("[WIFI-AP] Remaining: ");
    Serial.print((unsigned long)wifi_ap_runtime.remaining_seconds);
    Serial.println(" seconds");
  }

  if (now_ms - last_client_poll_ms >= 1000UL) {
    last_client_poll_ms = now_ms;
    wifi_ap_runtime.clients = WiFi.softAPgetStationNum();
  }
#endif
}

// ============================================================
// Ethernet Bring-Up / Runtime
// ============================================================

const char *ethernet_state_text(EthernetState state) {
  switch (state) {
    case ETH_STATE_DISABLED:
      return "DISABLED";
    case ETH_STATE_STARTING:
      return "STARTING";
    case ETH_STATE_LINK_DOWN:
      return "LINK DOWN";
    case ETH_STATE_LINK_UP:
      return "LINK UP";
    case ETH_STATE_GOT_IP:
      return "UP";
    case ETH_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

void WiFiEvent(WiFiEvent_t event) {
  ethernet_runtime.last_event_ms = millis();

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ethernet_runtime.started = true;
      ethernet_runtime.state = ETH_STATE_STARTING;
      ETH.setHostname(network_config.hostname);
      Serial.println("[ETH] Started");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      ethernet_runtime.link_up = true;
      ethernet_runtime.link_up_count++;
      ethernet_runtime.state = ETH_STATE_LINK_UP;
      Serial.println("[ETH] Link up");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      ethernet_runtime.got_ip = true;
      ethernet_runtime.link_up = true;
      ethernet_runtime.got_ip_count++;
      ethernet_runtime.state = ETH_STATE_GOT_IP;

      format_ip_address(
        ETH.localIP(),
        ethernet_runtime.ip,
        sizeof(ethernet_runtime.ip)
      );
      format_ip_address(
        ETH.gatewayIP(),
        ethernet_runtime.gateway,
        sizeof(ethernet_runtime.gateway)
      );
      format_ip_address(
        ETH.subnetMask(),
        ethernet_runtime.subnet,
        sizeof(ethernet_runtime.subnet)
      );
      snprintf(ethernet_runtime.mac, sizeof(ethernet_runtime.mac), "%s",
               ETH.macAddress().c_str());

      network_config.ethernet_enabled = true;

      Serial.print("[ETH] IP: ");
      Serial.println(ETH.localIP());
      Serial.print("[ETH] Gateway: ");
      Serial.println(ETH.gatewayIP());
      Serial.print("[ETH] Subnet: ");
      Serial.println(ETH.subnetMask());
      Serial.print("[ETH] MAC: ");
      Serial.println(ETH.macAddress());
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      ethernet_runtime.link_up = false;
      ethernet_runtime.got_ip = false;
      ethernet_runtime.link_down_count++;
      ethernet_runtime.state = ETH_STATE_LINK_DOWN;
      network_config.ethernet_enabled = false;
      Serial.println("[ETH] Link down");
      break;

    case ARDUINO_EVENT_ETH_STOP:
      ethernet_runtime.started = false;
      ethernet_runtime.link_up = false;
      ethernet_runtime.got_ip = false;
      ethernet_runtime.state = ETH_STATE_DISABLED;
      network_config.ethernet_enabled = false;
      Serial.println("[ETH] Stopped");
      break;

    default:
      break;
  }
}

void init_ethernet() {
  Serial.println("[ETH] Initializing Ethernet bring-up");

  ethernet_runtime.state = ETH_STATE_STARTING;
  ethernet_runtime.started = false;
  ethernet_runtime.link_up = false;
  ethernet_runtime.got_ip = false;
  ethernet_runtime.static_ip_applied = false;
  ethernet_runtime.last_event_ms = millis();

  WiFi.onEvent(WiFiEvent);

#if SKYTIME_USE_EXPLICIT_ETH_PHY
  bool begin_ok = ETH.begin(
    SKYTIME_ETH_PHY_TYPE,
    SKYTIME_ETH_PHY_ADDR,
    SKYTIME_ETH_MDC_PIN,
    SKYTIME_ETH_MDIO_PIN,
    SKYTIME_ETH_PHY_POWER,
    SKYTIME_ETH_CLK_MODE
  );
#else
  bool begin_ok = ETH.begin();
#endif

  Serial.printf("[ETH] begin_ok=%s\n", begin_ok ? "YES" : "NO");

  if (!begin_ok) {
    ethernet_runtime.state = ETH_STATE_ERROR;
    network_config.ethernet_enabled = false;
    Serial.println("[ETH] ETH.begin() failed");
    return;
  }

  // In Arduino-ESP32, static IP is most reliable when applied after ETH.begin().
  if (!network_config.dhcp_enabled) {
    bool config_ok = ETH.config(
      eth_static_ip,
      eth_gateway,
      eth_subnet,
      eth_dns,
      eth_dns
    );

    ethernet_runtime.static_ip_applied = config_ok;

    if (config_ok) {
      Serial.println("[ETH] Static IP config applied");
      Serial.print("[ETH] Static IP: ");
      Serial.println(eth_static_ip);
    } else {
      Serial.println("[ETH] Static IP config failed");
    }
  }

  network_config.ethernet_enabled = true;
}

void update_ethernet_runtime() {
  if (!ethernet_runtime.started) {
    return;
  }

  bool link = ETH.linkUp();

  if (link != ethernet_runtime.link_up) {
    ethernet_runtime.link_up = link;
    ethernet_runtime.last_event_ms = millis();

    if (link) {
      ethernet_runtime.link_up_count++;
      if (!ethernet_runtime.got_ip) {
        ethernet_runtime.state = ETH_STATE_LINK_UP;
      }
    } else {
      ethernet_runtime.link_down_count++;
      ethernet_runtime.got_ip = false;
      ethernet_runtime.state = ETH_STATE_LINK_DOWN;
      network_config.ethernet_enabled = false;
    }
  }

  if (link) {
    IPAddress ip = ETH.localIP();

    if (ip != IPAddress(0, 0, 0, 0)) {
      ethernet_runtime.got_ip = true;
      ethernet_runtime.state = ETH_STATE_GOT_IP;
      network_config.ethernet_enabled = true;

      format_ip_address(
        ip,
        ethernet_runtime.ip,
        sizeof(ethernet_runtime.ip)
      );
      format_ip_address(
        ETH.gatewayIP(),
        ethernet_runtime.gateway,
        sizeof(ethernet_runtime.gateway)
      );
      format_ip_address(
        ETH.subnetMask(),
        ethernet_runtime.subnet,
        sizeof(ethernet_runtime.subnet)
      );
      snprintf(ethernet_runtime.mac, sizeof(ethernet_runtime.mac), "%s",
               ETH.macAddress().c_str());
    }
  }
}


// ============================================================
// NTP Service
// ============================================================

const char *ntp_state_text(NtpServiceState state) {
  switch (state) {
    case NTP_STATE_DISABLED:
      return "DISABLED";
    case NTP_STATE_WAIT_ETH:
      return "WAIT ETH";
    case NTP_STATE_LISTENING:
      return "LISTEN";
    case NTP_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

void init_ntp_listener() {
  ntp_runtime.replies_enabled = true;

  if (!ethernet_runtime.got_ip) {
    if (ntp_runtime.state != NTP_STATE_WAIT_ETH) {
      Serial.println("[NTPD] Waiting for Ethernet IP before binding UDP");
    }
    ntp_runtime.state = NTP_STATE_WAIT_ETH;
    network_config.ntp_enabled = false;
    return;
  }

  if (ntp_runtime.udp_started) {
    return;
  }

  Serial.println("[NTPD] Starting UDP listener on port 123");

  if (udp4123.begin(4123)) {
    Serial.println("[UDP4123] Debug listener active on port 4123");
  } else {
    Serial.println("[UDP4123] Debug listener failed");
  }

  if (ntp_udp.begin(NTP_PORT)) {
    ntp_runtime.udp_started = true;
    ntp_runtime.state = NTP_STATE_LISTENING;
    network_config.ntp_enabled = true;
    Serial.println("[NTPD] UDP listener active");
    Serial.println("[NTPD] Replies enabled - Stratum-1 GPS/PPS mode");
  } else {
    ntp_runtime.udp_started = false;
    ntp_runtime.state = NTP_STATE_ERROR;
    network_config.ntp_enabled = false;
    Serial.println("[NTPD] UDP listener failed");
  }
}


void write_ntp_u32(uint8_t *packet, int offset, uint32_t value) {
  packet[offset + 0] = (uint8_t)((value >> 24) & 0xFF);
  packet[offset + 1] = (uint8_t)((value >> 16) & 0xFF);
  packet[offset + 2] = (uint8_t)((value >> 8) & 0xFF);
  packet[offset + 3] = (uint8_t)(value & 0xFF);
}

uint32_t us_to_ntp_short(uint32_t microseconds) {
  // Convert microseconds to NTP short format, 16.16 seconds.
  return (uint32_t)(((uint64_t)microseconds * 65536ULL) / 1000000ULL);
}

void snapshot_ntp_timing(NtpTimingSnapshot *snapshot) {
  if (!snapshot) return;

  portENTER_CRITICAL(&timing_mux);
  snapshot->current_epoch = timing_data.current_epoch;
  snapshot->reference_epoch = timing_data.reference_epoch;
  snapshot->microseconds_since_pps =
    timing_data.microseconds_since_pps;
  snapshot->ntp_fraction = timing_data.ntp_fraction;
  snapshot->ntp_precision = timing_data.ntp_precision;
  snapshot->root_delay = timing_data.root_delay;
  snapshot->root_dispersion = timing_data.root_dispersion;
  snapshot->disciplined = timing_data.disciplined;
  snapshot->holdover = timing_data.holdover;
  portEXIT_CRITICAL(&timing_mux);
}

bool send_ntp_reply(
  const uint8_t *request_packet,
  uint8_t client_version,
  uint8_t client_mode
) {
  if (client_mode != 3) {
    ntp_runtime.packets_bad_mode++;

    Serial.printf(
      "[NTPD] Bad mode:%u - not replying\n",
      client_mode
    );

    return false;
  }

  NtpTimingSnapshot receive_timing = {};
  snapshot_ntp_timing(&receive_timing);

  if (receive_timing.current_epoch == 0 ||
      !receive_timing.disciplined) {
    ntp_runtime.packets_not_ready++;

    Serial.println("[NTPD] Not disciplined - not replying");

    return false;
  }

  uint8_t reply[NTP_PACKET_SIZE];
  memset(reply, 0, sizeof(reply));

  uint32_t receive_seconds = receive_timing.current_epoch;
  uint32_t receive_fraction = receive_timing.ntp_fraction;
  uint32_t transmit_seconds = 0;
  uint32_t transmit_fraction = 0;

  uint8_t li = 0;
  uint8_t vn = client_version;

  if (vn < 3 || vn > 4) {
    vn = 4;
  }

  // LI=0, VN=client version, Mode=4 server.
  reply[0] = (uint8_t)((li << 6) | (vn << 3) | 4);

  // Stratum 1 while GPS/PPS disciplined; stratum 2 during holdover.
  reply[1] = receive_timing.holdover ? 2 : 1;

  // Poll copied from client. Precision from timing engine.
  reply[2] = request_packet[2];
  reply[3] = (uint8_t)receive_timing.ntp_precision;

  write_ntp_u32(reply, 4, us_to_ntp_short(receive_timing.root_delay));
  write_ntp_u32(reply, 8, us_to_ntp_short(receive_timing.root_dispersion));

  // Reference ID "GPS".
  reply[12] = 'G';
  reply[13] = 'P';
  reply[14] = 'S';
  reply[15] = 0;

  // Reference timestamp: last PPS epoch, fractional zero.
  uint32_t ref_epoch =
    receive_timing.reference_epoch > 0 ?
    receive_timing.reference_epoch :
    receive_timing.current_epoch;

  write_ntp_u32(reply, 16, ref_epoch);
  write_ntp_u32(reply, 20, 0);

  // Originate timestamp: client's transmit timestamp from request.
  memcpy(&reply[24], &request_packet[40], 8);

  // Receive timestamp.
  write_ntp_u32(reply, 32, receive_seconds);
  write_ntp_u32(reply, 36, receive_fraction);

  NtpTimingSnapshot transmit_timing = {};
  snapshot_ntp_timing(&transmit_timing);
  transmit_seconds = transmit_timing.current_epoch;
  transmit_fraction = transmit_timing.ntp_fraction;

  // Transmit timestamp.
  write_ntp_u32(reply, 40, transmit_seconds);
  write_ntp_u32(reply, 44, transmit_fraction);

  ntp_udp.beginPacket(
    ntp_runtime.last_remote_ip,
    ntp_runtime.last_remote_port
  );

  ntp_udp.write(reply, NTP_PACKET_SIZE);
  bool ok = ntp_udp.endPacket();

  if (ok) {
    ntp_runtime.packets_tx++;

    Serial.printf(
      "[NTPD] TX #%lu to %u.%u.%u.%u:%u Epoch:%lu Frac:0x%08lX Stratum:%u\n",
      (unsigned long)ntp_runtime.packets_tx,
      (unsigned int)ntp_runtime.last_remote_ip[0],
      (unsigned int)ntp_runtime.last_remote_ip[1],
      (unsigned int)ntp_runtime.last_remote_ip[2],
      (unsigned int)ntp_runtime.last_remote_ip[3],
      ntp_runtime.last_remote_port,
      (unsigned long)transmit_seconds,
      (unsigned long)transmit_fraction,
      reply[1]
    );
  } else {
    ntp_runtime.packets_ignored++;
    Serial.println("[NTPD] TX failed");
  }

  operational_stats.last_ntp_query_epoch = transmit_timing.current_epoch;
  log_ntp_request(client_version, client_mode, ok);

  return ok;
}


void update_ntp_listener() {
  if (!ethernet_runtime.got_ip) {
    if (!ntp_runtime.udp_started) {
      ntp_runtime.state = NTP_STATE_WAIT_ETH;
      network_config.ntp_enabled = false;
    }
    return;
  }

  if (!ntp_runtime.udp_started) {
    init_ntp_listener();
    return;
  }


  int debug_size = udp4123.parsePacket();

  if (debug_size > 0) {
    udp4123_debug.total_rx++;
    udp4123_debug.last_size = (uint16_t)debug_size;
    udp4123_debug.last_port = udp4123.remotePort();
    udp4123_debug.last_ip = udp4123.remoteIP();

    Serial.printf(
      "[UDP4123] RX #%lu Size:%u From:%u.%u.%u.%u:%u\n",
      (unsigned long)udp4123_debug.total_rx,
      (unsigned int)udp4123_debug.last_size,
      (unsigned int)udp4123_debug.last_ip[0],
      (unsigned int)udp4123_debug.last_ip[1],
      (unsigned int)udp4123_debug.last_ip[2],
      (unsigned int)udp4123_debug.last_ip[3],
      udp4123_debug.last_port
    );

    while (udp4123.available()) {
      udp4123.read();
    }
  }

  int packet_size = ntp_udp.parsePacket();

  if (packet_size <= 0) {
    return;
  }

  udp123_debug.total_rx++;
  udp123_debug.last_size = (uint16_t)packet_size;
  udp123_debug.last_port = ntp_udp.remotePort();
  udp123_debug.last_ip = ntp_udp.remoteIP();

  ntp_runtime.last_packet_ms = millis();
  ntp_runtime.last_remote_ip = udp123_debug.last_ip;
  ntp_runtime.last_remote_port = udp123_debug.last_port;
  ntp_runtime.last_packet_size = udp123_debug.last_size;

  Serial.printf(
    "[UDP123] RX #%lu Size:%u From:%u.%u.%u.%u:%u\n",
    (unsigned long)udp123_debug.total_rx,
    (unsigned int)udp123_debug.last_size,
    (unsigned int)udp123_debug.last_ip[0],
    (unsigned int)udp123_debug.last_ip[1],
    (unsigned int)udp123_debug.last_ip[2],
    (unsigned int)udp123_debug.last_ip[3],
    udp123_debug.last_port
  );

  if (packet_size < NTP_PACKET_SIZE) {
    udp123_debug.short_rx++;
    ntp_runtime.packets_short++;

    Serial.printf(
      "[UDP123] SHORT #%lu Size:%u\n",
      (unsigned long)udp123_debug.short_rx,
      (unsigned int)udp123_debug.last_size
    );

    while (ntp_udp.available()) {
      ntp_udp.read();
    }

    return;
  }

  uint8_t packet[NTP_PACKET_SIZE];
  int bytes_read = ntp_udp.read(packet, NTP_PACKET_SIZE);

  while (ntp_udp.available()) {
    ntp_udp.read();
  }

  if (bytes_read != NTP_PACKET_SIZE) {
    udp123_debug.ignored_rx++;
    ntp_runtime.packets_ignored++;

    Serial.printf(
      "[UDP123] READ ERROR Read:%d Expected:%u\n",
      bytes_read,
      NTP_PACKET_SIZE
    );

    return;
  }

  udp123_debug.ntp_rx++;
  ntp_runtime.packets_rx++;

  uint8_t li = (packet[0] >> 6) & 0x03;
  uint8_t version = (packet[0] >> 3) & 0x07;
  uint8_t mode = packet[0] & 0x07;

  Serial.printf(
    "[NTPD] VALID #%lu LI:%u VN:%u Mode:%u Replies:%s\n",
    (unsigned long)udp123_debug.ntp_rx,
    li,
    version,
    mode,
    ntp_runtime.replies_enabled ? "ON" : "OFF"
  );

  if (ntp_runtime.replies_enabled) {
    send_ntp_reply(packet, version, mode);
  }
}


// ============================================================
// MicroSD / TF Card Runtime
// ============================================================

const char *sd_state_text(SdCardState state) {
  switch (state) {
    case SD_STATE_DISABLED:
      return "DISABLED";
    case SD_STATE_STARTING:
      return "STARTING";
    case SD_STATE_MOUNTED:
      return "MOUNTED";
    case SD_STATE_NO_CARD:
      return "NO CARD";
    case SD_STATE_TEST_FAILED:
      return "TEST FAIL";
    case SD_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

const char *sd_card_type_text(uint8_t card_type) {
  switch (card_type) {
    case CARD_NONE:
      return "NONE";
    case CARD_MMC:
      return "MMC";
    case CARD_SD:
      return "SDSC";
    case CARD_SDHC:
      return "SDHC";
    default:
      return "UNKNOWN";
  }
}

bool sd_self_test() {
  if (!sd_runtime.mounted) {
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "not mounted");
    return false;
  }

  File file = SD_MMC.open(SD_TEST_FILE, FILE_WRITE);

  if (!file) {
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "open write failed");
    return false;
  }

  uint32_t now_ms = millis();
  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  file.printf("SkyTime SD self-test\n");
  file.printf("Millis: %lu\n", (unsigned long)now_ms);
  file.printf("NTP epoch: %lu\n", (unsigned long)timing.current_epoch);
  file.close();

  sd_runtime.test_writes++;

  file = SD_MMC.open(SD_TEST_FILE, FILE_READ);

  if (!file) {
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "open read failed");
    return false;
  }

  size_t available = file.available();
  file.close();

  if (available == 0) {
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "read empty");
    return false;
  }

  sd_runtime.test_reads++;
  sd_runtime.test_passed = true;
  snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "OK");

  return true;
}

void init_sd_card() {
  Serial.println("[SD] Initializing onboard TF card via SD_MMC");

  sd_runtime.state = SD_STATE_STARTING;
  sd_runtime.mount_attempts++;
  sd_runtime.mounted = false;
  sd_runtime.test_passed = false;

#if SD_FORCE_1BIT_MODE
  bool one_bit = true;
#else
  bool one_bit = false;
#endif

  if (!SD_MMC.begin(SD_MOUNT_POINT, one_bit, false)) {
    sd_runtime.state = SD_STATE_ERROR;
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "mount failed");

    Serial.println("[SD] Mount failed");
    Serial.println("[SD] If the card is inserted, try SD_FORCE_1BIT_MODE=1");
    return;
  }

  uint8_t card_type = SD_MMC.cardType();

  snprintf(
    sd_runtime.card_type,
    sizeof(sd_runtime.card_type),
    "%s",
    sd_card_type_text(card_type)
  );

  if (card_type == CARD_NONE) {
    sd_runtime.state = SD_STATE_NO_CARD;
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "no card");

    Serial.println("[SD] No card detected");
    return;
  }

  sd_runtime.mounted = true;
  sd_runtime.card_size_bytes = SD_MMC.cardSize();
  sd_runtime.total_bytes = SD_MMC.totalBytes();
  sd_runtime.used_bytes = SD_MMC.usedBytes();

  Serial.printf("[SD] Mounted type:%s\n", sd_runtime.card_type);
  Serial.printf(
    "[SD] Card:%llu MB Total:%llu MB Used:%llu MB\n",
    (unsigned long long)(sd_runtime.card_size_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.total_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.used_bytes / (1024ULL * 1024ULL))
  );

  if (sd_self_test()) {
    sd_runtime.state = SD_STATE_MOUNTED;
    Serial.println("[SD] Read/write self-test PASS");
  } else {
    sd_runtime.state = SD_STATE_TEST_FAILED;
    Serial.print("[SD] Read/write self-test FAIL: ");
    Serial.println(sd_runtime.last_error);
  }
}

void update_sd_runtime() {
  if (!sd_runtime.mounted) {
    return;
  }

  static uint32_t last_sd_refresh_ms = 0;
  uint32_t now_ms = millis();

  if (now_ms - last_sd_refresh_ms < 10000UL) {
    return;
  }

  last_sd_refresh_ms = now_ms;

  sd_runtime.total_bytes = SD_MMC.totalBytes();
  sd_runtime.used_bytes = SD_MMC.usedBytes();
}


// ============================================================
// System Health Monitoring
// ============================================================

const char *system_temperature_state_text(SystemTemperatureState state) {
  switch (state) {
    case SYSTEM_TEMP_NORMAL: return "NORMAL";
    case SYSTEM_TEMP_WARNING: return "WARNING";
    case SYSTEM_TEMP_CRITICAL: return "CRITICAL";
    default: return "UNAVAILABLE";
  }
}

void init_system_health() {
  // Try valid ESP32-P4 measurement windows from normal operating
  // temperatures through the high-temperature range.
  const int range_min_c[] = {-10, 20, 50};
  const int range_max_c[] = { 80, 100, 125};
  esp_err_t last_result = ESP_FAIL;

  for (size_t index = 0; index < 3; index++) {
    temperature_sensor_config_t config =
      TEMPERATURE_SENSOR_CONFIG_DEFAULT(range_min_c[index], range_max_c[index]);

    p4_temperature_handle = nullptr;
    last_result = temperature_sensor_install(&config, &p4_temperature_handle);

    if (last_result != ESP_OK) {
      Serial.printf(
        "[HEALTH] Temperature range %d..%d C install failed: %s\n",
        range_min_c[index], range_max_c[index], esp_err_to_name(last_result)
      );
      continue;
    }

    last_result = temperature_sensor_enable(p4_temperature_handle);

    if (last_result == ESP_OK) {
      system_health.temperature_available = true;
      system_health.temperature_state = SYSTEM_TEMP_NORMAL;
      Serial.printf(
        "[HEALTH] ESP32-P4 internal temperature sensor enabled: %d..%d C\n",
        range_min_c[index], range_max_c[index]
      );
      return;
    }

    Serial.printf(
      "[HEALTH] Temperature range %d..%d C enable failed: %s\n",
      range_min_c[index], range_max_c[index], esp_err_to_name(last_result)
    );
    temperature_sensor_uninstall(p4_temperature_handle);
    p4_temperature_handle = nullptr;
  }

  system_health.temperature_available = false;
  system_health.temperature_state = SYSTEM_TEMP_UNAVAILABLE;
  system_health.read_errors++;
  Serial.printf(
    "[HEALTH] ESP32-P4 temperature sensor unavailable: %s\n",
    esp_err_to_name(last_result)
  );
}

void update_system_health() {
  uint32_t now_ms = millis();
  if (now_ms - system_health.last_update_ms < SYSTEM_HEALTH_UPDATE_MS) return;
  system_health.last_update_ms = now_ms;
  system_health.heap_free_bytes = ESP.getFreeHeap();
  system_health.heap_min_bytes = ESP.getMinFreeHeap();
  system_health.heap_largest_block_bytes =
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  system_health.internal_free_bytes = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  system_health.psram_total_bytes = ESP.getPsramSize();
  system_health.psram_free_bytes = ESP.getFreePsram();
  if (timing_task_handle) system_health.timing_stack_free_bytes = uxTaskGetStackHighWaterMark(timing_task_handle) * sizeof(StackType_t);
  if (network_task_handle) system_health.network_stack_free_bytes = uxTaskGetStackHighWaterMark(network_task_handle) * sizeof(StackType_t);
  if (display_task_handle) system_health.display_stack_free_bytes = uxTaskGetStackHighWaterMark(display_task_handle) * sizeof(StackType_t);
  if (p4_temperature_handle) {
    float t=0;
    if (temperature_sensor_get_celsius(p4_temperature_handle,&t)==ESP_OK) {
      system_health.temperature_available=true;
      system_health.temperature_c=t;
      if (system_health.temperature_max_c==0 || t>system_health.temperature_max_c) system_health.temperature_max_c=t;
      system_health.temperature_state = t>=P4_DIE_TEMP_CRITICAL_C ? SYSTEM_TEMP_CRITICAL : (t>=P4_DIE_TEMP_WARNING_C ? SYSTEM_TEMP_WARNING : SYSTEM_TEMP_NORMAL);
    } else {
      system_health.temperature_available=false;
      system_health.temperature_state=SYSTEM_TEMP_UNAVAILABLE;
      system_health.read_errors++;
    }
  }
}

// ============================================================
// Web Server Runtime
// ============================================================

struct StatusJsonWriter {
  char *buffer;
  size_t capacity;
  size_t length;
  bool overflow;
};

bool status_json_appendf(
  StatusJsonWriter *writer,
  const char *format,
  ...
) {
  if (!writer ||
      !writer->buffer ||
      writer->capacity == 0 ||
      writer->overflow ||
      !format) {
    return false;
  }

  if (writer->length >= writer->capacity) {
    writer->overflow = true;
    return false;
  }

  va_list args;
  va_start(args, format);

  int written = vsnprintf(
    writer->buffer + writer->length,
    writer->capacity - writer->length,
    format,
    args
  );

  va_end(args);

  if (written < 0 ||
      (size_t)written >=
        (writer->capacity - writer->length)) {
    writer->overflow = true;
    return false;
  }

  writer->length += (size_t)written;
  return true;
}

void status_json_write_identity(
  StatusJsonWriter *writer
) {
  char device_name_json[80];
  char node_id_json[80];
  char role_json[64];
  char site_name_json[96];

  json_escape_string(
    system_config.device_name,
    device_name_json,
    sizeof(device_name_json)
  );
  json_escape_string(
    system_config.node_id,
    node_id_json,
    sizeof(node_id_json)
  );
  json_escape_string(
    system_config.role,
    role_json,
    sizeof(role_json)
  );
  json_escape_string(
    system_config.site_name,
    site_name_json,
    sizeof(site_name_json)
  );

  status_json_appendf(
    writer,
    "\"device\":\"%s\","
    "\"identity\":{"
      "\"node_id\":\"%s\","
      "\"role\":\"%s\","
      "\"site_name\":\"%s\""
    "},"
    "\"release\":\"%s\"",
    device_name_json,
    node_id_json,
    role_json,
    site_name_json,
    SKYTIME_VERSION
  );
}

void status_json_write_timing(
  StatusJsonWriter *writer,
  const NtpTimingSnapshot &timing
) {
  const char *gps_state =
    gps_data.state == GPS_LOCKED ? "LOCKED" :
    gps_data.state == GPS_HOLDOVER ? "HOLDOVER" :
    gps_data.state == GPS_STALE ? "STALE" :
    "SEARCH";

  status_json_appendf(
    writer,
    ",\"gps\":{"
      "\"state\":\"%s\","
      "\"satellites\":%u,"
      "\"lat\":%.6f,"
      "\"lon\":%.6f"
    "},"
    "\"pps\":{"
      "\"state\":%d,"
      "\"count\":%lu,"
      "\"good\":%lu,"
      "\"bad\":%lu,"
      "\"rejected\":%lu,"
      "\"jitter_us\":%llu,"
      "\"align_bad\":%lu"
    "},"
    "\"ntp\":{"
      "\"disciplined\":%s,"
      "\"holdover\":%s,"
      "\"rx\":%lu,"
      "\"tx\":%lu,"
      "\"bad_mode\":%lu,"
      "\"not_ready\":%lu,"
      "\"epoch\":%lu"
    "}",
    gps_state,
    gps_data.satellites,
    gps_data.latitude,
    gps_data.longitude,
    (int)pps_data.state,
    (unsigned long)pps_data.pps_count,
    (unsigned long)pps_data.valid_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long)atomic_read_pps_rejected(),
    (unsigned long long)pps_data.jitter_us,
    (unsigned long)pps_data.align_bad_count,
    timing.disciplined ? "true" : "false",
    timing.holdover ? "true" : "false",
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)ntp_runtime.packets_bad_mode,
    (unsigned long)ntp_runtime.packets_not_ready,
    (unsigned long)timing.current_epoch
  );
}

void status_json_write_network_storage(
  StatusJsonWriter *writer
) {
  uint32_t link_up_seconds =
    ethernet_runtime.link_up ?
    (millis() - ethernet_runtime.last_event_ms) / 1000UL :
    0;

  uint64_t sd_free_bytes =
    sd_runtime.total_bytes > sd_runtime.used_bytes ?
    sd_runtime.total_bytes - sd_runtime.used_bytes :
    0;

  float sd_used_percent =
    sd_runtime.total_bytes ?
    100.0f *
      (float)sd_runtime.used_bytes /
      (float)sd_runtime.total_bytes :
    0.0f;

  status_json_appendf(
    writer,
    ",\"ethernet\":{"
      "\"state\":\"%s\","
      "\"ip\":\"%s\","
      "\"gateway\":\"%s\","
      "\"subnet\":\"%s\","
      "\"dns\":\"%s\","
      "\"link\":%s,"
      "\"link_uptime_seconds\":%lu,"
      "\"link_up_count\":%lu,"
      "\"link_down_count\":%lu"
    "},"
    "\"sd\":{"
      "\"state\":\"%s\","
      "\"mounted\":%s,"
      "\"test\":\"%s\","
      "\"type\":\"%s\","
      "\"total_mb\":%llu,"
      "\"used_mb\":%llu,"
      "\"free_mb\":%llu,"
      "\"used_percent\":%.2f,"
      "\"errors\":%lu"
    "}",
    ethernet_state_text(ethernet_runtime.state),
    ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip,
    ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway,
    ethernet_runtime.got_ip ? ethernet_runtime.subnet : network_config.subnet,
    network_config.dns,
    ethernet_runtime.link_up ? "true" : "false",
    (unsigned long)link_up_seconds,
    (unsigned long)ethernet_runtime.link_up_count,
    (unsigned long)ethernet_runtime.link_down_count,
    sd_state_text(sd_runtime.state),
    sd_runtime.mounted ? "true" : "false",
    sd_runtime.test_passed ? "PASS" : "NO",
    sd_runtime.card_type,
    (unsigned long long)(sd_runtime.total_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.used_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_free_bytes / (1024ULL * 1024ULL)),
    sd_used_percent,
    (unsigned long)sd_runtime.errors
  );
}

void status_json_write_system_health(
  StatusJsonWriter *writer
) {
  float heap_fragmentation = 0.0f;

  if (system_health.internal_free_bytes > 0) {
    heap_fragmentation =
      100.0f *
      (
        1.0f -
        (float)system_health.heap_largest_block_bytes /
        (float)system_health.internal_free_bytes
      );

    if (heap_fragmentation < 0.0f) heap_fragmentation = 0.0f;
    if (heap_fragmentation > 100.0f) heap_fragmentation = 100.0f;
  }

  uint32_t uptime_ms = millis();

  status_json_appendf(
    writer,
    ",\"system\":{"
      "\"uptime_ms\":%lu,"
      "\"uptime_seconds\":%lu,"
      "\"reset_reason\":\"%s\","
      "\"cpu_frequency_mhz\":%u"
    "},"
    "\"health\":{"
      "\"temperature_available\":%s,"
      "\"temperature_c\":%.1f,"
      "\"temperature_max_c\":%.1f,"
      "\"temperature_state\":\"%s\","
      "\"temperature_warning_c\":%.1f,"
      "\"temperature_critical_c\":%.1f,"
      "\"heap_free\":%lu,"
      "\"heap_minimum\":%lu,"
      "\"largest_block\":%lu,"
      "\"heap_fragmentation_percent\":%.1f,"
      "\"internal_free\":%lu,"
      "\"psram_total\":%lu,"
      "\"psram_free\":%lu,"
      "\"read_errors\":%lu"
    "},"
    "\"tasks\":{"
      "\"timing_stack_free\":%lu,"
      "\"network_stack_free\":%lu,"
      "\"display_stack_free\":%lu"
    "}",
    (unsigned long)uptime_ms,
    (unsigned long)(uptime_ms / 1000UL),
    reset_reason_text(esp_reset_reason()),
    (unsigned int)getCpuFrequencyMhz(),
    system_health.temperature_available ? "true" : "false",
    system_health.temperature_c,
    system_health.temperature_max_c,
    system_temperature_state_text(system_health.temperature_state),
    P4_DIE_TEMP_WARNING_C,
    P4_DIE_TEMP_CRITICAL_C,
    (unsigned long)system_health.heap_free_bytes,
    (unsigned long)system_health.heap_min_bytes,
    (unsigned long)system_health.heap_largest_block_bytes,
    heap_fragmentation,
    (unsigned long)system_health.internal_free_bytes,
    (unsigned long)system_health.psram_total_bytes,
    (unsigned long)system_health.psram_free_bytes,
    (unsigned long)system_health.read_errors,
    (unsigned long)system_health.timing_stack_free_bytes,
    (unsigned long)system_health.network_stack_free_bytes,
    (unsigned long)system_health.display_stack_free_bytes
  );
}

void status_json_write_logging_reset(
  StatusJsonWriter *writer
) {
  status_json_appendf(
    writer,
    ",\"logging\":{"
      "\"temperature_file\":\"%s\","
      "\"temperature_entries\":%lu,"
      "\"temperature_errors\":%lu,"
      "\"temperature_last_epoch\":%lu,"
      "\"event_entries\":%lu,"
      "\"event_errors\":%lu,"
      "\"event_rotations\":%lu,"
      "\"event_max_bytes\":%lu,"
      "\"event_total_files\":%u,"
      "\"temperature_interval_seconds\":%lu,"
      "\"temperature_keep_months\":%u"
    "},"
    "\"reset_history\":{"
      "\"initialized\":%s,"
      "\"total_boots\":%lu,"
      "\"power_on\":%lu,"
      "\"software\":%lu,"
      "\"watchdog\":%lu,"
      "\"panic\":%lu,"
      "\"brownout\":%lu,"
      "\"external\":%lu,"
      "\"unknown\":%lu,"
      "\"unexpected\":%lu,"
      "\"previous_uptime_seconds\":%lu,"
      "\"longest_uptime_seconds\":%lu,"
      "\"last_checkpoint_seconds\":%lu,"
      "\"checkpoint_interval_seconds\":%lu"
    "}",
    log_runtime.temperature_log_path,
    (unsigned long)log_runtime.temperature_entries,
    (unsigned long)log_runtime.temperature_errors,
    (unsigned long)log_runtime.temperature_last_sample_epoch,
    (unsigned long)log_runtime.event_entries,
    (unsigned long)log_runtime.event_errors,
    (unsigned long)log_runtime.event_rotations,
    (unsigned long)EVENT_LOG_MAX_BYTES,
    (unsigned int)EVENT_LOG_TOTAL_FILES,
    (unsigned long)(TEMPERATURE_LOG_INTERVAL_MS / 1000UL),
    (unsigned int)TEMPERATURE_LOG_KEEP_MONTHS,
    reset_history.initialized ? "true" : "false",
    (unsigned long)reset_history.total_boots,
    (unsigned long)reset_history.power_on_resets,
    (unsigned long)reset_history.software_resets,
    (unsigned long)reset_history.watchdog_resets,
    (unsigned long)reset_history.panic_resets,
    (unsigned long)reset_history.brownout_resets,
    (unsigned long)reset_history.external_resets,
    (unsigned long)reset_history.unknown_resets,
    (unsigned long)reset_history.unexpected_resets,
    (unsigned long)reset_history.previous_uptime_seconds,
    (unsigned long)reset_history.longest_uptime_seconds,
    (unsigned long)reset_history.last_checkpoint_seconds,
    (unsigned long)(UPTIME_CHECKPOINT_INTERVAL_MS / 1000UL)
  );
}

void status_json_write_services(
  StatusJsonWriter *writer
) {
  char last_remote_ip[16];

  format_ip_address(
    ntp_runtime.last_remote_ip,
    last_remote_ip,
    sizeof(last_remote_ip)
  );

  status_json_appendf(
    writer,
    ",\"config\":{"
      "\"state\":\"%s\","
      "\"network\":%s,"
      "\"system\":%s"
    "},"
    "\"stats\":{"
      "\"uptime_seconds\":%lu,"
      "\"gps_lock_seconds\":%lu,"
      "\"web_requests\":%lu,"
      "\"api_requests\":%lu"
    "},"
    "\"ntp_stats\":{"
      "\"requests\":%lu,"
      "\"replies\":%lu,"
      "\"bad_mode\":%lu,"
      "\"not_ready\":%lu"
    "},"
    "\"client\":{"
      "\"ip\":\"%s\","
      "\"port\":%u,"
      "\"size\":%u,"
      "\"last_query_epoch\":%lu"
    "},"
    "\"web\":{"
      "\"state\":\"%s\","
      "\"requests\":%lu"
    "}",
    config_state_text(config_runtime.state),
    config_runtime.network_loaded ? "true" : "false",
    config_runtime.system_loaded ? "true" : "false",
    (unsigned long)(millis() / 1000UL),
    (unsigned long)gps_lock_seconds(),
    (unsigned long)web_runtime.requests_total,
    (unsigned long)web_runtime.requests_api,
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)ntp_runtime.packets_bad_mode,
    (unsigned long)ntp_runtime.packets_not_ready,
    last_remote_ip,
    ntp_runtime.last_remote_port,
    ntp_runtime.last_packet_size,
    (unsigned long)operational_stats.last_ntp_query_epoch,
    web_state_text(web_runtime.state),
    (unsigned long)web_runtime.requests_total
  );
}

void build_status_json(
  char *buffer,
  size_t buffer_size
) {
  if (!buffer || buffer_size == 0) {
    return;
  }

  buffer[0] = '\0';

  StatusJsonWriter writer = {
    buffer,
    buffer_size,
    0,
    false
  };

  NtpTimingSnapshot status_timing = {};
  snapshot_ntp_timing(&status_timing);

  status_json_appendf(&writer, "{");
  status_json_write_identity(&writer);
  status_json_write_timing(&writer, status_timing);
  status_json_write_network_storage(&writer);
  status_json_write_system_health(&writer);
  status_json_write_logging_reset(&writer);
  status_json_write_services(&writer);
  status_json_appendf(&writer, "}");

  if (writer.overflow) {
    web_runtime.errors++;

    snprintf(
      buffer,
      buffer_size,
      "{\"ok\":false,\"error\":\"status_json_truncated\"}"
    );
  }
}

void build_status_html(char *buffer, size_t buffer_size) {
  if (!buffer || buffer_size == 0) {
    return;
  }

  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  int written = snprintf(
    buffer,
    buffer_size,
    "<!doctype html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>SkyTime Status</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:20px;}"
    ".card{background:#1d1d1d;border:1px solid #333;border-radius:8px;padding:14px;margin:12px 0;}"
    ".ok{color:#5cff7a}.warn{color:#ffd45c}.bad{color:#ff6666}"
    "table{border-collapse:collapse;width:100%%}td{padding:4px 8px;border-bottom:1px solid #333}"
    "h1,h2{color:#67d7ff}"
    "</style></head><body>"
    "<h1>SkyTime</h1>"
    "<div class='card'><h2>Timing</h2><table>"
    "<tr><td>GPS</td><td class='%s'>%s SAT:%u</td></tr>"
    "<tr><td>PPS</td><td class='%s'>State:%d Count:%lu Bad:%lu Jitter:%llu us</td></tr>"
    "<tr><td>NTP</td><td class='%s'>Disciplined:%s RX:%lu TX:%lu</td></tr>"
    "<tr><td>Epoch</td><td>%lu</td></tr>"
    "</table></div>"
    "<div class='card'><h2>Network</h2><table>"
    "<tr><td>Ethernet</td><td class='%s'>%s</td></tr>"
    "<tr><td>IP</td><td>%s</td></tr>"
    "<tr><td>Gateway</td><td>%s</td></tr>"
    "</table></div>"
    "<div class='card'><h2>Storage / Config</h2><table>"
    "<tr><td>SD</td><td class='%s'>%s %s</td></tr>"
    "<tr><td>Config</td><td>%s</td></tr>"
    "<tr><td>Web</td><td>%s Requests:%lu</td></tr>"
    "</table></div>"
    "<p><a href='/api/status' style='color:#67d7ff'>JSON Status</a></p>"
    "</body></html>",
    gps_data.state == GPS_LOCKED ? "ok" : "warn",
    gps_data.state == GPS_LOCKED ? "LOCKED" :
      (gps_data.state == GPS_HOLDOVER ? "HOLDOVER" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites,
    pps_data.state == PPS_LOCKED ? "ok" : "warn",
    (int)pps_data.state,
    (unsigned long)pps_data.pps_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long long)pps_data.jitter_us,
    timing.disciplined ? "ok" : "bad",
    timing.disciplined ? "YES" : "NO",
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)timing.current_epoch,
    ethernet_runtime.got_ip ? "ok" : "bad",
    ethernet_state_text(ethernet_runtime.state),
    ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip,
    ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway,
    sd_runtime.state == SD_STATE_MOUNTED ? "ok" : "warn",
    sd_state_text(sd_runtime.state),
    sd_runtime.card_type,
    config_state_text(config_runtime.state),
    web_state_text(web_runtime.state),
    (unsigned long)web_runtime.requests_total
  );

  if (written < 0 ||
      (size_t)written >= buffer_size) {
    web_runtime.errors++;

    snprintf(
      buffer,
      buffer_size,
      "<!doctype html><html><body>"
      "<h1>SkyTime</h1>"
      "<p>Status page unavailable: "
      "response truncated.</p>"
      "</body></html>"
    );
  }
}

// Deferred startup events are flushed after valid UTC is available.
void flush_pending_event_logs();

// ============================================================
// Persistent Reset History Runtime
// ============================================================

void init_reset_history() {
  Preferences prefs;

  if (!prefs.begin(RESET_HISTORY_NAMESPACE, false)) {
    Serial.println("[RESET-HISTORY] NVS open failed");
    return;
  }

  reset_history.total_boots = prefs.getULong("boots", 0);
  reset_history.power_on_resets = prefs.getULong("poweron", 0);
  reset_history.software_resets = prefs.getULong("software", 0);
  reset_history.watchdog_resets = prefs.getULong("watchdog", 0);
  reset_history.panic_resets = prefs.getULong("panic", 0);
  reset_history.brownout_resets = prefs.getULong("brownout", 0);
  reset_history.external_resets = prefs.getULong("external", 0);
  reset_history.unknown_resets = prefs.getULong("unknown", 0);
  reset_history.unexpected_resets = prefs.getULong("unexpected", 0);
  reset_history.previous_uptime_seconds = prefs.getULong("checkpoint", 0);
  reset_history.longest_uptime_seconds = prefs.getULong("longest", 0);

  if (reset_history.previous_uptime_seconds > reset_history.longest_uptime_seconds) {
    reset_history.longest_uptime_seconds = reset_history.previous_uptime_seconds;
  }

  reset_history.total_boots++;

  esp_reset_reason_t reason = esp_reset_reason();
  bool unexpected = false;

  switch (reason) {
    case ESP_RST_POWERON:
      reset_history.power_on_resets++;
      break;

    case ESP_RST_SW:
      reset_history.software_resets++;
      break;

    case ESP_RST_EXT:
      reset_history.external_resets++;
      break;

    case ESP_RST_PANIC:
      reset_history.panic_resets++;
      unexpected = true;
      break;

    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      reset_history.watchdog_resets++;
      unexpected = true;
      break;

    case ESP_RST_BROWNOUT:
      reset_history.brownout_resets++;
      unexpected = true;
      break;

    case ESP_RST_DEEPSLEEP:
      // Deep-sleep wakeups are expected, but are grouped with software resets.
      reset_history.software_resets++;
      break;

    case ESP_RST_SDIO:
    default:
      reset_history.unknown_resets++;
      unexpected = true;
      break;
  }

  if (unexpected) {
    reset_history.unexpected_resets++;
  }

  prefs.putULong("boots", reset_history.total_boots);
  prefs.putULong("poweron", reset_history.power_on_resets);
  prefs.putULong("software", reset_history.software_resets);
  prefs.putULong("watchdog", reset_history.watchdog_resets);
  prefs.putULong("panic", reset_history.panic_resets);
  prefs.putULong("brownout", reset_history.brownout_resets);
  prefs.putULong("external", reset_history.external_resets);
  prefs.putULong("unknown", reset_history.unknown_resets);
  prefs.putULong("unexpected", reset_history.unexpected_resets);
  prefs.putULong("longest", reset_history.longest_uptime_seconds);

  // Begin a fresh checkpoint for this boot.
  prefs.putULong("checkpoint", 0);
  prefs.end();

  reset_history.last_checkpoint_seconds = 0;
  reset_history.last_checkpoint_ms = millis();
  reset_history.initialized = true;

  Serial.printf(
    "[RESET-HISTORY] Boot:%lu Previous:%lus Longest:%lus Unexpected:%lu\n",
    (unsigned long)reset_history.total_boots,
    (unsigned long)reset_history.previous_uptime_seconds,
    (unsigned long)reset_history.longest_uptime_seconds,
    (unsigned long)reset_history.unexpected_resets
  );
}

void save_uptime_checkpoint_now() {
  if (!reset_history.initialized) return;

  uint32_t uptime_seconds = millis() / 1000UL;

  Preferences prefs;
  if (!prefs.begin(RESET_HISTORY_NAMESPACE, false)) {
    return;
  }

  prefs.putULong("checkpoint", uptime_seconds);

  if (uptime_seconds > reset_history.longest_uptime_seconds) {
    reset_history.longest_uptime_seconds = uptime_seconds;
    prefs.putULong("longest", reset_history.longest_uptime_seconds);
  }

  prefs.end();

  reset_history.last_checkpoint_seconds = uptime_seconds;
  reset_history.last_checkpoint_ms = millis();
}

void update_uptime_checkpoint() {
  if (!reset_history.initialized) return;

  if ((uint32_t)(millis() - reset_history.last_checkpoint_ms) <
      UPTIME_CHECKPOINT_INTERVAL_MS) {
    return;
  }

  save_uptime_checkpoint_now();
}

// ============================================================
// Operational Statistics
// ============================================================

void update_operational_stats() {
  bool gps_locked_now = (gps_data.state == GPS_LOCKED);

  if (gps_locked_now && !operational_stats.gps_lock_timer_active) {
    operational_stats.gps_lock_start_ms = millis();
    operational_stats.gps_lock_timer_active = true;
  }

  if (!gps_locked_now) {
    operational_stats.gps_lock_timer_active = false;
    operational_stats.gps_lock_start_ms = 0;
  }
}

uint32_t gps_lock_seconds() {
  if (!operational_stats.gps_lock_timer_active) {
    return 0;
  }

  return (millis() - operational_stats.gps_lock_start_ms) / 1000UL;
}


// ============================================================
// Debug Output
// ============================================================

void debug_output() {
  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  Serial.println();
  Serial.println("=======================================");

  Serial.printf(
    "[GPS] %s | SAT:%u | Lat:%.5f | Lon:%.5f\n",
    gps_data.state == GPS_LOCKED ? "LOCKED" :
      (gps_data.state == GPS_HOLDOVER ? "HOLDOVER" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites,
    gps_data.latitude,
    gps_data.longitude
  );

  Serial.printf(
    "[GPS] Time: %02d:%02d:%02d | Date: %02d/%02d/%04d\n",
    gps_data.hour,
    gps_data.minute,
    gps_data.second,
    gps_data.month,
    gps_data.day,
    gps_data.year
  );

  Serial.printf(
    "[PPS] Count:%lu | State:%d | Interval:%llu us | Good:%lu | Bad:%lu | Rejected:%lu\n",
    (unsigned long)pps_data.pps_count,
    (int)pps_data.state,
    (unsigned long long)pps_data.last_interval_us,
    (unsigned long)pps_data.valid_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long)atomic_read_pps_rejected()
  );

  Serial.printf(
    "[PPS] Min:%llu us | Max:%llu us | Avg:%llu us | Jitter:%llu us | Align G:%lu B:%lu\n",
    (unsigned long long)pps_data.min_interval_us,
    (unsigned long long)pps_data.max_interval_us,
    (unsigned long long)pps_data.avg_interval_us,
    (unsigned long long)pps_data.jitter_us,
    (unsigned long)pps_data.align_good_count,
    (unsigned long)pps_data.align_bad_count
  );

  if (timing.current_epoch > 0) {
    Serial.printf(
      "[NTP] Epoch:%lu | Fraction:0x%08lX | us:%lu | Disciplined:%s | Holdover:%s\n",
      (unsigned long)timing.current_epoch,
      (unsigned long)timing.ntp_fraction,
      (unsigned long)timing.microseconds_since_pps,
      timing.disciplined ? "YES" : "NO",
      timing.holdover ? "YES" : "NO"
    );

    Serial.printf(
      "[NTP] Precision:2^%d | Root Delay:%lu us | Dispersion:%lu us\n",
      timing.ntp_precision,
      (unsigned long)timing.root_delay,
      (unsigned long)timing.root_dispersion
    );
  } else {
    Serial.println("[NTP] Waiting for GPS/PPS discipline...");
  }

  Serial.printf(
    "[ETH] State:%s | Link:%s | IP:%s | GW:%s | Static:%s | Up:%lu Down:%lu\n",
    ethernet_state_text(ethernet_runtime.state),
    ethernet_runtime.link_up ? "UP" : "DOWN",
    ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip,
    ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway,
    ethernet_runtime.static_ip_applied ? "YES" : "NO",
    (unsigned long)ethernet_runtime.link_up_count,
    (unsigned long)ethernet_runtime.link_down_count
  );

  Serial.printf(
    "[NTPD] State:%s | UDP:%s | RX:%lu | TX:%lu | BadMode:%lu | NotReady:%lu | Replies:%s\n",
    ntp_state_text(ntp_runtime.state),
    ntp_runtime.udp_started ? "UP" : "DOWN",
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)ntp_runtime.packets_bad_mode,
    (unsigned long)ntp_runtime.packets_not_ready,
    ntp_runtime.replies_enabled ? "ON" : "OFF"
  );

  Serial.printf(
    "[UDP123] Total:%lu Valid:%lu Short:%lu Ignored:%lu Last:%u\n",
    (unsigned long)udp123_debug.total_rx,
    (unsigned long)udp123_debug.ntp_rx,
    (unsigned long)udp123_debug.short_rx,
    (unsigned long)udp123_debug.ignored_rx,
    (unsigned int)udp123_debug.last_size
  );

  if (udp123_debug.total_rx > 0) {
    Serial.printf(
      "[UDP123] Last From:%u.%u.%u.%u:%u\n",
      (unsigned int)udp123_debug.last_ip[0],
      (unsigned int)udp123_debug.last_ip[1],
      (unsigned int)udp123_debug.last_ip[2],
      (unsigned int)udp123_debug.last_ip[3],
      udp123_debug.last_port
    );
  }

  Serial.printf(
    "[UDP4123] Total:%lu Last:%u\n",
    (unsigned long)udp4123_debug.total_rx,
    (unsigned int)udp4123_debug.last_size
  );

  if (udp4123_debug.total_rx > 0) {
    Serial.printf(
      "[UDP4123] Last From:%u.%u.%u.%u:%u\n",
      (unsigned int)udp4123_debug.last_ip[0],
      (unsigned int)udp4123_debug.last_ip[1],
      (unsigned int)udp4123_debug.last_ip[2],
      (unsigned int)udp4123_debug.last_ip[3],
      udp4123_debug.last_port
    );
  }

  Serial.printf(
    "[SD] State:%s | Mounted:%s | Test:%s | Type:%s | Total:%llu MB | Used:%llu MB | Err:%lu | Last:%s\n",
    sd_state_text(sd_runtime.state),
    sd_runtime.mounted ? "YES" : "NO",
    sd_runtime.test_passed ? "PASS" : "NO",
    sd_runtime.card_type,
    (unsigned long long)(sd_runtime.total_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.used_bytes / (1024ULL * 1024ULL)),
    (unsigned long)sd_runtime.errors,
    sd_runtime.last_error
  );

  Serial.printf(
    "[CFG] State:%s | Network:%s | System:%s | DefaultsCreated:%s | Errors:%lu | Last:%s\n",
    config_state_text(config_runtime.state),
    config_runtime.network_loaded ? "YES" : "NO",
    config_runtime.system_loaded ? "YES" : "NO",
    config_runtime.defaults_created ? "YES" : "NO",
    (unsigned long)config_runtime.errors,
    config_runtime.last_error
  );

  Serial.printf(
    "[WEB] State:%s | Started:%s | Ticks:%lu | Req:%lu | API:%lu | Static:%lu | 404:%lu | Raw8080:%lu | Last:%s\n",
    web_state_text(web_runtime.state),
    web_runtime.started ? "YES" : "NO",
    (unsigned long)web_runtime.handle_ticks,
    (unsigned long)web_runtime.requests_total,
    (unsigned long)web_runtime.requests_api,
    (unsigned long)web_runtime.requests_static,
    (unsigned long)web_runtime.requests_not_found,
    (unsigned long)web_runtime.raw8080_requests,
    web_runtime.last_uri
  );

  Serial.printf(
    "[CFG] WebEnabledRuntime:%s | Device:%s | Timeout:%lu\n",
    system_config.web_enabled ? "true" : "false",
    system_config.device_name,
    (unsigned long)system_config.screen_timeout_seconds
  );

  Serial.printf(
    "[CFG] Identity:%s | Role:%s | Site Name:%s\n",
    system_config.node_id,
    system_config.role,
    system_config.site_name
  );

  Serial.printf(
    "[LOG] Enabled:%s | Dir:%s | NTP:%lu Err:%lu | Event:%lu Err:%lu | Last:%s\n",
    log_runtime.enabled ? "YES" : "NO",
    log_runtime.log_dir_ready ? "YES" : "NO",
    (unsigned long)log_runtime.ntp_entries,
    (unsigned long)log_runtime.ntp_errors,
    (unsigned long)log_runtime.event_entries,
    (unsigned long)log_runtime.event_errors,
    log_runtime.last_error
  );

  Serial.print("[DISPLAY] Mode:");
  Serial.print((int)screen_manager_get_current());
  Serial.print(" | ScreenAge:");
  Serial.print((unsigned long)screen_manager_get_elapsed_ms());
  Serial.print(" ms | RedrawPending:");
  Serial.println(display_cache.force_full_redraw ? "YES" : "NO");

  Serial.println("=======================================");
  Serial.println();
}


// ============================================================
// FreeRTOS Tasks
// ============================================================

void timing_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);

  while (true) {
    process_gps();
    update_pps_state();
    calculate_ntp_timestamp();
    handle_button();

    vTaskDelayUntil(&last_wake, period);
  }
}


void network_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5);

  // Allow network event handlers to settle after startup.
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.println("[WEB] Network task servicing web");

  while (true) {
    update_ethernet_runtime();
    update_wifi_config_ap();
    update_ntp_listener();
    update_sd_runtime();
    update_system_health();
    update_uptime_checkpoint();

    // Flush queued startup events before later runtime log entries.
    flush_pending_event_logs();

    update_temperature_history_log();
    update_monitored_event_transitions();
    update_web_server();

    uint32_t now_ms = millis();
    if (now_ms - last_debug_time_ms >= DEBUG_INTERVAL_MS) {
      last_debug_time_ms = now_ms;
      debug_output();
    }

    vTaskDelayUntil(&last_wake, period);
  }
}

void display_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);

  while (true) {
    scheduler_update_display();
    vTaskDelayUntil(&last_wake, period);
  }
}

// ============================================================
// Reset Diagnostics
// ============================================================

const char *reset_reason_text(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXTERNAL";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

// ============================================================
// Arduino Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  esp_reset_reason_t reset_reason = esp_reset_reason();
  boot_is_power_on_reset = (reset_reason == ESP_RST_POWERON);

  Serial.print("[RESET] Reason: ");
  Serial.print(reset_reason_text(reset_reason));
  Serial.print(" (");
  Serial.print((int)reset_reason);
  Serial.println(")");

  init_systems();
  init_system_health();
  init_reset_history();
  detect_wifi_boot_button();

  screen_manager_init();
  display_startup();

  if (wifi_boot_button_requested) {
    wait_for_wifi_boot_button_release();
  }

  Serial.println("[BOOT] Starting SD card");
  init_sd_card();

  Serial.println("[BOOT] Starting configuration");
  init_config();

  update_display_brightness(true);

  Serial.println("[BOOT] Starting logging");
  init_logging();

  Serial.println("[BOOT] Starting Wi-Fi configuration AP");
  init_wifi_config_ap();

  Serial.println("[BOOT] Starting Ethernet");
  init_ethernet();
  init_ntp_listener();

  Serial.println("[BOOT] Web server will start from network task");
  Serial.println("[BOOT] Web enabled is controlled by /config/system.json");


  Serial.println("[BOOT] System initialized successfully");
  Serial.print("[BOOT] Release: ");
  Serial.println(SKYTIME_VERSION);
  Serial.println("[TASK] Starting timing/control, network, and display tasks");

  BaseType_t timing_ok = xTaskCreatePinnedToCore(
    timing_task,
    "SkyTimeTiming",
    TASK_TIMING_STACK,
    nullptr,
    TASK_TIMING_PRIORITY,
    &timing_task_handle,
    TASK_TIMING_CORE
  );

  BaseType_t network_ok = xTaskCreatePinnedToCore(
    network_task,
    "SkyTimeNetwork",
    TASK_NETWORK_STACK,
    nullptr,
    TASK_NETWORK_PRIORITY,
    &network_task_handle,
    TASK_NETWORK_CORE
  );

  BaseType_t display_ok = xTaskCreatePinnedToCore(
    display_task,
    "SkyTimeDisplay",
    TASK_DISPLAY_STACK,
    nullptr,
    TASK_DISPLAY_PRIORITY,
    &display_task_handle,
    TASK_DISPLAY_CORE
  );

  if (timing_ok != pdPASS || network_ok != pdPASS || display_ok != pdPASS) {
    Serial.println("[TASK] Failed to create one or more tasks");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[TASK] Tasks running");
}

void loop() {
  update_operational_stats();
  flush_pending_event_logs();
  // Runtime work is handled by the FreeRTOS tasks.
  delay(1000);
}