#pragma once

#include <Arduino.h>

// ============================================================
// SkyTime Shared Types
// ============================================================

// GNSS collection capacities used by shared structures.
#define GNSS_MAX_SATELLITES       24
#define GNSS_MAX_USED_PRNS        12
#define GNSS_NMEA_LINE_MAX        128

enum DisplayMode {
  PRIMARY_SCREEN = 0,
  NETWORK_SCREEN = 1,
  DIAGNOSTIC_SCREEN = 2
};

enum ScreenEvent {
  SCREEN_EVENT_NONE = 0,
  SCREEN_EVENT_SHORT_PRESS = 1,
  SCREEN_EVENT_TIMEOUT = 2,
  SCREEN_EVENT_FORCE_MAIN = 3
};

enum GPSLockState {
  GPS_SEARCHING = 0,
  GPS_LOCKED = 1,
  GPS_STALE = 2,
  GPS_HOLDOVER = 3
};

enum PPSLockState {
  PPS_WAITING = 0,
  PPS_LOCKED = 1,
  PPS_BAD_INTERVAL = 2,
  PPS_TIMEOUT = 3,
  PPS_ALIGN_WAIT = 4,
  PPS_ALIGN_BAD = 5
};

enum ButtonPhysicalState {
  BUTTON_IDLE = 0,
  BUTTON_DEBOUNCE_PRESS = 1,
  BUTTON_HELD = 2,
  BUTTON_DEBOUNCE_RELEASE = 3
};

struct PPSCapture {
  volatile uint64_t pps_time_us;
  volatile uint32_t pps_count;
  volatile uint32_t rejected_edges;
  volatile bool pps_triggered;
};

struct PPSData {
  uint64_t current_pps_time_us;
  uint64_t previous_pps_time_us;
  uint64_t last_interval_us;

  uint64_t min_interval_us;
  uint64_t max_interval_us;
  uint64_t avg_interval_us;
  uint64_t jitter_us;
  uint64_t jitter_accum_us;
  uint32_t jitter_samples;

  uint32_t pps_count;
  uint32_t last_pps_epoch;
  uint32_t valid_count;
  uint32_t bad_count;
  uint32_t align_good_count;
  uint32_t align_bad_count;

  PPSLockState state;
  bool edge_seen;
  bool gps_aligned;
};

struct TimingData {
  uint32_t current_epoch;
  uint32_t reference_epoch;
  uint64_t current_time_us;
  uint32_t microseconds_since_pps;
  uint32_t ntp_fraction;
  int8_t ntp_precision;
  uint32_t root_delay;
  uint32_t root_dispersion;
  bool disciplined;
  bool holdover;
  uint32_t holdover_start_ms;
};

struct NtpTimingSnapshot {
  uint32_t current_epoch;
  uint32_t reference_epoch;
  uint32_t microseconds_since_pps;
  uint32_t ntp_fraction;
  int8_t ntp_precision;
  uint32_t root_delay;
  uint32_t root_dispersion;
  bool disciplined;
  bool holdover;
};

struct GPSData {
  GPSLockState state;
  bool locked;
  bool time_valid;
  bool date_valid;
  bool location_valid;
  uint8_t satellites;
  double latitude;
  double longitude;
  double altitude;
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  uint32_t last_update_ms;
  uint32_t last_time_update_ms;
  uint32_t last_location_update_ms;
  uint32_t last_locked_ms;
};

enum GnssConstellation {
  GNSS_CONSTELLATION_UNKNOWN = 0,
  GNSS_CONSTELLATION_GPS = 1,
  GNSS_CONSTELLATION_BDS = 4
};

struct GnssSatellite {
  GnssConstellation constellation;
  uint16_t prn;
  uint8_t elevation_deg;
  uint16_t azimuth_deg;
  uint8_t snr_dbhz;
  bool snr_valid;
  bool used_in_fix;
  uint32_t last_seen_ms;
};

struct GnssRuntime {
  GnssSatellite satellites[GNSS_MAX_SATELLITES];
  uint8_t satellite_count;
  uint8_t gps_visible;
  uint8_t bds_visible;
  uint8_t used_count;
  uint8_t fix_dimension;
  uint32_t last_gsv_ms;
  uint32_t last_gsa_ms;
  uint32_t sentences_parsed;
  uint32_t checksum_errors;
  bool antenna_ok;
  char antenna_status[24];
};

struct ButtonState {
  ButtonPhysicalState state;
  bool irq_pending;
  uint32_t transition_ms;
  uint32_t press_start_ms;
  bool long_press_reported;
};

struct ScreenManager {
  DisplayMode current;
  DisplayMode previous;
  uint32_t entered_ms;
  bool timed_screen;
  bool redraw_required;
};

struct DisplayCache {
  DisplayMode active_mode;
  DisplayMode drawn_mode;
  bool force_full_redraw;

  char local_time_text[40];
  char utc_time_text[40];
  char date_text[40];
  char pps_us_text[40];
  char pps_interval_text[40];
  char epoch_text[40];
  char fraction_text[40];
  char location_lat_text[40];
  char location_lon_text[40];
  char altitude_text[40];
  char countdown_text[40];
  char diagnostic_1[40];
  char diagnostic_2[40];
  char diagnostic_3[40];
  char diagnostic_4[40];
  char diagnostic_5[40];
  char diagnostic_6[40];
};

struct NetworkConfig {
  char ip[16];
  char gateway[16];
  char subnet[16];
  char dns[16];
  char hostname[32];
  bool dhcp_enabled;
  bool ethernet_enabled;
  bool ntp_enabled;
  bool web_config_from_sd;
};

struct WifiApRuntime {
  bool enabled;
  bool config_ok;
  bool started;
  uint8_t clients;
  uint32_t start_attempts;
  uint32_t errors;
  uint32_t started_ms;
  uint32_t remaining_seconds;
  bool window_expired;
  char ip[16];
  char last_error[48];
};

enum EthernetState {
  ETH_STATE_DISABLED = 0,
  ETH_STATE_STARTING = 1,
  ETH_STATE_LINK_DOWN = 2,
  ETH_STATE_LINK_UP = 3,
  ETH_STATE_GOT_IP = 4,
  ETH_STATE_ERROR = 5
};

struct EthernetRuntime {
  EthernetState state;
  bool started;
  bool link_up;
  bool got_ip;
  bool static_ip_applied;
  uint32_t last_event_ms;
  uint32_t link_up_count;
  uint32_t link_down_count;
  uint32_t got_ip_count;
  char ip[16];
  char gateway[16];
  char subnet[16];
  char mac[24];
};

enum NtpServiceState {
  NTP_STATE_DISABLED = 0,
  NTP_STATE_WAIT_ETH = 1,
  NTP_STATE_LISTENING = 2,
  NTP_STATE_ERROR = 3
};

struct NtpRuntime {
  NtpServiceState state;
  bool udp_started;
  bool replies_enabled;
  uint32_t packets_rx;
  uint32_t packets_tx;
  uint32_t packets_ignored;
  uint32_t packets_short;
  uint32_t packets_bad_mode;
  uint32_t packets_not_ready;
  uint32_t last_packet_ms;
  IPAddress last_remote_ip;
  uint16_t last_remote_port;
  uint16_t last_packet_size;
};

struct Udp123Debug {
  uint32_t total_rx;
  uint32_t ntp_rx;
  uint32_t short_rx;
  uint32_t ignored_rx;
  uint16_t last_size;
  uint16_t last_port;
  IPAddress last_ip;
};

struct Udp4123Debug {
  uint32_t total_rx;
  uint16_t last_size;
  uint16_t last_port;
  IPAddress last_ip;
};

enum SdCardState {
  SD_STATE_DISABLED = 0,
  SD_STATE_STARTING = 1,
  SD_STATE_MOUNTED = 2,
  SD_STATE_NO_CARD = 3,
  SD_STATE_TEST_FAILED = 4,
  SD_STATE_ERROR = 5
};

struct SdRuntime {
  SdCardState state;
  bool mounted;
  bool test_passed;
  uint64_t card_size_bytes;
  uint64_t total_bytes;
  uint64_t used_bytes;
  uint32_t mount_attempts;
  uint32_t test_writes;
  uint32_t test_reads;
  uint32_t errors;
  char card_type[16];
  char last_error[48];
};

enum ConfigState {
  CONFIG_STATE_DEFAULTS = 0,
  CONFIG_STATE_LOADED = 1,
  CONFIG_STATE_PARTIAL = 2,
  CONFIG_STATE_ERROR = 3
};

struct SystemConfig {
  char device_name[32];
  char node_id[32];
  char role[24];
  char site_name[40];
  int32_t local_utc_offset_minutes;
  uint32_t screen_timeout_seconds;
  uint32_t holdover_minutes;
  bool night_dim_enabled;
  uint16_t night_dim_start_minutes;
  uint16_t night_dim_stop_minutes;
  uint8_t night_dim_percent;
  uint16_t night_dim_wake_seconds;
  bool debug_serial;
  bool web_enabled;
};

struct DisplayDimmingRuntime {
  bool pwm_attached;
  bool schedule_active;
  bool local_time_valid;
  uint8_t brightness_percent;
  uint16_t local_minutes;
  uint32_t wake_until_ms;
  uint32_t last_update_ms;
};

struct ConfigRuntime {
  ConfigState state;
  bool network_loaded;
  bool system_loaded;
  bool defaults_created;
  uint32_t load_attempts;
  uint32_t errors;
  char last_error[64];
};

enum WebState {
  WEB_STATE_DISABLED = 0,
  WEB_STATE_WAIT_ETH = 1,
  WEB_STATE_RUNNING = 2,
  WEB_STATE_ERROR = 3
};

struct WebRuntime {
  WebState state;
  bool started;
  bool sd_index_available;
  uint32_t requests_total;
  uint32_t requests_status;
  uint32_t requests_api;
  uint32_t requests_static;
  uint32_t requests_not_found;
  uint32_t raw8080_requests;
  uint32_t handle_ticks;
  uint32_t start_attempts;
  uint32_t errors;
  char last_uri[64];
  char last_error[64];
};

enum SystemTemperatureState { SYSTEM_TEMP_UNAVAILABLE=0, SYSTEM_TEMP_NORMAL=1, SYSTEM_TEMP_WARNING=2, SYSTEM_TEMP_CRITICAL=3 };

struct SystemHealthRuntime {
  bool temperature_available;
  float temperature_c;
  float temperature_max_c;
  SystemTemperatureState temperature_state;
  uint32_t heap_free_bytes;
  uint32_t heap_min_bytes;
  uint32_t heap_largest_block_bytes;
  uint32_t internal_free_bytes;
  uint32_t psram_total_bytes;
  uint32_t psram_free_bytes;
  uint32_t timing_stack_free_bytes;
  uint32_t network_stack_free_bytes;
  uint32_t display_stack_free_bytes;
  uint32_t last_update_ms;
  uint32_t read_errors;
};

struct ResetHistoryRuntime {
  bool initialized;
  uint32_t total_boots;
  uint32_t power_on_resets;
  uint32_t software_resets;
  uint32_t watchdog_resets;
  uint32_t panic_resets;
  uint32_t brownout_resets;
  uint32_t external_resets;
  uint32_t unknown_resets;
  uint32_t unexpected_resets;
  uint32_t previous_uptime_seconds;
  uint32_t longest_uptime_seconds;
  uint32_t last_checkpoint_seconds;
  uint32_t last_checkpoint_ms;
};

struct LogRuntime {
  bool enabled;
  bool log_dir_ready;
  uint32_t ntp_entries;
  uint32_t ntp_errors;
  uint32_t event_entries;
  uint32_t event_errors;
  uint32_t temperature_entries;
  uint32_t temperature_errors;
  uint32_t temperature_last_sample_epoch;
  uint32_t temperature_last_sample_ms;
  uint32_t event_rotations;
  char temperature_log_path[48];
  char last_error[64];
};

struct MonitoredStateSnapshot {
  bool initialized;
  SystemTemperatureState temperature_state;
  bool ethernet_link;
  GPSLockState gps_state;
  PPSLockState pps_state;
  bool holdover;
};

struct OperationalStats {
  uint32_t gps_lock_start_ms;
  bool gps_lock_timer_active;
  uint32_t last_ntp_query_epoch;
};
