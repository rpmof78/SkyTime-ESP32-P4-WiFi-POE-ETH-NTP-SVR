#pragma once

#include <Arduino.h>

// Logging policy values reported by the status API.
#define EVENT_LOG_MAX_BYTES          (1024UL * 1024UL)
#define EVENT_LOG_TOTAL_FILES        5
#define TEMPERATURE_LOG_INTERVAL_MS  300000UL
#define TEMPERATURE_LOG_KEEP_MONTHS  3

void init_logging();
void flush_pending_event_logs();
void update_temperature_history_log();
void update_monitored_event_transitions();

void log_event_line(
  const char *event,
  const char *detail
);
void log_ntp_request(
  uint8_t version,
  uint8_t mode,
  bool tx_ok
);

void handle_web_logs_page();
void handle_api_log_events();
void handle_api_log_ntp();
void handle_api_log_files();
void handle_api_log_selected();
void handle_api_log_download();
