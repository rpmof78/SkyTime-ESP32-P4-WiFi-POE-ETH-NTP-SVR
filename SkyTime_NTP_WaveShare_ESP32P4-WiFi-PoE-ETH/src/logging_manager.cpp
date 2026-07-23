#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WebServer.h>
#include <time.h>

#include "skytime_types.h"
#include "logging_manager.h"

#define LOG_DIR_PATH                 "/logs"
#define NTP_LOG_FILE                 "/logs/ntp.log"
#define EVENT_LOG_FILE               "/logs/events.log"
#define NTP_LOG_MAX_BYTES            (1024UL * 1024UL)
#define LOG_VIEWER_MAX_LINES         100
#define LOG_VIEWER_MAX_LINE_LENGTH   320
#define LOG_VIEWER_MAX_BYTES         12000
#define PENDING_EVENT_LOG_MAX        6

struct PendingEventLogEntry {
  bool used;
  char event[24];
  char detail[96];
};

PendingEventLogEntry pending_event_logs[PENDING_EVENT_LOG_MAX] = {};
bool pending_event_logs_flushed = false;

extern WebServer web_server;
extern WebRuntime web_runtime;
extern SdRuntime sd_runtime;
extern LogRuntime log_runtime;
extern MonitoredStateSnapshot monitored_state;
extern SystemHealthRuntime system_health;
extern EthernetRuntime ethernet_runtime;
extern GPSData gps_data;
extern PPSData pps_data;
extern NtpRuntime ntp_runtime;
extern ResetHistoryRuntime reset_history;

extern void snapshot_ntp_timing(
  NtpTimingSnapshot *snapshot
);
extern const char *system_temperature_state_text(
  SystemTemperatureState state
);
extern const char *reset_reason_text(
  esp_reset_reason_t reason
);
extern bool append_json_uint(
  String &output,
  uint64_t value
);

// Private helper declarations.
void handle_api_log_file(
  const char *path,
  const char *log_name
);
bool valid_log_filename_cstr(
  const char *name
);
const char *log_kind_from_filename_cstr(
  const char *name
);
void format_log_utc(
  char *out,
  size_t out_size
);
bool utc_log_time_valid();
void queue_event_log_line(
  const char *event,
  const char *detail
);
void format_utc_timestamp(
  char *buffer,
  size_t buffer_size
);
bool ensure_log_directory();
void rotate_event_log_if_needed();
void rotate_log_if_needed(
  const char *path
);
bool append_log_line(
  const char *path,
  const char *line
);
bool build_temperature_log_path(
  char *path,
  size_t path_size
);
void enforce_temperature_log_retention(
  int current_year,
  int current_month
);

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

void handle_web_logs_page() {
  web_runtime.requests_total++;
  web_runtime.requests_static++;

  if (!sd_runtime.mounted) {
    web_server.send(503, "text/plain", "SD card not mounted");
    return;
  }

  const char *path = "/web/logs.html";

  if (!SD_MMC.exists(path)) {
    web_server.send(404, "text/plain", "logs.html not found");
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    web_server.send(500, "text/plain", "Unable to open logs.html");
    return;
  }

  web_server.streamFile(file, "text/html");
  file.close();
}

void handle_api_log_file(const char *path, const char *log_name) {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  if (!sd_runtime.mounted) {
    web_server.send(
      503,
      "application/json",
      "{\"ok\":false,\"error\":\"SD not mounted\"}"
    );
    return;
  }

  if (!SD_MMC.exists(path)) {
    char missing[192];

    snprintf(
      missing,
      sizeof(missing),
      "{\"ok\":true,\"name\":\"%s\","
      "\"path\":\"%s\",\"lines\":[],"
      "\"count\":0,"
      "\"message\":\"Log file not found\"}",
      log_name,
      path
    );

    web_server.send(
      200,
      "application/json",
      missing
    );
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    web_server.send(
      500,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Unable to open log\"}"
    );
    return;
  }

  size_t size = file.size();

  if (size > LOG_VIEWER_MAX_BYTES) {
    file.seek(size - LOG_VIEWER_MAX_BYTES);

    while (file.available()) {
      char c = file.read();

      if (c == '\n') {
        break;
      }
    }
  }

  static char line_storage[
    LOG_VIEWER_MAX_LINES
  ][LOG_VIEWER_MAX_LINE_LENGTH];

  uint16_t line_count = 0;
  uint16_t write_index = 0;
  char line_buffer[LOG_VIEWER_MAX_LINE_LENGTH];
  size_t line_length = 0;

  auto store_line = [&]() {
    while (line_length > 0 &&
           (line_buffer[line_length - 1] == '\r' ||
            line_buffer[line_length - 1] == '\n')) {
      line_length--;
    }

    if (line_length == 0) {
      return;
    }

    line_buffer[line_length] = '\0';

    snprintf(
      line_storage[write_index],
      LOG_VIEWER_MAX_LINE_LENGTH,
      "%s",
      line_buffer
    );

    write_index =
      (write_index + 1) % LOG_VIEWER_MAX_LINES;

    if (line_count < LOG_VIEWER_MAX_LINES) {
      line_count++;
    }
  };

  while (file.available()) {
    int value = file.read();

    if (value < 0) {
      break;
    }

    char c = (char)value;

    if (c == '\n') {
      store_line();
      line_length = 0;
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (line_length <
        LOG_VIEWER_MAX_LINE_LENGTH - 1) {
      line_buffer[line_length++] = c;
    }
  }

  if (line_length > 0) {
    store_line();
  }

  file.close();

  String json;
  json.reserve(LOG_VIEWER_MAX_BYTES + 256);

  json += "{\"ok\":true,\"name\":\"";
  append_json_escaped(json, log_name);
  json += "\",\"path\":\"";
  append_json_escaped(json, path);
  json += "\",\"count\":";
  append_json_uint(json, line_count);
  json += ",\"lines\":[";

  for (uint16_t output_index = 0;
       output_index < line_count;
       output_index++) {
    if (output_index > 0) {
      json += ",";
    }

    int32_t storage_index =
      (int32_t)write_index - 1 - output_index;

    while (storage_index < 0) {
      storage_index += LOG_VIEWER_MAX_LINES;
    }

    json += "\"";
    append_json_escaped(
      json,
      line_storage[storage_index]
    );
    json += "\"";
  }

  json += "]}";

  web_server.send(
    200,
    "application/json",
    json
  );
}

void handle_api_log_events() {
  handle_api_log_file(EVENT_LOG_FILE, "events");
}

void handle_api_log_ntp() {
  handle_api_log_file(NTP_LOG_FILE, "ntp");
}

bool valid_log_filename_cstr(const char *name) {
  if (!name) return false;

  size_t length = strlen(name);

  if (length == 0 || length > 48) return false;
  if (strchr(name, '/') ||
      strchr(name, '\\') ||
      strstr(name, "..")) {
    return false;
  }

  if (strcmp(name, "events.log") == 0 ||
      strcmp(name, "events.1.log") == 0 ||
      strcmp(name, "events.2.log") == 0 ||
      strcmp(name, "events.3.log") == 0 ||
      strcmp(name, "events.4.log") == 0 ||
      strcmp(name, "ntp.log") == 0 ||
      strcmp(name, "ntp.log.old") == 0) {
    return true;
  }

  int year = 0;
  int month = 0;
  char trailing = '\0';

  if (sscanf(
        name,
        "temperature-%4d-%2d.csv%c",
        &year,
        &month,
        &trailing
      ) == 2) {
    return year >= 2020 &&
           year <= 2099 &&
           month >= 1 &&
           month <= 12;
  }

  return false;
}

const char *log_kind_from_filename_cstr(
  const char *name
) {
  if (!name) return "unknown";
  if (strncmp(name, "events", 6) == 0) return "events";
  if (strncmp(name, "ntp", 3) == 0) return "ntp";
  if (strncmp(name, "temperature-", 12) == 0) {
    return "temperature";
  }

  return "unknown";
}

void handle_api_log_files() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  if (!sd_runtime.mounted) {
    web_server.send(503, "application/json", "{\"ok\":false,\"error\":\"SD not mounted\"}");
    return;
  }

  File dir = SD_MMC.open(LOG_DIR_PATH);

  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"Unable to open log directory\"}");
    return;
  }

  String json;
  json.reserve(1536);
  json = "{\"ok\":true,\"files\":[";
  bool first = true;

  File entry = dir.openNextFile();

  while (entry) {
    if (!entry.isDirectory()) {
      const char *full_name = entry.name();
      const char *base_name = strrchr(full_name, '/');

      base_name =
        base_name ?
        base_name + 1 :
        full_name;

      if (valid_log_filename_cstr(base_name)) {
        if (!first) json += ",";
        first = false;

        json += "{\"name\":\"";
        json += base_name;
        json += "\",\"path\":\"/logs/";
        json += base_name;
        json += "\",\"kind\":\"";
        json += log_kind_from_filename_cstr(base_name);
        json += "\",\"size\":";
        append_json_uint(json, (uint64_t)entry.size());
        json += "}";
      }
    }

    entry.close();
    entry = dir.openNextFile();
  }

  dir.close();
  json += "]}";
  web_server.send(200, "application/json", json);
}

void handle_api_log_selected() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  if (!sd_runtime.mounted) {
    web_server.send(503, "text/plain", "SD not mounted");
    return;
  }

  if (!web_server.hasArg("file")) {
    web_server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  const String &name = web_server.arg("file");

  if (!valid_log_filename_cstr(name.c_str())) {
    web_server.send(400, "text/plain", "Invalid log file");
    return;
  }

  char path[96];

  int path_written = snprintf(
    path,
    sizeof(path),
    "%s/%s",
    LOG_DIR_PATH,
    name.c_str()
  );

  if (path_written < 0 ||
      (size_t)path_written >= sizeof(path)) {
    web_server.send(400, "text/plain", "Log path too long");
    return;
  }

  if (!SD_MMC.exists(path)) {
    web_server.send(404, "text/plain", "Log file not found");
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    web_server.send(500, "text/plain", "Unable to open log file");
    return;
  }

  web_server.sendHeader("Cache-Control", "no-store");
  web_server.sendHeader("X-Content-Type-Options", "nosniff");

  const char *content_type = name.endsWith(".csv")
    ? "text/csv"
    : "text/plain";

  web_server.streamFile(file, content_type);
  file.close();
}

void handle_api_log_download() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  if (!sd_runtime.mounted) {
    web_server.send(503, "text/plain", "SD not mounted");
    return;
  }

  if (!web_server.hasArg("file")) {
    web_server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  const String &name = web_server.arg("file");

  if (!valid_log_filename_cstr(name.c_str())) {
    web_server.send(400, "text/plain", "Invalid log file");
    return;
  }

  char path[96];

  int path_written = snprintf(
    path,
    sizeof(path),
    "%s/%s",
    LOG_DIR_PATH,
    name.c_str()
  );

  if (path_written < 0 ||
      (size_t)path_written >= sizeof(path)) {
    web_server.send(400, "text/plain", "Log path too long");
    return;
  }

  if (!SD_MMC.exists(path)) {
    web_server.send(404, "text/plain", "Log file not found");
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    web_server.send(500, "text/plain", "Unable to open log file");
    return;
  }

  char disposition[96];

  int disposition_written = snprintf(
    disposition,
    sizeof(disposition),
    "attachment; filename=\"%s\"",
    name.c_str()
  );

  if (disposition_written < 0 ||
      (size_t)disposition_written >=
        sizeof(disposition)) {
    file.close();
    web_server.send(
      400,
      "text/plain",
      "Download filename too long"
    );
    return;
  }

  web_server.sendHeader(
    "Content-Disposition",
    disposition
  );

  const char *content_type = name.endsWith(".csv")
    ? "text/csv"
    : "text/plain";

  web_server.streamFile(file, content_type);
  file.close();
}

void format_log_utc(char *out, size_t out_size) {
  if (!out || out_size == 0) {
    return;
  }

  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  if (timing.current_epoch == 0) {
    snprintf(out, out_size, "0000-00-00 00:00:00 UTC");
    return;
  }

  uint32_t unix_epoch =
    timing.current_epoch - 2208988800UL;
  time_t raw = (time_t)unix_epoch;
  struct tm tm_utc;

  gmtime_r(&raw, &tm_utc);

  snprintf(
    out,
    out_size,
    "%04d-%02d-%02d %02d:%02d:%02d UTC",
    tm_utc.tm_year + 1900,
    tm_utc.tm_mon + 1,
    tm_utc.tm_mday,
    tm_utc.tm_hour,
    tm_utc.tm_min,
    tm_utc.tm_sec
  );
}

bool utc_log_time_valid() {
  return (
    gps_data.year >= 2024 &&
    gps_data.month >= 1 &&
    gps_data.month <= 12 &&
    gps_data.day >= 1 &&
    gps_data.day <= 31
  );
}

void queue_event_log_line(const char *event, const char *detail) {
  for (uint8_t i = 0; i < PENDING_EVENT_LOG_MAX; i++) {
    if (!pending_event_logs[i].used) {
      pending_event_logs[i].used = true;
      snprintf(pending_event_logs[i].event, sizeof(pending_event_logs[i].event), "%s", event ? event : "");
      snprintf(pending_event_logs[i].detail, sizeof(pending_event_logs[i].detail), "%s", detail ? detail : "");
      return;
    }
  }
}

void format_utc_timestamp(char *buffer, size_t buffer_size) {
  if (gps_data.year > 0 &&
      gps_data.month > 0 &&
      gps_data.day > 0) {
    snprintf(
      buffer,
      buffer_size,
      "%04u-%02u-%02u %02u:%02u:%02u UTC",
      gps_data.year,
      gps_data.month,
      gps_data.day,
      gps_data.hour,
      gps_data.minute,
      gps_data.second
    );
  } else {
    snprintf(
      buffer,
      buffer_size,
      "UTC_PENDING uptime:%lu",
      (unsigned long)(millis() / 1000UL)
    );
  }
}

bool ensure_log_directory() {
  if (!sd_runtime.mounted) {
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "SD not mounted");
    return false;
  }

  if (!SD_MMC.exists(LOG_DIR_PATH)) {
    if (!SD_MMC.mkdir(LOG_DIR_PATH)) {
      snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "mkdir failed");
      return false;
    }
  }

  log_runtime.log_dir_ready = true;
  snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "OK");
  return true;
}

void rotate_event_log_if_needed() {
  if (!sd_runtime.mounted || !SD_MMC.exists(EVENT_LOG_FILE)) return;

  File file = SD_MMC.open(EVENT_LOG_FILE, FILE_READ);
  if (!file) return;
  size_t size = file.size();
  file.close();

  if (size < EVENT_LOG_MAX_BYTES) return;

  char source[48];
  char destination[48];

  snprintf(destination, sizeof(destination), "/logs/events.%u.log",
           EVENT_LOG_TOTAL_FILES - 1);
  if (SD_MMC.exists(destination)) SD_MMC.remove(destination);

  for (int index = EVENT_LOG_TOTAL_FILES - 2; index >= 1; --index) {
    snprintf(source, sizeof(source), "/logs/events.%d.log", index);
    snprintf(destination, sizeof(destination), "/logs/events.%d.log", index + 1);
    if (SD_MMC.exists(source)) SD_MMC.rename(source, destination);
  }

  snprintf(destination, sizeof(destination), "/logs/events.1.log");
  if (SD_MMC.exists(destination)) SD_MMC.remove(destination);

  if (SD_MMC.rename(EVENT_LOG_FILE, destination)) {
    log_runtime.event_rotations++;
  }
}

void rotate_log_if_needed(const char *path) {
  if (!path) return;

  if (strcmp(path, EVENT_LOG_FILE) == 0) {
    rotate_event_log_if_needed();
    return;
  }

  if (!sd_runtime.mounted || !SD_MMC.exists(path)) return;

  File file = SD_MMC.open(path, FILE_READ);
  if (!file) return;
  size_t size = file.size();
  file.close();

  if (size < NTP_LOG_MAX_BYTES) return;

  char old_path[64];
  snprintf(old_path, sizeof(old_path), "%s.old", path);
  if (SD_MMC.exists(old_path)) SD_MMC.remove(old_path);
  SD_MMC.rename(path, old_path);
}

bool append_log_line(const char *path, const char *line) {
  if (!log_runtime.enabled) {
    return false;
  }

  if (!ensure_log_directory()) {
    return false;
  }

  rotate_log_if_needed(path);

  File file = SD_MMC.open(path, FILE_APPEND);

  if (!file) {
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "append open failed");
    return false;
  }

  file.println(line);
  file.close();

  snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "OK");
  return true;
}

void log_event_line(const char *event, const char *detail) {
  if (!utc_log_time_valid()) {
    queue_event_log_line(event, detail);
    return;
  }
  char timestamp[32];
  char line[192];

  format_utc_timestamp(timestamp, sizeof(timestamp));

  snprintf(
    line,
    sizeof(line),
    "%s | EVENT | %s | %s",
    timestamp,
    event,
    detail ? detail : ""
  );

  if (append_log_line(EVENT_LOG_FILE, line)) {
    log_runtime.event_entries++;
  } else {
    log_runtime.event_errors++;
  }
}

void flush_pending_event_logs() {
  if (pending_event_logs_flushed) {
    return;
  }

  if (!utc_log_time_valid()) {
    return;
  }

  for (uint8_t i = 0; i < PENDING_EVENT_LOG_MAX; i++) {
    if (pending_event_logs[i].used) {
      log_event_line(pending_event_logs[i].event, pending_event_logs[i].detail);
      pending_event_logs[i].used = false;
    }
  }

  pending_event_logs_flushed = true;
}

void log_ntp_request(uint8_t version, uint8_t mode, bool tx_ok) {
  if (!log_runtime.enabled || !sd_runtime.mounted) {
    return;
  }

  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  char timestamp[32];
  char line[256];

  format_utc_timestamp(timestamp, sizeof(timestamp));

  snprintf(
    line,
    sizeof(line),
    "%s | NTP | client=%u.%u.%u.%u:%u | vn=%u | mode=%u | tx=%s | stratum=%u | disciplined=%s | holdover=%s",
    timestamp,
    (unsigned int)ntp_runtime.last_remote_ip[0],
    (unsigned int)ntp_runtime.last_remote_ip[1],
    (unsigned int)ntp_runtime.last_remote_ip[2],
    (unsigned int)ntp_runtime.last_remote_ip[3],
    ntp_runtime.last_remote_port,
    version,
    mode,
    tx_ok ? "OK" : "FAIL",
    timing.holdover ? 2 : 1,
    timing.disciplined ? "YES" : "NO",
    timing.holdover ? "YES" : "NO"
  );

  if (append_log_line(NTP_LOG_FILE, line)) {
    log_runtime.ntp_entries++;
  } else {
    log_runtime.ntp_errors++;
  }
}

bool build_temperature_log_path(char *path, size_t path_size) {
  if (!path || path_size == 0 || !utc_log_time_valid()) return false;

  snprintf(path, path_size, "/logs/temperature-%04u-%02u.csv",
           gps_data.year, gps_data.month);
  return true;
}

void enforce_temperature_log_retention(int current_year, int current_month) {
  File directory = SD_MMC.open(LOG_DIR_PATH);
  if (!directory || !directory.isDirectory()) {
    if (directory) directory.close();
    return;
  }

  File entry = directory.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      const char *full_name = entry.name();
      const char *base_name = strrchr(full_name, '/');

      base_name =
        base_name ?
        base_name + 1 :
        full_name;

      int year = 0;
      int month = 0;

      if (sscanf(
            base_name,
            "temperature-%4d-%2d.csv",
            &year,
            &month
          ) == 2) {
        int current_index =
          current_year * 12 + current_month - 1;
        int file_index =
          year * 12 + month - 1;
        int age_months =
          current_index - file_index;

        if (age_months < 0 ||
            age_months >= TEMPERATURE_LOG_KEEP_MONTHS) {
          char remove_path[96];

          int path_written = snprintf(
            remove_path,
            sizeof(remove_path),
            "%s/%s",
            LOG_DIR_PATH,
            base_name
          );

          entry.close();

          if (path_written >= 0 &&
              (size_t)path_written < sizeof(remove_path)) {
            SD_MMC.remove(remove_path);
          }

          entry = directory.openNextFile();
          continue;
        }
      }
    }

    entry.close();
    entry = directory.openNextFile();
  }

  directory.close();
}

void update_temperature_history_log() {
  if (!log_runtime.enabled || !sd_runtime.mounted ||
      !system_health.temperature_available || !utc_log_time_valid()) return;

  uint32_t now_ms = millis();
  if (log_runtime.temperature_last_sample_ms != 0 &&
      now_ms - log_runtime.temperature_last_sample_ms <
      TEMPERATURE_LOG_INTERVAL_MS) return;

  char path[48];
  if (!build_temperature_log_path(path, sizeof(path))) return;

  bool new_file = !SD_MMC.exists(path);
  if (new_file) {
    enforce_temperature_log_retention(gps_data.year, gps_data.month);
  }

  File file = SD_MMC.open(path, FILE_APPEND);
  if (!file) {
    log_runtime.temperature_errors++;
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error),
             "temperature append failed");
    return;
  }

  if (new_file || file.size() == 0) {
    file.println("utc_date,utc_time,uptime_s,p4_temp_c,temp_state,"
                 "heap_free_kb,heap_min_kb,fragmentation_pct");
  }

  float fragmentation = 0.0f;
  if (system_health.internal_free_bytes > 0) {
    fragmentation = 100.0f *
      (1.0f - (float)system_health.heap_largest_block_bytes /
              (float)system_health.internal_free_bytes);
    if (fragmentation < 0.0f) fragmentation = 0.0f;
    if (fragmentation > 100.0f) fragmentation = 100.0f;
  }

  char line[192];
  snprintf(line, sizeof(line),
    "%04u-%02u-%02u,%02u:%02u:%02u,%lu,%.1f,%s,%.1f,%.1f,%.1f",
    gps_data.year, gps_data.month, gps_data.day,
    gps_data.hour, gps_data.minute, gps_data.second,
    (unsigned long)(now_ms / 1000UL),
    system_health.temperature_c,
    system_temperature_state_text(system_health.temperature_state),
    system_health.heap_free_bytes / 1024.0f,
    system_health.heap_min_bytes / 1024.0f,
    fragmentation);

  file.println(line);
  file.close();

  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  log_runtime.temperature_entries++;
  log_runtime.temperature_last_sample_ms = now_ms;
  log_runtime.temperature_last_sample_epoch = timing.current_epoch;
  snprintf(log_runtime.temperature_log_path,
           sizeof(log_runtime.temperature_log_path), "%s", path);
  snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "OK");
}

void update_monitored_event_transitions() {
  if (!log_runtime.enabled) return;

  NtpTimingSnapshot timing = {};
  snapshot_ntp_timing(&timing);

  if (!monitored_state.initialized) {
    monitored_state.initialized = true;
    monitored_state.temperature_state = system_health.temperature_state;
    monitored_state.ethernet_link = ethernet_runtime.link_up;
    monitored_state.gps_state = gps_data.state;
    monitored_state.pps_state = pps_data.state;
    monitored_state.holdover = timing.holdover;

    char detail[128];
    snprintf(detail, sizeof(detail),
      "state=%s current=%.1fC max=%.1fC",
      system_temperature_state_text(system_health.temperature_state),
      system_health.temperature_c,
      system_health.temperature_max_c);
    log_event_line("TEMP_SENSOR", detail);
    return;
  }

  if (system_health.temperature_state != monitored_state.temperature_state) {
    char detail[128];
    snprintf(detail, sizeof(detail),
      "from=%s to=%s current=%.1fC max=%.1fC",
      system_temperature_state_text(monitored_state.temperature_state),
      system_temperature_state_text(system_health.temperature_state),
      system_health.temperature_c,
      system_health.temperature_max_c);
    log_event_line("TEMP_STATE", detail);
    monitored_state.temperature_state = system_health.temperature_state;
  }

  if (ethernet_runtime.link_up != monitored_state.ethernet_link) {
    log_event_line(ethernet_runtime.link_up ? "ETH_LINK_UP" : "ETH_LINK_DOWN",
                   ethernet_runtime.link_up ? ethernet_runtime.ip :
                   "link unavailable");
    monitored_state.ethernet_link = ethernet_runtime.link_up;
  }

  if (gps_data.state != monitored_state.gps_state) {
    char detail[96];
    snprintf(detail, sizeof(detail), "from=%d to=%d satellites=%u",
             (int)monitored_state.gps_state, (int)gps_data.state,
             gps_data.satellites);
    log_event_line("GPS_STATE", detail);
    monitored_state.gps_state = gps_data.state;
  }

  if (pps_data.state != monitored_state.pps_state) {
    char detail[96];
    snprintf(detail, sizeof(detail), "from=%d to=%d count=%lu",
             (int)monitored_state.pps_state, (int)pps_data.state,
             (unsigned long)pps_data.pps_count);
    log_event_line("PPS_STATE", detail);
    monitored_state.pps_state = pps_data.state;
  }

  if (timing.holdover != monitored_state.holdover) {
    log_event_line(timing.holdover ?
                   "HOLDOVER_ENTER" : "HOLDOVER_EXIT",
                   timing.holdover ?
                   "GPS/PPS holdover active" : "discipline restored");
    monitored_state.holdover = timing.holdover;
  }
}

void init_logging() {
  Serial.println("[LOG] Initializing operational logging");

  if (!sd_runtime.mounted) {
    log_runtime.enabled = false;
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "SD not mounted");
    Serial.println("[LOG] Disabled - SD not mounted");
    return;
  }

  log_runtime.enabled = true;

  if (ensure_log_directory()) {
    Serial.println("[LOG] Log directory ready");

    char boot_detail[128];
    snprintf(boot_detail, sizeof(boot_detail),
      "reset=%s firmware=1.0 boot=%lu previous_uptime_s=%lu unexpected=%lu startup_uptime_s=%lu",
      reset_reason_text(esp_reset_reason()),
      (unsigned long)reset_history.total_boots,
      (unsigned long)reset_history.previous_uptime_seconds,
      (unsigned long)reset_history.unexpected_resets,
      (unsigned long)(millis() / 1000UL));
    log_event_line("BOOT", boot_detail);
  } else {
    log_runtime.enabled = false;
    Serial.print("[LOG] Disabled - ");
    Serial.println(log_runtime.last_error);
  }
}
