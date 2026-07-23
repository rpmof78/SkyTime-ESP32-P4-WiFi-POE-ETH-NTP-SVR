#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WebServer.h>
#include <IPAddress.h>
#include <limits.h>

#include "skytime_types.h"
#include "display_manager.h"
#include "config_manager.h"

#define CONFIG_NETWORK_FILE      "/config/network.json"
#define CONFIG_SYSTEM_FILE       "/config/system.json"

extern WebServer web_server;
extern WebRuntime web_runtime;
extern SdRuntime sd_runtime;
extern ConfigRuntime config_runtime;
extern SystemConfig system_config;
extern NetworkConfig network_config;
extern DisplayDimmingRuntime display_dimming;

extern IPAddress eth_static_ip;
extern IPAddress eth_gateway;
extern IPAddress eth_subnet;
extern IPAddress eth_dns;
extern char ip_address[16];

extern portMUX_TYPE state_mux;

extern bool json_escape_string(
  const char *input,
  char *output,
  size_t output_size
);
extern void log_event_line(
  const char *event,
  const char *detail
);

// Web and validation helpers still owned by main.cpp.
extern bool send_sd_file(
  const char *path,
  const char *content_type
);
extern bool valid_hostname_text(
  const char *text,
  size_t max_len
);

// Private configuration helper declarations.
bool read_text_file(
  const char *path,
  char *buffer,
  size_t buffer_size
);
bool write_text_file(
  const char *path,
  const char *content
);
bool json_get_string(
  const char *json,
  const char *key,
  char *out,
  size_t out_size
);
bool json_get_bool(
  const char *json,
  const char *key,
  bool *out
);
bool json_get_uint32(
  const char *json,
  const char *key,
  uint32_t *out
);
bool json_get_int32(
  const char *json,
  const char *key,
  int32_t *out
);
bool ip_from_string(
  const char *text,
  IPAddress *ip
);

bool valid_identity_text(const char *text, size_t max_len) {
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
      c == ' ' ||
      c == '-' ||
      c == '_' ||
      c == '.';

    if (!ok) {
      return false;
    }
  }

  return true;
}

bool save_system_config_file() {
  if (!sd_runtime.mounted) {
    return false;
  }

  char content[1024];

  snprintf(
    content,
    sizeof(content),
    "{\n"
    "  \"device_name\": \"%s\",\n"
    "  \"node_id\": \"%s\",\n"
    "  \"role\": \"%s\",\n"
    "  \"site_name\": \"%s\",\n"
    "  \"local_utc_offset_minutes\": %ld,\n"
    "  \"screen_timeout\": %lu,\n"
    "  \"holdover_minutes\": %lu,\n"
    "  \"night_dim_enabled\": %s,\n"
    "  \"night_dim_start_minutes\": %u,\n"
    "  \"night_dim_stop_minutes\": %u,\n"
    "  \"night_dim_percent\": %u,\n"
    "  \"night_dim_wake_seconds\": %u,\n"
    "  \"debug_serial\": %s,\n"
    "  \"web_enabled\": %s\n"
    "}\n",
    system_config.device_name,
    system_config.node_id,
    system_config.role,
    system_config.site_name,
    (long)system_config.local_utc_offset_minutes,
    (unsigned long)system_config.screen_timeout_seconds,
    (unsigned long)system_config.holdover_minutes,
    system_config.night_dim_enabled ? "true" : "false",
    (unsigned int)system_config.night_dim_start_minutes,
    (unsigned int)system_config.night_dim_stop_minutes,
    (unsigned int)system_config.night_dim_percent,
    (unsigned int)system_config.night_dim_wake_seconds,
    system_config.debug_serial ? "true" : "false",
    system_config.web_enabled ? "true" : "false"
  );

  if (SD_MMC.exists(CONFIG_SYSTEM_FILE)) {
    SD_MMC.remove(CONFIG_SYSTEM_FILE);
  }

  return write_text_file(CONFIG_SYSTEM_FILE, content);
}

void handle_web_config_page() {
  web_runtime.requests_total++;
  web_runtime.requests_static++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  if (!send_sd_file("/web/config.html", "text/html")) {
    web_server.send(404, "text/plain", "config.html not found");
  }
}

void handle_api_config_identity_get() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  char node[80];
  char role[80];
  char site[96];

  json_escape_string(system_config.node_id, node, sizeof(node));
  json_escape_string(system_config.role, role, sizeof(role));
  json_escape_string(system_config.site_name, site, sizeof(site));

  char json[384];

  snprintf(
    json,
    sizeof(json),
    "{\"ok\":true,\"node_id\":\"%s\",\"role\":\"%s\",\"site_name\":\"%s\",\"local_utc_offset_minutes\":%ld}",
    node,
    role,
    site,
    (long)system_config.local_utc_offset_minutes
  );

  web_server.send(200, "application/json", json);
}

void handle_api_config_identity_post() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  const String &body =
    web_server.arg("plain");

  if (body.length() == 0) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Missing request body\"}"
    );
    return;
  }

  char json[512];

  if (body.length() >= sizeof(json)) {
    web_server.send(
      413,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Request body too large\"}"
    );
    return;
  }

  memcpy(
    json,
    body.c_str(),
    body.length() + 1
  );

  char node_id[32];
  char role[24];
  char site_name[40];
  int32_t local_utc_offset_minutes = system_config.local_utc_offset_minutes;

  if (!json_get_string(json, "node_id", node_id, sizeof(node_id)) ||
      !json_get_string(json, "role", role, sizeof(role)) ||
      !json_get_string(json, "site_name", site_name, sizeof(site_name))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing identity fields\"}");
    return;
  }

  if (!valid_identity_text(node_id, sizeof(system_config.node_id)) ||
      !valid_identity_text(role, sizeof(system_config.role)) ||
      !valid_identity_text(site_name, sizeof(system_config.site_name))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Invalid characters or field length\"}");
    return;
  }

  if (json_get_int32(json, "local_utc_offset_minutes", &local_utc_offset_minutes)) {
    if (local_utc_offset_minutes < -720 || local_utc_offset_minutes > 840) {
      web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"UTC offset must be between -720 and +840 minutes\"}");
      return;
    }
  }

  snprintf(system_config.node_id, sizeof(system_config.node_id), "%s", node_id);
  snprintf(system_config.role, sizeof(system_config.role), "%s", role);
  snprintf(system_config.site_name, sizeof(system_config.site_name), "%s", site_name);
  system_config.local_utc_offset_minutes = local_utc_offset_minutes;

  if (!save_system_config_file()) {
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"Unable to save system.json\"}");
    return;
  }

  char detail[128];
  snprintf(
    detail,
    sizeof(detail),
    "System=%s Role=%s Site=%s",
    system_config.node_id,
    system_config.role,
    system_config.site_name
  );

  log_event_line("CONFIG", detail);

  web_server.send(200, "application/json", "{\"ok\":true,\"message\":\"Identity configuration saved\"}");
}

bool valid_ip_config_text(const char *text) {
  IPAddress ip;
  return ip_from_string(text, &ip);
}

bool save_network_config_file() {
  if (!sd_runtime.mounted) {
    return false;
  }

  char content[640];

  snprintf(
    content,
    sizeof(content),
    "{\n"
    "  \"hostname\": \"%s\",\n"
    "  \"dhcp\": %s,\n"
    "  \"ip\": \"%s\",\n"
    "  \"subnet\": \"%s\",\n"
    "  \"gateway\": \"%s\",\n"
    "  \"dns\": \"%s\",\n"
    "  \"ntp_enabled\": %s\n"
    "}\n",
    network_config.hostname,
    network_config.dhcp_enabled ? "true" : "false",
    network_config.ip,
    network_config.subnet,
    network_config.gateway,
    network_config.dns,
    network_config.ntp_enabled ? "true" : "false"
  );

  if (SD_MMC.exists(CONFIG_NETWORK_FILE)) {
    SD_MMC.remove(CONFIG_NETWORK_FILE);
  }

  return write_text_file(CONFIG_NETWORK_FILE, content);
}

void handle_api_config_display_get() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    web_server.uri().c_str()
  );

  bool enabled = false;
  uint16_t start_minutes = 0;
  uint16_t stop_minutes = 0;
  uint8_t dim_percent = 0;
  uint16_t wake_seconds = 0;

  portENTER_CRITICAL(&state_mux);
  enabled = system_config.night_dim_enabled;
  start_minutes =
    system_config.night_dim_start_minutes;
  stop_minutes =
    system_config.night_dim_stop_minutes;
  dim_percent =
    system_config.night_dim_percent;
  wake_seconds =
    system_config.night_dim_wake_seconds;
  portEXIT_CRITICAL(&state_mux);

  char json[384];

  snprintf(
    json,
    sizeof(json),
    "{\"ok\":true,"
    "\"enabled\":%s,"
    "\"start_minutes\":%u,"
    "\"stop_minutes\":%u,"
    "\"dim_percent\":%u,"
    "\"wake_seconds\":%u,"
    "\"active\":%s,"
    "\"wake_active\":%s,"
    "\"wake_remaining_seconds\":%lu,"
    "\"brightness_percent\":%u,"
    "\"local_time_valid\":%s,"
    "\"local_minutes\":%u}",
    enabled ? "true" : "false",
    (unsigned int)start_minutes,
    (unsigned int)stop_minutes,
    (unsigned int)dim_percent,
    (unsigned int)wake_seconds,
    display_dimming.schedule_active ?
      "true" : "false",
    display_wake_override_active(millis()) ?
      "true" : "false",
    (unsigned long)(
      display_wake_override_active(millis()) ?
      ((display_dimming.wake_until_ms - millis() + 999UL) / 1000UL) :
      0UL
    ),
    (unsigned int)
      display_dimming.brightness_percent,
    display_dimming.local_time_valid ?
      "true" : "false",
    (unsigned int)display_dimming.local_minutes
  );

  web_server.send(
    200,
    "application/json",
    json
  );
}

void handle_api_config_display_post() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    web_server.uri().c_str()
  );

  const String &body =
    web_server.arg("plain");

  if (body.length() == 0) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Missing request body\"}"
    );
    return;
  }

  char json[384];

  if (body.length() >= sizeof(json)) {
    web_server.send(
      413,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Request body too large\"}"
    );
    return;
  }

  memcpy(
    json,
    body.c_str(),
    body.length() + 1
  );

  bool enabled = false;
  uint32_t start_minutes = 0;
  uint32_t stop_minutes = 0;
  uint32_t dim_percent = 0;
  uint32_t wake_seconds = 0;

  if (!json_get_bool(
        json,
        "enabled",
        &enabled
      ) ||
      !json_get_uint32(
        json,
        "start_minutes",
        &start_minutes
      ) ||
      !json_get_uint32(
        json,
        "stop_minutes",
        &stop_minutes
      ) ||
      !json_get_uint32(
        json,
        "dim_percent",
        &dim_percent
      ) ||
      !json_get_uint32(
        json,
        "wake_seconds",
        &wake_seconds
      )) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Missing display dimming fields\"}"
    );
    return;
  }

  if (start_minutes > 1439 ||
      stop_minutes > 1439) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Start and stop times must be valid\"}"
    );
    return;
  }

  if (start_minutes == stop_minutes) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Start and stop times must differ\"}"
    );
    return;
  }

  if (dim_percent > 100) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Dim percentage must be 0 through 100\"}"
    );
    return;
  }

  if (wake_seconds < 1 ||
      wake_seconds > 3600) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Wake duration must be 1 through 3600 seconds\"}"
    );
    return;
  }

  portENTER_CRITICAL(&state_mux);
  system_config.night_dim_enabled = enabled;
  system_config.night_dim_start_minutes =
    (uint16_t)start_minutes;
  system_config.night_dim_stop_minutes =
    (uint16_t)stop_minutes;
  system_config.night_dim_percent =
    (uint8_t)dim_percent;
  system_config.night_dim_wake_seconds =
    (uint16_t)wake_seconds;
  portEXIT_CRITICAL(&state_mux);

  if (!save_system_config_file()) {
    web_server.send(
      500,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Unable to save system.json\"}"
    );
    return;
  }

  update_display_brightness(true);

  char detail[128];

  snprintf(
    detail,
    sizeof(detail),
    "enabled=%s start=%02lu:%02lu "
    "stop=%02lu:%02lu dim=%lu%% wake=%lus",
    enabled ? "true" : "false",
    (unsigned long)(start_minutes / 60),
    (unsigned long)(start_minutes % 60),
    (unsigned long)(stop_minutes / 60),
    (unsigned long)(stop_minutes % 60),
    (unsigned long)dim_percent,
    (unsigned long)wake_seconds
  );

  log_event_line(
    "DISPLAY_DIM_CONFIG",
    detail
  );

  web_server.send(
    200,
    "application/json",
    "{\"ok\":true,"
    "\"message\":\"Display dimming saved\"}"
  );
}

void handle_api_config_network_get() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  char hostname[80];
  char ip[32];
  char subnet[32];
  char gateway[32];
  char dns[32];

  json_escape_string(network_config.hostname, hostname, sizeof(hostname));
  json_escape_string(network_config.ip, ip, sizeof(ip));
  json_escape_string(network_config.subnet, subnet, sizeof(subnet));
  json_escape_string(network_config.gateway, gateway, sizeof(gateway));
  json_escape_string(network_config.dns, dns, sizeof(dns));

  char json[512];

  snprintf(
    json,
    sizeof(json),
    "{\"ok\":true,\"hostname\":\"%s\",\"dhcp\":%s,\"ip\":\"%s\",\"subnet\":\"%s\",\"gateway\":\"%s\",\"dns\":\"%s\",\"ntp_enabled\":%s,\"reboot_required\":true}",
    hostname,
    network_config.dhcp_enabled ? "true" : "false",
    ip,
    subnet,
    gateway,
    dns,
    network_config.ntp_enabled ? "true" : "false"
  );

  web_server.send(200, "application/json", json);
}

void handle_api_config_network_post() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  const String &body =
    web_server.arg("plain");

  if (body.length() == 0) {
    web_server.send(
      400,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Missing request body\"}"
    );
    return;
  }

  char json[640];

  if (body.length() >= sizeof(json)) {
    web_server.send(
      413,
      "application/json",
      "{\"ok\":false,"
      "\"error\":\"Request body too large\"}"
    );
    return;
  }

  memcpy(
    json,
    body.c_str(),
    body.length() + 1
  );

  char hostname[32];
  char ip[16];
  char subnet[16];
  char gateway[16];
  char dns[16];
  bool dhcp = false;
  bool ntp_enabled = true;

  if (!json_get_string(json, "hostname", hostname, sizeof(hostname)) ||
      !json_get_bool(json, "dhcp", &dhcp) ||
      !json_get_string(json, "ip", ip, sizeof(ip)) ||
      !json_get_string(json, "subnet", subnet, sizeof(subnet)) ||
      !json_get_string(json, "gateway", gateway, sizeof(gateway)) ||
      !json_get_string(json, "dns", dns, sizeof(dns))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing network fields\"}");
    return;
  }

  json_get_bool(json, "ntp_enabled", &ntp_enabled);

  if (!valid_hostname_text(hostname, sizeof(network_config.hostname)) ||
      !valid_ip_config_text(ip) ||
      !valid_ip_config_text(subnet) ||
      !valid_ip_config_text(gateway) ||
      !valid_ip_config_text(dns)) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Invalid hostname or IP address\"}");
    return;
  }

  snprintf(network_config.hostname, sizeof(network_config.hostname), "%s", hostname);
  network_config.dhcp_enabled = dhcp;
  snprintf(network_config.ip, sizeof(network_config.ip), "%s", ip);
  snprintf(network_config.subnet, sizeof(network_config.subnet), "%s", subnet);
  snprintf(network_config.gateway, sizeof(network_config.gateway), "%s", gateway);
  snprintf(network_config.dns, sizeof(network_config.dns), "%s", dns);
  network_config.ntp_enabled = ntp_enabled;

  if (!save_network_config_file()) {
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"Unable to save network.json\"}");
    return;
  }

  char detail[160];
  snprintf(
    detail,
    sizeof(detail),
    "Network Saved hostname=%s dhcp=%s ip=%s gateway=%s reboot_required=YES",
    network_config.hostname,
    network_config.dhcp_enabled ? "true" : "false",
    network_config.ip,
    network_config.gateway
  );

  log_event_line("CONFIG", detail);

  web_server.send(200, "application/json", "{\"ok\":true,\"message\":\"Network configuration saved. Reboot required to apply network changes.\",\"reboot_required\":true}");
}

const char *config_state_text(ConfigState state) {
  switch (state) {
    case CONFIG_STATE_DEFAULTS: return "DEFAULTS";
    case CONFIG_STATE_LOADED:   return "LOADED";
    case CONFIG_STATE_PARTIAL:  return "PARTIAL";
    case CONFIG_STATE_ERROR:    return "ERROR";
    default:                    return "UNKNOWN";
  }
}

bool read_text_file(const char *path, char *buffer, size_t buffer_size) {
  if (!sd_runtime.mounted) return false;
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) return false;

  size_t idx = 0;
  while (file.available() && idx < buffer_size - 1) {
    buffer[idx++] = (char)file.read();
  }
  buffer[idx] = '\0';
  file.close();
  return idx > 0;
}

bool write_text_file(const char *path, const char *content) {
  if (!sd_runtime.mounted) return false;
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) return false;
  file.print(content);
  file.close();
  return true;
}

bool json_get_string(const char *json, const char *key, char *out, size_t out_size) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (*p != '"') return false;
  p++;

  size_t idx = 0;
  while (*p && *p != '"' && idx < out_size - 1) {
    out[idx++] = *p++;
  }
  out[idx] = '\0';
  return idx > 0;
}

bool json_get_bool(const char *json, const char *key, bool *out) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);

  const char *p = strstr(json, pattern);
  if (!p) {
    return false;
  }

  p = strchr(p, ':');
  if (!p) {
    return false;
  }

  p++;

  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
    p++;
  }

  if (strncasecmp(p, "true", 4) == 0) {
    *out = true;
    return true;
  }

  if (strncasecmp(p, "false", 5) == 0) {
    *out = false;
    return true;
  }

  if (*p == '"') {
    p++;

    if (strncasecmp(p, "true", 4) == 0) {
      *out = true;
      return true;
    }

    if (strncasecmp(p, "false", 5) == 0) {
      *out = false;
      return true;
    }

    if (*p == '1') {
      *out = true;
      return true;
    }

    if (*p == '0') {
      *out = false;
      return true;
    }
  }

  if (*p == '1') {
    *out = true;
    return true;
  }

  if (*p == '0') {
    *out = false;
    return true;
  }

  return false;
}

bool json_get_uint32(const char *json, const char *key, uint32_t *out) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (*p < '0' || *p > '9') return false;
  *out = (uint32_t)strtoul(p, nullptr, 10);
  return true;
}

bool json_get_int32(const char *json, const char *key, int32_t *out) {
  if (!json || !key || !out) {
    return false;
  }

  char pattern[64];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);

  const char *key_pos = strstr(json, pattern);
  if (!key_pos) {
    return false;
  }

  const char *colon = strchr(key_pos + strlen(pattern), ':');
  if (!colon) {
    return false;
  }

  const char *value = colon + 1;
  while (*value == ' ' || *value == '\t' || *value == '\r' || *value == '\n') {
    value++;
  }

  char *end_ptr = nullptr;
  long parsed = strtol(value, &end_ptr, 10);
  if (end_ptr == value || parsed < INT32_MIN || parsed > INT32_MAX) {
    return false;
  }

  *out = (int32_t)parsed;
  return true;
}

bool ip_from_string(const char *text, IPAddress *ip) {
  int a, b, c, d;
  if (sscanf(text, "%d.%d.%d.%d", &a, &b, &c, &d) != 4) return false;
  if (a < 0 || a > 255 || b < 0 || b > 255 ||
      c < 0 || c > 255 || d < 0 || d > 255) return false;
  *ip = IPAddress((uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
  return true;
}

void apply_network_config_to_runtime() {
  IPAddress parsed;
  if (ip_from_string(network_config.ip, &parsed)) eth_static_ip = parsed;
  if (ip_from_string(network_config.gateway, &parsed)) eth_gateway = parsed;
  if (ip_from_string(network_config.subnet, &parsed)) eth_subnet = parsed;
  if (ip_from_string(network_config.dns, &parsed)) eth_dns = parsed;
  snprintf(ip_address, sizeof(ip_address), "%s", network_config.ip);
}

bool load_network_config() {
  char json[1024];
  Serial.print("[CFG] Loading ");
  Serial.println(CONFIG_NETWORK_FILE);

  if (!read_text_file(CONFIG_NETWORK_FILE, json, sizeof(json))) {
    Serial.println("[CFG] network.json missing or unreadable");
    return false;
  }

  char value[64];
  bool bool_value = false;

  if (json_get_string(json, "hostname", value, sizeof(value))) {
    snprintf(network_config.hostname, sizeof(network_config.hostname), "%s", value);
  }
  if (json_get_bool(json, "dhcp", &bool_value)) {
    network_config.dhcp_enabled = bool_value;
  }
  if (json_get_string(json, "ip", value, sizeof(value))) {
    snprintf(network_config.ip, sizeof(network_config.ip), "%s", value);
  }
  if (json_get_string(json, "subnet", value, sizeof(value))) {
    snprintf(network_config.subnet, sizeof(network_config.subnet), "%s", value);
  }
  if (json_get_string(json, "gateway", value, sizeof(value))) {
    snprintf(network_config.gateway, sizeof(network_config.gateway), "%s", value);
  }
  if (json_get_string(json, "dns", value, sizeof(value))) {
    snprintf(network_config.dns, sizeof(network_config.dns), "%s", value);
  }
  if (json_get_bool(json, "ntp_enabled", &bool_value)) {
    network_config.ntp_enabled = bool_value;
  }

  apply_network_config_to_runtime();

  Serial.println("[CFG] Network config loaded");
  Serial.print("[CFG] Hostname: "); Serial.println(network_config.hostname);
  Serial.print("[CFG] IP: "); Serial.println(network_config.ip);
  Serial.print("[CFG] Subnet: "); Serial.println(network_config.subnet);
  Serial.print("[CFG] Gateway: "); Serial.println(network_config.gateway);
  Serial.print("[CFG] DNS: "); Serial.println(network_config.dns);

  return true;
}

bool load_system_config() {
  char json[1024];
  Serial.print("[CFG] Loading ");
  Serial.println(CONFIG_SYSTEM_FILE);

  if (!read_text_file(CONFIG_SYSTEM_FILE, json, sizeof(json))) {
    Serial.println("[CFG] system.json missing or unreadable");
    return false;
  }

  char value[64];
  bool bool_value = false;
  uint32_t uint_value = 0;
  int32_t int_value = 0;

  if (json_get_string(json, "device_name", value, sizeof(value))) {
    snprintf(system_config.device_name, sizeof(system_config.device_name), "%s", value);
  }

  if (json_get_string(json, "node_id", value, sizeof(value))) {
    snprintf(system_config.node_id, sizeof(system_config.node_id), "%s", value);
  }

  if (json_get_string(json, "role", value, sizeof(value))) {
    snprintf(system_config.role, sizeof(system_config.role), "%s", value);
  }

  if (json_get_string(json, "site_name", value, sizeof(value))) {
    snprintf(system_config.site_name, sizeof(system_config.site_name), "%s", value);
  }
  if (json_get_int32(json, "local_utc_offset_minutes", &int_value)) {
    if (int_value >= -720 && int_value <= 840) {
      system_config.local_utc_offset_minutes = int_value;
    }
  }
  if (json_get_uint32(json, "screen_timeout", &uint_value)) {
    if (uint_value >= 5 && uint_value <= 3600) system_config.screen_timeout_seconds = uint_value;
  }
  if (json_get_uint32(json, "holdover_minutes", &uint_value)) {
    if (uint_value >= 1 && uint_value <= 1440) system_config.holdover_minutes = uint_value;
  }
  if (json_get_bool(json, "night_dim_enabled", &bool_value)) {
    system_config.night_dim_enabled = bool_value;
  }
  if (json_get_uint32(json, "night_dim_start_minutes", &uint_value)) {
    if (uint_value <= 1439) {
      system_config.night_dim_start_minutes = (uint16_t)uint_value;
    }
  }
  if (json_get_uint32(json, "night_dim_stop_minutes", &uint_value)) {
    if (uint_value <= 1439) {
      system_config.night_dim_stop_minutes = (uint16_t)uint_value;
    }
  }
  if (json_get_uint32(json, "night_dim_percent", &uint_value)) {
    if (uint_value <= 100) {
      system_config.night_dim_percent = (uint8_t)uint_value;
    }
  }
  if (json_get_uint32(json, "night_dim_wake_seconds", &uint_value)) {
    if (uint_value >= 1 && uint_value <= 3600) {
      system_config.night_dim_wake_seconds = (uint16_t)uint_value;
    }
  }
  if (json_get_bool(json, "debug_serial", &bool_value)) {
    system_config.debug_serial = bool_value;
  }
  if (json_get_bool(json, "web_enabled", &bool_value)) {
    system_config.web_enabled = bool_value;
  }

  Serial.println("[CFG] System config loaded");
  Serial.print("[CFG] Device: "); Serial.println(system_config.device_name);
  Serial.print("[CFG] Local UTC offset minutes: "); Serial.println(system_config.local_utc_offset_minutes);
  Serial.print("[CFG] Screen timeout: "); Serial.println(system_config.screen_timeout_seconds);
  Serial.print("[CFG] Holdover minutes: "); Serial.println(system_config.holdover_minutes);
  Serial.print("[CFG] Night dim enabled: "); Serial.println(system_config.night_dim_enabled ? "true" : "false");
  Serial.print("[CFG] Night dim start minutes: "); Serial.println(system_config.night_dim_start_minutes);
  Serial.print("[CFG] Night dim stop minutes: "); Serial.println(system_config.night_dim_stop_minutes);
  Serial.print("[CFG] Night dim percent: "); Serial.println(system_config.night_dim_percent);
  Serial.print("[CFG] Night dim wake seconds: "); Serial.println(system_config.night_dim_wake_seconds);
  Serial.print("[CFG] Debug serial: "); Serial.println(system_config.debug_serial ? "true" : "false");
  Serial.print("[CFG] Web enabled: "); Serial.println(system_config.web_enabled ? "true" : "false");

  return true;
}

bool create_default_config_files() {
  if (!sd_runtime.mounted) return false;

  bool created_any = false;

  if (!SD_MMC.exists("/config")) {
    SD_MMC.mkdir("/config");
  }

  if (!SD_MMC.exists(CONFIG_NETWORK_FILE)) {
    const char *network_default =
      "{\n"
      "  \"hostname\": \"skytime-p4\",\n"
      "  \"dhcp\": false,\n"
      "  \"ip\": \"192.168.0.123\",\n"
      "  \"subnet\": \"255.255.255.0\",\n"
      "  \"gateway\": \"192.168.0.1\",\n"
      "  \"dns\": \"192.168.0.1\",\n"
      "  \"ntp_enabled\": true\n"
      "}\n";
    if (write_text_file(CONFIG_NETWORK_FILE, network_default)) {
      Serial.println("[CFG] Created default network.json");
      created_any = true;
    }
  }

  if (!SD_MMC.exists(CONFIG_SYSTEM_FILE)) {
    const char *system_default =
      "{\n"
      "  \"device_name\": \"SkyTime\",\n"
      "  \"node_id\": \"SkyTime\",\n"
      "  \"role\": \"Standalone\",\n"
      "  \"site_name\": \"Enter Location in Configuration\",\n"
      "  \"local_utc_offset_minutes\": 0,\n"
      "  \"screen_timeout\": 30,\n"
      "  \"holdover_minutes\": 60,\n"
      "  \"night_dim_enabled\": false,\n"
      "  \"night_dim_start_minutes\": 1320,\n"
      "  \"night_dim_stop_minutes\": 360,\n"
      "  \"night_dim_percent\": 70,\n"
      "  \"night_dim_wake_seconds\": 30,\n"
      "  \"debug_serial\": true,\n"
      "  \"web_enabled\": true\n"
      "}\n";
    if (write_text_file(CONFIG_SYSTEM_FILE, system_default)) {
      Serial.println("[CFG] Created default system.json");
      created_any = true;
    }
  }

  config_runtime.defaults_created = created_any;
  return true;
}

bool load_config_files() {
  config_runtime.load_attempts++;

  if (!sd_runtime.mounted) {
    config_runtime.state = CONFIG_STATE_DEFAULTS;
    config_runtime.errors++;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "SD not mounted");
    Serial.println("[CFG] SD not mounted - using defaults");
    return false;
  }

  create_default_config_files();

  config_runtime.network_loaded = load_network_config();
  config_runtime.system_loaded = load_system_config();

  if (config_runtime.network_loaded && config_runtime.system_loaded) {
    config_runtime.state = CONFIG_STATE_LOADED;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "OK");
  } else if (config_runtime.network_loaded || config_runtime.system_loaded) {
    config_runtime.state = CONFIG_STATE_PARTIAL;
    config_runtime.errors++;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "partial config");
  } else {
    config_runtime.state = CONFIG_STATE_DEFAULTS;
    config_runtime.errors++;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "using defaults");
  }

  return config_runtime.network_loaded || config_runtime.system_loaded;
}

void init_config() {
  Serial.println("[CFG] Initializing configuration framework");
  load_config_files();
}
