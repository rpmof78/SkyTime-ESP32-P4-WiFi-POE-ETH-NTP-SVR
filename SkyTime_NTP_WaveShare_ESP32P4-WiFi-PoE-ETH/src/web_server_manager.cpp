#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WebServer.h>
#include <WiFi.h>
#include <DNSServer.h>

#include "skytime_types.h"
#include "config_manager.h"
#include "logging_manager.h"
#include "gnss_manager.h"
#include "web_server_manager.h"

#define WEB_INDEX_FILE              "/web/index.html"
#define WEB_ENABLE_RAW_8080_DEBUG    0
#define SKYTIME_WIFI_AP_SSID         "SkyTime-Setup"

extern WebServer web_server;
extern WebRuntime web_runtime;
extern SdRuntime sd_runtime;
extern SystemConfig system_config;
extern NetworkConfig network_config;
extern EthernetRuntime ethernet_runtime;
extern WifiApRuntime wifi_ap_runtime;

extern IPAddress wifi_ap_ip;
extern bool commissioning_dns_started;
extern bool boot_is_power_on_reset;

extern bool web_network_ready();
extern void handle_web_ping();
extern void update_raw_http_debug_server();
extern void handle_api_system_reboot();

extern void build_status_json(
  char *buffer,
  size_t buffer_size
);
extern void build_status_html(
  char *buffer,
  size_t buffer_size
);

const char *content_type_from_path(
  const char *path
);
bool request_from_wifi_ap();
void handle_commissioning_probe();
void handle_api_commissioning_status();
void handle_web_root();
void handle_web_status_page();
void handle_web_api_status();
void handle_web_not_found();

const char *web_state_text(WebState state) {
  switch (state) {
    case WEB_STATE_DISABLED: return "DISABLED";
    case WEB_STATE_WAIT_ETH:  return "WAIT ETH";
    case WEB_STATE_RUNNING:   return "RUNNING";
    case WEB_STATE_ERROR:     return "ERROR";
    default:                  return "UNKNOWN";
  }
}

const char *content_type_from_path(const char *path) {
  const char *ext = strrchr(path, '.');

  if (!ext) return "text/plain";

  if (strcasecmp(ext, ".html") == 0) return "text/html";
  if (strcasecmp(ext, ".htm") == 0)  return "text/html";
  if (strcasecmp(ext, ".css") == 0)  return "text/css";
  if (strcasecmp(ext, ".js") == 0)   return "application/javascript";
  if (strcasecmp(ext, ".json") == 0) return "application/json";
  if (strcasecmp(ext, ".txt") == 0)  return "text/plain";
  if (strcasecmp(ext, ".png") == 0)  return "image/png";
  if (strcasecmp(ext, ".jpg") == 0)  return "image/jpeg";
  if (strcasecmp(ext, ".ico") == 0)  return "image/x-icon";

  return "application/octet-stream";
}

bool send_sd_file(const char *path, const char *content_type) {
  if (!sd_runtime.mounted) {
    return false;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file || file.isDirectory()) {
    if (file) {
      file.close();
    }
    return false;
  }

  web_server.streamFile(file, content_type);
  file.close();

  web_runtime.requests_static++;
  return true;
}

bool request_from_wifi_ap() {
  if (!wifi_ap_runtime.started) {
    return false;
  }

  IPAddress local_ip = web_server.client().localIP();
  return local_ip == wifi_ap_ip;
}

void handle_commissioning_probe() {
  web_runtime.requests_total++;
  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    web_server.uri().c_str()
  );

  if (request_from_wifi_ap()) {
    web_server.sendHeader("Location", "/config.html", true);
    web_server.sendHeader("Cache-Control", "no-store");
    web_server.send(302, "text/plain", "SkyTime configuration");
    return;
  }

  web_server.send(204, "text/plain", "");
}

void handle_api_commissioning_status() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    web_server.uri().c_str()
  );

  char json[320];

  snprintf(
    json,
    sizeof(json),
    "{\"ok\":true,"
    "\"power_on_boot\":%s,"
    "\"active\":%s,"
    "\"expired\":%s,"
    "\"captive_dns\":%s,"
    "\"remaining_seconds\":%lu,"
    "\"clients\":%u,"
    "\"ssid\":\"%s\","
    "\"ip\":\"%s\"}",
    boot_is_power_on_reset ? "true" : "false",
    wifi_ap_runtime.started ? "true" : "false",
    wifi_ap_runtime.window_expired ? "true" : "false",
    commissioning_dns_started ? "true" : "false",
    (unsigned long)wifi_ap_runtime.remaining_seconds,
    (unsigned int)wifi_ap_runtime.clients,
    SKYTIME_WIFI_AP_SSID,
    wifi_ap_runtime.started ? wifi_ap_runtime.ip : "0.0.0.0"
  );

  web_server.sendHeader("Cache-Control", "no-store");
  web_server.send(200, "application/json", json);
}

void handle_web_root() {
  web_runtime.requests_total++;
  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    web_server.uri().c_str()
  );

  // The temporary Wi-Fi interface is a commissioning portal.
  // Ethernet retains the normal operational home page.
  if (request_from_wifi_ap()) {
    if (send_sd_file("/web/config.html", "text/html")) {
      return;
    }

    web_server.send(
      404,
      "text/plain",
      "SkyTime configuration page not found on SD card"
    );
    return;
  }

  if (web_runtime.sd_index_available &&
      send_sd_file(WEB_INDEX_FILE, "text/html")) {
    return;
  }

  handle_web_status_page();
}

void handle_web_status_page() {
  web_runtime.requests_total++;
  web_runtime.requests_status++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  static char html[4096];
  build_status_html(html, sizeof(html));
  web_server.send(200, "text/html", html);
}

void handle_web_api_status() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  static char json[7168];
  build_status_json(json, sizeof(json));
  web_server.send(200, "application/json", json);
}

void handle_web_not_found() {
  web_runtime.requests_total++;
  web_runtime.requests_not_found++;

  String uri = web_server.uri();

  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    uri.c_str()
  );

  if (uri.indexOf("..") >= 0) {
    web_server.send(
      400,
      "text/plain",
      "Bad request"
    );
    return;
  }

  if (uri.endsWith("/")) {
    uri += "index.html";
  }

  char path[128];
  int path_written = 0;

  if (uri.startsWith("/web/")) {
    path_written = snprintf(
      path,
      sizeof(path),
      "%s",
      uri.c_str()
    );
  } else {
    path_written = snprintf(
      path,
      sizeof(path),
      "/web%s",
      uri.c_str()
    );
  }

  if (path_written < 0 ||
      (size_t)path_written >= sizeof(path)) {
    web_server.send(
      414,
      "text/plain",
      "Request URI too long"
    );
    return;
  }

  if (send_sd_file(
        path,
        content_type_from_path(path)
      )) {
    return;
  }

  web_server.send(
    404,
    "text/plain",
    "Not found"
  );
}

void init_web_server() {
  if (!system_config.web_enabled) {
    web_runtime.state = WEB_STATE_DISABLED;
    Serial.print("[WEB] Disabled by system config; web_enabled=");
    Serial.println(system_config.web_enabled ? "true" : "false");
    return;
  }

  if (!web_network_ready()) {
    web_runtime.state = WEB_STATE_WAIT_ETH;
    return;
  }

  if (web_runtime.started) {
    return;
  }

  web_runtime.start_attempts++;

  web_runtime.sd_index_available =
    sd_runtime.mounted && SD_MMC.exists(WEB_INDEX_FILE);

  web_server.on("/", HTTP_GET, handle_web_root);
  web_server.on("/status", HTTP_GET, handle_web_status_page);
  web_server.on("/api/status", HTTP_GET, handle_web_api_status);
  web_server.on("/api/gnss", HTTP_GET, handle_api_gnss);
  web_server.on(
    "/api/commissioning/status",
    HTTP_GET,
    handle_api_commissioning_status
  );

  // Common captive-portal detection requests from phones and computers.
  web_server.on("/generate_204", HTTP_GET, handle_commissioning_probe);
  web_server.on("/gen_204", HTTP_GET, handle_commissioning_probe);
  web_server.on("/hotspot-detect.html", HTTP_GET, handle_commissioning_probe);
  web_server.on("/library/test/success.html", HTTP_GET, handle_commissioning_probe);
  web_server.on("/connecttest.txt", HTTP_GET, handle_commissioning_probe);
  web_server.on("/ncsi.txt", HTTP_GET, handle_commissioning_probe);
  web_server.on("/fwlink", HTTP_GET, handle_commissioning_probe);

  web_server.on("/logs.html", HTTP_GET, handle_web_logs_page);
  web_server.on("/config.html", HTTP_GET, handle_web_config_page);
  web_server.on("/api/config/identity", HTTP_GET, handle_api_config_identity_get);
  web_server.on("/api/config/identity", HTTP_POST, handle_api_config_identity_post);
  web_server.on("/api/config/display", HTTP_GET, handle_api_config_display_get);
  web_server.on("/api/config/display", HTTP_POST, handle_api_config_display_post);
  web_server.on("/api/config/network", HTTP_GET, handle_api_config_network_get);
  web_server.on("/api/config/network", HTTP_POST, handle_api_config_network_post);
  web_server.on("/api/system/reboot", HTTP_POST, handle_api_system_reboot);
  web_server.on("/api/logs/events", HTTP_GET, handle_api_log_events);
  web_server.on("/api/logs/ntp", HTTP_GET, handle_api_log_ntp);
  web_server.on("/api/logs/files", HTTP_GET, handle_api_log_files);
  web_server.on("/api/logs/view", HTTP_GET, handle_api_log_selected);
  web_server.on("/api/logs/download", HTTP_GET, handle_api_log_download);
  web_server.on("/ping", HTTP_GET, handle_web_ping);
  web_server.onNotFound(handle_web_not_found);

  web_server.begin();
#if WEB_ENABLE_RAW_8080_DEBUG
  raw_http_debug_server.begin();
#endif

  web_runtime.started = true;
  web_runtime.state = WEB_STATE_RUNNING;

  snprintf(web_runtime.last_error, sizeof(web_runtime.last_error), "OK");

  Serial.println("[WEB] HTTP server started on port 80");
  Serial.print("[WEB] SD index: ");
  Serial.println(web_runtime.sd_index_available ? "YES" : "NO");
  if (ethernet_runtime.got_ip) {
    Serial.print("[WEB] Ethernet URL: http://");
    Serial.println(ethernet_runtime.ip);
  }

  if (wifi_ap_runtime.started) {
    Serial.print("[WEB] Setup AP URL: http://");
    Serial.println(wifi_ap_runtime.ip);
    Serial.println("[WEB] Wi-Fi root mode: commissioning portal");
    Serial.println("[WEB] Captive DNS: wildcard -> 192.168.4.123");
    Serial.println("[WEB] Commissioning API: /api/commissioning/status");
  }
#if WEB_ENABLE_RAW_8080_DEBUG
  Serial.print("[WEB] Raw debug URL: http://");
  Serial.print(ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
  Serial.println(":8080/");
#endif
}

void update_web_server() {
  if (!system_config.web_enabled) {
    web_runtime.state = WEB_STATE_DISABLED;
    return;
  }

  if (!web_network_ready()) {
    web_runtime.state = WEB_STATE_WAIT_ETH;
    return;
  }

  if (!web_runtime.started) {
    init_web_server();
    return;
  }

  web_runtime.handle_ticks++;

  update_raw_http_debug_server();
  web_server.handleClient();
}
